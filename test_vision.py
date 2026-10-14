"""Ryan detector frames, headless camera setup, worker lifecycle and FC integration."""

import contextlib
import io
import math
from pathlib import Path
from queue import Queue
import sys
import time
import types
import unittest
import tempfile
from unittest.mock import patch

import ir_confidence as ir
import app as menu
from fake_power import FakeGuard
import motors_optimized as motor
import vision_support as vision
import vision_settings as settings
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


def sample(sequence=1, timestamp=0, labels=(), area=0.03):
    return vision.VisionSample(sequence, timestamp, tuple((label, area) for label in labels), 0.01)


class FakeStream:
    def __init__(self, clock, labels=lambda now: ()):
        self.clock, self.labels = clock, labels
        self.closed = False

    def latest(self):
        tick = round(self.clock.now * 50) // 5
        return sample(tick + 1, tick / 10, self.labels(tick / 10))

    def close(self):
        self.closed = True
        return True


class SampleTests(unittest.TestCase):
    """The light rules themselves are in test_v9_9.TrafficLightTests."""

    def test_stale_invalid_and_out_of_order_results_fail(self):
        for value in (sample(timestamp=-1), sample(timestamp=1), sample(labels=("unknown",)),
                      sample(area=math.nan, labels=("red_light",)), sample(sequence=0)):
            with self.subTest(value=value), self.assertRaises(vision.VisionFault):
                vision.check_vision_sample(value, 0)


@unittest.skipIf(cv2 is None, "OpenCV/numpy are needed for image-based detector tests")
class RyanDetectorTests(unittest.TestCase):
    def detector(self):
        from ryan_vision.detector_geometric import GeometricDetector
        return GeometricDetector(settings)

    def blank(self):
        return np.zeros((480, 640, 3), np.uint8)

    def sign_frame(self, center=(480, 180), radius=65, value=165):
        frame = self.blank()
        pts = np.array([[center[0] + radius * math.cos(math.radians(22.5 + 45 * k)),
                         center[1] + radius * math.sin(math.radians(22.5 + 45 * k))]
                        for k in range(8)], np.int32)
        cv2.fillPoly(frame, [pts], (0, 0, value))
        return frame

    def test_actual_detector_recognizes_stop_sign_and_all_three_lights(self):
        detector = self.detector()
        for expected, color in (("red_light", (0, 0, 255)), ("yellow_light", (0, 255, 255)),
                                ("green_light", (0, 255, 0)), ("stop_sign", (0, 0, 165))):
            with self.subTest(expected=expected):
                frame = self.blank()
                if expected == "stop_sign":
                    frame = self.sign_frame()
                else:
                    cv2.circle(frame, (320, 120), 35, color, -1)
                detections = detector.detect(frame)
                self.assertEqual({d.label for d in detections}, {expected})
                self.assertTrue(all(d.area_frac > 0 for d in detections))
                detection = detections[0]
                self.assertEqual(detection.position, "RIGHT" if expected == "stop_sign" else "CENTER")
                expected_center = (480, 180) if expected == "stop_sign" else (320, 120)
                for actual, wanted in zip(detection.center, expected_center):
                    self.assertLessEqual(abs(actual - wanted), 2)

    def test_empty_frame_and_bottom_band_do_not_report_hazards(self):
        detector = self.detector()
        frame = self.blank()
        self.assertEqual(detector.detect(frame), [])
        cv2.circle(frame, (320, 450), 25, (0, 255, 0), -1)
        self.assertEqual(detector.detect(frame), [])

    def test_same_red_object_uses_ryans_stop_sign_priority(self):
        from ryan_vision import vision as ryan
        detector = self.detector()
        with patch.object(ryan, "detect_stop_sign", return_value={"box": (400, 60, 80, 80), "center": (440, 100)}), \
             patch.object(ryan, "detect_traffic_light", return_value={"box": (420, 80, 30, 30), "center": (435, 95), "color": "RED"}):
            detections = detector.detect(self.blank())
        self.assertEqual([d.label for d in detections], ["stop_sign"])

    def test_green_yellow_and_separate_red_are_not_suppressed(self):
        from ryan_vision import vision as ryan
        for color, box in (("GREEN", (420, 80, 30, 30)), ("YELLOW", (420, 80, 30, 30)),
                           ("RED", (80, 80, 30, 30))):
            with self.subTest(color=color), \
                 patch.object(ryan, "detect_stop_sign", return_value={"box": (400, 60, 80, 80), "center": (440, 100)}), \
                 patch.object(ryan, "detect_traffic_light", return_value={"box": box, "center": (box[0]+15, 95), "color": color}):
                labels = {d.label for d in self.detector().detect(self.blank())}
                self.assertEqual(labels, {"stop_sign", color.lower() + "_light"})

    def test_stop_sign_region_covers_full_frame_but_keeps_shape_size_gates(self):
        detector = self.detector()
        for center, radius in (((150, 180), 65), ((320, 180), 65), ((480, 180), 65),
                               ((480, 430), 35), ((100, 45), 35)):
            with self.subTest(center=center):
                self.assertEqual([d.label for d in detector.detect(self.sign_frame(center, radius))], ["stop_sign"])
        self.assertEqual(detector.detect(self.sign_frame((480, 10), 7)), [])

    def test_lights_are_detected_across_top_half_but_not_bottom_half(self):
        detector = self.detector()
        for x, position in ((90, "LEFT"), (320, "CENTER"), (550, "RIGHT")):
            frame = self.blank()
            cv2.circle(frame, (x, 100), 30, (0, 255, 0), -1)
            result = detector.detect(frame)
            self.assertEqual([(d.label, d.position) for d in result], [("green_light", position)])
        frame = self.blank()
        cv2.circle(frame, (320, 350), 30, (0, 255, 0), -1)
        self.assertEqual(detector.detect(frame), [])

    def test_small_right_side_sign_is_detected_but_below_fc_action_size(self):
        result = self.detector().detect(self.sign_frame(radius=16))
        self.assertEqual([d.label for d in result], ["stop_sign"])
        self.assertLess(result[0].area_frac, settings.STOPSIGN_ACT_AREA_FRAC)

    def test_bright_octagon_is_treated_as_a_light_and_dim_circle_is_rejected(self):
        detector = self.detector()
        # Red-light rejection also applies when a glowing object looks octagonal.
        self.assertEqual([d.label for d in detector.detect(self.sign_frame(radius=45, value=255))], ["red_light"])
        frame = self.blank()
        cv2.circle(frame, (480, 120), 35, (0, 0, 165), -1)
        self.assertEqual(detector.detect(frame), [])

    def test_detection_cache_cannot_reuse_a_green_frame_after_wall_clock_rollback(self):
        from ryan_vision import traffic_light_detector as lights
        detector = self.detector()
        frame = self.blank()
        cv2.circle(frame, (320, 100), 30, (0, 255, 0), -1)
        with patch.object(lights.time, "time", side_effect=[100.0, 99.0]):
            self.assertEqual([d.label for d in detector.detect(frame)], ["green_light"])
            self.assertEqual(detector.detect(self.blank()), [])

    def test_worker_publishes_current_detector_labels_positions_and_capture_age(self):
        frame = self.sign_frame()
        closed = []
        stopped = []
        camera = types.SimpleNamespace(read=lambda: frame, close=lambda: closed.append(True))
        done = types.SimpleNamespace(is_set=lambda: bool(stopped), wait=lambda delay: stopped.append(True))
        results = Queue(maxsize=4)
        with patch.object(vision, "HeadlessCamera", return_value=camera), \
             patch.object(vision.time, "monotonic", side_effect=[0.0, 2.0, 2.02]):
            vision.vision_worker(vision.CameraOptions(), results, done)
        result = vision.check_vision_sample(results.get_nowait(), 2.03)
        self.assertEqual(result.sequence, 1)
        self.assertEqual(result.timestamp, 2.0)
        self.assertAlmostEqual(result.processing_seconds, 0.02)
        self.assertEqual(result.detections[0][0], "stop_sign")
        self.assertGreater(result.detections[0][1], settings.STOPSIGN_ACT_AREA_FRAC)
        self.assertEqual(result.locations[0][:2], ("stop_sign", "RIGHT"))
        self.assertTrue(all(isinstance(v, int) for v in result.locations[0][2]))
        self.assertEqual(closed, [True])

    def test_csi_camera_uses_bgr_memory_format_no_preview_and_requested_rotation(self):
        frame = np.zeros((4, 4, 3), np.uint8)
        frame[0, 0] = (1, 2, 3)

        class FakePicamera:
            def __init__(self, index):
                self.index = index
                self.closed = False

            def create_video_configuration(self, **kwargs):
                self.config = kwargs
                return kwargs

            def configure(self, config):
                pass

            def start(self, **kwargs):
                self.start_options = kwargs

            def capture_array(self, name):
                return frame.copy()

            def stop(self):
                pass

            def close(self):
                self.closed = True

        with patch.dict(sys.modules, {"picamera2": types.SimpleNamespace(Picamera2=FakePicamera)}):
            camera = vision.HeadlessCamera(vision.CameraOptions(rotate_180=True))
            device = camera.picam
            self.assertEqual(device.config["main"]["format"], "RGB888")
            self.assertEqual(device.config["main"]["size"], (640, 480))
            self.assertFalse(device.config["queue"])
            self.assertEqual(device.start_options, {"show_preview": False})
            np.testing.assert_array_equal(camera.read()[-1, -1], [1, 2, 3])
            camera.close()
            self.assertTrue(device.closed)


def hanging_worker(options, queue, done):
    queue.put(sample(timestamp=time.monotonic()))
    time.sleep(30)  # Intentionally ignores cancellation, like a stuck camera syscall.


def failing_worker(options, queue, done):
    queue.put(("error", "Injected camera failure"))
    done.wait(5)


class CameraWorkerTests(unittest.TestCase):
    def wait_result(self, stream):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            value = stream.latest()
            if value is not None:
                return value
            time.sleep(0.02)
        self.fail("Spawned test worker produced no result")

    def test_real_spawned_stuck_camera_process_can_be_stopped(self):
        stream = vision.VisionStream(worker=hanging_worker)
        try:
            self.assertIsInstance(self.wait_result(stream), vision.VisionSample)
            started = time.monotonic()
            self.assertTrue(stream.close())
            self.assertLess(time.monotonic() - started, 2.0)
            self.assertFalse(stream.process.is_alive())
        finally:
            stream.close()

    def test_worker_error_is_reported_and_process_closes(self):
        stream = vision.VisionStream(worker=failing_worker)
        try:
            with self.assertRaisesRegex(vision.VisionFault, "Injected camera failure"):
                self.wait_result(stream)
        finally:
            self.assertTrue(stream.close())


class VisionControlTests(unittest.TestCase):
    def make(self, events=(), labels=lambda now: ()):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        messages = []
        control = ir.ConfidenceTests(driver, FakeKeys(clock, events), clock,
                                     lambda text: messages.append((clock.now, text)))
        stream = FakeStream(clock, labels)
        session = vision.VisionSession(stream_factory=lambda options: stream)
        return control, session, stream, driver, gpio, clock, messages

    def assert_stopped(self, driver, gpio):
        self.assertEqual(driver.duties, [0] * 4)
        self.assertTrue(all(pwm.duty == 0 for pwm in gpio.pwms))

    def test_rv_reports_detections_without_powering_wheels(self):
        control, session, stream, driver, gpio, _, messages = self.make([(0.6, "s")], lambda now: ("green_light",))
        with self.assertRaises(motor.StopRequested):
            session.readings(control)
        self.assertTrue(any("green_light" in text and "Camera:" in text for _, text in messages))
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assertTrue(stream.closed)

    def test_rv_reports_ryans_position_and_center_with_wheels_off(self):
        control, session, stream, _, gpio, clock, messages = self.make([(0.6, "s")])
        def latest():
            base = sample(round(clock.now * 50) + 1, clock.now, ("stop_sign",))
            return vision.VisionSample(base.sequence, base.timestamp, base.detections, 0.01,
                                       (("stop_sign", "RIGHT", (480, 180)),))
        stream.latest = latest
        with self.assertRaises(motor.StopRequested):
            session.readings(control)
        self.assertTrue(any("stop_sign RIGHT center=(480, 180)" in text for _, text in messages))
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))

    def test_red_away_from_a_line_arms_it_but_does_not_stop_fc(self):
        labels = lambda now: ("red_light",) if 0.6 <= now < 1.0 else (("green_light",) if now >= 1.0 else ())
        control, session, stream, driver, gpio, _, messages = self.make([(2.0, "s")], labels)
        with self.assertRaises(motor.StopRequested):
            session.forward(control)
        self.assertFalse(any(duty == 0 for t, _, duty in gpio.pwm_writes if 0.5 <= t < 2.0))
        self.assertIn("Camera: red light, front stop line armed", [text for _, text in messages])
        self.assertLessEqual(max(d for _, _, d in gpio.pwm_writes), ir.MAX_DUTY)
        self.assertTrue(stream.closed)
        self.assert_stopped(driver, gpio)

    def test_fc_time_limit_still_bounds_a_camera_run(self):
        control, session, _, driver, gpio, clock, _ = self.make(labels=lambda now: ("red_light",) if 0.6 <= now < 0.8 else ())
        session.forward(control)
        self.assertEqual(clock.now, ir.FORWARD_SECONDS)
        self.assert_stopped(driver, gpio)

    def test_confirmed_stop_sign_ends_fc_at_its_line_and_closes_camera(self):
        control, session, stream, driver, gpio, clock, _ = self.make(labels=lambda now: ("stop_sign",) if now >= 0.6 else ())
        gpio.readings = lambda now: (1, 1, 0 if now >= 1.2 else 1)
        session.forward(control)
        self.assertEqual(clock.now, 1.2)
        self.assertTrue(any(duty > 0 for t, _, duty in gpio.pwm_writes if 0.7 <= t < 1.2))
        self.assertTrue(stream.closed)
        self.assert_stopped(driver, gpio)

    def test_stale_camera_stops_without_falling_back_to_ir_only(self):
        control, session, stream, driver, gpio, clock, _ = self.make()
        stream.latest = lambda: sample(min(round(clock.now * 50) // 5 + 1, 5), min((round(clock.now * 50) // 5) / 10, 0.4))
        with self.assertRaisesRegex(vision.VisionFault, "stale"):
            session.forward(control)
        self.assertLessEqual(clock.now, 1.18)
        self.assert_stopped(driver, gpio)

    def test_stop_key_remains_live_during_startup_and_red_hold(self):
        for starting in (True, False):
            with self.subTest(starting=starting):
                control, session, stream, driver, gpio, clock, _ = self.make([(0.08, "s")], lambda now: ("red_light",))
                if starting:
                    stream.latest = lambda: None
                with self.assertRaises(motor.StopRequested):
                    session.forward(control)
                self.assertEqual(clock.now, 0.08)
                self.assertTrue(stream.closed)
                self.assert_stopped(driver, gpio)

    def test_a_camera_fault_at_the_front_line_still_stops_the_motors(self):
        control, session, stream, driver, gpio, clock, _ = self.make()
        old_latest = stream.latest
        stream.latest = lambda: (_ for _ in ()).throw(vision.VisionFault("camera error")) if clock.now >= 0.6 else old_latest()
        gpio.readings = lambda now: (1, 1, 0 if now >= 0.6 else 1)
        with self.assertRaisesRegex(vision.VisionFault, "camera error"):
            session.forward(control)
        self.assertEqual(clock.now, 0.6)
        self.assertTrue(stream.closed)
        self.assert_stopped(driver, gpio)

    def test_startup_timeout_and_camera_error_never_power_motors(self):
        for error in (False, True):
            control, session, stream, driver, gpio, clock, _ = self.make()
            stream.latest = (lambda: (_ for _ in ()).throw(vision.VisionFault("missing camera"))) if error else lambda: None
            with self.assertRaises(vision.VisionFault):
                session.forward(control)
            self.assertLessEqual(clock.now, vision.VISION_START_TIMEOUT)
            self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
            self.assertTrue(stream.closed)

    def test_blocked_previous_camera_cannot_be_replaced(self):
        control, session, stream, driver, gpio, _, _ = self.make()
        stream.close = lambda: False
        session.stream = stream
        session.stream_factory = lambda options: self.fail("Opened duplicate camera owner")
        with self.assertRaisesRegex(vision.VisionFault, "still blocked"):
            session.forward(control)
        self.assertIs(session.stream, stream)
        self.assert_stopped(driver, gpio)

    def test_menu_toggle_is_off_by_default_and_only_changes_fc(self):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        rpi = types.ModuleType("RPi")
        rpi.GPIO = gpio
        calls = []
        fake_vision = types.SimpleNamespace(enabled=False, close=lambda: calls.append("close"),
                    forward=lambda control: calls.append("vision FC"), readings=lambda control: calls.append("RV"))
        fake_control = types.SimpleNamespace(run_test=lambda command: calls.append(command))
        choices = iter(["FC", "VT", "FC", "RV", "IRC", "VT", "FC", "Q"])
        with patch.dict(sys.modules, {"RPi": rpi, "RPi.GPIO": gpio}), \
             patch.object(menu.sys.stdin, "isatty", return_value=True), \
             patch.object(menu, "PowerGuard", FakeGuard), \
             patch.object(menu, "VisionSession", return_value=fake_vision), \
             patch.object(menu, "ConfidenceTests", return_value=fake_control), \
             patch.object(motor, "prompt_line", side_effect=lambda text: next(choices)), \
             patch.object(motor, "Keyboard", side_effect=lambda: contextlib.nullcontext(FakeKeys(clock))), \
             contextlib.redirect_stdout(io.StringIO()):
            with tempfile.TemporaryDirectory() as folder:
                self.assertEqual(menu.main(["--bot", "500", "--settings", str(Path(folder) / "settings.json")]), 0)
        self.assertEqual(calls, ["FC", "vision FC", "RV", "IRC", "FC", "close"])
        self.assertFalse(fake_vision.enabled)
        self.assertTrue(gpio.cleaned)


if __name__ == "__main__":
    unittest.main(verbosity=2)

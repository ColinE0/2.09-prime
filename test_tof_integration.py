"""Hardware-free checks for FC combining IR, camera, and optional ToF decisions.

v9.9: the camera no longer holds FC mid-lane. Its light decides the front line (see
test_v9_9); only the ToF holds mid-lane.
"""

import contextlib
import io
import math
import sys
import types
import unittest
from pathlib import Path
import tempfile
from unittest.mock import patch

import ir_confidence as ir
import app as menu
from fake_power import FakeGuard
import motors_optimized as motor
import tof_support as tof
import vision_support as vision
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys
from test_vision import FakeStream


class FakeToFStream:
    def __init__(self, clock, distance=lambda now: 600, status=lambda now: 11):
        self.clock, self.distance, self.status = clock, distance, status
        self.closed = False

    def latest(self):
        tick = round(self.clock.now * 50) // 5
        return tof.ToFSample(tick + 1, tick / 10, self.distance(tick / 10), self.status(tick / 10))

    def close(self):
        self.closed = True
        return True


class ToFControlIntegrationTests(unittest.TestCase):
    def make(self, events=(), tof_gate=None, labels=None):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        messages = []
        control = ir.ConfidenceTests(
            driver, FakeKeys(clock, events), clock,
            lambda text: messages.append((clock.now, text)), tof_gate=tof_gate)
        if labels is not None:  # the camera on: what each frame shows
            control.light = vision.TrafficLight(FakeStream(clock, labels), control.log)
        return control, driver, gpio, clock, messages

    def assert_stopped(self, driver, gpio):
        self.assertEqual(driver.duties, [0] * 4)
        self.assertTrue(all(pwm.duty == 0 for pwm in gpio.pwms))

    def test_tof_hold_stops_immediately_then_restarts_with_ir_warmup_and_ramp(self):
        gate = lambda now: ("hold", 0.0, "Obstacle") if 0.6 <= now < 1.0 else ("drive", 1.0, "Clear")
        control, driver, gpio, _, _ = self.make([(1.8, "s")], tof_gate=gate)
        with self.assertRaises(motor.StopRequested):
            control.run_test("FC")
        self.assertTrue(any(duty > 0 for now, _, duty in gpio.pwm_writes if now < 0.6))
        self.assertTrue(all(duty == 0 for now, _, duty in gpio.pwm_writes if 0.6 <= now < 1.12))
        resumed = [(now, duty) for now, _, duty in gpio.pwm_writes if now >= 1.12 and duty > 0]
        self.assertTrue(resumed)
        self.assertLessEqual(resumed[0][1], motor.RAMP_RATE * motor.LOOP_DT + 1e-9)
        self.assert_stopped(driver, gpio)

    def test_half_scale_caps_wheel_targets_at_half_normal_fc_duty(self):
        control, driver, gpio, _, _ = self.make(
            [(1.0, "s")], tof_gate=lambda now: ("drive", 0.5, "Slow zone"))
        with self.assertRaises(motor.StopRequested):
            control.run_test("FC")
        self.assertAlmostEqual(max(duty for _, _, duty in gpio.pwm_writes), ir.MAX_DUTY * 0.5)
        self.assert_stopped(driver, gpio)

    def test_slow_zone_applies_normal_deceleration_before_reaching_lower_target(self):
        control, driver, gpio, _, _ = self.make(
            [(1.2, "s")], tof_gate=lambda now: ("drive", 1.0 if now < 0.6 else 0.25, "Clear or slow"))
        with self.assertRaises(motor.StopRequested):
            control.run_test("FC")
        at_transition = [duty for now, _, duty in gpio.pwm_writes if now == 0.6]
        self.assertTrue(at_transition)
        self.assertGreater(max(at_transition), ir.MAX_DUTY * 0.25)
        self.assertLess(max(at_transition), ir.MAX_DUTY)
        self.assertTrue(all(duty <= ir.MAX_DUTY * 0.25 + 1e-9
                            for now, _, duty in gpio.pwm_writes if now >= 0.9))
        self.assert_stopped(driver, gpio)

    def test_zero_scale_never_powers_wheels(self):
        control, driver, gpio, _, _ = self.make(
            [(0.5, "s")], tof_gate=lambda now: ("drive", 0.0, "Zero target"))
        with self.assertRaises(motor.StopRequested):
            control.run_test("FC")
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assert_stopped(driver, gpio)

    def test_invalid_tof_decisions_raise_lane_fault_and_stop(self):
        decisions = [("end", 1.0, "Unsupported"), ("drive", -0.1, "Negative"),
                     ("drive", 1.1, "Too fast"), ("drive", math.nan, "NaN"),
                     ("drive", math.inf, "Infinity"), ("drive", "bad", "Text"),
                     ("drive",), None]
        for decision in decisions:
            with self.subTest(decision=decision):
                control, driver, gpio, _, _ = self.make(
                    tof_gate=lambda now: ("drive", 1.0, "Clear") if now < 0.6 else decision)
                with self.assertRaises(motor.LaneFault):
                    control.run_test("FC")
                self.assertTrue(any(duty > 0 for _, _, duty in gpio.pwm_writes))
                self.assert_stopped(driver, gpio)

    def test_stale_tof_fault_stops_fc_while_the_camera_runs(self):
        def tof_gate(now):
            if now >= 0.6:
                raise tof.ToFFault("ToF reading stale")
            return "drive", 1.0, "Clear"

        control, driver, gpio, clock, _ = self.make(tof_gate=tof_gate, labels=lambda now: ("red_light",))
        with self.assertRaisesRegex(tof.ToFFault, "stale"):
            control.run_test("FC")
        self.assertEqual(clock.now, 0.6)
        self.assert_stopped(driver, gpio)

    def test_red_seen_during_a_tof_hold_still_stops_fc_at_the_front_line(self):
        control, driver, gpio, clock, messages = self.make(
            [(3.0, "s")], tof_gate=lambda now: ("hold", 0.0, "Obstacle") if now < 1.0 else ("drive", 1.0, "Clear"),
            labels=lambda now: ("red_light",) if now >= 0.6 else ())
        gpio.readings = lambda now: (1, 1, 0 if now >= 1.6 else 1)
        with self.assertRaises(motor.StopRequested):
            control.run_test("FC")
        self.assertTrue(any(duty > 0 for now, _, duty in gpio.pwm_writes if 1.0 <= now < 1.6))
        self.assertTrue(all(duty == 0 for now, _, duty in gpio.pwm_writes if now >= 1.6))
        self.assertIn("Red light at the front line: waiting for green. 's' stops.", [text for _, text in messages])
        self.assert_stopped(driver, gpio)

    def test_stale_camera_fault_stops_even_while_tof_holds(self):
        control, driver, gpio, clock, _ = self.make(tof_gate=lambda now: ("hold", 0.0, "Obstacle"),
                                                    labels=lambda now: ())
        old_latest = control.light.stream.latest
        control.light.stream.latest = lambda: (old_latest() if clock.now < 0.6 else
                                               (_ for _ in ()).throw(vision.VisionFault("Camera reading stale")))
        with self.assertRaisesRegex(vision.VisionFault, "stale"):
            control.run_test("FC")
        self.assertEqual(clock.now, 0.6)
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assert_stopped(driver, gpio)

    def test_a_stop_sign_seen_during_a_tof_hold_ends_fc_at_the_next_front_line(self):
        control, driver, gpio, clock, messages = self.make(
            tof_gate=lambda now: ("hold", 0.0, "Obstacle") if now < 1.0 else ("drive", 1.0, "Clear"),
            labels=lambda now: ("stop_sign",) if now >= 0.6 else ())
        gpio.readings = lambda now: (1, 1, 0 if now >= 1.6 else 1)
        control.run_test("FC")
        self.assertEqual(clock.now, 1.6)
        self.assertIn("Stop sign: stopped at its front line. FC complete.", [text for _, text in messages])
        self.assert_stopped(driver, gpio)

    def test_front_ir_and_stop_key_take_priority_over_gate_failures(self):
        for stop_kind in ("front", "key"):
            with self.subTest(stop_kind=stop_kind):
                callback_times = []

                def tof_gate(now):
                    callback_times.append(now)
                    if now >= 0.6:
                        raise tof.ToFFault("Callback should not run after stop")
                    return "drive", 1.0, "Clear"

                control, driver, gpio, clock, _ = self.make(
                    [(0.6, "s")] if stop_kind == "key" else (), tof_gate)
                if stop_kind == "front":
                    gpio.readings = lambda now: (1, 1, 0 if now >= 0.6 else 1)
                    control.run_test("FC")
                else:
                    with self.assertRaises(motor.StopRequested):
                        control.run_test("FC")
                self.assertEqual(clock.now, 0.6)
                self.assertTrue(all(now < 0.6 for now in callback_times))
                self.assert_stopped(driver, gpio)

    def test_stop_key_takes_priority_over_a_failing_camera(self):
        control, driver, gpio, clock, _ = self.make([(0.6, "s")], labels=lambda now: ())
        old_latest = control.light.stream.latest
        control.light.stream.latest = lambda: (old_latest() if clock.now < 0.6 else
                                               (_ for _ in ()).throw(vision.VisionFault("Should not be read")))
        with self.assertRaises(motor.StopRequested):
            control.run_test("FC")
        self.assertEqual(clock.now, 0.6)
        self.assert_stopped(driver, gpio)

    def test_continuous_tof_hold_ends_at_existing_fc_time_limit(self):
        calls = []

        def gate(now):
            calls.append(now)
            return "hold", 0.0, "Obstacle"

        control, driver, gpio, clock, _ = self.make(tof_gate=gate)
        control.run_test("FC")
        self.assertEqual(clock.now, ir.FORWARD_SECONDS)
        self.assertTrue(all(now < ir.FORWARD_SECONDS for now in calls))
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assert_stopped(driver, gpio)

    def test_ir_display_does_not_start_either_fc_gate(self):
        def unexpected_gate(now):
            self.fail("IRC must not invoke an FC sensor gate")

        control, driver, gpio, _, _ = self.make([(0.4, "s")], unexpected_gate)
        with self.assertRaises(motor.StopRequested):
            control.run_test("IRC")
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assert_stopped(driver, gpio)


class ToFSessionIntegrationTests(unittest.TestCase):
    def make(self, events=(), distance=lambda now: 600, status=lambda now: 11):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        messages = []
        control = ir.ConfidenceTests(driver, FakeKeys(clock, events), clock,
                                     lambda text: messages.append((clock.now, text)))
        stream = FakeToFStream(clock, distance, status)
        opened = []

        def factory(bus, address):
            opened.append((bus, address))
            return stream

        session = tof.ToFSession(bus=2, address=0x30, stream_factory=factory)
        return control, session, stream, driver, gpio, clock, messages, opened

    def assert_stopped(self, driver, gpio):
        self.assertEqual(driver.duties, [0] * 4)
        self.assertTrue(all(pwm.duty == 0 for pwm in gpio.pwms))

    def test_session_starts_disabled_and_does_not_open_hardware_at_construction(self):
        _, session, _, _, _, _, _, opened = self.make()
        self.assertFalse(session.enabled)
        self.assertIsNone(session.stream)
        self.assertEqual(opened, [])
        session.close()
        self.assertEqual(opened, [])

    def test_tof_only_forward_uses_requested_bus_and_address_and_clears_hook(self):
        control, session, stream, driver, gpio, _, _, opened = self.make([(1.4, "s")])
        disabled_vision = types.SimpleNamespace(
            enabled=False, forward=lambda control: self.fail("Disabled vision started"))
        with self.assertRaises(motor.StopRequested):
            session.forward(control, disabled_vision)
        self.assertEqual(opened, [(2, 0x30)])
        self.assertTrue(any(duty > 0 for _, _, duty in gpio.pwm_writes))
        self.assertTrue(stream.closed)
        self.assertIsNone(control.tof_gate)
        self.assertIsNone(session.stream)
        self.assert_stopped(driver, gpio)

    def test_combined_sessions_drive_through_a_red_away_from_lines_and_close_both(self):
        control, session, stream, driver, gpio, clock, messages, _ = self.make()
        camera_stream = FakeStream(clock, lambda now: ("red_light",) if 0.2 <= now < 0.4 else ())
        camera_session = vision.VisionSession(stream_factory=lambda options: camera_stream)
        camera_session.enabled = True
        session.forward(control, camera_session)
        self.assertEqual(clock.now, ir.FORWARD_SECONDS)  # no front line: FC's time limit
        self.assertTrue(any(duty > 0 for _, _, duty in gpio.pwm_writes))
        self.assertTrue(stream.closed)
        self.assertTrue(camera_stream.closed)
        self.assertIsNone(control.tof_gate)
        self.assertIsNone(control.light)
        self.assert_stopped(driver, gpio)

    def test_combined_sessions_start_both_sensors_then_drive_and_cleanup_on_stop(self):
        control, session, stream, driver, gpio, clock, messages, _ = self.make([(1.4, "s")])
        camera_stream = FakeStream(clock)
        camera_session = vision.VisionSession(stream_factory=lambda options: camera_stream)
        camera_session.enabled = True
        with self.assertRaises(motor.StopRequested):
            session.forward(control, camera_session)
        started = max(t for t, text in messages if text.startswith(("Starting VL53L0X", "Starting Ryan")))
        self.assertTrue(all(duty == 0 for now, _, duty in gpio.pwm_writes if now <= started))
        self.assertTrue(any(duty > 0 for _, _, duty in gpio.pwm_writes))
        self.assertTrue(stream.closed)
        self.assertTrue(camera_stream.closed)
        self.assertIsNone(control.tof_gate)
        self.assertIsNone(control.light)
        self.assert_stopped(driver, gpio)

    def test_camera_startup_failure_closes_tof_without_powering_wheels(self):
        control, session, stream, driver, gpio, clock, _, _ = self.make()
        camera_stream = FakeStream(clock)

        def failure():
            raise vision.VisionFault("Injected camera startup failure")

        camera_stream.latest = failure
        camera_session = vision.VisionSession(stream_factory=lambda options: camera_stream)
        camera_session.enabled = True
        with self.assertRaisesRegex(vision.VisionFault, "startup failure"):
            session.forward(control, camera_session)
        self.assertTrue(stream.closed)
        self.assertTrue(camera_stream.closed)
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assertIsNone(control.tof_gate)
        self.assertIsNone(control.light)
        self.assert_stopped(driver, gpio)

    def test_distance_display_reports_valid_and_invalid_readings_with_motors_off(self):
        control, session, stream, driver, gpio, clock, messages, _ = self.make(
            [(1.6, "s")], status=lambda now: 11 if now < 0.2 else 4)
        with self.assertRaises(motor.StopRequested):
            session.readings(control)
        self.assertEqual(clock.now, 1.6)  # Invalid target must not close the diagnostic after 1s.
        self.assertTrue(any("60.0 cm" in text for _, text in messages))
        self.assertTrue(any("no target in range" in text and "status 4" in text for _, text in messages))
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assertTrue(stream.closed)
        self.assert_stopped(driver, gpio)

    def test_startup_timeout_or_error_closes_stream_without_movement(self):
        for fail in (False, True):
            with self.subTest(fail=fail):
                control, session, stream, driver, gpio, clock, _, _ = self.make()

                def failed_latest():
                    raise tof.ToFFault("Injected missing sensor")

                stream.latest = failed_latest if fail else lambda: None
                with self.assertRaises(tof.ToFFault):
                    session.forward(control)
                self.assertLessEqual(clock.now, tof.TOF_START_TIMEOUT)
                self.assertTrue(stream.closed)
                self.assertIsNone(control.tof_gate)
                self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
                self.assert_stopped(driver, gpio)

    def test_stop_key_cancels_tof_startup_without_waiting_for_timeout(self):
        control, session, stream, driver, gpio, clock, _, _ = self.make([(0.08, "s")])
        stream.latest = lambda: None
        with self.assertRaises(motor.StopRequested):
            session.forward(control)
        self.assertEqual(clock.now, 0.08)
        self.assertTrue(stream.closed)
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assertIsNone(control.tof_gate)
        self.assert_stopped(driver, gpio)

    def test_unclosed_prior_stream_cannot_create_duplicate_sensor_owner(self):
        control, session, stream, driver, gpio, _, _, _ = self.make()
        stream.close = lambda: False
        session.stream = stream
        session.stream_factory = lambda bus, address: self.fail("Created a duplicate ToF owner")
        with self.assertRaisesRegex(tof.ToFFault, "still blocked"):
            session.forward(control)
        self.assertIs(session.stream, stream)
        self.assertIsNone(control.tof_gate)
        self.assert_stopped(driver, gpio)


class ToFMenuIntegrationTests(unittest.TestCase):
    def test_toggles_are_off_by_default_independent_and_only_change_fc(self):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        rpi = types.ModuleType("RPi")
        rpi.GPIO = gpio
        calls = []
        fake_vision = types.SimpleNamespace(
            enabled=False, close=lambda: calls.append("vision close"),
            forward=lambda control: calls.append("vision FC"),
            readings=lambda control: calls.append("RV"))

        def tof_forward(control, vision=None):
            self.assertIs(vision, fake_vision)
            calls.append("ToF and vision FC" if vision.enabled else "ToF FC")

        fake_tof = types.SimpleNamespace(
            enabled=False, close=lambda: calls.append("ToF close"),
            forward=tof_forward, readings=lambda control: calls.append("TOF"))
        fake_control = types.SimpleNamespace(run_test=lambda command: calls.append(command))
        choices = iter(["FC", "TT", "FC", "VT", "FC", "IRC", "RV", "TOF",
                        "TT", "FC", "VT", "FC", "Q"])
        with patch.dict(sys.modules, {"RPi": rpi, "RPi.GPIO": gpio}), \
             patch.object(menu.sys.stdin, "isatty", return_value=True), \
             patch.object(menu, "PowerGuard", FakeGuard), \
             patch.object(menu, "VisionSession", return_value=fake_vision), \
             patch.object(menu, "ToFSession", return_value=fake_tof), \
             patch.object(menu, "ConfidenceTests", return_value=fake_control), \
             patch.object(motor, "prompt_line", side_effect=lambda text: next(choices)), \
             patch.object(motor, "Keyboard", side_effect=lambda: contextlib.nullcontext(FakeKeys(clock))), \
             contextlib.redirect_stdout(io.StringIO()):
            with tempfile.TemporaryDirectory() as folder:
                self.assertEqual(menu.main(["--bot", "500", "--settings", str(Path(folder) / "settings.json")]), 0)
        self.assertEqual(calls[:-2], ["FC", "ToF FC", "ToF and vision FC", "IRC", "RV", "TOF", "vision FC", "FC"])
        self.assertEqual(set(calls[-2:]), {"vision close", "ToF close"})
        self.assertFalse(fake_tof.enabled)
        self.assertFalse(fake_vision.enabled)
        self.assertTrue(gpio.cleaned)
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))


if __name__ == "__main__":
    unittest.main(verbosity=2)

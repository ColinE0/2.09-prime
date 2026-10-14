"""v9.9: camera traffic lights at the front line, plus the line-following pieces from v9.8.

With the camera on (VT), a confirmed green lets every lane mode cross a front line without
stopping; red or yellow stops at the line until green; nothing seen is the usual stop. The
real MotorDriver, Controller, GyroLane and TrafficLight run against the v9.4 lane stand-in
and a scripted camera whose detections follow the robot's travel.
"""

from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

import app
import go_button as go
import ir_confidence as ir
import lane_gyro
import motors_optimized as motor
import robot_profiles
import robot_settings as settings
import terminal_ui as ui
import vision_support as vision
from test_go_button import StubMotors
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys
from test_v9_4 import Rig
from test_vision import FakeStream, sample


class CameraRig(Rig):
    """The v9.4 rig with the camera on: ``labels(rig, now)`` is what each frame shows."""

    def __init__(self, labels=lambda rig, now: (), **kwargs):
        super().__init__(**kwargs)
        # Every poll reads the camera, so the stand-in moves through waits and timed moves
        # too (it otherwise integrates only when the IR or gyro is read).
        self.camera = FakeStream(self.clock, lambda now: (self.robot.step(self.clock.now), labels(self, now))[1])
        self.control.light = vision.TrafficLight(self.camera, self.control.log)

    def times(self, prefix):
        return [t for t, text in self.messages if text.startswith(prefix)]

    def stopped_between(self, start, end):
        return any(duty == 0 for t, _, duty in self.gpio.pwm_writes if start < t < end)


def near(lo, hi, *labels):
    """Detections shown while the robot's travel is between lo and hi metres."""
    return lambda rig, now: labels if lo <= rig.robot.odometer < hi else ()


def go_run(rig):
    keys = go.ButtonKeys(rig.control.keys, StubMotors(rig.driver), rig.clock)
    rig.control.keys = keys
    timer = go.RunTimer(rig.clock)
    keys.begin_run(timer, None)
    timer.start()
    return keys, timer


class Case(unittest.TestCase):
    def setUp(self):
        settings.apply_settings(deepcopy(settings.DEFAULTS))
        self.addCleanup(settings.apply_settings, deepcopy(settings.DEFAULTS))


class TrafficLightTests(Case):
    def make(self):
        stream = types.SimpleNamespace(latest=lambda: sample())
        messages = []
        return vision.TrafficLight(stream, messages.append), stream, messages

    def feed(self, light, stream, index, labels=(), area=0.03):
        stream.latest = lambda: sample(index + 1, index / 10, labels, area)
        light.update(index / 10)

    def test_line_stays_armed_until_two_fresh_frames_show_green(self):
        light, stream, _ = self.make()
        self.feed(light, stream, 0, ("green_light",))
        for now in (0.02, 0.04):  # the same frame again is not a second frame
            light.update(now)
        self.assertFalse(light.crossing(0.05))
        self.feed(light, stream, 1, ("green_light",))
        self.assertTrue(light.crossing(0.1))

    def test_red_or_yellow_after_green_rearms_the_line(self):
        for color in ("red", "yellow"):
            with self.subTest(color=color):
                light, stream, messages = self.make()
                for i in (0, 1):
                    self.feed(light, stream, i, ("green_light",))
                self.assertTrue(light.crossing(0.1))
                for i in (2, 3):
                    self.feed(light, stream, i, (color + "_light",))
                self.assertFalse(light.crossing(0.3))
                self.assertEqual(light.state, color)
                self.assertIn(f"Camera: {color} light, front stop line armed", messages)

    def test_red_outranks_green_in_one_window(self):
        light, stream, _ = self.make()
        self.feed(light, stream, 0, ("green_light",))
        self.feed(light, stream, 1, ("red_light", "green_light"))
        self.feed(light, stream, 2, ("red_light",))
        self.assertEqual(light.state, "red")
        self.assertFalse(light.crossing(0.2))

    def test_a_green_crosses_for_green_hold_after_its_last_confirming_frame(self):
        light, stream, _ = self.make()
        for i in (0, 1):
            self.feed(light, stream, i, ("green_light",))
        for i in (2, 3, 4):
            self.feed(light, stream, i)
        # Frame 2 still makes 2 greens of the last 3, so 0.2 s is the last confirmation.
        self.assertTrue(light.crossing(0.2 + vision.GREEN_HOLD))
        self.assertFalse(light.crossing(0.21 + vision.GREEN_HOLD))

    def test_the_light_just_passed_is_ignored_until_it_is_out_of_view(self):
        light, stream, _ = self.make()
        for i in (0, 1):
            self.feed(light, stream, i, ("green_light",))
        light.went_on()
        for i in (2, 3, 4):  # the same lamp, still in frame after the line
            self.feed(light, stream, i, ("green_light",))
        self.assertFalse(light.crossing(0.4))
        for i in (5, 6):
            self.feed(light, stream, i)
        for i in (7, 8):  # the next intersection's light
            self.feed(light, stream, i, ("green_light",))
        self.assertTrue(light.crossing(0.8))

    def test_a_stop_sign_ends_the_run_at_the_next_line_and_never_crosses(self):
        light, stream, messages = self.make()
        for i in (0, 1):
            self.feed(light, stream, i, ("green_light", "stop_sign"))
        self.assertFalse(light.crossing(0.1))
        light.at_line(0.1)
        self.assertEqual(light.decision(), "stop_sign")
        self.assertIn("Camera: stop sign, the next front line ends the run", messages)

    def test_stopped_at_a_line_only_what_is_seen_from_it_counts(self):
        light, stream, _ = self.make()
        for i in (0, 1):
            self.feed(light, stream, i, ("red_light",))  # well before the line
        for i in (2, 3):
            self.feed(light, stream, i)
        light.at_line(0.35)
        for i in (4, 5, 6, 7):
            self.feed(light, stream, i)
        self.assertEqual(light.decision(), "none")
        for i in (8, 9):
            self.feed(light, stream, i, ("red_light",))
        self.assertEqual(light.decision(), "red")
        self.assertFalse(light.green())
        for i in (10, 11):
            self.feed(light, stream, i, ("green_light",))
        self.assertTrue(light.green())

    def test_a_red_in_the_last_frames_before_the_stop_still_counts_at_the_line(self):
        light, stream, _ = self.make()
        for i in (0, 1):
            self.feed(light, stream, i, ("red_light",))
        light.at_line(0.15)
        self.feed(light, stream, 2)  # red is still 2 of the last 3 frames
        self.assertEqual(light.decision(), "red")

    def test_detections_below_the_action_size_do_not_count(self):
        light, stream, _ = self.make()
        small = vision.settings.LIGHT_ACT_AREA_FRAC / 2
        for i in range(3):
            self.feed(light, stream, i, ("green_light",), area=small)
        self.assertEqual(light.confirmed, set())
        self.assertFalse(light.crossing(0.2))
        self.assertEqual(light.preview(), "stop")

    def test_preview_tells_rv_what_a_line_would_get(self):
        for labels, expected in ((("green_light",), "cross"), (("yellow_light",), "wait"),
                                 (("red_light", "green_light"), "wait"), (("stop_sign",), "end"), ((), "stop")):
            with self.subTest(labels=labels):
                light, stream, _ = self.make()
                for i in (0, 1):
                    self.feed(light, stream, i, labels)
                self.assertEqual(light.preview(), expected)

    def test_stale_and_out_of_order_frames_fault(self):
        light, stream, _ = self.make()
        self.feed(light, stream, 2)
        stream.latest = lambda: sample(2, 0.1)
        with self.assertRaisesRegex(vision.VisionFault, "order"):
            light.update(0.3)
        light, stream, _ = self.make()
        stream.latest = lambda: sample(1, 0.0)
        with self.assertRaisesRegex(vision.VisionFault, "stale"):
            light.update(1.0)


class LaneModeTests(Case):
    def test_fa_crosses_a_green_line_without_stopping_then_stops_where_no_light_is(self):
        rig = CameraRig(near(0.2, 0.6, "green_light"), controller=motor.Controller,
                        lines=(0.5, 1.2), margin=10.0, events=[(8.0, "s")])
        with self.assertRaises(motor.StopRequested):
            rig.control.forward(auto=True)
        crossed = rig.times("Green light: crossing the front line without stopping")
        detected = rig.times("Line detected")
        self.assertEqual(len(crossed), 1)
        self.assertEqual(len(detected), 2)
        self.assertFalse(rig.stopped_between(0.5, detected[1]))
        self.assertIn("No light confirmed from the front line", rig.text())
        self.assertIn("Press 'g' to continue", rig.text())
        self.assertGreater(rig.robot.odometer, 1.2)
        rig.assert_off(self)

    def test_fa_red_stops_at_the_line_waits_for_green_then_drives_across(self):
        labels = lambda rig, now: (("red_light",) if now < 4.0 else ("green_light",)) if rig.robot.odometer > 0.2 else ()
        rig = CameraRig(labels, controller=motor.Controller, lines=(0.5,), margin=10.0, events=[(6.0, "s")])
        with self.assertRaises(motor.StopRequested):
            rig.control.forward(auto=True)
        stop = rig.times("Line detected")[0]
        self.assertIn("Red light at the front line: waiting for green", rig.text())
        self.assertTrue(all(duty == 0 for t, _, duty in rig.gpio.pwm_writes if stop <= t < 4.1))
        self.assertTrue(any(duty > 0 for t, _, duty in rig.gpio.pwm_writes if t >= 4.1))
        self.assertGreater(rig.robot.odometer, 0.6)  # drove on across the line after green
        rig.assert_off(self)

    def test_a_green_seen_long_before_the_line_has_expired_there(self):
        rig = CameraRig(near(0.0, 0.15, "green_light"), controller=motor.Controller,
                        lines=(0.6,), margin=10.0, events=[(6.0, "s")])
        with self.assertRaises(motor.StopRequested):
            rig.control.forward(auto=True)
        self.assertNotIn("Green light: crossing the front line without stopping", rig.text())
        self.assertIn("No light confirmed from the front line", rig.text())

    def test_the_green_just_passed_cannot_carry_the_robot_over_the_next_line(self):
        rig = CameraRig(lambda rig, now: ("green_light",), controller=motor.Controller,
                        lines=(0.5, 0.8), margin=10.0, events=[(5.0, "s")])
        with self.assertRaises(motor.StopRequested):
            rig.control.forward(auto=True)
        self.assertEqual(len(rig.times("Green light: crossing the front line without stopping")), 1)
        second = rig.times("Line detected")[1]
        self.assertTrue(rig.stopped_between(second - 0.01, second + 0.01))
        self.assertIn("Green light seen from the front line: going on", rig.text())

    def test_a_go_run_clocks_the_stop_not_the_look_and_ends_at_the_unlit_line(self):
        rig = CameraRig(near(0.2, 0.6, "green_light"), controller=motor.Controller, lines=(0.5, 1.2), margin=10.0)
        keys, timer = go_run(rig)
        rig.control.forward(auto=True)
        self.assertIn("Front line: stopped. Run complete.", rig.text())
        stop = rig.times("Line detected")[1]
        self.assertAlmostEqual(timer.stopped, stop)
        self.assertGreaterEqual(rig.clock.now, stop + vision.LOOK_SECONDS - 1e-9)
        self.assertTrue(keys.moved)
        rig.assert_off(self)

    def test_a_stop_sign_ends_the_run_at_its_line_without_a_look(self):
        rig = CameraRig(near(0.1, 9.0, "stop_sign"), controller=motor.Controller, lines=(0.5,), margin=10.0)
        _, timer = go_run(rig)
        rig.control.forward(auto=True)
        self.assertIn("Stop sign: stopped at its front line. Run complete.", rig.text())
        self.assertAlmostEqual(rig.clock.now, rig.times("Line detected")[0])
        self.assertAlmostEqual(timer.stopped, rig.clock.now)

    def test_camera_off_keeps_the_v9_5_stop_at_every_line(self):
        rig = Rig(controller=motor.Controller, lines=(0.5,), margin=10.0, events=[(4.0, "s")])
        with self.assertRaises(motor.StopRequested):
            rig.control.forward(auto=True)
        self.assertIn("Front line: stopped. Press 'g'", rig.text())
        self.assertNotIn("light", rig.text().lower().replace("line detected", ""))


class CourseTests(Case):
    def test_c_green_rolls_through_red_waits_and_the_final_lane_ends_at_the_stop_sign(self):
        def labels(rig, now):
            odometer = rig.robot.odometer
            if 0.2 <= odometer < 0.55:
                return ("green_light",)
            if 1.0 <= odometer < 1.55:
                return ("red_light",) if now < 9.0 else ("green_light",)
            if odometer >= 1.9:
                return ("stop_sign",)
            return ()

        rig = CameraRig(labels, controller=motor.Controller, lines=(0.5, 1.5, 2.5), margin=10.0)
        rig.control.run_mode("C", "FF")
        text = rig.text()
        self.assertIn("Intersection reached on a green light: F without stopping", text)
        self.assertIn("Red light at the front line: waiting for green", text)
        self.assertIn("Final lane: driving to the stop sign's front line", text)
        self.assertTrue(text.rstrip().endswith("Stop sign: course complete"))
        self.assertFalse(rig.stopped_between(0.3, rig.times("Line detected")[1] - 0.05))
        self.assertGreater(rig.robot.odometer, 2.5)
        rig.assert_off(self)

    def test_c_without_a_light_stops_the_course_time_then_goes_on(self):
        rig = CameraRig(controller=motor.Controller, lines=(0.5, 1.5), margin=10.0)
        rig.control.run_mode("C", "F")
        text = rig.text()
        self.assertIn(f"stopped {motor.COURSE_STOP_TIME:g}s while the camera looks", text)
        self.assertIn("No light confirmed from the front line", text)
        self.assertIn("no stop sign confirmed", text)

    def test_cg_crosses_on_green_and_re_zeroes_while_waiting_at_a_red(self):
        def labels(rig, now):
            odometer = rig.robot.odometer
            if 0.15 <= odometer < 0.35:
                return ("green_light",)
            if 0.7 <= odometer < 1.0:
                return ("red_light",) if now < 10.0 else ("green_light",)
            return ()

        rig = CameraRig(labels, lines=(0.3, 0.9, 1.6), margin=10.0, gyro_bias=0.3)
        rig.control.run("CG", "FF")
        text = rig.text()
        self.assertIn("Intersection reached on a green light: F without stopping", text)
        self.assertIn("Red light at the front line: waiting for green", text)
        self.assertIn("Gyro re-zeroed while stopped", text)
        self.assertIn("Course complete at the final front line", text)
        rig.assert_off(self)

    def test_cg_with_the_camera_off_is_unchanged(self):
        rig = Rig(lines=(0.3,), margin=10.0)
        rig.control.run("CG", "F")
        self.assertIn("Intersection reached; waiting", rig.text())
        self.assertIn("Gyro course complete", rig.text())


class FCTests(Case):
    def make(self, labels=lambda now: (), front=lambda now: 1, events=()):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        gpio.readings = lambda now: (1, 1, front(now))
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        messages = []
        control = ir.ConfidenceTests(driver, FakeKeys(clock, events), clock,
                                     lambda text: messages.append((clock.now, text)))
        control.light = vision.TrafficLight(FakeStream(clock, labels), control.log)
        return control, gpio, clock, messages

    def test_fc_crosses_a_green_line_and_keeps_driving(self):
        control, gpio, _, messages = self.make(lambda now: ("green_light",),
                                               lambda now: 0 if 0.6 <= now < 0.7 else 1, [(1.5, "s")])
        with self.assertRaises(motor.StopRequested):
            control.run_test("FC")
        self.assertIn("Green light: crossing the front line without stopping", [text for _, text in messages])
        self.assertFalse(any(duty == 0 for t, _, duty in gpio.pwm_writes if 0.5 < t < 1.5))

    def test_fc_red_waits_at_the_line_without_using_up_its_time_limit(self):
        labels = lambda now: ("red_light",) if now < 12.0 else ("green_light",)
        control, gpio, clock, messages = self.make(labels, lambda now: 0 if 0.6 <= now < 0.7 else 1, [(13.0, "s")])
        with self.assertRaises(motor.StopRequested):
            control.run_test("FC")
        texts = [text for _, text in messages]
        self.assertIn("Red light at the front line: waiting for green. 's' stops.", texts)
        self.assertNotIn("Confidence test time limit reached", texts)
        self.assertTrue(all(duty == 0 for t, _, duty in gpio.pwm_writes if 0.6 <= t < 12.1))
        self.assertTrue(any(duty > 0 for t, _, duty in gpio.pwm_writes if t >= 12.1))

    def test_fc_ends_at_the_stop_sign_line_not_where_it_saw_the_sign(self):
        control, gpio, clock, messages = self.make(lambda now: ("stop_sign",) if now >= 0.3 else (),
                                                   lambda now: 0 if now >= 1.0 else 1)
        control.run_test("FC")
        self.assertEqual(clock.now, 1.0)
        self.assertIn("Stop sign: stopped at its front line. FC complete.", [text for _, text in messages])
        self.assertTrue(any(duty > 0 for t, _, duty in gpio.pwm_writes if 0.5 <= t < 1.0))

    def test_fc_with_no_light_ends_at_the_line_after_the_look(self):
        control, _, clock, messages = self.make(front=lambda now: 0 if now >= 0.6 else 1)
        control.run_test("FC")
        self.assertAlmostEqual(clock.now, 0.6 + vision.LOOK_SECONDS)
        self.assertIn("Front line: confidence test stopped", [text for _, text in messages])


class FrontLineAndApproachTests(Case):
    def make(self, readings, events=()):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        gpio.readings = readings
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        messages = []
        control = motor.Controller(driver, FakeKeys(clock, events), clock,
                                   lambda text: messages.append((clock.now, text)))
        return control, driver, gpio, clock, messages

    def test_a_one_read_white_glitch_is_not_a_line(self):
        control, _, _, clock, messages = self.make(
            lambda now: (1, 1, 0 if abs(now - 0.3) < 0.005 or now >= 0.6 else 1))
        self.assertEqual(control.drive_to_line(), "stopped")
        self.assertEqual(clock.now, 0.62)
        line = [text for _, text in messages if text.startswith("Line detected")]
        self.assertEqual(len(line), 1)
        self.assertIn("front white 2 reads in a row, side IR L/R=1/1, straight", line[0])
        self.assertIn("slowest tick 20 ms (target 20)", line[0])

    def test_approach_slowdown_is_off_by_default_and_scales_both_duties_when_on(self):
        control, driver, gpio, _, messages = self.make(lambda now: (1, 1, 0 if now >= 3.0 else 1))
        control.drive_to_line()
        self.assertEqual(max(duty for _, _, duty in gpio.pwm_writes), motor.LEFT_SPEED)
        self.assertFalse(any("Slowing" in text for _, text in messages))
        with patch.object(motor, "APPROACH_SLOW_AFTER", 1.0):
            control, driver, gpio, _, messages = self.make(lambda now: (1, 1, 0 if now >= 3.0 else 1))
            control.drive_to_line()
        late = [duty for t, _, duty in gpio.pwm_writes if 1.0 <= t < 3.0 and duty > 0]
        self.assertEqual(late[-1], motor.LEFT_SPEED * motor.APPROACH_SPEED_PCT / 100)
        self.assertEqual(sum("Slowing to 60% for the front line" == text for _, text in messages), 1)

    def test_fg_balance_rescales_slow_approach_corrections(self):
        rig = Rig(margin=10.0)
        rig.control.start_heading()
        rig.driver.set_pattern(motor.FORWARD)
        rig.driver.set_duties((10.0,) * 4)
        rig.control.tracker.angle = 0.5

        def balance(scale):
            rig.control.balance_sum = rig.control.integral = 0.0
            rig.control.settled_since, rig.control.last_steer = 0.0, 1.0
            rig.control.approach_scale = scale
            rig.control.steer((15.0,) * 4, "straight", 1.1)
            return rig.control.balance_sum

        self.assertNotEqual(balance(1.0), 0.0)
        self.assertAlmostEqual(balance(0.5), 2 * balance(1.0))


class WiringTests(Case):
    def test_vt_routes_every_lane_mode_through_the_camera_and_nothing_else(self):
        calls = []
        vision_on = types.SimpleNamespace(enabled=True, attach=lambda control, run: calls.append("camera") or run())
        with patch.object(app, "GyroLane") as lane, patch.object(app.motor, "Controller") as controller:
            for command in ("F", "FA", "C", "CG", "FG", "SR", "TR"):
                app.dispatch(command, "F" if command in ("C", "CG") else "", "motors", "keys", "session",
                             None, vision_on, None, lambda text: None)
        self.assertEqual(calls, ["camera"] * 5)
        self.assertEqual(lane.return_value.run.call_count, 2)
        self.assertEqual(controller.return_value.run_mode.call_count, 5)

    def test_go_counts_down_after_the_camera_starts_for_camera_runs_only(self):
        on, off = types.SimpleNamespace(enabled=True), types.SimpleNamespace(enabled=False)
        self.assertTrue(app.camera_first("FC", off))
        self.assertTrue(app.camera_first("CG", on))
        self.assertFalse(app.camera_first("CG", off))
        self.assertFalse(app.camera_first("SR", on))

    def test_attach_starts_the_camera_then_counts_down_then_runs_and_always_closes(self):
        clock = FakeClock()
        stream = FakeStream(clock)
        session = vision.VisionSession(stream_factory=lambda options: stream)
        order = []
        control = types.SimpleNamespace(clock=clock, log=lambda text: None, light=None,
                                        motors=types.SimpleNamespace(stop=lambda brake=None: order.append("stop")),
                                        poll=lambda: None, tick_sleep=lambda started: clock.sleep(0.02),
                                        go_ready=lambda: order.append("countdown"))
        session.attach(control, lambda: order.append(("run", type(control.light).__name__)))
        self.assertEqual(order[-3:-1], ["countdown", ("run", "TrafficLight")])
        self.assertIsNone(control.light)
        self.assertTrue(stream.closed)

    def test_armed_session_waits_for_the_camera_when_told_to(self):
        self.assertIn("sensors_first", go.armed_session.__code__.co_varnames)


class SettingsTests(Case):
    def test_action_sizes_apply_from_the_profile_and_default_to_colins_pick(self):
        self.assertEqual(settings.DEFAULTS["camera_light_act_pct"], 0.04)
        self.assertEqual(settings.DEFAULTS["camera_stop_sign_act_pct"], 1.2)
        config = settings.validate(dict(deepcopy(settings.DEFAULTS), camera_light_act_pct=0.5,
                                        camera_stop_sign_act_pct=2.0))
        settings.apply_settings(config)
        self.assertAlmostEqual(vision.settings.LIGHT_ACT_AREA_FRAC, 0.005)
        self.assertAlmostEqual(vision.settings.STOPSIGN_ACT_AREA_FRAC, 0.02)

    def test_approach_settings_apply_and_default_off(self):
        self.assertEqual(settings.DEFAULTS["approach_slow_after"], 0.0)
        config = settings.validate(dict(deepcopy(settings.DEFAULTS), approach_slow_after=2.0, approach_speed_pct=70.0))
        settings.apply_settings(config)
        self.assertEqual((motor.APPROACH_SLOW_AFTER, motor.APPROACH_SPEED_PCT), (2.0, 70.0))
        groups = {field.group for field in settings.FIELDS.values()}
        self.assertIn("Line approach", groups)

    def test_a_v9_5_profile_loads_and_gains_the_new_settings(self):
        old = {key: value for key, value in settings.validate({}, "100rpm").items()
               if not key.startswith(("approach_", "camera_light_", "camera_stop_sign_"))}
        loaded = settings.validate(old, "100rpm")
        self.assertEqual(loaded["approach_slow_after"], 0.0)
        self.assertEqual(loaded["camera_light_act_pct"], 0.04)


KALEB_500RPM = {  # profiles/500rpm.json from Kaleb's v9.8 zip (2026-10-05), as saved on bot 2
    "reversed": [False, False, False, False], "ir_pins": [21, 7, 26],
    "motor_channels": ["M1A", "M1B", "M2B", "M2A"], "left_speed": 25.0, "right_speed": 25.0,
    "ramp_rate": 35.0, "auto_slew_rate": 70.0, "brake_on_stop": True, "direction_pause": 0.05,
    "auto_secondary_speed": 15.0, "auto_debounce": 0.04, "auto_min_correct_left": 0.3,
    "auto_min_correct_right": 0.2, "auto_overshoot_left": 0.1, "auto_overshoot_right": 0.1,
    "auto_max_correct": 0.8, "auto_settle": 0.3, "auto_yaw_trim": 0.0, "approach_slow_after": 3.0,
    "approach_speed_pct": 60.0, "diagonal_yaw_trim": 0.0, "diagonal_secondary_speed": 11.0,
    "strafe_duration": 0.6, "turn_duration_right": 0.12, "turn_duration_left": 0.14,
    "turn_approach_time_right": 0.2, "turn_approach_time_left": 0.7, "turn_exit_straight_time": 1.0,
    "course_cross_time": 0.4, "course_exit_straight_time": 1.0, "diagonal_duration": 0.6,
    "course_stop_time": 1.0, "fc_max_duty": 25.0, "fc_uncertain_duty": 20.0, "fc_filter_seconds": 0.06,
    "single_motor_duty": 25.0, "single_motor_seconds": 1.0, "camera_backend": "csi", "camera_index": 0,
    "camera_rotate_180": True, "camera_auto_exposure": False, "camera_startup_timeout": 30.0,
    "camera_exposure_us": 4000, "camera_analogue_gain": 1.0, "imu_drive_duty": 25.0,
    "imu_turn_max_duty": 25.0, "imu_turn_min_duty": 16.0, "imu_heading_kp": 0.6, "imu_heading_kd": 0.075,
    "imu_max_heading_trim": 5.0, "imu_turn_slow_gain": 0.4, "imu_turn_stop_lead_s": 0.0,
    "imu_log_timed_turns": True, "gyro_lane_ki": 0.15, "gyro_lane_ref_step": 0.5, "gyro_lane_ref_max": 4.0,
    "gyro_lane_rezero": True, "power_guard": True, "power_overcurrent_a": 4.5,
    "power_overcurrent_hold_s": 0.2, "power_low_voltage_v": 6.4, "power_low_voltage_hold_s": 2.0,
    "motion_profile": "n20-500rpm-v1", "robot_id": "500rpm", "tach_connector": "j8",
    "tach_reference_duty": 25.0, "wheel_calibration_enabled": False, "wheel_calibration": {},
}


class KalebProfileTests(Case):
    def write(self, root, folder, config):
        path = root / folder / "profiles" / "500rpm.json"
        path.parent.mkdir(parents=True)
        path.write_text(json.dumps(config), encoding="utf-8")
        return path

    def test_bot_2_starts_from_kalebs_latest_tuning_ahead_of_an_older_v9_5_copy(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = self.write(root, "motor9_7", KALEB_500RPM)
            self.write(root, "MPU6050_Motor_Test_v9_5", settings.validate({"left_speed": 22.0}, "500rpm"))
            original = source.read_bytes()
            _, loaded, report = robot_profiles.load_profile(root / "MPU6050_Motor_Test_v9_9", "500")
            self.assertEqual(source.read_bytes(), original)  # his file is read, never written
        self.assertTrue(any("motor9_7" in line for line in report))
        for key in ("auto_min_correct_left", "auto_min_correct_right", "auto_secondary_speed",
                    "approach_slow_after", "course_stop_time", "turn_approach_time_right"):
            self.assertEqual(loaded[key], KALEB_500RPM[key], key)
        self.assertEqual(loaded["camera_light_act_pct"], 0.04)  # his file predates the setting: today's default

    def test_v9_10_starts_from_the_v9_9_tuning_ahead_of_kalebs(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self.write(root, "motor9_7", KALEB_500RPM)
            self.write(root, "MPU6050_Motor_Test_v9_9",
                       settings.validate(dict(KALEB_500RPM, course_stop_time=2.0), "500rpm"))
            _, loaded, report = robot_profiles.load_profile(root / "MPU6050_Motor_Test_v9_10", "500")
        self.assertTrue(any("MPU6050_Motor_Test_v9_9" in line for line in report))
        self.assertEqual(loaded["course_stop_time"], 2.0)

    def test_bot_1_on_colins_pi_still_carries_over_from_v9_5(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            path = root / "MPU6050_Motor_Test_v9_5" / "profiles" / "100rpm.json"
            path.parent.mkdir(parents=True)
            path.write_text(json.dumps(settings.validate({"imu_turn_stop_lead_s": 0.01}, "100rpm")),
                            encoding="utf-8")
            _, loaded, report = robot_profiles.load_profile(root / "MPU6050_Motor_Test_v9_9", "100")
        self.assertEqual(loaded["imu_turn_stop_lead_s"], 0.01)
        self.assertTrue(any("MPU6050_Motor_Test_v9_5" in line for line in report))


class ButtonPinTests(Case):
    def test_each_bot_gets_its_own_start_and_reset_pins(self):
        bot2, bot1 = settings.validate({}, "500rpm"), settings.validate({}, "100rpm")
        self.assertEqual((bot2["button_start"], bot2["button_reset"]), ("gpio9", "off"))
        self.assertEqual((bot1["button_start"], bot1["button_reset"]), ("gpio10", "gpio9"))
        settings.apply_settings(bot2)
        self.assertEqual((motor.START_BTN, motor.RESET_BTN), (9, None))
        settings.apply_settings(bot1)
        self.assertEqual((motor.START_BTN, motor.RESET_BTN), (10, 9))

    def test_start_and_reset_cannot_share_a_pin(self):
        with self.assertRaisesRegex(ValueError, "cannot use the START pin"):
            settings.validate({"button_start": "gpio9", "button_reset": "gpio9"}, "100rpm")

    def test_with_reset_off_only_start_is_claimed_and_reset_never_reads_pressed(self):
        from test_board_ui import ButtonGPIO
        settings.apply_settings(settings.validate({}, "500rpm"))
        clock = FakeClock()
        gpio = ButtonGPIO(clock, [(9, 0.0, 9.0)])  # START (GPIO9) held
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        driver.claim_board_ui()
        self.assertIn(9, gpio.setups)
        self.assertNotIn(10, gpio.setups)
        self.assertEqual(driver.read_buttons(), (True, False))

    def test_btn_and_go_say_when_reset_is_off(self):
        from board_ui import ButtonTests
        from test_board_ui import ButtonGPIO
        settings.apply_settings(settings.validate({}, "500rpm"))
        clock = FakeClock()
        driver = motor.MotorDriver(ButtonGPIO(clock))
        driver.initialize()
        lines = []
        with self.assertRaises(motor.StopRequested):
            ButtonTests(driver, FakeKeys(clock, [(0.2, "s")]), clock, lines.append).run_test("BTN")
        self.assertTrue(any("Press START (GPIO9); RESET is off" in line for line in lines))
        self.assertEqual(lines[-1], "Buttons: START pressed 0 times.")
        written = []
        keys = types.SimpleNamespace(poll=lambda: "s")
        go.armed_session("SR", "", types.SimpleNamespace(read_buttons=lambda: (False, False), claim_board_ui=lambda: None,
                                                      set_status_led=lambda on: None, duties=[0.0] * 4),
                         execute=None, live_factory=None, keyboard=lambda: _Null(keys), clock=clock,
                         write=written.append)
        self.assertTrue(any("'s' stops a run and returns to the menu (RESET is off)" in line for line in written))


class _Null:
    def __init__(self, value):
        self.value = value

    def __enter__(self):
        return self.value

    def __exit__(self, *exc):
        return False


class DisplayTests(Case):
    def test_rv_lines_keep_the_line_decision_in_the_compact_summary(self):
        output = io.StringIO()
        output.isatty = lambda: True
        status = ui.LiveStatus(output)
        status("Camera: green_light center=(320, 80) (0.12% frame) | confirmed: green_light | line: cross"
               " | frame age 0.05s | processing 0.04s")
        self.assertIn("line cross", output.getvalue())


if __name__ == "__main__":
    unittest.main()

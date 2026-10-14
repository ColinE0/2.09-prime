"""Review regressions using real controllers and drivers, with recorded fake hardware."""

from copy import deepcopy
import unittest
from unittest.mock import patch

import ir_confidence as ir
import lane_gyro
import motors_optimized as motor
import robot_settings as settings
import vision_support as vision
from test_v9_4 import Rig
from test_vision import FakeStream


class HardeningCase(unittest.TestCase):
    def setUp(self):
        settings.apply_settings(deepcopy(settings.DEFAULTS))
        self.addCleanup(settings.apply_settings, deepcopy(settings.DEFAULTS))

    def rig(self, mode="F", labels=lambda now: (), events=()):
        controller = (lane_gyro.GyroLane if mode in ("FG", "CG") else
                      ir.ConfidenceTests if mode == "FC" else motor.Controller)
        rig = Rig(controller=controller, yaw_per_duty=0, margin=10, events=events)
        self.addCleanup(rig.driver.close)
        rig.control.light = vision.TrafficLight(FakeStream(rig.clock, labels), rig.control.log)
        # All modes start on clear floor, then reach the same white front line.
        rig.gpio.readings = lambda now: (1, 1, 0 if 0.3 <= now < 1.7 else 1)
        return rig

    def run_lane(self, rig, mode):
        if mode == "FG":
            rig.control.run("FG")
        elif mode == "FC":
            rig.control.run_test("FC")
        else:
            rig.control.run_mode(mode)

    def assert_stayed_off(self, rig, since):
        at_start = {}
        for t, pin, duty in rig.gpio.pwm_writes:
            if t <= since:
                at_start[pin] = duty
        self.assertEqual(len(at_start), 4)
        self.assertTrue(all(duty == 0 for duty in at_start.values()), at_start)
        writes = [duty for t, _, duty in rig.gpio.pwm_writes if t >= since]
        self.assertTrue(all(duty == 0 for duty in writes), writes)
        rig.assert_off(self)


class StoppedLightTests(HardeningCase):
    def test_newest_hazard_at_look_end_prevents_restart_in_each_lane_mode(self):
        for mode in ("F", "FA", "FG", "FC"):
            for color in ("red", "yellow"):
                with self.subTest(mode=mode, color=color):
                    # One green before arrival; the second is confirmed while stopped.
                    # The first hazard lands exactly at the end of the camera look.
                    labels = lambda now: (() if now < 0.3 else ("green_light",)
                                          if now < 0.9 else (color + "_light",))
                    rig = self.rig(mode, labels, [(1.3, "s")])
                    with self.assertRaises(motor.StopRequested):
                        self.run_lane(rig, mode)
                    self.assert_stayed_off(rig, 0.9)
                    self.assertIn("waiting for green", rig.text())

    def test_newest_hazard_vetoes_a_green_confirmation_during_red_wait(self):
        for mode in ("F", "FG"):
            for color in ("red", "yellow"):
                with self.subTest(mode=mode, color=color):
                    labels = lambda now: (("red_light",) if now < 0.6 else
                                          ("green_light",) if now < 0.7 else
                                          () if now < 0.8 else ("green_light", color + "_light"))
                    rig = self.rig(mode, labels, [(1.2, "s")])
                    if mode == "FG":
                        rig.control.start_heading()
                    with self.assertRaises(motor.StopRequested):
                        rig.control.look_at_line()
                    self.assertFalse(rig.control.light.green())
                    self.assert_stayed_off(rig, 0)

    def test_hazard_wait_releases_when_subsequent_frames_confirm_green(self):
        for mode in ("F", "FG"):
            with self.subTest(mode=mode):
                labels = lambda now: (("green_light",) if now < 0.6 or now >= 1.0
                                      else ("red_light",))
                rig = self.rig(mode, labels, [(2.0, "s")])
                if mode == "FG":
                    rig.control.start_heading()
                self.assertEqual(rig.control.look_at_line(), "green")
                self.assertGreaterEqual(rig.clock.now, 1.1)
                self.assertLess(rig.clock.now, 1.2)

    def test_courses_do_not_treat_newest_hazard_as_unknown_and_continue(self):
        for mode in ("C", "CG"):
            for color in ("red", "yellow"):
                with self.subTest(mode=mode, color=color):
                    labels = lambda now: ("green_light",) if now < 3.0 else (color + "_light",)
                    rig = self.rig(mode, labels, [(3.5, "s")])
                    if mode == "CG":
                        rig.control.start_heading()
                    with self.assertRaises(motor.StopRequested):
                        rig.control.intersection("F", "stopped")
                    self.assertFalse(rig.control.light.passed)
                    self.assert_stayed_off(rig, 0)

    def test_stop_sign_during_red_wait_ends_each_lane_mode_without_restart(self):
        for mode in ("F", "FA", "FG", "FC"):
            with self.subTest(mode=mode):
                labels = lambda now: (("red_light",) if now < 1.3 else ("green_light",)) + (
                    ("stop_sign",) if now >= 1.1 else ())
                rig = self.rig(mode, labels, [(2.0, "s")])
                self.run_lane(rig, mode)
                self.assertGreaterEqual(rig.clock.now, 1.2)
                self.assertLess(rig.clock.now, 1.3)  # ends even if green never arrives
                self.assert_stayed_off(rig, 1.1)
                self.assertIn("Stop sign: stopped at its front line", rig.text())
                self.assertFalse(rig.control.light.passed)

    def test_stop_sign_during_red_wait_ends_both_final_lane_controllers(self):
        for mode in ("C", "CG"):
            with self.subTest(mode=mode):
                labels = lambda now: ("red_light",) + (("stop_sign",) if now >= 1.1 else ())
                rig = self.rig(mode, labels, [(2.0, "s")])
                if mode == "CG":
                    rig.control.start_heading()
                rig.control.final_lane()
                self.assertLess(rig.clock.now, 1.3)
                self.assert_stayed_off(rig, 1.1)
                self.assertIn("Stop sign: course complete", rig.text())

    def test_intermediate_course_sign_keeps_existing_continue_policy(self):
        for mode in ("C", "CG"):
            with self.subTest(mode=mode):
                labels = lambda now: ("red_light",) + (("stop_sign",) if now >= 3.2 else ())
                rig = self.rig(mode, labels, [(4.0, "s")])
                if mode == "CG":
                    rig.control.start_heading()
                rig.control.intersection("F", "stopped")
                self.assertGreaterEqual(rig.clock.now, 3.3)
                self.assertLess(rig.clock.now, 3.5)
                self.assertIn("the course has more legs, so F goes on", rig.text())
                self.assertTrue(rig.control.light.passed)

    def test_unknown_and_aged_stopped_green_keep_existing_course_policy(self):
        for mode in ("C", "CG"):
            for green in (False, True):
                with self.subTest(mode=mode, green=green):
                    labels = lambda now: ("green_light",) if green and 0.1 <= now <= 0.2 else ()
                    rig = self.rig(mode, labels)
                    if mode == "CG":
                        rig.control.start_heading()
                    rig.control.intersection("F", "stopped")
                    self.assertAlmostEqual(rig.clock.now, 3.0, delta=0.02)
                    self.assertTrue(rig.control.light.passed)
                    self.assertIn("Green light seen" if green else "No light confirmed", rig.text())

    def test_stale_camera_fault_still_interrupts_red_wait(self):
        rig = self.rig("F", lambda now: ("red_light",), [(2.0, "s")])
        rig.control.light.update(0)
        frame = rig.control.light.previous
        rig.control.light.stream.latest = lambda: frame
        with self.assertRaisesRegex(vision.VisionFault, "stale"):
            rig.control.run_mode("F")
        self.assert_stayed_off(rig, 0.34)


class TimedGyroTests(HardeningCase):
    def moving_rig(self, rate=1.0, initial_speed=True):
        settings.apply_settings(settings.validate({"ramp_rate": rate}))
        rig = self.rig("FG", events=[(29.0, "s")])
        rig.robot.gyro_extra = lambda now: 0.3 if round(now / motor.LOOP_DT) % 2 == 0 else -0.3
        rig.control.start_heading()
        if initial_speed:
            rig.driver.set_pattern(motor.FORWARD)
            rig.driver.set_duties(motor.straight_targets())
        return rig

    def test_noisy_gyro_cannot_extend_an_already_moving_timed_hold(self):
        for rate in (1.0, 35.0):
            with self.subTest(rate=rate):
                rig = self.moving_rig(rate)
                rig.control.hold_forward(0.4)
                self.assertAlmostEqual(rig.clock.now, 0.4, delta=motor.LOOP_DT)
                self.assertTrue(any(rig.driver.duties))  # seamless course handoff remains

    def test_noisy_gyro_from_rest_keeps_acceleration_then_a_bounded_hold(self):
        for rate in (1.0, 35.0):
            with self.subTest(rate=rate):
                rig = self.moving_rig(rate, initial_speed=False)
                expected = max(motor.straight_targets()) / rate + 0.4
                rig.control.hold_forward(0.4)
                self.assertAlmostEqual(rig.clock.now, expected, delta=2 * motor.LOOP_DT)
                self.assertTrue(any(rig.driver.duties))

    def test_slow_ticks_abort_acceleration_and_disable_outputs(self):
        rig = self.moving_rig(35.0, initial_speed=False)
        # Fresh gyro each poll, but ticks at 100 ms cannot achieve the requested
        # acceleration because the real motor driver caps each slew to 20 ms.
        with patch.object(rig.control, "tick_sleep", side_effect=lambda now: rig.clock.sleep(0.1)):
            with self.assertRaisesRegex(motor.LaneFault, "deadline"):
                rig.control.hold_forward(0.4)
        self.assertLess(rig.clock.now, 2.3)
        rig.assert_off(self)

    def test_operator_stop_still_interrupts_timed_course_motion(self):
        rig = self.moving_rig()
        rig.control.keys.events = [(0.2, "s")]
        with self.assertRaises(motor.StopRequested):
            rig.control.hold_forward(0.4)
        self.assertLessEqual(rig.clock.now, 0.22)
        rig.assert_off(self)

    def test_delayed_poll_past_hold_deadline_faults_before_more_pwm(self):
        rig = self.moving_rig()
        poll = rig.control.poll

        def delayed_poll():
            result = poll()
            if rig.clock.now >= 0.2:
                rig.clock.sleep(2.0)  # simulate a call returning late
            return result

        with patch.object(rig.control, "poll", side_effect=delayed_poll):
            with self.assertRaisesRegex(motor.LaneFault, "deadline"):
                rig.control.hold_forward(0.4)
        self.assert_stayed_off(rig, 2.2)


if __name__ == "__main__":
    unittest.main()

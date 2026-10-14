"""Profile migration and real controller/driver checks for independent turn controls."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import imu_tests
import motors_optimized as motor
import robot_settings as settings
from motion_profile import PROFILE_IDS
from test_v9_4 import Rig


class TurnControlTests(unittest.TestCase):
    def setUp(self):
        settings.apply_settings(deepcopy(settings.DEFAULTS))
        self.addCleanup(settings.apply_settings, deepcopy(settings.DEFAULTS))

    def rig(self, **kwargs):
        rig = Rig(margin=10, **kwargs)
        self.addCleanup(rig.driver.close)
        return rig

    def test_older_tagged_profiles_keep_custom_ramp_and_save_independent_values(self):
        with tempfile.TemporaryDirectory() as folder:
            for bot, normal in (("500rpm", 47), ("100rpm", 123)):
                with self.subTest(bot=bot):
                    path = Path(folder) / (bot + ".json")
                    path.write_text(json.dumps({"robot_id": bot, "motion_profile": PROFILE_IDS[bot],
                                                "ramp_rate": normal}), encoding="utf-8")
                    config = settings.load_settings(path, bot)
                    self.assertEqual(config["turn_ramp_rate"], normal)
                    self.assertEqual(config["turn_approach_speed_pct"], 100)
                    config.update(turn_ramp_rate=28, turn_approach_speed_pct=75)
                    settings.save_settings(path, config, bot)
                    loaded = settings.load_settings(path, bot)
                    settings.apply_settings(loaded, bot)
                    self.assertEqual((motor.RAMP_RATE, motor.TURN_RAMP_RATE,
                                      motor.TURN_APPROACH_SPEED_PCT), (normal, 28, 75))

    def test_new_robot_defaults_match_original_acceleration(self):
        for bot, rate in (("500rpm", 35), ("100rpm", 100)):
            with self.subTest(bot=bot):
                config = settings.validate({}, bot)
                self.assertEqual((config["ramp_rate"], config["turn_ramp_rate"]), (rate, rate))

    def test_controls_reject_nonfinite_boolean_and_out_of_range_values(self):
        for key, values in (("turn_ramp_rate", (0, 501, True, float("nan"))),
                            ("turn_approach_speed_pct", (0, 101, False, float("inf")))):
            for value in values:
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    settings.validate({key: value})

    def test_turn_ramp_only_changes_rising_power_and_keeps_tick_cap(self):
        settings.apply_settings(settings.validate({"ramp_rate": 80, "turn_ramp_rate": 20}))
        rig = self.rig()
        rig.driver.set_pattern(motor.TURN_RIGHT)
        rig.driver.set_duties((5,) * 4)
        rig.driver.slew_turn((10, 0, 10, 0), 1.0)  # delayed tick still capped at 20 ms
        for actual, expected in zip(rig.driver.duties, (5.4, 3.4, 5.4, 3.4)):
            self.assertAlmostEqual(actual, expected)
        rig.driver.set_pattern(motor.FORWARD)
        rig.driver.slew((10,) * 4, motor.RAMP_RATE, motor.LOOP_DT)
        self.assertTrue(all(abs(duty - 1.6) < 1e-9 for duty in rig.driver.duties))

    def test_timed_turns_use_separate_acceleration_but_straights_do_not(self):
        settings.apply_settings(settings.validate({"ramp_rate": 50, "turn_ramp_rate": 20,
                                                   "left_speed": 20, "right_speed": 20}))
        for command, hold in (("R", motor.TURN_DURATION_RIGHT), ("L", motor.TURN_DURATION_LEFT)):
            with self.subTest(command=command):
                rig = self.rig(controller=motor.Controller)
                rig.control.run_mode(command)
                self.assertAlmostEqual(rig.clock.now, 20 / 20 + hold, delta=motor.LOOP_DT)
        rig = self.rig(controller=motor.Controller)
        rig.control.timed_move(motor.FORWARD, motor.straight_targets(), 0.1)
        self.assertAlmostEqual(rig.clock.now, 20 / 50 + 0.1, delta=motor.LOOP_DT)

    def test_cg_and_isolated_imu_turns_use_the_same_independent_ramp(self):
        settings.apply_settings(settings.validate({"ramp_rate": 80, "turn_ramp_rate": 20}))
        for mode in ("CG", "ITR", "ITL"):
            with self.subTest(mode=mode):
                rig = self.rig()
                if mode == "CG":
                    rig.control.start_heading()
                    rig.control.measured_turn("right", "1 R")
                else:
                    control = imu_tests.IMUTests(rig.driver, rig.control.keys, rig.session,
                                                 rig.clock, rig.control.log)
                    control.run_test(mode)
                first_power = next(duty for _, _, duty in rig.gpio.pwm_writes if duty > 0)
                self.assertAlmostEqual(first_power, 20 * motor.LOOP_DT)
                rig.assert_off(self)

    def test_timed_intersection_approach_scales_power_but_not_turn_power(self):
        settings.apply_settings(settings.validate({"turn_approach_speed_pct": 50}))
        for direction in ("left", "right"):
            with self.subTest(direction=direction):
                rig = self.rig(controller=motor.Controller)
                original = rig.control.timed_move
                completed = []

                def record(pattern, targets, hold):
                    original(pattern, targets, hold)
                    completed.append((pattern, tuple(rig.driver.duties)))

                with patch.object(rig.control, "timed_move", side_effect=record):
                    rig.control.turn_maneuver(direction)
                self.assertEqual(completed[0], (motor.FORWARD, (12.5,) * 4))
                self.assertEqual(completed[1][1], (25,) * 4)

    def test_cg_scales_only_the_pre_turn_approach_not_crossing_or_exit(self):
        settings.apply_settings(settings.validate({"turn_approach_speed_pct": 50}))
        for route in ("R", "L", "F"):
            with self.subTest(route=route):
                rig = self.rig()
                rig.control.start_heading()
                original = rig.control.hold_forward
                completed = []

                def record(duration, **kwargs):
                    original(duration, **kwargs)
                    completed.append(max(rig.driver.duties))

                with patch.object(rig.control, "drive_to_line", return_value="stopped"), \
                        patch.object(rig.control, "intersection"), \
                        patch.object(rig.control, "hold_forward", side_effect=record):
                    rig.control.course(route)
                self.assertAlmostEqual(completed[0], 25 if route == "F" else 12.5)
                self.assertAlmostEqual(completed[-1], 25)

    def test_green_crossing_decelerates_to_reduced_approach_target_with_bounded_hold(self):
        rig = self.rig()
        rig.control.start_heading()
        rig.driver.set_pattern(motor.FORWARD)
        rig.driver.set_duties((25,) * 4)
        rig.control.hold_forward(0.2, speed_pct=50)
        self.assertTrue(all(abs(duty - 12.5) < 1e-9 for duty in rig.driver.duties))
        self.assertAlmostEqual(rig.clock.now, 12.5 / motor.RAMP_RATE + 0.2,
                               delta=2 * motor.LOOP_DT)

    def test_stop_key_still_interrupts_reduced_approach(self):
        rig = Rig(events=[(0.1, "s")], margin=10)
        self.addCleanup(rig.driver.close)
        rig.control.start_heading()
        with self.assertRaises(motor.StopRequested):
            rig.control.hold_forward(0.3, speed_pct=50)
        rig.assert_off(self)


if __name__ == "__main__":
    unittest.main()

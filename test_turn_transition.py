"""Exercise approach-to-turn yaw using the real driver and IMU integration."""
from copy import deepcopy
import unittest

import imu_support as imu
import lane_gyro
import motors_optimized as motor
import robot_settings as settings
from test_v9_4 import Rig


class TurnTransitionTests(unittest.TestCase):
    def setUp(self):
        settings.apply_settings(deepcopy(settings.DEFAULTS))
        self.addCleanup(settings.apply_settings, deepcopy(settings.DEFAULTS))

    def rig(self, **kwargs):
        rig = Rig(margin=10, **kwargs)
        self.addCleanup(rig.driver.close)
        rig.control.start_heading()
        return rig

    def test_settling_yaw_before_turn_power_does_not_trip_direction_guard(self):
        for direction, sign in (("left", 1), ("right", -1)):
            with self.subTest(direction=direction):
                rig = self.rig()
                rig.driver.set_pattern(motor.FORWARD)
                rig.driver.set_duties((10, 10, 10, 10))
                # Four measured degrees opposite the upcoming turn accrue
                # during prepare's braking pause, while turn power is zero.
                rig.robot.gyro_extra = lambda now, sign=sign: (
                    -sign * 100 if now <= motor.DIRECTION_PAUSE else 0)
                rig.control.measured_turn(direction, direction)
                self.assertEqual(rig.control.reference, sign * 90)
                self.assertLess(abs(rig.control.tracker.angle - sign * 90), 2.5)
                rig.assert_off(self)

    def test_actual_wrong_direction_after_turn_power_still_faults(self):
        for direction in ("left", "right"):
            with self.subTest(direction=direction):
                rig = self.rig(yaw_per_duty=-3)
                with self.assertRaisesRegex(imu.IMUFault, "wrong measured direction") as raised:
                    rig.control.measured_turn(direction, direction)
                self.assertIn("requested " + direction, str(raised.exception))
                self.assertIn("commanded PWM", str(raised.exception))
                rig.assert_off(self)

    def test_settling_yaw_does_not_hide_a_stalled_turn(self):
        rig = self.rig(yaw_per_duty=0)
        rig.driver.set_pattern(motor.FORWARD)
        rig.driver.set_duties((10, 10, 10, 10))
        rig.robot.gyro_extra = lambda now: 100 if now <= motor.DIRECTION_PAUSE else 0
        with self.assertRaisesRegex(imu.IMUFault, "No measurable turn progress"):
            rig.control.measured_turn("right", "right")

    def test_two_right_turns_wait_out_residual_yaw_beyond_old_pause(self):
        rig = self.rig()
        rig.control.measured_turn("right", "1 R")
        rig.control.hold_forward(0.2)
        began = rig.clock.now
        # Opposite residual yaw outlasts the previous fixed 50 ms pause.
        rig.robot.gyro_extra = lambda now: 100 if began <= now <= began + 0.12 else 0
        writes = len(rig.gpio.pwm_writes)
        rig.control.measured_turn("right", "2 R")
        powered = [t for t, _, duty in rig.gpio.pwm_writes[writes:] if duty > 0]
        self.assertTrue(powered)
        self.assertGreaterEqual(min(powered) - began, 0.12 + lane_gyro.TURN_SETTLE_SECONDS)
        self.assertEqual(rig.control.reference, -180)
        self.assertLess(abs(rig.control.tracker.angle + 180), 2.5)
        self.assertIn("Turn 2 R start: settled", rig.text())
        rig.assert_off(self)

    def test_continuous_rotation_times_out_without_turn_power(self):
        rig = self.rig()
        rig.robot.gyro_extra = lambda now: 100
        with self.assertRaisesRegex(imu.IMUFault, "Turn start did not settle"):
            rig.control.measured_turn("right", "1 R")
        self.assertLessEqual(rig.clock.now, lane_gyro.TURN_SETTLE_TIMEOUT + motor.LOOP_DT)
        self.assertTrue(all(duty == 0 for _, _, duty in rig.gpio.pwm_writes))
        rig.assert_off(self)

    def test_a_new_yaw_pulse_restarts_the_quiet_window(self):
        rig = self.rig()
        rig.robot.gyro_extra = lambda now: 100 if 0.1 <= now <= 0.14 else 0
        elapsed, _ = rig.control.settle_turn_start()
        self.assertGreaterEqual(elapsed, 0.14 + lane_gyro.TURN_SETTLE_SECONDS)
        rig.assert_off(self)

    def test_repeated_old_samples_cannot_count_as_settled(self):
        rig = self.rig()
        frozen = rig.control.tracker.previous
        rig.stream.latest = lambda: frozen
        with self.assertRaisesRegex(imu.IMUFault, "stale"):
            rig.control.measured_turn("right", "1 R")
        self.assertTrue(all(duty == 0 for _, _, duty in rig.gpio.pwm_writes))
        rig.assert_off(self)

    def test_stop_key_remains_active_during_settle(self):
        rig = Rig(events=[(0.06, "s")], margin=10)
        self.addCleanup(rig.driver.close)
        rig.control.start_heading()
        with self.assertRaises(motor.StopRequested):
            rig.control.measured_turn("right", "1 R")
        self.assertTrue(all(duty == 0 for _, _, duty in rig.gpio.pwm_writes))
        rig.assert_off(self)


if __name__ == "__main__":
    unittest.main()

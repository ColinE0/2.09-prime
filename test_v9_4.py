"""v9.4: FG and CG, IR lane keeping with a gyro heading hold.

The real MotorDriver, Controller and GyroLane run against a kinematic stand-in
for the mecanum robot. The stand-in is a test fixture, not a model of either
physical robot: its gains only need to be plausible enough to exercise the
control logic.
"""

from copy import deepcopy
import json
import math
from pathlib import Path
import re
import signal
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import app
import imu_support as imu
import imu_tests
import lane_gyro
import motors_optimized as motor
import robot_profiles
import robot_settings as settings
import terminal_ui
from imu_tests import Session
from test_imu import DynamicStream
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys


class LanePlant:
    """Wheel duties in, gyro and IR out. y is metres left of lane centre;
    heading is degrees from the lane, positive left. Slowing a diagonal pair
    moves sideways (the IR lean); slowing one side rotates (the heading hold)."""

    KV = 0.012      # m/s forward per duty point
    KL = 0.006      # m/s sideways per duty point of diagonal difference

    def __init__(self, driver, clock, right_gain=1.0, yaw_per_duty=3.0, lean_yaw=0.0,
                 heading=0.0, y=0.0, lines=(), gyro_bias=0.0, margin=0.015, yaw_lag=0.0):
        self.driver, self.clock = driver, clock
        self.yaw_lag = yaw_lag  # s; above 0 the yaw rate lags the wheels, so turns coast
        self.right_gain, self.yaw_per_duty, self.lean_yaw = right_gain, yaw_per_duty, lean_yaw
        self.heading, self.y, self.lines = heading, y, tuple(lines)
        self.gyro_bias, self.margin = gyro_bias, margin
        self.gyro_extra = lambda now: 0.0
        self.rate = self.odometer = 0.0
        self.t = clock.now
        self.history = []

    def step(self, now):
        dt = now - self.t
        if dt <= 0:
            return
        pattern = self.driver.pattern
        if pattern is None:
            speeds = [0.0] * 4
        else:
            speeds = [duty if forward else -duty for duty, forward in zip(self.driver.duties, pattern)]
        fl, fr, bl, br = speeds
        fr, br = fr * self.right_gain, br * self.right_gain
        commanded = (self.yaw_per_duty * ((fr + br) - (fl + bl)) / 2
                     + self.lean_yaw * ((fr + bl) - (fl + br)) / 2)
        if self.yaw_lag > 0:
            self.rate += (commanded - self.rate) * (1 - math.exp(-dt / self.yaw_lag))
        else:
            self.rate = commanded
        forward = self.KV * (fl + fr + bl + br) / 4
        # A right lean (FR, BL slowed) moves right, as SR did on the mat (2026-09-24);
        # the model had this mirrored, matching the mirrored wheel pairs it tested.
        sideways = self.KL * ((fr + bl) - (fl + br)) / 2
        self.heading += self.rate * dt
        radians = math.radians(self.heading)
        self.y += (forward * math.sin(radians) + sideways * math.cos(radians)) * dt
        self.odometer += abs(forward) * dt
        self.t = now
        self.history.append((now, self.heading, self.y))

    def ir(self, now):
        self.step(now)
        left = 0 if self.y > self.margin else 1
        right = 0 if self.y < -self.margin else 1
        front = 0 if any(line <= self.odometer <= line + 0.03 for line in self.lines) else 1
        return (left, right, front)

    def imu_values(self, now):
        self.step(now)
        return (0.0, 0.0, self.rate + self.gyro_bias + self.gyro_extra(now)), (0.0, 0.0, 1.0)

    def worst_heading(self, after):
        return max(abs(heading) for t, heading, _ in self.history if t >= after)


class Rig:
    def __init__(self, events=(), calibrated=True, controller=lane_gyro.GyroLane, **plant):
        self.clock = clock = FakeClock()
        self.gpio = FakeGPIO(clock)
        self.driver = motor.MotorDriver(self.gpio)
        self.driver.initialize()
        self.robot = LanePlant(self.driver, clock, **plant)
        self.gpio.readings = self.robot.ir
        self.stream = DynamicStream(clock, self.robot.imu_values)
        self.session = Session(stream_factory=lambda bus, address: self.stream)
        self.session.stream = self.stream
        if calibrated:
            self.session.calibration = imu.Calibration((0, 0, 0), (0, 0, 1), 0, axis=2)
        self.messages = []
        keys = FakeKeys(clock, events)
        log = lambda text: self.messages.append((clock.now, text))
        if controller is lane_gyro.GyroLane:
            self.control = controller(self.driver, keys, self.session, clock, log)
        else:
            self.control = controller(self.driver, keys, clock, log)

    def text(self):
        return "\n".join(text for _, text in self.messages)

    def assert_off(self, case):
        case.assertEqual(self.driver.duties, [0.0] * 4)
        case.assertTrue(all(pwm.duty == 0 for pwm in self.gpio.pwms))


class SimulatedLaneCase(unittest.TestCase):
    def setUp(self):
        settings.apply_settings(deepcopy(settings.DEFAULTS))
        self.addCleanup(settings.apply_settings, deepcopy(settings.DEFAULTS))


class GyroLaneTests(SimulatedLaneCase):
    def run_fg(self, rig):
        with self.assertRaises(motor.StopRequested):
            rig.control.run("FG")
        rig.assert_off(self)

    def test_fa_has_no_heading_correction_but_fg_holds_heading(self):
        # Right side 4% fast: about 3 deg/s of yaw with nothing correcting it.
        fa = Rig(events=[(8.0, "s")], controller=motor.Controller, right_gain=1.04)
        with self.assertRaises((motor.LaneFault, motor.StopRequested)):
            fa.control.forward(auto=True)
        self.assertGreater(fa.robot.worst_heading(0), 5.0)

        fg = Rig(events=[(8.0, "s")], right_gain=1.04)
        self.run_fg(fg)
        self.assertLess(fg.robot.worst_heading(2.0), 1.5)
        self.assertNotIn("timed out", fg.text())

    def test_integral_removes_the_offset_a_motor_mismatch_leaves(self):
        wide = dict(right_gain=1.04, margin=10.0)  # no edge hits: heading hold alone
        with_integral = Rig(events=[(10.0, "s")], **wide)
        self.run_fg(with_integral)
        self.assertLess(abs(with_integral.robot.heading), 0.3)
        with patch.object(lane_gyro, "KI", 0.0):
            proportional_only = Rig(events=[(10.0, "s")], **wide)
            self.run_fg(proportional_only)
        # Steady offset = needed correction / kp, about 0.96 / 0.6 = 1.6 deg, to the left.
        self.assertGreater(proportional_only.robot.heading, 1.0)

    def test_auto_balance_suggests_slowing_the_fast_side(self):
        rig = Rig(events=[(10.0, "s")], right_gain=1.04, margin=10.0)
        self.run_fg(rig)
        line = next(text for _, text in rig.messages if text.startswith("Auto-balance"))
        self.assertIn("held the right side", line)
        left, right = map(float, re.search(r"left ([\d.]+), right ([\d.]+)", line).groups())
        self.assertEqual(left, motor.LEFT_SPEED)
        self.assertAlmostEqual(right, motor.RIGHT_SPEED / 1.04, delta=0.3)

    def test_crooked_start_is_trimmed_out_by_ir_nudges(self):
        # Placed 2 deg left of the lane; the gyro calls that zero.
        rig = Rig(events=[(20.0, "s")], heading=2.0)
        self.run_fg(rig)
        self.assertTrue(-2.6 <= rig.control.reference <= -1.4, rig.control.reference)
        corrections = [t for t, text in rig.messages if text.startswith("Correcting")]
        self.assertGreaterEqual(len(corrections), 3)
        self.assertFalse([t for t in corrections if t > 15.0])
        self.assertLess(abs(rig.robot.heading), 0.5)

    def test_lean_that_rotates_the_robot_is_held_straight(self):
        # Imperfect mecanum: each lean also yaws. Start past the left edge to force leans.
        # -0.5 yaws the robot LEFT while it leans right, the harmful case (FA drifts). It was
        # +0.5 before the 2026-09-25 lean-pair fix; the sign flipped with the slowed pair.
        fa = Rig(events=[(6.0, "s")], controller=motor.Controller, lean_yaw=-0.5, y=0.02)
        with self.assertRaises((motor.LaneFault, motor.StopRequested)):
            fa.control.forward(auto=True)
        fg = Rig(events=[(6.0, "s")], lean_yaw=-0.5, y=0.02)
        self.run_fg(fg)
        self.assertGreater(fa.robot.worst_heading(0), 1.5)
        self.assertLess(fg.robot.worst_heading(0), 1.0)

    def test_nudges_move_away_from_the_edge_and_stop_at_the_limit(self):
        rig = Rig()
        for _ in range(10):
            rig.control.lane_event("right", 0.0)
        self.assertEqual(rig.control.reference, -lane_gyro.REF_MAX)
        rig.control.lane_event("left", 0.0)
        self.assertEqual(rig.control.reference, -lane_gyro.REF_MAX + lane_gyro.REF_STEP)

    def test_requires_current_ic_and_iv_before_any_motion(self):
        for calibrated, missing_axis, message in ((False, False, "Run IC"), (True, True, "Run IV")):
            with self.subTest(message=message):
                rig = Rig(calibrated=calibrated)
                if missing_axis:
                    rig.session.calibration.axis = None
                with self.assertRaisesRegex(imu.IMUFault, message):
                    rig.control.run("FG")
                self.assertFalse(any(duty > 0 for _, _, duty in rig.gpio.pwm_writes))

    def test_stale_imu_data_stops_the_motors(self):
        rig = Rig(events=[(8.0, "s")])
        latest = rig.stream.latest

        def stale():
            reading = latest()
            if rig.clock.now > 1.0:
                return imu.Sample(reading.sequence, rig.clock.now - 0.5, reading.acceleration,
                                  reading.gyro, reading.temperature)
            return reading

        rig.stream.latest = stale
        with self.assertRaisesRegex(imu.IMUFault, "stale"):
            rig.control.run("FG")
        rig.assert_off(self)

    def test_wrong_iv_sign_trips_the_heading_limit(self):
        rig = Rig(events=[(20.0, "s")], right_gain=1.02, margin=10.0)
        rig.session.calibration.sign = -1  # heading hold now pushes the wrong way
        with self.assertRaisesRegex(imu.IMUFault, "from its target"):
            rig.control.run("FG")
        rig.assert_off(self)

    def test_power_guard_sees_every_fg_tick(self):
        rig = Rig(events=[(2.0, "s")])
        checks = []
        rig.driver.power_guard = types.SimpleNamespace(check=lambda motors: checks.append(rig.clock.now),
                                                       trip_reason=None)
        self.run_fg(rig)
        self.assertGreater(len(checks), 80)

    def test_base_controller_hooks_leave_fa_targets_unchanged(self):
        control = motor.Controller(None, None)
        targets = (25.0, 25.0, 14.0, 25.0)
        self.assertIs(control.steer(targets, "right", 0.0), targets)
        self.assertIsNone(control.lane_event("right", 0.0))


class GyroCourseTests(SimulatedLaneCase):
    def test_course_turns_are_measured_and_the_gyro_rezeroes_at_stops(self):
        # 0.3 deg/s of gyro offset appeared after IC. Side lines are out of reach here;
        # the plant's lane frame does not follow a turn.
        rig = Rig(lines=(0.3, 1.2), gyro_bias=0.3, margin=10.0)
        rig.control.run("CG", "L R")
        rig.assert_off(self)
        text = rig.text()
        self.assertIn("Gyro course complete", text)
        shifts = re.findall(r"yaw offset moved ([+-][\d.]+)", text)
        self.assertEqual(len(shifts), 2)
        self.assertAlmostEqual(float(shifts[0]), 0.3, delta=0.01)
        self.assertAlmostEqual(float(shifts[1]), 0.0, delta=0.01)
        turns = [float(value) for value in re.findall(r"Turn \d [LR]: measured ([\d.]+) deg", text)]
        self.assertEqual(len(turns), 2)
        for turned in turns:
            self.assertTrue(85 <= turned <= 95, turned)
        self.assertLess(abs(rig.robot.heading), 3.0)
        # The run's private copy took the new offset; the session's IC result did not change.
        self.assertAlmostEqual(rig.control.calibration.bias[2], 0.3, delta=0.01)
        self.assertEqual(rig.session.calibration.bias, (0, 0, 0))

    def test_wrong_turn_direction_stops(self):
        rig = Rig(lines=(0.3,), yaw_per_duty=-3.0, margin=10.0)
        with self.assertRaisesRegex(imu.IMUFault, "wrong measured direction"):
            rig.control.run("CG", "L")
        rig.assert_off(self)

    def test_rezero_refuses_a_moving_robot_and_a_short_stop(self):
        rig = Rig()
        rig.control.start_heading()
        rig.robot.gyro_extra = lambda now: 3.0 if int(now * 50) % 2 else -3.0
        rig.control.rezero(3.0)
        self.assertIn("not re-zeroed: robot not still", rig.text())
        rig.robot.gyro_extra = lambda now: 0.0
        rig.control.rezero(1.0)
        self.assertIn("stop was too short", rig.text())
        self.assertEqual(rig.control.calibration.bias, (0, 0, 0))
        self.assertEqual(rig.session.calibration.bias, (0, 0, 0))

    def test_invalid_course_is_rejected_before_motion(self):
        rig = Rig()
        with self.assertRaisesRegex(ValueError, "only F, R, and L"):
            rig.control.run("CG", "FX")
        self.assertFalse(any(duty > 0 for _, _, duty in rig.gpio.pwm_writes))


class GyroStopTests(SimulatedLaneCase):
    def test_front_line_wait_rezeros_and_starts_the_next_lane_from_the_current_heading(self):
        # 0.3 deg/s of offset after IC, and 14 s parked at a line: before this fix the
        # heading kept integrating and 'g' swung the robot about 4 deg off its lane.
        rig = Rig(events=[(18.0, "g"), (24.0, "s")], lines=(1.0,), gyro_bias=0.3)
        with self.assertRaises(motor.StopRequested):
            rig.control.run("FG")
        rig.assert_off(self)
        text = rig.text()
        shift = float(re.search(r"yaw offset moved ([+-][\d.]+)", text).group(1))
        self.assertAlmostEqual(shift, 0.3, delta=0.01)
        self.assertIn("Heading re-referenced", text)
        # The robot was 1 deg right of the lane at 'g'. IR nudges may turn it back toward
        # the lane; nothing may swing it further away (before the fix: about 4 deg more).
        at_g = [heading for t, heading, _ in rig.robot.history if t <= 18.0][-1]
        after = [heading for t, heading, _ in rig.robot.history if t > 18.0]
        self.assertLess(at_g, -0.5)
        self.assertGreater(min(after), at_g - 0.3)
        self.assertLess(max(after), 0.5)

    def test_g_waits_while_the_robot_is_being_moved(self):
        rig = Rig(events=[(8.0, "g"), (13.0, "g"), (16.0, "s")], lines=(0.5,))
        rig.robot.gyro_extra = lambda now: 20.0 if 7.0 < now < 9.0 else 0.0
        with self.assertRaises(motor.StopRequested):
            rig.control.run("FG")
        text = rig.text()
        self.assertIn("Robot is moving; hold it still", text)
        self.assertIn("not re-zeroed: robot not still", text)  # the last 5 s include the handling
        self.assertEqual(text.count("Heading re-referenced"), 1)
        self.assertTrue(any(t >= 13.0 for t, line in rig.messages if line.startswith("Heading re-referenced")))

    def test_expired_calibration_blocks_continuing_without_a_rezero(self):
        rig = Rig(events=[(15.0, "g")], lines=(0.5,))
        rig.session.calibration.timestamp = -290.0  # IC 290 s before the run: expires at 10 s
        with patch.object(lane_gyro, "REZERO", False), self.assertRaisesRegex(imu.IMUFault, "expired"):
            rig.control.run("FG")
        rig.assert_off(self)

    def test_rezero_drops_the_drift_integrated_while_stopped(self):
        rig = Rig(gyro_bias=0.3)
        rig.control.start_heading()
        before = rig.control.tracker.angle
        rig.control.rezero(3.0)
        self.assertLess(abs(rig.control.tracker.angle - before), 0.2)  # only the 0.5 s settle remains
        self.assertAlmostEqual(rig.control.calibration.bias[2], 0.3, delta=0.01)

    def test_rezero_refuses_a_large_offset_change(self):
        rig = Rig()
        rig.control.start_heading()
        rig.robot.gyro_bias = 2.5
        rig.control.rezero(3.0)
        self.assertIn("offset change +2.50", rig.text())
        self.assertEqual(rig.control.calibration.bias, (0, 0, 0))


class MeasuredTurnTests(SimulatedLaneCase):
    def turned(self, rig):
        return float(re.search(r"measured ([\d.]+) deg", rig.text()).group(1))

    def test_turn_aims_at_the_lane_heading_not_90_from_where_it_started(self):
        rig = Rig(margin=10.0)
        rig.control.start_heading()
        rig.control.tracker.angle = 3.0  # entered the intersection 3 deg left of the lane
        rig.control.measured_turn("left", "1 L")
        self.assertTrue(85.5 <= self.turned(rig) <= 88.5, self.turned(rig))
        self.assertLess(abs(rig.control.tracker.angle - rig.control.reference), 2.5)

    def test_nudge_limit_follows_the_lane_after_a_turn_and_warns_once(self):
        rig = Rig(margin=10.0)
        rig.control.start_heading()
        rig.control.measured_turn("left", "1 L")
        for _ in range(10):
            rig.control.lane_event("right", 0.0)
        self.assertEqual(rig.control.reference, 90.0 - lane_gyro.REF_MAX)
        self.assertEqual(rig.text().count("at its 4 deg limit"), 1)

    def test_no_rotation_stalls_and_slow_rotation_times_out(self):
        for gain, message in ((0.0, "No measurable turn progress"), (0.1, "timed out")):
            with self.subTest(message=message):
                rig = Rig(margin=10.0, yaw_per_duty=gain)
                rig.control.start_heading()
                with self.assertRaisesRegex(imu.IMUFault, message):
                    rig.control.measured_turn("right", "1 R")

    def test_stop_lead_cuts_power_before_the_target(self):
        results = []
        for lead in (0.0, 0.1):
            rig = Rig(margin=10.0)
            rig.control.start_heading()
            with patch.object(lane_gyro.imu, "TURN_STOP_LEAD", lead):
                rig.control.measured_turn("left", "1 L")
            results.append(self.turned(rig))
        self.assertLess(results[1], results[0] - 5.0)


class AutoBalanceTests(SimulatedLaneCase):
    def test_cg_leaves_auto_balance_to_fg(self):
        # Matched motors; the yaw lag makes each measured turn coast about 8 deg past its target.
        rig = Rig(lines=(0.3, 1.2), margin=10.0, yaw_lag=0.1)
        rig.control.run("CG", "LL")
        self.assertIn("Auto-balance is measured by FG on a straight lane", rig.text())
        self.assertNotIn("Suggested", rig.text())

    def test_restarts_from_rest_do_not_read_as_a_motor_mismatch(self):
        # Matched motors with yaw lag, crooked by 1 deg at each start: three lanes via 'g'.
        rig = Rig(events=[(6.0, "g"), (12.0, "g"), (19.0, "s")], lines=(1.0, 2.2), yaw_lag=0.1,
                  heading=1.0)
        with self.assertRaises(motor.StopRequested):
            rig.control.run("FG")
        line = next(text for _, text in rig.messages if text.startswith("Auto-balance"))
        self.assertIn("no baseline change suggested", line)

    def test_no_suggestion_after_a_fault(self):
        rig = Rig(margin=10.0)
        rig.control.start_heading()
        rig.control.balance_sum, rig.control.balance_time = 5.0, 5.0
        rig.control.report(balance=False)
        self.assertIn("Auto-balance skipped", rig.text())
        self.assertNotIn("Suggested", rig.text())

    def test_slow_runaway_from_a_wrong_iv_sign_suggests_nothing(self):
        rig = Rig(margin=10.0, yaw_per_duty=1.0, right_gain=1.01)
        rig.session.calibration.sign = -1
        with self.assertRaisesRegex(imu.IMUFault, "from its target"):
            rig.control.run("FG")
        self.assertNotIn("Suggested", rig.text())


class ControllerDetailTests(SimulatedLaneCase):
    """Direct checks on gates the simulated runs rarely reach."""

    def started(self, **plant):
        rig = Rig(margin=10.0, **plant)
        rig.control.start_heading()
        rig.driver.duties = [25.0] * 4  # steer() only reads whether the wheels are driven
        return rig

    def steer_for(self, rig, seconds, error, state="straight"):
        now = rig.control.last_steer or 0.0
        for _ in range(round(seconds / 0.02)):
            now += 0.02
            rig.control.tracker.angle = rig.control.reference + error
            rig.control.tracker.rate = 0.0
            rig.control.steer((25.0,) * 4, state, now)

    def test_auto_balance_counts_only_settled_straight_driving(self):
        rig = self.started()
        self.steer_for(rig, 1.0, 3.0)   # a transient: never counts
        self.steer_for(rig, 0.5, 0.5)   # settled, but not for long enough yet
        self.assertEqual(rig.control.balance_time, 0.0)
        self.steer_for(rig, 1.5, 0.5)   # 1 s after settling it counts
        self.assertAlmostEqual(rig.control.balance_time, 1.0, delta=0.05)
        counted = rig.control.balance_time
        self.steer_for(rig, 1.0, 0.5, state="right")  # an IR lean is not straight driving
        self.assertEqual(rig.control.balance_time, counted)

    def test_integrator_is_clamped_at_the_correction_limit(self):
        rig = self.started()
        self.steer_for(rig, 100.0, 4.0)
        self.assertAlmostEqual(abs(rig.control.integral), imu_tests.MAX_HEADING_TRIM / lane_gyro.KI)

    def test_integral_separation_limits_overshoot_after_a_coasting_turn(self):
        rig = Rig(lines=(0.3,), margin=10.0, yaw_lag=0.1)  # the turn coasts about 9 deg past
        rig.control.run("CG", "L")
        turn_end = next(t for t, text in rig.messages if text.startswith("Turn 1 L: measured"))
        after = [heading - 90.0 for t, heading, _ in rig.robot.history if t > turn_end + 0.5]
        self.assertGreater(after[0], 5.0)
        self.assertGreater(min(after), -0.8)  # integrating the landing error overshot about 1.9 deg

    def test_moving_robot_is_refused_before_any_motion(self):
        rig = Rig(margin=10.0)
        rig.robot.gyro_extra = lambda now: 10.0
        with self.assertRaisesRegex(imu.IMUFault, "Robot is moving"):
            rig.control.run("FG")
        self.assertFalse(any(duty > 0 for _, _, duty in rig.gpio.pwm_writes))

    def test_under_two_settled_seconds_says_why_there_is_no_suggestion(self):
        # v9.5: a short run used to end with no auto-balance line at all.
        rig = self.started()
        self.steer_for(rig, 2.0, 0.5)  # 1 s counted
        self.assertAlmostEqual(rig.control.balance_time, 1.0, delta=0.05)
        rig.control.report()
        self.assertIn("Auto-balance: only 1.0s of settled straight driving (needs 2s)", rig.text())
        self.assertNotIn("Suggested", rig.text())

    def test_sparse_imu_samples_skip_the_rezero(self):
        rig = Rig(margin=10.0)
        latest = rig.stream.latest

        def every_100_ms():
            reading = latest()
            tick = int(rig.clock.now * 10 + 1e-9)
            return imu.Sample(tick + 1, tick / 10, reading.acceleration, reading.gyro, reading.temperature)

        rig.stream.latest = every_100_ms
        rig.control.start_heading()
        rig.control.rezero(3.0)
        self.assertIn("too short to measure", rig.text())

class StopKeyAndSignalTests(unittest.TestCase):
    def test_ctrl_s_ctrl_z_and_ctrl_backslash_stop_like_s(self):
        keyboard = motor.Keyboard()
        keyboard.fd = 0
        for data, expected in ((b"\x13", "s"), (b"\x1a", "s"), (b"\x1c", "s"), (b"g", "g")):
            with self.subTest(data=data), \
                    patch.object(motor.select, "select", return_value=([0], [], [])), \
                    patch.object(motor.os, "read", return_value=data):
                self.assertEqual(keyboard.poll(), expected)

    def test_keyboard_disables_flow_control_suspend_and_quit_while_driving(self):
        calls = []
        termios = types.SimpleNamespace(
            TCSANOW=0, IXON=0x400, IXOFF=0x1000, VINTR=0, VQUIT=1, VSUSP=10, error=OSError,
            tcgetattr=lambda fd: [0x400 | 0x1000 | 0x2, 0, 0, 0, 0, 0,
                                  [bytes([n]) for n in (3, 28, 127, 21, 4, 0, 1, 0, 17, 19, 26)]],
            tcsetattr=lambda fd, when, attrs: calls.append(attrs))
        tty = types.SimpleNamespace(setcbreak=lambda fd, when: None)
        with patch.dict(sys.modules, {"termios": termios, "tty": tty}), \
                patch.object(motor.sys.stdin, "isatty", return_value=True), \
                patch.object(motor.sys.stdin, "fileno", return_value=0):
            with motor.Keyboard():
                configured = calls[-1]
        self.assertEqual(configured[0], 0x2)  # IXON and IXOFF cleared, other input flags kept
        self.assertEqual(configured[6][termios.VSUSP], b"\x00")
        self.assertEqual(configured[6][termios.VQUIT], b"\x00")
        self.assertEqual(configured[6][termios.VINTR], bytes([3]))  # Ctrl+C still exits with cleanup

    def test_stop_signals_raise_the_ctrl_c_cleanup_once_then_restore(self):
        original = signal.getsignal(signal.SIGTERM)
        with app.stop_on_signals():
            handler = signal.getsignal(signal.SIGTERM)
            with self.assertRaises(KeyboardInterrupt):
                handler(signal.SIGTERM, None)
            # A second signal must not cut the cleanup short.
            self.assertEqual(signal.getsignal(signal.SIGTERM), signal.SIG_IGN)
        self.assertEqual(signal.getsignal(signal.SIGTERM), original)

class SettingsAndMenuTests(unittest.TestCase):
    def test_per_bot_integral_defaults_and_nudge_validation(self):
        self.assertEqual(settings.validate({}, "500rpm")["gyro_lane_ki"], 0.15)
        self.assertEqual(settings.validate({}, "100rpm")["gyro_lane_ki"], 0.3)
        with self.assertRaisesRegex(ValueError, "gyro_lane_ref_step"):
            settings.validate({"gyro_lane_ref_step": 2.0, "gyro_lane_ref_max": 1.0}, "500rpm")
        groups = {field.group for key, field in settings.FIELDS.items() if key.startswith("gyro_lane_")}
        self.assertEqual(groups, {"Gyro lane (FG/CG)"})

    def test_fg_and_cg_are_powered_menu_commands_routed_to_gyro_lane(self):
        self.assertLessEqual({"FG", "CG"}, app.POWERED_COMMANDS)
        self.assertLessEqual({"FG", "CG"}, terminal_ui.COMMANDS)
        self.assertNotIn("CG", app.TURN_COMMANDS)  # CG measures its own turns
        vision = types.SimpleNamespace(enabled=False)
        with patch.object(app, "GyroLane") as lane:
            app.dispatch("CG", "FRL", "motors", "keys", "session", None, vision, None, print)
        lane.assert_called_once_with("motors", "keys", "session", log=print)
        lane.return_value.run.assert_called_once_with("CG", "FRL")

    def test_v9_3_profiles_migrate_first_and_gain_the_new_defaults(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for name, left in (("MPU6050_Motor_Test_v9_3", 22.0), ("MPU6050_Motor_Test_v9_2", 20.0)):
                # A v9.3 file has no gyro_lane_* keys.
                saved = {key: value for key, value in settings.validate({"left_speed": left}, "500rpm").items()
                         if not key.startswith("gyro_lane_")}
                path = root / name / "profiles" / "500rpm.json"
                path.parent.mkdir(parents=True)
                path.write_text(json.dumps(saved), encoding="utf-8")
            _, loaded, report = robot_profiles.load_profile(root / "MPU6050_Motor_Test_v9_4", "500")
        self.assertEqual(loaded["left_speed"], 22.0)
        self.assertEqual(loaded["gyro_lane_ki"], 0.15)
        self.assertTrue(any("MPU6050_Motor_Test_v9_3" in line for line in report))


if __name__ == "__main__":
    unittest.main()

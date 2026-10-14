"""CG obstacle protection against real controllers and simulated wheel/IMU data."""
from copy import deepcopy
import types
import unittest
from unittest.mock import patch
import app
import motors_optimized as motor
import robot_settings as settings
import tof_support as tof
from test_v9_4 import Rig
from test_tof_integration import FakeToFStream as BaseStream

class FakeToFStream(BaseStream):
    def latest(self):
        tick = int(self.clock.now * 10 + 1e-8)
        stamp = tick / 10
        return tof.ToFSample(tick + 1, stamp, self.distance(stamp), self.status(stamp))

class CGToFTests(unittest.TestCase):
    def setUp(self):
        settings.apply_settings(deepcopy(settings.DEFAULTS))
        self.addCleanup(settings.apply_settings, deepcopy(settings.DEFAULTS))

    def rig(self, **kwargs):
        rig = Rig(margin=10, **kwargs)
        self.addCleanup(rig.driver.close)
        return rig

    def gate(self, rig, distance):
        stream = FakeToFStream(rig.clock, distance, lambda now: 11)
        rig.control.tof_gate = tof.ToFGate(stream, log=lambda text: None).check
        return stream

    def assert_no_power(self, rig, start, end):
        values = [d for t, _, d in rig.gpio.pwm_writes if start <= t < end]
        self.assertTrue(values)
        self.assertTrue(all(d == 0 for d in values))

    def test_lane_brakes_and_resumes_after_fresh_clear_without_skipping_line(self):
        rig = self.rig(lines=(0.5,))
        self.gate(rig, lambda t: 80 if 1.2 <= t < 3.2 else 600)
        rig.control.start_heading()
        self.assertEqual(rig.control.drive_to_line(), "stopped")
        self.assert_no_power(rig, 1.21, 3.94)
        self.assertGreater(rig.clock.now, 3.94)
        self.assertTrue(any(t > 3.95 and d > 0 for t, _, d in rig.gpio.pwm_writes))
        rig.assert_off(self)

    def test_slow_zone_reduces_lane_targets(self):
        rig = self.rig(events=[(2, "s")])
        rig.control.tof_gate = lambda now: ("drive", 0.7, "90 mm")
        with self.assertRaises(motor.StopRequested):
            rig.control.run("CG", "F")
        self.assertAlmostEqual(max(d for _, _, d in rig.gpio.pwm_writes), motor.LEFT_SPEED * 0.7)
        rig.assert_off(self)

    def test_timed_move_preserves_remaining_hold_and_reaccelerates(self):
        baseline = self.rig()
        baseline.control.start_heading()
        baseline.control.hold_forward(1.0)
        rig = self.rig()
        rig.control.start_heading()
        rig.control.tof_gate = lambda t: ("hold", 0, "obstacle") if 1 <= t < 3 else ("drive", 1, "clear")
        rig.control.hold_forward(1.0)
        self.assert_no_power(rig, 1.01, 3)
        # Two seconds stopped plus a fresh acceleration; elapsed hold is not repeated.
        expected = baseline.clock.now + 2 + motor.LEFT_SPEED / motor.RAMP_RATE
        self.assertAlmostEqual(rig.clock.now, expected, delta=0.1)

    def test_turn_hold_longer_than_timeout_resumes_to_same_heading(self):
        rig = self.rig()
        rig.control.start_heading()
        rig.control.tof_gate = lambda t: ("hold", 0, "obstacle") if 0.5 <= t < 12 else ("drive", 1, "clear")
        rig.control.measured_turn("left", "test")
        self.assert_no_power(rig, 0.51, 12)
        self.assertAlmostEqual(rig.control.tracker.angle, 90, delta=3)
        rig.assert_off(self)

    def test_stop_key_remains_live_during_hold(self):
        rig = self.rig(events=[(1, "s")])
        rig.control.tof_gate = lambda now: ("hold", 0, "obstacle")
        with self.assertRaises(motor.StopRequested):
            rig.control.run("CG", "L")
        self.assert_no_power(rig, 0, 2)
        rig.assert_off(self)

    def test_sensor_fault_during_motion_stops_and_closes_session(self):
        rig = self.rig(lines=(0.5,))
        stream = FakeToFStream(rig.clock, lambda t: 600, lambda t: 11)
        original = stream.latest
        def latest():
            if rig.clock.now >= 1.2:
                raise tof.ToFFault("stale test reading")
            return original()
        stream.latest = latest
        session = tof.ToFSession(stream_factory=lambda *args: stream)
        with self.assertRaisesRegex(tof.ToFFault, "stale"):
            session.attach(rig.control, lambda: rig.control.run("CG", "F"))
        self.assertTrue(stream.closed)
        self.assertIsNone(rig.control.tof_gate)
        rig.assert_off(self)

    def test_full_course_runs_with_real_gate_and_cleanup(self):
        rig = self.rig(lines=(0.3, 1.2))
        stream = FakeToFStream(rig.clock, lambda t: 600, lambda t: 11)
        session = tof.ToFSession(stream_factory=lambda *args: stream)
        session.attach(rig.control, lambda: rig.control.run("CG", "LR"))
        self.assertIn("Gyro course complete", rig.text())
        self.assertTrue(stream.closed)
        self.assertIsNone(rig.control.tof_gate)
        rig.assert_off(self)

    def test_dispatch_starts_tof_before_camera_and_run(self):
        for camera in (False, True):
            events = []
            control = types.SimpleNamespace(go_ready=lambda: events.append("ready"),
                run=lambda *args: events.append("run"))
            session = types.SimpleNamespace(enabled=True,
                attach=lambda c, run: (events.append("tof"), run()))
            vision = types.SimpleNamespace(enabled=camera,
                attach=lambda c, run: (events.append("camera"), c.go_ready(), run()))
            with patch.object(app, "GyroLane", return_value=control):
                app.dispatch("CG", "F", None, None, None, None, vision, session, print)
            self.assertEqual(events, ["tof"] + (["camera"] if camera else []) + ["ready", "run"])
            self.assertTrue(app.camera_first("CG", vision, session))

    def test_camera_fault_is_checked_while_obstacle_holds(self):
        rig = self.rig()
        rig.control.tof_gate = lambda now: ("hold", 0, "obstacle")
        def update(now):
            if now >= 1:
                raise RuntimeError("camera stale")
        rig.control.light = types.SimpleNamespace(update=update)
        with self.assertRaisesRegex(RuntimeError, "camera stale"):
            rig.control.run("CG", "F")
        rig.assert_off(self)
        self.assert_no_power(rig, 0, 2)

    def test_turn_slowdown_scales_targets(self):
        rig = self.rig()
        rig.control.start_heading()
        rig.control.tof_gate = lambda now: ("drive", 0.7, "slow")
        rig.control.measured_turn("right", "slow")
        self.assertLessEqual(max(d for _, _, d in rig.gpio.pwm_writes), 0.7 * 25 + 1e-8)
        self.assertAlmostEqual(rig.control.tracker.angle, -90, delta=3)
        rig.assert_off(self)

    def test_invalid_gate_cannot_power_motors(self):
        rig = self.rig()
        rig.control.tof_gate = lambda now: ("drive", float("nan"), "bad")
        with self.assertRaises(ValueError):
            rig.control.run("CG", "F")
        rig.assert_off(self)

if __name__ == "__main__":
    unittest.main()

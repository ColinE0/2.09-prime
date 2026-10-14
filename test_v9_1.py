"""v9.1 additions: power guard, open-space ToF, documented wiring defaults,
gyro-measured timed turns, and the ITL/ITR stop lead."""

from copy import deepcopy
import time
import types
import unittest
from unittest.mock import patch

import app
from battery_support import BatteryFault, BatterySample
import imu_support as imu
import imu_tests
import motors_optimized as motor
import power_guard
import robot_settings as settings
import tof_support as tof
import wiring
from test_app import AppHarness
from test_imu import DynamicStream
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys


class ScriptedStream:
    """INA219 stand-in: returns whatever the test sets, or raises its error."""

    def __init__(self, clock):
        self.clock = clock
        self.current = 0.2
        self.voltage = 8.0
        self.error = None
        self.age = 0.0
        self.none = False
        self.closed = False

    def latest(self):
        if self.error is not None:
            raise BatteryFault(self.error)
        if self.none:
            return None
        shunt = self.current * 0.01
        return BatterySample(self.clock.monotonic() - self.age, self.voltage - shunt, shunt, self.current)

    def close(self):
        self.closed = True
        return True


class FakeMotors:
    def __init__(self):
        self.duties = [0.0] * 4
        self.stops = []

    def stop(self, brake=None):
        self.stops.append(brake)
        self.duties = [0.0] * 4


def make_guard(clock, stream, oc=4.5, oc_hold=0.2, lv=6.4, lv_hold=2.0):
    return power_guard.PowerGuard(oc, oc_hold, lv, lv_hold, stream_factory=lambda: stream, clock=clock)


class PowerGuardTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.stream = ScriptedStream(self.clock)
        self.guard = make_guard(self.clock, self.stream)
        self.motors = FakeMotors()
        self.motors.duties = [25.0] * 4

    def step(self, seconds, dt=0.02):
        end = self.clock.now + seconds
        while self.clock.now < end - 1e-9:
            self.clock.now = round(self.clock.now + dt, 6)
            self.guard.check(self.motors)

    def test_sustained_overcurrent_trips_and_drops_stby(self):
        self.stream.current = 5.0
        with self.assertRaisesRegex(power_guard.PowerFault, "Overcurrent: 5.00 A") as caught:
            self.step(1.0)
        self.assertIn("STBY dropped", str(caught.exception))
        self.assertEqual(self.motors.stops, [False])  # coast with STBY low, never a brake
        self.assertAlmostEqual(self.clock.now, 0.22, places=6)  # first high sample, then 0.2 s held

    def test_overcurrent_spike_shorter_than_hold_does_not_trip(self):
        self.stream.current = 6.0
        self.step(0.1)
        self.stream.current = 1.0
        self.step(1.0)
        self.assertEqual(self.motors.stops, [])

    def test_low_voltage_held_trips_but_short_sag_does_not(self):
        self.stream.voltage = 6.2
        self.step(1.5)
        self.stream.voltage = 7.5
        self.step(0.5)
        self.assertEqual(self.motors.stops, [])
        self.stream.voltage = 6.3
        with self.assertRaisesRegex(power_guard.PowerFault, "Low battery"):
            self.step(3.0)
        self.assertEqual(self.motors.stops, [False])

    def test_nothing_trips_while_every_wheel_is_off(self):
        self.motors.duties = [0.0] * 4
        self.stream.current = 7.0
        self.stream.voltage = 5.5
        self.stream.age = 0.0
        self.step(3.0)
        self.assertEqual(self.motors.stops, [])
        # Timers restart when driving resumes rather than carrying idle time.
        self.stream.age = 0.0
        self.motors.duties = [25.0] * 4
        self.step(0.1)
        self.assertEqual(self.motors.stops, [])

    def test_stale_or_missing_readings_trip_while_driving(self):
        self.stream.age = 0.6
        with self.assertRaisesRegex(power_guard.PowerFault, "old; the INA219 read stalled"):
            self.step(0.1)
        guard = make_guard(self.clock, ScriptedStream(self.clock))
        guard.stream.none = True
        motors = FakeMotors()
        motors.duties = [30.0, 0, 0, 0]
        with self.assertRaisesRegex(power_guard.PowerFault, "no current reading"):
            guard.check(motors)

    def test_monitor_failure_trips_and_shunt_saturation_reads_as_overcurrent(self):
        self.stream.error = "INA219 shunt reading is at its +/-80 mV limit; current is unreliable"
        with self.assertRaisesRegex(power_guard.PowerFault, "beyond the INA219's measurable range"):
            self.guard.check(self.motors)
        guard = make_guard(self.clock, ScriptedStream(self.clock))
        guard.stream.error = "OSError: [Errno 121] Remote I/O error"
        self.motors.duties = [25.0] * 4  # the first trip stopped them
        with self.assertRaisesRegex(power_guard.PowerFault, "current monitor failed"):
            guard.check(self.motors)

    def test_wait_ready_blocks_motion_without_a_reading(self):
        self.stream.none = True
        with self.assertRaisesRegex(power_guard.PowerFault, "no INA219 reading"):
            self.guard.wait_ready()
        self.stream.none = False
        self.stream.error = "No such device"
        with self.assertRaisesRegex(power_guard.PowerFault, "could not read the INA219"):
            self.guard.wait_ready()
        self.stream.error = None
        self.assertIsNotNone(self.guard.wait_ready())

    def test_summary_reports_peak_current_and_lowest_voltage(self):
        for current, voltage in ((1.0, 8.1), (2.4, 7.6), (1.5, 7.9)):
            self.stream.current, self.stream.voltage = current, voltage
            self.step(0.02)
        self.assertEqual(self.guard.summary(), "Power: peak 2.40 A, lowest 7.60 V")

    def test_limits_must_be_positive(self):
        with self.assertRaises(ValueError):
            make_guard(self.clock, self.stream, oc=0)
        with self.assertRaises(ValueError):
            make_guard(self.clock, self.stream, lv=float("nan"))


class PollIntegrationTests(unittest.TestCase):
    def test_every_control_poll_checks_the_guard_and_a_trip_leaves_stby_low(self):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        stream = ScriptedStream(clock)
        driver.power_guard = make_guard(clock, stream)
        controller = motor.Controller(driver, FakeKeys(clock), clock, lambda text: None)
        stream.current = 5.5
        with self.assertRaises(power_guard.PowerFault):
            controller.timed_move(motor.FORWARD, motor.straight_targets(), 5.0)
        self.assertLess(clock.now, 1.0)
        self.assertEqual(driver.duties, [0.0] * 4)
        self.assertTrue(all(pwm.duty == 0 for pwm in gpio.pwms))
        self.assertEqual(gpio.values[motor.STBY], 0)


class RecordingGuard:
    """Real guard logic over a scripted INA219, recorded for the app tests."""
    made = []
    current = 0.3

    def __init__(self, settings_):
        self.settings = settings_
        self.stream = None
        RecordingGuard.made.append(self)

    @classmethod
    def from_settings(cls, settings_, **kwargs):
        guard = cls(settings_)
        guard.inner = None
        return guard

    def bind(self, clock):
        self.stream = ScriptedStream(clock)
        self.stream.current = RecordingGuard.current
        self.inner = power_guard.PowerGuard.from_settings(self.settings, stream_factory=lambda: self.stream,
                                                           clock=clock)

    def wait_ready(self, poll=None):
        return self.inner.wait_ready(poll=poll)

    def check(self, motors):
        self.inner.check(motors)

    def summary(self):
        return self.inner.summary()

    @property
    def trip_reason(self):
        return self.inner.trip_reason

    def close(self):
        return self.inner.close()


class AppGuardTests(unittest.TestCase):
    def setUp(self):
        import tempfile
        from pathlib import Path
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.settings_path = Path(self.temp.name) / "robot_settings.json"
        self.addCleanup(settings.apply_settings, deepcopy(settings.DEFAULTS))
        RecordingGuard.made = []
        RecordingGuard.current = 0.3

    def run_commands(self, *choices):
        harness = AppHarness(self, choices=(*choices, "Q"))

        def factory(settings_, **kwargs):
            guard = RecordingGuard.from_settings(settings_)
            guard.bind(time)  # the app's control loops run on real time
            return guard

        with harness.environment(), patch.object(app, "PowerGuard", types.SimpleNamespace(from_settings=factory)):
            self.assertEqual(app.main(["--bot", "500", "--settings", str(self.settings_path)]), 0)
        return harness

    def test_powered_commands_run_under_a_guard_and_others_do_not(self):
        harness = self.run_commands("FL", "BAT", "IRC", "R")
        self.assertEqual(len(RecordingGuard.made), 2)  # FL and R only
        self.assertTrue(all(guard.stream.closed for guard in RecordingGuard.made))
        self.assertIn("Power guard ON: stops at 4.5 A for 0.2 s, or 6.4 V for 2 s", harness.stdout.getvalue())
        self.assertIsNone(harness.drivers[0].power_guard)

    def test_trip_ends_the_command_with_stby_low(self):
        RecordingGuard.current = 6.0
        harness = self.run_commands("FL")
        out = harness.stdout.getvalue()
        self.assertIn("Stopped: Overcurrent", out)
        self.assertEqual(harness.gpio.values[motor.STBY], 0)
        self.assertTrue(all(pwm.duty == 0 for pwm in harness.gpio.pwms))

    def test_guard_off_setting_runs_unguarded_and_says_so(self):
        settings.save_settings(self.settings_path, dict(deepcopy(settings.validate({}, "500rpm")),
                                                        power_guard=False), "500rpm")
        harness = self.run_commands("FL")
        self.assertEqual(RecordingGuard.made, [])
        self.assertIn("Power guard OFF", harness.stdout.getvalue())


class ToFClassificationTests(unittest.TestCase):
    def test_classify(self):
        cases = {(600, 11): "range", (2000, 11): "range", (2001, 11): "clear", (8190, 11): "clear",
                 (0, 11): "unknown", (8190, 4): "clear", (8191, 6): "clear", (8190, 9): "clear",
                 (8190, 14): "clear", (20, 8): "close", (0, 10): "close", (500, 1): "unknown",
                 (500, 0): "unknown", (500, 5): "unknown", (500, 7): "unknown", (500, 12): "unknown"}
        for (mm, status), kind in cases.items():
            with self.subTest(mm=mm, status=status):
                self.assertEqual(tof.classify(tof.ToFSample(1, 0.0, mm, status)), kind)


class DefaultMapTests(unittest.TestCase):
    def tearDown(self):
        settings.apply_settings(deepcopy(settings.DEFAULTS))

    def test_new_profiles_start_on_the_documented_rev_c_map(self):
        self.assertEqual(wiring.DEFAULT_CHANNELS, ("M1A", "M1B", "M2B", "M2A"))
        for bot in ("500rpm", "100rpm"):
            with self.subTest(bot=bot):
                data = settings.validate({}, bot)
                self.assertEqual(data["motor_channels"], wiring.DOCUMENTED_CHANNELS)
                self.assertEqual(data["ir_pins"], wiring.DOCUMENTED_IR_PINS)
                settings.apply_settings(data, bot)
                self.assertEqual(motor.MOTOR_PINS, ((17, 27, 18), (22, 23, 25), (16, 19, 13), (5, 6, 12)))
                self.assertEqual(motor.IR_PINS, (21, 7, 26))

    def test_saved_profiles_keep_their_own_map(self):
        data = settings.validate({"motor_channels": ["M1A", "M1B", "M2A", "M2B"], "ir_pins": [7, 26, 21]}, "500rpm")
        self.assertEqual(data["motor_channels"], ("M1A", "M1B", "M2A", "M2B"))
        self.assertEqual(data["ir_pins"], (7, 26, 21))


class NewSettingsTests(unittest.TestCase):
    def tearDown(self):
        settings.apply_settings(deepcopy(settings.DEFAULTS))

    def test_power_and_turn_settings_have_defaults_limits_and_a_set_group(self):
        data = settings.validate({}, "100rpm")
        self.assertEqual((data["power_guard"], data["power_overcurrent_a"], data["power_overcurrent_hold_s"],
                          data["power_low_voltage_v"], data["power_low_voltage_hold_s"]), (True, 4.5, 0.2, 6.4, 2.0))
        self.assertEqual((data["imu_turn_stop_lead_s"], data["imu_log_timed_turns"]), (0.0, True))
        for key, bad in (("power_overcurrent_a", 8.0), ("power_overcurrent_a", 0.4),
                         ("power_low_voltage_v", 4.9), ("power_overcurrent_hold_s", 0.0),
                         ("imu_turn_stop_lead_s", 0.6), ("power_guard", "yes")):
            with self.subTest(key=key, bad=bad):
                with self.assertRaises(ValueError):
                    settings.validate({key: bad}, "100rpm")
        self.assertEqual({settings.FIELDS[key].group for key in settings.FIELDS if key.startswith("power_")},
                         {"Power protection"})
        self.assertFalse(any(key.startswith("power_") for key in settings._MOTOR_KEYS.values()))

    def test_apply_sets_the_imu_turn_constants(self):
        settings.apply_settings(settings.validate({"imu_turn_stop_lead_s": 0.05, "imu_log_timed_turns": False},
                                                  "500rpm"), "500rpm")
        self.assertEqual((imu_tests.TURN_STOP_LEAD, imu_tests.LOG_TIMED_TURNS), (0.05, False))


def imu_rig(ready=True):
    clock = FakeClock()
    gpio = FakeGPIO(clock)
    driver = motor.MotorDriver(gpio)
    driver.initialize()
    stream = DynamicStream(clock)
    session = imu_tests.Session(stream_factory=lambda bus, address: stream)
    session.stream = stream
    if ready:
        session.calibration = imu.Calibration((0, 0, 0), (0, 0, 1), 0, axis=2)
    messages = []
    return clock, gpio, driver, stream, session, messages


def rotating_plant(clock, driver, stream, gain=3.0, coast_tau=0.0):
    """Yaw rate follows the wheel pattern; after power is cut it decays with coast_tau."""
    plant = {"rate": 0.0, "angle": 0.0}
    original_sleep = clock.sleep

    def sleep(duration):
        mean = sum(driver.duties) / 4
        if driver.pattern == motor.TURN_LEFT and mean > 0:
            plant["rate"] = mean * gain
        elif driver.pattern == motor.TURN_RIGHT and mean > 0:
            plant["rate"] = -mean * gain
        elif coast_tau > 0:
            plant["rate"] *= max(0.0, 1 - duration / coast_tau)
        else:
            plant["rate"] = 0.0
        plant["angle"] += plant["rate"] * duration
        original_sleep(duration)

    clock.sleep = sleep
    stream.values = lambda now: ((0, 0, plant["rate"]), (0, 0, 1))
    return plant


class TurnLoggingTests(unittest.TestCase):
    def tearDown(self):
        settings.apply_settings(deepcopy(settings.DEFAULTS))

    def controller(self, driver, clock, session, messages):
        control = motor.Controller(driver, FakeKeys(clock), clock, lambda text: messages.append(text))
        control.turn_observer = imu_tests.TurnYawLogger(session, lambda text: messages.append(text), clock)
        control.turn_observer.start()
        return control

    def test_timed_left_turn_is_measured_and_reported(self):
        clock, gpio, driver, stream, session, messages = imu_rig()
        plant = rotating_plant(clock, driver, stream, coast_tau=0.05)
        control = self.controller(driver, clock, session, messages)
        control.run_mode("L")
        report = [text for text in messages if text.startswith("Turn L:")]
        self.assertEqual(len(report), 1)
        measured = float(report[0].split("measured ")[1].split(" deg")[0])
        self.assertAlmostEqual(measured, plant["angle"], delta=1.5)
        self.assertEqual(driver.duties, [0.0] * 4)

    def test_course_reports_every_turn_and_heading_against_the_plan(self):
        clock, gpio, driver, stream, session, messages = imu_rig()
        rotating_plant(clock, driver, stream)
        lines = iter([(1, 1, 1)] * 10 + [(1, 1, 0)] * 6 + [(1, 1, 1)] * 400 + [(1, 1, 0)] * 6 + [(1, 1, 1)] * 10000)
        state = {"last": (1, 1, 1)}

        def readings(now):
            try:
                state["last"] = next(lines)
            except StopIteration:
                pass
            return state["last"]

        gpio.readings = readings
        control = self.controller(driver, clock, session, messages)
        control.run_mode("C", "LL")
        reports = [text for text in messages if text.startswith("Turn ")]
        self.assertTrue(reports[0].startswith("Turn 1 L:"))
        self.assertTrue(reports[1].startswith("Turn 2 L:"))
        self.assertTrue(any(text.startswith("Turn log: 2 turns") for text in messages))

    def test_logging_without_calibration_says_so_once_and_never_blocks_motion(self):
        clock, gpio, driver, stream, session, messages = imu_rig(ready=False)
        control = self.controller(driver, clock, session, messages)
        control.run_mode("R")
        self.assertEqual(sum("Turn angles not measured" in text for text in messages), 1)
        self.assertFalse(any(text.startswith("Turn R:") for text in messages))

    def test_imu_failure_mid_turn_stops_logging_but_not_the_turn(self):
        clock, gpio, driver, stream, session, messages = imu_rig()
        control = self.controller(driver, clock, session, messages)

        def broken():
            raise imu.IMUFault("IMU data is stale; movement stopped")

        stream.latest = broken
        control.run_mode("L")
        self.assertTrue(any("no longer measured" in text for text in messages))
        self.assertEqual(driver.duties, [0.0] * 4)

    def test_setting_off_disables_logging(self):
        clock, gpio, driver, stream, session, messages = imu_rig()
        settings.apply_settings(settings.validate({"imu_log_timed_turns": False}, "500rpm"), "500rpm")
        control = self.controller(driver, clock, session, messages)
        control.run_mode("L")
        self.assertFalse(any(text.startswith("Turn") for text in messages))


class TurnStopLeadTests(unittest.TestCase):
    def tearDown(self):
        settings.apply_settings(deepcopy(settings.DEFAULTS))

    def run_turn(self, lead):
        settings.apply_settings(settings.validate({"imu_turn_stop_lead_s": lead}, "100rpm"), "100rpm")
        clock, gpio, driver, stream, session, messages = imu_rig()
        plant = rotating_plant(clock, driver, stream, coast_tau=0.08)
        control = imu_tests.IMUTests(driver, FakeKeys(clock), session, clock, messages.append)
        control.run_test("ITL")
        return plant["angle"], messages

    def test_coast_is_reported_with_a_suggested_lead_that_lands_nearer_90(self):
        angle, messages = self.run_turn(0.0)
        hint = [text for text in messages if text.startswith("Coast after cutoff")]
        self.assertEqual(len(hint), 1)
        suggested = float(hint[0].split("about ")[1].split(" s")[0])
        self.assertGreater(suggested, 0)
        better, _ = self.run_turn(round(suggested, 3))
        self.assertLess(abs(better - 90), abs(angle - 90))


if __name__ == "__main__":
    unittest.main(verbosity=2)

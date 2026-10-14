"""Exercise calibration persistence, complete fake-plant runs and app protection."""

import contextlib
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import app
import calibration_data
import ir_tach
import motors_optimized as motor
import pwm_calibration as wizard
import robot_profiles
import robot_settings as settings
from fake_power import FakeGuard
from test_app import AppHarness
from test_ir_tach import PulseClock, PulseGPIO
from test_motors_optimized import FakeKeys


def record(config):
    return dict(version=1, robot_id=config["robot_id"], motor_channels=list(config["motor_channels"]),
        reversed=list(config["reversed"]), pwm_hz=1000, reference_duty=config["tach_reference_duty"],
        target_rpm=[180, 180], baseline_rpm=[[200, 220, 240, 260]] * 2,
        verified_rpm=[[180] * 4] * 2, factors=[[0.90, 0.82, 0.75, 0.69], [0.91, 0.83, 0.76, 0.70]],
        created_utc="2026-09-24T00:00:00+00:00", probe_connector="j8")


class ProfileTests(unittest.TestCase):
    def tearDown(self):
        settings.apply_settings(settings.DEFAULTS)

    def test_old_profiles_default_to_identity_and_j8(self):
        for bot in ("500", "100"):
            config = settings.apply_settings({}, bot)
            self.assertFalse(config["wheel_calibration_enabled"])
            self.assertEqual(config["tach_connector"], "j8")
            self.assertEqual(motor.WHEEL_FACTORS_FORWARD, (1,) * 4)
            self.assertEqual(motor.WHEEL_FACTORS_REVERSE, (1,) * 4)

    def test_wiring_reference_and_robot_changes_disable_corrections(self):
        config = settings.validate({})
        calibrated = dict(config, wheel_calibration=record(config), wheel_calibration_enabled=True)
        self.assertTrue(settings.apply_settings(calibrated)["wheel_calibration_enabled"])
        self.assertEqual(motor.WHEEL_FACTORS_FORWARD, (0.90, 0.82, 0.75, 0.69))
        for changes in ({"reversed": (True, False, False, False)},
                        {"motor_channels": ("M1A", "M1B", "M2A", "M2B")},
                        {"tach_reference_duty": 30},
                        {"robot_id": "100rpm", "motion_profile": settings.PROFILE_IDS["100rpm"]}):
            with self.subTest(changes=changes):
                result = settings.apply_settings(dict(calibrated, **changes))
                self.assertFalse(result["wheel_calibration_enabled"])
                self.assertEqual(motor.WHEEL_FACTORS_FORWARD, (1,) * 4)
                self.assertEqual(motor.WHEEL_FACTORS_REVERSE, (1,) * 4)
        # Turning off the unplugged calibration probe does not erase the measurement.
        self.assertTrue(settings.apply_settings(dict(calibrated, tach_connector="off"))["wheel_calibration_enabled"])

    def test_corrupt_or_unverified_records_rejected(self):
        base = record(settings.validate({}))
        changes = ({"factors": [[1.1] * 4] * 2}, {"factors": [[float("nan")] * 4] * 2},
                   {"factors": [[10 ** 1000] * 4] * 2}, {"target_rpm": [180, 181]},
                   {"verified_rpm": [[100] * 4] * 2}, {"pwm_hz": 2000},
                   {"reversed": [1] * 4}, {"factors": []}, {"surprise": True})
        for change in changes:
            with self.subTest(change=list(change)), self.assertRaises(ValueError):
                settings.validate({"wheel_calibration": dict(base, **change)})
        with self.assertRaises(ValueError):
            settings.validate({"wheel_calibration_enabled": True})

    def test_v93_profiles_migrate_separately_and_enable_only_matching_bot(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            old = root / "MPU6050_Motor_Test_v9_3" / "profiles"
            old.mkdir(parents=True)
            for bot in ("500", "100"):
                config = settings.validate({"reversed": (False, bot == "100", True, False)}, bot)
                settings.save_settings(old / f"{bot}rpm.json", config)
                before = (old / f"{bot}rpm.json").read_bytes()
                _, loaded, _ = robot_profiles.load_profile(root / "MPU6050_Motor_Test_v9_4", bot)
                self.assertEqual(config, loaded)
                self.assertEqual(before, (old / f"{bot}rpm.json").read_bytes())


class WizardTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "500rpm.json"
        self.config = settings.apply_settings({})
        self.addCleanup(settings.apply_settings, settings.DEFAULTS)
        settings.save_settings(self.path, self.config)
        self.original = self.path.read_bytes()
        self.clock = PulseClock()
        self.gpio = PulseGPIO(self.clock)
        self.motors = motor.MotorDriver(self.gpio)
        self.motors.initialize()
        self.clock.driver = self.motors
        self.addCleanup(self.motors.close)
        self.keys = FakeKeys(self.clock)
        self.answers = iter(("LIFTED", *wizard.WHEELS, *wizard.WHEELS, "SAVE"))
        self.checks = []
        self.measurements = []

    def read(self, prompt):
        self.assertFalse(any(self.motors.applied_duties), prompt)
        return next(self.answers)

    def write(self, message):
        self.assertFalse(any(self.motors.applied_duties), message)

    def powered(self, action):
        try:
            action(self.keys)
        finally:
            self.motors.stop(brake=False)

    def probe_check(self, *args, **kwargs):
        self.assertFalse(any(self.motors.applied_duties))
        self.checks.append(True)

    def fake_factory(self, *args, **kwargs):
        owner = self
        class Runner:
            def __enter__(self):
                return self
            def __exit__(self, *args):
                owner.motors.stop(brake=False)
            def measure(self, wheel, forward, duty):
                owner.measurements.append((wheel, forward, duty))
                return 200 + 10 * wheel + (5 if forward else 0)
            def calibrate(self, wheel, forward, target, reference):
                owner.measurements.append((wheel, forward, target))
                return target / (200 + 10 * wheel + (5 if forward else 0)), target
        return Runner()

    def run_wizard(self, **kwargs):
        return wizard.calibrate_profile(self.config, self.path, self.motors, self.powered,
            read=self.read, write=self.write, runner_factory=kwargs.get("runner_factory", self.fake_factory),
            probe_check=kwargs.get("probe_check", self.probe_check))

    def test_complete_wizard_saves_eight_factors_one_shared_target(self):
        candidate = self.run_wizard()
        self.assertEqual(len(self.checks), 8)
        self.assertEqual(len(self.measurements), 16)
        self.assertEqual(candidate, settings.load_settings(self.path))
        self.assertTrue(candidate["wheel_calibration_enabled"])
        self.assertEqual(candidate["wheel_calibration"]["target_rpm"], [190, 190])
        self.assertEqual(self.config, settings.validate({}))
        self.assertEqual(self.motors.duties, [0] * 4)

    def test_real_driver_engine_and_simulated_unequal_motors_converge(self):
        self.clock.rpm_for = lambda duty, wheel, forward: max(0, duty - 2) * (8 + wheel + (0.3 if forward else 0))
        def factory(motors, keys, pin, **kwargs):
            # Each stage is independent; limit the fake clock's local watchdog scope.
            self.clock.now = 0
            return ir_tach.CalibrationRunner(motors, keys, pin, self.clock, self.write)
        candidate = self.run_wizard(runner_factory=factory)
        data = candidate["wheel_calibration"]
        target = data["target_rpm"][0]
        self.assertAlmostEqual(target, 23 * 8 * .95, delta=.05)
        for row in data["verified_rpm"]:
            self.assertTrue(all(abs(rpm / target - 1) <= .05 for rpm in row))
        settings.apply_settings(candidate)
        self.motors.set_pattern(motor.FORWARD)
        self.motors.set_duties((25,) * 4)
        actual = [self.clock.rpm_for(duty, wheel, True) for wheel, duty in enumerate(self.motors.applied_duties)]
        self.assertTrue(all(abs(rpm / target - 1) <= .05 for rpm in actual))
        self.motors.stop(brake=False)

    def test_cancellation_at_each_prompt_retains_existing_bytes(self):
        complete = ["LIFTED", *wizard.WHEELS, *wizard.WHEELS, "SAVE"]
        for index in range(len(complete)):
            with self.subTest(index=index):
                self.answers = iter(complete[:index] + [""])
                self.assertIsNone(self.run_wizard())
                self.assertEqual(self.original, self.path.read_bytes())

    def test_probe_failure_and_stop_never_save(self):
        for error in (ir_tach.TachFault("missing probe"), motor.StopRequested()):
            self.answers = iter(("LIFTED", "FL"))
            with self.assertRaises(type(error)):
                self.run_wizard(probe_check=Mock(side_effect=error))
            self.assertEqual(self.path.read_bytes(), self.original)

    def test_late_verification_failure_never_saves_partial_results(self):
        def factory(*args, **kwargs):
            runner = self.fake_factory(*args, **kwargs)
            real = runner.calibrate
            def calibrate(wheel, forward, *args):
                if wheel == 3 and not forward:
                    raise ir_tach.TachFault("verification failed")
                return real(wheel, forward, *args)
            runner.calibrate = calibrate
            return runner
        with self.assertRaises(ir_tach.TachFault):
            self.run_wizard(runner_factory=factory)
        self.assertEqual(self.path.read_bytes(), self.original)

    def test_save_io_failure_preserves_prior_profile(self):
        with patch.object(settings.os, "replace", side_effect=OSError("disk full")), self.assertRaises(OSError):
            self.run_wizard()
        self.assertEqual(self.path.read_bytes(), self.original)

    def test_concurrent_settings_change_does_not_get_overwritten(self):
        original_read = self.read
        def read(prompt):
            if "SAVE" in prompt:
                settings.save_settings(self.path, dict(self.config, left_speed=24))
            return original_read(prompt)
        with patch.object(self, "read", side_effect=read), self.assertRaisesRegex(ValueError, "changed on disk"):
            self.run_wizard()
        self.assertEqual(settings.load_settings(self.path)["left_speed"], 24)

    def test_probe_connector_conflict_or_off_never_invokes_powered_stage(self):
        for update in ({"tach_connector": "j7"}, {"tach_connector": "off"}):
            with self.subTest(update=update), self.assertRaises(ir_tach.TachFault):
                wizard.calibrate_profile(dict(self.config, **update), self.path, self.motors,
                    powered=Mock(side_effect=AssertionError("must not power")))


class AppCalibrationTests(unittest.TestCase):
    def tearDown(self):
        settings.apply_settings(settings.DEFAULTS)

    def test_successful_pcal_reloads_saved_factors_before_next_command(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "robot.json"
            harness = AppHarness(self, ("PCAL", "FL", "Q"))
            observed = []
            def calibrate(config, path, motors, **kwargs):
                self.assertFalse(any(motors.duties))
                candidate = settings.validate(dict(config, wheel_calibration=record(config), wheel_calibration_enabled=True))
                settings.save_settings(path, candidate)
                return candidate
            def dispatch(command, sequence, motors, *args):
                motors.set_pattern(motor.FORWARD)
                motors.set_duties((25, 0, 0, 0))
                observed.append(motors.applied_duties[0])
            with harness.environment(), patch.object(app, "calibrate_profile", side_effect=calibrate), \
                    patch.object(app, "dispatch", side_effect=dispatch):
                self.assertEqual(app.main(["--bot", "500", "--settings", str(path)]), 0)
            self.assertEqual(len(harness.drivers), 2)
            self.assertEqual(observed, [22.5])

    def test_segment_guard_on_cancel_and_cleanup_failure(self):
        clock = PulseClock()
        gpio = PulseGPIO(clock)
        motors = motor.MotorDriver(gpio)
        motors.initialize()
        self.addCleanup(motors.close)
        clock.driver = motors
        args = app.build_parser().parse_args([])
        config = settings.validate({})
        guard = FakeGuard()
        def action(keys):
            self.assertIs(motors.power_guard, guard)
            motors.set_pattern(motor.FORWARD)
            motors.set_duties((25, 0, 0, 0))
            raise motor.StopRequested()
        with patch.object(motor, "Keyboard", return_value=contextlib.nullcontext(FakeKeys(clock))), \
                patch.object(app.PowerGuard, "from_settings", return_value=guard):
            with self.assertRaises(motor.StopRequested):
                app.calibration_segment(args, motors, Mock(), config, action)
        self.assertTrue(guard.closed)
        self.assertIsNone(motors.power_guard)
        self.assertEqual(motors.applied_duties, [0] * 4)
        self.assertEqual(gpio.values[motor.STBY], 0)
        guard.close = lambda: False
        with patch.object(motor, "Keyboard", return_value=contextlib.nullcontext(FakeKeys(clock))), \
                patch.object(app.PowerGuard, "from_settings", return_value=guard), self.assertRaisesRegex(RuntimeError, "did not close"):
            app.calibration_segment(args, motors, Mock(), config, lambda keys: None)

    def test_probe_command_never_creates_power_guard(self):
        with tempfile.TemporaryDirectory() as folder:
            harness = AppHarness(self, ("PPROBE", "Q"))
            with harness.environment(), patch.object(app, "check_probe") as check, \
                    patch.object(app, "PowerGuard") as guard:
                self.assertEqual(app.main(["--bot", "500", "--settings", str(Path(folder)/"bot.json")]), 0)
            check.assert_called_once()
            guard.from_settings.assert_not_called()
            self.assertTrue(all(duty == 0 for _, _, duty in harness.gpio.pwm_writes))


class ProbeCheckTests(unittest.TestCase):
    def make(self, edges=10, key="g"):
        clock = PulseClock()
        gpio = PulseGPIO(clock)
        motors = motor.MotorDriver(gpio)
        motors.initialize()
        self.addCleanup(motors.close)
        clock.driver = motors
        sent = 0
        original_sleep = clock.sleep
        def sleep(duration):
            nonlocal sent
            original_sleep(duration)
            if sent < edges and clock.now >= (sent + 1) * .1:
                gpio.emit(clock.now, 1 - gpio.probe_level)
                sent += 1
        clock.sleep = sleep
        keys = FakeKeys(clock, ((1.3, key),))
        return clock, gpio, motors, keys

    def test_five_full_turns_pass_and_cleanup_without_any_pwm(self):
        clock, gpio, motors, keys = self.make()
        wizard.check_probe(motors, keys, 20, clock=clock, write=lambda _: None)
        self.assertEqual(gpio.removed, [20])
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))

    def test_stationary_extra_and_incomplete_turns_are_rejected(self):
        for edges in (0, 9, 12):
            with self.subTest(edges=edges):
                clock, gpio, motors, keys = self.make(edges)
                with self.assertRaises(ir_tach.TachFault):
                    wizard.check_probe(motors, keys, 20, clock=clock, write=lambda _: None)
                self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))

    def test_probe_stop_and_timeout_leave_motors_off(self):
        for key, expected in (("s", motor.StopRequested), ("", ir_tach.TachFault)):
            clock, gpio, motors, keys = self.make(edges=0, key=key)
            with self.assertRaises(expected):
                wizard.check_probe(motors, keys, 20, clock=clock, write=lambda _: None)
            self.assertEqual(gpio.values[motor.STBY], 0 if key == "" else int(motor.BRAKE_ON_STOP))
            self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))


if __name__ == "__main__":
    unittest.main()

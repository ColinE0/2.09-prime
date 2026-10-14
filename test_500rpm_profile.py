"""500 RPM profile regressions at the settings and recorded GPIO boundaries."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import app
import imu_support
import imu_tests
import ir_confidence
import motors_optimized as motor
import robot_settings as settings
import single_motor
import test_imu as imu_fakes
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys


class ProfileOutputTests(unittest.TestCase):
    def setUp(self):
        settings.apply_settings(dict(settings.DEFAULTS))
        self.addCleanup(settings.apply_settings, dict(settings.DEFAULTS))

    def hardware(self, events=()):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        self.addCleanup(driver.close)
        return clock, gpio, driver, FakeKeys(clock, events)

    def assert_profile_output(self, gpio, driver, expected_peak=25):
        powered = [duty for _, _, duty in gpio.pwm_writes if duty > 0]
        self.assertTrue(powered, "Test must exercise powered GPIO writes")
        self.assertAlmostEqual(max(powered), expected_peak)
        self.assertTrue(all(0 <= duty <= 25 for _, _, duty in gpio.pwm_writes))
        self.assertEqual(driver.duties, [0] * 4)

    def test_every_manual_command_and_course_leg_uses_25_percent_profile(self):
        cases = [(mode, "") for mode in sorted(motor.MODES - {"IR", "C"})]
        cases += [("C", leg) for leg in "FRL"]
        for command, sequence in cases:
            with self.subTest(command=command, sequence=sequence):
                clock, gpio, driver, keys = self.hardware([(2.0, "s")] if command in ("F", "FA") else ())
                if command == "C":
                    gpio.readings = lambda now: (1, 1, 0 if now >= 0.9 else 1)
                controller = motor.Controller(driver, keys, clock, lambda message: None)
                if command in ("F", "FA"):
                    with self.assertRaises(motor.StopRequested):
                        controller.run_mode(command, sequence)
                else:
                    controller.run_mode(command, sequence)
                self.assert_profile_output(gpio, driver)

    def test_fa_correction_reaches_lower_secondary_duty_then_recovers(self):
        # LEFT sensor leans right (FR+BL slowed); RIGHT sensor leans left (FL+BR).
        for side, slower in ((0, (1, 2)), (1, (0, 3))):
            with self.subTest(side=side):
                clock, gpio, driver, keys = self.hardware([(2.0, "s")])
                observed = []

                def readings(now):
                    observed.append((now, tuple(driver.duties)))
                    result = [1, 1, 1]
                    if 1.0 <= now < 1.3:
                        result[side] = 0
                    return tuple(result)

                gpio.readings = readings
                controller = motor.Controller(driver, keys, clock, lambda message: None)
                with self.assertRaises(motor.StopRequested):
                    controller.run_mode("FA")
                during = next(duties for stamp, duties in observed if stamp == 1.28)
                after = next(duties for stamp, duties in observed if stamp == 1.98)
                for wheel in slower:
                    self.assertAlmostEqual(during[wheel], 14)
                self.assertEqual(after, (25,) * 4)
                self.assert_profile_output(gpio, driver)

    def test_fc_clear_corrections_and_uncertainty_stay_within_profile(self):
        for sides in ((1, 1), (0, 1), (1, 0), (0, 0)):
            with self.subTest(sides=sides):
                clock, gpio, driver, keys = self.hardware([(1.48, "s")])
                observed = []

                def readings(now):
                    observed.append((now, tuple(driver.duties)))
                    return (*(sides if now >= 1.0 else (1, 1)), 1)

                gpio.readings = readings
                controller = ir_confidence.ConfidenceTests(driver, keys, clock, lambda message: None)
                with self.assertRaises(motor.StopRequested):
                    controller.run_test("FC")
                during = next(duties for stamp, duties in observed if stamp == 1.4)
                if sides == (0, 0):
                    self.assertLessEqual(max(during), 20)
                    self.assertGreater(min(during), 0)
                elif sides != (1, 1):
                    self.assertLess(min(during), 20)
                    self.assertAlmostEqual(max(during), 25)
                self.assert_profile_output(gpio, driver)

    def test_single_wheel_commands_reach_25_percent_only_on_selected_channel(self):
        for command, wheel in (("FL", 0), ("FR", 1), ("BL", 2), ("BR", 3)):
            with self.subTest(command=command):
                clock, gpio, driver, keys = self.hardware()
                single_motor.SingleMotorTests(driver, keys, clock, lambda message: None).run_test(command)
                self.assertEqual({pin for _, pin, duty in gpio.pwm_writes if duty > 0},
                                 {motor.MOTOR_PINS[wheel][2]})
                self.assert_profile_output(gpio, driver)

    def test_imu_forward_and_turns_cannot_rescale_above_25_percent(self):
        for command in ("IF", "ITL", "ITR"):
            with self.subTest(command=command):
                clock, gpio, driver, keys = self.hardware()
                stream = imu_fakes.DynamicStream(clock)
                session = imu_tests.Session(stream_factory=lambda bus, address: stream)
                session.stream = stream
                session.calibration = imu_support.Calibration((0, 0, 0), (0, 0, 1), 0, axis=2)
                rate = {"IF": 2, "ITL": 30, "ITR": -30}[command]
                stream.values = lambda now: ((0, 0, 0 if now == 0 else rate), (0, 0, 1))
                controller = imu_tests.IMUTests(driver, keys, session, clock, lambda message: None)
                controller.run_test(command)
                self.assert_profile_output(gpio, driver)
                if command != "IF":
                    # PWM writes are skipped while the target stays unchanged;
                    # include the deceleration down to the final plateau.
                    late = [duty for stamp, _, duty in gpio.pwm_writes if stamp >= 1 and duty > 0]
                    self.assertTrue(late)
                    self.assertAlmostEqual(min(late), 16)

    def test_independent_fc_and_imu_caps_survive_reduced_manual_baselines(self):
        settings.apply_settings(dict(settings.DEFAULTS, left_speed=12, right_speed=10,
                                     auto_secondary_speed=7, diagonal_secondary_speed=5))
        state = ir_confidence.ConfidenceState((1, 1, 1), (0, 0), (False, False), "clear")
        self.assertEqual(max(ir_confidence.confidence_targets(state)), 25)
        self.assertEqual(max(imu_tests.forward_targets(0, 0)), 25)

    def test_project_hard_ceiling_remains_70_percent_for_later_tuning(self):
        _, gpio, driver, _ = self.hardware()
        driver.set_pattern(motor.FORWARD)
        driver.set_duties((70,) * 4)
        self.assertEqual(driver.duties, [70] * 4)
        with self.assertRaises(ValueError):
            driver.set_duties((70.01,) * 4)
        driver.stop()


class ProfileMigrationTests(unittest.TestCase):
    def legacy_config(self):
        return dict(left_speed=70, right_speed=69, auto_secondary_speed=40,
                    diagonal_secondary_speed=30, fc_max_duty=50, fc_uncertain_duty=30,
                    single_motor_duty=35, ramp_rate=100, auto_slew_rate=200,
                    reversed=[False, False, True, False], ir_pins=[21, 7, 26],
                    motor_channels=["M1A", "M1B", "M2B", "M2A"], camera_backend="usb",
                    camera_index=2, camera_rotate_180=False, camera_exposure_us=8000,
                    camera_analogue_gain=2)

    def test_legacy_saved_settings_get_profile_without_changing_wiring_or_camera(self):
        old = self.legacy_config()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "robot_settings.json"
            path.write_text(json.dumps(old), encoding="utf-8")
            original = path.read_bytes()
            result = settings.load_settings(path)
            self.assertEqual(path.read_bytes(), original, "Read-only previews must not migrate on disk")
        for key in ("left_speed", "right_speed", "fc_max_duty", "single_motor_duty"):
            self.assertEqual(result[key], 25)
        self.assertEqual(result["auto_secondary_speed"], 14)
        self.assertEqual(result["diagonal_secondary_speed"], 11)
        self.assertEqual(result["fc_uncertain_duty"], 20)
        self.assertEqual(result["motion_profile"], settings.DEFAULTS["motion_profile"])
        for key in ("reversed", "ir_pins", "motor_channels"):
            self.assertEqual(tuple(result[key]), tuple(old[key]))
        for key in ("camera_backend", "camera_index", "camera_rotate_180",
                    "camera_exposure_us", "camera_analogue_gain"):
            self.assertEqual(result[key], old[key])

    def test_legacy_partial_config_normalizes_before_new_secondary_bounds(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "robot_settings.json"
            path.write_text('{"auto_secondary_speed": 40, "diagonal_secondary_speed": 30}', encoding="utf-8")
            result = settings.load_settings(path)
        self.assertEqual((result["left_speed"], result["auto_secondary_speed"]), (25, 14))

    def test_current_profile_preserves_deliberate_subsequent_tuning(self):
        tuned = dict(settings.DEFAULTS, left_speed=30, right_speed=29,
                     fc_max_duty=28, fc_uncertain_duty=22, single_motor_duty=27)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "robot_settings.json"
            settings.save_settings(path, tuned)
            reloaded = settings.load_settings(path)
        self.assertEqual(reloaded, settings.validate(tuned))

    def test_literal_v7_import_applies_new_profile_and_retains_observed_wiring(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "motors_optimized.py").write_text(
                "LEFT_SPEED = 70\nRIGHT_SPEED = 69\nAUTO_SECONDARY_SPEED = 40\n"
                "DIAGONAL_SECONDARY_SPEED = 30\nREVERSED = (False, False, True, False)\n"
                "IR_PINS = (21, 7, 26)\n"
                "MOTOR_PINS = ((17, 27, 18), (22, 23, 25), (16, 19, 13), (5, 6, 12))\n",
                encoding="utf-8")
            (root / "ir_confidence.py").write_text("MAX_DUTY = 50\nUNCERTAIN_DUTY = 30\n", encoding="utf-8")
            (root / "single_motor.py").write_text("TEST_DUTY = 35\n", encoding="utf-8")
            result, report = settings.import_v7(root)
        self.assertEqual((result["left_speed"], result["right_speed"], result["fc_max_duty"],
                          result["single_motor_duty"]), (25, 25, 25, 25))
        self.assertEqual(tuple(result["motor_channels"]), ("M1A", "M1B", "M2B", "M2A"))
        self.assertEqual(tuple(result["ir_pins"]), (21, 7, 26))
        self.assertEqual(tuple(result["reversed"]), (False, False, True, False))
        self.assertTrue(report)

    def test_normal_launch_backs_up_legacy_bytes_once_and_retains_later_tuning(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "robot_settings.json"
            original = json.dumps(self.legacy_config()).encode("utf-8")
            path.write_bytes(original)
            args = app.build_parser().parse_args(["--bot", "500", "--settings", str(path)])
            with contextlib.redirect_stdout(io.StringIO()):
                _, result = app.load_configuration(args)
            backup = path.with_name("robot_settings.pre-profiles.json")
            self.assertEqual(backup.read_bytes(), original)
            self.assertEqual(json.loads(path.read_text())["left_speed"], 25)
            result["left_speed"] = 29
            settings.save_settings(path, result)
            tuned = path.read_bytes()
            with contextlib.redirect_stdout(io.StringIO()):
                _, reloaded = app.load_configuration(args)
            self.assertEqual(reloaded["left_speed"], 29)
            self.assertEqual(path.read_bytes(), tuned)
            self.assertEqual(backup.read_bytes(), original)
            self.assertEqual(len(list(Path(folder).glob("*.pre-profiles*.json"))), 1)

    def test_show_settings_previews_sibling_json_without_creating_or_changing_files(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            current = root / "MPU6050_Motor_Test_v9"
            previous = root / "MPU6050_Motor_Test_v8_500rpm"
            current.mkdir()
            previous.mkdir()
            source = previous / "robot_settings.json"
            original = json.dumps(dict(self.legacy_config(), left_speed=25, right_speed=25,
                                       auto_secondary_speed=14, diagonal_secondary_speed=11,
                                       motion_profile=settings.DEFAULTS["motion_profile"])).encode("utf-8")
            source.write_bytes(original)
            destination = current / "profiles" / "500rpm.json"
            with patch.object(app, "PACKAGE_DIR", current):
                _, result = app.load_configuration(app.build_parser().parse_args(["--show-settings"]))
            self.assertEqual(result["left_speed"], 25)
            self.assertEqual(tuple(result["motor_channels"]), ("M1A", "M1B", "M2B", "M2A"))
            self.assertFalse(destination.exists())
            self.assertEqual(source.read_bytes(), original)
            self.assertEqual(list(current.iterdir()), [])

    def test_imu_turn_minimum_cannot_override_a_lower_turn_maximum(self):
        with self.assertRaises(ValueError):
            settings.validate(dict(settings.DEFAULTS, imu_turn_max_duty=25, imu_turn_min_duty=32))


if __name__ == "__main__":
    unittest.main(verbosity=2)

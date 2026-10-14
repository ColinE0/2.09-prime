"""Application lifecycle, saved-settings boundary, and terminal dispatch checks."""

import contextlib
from copy import deepcopy
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch, Mock

import app
from fake_power import FakeGuard
import motors_optimized as motor
import robot_settings as settings
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys


class AppHarness:
    """Real MotorDriver with recording GPIO, fake sensor sessions, and menu input."""

    def __init__(self, case, choices=("Q",), initialize_error=None):
        self.case = case
        self.clock = FakeClock()
        self.gpio = FakeGPIO(self.clock)
        self.events, self.states, self.drivers = [], [], []
        self.choices = iter(choices)
        self.initialize_error = initialize_error
        self.before_choice = None
        self.stdout, self.stderr = io.StringIO(), io.StringIO()
        self.sessions, self.factories = {}, {}
        for name in ("imu", "battery", "vision", "tof"):
            self.sessions[name] = types.SimpleNamespace(
                enabled=False, calibration=None, stream=None,
                close=lambda name=name: self.close_sensor(name),
                readings=lambda control, name=name: self.events.append(name + " read"))
        self.sessions["vision"].forward = lambda control: self.events.append("camera FC")
        self.sessions["tof"].forward = self.tof_forward

    def close_sensor(self, name):
        self.case.assertTrue(all(driver.duties == [0] * 4 for driver in self.drivers))
        self.events.append(name + " close")
        return True

    def tof_forward(self, control, vision=None):
        self.case.assertIs(vision, self.sessions["vision"])
        self.events.append("ToF+camera FC" if vision.enabled else "ToF FC")

    def choose(self, status):
        self.states.append(deepcopy(status))
        if self.before_choice is not None:
            self.before_choice(len(self.states), status)
        return next(self.choices)

    @contextlib.contextmanager
    def environment(self):
        original_driver = motor.MotorDriver
        rpi = types.ModuleType("RPi")
        rpi.GPIO = self.gpio

        def make_driver(gpio):
            self.events.append(("driver construction", motor.LEFT_SPEED))
            driver = original_driver(gpio)
            self.drivers.append(driver)
            original_initialize = driver.initialize
            original_close = driver.close

            def initialize():
                self.events.append(("driver initialize", motor.LEFT_SPEED))
                original_initialize()
                if self.initialize_error is not None:
                    raise self.initialize_error

            def close():
                original_close()
                self.events.append("motors close")

            driver.initialize, driver.close = initialize, close
            return driver

        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules, {"RPi": rpi, "RPi.GPIO": self.gpio}))
            stack.enter_context(patch.object(app.sys.stdin, "isatty", return_value=True))
            stack.enter_context(patch.object(app, "PowerGuard", FakeGuard))
            stack.enter_context(patch.object(motor, "MotorDriver", side_effect=make_driver))
            stack.enter_context(patch.object(motor, "Keyboard", side_effect=lambda: contextlib.nullcontext(FakeKeys(self.clock))))
            stack.enter_context(patch.object(app, "TerminalMenu", return_value=types.SimpleNamespace(choose=self.choose)))
            for attribute, name in (("Session", "imu"), ("BatterySession", "battery"),
                                    ("VisionSession", "vision"), ("ToFSession", "tof")):
                self.factories[name] = stack.enter_context(patch.object(app, attribute, return_value=self.sessions[name]))
            stack.enter_context(patch.object(app, "ConfidenceTests", return_value=types.SimpleNamespace(
                run_test=lambda command: self.events.append(command))))
            stack.enter_context(contextlib.redirect_stdout(self.stdout))
            stack.enter_context(contextlib.redirect_stderr(self.stderr))
            yield self


class ApplicationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.settings_path = Path(self.temp.name) / "robot_settings.json"
        # App startup mutates runtime tuning modules; do not leak configuration
        # from these tests into the pre-existing motor/sensor regression suites.
        self.addCleanup(settings.apply_settings, deepcopy(settings.DEFAULTS))

    def args(self, *extra):
        return ["--bot", "500", "--settings", str(self.settings_path), *extra]

    def test_settings_are_applied_before_driver_construction_and_gpio_initialization(self):
        harness = AppHarness(self)
        config = dict(settings.DEFAULTS, left_speed=60.0)
        settings.save_settings(self.settings_path, config)
        applied = []
        original_apply = settings.apply_settings

        def apply(config):
            applied.append(deepcopy(config))
            harness.events.append("settings applied")
            return original_apply(config)

        with harness.environment(), patch.object(settings, "apply_settings", side_effect=apply):
            self.assertEqual(app.main(self.args()), 0)
        self.assertEqual(applied, [config])
        self.assertEqual(harness.events[:3], ["settings applied", ("driver construction", 60.0),
                                            ("driver initialize", 60.0)])
        self.assertEqual(settings.load_settings(self.settings_path), config)
        self.assertTrue(harness.gpio.cleaned)

    def test_invalid_saved_settings_fail_before_any_gpio_import(self):
        self.settings_path.write_text('{"not_a_robot_setting": 1}', encoding="utf-8")
        imports = []
        original_import = __import__

        def guarded_import(name, *args, **kwargs):
            if name == "RPi" or name.startswith("RPi."):
                imports.append(name)
            return original_import(name, *args, **kwargs)

        with patch("builtins.__import__", side_effect=guarded_import), \
             patch.object(motor, "MotorDriver") as driver, \
             contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(app.main(self.args()), 1)
        self.assertEqual(imports, [])
        driver.assert_not_called()

    def test_show_settings_prints_valid_json_without_gpio_or_settings_write(self):
        output = io.StringIO()
        with patch.object(motor, "MotorDriver") as driver, contextlib.redirect_stdout(output):
            self.assertEqual(app.main(self.args("--show-settings")), 0)
        self.assertEqual(settings.validate(json.loads(output.getvalue())), settings.DEFAULTS)
        driver.assert_not_called()
        self.assertFalse(self.settings_path.exists())

    def test_fc_dispatch_preserves_both_toggles_and_display_routes(self):
        harness = AppHarness(self, ("FC", "VT", "FC", "TT", "FC", "RV", "TOF",
                                    "IRC", "VT", "FC", "TT", "FC", "Q"))
        with harness.environment():
            self.assertEqual(app.main(self.args()), 0)
        actions = [event for event in harness.events if event in {
            "FC", "camera FC", "ToF FC", "ToF+camera FC", "vision read", "tof read", "IRC"}]
        self.assertEqual(actions, ["FC", "camera FC", "ToF+camera FC", "vision read", "tof read",
                                   "IRC", "ToF FC", "FC"])
        self.assertFalse(harness.states[0]["vision_enabled"])
        self.assertFalse(harness.states[0]["tof_enabled"])
        self.assertFalse(harness.sessions["vision"].enabled)
        self.assertFalse(harness.sessions["tof"].enabled)
        self.assertTrue(all(duty == 0 for _, _, duty in harness.gpio.pwm_writes))

    def test_settings_editor_stops_motors_and_saved_changes_apply_on_return(self):
        harness = AppHarness(self, ("SET", "Q"))
        changed = dict(settings.DEFAULTS, left_speed=60.0)

        def before_choice(number, status):
            if number == 1:
                driver = harness.drivers[0]
                driver.set_pattern(motor.FORWARD)
                driver.set_duties((15,) * 4)
            else:
                self.assertFalse(status["settings_pending"])
                self.assertEqual(motor.LEFT_SPEED, 60.0)
                self.assertTrue(harness.drivers[0].ready is False)

        def edit(config, path, read):
            self.assertEqual(harness.drivers[0].duties, [0] * 4)
            self.assertEqual(harness.gpio.values[motor.STBY], 0)
            self.assertEqual(config, settings.DEFAULTS)
            self.assertEqual(path, self.settings_path)
            settings.save_settings(path, changed)
            return changed

        harness.before_choice = before_choice
        with harness.environment(), patch.object(app, "edit_settings", side_effect=edit), \
             patch.object(settings, "apply_settings", wraps=settings.apply_settings) as apply:
            self.assertEqual(app.main(self.args()), 0)
            self.assertEqual(apply.call_count, 2)
        self.assertEqual(motor.LEFT_SPEED, 60.0)
        self.assertEqual(settings.load_settings(self.settings_path), changed)

        restarted = AppHarness(self)
        with restarted.environment():
            self.assertEqual(app.main(self.args()), 0)
        self.assertEqual(restarted.events[0], ("driver construction", 60.0))
        self.assertFalse(restarted.states[0]["settings_pending"])
        self.assertFalse(restarted.states[0]["vision_enabled"])
        self.assertFalse(restarted.states[0]["tof_enabled"])

    def test_editor_save_error_preserves_current_configuration_and_menu(self):
        harness = AppHarness(self, ("SET", "Q"))
        with harness.environment(), patch.object(app, "edit_settings", side_effect=OSError("Disk full")):
            self.assertEqual(app.main(self.args()), 0)
        self.assertFalse(harness.states[-1]["settings_pending"])
        self.assertIn("Disk full", harness.states[-1]["last_result"])
        self.assertEqual(motor.LEFT_SPEED, settings.DEFAULTS["left_speed"])
        self.assertEqual(settings.load_settings(self.settings_path), settings.validate({}))

    def test_saved_camera_options_and_cli_overrides_reach_session_for_worker_spawn(self):
        config = dict(settings.DEFAULTS, camera_index=2, camera_exposure_us=9000,
                      camera_analogue_gain=2.5)
        settings.save_settings(self.settings_path, config)
        harness = AppHarness(self)
        with harness.environment():
            self.assertEqual(app.main(self.args("--camera-backend", "usb", "--camera-index", "1",
                                               "--camera-upright", "--camera-auto-exposure")), 0)
        options = harness.factories["vision"].call_args.args[0]
        self.assertEqual((options.backend, options.index), ("usb", 1))
        self.assertFalse(options.rotate_180)
        self.assertTrue(options.auto_exposure)
        self.assertEqual((options.exposure_us, options.analogue_gain), (9000, 2.5))
        self.assertEqual(settings.load_settings(self.settings_path), config)

    def test_outputs_stop_before_finishing_live_terminal_status(self):
        harness = AppHarness(self, ("FC", "Q"))
        finished = []

        def running_dispatch(command, sequence, motors, *args):
            motors.set_pattern(motor.FORWARD)
            motors.set_duties((15,) * 4)

        def finish():
            self.assertEqual(harness.drivers[0].duties, [0] * 4)
            finished.append(True)

        with harness.environment(), patch.object(app, "dispatch", side_effect=running_dispatch), \
             patch.object(app, "LiveStatus", return_value=Mock(finish=finish)):
            self.assertEqual(app.main(self.args()), 0)
        self.assertEqual(finished, [True])

    def test_import_cli_reads_constants_without_executing_code_or_opening_hardware(self):
        old_folder = Path(self.temp.name) / "old_v7"
        old_folder.mkdir()
        source = old_folder / "motors_optimized.py"
        old_text = "LEFT_SPEED = 60.0\nraise RuntimeError('Old Python must never execute')\n"
        source.write_text(old_text, encoding="utf-8")
        with patch.object(motor, "MotorDriver") as driver, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(app.main(self.args("--import-v7", str(old_folder))), 0)
        driver.assert_not_called()
        self.assertEqual(settings.load_settings(self.settings_path)["left_speed"], 25.0)
        self.assertEqual(source.read_text(encoding="utf-8"), old_text)
        saved_bytes = self.settings_path.read_bytes()
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(app.main(self.args("--import-v7", str(old_folder))), 1)
        self.assertEqual(self.settings_path.read_bytes(), saved_bytes)

    def migration_folders(self):
        package = Path(self.temp.name) / "MPU6050_Motor_Test_v9"
        package.mkdir()
        (package / "profiles").mkdir()
        old = package.parent / "MPU6050_Motor_Test_v7"
        old.mkdir()
        (old / "motors_optimized.py").write_text("LEFT_SPEED = 60.0\n", encoding="utf-8")
        return package, package / "profiles" / "500rpm.json"

    def test_first_default_launch_does_not_guess_which_bot_owns_sibling_v7(self):
        package, default_path = self.migration_folders()
        harness = AppHarness(self)
        with harness.environment(), patch.object(app, "PACKAGE_DIR", package):
            self.assertEqual(app.main(["--bot", "500"]), 0)
        self.assertEqual(harness.events[0], ("driver construction", 25.0))
        self.assertEqual(settings.load_settings(default_path)["left_speed"], 25.0)
        self.assertEqual((package.parent / "MPU6050_Motor_Test_v7" / "motors_optimized.py").read_text(),
                         "LEFT_SPEED = 60.0\n")

    def test_custom_settings_path_disables_automatic_sibling_import(self):
        package, default_path = self.migration_folders()
        harness = AppHarness(self)
        with harness.environment(), patch.object(app, "PACKAGE_DIR", package):
            self.assertEqual(app.main(self.args()), 0)
        self.assertEqual(harness.events[0], ("driver construction", settings.DEFAULTS["left_speed"]))
        self.assertFalse(default_path.exists())
        self.assertEqual(settings.load_settings(self.settings_path), settings.validate({}))

    def test_existing_default_settings_take_precedence_over_sibling_v7(self):
        package, default_path = self.migration_folders()
        settings.save_settings(default_path, dict(settings.DEFAULTS, left_speed=65.0))
        original = default_path.read_bytes()
        harness = AppHarness(self)
        with harness.environment(), patch.object(app, "PACKAGE_DIR", package):
            self.assertEqual(app.main(["--bot", "500"]), 0)
        self.assertEqual(harness.events[0], ("driver construction", 65.0))
        self.assertEqual(default_path.read_bytes(), original)

    def test_imu_heading_targets_remain_capped_after_asymmetric_speed_configuration(self):
        settings.apply_settings(dict(settings.DEFAULTS, left_speed=40.0, right_speed=70.0))
        for angle, rate in ((0, 0), (5, 1), (-5, -1), (1000, 1000)):
            with self.subTest(angle=angle, rate=rate):
                targets = app.forward_targets(angle, rate)
                self.assertEqual(len(targets), 4)
                self.assertTrue(all(0 <= duty <= app.IMU_DRIVE_DUTY for duty in targets))

    def test_quit_closes_motors_before_all_sensor_sessions(self):
        harness = AppHarness(self)
        with harness.environment():
            self.assertEqual(app.main(self.args()), 0)
        close_events = [event for event in harness.events if isinstance(event, str) and event.endswith(" close")]
        self.assertEqual(close_events[0], "motors close")
        self.assertEqual(set(close_events[1:]), {"imu close", "battery close", "vision close", "tof close"})
        self.assertTrue(harness.gpio.cleaned)
        self.assertTrue(all(pwm.stopped for pwm in harness.gpio.pwms))

    def test_gpio_initialization_error_cleans_up_and_returns_failure(self):
        harness = AppHarness(self, initialize_error=RuntimeError("Injected GPIO initialization failure"))
        with harness.environment():
            self.assertEqual(app.main(self.args()), 1)
        self.assertEqual(harness.states, [])
        self.assertTrue(harness.gpio.cleaned)
        self.assertTrue(all(pwm.stopped for pwm in harness.gpio.pwms))
        self.assertIn("Injected GPIO initialization failure", harness.stdout.getvalue() + harness.stderr.getvalue())
        self.assertTrue(all(driver.duties == [0] * 4 for driver in harness.drivers))
        close_events = [event for event in harness.events if isinstance(event, str) and event.endswith(" close")]
        self.assertEqual(close_events[0], "motors close")
        self.assertEqual(set(close_events[1:]), {"imu close", "battery close", "vision close", "tof close"})


if __name__ == "__main__":
    unittest.main(verbosity=2)

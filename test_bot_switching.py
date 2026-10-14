"""Two-robot selection, profile isolation, and hardware handoff regressions."""

import contextlib
from copy import deepcopy
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import app
import motors_optimized as motor
import robot_settings as settings
import terminal_ui as ui
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys


class SwitchingHarness:
    """Use real motor ownership and profile files with recorded sensor lifetimes."""

    def __init__(self, case, package, robot_answers=("1",), commands=("Q",)):
        self.case, self.package = case, package
        self.clock = FakeClock()
        self.gpio = FakeGPIO(self.clock)
        self.robot_answers, self.commands = iter(robot_answers), iter(commands)
        self.events, self.drivers, self.sessions, self.states = [], [], [], []
        self.close_failures = {}
        self.gpio_cleanup_error = False
        self.selection_count = 0
        self.stdout, self.stderr = io.StringIO(), io.StringIO()
        self.edit = None

    def read_robot(self, prompt):
        answer = next(self.robot_answers)
        self.selection_count += 1
        self.events.append(("selection", answer))
        return answer

    def choose_command(self, status):
        generation = len(self.drivers) - 1
        self.states.append((generation, deepcopy(status)))
        command = next(self.commands)
        self.events.append(("command", generation, command))
        if command == "BOT":
            # Cleanup must also handle live output, not merely an already idle menu.
            driver = self.drivers[generation]
            driver.set_pattern(motor.FORWARD)
            driver.set_duties((7,) * 4)
        return command

    def make_sensor(self, name, *args, **kwargs):
        generation = len(self.drivers) - 1
        owner = types.SimpleNamespace(enabled=False, calibration=None, stream=None)

        def close():
            self.case.assertEqual(self.drivers[generation].duties, [0] * 4)
            self.events.append(("close", generation, name))
            outcome = self.close_failures.get(name, True)
            if isinstance(outcome, Exception):
                raise outcome
            return outcome

        owner.close = close
        self.sessions.append((generation, name, owner))
        self.events.append(("construct", generation, name))
        return owner

    def fake_dispatch(self, command, sequence, motors, keys, session, battery, vision, tof, log):
        self.case.assertEqual(command, "IC")
        session.calibration = types.SimpleNamespace(axis=2, require_ready=lambda *args, **kwargs: None)

    def fake_edit(self, config, path, read):
        self.case.assertTrue(all(driver.duties == [0] * 4 for driver in self.drivers))
        if self.edit is None:
            raise AssertionError("Unexpected settings editor")
        changed = self.edit(deepcopy(config), path)
        settings.save_settings(path, changed)
        return changed

    @contextlib.contextmanager
    def environment(self):
        original_driver = motor.MotorDriver
        original_apply = settings.apply_settings
        original_cleanup = self.gpio.cleanup
        rpi = types.ModuleType("RPi")
        rpi.GPIO = self.gpio

        def apply(config):
            self.events.append(("apply", config["left_speed"], tuple(config["motor_channels"])))
            return original_apply(config)

        def cleanup(pins=None):
            generation = len(self.drivers) - 1
            self.events.append(("gpio cleanup", generation, tuple(motor.MOTOR_PINS), tuple(motor.IR_PINS)))
            original_cleanup(pins)
            if self.gpio_cleanup_error:
                raise RuntimeError("Injected GPIO cleanup failure")

        def make_driver(gpio):
            generation = len(self.drivers)
            driver = original_driver(gpio)
            self.drivers.append(driver)
            expected_pins, expected_ir = tuple(motor.MOTOR_PINS), tuple(motor.IR_PINS)
            self.events.append(("construct", generation, "motors", motor.LEFT_SPEED, expected_pins, expected_ir))
            original_initialize, original_close = driver.initialize, driver.close

            def initialize():
                self.events.append(("initialize", generation, motor.LEFT_SPEED))
                return original_initialize()

            def close():
                self.case.assertEqual(tuple(motor.MOTOR_PINS), expected_pins,
                                      "Old GPIO must close before the next wheel mapping is applied")
                self.case.assertEqual(tuple(motor.IR_PINS), expected_ir)
                result = original_close()
                self.events.append(("close", generation, "motors"))
                outcome = self.close_failures.get("motors", result)
                if isinstance(outcome, Exception):
                    raise outcome
                return outcome

            driver.initialize, driver.close = initialize, close
            return driver

        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules, {"RPi": rpi, "RPi.GPIO": self.gpio}))
            stack.enter_context(patch.object(app, "PACKAGE_DIR", self.package))
            stack.enter_context(patch.object(app.sys.stdin, "isatty", return_value=True))
            stack.enter_context(patch.object(motor, "prompt_line", side_effect=self.read_robot))
            stack.enter_context(patch.object(motor, "MotorDriver", side_effect=make_driver))
            stack.enter_context(patch.object(settings, "apply_settings", side_effect=apply))
            stack.enter_context(patch.object(self.gpio, "cleanup", side_effect=cleanup))
            stack.enter_context(patch.object(motor, "Keyboard", side_effect=lambda: contextlib.nullcontext(FakeKeys(self.clock))))
            stack.enter_context(patch.object(app, "TerminalMenu", side_effect=lambda *args, **kwargs:
                                           types.SimpleNamespace(choose=self.choose_command)))
            for attribute, name in (("Session", "imu"), ("BatterySession", "battery"),
                                    ("VisionSession", "vision"), ("ToFSession", "tof")):
                stack.enter_context(patch.object(app, attribute,
                                                side_effect=lambda *args, name=name, **kwargs:
                                                self.make_sensor(name, *args, **kwargs)))
            stack.enter_context(patch.object(app, "dispatch", side_effect=self.fake_dispatch))
            stack.enter_context(patch.object(app, "edit_settings", side_effect=self.fake_edit))
            stack.enter_context(contextlib.redirect_stdout(self.stdout))
            stack.enter_context(contextlib.redirect_stderr(self.stderr))
            yield self


class RobotSelectionTests(unittest.TestCase):
    def choose(self, *answers):
        iterator = iter(answers)
        with contextlib.redirect_stdout(io.StringIO()):
            return ui.choose_robot(read=lambda prompt: next(iterator))

    def test_numbered_robot_choices(self):
        self.assertEqual(self.choose("1"), "500")
        self.assertEqual(self.choose("2"), "100")

    def test_cancel_does_not_select_a_default_robot(self):
        for answer in ("Q", "q", "0"):
            with self.subTest(answer=answer):
                self.assertIsNone(self.choose(answer))

    def test_invalid_robot_choice_reprompts(self):
        self.assertEqual(self.choose("nonsense", "3", "2"), "100")

    def test_bot_shortcut_is_available_inside_a_menu_category(self):
        answers = iter(("1", "BOT"))
        output = []
        menu = ui.TerminalMenu(read=lambda prompt: next(answers), write=output.append)
        self.assertEqual(menu.choose({}), "BOT")
        self.assertIn("BOT", ui.COMMANDS)
        self.assertIn("BOT", "\n".join(output))


class RobotSwitchingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.package = Path(self.temp.name)
        (self.package / "profiles").mkdir()
        self.addCleanup(settings.apply_settings, deepcopy(settings.DEFAULTS))
        self.config500 = settings.validate(dict(left_speed=27, right_speed=26,
            motor_channels=("M1A", "M1B", "M2A", "M2B"), ir_pins=(7, 26, 21)), bot_id="500rpm")
        self.config100 = settings.validate(dict(left_speed=55, right_speed=54,
            motor_channels=("M1A", "M1B", "M2B", "M2A"), ir_pins=(21, 7, 26),
            reversed=(False, False, True, False)), bot_id="100rpm")
        settings.save_settings(self.profile_path("500"), self.config500)
        settings.save_settings(self.profile_path("100"), self.config100)

    def profile_path(self, bot):
        return self.package / "profiles" / (bot + "rpm.json")

    def assert_closed(self, harness, generation):
        closed = {event[2] for event in harness.events if event[:2] == ("close", generation)}
        self.assertEqual(closed, {"motors", "imu", "battery", "vision", "tof"})
        self.assertEqual(harness.drivers[generation].duties, [0] * 4)

    def test_select_100_at_startup_applies_its_mapping_and_tuning(self):
        harness = SwitchingHarness(self, self.package, robot_answers=("2",))
        with harness.environment():
            self.assertEqual(app.main(["--battery-profile", "none"]), 0)
        self.assertEqual(harness.selection_count, 1)
        self.assertEqual([event[2] for event in harness.events if event[0] == "initialize"], [55])
        self.assertEqual(tuple(motor.IR_PINS), tuple(self.config100["ir_pins"]))
        self.assertEqual(tuple(motor.REVERSED), tuple(self.config100["reversed"]))
        self.assert_closed(harness, 0)

    def test_switch_500_to_100_to_500_preserves_separate_tuning_and_resets_sessions(self):
        harness = SwitchingHarness(self, self.package, robot_answers=("1", "2", "1"),
            commands=("VT", "TT", "IC", "SET", "BOT", "VT", "TT", "IC", "SET", "BOT", "Q"))
        edits = []

        def edit(config, path):
            edits.append(path)
            config["left_speed"] += 2
            return config

        harness.edit = edit
        with harness.environment():
            self.assertEqual(app.main(["--battery-profile", "none"]), 0)
        self.assertEqual([event[2] for event in harness.events if event[0] == "initialize"], [27, 29, 55, 57, 29])
        self.assertEqual(edits, [self.profile_path("500"), self.profile_path("100")])
        self.assertEqual(json.loads(self.profile_path("500").read_text())["left_speed"], 29)
        self.assertEqual(json.loads(self.profile_path("100").read_text())["left_speed"], 57)
        self.assertEqual(harness.selection_count, 3)
        self.assertEqual(len(harness.sessions), 20)
        self.assertEqual(len({id(owner) for _, _, owner in harness.sessions}), 20)
        for generation in range(5):
            first_status = next(status for number, status in harness.states if number == generation)
            self.assertFalse(first_status["vision_enabled"])
            self.assertFalse(first_status["tof_enabled"])
            self.assertEqual(first_status["imu_status"], "Needs IC")
            self.assert_closed(harness, generation)
        for generation in range(4):
            next_apply = [i for i, event in enumerate(harness.events) if event[0] == "apply"][generation + 1]
            closed = [i for i, event in enumerate(harness.events) if event[:2] == ("close", generation)]
            cleanup = next(i for i, event in enumerate(harness.events) if event[:2] == ("gpio cleanup", generation))
            self.assertTrue(all(i < next_apply for i in closed))
            self.assertLess(cleanup, next_apply)
        selections = [i for i, event in enumerate(harness.events) if event[0] == "selection"]
        for selection, old_generation in zip(selections[1:], (1, 3)):
            closed = [i for i, event in enumerate(harness.events) if event[:2] == ("close", old_generation)]
            self.assertTrue(all(i < selection for i in closed))

    def test_startup_cancel_quits_without_applying_or_initializing_hardware(self):
        for answer in ("Q", "0"):
            with self.subTest(answer=answer):
                harness = SwitchingHarness(self, self.package, robot_answers=(answer,), commands=())
                with harness.environment():
                    self.assertEqual(app.main(["--battery-profile", "none"]), 0)
                self.assertEqual(harness.drivers, [])
                self.assertEqual(harness.sessions, [])
                self.assertFalse(any(event[0] == "apply" for event in harness.events))

    def test_any_failed_close_blocks_switch_and_still_attempts_all_cleanup(self):
        for owner in ("motors", "imu", "battery", "vision", "tof"):
            for outcome in (False, RuntimeError("Injected cleanup failure")):
                with self.subTest(owner=owner, outcome=outcome):
                    harness = SwitchingHarness(self, self.package, robot_answers=("1", "2"), commands=("BOT", "Q"))
                    harness.close_failures[owner] = outcome
                    with harness.environment():
                        self.assertEqual(app.main(["--battery-profile", "none"]), 1)
                    self.assertEqual(harness.selection_count, 1)
                    self.assertEqual(len(harness.drivers), 1)
                    self.assertEqual(len(harness.sessions), 4)
                    self.assert_closed(harness, 0)

    def test_real_gpio_cleanup_failure_blocks_next_robot(self):
        harness = SwitchingHarness(self, self.package, robot_answers=("1", "2"), commands=("BOT", "Q"))
        harness.gpio_cleanup_error = True
        with harness.environment():
            self.assertEqual(app.main(["--battery-profile", "none"]), 1)
        self.assertEqual(harness.selection_count, 1)
        self.assertEqual(len(harness.drivers), 1)
        self.assert_closed(harness, 0)

    def test_next_profile_load_error_never_constructs_new_hardware(self):
        self.profile_path("100").write_text('{"broken":', encoding="utf-8")
        harness = SwitchingHarness(self, self.package, robot_answers=("1", "2"), commands=("BOT", "Q"))
        with harness.environment():
            self.assertEqual(app.main(["--battery-profile", "none"]), 1)
        self.assertEqual(harness.selection_count, 2)
        self.assertEqual(len(harness.drivers), 1)
        self.assertEqual(len(harness.sessions), 4)
        self.assertEqual(sum(event[0] == "apply" for event in harness.events), 1)
        self.assert_closed(harness, 0)

    def test_custom_settings_file_stays_bound_to_initial_robot_across_switches(self):
        custom = self.package / "custom500.json"
        settings.save_settings(custom, dict(self.config500, left_speed=31))
        harness = SwitchingHarness(self, self.package, robot_answers=("2", "1"), commands=("BOT", "BOT", "Q"))
        with harness.environment():
            self.assertEqual(app.main(["--bot", "500", "--settings", str(custom), "--battery-profile", "none"]), 0)
        self.assertEqual(harness.selection_count, 2)
        self.assertEqual([event[2] for event in harness.events if event[0] == "initialize"], [31, 55, 31])
        self.assertEqual(json.loads(self.profile_path("500").read_text())["left_speed"], 27)
        for generation in range(3):
            self.assert_closed(harness, generation)


if __name__ == "__main__":
    unittest.main(verbosity=2)

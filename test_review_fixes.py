"""Regression tests for findings from the external v8 review."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

import app
import robot_settings as settings
import motors_optimized as motor
from single_motor import SingleMotorTests
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys
import test_vision
import vision_support as vision


class WiringAndDutyTests(unittest.TestCase):
    def tearDown(self):
        settings.apply_settings({})

    def test_rear_swap_and_inversion_reach_actual_gpio_for_bl(self):
        settings.apply_settings({"motor_channels": ["M1A", "M1B", "M2B", "M2A"],
                                 "reversed": [False, False, True, False]})
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        try:
            SingleMotorTests(driver, FakeKeys(clock), clock=clock, log=lambda _: None).run_test("BL")
            self.assertEqual({pin for _, pin, duty in gpio.pwm_writes if duty > 0}, {13})
            enabled = next(i for i, (_, pin, value, _, _) in enumerate(gpio.pin_writes)
                           if pin == motor.STBY and value == 1)
            directions = {pin: value for _, pin, value, _, _ in gpio.pin_writes[:enabled]}
            self.assertEqual((directions[16], directions[19]), (0, 1))
            self.assertEqual(driver.duties, [0] * 4)
        finally:
            driver.close()

    def test_channel_editor_and_validation(self):
        self.assertEqual(settings.parse_value("motor_channels", "m1a, m1b, m2b, m2a"),
                         ("M1A", "M1B", "M2B", "M2A"))
        for channels in (["M1A"] * 4, ["M1A", "M1B", "M2A", "GPIO24"], ["M1A"], [[], 1, 2, 3]):
            with self.subTest(channels=channels), self.assertRaises(ValueError):
                settings.validate({"motor_channels": channels})

    def test_overspeed_rejected_on_editor_load_import_and_driver(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for key in ("left_speed", "right_speed"):
                with self.assertRaises(ValueError):
                    settings.parse_value(key, "70.01")
                path = root / "robot_settings.json"
                path.write_text(json.dumps({key: 100}), encoding="utf-8")
                with self.assertRaises(ValueError):
                    settings.load_settings(path)
            (root / "motors_optimized.py").write_text("LEFT_SPEED = 100\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                settings.import_v7(root)
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        try:
            driver.set_pattern(motor.FORWARD)
            driver.set_duties((70,) * 4)
            before = list(gpio.pwm_writes)
            for duty in (70.01, 100, float("nan"), float("inf")):
                with self.assertRaises(ValueError):
                    driver.set_duties((0, 0, 0, duty))
            self.assertEqual(gpio.pwm_writes, before)
        finally:
            driver.close()

    def test_literal_motor_pins_import_preserves_swap_without_execution(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "motors_optimized.py").write_text(
                "MOTOR_PINS = ((17,27,18),(22,23,25),(16,19,13),(5,6,12))\n"
                "raise RuntimeError('must not run')\n", encoding="utf-8")
            data, _ = settings.import_v7(root)
            self.assertEqual(data["motor_channels"], ("M1A", "M1B", "M2B", "M2A"))

    def test_invalid_or_computed_channel_map_stops_import(self):
        for source in (
                "MOTOR_PINS = ((17,27,18),) * 4\n",
                "MOTOR_PINS = ((17,27,18),(22,23,25),(16,19,13),(16,19,13))\n",
                "MOTOR_PINS = ((17,27,18),(22,23,25),(16,19,13),(5,6,24))\n",
                "MOTOR_PINS = get_pins()\n"):
            with self.subTest(source=source), tempfile.TemporaryDirectory() as folder:
                (Path(folder) / "motors_optimized.py").write_text(source, encoding="utf-8")
                with self.assertRaises(ValueError):
                    settings.import_v7(folder)

    def test_explicit_v8_import_preserves_json_and_python_wiring(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            previous = root / "MPU6050_Motor_Test_v8"
            current = root / "MPU6050_Motor_Test_v9"
            previous.mkdir()
            current.mkdir()
            old = previous / "robot_settings.json"
            old.write_text(json.dumps({"left_speed": 62, "ir_pins": [21, 7, 26],
                                       "reversed": [False, False, True, False]}), encoding="utf-8")
            original = old.read_bytes()
            (previous / "motors_optimized.py").write_text(
                "MOTOR_PINS = ((17,27,18),(22,23,25),(16,19,13),(5,6,12))\n", encoding="utf-8")
            destination = current / "profiles" / "500rpm.json"
            with patch.object(app, "PACKAGE_DIR", current), \
                    contextlib.redirect_stdout(io.StringIO()):
                # Inspection is read-only and must not pretend it imported anything.
                options = ["--bot", "500", "--import-settings", str(old)]
                app.load_configuration(app.build_parser().parse_args(options + ["--show-settings"]))
                self.assertFalse(destination.exists())
                _, config = app.load_configuration(app.build_parser().parse_args(options))
                self.assertEqual(config["left_speed"], 25)
                self.assertEqual(config["ir_pins"], (21, 7, 26))
                self.assertEqual(config["motor_channels"], ("M1A", "M1B", "M2B", "M2A"))
                self.assertEqual(config["reversed"], (False, False, True, False))
                self.assertEqual(settings.load_settings(destination), config)
                self.assertEqual(old.read_bytes(), original)


class LightConfirmationTests(unittest.TestCase):
    """v9.9: the TrafficLight keeps v9.5's FC gate rules for one-frame and mixed reads."""

    def make(self):
        stream = types.SimpleNamespace(latest=lambda: test_vision.sample())
        return vision.TrafficLight(stream, log=lambda text: None), stream

    def feed(self, light, stream, index, labels=()):
        stream.latest = lambda: test_vision.sample(index + 1, index / 10, labels)
        light.update(index / 10)

    def test_a_single_red_or_yellow_frame_does_not_confirm_but_blocks_a_crossing(self):
        for label in ("red_light", "yellow_light"):
            light, stream = self.make()
            for i in (0, 1):
                self.feed(light, stream, i, ("green_light",))
            self.feed(light, stream, 2, (label,))
            self.assertEqual(light.state, "green")  # green is still 2 of 3
            self.assertFalse(light.crossing(0.2))   # but the newest frame shows a stop light
            self.feed(light, stream, 3, ("green_light",))
            self.assertTrue(light.crossing(0.3))

    def test_red_then_yellow_on_one_lamp_confirms_as_red(self):
        light, stream = self.make()
        self.feed(light, stream, 0, ("red_light",))
        self.feed(light, stream, 1)
        self.feed(light, stream, 2, ("yellow_light",))
        self.assertEqual(light.light(), "red")
        self.assertEqual(light.preview(), "wait")

    def test_duplicate_red_frame_cannot_confirm(self):
        light, stream = self.make()
        self.feed(light, stream, 0, ("red_light",))
        for now in (.01, .02, .03):
            light.update(now)
        self.assertIsNone(light.state)

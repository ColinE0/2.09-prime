"""Single-wheel isolation, bounded motion, stop paths and terminal dispatch."""

import contextlib
import io
import sys
import types
import unittest
from pathlib import Path
import tempfile
from unittest.mock import patch

import app as menu
from fake_power import FakeGuard
import motors_optimized as motor
import single_motor as single
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys


class SingleMotorTests(unittest.TestCase):
    def make(self, events=()):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        control = single.SingleMotorTests(driver, FakeKeys(clock, events), clock, lambda text: None)
        return control, driver, gpio, clock

    def assert_stopped(self, driver, gpio):
        self.assertEqual(driver.duties, [0.0] * 4)
        self.assertTrue(all(pwm.duty == 0 for pwm in gpio.pwms))
        if gpio.values[motor.STBY]:
            for ain1, ain2, _ in motor.MOTOR_PINS:
                self.assertEqual((gpio.values[ain1], gpio.values[ain2]), (1, 1))

    def test_each_command_powers_only_its_named_channel_and_stops_at_one_second(self):
        for command, wheel in (("FL", 0), ("FR", 1), ("BL", 2), ("BR", 3)):
            with self.subTest(command=command):
                control, driver, gpio, clock = self.make()
                control.run_test(command)
                powered = [(stamp, pin, duty) for stamp, pin, duty in gpio.pwm_writes if duty > 0]
                self.assertTrue(powered)
                self.assertEqual({pin for _, pin, _ in powered}, {motor.MOTOR_PINS[wheel][2]})
                self.assertEqual(max(duty for _, _, duty in powered), 25.0)
                self.assertTrue(all(stamp < 1.0 for stamp, _, _ in powered))
                self.assertEqual(clock.now, 1.0)
                self.assert_stopped(driver, gpio)

    def test_selected_motor_ramps_without_initial_full_duty_jump(self):
        control, driver, gpio, _ = self.make()
        control.run_test("FL")
        values = [duty for _, pin, duty in gpio.pwm_writes if pin == motor.MOTOR_PINS[0][2] and duty > 0]
        self.assertLessEqual(values[0], motor.RAMP_RATE * motor.LOOP_DT)
        self.assertTrue(all(0 < later - earlier <= 2.0 + 1e-9 for earlier, later in zip(values, values[1:])))

    def test_completion_message_is_printed_only_after_motors_stop(self):
        control, driver, gpio, _ = self.make()
        messages = []
        def log(text):
            if "test complete" in text:
                self.assert_stopped(driver, gpio)
                messages.append(text)
        control.log = log
        control.run_test("FR")
        self.assertEqual(messages, ["FR test complete"])

    def test_queued_stop_never_powers_any_wheel(self):
        control, driver, gpio, clock = self.make([(0, "s")])
        with self.assertRaises(motor.StopRequested):
            control.run_test("BR")
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assertEqual(clock.now, 0)
        self.assert_stopped(driver, gpio)

    def test_stop_during_ramp_or_full_duty_stops_promptly(self):
        for stop_time in (0.10, 0.70):
            with self.subTest(stop_time=stop_time):
                control, driver, gpio, clock = self.make([(stop_time, "s")])
                with self.assertRaises(motor.StopRequested):
                    control.run_test("FR")
                self.assertEqual(clock.now, stop_time)
                self.assertTrue(any(duty > 0 for _, _, duty in gpio.pwm_writes))
                self.assert_stopped(driver, gpio)

    def test_switching_wheels_clears_prior_output_before_direction_writes(self):
        control, driver, gpio, _ = self.make()
        driver.set_pattern(motor.TURN_LEFT)
        driver.set_duties((70, 70, 70, 70))
        gpio.pwm_writes.clear()
        gpio.pin_writes.clear()
        control.run_test("BL")
        self.assertEqual({pin for _, pin, duty in gpio.pwm_writes if duty > 0}, {motor.MOTOR_PINS[2][2]})
        direction_pins = {pin for pins in motor.MOTOR_PINS for pin in pins[:2]}
        for _, pin, _, duties, standby in gpio.pin_writes:
            if pin in direction_pins:
                self.assertEqual(standby, 0)
                self.assertTrue(all(duty == 0 for duty in duties))
        self.assert_stopped(driver, gpio)

    def test_invalid_command_keeps_all_wheels_off(self):
        control, driver, gpio, _ = self.make()
        with self.assertRaises(ValueError):
            control.run_test("ALL")
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assert_stopped(driver, gpio)

    def test_keyboard_interrupt_and_pwm_failure_stop_all_channels(self):
        for error in (KeyboardInterrupt(), RuntimeError("PWM failed")):
            with self.subTest(error=type(error).__name__):
                control, driver, gpio, _ = self.make()
                original = driver.slew
                def fail_after_write(*args):
                    original(*args)
                    if any(driver.duties):
                        raise error
                with patch.object(driver, "slew", side_effect=fail_after_write), self.assertRaises(type(error)):
                    control.run_test("BR")
                self.assert_stopped(driver, gpio)

    def test_loop_delay_ends_test_instead_of_resuming_motion(self):
        control, driver, gpio, clock = self.make()
        original = clock.sleep
        clock.sleep = lambda duration: original(0.2 if clock.now >= 0.04 else duration)
        with self.assertRaisesRegex(motor.LaneFault, "interrupted"):
            control.run_test("FL")
        self.assertTrue(all(duty == 0 for stamp, _, duty in gpio.pwm_writes if stamp > 0.04))
        self.assert_stopped(driver, gpio)

    def test_wheel_tests_do_not_read_ir_or_change_based_on_front_line(self):
        control, driver, gpio, _ = self.make()
        gpio.readings = lambda now: (0, 0, 0)
        control.run_test("FL")
        self.assertEqual(gpio.read_times, [])
        self.assertTrue(any(duty > 0 for _, _, duty in gpio.pwm_writes))
        self.assert_stopped(driver, gpio)

    def test_menu_dispatches_each_wheel_with_sensor_toggles_on_without_starting_sensors(self):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        rpi = types.ModuleType("RPi")
        rpi.GPIO = gpio
        calls = []
        def no_sensor(*args, **kwargs):
            self.fail("Single-wheel command started a sensor or FC")
        sensor = lambda: types.SimpleNamespace(enabled=False, close=lambda: None,
                                               forward=no_sensor, readings=no_sensor)
        choices = iter(["VT", "TT", "fl", "FR", "BL", "br", "Q"])
        with patch.dict(sys.modules, {"RPi": rpi, "RPi.GPIO": gpio}), \
             patch.object(menu.sys.stdin, "isatty", return_value=True), \
             patch.object(menu, "PowerGuard", FakeGuard), \
             patch.object(menu, "VisionSession", return_value=sensor()), \
             patch.object(menu, "ToFSession", return_value=sensor()), \
             patch.object(menu, "SingleMotorTests", return_value=types.SimpleNamespace(run_test=calls.append)), \
             patch.object(motor, "prompt_line", side_effect=lambda text: next(choices)), \
             patch.object(motor, "Keyboard", side_effect=lambda: contextlib.nullcontext(FakeKeys(clock))), \
             contextlib.redirect_stdout(io.StringIO()) as output:
            with tempfile.TemporaryDirectory() as folder:
                self.assertEqual(menu.main(["--bot", "500", "--settings", str(Path(folder) / "settings.json")]), 0)
        self.assertEqual(calls, ["FL", "FR", "BL", "BR"])
        self.assertIn("MENU v9", output.getvalue())
        self.assertTrue(gpio.cleaned)


if __name__ == "__main__":
    unittest.main(verbosity=2)

"""BTN: the board's START/RESET buttons and status LED, run against recording GPIO fakes.

Run: python -m unittest -v test_board_ui
"""

from copy import deepcopy
import unittest
from unittest.mock import patch

import app
import motors_optimized as motor
import robot_settings as settings
import terminal_ui
from board_ui import ButtonTests
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys


class ButtonGPIO(FakeGPIO):
    PUD_UP = "UP"

    def __init__(self, clock, presses=()):
        super().__init__(clock)
        self.presses = presses  # (pin, start, end): the switch holds the pin LOW in [start, end)
        self.pulls, self.setups = {}, []

    def setup(self, pin, mode, initial=None, pull_up_down=None):
        self.setups.append(pin)
        self.pulls[pin] = pull_up_down
        super().setup(pin, mode, initial)

    def input(self, pin):
        if pin in (motor.START_BTN, motor.RESET_BTN):
            assert self.modes[pin] == self.IN
            now = self.clock.now
            held = any(p == pin and start <= now < end for p, start, end in self.presses)
            return self.LOW if held else self.HIGH
        return super().input(pin)

    def led_writes(self):
        return [(now, value) for now, pin, value, _, _ in self.pin_writes if pin == motor.STATUS_LED]


class ButtonTestsCase(unittest.TestCase):
    def setUp(self):
        # Both switches wired, as on bot 1; bot 2's START-only wiring is in test_v9_9.
        settings.apply_settings(dict(deepcopy(settings.DEFAULTS), button_start="gpio10", button_reset="gpio9"))
        self.addCleanup(settings.apply_settings, deepcopy(settings.DEFAULTS))

    def run_btn(self, presses=(), stop_at=1.5):
        clock = FakeClock()
        gpio = ButtonGPIO(clock, presses)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        lines = []
        with self.assertRaises(motor.StopRequested):
            ButtonTests(driver, FakeKeys(clock, [(stop_at, "s")]), clock, lines.append).run_test("BTN")
        return driver, gpio, lines

    def test_claims_pulled_up_buttons_and_a_low_led_once(self):
        driver, gpio, _ = self.run_btn()
        driver.claim_board_ui()  # a second BTN in the same session claims nothing new
        for pin in (motor.START_BTN, motor.RESET_BTN):
            self.assertEqual(gpio.setups.count(pin), 1)
            self.assertEqual(gpio.pulls[pin], gpio.PUD_UP)
        self.assertEqual(gpio.setups.count(motor.STATUS_LED), 1)
        self.assertTrue(driver.close())

    def test_presses_are_debounced_counted_and_released(self):
        start, reset = motor.START_BTN, motor.RESET_BTN
        _, _, lines = self.run_btn([(start, 0.10, 0.30), (start, 0.50, 0.52), (reset, 0.70, 0.90)])
        self.assertIn("At start: START up, RESET up", lines)
        self.assertEqual(lines.count("START pressed (1)"), 1)
        self.assertIn("START released", lines)
        self.assertIn("RESET pressed (1)", lines)
        self.assertNotIn("START pressed (2)", lines, "A one-tick blip counted as a press")
        self.assertEqual(lines[-1], "Buttons: START pressed 1 time, RESET pressed 1 time.")

    def test_led_blinks_stays_on_while_held_and_ends_off(self):
        _, gpio, _ = self.run_btn([(motor.START_BTN, 0.60, 1.20)], stop_at=1.8)
        writes = gpio.led_writes()
        self.assertEqual(writes[:2], [(0.0, 1), (0.5, 0)])  # blinking before any press
        lit = next(now for now, value in writes if value == 1 and now > 0.5)
        self.assertAlmostEqual(lit, 0.62)  # the debounced press lights it
        self.assertFalse([now for now, value in writes if value == 0 and lit < now < 1.22],
                         "The LED went off while START was held")
        self.assertEqual(writes[-1][1], 0)

    def test_a_held_button_at_start_is_flagged(self):
        _, _, lines = self.run_btn([(motor.RESET_BTN, 0.0, 9.0)], stop_at=0.2)
        self.assertTrue(any(line.startswith("At start: START up, RESET DOWN") and "short" in line
                            for line in lines))

    def test_motors_are_never_driven(self):
        _, gpio, _ = self.run_btn([(motor.START_BTN, 0.1, 0.3), (motor.RESET_BTN, 0.4, 0.6)])
        self.assertFalse([write for write in gpio.pwm_writes if write[2] > 0])

    def test_a_busy_pin_is_reported_not_fatal(self):
        clock = FakeClock()
        gpio = ButtonGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        gpio.fail_setup_pin = motor.START_BTN
        with self.assertRaisesRegex(ValueError, "SPI must stay off"):
            ButtonTests(driver, FakeKeys(clock), clock, print).run_test("BTN")

    def test_btn_is_listed_and_runs_without_the_power_guard(self):
        self.assertIn("BTN", terminal_ui.COMMANDS)
        self.assertNotIn("BTN", app.POWERED_COMMANDS)
        with patch.object(app, "ButtonTests") as tests:
            app.dispatch("BTN", "", "motors", "keys", None, None, None, None, print)
        tests.return_value.run_test.assert_called_once_with("BTN")


if __name__ == "__main__":
    unittest.main()

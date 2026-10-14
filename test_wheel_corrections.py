"""Per-wheel PWM mapping and temporary calibration bypass, without hardware."""

import types
import unittest
from unittest.mock import patch

import motors_optimized as motor
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys


class WheelCorrectionTests(unittest.TestCase):
    def setUp(self):
        self.factors = patch.multiple(motor,
                                      WHEEL_FACTORS_FORWARD=(0.5, 0.6, 0.7, 0.8),
                                      WHEEL_FACTORS_REVERSE=(0.9, 0.8, 0.7, 0.6))
        self.factors.start()
        self.addCleanup(self.factors.stop)

    def make(self):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        self.addCleanup(driver.close)
        return driver, gpio, clock

    def assert_duties(self, driver, gpio, logical, applied):
        self.assertEqual(driver.duties, list(logical))
        for actual, expected in zip(driver.applied_duties, applied):
            self.assertAlmostEqual(actual, expected)
        for pwm, expected in zip(gpio.pwms, applied):
            self.assertAlmostEqual(pwm.duty, expected)

    def test_default_unit_factors_preserve_requests(self):
        with patch.multiple(motor, WHEEL_FACTORS_FORWARD=(1.0,) * 4,
                            WHEEL_FACTORS_REVERSE=(1.0,) * 4):
            driver, gpio, _ = self.make()
            driver.set_pattern(motor.TURN_LEFT)
            driver.set_duties((10, 20, 30, 40))
            self.assert_duties(driver, gpio, (10, 20, 30, 40), (10, 20, 30, 40))

    def test_mapping_uses_logical_wheels_and_directions_with_remapped_reversed_wiring(self):
        pins = (motor.MOTOR_PINS[3], motor.MOTOR_PINS[2],
                motor.MOTOR_PINS[1], motor.MOTOR_PINS[0])
        with patch.multiple(motor, MOTOR_PINS=pins, REVERSED=(True, False, True, False)):
            driver, gpio, _ = self.make()
            pattern = (True, False, False, True)
            driver.set_pattern(pattern)
            driver.set_duties((40, 40, 40, 40))
            self.assert_duties(driver, gpio, (40,) * 4, (20, 32, 28, 32))
            self.assertEqual(tuple(pwm.pin for pwm in gpio.pwms), tuple(p[2] for p in pins))
            for channel, forward, reverse in zip(pins, pattern, motor.REVERSED):
                self.assertEqual(gpio.values[channel[0]], int(forward != reverse))
                self.assertEqual(gpio.values[channel[1]], int(forward == reverse))
            # Cleanup must occur while the remapped pin configuration is in force.
            driver.close()

    def test_forward_reverse_arrays_are_selected_independently(self):
        driver, gpio, _ = self.make()
        driver.set_pattern(motor.FORWARD)
        driver.set_duties((20,) * 4)
        self.assert_duties(driver, gpio, (20,) * 4, (10, 12, 14, 16))
        driver.set_pattern((False,) * 4)
        self.assert_duties(driver, gpio, (0,) * 4, (0,) * 4)
        driver.set_duties((20,) * 4)
        self.assert_duties(driver, gpio, (20,) * 4, (18, 16, 14, 12))

    def test_slew_preserves_logical_trajectory_and_never_compounds_mapping(self):
        driver, gpio, _ = self.make()
        driver.set_pattern(motor.FORWARD)
        for value in range(2, 22, 2):
            reached = driver.slew((20,) * 4, rate=100, dt=0.02)
            self.assert_duties(driver, gpio, (value,) * 4,
                               tuple(value * factor for factor in motor.WHEEL_FACTORS_FORWARD))
            self.assertEqual(reached, value == 20)
        writes = len(gpio.pwm_writes)
        for _ in range(10):
            self.assertTrue(driver.slew((20,) * 4, rate=100, dt=0.02))
        self.assertEqual(len(gpio.pwm_writes), writes)
        self.assert_duties(driver, gpio, (20,) * 4, (10, 12, 14, 16))
        driver.slew((0,) * 4, rate=100, dt=0.02)
        self.assert_duties(driver, gpio, (18,) * 4, (9, 10.8, 12.6, 14.4))

    def test_timed_controller_mode_uses_mapping_without_changing_logical_targets(self):
        driver, gpio, clock = self.make()
        controller = motor.Controller(driver, FakeKeys(clock), clock, lambda text: None)
        controller.timed_move(motor.FORWARD, (20, 30, 40, 50), 0.1)
        self.assert_duties(driver, gpio, (20, 30, 40, 50), (10, 18, 28, 40))

    def test_zero_remains_zero_and_70_ceiling_applies_before_attenuation(self):
        driver, gpio, _ = self.make()
        driver.set_duties((0,) * 4)  # No direction pattern is required for zero.
        driver.set_pattern(motor.FORWARD)
        driver.set_duties((70,) * 4)
        self.assert_duties(driver, gpio, (70,) * 4, (35, 42, 49, 56))
        before = list(gpio.pwm_writes)
        for bad in (70.01, 100, -1, float("nan"), float("inf")):
            with self.subTest(value=bad), self.assertRaises(ValueError):
                driver.set_duties((20, 20, 20, bad))
        self.assertEqual(gpio.pwm_writes, before)
        driver.set_duties((0,) * 4)
        self.assert_duties(driver, gpio, (0,) * 4, (0,) * 4)

    def test_factor_update_is_applied_even_if_logical_request_is_unchanged(self):
        driver, gpio, _ = self.make()
        driver.set_pattern(motor.FORWARD)
        driver.set_duties((20,) * 4)
        with patch.object(motor, "WHEEL_FACTORS_FORWARD", (1.0,) * 4):
            driver.set_duties((20,) * 4)
            self.assert_duties(driver, gpio, (20,) * 4, (20,) * 4)

    def test_invalid_factors_rejected_before_initializing_any_gpio(self):
        for attribute in ("WHEEL_FACTORS_FORWARD", "WHEEL_FACTORS_REVERSE"):
            for bad in ((1, 1, 1), (1, 1, 1, 1, 1), (1, 1, 1, 0.49),
                        (1, 1, 1, 1.01), (1, 1, 1, float("nan")),
                        (1, 1, 1, float("inf")), (1, 1, 1, None), None):
                with self.subTest(attribute=attribute, factors=bad), \
                     patch.object(motor, attribute, bad):
                    gpio = FakeGPIO(FakeClock())
                    driver = motor.MotorDriver(gpio)
                    with self.assertRaises(ValueError):
                        driver.initialize()
                    self.assertEqual(gpio.modes, {})
                    self.assertEqual(gpio.pwm_writes, [])

    def test_invalid_unused_direction_factors_rejected_before_partial_output(self):
        driver, gpio, _ = self.make()
        driver.set_pattern(motor.FORWARD)
        before = list(gpio.pwm_writes)
        with patch.object(motor, "WHEEL_FACTORS_REVERSE", (1, 1, 1, 1.01)):
            with self.assertRaises(ValueError):
                driver.set_duties((20,) * 4)
        self.assertEqual(gpio.pwm_writes, before)
        self.assert_duties(driver, gpio, (0,) * 4, (0,) * 4)

    def test_stop_clears_logical_and_applied_and_works_with_invalid_profile(self):
        driver, gpio, _ = self.make()
        driver.set_pattern(motor.FORWARD)
        driver.set_duties((20,) * 4)
        with patch.object(motor, "WHEEL_FACTORS_FORWARD", None):
            driver.stop(brake=False)
        self.assert_duties(driver, gpio, (0,) * 4, (0,) * 4)
        self.assertEqual(gpio.values[motor.STBY], 0)

    def test_raw_context_bypasses_once_then_stops_and_restores_on_success(self):
        driver, gpio, _ = self.make()
        writes = (len(gpio.pwm_writes), len(gpio.pin_writes))
        with driver.raw_calibration() as raw:
            self.assertIs(raw, driver)
            self.assertEqual(writes, (len(gpio.pwm_writes), len(gpio.pin_writes)))
            driver.set_pattern(motor.FORWARD)
            driver.set_duties((20, 30, 40, 50))
            self.assert_duties(driver, gpio, (20, 30, 40, 50), (20, 30, 40, 50))
            exit_index = len(gpio.pwm_writes)
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes[exit_index:]))
        self.assert_duties(driver, gpio, (0,) * 4, (0,) * 4)
        self.assertEqual(gpio.values[motor.STBY], 0)
        self.assertIsNone(driver.pattern)
        driver.set_pattern(motor.FORWARD)
        driver.set_duties((20,) * 4)
        self.assert_duties(driver, gpio, (20,) * 4, (10, 12, 14, 16))

    def test_raw_context_stops_and_restores_even_on_keyboard_interrupt(self):
        driver, gpio, _ = self.make()
        for error in (RuntimeError("sensor failed"), KeyboardInterrupt()):
            with self.subTest(error=type(error).__name__), self.assertRaises(type(error)):
                with driver.raw_calibration():
                    driver.set_pattern((False,) * 4)
                    driver.set_duties((30,) * 4)
                    raise error
            self.assert_duties(driver, gpio, (0,) * 4, (0,) * 4)
            self.assertFalse(driver._raw_calibration)
            self.assertEqual(gpio.values[motor.STBY], 0)
        driver.set_pattern((False,) * 4)
        driver.set_duties((20,) * 4)
        self.assert_duties(driver, gpio, (20,) * 4, (18, 16, 14, 12))

    def test_raw_context_restores_mapping_even_if_cleanup_raises(self):
        driver, _, _ = self.make()
        with patch.object(driver, "stop", side_effect=RuntimeError("PWM cleanup failed")):
            with self.assertRaisesRegex(RuntimeError, "cleanup failed"):
                with driver.raw_calibration():
                    pass
        self.assertFalse(driver._raw_calibration)

    def test_raw_context_rejects_uninitialized_moving_or_nested_entry(self):
        gpio = FakeGPIO(FakeClock())
        uninitialized = motor.MotorDriver(gpio)
        with self.assertRaisesRegex(RuntimeError, "initialized"):
            with uninitialized.raw_calibration():
                self.fail("entered uninitialized calibration")
        driver, gpio, _ = self.make()
        driver.set_pattern(motor.FORWARD)
        driver.set_duties((20, 0, 0, 0))
        before = list(gpio.pwm_writes)
        with self.assertRaisesRegex(RuntimeError, "Stop all"):
            with driver.raw_calibration():
                self.fail("entered moving calibration")
        self.assertEqual(gpio.pwm_writes, before)
        driver.stop(brake=False)
        with driver.raw_calibration():
            with self.assertRaisesRegex(RuntimeError, "nested"):
                with driver.raw_calibration():
                    self.fail("entered nested calibration")
            self.assertTrue(driver._raw_calibration)

    def test_raw_context_still_validates_maximum_and_factors(self):
        driver, gpio, _ = self.make()
        with driver.raw_calibration():
            driver.set_pattern(motor.FORWARD)
            before = list(gpio.pwm_writes)
            with self.assertRaises(ValueError):
                driver.set_duties((71, 0, 0, 0))
            with patch.object(motor, "WHEEL_FACTORS_REVERSE", (1, 1, 1, -1)):
                with self.assertRaises(ValueError):
                    driver.set_duties((20, 0, 0, 0))
            self.assertEqual(gpio.pwm_writes, before)

    def test_power_guard_checks_before_positive_output_even_in_raw_mode(self):
        driver, gpio, _ = self.make()
        observed = []
        def check(motors):
            observed.append(tuple(pwm.duty for pwm in gpio.pwms))
            raise RuntimeError("guard prevented output")
        guard = types.SimpleNamespace(trip_reason=None, check=check)
        driver.power_guard = guard
        with self.assertRaisesRegex(RuntimeError, "guard prevented"):
            with driver.raw_calibration():
                self.assertIs(driver.power_guard, guard)
                driver.set_pattern(motor.FORWARD)
                driver.set_duties((20,) * 4)
        self.assertEqual(observed, [(0,) * 4])
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assert_duties(driver, gpio, (0,) * 4, (0,) * 4)
        self.assertIs(driver.power_guard, guard)

    def test_latched_power_fault_never_repowers_in_raw_mode(self):
        driver, gpio, _ = self.make()
        driver.power_guard = types.SimpleNamespace(trip_reason="overcurrent",
                                                   check=lambda motors: self.fail("guard latch lost"))
        with self.assertRaisesRegex(RuntimeError, "already tripped"):
            with driver.raw_calibration():
                driver.set_pattern(motor.FORWARD)
                driver.set_duties((20,) * 4)
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assertEqual(gpio.values[motor.STBY], 0)
        self.assertFalse(driver._raw_calibration)


if __name__ == "__main__":
    unittest.main(verbosity=2)

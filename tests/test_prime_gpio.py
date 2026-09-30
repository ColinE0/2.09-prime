"""Fault-injection checks for 2.09-prime GPIO setup and shutdown."""
import contextlib
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import warnings

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import motors_optimized as motor


class FakePWM:
    def __init__(self, gpio, pin):
        self.gpio = gpio
        self.pin = pin
        self.duty = 0
        self.stopped = False

    def start(self, value):
        self.gpio.record('start', self.pin, value)
        self.duty = value

    def ChangeDutyCycle(self, value):
        self.gpio.record('duty', self.pin, value)
        self.duty = value

    def stop(self):
        self.gpio.record('pwm_stop', self.pin)
        self.duty = 0
        self.stopped = True


class FakeGPIO:
    BCM = 11
    BOARD = 10
    IN = 1
    OUT = 0
    LOW = 0
    HIGH = 1
    SPI = 41

    def __init__(self):
        self.mode = None
        self.functions = {}
        self.levels = {}
        self.pwms = []
        self.events = []
        self.failures = {}
        self.setup_warning_pin = None

    def record(self, action, *args):
        event = (action, *args)
        self.events.append(event)
        failures = self.failures.get(event, 0)
        if failures:
            self.failures[event] -= 1
            raise OSError('injected ' + repr(event))

    def getmode(self):
        return self.mode

    def setmode(self, value):
        self.mode = value

    def setwarnings(self, value):
        self.warnings = value

    def gpio_function(self, pin):
        return self.functions.get(pin, self.IN)

    def setup(self, pin, mode, initial):
        # Emulate a call which changes hardware before it raises an error.
        self.functions[pin] = mode
        self.levels[pin] = initial
        self.record('setup', pin, mode, initial)
        if pin == self.setup_warning_pin:
            warnings.warn('channel already in use', RuntimeWarning)

    def output(self, pin, value):
        self.record('output', pin, value)
        self.levels[pin] = value

    def PWM(self, pin, frequency):
        self.record('pwm_create', pin, frequency)
        pwm = FakePWM(self, pin)
        self.pwms.append(pwm)
        return pwm

    def cleanup(self, pin):
        self.record('cleanup', pin)
        self.functions[pin] = self.IN


class GPIOTests(unittest.TestCase):
    def setUp(self):
        self.gpio = FakeGPIO()
        self.driver = motor.MotorDriver(self.gpio)
        self.output = io.StringIO()
        redirect = contextlib.redirect_stderr(self.output)
        redirect.__enter__()
        self.addCleanup(redirect.__exit__, None, None, None)
        self.pins = [motor.STBY]
        for group in motor.MOTOR_PINS:
            self.pins.extend(group)

    def initialize(self):
        self.driver.initialize()
        return self.driver

    def test_invalid_pin_maps_rejected_before_any_gpio_write(self):
        original = motor.MOTOR_PINS
        invalid = [
            ((17, 17, 18),) + original[1:],
            ((motor.STBY, 27, 18),) + original[1:],
            ((2, 27, 18),) + original[1:],
            ((3, 27, 18),) + original[1:],
            ((0, 27, 18),) + original[1:],
            ((True, 27, 18),) + original[1:],
            ((40, 27, 18),) + original[1:],
            ((17, 27),) + original[1:],
            original[:3],
        ]
        for pins in invalid:
            with self.subTest(pins=pins), patch.object(motor, 'MOTOR_PINS', pins):
                with self.assertRaises(ValueError):
                    self.driver.initialize()
                self.assertEqual(self.gpio.events, [])

    def test_bad_polarity_frequency_or_factors_rejected_before_writes(self):
        for name, value in [('REVERSED', (0, 0, 0, 0)), ('REVERSED', (False,)),
                            ('PWM_FREQ', 0), ('PWM_FREQ', float('nan')),
                            ('WHEEL_FACTORS_FORWARD', (1, 1, 2, 1))]:
            with self.subTest(name=name, value=value), patch.object(motor, name, value):
                with self.assertRaises(ValueError):
                    self.driver.initialize()
                self.assertEqual(self.gpio.events, [])

    def test_board_numbering_rejected_before_any_output(self):
        self.gpio.mode = self.gpio.BOARD
        with self.assertRaisesRegex(motor.GPIOFault, 'BCM'):
            self.driver.initialize()
        self.assertEqual(self.gpio.events, [])

    def test_any_busy_pin_aborts_entire_setup_without_resetting_it(self):
        for state in [self.gpio.OUT, self.gpio.SPI, 999]:
            self.gpio.functions[self.pins[-1]] = state
            with self.assertRaisesRegex(motor.GPIOFault, 'already'):
                self.driver.initialize()
            self.assertTrue(self.driver.close())
            self.assertEqual(self.gpio.events, [])
            self.assertEqual(self.gpio.functions[self.pins[-1]], state)

    def test_pin_function_rechecked_immediately_before_setup(self):
        read_function = self.gpio.gpio_function
        calls = {}
        busy_pin = self.pins[-1]
        def changing_function(pin):
            calls[pin] = calls.get(pin, 0) + 1
            if pin == busy_pin and calls[pin] == 2:
                self.gpio.functions[pin] = self.gpio.SPI
            return read_function(pin)
        with patch.object(self.gpio, 'gpio_function', side_effect=changing_function):
            with self.assertRaisesRegex(motor.GPIOFault, 'changed function'):
                self.driver.initialize()
        self.assertNotIn(('cleanup', busy_pin), self.gpio.events)
        self.assertNotIn(('setup', busy_pin, self.gpio.OUT, 0), self.gpio.events)

    def test_startup_standby_first_all_outputs_low_and_pwm_zero(self):
        self.initialize()
        self.assertEqual(self.gpio.events[0], ('setup', motor.STBY, self.gpio.OUT, 0))
        setups = [e for e in self.gpio.events if e[0] == 'setup']
        self.assertEqual([e[1] for e in setups], self.pins)
        self.assertTrue(all(e[-1] == 0 for e in setups))
        self.assertEqual([pwm.duty for pwm in self.gpio.pwms], [0, 0, 0, 0])
        self.assertEqual(self.gpio.levels[motor.STBY], 0)
        self.assertTrue(self.gpio.warnings)

    def test_repeated_initialize_does_not_reassign_live_pins(self):
        self.initialize()
        before = list(self.gpio.events)
        with self.assertRaises(motor.GPIOFault):
            self.driver.initialize()
        self.assertEqual(self.gpio.events, before)

    def test_second_driver_cannot_take_over_active_outputs(self):
        self.initialize()
        other = motor.MotorDriver(self.gpio)
        before = list(self.gpio.events)
        with self.assertRaises(motor.GPIOFault):
            other.initialize()
        self.assertTrue(other.close())
        self.assertEqual(self.gpio.events, before)
        self.assertTrue(self.driver.ready)

    def test_changed_global_pin_map_does_not_change_active_wiring_or_cleanup(self):
        self.initialize()
        self.gpio.events.clear()
        with patch.object(motor, 'MOTOR_PINS', ((4, 7, 8),) * 4), \
             patch.object(motor, 'STBY', 9), patch.object(motor, 'REVERSED', (True,) * 4):
            self.driver.set_pattern(motor.FORWARD)
            self.driver.set_duties((10, 10, 10, 10))
            self.assertEqual(self.gpio.levels[17], self.gpio.HIGH)
            self.assertTrue(self.driver.close())
        touched = {e[1] for e in self.gpio.events if e[0] in ['output', 'cleanup']}
        self.assertEqual(touched, set(self.pins))

    def test_setup_failure_cleans_partially_configured_pin(self):
        pin = motor.MOTOR_PINS[1][0]
        self.gpio.failures[('setup', pin, self.gpio.OUT, 0)] = 1
        with self.assertRaises(OSError):
            self.driver.initialize()
        configured = [e[1] for e in self.gpio.events if e[0] == 'setup']
        cleaned = [e[1] for e in self.gpio.events if e[0] == 'cleanup']
        self.assertEqual(cleaned, configured)
        self.assertIn(pin, cleaned)
        self.assertTrue(all(self.gpio.functions[p] == self.gpio.IN for p in cleaned))
        self.assertTrue(self.driver.faulted)
        with self.assertRaises(motor.GPIOFault):
            self.driver.initialize()

    def test_pwm_creation_failure_cleans_partial_setup(self):
        pin = motor.MOTOR_PINS[1][2]
        self.gpio.failures[('pwm_create', pin, motor.PWM_FREQ)] = 1
        with self.assertRaises(OSError):
            self.driver.initialize()
        self.assertFalse(self.driver.ready)
        self.assertTrue(all(p.stopped for p in self.gpio.pwms))
        self.assertTrue(all(v == self.gpio.IN for v in self.gpio.functions.values()))

    def test_pwm_start_failure_stops_created_pwm(self):
        pin = motor.MOTOR_PINS[1][2]
        self.gpio.failures[('start', pin, 0)] = 1
        with self.assertRaises(OSError):
            self.driver.initialize()
        self.assertIn(('pwm_stop', pin), self.gpio.events)
        self.assertTrue(all(v == self.gpio.IN for v in self.gpio.functions.values()))

    def test_setup_warning_becomes_error_and_runs_cleanup(self):
        self.gpio.setup_warning_pin = motor.MOTOR_PINS[0][0]
        with self.assertRaises(RuntimeWarning):
            self.driver.initialize()
        self.assertTrue(self.driver.faulted)
        self.assertTrue(all(v == self.gpio.IN for v in self.gpio.functions.values()))

    def test_direction_change_disables_board_and_zeros_pwm_before_reenable(self):
        self.initialize()
        self.driver.set_pattern(motor.FORWARD)
        self.driver.set_duties((20,) * 4)
        self.gpio.events.clear()
        self.driver.set_pattern(motor.TURN_LEFT)
        self.assertEqual(self.gpio.events[0], ('output', motor.STBY, 0))
        self.assertEqual(self.gpio.events[-1], ('output', motor.STBY, 1))
        zeros = [e for e in self.gpio.events if e[0] == 'duty' and e[-1] == 0]
        self.assertEqual(len(zeros), 4)
        self.assertEqual(self.driver.applied_duties, [0] * 4)

    def test_pwm_write_failure_attempts_stop_and_blocks_future_motion(self):
        self.initialize()
        self.driver.set_pattern(motor.FORWARD)
        pin = motor.MOTOR_PINS[1][2]
        self.gpio.failures[('duty', pin, 20.0)] = 1
        with self.assertRaises(motor.GPIOFault):
            self.driver.set_duties((20,) * 4)
        self.assertTrue(self.driver.faulted)
        self.assertEqual(self.gpio.levels[motor.STBY], 0)
        self.assertEqual([p.duty for p in self.gpio.pwms], [0] * 4)
        with self.assertRaises(motor.GPIOFault):
            self.driver.set_pattern(motor.FORWARD)
        with self.assertRaises(motor.GPIOFault):
            self.driver.set_duties((10,) * 4)

    def test_direction_write_failure_leaves_standby_low(self):
        self.initialize()
        self.gpio.failures[('output', motor.MOTOR_PINS[1][0], 1)] = 1
        with self.assertRaises(motor.GPIOFault):
            self.driver.set_pattern(motor.FORWARD)
        self.assertEqual(self.gpio.levels[motor.STBY], 0)
        self.assertTrue(self.driver.faulted)

    def test_stop_continues_after_one_pwm_failure_and_does_not_brake_enable(self):
        self.initialize()
        self.driver.set_pattern(motor.FORWARD)
        self.driver.set_duties((20,) * 4)
        self.gpio.events.clear()
        self.gpio.failures[('duty', motor.MOTOR_PINS[0][2], 0)] = 1
        with self.assertRaises(motor.GPIOFault):
            self.driver.stop()
        for pins in motor.MOTOR_PINS:
            self.assertIn(('duty', pins[2], 0), self.gpio.events)
            self.assertIn(('output', pins[0], 0), self.gpio.events)
        self.assertNotIn(('output', motor.STBY, 1), self.gpio.events)
        self.assertEqual(self.gpio.levels[motor.STBY], 0)

    def test_standby_failure_does_not_skip_pwm_zeroing(self):
        self.initialize()
        self.gpio.events.clear()
        self.gpio.failures[('output', motor.STBY, 0)] = 2
        with self.assertRaises(motor.GPIOFault):
            self.driver.stop()
        self.assertEqual(len([e for e in self.gpio.events if e[0] == 'duty']), 4)
        self.assertNotIn(('output', motor.STBY, 1), self.gpio.events)

    def test_brake_enable_failure_retries_standby_low(self):
        self.initialize()
        self.gpio.failures[('output', motor.STBY, 1)] = 1
        with self.assertRaises(motor.GPIOFault):
            self.driver.stop()
        self.assertEqual(self.gpio.events[-1], ('output', motor.STBY, 0))

    def test_normal_close_stops_all_pwm_and_releases_only_owned_pins(self):
        self.initialize()
        self.gpio.functions[26] = self.gpio.OUT
        self.assertTrue(self.driver.close())
        self.assertTrue(all(p.stopped for p in self.gpio.pwms))
        self.assertEqual([e[1] for e in self.gpio.events if e[0] == 'cleanup'], self.pins)
        self.assertEqual(self.gpio.functions[26], self.gpio.OUT)
        self.assertFalse(self.driver.ready)
        before = list(self.gpio.events)
        self.assertTrue(self.driver.close())
        self.assertEqual(self.gpio.events, before)

    def test_close_continues_after_pwm_stop_and_pin_cleanup_failures(self):
        self.initialize()
        self.gpio.failures[('pwm_stop', motor.MOTOR_PINS[0][2])] = 1
        self.gpio.failures[('cleanup', motor.STBY)] = 1
        self.assertFalse(self.driver.close())
        self.assertEqual(len([e for e in self.gpio.events if e[0] == 'pwm_stop']), 4)
        self.assertEqual([e[1] for e in self.gpio.events if e[0] == 'cleanup'], self.pins)
        self.assertTrue(self.driver.faulted)
        with self.assertRaises(motor.GPIOFault):
            self.driver.initialize()

    def test_duty_validation_still_rejects_out_of_range_before_output(self):
        self.initialize()
        self.driver.set_pattern(motor.FORWARD)
        before = list(self.gpio.events)
        with self.assertRaises(ValueError):
            self.driver.set_duties((100,) * 4)
        self.assertEqual(self.gpio.events, before)


if __name__ == '__main__':
    unittest.main(verbosity=2)

"""Simulated pulse train + real driver checks; no Raspberry Pi required."""

import math
import unittest
from unittest.mock import patch

import ir_tach as tach
import motors_optimized as motor
from test_motors_optimized import FakeGPIO, FakeKeys


class PulseClock:
    def __init__(self):
        self.now = 0.0
        self.gpio = self.driver = None
        self.phase = 0.5
        self.emit_pulses = True
        self.jump_at = None
        self.rpm_for = lambda duty, wheel, forward: duty * 8.0
        self.samples = []

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        if duration <= 0:
            raise AssertionError("Nonpositive sleep")
        end = self.now + duration
        if self.jump_at is not None and end >= self.jump_at:
            end += 0.25
            self.jump_at = None
        active = [(i, pwm.duty) for i, pwm in enumerate(self.gpio.pwms) if pwm.duty]
        if len(active) > 1:
            raise AssertionError("More than one wheel powered")
        if active:
            wheel, duty = active[0]
            forward = self.driver.pattern[wheel]
            self.samples.append((wheel, forward, duty, self.driver._raw_calibration))
            frequency = self.rpm_for(duty, wheel, forward) / 60.0
            if frequency > 0 and self.emit_pulses:
                while self.now < end - 1e-12:
                    whole = math.floor(self.phase + 1e-10)
                    fraction = self.phase - whole
                    edge_phase = whole + (0.25 if fraction < 0.25 - 1e-10 else 1.0)
                    edge_time = self.now + (edge_phase - self.phase) / frequency
                    if edge_time > end:
                        self.phase += (end - self.now) * frequency
                        break
                    self.now, self.phase = edge_time, edge_phase
                    self.gpio.emit(self.now, int(abs(edge_phase - round(edge_phase)) < 1e-9))
        self.now = end
        if self.now > 240:
            raise AssertionError("Calibration was not bounded")


class PulseGPIO(FakeGPIO):
    BOTH, PUD_DOWN = "BOTH", "DOWN"

    def __init__(self, clock):
        super().__init__(clock)
        clock.gpio = self
        self.probe_level = 0
        self.callback = None
        self.cleanups = []
        self.setup_calls = []
        self.removed = []
        self.fail_detect = False

    def setup(self, pin, mode, initial=None, pull_up_down=None):
        self.setup_calls.append((pin, mode, pull_up_down))
        super().setup(pin, mode, initial)

    def input(self, pin):
        if pin == 20:
            return self.probe_level
        return super().input(pin)

    def add_event_detect(self, pin, edge, callback):
        if self.fail_detect:
            raise RuntimeError("Edge registration failed")
        assert pin == 20 and edge == self.BOTH
        self.callback = callback

    def remove_event_detect(self, pin):
        self.removed.append(pin)
        self.callback = None

    def emit(self, when, level):
        self.clock.now = when
        self.probe_level = level
        if self.callback:
            self.callback(20)

    def cleanup(self, pins=None):
        self.cleanups.append(pins)
        if pins == 20:
            self.modes.pop(20, None)
        else:
            super().cleanup(pins)


class TachometerTests(unittest.TestCase):
    def make(self):
        clock = PulseClock()
        gpio = PulseGPIO(clock)
        return tach.Tachometer(gpio, 20, clock), gpio, clock

    def train(self, gpio, periods=(0.25,) * 4):
        when = 0.1
        gpio.emit(when, 1)
        for period in periods:
            gpio.emit(when + 0.04, 0)
            when += period
            gpio.emit(when, 1)

    def test_one_same_edge_per_revolution_and_averaged_rpm(self):
        sensor, gpio, clock = self.make()
        with sensor:
            self.train(gpio)
            self.assertAlmostEqual(sensor.rpm(), 240)
            snapshot = sensor.snapshot()
            self.assertEqual(snapshot.turn_count, 5)
            self.assertEqual(snapshot.edge_count, 9)
            self.assertEqual(len(snapshot.periods), 4)
        self.assertEqual(gpio.removed, [20])
        self.assertEqual(gpio.cleanups, [20])

    def test_bounded_callback_memory(self):
        sensor, gpio, _ = self.make()
        with sensor:
            self.train(gpio, (0.25,) * 100)
            self.assertEqual(len(sensor.snapshot().periods), 8)

    def test_conflicting_pins_rejected_before_any_setup(self):
        _, gpio, clock = self.make()
        for pin in (7, 21, 26, motor.STBY, 17, 4, True, "20"):
            with self.subTest(pin=pin), self.assertRaises(ValueError):
                tach.Tachometer(gpio, pin, clock)
        self.assertFalse(gpio.setup_calls)
        sensor = tach.Tachometer(gpio, 20, clock)
        with patch.object(motor, "IR_PINS", (20, 7, 26)), self.assertRaises(ValueError):
            sensor.__enter__()
        self.assertFalse(gpio.setup_calls)

    def test_partial_open_failure_only_cleans_probe(self):
        sensor, gpio, _ = self.make()
        gpio.fail_detect = True
        with self.assertRaisesRegex(RuntimeError, "registration"):
            sensor.__enter__()
        self.assertEqual(gpio.cleanups, [20])

    def test_stationary_hand_probe_allowed_but_motion_requires_signal(self):
        sensor, _, clock = self.make()
        with sensor:
            clock.now = 20
            sensor.check(require_motion=False)
            with self.assertRaisesRegex(tach.TachFault, "No fresh"):
                sensor.rpm()

    def test_stale_previously_valid_signal_rejected(self):
        sensor, gpio, clock = self.make()
        with sensor:
            self.train(gpio)
            clock.now += tach.PULSE_TIMEOUT + 0.01
            with self.assertRaisesRegex(tach.TachFault, "No fresh"):
                sensor.rpm()

    def test_noise_and_missed_edge_latch_fault(self):
        for when, level, message in ((0.1001, 0, "narrow"), (0.2, 1, "missed")):
            with self.subTest(message=message):
                sensor, gpio, _ = self.make()
                with sensor:
                    gpio.emit(0.1, 1)
                    gpio.emit(when, level)
                    with self.assertRaisesRegex(tach.TachFault, message):
                        sensor.check()

    def test_irregular_revolutions_or_skipped_pulse_rejected(self):
        sensor, gpio, _ = self.make()
        with sensor:
            self.train(gpio, (0.25, 0.25, 0.5, 0.25))
            with self.assertRaisesRegex(tach.TachFault, "inconsistent"):
                sensor.rpm()

    def test_speed_range_and_backwards_clock(self):
        sensor, gpio, _ = self.make()
        with sensor:
            self.train(gpio, (0.045,) * 4)
            with self.assertRaisesRegex(tach.TachFault, "range"):
                sensor.rpm()
            sensor.reset()
            gpio.emit(0.001, 1 - gpio.probe_level)
            with self.assertRaisesRegex(tach.TachFault, "backwards"):
                sensor.check()


class CalibrationTests(unittest.TestCase):
    def make(self, events=()):
        clock = PulseClock()
        gpio = PulseGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        clock.driver = driver
        def log(_):
            self.assertFalse(any(driver.duties), "Printing while energized")
        runner = tach.CalibrationRunner(driver, FakeKeys(clock, events), 20, clock, log)
        return runner, driver, gpio, clock

    def assert_stopped(self, driver, gpio):
        self.assertEqual(driver.duties, [0.0] * 4)
        self.assertEqual(driver.applied_duties, [0.0] * 4)
        self.assertEqual(gpio.values[motor.STBY], 0)
        self.assertFalse(driver._raw_calibration)

    def test_measure_real_driver_correct_wheel_direction_and_raw_duty(self):
        with patch.object(motor, "WHEEL_FACTORS_FORWARD", (0.6,) * 4), \
                patch.object(motor, "WHEEL_FACTORS_REVERSE", (0.7,) * 4):
            for wheel in range(4):
                for forward in (True, False):
                    with self.subTest(wheel=wheel, forward=forward):
                        runner, driver, gpio, clock = self.make()
                        with runner:
                            rpm = runner.measure(wheel, forward, 25)
                            self.assertAlmostEqual(rpm, 200, places=6)
                            self.assert_stopped(driver, gpio)
                        self.assertTrue(all(i == wheel and direction == forward and raw
                                            for i, direction, _, raw in clock.samples))
                        self.assertEqual(max(duty for _, _, duty, _ in clock.samples), 25)
                        self.assertEqual(gpio.cleanups, [20])

    def test_missing_disconnected_and_stalled_sensor_are_bounded(self):
        for stuck in (0, 1):
            runner, driver, gpio, clock = self.make()
            clock.emit_pulses = False
            gpio.probe_level = stuck
            with runner, self.assertRaisesRegex(tach.TachFault, "No fresh"):
                runner.measure(0, True, 25)
            self.assertLess(clock.now, 5.5)
            self.assert_stopped(driver, gpio)

    def test_stop_during_coast_ramp_settling_measurement_and_before_motion(self):
        for at in (0, 0.12, 0.62, 1.5, 2.2):
            runner, driver, gpio, clock = self.make([(at, "s")])
            with runner, self.assertRaises(motor.StopRequested):
                runner.measure(3, False, 25)
            self.assertLessEqual(clock.now, at + motor.LOOP_DT + 1e-8)
            self.assert_stopped(driver, gpio)
            if at < tach.COAST_SECONDS:
                self.assertFalse(clock.samples)

    def test_guard_polled_and_fault_coasts(self):
        runner, driver, gpio, clock = self.make()
        class Guard:
            trip_reason = None
            calls = 0
            def check(self, motors):
                self.calls += 1
                if clock.now >= 1.0:
                    self.trip_reason = "test power fault"
                    raise RuntimeError(self.trip_reason)
        driver.power_guard = Guard()
        with runner, self.assertRaisesRegex(RuntimeError, "power fault"):
            runner.measure(1, True, 25)
        self.assertGreater(driver.power_guard.calls, 20)
        self.assert_stopped(driver, gpio)

    def test_control_gap_stops_instead_of_catching_up(self):
        runner, driver, gpio, clock = self.make()
        clock.jump_at = 0.6
        with runner, self.assertRaisesRegex(tach.TachFault, "interrupted"):
            runner.measure(0, True, 25)
        self.assert_stopped(driver, gpio)

    def test_hard_deadline_includes_an_excessively_slow_ramp(self):
        runner, driver, gpio, clock = self.make()
        with patch.object(motor, "RAMP_RATE", 0.1), runner, \
                self.assertRaisesRegex(tach.TachFault, "timed out"):
            runner.measure(0, True, 25)
        self.assertLessEqual(clock.now, tach.ATTEMPT_TIMEOUT + tach.COAST_SECONDS + motor.LOOP_DT)
        self.assert_stopped(driver, gpio)

    def test_direction_changes_and_repeated_starts_have_unpowered_pause(self):
        runner, driver, gpio, clock = self.make()
        with runner:
            runner.measure(0, True, 25)
            first_stopped = clock.now
            old_writes = len(gpio.pwm_writes)
            runner.measure(0, False, 25)
            first_powered = next(when for when, _, duty in gpio.pwm_writes[old_writes:] if duty)
            self.assertGreaterEqual(first_powered - first_stopped, tach.COAST_SECONDS)
            for when, _, duty in gpio.pwm_writes[old_writes:]:
                if when < first_stopped + tach.COAST_SECONDS:
                    self.assertEqual(duty, 0)
        self.assert_stopped(driver, gpio)

    def test_minimum_supported_rpm_finishes_within_powered_deadline(self):
        runner, driver, gpio, clock = self.make()
        clock.rpm_for = lambda duty, wheel, forward: duty * 1.2
        with runner:
            self.assertAlmostEqual(runner.measure(0, True, 25), 30)
        self.assertLess(clock.now, tach.ATTEMPT_TIMEOUT + tach.COAST_SECONDS)
        self.assert_stopped(driver, gpio)

    def test_bad_duty_or_wheel_does_not_power_anything(self):
        runner, driver, gpio, clock = self.make()
        with runner:
            for wheel, direction, duty in ((0, True, 71), (0, True, 0), (0, True, math.nan),
                                            (4, True, 25), (0, "forward", 25)):
                with self.assertRaises(ValueError):
                    runner.measure(wheel, direction, duty)
            self.assertFalse(clock.samples)
        self.assert_stopped(driver, gpio)

    def test_calibration_converges_and_never_exceeds_reference(self):
        runner, driver, gpio, clock = self.make()
        with runner:
            factor, rpm = runner.calibrate(2, False, 160, 25)
            self.assertLessEqual(abs(rpm / 160 - 1), tach.RPM_TOLERANCE)
            self.assertLessEqual(abs(factor - 0.8), 0.04)
            self.assert_stopped(driver, gpio)
        self.assertLessEqual(max(sample[2] for sample in clock.samples), 25)
        self.assertTrue(all(sample[0:2] == (2, False) for sample in clock.samples))

    def test_unattainable_target_or_half_duty_floor_fails(self):
        for target in (250, 50):
            runner, driver, gpio, clock = self.make()
            with runner, self.assertRaises(tach.TachFault):
                runner.calibrate(0, True, target, 25)
            self.assert_stopped(driver, gpio)
            self.assertLess(clock.now, 60)

    def test_independent_final_verification_required(self):
        runner, driver, gpio, _ = self.make()
        with runner, patch.object(runner, "measure", side_effect=(200, 160, 190)) as measure, \
                self.assertRaisesRegex(tach.TachFault, "verification"):
            runner.calibrate(0, True, 160, 25)
        self.assertEqual(measure.call_count, 3)
        self.assert_stopped(driver, gpio)

    def test_target_below_supported_rpm_rejected_before_motor_run(self):
        runner, driver, gpio, clock = self.make()
        with runner, self.assertRaises(ValueError):
            runner.calibrate(0, True, 15, 25)
        self.assertFalse(clock.samples)
        self.assert_stopped(driver, gpio)


if __name__ == "__main__":
    unittest.main()

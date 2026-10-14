"""Offline confidence filtering, lane decisions, stop paths and menu tests."""

import contextlib
import io
import math
import sys
import types
import unittest
from pathlib import Path
import tempfile
from unittest.mock import patch

import ir_confidence as ir
import app as menu
from fake_power import FakeGuard
import motors_optimized as motor
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys


class ConfidenceFilterTests(unittest.TestCase):
    def settled_tracker(self):
        tracker = ir.ConfidenceTracker()
        for i in range(10):
            state = tracker.update((1, 1, 1), i * 0.02)
        self.assertEqual(state.action, "clear")
        return tracker

    def test_unknown_startup_does_not_imply_clear_lane(self):
        for raw in ((1, 1, 1), (0, 1, 1), (0, 0, 1)):
            tracker = ir.ConfidenceTracker()
            for i in range(6):
                self.assertEqual(tracker.update(raw, i * 0.02).action, "warming")
            self.assertNotEqual(tracker.update(raw, 0.12).action, "warming")

    def test_one_sample_glitch_does_not_request_steering(self):
        tracker = self.settled_tracker()
        states = [tracker.update((0, 1, 1), 0.20)]
        states.extend(tracker.update((1, 1, 1), 0.22 + i * 0.02) for i in range(6))
        self.assertTrue(all(state.action not in ("left", "right") for state in states))
        self.assertEqual(states[-1].action, "clear")

    def test_persistent_edges_correct_away_from_corresponding_side(self):
        for raw, expected in (((0, 1, 1), "right"), ((1, 0, 1), "left")):
            tracker = self.settled_tracker()
            states = [tracker.update(raw, 0.20 + i * 0.02) for i in range(6)]
            self.assertEqual(states[-1].action, expected)
            self.assertTrue(any(state.action == "uncertain" for state in states))

    def test_hysteresis_survives_one_clear_read_then_releases(self):
        tracker = self.settled_tracker()
        for i in range(10):
            state = tracker.update((0, 1, 1), 0.20 + i * 0.02)
        self.assertEqual(state.action, "right")
        state = tracker.update((1, 1, 1), 0.40)
        self.assertEqual(state.action, "right")
        for i in range(1, 7):
            state = tracker.update((1, 1, 1), 0.40 + i * 0.02)
        self.assertEqual(state.action, "clear")

    def test_elapsed_time_not_sample_count_controls_smoothing(self):
        results = []
        for interval in (0.01, 0.02, 0.03):
            tracker = ir.ConfidenceTracker()
            for i in range(round(0.12 / interval) + 1):
                state = tracker.update((0, 1, 1), round(i * interval, 8))
            results.append(state.scores)
        for scores in results[1:]:
            for value, expected in zip(scores, results[0]):
                self.assertAlmostEqual(value, expected)

    def test_alternating_and_both_white_inputs_are_ambiguous(self):
        for alternating in (True, False):
            tracker = ir.ConfidenceTracker()
            for i in range(30):
                value = i % 2 if alternating else 0
                state = tracker.update((value, value, 1), i * 0.02)
            self.assertEqual(state.action, "uncertain")
            self.assertLessEqual(max(ir.confidence_targets(state)), ir.UNCERTAIN_DUTY)
            self.assertEqual(ir.confidence_targets(state, driving=False), (0,) * 4)

    def test_steering_strength_grows_with_white_support_and_stays_bounded(self):
        # Lean right slows FR+BL; lean left slows FL+BR (see motor.STRAFE_RIGHT).
        for action, index, slower in (("right", 0, (1, 2)), ("left", 1, (0, 3))):
            values = []
            for support in (0.31, 0.7, 1.0):
                scores = [0.0, 0.0]
                scores[index] = support
                state = ir.ConfidenceState((0, 1, 1), tuple(scores), (True, False), action)
                targets = ir.confidence_targets(state)
                self.assertTrue(all(0 <= value <= ir.MAX_DUTY for value in targets))
                values.append(targets)
            for wheel in slower:
                self.assertGreater(values[0][wheel], values[1][wheel])
                self.assertGreater(values[1][wheel], values[2][wheel])

    def test_bad_inputs_nonmonotonic_time_and_sampling_gaps_fail(self):
        for raw, now in (((2, 1, 1), 0.02), ((1, 1), 0.02), ((1, 1, 1), math.nan),
                         ((1, 1, 1), 0), ((1, 1, 1), -0.1), ((1, 1, 1), 0.2)):
            with self.subTest(raw=raw, now=now):
                tracker = ir.ConfidenceTracker()
                tracker.update((1, 1, 1), 0)
                with self.assertRaises(motor.LaneFault):
                    tracker.update(raw, now)


class ConfidenceControlTests(unittest.TestCase):
    def make(self, events=()):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        messages = []
        control = ir.ConfidenceTests(driver, FakeKeys(clock, events), clock,
                                     lambda text: messages.append((clock.now, text)))
        return control, driver, gpio, clock, messages

    def assert_stopped(self, driver, gpio):
        self.assertEqual(driver.duties, [0] * 4)
        self.assertTrue(all(pwm.duty == 0 for pwm in gpio.pwms))

    def test_display_shows_scores_and_front_state_without_powering_wheels(self):
        control, driver, gpio, _, messages = self.make([(0.55, "s")])
        gpio.readings = lambda now: (0, 1, 0)
        with self.assertRaises(motor.StopRequested):
            control.run_test("IRC")
        self.assertTrue(any("white support" in text and "FRONT STOP" in text for _, text in messages))
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assert_stopped(driver, gpio)

    def test_clear_lane_warms_up_then_runs_at_capped_duty_until_time_limit(self):
        control, driver, gpio, clock, messages = self.make()
        control.run_test("FC")
        self.assertEqual(clock.now, ir.FORWARD_SECONDS)
        positive = [(t, duty) for t, _, duty in gpio.pwm_writes if duty > 0]
        self.assertGreaterEqual(min(t for t, _ in positive), ir.WARMUP_SECONDS)
        self.assertAlmostEqual(max(duty for _, duty in positive), ir.MAX_DUTY)
        self.assertTrue(any("time limit" in text for _, text in messages))
        self.assert_stopped(driver, gpio)

    def test_front_stop_bypasses_filter_during_startup_ramp_correction_and_ambiguity(self):
        for stop_at, sides in ((0, (1, 1)), (0.06, (1, 1)), (0.24, (1, 1)),
                               (0.9, (0, 1)), (0.9, (0, 0))):
            with self.subTest(stop_at=stop_at, sides=sides):
                control, driver, gpio, clock, _ = self.make()
                gpio.readings = lambda now: (*(sides if now >= 0.7 else (1, 1)), 0 if now >= stop_at else 1)
                control.run_test("FC")
                self.assertEqual(clock.now, stop_at)
                self.assertTrue(all(duty == 0 for t, _, duty in gpio.pwm_writes if t >= stop_at))
                self.assert_stopped(driver, gpio)

    def test_stop_key_during_warmup_ramp_correction_and_uncertainty(self):
        for stop_at, sides in ((0, (1, 1)), (0.04, (1, 1)), (0.30, (1, 1)),
                               (0.90, (0, 1)), (0.90, (0, 0))):
            with self.subTest(stop_at=stop_at, sides=sides):
                control, driver, gpio, clock, _ = self.make([(stop_at, "s")])
                gpio.readings = lambda now: (*(sides if now >= 0.7 else (1, 1)), 1)
                with self.assertRaises(motor.StopRequested):
                    control.run_test("FC")
                self.assertEqual(clock.now, stop_at)
                self.assert_stopped(driver, gpio)

    def test_left_and_right_edge_corrections_slow_the_correct_diagonal_pair_then_recover(self):
        # LEFT edge leans right (FR+BL slowed); RIGHT edge leans left (FL+BR).
        for side, slower in ((0, (1, 2)), (1, (0, 3))):
            with self.subTest(side=side):
                control, driver, gpio, _, _ = self.make([(1.5, "s")])
                observations = []

                def readings(now):
                    observations.append((now, tuple(driver.duties)))
                    raw = [1, 1, 1]
                    if 0.7 <= now < 1.0:
                        raw[side] = 0
                    return tuple(raw)

                gpio.readings = readings
                with self.assertRaises(motor.StopRequested):
                    control.run_test("FC")
                during = next(duties for t, duties in observations if t == 0.98)
                after = next(duties for t, duties in observations if t == 1.48)
                for wheel in slower:
                    self.assertLess(during[wheel], 20)
                    self.assertGreater(after[wheel], 24)
                for wheel in set(range(4)) - set(slower):
                    self.assertGreater(during[wheel], 24)
                self.assert_stopped(driver, gpio)

    def test_both_white_at_start_never_launches_and_times_out(self):
        control, driver, gpio, clock, _ = self.make()
        gpio.readings = lambda now: (0, 0, 1)
        with self.assertRaisesRegex(motor.LaneFault, "unclear"):
            control.run_test("FC")
        self.assertLessEqual(clock.now, 0.64)
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assert_stopped(driver, gpio)

    def test_ambiguous_lane_slows_then_stops_instead_of_continuing_indefinitely(self):
        control, driver, gpio, clock, _ = self.make()
        observations = []

        def readings(now):
            observations.append((now, tuple(driver.duties)))
            return (0, 0, 1) if now >= 1.0 else (1, 1, 1)

        gpio.readings = readings
        with self.assertRaisesRegex(motor.LaneFault, "unclear"):
            control.run_test("FC")
        self.assertLessEqual(clock.now, 1.56)
        self.assertLessEqual(max(next(d for t, d in observations if t == 1.4)), ir.UNCERTAIN_DUTY)
        self.assert_stopped(driver, gpio)

    def test_stuck_side_cannot_produce_endless_high_confidence_correction(self):
        for noisy in (False, True):
            with self.subTest(noisy=noisy):
                control, driver, gpio, clock, _ = self.make()
                gpio.readings = lambda now: (1 if noisy and round(now * 50) % 10 == 0 else 0, 1, 1)
                with self.assertRaisesRegex(motor.LaneFault, "correction timed out"):
                    control.run_test("FC")
                self.assertLessEqual(clock.now, 1.0)
                self.assert_stopped(driver, gpio)

    def test_brief_clear_blips_do_not_reset_correction_timeout(self):
        control, driver, gpio, clock, _ = self.make()
        gpio.readings = lambda now: (1 if round(now * 50) % 15 >= 10 else 0, 1, 1)
        with self.assertRaisesRegex(motor.LaneFault, "correction timed out"):
            control.run_test("FC")
        self.assertLessEqual(clock.now, 1.0)
        self.assert_stopped(driver, gpio)

    def test_recovered_lane_resets_timeout_for_a_later_independent_correction(self):
        control, driver, gpio, _, _ = self.make([(2.3, "s")])
        gpio.readings = lambda now: (0 if 0.7 <= now < 1.0 or 1.7 <= now < 2.0 else 1, 1, 1)
        with self.assertRaises(motor.StopRequested):
            control.run_test("FC")
        self.assert_stopped(driver, gpio)

    def test_delayed_control_loop_and_sensor_read_stop_movement(self):
        for delay_read in (False, True):
            with self.subTest(delay_read=delay_read):
                control, driver, gpio, clock, _ = self.make()
                sleep = clock.sleep
                if delay_read:
                    def slow_read():
                        if clock.now >= 0.6:
                            sleep(0.2)
                        return (1, 1, 1)
                    driver.read_ir = slow_read
                else:
                    clock.sleep = lambda duration: sleep(duration + (0.2 if clock.now >= 0.6 else 0))
                with self.assertRaises(motor.LaneFault):
                    control.run_test("FC")
                self.assertLessEqual(clock.now, 0.82)
                self.assert_stopped(driver, gpio)

    def test_gpio_exception_stops_motors(self):
        control, driver, gpio, _, _ = self.make()

        def readings(now):
            if now >= 0.7:
                raise OSError("Injected GPIO error")
            return (1, 1, 1)

        gpio.readings = readings
        with self.assertRaises(OSError):
            control.run_test("FC")
        self.assert_stopped(driver, gpio)

    def test_menu_routes_both_new_commands_without_changing_old_fa_dispatch(self):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        rpi = types.ModuleType("RPi")
        rpi.GPIO = gpio
        calls = []
        new_control = types.SimpleNamespace(run_test=lambda command: calls.append(command))
        old_control = types.SimpleNamespace(run_mode=lambda command, sequence: calls.append(command), poll=lambda: None)
        choices = iter(["IRC", "FC", "FA", "Q"])
        with patch.dict(sys.modules, {"RPi": rpi, "RPi.GPIO": gpio}), \
             patch.object(menu.sys.stdin, "isatty", return_value=True), \
             patch.object(menu, "PowerGuard", FakeGuard), \
             patch.object(menu, "ConfidenceTests", return_value=new_control), \
             patch.object(motor, "Controller", return_value=old_control), \
             patch.object(motor, "prompt_line", side_effect=lambda text: next(choices)), \
             patch.object(motor, "Keyboard", side_effect=lambda: contextlib.nullcontext(FakeKeys(clock))), \
             contextlib.redirect_stdout(io.StringIO()):
            with tempfile.TemporaryDirectory() as folder:
                self.assertEqual(menu.main(["--bot", "500", "--settings", str(Path(folder) / "settings.json")]), 0)
        self.assertEqual(calls, ["IRC", "FC", "FA"])
        self.assertTrue(gpio.cleaned)


if __name__ == "__main__":
    unittest.main(verbosity=2)

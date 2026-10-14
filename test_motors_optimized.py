"""Hardware-free regression tests for the optimized controller.

Run: python -m unittest -v test_motors_optimized
The real MotorDriver and Controller run against recording GPIO and clock fakes.
"""

import contextlib
import io
import sys
import types
import unittest
from unittest.mock import patch

import motors_optimized as motor


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        if duration <= 0:
            raise AssertionError("A loop attempted a nonpositive sleep")
        self.now = round(self.now + duration, 10)
        if self.now > 30:
            raise AssertionError("Control loop did not finish within 30 simulated seconds")


class FakePWM:
    def __init__(self, gpio, pin):
        self.gpio = gpio
        self.pin = pin
        self.duty = 0
        self.stopped = False

    def start(self, duty):
        self.ChangeDutyCycle(duty)

    def ChangeDutyCycle(self, duty):
        assert 0 <= duty <= 100
        self.duty = duty
        self.gpio.pwm_writes.append((self.gpio.clock.now, self.pin, duty))

    def stop(self):
        self.duty = 0
        self.stopped = True


class FakeGPIO:
    BCM, OUT, IN, HIGH, LOW = "BCM", "OUT", "IN", 1, 0

    def __init__(self, clock):
        self.clock = clock
        self.pwms = []
        self.pwm_writes = []
        self.pin_writes = []
        self.values = {}
        self.modes = {}
        self.read_times = []
        self.readings = lambda now: (1, 1, 1)
        self.cleaned = False
        self.fail_setup_pin = None

    def setmode(self, mode):
        assert mode == self.BCM

    def setup(self, pin, mode, initial=None):
        if pin == self.fail_setup_pin:
            raise RuntimeError("Injected GPIO initialization failure")
        self.modes[pin] = mode
        if initial is not None:
            assert initial == 0  # Hardware must initialize with every output LOW.
            self.values[pin] = initial

    def output(self, pin, value):
        assert self.modes[pin] == self.OUT
        self.pin_writes.append((self.clock.now, pin, value,
                                tuple(pwm.duty for pwm in self.pwms),
                                self.values.get(motor.STBY)))
        self.values[pin] = value

    def input(self, pin):
        assert self.modes[pin] == self.IN
        self.read_times.append(self.clock.now)
        return self.readings(self.clock.now)[motor.IR_PINS.index(pin)]

    def PWM(self, pin, frequency):
        assert frequency == motor.PWM_FREQ
        pwm = FakePWM(self, pin)
        self.pwms.append(pwm)
        return pwm

    def cleanup(self, pins=None):
        # None is RPi.GPIO's every-channel cleanup, which also closes rpi-lgpio's chip handle.
        assert pins is None or set(pins) == set(self.modes)
        self.cleaned = True


class LgpioLikeGPIO(FakeGPIO):
    """rpi-lgpio as measured on the Trixie card (2026-09-25): a pin freed and claimed
    again through a still-open gpiochip handle outputs no PWM; a fresh handle is clean."""

    def __init__(self, clock):
        super().__init__(clock)
        self.handle_open = False
        self.freed, self.dead = set(), set()

    def setmode(self, mode):
        super().setmode(mode)
        if not self.handle_open:
            self.handle_open = True
            self.freed.clear()

    def setup(self, pin, mode, initial=None):
        super().setup(pin, mode, initial)
        if pin in self.freed:
            self.dead.add(pin)

    def cleanup(self, pins=None):
        super().cleanup(pins)
        self.freed |= set(self.modes if pins is None else pins)
        if pins is None:
            self.handle_open = False


class FakeKeys:
    def __init__(self, clock, events=()):
        self.clock = clock
        self.events = list(events)

    def poll(self):
        result = ""
        while self.events and self.events[0][0] <= self.clock.now + 1e-9:
            result += self.events.pop(0)[1]
        return result


class ControllerTests(unittest.TestCase):
    def make(self, events=()):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        messages = []
        controller = motor.Controller(driver, FakeKeys(clock, events), clock,
                                      lambda text: messages.append((clock.now, text)))
        return controller, driver, gpio, clock, messages

    def assert_off(self, driver, gpio):
        self.assertEqual(driver.duties, [0.0] * 4)
        self.assertTrue(all(pwm.duty == 0 for pwm in gpio.pwms))
        if gpio.values[motor.STBY] == 1:
            # Enabled bridges may only hold the TB6612FNG's short-brake pattern.
            for ain1, ain2, _ in motor.MOTOR_PINS:
                self.assertEqual((gpio.values[ain1], gpio.values[ain2]), (1, 1))

    def assert_unpowered_direction_writes(self, gpio):
        direction_pins = {pin for pins in motor.MOTOR_PINS for pin in pins[:2]}
        for _, pin, _, duties, standby in gpio.pin_writes:
            if pin in direction_pins:
                self.assertTrue(all(duty == 0 for duty in duties))
                self.assertEqual(standby, 0)

    def test_initialization_does_not_enable_drivers(self):
        _, driver, gpio, _, _ = self.make()
        self.assert_off(driver, gpio)
        self.assertEqual(gpio.values[motor.STBY], 0)
        driver.close()
        self.assertTrue(gpio.cleaned)
        self.assertTrue(all(pwm.stopped for pwm in gpio.pwms))
        self.assertEqual(gpio.values[motor.STBY], 0)

    def test_stop_brakes_and_close_disables_bridges(self):
        _, driver, gpio, _, _ = self.make()
        driver.set_pattern(motor.FORWARD)
        driver.set_duties(motor.straight_targets())
        driver.stop()
        self.assert_off(driver, gpio)
        self.assertEqual(gpio.values[motor.STBY], 1)
        self.assert_unpowered_direction_writes(gpio)
        driver.close()
        self.assertEqual(gpio.values[motor.STBY], 0)

    def test_pwm_still_drives_after_a_set_reload(self):
        # Leaving SET closes the driver and opens a new one in the same process.
        gpio = LgpioLikeGPIO(FakeClock())
        pwm_pins = {pins[2] for pins in motor.MOTOR_PINS}
        for session in ("launch", "reload"):
            driver = motor.MotorDriver(gpio)
            driver.initialize()
            self.assertFalse(pwm_pins & gpio.dead, f"PWM dead after {session}")
            self.assertTrue(driver.close())
            self.assertEqual(driver.pwm, [])
            self.assertFalse(gpio.handle_open, "close() must release the gpiochip handle")

    def test_coast_option_keeps_standby_low(self):
        _, driver, gpio, _, _ = self.make()
        driver.set_pattern(motor.FORWARD)
        driver.set_duties(motor.straight_targets())
        with patch.object(motor, "BRAKE_ON_STOP", False):
            driver.stop()
        self.assert_off(driver, gpio)
        self.assertEqual(gpio.values[motor.STBY], 0)

    def test_stop_during_acceleration_for_every_movement_mode(self):
        for mode in sorted(motor.MODES - {"IR"}):
            with self.subTest(mode=mode):
                controller, driver, gpio, clock, _ = self.make([(0.05, "s")])
                with self.assertRaises(motor.StopRequested):
                    controller.run_mode(mode, "FRL")
                self.assertLessEqual(clock.now, 0.07)
                self.assert_off(driver, gpio)
                self.assertLess(max(duty for _, _, duty in gpio.pwm_writes), 10)

    def test_stop_during_timed_hold_for_all_manual_moves(self):
        for mode in ("SR", "SL", "R", "L", "DR", "DL"):
            with self.subTest(mode=mode):
                controller, driver, gpio, clock, _ = self.make([(0.75, "s")])
                with self.assertRaises(motor.StopRequested):
                    controller.run_mode(mode)
                self.assertLessEqual(clock.now, 0.77)
                self.assert_off(driver, gpio)

    def test_queued_stop_prevents_any_positive_pwm(self):
        controller, driver, gpio, _, _ = self.make([(0.0, "s")])
        with self.assertRaises(motor.StopRequested):
            controller.run_mode("TR")
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assert_off(driver, gpio)

    def test_front_line_during_acceleration_is_detected(self):
        for auto in (False, True):
            with self.subTest(auto=auto):
                controller, driver, gpio, clock, _ = self.make()
                gpio.readings = lambda now: (1, 1, 0 if 0.2 <= now < 0.5 else 1)
                controller.drive_to_line(auto=auto)
                self.assertEqual(clock.now, 0.22)  # second white read in a row
                self.assertEqual(min(gpio.read_times), 0.0)
                self.assert_off(driver, gpio)

    def test_plain_forward_stops_immediately_if_already_on_line(self):
        controller, driver, gpio, clock, _ = self.make()
        gpio.readings = lambda now: (1, 1, 0)
        controller.drive_to_line(auto=False, ignore_initial_line=False)
        self.assertEqual(clock.now, 0.02)  # second white read in a row
        self.assert_off(driver, gpio)

    def test_auto_can_depart_initial_line_then_detect_next(self):
        controller, driver, gpio, clock, _ = self.make()
        gpio.readings = lambda now: (1, 1, 1 if 0.2 <= now < 0.4 else 0)
        controller.drive_to_line()
        self.assertEqual(clock.now, 0.42)  # v9.9: the second white read in a row confirms the line
        self.assert_off(driver, gpio)

    def test_ramp_continues_from_existing_speed(self):
        controller, driver, gpio, clock, _ = self.make()
        driver.set_pattern(motor.FORWARD)
        driver.set_duties(motor.straight_targets())
        previous_count = len(gpio.pwm_writes)
        controller.timed_move(motor.FORWARD, motor.straight_targets(), 0.4)
        self.assertEqual(clock.now, 0.4)
        self.assertEqual(len(gpio.pwm_writes), previous_count)

    def test_continuing_lane_following_does_not_reramp(self):
        controller, driver, gpio, _, _ = self.make()
        driver.set_pattern(motor.FORWARD)
        driver.set_duties(motor.straight_targets())
        gpio.readings = lambda now: (1, 1, 0 if now >= 0.2 else 1)
        gpio.pwm_writes.clear()
        controller.drive_to_line()
        self.assertTrue(all(t >= 0.2 for t, _, _ in gpio.pwm_writes))
        self.assert_off(driver, gpio)

    def test_all_timed_modes_complete_with_unpowered_direction_changes(self):
        for mode in ("SR", "SL", "R", "L", "DR", "DL", "TR", "TL"):
            with self.subTest(mode=mode):
                controller, driver, gpio, _, _ = self.make()
                controller.run_mode(mode)
                self.assert_off(driver, gpio)
                self.assert_unpowered_direction_writes(gpio)

    def test_stuck_side_sensor_stops_instead_of_retriggering(self):
        for readings in ((0, 1, 1), (1, 0, 1)):
            with self.subTest(readings=readings):
                controller, driver, gpio, clock, messages = self.make()
                gpio.readings = lambda now: readings
                with self.assertRaises(motor.LaneFault):
                    controller.run_mode("FA")
                self.assertLessEqual(clock.now, 0.86)
                self.assertEqual(sum(text.startswith("Correcting") for _, text in messages), 1)
                self.assert_off(driver, gpio)

    def test_brief_pulse_has_short_correction_and_correct_wheel_pair(self):
        # LEFT sensor leans right (FR+BL slowed); RIGHT sensor leans left (FL+BR).
        for sensor, slowed in ((0, (1, 2)), (1, (0, 3))):
            with self.subTest(sensor=sensor):
                controller, driver, gpio, _, messages = self.make()
                driver.set_pattern(motor.FORWARD)
                driver.set_duties(motor.straight_targets())
                gpio.pwm_writes.clear()

                def readings(now):
                    values = [1, 1, 0 if now >= 0.7 else 1]
                    if 0.1 <= now < 0.2:
                        values[sensor] = 0
                    return tuple(values)

                gpio.readings = readings
                controller.drive_to_line()
                start = next(t for t, text in messages if text.startswith("Correcting"))
                end = next(t for t, text in messages if text == "Settling straight")
                self.assertGreaterEqual(end - start, 0.15)
                self.assertLessEqual(end - start, 0.20)
                changed_pins = {pin for t, pin, duty in gpio.pwm_writes if t < 0.3}
                self.assertEqual(changed_pins, {motor.MOTOR_PINS[i][2] for i in slowed})

    def test_sensor_flicker_shorter_than_debounce_does_not_correct(self):
        controller, _, gpio, _, messages = self.make()
        gpio.readings = lambda now: (0 if 0.1 <= now < 0.12 else 1,
                                    1, 0 if now >= 0.4 else 1)
        controller.drive_to_line()
        self.assertFalse(any(text.startswith("Correcting") for _, text in messages))

    def test_opposite_sensor_ends_correction_early(self):
        controller, _, gpio, _, messages = self.make()
        gpio.readings = lambda now: (0, 0 if now >= 0.2 else 1, 0 if now >= 0.5 else 1)
        controller.drive_to_line()
        end = next(t for t, text in messages if text == "Settling straight")
        self.assertAlmostEqual(end, 0.24)

    def test_front_line_overrides_active_correction(self):
        controller, driver, gpio, clock, _ = self.make()
        gpio.readings = lambda now: (0, 1, 0 if now >= 0.1 else 1)
        controller.drive_to_line()
        self.assertEqual(clock.now, 0.12)  # second white read in a row
        self.assert_off(driver, gpio)

    def test_pause_resume_and_next_line(self):
        controller, driver, gpio, _, messages = self.make([(0.3, "g"), (0.75, "s")])
        gpio.readings = lambda now: (1, 1, 0 if 0.2 <= now < 0.4 or now >= 0.6 else 1)
        with self.assertRaises(motor.StopRequested):
            controller.run_mode("FA")
        stops = [t for t, text in messages if text.startswith("Front line:")]
        self.assertEqual(stops, [0.22, 0.62])  # each on the second white read in a row
        self.assertTrue(all(duty == 0 for t, _, duty in gpio.pwm_writes if 0.22 <= t < 0.3))
        self.assert_off(driver, gpio)

    def test_stop_takes_precedence_over_resume(self):
        controller, driver, gpio, _, _ = self.make([(0.25, "gs")])
        gpio.readings = lambda now: (1, 1, 0 if now >= 0.2 else 1)
        with self.assertRaises(motor.StopRequested):
            controller.run_mode("F")
        self.assertTrue(all(duty == 0 for t, _, duty in gpio.pwm_writes if t >= 0.22))
        self.assert_off(driver, gpio)

    def test_straight_course_does_not_drop_speed_between_segments(self):
        controller, driver, gpio, clock, messages = self.make()
        gpio.readings = lambda now: (1, 1, 0 if 0.2 <= now < 0.35 or 4.5 <= now < 4.6 else 1)
        controller.run_mode("C", " f f ")
        self.assertAlmostEqual(clock.now, 10.64)  # each line one tick later: second white read
        self.assertFalse(any(4.3 <= t < 4.5 and duty < 25 for t, _, duty in gpio.pwm_writes))
        self.assertTrue(any(text == "Course complete" for _, text in messages))
        self.assert_off(driver, gpio)

    def test_course_right_and_left_turns(self):
        controller, driver, gpio, _, messages = self.make()
        gpio.readings = lambda now: (1, 1, 0 if 0.2 <= now < 0.35 or 6.2 <= now < 6.4 else 1)
        controller.run_mode("C", "RL")
        self.assertTrue(any(text == "Course complete" for _, text in messages))
        self.assert_unpowered_direction_writes(gpio)
        self.assert_off(driver, gpio)

    def test_stop_during_course_intersection_wait(self):
        controller, driver, gpio, clock, _ = self.make([(0.5, "s")])
        gpio.readings = lambda now: (1, 1, 0 if now >= 0.2 else 1)
        with self.assertRaises(motor.StopRequested):
            controller.run_mode("C", "F")
        self.assertEqual(clock.now, 0.5)
        self.assert_off(driver, gpio)

    def test_stop_during_zero_output_direction_pause(self):
        controller, driver, gpio, clock, _ = self.make([(1.27, "s")])
        with self.assertRaises(motor.StopRequested):
            controller.run_mode("TR")
        self.assertLessEqual(clock.now, 1.29)
        self.assert_off(driver, gpio)
        self.assert_unpowered_direction_writes(gpio)

    def test_ir_test_never_energizes_wheels(self):
        controller, driver, gpio, _, messages = self.make([(0.1, "s")])
        with self.assertRaises(motor.StopRequested):
            controller.run_mode("IR")
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assertEqual(len(messages), 1)
        self.assert_off(driver, gpio)

    def test_sensor_exception_stops_motors(self):
        controller, driver, gpio, _, _ = self.make()

        def bad_read(now):
            if now >= 0.1:
                raise RuntimeError("Injected sensor failure")
            return (1, 1, 1)

        gpio.readings = bad_read
        with self.assertRaisesRegex(RuntimeError, "sensor failure"):
            controller.run_mode("FA")
        self.assert_off(driver, gpio)

    def test_invalid_course_never_moves(self):
        for sequence in ("", "FX", "12"):
            with self.subTest(sequence=sequence):
                controller, driver, gpio, _, _ = self.make()
                with self.assertRaises(ValueError):
                    controller.run_mode("C", sequence)
                self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
                self.assert_off(driver, gpio)

    def test_invalid_pwm_rejected_before_any_writes(self):
        _, driver, gpio, _, _ = self.make()
        for targets in ((-1, 0, 0, 0), (101, 0, 0, 0), (float("nan"), 0, 0, 0), (0, 0)):
            with self.subTest(targets=targets):
                count = len(gpio.pwm_writes)
                with self.assertRaises(ValueError):
                    driver.set_duties(targets)
                self.assertEqual(len(gpio.pwm_writes), count)

    def test_partial_initialization_is_cleaned_up(self):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        gpio.fail_setup_pin = motor.MOTOR_PINS[2][0]
        driver = motor.MotorDriver(gpio)
        with self.assertRaisesRegex(RuntimeError, "initialization failure"):
            driver.initialize()
        driver.close()
        self.assertTrue(gpio.cleaned)
        self.assertTrue(all(pwm.stopped for pwm in gpio.pwms))
        self.assertEqual(gpio.values[motor.STBY], 0)

    def test_delayed_loop_does_not_cause_large_duty_jump(self):
        _, driver, _, _, _ = self.make()
        driver.set_pattern(motor.FORWARD)
        driver.slew(motor.straight_targets(), motor.RAMP_RATE, 10.0)
        for duty in driver.duties:
            self.assertAlmostEqual(duty, 0.7)

    def test_optional_trim_and_default_diagonals(self):
        # Right slows FR+BL, the pair STRAFE_RIGHT reverses (SR went left before).
        self.assertEqual(motor.diagonal_targets("right", 14), (25, 14, 14, 25))
        self.assertEqual(motor.diagonal_targets("left", 14), (14, 25, 25, 14))
        # Trim stays side-based: +4 slows the right wheels (yaw toward a right slide).
        self.assertEqual(motor.diagonal_targets("right", 14, 4), (25, 10, 14, 21))
        self.assertEqual(motor.diagonal_targets("right", 14, -4), (21, 14, 10, 25))

    def test_each_lean_slows_the_pair_its_strafe_reverses(self):
        # A lean is forward plus a strafe. If these disagree, FA/FC lean toward
        # the line they just saw and time out (seen on the 500 RPM bot, Sept 24).
        for direction, strafe in (("right", motor.STRAFE_RIGHT), ("left", motor.STRAFE_LEFT)):
            with self.subTest(direction=direction):
                targets = motor.diagonal_targets(direction, 3.5)
                slowed = {index for index, duty in enumerate(targets) if duty == 3.5}
                reversed_by_strafe = {index for index, forward in enumerate(strafe) if not forward}
                self.assertEqual(slowed, reversed_by_strafe)

    def test_pin_mapping_and_patterns_match_documented_rev_c_map(self):
        # FL M1A (J2), FR M1B (J4), BL M2B (J12), BR M2A (J5); IR left J7, right J10, front J9.
        self.assertEqual(motor.MOTOR_PINS, ((17, 27, 18), (22, 23, 25), (16, 19, 13), (5, 6, 12)))
        self.assertEqual(motor.IR_PINS, (21, 7, 26))
        # Checked on the mat: (False, True, True, False) moved the 500 RPM bot left.
        self.assertEqual(motor.STRAFE_RIGHT, (True, False, False, True))
        self.assertEqual(motor.STRAFE_LEFT, (False, True, True, False))
        self.assertEqual(motor.TURN_RIGHT, (True, False, True, False))

    def test_menu_reads_only_through_newline_preserving_stop_typeahead(self):
        data = iter((b"F", b"\n", b"s"))
        with patch.object(motor.os, "read", side_effect=lambda *args: next(data)):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(motor.prompt_line("Choice: "), "F")
            self.assertEqual(next(data), b"s")

    def test_main_quit_closes_hardware(self):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        rpi = types.ModuleType("RPi")
        rpi.GPIO = gpio
        with patch.dict(sys.modules, {"RPi": rpi, "RPi.GPIO": gpio}):
            with patch.object(motor.sys.stdin, "isatty", return_value=True):
                with patch.object(motor, "prompt_line", return_value="Q"):
                    with contextlib.redirect_stdout(io.StringIO()):
                        self.assertEqual(motor.main(), 0)
        self.assertTrue(gpio.cleaned)
        self.assertTrue(all(pwm.stopped for pwm in gpio.pwms))


if __name__ == "__main__":
    unittest.main(verbosity=2)

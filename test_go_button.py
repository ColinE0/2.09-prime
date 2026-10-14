"""GO: the START button runs the armed drive command, with the PRD countdown and run timer.

The real MotorDriver and Controller run against recording GPIO with scripted button presses.
AppGoTests drive GO through the real app.execute (power guard, countdown placement, timer stop
and the ending of each run) with a fake command that runs on the test clock.

Run: python -m unittest -v test_go_button
"""

import contextlib
from copy import deepcopy
import io
from pathlib import Path
import re
import tempfile
import time
import types
import unittest
from unittest.mock import Mock, patch

import app
import go_button as go
import imu_support as imu
import motors_optimized as motor
import robot_settings as settings
from battery_support import BatteryFault
from fake_power import FakeGuard
from ir_confidence import ConfidenceTests
from power_guard import PowerFault
from terminal_ui import LiveStatus, TerminalMenu
from test_app import AppHarness
from test_board_ui import ButtonGPIO
from test_motors_optimized import FakeClock, FakeKeys
from test_v9_4 import Rig

# These tests run with both switches wired (bot 1's pins); bot 2's START-only wiring is in test_v9_9.
START, RESET = 10, 9
BOTH_BUTTONS = dict(button_start="gpio10", button_reset="gpio9")


def wired():
    return dict(deepcopy(settings.DEFAULTS), **BOTH_BUTTONS)


def setUpModule():
    settings.apply_settings(wired())


def tearDownModule():
    settings.apply_settings(deepcopy(settings.DEFAULTS))


class LongClock(FakeClock):
    """FakeClock without the 30 s cap: an armed session spans several countdowns."""

    def sleep(self, duration):
        if duration <= 0:
            raise AssertionError("A loop attempted a nonpositive sleep")
        self.now = round(self.now + duration, 10)
        if self.now > 200:
            raise AssertionError("The armed session did not end within 200 simulated seconds")


class Recorder:
    """Lines written by GO, in order."""

    def __init__(self):
        self.lines = []

    def __call__(self, text=""):
        self.lines.append(str(text))

    def text(self):
        return "\n".join(self.lines)


def drive(control, seconds):
    """Drive forward at a fixed duty for exactly ``seconds`` (no ramp), then stop."""
    control.motors.set_pattern(motor.FORWARD)
    control.motors.set_duties((20.0,) * 4)
    control.wait(seconds)
    control.motors.stop()


class SessionRig:
    """An armed GO session on a real MotorDriver, with a fake of app.execute around a scripted run."""

    def __init__(self, presses=(), keys=(), run=None, preflight=None, ending=None):
        self.clock = LongClock()
        self.gpio = ButtonGPIO(self.clock, presses)
        self.driver = motor.MotorDriver(self.gpio)
        self.driver.initialize()
        self.keys = FakeKeys(self.clock, keys)
        self.write = Recorder()
        self.run = run or (lambda control: drive(control, 2.0))
        self.preflight = preflight
        self.ending = ending  # force an app ending such as "power" after the run
        self.calls, self.lives = [], []

    def execute(self, command, sequence, live, keys, before=None, after=None):
        """Mimic app.execute: before() once the guard is ready, after() the moment the run ends."""
        self.calls.append((command, sequence, self.clock.now))
        try:
            before(keys)
            try:
                self.run(motor.Controller(self.driver, keys, clock=self.clock, log=live))
            finally:
                after()
            if self.ending == "power":
                return "Stopped: Power guard: low pack", "power"
            return f"{command} finished.", "finished"
        except motor.StopRequested:
            return f"{command}: stopped.", "stopped"
        except motor.LaneFault as exc:
            return "Stopped: " + str(exc), "fault"
        finally:
            self.driver.stop()
            live.finish()

    def live_factory(self, status):
        live = LiveStatus(stream=io.StringIO(), compact=False, status=status)
        self.lives.append(live)
        return live

    def session(self, command="TR", sequence=""):
        self.result = go.armed_session(
            command, sequence, self.driver, self.execute, self.live_factory,
            preflight=self.preflight, keyboard=lambda: contextlib.nullcontext(self.keys),
            clock=self.clock, write=self.write)
        return self.result

    def led(self):
        return self.gpio.led_writes()

    def first_drive(self):
        return next((now for now, _, duty in self.gpio.pwm_writes if duty), None)

    def live_text(self):
        return "".join(live.stream.getvalue() for live in self.lives)


class ResolveTests(unittest.TestCase):
    def test_bare_go_arms_the_last_drive_command(self):
        self.assertEqual(go.resolve("", ("FG", "")), ("FG", ""))
        self.assertEqual(go.resolve("  ", ("CG", "FRL")), ("CG", "FRL"))

    def test_a_typed_command_is_armed_without_running_it(self):
        self.assertEqual(go.resolve("cg f r l", ("FG", "")), ("CG", "FRL"))
        self.assertEqual(go.resolve("FC", None), ("FC", ""))

    def test_a_course_without_its_letters_is_asked_for(self):
        self.assertEqual(go.resolve("CG", None, read=lambda prompt: "lrf"), ("CG", "LRF"))
        with self.assertRaisesRegex(ValueError, "cancelled"):
            go.resolve("C", None, read=lambda prompt: "")

    def test_readouts_settings_and_wheel_tests_cannot_be_armed(self):
        for command in ("IR", "IRC", "IMU", "IC", "IV", "BAT", "BTN", "SET", "BOT", "FL", "GO", "PCAL"):
            with self.subTest(command=command), self.assertRaisesRegex(ValueError, "cannot be armed"):
                go.resolve(command, None)

    def test_nothing_to_arm_and_bad_courses_are_refused(self):
        with self.assertRaisesRegex(ValueError, "nothing to arm"):
            go.resolve("", None)
        with self.assertRaisesRegex(ValueError, "only F, R, and L"):
            go.resolve("CG FRX", None)
        with self.assertRaisesRegex(ValueError, "takes no course"):
            go.resolve("FG FRL", None)

    def test_every_drive_command_and_nothing_else_can_be_armed(self):
        self.assertEqual(go.GO_COMMANDS, {"F", "FA", "SR", "SL", "R", "L", "TR", "TL", "C", "DR", "DL",
                                          "FC", "FG", "CG", "IF", "ITL", "ITR"})
        self.assertTrue(go.GO_COMMANDS <= app.POWERED_COMMANDS)
        self.assertTrue(go.IMU_COMMANDS <= go.GO_COMMANDS)


class TimerFormatTests(unittest.TestCase):
    def test_tenths_truncate_like_a_stopwatch(self):
        self.assertEqual(go.tenths(12.39), "12.3")
        self.assertEqual(go.tenths(0.0), "0.0")
        self.assertEqual(go.tenths(2.9999999999), "3.0")  # float noise, not a real 2.9
        self.assertEqual(go.tenths(59.96), "59.9")
        self.assertEqual(go.tenths(123.45), "123.4")

    def test_run_timer_counts_from_start_and_freezes_at_stop(self):
        clock = FakeClock()
        timer = go.RunTimer(clock)
        self.assertIsNone(timer.elapsed())
        self.assertEqual(timer.status(), "")
        timer.start()
        clock.sleep(1.25)
        self.assertEqual(timer.status(), "RUN 1.2 s")
        timer.stop()
        clock.sleep(5.0)
        timer.stop()  # a second stop keeps the first time
        self.assertAlmostEqual(timer.elapsed(), 1.25)

    def test_big_digits_are_five_even_rows(self):
        rows = go.big_text("10.5")
        self.assertEqual(len(rows), 5)
        self.assertTrue(rows[4].startswith("    ##"))  # the 1 is a right-hand stroke
        self.assertEqual(rows[4][16:18], "##")  # the decimal point sits on the bottom row only
        self.assertNotIn("#", "".join(row[16:18] for row in rows[:4]))
        self.assertEqual(len(go.big_text("123.4")), 5)


class SwitchTests(unittest.TestCase):
    def test_start_press_counts_only_after_it_holds_40_ms(self):
        switch = go.Switch()
        self.assertFalse(switch.update(False, 0.0))
        self.assertFalse(switch.update(True, 1.00))
        self.assertFalse(switch.update(True, 1.02))
        self.assertTrue(switch.update(True, 1.04))
        self.assertFalse(switch.update(True, 1.06))  # held, not pressed again

    def test_a_blip_and_a_switch_held_at_arming_are_not_presses(self):
        switch = go.Switch()
        switch.update(False, 0.0)
        self.assertFalse(switch.update(True, 0.02))
        self.assertFalse(switch.update(False, 0.04))
        self.assertFalse(switch.update(True, 0.06))
        held = go.Switch()
        self.assertFalse(held.update(True, 0.0))
        self.assertFalse(held.update(True, 1.0))
        self.assertTrue(held.down)

    def test_reset_counts_on_its_second_low_read_but_not_one(self):
        reset = go.Switch(hold=0.0)
        reset.update(False, 0.0)
        self.assertFalse(reset.update(True, 1.00))
        self.assertFalse(reset.update(False, 1.02))  # a single LOW read is noise
        self.assertFalse(reset.update(True, 1.04))
        self.assertTrue(reset.update(True, 1.06))


class ArmedSessionTests(unittest.TestCase):
    def setUp(self):
        settings.apply_settings(wired())
        self.addCleanup(settings.apply_settings, wired())

    def test_start_counts_down_five_seconds_then_runs_and_times_the_run(self):
        rig = SessionRig(presses=[(START, 1.0, 1.2)], keys=[(15.0, "s")],
                         run=lambda control: control.timed_move(motor.FORWARD, motor.straight_targets(), 2.0))
        result = rig.session("CG", "FRL")
        self.assertEqual([call[:2] for call in rig.calls], [("CG", "FRL")])
        pressed = rig.calls[0][2]
        self.assertAlmostEqual(pressed, 1.04)  # debounced press
        self.assertGreaterEqual(rig.first_drive(), pressed + 5.0 - 1e-9, "Moved before the 5 s countdown ended")
        self.assertEqual(re.findall(r"Starting in (\d)", rig.live_text()), ["5", "4", "3", "2", "1"])
        elapsed = re.search(r"Run 1: ([\d.]+) s, completed \(CG FRL\)", rig.write.text())
        self.assertIsNotNone(elapsed, rig.write.text())
        # A ramp plus a 2 s hold, timed from the end of the countdown.
        self.assertGreater(float(elapsed.group(1)), 2.0)
        self.assertLess(float(elapsed.group(1)), 3.0)
        self.assertIn(go.big_text(elapsed.group(1))[0], rig.write.text())
        self.assertEqual(result, f"GO CG FRL: 1 run, best {elapsed.group(1)} s.")
        self.assertEqual(rig.driver.duties, [0.0] * 4)

    def test_it_stays_armed_and_keeps_the_best_time(self):
        rig = SessionRig(presses=[(START, 1.0, 1.2), (START, 12.0, 12.2)], keys=[(25.0, "s")],
                         run=lambda control: drive(control, 3.0))
        result = rig.session()
        self.assertEqual(len(rig.calls), 2)
        self.assertEqual(rig.write.text().count("ARMED: TR"), 3)
        self.assertIn("Run 1: 3.0 s, completed (TR)", rig.write.text())
        self.assertIn("Run 2: 3.0 s, completed (TR)", rig.write.text())
        self.assertEqual(result, "GO TR: 2 runs, best 3.0 s.")

    def test_reset_stops_a_run_records_dnf_and_stays_armed(self):
        rig = SessionRig(presses=[(START, 1.0, 1.2), (RESET, 7.0, 7.3), (START, 12.0, 12.2)],
                         keys=[(25.0, "s")], run=lambda control: drive(control, 3.0))
        result = rig.session()
        text = rig.write.text()
        self.assertIn("Run 1: DNF, stopped by RESET at 0.9 s (TR)", text)  # second LOW read at 7.02
        self.assertIn("Run 2: 3.0 s, completed (TR)", text)
        self.assertEqual(result, "GO TR: 2 runs, best 3.0 s.")

    def test_a_short_reset_tap_stops_the_run_at_every_phase(self):
        for k in range(10):  # a 45 ms tap starting anywhere across one 20 ms tick
            with self.subTest(offset_ms=2 * k):
                begin = 7.0 + 0.002 * k
                rig = SessionRig(presses=[(START, 1.0, 1.2), (RESET, begin, begin + 0.045)],
                                 keys=[(12.0, "s")], run=lambda control: drive(control, 3.0))
                rig.session()
                self.assertIn("DNF, stopped by RESET", rig.write.text())

    def test_reset_during_the_countdown_cancels_before_anything_moves(self):
        rig = SessionRig(presses=[(START, 1.0, 1.2), (RESET, 3.0, 3.2)], keys=[(10.0, "s")])
        result = rig.session()
        self.assertIn("Countdown cancelled; the robot did not move.", rig.write.text())
        self.assertIsNone(rig.first_drive())
        self.assertEqual(rig.write.text().count("ARMED: TR"), 2)  # still armed after the cancel
        self.assertEqual(result, "GO TR: no runs.")

    def test_reset_held_while_start_is_pressed_cancels_the_countdown(self):
        rig = SessionRig(presses=[(RESET, 0.5, 9.0), (START, 1.0, 1.2)], keys=[(12.0, "s")])
        rig.session()
        self.assertEqual(len(rig.calls), 1)
        self.assertIn("Countdown cancelled", rig.write.text())
        self.assertIsNone(rig.first_drive())

    def test_s_during_a_run_stops_it_and_leaves_go(self):
        rig = SessionRig(presses=[(START, 1.0, 1.2), (START, 12.0, 12.2)], keys=[(7.0, "s")],
                         run=lambda control: drive(control, 3.0))
        result = rig.session()
        self.assertEqual(len(rig.calls), 1)
        self.assertIn("Run 1: DNF, stopped with 's' at 0.9 s (TR)", rig.write.text())
        self.assertEqual(result, "GO TR: 1 run, none completed.")

    def test_s_after_an_earlier_reset_still_leaves_go(self):
        rig = SessionRig(presses=[(START, 1.0, 1.2), (RESET, 7.0, 7.3), (START, 12.0, 12.2),
                                  (START, 25.0, 25.2)],
                         keys=[(18.0, "s")], run=lambda control: drive(control, 3.0))
        rig.session()
        self.assertEqual(len(rig.calls), 2)
        self.assertIn("Run 2: DNF, stopped with 's'", rig.write.text())

    def test_a_start_held_at_arming_must_be_released_before_it_counts(self):
        rig = SessionRig(presses=[(START, 0.0, 2.0), (START, 3.0, 3.2)], keys=[(12.0, "s")],
                         run=lambda control: drive(control, 1.0))
        rig.session()
        self.assertEqual(len(rig.calls), 1)
        self.assertAlmostEqual(rig.calls[0][2], 3.04)

    def test_a_bounce_shorter_than_the_debounce_does_nothing(self):
        rig = SessionRig(presses=[(START, 1.0, 1.02)], keys=[(3.0, "s")])
        self.assertEqual(rig.session(), "GO TR: no runs.")
        self.assertEqual(rig.calls, [])

    def test_a_preflight_problem_keeps_it_armed_without_a_countdown(self):
        rig = SessionRig(presses=[(START, 1.0, 1.2)], keys=[(3.0, "s")],
                         preflight=lambda command: "IMU Needs IC.")
        rig.session("FG")
        self.assertIn("Not started: IMU Needs IC.", rig.write.text())
        self.assertEqual(rig.calls, [])
        self.assertEqual(rig.write.text().count("ARMED: FG"), 2)

    def test_a_power_stop_disarms(self):
        rig = SessionRig(presses=[(START, 1.0, 1.2), (START, 12.0, 12.2)], keys=[(25.0, "s")],
                         run=lambda control: drive(control, 1.0), ending="power")
        result = rig.session()
        self.assertEqual(len(rig.calls), 1)
        self.assertIn("GO disarmed", rig.write.text())
        self.assertIn("Run 1: DNF, power stop at 1.0 s (TR)", rig.write.text())
        self.assertEqual(result, "GO TR: 1 run, none completed.")

    def test_a_run_that_ends_before_the_wheels_drive_is_not_counted(self):
        rig = SessionRig(presses=[(START, 1.0, 1.2), (START, 9.0, 9.2)], keys=[(17.0, "s")],
                         run=lambda control: control.wait(0.5))
        result = rig.session()
        self.assertEqual(len(rig.calls), 2)
        self.assertEqual(rig.write.text().count("Not counted: the run ended before the wheels drove"), 2)
        self.assertNotIn("Run 1", rig.write.text())
        self.assertEqual(result, "GO TR: no runs.")

    def test_led_blips_while_armed_flashes_the_count_and_is_solid_for_the_run(self):
        rig = SessionRig(presses=[(START, 2.0, 2.2)], keys=[(12.0, "s")],
                         run=lambda control: drive(control, 2.0))
        rig.session()
        writes = rig.led()
        self.assertEqual(writes[:4], [(0.0, 1), (0.16, 0), (1.0, 1), (1.16, 0)])  # armed blips
        countdown = [(now, value) for now, value in writes if 2.04 <= now < 7.04]
        self.assertEqual([value for _, value in countdown], [1, 0] * 5)  # one flash per second
        self.assertEqual(writes[[now for now, _ in writes].index(countdown[-1][0]) + 1], (7.04, 1))
        self.assertFalse([now for now, value in writes if 7.04 < now < 9.04], "The LED changed during the run")
        self.assertEqual(writes[-1][1], 0)
        self.assertEqual(rig.gpio.values[motor.STATUS_LED], 0)

    def test_an_unclaimable_button_pin_is_reported_not_fatal(self):
        rig = SessionRig()
        rig.gpio.fail_setup_pin = START
        result = rig.session()
        self.assertIn("could not claim the START/RESET/LED pins", result)
        self.assertEqual(rig.calls, [])

    def test_ctrl_c_while_armed_or_counting_down_ends_with_led_and_motors_off(self):
        for moment in (1.06, 3.0):  # armed (LED blip lit), then mid-countdown
            with self.subTest(at=moment):
                rig = SessionRig(presses=[(START, 2.0, 2.2)])

                def poll(clock=rig.clock, at=moment):
                    if clock.now >= at:
                        raise KeyboardInterrupt
                    return ""

                rig.keys = types.SimpleNamespace(poll=poll)
                with self.assertRaises(KeyboardInterrupt):
                    rig.session()
                self.assertEqual(rig.gpio.values[motor.STATUS_LED], 0)
                self.assertIsNone(rig.first_drive())


class FCRunTests(unittest.TestCase):
    """FC starts its sensors first: the countdown waits for them and the clock starts with the wheels."""

    def setUp(self):
        settings.apply_settings(wired())
        self.addCleanup(settings.apply_settings, wired())

    def test_fc_counts_down_after_its_sensors_start_and_times_only_the_drive(self):
        def fc(control):
            control.wait(2.0)  # camera and ToF starting, robot still
            control.go_ready()
            drive(control, 1.5)
            control.go_end()  # the stop sign
            control.wait(0.5)  # sensors closing after the robot stopped

        rig = SessionRig(presses=[(START, 1.0, 1.2)], keys=[(15.0, "s")], run=fc)
        rig.session("FC")
        self.assertIn("Run 1: 1.5 s, completed (FC)", rig.write.text())
        self.assertGreaterEqual(rig.first_drive(), 1.04 + 2.0 + 5.0 - 1e-9)

    def test_fc_driving_before_its_countdown_is_stopped(self):
        rig = SessionRig(presses=[(START, 1.0, 1.2)], keys=[(6.0, "s")],
                         run=lambda control: drive(control, 1.0))  # never calls go_ready
        rig.session("FC")
        self.assertIn("a wheel was driven before the countdown", rig.write.text())
        self.assertIn("Not started", rig.write.text())
        self.assertEqual(rig.driver.duties, [0.0] * 4)

    def test_the_real_fc_time_limit_is_a_dnf_not_a_completed_run(self):
        rig = SessionRig(presses=[(START, 1.0, 1.2)], keys=[(25.0, "s")])
        rig.run = lambda control: ConfidenceTests(rig.driver, control.keys, rig.clock, control.log).run_test("FC")
        result = rig.session("FC")
        self.assertIn("Run 1: DNF, FC time limit at 10.0 s (FC)", rig.write.text())
        self.assertNotIn("Best completed", rig.write.text())
        self.assertEqual(result, "GO FC: 1 run, none completed.")

    def test_the_real_fc_at_a_front_line_ends_completed_and_on_one_at_the_start_is_not_counted(self):
        rig = SessionRig(presses=[(START, 1.0, 1.2), (START, 20.0, 20.2)], keys=[(35.0, "s")])
        rig.gpio.readings = lambda now: (1, 1, 0 if 9.0 <= now < 9.2 or now >= 20.0 else 1)
        rig.run = lambda control: ConfidenceTests(rig.driver, control.keys, rig.clock, control.log).run_test("FC")
        rig.session("FC")
        text = rig.write.text()
        self.assertRegex(text, r"Run 1: [\d.]+ s, completed \(FC\)")
        self.assertIn("Not counted: the run ended before the wheels drove", text)


class TTY(io.StringIO):
    def isatty(self):
        return True


class LiveTimerTests(unittest.TestCase):
    def make(self, compact=True, stream=None):
        clock = FakeClock()
        timer = go.RunTimer(clock)
        stream = TTY() if stream is None else stream
        return clock, timer, LiveStatus(stream=stream, compact=compact, status=timer.status), stream

    def test_no_heading_until_the_timer_starts(self):
        clock, timer, live, stream = self.make()
        live("Starting in 5")
        live.refresh(clock.now)
        self.assertEqual(stream.getvalue(), "Starting in 5\n")

    def test_the_timer_heads_the_live_line_and_an_event_erases_then_redraws_it(self):
        clock, timer, live, stream = self.make()
        timer.start()
        clock.sleep(1.0)
        live("Heading +1.0 deg")
        self.assertEqual(stream.getvalue(), "\rRUN 1.0 s | Heading +1.0 deg")
        live("Correcting right")
        bar = "RUN 1.0 s | Heading +1.0 deg"
        self.assertEqual(stream.getvalue(),
                         "\r" + bar + "\r" + " " * len(bar) + "\r"  # erased, not left in the scrollback
                         + "RUN 1.0 s | Correcting right\n"  # the event carries the time
                         + "\r" + bar)  # redrawn at once, before any refresh
        clock.sleep(0.5)
        live.refresh(clock.now)
        self.assertTrue(stream.getvalue().endswith("\rRUN 1.5 s | Heading +1.0 deg"))

    def test_refresh_is_limited_to_ten_a_second_in_place(self):
        clock, timer, live, stream = self.make()
        timer.start()
        draws = 0
        for _ in range(50):  # one second of 20 ms polls
            before = stream.getvalue()
            live.refresh(clock.now)
            draws += stream.getvalue() != before
            clock.sleep(motor.LOOP_DT)
        self.assertEqual(draws, 10)

    def test_plain_output_prints_one_timer_line_a_second(self):
        clock, timer, live, stream = self.make(compact=False)
        timer.start()
        for _ in range(150):
            live.refresh(clock.now)
            clock.sleep(motor.LOOP_DT)
        self.assertEqual(stream.getvalue().splitlines(), ["RUN 0.0 s", "RUN 1.0 s", "RUN 2.0 s"])

    def test_a_camera_line_fits_beside_the_timer(self):
        clock, timer, live, stream = self.make()
        timer.start()
        live("Camera: stop_sign center=(320, 240) (4.0% frame) | confirmed: stop_sign | "
             "line: end | frame age 0.05s | processing 0.04s")
        line = stream.getvalue().split("\r")[-1]
        self.assertTrue(line.startswith("RUN 0.0 s | Camera: stop@(320,240)/4.0%"), line)
        self.assertTrue(line.rstrip().endswith("line end | age 0.05s | proc 0.04s"), line)
        self.assertLessEqual(len(line), 79)

    def test_without_a_status_the_output_is_unchanged(self):
        stream = TTY()
        live = LiveStatus(stream=stream)
        live("Heading +1.0 deg")
        live("Correcting right")
        live.refresh(5.0)
        self.assertEqual(stream.getvalue(), "\rHeading +1.0 deg\nCorrecting right\n")


class StubMotors:
    """ButtonKeys' view of a driver with no buttons pressed: the duties are the real driver's."""

    def __init__(self, driver):
        self.driver = driver
        self.duties = driver.duties

    def read_buttons(self):
        return (False, False)

    def stop(self, brake=None):
        self.driver.stop(brake)


def go_keys(rig):
    keys = go.ButtonKeys(rig.control.keys, StubMotors(rig.driver), rig.clock)
    rig.control.keys = keys
    return keys


class FrontLineTests(unittest.TestCase):
    def setUp(self):
        settings.apply_settings(wired())
        self.addCleanup(settings.apply_settings, wired())

    def test_go_runs_of_f_and_fa_end_at_their_first_front_line(self):
        for auto in (False, True):
            with self.subTest(command="FA" if auto else "F"):
                rig = Rig(controller=motor.Controller, lines=(0.5,), margin=10.0)
                keys = go_keys(rig)
                timer = go.RunTimer(rig.clock)
                keys.begin_run(timer, None)
                timer.start()
                rig.control.forward(auto=auto)  # returns instead of waiting for 'g'
                self.assertIn("Front line: stopped. Run complete.", rig.text())
                self.assertNotIn("Press 'g'", rig.text())
                self.assertTrue(keys.moved)
                self.assertIsNotNone(timer.stopped)
                rig.assert_off(self)

    def test_a_go_run_of_fg_ends_at_its_first_front_line_with_a_go_balance_hint(self):
        rig = Rig(lines=(0.4,), margin=10.0)  # a short straight: under the 2 s balance needs
        go_keys(rig)
        rig.control.run("FG")
        self.assertIn("Front line: stopped. Run complete.", rig.text())
        self.assertIn("Heading at the end", rig.text())
        self.assertIn("GO runs do not add up", rig.text())
        self.assertNotIn("press 'g' and run", rig.text())
        rig.assert_off(self)

    def test_menu_runs_still_wait_for_g(self):
        rig = Rig(controller=motor.Controller, lines=(0.5,), margin=10.0, events=[(6.0, "s")])
        with self.assertRaises(motor.StopRequested):
            rig.control.forward(auto=True)
        self.assertIn("Press 'g' to continue", rig.text())

    def test_only_a_real_true_marks_a_go_run(self):
        keys = types.SimpleNamespace(poll=lambda: "", go_run=Mock(), ready=Mock(), end_run=Mock())
        control = motor.Controller(None, keys)
        self.assertFalse(control.go_run())
        control.go_ready()
        control.go_end()
        keys.ready.assert_not_called()
        keys.end_run.assert_not_called()


class SlowGuard(FakeGuard):
    """Takes 1 s to get its first INA219 reading (polling) and 0.5 s to close."""

    clock = None
    trip_at = None
    close_result = True

    def wait_ready(self, poll=None):
        end = self.clock.now + 1.0
        while self.clock.now < end - 1e-9:
            poll()
            self.clock.sleep(motor.LOOP_DT)

    def check(self, motors):
        super().check(motors)
        if self.trip_at is not None and self.clock.now >= self.trip_at:
            self.trip_reason = "low pack"
            raise PowerFault("Power guard: pack below the low-voltage limit")

    def close(self):
        self.clock.sleep(0.5)
        return super().close() and self.close_result


class AppGoTests(unittest.TestCase):
    """GO through the real app.execute, with a fake command that drives on the test clock."""

    def setUp(self):
        settings.apply_settings(wired())
        self.addCleanup(settings.apply_settings, wired())
        self.drive_seconds = 1.0
        self.raise_in_run = None

    def fake_dispatch(self, clock):
        def dispatch(command, sequence, motors, keys, session, battery, vision, tof, log):
            self.dispatched.append((command, sequence, clock.now))
            if not command.startswith(("F", "S", "T", "C", "D", "R", "L", "I")) or command in {"IRC", "IMU", "ID"}:
                return  # readouts: nothing to drive
            control = motor.Controller(motors, keys, clock=clock, log=log)
            if command == "FC":
                control.go_ready()
            drive(control, self.drive_seconds)
            if self.raise_in_run is not None:
                raise self.raise_in_run
            control.go_end()
        return dispatch

    def run_app(self, choices, presses=(), keys=((20.0, "s"),), guard=FakeGuard, calibration=None):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        harness = AppHarness(self, choices)
        harness.gpio = ButtonGPIO(harness.clock, presses)
        harness.sessions["imu"].calibration = calibration
        clock = harness.clock
        self.dispatched = []
        original = go.armed_session
        go_keys = (lambda: keys(clock)) if callable(keys) else (lambda: FakeKeys(clock, list(keys)))

        def session(*args, **kwargs):
            return original(*args, keyboard=lambda: contextlib.nullcontext(go_keys()), clock=clock, **kwargs)

        SlowGuard.clock = clock
        with harness.environment(), patch.object(app, "armed_session", side_effect=session), \
             patch.object(app, "dispatch", side_effect=self.fake_dispatch(clock)), \
             patch.object(app, "PowerGuard", guard):
            path = Path(temp.name) / "robot_settings.json"
            settings.save_settings(path, settings.validate(dict(BOTH_BUTTONS), "500rpm"))
            result = app.main(["--bot", "500", "--settings", str(path)])
        harness.result = result
        return harness

    def test_go_arms_the_last_drive_command_and_runs_it_on_start(self):
        harness = self.run_app(("SR", "GO", "Q"), presses=[(START, 3.0, 3.2)])  # after the 1 s menu run
        self.assertEqual([call[0] for call in self.dispatched], ["SR", "SR"])
        self.assertEqual(harness.states[1]["go_command"], "SR")
        out = harness.stdout.getvalue()
        self.assertIn("ARMED: SR", out)
        self.assertIn("Run 1: 1.0 s, completed (SR)", out)
        self.assertEqual(harness.states[2]["last_result"], "GO SR: 1 run, best 1.0 s.")
        self.assertEqual(harness.gpio.values[motor.STATUS_LED], 0)

    def test_the_countdown_follows_the_guard_wait_and_the_timer_excludes_its_close(self):
        harness = self.run_app(("GO SR", "Q"), presses=[(START, 1.0, 1.2)], guard=SlowGuard)
        self.assertAlmostEqual(self.dispatched[0][2], 1.04 + 1.0 + 5.0)  # guard 1 s, then 5 s
        self.assertIn("Run 1: 1.0 s, completed (SR)", harness.stdout.getvalue())

    def test_reset_through_the_real_execute_is_a_dnf_timed_without_the_guard_close(self):
        harness = self.run_app(("GO SR", "Q"), presses=[(START, 1.0, 1.2), (RESET, 7.54, 7.7)],
                               guard=SlowGuard)
        out = harness.stdout.getvalue()
        self.assertIn("Run 1: DNF, stopped by RESET at 0.5 s (SR)", out)
        self.assertIn("ARMED: SR", out.split("stopped by RESET")[1])  # still armed afterwards

    def test_reset_during_the_guard_wait_cancels_the_run(self):
        harness = self.run_app(("GO SR", "Q"), presses=[(START, 1.0, 1.2), (RESET, 1.5, 1.7)],
                               guard=SlowGuard)
        self.assertIn("Countdown cancelled; the robot did not move.", harness.stdout.getvalue())
        self.assertEqual(self.dispatched, [])

    def test_a_guard_trip_through_the_real_execute_disarms_go(self):
        SlowGuard.trip_at = 7.5
        self.addCleanup(setattr, SlowGuard, "trip_at", None)
        harness = self.run_app(("GO SR", "Q"), presses=[(START, 1.0, 1.2), (START, 12.0, 12.2)],
                               guard=SlowGuard)
        out = harness.stdout.getvalue()
        self.assertIn("GO disarmed", out)
        self.assertEqual(len(self.dispatched), 1)
        self.assertIn("power stop", out)

    def test_a_battery_fault_mid_run_disarms_go(self):
        self.raise_in_run = BatteryFault("INA219 reading stale")
        harness = self.run_app(("GO SR", "Q"), presses=[(START, 1.0, 1.2), (START, 12.0, 12.2)])
        self.assertIn("GO disarmed", harness.stdout.getvalue())
        self.assertEqual(len(self.dispatched), 1)

    def test_a_blocked_ina219_is_refused_at_the_press(self):
        SlowGuard.close_result = False
        self.addCleanup(setattr, SlowGuard, "close_result", True)
        harness = self.run_app(("SR", "GO", "Q"), presses=[(START, 4.0, 4.2)], guard=SlowGuard)
        self.assertIn("Not started: a previous INA219 read is still blocked", harness.stdout.getvalue())
        self.assertEqual(len(self.dispatched), 1)  # only the menu run

    def test_imu_runs_need_a_current_ic_and_iv_at_the_press(self):
        for command in sorted(go.IMU_COMMANDS):
            with self.subTest(command=command):
                choice = "GO CG FRL" if command == "CG" else "GO " + command
                harness = self.run_app((choice, "Q"), presses=[(START, 1.0, 1.2)], keys=((3.0, "s"),))
                self.assertIn("Not started: IMU Needs IC.", harness.stdout.getvalue())
                self.assertEqual(self.dispatched, [])

    def test_an_ic_about_to_expire_is_refused_and_a_fresh_one_shows_its_time_left(self):
        old = imu.Calibration((0, 0, 0), (0, 0, 1), time.monotonic() - 295, axis=2)
        harness = self.run_app(("GO FG", "Q"), presses=[(START, 1.0, 1.2)], keys=((3.0, "s"),),
                               calibration=old)
        self.assertRegex(harness.stdout.getvalue(), r"Not started: the IC expires in \d+ s")
        fresh = imu.Calibration((0, 0, 0), (0, 0, 1), time.monotonic(), axis=2)
        harness = self.run_app(("GO FG", "Q"), keys=((1.0, "s"),), calibration=fresh)
        self.assertRegex(harness.stdout.getvalue(), r"ARMED: FG \(IMU Ready, IC 4:5\d left\)")

    def test_readouts_do_not_replace_the_armed_command_and_a_typed_go_is_remembered(self):
        harness = self.run_app(("SR", "IRC", "GO", "GO SL", "GO", "Q"), keys=((0.0, "s"),))
        out = harness.stdout.getvalue()
        self.assertEqual(re.findall(r"ARMED: (\w+)", out), ["SR", "SL", "SL"])
        self.assertEqual(harness.states[2]["go_command"], "SR")

    def test_go_with_nothing_to_arm_or_a_readout_explains_itself(self):
        harness = self.run_app(("GO", "GO IR", "Q"))
        self.assertIn("nothing to arm yet", harness.states[1]["last_result"])
        self.assertIn("IR cannot be armed", harness.states[2]["last_result"])

    def test_ctrl_c_while_armed_exits_with_motors_led_and_gpio_clean(self):
        def keys(clock):
            def poll():
                if clock.now >= 1.06:  # armed, with the LED blip lit
                    raise KeyboardInterrupt
                return ""
            return types.SimpleNamespace(poll=poll)

        harness = self.run_app(("GO SR",), presses=[(START, 2.0, 2.2)], keys=keys)
        self.assertEqual(harness.result, 0)
        self.assertIn("Exiting", harness.stdout.getvalue())
        self.assertEqual(harness.gpio.values[motor.STATUS_LED], 0)
        self.assertTrue(harness.gpio.cleaned)
        self.assertTrue(all(duty == 0 for _, _, duty in harness.gpio.pwm_writes))

    def test_go_is_listed_with_its_armed_command(self):
        lines = []
        answers = iter(["2", "Q"])
        menu = TerminalMenu(read=lambda prompt: next(answers), write=lines.append)
        self.assertEqual(menu.choose({"go_command": "CG FRL"}), "Q")
        self.assertIn("START button (GO): CG FRL", lines)
        self.assertTrue(any(re.match(r"\s+\d+\. GO\s", line) for line in lines))


if __name__ == "__main__":
    unittest.main()

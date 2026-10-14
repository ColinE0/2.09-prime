#!/usr/bin/env python3
"""Standalone motor controller based on motors2_diagonal.py.

Run on the robot's Raspberry Pi: python3 motors_optimized.py
Commands: F FA SR SL R L TR TL C FRL DR DL IR Q. 's' stops every mode;
'g' resumes F/FA at a front line. Ctrl+C stops and exits.

GPIO mapping and nominal calibration are inherited from motors2_diagonal.py.
Timed maneuver durations are HOLD times after acceleration. Recheck them on
the robot: acceleration and direction transitions have been corrected.
FA/course initially allow departure from a front line, as in the originals.
Sensor correction timeout now stops the mode instead of retrying indefinitely.
With the camera on (VT, from the bench menu), a confirmed green lets F, FA and C
cross a front line without stopping; red or yellow stops at it until green.

Importing this file does not import RPi.GPIO or initialize hardware.
"""

from contextlib import contextmanager
import math
import os
import select
import sys
import time
from motion_profile import PRESET


# Wheel order everywhere: FL, FR, BL, BR. Pins use BCM numbering.
STBY = 24
# FL M1A (J2), FR M1B (J4), BL M2B (J12), BR M2A (J5): the documented rev C map.
MOTOR_PINS = ((17, 27, 18), (22, 23, 25), (16, 19, 13), (5, 6, 12))
IR_PINS = (21, 7, 26)  # left J7, right J10, front J9; 0 means white line
# Board UI (header pins 19, 21, 7). Both switches have 10k pull-ups and short to GND when
# pressed; the LED is GPIO -> 1k -> LED -> GND. GPIO9/10 are SPI0 pins, so SPI stays off.
START_BTN, RESET_BTN, STATUS_LED = 10, 9, 4
REVERSED = (False, False, False, False)
PWM_FREQ = 1000
MAX_MOTOR_DUTY = 70.0  # Project ceiling; does not provide stall/current protection.
# Logical wheel order FL, FR, BL, BR, independent of electrical REVERSED wiring.
# Attenuation only: a saved calibration can never raise an output above its request.
WHEEL_FACTORS_FORWARD = (1.0, 1.0, 1.0, 1.0)
WHEEL_FACTORS_REVERSE = (1.0, 1.0, 1.0, 1.0)

# Baseline calibration from motors2_diagonal.py.
LEFT_SPEED = PRESET["left_speed"]
RIGHT_SPEED = PRESET["right_speed"]
LOOP_DT = 0.02
RAMP_RATE = PRESET["ramp_rate"]
TURN_RAMP_RATE = PRESET["turn_ramp_rate"]
TURN_APPROACH_SPEED_PCT = 100.0
AUTO_SLEW_RATE = PRESET["auto_slew_rate"]
DIRECTION_PAUSE = 0.05  # zero output before changing an active direction pattern
BRAKE_ON_STOP = True  # TB6612FNG short brake; False selects coast/standby
STRAFE_DURATION = 0.6
TURN_DURATION_RIGHT = 0.12
TURN_DURATION_LEFT = 0.14
TURN_APPROACH_TIME_RIGHT = 0.55
TURN_APPROACH_TIME_LEFT = 1.5
TURN_EXIT_STRAIGHT_TIME = 2.0
COURSE_STOP_TIME = 3.0
COURSE_CROSS_TIME = 0.4
COURSE_EXIT_STRAIGHT_TIME = 2.0
DIAGONAL_DURATION = 0.6
DIAGONAL_SECONDARY_SPEED = PRESET["diagonal_secondary_speed"]

AUTO_SECONDARY_SPEED = PRESET["auto_secondary_speed"]
AUTO_DEBOUNCE = 0.04
# Front line: white on two reads in a row (one tick apart), so a one-read glitch is not an
# intersection. Tape 19 mm wide at 0.3 m/s still gives about three reads.
FRONT_DEBOUNCE = LOOP_DT
AUTO_MIN_CORRECT_LEFT = 0.15  # triggered by LEFT sensor: lean right
AUTO_MIN_CORRECT_RIGHT = 0.15
AUTO_OVERSHOOT_LEFT = 0.10
AUTO_OVERSHOOT_RIGHT = 0.10
AUTO_MAX_CORRECT = 0.80
AUTO_SETTLE = 0.30
# Drive-to-line slowdown: after this long, scale duties so the front IR catches the line.
APPROACH_SLOW_AFTER = 0.0  # seconds; 0 (the default) turns slowing off
APPROACH_SPEED_PCT = 60.0  # percent of the normal straight/lean duties

# Optional features from motors3.py, disabled until physically calibrated.
# Positive trim rotates toward the slide; negative rotates away.
AUTO_YAW_TRIM = 0.0
DIAGONAL_YAW_TRIM = 0.0

FORWARD = (True, True, True, True)
# Mecanum strafes and leans, checked on the robot: SR with the older pattern
# (FL/BR reverse) moved the 500 RPM bot LEFT on the mat (2026-09-24), matching the
# Sept 16 bench script whose strafe right (FL/BR forward) was confirmed on the
# chassis. Leans are forward plus a strafe, so the strafe's reversed pair is the
# slowed pair in diagonal_targets(). Change both together, and re-run SR on each bot.
STRAFE_RIGHT = (True, False, False, True)
STRAFE_LEFT = (False, True, True, False)
TURN_RIGHT = (True, False, True, False)
TURN_LEFT = (False, True, False, True)


STOP_CONTROL_KEYS = (b"\x13", b"\x1a", b"\x1c")  # Ctrl+S, Ctrl+Z, Ctrl+\ stop like 's'


class StopRequested(Exception):
    """Return to the menu with motors stopped."""


class LaneFault(Exception):
    """A correction exceeded its permitted duration."""


def straight_targets():
    return (LEFT_SPEED, RIGHT_SPEED, LEFT_SPEED, RIGHT_SPEED)


def diagonal_targets(direction, secondary, trim=0.0):
    # Slow the pair that STRAFE_RIGHT/STRAFE_LEFT reverses: FR+BL to go right,
    # FL+BR to go left. Trim below stays side-based (right wheels slow = yaw right).
    if direction == "right":
        targets = [LEFT_SPEED, secondary, secondary, RIGHT_SPEED]
    elif direction == "left":
        targets = [secondary, RIGHT_SPEED, LEFT_SPEED, secondary]
    else:
        raise ValueError("Direction must be 'left' or 'right'")
    if trim:
        indices = (1, 3) if (direction == "right") == (trim > 0) else (0, 2)
        for index in indices:
            targets[index] = max(0.0, targets[index] - abs(trim))
    return tuple(targets)


class MotorDriver:
    """Own GPIO writes, logical requested duties, and calibrated PWM outputs."""

    def __init__(self, gpio):
        self.gpio = gpio
        self.pwm = []
        self.duties = [0.0] * 4
        self.applied_duties = [0.0] * 4
        self._raw_calibration = False
        self.pattern = None
        self.configured = []
        self.ready = False
        self.power_guard = None  # set by the app around powered commands

    def initialize(self):
        self.validate_wheel_factors()  # Reject a corrupt profile before any GPIO write.
        gpio = self.gpio
        gpio.setmode(gpio.BCM)
        gpio.setup(STBY, gpio.OUT, initial=gpio.LOW)
        self.configured.append(STBY)
        for pins in MOTOR_PINS:
            for pin in pins:
                gpio.setup(pin, gpio.OUT, initial=gpio.LOW)
                self.configured.append(pin)
            pwm = gpio.PWM(pins[2], PWM_FREQ)
            self.pwm.append(pwm)
            pwm.start(0)
        for pin in IR_PINS:
            gpio.setup(pin, gpio.IN)
            self.configured.append(pin)
        self.ready = True
        # Standby remains LOW until a movement pattern has been set.

    @staticmethod
    def validate_targets(targets):
        values = tuple(float(value) for value in targets)
        if len(values) != 4 or any(not math.isfinite(v) or not 0 <= v <= MAX_MOTOR_DUTY
                                   for v in values):
            raise ValueError(f"Four finite duty cycles between 0 and {MAX_MOTOR_DUTY:g} are required")
        return values

    @staticmethod
    def validate_wheel_factors():
        """Validate both directions before output, including raw calibration output."""
        validated = []
        for name, factors in (("forward", WHEEL_FACTORS_FORWARD),
                              ("reverse", WHEEL_FACTORS_REVERSE)):
            try:
                values = tuple(float(value) for value in factors)
                valid = len(values) == 4 and all(math.isfinite(v) and 0.5 <= v <= 1.0
                                                for v in values)
            except (TypeError, ValueError, OverflowError):
                valid = False
            if not valid:
                raise ValueError(f"Four finite {name} wheel factors between 0.5 and 1 are required")
            validated.append(values)
        return tuple(validated)

    @contextmanager
    def raw_calibration(self):
        """Temporarily use uncorrected PWM; always coast-stop before restoring mapping.

        Call only with all wheels stopped. Existing power protection remains attached.
        Entry does not enable the motor bridge or write PWM; exit never resumes motion.
        """
        if not self.ready:
            raise RuntimeError("Motor driver has not been initialized")
        if self._raw_calibration:
            raise RuntimeError("Raw calibration contexts cannot be nested")
        if any(self.duties) or any(self.applied_duties):
            raise RuntimeError("Stop all wheels before raw calibration")
        self.validate_wheel_factors()
        self._raw_calibration = True
        try:
            yield self
        finally:
            try:
                self.stop(brake=False)
            finally:
                self._raw_calibration = False

    def set_duties(self, targets):
        targets = self.validate_targets(targets)
        forward_factors, reverse_factors = self.validate_wheel_factors()
        if not self.ready:
            raise RuntimeError("Motor driver has not been initialized")
        if self.pattern is None and any(targets):
            raise RuntimeError("Set a direction pattern before applying PWM")
        if self.power_guard is not None and any(targets):
            if self.power_guard.trip_reason is not None:
                self.stop(brake=False)
                raise RuntimeError("Power guard has already tripped: " + self.power_guard.trip_reason)
            self.power_guard.check(self)
        # Compute and validate every output before writing any channel. Keep requests
        # separate so repeated slew/control updates cannot multiply a factor twice.
        applied = tuple(target if self._raw_calibration or not target else target *
                        (forward_factors[index] if self.pattern[index] else reverse_factors[index])
                        for index, target in enumerate(targets))
        applied = self.validate_targets(applied)
        for index, target in enumerate(applied):
            if target != self.applied_duties[index]:
                self.pwm[index].ChangeDutyCycle(target)
                self.applied_duties[index] = target
            self.duties[index] = targets[index]

    def slew(self, targets, rate, dt, *, deceleration_rate=None):
        targets = self.validate_targets(targets)
        # Do not make a large catch-up jump after a delayed control iteration.
        bounded_dt = max(0.0, min(dt, LOOP_DT))
        falling_rate = rate if deceleration_rate is None else deceleration_rate
        values = []
        for current, target in zip(self.duties, targets):
            diff = target - current
            step = (rate if diff >= 0 else falling_rate) * bounded_dt
            values.append(target if abs(diff) <= step + 1e-9
                          else current + math.copysign(step, diff))
        self.set_duties(values)
        return tuple(self.duties) == targets

    def slew_turn(self, targets, dt):
        """Independent turn acceleration; keep the established deceleration rate."""
        return self.slew(targets, TURN_RAMP_RATE, dt, deceleration_rate=RAMP_RATE)

    def set_pattern(self, pattern):
        pattern = tuple(pattern)
        if len(pattern) != 4 or any(type(value) is not bool for value in pattern):
            raise ValueError("Four boolean wheel directions are required")
        if not self.ready:
            raise RuntimeError("Motor driver has not been initialized")
        if pattern == self.pattern:
            return  # Keep the current duty when continuing the same movement.
        self.stop(brake=False)  # Disable and zero PWM BEFORE any direction write.
        for pins, forward, reverse in zip(MOTOR_PINS, pattern, REVERSED):
            actual_forward = forward != reverse
            self.gpio.output(pins[0], self.gpio.HIGH if actual_forward else self.gpio.LOW)
            self.gpio.output(pins[1], self.gpio.LOW if actual_forward else self.gpio.HIGH)
        self.pattern = pattern
        self.gpio.output(STBY, self.gpio.HIGH)

    def read_ir(self):
        return tuple(self.gpio.input(pin) for pin in IR_PINS)

    def claim_board_ui(self):
        """Claim START/RESET (pulled up) and the status LED (LOW) on first use.

        They stay claimed until close(): a pin freed and claimed again through the
        same rpi-lgpio handle can misbehave (see close()).
        """
        if not self.ready:
            raise RuntimeError("Motor driver has not been initialized")
        for pin in (START_BTN, RESET_BTN):
            if pin not in self.configured:
                self.gpio.setup(pin, self.gpio.IN, pull_up_down=self.gpio.PUD_UP)
                self.configured.append(pin)
        if STATUS_LED not in self.configured:
            self.gpio.setup(STATUS_LED, self.gpio.OUT, initial=self.gpio.LOW)
            self.configured.append(STATUS_LED)

    def read_buttons(self):
        """(start, reset), True while held: a pressed switch pulls its pin LOW."""
        return tuple(self.gpio.input(pin) == self.gpio.LOW for pin in (START_BTN, RESET_BTN))

    def set_status_led(self, on):
        self.gpio.output(STATUS_LED, self.gpio.HIGH if on else self.gpio.LOW)

    def stop(self, brake=None):
        # Disable both H-bridges first, even if a later PWM operation fails.
        if brake is None:
            brake = BRAKE_ON_STOP
        if self.power_guard is not None and self.power_guard.trip_reason is not None:
            brake = False  # Nested controller cleanup must not undo a power fault's coast.
        if STBY in self.configured:
            self.gpio.output(STBY, self.gpio.LOW)
        for index, pwm in enumerate(self.pwm):
            pwm.ChangeDutyCycle(0)
            self.duties[index] = 0.0
            self.applied_duties[index] = 0.0
        for ain1, ain2, _ in MOTOR_PINS:
            for pin in (ain1, ain2):
                if pin in self.configured:
                    self.gpio.output(pin, self.gpio.HIGH if brake and self.ready else self.gpio.LOW)
        self.pattern = None
        if brake and self.ready:
            # Datasheet: IN1=IN2=HIGH, STBY=HIGH is short brake for either PWM level.
            # See Toshiba TB6612FNG, H-SW Control Function table (page 4).
            self.gpio.output(STBY, self.gpio.HIGH)

    def close(self):
        """Also works after partial initialization; attempt every cleanup step."""
        errors = []
        try:
            self.stop(brake=False)
        except Exception as exc:
            errors.append(str(exc))
        # Release every PWM object while its pin is still claimed; a PWM that outlives
        # the cleanup below would stop itself later on a closed gpiochip handle.
        while self.pwm:
            try:
                self.pwm.pop().stop()
            except Exception as exc:
                errors.append(str(exc))
        try:
            if self.configured:
                # Clean up every channel, not just ours: that also closes rpi-lgpio's
                # gpiochip handle. On the Trixie card a pin freed and claimed again through
                # a still-open handle never outputs PWM, so after a SET reload all four
                # motors sat dead at 0.15 A (reproduced on unconnected GPIO8, 2026-09-25).
                self.gpio.cleanup()
        except Exception as exc:
            errors.append(str(exc))
        self.ready = False
        self.configured.clear()
        if errors:
            print("GPIO cleanup errors: " + "; ".join(errors), file=sys.stderr)
        return not errors


class Keyboard:
    """Poll keys in the control loop, with no background stdin thread."""

    def __enter__(self):
        import termios
        import tty

        if not sys.stdin.isatty():
            raise RuntimeError("Use an interactive terminal so the stop key is available")
        self.termios = termios
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        try:
            # TCSANOW retains any stop key already queued by the operator.
            tty.setcbreak(self.fd, termios.TCSANOW)
            # Ctrl+S (XOFF) would freeze the next log write, and with it this loop, while
            # the wheels keep their duty. Ctrl+Z and Ctrl+\ would suspend or kill the process
            # with STBY high and PWM pins latched. Deliver all three as keys; poll() stops on them.
            attrs = termios.tcgetattr(self.fd)
            attrs[0] &= ~(termios.IXON | termios.IXOFF)
            try:
                disabled = os.fpathconf(self.fd, "PC_VDISABLE")
            except (AttributeError, OSError, ValueError):
                disabled = 0  # Linux's _POSIX_VDISABLE
            for name in ("VSUSP", "VQUIT", "VDSUSP"):
                index = getattr(termios, name, None)
                if index is not None:
                    attrs[6][index] = bytes([disabled])
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except BaseException:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
            raise
        return self

    def poll(self):
        if select.select([self.fd], [], [], 0)[0]:
            data = os.read(self.fd, 64)
            if not data or b"\x04" in data:  # EOF/Ctrl+D also stops movement.
                return "s"
            if any(key in data for key in STOP_CONTROL_KEYS):
                return "s"
            return data.decode("ascii", errors="ignore").lower()
        return ""

    def __exit__(self, *_):
        self.termios.tcsetattr(self.fd, self.termios.TCSANOW, self.saved)


class Controller:
    def __init__(self, motors, keys, clock=time, log=print):
        self.motors = motors
        self.keys = keys
        self.clock = clock
        self.log = log
        self.turn_observer = None  # optional read-only gyro log of timed turns
        self.light = None  # vision_support.TrafficLight while the camera is on (VT)
        self.approach_scale = 1.0  # drive_to_line's slow-approach duty fraction; 1 otherwise

    def poll(self):
        keys = self.keys.poll()
        if "s" in keys:
            self.motors.stop()
            raise StopRequested()
        # Every control loop polls here, so the power guard sees every powered tick.
        guard = getattr(self.motors, "power_guard", None)
        if guard is not None:
            guard.check(self.motors)
        if self.turn_observer is not None:
            self.turn_observer.tick()
        if self.light is not None:
            # Every wait and move polls, so the light stays current and a stale camera stops the run.
            self.light.update(self.clock.monotonic())
        return keys

    def steer(self, targets, state, now):
        """Lane-loop hook. FG adds a gyro heading trim here; FA and C use targets as-is."""
        return targets

    def motion_gate(self, pattern):
        """Optional obstacle gate: speed scale and elapsed stopped time."""
        return 1.0, 0.0

    def lane_event(self, direction, now):
        """Lane-loop hook, called when a side correction starts. FG nudges its heading target."""

    def go_run(self):
        """True when GO started this run (its keys say so): F, FA and FG then end at their
        first front line instead of waiting for 'g'."""
        return getattr(self.keys, "go_run", False) is True

    def go_ready(self):
        """A command that starts sensors first (FC) calls this just before it can move: GO's
        countdown runs here, so its clock starts with the wheels."""
        if self.go_run():
            self.keys.ready()

    def go_end(self, outcome=None, at=None):
        """The command's own end: GO stops its clock now, or at ``at`` (when the robot stopped
        at a line the camera then looked at). ``outcome`` names an end that is not a
        completed run, such as FC's time limit."""
        if self.go_run():
            self.keys.end_run(outcome, at)

    def tick_sleep(self, started):
        delay = LOOP_DT - (self.clock.monotonic() - started)
        if delay > 0:
            self.clock.sleep(delay)

    def wait(self, duration):
        deadline = self.clock.monotonic() + duration
        while True:
            self.poll()
            remaining = deadline - self.clock.monotonic()
            if remaining <= 1e-9:
                return
            self.clock.sleep(min(LOOP_DT, remaining))

    def prepare(self, pattern):
        self.approach_scale = 1.0  # every new movement starts at normal speed
        self.poll()
        if self.motors.pattern not in (None, pattern) and any(self.motors.duties):
            self.motors.stop()
            self.wait(DIRECTION_PAUSE)
        self.motors.set_pattern(pattern)

    def ramp(self, targets):
        targets = self.motors.validate_targets(targets)
        previous = self.clock.monotonic()
        while True:
            started = self.clock.monotonic()
            self.poll()
            if self.motors.pattern in (TURN_LEFT, TURN_RIGHT):
                reached = self.motors.slew_turn(targets, started - previous)
            else:
                reached = self.motors.slew(targets, RAMP_RATE, started - previous)
            previous = started
            if reached:
                return
            self.tick_sleep(started)

    def timed_move(self, pattern, targets, hold):
        """Timed tests/crossings intentionally ignore IR; stop keys stay active."""
        self.prepare(pattern)
        self.ramp(targets)
        self.wait(hold)

    def drive_to_line(self, auto=True, ignore_initial_line=True, *, allow_unlit=False):
        """Read IR and stop keys during acceleration and all lane corrections.

        Returns "stopped" at a front line (motors stopped), or "crossed" when the camera (VT)
        had a recent green: the robot notes the line and drives on, motors still running.
        CG intermediate legs may opt into "crossed_unlit" for no detected lamp.
        """
        self.prepare(FORWARD)
        straight = straight_targets()
        leans = {direction: diagonal_targets(direction, AUTO_SECONDARY_SPEED, AUTO_YAW_TRIM)
                 for direction in ("right", "left")}
        previous = self.clock.monotonic()
        drive_started = state_since = previous
        slowed = False
        state = "straight"
        front_armed = not ignore_initial_line
        line_since = None  # first white front read of a possible line
        line_reads = 0     # white front reads in a row
        slowest = 0.0      # longest gap between control ticks this lane: CPU load shows here
        last_tick = previous
        lost_since = [None, None]
        recovered_at = None
        accelerating = tuple(self.motors.duties) != straight
        both_warned = False

        while True:
            now = self.clock.monotonic()
            slowest, last_tick = max(slowest, now - last_tick), now
            self.poll()
            speed_scale, paused = self.motion_gate(FORWARD)
            if paused:
                previous += paused
                drive_started += paused
                state_since += paused
                if recovered_at is not None:
                    recovered_at += paused
                lost_since = [None if stamp is None else stamp + paused for stamp in lost_since]
                line_since, line_reads = None, 0
                accelerating = True
                last_tick = self.clock.monotonic()
            now = self.clock.monotonic()
            left, right, front = self.motors.read_ir()
            if front == 1:
                front_armed = True
                line_since, line_reads = None, 0
            elif front_armed:
                if line_since is None:
                    line_since = now
                line_reads += 1
                if now - line_since >= FRONT_DEBOUNCE - 1e-9:
                    crossing = self.light is not None and self.light.crossing(now)
                    unlit = (allow_unlit and self.light is not None
                             and not crossing and self.light.unlit_crossing())
                    if not crossing and not unlit:
                        self.motors.stop()
                    # Tells a real line from a false one: a lean or a side IR on white points at
                    # a lane edge under the front sensor; a slow tick points at CPU load.
                    self.log(f"Line detected: front white {line_reads} reads in a row, side IR L/R="
                             f"{left}/{right}, {'lane ' + state if state != 'straight' else 'straight'}; "
                             f"slowest tick {slowest * 1000:.0f} ms (target {LOOP_DT * 1000:.0f})")
                    if unlit:
                        self.light.went_on()
                        self.log("No light detected: crossing the CG intersection front line without stopping")
                        return "crossed_unlit"
                    if crossing:
                        self.light.went_on()
                        self.log("Green light: crossing the front line without stopping")
                        return "crossed"
                    return "stopped"

            if auto:
                for index, value in enumerate((left, right)):
                    if value == 0 and lost_since[index] is None:
                        lost_since[index] = now
                    elif value == 1:
                        lost_since[index] = None
                left_lost, right_lost = (
                    stamp is not None and now - stamp >= AUTO_DEBOUNCE - 1e-9
                    for stamp in lost_since)
                if state == "straight":
                    if left_lost != right_lost:
                        state = "right" if left_lost else "left"
                        state_since, recovered_at = now, None
                        self.log("Correcting " + state)
                        self.lane_event(state, now)
                    elif left_lost and not both_warned:
                        self.log("Both side sensors see white; continuing straight")
                        both_warned = True
                    elif not left_lost:
                        both_warned = False
                elif state in ("right", "left"):
                    if state == "right":
                        sensor_ok, opposite_lost = left == 1, right_lost
                        min_hold, recovery_hold = AUTO_MIN_CORRECT_LEFT, AUTO_OVERSHOOT_LEFT
                    else:
                        sensor_ok, opposite_lost = right == 1, left_lost
                        min_hold, recovery_hold = AUTO_MIN_CORRECT_RIGHT, AUTO_OVERSHOOT_RIGHT
                    if not sensor_ok:
                        recovered_at = None
                    elif recovered_at is None:
                        recovered_at = now
                    recovered = (recovered_at is not None
                                 and now - recovered_at >= recovery_hold - 1e-9)
                    elapsed = now - state_since
                    if opposite_lost or (elapsed >= min_hold - 1e-9 and recovered):
                        state, state_since = "settle", now
                        self.log("Settling straight")
                    elif elapsed >= AUTO_MAX_CORRECT - 1e-9:
                        self.motors.stop()
                        raise LaneFault("Correction timed out. Check lane position and IR sensors.")
                elif now - state_since >= AUTO_SETTLE - 1e-9:
                    state, state_since = "straight", now

            target = leans[state] if state in leans else straight
            if APPROACH_SLOW_AFTER > 0 and now - drive_started >= APPROACH_SLOW_AFTER - 1e-9:
                if not slowed:
                    slowed = True
                    self.approach_scale = APPROACH_SPEED_PCT / 100.0
                    self.log(f"Slowing to {APPROACH_SPEED_PCT:g}% for the front line")
                target = tuple(value * self.approach_scale for value in target)
            target = tuple(value * speed_scale for value in self.steer(target, state, now))
            rate = RAMP_RATE if accelerating or not auto else AUTO_SLEW_RATE
            reached = self.motors.slew(target, rate, now - previous)
            previous = now
            if reached:
                accelerating = False
            self.tick_sleep(now)

    def wait_for_green(self):
        """Wait for green or a terminal sign; return the current prioritized decision."""
        while True:
            started = self.clock.monotonic()
            self.poll()
            decision = self.light.decision()
            if decision in ("green", "stop_sign"):
                return decision
            self.tick_sleep(started)

    def wait_out_light(self):
        """After stopping and looking at a front line with the camera on: wait out a red or
        yellow until green or a sign. Returns "green", "stop_sign" or "none".
        The caller decides whether a sign ends the run or is an intermediate course stop.
        """
        self.poll()  # refresh at the decision boundary, including after CG's re-zero wait
        decision = self.light.decision()
        if decision in ("red", "yellow"):
            self.log(f"{decision.capitalize()} light at the front line: waiting for green. 's' stops.")
            decision = self.wait_for_green()
            if decision == "green":
                self.log("Green light: going on across the front line")
            return decision
        if decision == "green":
            self.log("Green light seen from the front line: going on")
        return decision

    def look_at_line(self):
        """Stopped at a front line with the camera on: look, then wait out a red or yellow.
        A stop sign already confirmed on the approach needs no look."""
        self.light.at_line(self.clock.monotonic())
        if not self.light.stop_sign:
            self.wait(self.light.look_seconds)
        return self.wait_out_light()

    def forward(self, auto):
        ignore_line = auto  # Retain FA's original ability to depart a starting line.
        while True:
            if self.drive_to_line(auto=auto, ignore_initial_line=ignore_line) == "crossed":
                ignore_line = True
                continue
            ignore_line = True
            stopped_at = self.clock.monotonic()
            if self.light is not None:
                decision = self.look_at_line()
                if decision == "green":
                    self.light.went_on()
                    continue
                if decision == "stop_sign":
                    self.go_end(at=stopped_at)
                    self.log("Stop sign: stopped at its front line. Run complete.")
                    return
                self.log("No light confirmed from the front line")
            if self.go_run():
                self.go_end(at=stopped_at)
                self.log("Front line: stopped. Run complete.")
                return
            self.log("Front line: stopped. Press 'g' to continue or 's' for the menu.")
            while True:
                started = self.clock.monotonic()
                if "g" in self.poll():
                    break
                self.tick_sleep(started)
            if self.light is not None:
                self.light.went_on()

    def turn_maneuver(self, direction, label=None):
        approach = TURN_APPROACH_TIME_LEFT if direction == "left" else TURN_APPROACH_TIME_RIGHT
        hold = TURN_DURATION_LEFT if direction == "left" else TURN_DURATION_RIGHT
        pattern = TURN_LEFT if direction == "left" else TURN_RIGHT
        self.log(f"Approaching intersection at {TURN_APPROACH_SPEED_PCT:g}% of normal")
        approach_targets = tuple(value * TURN_APPROACH_SPEED_PCT / 100.0
                                 for value in straight_targets())
        self.timed_move(FORWARD, approach_targets, approach)
        self.log("Turning " + direction)
        observer = self.turn_observer
        if observer is not None:
            observer.begin_turn(direction, label or ("TL" if direction == "left" else "TR"))
        self.timed_move(pattern, straight_targets(), hold)
        self.motors.stop()
        if observer is not None:
            observer.end_turn()

    def stop_wait(self, duration):
        """A course's stop at a front line. CG re-zeroes its gyro here instead."""
        self.wait(duration)

    def intersection(self, command, reached):
        """What a course does at a front line before its maneuver.

        Camera off: stop COURSE_STOP_TIME, as always. Camera on (VT): a green crossing goes
        straight into the maneuver; otherwise the robot stops at least the camera's look,
        waits out a red or yellow until green, and goes on.
        """
        if reached == "crossed_unlit":
            self.log(f"CG intersection reached with no light: {command} without a front-line wait")
            return
        if reached == "crossed":
            self.log(f"Intersection reached on a green light: {command} without stopping")
            return
        if self.light is None:
            self.log(f"Intersection reached; waiting {COURSE_STOP_TIME}s, then {command}")
            self.stop_wait(COURSE_STOP_TIME)
            return
        stop = max(COURSE_STOP_TIME, self.light.look_seconds)
        self.log(f"Intersection reached; stopped {stop:g}s while the camera looks, then {command}")
        self.light.at_line(self.clock.monotonic())
        self.stop_wait(stop)
        decision = self.wait_out_light()
        if decision == "stop_sign":
            self.log(f"Stop sign at this intersection; the course has more legs, so {command} goes on")
        elif decision == "none":
            self.log("No light confirmed from the front line")
        self.light.went_on()

    def final_lane(self):
        """Camera courses (VT): after the last maneuver, lane-drive to the stop sign's line.

        The run ends at the first front line the robot stops at for good: confirmed as the
        stop sign's when the camera saw it (PRD course end), otherwise logged as unconfirmed.
        Lights on the way are handled as at any front line.
        """
        self.log("Final lane: driving to the stop sign's front line")
        while True:
            if self.drive_to_line() == "crossed":
                continue
            stopped_at = self.clock.monotonic()
            decision = self.look_at_line()
            if decision == "green":
                self.light.went_on()
                continue
            self.go_end(at=stopped_at)
            self.log("Stop sign: course complete" if decision == "stop_sign" else
                     "Course complete at the final front line; no stop sign confirmed (check it with RV)")
            return

    def course(self, sequence):
        for index, command in enumerate(sequence, 1):
            self.log(f"[{index}/{len(sequence)}] Driving to intersection")
            self.intersection(command, self.drive_to_line())
            if command == "F":
                if COURSE_CROSS_TIME > 0:
                    self.timed_move(FORWARD, straight_targets(), COURSE_CROSS_TIME)
            else:
                self.turn_maneuver("right" if command == "R" else "left", f"{index} {command}")
        if self.light is not None:
            self.final_lane()
            return
        self.timed_move(FORWARD, straight_targets(), COURSE_EXIT_STRAIGHT_TIME)
        self.go_end()
        self.log("Course complete")

    def ir_test(self):
        self.motors.stop()
        last = None
        while True:
            started = self.clock.monotonic()
            self.poll()
            readings = self.motors.read_ir()
            if readings != last:
                self.log(f"Left IR={readings[0]}  Right IR={readings[1]}  Front IR={readings[2]}")
                last = readings
            self.tick_sleep(started)

    def run_mode(self, command, sequence=""):
        """Every path (normal, stop key, fault, exception) ends with motors off."""
        try:
            if command in ("F", "FA"):
                self.forward(auto=command == "FA")
            elif command in ("SR", "SL"):
                pattern = STRAFE_RIGHT if command == "SR" else STRAFE_LEFT
                self.timed_move(pattern, straight_targets(), STRAFE_DURATION)
            elif command in ("R", "L"):
                pattern = TURN_RIGHT if command == "R" else TURN_LEFT
                hold = TURN_DURATION_RIGHT if command == "R" else TURN_DURATION_LEFT
                observer = self.turn_observer
                if observer is not None:
                    observer.begin_turn("right" if command == "R" else "left", command)
                self.timed_move(pattern, straight_targets(), hold)
                if observer is not None:
                    self.motors.stop()
                    observer.end_turn()
                    self.wait(observer.WINDOW)  # motors already off: measure the coast
            elif command in ("TR", "TL"):
                self.turn_maneuver("right" if command == "TR" else "left", command)
                self.timed_move(FORWARD, straight_targets(), TURN_EXIT_STRAIGHT_TIME)
            elif command in ("DR", "DL"):
                direction = "right" if command == "DR" else "left"
                targets = diagonal_targets(direction, DIAGONAL_SECONDARY_SPEED, DIAGONAL_YAW_TRIM)
                self.timed_move(FORWARD, targets, DIAGONAL_DURATION)
            elif command == "IR":
                self.ir_test()
            elif command == "C":
                sequence = "".join(sequence.upper().split())
                if not sequence or any(char not in "FRL" for char in sequence):
                    raise ValueError("Course must contain only F, R, and L, e.g. FRL")
                self.course(sequence)
            else:
                raise ValueError("Unknown mode: " + command)
        finally:
            self.motors.stop()
            if self.turn_observer is not None:
                self.turn_observer.finish()


MENU = ("\nF forward | FA lane centering | SR/SL strafe | R/L turn | "
        "TR/TL intersection turn | C FRL course | DR/DL diagonal | IR sensors | Q quit")
MODES = {"F", "FA", "SR", "SL", "R", "L", "TR", "TL", "C", "DR", "DL", "IR"}


def prompt_line(prompt):
    """Use the same unbuffered fd as Keyboard so type-ahead 's' is not hidden."""
    print(prompt, end="", flush=True)
    result = bytearray()
    while True:
        char = os.read(sys.stdin.fileno(), 1)
        if not char:
            raise EOFError()
        if char == b"\n":
            return result.decode("utf-8", errors="replace").rstrip("\r")
        result.extend(char)


def main():
    if not sys.stdin.isatty():
        print("Run in an interactive terminal on the Raspberry Pi.", file=sys.stderr)
        return 1
    try:
        import RPi.GPIO as GPIO
    except (ImportError, RuntimeError) as exc:
        print(f"Cannot load RPi.GPIO: {exc}", file=sys.stderr)
        return 1

    motors = MotorDriver(GPIO)
    try:
        motors.initialize()
        while True:
            print(MENU)
            parts = prompt_line("Enter choice: ").strip().upper().split(maxsplit=1)
            if not parts:
                continue
            command = parts[0]
            if command == "Q":
                break
            if command not in MODES:
                print("Unknown command")
                continue
            sequence = parts[1] if len(parts) == 2 else ""
            if command == "C" and not sequence:
                sequence = prompt_line("Enter course, e.g. FRL: ")
            print("Press 's' to stop and return to the menu.")
            try:
                with Keyboard() as keys:
                    Controller(motors, keys).run_mode(command, sequence)
            except StopRequested:
                print("Stopped")
            except (LaneFault, ValueError) as exc:
                print("Stopped: " + str(exc))
    except (KeyboardInterrupt, EOFError):
        print("\nExiting")
    except Exception as exc:
        print(f"Motor controller error: {exc}", file=sys.stderr)
        return 1
    finally:
        motors.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

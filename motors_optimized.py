#!/usr/bin/env python3
"""Mecanum motor driver and timed maneuvers for the TB6612FNG rev C board.

Run on the robot's Raspberry Pi: python3 motors_optimized.py
Commands: F SR SL R L TR TL DR DL Q. 's' stops every mode; Ctrl+C stops and exits.

Timed maneuver durations are HOLD times after acceleration. Recheck them on
the robot. Importing this file does not import RPi.GPIO or initialize hardware.
"""

import math
import os
import select
import sys
import time
from motion_profile import PRESET


# Wheel order everywhere: FL, FR, BL, BR. Pins use BCM numbering.
STBY = 24
# FL M1A (J2), FR M1B (J4), BL M2B (J12), BR M2A (J5): the documented rev C map.
# Each triple is (IN1, IN2, PWM).
MOTOR_PINS = ((17, 27, 18), (22, 23, 25), (16, 19, 13), (5, 6, 12))
REVERSED = (False, False, False, False)
PWM_FREQ = 1000
MAX_MOTOR_DUTY = 70.0  # Project ceiling; does not provide stall/current protection.
# Logical wheel order FL, FR, BL, BR, independent of electrical REVERSED wiring.
# Attenuation only: a factor can never raise an output above its request.
WHEEL_FACTORS_FORWARD = (1.0, 1.0, 1.0, 1.0)
WHEEL_FACTORS_REVERSE = (1.0, 1.0, 1.0, 1.0)

LEFT_SPEED = PRESET["left_speed"]
RIGHT_SPEED = PRESET["right_speed"]
LOOP_DT = 0.02
RAMP_RATE = PRESET["ramp_rate"]
DIRECTION_PAUSE = 0.05  # zero output before changing an active direction pattern
BRAKE_ON_STOP = True  # TB6612FNG short brake; False selects coast/standby
STRAFE_DURATION = 0.6
TURN_DURATION_RIGHT = 0.12
TURN_DURATION_LEFT = 0.14
TURN_APPROACH_TIME_RIGHT = 0.55
TURN_APPROACH_TIME_LEFT = 1.5
TURN_EXIT_STRAIGHT_TIME = 2.0
DIAGONAL_DURATION = 0.6
DIAGONAL_SECONDARY_SPEED = PRESET["diagonal_secondary_speed"]
# Positive trim rotates toward the slide; negative rotates away.
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
    """A motion loop was interrupted or exceeded its permitted duration."""


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
        self.pattern = None
        self.configured = []
        self.ready = False

    def initialize(self):
        self.validate_wheel_factors()  # Reject bad factors before any GPIO write.
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

    def set_duties(self, targets):
        targets = self.validate_targets(targets)
        forward_factors, reverse_factors = self.validate_wheel_factors()
        if not self.ready:
            raise RuntimeError("Motor driver has not been initialized")
        if self.pattern is None and any(targets):
            raise RuntimeError("Set a direction pattern before applying PWM")
        # Compute and validate every output before writing any channel. Keep requests
        # separate so repeated slew/control updates cannot multiply a factor twice.
        applied = tuple(target if not target else target *
                        (forward_factors[index] if self.pattern[index] else reverse_factors[index])
                        for index, target in enumerate(targets))
        applied = self.validate_targets(applied)
        for index, target in enumerate(applied):
            if target != self.applied_duties[index]:
                self.pwm[index].ChangeDutyCycle(target)
                self.applied_duties[index] = target
            self.duties[index] = targets[index]

    def slew(self, targets, rate, dt):
        targets = self.validate_targets(targets)
        # Do not make a large catch-up jump after a delayed control iteration.
        step = rate * max(0.0, min(dt, LOOP_DT))
        values = []
        for current, target in zip(self.duties, targets):
            diff = target - current
            values.append(target if abs(diff) <= step + 1e-9
                          else current + math.copysign(step, diff))
        self.set_duties(values)
        return tuple(self.duties) == targets

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

    def stop(self, brake=None):
        # Disable both H-bridges first, even if a later PWM operation fails.
        if brake is None:
            brake = BRAKE_ON_STOP
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
        for pwm in self.pwm:
            try:
                pwm.stop()
            except Exception as exc:
                errors.append(str(exc))
        try:
            if self.configured:
                self.gpio.cleanup(tuple(self.configured))
        except Exception as exc:
            errors.append(str(exc))
        self.ready = False
        self.pwm.clear()
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

    def poll(self):
        keys = self.keys.poll()
        if "s" in keys:
            self.motors.stop()
            raise StopRequested()
        if self.turn_observer is not None:
            self.turn_observer.tick()
        return keys

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
            reached = self.motors.slew(targets, RAMP_RATE, started - previous)
            previous = started
            if reached:
                return
            self.tick_sleep(started)

    def timed_move(self, pattern, targets, hold):
        """Ramp up, then hold for `hold` seconds; stop keys stay active."""
        self.prepare(pattern)
        self.ramp(targets)
        self.wait(hold)

    def forward(self):
        """Drive straight until 's' is pressed."""
        self.prepare(FORWARD)
        self.ramp(straight_targets())
        self.log("Driving forward; press 's' to stop.")
        while True:
            started = self.clock.monotonic()
            self.poll()
            self.tick_sleep(started)

    def turn_maneuver(self, direction, label=None):
        approach = TURN_APPROACH_TIME_LEFT if direction == "left" else TURN_APPROACH_TIME_RIGHT
        hold = TURN_DURATION_LEFT if direction == "left" else TURN_DURATION_RIGHT
        pattern = TURN_LEFT if direction == "left" else TURN_RIGHT
        self.log("Approaching intersection")
        self.timed_move(FORWARD, straight_targets(), approach)
        self.log("Turning " + direction)
        observer = self.turn_observer
        if observer is not None:
            observer.begin_turn(direction, label or ("TL" if direction == "left" else "TR"))
        self.timed_move(pattern, straight_targets(), hold)
        self.motors.stop()
        if observer is not None:
            observer.end_turn()

    def run_mode(self, command, sequence=""):
        """Every path (normal, stop key, fault, exception) ends with motors off."""
        try:
            if command == "F":
                self.forward()
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
            else:
                raise ValueError("Unknown mode: " + command)
        finally:
            self.motors.stop()
            if self.turn_observer is not None:
                self.turn_observer.finish()


MENU = ("\nF forward | SR/SL strafe | R/L turn | TR/TL intersection turn | "
        "DR/DL diagonal | Q quit")
MODES = {"F", "SR", "SL", "R", "L", "TR", "TL", "DR", "DL"}


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
            print("Press 's' to stop and return to the menu.")
            try:
                with Keyboard() as keys:
                    Controller(motors, keys).run_mode(command)
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

"""Removable digital IR tachometer and bounded, unloaded PWM calibration.

An FC-51's digital output observes ONE contrasting patch per wheel revolution.
Both edges are checked, but only rising edges count rotations. GPIO is supplied
by the application; importing this module never imports or initializes hardware.
These measurements cannot prove marker count, loaded speed, or motor direction.
"""

from collections import deque
from dataclasses import dataclass
import math
import threading
import time

import motors_optimized as motor


CONNECTOR_PINS = (7, 20, 21, 26)
MIN_RPM = 30.0
MAX_RPM = 1200.0
MIN_PULSE_WIDTH = 0.001
MIN_PERIODS = 4
MAX_PERIOD_JITTER = 0.12
PULSE_TIMEOUT = 3.0
ATTEMPT_TIMEOUT = 12.0
SETTLE_SECONDS = 0.75
COAST_SECONDS = 0.5
MAX_LOOP_GAP = 0.12
RPM_TOLERANCE = 0.05
MIN_FACTOR = 0.5
SEARCH_STEPS = 7


class TachFault(motor.LaneFault):
    """The calibration probe or measurement could not be trusted."""


@dataclass(frozen=True)
class TachSnapshot:
    edge_count: int
    turn_count: int
    periods: tuple
    last_edge: object
    fault: object


def validate_pin(pin):
    """Reject conflicts against CURRENT wiring before any GPIO operation."""
    used = {motor.STBY, *motor.IR_PINS}
    used.update(pin for pins in motor.MOTOR_PINS for pin in pins)
    if type(pin) is not int or pin not in CONNECTOR_PINS:
        raise ValueError("Probe must use an explicit Rev C IR connector BCM pin: 7, 20, 21 or 26")
    if pin in used:
        raise ValueError(f"Probe BCM{pin} conflicts with an active motor or lane IR pin")
    return pin


class Tachometer:
    """Read a digital comparator with bounded callback state and no worker loop.

    snapshot() is useful for a motors-OFF, hand-rotation check. Count one rising
    transition per full turn; software cannot detect two identical markers.
    check(require_motion=False) permits a stationary probe but still rejects
    electrical noise. rpm() needs four complete, consistent periods.
    """

    def __init__(self, gpio, pin, clock=time):
        self.gpio, self.pin, self.clock = gpio, validate_pin(pin), clock
        self._lock = threading.Lock()
        self._configured = self._detecting = self._opened = False
        self._periods = deque(maxlen=8)
        self._fault = None
        self._edge_count = self._turn_count = 0
        self._last_edge = self._last_rising = self._level = None
        self._started = None

    def __enter__(self):
        validate_pin(self.pin)  # Settings may have changed since construction.
        if self._opened or self._configured:
            raise RuntimeError("Tachometer is already open")
        try:
            self.gpio.setup(self.pin, self.gpio.IN, pull_up_down=self.gpio.PUD_DOWN)
            self._configured = True
            self.reset()
            self.gpio.add_event_detect(self.pin, self.gpio.BOTH, callback=self._edge)
            self._detecting = self._opened = True
            return self
        except BaseException:
            self.close()
            raise

    def reset(self):
        """Discard ramp/settling pulses before a fresh steady-speed sample."""
        if not self._configured:
            raise RuntimeError("Open the tachometer before resetting it")
        with self._lock:
            now = self.clock.monotonic()
            level = self.gpio.input(self.pin)
            if not math.isfinite(now) or level not in (0, 1):
                raise TachFault("Invalid tachometer clock or digital level")
            self._periods.clear()
            self._fault = None
            self._edge_count = self._turn_count = 0
            self._last_edge = self._last_rising = None
            self._level, self._started = level, now

    def _edge(self, channel):
        # RPi.GPIO calls this on its own thread. Never raise or print there.
        with self._lock:
            if not self._configured or self._fault is not None:
                return
            try:
                now = self.clock.monotonic()
                level = self.gpio.input(self.pin)
                if channel != self.pin or level not in (0, 1) or not math.isfinite(now):
                    raise TachFault("Invalid tachometer edge")
                previous = self._last_edge if self._last_edge is not None else self._started
                if now < previous:
                    raise TachFault("Tachometer clock moved backwards")
                if level == self._level:
                    raise TachFault("Tachometer missed an edge or has a noisy digital signal")
                if self._last_edge is not None and now - self._last_edge < MIN_PULSE_WIDTH:
                    raise TachFault("Tachometer pulses are too narrow/noisy; adjust probe gap or sensitivity")
                self._level, self._last_edge = level, now
                self._edge_count += 1
                if level == 1:
                    if self._last_rising is not None:
                        self._periods.append(now - self._last_rising)
                    self._last_rising = now
                    self._turn_count += 1
            except Exception as exc:
                self._fault = str(exc) or "Tachometer callback failed"

    def snapshot(self):
        with self._lock:
            return TachSnapshot(self._edge_count, self._turn_count,
                                tuple(self._periods), self._last_edge, self._fault)

    def check(self, now=None, require_motion=False):
        if not self._opened:
            raise RuntimeError("Open the tachometer before reading it")
        now = self.clock.monotonic() if now is None else now
        with self._lock:
            if self._fault is not None:
                raise TachFault(self._fault)
            if not math.isfinite(now) or now < self._started:
                raise TachFault("Invalid tachometer clock")
            last = self._last_edge if self._last_edge is not None else self._started
            if require_motion and now - last >= PULSE_TIMEOUT:
                raise TachFault("No fresh tachometer pulses: disconnected/stuck probe, stalled wheel, "
                                "or speed below the 30 RPM calibration range")

    def rpm(self, now=None):
        self.check(now, require_motion=True)
        snapshot = self.snapshot()
        # A callback may have faulted after check() released the lock. Use the
        # fault and periods from the same immutable snapshot before accepting.
        if snapshot.fault is not None:
            raise TachFault(snapshot.fault)
        periods = snapshot.periods
        if len(periods) < MIN_PERIODS:
            return None
        average = sum(periods) / len(periods)
        rpm = 60.0 / average if average > 0 else math.inf
        if not MIN_RPM - 1e-9 <= rpm <= MAX_RPM + 1e-9:
            raise TachFault(f"Measured RPM is outside the supported {MIN_RPM:g}–{MAX_RPM:g} range")
        if (max(periods) - min(periods)) / average > MAX_PERIOD_JITTER:
            raise TachFault("Tachometer periods are inconsistent; check marker, gap and wheel settling")
        return rpm

    def close(self):
        try:
            if self._detecting:
                self.gpio.remove_event_detect(self.pin)
        finally:
            self._detecting = self._opened = False
            with self._lock:
                configured, self._configured = self._configured, False
            if configured:
                self.gpio.cleanup(self.pin)  # This probe only; never clean up motor GPIO.

    def __exit__(self, *_):
        self.close()


class CalibrationRunner:
    """One-wheel-at-a-time raw measurement; no prompts or persistence.

    Each measure() invocation starts/stops the motor itself. The caller owns
    Keyboard and the existing power guard. Prompts and printing belong BETWEEN
    invocations, while the wheels are stopped. Keep wheels lifted and the probe
    physically secured. Only attenuation is allowed: raw duty never exceeds the
    explicitly selected reference duty or the driver's project ceiling.
    """

    def __init__(self, motors, keys, pin, clock=time, log=print):
        self.motors, self.clock = motors, clock
        self.controller = motor.Controller(motors, keys, clock=clock, log=log)
        self.tach = Tachometer(motors.gpio, pin, clock=clock)
        self._opened = False

    def __enter__(self):
        if not self.motors.ready or any(self.motors.duties):
            raise RuntimeError("Initialize and stop the motors before opening calibration")
        self.tach.__enter__()
        self._opened = True
        return self

    def __exit__(self, *_):
        try:
            self.motors.stop(brake=False)
        finally:
            self._opened = False
            self.tach.close()

    @staticmethod
    def _validate(wheel, forward, duty):
        if type(wheel) is not int or not 0 <= wheel < 4 or type(forward) is not bool:
            raise ValueError("Select a wheel index 0–3 and a boolean logical direction")
        if isinstance(duty, bool) or not isinstance(duty, (int, float)) or not math.isfinite(duty):
            raise ValueError("Calibration duty must be finite")
        if not 0 < duty <= motor.MAX_MOTOR_DUTY:
            raise ValueError(f"Calibration duty must be above zero and at most {motor.MAX_MOTOR_DUTY:g}%")

    def measure(self, wheel, forward, duty):
        try:
            return self._measure(wheel, forward, duty)
        finally:
            # Includes validation/open-state errors as well as powered failures.
            self.motors.stop(brake=False)

    def _measure(self, wheel, forward, duty):
        self._validate(wheel, forward, duty)
        if not self._opened:
            raise RuntimeError("Open the calibration runner before measuring")
        if any(self.motors.duties):
            raise RuntimeError("Stop all wheels before a calibration measurement")
        with self.motors.raw_calibration():
            try:
                # Repeated forward/reverse measurements must not reverse a
                # still-energized bridge or immediately reuse a spinning wheel.
                # This unpowered pause retains stop/power-guard polling, and is
                # outside the hard limit on each powered measurement attempt.
                self.motors.stop(brake=False)
                self.controller.wait(max(COAST_SECONDS, motor.DIRECTION_PAUSE))
                # Clear old stopped/handling events before motion. Do not discard
                # errors again until the full ramp and settling phase is checked.
                self.tach.reset()
                self.controller.poll()  # queued stop/power trip before first PWM
                pattern = tuple(forward if index == wheel else True for index in range(4))
                targets = tuple(float(duty) if index == wheel else 0.0 for index in range(4))
                started = previous = self.clock.monotonic()
                if not math.isfinite(started):
                    raise TachFault("Invalid calibration clock")
                self.motors.set_pattern(pattern)
                settled_at = None
                measuring = False
                while True:
                    self.controller.poll()
                    now = self.clock.monotonic()
                    if not math.isfinite(now) or not 0 <= now - previous <= MAX_LOOP_GAP:
                        raise TachFault("Calibration control loop was interrupted; motors stopped")
                    if now - started >= ATTEMPT_TIMEOUT:
                        raise TachFault("Calibration measurement timed out; check probe or reduce the target")
                    self.tach.check(now, require_motion=measuring)
                    if settled_at is None:
                        if self.motors.slew(targets, motor.RAMP_RATE, now - previous):
                            settled_at = now + SETTLE_SECONDS
                    elif not measuring and now >= settled_at:
                        self.tach.reset()
                        measuring = True
                    elif measuring:
                        rpm = self.tach.rpm(now)
                        if rpm is not None:
                            return rpm
                    previous = now
                    self.controller.tick_sleep(now)
            finally:
                self.motors.stop(brake=False)

    def calibrate(self, wheel, forward, target_rpm, reference_duty):
        self._validate(wheel, forward, reference_duty)
        if (isinstance(target_rpm, bool) or not isinstance(target_rpm, (int, float))
                or not math.isfinite(target_rpm) or not MIN_RPM <= target_rpm <= MAX_RPM):
            raise ValueError(f"Target RPM must be between {MIN_RPM:g} and {MAX_RPM:g}")
        low, high = reference_duty * MIN_FACTOR, float(reference_duty)
        candidate = high
        rpm = self.measure(wheel, forward, candidate)
        if rpm < target_rpm * (1 - RPM_TOLERANCE):
            raise TachFault("Wheel cannot reach the target at the reference duty; no calibration saved")
        for step in range(SEARCH_STEPS + 1):
            if abs(rpm / target_rpm - 1.0) <= RPM_TOLERANCE:
                verified = self.measure(wheel, forward, candidate)
                if abs(verified / target_rpm - 1.0) > RPM_TOLERANCE:
                    raise TachFault("Final RPM verification failed; no calibration saved")
                return candidate / reference_duty, verified
            if step == SEARCH_STEPS:
                break
            if rpm > target_rpm:
                high = candidate
            else:
                low = candidate
            candidate = (low + high) / 2.0
            rpm = self.measure(wheel, forward, candidate)
        raise TachFault("No repeatable target within the 0.5–1.0 correction range; no calibration saved")

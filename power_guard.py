"""Stop the motors on sustained overcurrent or a low pack while driving.

Uses the board's INA219 through battery_support's worker thread, sampling
with a 20 ms pause between paired conversions (roughly 40 ms total with
the configured averaging), only around powered commands. Every control-loop poll
calls check(); a trip parks every PWM, drops STBY (coast) and raises
PowerFault. Limits are per-robot settings (SET -> Power protection).

This is software protection. It cannot act before the program runs, it does
not replace a fuse (the battery line has none), and its limits are modeled
values until stall current is measured on each robot.
"""

import math
import time

from battery_support import BatteryFault, BatteryStream, check_battery_sample

GUARD_PERIOD = 0.02      # worker wait between paired INA219 conversions
MAX_SAMPLE_AGE = 0.5     # a reading older than this while driving trips
START_TIMEOUT = 1.5      # first reading must arrive before any motion
POWERED_HINT = ("run BAT or --check-ina to diagnose it. SET -> Power protection can explicitly disable "
               "the guard for this robot; that removes current/voltage protection. Changes apply on leaving SET")


class PowerFault(Exception):
    """The power guard stopped the motors."""


def _positive(name, value):
    try:
        ok = math.isfinite(value) and value > 0
    except TypeError:
        ok = False
    if not ok:
        raise ValueError(f"{name} must be a positive finite number")
    return float(value)


class PowerGuard:
    """Overcurrent and low-voltage trip for one powered command."""

    def __init__(self, overcurrent_a, overcurrent_hold_s, low_voltage_v, low_voltage_hold_s,
                 bus=1, address=0x40, shunt_ohms=0.01, stream_factory=None, clock=time):
        self.overcurrent_a = _positive("overcurrent_a", overcurrent_a)
        self.overcurrent_hold_s = _positive("overcurrent_hold_s", overcurrent_hold_s)
        self.low_voltage_v = _positive("low_voltage_v", low_voltage_v)
        self.low_voltage_hold_s = _positive("low_voltage_hold_s", low_voltage_hold_s)
        self.clock = clock
        self.shunt_ohms = _positive("shunt_ohms", shunt_ohms)
        if stream_factory is None:
            def stream_factory():
                return BatteryStream(bus, address, shunt_ohms, clock=clock, period=GUARD_PERIOD)
        self.stream = stream_factory()
        self.peak_current = None
        self.min_voltage = None
        self.drove = False
        self.trip_reason = None
        self._over_since = None
        self._low_since = None

    @classmethod
    def from_settings(cls, settings, bus=1, address=0x40, shunt_ohms=0.01, **kwargs):
        return cls(settings["power_overcurrent_a"], settings["power_overcurrent_hold_s"],
                   settings["power_low_voltage_v"], settings["power_low_voltage_hold_s"],
                   bus=bus, address=address, shunt_ohms=shunt_ohms, **kwargs)

    def describe(self):
        return (f"Power guard ON: stops at {self.overcurrent_a:g} A for {self.overcurrent_hold_s:g} s, "
                f"or {self.low_voltage_v:g} V for {self.low_voltage_hold_s:g} s")

    def wait_ready(self, poll=None):
        """Block motion until the first reading arrives; nothing is powered yet."""
        deadline = self.clock.monotonic() + START_TIMEOUT
        while True:
            if poll is not None:
                poll()
            try:
                sample = self.stream.latest()
                if sample is not None:
                    self._validate(sample, self.clock.monotonic())
            except BatteryFault as exc:
                raise PowerFault(f"Power guard could not read the INA219 ({exc}). "
                                 f"Powered tests are blocked: {POWERED_HINT}.") from exc
            if sample is not None:
                self._record(sample)
                return sample
            if self.clock.monotonic() >= deadline:
                raise PowerFault(f"Power guard got no INA219 reading in {START_TIMEOUT:g} s. "
                                 f"Powered tests are blocked: {POWERED_HINT}.")
            self.clock.sleep(0.01)

    def check(self, motors):
        """Validate before motion as well as during it; never allow a blind first pulse."""
        now = self.clock.monotonic()
        active = any(duty > 0 for duty in motors.duties)
        try:
            sample = self.stream.latest()
            if sample is not None:
                self._validate(sample, now)
        except BatteryFault as exc:
            self._trip(motors, self._failure_reason(exc))
        if sample is None:
            self._trip(motors, "Power guard: no current reading while driving.")
        if active:
            self.drove = True
        self._record(sample)
        if not active:
            self._over_since = self._low_since = None
            return
        if abs(sample.current) >= self.overcurrent_a:
            if self._over_since is None:
                self._over_since = sample.timestamp
            held = sample.timestamp - self._over_since
            if held >= self.overcurrent_hold_s - 1e-9:
                self._trip(motors, f"Overcurrent: {abs(sample.current):.2f} A for {held:.2f} s "
                                   f"(limit {self.overcurrent_a:g} A).")
        else:
            self._over_since = None
        voltage = sample.input_voltage
        if voltage <= self.low_voltage_v:
            if self._low_since is None:
                self._low_since = sample.timestamp
            held = sample.timestamp - self._low_since
            if held >= self.low_voltage_hold_s - 1e-9:
                self._trip(motors, f"Low battery: {voltage:.2f} V for {held:.2f} s "
                                   f"(limit {self.low_voltage_v:g} V). Charge the pack.")
        else:
            self._low_since = None

    def summary(self):
        if self.peak_current is None:
            return None
        text = f"Power: peak {self.peak_current:.2f} A, lowest {self.min_voltage:.2f} V"
        return text + ("" if self.drove else " (motors never driven)")

    def close(self):
        return self.stream.close()

    def _record(self, sample):
        current, voltage = abs(sample.current), sample.input_voltage
        if self.peak_current is None or current > self.peak_current:
            self.peak_current = current
        if self.min_voltage is None or voltage < self.min_voltage:
            self.min_voltage = voltage

    @staticmethod
    def _validate(sample, now):
        age = now - sample.timestamp
        if age > MAX_SAMPLE_AGE:
            raise BatteryFault(f"current reading is {age:.2f} s old; the INA219 read stalled")
        check_battery_sample(sample, now)

    def _failure_reason(self, exc):
        if "80 mV" in str(exc):
            return ("Overcurrent: current beyond the INA219's measurable range "
                    f"(about {0.08 / self.shunt_ohms:g} A for the configured shunt).")
        return f"Power guard: the current monitor failed ({exc})."

    def _trip(self, motors, reason):
        self.trip_reason = reason
        motors.stop(brake=False)  # PWM 0, inputs low, STBY low before the fault surfaces.
        raise PowerFault(reason + " Motors stopped and STBY dropped.")

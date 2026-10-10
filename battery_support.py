"""INA219 measurements and a battery readout; no hardware on import.

Defaults come from the project's PCB/config: I2C 1, 0x40, 0.01 ohm shunt.
Register definitions: https://www.ti.com/lit/ds/symlink/ina219.pdf
Current is calculated from signed shunt voltage, independent of the chip's
current/power calibration registers. This module does not control charging.
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time

from imu_support import open_bus

MAX_BATTERY_AGE = 0.75
INA_CONFIG = 0x2E63  # 32 V range, +/-80 mV, 16 averages per ADC, triggered both.


class BatteryFault(Exception):
    pass


@dataclass(frozen=True)
class BatterySample:
    timestamp: float
    bus_voltage: float     # IN- to GND, load side of the shunt
    shunt_voltage: float   # IN+ minus IN-
    current: float         # positive: IN+ toward IN-

    @property
    def input_voltage(self):
        return self.bus_voltage + self.shunt_voltage

    @property
    def power(self):
        return self.input_voltage * self.current


def check_battery_sample(sample, now):
    if sample is None:
        raise BatteryFault("No battery reading yet")
    if not all(map(math.isfinite, (sample.timestamp, sample.bus_voltage,
                                  sample.shunt_voltage, sample.current))):
        raise BatteryFault("Invalid INA219 reading")
    if not -0.01 <= now - sample.timestamp <= MAX_BATTERY_AGE:
        raise BatteryFault("Battery reading is stale; charge is unavailable")
    if not 0 <= sample.bus_voltage <= 26 or not -0.1 <= sample.input_voltage <= 26:
        raise BatteryFault("INA219 voltage is outside its 0-26 V input range")
    if abs(sample.shunt_voltage) >= 0.0799:
        raise BatteryFault("INA219 shunt reading is at its +/-80 mV limit; current is unreliable")
    return sample


class INA219:
    def __init__(self, bus, address=0x40, shunt_ohms=0.01, clock=time):
        if not 0x40 <= address <= 0x4F:
            raise ValueError("INA219 address must be 0x40 through 0x4F")
        if not math.isfinite(shunt_ohms) or shunt_ohms <= 0:
            raise ValueError("Shunt resistance must be positive and finite")
        self.bus, self.address, self.shunt_ohms, self.clock = bus, address, shunt_ohms, clock

    def read_register(self, register):
        raw = self.bus.read_i2c_block_data(self.address, register, 2)
        if len(raw) != 2:
            raise BatteryFault("Incomplete INA219 register read")
        return (raw[0] << 8) | raw[1]

    def write_register(self, register, value):
        self.bus.write_i2c_block_data(self.address, register, [value >> 8, value & 0xFF])

    def initialize(self):
        # INA219 has no WHO_AM_I register. Only access the configured address;
        # readback checks configuration, not definitive device identity.
        config = self.read_register(0)
        if config & 0xC000:
            raise BatteryFault(f"Unexpected INA219 configuration 0x{config:04X} at 0x{self.address:02X}; "
                               "check device type, address and I2C wiring")
        self.write_register(0, INA_CONFIG & ~7)  # Hold in power-down before first trigger.
        actual = self.read_register(0)
        if actual != INA_CONFIG & ~7:
            raise BatteryFault(f"INA219 configuration readback failed: wrote 0x{INA_CONFIG & ~7:04X}, "
                               f"read 0x{actual:04X} at 0x{self.address:02X}")

    def read_sample(self):
        started = self.clock.monotonic()
        self.write_register(0, INA_CONFIG)  # Each write starts a fresh, paired conversion.
        deadline = started + 0.25
        while True:
            raw_bus = self.read_register(2)
            if self.clock.monotonic() >= deadline:
                raise BatteryFault(f"INA219 conversion timed out at 0x{self.address:02X}; "
                                   f"last bus register 0x{raw_bus:04X}, configuration 0x{INA_CONFIG:04X}")
            if raw_bus & 2:
                break
            self.clock.sleep(0.005)
        raw_shunt = self.read_register(1)
        if raw_shunt & 0x8000:
            raw_shunt -= 0x10000
        shunt_voltage = raw_shunt * 0.00001
        # OVF refers to the chip's calibrated current/power math. We use the
        # raw voltage registers and our own shunt scaling instead.
        sample = BatterySample(started, (raw_bus >> 3) * 0.004, shunt_voltage,
                               shunt_voltage / self.shunt_ohms)
        return check_battery_sample(sample, self.clock.monotonic())


class BatteryStream:
    """All INA219 I/O happens in a worker so the control loop never waits on I2C."""

    def __init__(self, bus_number=1, address=0x40, shunt_ohms=0.01,
                 bus_factory=open_bus, clock=time, period=0.1):
        self.bus_number, self.address, self.shunt_ohms = bus_number, address, shunt_ohms
        if not math.isfinite(period) or period <= 0:
            raise ValueError("INA219 sampling period must be positive and finite")
        self.period = period  # the readout uses 0.1 s; the power guard samples faster
        self.bus_factory, self.clock = bus_factory, clock
        self._sample = self._error = None
        self._lock = threading.Lock()
        self._done = threading.Event()
        self._thread = threading.Thread(target=self._run, name="ina219-reader", daemon=True)
        self._thread.start()

    def _run(self):
        bus = None
        try:
            bus = self.bus_factory(self.bus_number)
            sensor = INA219(bus, self.address, self.shunt_ohms, self.clock)
            # Retry transient bus errors only during initialization, with outputs
            # still off. Mid-motion read errors must reach the guard immediately.
            for attempt in range(3):
                try:
                    sensor.initialize()
                    break
                except OSError:
                    if attempt == 2:
                        raise
                    if self._done.wait(0.05):
                        return
            while not self._done.is_set():
                sample = sensor.read_sample()
                with self._lock:
                    self._sample = sample
                self._done.wait(self.period)
        except Exception as exc:
            with self._lock:
                self._error = (f"INA219 on I2C bus {self.bus_number}, address 0x{self.address:02X}, "
                               f"shunt {self.shunt_ohms:g} ohm: {type(exc).__name__}: {exc}")
        finally:
            if bus is not None:
                try:
                    bus.close()
                except Exception:
                    pass

    def latest(self):
        with self._lock:
            sample, error = self._sample, self._error
        if error is not None:
            raise BatteryFault(error)
        return sample

    def close(self):
        self._done.set()
        self._thread.join(timeout=0.25)
        return not self._thread.is_alive()


@dataclass(frozen=True)
class BatteryProfile:
    name: str
    cells: int
    # Per-cell voltage versus percentage at light load. The bundled generic
    # profile is provisional; replace it with pack measurements for accuracy.
    voltage_percent: tuple
    low_cell_voltage: float
    max_estimate_current: float

    def __post_init__(self):
        if type(self.cells) is not int or not 1 <= self.cells <= 6:
            raise ValueError("Battery cells must be an integer from 1 to 6")
        points = self.voltage_percent
        if len(points) < 2 or any(len(p) != 2 for p in points):
            raise ValueError("Battery profile needs voltage/percentage pairs")
        if not all(math.isfinite(v) and math.isfinite(p) and 0 < v <= 4.5 and 0 <= p <= 100
                   for v, p in points):
            raise ValueError("Battery profile has invalid voltage/percentage values")
        if points[0][1] != 0 or points[-1][1] != 100:
            raise ValueError("Battery profile must run from 0 to 100 percent")
        if any(b[0] <= a[0] or b[1] <= a[1] for a, b in zip(points, points[1:])):
            raise ValueError("Battery profile voltage and percentage must increase")
        if (not math.isfinite(self.low_cell_voltage) or self.low_cell_voltage <= 0
                or not math.isfinite(self.max_estimate_current) or self.max_estimate_current <= 0):
            raise ValueError("Battery low-voltage and estimation-current limits must be positive")

    def estimate(self, voltage):
        if not math.isfinite(voltage):
            raise ValueError("Battery voltage must be finite")
        cell_voltage = voltage / self.cells
        if cell_voltage <= self.voltage_percent[0][0]:
            return 0.0
        for (v0, p0), (v1, p1) in zip(self.voltage_percent, self.voltage_percent[1:]):
            if cell_voltage <= v1:
                return p0 + (p1 - p0) * (cell_voltage - v0) / (v1 - v0)
        return 100.0


def load_battery_profile(path):
    if path is None:
        return None
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return BatteryProfile(data["name"], data["cells"], tuple(map(tuple, data["voltage_percent"])),
                          data["low_cell_voltage"], data["max_estimate_current"])


def battery_status(sample, profile=None):
    voltage = sample.input_voltage
    readings = (f"Battery at sensor: {voltage:.2f} V | Current: {sample.current:+.3f} A | "
                f"Power: {sample.power:+.2f} W")
    if profile is None:
        return readings + " | Charge: unavailable (battery profile not configured)"
    if voltage < profile.cells * 2.0 or voltage > profile.cells * 4.3:
        return readings + " | Charge: unavailable (voltage does not match battery profile)"
    low = " | LOW BATTERY" if voltage <= profile.cells * profile.low_cell_voltage else ""
    if sample.current < -0.02:
        return readings + " | Charge: unavailable (reverse current/charging; voltage estimate unreliable)" + low
    if sample.current > profile.max_estimate_current:
        return readings + " | Charge: unavailable (load too high for voltage estimate)" + low
    percent = profile.estimate(voltage)
    bars = round(percent / 10)
    # Display coarse steps instead of suggesting a precise coulomb-counted SOC.
    percent = int(round(percent / 5) * 5)
    return readings + f" | Charge estimate: ~{percent}% [{'#' * bars}{'-' * (10 - bars)}]" + low


class BatterySession:
    def __init__(self, bus=1, address=0x40, shunt_ohms=0.01, profile=None, stream_factory=BatteryStream):
        self.bus, self.address, self.shunt_ohms = bus, address, shunt_ohms
        self.profile, self.stream_factory, self.stream = profile, stream_factory, None

    def close(self):
        if self.stream is not None and self.stream.close() is False:
            return False
        self.stream = None
        return True

    def start(self, period=0.1):
        """One tracked INA219 owner shared by the battery readout and the motion power guard."""
        if not self.close():
            raise BatteryFault("Previous INA219 read is still blocked; restart the program before another INA219 test")
        if period == 0.1:
            self.stream = self.stream_factory(self.bus, self.address, self.shunt_ohms)
        else:
            self.stream = self.stream_factory(self.bus, self.address, self.shunt_ohms, period=period)
        return self.stream

    def readings(self, control):
        control.motors.stop(brake=False)
        try:
            control.poll()
            self.start()
            control.log(f"INA219 battery monitor: bus {self.bus}, address 0x{self.address:02X}, "
                        f"shunt {self.shunt_ohms:g} ohm. Motors disabled.")
            if self.profile:
                control.log(f"Profile: {self.profile.name}. Charge percentage is a rough voltage estimate; "
                            "let the battery settle after driving.")
            else:
                control.log("Voltage/current are available. Charge percentage needs a battery profile.")
            deadline = control.clock.monotonic() + 2.0
            next_print = 0.0
            while True:
                now = control.clock.monotonic()
                control.poll()
                sample = self.stream.latest()
                if sample is None:
                    if now >= deadline:
                        raise BatteryFault("INA219 produced no reading; check battery sensor address/wiring")
                else:
                    check_battery_sample(sample, now)
                    if now >= next_print:
                        control.log(battery_status(sample, self.profile))
                        next_print = now + 1.0
                control.tick_sleep(now)
        finally:
            control.motors.stop()
            self.close()

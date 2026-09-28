"""MPU-6050-compatible I2C input and relative gyro heading for short tests.

Only WHO_AM_I == 0x68 is supported. No configuration writes occur before
identity validation. Source: TDK MPU-6000/MPU-6050 Register Map, rev. 4.2.
No hardware is accessed on import. Sensor I/O runs outside the motor loop.
"""

from dataclasses import dataclass
import math
import statistics
import struct
import threading
import time

MAX_SAMPLE_AGE = 0.15
CALIBRATION_LIFETIME = 300.0


class IMUFault(Exception):
    pass


@dataclass(frozen=True)
class Sample:
    sequence: int
    timestamp: float
    acceleration: tuple  # g, sensor X/Y/Z
    gyro: tuple  # degrees/second, sensor X/Y/Z
    temperature: float  # degrees C


def check_sample(sample, now):
    if sample is None:
        raise IMUFault("No IMU sample is available")
    values = (*sample.acceleration, *sample.gyro, sample.temperature, sample.timestamp)
    if len(sample.acceleration) != 3 or len(sample.gyro) != 3 or not all(map(math.isfinite, values)):
        raise IMUFault("Invalid IMU sample")
    age = now - sample.timestamp
    if age < -0.01 or age > MAX_SAMPLE_AGE:
        raise IMUFault("IMU data is stale; movement stopped")
    if max(map(abs, sample.gyro)) >= 490:
        raise IMUFault("Gyroscope is near its configured range limit")
    if sum(v * v for v in sample.acceleration) < 0.01:
        raise IMUFault("IMU acceleration data is invalid or the robot is in free fall")
    return sample


class MPU6050:
    def __init__(self, bus, address=0x68, clock=time):
        self.bus = bus
        self.address = address
        self.clock = clock
        self.sequence = 0

    def initialize(self):
        identity = self.bus.read_byte_data(self.address, 0x75)
        if identity != 0x68:
            raise IMUFault(f"Unsupported IMU: WHO_AM_I=0x{identity:02X} at 0x{self.address:02X}. "
                           "This test requires an MPU-6050-compatible ID of 0x68.")
        self.bus.write_byte_data(self.address, 0x6B, 0x80)  # reset
        self.clock.sleep(0.1)
        deadline = self.clock.monotonic() + 1.0
        while self.bus.read_byte_data(self.address, 0x6B) & 0x80:
            if self.clock.monotonic() >= deadline:
                raise IMUFault("IMU reset did not complete")
            self.clock.sleep(0.01)
        settings = (
            (0x6B, 0x01),  # wake, X gyro PLL clock
            (0x6C, 0x00),  # enable every accel/gyro axis
            (0x1A, 0x03),  # digital low-pass filter, approximately 44 Hz gyro
            (0x19, 9),     # 1 kHz / (1 + 9) = 100 Hz
            (0x1B, 0x08),  # gyro +/-500 degrees/s, 65.5 LSB/(degree/s)
            (0x1C, 0x00),  # accel +/-2 g, 16384 LSB/g
            (0x23, 0x00),  # FIFO disabled
            (0x6A, 0x00),  # internal I2C master and FIFO disabled
            (0x37, 0x00),  # status clears when INT_STATUS is read
            (0x38, 0x01),  # data-ready status; no interrupt wire required
        )
        for register, value in settings:
            self.bus.write_byte_data(self.address, register, value)
        self.clock.sleep(0.1)
        for register, value in settings:
            if self.bus.read_byte_data(self.address, register) != value:
                raise IMUFault(f"IMU configuration readback failed at register 0x{register:02X}")

    def read_sample(self):
        started = self.clock.monotonic()
        if not self.bus.read_byte_data(self.address, 0x3A) & 0x01:
            return None  # Never publish old register contents as a fresh sample.
        raw = self.bus.read_i2c_block_data(self.address, 0x3B, 14)
        if len(raw) != 14:
            raise IMUFault("Incomplete IMU register burst")
        ax, ay, az, temperature, gx, gy, gz = struct.unpack(">7h", bytes(raw))
        self.sequence += 1
        sample = Sample(self.sequence, started,
                        tuple(v / 16384.0 for v in (ax, ay, az)),
                        tuple(v / 65.5 for v in (gx, gy, gz)),
                        temperature / 340.0 + 36.53)
        return check_sample(sample, self.clock.monotonic())


def open_bus(number):
    try:
        from smbus2 import SMBus
    except ImportError:
        try:
            from smbus import SMBus
        except ImportError as exc:
            raise IMUFault("Install python3-smbus (or smbus2 in your Python environment)") from exc
    try:
        return SMBus(number)
    except FileNotFoundError as exc:
        raise FileNotFoundError(
            exc.errno, f"/dev/i2c-{number} not found. I2C is disabled or the bus number is wrong: "
            "enable it with 'sudo raspi-config' > Interface Options > I2C, reboot, "
            "then check 'ls /dev/i2c-*' and pass --bus N if needed") from exc


class IMUStream:
    """One I2C owner. A blocked read cannot block the motor/stop-key loop."""

    def __init__(self, bus_number=1, address=0x68, bus_factory=open_bus, clock=time):
        self.bus_number = bus_number
        self.address = address
        self.bus_factory = bus_factory
        self.clock = clock
        self._sample = None
        self._error = None
        self._lock = threading.Lock()
        self._done = threading.Event()
        self._thread = threading.Thread(target=self._run, name="imu-reader", daemon=True)
        self._thread.start()

    def _run(self):
        bus = None
        try:
            bus = self.bus_factory(self.bus_number)
            sensor = MPU6050(bus, self.address, self.clock)
            sensor.initialize()
            while not self._done.is_set():
                sample = sensor.read_sample()
                if sample is not None:
                    with self._lock:
                        self._sample = sample
                self._done.wait(0.005)
        except Exception as exc:
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
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
            raise IMUFault(error)
        return sample

    def close(self):
        self._done.set()
        # The worker closes its own bus after any outstanding syscall returns.
        self._thread.join(timeout=0.25)
        return not self._thread.is_alive()


@dataclass
class Calibration:
    bias: tuple
    gravity: tuple
    timestamp: float
    axis: object = None
    sign: int = 1

    def require_ready(self, now, orientation=True):
        if not 0 <= now - self.timestamp <= CALIBRATION_LIFETIME:
            raise IMUFault("Calibration expired; run IC again")
        if orientation and self.axis is None:
            raise IMUFault("Run IV to verify the yaw axis and direction by hand first")


@dataclass(frozen=True)
class CalibrationReport:
    count: int
    duration: float
    timestamp: float
    bias: tuple
    gravity: tuple
    gyro_noise: tuple
    accel_noise: tuple

    @property
    def acceleration_magnitude(self):
        return math.sqrt(sum(v * v for v in self.gravity))

    def summary(self):
        def xyz(values):
            return "/".join(f"{value:+.3f}" for value in values)
        return (f"Calibration: {self.count} samples over {self.duration:.2f}s\n"
                f"  Gyro mean XYZ [deg/s]: {xyz(self.bias)}\n"
                f"  Gyro noise XYZ [std dev, deg/s]: {xyz(self.gyro_noise)} (limit 1.000)\n"
                f"  Accel mean XYZ [g]: {xyz(self.gravity)}; magnitude {self.acceleration_magnitude:.3f} g\n"
                f"  Accel noise XYZ [std dev, g]: {xyz(self.accel_noise)} (limit 0.025)")

    def warnings(self):
        if not 0.85 <= self.acceleration_magnitude <= 1.15:
            return (f"Acceleration warning: stationary magnitude is {self.acceleration_magnitude:.3f} g "
                    "(expected 0.85-1.15 g). Acceleration accuracy is unverified. "
                    "IC calibrates gyro bias only; acceleration is NOT corrected.",)
        return ()

    def calibration(self):
        failures = []
        for axis, noise in zip("XYZ", self.gyro_noise):
            if noise > 1.0:
                failures.append(f"gyro {axis} noise {noise:.3f} deg/s exceeds 1.000")
        for axis, noise in zip("XYZ", self.accel_noise):
            if noise > 0.025:
                failures.append(f"accel {axis} noise {noise:.3f} g exceeds 0.025")
        for axis, bias in zip("XYZ", self.bias):
            if abs(bias) > 20:
                failures.append(f"gyro {axis} offset {bias:+.3f} deg/s exceeds +/-20.000")
        if failures:
            raise IMUFault("Gyro calibration rejected: " + "; ".join(failures)
                           + ". Keep the robot still and retry IC; persistent offsets need investigation.")
        # Relative heading integrates the gyro only. A stable acceleration offset
        # must not be confused with motion, or silently 'fixed' from one pose.
        return Calibration(self.bias, self.gravity, self.timestamp)


def calibration_report(samples):
    if len(samples) < 50 or len({s.sequence for s in samples}) != len(samples):
        raise IMUFault("Not enough distinct IMU samples for calibration")
    for sample in samples:
        check_sample(sample, sample.timestamp)
    for previous, sample in zip(samples, samples[1:]):
        if sample.sequence <= previous.sequence or not 0 < sample.timestamp - previous.timestamp <= MAX_SAMPLE_AGE:
            raise IMUFault("IMU samples are discontinuous during calibration; retry IC")
    duration = samples[-1].timestamp - samples[0].timestamp
    if duration < 1.5:
        raise IMUFault("Calibration needs at least 1.5 seconds of stationary data")
    gyro_axes = list(zip(*(s.gyro for s in samples)))
    accel_axes = list(zip(*(s.acceleration for s in samples)))
    bias = tuple(statistics.mean(axis) for axis in gyro_axes)
    gravity = tuple(statistics.mean(axis) for axis in accel_axes)
    return CalibrationReport(len(samples), duration, samples[-1].timestamp, bias, gravity,
                             tuple(statistics.pstdev(axis) for axis in gyro_axes),
                             tuple(statistics.pstdev(axis) for axis in accel_axes))


def calibrate_samples(samples):
    return calibration_report(samples).calibration()


def verify_axis(calibration, angles):
    """Operator rotates chassis LEFT by hand, without tilting it."""
    axis = max(range(3), key=lambda i: abs(angles[i]))
    primary = abs(angles[axis])
    others = math.sqrt(sum(angles[i] ** 2 for i in range(3) if i != axis))
    gravity_norm = math.sqrt(sum(v * v for v in calibration.gravity))
    if not 15 <= primary <= 180 or others > primary * 0.25:
        raise IMUFault("Rotate left by about 30-90 degrees without tilting, then press g; retry IV.")
    if abs(calibration.gravity[axis]) / gravity_norm < 0.85:
        raise IMUFault("The measured rotation axis is not vertical. Retry IV without tilting the robot.")
    calibration.axis = axis
    calibration.sign = 1 if angles[axis] > 0 else -1


class Heading:
    """Unwrapped relative gyro angle; positive means a LEFT chassis turn."""

    def __init__(self, calibration, first):
        if calibration.axis is None:
            raise IMUFault("Yaw orientation is not verified")
        self.calibration = calibration
        self.previous = first
        self.angle = 0.0
        self.rate = self._rate(first)

    def _rate(self, sample):
        axis = self.calibration.axis
        return (sample.gyro[axis] - self.calibration.bias[axis]) * self.calibration.sign

    def update(self, sample):
        if sample.sequence == self.previous.sequence:
            return self.angle
        dt = sample.timestamp - self.previous.timestamp
        if sample.sequence < self.previous.sequence or not 0 < dt <= MAX_SAMPLE_AGE:
            raise IMUFault("IMU samples are discontinuous; movement stopped")
        rate = self._rate(sample)
        self.angle += 0.5 * (self.rate + rate) * dt
        self.previous, self.rate = sample, rate
        return self.angle

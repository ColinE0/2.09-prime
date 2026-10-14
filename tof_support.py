"""Optional VL53L0X distance display and FC/CG obstacle gate; no hardware on import.

Driver/status references and pinned dependencies are documented in TOF_SETUP.md.
I2C operations run only in a spawned process; the motor loop reads snapshots.
"""

from dataclasses import dataclass
import math
import multiprocessing
from queue import Empty, Full
import time

TOF_MAX_AGE = 0.30
TOF_START_TIMEOUT = 5.0
MEASUREMENT_TIMEOUT = 0.20
POLL_SECONDS = 0.05
HOLD_MM = 80
RESUME_MM = 90
SLOW_MM = 100
MAX_VALID_MM = 2000
MIN_SPEED_SCALE = 0.40
CLEAR_SECONDS = 0.75
CLEAR_MAX_GAP = 0.15
INVALID_TIMEOUT = 1.0


class ToFFault(Exception):
    pass


@dataclass(frozen=True)
class ToFSample:
    sequence: int
    timestamp: float  # Before requesting this single-shot measurement.
    distance_mm: int
    status: int  # Raw hardware status: 11 = range complete, not ST PAL status 0.

    @property
    def usable(self):
        return self.status == 11 and 0 < self.distance_mm <= MAX_VALID_MM


# VL53L0X device range-status codes, as in ST's API (VL53L0X_DEVICEERROR_*).
# 11 = range complete. These mean nothing returned within range, which is the
# normal case down an open lane: 4 no target, 6 and 9 phase checks (target
# beyond the unambiguous range), 14 below the ignore threshold.
RANGE_COMPLETE = 11
NO_TARGET_STATUSES = frozenset({4, 6, 9, 14})
# 8 (TCC) and 10 (min clip): something inside the sensor's minimum range.
TOO_CLOSE_STATUSES = frozenset({8, 10})


def classify(sample):
    """'range' (usable distance), 'clear' (nothing within range), 'close' or 'unknown'.

    Hardware failures and unexplained codes stay 'unknown': FC/CG holds and then
    faults on them, exactly as before. TOF prints the raw status to check this.
    """
    if sample.status == RANGE_COMPLETE:
        if 0 < sample.distance_mm <= MAX_VALID_MM:
            return "range"
        return "clear" if sample.distance_mm > MAX_VALID_MM else "unknown"
    if sample.status in NO_TARGET_STATUSES:
        return "clear"
    if sample.status in TOO_CLOSE_STATUSES:
        return "close"
    return "unknown"


def check_tof_sample(sample, now):
    if (not isinstance(sample, ToFSample) or type(sample.sequence) is not int
            or sample.sequence < 1 or type(sample.timestamp) not in (int, float)
            or not math.isfinite(sample.timestamp) or type(now) not in (int, float)
            or not math.isfinite(now) or type(sample.distance_mm) is not int
            or not 0 <= sample.distance_mm <= 65535 or type(sample.status) is not int
            or not 0 <= sample.status <= 15):
        raise ToFFault("Invalid ToF worker result")
    if not -0.01 <= now - sample.timestamp <= TOF_MAX_AGE:
        raise ToFFault("ToF reading is stale; run stopped")
    return sample


class VL53L0XDevice:
    """Single shots through Adafruit, with coherent distance + raw-status reads.

    The public range property omits status, so this small adapter uses the pinned
    driver's I2CDevice and interrupt helper. No custom sensor tuning is applied.
    """

    def __init__(self, bus, address):
        self.i2c = self.sensor = None
        try:
            from adafruit_extended_bus import ExtendedI2C
            import adafruit_vl53l0x
        except ImportError as exc:
            raise ToFFault("ToF driver missing. Install requirements-tof.txt in your Pi Python environment; "
                           "see TOF_SETUP.md") from exc
        try:
            self.i2c = ExtendedI2C(bus)
            self.sensor = adafruit_vl53l0x.VL53L0X(
                self.i2c, address=address, io_timeout_s=MEASUREMENT_TIMEOUT)
            # End any prior continuous ranging and discard the single shot the
            # driver's stop method starts. This is not a hardware-reset guarantee.
            self.sensor.stop_continuous()
            self.sensor.read_range()
            self.sensor.measurement_timing_budget = 33000
        except BaseException:
            self.close()
            raise

    def read(self, done, clock=time):
        # Clear old IRQ/cache before requesting a new conversion. Never count a
        # repeated register value as a fresh measurement without a new ready IRQ.
        self.sensor._write_u8(0x0B, 1)
        self.sensor._data_ready = False
        started = clock.monotonic()
        self.sensor.do_range_measurement()
        while not self.sensor.data_ready:
            if done.is_set():
                return None
            if clock.monotonic() - started >= MEASUREMENT_TIMEOUT:
                raise ToFFault("VL53L0X measurement timed out")
            done.wait(0.002)
        if done.is_set():
            return None
        if clock.monotonic() - started >= MEASUREMENT_TIMEOUT:
            raise ToFFault("VL53L0X measurement was delayed")
        result = bytearray(12)
        with self.sensor._device as device:
            device.write(bytes((0x14,)))
            device.readinto(result)
        self.sensor._write_u8(0x0B, 1)
        self.sensor._data_ready = False
        status = (result[0] & 0x78) >> 3
        distance = (result[10] << 8) | result[11]
        return started, distance, status

    def close(self):
        # Single shots end automatically. Do not start a final measurement via
        # stop_continuous(), which the Adafruit API would otherwise do.
        if self.i2c is not None:
            self.i2c.deinit()
            self.i2c = None
        self.sensor = None


def _publish(queue, value):
    try:
        queue.put_nowait(value)
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            pass
        try:
            queue.put_nowait(value)
        except Full:
            pass


def tof_worker(bus, address, queue, done):
    device = None
    try:
        device = VL53L0XDevice(bus, address)
        sequence = 0
        while not done.is_set():
            value = device.read(done)
            if value is None:
                break
            started, distance, status = value
            sequence += 1
            _publish(queue, ToFSample(sequence, started, distance, status))
            done.wait(max(0, POLL_SECONDS - (time.monotonic() - started)))
    except Exception as exc:
        _publish(queue, ("error", f"{type(exc).__name__}: {exc}"))
    finally:
        if device is not None:
            device.close()


class ToFStream:
    def __init__(self, bus=1, address=0x29, context=None, worker=tof_worker):
        context = context or multiprocessing.get_context("spawn")
        self.queue = context.Queue(maxsize=4)
        self.done = context.Event()
        self.process = context.Process(target=worker, args=(bus, address, self.queue, self.done), daemon=True)
        self.sample = self.error = None
        self.closed = False
        self.forced_shutdown = False
        self.process.start()

    def latest(self):
        if self.closed:
            raise ToFFault("ToF worker is closed")
        for _ in range(4):
            try:
                value = self.queue.get_nowait()
            except Empty:
                break
            if isinstance(value, ToFSample):
                self.sample = value
            elif isinstance(value, tuple) and len(value) == 2 and value[0] == "error":
                self.error = value[1]
            else:
                self.error = "Unexpected ToF worker message"
        if self.error:
            raise ToFFault(self.error)
        if not self.process.is_alive():
            raise ToFFault("ToF worker exited; check sensor wiring and dependencies")
        return self.sample

    def close(self):
        if self.closed:
            return True
        self.done.set()
        self.process.join(timeout=0.25)
        if self.process.is_alive():
            self.forced_shutdown = True
            self.process.terminate()
            self.process.join(timeout=0.25)
        if self.process.is_alive() and hasattr(self.process, "kill"):
            self.process.kill()
            self.process.join(timeout=0.25)
        if self.process.is_alive():
            return False
        self.queue.cancel_join_thread()
        self.queue.close()
        self.closed = True
        return True


class ToFGate:
    """Immediate obstacle hold, distance-based slowdown, fresh clear hysteresis."""

    def __init__(self, stream, log=print, invalid_timeout=INVALID_TIMEOUT):
        self.stream, self.log = stream, log
        self.invalid_timeout = invalid_timeout
        self.previous = None
        self.holding = True
        self.clear_since = self.invalid_since = None
        self.clear_count = 0
        self.last_message = None
        self.decision = ("hold", 0.0, "Waiting for fresh clear ToF readings")

    def check(self, now):
        sample = check_tof_sample(self.stream.latest(), now)
        if self.previous is not None:
            if sample.sequence < self.previous.sequence or sample.timestamp < self.previous.timestamp:
                raise ToFFault("ToF results arrived out of order")
            if sample.sequence == self.previous.sequence:
                if sample != self.previous:
                    raise ToFFault("ToF reading changed without a new measurement")
                return self.decision
            if sample.timestamp <= self.previous.timestamp:
                raise ToFFault("ToF measurement timestamp did not advance")
            if sample.timestamp - self.previous.timestamp > CLEAR_MAX_GAP:
                self.clear_since, self.clear_count = None, 0
        self.previous = sample
        kind = classify(sample)
        if kind == "unknown":
            self.holding = True
            self.clear_since, self.clear_count = None, 0
            if self.invalid_since is None:
                self.invalid_since = sample.timestamp
            if (self.invalid_timeout is not None
                    and sample.timestamp - self.invalid_since >= self.invalid_timeout - 1e-9):
                raise ToFFault(f"ToF gave no usable reading for 1s (raw status {sample.status}); run stopped. "
                               "Inspect TOF and sensor alignment.")
            self.decision = ("hold", 0.0, f"ToF unknown (raw status {sample.status}, {sample.distance_mm} mm): hold")
        else:
            self.invalid_since = None
            # Nothing returned within range is the normal open-lane case: farther than
            # every threshold. A target inside the minimum range counts as touching.
            distance = (sample.distance_mm if kind == "range"
                        else 0 if kind == "close" else math.inf)
            if distance <= HOLD_MM:
                self.holding = True
            if self.holding:
                if distance > RESUME_MM:
                    if self.clear_since is None:
                        self.clear_since = sample.timestamp
                    self.clear_count += 1
                    if (self.clear_count >= 3
                            and sample.timestamp - self.clear_since >= CLEAR_SECONDS - 1e-9):
                        self.holding = False
                else:
                    self.clear_since, self.clear_count = None, 0
            if self.holding:
                reason = ("ToF hold: something inside the sensor's minimum range" if kind == "close"
                          else f"ToF hold: need >{RESUME_MM / 10:g} cm clear for {CLEAR_SECONDS:g}s")
                self.decision = ("hold", 0.0, reason)
            else:
                scale = 1.0 if kind == "clear" else min(1.0, MIN_SPEED_SCALE + (1 - MIN_SPEED_SCALE)
                                                         * (distance - HOLD_MM) / (SLOW_MM - HOLD_MM))
                reason = "ToF clear" if scale >= 1 else f"ToF slowing within {SLOW_MM / 10:g} cm"
                self.decision = ("drive", scale, reason)
        if self.decision[2] != self.last_message:
            self.log(self.decision[2])
            self.last_message = self.decision[2]
        return self.decision


class ToFSession:
    def __init__(self, bus=1, address=0x29, stream_factory=ToFStream):
        self.bus, self.address, self.stream_factory = bus, address, stream_factory
        self.enabled = False
        self.stream = None
        self.log = print

    def close(self):
        if self.stream is not None and self.stream.close() is False:
            return False
        if self.stream is not None and getattr(self.stream, "forced_shutdown", False):
            self.log("ToF worker required forced shutdown. If TOF will not reconnect, "
                     "quit and power-cycle the sensor/robot before retrying.")
        self.stream = None
        return True

    def start(self, control):
        self.log = control.log
        control.motors.stop(brake=False)
        control.poll()
        if not self.close():
            raise ToFFault("Previous ToF worker is still blocked; restart before retrying")
        self.stream = self.stream_factory(self.bus, self.address)
        deadline = control.clock.monotonic() + TOF_START_TIMEOUT
        control.log(f"Starting VL53L0X on bus {self.bus}, address 0x{self.address:02X}; motors disabled.")
        while True:
            now = control.clock.monotonic()
            control.poll()
            sample = self.stream.latest()
            if sample is not None:
                return check_tof_sample(sample, now)
            if now >= deadline:
                raise ToFFault("ToF startup timed out; check VL53L0X wiring and requirements-tof.txt")
            control.tick_sleep(now)

    def readings(self, control):
        try:
            self.start(control)
            # Keep the motors-off diagnostic open when a target is not usable.
            gate = ToFGate(self.stream, log=lambda text: None, invalid_timeout=None)
            next_print = 0.0
            control.log("TOF distance display; motors disabled. 's' returns to menu.")
            while True:
                now = control.clock.monotonic()
                control.poll()
                action, scale, reason = gate.check(now)
                if now >= next_print:
                    sample = gate.previous
                    distance = {"range": f"{sample.distance_mm / 10:.1f} cm",
                                "clear": "no target in range",
                                "close": "too close to measure"}.get(classify(sample), "unavailable")
                    control.log(f"ToF: {distance} | raw {sample.distance_mm} mm, status {sample.status} "
                                f"| FC/CG would {action}, speed {scale:.0%}: {reason} "
                                f"| age {now - sample.timestamp:.2f}s")
                    next_print = now + 0.25
                control.tick_sleep(now)
        finally:
            control.motors.stop()
            self.close()

    def attach(self, control, run):
        """Attach obstacle protection to CG; caller owns camera/GO startup ordering."""
        try:
            self.start(control)
            control.tof_gate = ToFGate(self.stream, control.log).check
            return run()
        finally:
            control.motors.stop()
            control.tof_gate = None
            self.close()

    def forward(self, control, vision=None):
        try:
            self.start(control)
            control.tof_gate = ToFGate(self.stream, control.log).check
            if vision is not None and vision.enabled:
                vision.forward(control)
            else:
                control.run_test("FC")
        finally:
            control.motors.stop()
            control.tof_gate = None
            self.close()

"""Isolated MPU6050 calibration and movement tests; hardware opens only on command."""

import time
from motion_profile import PRESET
import motors_optimized as motor
from imu_support import (IMUFault, IMUStream, Heading, MAX_SAMPLE_AGE,
                         calibration_report, check_sample, verify_axis)

# Lower initial duty for the separate IMU movement tests only.
IMU_DRIVE_DUTY = PRESET["imu_drive_duty"]
IMU_DRIVE_SECONDS = 2.0  # total powered duration, including acceleration
HEADING_KP = PRESET["imu_heading_kp"]
HEADING_KD = PRESET["imu_heading_kd"]
MAX_HEADING_TRIM = PRESET["imu_max_heading_trim"]
MAX_HEADING_ERROR = 12.0
TURN_ANGLE = 90.0
TURN_MAX_DUTY = PRESET["imu_turn_max_duty"]
TURN_MIN_DUTY = PRESET["imu_turn_min_duty"]
TURN_SLOW_GAIN = PRESET["imu_turn_slow_gain"]
TURN_TOLERANCE = 2.0
TURN_TIMEOUT = 6.0
TURN_STALL_TIME = 1.0
TURN_STOP_LEAD = 0.0    # s of yaw rate anticipated before cutting power
LOG_TIMED_TURNS = True  # gyro-measure R/L/TR/TL turns when IC and IV are current
CALIBRATION_SECONDS = 2.0
CALIBRATION_SETTLE_SECONDS = 2.0
ORIENTATION_TIMEOUT = 15.0
IMU_COMMANDS = {"ID", "IMU", "IC", "IV", "IF", "ITL", "ITR"}


class Session:
    """IMU connection and in-memory calibration persist between menu selections."""

    def __init__(self, bus=1, address=0x68, stream_factory=IMUStream):
        self.bus = bus
        self.address = address
        self.stream_factory = stream_factory
        self.stream = None
        self.calibration = None

    def close(self):
        self.calibration = None
        if self.stream is not None:
            if self.stream.close() is False:
                return False  # Retain ownership: never start a second I2C worker.
        self.stream = None
        return True

    def connect(self, control):
        if self.stream is None:
            self.stream = self.stream_factory(self.bus, self.address)
        deadline = control.clock.monotonic() + 2.0
        while True:
            started = control.clock.monotonic()
            control.poll()
            sample = self.stream.latest()
            if sample is not None:
                return check_sample(sample, control.clock.monotonic())
            if started >= deadline:
                raise IMUFault("IMU did not produce data. Check I2C bus/address; use ID to retry.")
            control.tick_sleep(started)

    def sample(self, now):
        if self.stream is None:
            raise IMUFault("IMU is not connected; run ID")
        return check_sample(self.stream.latest(), now)

    def require_calibration(self, now, orientation=True):
        if self.calibration is None:
            raise IMUFault("Run IC while the robot is stationary first")
        self.calibration.require_ready(now, orientation=orientation)
        return self.calibration


def forward_targets(angle, rate):
    """Positive gyro yaw is left; a positive left command slows the left side."""
    correction = max(-MAX_HEADING_TRIM,
                     min(MAX_HEADING_TRIM, -HEADING_KP * angle - HEADING_KD * rate))
    scale = IMU_DRIVE_DUTY / max(motor.LEFT_SPEED, motor.RIGHT_SPEED)
    left, right = motor.LEFT_SPEED * scale, motor.RIGHT_SPEED * scale
    if correction > 0:
        left = max(0, left - correction)
    else:
        right = max(0, right + correction)
    return (left, right, left, right)


class IMUTests(motor.Controller):
    def __init__(self, motors, keys, session, clock=time, log=print):
        super().__init__(motors, keys, clock, log)
        self.session = session

    def identify(self):
        if not self.session.close():
            raise IMUFault("Previous IMU read is still blocked. Wait before retrying ID or restart the test.")
        self.session.connect(self)
        self.log(f"MPU-6050-compatible ID 0x68 verified; I2C bus {self.session.bus}, "
                 f"address 0x{self.session.address:02X}. Run IC, then IV.")

    def calibrate(self):
        self.session.calibration = None  # Never retain a previous calibration after a failed attempt.
        self.motors.stop(brake=False)
        self.log("Keep the robot resting completely still: 2 seconds to settle, then 2 seconds to measure. "
                 "Motors disabled. 's' cancels.")
        self.session.connect(self)
        deadline = self.clock.monotonic() + CALIBRATION_SETTLE_SECONDS
        while self.clock.monotonic() < deadline:
            started = self.clock.monotonic()
            self.poll()
            self.session.sample(started)  # Still reject stale/invalid data while settling.
            self.tick_sleep(started)
        self.log("Measuring gyro bias now; keep still.")
        samples = []
        last_sequence = None
        deadline = self.clock.monotonic() + CALIBRATION_SECONDS
        while self.clock.monotonic() < deadline:
            started = self.clock.monotonic()
            self.poll()
            sample = self.session.sample(started)
            if sample.sequence != last_sequence:
                samples.append(sample)
                last_sequence = sample.sequence
            self.tick_sleep(started)
        report = calibration_report(samples)
        self.log(report.summary())
        for warning in report.warnings():
            self.log(warning)
        self.session.calibration = report.calibration()
        bias = self.session.calibration.bias
        self.log(f"Gyro calibration passed. Bias X/Y/Z: {bias[0]:+.3f}/{bias[1]:+.3f}/{bias[2]:+.3f} deg/s. "
                 "Run IV to verify orientation.")

    def verify_orientation(self):
        calibration = self.session.require_calibration(self.clock.monotonic(), orientation=False)
        calibration.axis = None
        self.motors.stop(brake=False)  # Allow hand rotation; never power wheels in IV.
        previous = self.session.connect(self)
        angles = [0.0, 0.0, 0.0]
        self.log("Motors disabled. Turn the whole robot LEFT by hand about 30-90 degrees, "
                 "keeping it flat. Press 'g' when done; 's' cancels.")
        deadline = self.clock.monotonic() + ORIENTATION_TIMEOUT
        while self.clock.monotonic() < deadline:
            started = self.clock.monotonic()
            keys = self.poll()
            sample = self.session.sample(started)
            if sample.sequence != previous.sequence:
                dt = sample.timestamp - previous.timestamp
                if sample.sequence < previous.sequence or not 0 < dt <= MAX_SAMPLE_AGE:
                    raise IMUFault("IMU samples were interrupted during orientation verification")
                for index in range(3):
                    a = previous.gyro[index] - calibration.bias[index]
                    b = sample.gyro[index] - calibration.bias[index]
                    angles[index] += (a + b) * 0.5 * dt
                previous = sample
            if "g" in keys:
                verify_axis(calibration, angles)
                self.log(f"Orientation verified: sensor {'XYZ'[calibration.axis]}, "
                         f"sign {calibration.sign:+d}; positive heading means LEFT.")
                return
            self.tick_sleep(started)
        raise IMUFault("Orientation check timed out; retry IV")

    def readings(self):
        self.motors.stop(brake=False)
        first = self.session.connect(self)
        tracker = None
        calibration = self.session.calibration
        if calibration is not None and calibration.axis is not None:
            calibration.require_ready(self.clock.monotonic())
            tracker = Heading(calibration, first)
        next_print = 0.0
        self.log("Live IMU readings; motors disabled. 's' returns to the menu.")
        while True:
            started = self.clock.monotonic()
            self.poll()
            sample = self.session.sample(started)
            if tracker:
                tracker.update(sample)
            if started >= next_print:
                accel = "/".join(f"{v:+.2f}" for v in sample.acceleration)
                gyro = "/".join(f"{v:+.2f}" for v in sample.gyro)
                heading = f"{tracker.angle:+.1f} deg" if tracker else "run IC and IV first"
                self.log(f"Accel XYZ [g]: {accel} | gyro XYZ [deg/s]: {gyro} | "
                         f"relative yaw: {heading} | {sample.temperature:.1f} C")
                next_print = started + 0.2
            self.tick_sleep(started)

    def heading_tracker(self):
        calibration = self.session.require_calibration(self.clock.monotonic())
        first = self.session.connect(self)
        # Require quiet gyro before starting a powered test; the current pose is zero.
        if max(abs(v - b) for v, b in zip(first.gyro, calibration.bias)) > 5.0:
            raise IMUFault("Robot is moving; set it still before starting the test")
        return Heading(calibration, first)

    def heading_forward(self):
        tracker = self.heading_tracker()
        self.log(f"IMU heading hold for {IMU_DRIVE_SECONDS}s, maximum {IMU_DRIVE_DUTY}% duty. 's' stops.")
        self.prepare(motor.FORWARD)
        started = previous = self.clock.monotonic()
        next_print = started
        while self.clock.monotonic() - started < IMU_DRIVE_SECONDS:
            now = self.clock.monotonic()
            self.poll()
            tracker.update(self.session.sample(now))
            if abs(tracker.angle) > MAX_HEADING_ERROR:
                raise IMUFault("Heading departed by more than 12 degrees; check orientation and tuning")
            targets = forward_targets(tracker.angle, tracker.rate)
            self.motors.slew(targets, motor.RAMP_RATE, now - previous)
            previous = now
            if now >= next_print:
                self.log(f"Heading {tracker.angle:+.1f} deg | rate {tracker.rate:+.1f} deg/s")
                next_print = now + 0.2
            self.tick_sleep(now)
        self.log(f"Heading-hold test finished at {tracker.angle:+.1f} deg")

    def angle_turn(self, direction):
        tracker = self.heading_tracker()
        sign = 1 if direction == "left" else -1
        pattern = motor.TURN_LEFT if sign == 1 else motor.TURN_RIGHT
        self.log(f"IMU {direction} turn: target {TURN_ANGLE:.0f} degrees. 's' stops.")
        self.prepare(pattern)
        start = previous = progress_time = self.clock.monotonic()
        checkpoint = 0.0
        while True:
            now = self.clock.monotonic()
            self.poll()
            tracker.update(self.session.sample(now))
            progress = sign * tracker.angle
            if progress < -3:
                raise IMUFault("Turn went in the wrong measured direction; recheck IV and motor wiring")
            rate_toward = sign * tracker.rate
            if progress + max(0.0, rate_toward) * TURN_STOP_LEAD >= TURN_ANGLE - TURN_TOLERANCE:
                cut_angle, cut_rate = progress, rate_toward
                break
            if now - start >= TURN_TIMEOUT:
                raise IMUFault("IMU turn timed out")
            if progress >= checkpoint + 1.0:
                checkpoint, progress_time = progress, now
            elif now - progress_time >= TURN_STALL_TIME:
                raise IMUFault("No measurable turn progress; movement stopped")
            duty = max(TURN_MIN_DUTY, min(TURN_MAX_DUTY, (TURN_ANGLE - progress) * TURN_SLOW_GAIN))
            scale = duty / max(motor.LEFT_SPEED, motor.RIGHT_SPEED)
            targets = (motor.LEFT_SPEED * scale, motor.RIGHT_SPEED * scale) * 2
            self.motors.slew(targets, motor.RAMP_RATE, now - previous)
            previous = now
            self.tick_sleep(now)
        self.motors.stop()
        # Observe residual rotation after braking, without powering a correction.
        deadline = self.clock.monotonic() + 0.25
        while self.clock.monotonic() < deadline:
            now = self.clock.monotonic()
            self.poll()
            tracker.update(self.session.sample(now))
            self.tick_sleep(now)
        final = sign * tracker.angle
        self.log(f"Turn stopped: measured {final:.1f} degrees; target {TURN_ANGLE:.1f}.")
        coast = final - cut_angle
        if cut_rate > 5.0 and coast > 0.5:
            suggested = coast / cut_rate
            self.log(f"Coast after cutoff: {coast:.1f} degrees from {cut_rate:.0f} deg/s. "
                     f"An IMU turn stop lead of about {suggested:.3f} s (now {TURN_STOP_LEAD:g} s) "
                     f"should land nearer {TURN_ANGLE:.0f}.")

    def run_test(self, command):
        try:
            self.motors.stop()
            self.poll()
            actions = {"ID": self.identify, "IMU": self.readings, "IC": self.calibrate,
                       "IV": self.verify_orientation, "IF": self.heading_forward,
                       "ITL": lambda: self.angle_turn("left"),
                       "ITR": lambda: self.angle_turn("right")}
            actions[command]()
        finally:
            self.motors.stop()


class TurnYawLogger:
    """Read-only gyro measurement of timed turns: R, L, TR and TL.

    Never commands a wheel and never raises into the motor loop: any IMU problem
    ends logging for the rest of the command with one message. Needs a current
    IC and IV; otherwise it says so once and stays idle. Per-turn angles are
    reliable; heading against the plan also carries gyro drift over the run.
    """

    WINDOW = 0.3  # keep integrating after the motors stop, to include the coast

    def __init__(self, session, log=print, clock=time):
        self.session, self.log, self.clock = session, log, clock
        self.tracker = None
        self.active = False
        self.turn = None
        self.window_end = None
        self.expected = 0.0
        self.last_turn_end = 0.0
        self.turns = 0

    def start(self):
        if not LOG_TIMED_TURNS:
            return
        now = self.clock.monotonic()
        try:
            calibration = self.session.require_calibration(now)
            self.tracker = Heading(calibration, self.session.sample(now))
        except Exception as exc:  # measurement is optional; motion must not depend on it
            self.log(f"Turn angles not measured: {exc}. Run IC, then IV, to measure them.")
            return
        self.active = True

    def tick(self):
        if not self.active:
            return
        now = self.clock.monotonic()
        try:
            self.tracker.update(self.session.sample(now))
        except Exception as exc:
            self._stop(f"IMU data interrupted ({exc})")
            return
        if self.window_end is not None and now >= self.window_end:
            self._report()

    def begin_turn(self, direction, label):
        self.tick()
        if not self.active:
            return
        if self.window_end is not None:
            self._report()
        sign = 1 if direction == "left" else -1
        straight = self.tracker.angle - self.last_turn_end
        self.turn = (label, sign, self.tracker.angle, straight)

    def end_turn(self):
        if self.active and self.turn is not None:
            self.window_end = self.clock.monotonic() + self.WINDOW

    def finish(self):
        try:
            if self.active and self.turn is not None:
                self._report()
            if self.active and self.turns > 1:
                self.log(f"Turn log: {self.turns} turns, heading vs plan "
                         f"{self.tracker.angle - self.expected:+.1f} deg (includes gyro drift).")
        except Exception:
            pass
        self.active = False

    def _report(self):
        label, sign, start, straight = self.turn
        turned = sign * (self.tracker.angle - start)
        self.expected += sign * 90.0
        self.turns += 1
        flag = " WRONG DIRECTION" if turned < 0 else ""
        self.log(f"Turn {label}: measured {turned:.1f} deg ({turned - 90.0:+.1f} vs 90){flag} | "
                 f"yaw on the straight before it {straight:+.1f} deg | "
                 f"heading vs plan {self.tracker.angle - self.expected:+.1f} deg")
        self.turn = None
        self.window_end = None
        self.last_turn_end = self.tracker.angle

    def _stop(self, reason):
        self.active = False
        self.turn = None
        self.window_end = None
        self.log(f"Turn angles no longer measured: {reason}.")

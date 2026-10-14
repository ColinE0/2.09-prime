"""FG and CG: IR lane keeping with a gyro heading hold underneath.

The IR lean (FA's state machine, unchanged) moves the robot sideways without
rotating it, so on its own nothing corrects heading. FG adds a PID heading hold
that slows one side of the robot, which rotates without moving it sideways, so
the two do not fight. Each IR correction also nudges the heading target a
little away from that edge, up to REF_MAX from the lane heading. That trims out
a crooked start and slow gyro drift between stops. The gyro offset itself is
re-measured whenever the robot is stopped and still: at FG front lines and CG
intersections.

After a front-line stop, 'g' starts the next lane afresh: the robot's heading
at that moment becomes the lane heading. CG runs a course on FG lanes with
gyro-measured turns; the lane heading carries across each turn (+/-90 degrees).
With the camera on (VT), a confirmed green crosses a front line without stopping
and a red or yellow stops at it until green (re-zeroing the gyro meanwhile); after
its last maneuver CG drives on to the stop sign's front line (Controller.intersection
and final_lane).

Needs a current IC and IV. Any IMU fault stops the run. There is no IR-only
fallback in this bench mode. Heading gains are the IF settings (SET -> IMU
motion); the integral, nudge and re-zero settings are in SET -> Gyro lane.
No hardware is accessed on import.
"""

from collections import deque
import dataclasses
import math
import statistics
import time
from motion_profile import PRESET

import imu_tests as imu
import motors_optimized as motor
from imu_support import Heading, IMUFault

GYRO_LANE_COMMANDS = {"FG", "CG"}
KI = PRESET["gyro_lane_ki"]  # duty per degree-second; 0 leaves IF's proportional-derivative hold
REF_STEP = 0.5               # degrees the heading target moves per IR correction
REF_MAX = 4.0                # the target never moves further than this from the lane heading
REZERO = True                # re-measure the gyro offset whenever the robot is stopped and still
MAX_HEADING_ERROR = 20.0     # beyond this the heading hold has failed (IV sign, wiring or gains)
REZERO_SETTLE = 0.5          # let the robot stop rocking before measuring
REZERO_MIN_SECONDS = 1.0
REZERO_MIN_SAMPLES = 40
REZERO_WINDOW_SAMPLES = 250  # while waiting for 'g', use the most recent readings (about 5 s)
REZERO_MAX_NOISE = 1.0       # deg/s standard deviation, the IC limit
REZERO_MAX_SHIFT = 2.0       # deg/s; a larger change means the robot was moving
CONTINUE_MAX_RATE = 5.0      # deg/s; 'g' waits while someone is still moving the robot
INTEGRATE_MAX_ERROR = 5.0    # deg; a turn's landing error is a transient, not a mismatch to learn
BALANCE_SKIP = 1.0           # the heading must stay within BALANCE_MAX_ERROR this long first
BALANCE_MAX_ERROR = 1.0      # count only settled straight driving, not turn or start transients
BALANCE_MIN_SECONDS = 2.0
COAST_WINDOW = 0.25          # measure rotation after a turn's motors stop
TIMED_FORWARD_MARGIN = 1.0   # extra wall time for the bounded acceleration + hold
TURN_SETTLE_RATE = 5.0       # deg/s; fresh yaw samples must stay below this before turn power
TURN_SETTLE_SECONDS = 0.12   # continuous quiet sample span, with motors stopped
TURN_SETTLE_TIMEOUT = 1.0    # never wait indefinitely for a moving/noisy IMU
TURN_SETTLE_MIN_SAMPLES = 4


class GyroLane(motor.Controller):
    def __init__(self, motors, keys, session, clock=time, log=print):
        super().__init__(motors, keys, clock, log)
        self.session = session
        self.tracker = None
        self.calibration = None
        self.segment = 0.0      # planned lane heading: 0, then +/-90 per course turn
        self.reference = 0.0    # heading target: the lane heading plus IR nudges
        self.integral = 0.0
        self.last_steer = None
        self.settled_since = None
        self.limit_warned = False
        self.measure_balance = True
        self.balance_sum = 0.0
        self.balance_time = 0.0
        self.tof_gate = None

    def poll(self):
        keys = super().poll()
        if self.tracker is not None:
            # Every loop polls, so the heading stays current through stops and waits too.
            self.tracker.update(self.session.sample(self.clock.monotonic()))
        return keys

    def motion_gate(self, pattern):
        """Brake on hold, poll live sensors, then restore direction and ramp from zero."""
        if self.tof_gate is None:
            return 1.0, 0.0
        started = self.clock.monotonic()
        held = False
        while True:
            now = self.clock.monotonic()
            try:
                action, scale, reason = self.tof_gate(now)
                if (action not in ("hold", "drive") or not math.isfinite(scale)
                        or not 0 <= scale <= 1 or not isinstance(reason, str)):
                    raise ValueError("invalid decision")
            except BaseException:
                self.motors.stop()
                raise
            if action == "drive":
                if held:
                    self.motors.set_pattern(pattern)
                    self.last_steer = self.settled_since = None
                return scale, now - started if held else 0.0
            self.motors.stop()
            held = True
            self.tick_sleep(now)
            self.poll()  # Stop keys, camera, power guard and IMU remain live.

    def start_heading(self):
        now = self.clock.monotonic()
        calibration = self.session.require_calibration(now)
        # A private copy: re-zeroing at stops must not change the session's IC result.
        self.calibration = dataclasses.replace(calibration)
        first = self.session.connect(self)
        if max(abs(v - b) for v, b in zip(first.gyro, calibration.bias)) > 5.0:
            raise IMUFault("Robot is moving; set it still before starting the test")
        self.tracker = Heading(self.calibration, first)
        self.segment = self.reference = self.integral = 0.0
        self.balance_sum = self.balance_time = 0.0
        self.last_steer = self.settled_since = None
        self.limit_warned = False

    def steer(self, targets, state, now):
        if self.tracker is None:
            return targets
        error = self.tracker.angle - self.reference
        if abs(error) > MAX_HEADING_ERROR:
            raise IMUFault(f"Heading is {error:+.1f} deg from its target (limit {MAX_HEADING_ERROR:g}). "
                           "Check IV, the wheel wiring and the IMU gains.")
        dt = 0.0 if self.last_steer is None else max(0.0, min(now - self.last_steer, 0.1))
        self.last_steer = now
        driving = any(self.motors.duties)
        if not driving or abs(error) > BALANCE_MAX_ERROR:
            self.settled_since = None
        elif self.settled_since is None:
            self.settled_since = now
        if driving and KI > 0 and abs(error) <= INTEGRATE_MAX_ERROR:
            limit = imu.MAX_HEADING_TRIM / KI  # clamped integrator: no windup past the trim limit
            self.integral = max(-limit, min(limit, self.integral + error * dt))
        # Positive yaw is left; a positive correction slows the left side, as in IF.
        wanted = -(imu.HEADING_KP * error + imu.HEADING_KD * self.tracker.rate + KI * self.integral)
        correction = max(-imu.MAX_HEADING_TRIM, min(imu.MAX_HEADING_TRIM, wanted))
        adjusted = list(targets)
        for index in ((0, 2) if correction > 0 else (1, 3)):
            adjusted[index] = max(0.0, adjusted[index] - abs(correction))
        # The steady correction on a settled straight is the left/right mismatch. Starts,
        # turn landings and saturation would read as mismatch, so only settled time counts.
        if (self.measure_balance and state == "straight" and self.settled_since is not None
                and now - self.settled_since >= BALANCE_SKIP and abs(wanted) < imu.MAX_HEADING_TRIM):
            # Mismatch scales with duty: rescale slow-approach corrections to baseline duty.
            self.balance_sum += correction * dt / self.approach_scale
            self.balance_time += dt
        return tuple(adjusted)

    def lane_event(self, direction, now):
        # A right correction means the left sensor saw the edge: aim a little right.
        wanted = self.reference + (-REF_STEP if direction == "right" else REF_STEP)
        self.reference = max(self.segment - REF_MAX, min(self.segment + REF_MAX, wanted))
        self.log(f"Heading target {self.reference - self.segment:+.1f} deg from the lane")
        if self.reference != wanted and not self.limit_warned:
            self.limit_warned = True
            self.log(f"Heading target is at its {REF_MAX:g} deg limit. If this side keeps correcting, "
                     "the gyro has drifted or the robot is crooked: stop, then run IC again.")

    def hold_forward(self, duration, *, speed_pct=100.0):
        """Hold after the base acceleration budget, independent of changing gyro trim.

        IR is ignored, like the original timed moves. Count capped slew ticks
        toward acceleration, and stop if delayed ticks exceed the wall deadline.
        """
        self.prepare(motor.FORWARD)
        self.approach_scale = speed_pct / 100.0
        straight = tuple(value * self.approach_scale for value in motor.straight_targets())
        previous = self.clock.monotonic()
        ramp_seconds = max(abs(target - current) for target, current in
                           zip(straight, self.motors.duties)) / motor.RAMP_RATE
        ramp_elapsed = 0.0
        reached_at = previous if ramp_seconds <= 1e-9 else None
        deadline = previous + ramp_seconds + duration + TIMED_FORWARD_MARGIN
        while True:
            started = self.clock.monotonic()
            self.poll()
            before_gate = self.clock.monotonic()
            speed_scale, paused = self.motion_gate(motor.FORWARD)
            now = self.clock.monotonic()  # a delayed poll must not use the old tick time
            if paused:
                # A brake requires a fresh acceleration budget, not just extra wall time.
                if reached_at is not None:
                    duration = max(0.0, duration - (before_gate - reached_at))
                ramp_seconds = max(straight) / motor.RAMP_RATE
                ramp_elapsed, reached_at = 0.0, None
                deadline += paused + ramp_seconds
                previous = now
            if now >= deadline:
                self.motors.stop()
                raise motor.LaneFault("Gyro timed forward exceeded its acceleration/hold deadline; "
                                      "check control-loop timing before retrying")
            if reached_at is not None and now - reached_at >= duration - 1e-9:
                return
            self.motors.slew(tuple(value * speed_scale for value in self.steer(straight, "straight", now)),
                             motor.RAMP_RATE, now - previous)
            ramp_elapsed += max(0.0, min(now - previous, motor.LOOP_DT))
            previous = now
            if reached_at is None and ramp_elapsed >= ramp_seconds - 1e-9:
                reached_at = now
            self.tick_sleep(started)

    def settle_turn_start(self):
        """Brake until fresh yaw samples are quiet; preserve all integrated heading."""
        self.motors.stop()
        started = self.clock.monotonic()
        initial_angle = self.tracker.angle
        last = self.tracker.previous.sequence
        quiet_since, quiet_samples = None, 0
        while True:
            tick = self.clock.monotonic()
            self.poll()  # Keep stop key, camera, power guard and IMU freshness active.
            now = self.clock.monotonic()
            if now - started >= TURN_SETTLE_TIMEOUT:
                raise IMUFault(f"Turn start did not settle within {TURN_SETTLE_TIMEOUT:g}s; "
                               f"yaw rate {self.tracker.rate:+.1f} deg/s. Motors remain stopped; "
                               "check IMU mounting, vibration and stationary IC.")
            sample = self.tracker.previous
            if sample.sequence != last:
                last = sample.sequence
                if sample.timestamp >= started:
                    if abs(self.tracker.rate) > TURN_SETTLE_RATE:
                        quiet_since, quiet_samples = None, 0
                    else:
                        if quiet_since is None:
                            quiet_since = sample.timestamp
                        quiet_samples += 1
                        if (quiet_samples >= TURN_SETTLE_MIN_SAMPLES and
                                sample.timestamp - quiet_since >= TURN_SETTLE_SECONDS - 1e-9):
                            return now - started, self.tracker.angle - initial_angle
            self.tick_sleep(tick)

    def measured_turn(self, direction, label):
        """Turn onto the next lane heading, measured by the gyro."""
        sign = 1 if direction == "left" else -1
        self.log(f"Turn {label}: settling with motors stopped (turn-settle v2)")
        settled_seconds, settling_yaw = self.settle_turn_start()
        self.segment += sign * 90.0
        self.reference += sign * 90.0  # a crooked-start nudge applies to every lane of the grid
        self.limit_warned = False
        self.log(f"Turning {direction} (measured)")
        self.prepare(motor.TURN_LEFT if sign == 1 else motor.TURN_RIGHT)
        # Quiet braking yaw belongs to the approach. Keep it in absolute heading
        # and lane target, but measure powered-turn progress from this point.
        start_angle = self.tracker.angle
        self.log(f"Turn {label} start: settled {settled_seconds:.2f}s, "
                 f"settling yaw {settling_yaw:+.1f} deg, rate {self.tracker.rate:+.1f} deg/s; "
                 f"heading {start_angle:+.1f}, target {self.reference:+.1f} deg")
        start = previous = progress_time = self.clock.monotonic()
        checkpoint = 0.0
        while True:
            tick = self.clock.monotonic()
            self.poll()
            speed_scale, paused = self.motion_gate(motor.TURN_LEFT if sign == 1 else motor.TURN_RIGHT)
            now = self.clock.monotonic()
            if paused:
                start += paused
                progress_time += paused
                previous = now
            progress = sign * (self.tracker.angle - start_angle)
            if progress < -3:
                elapsed = now - start
                moved = self.tracker.angle - start_angle
                duty = max(self.motors.duties)
                self.motors.stop()
                raise IMUFault("Turn went in the wrong measured direction; "
                               f"requested {direction}, yaw moved {moved:+.1f} deg "
                               f"in {elapsed:.2f}s, rate {self.tracker.rate:+.1f} deg/s, "
                               f"commanded PWM {duty:.1f}%. Recheck IV, wheel response and IMU mounting.")
            remaining = sign * (self.reference - self.tracker.angle)
            if remaining - max(0.0, sign * self.tracker.rate) * imu.TURN_STOP_LEAD <= imu.TURN_TOLERANCE:
                break
            if now - start >= imu.TURN_TIMEOUT:
                raise IMUFault("Measured turn timed out")
            if progress >= checkpoint + 1.0:
                checkpoint, progress_time = progress, now
            elif now - progress_time >= imu.TURN_STALL_TIME:
                raise IMUFault("No measurable turn progress; movement stopped")
            duty = max(imu.TURN_MIN_DUTY, min(imu.TURN_MAX_DUTY, remaining * imu.TURN_SLOW_GAIN))
            scale = duty / max(motor.LEFT_SPEED, motor.RIGHT_SPEED)
            targets = (motor.LEFT_SPEED * scale * speed_scale, motor.RIGHT_SPEED * scale * speed_scale) * 2
            self.motors.slew_turn(targets, now - previous)
            previous = now
            self.tick_sleep(tick)
        self.motors.stop()
        self.wait(COAST_WINDOW)
        turned = sign * (self.tracker.angle - start_angle)
        self.log(f"Turn {label}: measured {turned:.1f} deg | "
                 f"{self.tracker.angle - self.reference:+.1f} deg from the new lane target")

    def _yaw_reading(self, readings, last, since):
        """Record the sample poll() just validated and integrated, once, after `since`."""
        sample = self.tracker.previous
        if sample.sequence != last and sample.timestamp >= since:
            readings.append((sample.timestamp, sample.gyro[self.calibration.axis]))
            return sample.sequence
        return last

    def _adopt_offset(self, readings, too_short):
        """Adopt the still robot's yaw offset. Returns True only if the stillness checks pass."""
        if not REZERO:
            return False
        span = readings[-1][0] - readings[0][0] if len(readings) > 1 else 0.0
        if span < REZERO_MIN_SECONDS - 1e-9 or len(readings) < REZERO_MIN_SAMPLES:
            self.log(f"Gyro not re-zeroed: the stop was too short to measure ({too_short})")
            return False
        values = [value for _, value in readings]
        mean, noise = statistics.mean(values), statistics.pstdev(values)
        old = self.calibration.bias[self.calibration.axis]
        if noise > REZERO_MAX_NOISE or abs(mean - old) > REZERO_MAX_SHIFT:
            self.log(f"Gyro not re-zeroed: robot not still (noise {noise:.2f} deg/s, "
                     f"offset change {mean - old:+.2f} deg/s)")
            return False
        bias = list(self.calibration.bias)
        bias[self.calibration.axis] = mean
        self.calibration.bias = tuple(bias)
        self.calibration.timestamp = self.clock.monotonic()  # a fresh offset renews the calibration
        self.log(f"Gyro re-zeroed while stopped: yaw offset moved {mean - old:+.3f} deg/s")
        return True

    def rezero(self, duration):
        """Wait out a CG intersection stop, measuring the yaw gyro offset while still."""
        self.motors.stop()
        settle = min(REZERO_SETTLE, duration)
        self.wait(settle)
        settled_angle = self.tracker.angle
        readings, last = [], None
        started = self.clock.monotonic()
        deadline = started + duration - settle
        while self.clock.monotonic() < deadline - 1e-9:
            now = self.clock.monotonic()
            self.poll()
            last = self._yaw_reading(readings, last, started)
            self.tick_sleep(now)
        if self._adopt_offset(readings, "raise the course stopped wait to at least 1.5 s"):
            self.tracker.angle = settled_angle  # a still robot did not turn: drop the drift integrated meanwhile
        self.calibration.require_ready(self.clock.monotonic())

    def stop_wait(self, duration):
        """CG's intersection stop: measure the yaw offset while still."""
        self.rezero(duration)

    def wait_for_green(self):
        """Wait for green or a terminal sign, re-zeroing the gyro while stopped."""
        self.motors.stop()
        settled_angle = self.tracker.angle  # already stopped for the camera's look
        readings = deque(maxlen=REZERO_WINDOW_SAMPLES)
        last = None
        started = self.clock.monotonic()
        while True:
            now = self.clock.monotonic()
            self.poll()
            last = self._yaw_reading(readings, last, started)
            decision = self.light.decision()
            if decision in ("green", "stop_sign"):
                break
            self.tick_sleep(now)
        if self._adopt_offset(list(readings), "the red light lasted under 1 s; the old offset is kept"):
            self.tracker.angle = settled_angle  # a still robot did not turn: drop the drift integrated meanwhile
        self.calibration.require_ready(self.clock.monotonic())
        return decision

    def wait_for_continue(self):
        """At an FG front line: re-zero while still, then 'g' starts the next lane afresh."""
        self.motors.stop()
        readings = deque(maxlen=REZERO_WINDOW_SAMPLES)
        last = None
        since = self.clock.monotonic() + REZERO_SETTLE
        while True:
            started = self.clock.monotonic()
            keys = self.poll()
            last = self._yaw_reading(readings, last, since)
            if "g" in keys:
                if abs(self.tracker.rate) <= CONTINUE_MAX_RATE:
                    break
                self.log("Robot is moving; hold it still, then press 'g'")
            self.tick_sleep(started)
        self._adopt_offset(list(readings), "wait at least 1.5 s before pressing 'g'")
        self.calibration.require_ready(self.clock.monotonic())
        # The robot was stopped, or placed by hand: its heading now is the lane heading.
        self.tracker.angle = 0.0
        self.segment = self.reference = 0.0
        self.limit_warned = False
        self.log("Heading re-referenced: the robot's current heading is the new lane heading")

    def lane_forward(self):
        """FA's forward loop with a re-zero and re-reference at every front line."""
        while True:
            if self.drive_to_line(auto=True, ignore_initial_line=True) == "crossed":
                continue
            stopped_at = self.clock.monotonic()
            if self.light is not None:
                decision = self.look_at_line()
                if decision == "green":
                    self.light.went_on()  # the robot never moved: the lane heading carries on
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
            self.wait_for_continue()
            if self.light is not None:
                self.light.went_on()

    def course(self, sequence):
        for index, command in enumerate(sequence, 1):
            self.log(f"[{index}/{len(sequence)}] Driving to intersection (gyro lane)")
            self.intersection(command, self.drive_to_line(allow_unlit=True))
            if command == "F":
                if motor.COURSE_CROSS_TIME > 0:
                    self.hold_forward(motor.COURSE_CROSS_TIME)
            else:
                direction = "right" if command == "R" else "left"
                self.log(f"Approaching intersection at {motor.TURN_APPROACH_SPEED_PCT:g}% of normal")
                self.hold_forward(motor.TURN_APPROACH_TIME_LEFT if direction == "left"
                                  else motor.TURN_APPROACH_TIME_RIGHT,
                                  speed_pct=motor.TURN_APPROACH_SPEED_PCT)
                self.measured_turn(direction, f"{index} {command}")
        if self.light is not None:
            self.final_lane()
            return
        self.hold_forward(motor.COURSE_EXIT_STRAIGHT_TIME)
        self.go_end()
        self.log("Gyro course complete")

    def report(self, balance=True):
        if self.tracker is None:
            return
        self.log(f"Heading at the end: {self.tracker.angle - self.segment:+.1f} deg from the lane "
                 f"(target {self.reference - self.segment:+.1f}; includes gyro drift)")
        if not self.measure_balance:
            self.log("Auto-balance is measured by FG on a straight lane; CG's short straights and turns "
                     "do not settle long enough.")
            return
        if self.balance_time < BALANCE_MIN_SECONDS and self.go_run():
            # A GO run ends at the front line and the next press starts afresh: nothing adds up.
            self.log(f"Auto-balance: only {self.balance_time:.1f}s of settled straight driving "
                     f"(needs {BALANCE_MIN_SECONDS:g}s). GO runs do not add up; measure balance with FG "
                     "from the menu, pressing 'g' between passes.")
            return
        if self.balance_time < BALANCE_MIN_SECONDS:
            self.log(f"Auto-balance: only {self.balance_time:.1f}s of settled straight driving "
                     f"(needs {BALANCE_MIN_SECONDS:g}s). It adds up across stops: at the front line, carry "
                     "the robot back to the start, set it down still for 2 seconds, press 'g' and run "
                     "the straight again.")
            return
        if not balance:
            self.log("Auto-balance skipped: the run ended on a fault, so its corrections do not "
                     "measure the motors.")
            return
        mean = self.balance_sum / self.balance_time
        if abs(mean) < 0.1:
            self.log(f"Auto-balance: sides matched within 0.1 duty points over {self.balance_time:.1f}s of "
                     "settled straight driving; no baseline change suggested.")
            return
        left, right = motor.LEFT_SPEED, motor.RIGHT_SPEED
        if mean > 0:
            left -= mean  # the hold slowed the left side, so the left side runs fast
        else:
            right += mean
        self.log(f"Auto-balance: over {self.balance_time:.1f}s of settled straight driving the gyro held the "
                 f"{'left' if mean > 0 else 'right'} side {abs(mean):.1f} duty points lower on average. "
                 f"Suggested FA/C baselines: left {left:.1f}, right {right:.1f} "
                 f"(now {motor.LEFT_SPEED:g}/{motor.RIGHT_SPEED:g}). Set them in SET -> Motor motion "
                 "once two runs agree." + self.correction_note())

    @staticmethod
    def correction_note():
        # Since v9.5: the suggestion sits on top of whatever PCAL factors were driving, so it
        # only holds while those same factors stay applied.
        if any(f != 1.0 for f in motor.WHEEL_FACTORS_FORWARD + motor.WHEEL_FACTORS_REVERSE):
            return (" Measured with saved wheel corrections applied; re-run FG after a new PCAL "
                    "or if corrections are turned off.")
        return " Measured with no wheel corrections applied."

    def run(self, command, sequence=""):
        """Every path ends with motors off and the run's heading summary."""
        clean = False
        try:
            self.motors.stop()
            self.poll()
            if command == "CG":
                sequence = "".join(sequence.upper().split())
                if not sequence or any(char not in "FRL" for char in sequence):
                    raise ValueError("Course must contain only F, R, and L, e.g. FRL")
            elif command != "FG":
                raise ValueError("Unknown gyro lane mode: " + command)
            self.start_heading()
            self.measure_balance = command == "FG"
            self.log(f"{command}: IR lane keeping with the gyro holding heading. 's' stops.")
            if command == "FG":
                self.lane_forward()
            else:
                self.course(sequence)
            clean = True
        except motor.StopRequested:
            clean = True  # the operator's stop is a normal end of an FG run
            raise
        finally:
            self.motors.stop()
            try:
                self.report(balance=clean)
            finally:
                self.tracker = None

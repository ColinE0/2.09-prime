"""Separate confidence-filtered IR lane tests; no hardware on import.

The existing PCB convention is 0 = white/edge and 1 = black/lane for
the side sensors. Front 0 is an immediate stop, unless the camera (VT) has a
recent green: then FC notes the line and drives across it. A red or yellow
stops FC at the front line until green; the wait does not use up FC's time
limit. Scores measure recent white-reading support, not distance, probability
of correctness, or health.
"""

from dataclasses import dataclass
import math
import time
from motion_profile import PRESET

import motors_optimized as motor

CONFIDENCE_COMMANDS = {"IRC", "FC"}
FILTER_SECONDS = 0.06
WARMUP_SECONDS = 0.12
EDGE_ENTER = 0.70
EDGE_EXIT = 0.30
MAX_SAMPLE_GAP = 0.12
MAX_DUTY = PRESET["fc_max_duty"]
UNCERTAIN_DUTY = PRESET["fc_uncertain_duty"]
AMBIGUOUS_TIMEOUT = 0.50
CORRECTION_TIMEOUT = motor.AUTO_MAX_CORRECT  # 0.8 s; includes ambiguous interruptions.
CLEAR_RESET_SECONDS = 0.10
FORWARD_SECONDS = 10.0
DISPLAY_SECONDS = 0.25


@dataclass(frozen=True)
class ConfidenceState:
    raw: tuple
    scores: tuple
    edges: tuple
    action: str

    def description(self):
        names = {"warming": "collecting readings", "clear": "lane clear",
                 "right": "correct right", "left": "correct left",
                 "uncertain": "lane unclear"}
        left, right, front = self.raw
        return (f"IR raw L/R/F={left}/{right}/{front} | white support "
                f"L={self.scores[0]:.0%} R={self.scores[1]:.0%} | {names[self.action]}"
                + (" | FRONT STOP" if front == 0 else ""))


class ConfidenceTracker:
    def __init__(self):
        self.first = self.previous = None
        self.scores = [0.5, 0.5]  # Unknown until enough real samples have arrived.
        self.edges = [False, False]

    def update(self, raw, now):
        if len(raw) != 3 or any(value not in (0, 1) for value in raw) or not math.isfinite(now):
            raise motor.LaneFault("Invalid IR reading or timestamp")
        if self.first is None:
            self.first = self.previous = now
        else:
            dt = now - self.previous
            if not 0 < dt <= MAX_SAMPLE_GAP + 1e-9:
                raise motor.LaneFault("IR sampling was interrupted; confidence test stopped")
            alpha = -math.expm1(-dt / FILTER_SECONDS)
            for index in range(2):
                self.scores[index] += alpha * ((1 - raw[index]) - self.scores[index])
            self.previous = now
        for index, score in enumerate(self.scores):
            if score >= EDGE_ENTER:
                self.edges[index] = True
            elif score <= EDGE_EXIT:
                self.edges[index] = False
        if now - self.first < WARMUP_SECONDS - 1e-9:
            action = "warming"
        elif self.edges[0] and not self.edges[1] and self.scores[1] <= EDGE_EXIT:
            action = "right"
        elif self.edges[1] and not self.edges[0] and self.scores[0] <= EDGE_EXIT:
            action = "left"
        elif not any(self.edges) and max(self.scores) <= EDGE_EXIT:
            action = "clear"
        else:
            action = "uncertain"
        return ConfidenceState(tuple(raw), tuple(self.scores), tuple(self.edges), action)


def confidence_targets(state, driving=True):
    cap = UNCERTAIN_DUTY if state.action == "uncertain" else MAX_DUTY
    baseline = motor.straight_targets()
    scale = cap / max(baseline)
    straight = tuple(value * scale for value in baseline)
    if state.action == "warming" or (state.action == "uncertain" and not driving):
        return (0.0,) * 4
    if state.action in ("right", "left"):
        index = 0 if state.action == "right" else 1
        strength = 0.35 + 0.65 * max(0.0, min(1.0, (state.scores[index] - EDGE_EXIT) / (1 - EDGE_EXIT)))
        lean = motor.diagonal_targets(state.action, motor.AUTO_SECONDARY_SPEED, trim=0)
        return tuple(a + strength * (b * scale - a) for a, b in zip(straight, lean))
    return straight


class ConfidenceTests(motor.Controller):
    def __init__(self, motors, keys, clock=time, log=print, tof_gate=None):
        super().__init__(motors, keys, clock, log)
        self.tof_gate = tof_gate

    def read_inputs(self):
        self.poll()
        started = self.clock.monotonic()
        raw = self.motors.read_ir()
        now = self.clock.monotonic()
        if now - started > MAX_SAMPLE_GAP:
            raise motor.LaneFault("IR read was delayed; confidence test stopped")
        return raw, now

    def readings(self):
        self.motors.stop(brake=False)
        tracker = ConfidenceTracker()
        next_print = 0.0
        self.log("IR confidence display. Motors disabled. 0=white, 1=black. 's' returns to menu.")
        self.log("White support is recent reading consistency, not sensor accuracy or distance.")
        while True:
            raw, now = self.read_inputs()
            state = tracker.update(raw, now)
            if now >= next_print:
                self.log(state.description())
                next_print = now + DISPLAY_SECONDS
            self.tick_sleep(now)

    def forward(self):
        self.motors.stop(brake=False)
        self.go_ready()  # a GO run counts down here, after the camera and ToF have started
        tracker = ConfidenceTracker()
        start = previous = self.clock.monotonic()
        next_print = 0.0
        unsettled_since = ambiguous_since = clear_since = None
        driving = False
        passing = False  # the front sensor is on a line FC is going on across
        self.log(f"FC confidence lane test: up to {FORWARD_SECONDS:g}s, maximum {MAX_DUTY:g}% duty. "
                 "Start clear of the front stop line. 's' stops.")
        while True:
            raw, now = self.read_inputs()
            if raw[2] == 1:
                passing = False
            elif not passing:
                # Front detection bypasses filtering, startup, and every steering state.
                if self.light is not None and self.light.crossing(now):
                    self.light.went_on()
                    passing = True
                    self.log("Green light: crossing the front line without stopping")
                else:
                    self.motors.stop()
                    decision = None
                    if self.light is not None:
                        decision = self.look_at_line()
                        start += self.clock.monotonic() - now  # a light stop is not driving time
                    if decision == "green":
                        # Go on across the line: a new IR warmup and ramp, as after a hold.
                        self.light.went_on()
                        passing, driving = True, False
                        tracker = ConfidenceTracker()
                        unsettled_since = ambiguous_since = clear_since = None
                        previous = self.clock.monotonic()
                        continue
                    self.go_end(at=now)
                    self.log("Stop sign: stopped at its front line. FC complete." if decision == "stop_sign"
                             else "Front line: confidence test stopped")
                    return
            if now - start >= FORWARD_SECONDS - 1e-9:
                self.motors.stop()
                self.go_end("FC time limit")
                self.log("Confidence test time limit reached")
                return

            # The camera is read in poll() every tick, holds included; only ToF holds mid-lane.
            tof_action, speed_scale, tof_reason = "drive", 1.0, ""
            if self.tof_gate is not None:
                decision = self.tof_gate(now)
                try:
                    tof_action, speed_scale, tof_reason = decision
                    valid = (tof_action in ("drive", "hold") and math.isfinite(speed_scale)
                             and 0 <= speed_scale <= 1 and isinstance(tof_reason, str))
                except (TypeError, ValueError):
                    valid = False
                if not valid:
                    raise motor.LaneFault("Invalid ToF decision; FC stopped")
            if tof_action == "hold":
                # Braking bypasses slew. Any resume gets a new IR warmup/ramp.
                self.motors.stop()
                driving = False
                tracker = ConfidenceTracker()
                unsettled_since = ambiguous_since = clear_since = None
                previous = now
                self.tick_sleep(now)
                continue

            state = tracker.update(raw, now)

            if state.action != "warming":
                if state.action == "clear":
                    if clear_since is None:
                        clear_since = now
                    if now - clear_since >= CLEAR_RESET_SECONDS - 1e-9:
                        unsettled_since = None
                else:
                    clear_since = None
                    if unsettled_since is None:
                        unsettled_since = now
                if state.action == "uncertain":
                    if ambiguous_since is None:
                        ambiguous_since = now
                else:
                    ambiguous_since = None
                if ambiguous_since is not None and now - ambiguous_since >= AMBIGUOUS_TIMEOUT - 1e-9:
                    raise motor.LaneFault("IR lane unclear for 0.5s. Check position and sensor readings in IRC.")
                if unsettled_since is not None and now - unsettled_since >= CORRECTION_TIMEOUT - 1e-9:
                    raise motor.LaneFault("Confidence correction timed out. Check lane position and IR sensors.")
                if not driving and state.action != "uncertain":
                    self.prepare(motor.FORWARD)
                    driving = True

            targets = tuple(value * speed_scale for value in confidence_targets(state, driving))
            rate = motor.AUTO_SLEW_RATE if any(a > b for a, b in zip(self.motors.duties, targets)) else motor.RAMP_RATE
            self.motors.slew(targets, rate, now - previous)
            previous = now
            if now >= next_print:
                self.log(state.description() + f" | max wheel duty={max(self.motors.duties):.0f}%")
                next_print = now + DISPLAY_SECONDS
            self.tick_sleep(now)

    def run_test(self, command):
        try:
            self.motors.stop()
            self.poll()
            if command == "IRC":
                self.readings()
            elif command == "FC":
                self.forward()
            else:
                raise ValueError("Unknown confidence test")
        finally:
            self.motors.stop()

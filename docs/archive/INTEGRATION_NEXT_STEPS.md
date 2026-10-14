> Historical document. For current operation, see the [navigation guide](../../README.md).

# Proposed edits after existing-mode hardening

These are design proposals, not added modes or changed operating policy. Colin's
current instruction is to harden the existing modes first. The three verified
review fixes are implemented in this copy; the steps below would prepare a later
course integration without needing to redesign the bench all at once.

## 1. Make front-line permission one explicit result

Start in `vision_support.py`, with callers in `motors_optimized.py`,
`lane_gyro.py` and `ir_confidence.py`. Replace combinations of `crossing`, `green`,
`decision` and preview assumptions with one result carrying permission, reason,
sample age and whether the robot is already stopped. Keep moving and stopped
policies explicit. RV should show the same reason the running mode would use.

Before changing defaults, settle these course-policy choices:

- Must green remain recent throughout a stopped wait, or may a confirmation since
  arrival remain sufficient after the lamp leaves view? The current code does the latter.
- At an unknown-light intersection, should a competition run wait, require manual
  continuation or use the present course fallback?
- Must completion require a confirmed stop sign, or may the final unconfirmed
  front line finish the run as it currently does?

Acceptance: replay mixed colors, sparse detections, expired frames, a late stop
sign and missing lights. Each mode's output and RV explanation should agree with
the selected policy. Bench behavior should stay available for diagnosis.

## 2. Give gyro lanes an explicit obstacle-hold state

Reuse `tof_support.py`'s existing gate rather than reproducing its thresholds.
The new connection belongs where `lane_gyro.py` and `motors_optimized.py` decide
the next motion, following the working FC pattern in `ir_confidence.py`.
Initially apply it to straight lane travel; turns need a separate decision about
what the forward-facing sensor can actually protect.

A hold must disable outputs immediately, preserve the pending course leg, keep
camera/IMU/power/RESET polling, and resume through a controlled ramp. Specify which
clocks pause: maneuver progress, total run time and sensor-expiry clocks serve
different purposes. Re-zero the gyro only after a stationary window. Stale or
failed ToF data should produce a fault rather than permission to continue.

Acceptance: an obstacle during acceleration, at a stop line, during a red wait
and immediately after green; noisy distances around both thresholds; disconnect
and stale data; RESET during every hold. Check that a course leg advances once.

## 3. Centralize competing stop reasons

Keep emergency/operator and power/sensor faults above all motion permissions.
Represent an obstacle hold, a front-line light wait and terminal completion as
distinct states with one place that authorizes motor output. A green must not
clear a separate obstacle or fault. Include the active reason in logs and GO's
result so a stopped robot is diagnosable.

This is a refactor worth doing after the existing mode tests are stable. It should
first reproduce current behavior; broader policy can then be added as a separate,
reviewable change.

## 4. Add one complete course scenario before a new mode

Extend the current fake rig with ToF and GO events. Exercise a complete route:
START/countdown, lane corrections, obstacle stop/resume, red stop/green release,
measured turn, final sign and timing. Repeat with RESET and power/sensor faults
injected at transitions, asserting no positive PWM after terminal outcomes.

Then validate on each bot with its own saved profile. Record actual stopping
distance, camera detection distance, turn landing error and slowest control tick.
Those measurements should set speed and timeout limits for an integrated mode.
If logs delay control, move high-volume diagnostic output to a bounded queue with
dropped-message counts before increasing camera or logging load.

Recommended order: agree light/completion policy, unify permission reporting,
connect ToF to straight gyro travel with hold tests, then build the complete
course scenario. Add a new integrated mode only after those pieces are verified.

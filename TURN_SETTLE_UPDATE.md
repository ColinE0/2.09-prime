# Measured-turn settling update (turn-settle v2), 2026-10-06

Includes the stop-sign fixes and requested 0.04% traffic-light action default.
A saved/imported profile overrides that default. Stop-sign action remains 1.2%.

## Evidence and limits

Colin's CG RR screenshot is confirmed to come from the previous combined update.
It shows successful re-zeroing, red-to-green handling and a first right turn measured
at 90.1 degrees (+0.8 from target). The second right turn hits the wrong-direction
guard. Final +94.3 degrees is relative to the already-advanced next lane, not a
94.3-degree left turn: the integrated heading is -85.7 degrees, or +4.3 degrees from
the previous nominal lane. The screenshot does not reveal turn duration, motor
response, measured rate or the source of that yaw.

The previous fix excluded yaw during a fixed 50 ms stop. It did not establish that
yaw was quiet before starting the turn. In the simulator two rights pass normally;
injecting opposite yaw beyond 50 ms reproduces the second-turn fault. This is a
reproduced failure path, not proof of the robot's physical root cause.

## Changes

Before each CG measured turn, stop the motors and require fresh gyro readings to
stay within 5 degrees/second for 0.12 seconds (at least four distinct samples).
Keep integrating heading while stopped, then retain the absolute next-lane target.
If quiet is not reached within one second, stop with a settling-specific error.
Stop key, camera freshness, IMU freshness and power checks remain active.

The three-degree wrong-direction guard and the turn stall/timeout limits remain.
A wrong-direction error now stops the motors immediately and reports request,
measured yaw delta, elapsed time, yaw rate and commanded PWM. The latter is the
logical command, not a measurement of actual wheel speed or applied correction.

Each turn prints `(turn-settle v2)` and `Turn 2 R start: settled ...`. These identify
the new revision and give useful diagnostics if the real fault persists.

613 offline tests pass, zero failures/errors/skips. New checks cover a second
right turn with longer residual yaw, an interrupted quiet window, continuous
rotation refusal, stale-data refusal and the stop key during settling. Existing
reversed-turn and stall tests still pass. One existing test now identifies the
completed-turn log explicitly so it does not mistake a new start diagnostic for
the end of the turn. No hardware verification.

## On the robot

Extract beside the old folder and explicitly transfer the working 500 RPM profile.
Run the usual stationary IC/LEFT-hand IV for the new program session. Verify the
active traffic-light action setting is 0.04% if importing an older profile.
Retry the short CG RR route. Keep the full `Turn 2 R start:` line and fault if it
stops again. If it says `Turn start did not settle`, turn power never started;
check stationary IMU readings, mounting and vibration rather than raising limits.

The earlier original, stop-sign and combined copies are unchanged. This ZIP has
all prior fixes. No GitHub change or automatic deployment. Earlier documents and
patches describe their parent revisions; TURN_SETTLE_CHANGES.patch records this
revision against the combined 0.04% copy. Verification and manifest are current.
Actual stop-sign performance remains awaiting camera evidence.

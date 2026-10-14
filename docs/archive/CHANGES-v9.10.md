> Historical document. For current operation, see the [navigation guide](../../README.md).

# v9.10 changes (2026-10-07)

v9.10 is v9.9 plus the five test copies built and tried on 2026-10-06 (`_hardened`,
`_stop_sign_update`, `_stop_sign_turn_update`, `_stop_sign_turn_settle_update` and
`_turn_controls`, each building on the one before). Those copies started from a v9.9 build that
predates the last three v9.9 commits on `ez`, so two of them are re-applied here unchanged:
bot 2's START on GPIO9 with per-robot START/RESET pins, and Kaleb's profile carry-over. The
third, taking out the stop-sign course end, is not: Colin chose the test copies' stop-sign
rules on 2026-10-07 (below). **None of this has a recorded run on either robot.**

## Turns

- **Turn-settle v2.** Before each CG measured turn the motors brake and the turn waits for fresh
  gyro readings under 5 deg/s for 0.12 s (at least 4 samples). If that takes over 1 s, the run
  stops with `Turn start did not settle` and the motors stay off. The log shows
  `Turn N R: settling with motors stopped (turn-settle v2)`, then a start line with the settle
  time, settling yaw, rate, heading and target.
- **Why:** turn progress used to be measured from a heading taken before the robot finished
  braking out of the approach. Yaw from that braking could count as the turn going the wrong way
  and trip `Turn went in the wrong measured direction` with no visible turn, which matches the
  second right turn faulting in a CG RR run on 10-06. Progress is now measured from after the
  settle and the direction switch. That fixes a software path that reproduces the fault offline;
  the physical cause on the robot is not confirmed.
- **The wrong-direction fault** now stops the motors at once and reports the requested direction,
  yaw moved, elapsed time, rate and commanded PWM. The 3 degree guard, stall guard and timeouts
  are unchanged.
- **Turn acceleration** (SET -> Motor motion, `turn_ramp_rate`): how fast power rises in R/L,
  TR/TL, C/CG turns and ITL/ITR. Slowing down still uses Acceleration. Defaults are 35 (500 RPM)
  and 100 (100 RPM); a saved profile without the field keeps its own Acceleration, so nothing
  changes until it is set.
- **Intersection approach speed** (`turn_approach_speed_pct`, 10 to 100%, default 100): scales
  only the forward move between the stop line and a turn in TR/TL and C/CG. Lowering it shortens
  how far the robot gets in the same approach hold, so retune the hold with it. It is separate
  from the line-approach slowdown.

## CG timed moves

- The hold clock for CG's crossings, approaches and exit straight starts after the expected
  acceleration time, instead of waiting for gyro-trimmed duties to match their targets exactly,
  which could keep it from ever starting. A deadline of acceleration + hold + 1 s stops the
  motors with a lane fault if the control loop falls that far behind.

## Traffic lights

- **Newest-frame veto.** A red or yellow in the newest camera frame blocks a green crossing and a
  green restart from a stop, even before it is confirmed in 2 of 3 frames. The camera and stop
  keys are re-read right before a stopped robot decides to go.
- **Light action size default 0.04%** (was 0.08%), at Colin's request on 10-06. A saved profile
  keeps its own value.

## Stop signs

- **The stop-sign course end stays, as in v9.9's first build** (Colin, 2026-10-07). A confirmed
  sign makes the next front line the end of the run in F, FA, FG and FC; with VT, C and CG drive
  a final lane after their last turn and end at the sign's line; a C/CG course with legs left
  stops there and carries on. The last v9.9 commit on `ez` had taken this out.
- **A sign confirmed while waiting at a red or yellow** now ends the run (or is the course stop)
  instead of being dropped until the light turns green.
- **Detector** (`ryan_vision/stop_sign_detector.py`, now changed from Ryan's pinned source; see
  `ryan_vision/SOURCE.md`): searches the whole camera frame instead of the right half, 5 to 80%
  height. White lettering that the shape step fills in no longer counts as lamp glare, and the
  white-core lamp check also needs bright red. Shape, the 1.2% size and 2-of-3 confirmation are
  unchanged. Checked against synthetic signs only, not the real one.

## RVD

- `rv_debug.py` now ships in the release (it was home testing only). RVD also prints why each red
  contour was rejected as a stop sign and boxes the chosen sign in snapshots. Motors stay off.

## Release housekeeping

- Menu and `--help` say v9.10. First-use profile carry-over now looks in `MPU6050_Motor_Test_v9_9`
  and the five test-copy folders first, then Kaleb's folders, then v9.5 and older.
- Left out of the repo from the test copies: their five `.patch` files, `BASELINE.json`,
  `PACKAGE_MANIFEST.json`, `HARDENING_VERIFICATION.json`, `HARDENING_TEST_RESULTS.txt`,
  `TURN_SETTLE_REPRODUCTION.json` and `verify_hardening.py`; git history holds the same diffs, and
  `python -m unittest discover -v` runs the tests. Their notes (`HARDENING.md`,
  `STOP_SIGN_UPDATE.md`, `TURN_TRANSITION_UPDATE.md`, `TURN_SETTLE_UPDATE.md`, `TURN_CONTROLS.md`,
  `INTEGRATION_NEXT_STEPS.md`) are kept as written, so their mentions of patches, zips and
  `verify_hardening.py` point at files that are not here.
- Line endings back to LF. One test now expects the 0.04% default; one new test checks that v9.10
  starts from a saved v9.9 profile ahead of Kaleb's. 630 offline tests pass
  (TEST_RESULTS-v9.10.txt).

## Watch for on the robot

- CG now brakes to a full stop before every turn, including right after a green crossing, so
  expect a short pause at each turn.
- A loose or vibrating IMU can stop a run with `Turn start did not settle`.
- The whole-frame stop-sign search can see red things the old right-half crop skipped, and a
  confirmed sign ends the run. Watch for early endings near red objects.
- A low Turn acceleration can trip `No measurable turn progress` before the wheels break free.
- Kaleb's 500 RPM profiles hold the right-turn approach far shorter than the left (25% file
  0.2 s vs 0.7 s, 35% file 0.01 s vs 0.25 s), so a right turn starts almost on the stop line.
  If right turns still land wrong once they turn the full 90 degrees, look there first.

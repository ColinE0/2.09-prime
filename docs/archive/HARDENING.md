> Historical document. For current operation, see the [navigation guide](../../README.md).

# v9.9 local hardening, 2026-10-06

This is a separate, runnable copy of the existing bench. Colin's scope is to fix
and harden the current modes first, with future integration edits proposed separately.
It contains no Git repository or remote and has not been deployed to either robot.

Baseline: `home-debug`, commit `5249af8f38acb22382ff3a94a99724873cfbd15d`.
`BASELINE.json` records the 81 original files and their SHA-256 hashes.

## Fixes

| Finding | Change | Modes affected |
| --- | --- | --- |
| R1: green can restart despite a newest red/yellow frame | Moving and stopped decisions share a newest-frame hazard veto. A hazard means wait, including before it gets two-frame confirmation. Refresh camera, stop keys and guards immediately before the restart decision. | F, FA, FC, FG, C, CG with VT |
| R4: a stop sign arriving during a red wait can be discarded | The wait returns its prioritized result, green or stop sign. Terminal callers finish at the line without briefly restarting. Gyro waits retain their stopped re-zero behavior. | F, FA, FC, FG and the C/CG final lane |
| R3: gyro trim can prevent a timed move's clock from starting | Start the hold after the base acceleration budget instead of waiting for every changing PID target to match. Bound the acceleration plus hold with a wall-clock deadline; a missed deadline disables motor outputs and raises a fault. | CG timed crossings, approaches and exit segments |

`lane_gyro.py`, `motors_optimized.py` and `vision_support.py` contain the runtime
edits. `test_hardening.py` adds reproducible scenarios against the real controllers
and motor driver with fake clocks, sensors and recorded PWM.

## Preserved behavior

- Intermediate C/CG stop signs still stop and continue to the next planned leg.
- Unknown lights still use each mode's existing fallback. Stopped green keeps the
  existing since-arrival policy; the moving one-second green limit was not extended
  to stopped waits. Those are future policy decisions, not silently changed defaults.
- Camera red/yellow arms the front stop line. ToF holds remain FC-only via TT.
- Power protection, START/RESET, profiles, calibration, motor limits and command
  names retain their existing configuration and behavior.

## Verification and use

On 2026-10-06, **597 tests passed, with no failures, errors or skips**, using Python
3.13.5 with OpenCV available. This includes 14 new regression tests with multiple
mode/color cases. No physical robot was used.

Run `python -B verify_hardening.py` in this folder to execute the offline suite and
write `HARDENING_TEST_RESULTS.txt` and `HARDENING_VERIFICATION.json`. The JSON includes
the tested Python-file hashes. Earlier `TEST_RESULTS-v9.*`, `REVIEW.md` and
`VERIFICATION.json` describe inherited releases, not this hardening pass.

On the Pi, unzip `MPU6050_Motor_Test_v9_9_hardened.zip` beside the existing project,
enter that new folder, then use `python3 main.py --show-settings` and the existing
launch procedure. The menu retains the underlying v9.9 version label. The copy
includes tracked source only; inspect and deliberately transfer the intended
per-bot settings/profile data if the Pi has customized settings. Do not overwrite
the existing folder or assume desktop defaults match either physical bot.

Offline simulation verifies control flow and commanded PWM, not braking distance,
camera accuracy or physical course completion. Before using this copy for a course,
check newest red/yellow while stopped, a sign appearing during a red wait, gyro
timed moves at the chosen ramp rate, and START/RESET on the actual robot. The new
deadline is cooperative: it is checked when the control loop runs, not by an
independent hardware watchdog.

Future edits are described in [INTEGRATION_NEXT_STEPS.md](INTEGRATION_NEXT_STEPS.md).

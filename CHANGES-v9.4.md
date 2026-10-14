# v9.4 changes (2026-09-23)

Built on `MPU6050_Motor_Test_v9_3.zip` unmodified (the first commit in this
folder's local git history is that baseline, so `git diff` shows every change).
Offline tests: 405 pass (v9.3's 365, plus 40 new in `test_v9_4.py`; one
existing settings test now also lists the new module). A 6-agent review ran
before release; its fixes are listed at the end. **Not yet run on either
robot.** FA, FC, C and every other existing command steer as in v9.3; two
stop-safety fixes below apply to all of them.

## Why

FA and FC steer by slowing one diagonal pair of wheels. On mecanum wheels
that moves the robot sideways without rotating it, by design. Nothing in either
mode reacts to heading. A robot pointed a couple of degrees off the lane
(placed crooked, a timed turn that landed short, one side's motors faster)
drifts to one edge, gets moved back sideways, and drifts again. Between edge
hits the IR sensors have no information. The gyro measures exactly the missing
state.

## FG: IR lane keeping with a gyro heading hold

`lane_gyro.py`. FA's IR state machine runs unchanged; FG adds to it:

- **Heading hold** every 20 ms tick: slows one side, which rotates the robot
  without moving it sideways, so it does not fight the lean. It uses IF's gains
  and correction limit (SET -> IMU motion), plus a new **integral** term
  (SET -> Gyro lane). Without the integral, a steady left/right mismatch leaves
  a steady heading offset of mismatch / kp. For example, 1 duty point at kp 0.6
  holds the robot about 1.7 degrees off, so it keeps drifting. The integral only
  accumulates while the heading is within 5 degrees of its target, so a turn's
  landing error does not wind it up. It is also clamped at the correction limit.
- **IR nudges the heading target.** Each IR correction moves the target 0.5
  degrees away from that edge, capped at 4 degrees from the lane heading.
  Repeated hits on one side mean a heading bias, and it gets trimmed out.
  Alternating hits cancel. This absorbs a crooked start and up to 4 degrees of
  gyro drift between stops. Reaching the cap prints a warning once per lane.
  If that side keeps correcting, the gyro has drifted: stop and run IC.
- **Every front-line stop re-zeros the gyro.** While FG waits for `g`, it
  measures the yaw offset from the most recent 5 seconds of still readings.
  `g` then starts the next lane from the robot's current heading, so heading
  integrated while parked, or a robot placed by hand, never steers the next
  lane. `g` waits while the robot is being moved. It fails with "Calibration
  expired" if IC is over 5 minutes old and no re-zero succeeded.
- **Auto-balance** at the end of an FG run: the average heading correction
  over settled straight driving is the left/right mismatch. That means the
  heading within 1 degree of its target for at least 1 second, not in an IR
  lean, and not at the correction limit. FG prints suggested FA/C baselines
  (for example, `left 25.0, right 24.0`). Enter them in SET -> Motor motion once
  two runs agree. There is no suggestion under 2 settled seconds, or after a run
  that ended on a fault. A gyro offset that appeared since IC also reads as
  mismatch, so run IC before a balance run.
- Stops at a front line like FA. It needs a current IC and IV, and refuses to
  start if the robot is moving.

## CG: course on FG lanes with measured turns

- Each straight is FG. Approach, crossing and exit holds keep the heading
  hold. As in C, those holds ignore the IR sensors.
- Turns use the gyro, with ITL/ITR's duty, slowdown, tolerance, timeout, stall
  and stop-lead settings. A turn aims at the next lane heading (+/-90 degrees
  from the last), not "90 degrees from wherever it is", so a heading error
  carried into the intersection is corrected by the turn.
- During each intersection stop (course stopped wait, default 3 s) the gyro's
  yaw offset is re-measured after 0.5 s of settling. Heading integrated while
  still is dropped. It is skipped with a message if the robot is not still or
  the stop is under 1.5 s. The session's IC result is not changed.
- Each turn prints `Turn <n> <R/L>: measured X deg | Y deg from the new lane
  target`. CG does not suggest a motor balance: its short straights and turns
  do not settle long enough. Use FG on a straight lane for that.

## Stop safety, all powered commands (pre-existing since v8)

- **Ctrl+S, Ctrl+Z and Ctrl+\\ now stop the motors like `s`.** Before, Ctrl+S
  froze terminal output. The next log line then blocked the control loop with
  the wheels still at their duty, and the IR stop, power guard and stop key all
  went dead. Ctrl+Z suspended the program and Ctrl+\\ killed it, both with STBY
  high and the PWM pins latched. While a test runs, the terminal now delivers
  all three as keys.
- **A closed SSH window, `kill`, or SIGQUIT now runs the Ctrl+C cleanup.**
  Before, these ended the program without running any cleanup, leaving the
  motors in whatever state they were last driven. Now the first such signal stops
  the motors, drops STBY and closes GPIO. Later signals are ignored so they
  cannot cut that cleanup short. SIGKILL (`kill -9`) and power loss cannot be
  caught: the battery plug is still the hardware stop.

## Faults

- Any IMU fault (stale data, discontinuity, near range limit) stops FG/CG with
  the motors off. There is **no IR-only fallback**. Whether a demo mode should
  fall back instead is an open decision.
- The heading more than 20 degrees from its target stops the run (wrong IV sign,
  wiring or gains).
- The power guard checks every FG/CG tick, like every other powered command.

## Settings (SET -> Gyro lane (FG/CG))

| Setting | 500 RPM | 100 RPM |
|---|---|---|
| Heading integral (duty/degree-second) | 0.15 | 0.3 |
| Heading target nudge per IR correction (deg) | 0.5 | 0.5 |
| Heading target nudge limit (deg) | 4 | 4 |
| Re-zero the gyro when stopped (FG lines, CG stops) | ON | ON |

These are starting values, not tuned on either robot. Tune the heading gains
with IF first, then use FG. FG and CG also use FA's IR steering settings, the
manual baselines, and the IMU turn settings; their SET help now says so.

## Review fixes (6-agent review, 2026-09-23)

Three reviewers (gyro lane code, safety and hardware state, integration) each
had a verifier try to refute their findings. Eight were confirmed; all are fixed.

| Finding | Severity | Fix |
|---|---|---|
| FG kept integrating gyro drift while parked at a front line and never re-zeroed; after a long wait `g` steered the robot off the lane | high | Re-zero and re-reference at every front line; calibration age checked at `g` |
| Ctrl+Z, Ctrl+\\, SSH hangup or `kill` left STBY high and PWM latched with no stop | high | Keys delivered as stop keys; signals run the Ctrl+C cleanup |
| Ctrl+S froze the control loop with the wheels powered | medium | Flow control off during tests; Ctrl+S stops |
| Auto-balance still printed after a fault, pointing the wrong way after a wrong IV sign | low | No suggestion after a fault |
| Auto-balance counted turn landings and restarts as mismatch | low | Settled-only counting; FG only; integral separation |
| Tests passed with most CG and re-zero behaviour broken (16 of 17 single-line changes survived) | low | 22 new tests; 29 of 29 single-line changes now caught |
| Re-zero validated samples against a clock read taken before `poll()` | low | Uses the sample `poll()` just validated |
| SET help called IR-steering, baseline and turn settings "FA/C only" or "ITL/ITR" | low | Help and README updated |

Five other findings were refuted by their verifiers, including a claimed
power-guard stale-sample race and integrator windup as the cause of turn
overshoot.

## Also

- Motor test menu, help, README and argparse name v9.4. FG and CG are under
  "Lane following and courses".
- `motors_optimized.Controller` gains two no-op hooks, `steer()` and
  `lane_event()`, called from `drive_to_line()`. FA's targets pass through
  unchanged.
- First-launch profile discovery now checks the sibling v9.3 folder first.
  Older profiles gain the new settings at their defaults.
- `TEST_RESULTS.txt` and `VERIFICATION.json` are v9.3's records;
  `TEST_RESULTS-v9.4.txt` is this revision's run.

## Suggested first run on a robot

After the tape check and the lifted-wheel FL/FR/BL/BR checks:

1. `IC`, then `IV`.
2. `IF` a few times: it should finish near 0 degrees. Tune the IMU heading gains
   here if not.
3. `FG` down a straight lane. Watch for `Heading target` lines and the
   `Auto-balance` line at the end.
4. `CG` with one turn, then the full course.

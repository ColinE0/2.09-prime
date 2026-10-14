> Historical document. For current operation, see the [navigation guide](../../README.md).

# v9.5 changes (2026-09-24)

v9.5 merges two separate v9.4 releases that both started from `MPU6050_Motor_Test_v9_3.zip`:

- **v9.4a, gyro lane keeping (FG, CG).** Built 2026-09-23 in `claude-home\motor-bench`.
  Changelog: [CHANGES-v9.4.md](CHANGES-v9.4.md).
- **v9.4b, removable IR wheel calibration (PPROBE, PCAL, SAVE).** Built by Colin and delivered
  as `MPU6050_Motor_Test_v9_4.zip` in OneDrive. Changelog:
  [CHANGES-v9.4-wheel-calibration.md](CHANGES-v9.4-wheel-calibration.md).

Local git history shows the whole thing: `d1e3caf` is the v9.3 baseline, v9.4a is on
`master`, v9.4b is on branch `ircal` exactly as zipped, and the merge commit holds
everything below. Neither feature was redesigned.

## What the merge changed

- Conflicts resolved as a union of the two releases: the menu dispatch in `app.py`
  recognises both FG/CG and PPROBE/PCAL, and the motor-key filter in
  `robot_settings.py` skips both the `gyro_lane_` and the `tach_`/`wheel_` fields.
- Profile migration now tries sibling `MPU6050_Motor_Test_v9_4/profiles/` first, then
  v9.3, v9.2, v9.1 and v9. A Pi has only one v9.4 folder, so whichever v9.4 was installed
  there is the one that gets imported. Profiles from either v9.4 load: fields the other
  release added fall back to their defaults, and wheel corrections stay OFF until a PCAL
  result is saved.
- Menu banner, CLI description and README now say v9.5. The two v9.4 changelogs are
  both kept under separate names.
- **The one behaviour change:** FG's auto-balance message now says whether saved wheel
  corrections were active during the run. Both features adjust baseline PWM: PCAL
  equalises each wheel with the robot lifted, and auto-balance measures what the gyro
  still had to correct on the floor. The auto-balance figures sit on top of whatever
  PCAL factors were active, so they are only valid while those same factors stay applied.

## Strafe and lane-lean direction fixed (added 2026-09-25)

Ported from Colin's 2026-09-24 fix on the wheel-calibration v9.4 line: SR with the old
pattern moved the 500 RPM bot LEFT on the mat, and FA/FC leans pushed the robot further
onto a lane line. `STRAFE_RIGHT` is now FL/BR forward and FR/BL reverse; a right lean
slows FR+BL and a left lean slows FL+BR. FG uses the same leans, so it gets the fix too.
Colin's updated tests and his new regression (each lean slows exactly the pair its
strafe reverses) are applied. The FG simulator in `test_v9_4.py` had the same mirror in
its sideways model and is corrected. **Check SR on the mat on each bot:** it must slide
right. The 100 RPM bot has not been checked yet.

## One-command launch (added 2026-09-25)

`python3 main.py` from the package folder runs the bench. With plain `python3` it
re-launches itself under `~/robot-test-env` (once, guarded by an environment flag), and
every option passes through. `motor_imu_test.py` still works. 4 tests in `test_main.py`.

## SET reload no longer leaves the motors dead (fixed 2026-09-25)

On the Trixie card, RPi.GPIO is the rpi-lgpio shim. Leaving SET with a change closes the
hardware and opens it again in the same process, and the old close cleaned up only the
bench's own pins, which kept the gpiochip handle open. A PWM pin freed and claimed again
through that handle outputs nothing, so after any SET change every motor command ran at 0%
(ITR stopped with "No measurable turn progress" at about 0.15 A). Reproduced on the
unconnected GPIO8: 48% high at 50% duty in a fresh process, 0% after the bench's close and
reopen, 48% after a full cleanup. `MotorDriver.close()` now stops and drops each PWM object
first, then runs a full `GPIO.cleanup()`, which closes the handle, so the next session opens
a fresh one. BOT (switching robots) used the same path and is fixed too. New test
`test_pwm_still_drives_after_a_set_reload` models the shim and fails on the old close; 473
offline tests pass.

## BTN: board buttons and status LED (added 2026-09-26)

A motors-off readout under Sensor readouts. Nothing in the bench touched the board's UI
pins before this. BTN claims START (GPIO10, header pin 19) and RESET (GPIO9, pin 21) as
pulled-up inputs and the status LED (GPIO4, pin 7) as an output, on first use, and keeps
them until the session closes. Both switches have 10k pull-ups on the board and short to
GND when pressed. A press or release counts after two control ticks (about 40 ms) and
prints as its own line, with a count; leaving with `s` prints the totals. The LED blinks
once a second and stays on while either button is held, then ends off. A button reading
pressed at the start is flagged as a possible short. If a pin cannot be claimed (GPIO9/10
are SPI0 pins, so SPI must stay off), BTN stops with that reason instead of ending the
program. 7 tests in `test_board_ui.py`; 480 offline tests pass. Not yet run on a robot.

## GO: START button runs and the PRD run timer (added 2026-10-04)

`GO` arms the last drive command (or `GO CG FRL` arms a typed one without running it) for
the board's START button, per the PRD's start button, 5 second countdown and trip timer in
tenths. Each debounced START press runs the guard check, a 5 s countdown (LED flash and
terminal count each second; `s`, RESET and the power guard can end it before anything
moves), then the command through the same path as the menu, with the run timer from the
end of the countdown to the moment the command returns. The time heads the live line
(`RUN 12.3 s | ...`) and every event line, prints in large digits at the end, and the best
completed time of the session follows each run. GO stays armed between runs; RESET stops a
countdown or run and stays armed, `s` stops and returns to the menu, and a power stop
disarms. F, FA and FG end at their first front line in a GO run (the keys GO hands in carry
`front_line_ends_run`); menu runs still wait for `g`. FG, CG, IF, ITL and ITR are refused at
the press, with the reason, unless IC and IV are current. The menu's powered-command block
moved unchanged into `execute()` in `app.py` so GO and the menu share it. New module
`go_button.py`; 35 tests in `test_go_button.py`; 517 offline tests pass. Not yet run on a
robot.

Review fixes, same day (a six-agent review: three reviewers, each finding checked by a
skeptic; 17 confirmed, none of them a way to move early or fail to stop):

- RESET stops on its second LOW read in a row, like BTN. The 40 ms debounce missed taps
  under about 60 ms. START keeps 40 ms.
- A run counts as completed only if the wheels drove and the command reached its own end.
  Commands report that end through `Controller.go_end()`: FC's front line and stop sign, the
  F/FA/FG front line, the C and CG course ends. FC's 10 s time limit is a DNF
  (`go_end("FC time limit")`); a run that ends before the wheels drive is not counted.
- FC counts down after its camera and ToF are running (`Controller.go_ready()`), so their
  startup is not on the clock, and the clock stops at `go_end()`, before the sensors close.
  If a wheel is driven while that countdown is still pending, GO stops it as a fault.
- A gyro run needs 15 s left on the IC at the press (the countdown plus margin); the armed
  line shows the IC time left.
- A GO FG run that is too short for auto-balance says to measure balance from the menu,
  not to press `g`.
- 20 more tests, including GO through the real `execute()` (RESET, guard trips, battery
  faults, a blocked INA219, timer placement around a slow guard). Each of 29 targeted code
  mutations now fails the suite. 537 offline tests pass.

## IV kept through IC (added 2026-10-04)

A new IC used to blank the yaw axis that IV had found, so every gyro run needed IV (turning
the robot by hand) again as well as IC. The axis and its sign depend only on how the IMU is
mounted, while IC re-measures the gyro bias, which drifts (hence the 5 minute lifetime). Now
a passed IV is remembered for the session (`Session.orientation`: axis, sign, and which way
gravity points along it) and a later IC reuses it, if gravity says the IMU still sits the
same way up: that axis still vertical (the same 0.85 share IV requires) and pointing the
same way. A remounted or flipped IMU, or a tilted robot, drops it and IC asks for a new IV.
Turning the board flat on the chassis keeps the same yaw axis and sign, so it is safe either
way. A cancelled or failed IV forgets it; a failed IC does not (the mounting is unchanged).
BOT, a settings reload and a restart start fresh. GO's refusals now ask for IV only when it
is needed. 5 tests in `test_imu.py`; 542 offline tests pass. Not yet run on a robot.

## Using the two together

1. With the bot lifted, run PPROBE, then PCAL, then SAVE (see WHEEL_CALIBRATION.md).
2. On the floor, run FG on a straight lane, twice. If the two auto-balance suggestions
   agree, set them in SET -> Motor motion.
3. After any new PCAL, or if corrections are turned off, repeat step 2.

## Verification

With simulated GPIO, IMU, IR and pulse trains, 467 offline tests passed at the merge (471 with the launcher tests): v9.4a's 405, the
57 that v9.4b added, and 5 new ones in `test_v9_5.py`. Those five check that FG drives
through the wheel factors (a deliberate bypass of the factors is caught), the
auto-balance correction note, the v9.4-first profile migration, and that SET and help
carry both features. The run is recorded in `TEST_RESULTS-v9.5.txt`.
**Not yet run on either robot.**

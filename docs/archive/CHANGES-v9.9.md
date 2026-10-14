> Historical document. For current operation, see the [navigation guide](../../README.md).

# v9.9 changes (2026-10-05)

v9.9 builds on v9.5 with GO and IV kept through IC. It is numbered after Kaleb's v9.8 (delivered 2026-10-05 as `MPU6050_Motor_Test_v9_8.zip`,
folder `MPU6050_Motor_Test_v9_7`) so the two never share a name. It takes a few line-following
pieces from v9.8 (listed below) and leaves the rest of v9.8 where it is.

## Camera traffic lights at the front line (VT)

Colin's rule, 2026-10-05: when the camera sees green, the robot notes the front line and
drives across it instead of stopping; when it sees red, the front IR stop line is active again
and the robot stops at it. Implemented as `vision_support.TrafficLight` and used by every lane
mode: F, FA, C, FG, CG and FC.

- **The front stop line is armed by default.** With VT on and no light seen, every front line
  stops the robot exactly as with the camera off. Only a confirmed green overrides it.
- **Green.** A green in 2 of the last 3 fresh camera frames, at its action size, lets the robot
  cross the next front line without stopping. It stays good for 1 s (`GREEN_HOLD`) after its
  last confirming frame, so a lamp that drops out of a frame or two right at the line does not
  stop the robot. The crossing is logged: `Green light: crossing the front line without
  stopping`.
- **Red or yellow** confirmed after a green re-arms the line, so the robot stops at it. Red
  outranks yellow, both outrank green, and a lamp read as red then yellow counts as red, as in
  v9.5's FC gate. A single red or yellow in the newest frame also blocks a crossing.
- **Stopped at a line**, the camera looks for 0.6 s (`LOOK_SECONDS`; in a course, the course
  stop time if longer). Red or yellow seen from the line: the robot waits there until a fresh
  green is confirmed, then drives on across it (`s` and RESET still stop it). Green seen from
  the line: it goes on. Nothing seen: the mode's usual front-line stop.
- **Yellow is treated as red**, per the PRD: "enter intersection only on green".
- **The light just passed cannot carry the robot over the next line.** After crossing (or going
  on after a stop), lights are ignored until none is confirmed, meaning the passed lamp has left
  the frame or shrunk below the action size. At worst that costs a stop and a look at the next
  line; it never skips one.
- **Stop sign** confirmed on the approach (at its own action size) makes the next front line the
  end of the run, without a look: F, FA, FG and FC end there; a C or CG course with legs left
  stops there for its course stop and carries on.
- **Camera courses end at the stop sign's line.** With VT on, C and CG drive one more lane after
  their last maneuver, instead of the timed exit straight, and end at the first front line they
  stop at for good: `Stop sign: course complete` when the sign was confirmed, otherwise
  `Course complete at the final front line; no stop sign confirmed`. PRD: "Upon stopping at the
  stop sign, the AutoBot will have completed the course."
- **CG** crosses on green without its intersection stop or re-zero, and re-zeroes the gyro
  while it waits at a red (`GyroLane.wait_for_green`). FG does the same at its front lines.
- **FC no longer stops mid-lane for a red.** v9.5's FC held wherever it first saw red; now red
  only arms the front line and FC stops at it. The red wait does not count toward FC's 10 s
  limit. A confirmed stop sign ends FC at its front line instead of where the sign was seen.
  ToF holds are unchanged (FC only).
- **Camera off, nothing changes** apart from the front-line debounce below.

Camera startup and faults are as before: motors stay off while it starts, a stale or failed
camera stops the run, and the camera closes at the end of every run.

## Action sizes are per-robot settings (SET -> Camera)

`camera_light_act_pct` (default **0.08%** of frame) and `camera_stop_sign_act_pct` (default
1.2%). 0.08% is Colin's 2026-10-05 pick, until now a Pi-only edit of
`LIGHT_ACT_AREA_FRAC` in `vision_settings.py` that any redeploy would undo; as a profile setting
it survives redeploys. Derived, not measured: on the bench prop it fires about 47 cm out.
Measuring the real lamp at 40 cm is still owed. RV prints both sizes and, for each frame, what a
front line would get: `line: cross`, `wait` (red or yellow), `end` (stop sign) or `stop` (no
light).

## From Kaleb's v9.8

- **Front-line debounce.** A front line now counts on the second white read in a row, one 20 ms
  tick later, so a one-read glitch is not an intersection. Each line logs `Line detected: front
  white N reads in a row, side IR L/R=..., <lane state>; slowest tick N ms`. A side sensor on
  white or a lean at the line points at a lane edge under the front sensor; a slow tick points
  at CPU load.
- **Camera worker at lower CPU priority** (`os.nice(10)`), so the motor loop gets the CPU first
  now that the camera runs during lane driving.
- **Line approach slowdown** (SET -> Line approach): after `approach_slow_after` seconds of a
  drive to a line, the straight and lean duties drop to `approach_speed_pct`. **Off by default
  here** (Kaleb's default is 2 s at 60%); on the 100 RPM bot check that the slowed wheel pair
  still turns at the reduced duty. FG's auto-balance rescales slowed corrections.
- The CG intersection and red-wait structure follows his `GyroLane.intersection` and
  `wait_for_green`, with the rules above. Two deliberate differences: v9.8 treats yellow and
  "no light seen" as go, and ends the final lane wherever the sign is first confirmed; v9.9
  stops for both and ends at the stop sign's line.

Not taken from v9.8: removing FC, IRC, PPROBE and PCAL; renaming FA and C to FIR and CIR; the
500 RPM speed profiles; ToF stops in the IR and gyro lane modes; START on GPIO9 with RESET unused
(v9.8 notes that bot 2's left switch never reads pressed, while bot 1's START on GPIO10 worked on
2026-10-05). Kaleb's tuned `500rpm.json` loads in v9.9 (its `approach_*` keys are now known).

## Other changes

- GO counts down after the camera starts for any camera run (not only FC), so the clock starts
  with the wheels; it stops the clock when the robot stops at its last line, not after the
  camera's look. A gyro run with the camera also needs the camera startup allowance of IC left.
- First-use profile carry-over tries `MPU6050_Motor_Test_v9_5` first, then v9.4 and older.
- Menu banner v9.9; VT now reads "camera traffic lights at front lines (lane modes)"; the status
  line shows `Camera lights (F/FA/C/FG/CG/FC)` and `ToF (FC only)`.

## 2026-10-06: bot 2 starts from Kaleb's tuning

First-use carry-over now looks in Kaleb's folders before v9.5's: `motor9_8`, `motor9_7`,
`motor9_6` (his Pi's names) and `MPU6050_Motor_Test_v9_8` / `_v9_7` (his zip's). On his Pi, v9.9's
500 RPM profile therefore starts from his latest saved `500rpm.json`, which loads unchanged:
`auto_min_correct_left` 0.3 and `_right` 0.2, `auto_secondary_speed` 15, `approach_slow_after` 3 s
at 60%, `course_stop_time` 1 s, turn approach 0.2 s right and 0.7 s left, turn and course exit
straights 1 s. The new camera action sizes take v9.9's defaults. His speed files
(`500rpm-<N>pwm.json`) are not used; v9.9 has no speed profiles. Nothing in his folders is written.
Colin's Pi has none of these folders, so bot 1 still carries over from `MPU6050_Motor_Test_v9_5`.

## 2026-10-06: START and RESET pins per robot (SET -> Wiring)

`button_start` (GPIO10 or GPIO9) and `button_reset` (GPIO9, GPIO10 or off), so GO works on bot 2.
Bot 2's left switch (GPIO10) never reads pressed (Kaleb's v9.8 notes), so the 500 RPM defaults are
START on GPIO9 and RESET off; there `s` on the laptop is the only stop. Bot 1 keeps START GPIO10
and RESET GPIO9, which is what it has always used. The two pins must differ. BTN and the GO
armed message name the pins and drop RESET when it is off.

## Tests

580 offline tests pass (`python -m unittest discover`). New: `test_v9_9.py` (43 tests:
TrafficLight rules, FA/C/CG/FC on the simulated robot with a scripted camera, debounce,
approach slowdown, settings, menu and GO wiring, Kaleb's profile, button pins). Updated: front-line timings one tick later in
`test_motors_optimized.py`; v9.5 FC camera-hold tests rewritten for the line rules in
`test_vision.py`, `test_review_fixes.py` and `test_tof_integration.py`.

**Not run on either robot.**

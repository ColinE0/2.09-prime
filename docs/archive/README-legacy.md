> Historical document. For current operation, see the [navigation guide](../../README.md).

# Navigation branch

This branch contains the complete terminal robot program with CG navigation and optional ToF obstacle protection. Launch with `python3 main.py` from this folder using the robot's configured Python environment.

- Open SET -> ToF obstacle range. Current defaults: stop 8 cm, resume above 9 cm for 0.75 seconds, slowdown within 10 cm. Existing saved settings take precedence; raise slowdown and resume before increasing stop.
- Enable TT after each restart. TT protects FC and CG, including CG lanes, timed intersection moves and measured turns.
- Complete IC/IV as required, then run CG or GO CG with the desired course. VT independently enables camera traffic responses.
- Keep your robot's saved profiles when updating. No on-robot tuning files were supplied with this upload.
- Validation: 169 affected tests pass; full offline suite has 640 passing tests and two known button-test errors. Physical robot validation remains pending.

The guide below documents the supplied program. Historical release notes describe earlier behavior; the CG ToF update in TOF_SETUP.md is current.

---

# Robot Test Bench v9.10: settled gyro turns, turn controls and stop-sign fixes

**v9.10 (2026-10-07)** is v9.9 with the 2026-10-06 test-copy fixes folded in: each CG turn
waits for a still gyro before it starts (the second right turn's "wrong measured direction"
fault), two new SET -> Motor motion controls (Turn acceleration, Intersection approach speed),
a newest-frame red/yellow veto, a deadline on CG's timed moves, a whole-frame stop-sign search
with better glare handling, RVD in the release, and a 0.04% traffic-light default. Bot 2 keeps
START on GPIO9 and Kaleb's profile carry-over. See [CHANGES-v9.10.md](CHANGES-v9.10.md).
**Not run on either robot.**

**v9.9 (2026-10-05)** lets the camera decide each front stop line (VT). The line stays armed by
default; a confirmed green lets F, FA, C, FG, CG and FC cross it without stopping, and a red or
yellow stops the robot at it until green. With VT on, C and CG end at the stop sign's front
line. It also takes the front-line debounce, the camera's lower CPU priority and an optional
line-approach slowdown from Kaleb's v9.8. See [CHANGES-v9.9.md](CHANGES-v9.9.md); the
sections below describe earlier releases where they differ. **Not run on either robot.**

How the light decides a front line (VT on):

| Camera | At the front line |
| --- | --- |
| Green confirmed in the last second, newest frame not red or yellow | Crosses without stopping and logs it |
| Red or yellow | Stops at the line, waits for green, then drives on across it |
| Stop sign confirmed on the approach | Stops at the line: the run ends (a C/CG course with legs left stops, then carries on) |
| Nothing confirmed | Stops, looks 0.6 s, then the mode's usual stop (`g`, the course stop, or the end of a GO run) |

Set the action sizes in SET -> Camera (light 0.04% of frame by default) and check them with RV,
which shows what a front line would get for each frame: `line: cross`, `wait`, `end` or `stop`.

**v9.5 (2026-09-24)** merges the two v9.4 branches that both started from v9.3: the gyro lane modes (FG, CG) and the removable IR wheel calibration (PPROBE, PCAL). Nothing in either was redesigned; see [CHANGES-v9.5.md](CHANGES-v9.5.md) for what the merge touched. **Neither feature has been run on a robot.**

## v9.4a: gyro lane keeping (FG, CG)

**Added 2026-09-23 on top of v9.3** (details in [CHANGES-v9.4.md](CHANGES-v9.4.md)):
`FG`, IR lane keeping with a gyro heading hold underneath, and `CG`, a course on FG
lanes with gyro-measured turns. FA and FC steer only by leaning, which moves the robot
sideways without rotating it, so nothing in them corrects heading. FG holds heading
with the gyro, lets each IR correction nudge the heading target away from that edge,
re-zeros the gyro at every stop, and prints a suggested left/right motor balance
after the run. Both need IC and IV first. FA, FC and C steer as before. For every
powered test, Ctrl+S, Ctrl+Z and Ctrl+\ now stop the motors like `s`, and a closed
SSH window or `kill` now runs the same cleanup as Ctrl+C. 405 offline tests pass,
40 of them new in `test_v9_4.py` against a simulated robot;
8 confirmed review findings are fixed. **Not yet run on either robot.**

## v9.4b: removable IR wheel calibration

**PPROBE** checks a removable FC-51 sensor on J8 / BCM20 with motors off.
**PCAL** measures, adjusts and verifies all four lifted wheels in both directions,
then saves per-wheel corrections for the selected bot. Power down and unplug the
probe after saving; normal driving uses the stored factors. See
[WHEEL_CALIBRATION.md](../../WHEEL_CALIBRATION.md) for setup and the step-by-step workflow,
and [CHANGES-v9.4-wheel-calibration.md](CHANGES-v9.4-wheel-calibration.md) for its changes. These corrections
are an unloaded baseline; floor testing and lower-duty startup checks still matter.

## v9.3 - Ryan's September 23 vision update
Ryan's September 23 rr commit, `1256a573380d4898c04537b883208109c806f0ed`, is included.
Traffic-light detection now uses revised brightness/color scoring, white-core
handling and shape filtering. Camera frames still rotate exactly once in the
menu, according to each bot's saved setting. See [CHANGES-v9.3.md](CHANGES-v9.3.md).
The v9.2 reversal and INA fixes below are retained.

SET changes now apply automatically through a hardware restart when you leave
the editor. Reverse one wheel by entering its code (FL, FR, BL or BR). INA219
startup errors return to the menu, with bus/address and fault details; BAT and
the power guard share one sensor owner. `--check-ina` tests the INA219 without
opening motor GPIO. See [CHANGES-v9.2.md](CHANGES-v9.2.md) for fixes and limits.

This retains v9.1's power guard, wiring defaults, ToF policy and gyro turn
logging. Power protection defaults ON and still requires a working INA219 for
powered tests. Sensor diagnostics and SET do not require a successful guard
check. This revision has not been run on either physical robot.

Choose the **500 RPM bot** or the **100 RPM bot (104 RPM rated)** at startup.
Both use 6 V N20 motors. Each robot has its own saved wiring, camera settings,
motor duties, steering adjustments and maneuver timings. Selecting one does not
change the other's saved settings. This package retains the reviewed wiring,
settings and vision fixes; prior deliveries remain intact.

The [motor wire-color reference](../../WIRING.md) records both bots' current cable
colors, viewed from behind. The colors identify physical motor leads; board
channel assignments and lead polarity still need to be recorded separately.

## Start on the Pi

Transfer the ZIP beside your existing test folders, then use the same environment:

```bash
cd ~
unzip -n MPU6050_Motor_Test_v9_10.zip
cd MPU6050_Motor_Test_v9_10
python3 main.py --show-settings
python3 main.py
```

`python3 main.py` switches itself into `~/robot-test-env` (where the ToF driver
is installed), so there is no need to type the venv path.

The preview prints effective settings for **both** robots without opening
hardware or creating files or folders. A normal launch asks:

```text
SELECT ROBOT
  1. 500 RPM bot
  2. 100 RPM bot (104 RPM rated)
Q Quit | Motors are off
```

Enter `1` or `2`; `Q` or `0` quits. To select directly, use
`--bot 500` or `--bot 100`. Add `--bot` to `--show-settings` to preview just that
robot. The selected robot's name remains visible in the test menu.

Enter **BOT** from any test-menu category to change robots or reload the same
robot's saved settings. BOT stops the motors, closes GPIO and all sensor
sessions, then returns to robot selection. The next selection applies its own
settings before hardware is initialized. A failed cleanup prevents switching.
Every selection starts fresh: camera/ToF toggles are OFF and IMU calibration is
cleared. Run IC and IV again before powered IMU tests.

## Separate settings and imports

Normal launches create and use these files beside the program:

| Robot | Settings file |
|---|---|
| 500 RPM | `profiles/500rpm.json` |
| 100 RPM (104 RPM rated) | `profiles/100rpm.json` |

Both include their own wheel-channel assignments, reversal flags, IR roles,
camera choices, duties, gains and timings. **Leave SET to apply saved edits.**
The app closes the old hardware before loading the same robot again. Calibration
and camera/ToF toggles reset. A cleanup failure prevents the reload. The robot/profile identity tags prevent a file identified as
one robot from being loaded into the other.

On first use of either bot, v9.10 checks the matching tagged profile in sibling folders, first
v9.9's (`MPU6050_Motor_Test_v9_9`, after the 10-06 test copies `MPU6050_Motor_Test_v9_9_turn_controls`,
`_stop_sign_turn_settle_update`, `_stop_sign_turn_update`, `_stop_sign_update` and `_hardened`), then Kaleb's (`motor9_8`, `motor9_7`, `motor9_6`, `MPU6050_Motor_Test_v9_8`, `MPU6050_Motor_Test_v9_7`),
then `MPU6050_Motor_Test_v9_5/profiles/`, `MPU6050_Motor_Test_v9_4/profiles/`, `MPU6050_Motor_Test_v9_3/profiles/`,
`MPU6050_Motor_Test_v9_2/profiles/`, `MPU6050_Motor_Test_v9_1/profiles/` and `MPU6050_Motor_Test_v9/profiles/`.
It copies that bot's saved tuning, reversals and wiring without modifying the
source; the startup lines name the folder it came from. An existing profile in this folder always wins. Custom `--settings` paths disable
automatic discovery. Unmarked old profiles require an explicit import.

If neither per-bot source exists, the 500 RPM bot also checks a marked
`robot_settings.json` beside v9.5, then the sibling folders
`MPU6050_Motor_Test_v8_500rpm_wire_colors`, `MPU6050_Motor_Test_v8_500rpm`, and
`imu-test-v8-500rpm`. These older sources require the 500 RPM identity marker.
Contradictory identity tags are rejected. Without a suitable source, each bot
uses the starting defaults below. Keep old folders beside v9.5 for discovery;
if yours have different names, use an explicit import before first normal use.

To import a known source explicitly, name its destination robot:

```bash
python3 main.py --bot 100 --import-settings ../OLD_FOLDER/robot_settings.json
python3 main.py --bot 500 --import-v7 ../YOUR_V7_FOLDER
```

Imports exit without opening hardware. Add `--show-settings` to preview without
writing. A destination that already exists is never overwritten by an import;
use an unused `--settings /path/to/new-profile.json` if needed. V7 import parses
recognized literal constants without executing the source Python files.

An unmarked legacy import into the 500 RPM profile receives the 25% starting
duties, related secondary duties, ramp rates, trims and IMU gains once; wiring,
camera settings and timings are retained. An unmarked import into the 100 RPM
profile retains its saved tuning and fills missing values from that bot's
defaults. Already marked profiles retain later tuning. When an existing target
file needs identity tags or migration, its original bytes are backed up as
`<name>.pre-profiles.json` (with a numbered suffix if necessary) before saving.
Invalid or out-of-range settings stop startup before hardware is opened.

A custom file requires an explicit bot, for example
`--bot 500 --settings /path/to/my-500.json`. This disables automatic source
discovery for that file. Within the same launch, BOT switching to the 100 RPM bot
uses `profiles/100rpm.json`; switching back uses `my-500.json` again. It never
shares that custom file between the robots.

## Starting motion profiles

| Setting | 500 RPM bot | 100 RPM bot (104 RPM rated) |
|---|---:|---:|
| Manual, FA and course left/right baselines | 25% / 25% | 70% / 69% |
| FA slowed diagonal pair | 14% | 40% |
| Manual diagonal secondary pair | 11% | 30% |
| FC maximum / uncertain duty | 25% / 20% | 50% / 30% |
| Single-wheel test duty | 25% | 35% |
| IMU forward / turn maximum | 25% / 25% | 50% / 50% |
| IMU turn minimum target | 16% | 32% |
| Acceleration / correction slew, duty points/s | 35 / 70 | 100 / 200 |
| IMU heading P / D / maximum trim | 0.6 / 0.075 / 5 | 1.2 / 0.15 / 10 |
| IMU turn slowdown gain, duty points/degree | 0.4 | 0.8 |
| Driver hard ceiling | 70% | 70% |

These are editable starting values. The 100 RPM column restores the earlier
104 RPM motor baseline; it is not new calibration of that assembled robot.
The 500 RPM column is the reduced starting profile from the previous delivery.
Neither column guarantees the same ground speed, reliable starting under load,
or matching turn angles. Do not multiply one robot's tuning by the RPM ratio
and assume the other is calibrated. FC scales the slowed-pair duty to its own
cap; with these defaults its full-correction slowed pair is 14% for the 500 RPM
bot and about 28.6% for the 100 RPM bot.

IMU motion has its own group in SET; the turn minimum cannot exceed its maximum,
and heading trim cannot exceed forward duty. IMU heading P is in duty points per
degree, D in duty points per degree/second, and maximum trim in duty points.
The lower secondary duties and turn floor on the 500 RPM bot need testing under
load. Wiring defaults also remain unconfirmed separately for each robot.

Check wiring first, then brief individual-wheel tests, then short supervised
straight runs. Stop a test if a wheel fails to turn; establish reliable duty and
measure ground speed before course tuning. Timed-turn, approach and correction
hold durations are retained for calibration, not claimed to remain correct with
new motors. A slower PWM ramp does not measure mechanical acceleration.

The 500 RPM profile starts every powered mode at 25% or less. In both profiles,
FC and IMU have independent limits: changing only manual baselines does not
change their maximum duties. The 70% driver ceiling is not stall/current
protection. The separate INA219 power guard inherited from v9.1 is described below.

## Menus

```text
ROBOT TEST MENU v9.10
Active robot: 500 RPM bot
Camera lights (F/FA/C/FG/CG/FC): OFF | ToF (FC only): OFF
IMU: Needs IC
Wheel correction: not calibrated
START button (GO): run a drive command first, or GO CG FRL

  1. Motor tests
  2. Lane following and courses
  3. Sensor readouts (motors off)
  4. Camera
  5. Calibration and settings

0 Back | BOT Select robot | H All commands | Q Quit | Command codes work anywhere
```

Choose a category and an item by number, or enter any existing command directly.
Tests return to the category you were using. `0` goes home, `H` lists all commands,
and `Q` quits. `B` remains the BAT alias. During tests, `s` stops and returns to the
menu; Ctrl+C exits with cleanup.

| Commands | Behavior |
| --- | --- |
| FL / FR / BL / BR | One named wheel forward for 1 second including acceleration; default 25% (500 bot) or 35% (100 bot). Lift wheels. |
| F | Forward to a front IR line. With VT on, the camera decides each line (see the top). |
| SR / SL / DR / DL | Existing timed strafing/diagonal movement. |
| R / L / TR / TL | Existing timed/intersection turns. |
| FA / C FRL | Original IR lane centering and course behavior. With VT on, the camera decides each front line and C ends at the stop sign's line. |
| FG | IR lane keeping with a gyro heading hold, to a front line like FA; `g` continues. Needs IC and IV. Prints a suggested left/right balance at the end. |
| CG FRL | Course on FG lanes with gyro-measured turns and a gyro re-zero at each intersection stop. Needs IC and IV. With VT on, a green crosses without the stop, a red waits (re-zeroing), and the course ends at the stop sign's line. |
| FC | Confidence lane test, 10 seconds, default maximum 25% (500 bot) or 50% (100 bot). With VT on, the camera decides its front line (a red wait does not use up the 10 s); with TT on, ToF holds mid-lane. |
| GO / GO CG FRL | Arms the last drive command (or the one typed after GO) for the board's START button: 5 s countdown on the LED and terminal, then the run with a timer in tenths. Stays armed; RESET stops a run, `s` leaves. See below. |
| IR / IRC | Raw IR / confidence readouts, motors off. |
| IMU / ID | IMU readings / identity, motors off. |
| IC / IV | Stationary gyro calibration (lasts 5 minutes), then orientation verification by hand. A passed IV carries over to later ICs in the same session while the IMU sits the same way up, so after the first IV only IC needs repeating. |
| PPROBE | Check exactly five hand rotations using the removable FC-51 probe, motors off. |
| PCAL | Calibrate lifted wheels forward/reverse with the removable probe; SAVE enables verified per-bot corrections. |
| IF | Separate 2-second IMU heading-hold drive, using the selected bot's IMU limit. |
| ITL / ITR | Separate measured 90-degree IMU turns, using the selected bot's IMU limit. |
| BAT / B | INA219 battery voltage, current and conditional charge estimate, motors off. |
| RV | Camera detections, what a front line would get (`line: cross/wait/end/stop`) and the action sizes, motors off. |
| VT | Toggle camera traffic lights at the front line for F, FA, C, FG, CG and FC. |
| TOF | Distance/raw status and proposed FC action, motors off. |
| BTN | Board START/RESET buttons: prints each debounced press and release, and the status LED blinks once a second (solid while a button is held), motors off. |
| TT | Toggle ToF obstacle responses for FC only. |
| SET | Edit this bot's settings; leave SET to apply through an automatic hardware restart. |
| BOT | Close hardware, then select a robot or reload the same one. |

Camera/ToF toggles start OFF on every robot selection. VT affects the lane modes (F, FA, C,
FG, CG, FC); TT affects FC only. Enabling a toggle does not start a background sensor: each
run starts the camera before it moves and closes it at its end. Timed and manual tests ignore
both. IF does not enable IMU correction in FA or FC; FG and CG are the gyro lane modes. The menu reads cached IMU
calibration state and displays Needs IC, Needs IV, Ready, or Expired.

## GO: START button runs and the run timer

The PRD asks for a start button, a 5 second wait with a visible countdown, and the trip time
in tenths of a second, starting when the countdown ends and stopping at the end of the run.
GO does that from the terminal on the laptop:

1. Run the drive command once from the menu (for example `CG FRL`), or type it after GO
   (`GO CG FRL` arms it without running it). Plain `GO` arms the last drive command.
2. The terminal shows `ARMED: CG FRL` and the status LED blips once a second, motors off.
3. Press START on the robot. The guard checks the pack, then the LED flashes once a second
   and the terminal counts `Starting in 5` to `1`. Nothing moves during the countdown.
4. The timer starts and the command runs exactly as from the menu, under the power guard.
   On a terminal the bottom line reads `RUN 12.3 s | <live readout>` and every event line
   carries the time; with `--plain-output` or `tee` a `RUN` line prints once a second.
5. At the end the time prints in large digits with `Run 1: 12.3 s, completed`. GO stays
   armed: put the robot back on the start line and press START again. The best completed
   time of the session is shown after each run.

Each robot's START and RESET pins are in SET -> Wiring. Bot 1 uses START GPIO10 and RESET GPIO9;
bot 2 uses START GPIO9 with RESET off (its left switch never reads pressed), so on bot 2 `s` is the
only stop. RESET stops a countdown or a run and stays armed (the run shows as DNF with the time it
stopped at); it acts on its second LOW read in a row, like BTN. `s` does the same and returns
to the menu. A power stop disarms GO. F, FA and FG end at their first front line when started
from GO, instead of waiting for `g` (with VT on, the first line they stop at for good: a green
crosses and a red waits for green).

A run counts as completed only if the wheels drove and the command reached its own end: the
front line, the course end, or the stop sign's line. FC's 10 s time limit is a DNF, and a run
that ends before the wheels drive (for example FC or F started on a line) is not counted. A run
with the camera (VT) or FC's ToF starts those sensors first and counts down once they are
running, so the clock starts with the wheels; the clock stops when the robot stops at its last
line (not after the camera's look there), before the sensors shut down. A gyro run with the
camera also needs the camera startup allowance of IC left at the press.

FG, CG, IF, ITL and ITR need a current IC and IV at each START press, with at least 15 s of
the IC's 5 minutes left (IV carries over to a new IC, so between runs only IC is needed); otherwise GO says why and stays armed. The armed line shows how
long the IC has left. CG also re-checks it at every intersection stop, so start a long
course on a fresh IC. GO accepts drive commands only: F FA SR SL R L TR TL DR DL C CG FC FG
IF ITL ITR. The timer is truncated to tenths like a stopwatch.

## Settings editor

Enter SET, select a group, then a setting. The saved value, accepted range and
help text are displayed. Enter a value to validate and save it; blank cancels
that edit. `0` navigates back. Each successful edit is saved immediately in
the selected profile file, whose path is shown in the settings menu. The home
header shows which bot is selected. **Leave SET to apply edits automatically.** Motors are disabled while editing, and GPIO direction/pin ownership
never changes under a running test.

In Wiring -> Motor direction reversed, enter `FL`, `FR`, `BL` or `BR` to flip
only that wheel. Four comma-separated on/off values also work. The motor menu
shows the active channel and reversal for every wheel. A reversal changes
direction, not which physical wheel a channel drives.

Motor channels are editable in Wiring. Each channel (M1A, M1B, M2A, M2B)
must appear exactly once in FL, FR, BL, BR order. Reversal flags remain attached
to logical wheels; after changing channels, check their directions too.

Manual baselines are capped at 70%, and the motor driver rejects any duty over
70% before writing PWM. Existing/imported values over 70% cause a settings error
before hardware opens; review and lower them explicitly. This is the project
ceiling, not a guarantee against stall current or overheating. FC/IMU/single-wheel
settings allow up to 50%; the starting values differ between the two robots as
shown above.

Groups cover Wiring, Motor motion, IR steering, Manual maneuvers, Confidence lane
following, Single-wheel tests, Camera, IMU motion, Gyro lane (FG/CG), Power protection and Wheel calibration. Invalid/nonfinite values, conflicting
IR pins and incompatible settings are rejected. Atomic saving preserves the old
file if a write fails.

Examples:

- Wiring -> Motor channel assignments: `M1A,M1B,M2B,M2A` for the
  v9.1 to v9.5 starting map; saved profiles retain their own assignments. Neither is a new
  measurement of your robot. See [REVIEW.md](REVIEW.md).
- Wiring -> Motor direction reversed: `on,on,on,on` reverses FL/FR/BL/BR.
- Wiring -> IR connector assignments: LEFT, RIGHT, FRONT in that order. Enter
  names such as `J10,J9,J7` or BCM pins. Documented Rev C mapping is J7=21, J8=20,
  J9=26, J10=7. Choose actual physical sensor roles, then verify in IR/IRC.
- IR steering -> Original IR settle interval edits AUTO_SETTLE for FA, C, FG and CG.
- Camera -> Automatic camera exposure: `on`.
- Camera -> Camera startup allowance: `30`.

FA minimum/recovery/settle timings do not affect FC. Secondary wheel-pair duty
affects both FA/C and FC (FC scales it to its cap). LEFT/RIGHT baseline settings
still control wheel pairs. When enabled, saved per-wheel correction factors are
applied at the final PWM output across normal drive modes. PCAL measures one
reference duty; it does not provide continuous RPM feedback while driving.
Lower secondary duty before reducing a baseline below it. Lower uncertain duty
before reducing FC's maximum below it. IMU tests normalize both sides against the
larger baseline to retain the configured IMU cap even if the right baseline exceeds the left.

Preview the effective settings, including migration, without opening hardware or saving:

```bash
python3 main.py --show-settings
```

`--bot 500 --settings /path/to/profile.json` selects a custom file for that robot;
the separate-settings rules above also apply when using BOT afterward.
Bus, IMU/INA219/ToF addresses, shunt and battery-profile CLI options apply to the
**entire program launch**, including after BOT switching. Camera CLI overrides
also remain active across switches and override each bot's saved camera choices.
They are not written into either profile. Relaunch with different options when
the other robot needs different bus/address values. To keep different camera
choices per robot, save them in SET -> Camera and omit the corresponding CLI
overrides. The Camera menu shows the active configuration.

## Display and hardware notes

Recurring readings update one line in an interactive terminal. Camera detection,
confirmation and line decision changes also print a full line. Errors and important
events remain in scrollback. To copy every full reading on a separate line:

```bash
python3 main.py --camera-auto-exposure --plain-output
```

No preview window, mouse, new UI dependency or background dashboard polling is
used. Existing v7 dependencies suffice. For camera dependencies, the system Python
packages are python3-opencv, python3-numpy and python3-picamera2; your venv needs
system-site-packages access as before. See [TOF_SETUP.md](../../TOF_SETUP.md) for ToF setup.

Ryan's September 23 rr detector files are pinned in ryan_vision/SOURCE.md. Light detection uses the upper
half after camera rotation; stop signs use the right half between 5% and 80%
height. New box refinement can change the distance at which a detection crosses
the action size. Test actual props in RV before VT. "No light" means no qualifying
detection was found, not proof that there is no light.

Camera startup keeps motors disabled. Its independent 0.75-second freshness
limit is retained. Fixed exposure/gain are explicitly sent to the camera worker, which runs at
a lower CPU priority than the motor loop. v9.9 replaces v9.5's FC hold policy (a red held FC
wherever it was seen) with the front-line rules at the top of this file: a label counts in 2 of
the latest 3 distinct fresh frames at its action size, red and yellow re-arm the line, and only
a fresh confirmed green ends a red wait. Validate with props in RV before enabling VT.

ToF thresholds: slow within 10 cm, hold at 8 cm or unknown data,
resume after clear readings above 9 cm for 0.75 seconds. Persistent invalid data
stops FC; the standalone TOF display continues diagnostic readings.
The v9.1 classification is retained: statuses 4/6/9/14 and valid ranges beyond
2 m count as clear; statuses 8/10 count as too close. This patch does not change
or independently validate that policy on your sensor.

Battery defaults are a 2S 7.4 V 2000 mAh Li-ion pack and R010 shunt. Percentage is
a coarse voltage estimate, suppressed under significant load. There is no
charging control. The separate power guard monitors current and voltage during
powered commands when enabled.

## INA219 diagnostics and power protection

From the v9.5 folder, run this before driving:

```bash
python3 main.py --check-ina
```

It needs no interactive terminal, does not load/create robot profiles, and never
opens motor GPIO. It reports a voltage/current sample or the actual error. The
normal menu's BAT command remains available for continuous motors-off readings.
Both use the same driver as the motion guard.

Defaults: I2C bus 1, address 0x40, R010 shunt (0.01 ohm). Change these only to
match your actual hardware, using `--bus`, `--battery-address` and `--shunt-ohms`.
The shunt value changes current scaling, not communication. For example, a real
R100 board requires `--shunt-ohms 0.1`; do not guess from a failed read.

- Missing `smbus` dependency: use the robot's existing Python environment with
  `python3-smbus` or `smbus2` available, as the error explains.
- `No such file` for the bus: check that I2C is enabled and the bus number is correct.
- `Remote I/O error`/no acknowledgement: check the configured address, power,
  ground, SDA and SCL. An address scan alone does not identify the chip model.
- Configuration mismatch/timeout: the message includes register values; keep
  that output for diagnosis instead of changing unrelated motor settings.

Startup retries transient I/O errors up to three times. A failed check never
pretends that current is zero and never silently disables protection. Press `s`
to cancel the pre-motion check. A stuck reader blocks another INA reader and
prevents BOT/SET from starting a new hardware session until the program exits.

Power protection remains ON by default: 4.5 A for 0.2 s, or 6.4 V for 2 s.
These are the v9.1 starting limits, not new measurements of either bot. The guard
checks current magnitude, including a negative reading from reversed shunt
orientation. Averaging plus the 20 ms inter-conversion pause produces roughly
40 ms between samples, not a guaranteed 20 ms response. The reported peak is a
sampled/averaged peak, not proof of true instantaneous stall current.

If you deliberately need unprotected bench tests, set **SET -> Power protection
-> Current and voltage stop while driving -> OFF**, then leave SET. It applies
immediately through the restart and the menu clearly shows OFF. No current or
low-voltage stop is provided in that mode. This choice is saved for that bot.

## Code and verification

- main.py: the launcher. `python3 main.py` from this folder; it switches itself into
  ~/robot-test-env when run with plain python3, and passes every option through.
- motor_imu_test.py: the old launcher name, kept so existing habits still work.
- app.py: robot selection, dispatch, sessions and cleanup before switching.
- terminal_ui.py: menus, settings editor and live display.
- go_button.py: GO, the START-button run with the PRD countdown and run timer.
- robot_settings.py: per-robot configuration validation and atomic saving.
- robot_profiles.py: separate files, explicit imports and identified-500-RPM migration.
- imu_tests.py: IMU calibration/movement tests extracted from the old launcher.
- motion_profile.py: independent 500 RPM and 100 RPM starting duties and gains.
- ir_tach.py: removable digital probe, bounded RPM measurement and duty search.
- pwm_calibration.py: motors-off placement prompts, two-pass calibration and explicit save.
- calibration_data.py: per-bot calibration record validation and wiring/reference matching.
- Existing motor, IR, battery, IMU, vision and ToF support remain separate modules.

Run offline checks with `python -m unittest discover -v`. TEST_RESULTS-v9.10.txt records
the v9.10 run; TEST_RESULTS-v9.9.txt the v9.9 run; TEST_RESULTS-v9.5.txt the merged v9.5 run. TEST_RESULTS-v9.4.txt is the gyro-lane v9.4 run; TEST_RESULTS.txt and
VERIFICATION.json are the wheel-calibration v9.4 run's records. Tests cover profile isolation,
menu selection, cleanup before changing GPIO maps, fresh sessions on reentry and
failure paths that block switching. **This revision has not been run on either
physical robot.**

The original external-review findings are retained in [REVIEW.md](REVIEW.md);
this README describes v9.10. Flight and Marshall files are outside this package.
BAT remains a motors-off diagnostic; the power guard runs during powered tests.

### CG ToF update (2026-10-14)

TT now enables ToF obstacle responses for FC and CG, including CG turns and timed moves. SET -> ToF obstacle range edits centimeter thresholds. Defaults: slow within 10 cm, stop at 8 cm, resume above 9 cm continuously for 0.75 seconds. Existing saved ToF settings take precedence. See TOF_SETUP.md for hold/resume details.

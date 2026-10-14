# Motor wire colors - both assembled robots

**v9.3:** select `1` for the 500 RPM bot or `2` for the 100 RPM bot at startup.
The selected robot's wiring is saved independently in `profiles/500rpm.json`
or `profiles/100rpm.json`. See [README.md](README.md) for switching and imports.

Recorded from Colin's description on 2026-09-23. Two robots are currently built:
the 500 RPM bot and the 100 RPM bot (previously specified as 104 RPM rated).
Both use 6 V N20 motors.

**Orientation:** stand behind the robot and look toward its front. Left and
right below are your left and right from that position. Front wheels are farther
from you; back wheels are nearer you.

| Wheel position | Command | 500 RPM bot: motor wire pair | 100 RPM bot: motor wire pair |
|---|---|---|---|
| Front left | FL | 🟥 Red + ⬛ Black | ⬜ White + ⬛ Black |
| Front right | FR | 🟨 Yellow + ⬜ White | ⬛ Black + 🟨 Yellow |
| Back left | BL | 🟦 Blue + ⬛ Black | ⬛ Black + 🟥 Red |
| Back right | BR | 🟩 Green + ⬛ Black | 🟨 Yellow + 🟩 Green |

Each pair identifies the two wires soldered to that motor's terminals. Color
order is reproduced as provided. The `+` joins the two color names; it does not
assign electrical polarity. Which color is on the marked positive or negative
motor terminal has not yet been specified.

These colors identify physical motor cables. Their connections to board channels
M1A, M1B, M2A and M2B have not yet been supplied, so they do not resolve the
earlier channel-map disagreement or establish the software reversal flags.
The IR connector-to-sensor roles also remain unconfirmed for each robot.

## 100 RPM bot: checked on hardware 2026-09-25

After re-plugging the back motors (J12 and J5) and the right and front IR sensors (J10 and J9),
the documented map below is correct on this bot, with **no** reversal flags. Checked with the
wheels lifted: `FL`, `FR`, `BL`, `BR` at 35% each spun their own wheel forward, and `IR` with tape
showed each sensor on its own label.

| Wheel | Header | Channel | N pin | S pin |
|---|---|---|---|---|
| Front left | J2 | M1A | White | Black |
| Front right | J4 | M1B | Black | Yellow |
| Back left | J12 | M2B | Red | Black |
| Back right | J5 | M2A | Green | Yellow |

IR: LEFT on J7 (BCM21), FRONT on J9 (BCM26), RIGHT on J10 (BCM7); J8 (BCM20) is spare.

N and S are the upper and lower pin of each header as Colin described them. If a lead is ever
re-plugged, match this table rather than changing settings. This is a wiring check, not a speed
or floor test. The 500 RPM bot has not been checked this way yet.

The command column names the **intended** wheel. Its actual wheel depends on the
selected channel assignments; a BL command does not itself verify the BL cable.

**v9.1-v9.3 defaults:** new profiles start on the documented rev C map, motor channels
`M1A,M1B,M2B,M2A` (from behind: J2 front-left, J4 front-right, J12 back-left, J5 back-right,
read off the fab files and the engraved base plate pockets) and IR `J7,J10,J9`. Saved profiles
keep their own map, and startup prints a note when a profile differs from it. The checks below
still decide what is true on each robot.

For each robot, select its profile and use `SET -> Wiring` to record the measured
channel assignments, direction reversals and IR roles. Leave SET to apply the saved map automatically. The app closes old GPIO and
all sensor sessions before reloading the same robot. Calibration clears and
camera/ToF toggles return to OFF. In Motor direction reversed, entering FL, FR,
BL or BR flips only that wheel; the other flags remain unchanged.

Check the map separately on each assembled robot: slide tape under each IR
sensor while running `IR`, then lift the wheels and run `FL`, `FR`, `BL` and
`BR`. Match the wheel that actually moves to the color pair above, and record
whether it turns forward. Neither the colors nor the profile name establish a
board-channel connection or electrical polarity.

The 500 RPM profile starts individual-wheel tests at 25% duty and manual driving
at 25%/25%. The 100 RPM profile restores the earlier 104 RPM motor defaults:
35% for individual-wheel tests and 70%/69% manual baselines. These are starting
settings, not measurements of the two robots' speed or traction. Saved tuning,
wiring and camera choices stay with the selected robot; they are not copied or
scaled when using BOT.

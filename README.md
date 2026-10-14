# Navigation

Robot navigation program for Team 2.09. CG follows a programmed course using IR lane sensors and MPU6050 heading control. TT enables ToF obstacle protection; VT independently enables camera traffic responses.

## Start on the Pi

From your navigation checkout, using the robot's configured Python environment:

```bash
python3 main.py
```

The launcher uses `~/robot-test-env` automatically when available. Select the 500 RPM or 100 RPM robot at startup, or use `python3 main.py --bot 500`. Preview effective settings without opening hardware with `python3 main.py --show-settings`.

Keep your robot's saved `profiles/` files when updating. This branch does not include your on-robot tuning. Use BOT to change robots or reload a profile; sensor toggles and IMU calibration reset when switching.

## Run a course

1. Use SET to check the selected robot's wiring, motor tuning and sensor settings. Changes apply when leaving SET.
2. Complete IC and IV as required for the IMU.
3. Use TOF to inspect distance readings with the motors off. Enable TT for obstacle protection.
4. Enable VT if camera traffic responses are wanted; RV shows the camera's detections.
5. Run CG and enter the course as F, R and L commands, such as FRL. GO can launch CG using the board controls.

Press `s` to stop and return to the menu. TT and VT start OFF after a program restart.

## ToF distances

Open **SET -> ToF obstacle range**. Values are in centimeters:

| Setting | Default | Behavior |
| --- | --- | --- |
| Slowdown | 10 cm | Reduce wheel targets progressively inside this distance |
| Stop | 8 cm | Brake at or below this distance |
| Resume | 9 cm | Require readings above this distance for 0.75 seconds |

Saved settings override defaults. Stop must be below both other thresholds. When changing older 3/5/8 cm settings, set slowdown to 10, resume to 9, then stop to 8.

With TT ON, ToF protects FC and CG. CG checks it during lane following, timed intersection moves and measured turns. Holds preserve remaining motion time and resume from stopped. Stop buttons, camera, power guard and IMU checks remain active while waiting. Slowdown reduces turn power too and does not compensate timed approaches for reduced travel distance.

## Guides

- [Wiring](WIRING.md): motor leads, connectors and assignments.
- [ToF setup and behavior](TOF_SETUP.md): driver installation, diagnostics and obstacle handling.
- [Wheel calibration](WHEEL_CALIBRATION.md): probe setup and saved wheel corrections.
- [Turn controls](TURN_CONTROLS.md): acceleration and intersection approach settings.
- [Historical documentation](docs/archive/): older release notes, reviews and the legacy operating guide. These describe earlier versions and are not current instructions.
- [Vision provenance](ryan_vision/SOURCE.md): source of the included detector modules.

## Code layout

`main.py` launches `app.py`, which owns the menu and sensor sessions. `lane_gyro.py` implements FG/CG; `motors_optimized.py` contains the motor driver and shared motion loops. `tof_support.py`, `vision_support.py` and `imu_support.py` provide sensor support. `robot_settings.py` and `robot_profiles.py` manage saved settings. Offline tests are the root `test_*.py` files.

## Verification

The CG ToF update passed 169 affected offline tests. The full suite passed 640 tests with two known button-test errors. Physical robot validation remains pending.

```bash
python3 -m unittest discover -p 'test_*.py'
```

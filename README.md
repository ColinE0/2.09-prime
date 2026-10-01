# 2.09-prime

Team 2.09 senior design repository, Texas State University, Fall 2026.

## Overview

Autonomous Prime is a small autonomous car that navigates a street map course
without human control or intervention. It follows a predetermined route
through a grid of 4-way intersections, staying in the right-hand lane, obeying
the traffic lights, and avoiding obstacles such as other AutoBots running the
course at the same time. At D2 Senior Design Day, each AutoBot runs the same
course three times, and its best single time counts.

## Current state

Integrated test bench combining the motor/IMU code from `kws`, the ToF code
from `cew`, and the camera code from `rr` under one terminal menu on `main`.

Motor commands retain their existing behavior. Camera and ToF commands are
readouts with the motors off. They do not stop, slow or steer powered commands.

## Team

| Initials | Name | Branch |
|---|---|---|
| KWS | Kaleb W. Smith | `kws` |
| RR | Ryan Reyes | `rr` |
| MM | Marshall Morris | `mm` |
| CEW | Colin Ezra W. | `cew` |

## Repository layout

```
main.py                    Shared menu and command dispatch
motors_optimized.py         Motor driver and timed movement
motion_profile.py          500 RPM and 100 RPM starting presets
single_motor.py            Individual-wheel checks
imu_support.py             MPU6050 reader and calibration support
imu_tests.py               Interactive IMU tests and gyro moves
tof_sensor.py              VL53L0X distance and obstacle readout
vision.py                  Camera capture and combined detections
stop_sign_detector.py       Stop-sign detection
traffic_light_detector.py  Traffic-light detection
README_Motor_IMU.md         Original motor/IMU guide
INTEGRATION.md              Integration scope and source commits
SOURCES.json                Original-file hashes and provenance
tests/                      Offline integration and GPIO fault-injection tests
```

Eight supporting Python modules are unchanged from their source branches.
`main.py` extends the `kws` launcher with the shared menu and sensor lifecycle.
`motors_optimized.py` adds pin checks and more thorough fault shutdown. Its
original pin assignments and motion commands are retained. The motor/IMU
guide's opening scope statement now points to the combined menu.

## Branching

- `main` is the integration branch. It should always build and run.
- Each member works on their own initials branch (`kws`, `rr`, `mm`, `cew`).
- Merge into `main` by pull request so the change is visible to the team.

## Getting started

On a Raspberry Pi, use an environment with the dependencies required by the
selected subsystem. Motor/IMU: `RPi.GPIO` and `smbus` or `smbus2`. Camera:
`picamera2`, `libcamera`, OpenCV (`cv2`) and NumPy. ToF:
`adafruit-circuitpython-vl53l0x` and `adafruit-extended-bus`.

Clone the repository or extract the ZIP, then run from its folder:

```bash
python3 main.py --preview
python3 main.py
```

`--preview` prints menus and starting presets without opening hardware. Normal
launch asks for the robot. Use `--bot 500` or `--bot 100` to choose directly.
The shared runtime requires an interactive Pi terminal and `RPi.GPIO`.
Camera and ToF dependencies load only when their readout is selected.

```text
2.09 PRIME | INTEGRATION PREVIEW
Active robot: 500 RPM bot
IMU: needs IC
Camera / ToF: readouts only; motor commands do not use them.
  1. Motor tests
  2. IMU tests and calibration
  3. ToF readout (motors off)
  4. Camera readout (motors off)
0 Home | H All commands | Q Quit | Command codes work anywhere
```

Choose a category, then a numbered action, or enter a command code directly.
`s` stops the active test and returns to its menu; Ctrl+C exits with cleanup.

| Commands | Actions |
|---|---|
| F | Forward until `s`; no sensor-controlled stopping |
| SR / SL, R / L, TR / TL, DR / DL | Existing timed strafes, spins, intersection turns and diagonals |
| FL / FR / BL / BR | One lifted wheel forward for one second |
| ID / IMU | Identify MPU6050 / display readings |
| IC / IV | Stationary gyro calibration / hand-turned yaw-axis verification |
| IF / ITL / ITR | Heading-hold forward / gyro turns after IC and IV |
| TOF | Display distance, obstacle, slowdown scale and sensor-fault status |
| VISION | Print detected stop signs and traffic lights with their positions |

Camera output remains quiet when nothing is detected, matching `rr`.
Sensor setup/read errors return to the menu after cleanup. A cleanup failure
ends the session. Reopen a readout to start a new camera or ToF session.

## Hardware

Raspberry Pi, TB6612FNG rev C motor board, mecanum wheels, MPU6050 IMU,
VL53L0X ToF sensor and Pi camera. Pin assignments and motor polarity are retained
from `kws`; camera configuration is retained from `rr`; distance thresholds are
retained from `cew`. Check wiring and lifted-wheel behavior on the actual robot.

## Software

Python 3. The launcher uses the motor/IMU classes and the existing
`read_tof()` / `print_tof_data()` and `read_vision()` / `print_vision_data()`
interfaces. Detector algorithms, motion profiles and sensor thresholds are
unchanged. The menu adds no autonomous navigation or persistent settings.

## GPIO setup and shutdown

The driver checks its full pin map before configuring outputs. It rejects
duplicate assignments, invalid BCM numbers and the BCM0/1 and BCM2/3 reserved
pins. It also refuses BOARD numbering and motor pins already configured as
outputs or alternate functions. No automatic reset or blanket cleanup runs at
startup. GPIO warnings remain enabled; setup warnings stop startup.

The motor board starts with standby LOW and PWM at zero. The active pin map is
kept fixed until shutdown. A GPIO write failure ends the session and attempts
to stop all four motors. Cleanup stops every PWM object and releases each
owned pin, even if another stop or cleanup step fails.

If startup reports a busy pin, stop the program using it and check enabled
peripherals and wiring. If a previous crash left outputs behind, disable motor
power before a controlled restart. Do not hide the warning or reset pins while
another GPIO program is using them. These checks are not an exclusive OS lock;
run only one program controlling these pins.

Reconfiguring a GPIO pin does not inherently damage it. Electrical contention,
short circuits, excessive current or voltage can damage the Pi. GPIO signals
are 3.3 V; cleanup returns used pins to inputs, but it cannot correct wiring or
protect against every failure. Power down before rewiring. A hardware standby
pull-down/interlock is needed if the driver must remain disabled when software
cannot run. This preview does not verify that board circuitry.

References: [Raspberry Pi GPIO electrical guidance](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#current-value)
and [RPi.GPIO setup and cleanup](https://sourceforge.net/p/raspberry-gpio-python/wiki/BasicUsage/).

## Testing

Run the offline checks from a full Git checkout with Python 3:

```bash
python3 -m unittest discover -s tests -p "test_prime_*.py" -v
```

The source-provenance check also needs Git and the source commit objects from
the member branches. A full clone includes these; a source ZIP does not.

The bench is checked offline with simulated menu, sensor and motor interfaces.
These checks cover the new connecting code; they do not establish physical
sensor detection, wiring, motor behavior or whole-program hardware correctness.
`imu_tests.py` contains interactive hardware actions, not an automated test suite.
GPIO fault-injection checks also cover partial startup, pin conflicts, active
map changes, write failures and continued cleanup after an error. Software
cannot guarantee zero GPIO errors or prevent every form of hardware damage.

## Milestones

| Date | Milestone |
|---|---|
| 2026-10-12/14 | Full Functional Demonstration |
| 2026-10-21 | Partial Test Report |
| 2026-11-09/11 | Final Design Reviews |
| 2026-11-20 | Draft Poster |
| 2026-11-30 | Final Team Docs |
| 2026-12-03 | Senior Design Day |

## Documentation

- [Motor and IMU commands](README_Motor_IMU.md)
- [Integration scope and source commits](INTEGRATION.md)
- [Source-file hashes](SOURCES.json)

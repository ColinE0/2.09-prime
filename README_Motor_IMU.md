# Motor + MPU6050 IMU code

Motor driver (TB6612FNG, rev C board, mecanum wheels) and MPU6050 gyro code only.
This folder has no IR, camera, ToF, battery/INA219 or wheel-calibration code.

Run on the Raspberry Pi from this folder:

```bash
python3 main.py --bot 500
```

`--bot 100` selects the 100 RPM bot. Without `--bot`, it asks at startup. You need `RPi.GPIO` and `python3-smbus` (or `smbus2`).

| File | Purpose |
|---|---|
| `motors_optimized.py` | GPIO pin map, `MotorDriver` (PWM, direction, brake/coast), `Controller` (ramping, timed moves, stop key). Can also run by itself as a motor-only menu. |
| `motion_profile.py` | Speed and IMU gain presets for the 500 RPM and 100 RPM bots |
| `imu_support.py` | MPU6050 I2C reader thread, gyro bias calibration, heading integration |
| `imu_tests.py` | IMU menu tests (ID, IMU, IC, IV, IF, ITL, ITR) and the turn-angle logger |
| `single_motor.py` | Spin one wheel (FL/FR/BL/BR) to check wiring |
| `main.py` | Combined motor + IMU terminal menu |

## Commands

| Command | Action |
|---|---|
| F | Drive forward until you press `s` |
| SR / SL | Timed strafe right / left |
| R / L | Timed spin right / left. The gyro measures the angle if IC and IV have been run. |
| TR / TL | Drive forward, turn, then drive forward again |
| DR / DL | Timed diagonal |
| FL FR BL BR | Spin one wheel forward for 1 s (lift the wheels first) |
| ID | Check that the MPU6050 answers at 0x68 |
| IMU | Show live accelerometer and gyro readings (motors off) |
| IC | Gyro bias calibration. Keep the robot still for 4 s. |
| IV | Find the yaw axis: turn the robot left by hand, then press `g` |
| IF | Drive forward for 2 s holding the gyro heading |
| ITL / ITR | 90-degree gyro turns |

`s` stops any command. Ctrl+C stops the motors and exits.

## Wiring and tuning

- Pins are set at the top of `motors_optimized.py` (`MOTOR_PINS` in FL, FR, BL, BR order, `STBY`, `REVERSED`). The defaults are the documented rev C map: FL J2/M1A, FR J4/M1B, BL J12/M2B, BR J5/M2A.
- Per-robot speeds and IMU gains are in `motion_profile.py`.
- Timed maneuver durations (`STRAFE_DURATION`, `TURN_DURATION_*`, etc.) are at the top of `motors_optimized.py`.

# ToF distance display and obstacle toggle (included in v7)

V5 had no ToF integration. V6 adds the **VL53L0X** identified in the older local project, on **I2C bus 1, address 0x29**. It is intended to face forward. The previous test packages and GitHub repositories are unchanged.

## Terminal setup

Transfer the complete `MPU6050_Motor_Test_v8_500rpm.zip` to the Pi, then:

```bash
cd ~
unzip MPU6050_Motor_Test_v8_500rpm.zip
cd MPU6050_Motor_Test_v8_500rpm
```

If you already have a virtual environment that runs the motor/camera tests, activate it and install the new dependencies there:

```bash
python3 -m pip install -r requirements-tof.txt
python3 main.py
```

If you normally use Raspberry Pi OS's system Python, create an environment that can also see your existing system GPIO/Picamera2 packages:

```bash
sudo apt install python3-venv
python3 -m venv --system-site-packages ~/robot-test-env
~/robot-test-env/bin/python -m pip install -r requirements-tof.txt
python3 main.py
```

Use that last command from this folder on later runs. No desktop, screen preview, or mouse is needed. The old `VL53L0X` C binding does not replace these new dependencies; v6 uses the Adafruit driver with an explicit measurement-status check. Other menu modes remain usable without installing the ToF dependencies.

I2C must be enabled, as for your IMU/battery tests. The sensor address differs from the MPU-6050 (0x68) and INA219 (0x40). This package opens only the configured address, does not scan the bus, and does not assign XSHUT pins or change the sensor address. It supports one VL53L0X. Stop any other program using that sensor before testing.

## Menu

- **TOF**: live distance in cm, raw millimeters/status, sample age and proposed FC/CG response. Motors stay off. Press **s** to return.
- **TT**: toggle ToF obstacle responses for **FC and CG**. Starts **OFF** on each launch. Toggling alone neither starts the sensor nor moves the wheels.
- **FC**: existing 10-second IR-confidence lane test, with ToF responses if TT is ON. **VT** independently adds camera responses. Both can be ON together.

TT applies **to FC and CG**. Manual motor modes, F/FA/C, IMU tests, RV, IRC and BAT retain their existing behavior. The sensor worker closes when TOF or a ToF-enabled FC/CG run ends. The toggle stays selected for the next FC/CG run during that menu session, but resets OFF when the program restarts.

## Obstacle behavior

Distances are measured from the sensor, not the front bumper. Starting values are taken from the earlier project's range configuration:

| Distance/state | FC/CG response when TT is ON |
|---|---|
| At least 10 cm | Configured FC speed, default maximum 25% PWM duty |
| Between 8 and 10 cm | Reduce wheel targets with distance, using the existing PWM slew limits |
| At or below 8 cm | Brake immediately and hold |
| After a hold/startup | Require readings above 9 cm for 0.75 seconds before resuming |
| Invalid status, zero distance, or greater than 2 m | Hold; never interpret an invalid reading as clear |
| Invalid readings persist for 1 second | End FC and return to the menu |
| Missing/stale result over 0.30 seconds, I2C error, worker exit | End FC; do not fall back silently to driving without ToF |

The slowdown multiplier varies from 100% at 10 cm toward 40% just above 8 cm; it scales FC's existing wheel targets and preserves steering ratios. Braking at the hold threshold bypasses slew. Resuming adds the existing 120-ms IR warmup and acceleration ramp. A momentarily interrupted clear streak resets confirmation; repeatedly reading one sample never advances it.

Camera and ToF decisions are both checked during holds. Removing an obstacle cannot override a red-light hold; seeing green cannot override a nearby obstacle. A confirmed stop sign still ends FC. The front IR stop, **s**, and 10-second FC limit retain priority. The timer includes obstacle and light holds; starting the sensors is a separate phase with the motors off.

## First test

1. Select **TOF** with the robot stationary and wheels off. Move a flat target between roughly 8, 9 and 12 cm from the sensor and compare the displayed distance with a ruler. Check the proposed hold/slow/clear responses. A raw hardware status of **11** is the accepted range-complete status.
2. Check that the forward-facing sensor sees the obstacle rather than the floor, bumper or sensor cover. Observe the real lane props and other robot surfaces: dark/glossy targets, strong ambient light and mounting affect usable readings.
3. Return with **s**, select **TT**, then try **FC** in a clear test area. The menu must say ToF obstacles for FC: ON. Add **VT** when ready to test both sensors together.

An open scene with no usable reflection can produce **unavailable** and hold/end FC. This conservative test does not equate “no return” with “clear.” TOF keeps its motors-off display open for invalid targets so you can diagnose them. Do not change the invalid-result rule simply to make the robot move; first check range, mounting and targets.

These distances are initial settings, not a measured stopping-distance guarantee. The package has not been run on this Pi or its hardware. Raw hardware status validation also does not implement all the ST API's software sigma/quality checks. Verify actual braking distance and performance before relying on the feature. Settings are at the top of `tof_support.py`; hardware address can be selected with `--tof-address`, and `--bus` selects the shared I2C bus.

## Driver and implementation

Dependencies are pinned in `requirements-tof.txt` to `adafruit-circuitpython-vl53l0x==3.6.19` and `adafruit-extended-bus==1.0.2`.

The driver checks sensor identity during initialization. Startup explicitly ends any previous continuous mode and discards the shot started by that driver operation. Subsequent reads trigger individual 33-ms-budget measurements, verify a new ready interrupt, read the 12-byte result block at register 0x14, and then acknowledge it. Distance and hardware status come from the same result block. Timestamps start before triggering acquisition. Results are requested at up to 20 Hz, not guaranteed at that rate.

The Adafruit public range property omits hardware status. A small adapter therefore uses the pinned driver's `_device`, `_write_u8` and `_data_ready` interfaces to read and acknowledge the full result. Other sensor tuning remains at the driver's defaults. The raw status check accepts only hardware range-complete **11**, not ST's translated PAL status **0**. Do not upgrade the pinned driver without reviewing these interfaces and rerunning tests.

All ToF I2C calls run in a separate process. The parent can stop the wheels while a sensor operation is blocked, and worker shutdown waits are bounded. A forced worker termination is reported; interrupted sensor initialization may require quitting and power-cycling the sensor/robot before retrying. This code does not claim to perform an automatic hardware reset.

Primary implementation references:

- [Adafruit VL53L0X 3.6.19 source](https://github.com/adafruit/Adafruit_CircuitPython_VL53L0X/blob/3.6.19/adafruit_vl53l0x.py)
- [Adafruit Extended Bus 1.0.2 source](https://github.com/adafruit/Adafruit_Python_Extended_Bus/blob/1.0.2/adafruit_extended_bus.py)
- [ST VL53L0X result/status decoding](https://github.com/stm32duino/VL53L0X/blob/main/src/vl53l0x_class.cpp)

The older local project supplied only the sensor identity/bus/address and initial obstacle thresholds (its `config.py`). Its adapter, which clipped oversized distances to clear and did not validate raw status, was not reused.

## Adjustable distances

Open SET -> ToF obstacle range to edit Stop distance, Resume distance, and Slowdown distance in centimeters. Defaults are 8, 9, and 10 cm. Changes save to the selected robot profile and apply when leaving SET. Stop must be below both other distances. If increasing stop beyond a current limit, raise resume and slowdown first; when reducing all distances, lower stop first. ToF obstacle control still requires TT enabled and FC or CG mode.

## CG obstacle protection

With TT ON, CG checks ToF during lane following (including the final lane), timed intersection approaches/crossings/exits, and measured turns. A hold brakes immediately; fresh clear readings restore direction and ramp from zero. Timed moves preserve their remaining hold duration and reaccelerate; turn timeout and stall clocks exclude obstacle waiting. Camera, stop buttons, power guard and gyro polling remain active during holds. Stopped intersection waits retain their normal timing. Slowdown scales wheel targets, including turn targets; it does not extend timed moves to compensate for reduced speed. Validate intersection approach distances and turn performance on the robot.

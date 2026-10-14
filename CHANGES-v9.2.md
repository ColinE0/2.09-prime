# v9.2: reversal application and INA219 recovery

Built from the supplied `MPU6050_Motor_Test_v9_1.zip`. The supplied ZIP is
preserved. Tests use simulated GPIO/I2C; neither physical robot was available.

## Motor reversal

The v9.1 editor saved a new reversal tuple but left the running motor module
unchanged until BOT/restart. This made the next wheel test use the old direction.
V9.2 closes all hardware when the user leaves SET after a change, then reloads
the same bot and the same settings path before returning to the main menu.
Cleanup failure prevents reinitialization. Calibration and runtime sensor
toggles reset, as with a BOT switch. Merely entering SET does not restart anything.

In SET -> Wiring -> Motor direction reversed, enter FL, FR, BL or BR to toggle
only that wheel. Existing four-value on/off input still works. The Motor tests
menu shows the active wheel/channel/reversal mapping. Direction flags belong to
logical wheels; the lifted-wheel channel check still establishes physical identity.

Regression coverage runs the real editor, profile save/load, automatic reload,
single-wheel controller and GPIO driver for every wheel on both profiles. It
checks that only the selected wheel's direction outputs invert, including the
second test without a manual BOT/restart. Disk errors retain the old settings.

## INA219 and the motion guard

V9.1 made INA219 readiness mandatory for every powered command while its power
guard was ON (the default). A missing device, wrong address, bus/dependency
failure or conversion error therefore refused every motor test. The exact error
seen on the robot was not available; the archive alone cannot identify that
hardware failure. The INA219 conversion format and scaling were checked against
the [TI INA219 datasheet](https://www.ti.com/lit/ds/symlink/ina219.pdf), sections
8.3 and 8.6. The configured triggered-conversion mode is retained.

Confirmed software problems fixed:

- BAT and the power guard could own separate readers. A blocked BAT reader
  could be followed by a second reader, and a blocked guard could be abandoned
  during BOT switching. Both now share BatterySession's tracked owner; failed
  cleanup cannot be bypassed by SET/BOT.
- Startup readiness waited outside the keyboard handler. The stop key now
  works during the wait, before any PWM output.
- A readiness failure could finish with standby enabled for braking. Power
  faults now leave PWM zero and standby LOW, including nested controller cleanup.
- A stale first sample could pass readiness, and errors while motors were off
  could be ignored until after the first powered tick. Samples are validated
  before motion and throughout the test.
- Negative measured current could evade the overcurrent comparison. The guard
  now compares current magnitude; BAT retains the signed reading.
- The saturation message assumed an R010 shunt. It now uses the configured
  shunt resistance when reporting the measurement limit.

Initialization retries transient I/O errors up to three times. Mid-command
errors are not retried or hidden from the guard. Error messages identify bus,
address and shunt, with expected/actual configuration or last bus-register
values when relevant. No device address or current calibration is guessed.

Use `~/robot-test-env/bin/python motor_imu_test.py --check-ina` for a bounded
standalone check without motor GPIO, a TTY or saved profiles. BAT remains the
motors-off live diagnostic. Sensor diagnostics and SET remain accessible after
a normal INA fault. A genuinely stuck worker requires exiting the program.

Protection stays ON by default with the v9.1 limits. It never silently falls
back to unprotected driving. If the operator explicitly chooses OFF in SET ->
Power protection, that choice now takes effect on leaving SET and is visible in
the menu. A working INA219 is still required for driving with protection ON.

## Profile upgrade and scope

Before creating defaults, each bot can copy its own tagged profile from a
sibling `MPU6050_Motor_Test_v9_1/profiles/`, then `MPU6050_Motor_Test_v9/profiles/`.
Existing v9.2 files take precedence. Sources stay unchanged and mismatched bot
identities fail validation. This preserves saved reversals when moving to the
new package. Explicit imports/custom paths remain available.

V9.1 motor/IR defaults, ToF classification, turn logging/lead, and motion
presets are retained. This patch does not establish the robots' physical map or
calibrate their power limits. The software's averaged current peak is not a
measurement of instantaneous stall current. See TEST_RESULTS.txt and
VERIFICATION.json for the final regression count and package hashes.

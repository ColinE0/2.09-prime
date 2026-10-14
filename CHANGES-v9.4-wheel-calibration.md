# v9.4: removable IR wheel calibration

Built on v9.3, retaining Ryan's September 23 vision update, the v9.2 motor-reversal
and INA219 fixes, and the existing independent bot profiles. The new commands use
one removable FC-51 probe on the confirmed spare **J8 / BCM20** connector.

- **PPROBE** checks exactly five hand rotations with motors off.
- **PCAL** measures all four lifted wheels forward and reverse, then adjusts and
  independently verifies each against one shared RPM target: 95% of the slowest
  of the eight baseline readings. It requires eight probe placements across two
  passes. Each placement repeats the hand check before powered measurements.
- Complete results within 5% of target can be saved explicitly with **SAVE**.
  Atomic saving retains the previous profile on incomplete or failed runs. The
  app reloads the selected robot with motors off after saving.
- Separate forward/reverse factors in the 0.5–1.0 range reduce each wheel's PWM
  once at the driver output. Logical requested duties stay separate, so ramps
  do not repeatedly multiply corrections. Existing driver ceilings remain in
  force. Raw measurements temporarily bypass factors and coast-stop before
  restoring normal output mapping; existing power protection stays attached.
- **SET -> Wheel calibration** selects the probe connector and reference duty,
  and enables/disables saved corrections. Changing motor channels, reversals or
  reference duty invalidates their application. Lane-sensor pin conflicts are
  refused. Normal driving does not require the probe after it is unplugged with
  power off.

Read [WHEEL_CALIBRATION.md](WHEEL_CALIBRATION.md) for setup, exact commands,
measurement bounds, troubleshooting, research references and the limits of a
single-reference unloaded calibration. No physical wheel RPM, connector voltage
or floor performance has been verified in this delivery. Software verification
passed all 422 tests using simulated GPIO, pulse trains, motor response and fault injection; see
`TEST_RESULTS.txt` and `VERIFICATION.json` for its record.

New installs look first for each bot's tagged profile in sibling
`MPU6050_Motor_Test_v9_3/profiles/`, then v9.2, v9.1 and v9. Existing v9.4 profiles
take precedence. Source profiles and earlier packages remain unchanged. Old
profiles gain calibration defaults with corrections OFF until a result is saved.

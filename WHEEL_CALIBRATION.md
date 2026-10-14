# Removable FC-51 wheel calibration

V9.4 can measure and balance lifted-wheel speed using one removable FC-51 probe
on **J8 / BCM20**. It saves separate forward and reverse duty factors for each
wheel in the selected robot's profile. After saving, power down and unplug the
probe; normal driving applies the saved factors without reading the probe.

This is an unloaded baseline calibration. It does not provide continuous speed
feedback after unplugging, and it cannot establish motor direction or physical
wheel identity. Check FL, FR, BL and BR assignments and forward direction first.

## Prepare the probe

1. Use the established FC-51 board harness on the confirmed spare **J8** connector.
   Keep its existing signal/power/ground arrangement; this guide does not assign
   a generic FC-51 pin order. The connector and signal voltages have not been
   physically validated as part of this software work.
2. Lift and support the robot so all wheels turn freely. Put **one contrasting
   mark** on each wheel's flat face, away from the mecanum rollers. The sensor
   must see one complete mark/background cycle per revolution.
3. Secure the probe facing that surface. Keep the long cable clear of the wheels
   and motor wiring where practical. Adjust gap and sensitivity until the mark
   switches the digital output cleanly. Extra reflections, a narrow mark, poor
   contrast or cable/motor noise can make RPM readings fail.
4. In **SET -> Wheel calibration**, leave the probe connector at **J8** and check
   **Raw calibration reference duty (%)**. The default is 25%; its range is
   10–50%. Leave SET to apply edits. The reference must reliably turn every wheel
   in both directions. Calibration never increases a wheel above this reference.

If J8 is assigned to a lane sensor in this bot's saved wiring, the probe commands
refuse to run. They do not borrow that sensor or silently choose another port.
Ordinary driving does not open the probe input.

## Check it with motors off: PPROBE

Enter **PPROBE**. Start with the mark facing the sensor, rotate the selected wheel
**exactly five complete turns by hand**, and finish with the mark facing the
sensor again. Press **g** when finished. The command succeeds only if it counted
five turns and ten transitions; the motors remain off. **s** cancels. The check times out after
60 seconds.

If the count is wrong, fix the marker, alignment, gap or sensitivity and repeat.
Five correct hand turns establish a useful optical check; they do not prove that
every edge will be captured at powered speed.

## Calibrate and save: PCAL

Enter **PCAL**, then type **LIFTED** to start. The wizard requests eight probe
placements in total:

| Pass | Placement order | What runs at each placement |
| --- | --- | --- |
| BASELINE | FL, FR, BL, BR | Selected wheel forward, then reverse, at raw reference duty. |
| ADJUST AND VERIFY | FL, FR, BL, BR | Selected wheel forward, then reverse, searching and independently remeasuring its correction. |

At each placement, secure the probe and type the requested wheel code. Repeat
the five-turn hand check. **Clear your hands before pressing g: inside PCAL, g
starts that wheel's powered measurements after the check passes.** Only the
selected wheel receives positive PWM. All wheels coast-stop between
measurements and before the next placement prompt. **s** stops a running stage;
Enter at a placement prompt cancels the wizard.

After the baseline pass, the target is **95% of the slowest of all eight
wheel/direction RPM readings**. Forward and reverse share this one target so
mixed-direction turns and strafes use the same calibration reference. The wizard
searches for duty factors between **0.5 and 1.0**, then takes another measurement
to require every result to be **within 5% of the target**. If the target cannot be
reached repeatably within that range, the run aborts without saving.

After all eight results pass, review the displayed factors and type **SAVE**.
Only then is the selected bot's profile atomically replaced and corrections
enabled. The app closes hardware and reloads that bot with motors off. Enter
instead keeps the previous settings. An incomplete run, sensor failure, stop,
power fault or changed profile on disk does not replace the previous calibration.
As with other hardware reloads, camera/ToF toggles and IMU calibration reset.

The existing INA219 guard remains active during powered measurements when
enabled in that bot's settings. Calibration does not disable it. A missing
INA219 reading blocks powered measurements under that setting; **PPROBE** remains
a motors-off check.

## What the saved factors do

Each normal motor command is multiplied once by that wheel's factor for its
logical direction, after the controller computes the requested duty. For example,
a request of 25% with factor 0.8 produces 20% PWM. Electrical reversal flags do
not swap the meaning of the saved forward/reverse factors. The normal 70% driver
ceiling remains enforced before multiplication.

The saved record includes bot identity, channel assignments, reversal flags,
reference duty, baseline and verified RPMs, factors and timestamp. It belongs to
that bot only. Changing channels, reversal flags or reference duty disables the
old corrections; recalibrate for the new setup. **SET -> Wheel calibration ->
Apply saved wheel corrections -> OFF** disables application while retaining the
record. Leave SET to apply the change. The menu shows correction status.

These are **single-reference scaling factors**, not a measured speed curve.
Agreement at the calibration duty does not establish agreement at other duties,
especially near the starting/stalling threshold. Battery voltage, temperature,
surface and floor load still affect speed. Finish with short floor runs to check
that all wheels start reliably and lane corrections behave properly. No real
wheel RPM or under-load performance has been measured in this software delivery.

## Measurement limits and failed checks

The digital edge reader counts one rising edge per revolution and calculates
`RPM = 60 / average seconds per revolution`. Light/dark polarity therefore does
not need a setting. It observes both edges to check signal consistency.

| Check | Current implementation |
| --- | --- |
| Accepted speed | 30–1200 RPM; the shared target must also be at least 30 RPM. |
| Pulse width | Successive high/low transitions must be at least 1 ms apart. |
| Steady sample | At least four complete periods; retain up to eight. |
| Period consistency | Largest minus smallest period must be at most 12% of the average. |
| Between attempts | Coast with zero PWM for at least 0.5 seconds, with stop/power checks still active. |
| Settling | Wait 0.75 seconds after reaching duty, then discard earlier pulses. |
| Missing pulses | Stop after 3 seconds without an edge during steady measurement. |
| Attempt limit | Each powered measurement, including ramp and settling, is bounded to 12 seconds. |
| Control-loop delay | Stop if a loop gap exceeds 0.12 seconds. |

These checks reject many disconnected, stuck or noisy signals; they cannot
detect every systematic missed pulse or extra optical feature. The supported
range does not guarantee a successful sample at every speed/reference/ramp
combination within the 12-second budget. If a measurement fails, inspect the
printed reason and fix the probe or startup behavior before trying again.

## Basis and verification scope

Pololu demonstrates a reflectance sensor observing a black mark on a motor-driven
white disk and shows how gap and motor noise affect contrast. Its QTR sensors
have different output electronics from the FC-51; this supports the optical
principle, not an FC-51 voltage or pin-order claim.
[Pololu QTR Reflectance Sensor Application Note, sections 2–3](https://www.pololu.com/docs/0J13/all)

NI describes one optical pulse per revolution and reciprocal pulse-period
measurement at low speed. Its example uses a photointerrupter and myRIO hardware;
our implementation uses the FC-51 digital output and Raspberry Pi GPIO callbacks.
[NI myRIO Project Essentials Guide, chapter 50, printed page 211](https://download.ni.com/evaluation/academic/myRIO_project_essentials_guide__Feb_09_2016___optimized.pdf#page=219)

All 422 offline tests pass, including a complete wizard run against simulated
unequal motors, callback pulse measurement, cancellation, power faults, failed
saves and applying saved corrections after reload. Those checks do not validate this physical probe, long cable, connector
voltage, Raspberry Pi callback timing or actual robot calibration. See
`TEST_RESULTS.txt` and `VERIFICATION.json` for the delivered verification record.

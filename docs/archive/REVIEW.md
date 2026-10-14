> Historical document. For current operation, see the [navigation guide](../../README.md).

> **v9.4:** gyro lane keeping (FG, CG) is added; see [CHANGES-v9.4.md](CHANGES-v9.4.md).
> **v9.3:** Ryan's September 23 vision update is included; see
> [CHANGES-v9.3.md](CHANGES-v9.3.md).

> **Historical review:** the text below describes v8/v9. For current behaviour, use
> [README.md](../../README.md) and [CHANGES-v9.2.md](CHANGES-v9.2.md). v9.1 added a
> power guard and changed default wiring/ToF policy. v9.2 applies SET edits on
> return and fixes INA reader ownership, startup handling and diagnostics.

> **v9 current behavior:** this file preserves the historical review of original
> v8 and reviewed-v8. Its old migration description and single-profile defaults
> are superseded by the two-robot behavior in [README.md](../../README.md).

V9 asks for the 500 RPM bot (`1`, or `--bot 500`) or the 100 RPM bot (`2`, or
`--bot 100`; these are the earlier 104 RPM rated motors). Separate
`profiles/500rpm.json` and `profiles/100rpm.json` retain each bot's wiring,
camera choices and tuning. The 500 RPM profile starts powered modes at 25% or
less; the 100 RPM profile restores the earlier 70%/69% manual baseline, 50%
FC/IMU maximum and 35% wheel-test duty. IMU turn floors are 16% and 32%
respectively. These presets are not physical calibration or a verified wiring
map for either robot.

`BOT` closes motors/GPIO and every sensor session before selecting another robot
or reloading the same one. A cleanup failure blocks switching. IMU calibration
is discarded and camera/ToF toggles return to OFF. Use BOT and reselect, or
restart, after SET edits. Bus/address and camera CLI overrides remain in effect
for the entire launch, including switches.

`--show-settings` now previews both effective profiles without writes or hardware
access; add `--bot` for one. Automatic migration is limited to a source explicitly
marked as the 500 RPM profile. Unmarked v8/v7 files require an explicit import
into a named robot, and imports never overwrite an existing destination. The
README documents these rules and custom-file binding. No existing flight config
or Marshall script is remapped by choosing a v9 profile.

Update, 2026-09-23: Colin supplied motor wire colors for both assembled bots.
They are recorded in [WIRING.md](../../WIRING.md). Board-channel connections and
positive/negative lead identities were not supplied, so the channel-map finding
below remains unresolved.

# Verification of the external v8 review

Reviewed the local v8 package against an earlier local workflow package (2026-09-17),
the original `motors2_diagonal.py`, and the pasted review. The original 257
offline tests passed. No physical robot measurements were available. The new
test count and file hashes are recorded in VERIFICATION.json.

## 1. Wiring disagreement: confirmed; current correct map is unknown

| Role | v8 defaults | Recorded flight config |
|---|---|---|
| LEFT IR | J10 / BCM7 | J7 / BCM21 |
| RIGHT IR | J9 / BCM26 | J10 / BCM7 |
| FRONT IR | J7 / BCM21 | J9 / BCM26 |
| FL | M1A, not reversed | M1A, not reversed |
| FR | M1B, not reversed | M1B, not reversed |
| BL | M2A, not reversed | M2B, reversed |
| BR | M2B, not reversed | M2A, not reversed |

Evidence: original v8 `motors_optimized.py` constants and `robot_settings.py`
defaults; flight `config.py` WHEELS and IR_PINS. Both use the same board nets.
Flight comments cite the September 16 motor ID test and September 17 IR roles.
The original `motors2_diagonal.py` also explicitly describes a motor ID test
**after a rewire**, removal of BL inversion, and a sensor rebuild. These are two
different recorded harness arrangements; the code alone cannot date or verify
the robot's current arrangement. Do not automatically overwrite v8 with flight.

REVERSED changes polarity, not wheel position. The required flight arrangement
includes BOTH exchanging the back channels AND inverting BL. A rear swap can
turn a diagonal correction into forward motion plus yaw. It does not establish
the cause of a particular left overcorrection. Recheck tuning after correction;
not every previous tuning value is necessarily invalid.

**Historical v8 behavior, superseded by v9:** `--show-settings` was read-only.
Before initial automatic migration it displayed
defaults, not the sibling v7 configuration that a normal launch would import.
Existing v8 JSON overrode defaults. Neither operation measured wiring.

**Implemented in reviewed-v8:** validated channel assignments in SET -> Wiring, applied before
GPIO setup, plus startup display of channels, IR connectors and reversal flags.
Literal MOTOR_PINS permutations now migrate from v7, and a sibling v8's saved
settings and literal channel map are copied on first normal launch. No arbitrary
GPIO numbers or repeated motor channels are accepted. Defaults stay as v8 until
the operator records a physical check.

**Physical check still needed:** run IR with tape under each sensor separately;
then lift the wheels and run FL, FR, BL, BR individually. Record wheel identity
AND whether it turns forward. BL alone distinguishes the two listed rear maps,
but does not verify every wheel or polarity. In v9, use BOT and reselect the same
robot (or restart) after SET changes, then repeat the check before lane tuning.
Enter that robot's measured map in flight config and
the Marshall scripts. No identifiable fixed Marshall scripts were found in the
inspected workspace, so those scripts have not been verified or changed.

## 2. ToF on an empty lane: behavior confirmed; proposed blanket fix rejected

`ToFSample.usable` accepts raw status 11 and 1..2000 mm. Fresh invalid readings
hold FC and fault after one second; stale/error results fault independently.
Flight's ranger clips positive oversized distances to its maximum without the
same raw-status validation. This discrepancy is real and already documented in
TOF_SETUP.md. Whether the actual empty lane yields invalid readings is untested.

No target, weak reflection, and unreliable measurement must not all become a
drive permission. ST's [VL53L0X datasheet, section 6](https://www.st.com/resource/en/datasheet/vl53l0x.pdf)
shows that maximum range and valid detection rates depend on target and ambient
conditions. My conclusion is that a missing return alone is insufficient proof
of a clear lane. Keep TOF's raw distance/status diagnostics and record both empty
lanes and actual obstacles before changing this policy. The conservative gate is
unchanged; this package can still hold/end FC on an empty scene.

## 3. One red frame: latch confirmed, but it holds rather than directly ends FC

The original gate latched red/yellow on one qualifying frame, requiring three
green frames to release. FC's ten-second run timer includes holds, so the run
can time out while waiting. A red-colored object is only a candidate if the
detector's geometry, region and area requirements pass; not every red pixel or
shirt triggers it.

**Implemented:** immediate hold on the first candidate remains. Red/yellow
must appear in two of the latest three distinct frames to latch; mixed red and
yellow count toward the same stop policy. One isolated candidate clears after
three hazard-free frames. A latched hold still requires three green frames.
Stop-sign termination and camera freshness checks remain. This is an explicit
policy change; test actual props in RV before VT/FC.

## 4. 100% manual duty: confirmed and fixed

v8 accepted 100% baselines and the driver allowed 100%. The flight project's
existing ceiling is 70%. The reviewed settings reject baselines over 70%,
including existing files and imports, and MotorDriver also rejects targets over
70% before writing any PWM. Related secondary duties cannot exceed the baselines.
Old test fixtures using 80/85% were changed to valid 60/65% values and dedicated
overspeed rejection tests added. FC, single-wheel and IMU limits remain 50%.

The 8.4 V * 0.70 = 5.88 V average-voltage calculation motivates this project
limit; it does not prove safe current or motor temperature under every load.

## 5. No running current/voltage trip: confirmed; fuse assertion unverified

BAT reads with motors stopped. No battery monitor is attached to the motor
control poll path. Correction timeouts are not general stall protection; F can
run until an IR stop/operator action, and timed moves stop on their own timers.
The presence or absence of a physical fuse cannot be verified from these files.

A dependable cutoff needs the measured current path, normal acceleration/current
envelope, stall limits, battery under-load limits, debounce, freshness failure
policy, and tests across all motor modes. It is not responsibly specified by
adding an arbitrary INA219 threshold. No current or low-voltage cutoff is claimed
or added here. The existing battery profile's display threshold is not validated
as an under-load shutdown threshold. Hardware protection remains a separate item.

## IMU and broader claims

Identity-before-writes, configuration readback, data-ready checks, stale/gap
rejection, calibration noise checks and five-minute calibration expiry are
present. Original TL/TR/course turns remain timed. Measured ITL/ITR request a
stop at 88 degrees toward the nominal 90-degree target and observe residual
rotation for 0.25 seconds. Their target duty floors at 32%; actual applied duty
also depends on slew and baseline scaling. Those are software behaviors, not
measured physical accuracy.

Two claims need correction: a non-0x68 WHO_AM_I means unsupported identity for
this driver, not proof of a counterfeit chip; and automatic axis identification
does not support arbitrary mounting angles. It selects one dominant sensor axis
and checks alignment with gravity. Timed-turn angle logging is a reasonable
future diagnostic, but predictive braking needs measured stopping behavior; no
unmeasured braking constant was added. Heading drift is a plausible explanation,
not a demonstrated diagnosis of the reported turn fault.

IMU integration into flight, physical wiring changes, and choosing a demo-day
controller remain separate decisions. This revision addresses the verified
settings, channel configurability and single-frame latch issues in the test
bench. Original v8 and flight files are preserved. Offline fake-hardware tests
do not certify robot wiring, current protection, stopping distance or course
performance.

# Independent turn controls, 2026-10-06

This delivery includes turn-settle v2, all prior stop-sign fixes, and the 0.04%
traffic-light action default. Imported profiles can override that camera default.

## New settings: SET -> Motor motion

- Turn acceleration (duty points/s): how quickly power increases during R/L,
  TR/TL, C/CG turns and ITL/ITR. Straight driving and turn deceleration continue
  to use the normal Acceleration value. Braking and the turn-stop lead are unchanged.
  New 500 RPM profiles start at 35; new 100 RPM profiles at 100. Older tagged
  profiles without this setting inherit their existing acceleration, including
  custom values. Once saved, the two controls are independent.
- Intersection approach speed (% of normal): forward motor-duty scale after the
  stop line and before a turn in TR/TL, C and CG. Default 100 preserves previous
  behavior. A value of 80 commands 80% of each normal straight duty. It does not
  change turn power, straight F route crossings, exits, or drive-to-line behavior.
  The earlier Line approach speed setting remains separate.

Both settings are validated, saved per robot, and apply when leaving SET through
its normal hardware reload. The allowed ranges are 1-500 duty points/s for turn
acceleration and 10-100% for approach. These are motor commands, not measured
vehicle speed/acceleration. The driver still caps late control ticks at 20 ms.

## Install and tune

Extract this ZIP beside the current copy and explicitly transfer your working
500 RPM profile. Check both fields in SET -> Motor motion and save. No automatic
reduction is applied to your calibrated values. Leaving SET requires the usual
IC/IV and runtime VT setup for a camera course.

Check suspected motor/wheel/chassis rubbing with power off before compensating
with tuning. Unequal mechanical drag can cause unequal wheel response, but no
physical cause is confirmed from the logs. Avoid forcing geared wheels by hand.

Change one control at a time. Reducing approach power also reduces distance for
an unchanged right/left approach hold: retune the corresponding Right-turn or
Left-turn approach hold under SET -> Manual maneuvers to reach the same turn point.
There is no automatic distance compensation. Very low turn acceleration may leave
the motors below breakaway power long enough to hit the existing stall guard.

The measured turn direction, stall, timeout and motors-stopped gyro-settling guards
remain. No guard is disabled to compensate for friction. Stop-key and power/IMU/
camera checks stay active. Actual track validation remains required.

## Verification

623 offline tests pass with zero failures/errors/skips. Ten new tests cover older
profile migration and save/load on both robots, validation, independent rising
power with unchanged falling power, late-tick caps, timed and gyro turn execution,
approach-only scaling, full-speed crossings/exits, green-line handoff deceleration
and stop-key response. No new hardware run or deployment was performed.

Earlier copies and ZIPs are unchanged. TURN_CONTROLS_CHANGES.patch compares this
copy against turn-settle v2; older documents/patches describe their parent releases.
HARDENING_VERIFICATION.json and PACKAGE_MANIFEST.json are regenerated for this copy.

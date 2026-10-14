# Stop-sign and turn-transition update, 2026-10-06

This copy includes the previous STOP_SIGN_UPDATE.md changes plus one CG turn fix.
Later on 2026-10-06 Colin requested the traffic-light action default change from
0.08% to 0.04% (fraction 0.0004). Existing/imported profiles can override it; set
SET -> Camera -> Traffic light action size (% of frame) to 0.04 and save.
Stop-sign action size remains 1.2%.
Colin reports the wrong measured direction error before a visible turn.

## Reproduced issue and fix

CG recorded start_angle before prepare() stopped the forward approach and polled
through its direction-change pause. Opposite yaw during that pause counted against
the new turn. An injected IMU transient of four degrees in that pause reproduced
the exception at 0.05 seconds while all turn duties were zero, for left and right.
This synthetic reproduction does not prove the robot experienced that transient.

Record start_angle after prepare() instead. Keep the absolute lane target and all
accumulated heading: this is not an IMU reset. Keep the three-degree wrong-direction
guard, stall guard and timeout unchanged. No wiring/polarity changes.

608 offline tests pass (zero errors, failures or skips). The new tests verify both
transition directions, continued rejection of actual wrong-way powered turns, and
continued detection of a stalled turn. The physical robot has not been tested.

## Use

Extract beside the old folder and explicitly transfer the working 500 RPM profile,
as described in STOP_SIGN_UPDATE.md. This ZIP includes both camera and turn fixes.
Run IC while still, explicitly run IV rotating the whole flat chassis LEFT by
30-90 degrees (counterclockwise viewed from above), and press g before returning it.
IC alone can retain a reversed IV. In IMU, left rotation should increase yaw and
right should decrease it. Then test ITR/ITL in clear space before CG. If those pass
but CG still faults, retain the preceding log and note physical movement.

For the sign, use RVD and verify the WHOLE frame banner; retain RVD stop output and
p snapshots if none. Actual sign recognition remains unverified.

TURN_TRANSITION_CHANGES.patch compares this copy to the prior stop-sign update.
Older patches/docs describe their respective parent deliveries. The verification
and package manifest have been regenerated for this copy. Earlier copies and ZIPs
are unchanged. No GitHub modification or deployment.

# Stop-sign update to the hardened v9.9 copy

2026-10-06. Colin reports RV shows no stop-sign detection in the hardened v9.9
copy. This separate update addresses reproduced detector limitations. It has
not been verified against a frame from Colin's actual sign or on the Pi.

## Changes

- Search the whole rotated camera frame for stop signs. The former search region
  excluded the left half, top 5% and bottom 20%; even a centered sign could be clipped.
- White STOP lettering/borders alone no longer trigger the red-lamp glare veto.
  Bright red pixels must support the white-core glare check. The other brightness,
  size and octagon geometry checks remain. Bright red lamps are still rejected as signs.
- RVD adds `RVD stop:` lines from the actual detector: no red contours, size,
  aspect, glare, octagon-fit rejection, or selected stop-sign box. Snapshot annotations
  now mark a selected stop sign too. RVD remains motors-off.

This is a color/shape detector, not text recognition. It does not read the word
STOP. The lettering fix keeps white text from vetoing an otherwise valid octagon.

The 1.2% stop-sign action threshold and two-of-three confirmation remain. A raw
detected sign can therefore still be too small to act on. No camera exposure,
motor, button, turn, course, deadline, or timeout settings were changed.

## Install beside the current copy

Extract this ZIP into a new folder on the Pi and explicitly transfer the active
500 RPM profile from the current hardened copy. Do not rely on automatic profile
discovery: it does not include the hardened/v9.8 folder names. An existing
destination profile is never overwritten by the importer. Preserve the current
working profile and folder. The main menu still identifies the base as v9.9.

The updated readout banner must say `Stop signs searched across the WHOLE frame`.
If it says RIGHT half, the old copy is running.

## Check the actual sign before another course run

1. Start this copy, select the intended robot/profile, and run `RVD` with motors off.
2. Put the sign at the actual approach distance and angle. Read the `RVD stop:`
   lines and the normal `Camera: ... | confirmed: ...` line.
3. If raw `stop_sign` appears but is not confirmed, compare its displayed frame
   percentage against the action size. This is a different issue from raw `none`.
4. If still undetected, press `p`. The terminal prints a snapshot path on the Pi;
   retain both the plain and annotated images and the rejection text. `s` exits.
5. When the real sign is consistently recognized, recheck actual red/yellow/green
   props before a short powered test. Full-frame search increases the area in
   which red background objects can become candidates.

The separate intersection timeout remains unresolved: capture the exact error.
`Measured turn timed out`, `Correction timed out`, and the hardened timed-forward
deadline represent different failures. Do not increase a timeout to mask it.

Existing limitations remain: CG ToF is not integrated, unknown intermediate lights
can continue after a stop, intermediate signs do not end a course with legs left,
and the final lane may end at an unconfirmed line. START/RESET mapping still needs
to match the actual robot before GO.

The earlier HARDENING.md and HARDENING_CHANGES.patch describe the parent delivery.
STOP_SIGN_CHANGES.patch records this update. HARDENING_VERIFICATION.json is
regenerated for this copy and includes hashes of all Python files recursively.

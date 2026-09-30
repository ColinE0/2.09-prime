# 2.09-prime branch integration

This test bench brings the accepted branch integration into `main`. The source
branch heads were inspected on September 29 and rechecked before publication
on September 30, 2026. `SOURCES.json` preserves their exact commit identities.

## Source boundary

All existing subsystem code comes from
[ColinE0/2.09-prime](https://github.com/ColinE0/2.09-prime).

| Source branch | Commit | Files |
|---|---|---|
| main | ff119bdc7262c2f9893f3c654543f17d1cfcfc0b | Base README and gitignore |
| kws | ef4b1c628bf772d78da88ce455f0768a6ce79eb2 | main.py, five supporting Python modules, README_Motor_IMU.md |
| cew | ae32bf9f6ea40d86a1cbfec0a8308c852e965dbc | tof_sensor.py |
| rr | 31943865d2f9f7e9696ab8360a0f9e2cc6dc10f0 | vision.py, stop_sign_detector.py, traffic_light_detector.py |
| mm | 2f44535af3955c012a2716bba16165765d00bfc6 | Initial scaffold; no additional subsystem files |

`SOURCES.json` records each source file's branch, commit and original SHA-256.
Eight supporting Python modules retain their exact source bytes.
`main.py`, `motors_optimized.py`, `README.md` and the opening scope statement in
`README_Motor_IMU.md` are modified for integration and GPIO handling. This
document and the source manifest are newly written integration documentation.
`tests/` contains the new offline regression checks used to verify the
integration and GPIO changes.

## Connecting code

The `kws` launcher is extended with four numbered menu categories, direct command
codes, a hardware-free `--preview`, and wrappers for `cew` and `rr` readouts.
The original motor and IMU dispatch paths and preset values are retained.

Before a camera or ToF readout, the motor driver is stopped in coast/standby
mode. The existing keyboard reader supplies `s` and the existing control-key
stop behavior. Optional sensor modules load on selection. The camera is stopped
and closed on exit; the ToF bus is released and its hold state reset. Partial
setup and read failures also run cleanup. Cleanup failures end the session.
Program exit releases motor GPIO before waiting for the IMU worker, and attempts
both cleanup operations even if either fails.

The September 30 update separates camera and ToF functions, expands the menu
tables and conditionals, and adds named comment sections. It replaces dynamic
sensor imports with ordinary imports inside the appropriate readout function.

The motor driver now checks for duplicate/reserved pins, an incompatible
numbering mode, and existing output/alternate functions before setup. It freezes
its active pin assignments and polarity until close. Setup warnings abort;
partial initialization also runs cleanup. GPIO write errors latch a fault and
end the session. Stop and close attempt the remaining PWM and GPIO operations
even when one fails. The original wiring defaults and motion profiles remain.

These are newly written changes to prime's existing files. No external
implementation was copied into the package.

## Scope

This combines subsystem testing in one program. Camera and ToF are readouts
only: no motion command consumes their output. There is no autonomous course
runner or sensor-based driving policy. `F` still drives until the stop key.
There are no added IR, battery-monitoring, wheel-calibration, saved-profile or
settings-editor features. Robot presets come directly from `motion_profile.py`.

Offline checks use simulated interfaces and validate menu routing, preset
application, sensor lifecycle and cleanup. Actual Pi and robot behavior remains
unverified. No hardware was operated to prepare this preview.

## Preview verification

The September 29 preview passed 21 integration checks. On September 30, all 46
checks passed, including GPIO fault injection and full launcher/driver shutdown. All 10
application Python files compile. Nine imported files (eight Python modules
and gitignore) still match their prime source bytes. Both robot presets can be
previewed without GPIO, camera or I2C imports. The regression tests are included
in `tests/`; the README gives their command. The source branches are recorded
snapshots, so later changes on those branches require an explicit integration.

GPIO mode checks do not reserve pins against another process or detect external
wiring faults. A forced kill, interpreter crash or power loss can bypass Python
cleanup. Per-pin cleanup behavior was checked against RPi.GPIO 0.7.1's published
source, but no Pi library or electrical tests were run. Hardware standby behavior
and pin voltage/current protection cannot be established by these offline tests.

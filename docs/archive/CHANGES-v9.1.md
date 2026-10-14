> Historical document. For current operation, see the [navigation guide](../../README.md).

# v9.1 changes (2026-09-23)

Built on `MPU6050_Motor_Test_v9.zip` unmodified (the first commit in this folder's
local git history is that baseline, so `git diff` shows every change). Offline
tests: 344 pass (v9's 317, adjusted where behaviour deliberately changed, plus
24 new in `test_v9_1.py`). **Not yet run on either robot.**

## 1. Power guard: current and voltage stop while driving

`power_guard.py`, hooked into every control loop through `Controller.poll()`.

- Runs around every command that can drive a wheel: F, FA, SR, SL, R, L, TR,
  TL, DR, DL, C, FL, FR, BL, BR, FC, IF, ITL, ITR. Motors-off readouts (IR, IRC,
  IMU, RV, TOF, BAT) do not use it.
- Reads the board's INA219 through the existing BAT worker, about every 20 ms.
  Motion waits for the first reading; if none arrives in 1.5 s the command is
  refused.
- Trips when battery current stays at or above the limit for the hold time
  (default **4.5 A for 0.2 s**), when the pack stays at or below the voltage
  limit (default **6.4 V for 2 s**, 3.2 V per cell under load), when the
  reading goes stale (over 0.5 s old) while driving, or when the INA219 fails
  mid-drive. A shunt reading at its 80 mV limit counts as overcurrent.
- A trip parks every PWM, drops STBY (coast, not brake) and ends the command
  with the reason. The motors stay off and STBY stays low.
- After every powered command it prints `Power: peak X A, lowest Y V`. That line
  is a measurement, and the same numbers settle the stall-current question.
- Per robot in **SET -> Power protection**: on/off, current limit and hold,
  voltage limit and hold. Switching it off prints a warning at startup and on
  every powered command.
- It is software only. It does nothing before the program runs, and it does
  not replace a fuse (the battery line still has none). 4.5 A is the modeled Q1
  limit, not a measured stall current. For the clamped-rotor demo, set the limit
  from the measured stall current.

## 2. ToF: open space reads clear

`tof_support.classify()` sorts every VL53L0X result into four kinds:

- **range**: status 11, 1 to 2000 mm. Unchanged.
- **clear**: nothing returned within range (status 4, 6, 9 or 14, or status 11
  beyond 2000 mm). This is the normal case down an open lane. v9 held and then
  ended FC after 1 s here, which made FC with TT unusable.
- **close**: status 8 or 10 (a target inside the minimum range). Holds.
- **unknown**: everything else (hardware failures, zero distance, unexplained
  codes). Holds, then ends FC after 1 s, as v9 did.

`TOF` shows "no target in range" or "too close to measure" beside the raw status.
Check it on the robot before turning TT on: a box ahead should read in cm, and
an empty lane should read "no target in range".

## 3. New profiles start on the documented rev C map

- Motor channels default to `M1A,M1B,M2B,M2A` (from behind: J2 front-left,
  J4 front-right, J12 back-left, J5 back-right). v9 defaulted to
  `M1A,M1B,M2A,M2B`, which swaps the back pair.
- IR roles default to `J7,J10,J9` (LEFT, RIGHT, FRONT). v9 defaulted to
  `J10,J9,J7`.
- Saved profiles keep their own map. At startup, a profile that differs from
  the documented map prints a note saying so. The single-wheel and tape checks
  still decide what is true on each robot.

## 4. Turns: measured with the gyro, and ITL/ITR stop lead

- With IC and IV current, R, L, TR, TL and every course turn print
  `Turn <label>: measured X deg (+/-Y vs 90) | yaw on the straight before it Z deg
  | heading vs plan W deg`. This is read-only: it never steers, and any IMU
  problem just stops the logging. Per-turn angles are reliable. Heading against
  the plan also carries gyro drift over a run. A course ends with a one-line
  summary. Off switch: SET -> IMU motion -> Measure timed turns with the gyro.
  - This is the capture the left-turn fault needs. Two good left turns and a
    bad one now come with numbers.
  - The yaw on each straight shows whether the lane keeper's corrections are
    rotating the robot. They should not, because the lean is meant to be
    rotation-free.
- ITL/ITR print the coast after cutoff and a suggested **IMU turn stop lead**.
  Set it in SET -> IMU motion so the turn cuts power early enough to land near
  90 degrees. The default of 0 keeps v9's behaviour.

## Also

- `BatteryStream` takes a sampling `period` (BAT keeps 0.1 s).
- Menu and help name v9.1.
- Menu-routing tests use `fake_power.FakeGuard`, since there is no INA219
  off the Pi.

#!/usr/bin/env python3
"""Motor and MPU6050 test bench: `python3 main.py [--bot 500|100]` on the Pi."""

import argparse
import contextlib
import signal
import sys
import time

import motion_profile
import motors_optimized as motor
import imu_tests
import single_motor
from imu_support import IMUFault
from imu_tests import IMU_COMMANDS, IMUTests, Session, TurnYawLogger
from single_motor import SINGLE_MOTOR_COMMANDS, SingleMotorTests

TURN_COMMANDS = {"R", "L", "TR", "TL"}
COMMANDS = motor.MODES | IMU_COMMANDS | SINGLE_MOTOR_COMMANDS

MENU = """
Motors      F forward (until 's') | SR/SL strafe | R/L spin | TR/TL intersection turn | DR/DL diagonal
Wheels      FL FR BL BR: one wheel forward for 1 s (lift the wheels)
IMU         ID identify | IMU live readings | IC calibrate (still) | IV verify axis (turn by hand)
IMU motion  IF heading-hold forward | ITL/ITR 90-degree gyro turns (IC and IV first)
            Q quit"""


def apply_preset(bot_id):
    """Point every module's tunables at the selected robot's preset."""
    preset = motion_profile.PRESETS[bot_id]
    motor.LEFT_SPEED = preset["left_speed"]
    motor.RIGHT_SPEED = preset["right_speed"]
    motor.RAMP_RATE = preset["ramp_rate"]
    motor.DIAGONAL_SECONDARY_SPEED = preset["diagonal_secondary_speed"]
    single_motor.TEST_DUTY = preset["single_motor_duty"]
    imu_tests.IMU_DRIVE_DUTY = preset["imu_drive_duty"]
    imu_tests.TURN_MAX_DUTY = preset["imu_turn_max_duty"]
    imu_tests.TURN_MIN_DUTY = preset["imu_turn_min_duty"]
    imu_tests.HEADING_KP = preset["imu_heading_kp"]
    imu_tests.HEADING_KD = preset["imu_heading_kd"]
    imu_tests.MAX_HEADING_TRIM = preset["imu_max_heading_trim"]
    imu_tests.TURN_SLOW_GAIN = preset["imu_turn_slow_gain"]


STOP_SIGNALS = ("SIGHUP", "SIGTERM", "SIGQUIT")


@contextlib.contextmanager
def stop_on_signals():
    """Run the Ctrl+C cleanup for a closed SSH window (SIGHUP), kill (SIGTERM) or SIGQUIT.

    Their default action ends the process without any finally block, leaving STBY
    high and PWM pins latched. The first signal raises KeyboardInterrupt; later
    ones are ignored so they cannot cut the cleanup short.
    """
    previous = {}

    def interrupt(signum, frame):
        for number in previous:
            signal.signal(number, signal.SIG_IGN)
        raise KeyboardInterrupt(f"signal {signum}")

    try:
        for name in STOP_SIGNALS:
            number = getattr(signal, name, None)
            if number is not None:
                try:
                    previous[number] = signal.signal(number, interrupt)
                except (OSError, ValueError):
                    pass  # not the main thread, or not supported on this platform
        yield
    finally:
        for number, handler in previous.items():
            signal.signal(number, handler)


def imu_status(session):
    """Inspect cached calibration only; drawing the menu never starts an IMU read."""
    calibration = session.calibration
    if calibration is None:
        return "needs IC"
    try:
        calibration.require_ready(time.monotonic(), orientation=False)
    except IMUFault:
        return "expired - run IC"
    return "needs IV" if calibration.axis is None else "ready"


def dispatch(command, motors, keys, session):
    if command in SINGLE_MOTOR_COMMANDS:
        SingleMotorTests(motors, keys).run_test(command)
    elif command in IMU_COMMANDS:
        IMUTests(motors, keys, session).run_test(command)
    else:
        controller = motor.Controller(motors, keys)
        if command in TURN_COMMANDS:
            controller.turn_observer = TurnYawLogger(session)
            controller.turn_observer.start()
        controller.run_mode(command)


def run(args, GPIO):
    motors = motor.MotorDriver(GPIO)
    session = Session(args.bus, args.address)
    result = 0
    try:
        motors.initialize()
        while True:
            print(MENU)
            print(f"IMU: {imu_status(session)}")
            parts = motor.prompt_line("Enter choice: ").strip().upper().split()
            if not parts:
                continue
            command = parts[0]
            if command == "Q":
                break
            if command not in COMMANDS:
                print("Unknown command")
                continue
            print("Press 's' to stop and return to the menu.")
            try:
                with motor.Keyboard() as keys:
                    dispatch(command, motors, keys, session)
                print(f"{command} finished.")
            except motor.StopRequested:
                print(f"{command}: stopped.")
            except (IMUFault, motor.LaneFault, ValueError) as exc:
                print("Stopped: " + str(exc))
            finally:
                motors.stop()
    except (KeyboardInterrupt, EOFError):
        print("\nExiting")
    except Exception as exc:
        print(f"Test error: {exc}", file=sys.stderr)
        result = 1
    finally:
        # Close the IMU worker and release GPIO even after an error.
        if session.close() is False:
            print("IMU reader did not close; restart before running again.", file=sys.stderr)
            result = 1
        if not motors.close():
            result = 1
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description="Motor and MPU6050 test bench")
    parser.add_argument("--bot", choices=("500", "100"), help="Select robot; otherwise ask at startup")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus (default: 1)")
    parser.add_argument("--address", type=lambda s: int(s, 0), default=0x68,
                        choices=(0x68, 0x69), help="MPU6050 address")
    args = parser.parse_args(argv)
    if args.bus < 0:
        parser.error("--bus must be nonnegative")
    if not sys.stdin.isatty():
        print("Run in an interactive terminal on the Raspberry Pi.", file=sys.stderr)
        return 1
    try:
        import RPi.GPIO as GPIO
    except (ImportError, RuntimeError) as exc:
        print(f"Cannot load RPi.GPIO: {exc}", file=sys.stderr)
        return 1
    try:
        with stop_on_signals():
            choice = args.bot
            while True:
                if choice is None:
                    choice = motor.prompt_line("Robot: 1 = 500 RPM, 2 = 100 RPM (blank quits): ").strip()
                    if not choice:
                        return 0
                try:
                    bot_id = motion_profile.normalize_bot_id(choice)
                    break
                except ValueError as exc:
                    print(exc)
                    choice = None
            apply_preset(bot_id)
            print(f"Selected robot: {motion_profile.BOT_LABELS[bot_id]}")
            return run(args, GPIO)
    except (KeyboardInterrupt, EOFError):
        print("\nExiting")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())

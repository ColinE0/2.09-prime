#!/usr/bin/env python3
"""Prime branch integration preview: `python3 main.py [--bot 500|100]`."""

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

# ============================================================
# Menu commands
# ============================================================

TURN_COMMANDS = {"R", "L", "TR", "TL"}
SENSOR_COMMANDS = {"TOF", "VISION"}
COMMANDS = motor.MODES | IMU_COMMANDS | SINGLE_MOTOR_COMMANDS | SENSOR_COMMANDS

MOTOR_MENU = (
    ("F", "Forward until s"),
    ("SR", "Strafe right"),
    ("SL", "Strafe left"),
    ("R", "Timed spin right"),
    ("L", "Timed spin left"),
    ("TR", "Forward / right turn / forward"),
    ("TL", "Forward / left turn / forward"),
    ("DR", "Diagonal right"),
    ("DL", "Diagonal left"),
    ("FL", "Front left wheel, 1 s"),
    ("FR", "Front right wheel, 1 s"),
    ("BL", "Back left wheel, 1 s"),
    ("BR", "Back right wheel, 1 s"),
)

IMU_MENU = (
    ("ID", "Identify MPU6050"),
    ("IMU", "Live IMU readings, motors off"),
    ("IC", "Calibrate gyro, keep still"),
    ("IV", "Verify yaw axis by hand"),
    ("IF", "Heading-hold forward, 2 s; IC and IV first"),
    ("ITL", "90-degree gyro turn left; IC and IV first"),
    ("ITR", "90-degree gyro turn right; IC and IV first"),
)

TOF_MENU = (
    ("TOF", "Live distance / obstacle / speed scale"),
)

CAMERA_MENU = (
    ("VISION", "Live stop-sign and traffic-light detections"),
)

CATEGORIES = {
    "1": ("Motor tests", MOTOR_MENU),
    "2": ("IMU tests and calibration", IMU_MENU),
    "3": ("ToF readout (motors off)", TOF_MENU),
    "4": ("Camera readout (motors off)", CAMERA_MENU),
}


# ============================================================
# Display the menu and read a choice
# ============================================================

def menu_text(bot_id, status, category=None):
    """Build the current menu without opening any hardware."""
    lines = [
        "\n2.09 PRIME | INTEGRATION PREVIEW",
        f"Active robot: {motion_profile.BOT_LABELS[bot_id]}",
        f"IMU: {status}",
        "Camera / ToF: readouts only; motor commands do not use them.",
    ]

    if category is None:
        for number, menu in CATEGORIES.items():
            title, commands = menu
            lines.append(f"  {number}. {title}")

    else:
        title, commands = CATEGORIES[category]
        lines.append(title)

        for number, item in enumerate(commands, 1):
            code, description = item
            lines.append(f"  {number}. {code}: {description}")

    lines.append("0 Home | H All commands | Q Quit | Command codes work anywhere")

    return "\n".join(lines)


def print_all_commands():
    """Show the command codes grouped by what they do."""
    for title, commands in CATEGORIES.values():
        print("\n" + title)

        for code, description in commands:
            print(f"  {code}: {description}")


def resolve_choice(choice, category):
    """Turn a menu number into its command, or change menu pages."""
    choice = choice.strip().upper()

    if choice == "0":
        return None, None

    if category is None and choice in CATEGORIES:
        return choice, None

    if category is not None and choice.isdecimal():
        commands = CATEGORIES[category][1]
        index = int(choice) - 1

        if 0 <= index < len(commands):
            code, description = commands[index]
            return category, code

        return category, ""

    return category, choice


# ============================================================
# Apply the selected robot's starting values
# ============================================================

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


# ============================================================
# Stop and clean up when the terminal closes
# ============================================================

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
    if calibration.axis is None:
        return "needs IV"

    return "ready"


# ============================================================
# ToF readout, with the motors off
# ============================================================

def run_tof_readout(motors, keys):
    """Print the existing ToF results until the stop key is pressed."""
    motors.stop(brake=False)
    tof = None

    try:
        # Import here so other tests can run without the ToF driver installed.
        import tof_sensor as tof

        control = motor.Controller(motors, keys)
        control.poll()

        tof.reset_tof_state()
        tof.setup_tof()
        print("TOF: motors off; press s to return.")

        while True:
            control.poll()
            data = tof.read_tof()
            control.poll()
            tof.print_tof_data(data)
            time.sleep(0.05)

    except motor.StopRequested:
        raise

    except Exception as exc:
        print(f"TOF unavailable: {type(exc).__name__}: {exc}")
        return False

    finally:
        if tof is not None:
            try:
                tof.stop_tof()
            except Exception as exc:
                raise RuntimeError(f"TOF cleanup failed; end this session: {exc}") from exc


# ============================================================
# Camera readout, with the motors off
# ============================================================

def run_camera_readout(motors, keys):
    """Print the existing camera detections until the stop key is pressed."""
    motors.stop(brake=False)
    camera = None

    try:
        # The camera libraries are needed only when this readout is selected.
        import vision as camera

        control = motor.Controller(motors, keys)
        control.poll()

        camera.setup_camera()
        print("VISION: motors off; press s to return.")

        while True:
            control.poll()
            data = camera.read_vision()
            control.poll()
            camera.print_vision_data(data)

    except motor.StopRequested:
        raise

    except Exception as exc:
        print(f"VISION unavailable: {type(exc).__name__}: {exc}")
        return False

    finally:
        if camera is not None and camera.picam2 is not None:
            try:
                try:
                    camera.picam2.stop()
                finally:
                    camera.picam2.close()

                camera.picam2 = None

            except Exception as exc:
                raise RuntimeError(f"VISION cleanup failed; end this session: {exc}") from exc


def sensor_readout(command, motors, keys):
    if command == "TOF":
        return run_tof_readout(motors, keys)

    if command == "VISION":
        return run_camera_readout(motors, keys)

    raise ValueError("Choose TOF or VISION")


# ============================================================
# Run the selected test
# ============================================================

def dispatch(command, motors, keys, session):
    if command in SENSOR_COMMANDS:
        return sensor_readout(command, motors, keys)
    elif command in SINGLE_MOTOR_COMMANDS:
        SingleMotorTests(motors, keys).run_test(command)
    elif command in IMU_COMMANDS:
        IMUTests(motors, keys, session).run_test(command)
    else:
        controller = motor.Controller(motors, keys)
        if command in TURN_COMMANDS:
            controller.turn_observer = TurnYawLogger(session)
            controller.turn_observer.start()
        controller.run_mode(command)


# ============================================================
# Main menu loop and shutdown
# ============================================================

def run(args, GPIO):
    motors = motor.MotorDriver(GPIO)
    session = Session(args.bus, args.address)
    result = 0
    category = None
    try:
        motors.initialize()
        while True:
            print(menu_text(args.bot_id, imu_status(session), category))
            choice = motor.prompt_line("Enter choice: ").strip().upper()
            if not choice:
                continue
            category, command = resolve_choice(choice, category)
            if command is None:
                continue
            if command == "Q":
                break
            if command == "H":
                print_all_commands()
                continue
            if command not in COMMANDS:
                print("Unknown command")
                continue
            print("Press 's' to stop and return to the menu.")
            try:
                with motor.Keyboard() as keys:
                    completed = dispatch(command, motors, keys, session)
                motors.stop()
                if completed is not False:
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
        # Release motor GPIO before waiting for sensor workers. Attempt both even
        # when one cleanup fails; never let an IMU-close error skip motor cleanup.
        try:
            if motors.close() is False:
                print("Motor GPIO did not close; check the hardware before restarting.", file=sys.stderr)
                result = 1

        except Exception as exc:
            print(f"Motor GPIO cleanup failed: {exc}", file=sys.stderr)
            result = 1

        try:
            if session.close() is False:
                print("IMU reader did not close; restart before running again.", file=sys.stderr)
                result = 1

        except Exception as exc:
            print(f"IMU reader cleanup failed: {exc}", file=sys.stderr)
            result = 1

    return result


# ============================================================
# Startup options and robot selection
# ============================================================

def parse_i2c_address(value):
    """Accept an address written as 0x68 or as a decimal number."""
    return int(value, 0)


def main(argv=None):
    parser = argparse.ArgumentParser(description="2.09-prime motor, IMU, ToF and camera bench")
    parser.add_argument("--bot", choices=("500", "100"), help="Select robot; otherwise ask at startup")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus (default: 1)")
    parser.add_argument("--address", type=parse_i2c_address, default=0x68,
                        choices=(0x68, 0x69), help="MPU6050 address")
    parser.add_argument("--preview", action="store_true", help="Show menu and presets without hardware")
    args = parser.parse_args(argv)
    if args.bus < 0:
        parser.error("--bus must be nonnegative")
    if args.preview:
        if args.bot:
            bots = [motion_profile.normalize_bot_id(args.bot)]
        else:
            bots = motion_profile.BOT_IDS

        for bot_id in bots:
            print(menu_text(bot_id, "needs IC"))
            print("Starting preset: " + str(motion_profile.PRESETS[bot_id]))
        return 0
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
                    if not choice or choice.upper() in {"Q", "0"}:
                        return 0
                try:
                    bot_id = motion_profile.normalize_bot_id(choice)
                    break
                except ValueError as exc:
                    print(exc)
                    choice = None
            apply_preset(bot_id)
            args.bot_id = bot_id
            print(f"Selected robot: {motion_profile.BOT_LABELS[bot_id]}")
            return run(args, GPIO)
    except (KeyboardInterrupt, EOFError):
        print("\nExiting")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())

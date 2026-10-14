"""Application setup and command dispatch for the terminal robot test bench."""

import argparse
import contextlib
import json
from pathlib import Path
import signal
import sys
import time

import motors_optimized as motor
import robot_settings
import calibration_data
from pwm_calibration import calibrate_profile, check_probe, probe_pin
from terminal_ui import TerminalMenu, LiveStatus, edit_settings, choose_robot
import robot_profiles
from battery_support import (BatteryFault, BatterySession, load_battery_profile,
                             battery_status, check_battery_sample)
from ir_confidence import CONFIDENCE_COMMANDS, ConfidenceTests
from lane_gyro import GYRO_LANE_COMMANDS, GyroLane
import vision_support
from vision_support import CameraOptions, LIGHT_COMMANDS, VisionFault, VisionSession
from tof_support import ToFFault, ToFSession
from single_motor import SINGLE_MOTOR_COMMANDS, SingleMotorTests
from board_ui import BUTTON_COMMANDS, ButtonTests
from go_button import (COUNTDOWN_SECONDS, COURSE_COMMANDS, GO_COMMANDS, IMU_COMMANDS as GO_IMU_COMMANDS,
                       IMU_MARGIN_SECONDS, armed_session, resolve as resolve_go)
from imu_support import CALIBRATION_LIFETIME, IMUFault
from power_guard import PowerFault, PowerGuard, GUARD_PERIOD
import wiring
from imu_tests import (Session, IMUTests, IMU_COMMANDS, forward_targets,
                       IMU_DRIVE_DUTY, IMU_DRIVE_SECONDS, CALIBRATION_SECONDS,
                       CALIBRATION_SETTLE_SECONDS, TURN_TIMEOUT, TurnYawLogger)

PACKAGE_DIR = Path(__file__).resolve().parent
# Commands that can drive a wheel run under the power guard.
POWERED_COMMANDS = (motor.MODES - {"IR"}) | SINGLE_MOTOR_COMMANDS | GYRO_LANE_COMMANDS | {"FC", "IF", "ITL", "ITR"}
TURN_COMMANDS = {"R", "L", "TR", "TL", "C"}


def build_parser():
    parser = argparse.ArgumentParser(description="Robot Test Bench v9.10: settled gyro turns with turn acceleration and approach controls, camera traffic lights and the stop-sign course end, FG/CG gyro lanes, GO start button and independent robot profiles")
    parser.add_argument("--check-ina", action="store_true", help="Check INA219 without opening motor GPIO or requiring a terminal")
    parser.add_argument("--bot", choices=("500", "100"), help="Select robot; otherwise ask at startup")
    parser.add_argument("--settings", type=Path, help="Custom settings for --bot; otherwise profiles/<bot>rpm.json")
    parser.add_argument("--import-settings", type=Path, metavar="JSON", help="Copy settings into the selected bot profile, then exit")
    parser.add_argument("--import-v7", type=Path, metavar="FOLDER",
                        help="Read literal tuning values from v7, create settings, then exit; never executes v7")
    parser.add_argument("--show-settings", action="store_true", help="Print saved/default settings without opening hardware")
    parser.add_argument("--plain-output", action="store_true", help="Print every full reading on a new line for copying logs")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus (default: 1)")
    parser.add_argument("--address", type=lambda s: int(s, 0), default=0x68,
                        choices=(0x68, 0x69), help="MPU6050 address")
    parser.add_argument("--battery-address", type=lambda s: int(s, 0), default=0x40,
                        choices=range(0x40, 0x50), help="INA219 address")
    parser.add_argument("--shunt-ohms", type=float, default=0.01, help="INA219 shunt (R010 = 0.01 ohm)")
    parser.add_argument("--battery-profile", default=str(PACKAGE_DIR / "battery_profile.json"),
                        help="Battery profile JSON, or 'none' for voltage/current only")
    parser.add_argument("--camera-backend", choices=("csi", "usb"), default=None)
    parser.add_argument("--camera-index", type=int, default=None)
    parser.add_argument("--camera-upright", action="store_true", default=None,
                        help="Disable 180-degree rotation for this launch")
    parser.add_argument("--camera-auto-exposure", action="store_true", default=None,
                        help="Use automatic exposure for this launch")
    parser.add_argument("--tof-address", type=lambda s: int(s, 0), default=0x29,
                        choices=range(0x08, 0x78), help="VL53L0X address")
    return parser


def load_configuration(args):
    """Load only the selected robot; preview is read-only."""
    bot_id = getattr(args, "bot", None) or "500"
    custom_bot = getattr(args, "custom_settings_bot", bot_id)
    custom_path = args.settings if custom_bot == bot_id else None
    path, config, report = robot_profiles.load_profile(
        PACKAGE_DIR, bot_id, settings_path=custom_path,
        import_v7=args.import_v7, import_settings=getattr(args, "import_settings", None),
        persist=not args.show_settings)
    if not args.show_settings:
        for line in report:
            print(line)
    return path, config


def camera_options(config, args):
    return CameraOptions(
        backend=args.camera_backend if args.camera_backend is not None else config["camera_backend"],
        index=args.camera_index if args.camera_index is not None else config["camera_index"],
        rotate_180=False if args.camera_upright else config["camera_rotate_180"],
        auto_exposure=True if args.camera_auto_exposure else config["camera_auto_exposure"],
        exposure_us=config["camera_exposure_us"],
        analogue_gain=config["camera_analogue_gain"],
    )


STOP_SIGNALS = ("SIGHUP", "SIGTERM", "SIGQUIT")


@contextlib.contextmanager
def stop_on_signals():
    """Run the Ctrl+C cleanup for a closed SSH window (SIGHUP), kill (SIGTERM) or SIGQUIT.

    Their default action ends the process without any finally block, leaving STBY
    high and PWM pins latched with no stop key, IR stop or power guard. The first
    signal raises KeyboardInterrupt; later ones are ignored so they cannot cut the
    cleanup short. SIGKILL cannot be caught: the battery plug stays the hardware stop.
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


def diagnose_ina(args):
    """Bounded standalone check: report the actual bus fault, never open GPIO."""
    battery = BatterySession(args.bus, args.battery_address, args.shunt_ohms)
    print(f"Checking INA219: I2C bus {args.bus}, address 0x{args.battery_address:02X}, "
          f"shunt {args.shunt_ohms:g} ohm. Motor GPIO is not opened.")
    result = 1
    try:
        stream = battery.start()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            sample = stream.latest()
            if sample is not None:
                check_battery_sample(sample, time.monotonic())
                print("INA219 check passed. " + battery_status(sample))
                result = 0
                break
            time.sleep(0.01)
        else:
            raise BatteryFault("No INA219 reading within 2 seconds; check I2C, address and sensor wiring")
    except (BatteryFault, OSError, ValueError) as exc:
        print(f"INA219 check failed: {exc}", file=sys.stderr)
    finally:
        if battery.close() is False:
            print("INA219 worker did not close; exit before trying again.", file=sys.stderr)
            result = 1
    return result


def imu_status(session, now=None):
    """Inspect cached calibration only; drawing the menu never starts an IMU read."""
    calibration = session.calibration
    if calibration is None:
        return "Needs IC"
    try:
        calibration.require_ready(time.monotonic() if now is None else now, orientation=False)
    except IMUFault:
        return "Expired - run IC"
    return "Needs IV" if calibration.axis is None else "Ready"


def with_camera(control, command, vision, run):
    """VT: a lane mode starts the camera first, so its light decides each front line."""
    if vision.enabled and command in LIGHT_COMMANDS:
        return vision.attach(control, run)
    return run()


def camera_first(command, vision, tof=None):
    """GO: true when the run starts the camera (or FC's sensors) before it may move."""
    return (command == "FC" or (command == "CG" and tof is not None and tof.enabled)
            or (vision.enabled and command in LIGHT_COMMANDS))


def dispatch(command, sequence, motors, keys, session, battery, vision, tof, log):
    """Run one menu command; with VT on, lane modes start the camera first."""
    if command in SINGLE_MOTOR_COMMANDS:
        SingleMotorTests(motors, keys, log=log).run_test(command)
    elif command == "RV":
        vision.readings(motor.Controller(motors, keys, log=log))
    elif command == "RVD":
        vision.readings(motor.Controller(motors, keys, log=log), debug=True)
    elif command == "TOF":
        tof.readings(motor.Controller(motors, keys, log=log))
    elif command == "FC" and tof.enabled:
        tof.forward(ConfidenceTests(motors, keys, log=log), vision)
    elif command == "FC" and vision.enabled:
        vision.forward(ConfidenceTests(motors, keys, log=log))
    elif command in CONFIDENCE_COMMANDS:
        ConfidenceTests(motors, keys, log=log).run_test(command)
    elif command in {"BAT", "B"}:
        battery.readings(motor.Controller(motors, keys, log=log))
    elif command in BUTTON_COMMANDS:
        ButtonTests(motors, keys, log=log).run_test(command)
    elif command in IMU_COMMANDS:
        IMUTests(motors, keys, session, log=log).run_test(command)
    elif command in GYRO_LANE_COMMANDS:
        control = GyroLane(motors, keys, session, log=log)
        if command == "CG" and tof is not None and tof.enabled:
            def run_cg():
                if vision.enabled:
                    return with_camera(control, command, vision, lambda: control.run(command, sequence))
                control.go_ready()
                return control.run(command, sequence)
            tof.attach(control, run_cg)
        else:
            with_camera(control, command, vision, lambda: control.run(command, sequence))
    else:
        controller = motor.Controller(motors, keys, log=log)
        if command in TURN_COMMANDS:
            controller.turn_observer = TurnYawLogger(session, log)
            controller.turn_observer.start()
        with_camera(controller, command, vision, lambda: controller.run_mode(command, sequence))


def calibration_segment(args, motors, battery, config, action):
    """One wizard stage: stop keys and INA protection stay active during motion.

    Human placement prompts are outside this scope. A blocked INA worker or
    GPIO cleanup failure escapes to the app's fatal cleanup; no next stage runs.
    """
    guard = None
    motors.stop(brake=False)
    try:
        with motor.Keyboard() as keys:
            if config["power_guard"]:
                print("Checking INA219 before calibration; s cancels.")
                guard = PowerGuard.from_settings(config, bus=args.bus, address=args.battery_address,
                    shunt_ohms=args.shunt_ohms, stream_factory=lambda: battery.start(period=GUARD_PERIOD))
                guard.wait_ready(poll=motor.Controller(motors, keys).poll)
                motors.power_guard = guard
            else:
                print("Power guard OFF for this robot, as selected in SET.")
            action(keys)
    finally:
        try:
            motors.stop(brake=False)
        finally:
            motors.power_guard = None
            if guard is not None:
                if guard.close() is False:
                    raise RuntimeError("INA219 reader did not close; exit before further calibration")
                summary = guard.summary()
                if summary:
                    print(summary)


def run_robot(args, bot_id, GPIO=None):
    """Own one robot session. Close every resource before another can start."""
    args = argparse.Namespace(**vars(args))
    args.bot = bot_id
    try:
        settings_path, config = load_configuration(args)
        applied = robot_settings.apply_settings(config)
    except (OSError, ValueError, SyntaxError, KeyError, TypeError) as exc:
        print(f"Settings error: {exc}. Hardware was not opened.", file=sys.stderr)
        return 1
    robot_label = robot_profiles.BOT_LABELS[robot_profiles.normalize_bot_id(bot_id)]
    print(f"Selected robot: {robot_label}")
    print("Configured motor channels (FL, FR, BL, BR): " + robot_settings.format_value("motor_channels", config["motor_channels"]))
    print("Configured direction reversals (FL, FR, BL, BR): " + robot_settings.format_value("reversed", config["reversed"]))
    print("Configured IR roles (LEFT, RIGHT, FRONT): " + robot_settings.format_value("ir_pins", config["ir_pins"]))
    print("Check IR and each lifted-wheel test after wiring changes; software cannot verify physical positions.")
    print(f"Manual L/R {config['left_speed']:g}/{config['right_speed']:g}% | "
          f"FC {config['fc_max_duty']:g}% | wheel test {config['single_motor_duty']:g}% | "
          f"IMU forward/turn {config['imu_drive_duty']:g}/{config['imu_turn_max_duty']:g}%")
    if applied["power_guard"]:
        print(f"Power guard ON: stops at {applied['power_overcurrent_a']:g} A for {applied['power_overcurrent_hold_s']:g} s, "
              f"or {applied['power_low_voltage_v']:g} V for {applied['power_low_voltage_hold_s']:g} s (SET -> Power protection)")
    else:
        print("Power guard OFF for this robot: nothing stops the motors on overcurrent or a low pack.")
    if tuple(applied["motor_channels"]) != wiring.DOCUMENTED_CHANNELS:
        print("Note: motor channels differ from the documented rev C map "
              + ",".join(wiring.DOCUMENTED_CHANNELS)
              + " (from behind: J2 front-left, J4 front-right, J12 back-left, J5 back-right)."
              " Confirm with FL, FR, BL, BR, wheels lifted.")
    if tuple(applied["ir_pins"]) != wiring.DOCUMENTED_IR_PINS:
        print("Note: IR roles differ from the documented map J7,J10,J9 (LEFT, RIGHT, FRONT)."
              " Confirm with the tape check in IR.")
    if GPIO is None:
        try:
            import RPi.GPIO as GPIO
        except (ImportError, RuntimeError) as exc:
            print(f"Cannot load RPi.GPIO: {exc}", file=sys.stderr)
            return 1
    try:
        profile = load_battery_profile(None if args.battery_profile.lower() == "none" else args.battery_profile)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"Battery profile unavailable: {exc}. BAT will show voltage/current only.")
        profile = None
    initial_config = dict(config)
    options = camera_options(config, args)
    exposure = "auto" if options.auto_exposure else f"fixed {options.exposure_us:g} us / gain {options.analogue_gain:g}"
    camera_summary = f"{options.backend.upper()} {options.index} | rotation {'180' if options.rotate_180 else '0'} deg | exposure {exposure}"
    owners = []
    result = 0
    cleanup_ok = True
    guard_blocked = False
    try:
        motors = motor.MotorDriver(GPIO)
        owners.append(("motors", motors))
        session = Session(args.bus, args.address)
        owners.append(("IMU", session))
        battery = BatterySession(args.bus, args.battery_address, args.shunt_ohms, profile)
        owners.append(("battery", battery))
        vision = VisionSession(options)
        owners.append(("camera", vision))
        tof = ToFSession(args.bus, args.tof_address)
        owners.append(("ToF", tof))
        menu = TerminalMenu(read=motor.prompt_line)
        last_result = ""
        last_drive = None  # (command, course): what GO arms without an argument

        def execute(command, sequence, live, keys=None, before=None, after=None):
            """Run one command under the power guard; return (result line, how it ended).

            The menu opens a fresh Keyboard; GO passes its button-aware keys, a ``before``
            (its countdown, once the guard is ready) and an ``after`` (its timer stop, the
            moment the command returns or raises). The ending is finished, stopped, fault
            or power.
            """
            nonlocal guard_blocked
            guard = None
            power_tripped = False
            try:
                if guard_blocked and command in POWERED_COMMANDS | {"BAT", "B"}:
                    raise PowerFault("A previous INA219 read is still blocked; restart the program first.")
                with motor.Keyboard() if keys is None else contextlib.nullcontext(keys) as keys:
                    if command in POWERED_COMMANDS:
                        motors.stop(brake=False)
                        if applied["power_guard"]:
                            live("Checking INA219 before motion; 's' cancels. BAT and SET remain available after an error.")
                            # BAT and the guard share one tracked owner. A blocked BAT
                            # reader cannot race a second reader or a new bot session.
                            guard = PowerGuard.from_settings(applied, bus=args.bus, address=args.battery_address,
                                shunt_ohms=args.shunt_ohms,
                                stream_factory=lambda: battery.start(period=GUARD_PERIOD))
                            guard.wait_ready(poll=motor.Controller(motors, keys).poll)
                            motors.power_guard = guard
                        else:
                            live("Power guard OFF for this robot: nothing watches current or pack voltage.")
                    if before is not None:
                        before(keys)
                    try:
                        dispatch(command, sequence, motors, keys, session, battery, vision, tof, live)
                    finally:
                        if after is not None:
                            after()
                return f"{command} finished.", "finished"
            except motor.StopRequested:
                return f"{command}: stopped.", "stopped"
            except PowerFault as exc:
                power_tripped = True  # Failed startup checks also leave standby LOW.
                return "Stopped: " + str(exc), "power"
            except (IMUFault, BatteryFault, VisionFault, ToFFault, motor.LaneFault, ValueError) as exc:
                if isinstance(exc, BatteryFault) and command in POWERED_COMMANDS:
                    power_tripped = True
                return "Stopped: " + str(exc), "power" if power_tripped else "fault"
            finally:
                motors.power_guard = None
                # Stop outputs before terminal formatting or another input prompt. A power
                # trip leaves STBY low (coast); every other ending brakes as before.
                try:
                    motors.stop(brake=False if power_tripped else None)
                finally:
                    if guard is not None:
                        summary = guard.summary()
                        if summary:
                            live(summary)
                        if guard.close() is False:
                            guard_blocked = True
                            live("The INA219 read did not finish; restart the program before more powered tests.")
                    live.finish()

        def ic_seconds_left():
            calibration = session.calibration
            return CALIBRATION_LIFETIME - (time.monotonic() - calibration.timestamp)

        def go_preflight(command):
            """Checked at each START press, before the countdown: why this run cannot start."""
            if guard_blocked and command in POWERED_COMMANDS:
                return "a previous INA219 read is still blocked; restart the program first."
            if command in GO_IMU_COMMANDS:
                status = imu_status(session)
                # IV carries over to a new IC while the IMU sits the same way up.
                redo = "run IV" if status == "Needs IV" else "run IC (and IV if IC asks for it)"
                if status != "Ready":
                    return (f"IMU {status}. Press 's', {redo}, then GO again "
                            f"(an IC lasts {CALIBRATION_LIFETIME / 60:g} minutes).")
                left = ic_seconds_left()
                # With the camera on, it starts before the countdown: allow for its startup too.
                camera = vision_support.VISION_START_TIMEOUT if camera_first(command, vision) else 0.0
                if left < COUNTDOWN_SECONDS + IMU_MARGIN_SECONDS + camera:
                    return (f"the IC expires in {max(0.0, left):.0f} s, before this run could start. "
                            f"Press 's', {redo}, then GO again.")
            return None

        def go_describe(command):
            if command == "FC":
                return (f"camera {'ON' if vision.enabled else 'OFF'}, "
                        f"ToF {'ON' if tof.enabled else 'OFF'}")
            parts = ["camera ON"] if vision.enabled and command in LIGHT_COMMANDS else []
            if command in GO_IMU_COMMANDS:
                status = imu_status(session)
                if status != "Ready":
                    parts.insert(0, "IMU " + status)
                else:
                    minutes, seconds = divmod(int(max(0.0, ic_seconds_left())), 60)
                    parts.insert(0, f"IMU Ready, IC {minutes}:{seconds:02d} left")
            return ", ".join(parts)

        motors.initialize()
        while True:
            status = dict(vision_enabled=vision.enabled, tof_enabled=tof.enabled,
                          imu_status=imu_status(session), last_result=last_result,
                          wheel_calibration=calibration_data.summary(config),
                          robot_label=robot_label, robot_id=bot_id,
                          settings_pending=config != initial_config,
                          motor_channels=initial_config["motor_channels"],
                          motor_reversed=initial_config["reversed"],
                          power_guard_enabled=applied["power_guard"],
                          single_duty=initial_config["single_motor_duty"],
                          single_seconds=initial_config["single_motor_seconds"],
                          fc_max_duty=initial_config["fc_max_duty"],
                          imu_drive_duty=initial_config["imu_drive_duty"],
                          imu_turn_max_duty=initial_config["imu_turn_max_duty"],
                          camera_summary=camera_summary, config_path=str(settings_path),
                          go_command=" ".join(part for part in last_drive if part) if last_drive else "")
            parts = menu.choose(status).strip().upper().split(maxsplit=1)
            if not parts:
                continue
            command = parts[0]
            if command == "Q":
                break
            if command == "BOT":
                motors.stop(brake=False)
                result = "switch"
                break
            if command == "SET":
                motors.stop(brake=False)
                try:
                    config = edit_settings(config, settings_path, read=motor.prompt_line)
                    if config != initial_config:
                        print("Settings saved. Closing hardware and applying them to this robot now.")
                        result = "reload"
                        break
                    last_result = "Settings unchanged."
                except (OSError, ValueError) as exc:
                    last_result = f"Settings not saved: {exc}"
                continue
            if command in {"PPROBE", "PCAL"}:
                motors.stop(brake=False)
                try:
                    if command == "PPROBE":
                        pin = probe_pin(config)
                        with motor.Keyboard() as keys:
                            check_probe(motors, keys, pin)
                        last_result = "Probe five-turn check passed; motors remained off."
                    else:
                        if guard_blocked:
                            raise PowerFault("A previous INA219 reader is blocked; restart the program first")
                        updated = calibrate_profile(config, settings_path, motors,
                            powered=lambda action: calibration_segment(args, motors, battery, applied, action),
                            read=motor.prompt_line)
                        if updated is not None:
                            result = "reload"
                            break
                        last_result = "Calibration cancelled; previous settings retained."
                except motor.StopRequested:
                    last_result = "Calibration stopped; previous settings retained."
                except (motor.LaneFault, PowerFault, BatteryFault, ValueError, OSError) as exc:
                    last_result = "Calibration stopped: " + str(exc)
                finally:
                    motors.stop(brake=False)
                print(last_result)
                continue
            if command == "GO":
                motors.stop(brake=False)
                try:
                    armed = resolve_go(parts[1] if len(parts) > 1 else "", last_drive, read=motor.prompt_line)
                except ValueError as exc:
                    last_result = "GO: " + str(exc)
                    continue
                last_drive = armed
                last_result = armed_session(
                    *armed, motors, execute,
                    live_factory=lambda status: LiveStatus(compact=not args.plain_output, status=status),
                    preflight=go_preflight, describe=go_describe,
                    sensors_first=lambda command: camera_first(command, vision, tof))
                print(last_result)
                continue
            if command not in motor.MODES | IMU_COMMANDS | CONFIDENCE_COMMANDS | SINGLE_MOTOR_COMMANDS | GYRO_LANE_COMMANDS | BUTTON_COMMANDS | {"BAT", "B", "RV", "RVD", "VT", "TOF", "TT"}:
                last_result = "Unknown command. H lists all shortcuts."
                continue
            if command in {"VT", "TT"}:
                motors.stop(brake=False)
                owner = vision if command == "VT" else tof
                owner.enabled = not owner.enabled
                if command == "VT":
                    last_result = (f"Camera traffic lights for F, FA, C, FG, CG and FC: "
                                   f"{'ON' if owner.enabled else 'OFF'}. A green crosses the front line; "
                                   "red or yellow stops at it until green.")
                else:
                    last_result = (f"ToF obstacle responses for FC and CG: {'ON' if owner.enabled else 'OFF'}. "
                                   "Other drive modes are unaffected.")
                continue
            sequence = parts[1] if len(parts) > 1 else ""
            if command in {"C", "CG"} and not sequence:
                sequence = motor.prompt_line("Enter course, e.g. FRL (blank cancels): ").strip()
                if not sequence:
                    last_result = "Course cancelled."
                    continue
            if command in GO_COMMANDS:
                last_drive = (command, sequence if command in COURSE_COMMANDS else "")
            print("Press 's' to stop and return to this menu.")
            last_result = execute(command, sequence, LiveStatus(compact=not args.plain_output))[0]
            print(last_result)
    except (KeyboardInterrupt, EOFError):
        print("\nExiting")
    except Exception as exc:
        print(f"Test error: {exc}", file=sys.stderr)
        result = 1
    finally:
        # The old GPIO map remains applied throughout cleanup. A blocked worker
        # or GPIO failure prevents a new profile/hardware owner from starting.
        for name, owner in owners:
            try:
                if owner.close() is False:
                    cleanup_ok = False
                    print(f"{name} did not close; restart the program before changing robots.", file=sys.stderr)
            except Exception as exc:
                cleanup_ok = False
                print(f"{name} cleanup error: {exc}", file=sys.stderr)
        if guard_blocked:
            cleanup_ok = False
    return result if cleanup_ok else 1


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 0 < args.shunt_ohms < float("inf"):
        parser.error("--shunt-ohms must be positive and finite")
    if args.camera_index is not None and args.camera_index < 0:
        parser.error("--camera-index must be nonnegative")
    if args.bus < 0:
        parser.error("--bus must be nonnegative")
    if args.tof_address in (args.address, args.battery_address):
        parser.error("--tof-address must differ from the IMU and battery addresses")
    if args.import_v7 is not None and args.import_settings is not None:
        parser.error("Choose only one import source")
    if (args.settings is not None or args.import_v7 is not None or args.import_settings is not None) and args.bot is None:
        parser.error("Use --bot 500 or --bot 100 with custom settings or imports")
    args.custom_settings_bot = args.bot
    if args.check_ina:
        if args.show_settings or args.import_v7 or args.import_settings:
            parser.error("Use --check-ina separately from settings previews/imports")
        return diagnose_ina(args)
    if args.show_settings or args.import_v7 is not None or args.import_settings is not None:
        try:
            if args.bot is not None:
                _, config = load_configuration(args)
            else:
                config = {}
                for bot_id in ("500", "100"):
                    preview = argparse.Namespace(**vars(args))
                    preview.bot = bot_id
                    config[bot_id + "rpm"] = load_configuration(preview)[1]
            if args.show_settings:
                print(json.dumps(config, indent=2))
            return 0
        except (OSError, ValueError, SyntaxError, KeyError, TypeError) as exc:
            print(f"Settings error: {exc}. Hardware was not opened.", file=sys.stderr)
            return 1
    if not sys.stdin.isatty():
        print("Run in an interactive terminal on the Raspberry Pi.", file=sys.stderr)
        return 1
    try:
        with stop_on_signals():
            bot_id = args.bot
            while True:
                if bot_id is None:
                    bot_id = choose_robot(read=motor.prompt_line)
                    if bot_id is None:
                        return 0
                outcome = run_robot(args, bot_id)
                if outcome == "reload":
                    continue  # Same bot, same custom settings path, fresh hardware owners.
                if outcome != "switch":
                    return outcome
                bot_id = None
    except (KeyboardInterrupt, EOFError):
        print("\nExiting")
        return 0

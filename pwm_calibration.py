"""Removable FC-51 calibration wizard. Prompts and saves only with motors off."""

from copy import deepcopy
from datetime import datetime, timezone
import time

import calibration_data
import motors_optimized as motor
import robot_settings as settings
from ir_tach import CalibrationRunner, Tachometer, TachFault

WHEELS = ("FL", "FR", "BL", "BR")


def probe_pin(config):
    connector = config["tach_connector"].upper()
    if connector == "OFF":
        raise TachFault("Select the removable probe connector in SET -> Wheel calibration first")
    pin = settings.CONNECTORS[connector]
    if pin in config["ir_pins"] or pin in motor.IR_PINS:
        raise TachFault(f"{connector} is assigned to a lane sensor. Choose an independent spare connector")
    return pin


def check_probe(motors, keys, pin, clock=time, write=print):
    """Require exactly five hand rotations before a measurement, motors disabled.

    This checks optical contrast and wiring without assuming dark/light polarity.
    It cannot detect every possible missed edge at motor speed; RPM checks follow.
    """
    motors.stop(brake=False)
    control = motor.Controller(motors, keys, clock, write)
    write("Motors OFF. Start with the mark facing the probe; rotate this wheel exactly FIVE full turns by hand.")
    write("End with the mark facing the probe again, then press g. s cancels. Limit: 60 seconds.")
    with Tachometer(motors.gpio, pin, clock=clock) as probe:
        deadline = clock.monotonic() + 60
        next_log = 0
        while clock.monotonic() < deadline:
            keys_pressed = control.poll()
            now = clock.monotonic()
            probe.check(now, require_motion=False)
            state = probe.snapshot()
            if state.turn_count > 5:
                raise TachFault("Probe counted more than five turns; check the marker, alignment and sensitivity, then retry")
            if "g" in keys_pressed:
                if state.turn_count != 5 or state.edge_count != 10:
                    raise TachFault(f"Probe counted {state.turn_count} turns / {state.edge_count} transitions; "
                                    "expected five complete turns / ten transitions. Finish at the same marker position and retry")
                write("Five-turn probe check passed. Keep the probe fixed and hands clear.")
                return
            if now >= next_log:
                write(f"Probe: {state.turn_count}/5 turns, {state.edge_count} transitions")
                next_log = now + 0.5
            clock.sleep(0.02)
    raise TachFault("Probe check timed out; no calibration was saved")


def calibrate_profile(config, path, motors, powered, read=input, write=print,
                      runner_factory=None, probe_check=None):
    """Return new settings after a full verified save, or None on user cancellation.

    powered(action) supplies stop keys and the configured current/voltage guard;
    it must stop/coast on every outcome and before returning to any prompt.
    Exceptions propagate. The prior profile survives any incomplete run.
    """
    runner_factory = runner_factory or CalibrationRunner
    probe_check = probe_check or check_probe
    config = settings.validate(config)
    motors.stop(brake=False)
    pin = probe_pin(config)
    reference = config["tach_reference_duty"]
    write("REMOVABLE IR WHEEL CALIBRATION - " + config["robot_id"])
    write(f"Probe {config['tach_connector'].upper()} / BCM{pin}; raw reference {reference:g}% duty.")
    write("All wheels must be lifted. One contrasting mark on a flat wheel face; secure the probe away from rollers.")
    write("Each selected wheel will run FORWARD AND REVERSE. Existing corrections are bypassed only during measurements.")
    write("Two passes: measure all wheels, then adjust and verify them against one shared RPM target.")
    write("A missing/noisy probe, stalled wheel, stop key or failed check aborts without changing the saved profile.")
    write("Saved corrections are an unloaded baseline; floor testing is required, especially at lower duties.")
    if read("Type LIFTED to begin, or Enter to cancel: ").strip().upper() != "LIFTED":
        return None
    baseline = [[0.0] * 4 for _ in range(2)]
    factors = [[1.0] * 4 for _ in range(2)]
    verified = [[0.0] * 4 for _ in range(2)]
    target = None
    for phase in ("BASELINE", "ADJUST AND VERIFY"):
        for wheel, label in enumerate(WHEELS):
            motors.stop(brake=False)
            write(f"{phase}: secure the probe at {label}. The other wheels stay at zero PWM.")
            if read(f"Type {label} when positioned, or Enter to cancel: ").strip().upper() != label:
                return None

            def action(keys):
                write("After the five hand turns, clear your hands: pressing g starts this wheel's powered tests.")
                probe_check(motors, keys, pin, write=write)
                # Probe check closes its GPIO event owner before the runner opens it.
                # g starts a segment only after the human has cleared the wheel.
                with runner_factory(motors, keys, pin, log=write) as runner:
                    for direction, forward in enumerate((True, False)):
                        if phase == "BASELINE":
                            baseline[direction][wheel] = runner.measure(wheel, forward, reference)
                        else:
                            factor, rpm = runner.calibrate(wheel, forward, target, reference)
                            factors[direction][wheel] = factor
                            verified[direction][wheel] = rpm

            powered(action)
            motors.stop(brake=False)
            if phase == "BASELINE":
                write(f"{label}: forward {baseline[0][wheel]:.1f} RPM; reverse {baseline[1][wheel]:.1f} RPM")
            else:
                write(f"{label}: forward factor {factors[0][wheel]:.4f} / {verified[0][wheel]:.1f} RPM; "
                      f"reverse factor {factors[1][wheel]:.4f} / {verified[1][wheel]:.1f} RPM")
        if phase == "BASELINE":
            target = min(rpm for row in baseline for rpm in row) * 0.95
            if target < 30:
                raise TachFault("Shared target is below 30 RPM. Check wheel startup and reference duty; nothing saved")
            write(f"Shared forward/reverse target: {target:.1f} RPM (95% of the slowest baseline).")
    record = dict(version=1, robot_id=config["robot_id"], motor_channels=list(config["motor_channels"]),
                  reversed=list(config["reversed"]), pwm_hz=motor.PWM_FREQ, reference_duty=reference,
                  target_rpm=[target, target], baseline_rpm=baseline, verified_rpm=verified, factors=factors,
                  created_utc=datetime.now(timezone.utc).isoformat(timespec="seconds"),
                  probe_connector=config["tach_connector"])
    candidate = settings.validate(dict(config, wheel_calibration=record, wheel_calibration_enabled=True))
    motors.stop(brake=False)
    write("All eight wheel/direction measurements verified within 5% of the shared target.")
    write("Factors in FL, FR, BL, BR order:")
    write("Forward: " + ", ".join(f"{x:.4f}" for x in factors[0]))
    write("Reverse: " + ", ".join(f"{x:.4f}" for x in factors[1]))
    if read("Type SAVE to save and enable, or Enter to keep the previous settings: ").strip().upper() != "SAVE":
        return None
    if settings.load_settings(path) != config:
        raise ValueError("Settings changed on disk during calibration. Nothing saved; reload and retry")
    settings.save_settings(path, candidate)
    write("Saved this bot's wheel corrections. Hardware will reload with motors off.")
    return deepcopy(candidate)

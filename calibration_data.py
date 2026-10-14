"""Strict, hardware-free validation of removable-probe calibration records."""

from copy import deepcopy
import math

from wiring import validate_channels

TOLERANCE = 0.05
KEYS = {"version", "robot_id", "motor_channels", "reversed", "pwm_hz",
        "reference_duty", "target_rpm", "baseline_rpm", "verified_rpm",
        "factors", "created_utc", "probe_connector"}


def number(value, low, high):
    try:
        return type(value) in (int, float) and math.isfinite(value) and low <= value <= high
    except OverflowError:
        return False


def validate_record(value):
    if not isinstance(value, dict):
        raise ValueError("wheel_calibration must be an object")
    if not value:
        return {}
    if set(value) != KEYS or type(value["version"]) is not int or value["version"] != 1:
        raise ValueError("Unsupported or incomplete wheel calibration record")
    if value["robot_id"] not in ("500rpm", "100rpm"):
        raise ValueError("Invalid calibration robot identity")
    validate_channels(value["motor_channels"])
    flags = value["reversed"]
    if not isinstance(flags, (tuple, list)) or len(flags) != 4 or any(type(x) is not bool for x in flags):
        raise ValueError("Invalid calibration motor directions")
    if type(value["pwm_hz"]) is not int or value["pwm_hz"] != 1000:
        raise ValueError("Unsupported calibration PWM frequency")
    if not number(value["reference_duty"], 10, 50):
        raise ValueError("Invalid calibration reference duty")
    if value["probe_connector"] not in ("j7", "j8", "j9", "j10"):
        raise ValueError("Invalid calibration probe connector")
    if not isinstance(value["created_utc"], str) or not 1 <= len(value["created_utc"]) <= 40:
        raise ValueError("Invalid calibration timestamp")
    target = value["target_rpm"]
    if not isinstance(target, (list, tuple)) or len(target) != 2 or any(not number(x, 30, 1200) for x in target):
        raise ValueError("Invalid forward/reverse target RPM")
    if target[0] != target[1]:
        raise ValueError("Forward and reverse must share one target RPM for mixed-direction maneuvers")
    for key, low, high in (("baseline_rpm", 30, 1200), ("verified_rpm", 30, 1200), ("factors", 0.5, 1)):
        rows = value[key]
        if (not isinstance(rows, (list, tuple)) or len(rows) != 2
                or any(not isinstance(row, (list, tuple)) or len(row) != 4 for row in rows)
                or any(not number(x, low, high) for row in rows for x in row)):
            raise ValueError("Invalid calibration " + key)
    for direction in range(2):
        if target[direction] > min(value["baseline_rpm"][direction]):
            raise ValueError("Calibration target exceeds the slowest baseline wheel")
        if any(abs(rpm / target[direction] - 1) > TOLERANCE + 1e-9
               for rpm in value["verified_rpm"][direction]):
            raise ValueError("Calibration verification is outside the 5% tolerance")
    # Keep a consistent JSON-shaped representation, detached from callers.
    record = deepcopy(value)
    for key in ("motor_channels", "reversed", "target_rpm"):
        record[key] = list(record[key])
    for key in ("baseline_rpm", "verified_rpm", "factors"):
        record[key] = [list(row) for row in record[key]]
    return record


def matches(config):
    record = config["wheel_calibration"]
    return bool(record and record["robot_id"] == config["robot_id"]
                and tuple(record["motor_channels"]) == tuple(config["motor_channels"])
                and tuple(record["reversed"]) == tuple(config["reversed"])
                and record["reference_duty"] == config["tach_reference_duty"]
                and record["pwm_hz"] == 1000)


def summary(config):
    if not config["wheel_calibration"]:
        return "not calibrated"
    if not matches(config):
        return "OFF: wiring/reference changed; recalibrate"
    return "ON (unloaded baseline)" if config["wheel_calibration_enabled"] else "OFF (saved result retained)"

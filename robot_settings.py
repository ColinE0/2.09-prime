"""Validated, persistent terminal settings. No hardware is accessed on import.

Edits apply through a hardware restart when the user leaves SET. Importing a v7
folder parses literal constants; it never imports or executes old Python files.
"""

import ast
from dataclasses import dataclass
import importlib
import json
import math
import os
from pathlib import Path
import tempfile
import calibration_data
import vision_settings
from wiring import CHANNEL_PINS, DEFAULT_CHANNELS, validate_channels, channels_from_pins
from motion_profile import (PROFILE_ID, PRESET, PROFILE_IDS, PRESETS, BOT_IDS,
                            settings_bot)


@dataclass(frozen=True)
class Field:
    label: str
    group: str
    kind: str
    minimum: float = None
    maximum: float = None
    help: str = ""
    choices: tuple = ()


FIELDS = {}
DEFAULTS = {}


def _field(key, default, label, group, kind="float", minimum=None, maximum=None,
           help="", choices=()):
    FIELDS[key] = Field(label, group, kind, minimum, maximum, help, choices)
    DEFAULTS[key] = PRESET.get(key, default)


_field("reversed", (False,) * 4, "Motor direction reversed", "Wiring", "bool4",
       help="Enter a wheel code to toggle it, or four on/off values in FL, FR, BL, BR order. Applies when you leave SET; motors remain off during reload.")
_field("ir_pins", (21, 7, 26), "IR connector assignments", "Wiring", "pins3",
       help="LEFT, RIGHT, FRONT. Rev C: J7=21, J8=20, J9=26, J10=7; documented map J7,J10,J9. Confirm with the tape check. Applies when you leave SET; motors remain off during reload.")
_field("button_start", "gpio10", "START button pin", "Wiring", "choice", choices=("gpio10", "gpio9"),
       help="The board switch GO waits for. Rev C: left switch GPIO10 (header pin 19), right switch GPIO9 "
            "(pin 21). Bot 2 uses GPIO9 because its left switch never reads pressed. Check with BTN. "
            "Applies when you leave SET.")
_field("button_reset", "gpio9", "RESET button pin", "Wiring", "choice", choices=("gpio9", "gpio10", "off"),
       help="Stops a GO countdown or run. 'off' leaves 's' on the laptop as the only stop. "
            "Must differ from the START pin.")
_field("motor_channels", DEFAULT_CHANNELS, "Motor channel assignments", "Wiring", "channels4",
       help="FL, FR, BL, BR. Documented rev C map M1A,M1B,M2B,M2A (from behind: J2 front-left, J4 front-right, J12 back-left, J5 back-right); confirm with FL/FR/BL/BR, wheels lifted. Direction flags stay with logical wheels. Applies when you leave SET; motors remain off during reload.")
_field("left_speed", 25.0, "Left motor baseline (%)", "Motor motion", minimum=1, maximum=70,
       help="Original manual/FA/C modes and FG/CG straights; FC has its own cap.")
_field("right_speed", 25.0, "Right motor baseline (%)", "Motor motion", minimum=1, maximum=70)
_field("ramp_rate", 35.0, "Acceleration (duty points/s)", "Motor motion", minimum=1, maximum=500)
_field("turn_ramp_rate", 35.0, "Turn acceleration (duty points/s)", "Motor motion", minimum=1, maximum=500,
       help="R/L, TR/TL, C/CG turns and ITL/ITR only. Controls rising motor power; "
            "deceleration still uses normal Acceleration and stopping keeps the brake setting. "
            "Very low values can stall a turn before the wheels overcome friction.")
_field("turn_approach_speed_pct", 100.0, "Intersection approach speed (% of normal)", "Motor motion", minimum=10, maximum=100,
       help="Forward move after the stop line and before a turn in TR/TL and C/CG. "
            "100 preserves normal driving power. Lowering it shortens travel at the same "
            "approach hold time; retune that hold on the track. Does not change the turn power "
            "or the separate Line approach slowdown.")
_field("auto_slew_rate", 70.0, "Correction slew (duty points/s)", "Motor motion", minimum=1, maximum=500)
_field("brake_on_stop", True, "Short brake on stop", "Motor motion", "bool",
       help="Off selects coast/standby. Changes stopping distance.")
_field("direction_pause", 0.05, "Direction change pause (s)", "Motor motion", minimum=0.05, maximum=1)
_field("auto_secondary_speed", 14.0, "Slowed wheel-pair duty (%)", "IR steering", minimum=0, maximum=70,
       help="Higher is gentler correction. Affects FA, C, FG, CG and FC. FC scales this by its cap.")
_field("auto_debounce", 0.04, "Original IR debounce (s)", "IR steering", minimum=0, maximum=1,
       help="FA, C, FG and CG; FC uses its confidence filter.")
for _key, _default, _label in (
        ("auto_min_correct_left", 0.15, "Left sensor minimum correction (s)"),
        ("auto_min_correct_right", 0.15, "Right sensor minimum correction (s)"),
        ("auto_overshoot_left", 0.10, "Left sensor recovery hold (s)"),
        ("auto_overshoot_right", 0.10, "Right sensor recovery hold (s)")):
    _field(_key, _default, _label, "IR steering", minimum=0, maximum=3,
           help="FA, C, FG and CG. Left sensor triggers a right correction, and vice versa.")
_field("auto_max_correct", 0.80, "Correction timeout (s)", "IR steering", minimum=0.02, maximum=3,
       help="Applies to FC, FA, C, FG and CG.")
_field("auto_settle", 0.30, "Original IR settle interval (s)", "IR steering", minimum=0, maximum=3,
       help="FA, C, FG and CG; FC has no fixed settle interval.")
_field("approach_slow_after", 0.0, "Slow down after driving (s)", "Line approach", minimum=0, maximum=30,
       help="F, FA, C, FG and CG: time after each drive-to-line start (from a stop, a 'g' or a turn) "
            "before dropping to the approach speed so the front IR catches the line. 0 (the default) turns slowing off.")
_field("approach_speed_pct", 60.0, "Approach speed (% of normal)", "Line approach", minimum=10, maximum=100,
       help="Scales the straight and correction duties together. Example: 60 at a 50% baseline drives at 30%. "
            "Check that the slowed wheel pair still turns at this speed.")
_field("auto_yaw_trim", 0.0, "Original auto yaw trim (%)", "IR steering", minimum=-100, maximum=100,
       help="FA, C, FG and CG leans; FC passes zero yaw trim. In FG/CG the gyro hold also acts on any rotation it causes.")
_field("diagonal_yaw_trim", 0.0, "Manual diagonal yaw trim (%)", "Manual maneuvers", minimum=-100, maximum=100)
_field("diagonal_secondary_speed", 11.0, "Manual diagonal secondary duty (%)", "Manual maneuvers", minimum=0, maximum=70)
for _key, _default, _label in (
        ("strafe_duration", 0.6, "Strafe hold (s)"),
        ("turn_duration_right", 0.12, "Right timed-turn hold (s)"),
        ("turn_duration_left", 0.14, "Left timed-turn hold (s)"),
        ("turn_approach_time_right", 0.55, "Right-turn approach hold (s)"),
        ("turn_approach_time_left", 1.5, "Left-turn approach hold (s)"),
        ("turn_exit_straight_time", 2.0, "Turn exit hold (s)"),
        ("course_cross_time", 0.4, "Course crossing hold (s)"),
        ("course_exit_straight_time", 2.0, "Course exit hold (s)"),
        ("diagonal_duration", 0.6, "Diagonal hold (s)")):
    _field(_key, _default, _label, "Manual maneuvers", minimum=0, maximum=10,
           help="Original timed mode: hold AFTER acceleration, not total powered time.")
_field("course_stop_time", 3.0, "Course stopped wait (s)", "Manual maneuvers", minimum=0, maximum=30)
_field("fc_max_duty", 25.0, "FC maximum duty (%)", "Confidence lane following", minimum=1, maximum=50,
       help="Does not change original F/FA/C or IMU test speed. Lower values also lower slowed-pair duty.")
_field("fc_uncertain_duty", 20.0, "FC uncertain-lane duty (%)", "Confidence lane following", minimum=0, maximum=50)
_field("fc_filter_seconds", 0.06, "FC confidence filter time (s)", "Confidence lane following", minimum=0.02, maximum=0.20)
_field("single_motor_duty", 25.0, "Single-wheel test duty (%)", "Single-wheel tests", minimum=1, maximum=50)
_field("single_motor_seconds", 1.0, "Single-wheel total duration (s)", "Single-wheel tests", minimum=0.1, maximum=5,
       help="Includes acceleration. Lift wheels; s stops the test.")
_field("camera_backend", "csi", "Camera connection", "Camera", "choice", choices=("csi", "usb"))
_field("camera_index", 0, "Camera index", "Camera", "int", minimum=0, maximum=32)
_field("camera_rotate_180", True, "Rotate camera image 180 degrees", "Camera", "bool")
_field("camera_auto_exposure", False, "Automatic camera exposure", "Camera", "bool",
       help="CSI only; when off, use the fixed exposure/gain below.")
_field("camera_startup_timeout", 30.0, "Camera startup allowance (s)", "Camera", minimum=5, maximum=60,
       help="Motors remain disabled. Does not change the 0.75-second fresh-frame requirement.")
_field("camera_exposure_us", 4000, "Fixed exposure (microseconds)", "Camera", "int", minimum=100, maximum=50000)
_field("camera_analogue_gain", 1.0, "Fixed analogue gain", "Camera", minimum=1, maximum=16)
_field("camera_light_act_pct", round(vision_settings.LIGHT_ACT_AREA_FRAC * 100, 4),
       "Traffic light action size (% of frame)", "Camera", minimum=0.01, maximum=5,
       help="A light counts once 2 of 3 frames show it at least this big. Smaller acts farther out. "
            "Aim for the light at about 40 cm from the lens and ignore the next block's (100 cm). Check with RV.")
_field("camera_stop_sign_act_pct", round(vision_settings.STOPSIGN_ACT_AREA_FRAC * 100, 4),
       "Stop sign action size (% of frame)", "Camera", minimum=0.01, maximum=10,
       help="A stop sign this big in 2 of 3 frames makes the next front line the end of the run.")

for _key, _label, _minimum, _maximum in (
        ("imu_drive_duty", "IMU forward maximum duty (%)", 1, 50),
        ("imu_turn_max_duty", "IMU turn maximum duty (%)", 1, 50),
        ("imu_turn_min_duty", "IMU turn minimum target duty (%)", 1, 50),
        ("imu_heading_kp", "IMU heading correction (duty/degree)", 0, 10),
        ("imu_heading_kd", "IMU yaw-rate damping (duty/(degree/s))", 0, 5),
        ("imu_max_heading_trim", "IMU maximum heading correction (%)", 0, 50),
        ("imu_turn_slow_gain", "IMU turn slowdown (duty/degree)", 0.01, 5)):
    _field(_key, PRESET[_key], _label, "IMU motion", minimum=_minimum, maximum=_maximum,
           help="Per-robot starting value, used by IF/ITL/ITR and FG/CG; calibrate on the selected robot. Applies when you leave SET; motors remain off during reload.")
_field("imu_turn_stop_lead_s", 0.0, "IMU turn stop lead (s)", "IMU motion", minimum=0, maximum=0.5,
       help="ITL/ITR and CG turns cut power this much yaw-rate time before 90 degrees. Each turn prints a suggested value from its measured coast.")
_field("imu_log_timed_turns", True, "Measure timed turns with the gyro", "IMU motion", "bool",
       help="Logs the measured angle of R, L, TR, TL and course turns when IC and IV are current. Read-only: never steers.")
_field("gyro_lane_ki", PRESET["gyro_lane_ki"], "Heading integral (duty/degree-second)", "Gyro lane (FG/CG)",
       minimum=0, maximum=5,
       help="Removes the steady heading offset a left/right motor mismatch leaves. 0 keeps IF's proportional-derivative hold. "
            "FG/CG share IF's heading gains and correction limit (SET -> IMU motion).")
_field("gyro_lane_ref_step", 0.5, "Heading target nudge per IR correction (deg)", "Gyro lane (FG/CG)",
       minimum=0, maximum=3, help="Each correction aims the robot this much away from that edge. 0 turns nudging off.")
_field("gyro_lane_ref_max", 4.0, "Heading target nudge limit (deg)", "Gyro lane (FG/CG)", minimum=0, maximum=15)
_field("gyro_lane_rezero", True, "Re-zero the gyro when stopped (FG lines, CG stops)", "Gyro lane (FG/CG)", "bool",
       help="Measures the yaw offset while stopped at CG intersections and FG front lines. Skipped with a message if the robot is not still or the stop is under 1.5 s.")
_field("power_guard", True, "Current and voltage stop while driving", "Power protection", "bool",
       help="Stops every powered test on sustained overcurrent or a low pack. Off leaves nothing watching.")
_field("power_overcurrent_a", 4.5, "Overcurrent trip (A)", "Power protection", minimum=0.5, maximum=7.9,
       help="Battery current held at or above this for the hold time stops the motors and drops STBY. 4.5 A is the modeled Q1 limit; set a demo value from measured stall current.")
_field("power_overcurrent_hold_s", 0.2, "Overcurrent hold (s)", "Power protection", minimum=0.05, maximum=2)
_field("power_low_voltage_v", 6.4, "Low-battery trip (V)", "Power protection", minimum=5.0, maximum=8.4,
       help="Pack voltage at the sensor, under load. 6.4 V is 3.2 V per cell on the 2S pack.")
_field("power_low_voltage_hold_s", 2.0, "Low-battery hold (s)", "Power protection", minimum=0.2, maximum=10,
       help="Long enough to ride through brief sag at a start; stalls are the overcurrent trip's job.")
# Migration metadata is shown by --show-settings, not as an editable SET item.
_field("motion_profile", PROFILE_ID, "Motor starting profile", "Motor motion", "choice",
       choices=tuple(PROFILE_IDS.values()))
_field("robot_id", "500rpm", "Robot identity", "Motor motion", "choice", choices=BOT_IDS)

_field("tach_connector", "j8", "Removable FC-51 probe connector", "Wheel calibration", "choice",
       choices=("off", "j7", "j8", "j9", "j10"),
       help="J8 is BCM20. Must be independent of all three lane sensors. Use PPROBE to check one marker per wheel turn.")
_field("tach_reference_duty", 25.0, "Raw calibration reference duty (%)", "Wheel calibration", minimum=10, maximum=50,
       help="PCAL runs lifted wheels up to this duty, forward and reverse. Changing it disables prior corrections. Start with 25 for the 500 RPM bot.")
_field("wheel_calibration_enabled", False, "Apply saved wheel corrections", "Wheel calibration", "bool",
       help="Requires a complete PCAL result for this bot and wiring. OFF retains the saved result. Corrections only reduce duty; validate under load.")
_field("wheel_calibration", {}, "Saved wheel calibration", "Wheel calibration", "calibration")

# ToF distances are edited in centimeters and applied in millimeters.
_field("tof_stop_cm", 8.0, "Stop distance (cm)", "ToF obstacle range", minimum=1, maximum=199,
       help="FC/CG with TT enabled brakes at or below this distance. Must be below resume and slowdown distances.")
_field("tof_resume_cm", 9.0, "Resume distance (cm)", "ToF obstacle range", minimum=1, maximum=200,
       help="After stopping, require readings above this distance for 0.75 seconds. Must exceed stop distance.")
_field("tof_slow_cm", 10.0, "Slowdown distance (cm)", "ToF obstacle range", minimum=1, maximum=200,
       help="FC/CG slows progressively inside this distance, toward 40% speed near the stop distance. Must exceed stop distance.")

CONNECTORS = {"J7": 21, "J8": 20, "J9": 26, "J10": 7}
BUTTON_PINS = {"gpio10": 10, "gpio9": 9}  # rev C board switches: left, right
_CAMERA_KEYS = {
    "CAMERA_BACKEND": "camera_backend", "CAMERA_INDEX": "camera_index",
    "CAMERA_ROTATE_180": "camera_rotate_180", "CAMERA_EXPOSURE_US": "camera_exposure_us",
    "CAMERA_ANALOGUE_GAIN": "camera_analogue_gain",
}
_MOTOR_KEYS = {key.upper(): key for key in FIELDS
               if key not in ("motor_channels", "motion_profile", "robot_id")
               and not key.startswith(("camera_", "fc_", "single_motor_", "imu_", "power_", "gyro_lane_", "tach_", "wheel_",
                                       "button_", "tof_"))}
_IMU_KEYS = {
    "IMU_DRIVE_DUTY": "imu_drive_duty", "TURN_MAX_DUTY": "imu_turn_max_duty",
    "TURN_MIN_DUTY": "imu_turn_min_duty", "HEADING_KP": "imu_heading_kp",
    "HEADING_KD": "imu_heading_kd", "MAX_HEADING_TRIM": "imu_max_heading_trim",
    "TURN_SLOW_GAIN": "imu_turn_slow_gain",
    "TURN_STOP_LEAD": "imu_turn_stop_lead_s", "LOG_TIMED_TURNS": "imu_log_timed_turns",
}
_GYRO_LANE_KEYS = {"KI": "gyro_lane_ki", "REF_STEP": "gyro_lane_ref_step",
                   "REF_MAX": "gyro_lane_ref_max", "REZERO": "gyro_lane_rezero"}
_IMPORT_FILES = {
    "motors_optimized.py": dict(_MOTOR_KEYS, MOTOR_PINS="_motor_pins"),
    "vision_settings.py": _CAMERA_KEYS,
    "vision_support.py": {"VISION_START_TIMEOUT": "camera_startup_timeout"},
    "ir_confidence.py": {"MAX_DUTY": "fc_max_duty", "UNCERTAIN_DUTY": "fc_uncertain_duty",
                         "FILTER_SECONDS": "fc_filter_seconds"},
    "single_motor.py": {"TEST_DUTY": "single_motor_duty", "TEST_SECONDS": "single_motor_seconds"},
}


def _one_value(key, value):
    field = FIELDS[key]
    if field.kind == "bool":
        if type(value) is not bool:
            raise ValueError(f"{key} must be true or false")
    elif field.kind in ("bool4", "pins3"):
        size = 4 if field.kind == "bool4" else 3
        if not isinstance(value, (list, tuple)) or len(value) != size:
            raise ValueError(f"{key} needs exactly {size} entries")
        if field.kind == "bool4":
            if any(type(item) is not bool for item in value):
                raise ValueError("reversed entries must be true or false")
        elif (any(type(item) is not int or item not in CONNECTORS.values() for item in value)
              or len(set(value)) != size):
            raise ValueError("ir_pins must contain three distinct BCM pins from 7, 20, 21, 26")
        value = tuple(value)
    elif field.kind == "channels4":
        value = validate_channels(value)
    elif field.kind == "choice":
        if not isinstance(value, str) or value not in field.choices:
            raise ValueError(f"{key} must be one of: {', '.join(field.choices)}")
    elif field.kind == "calibration":
        value = calibration_data.validate_record(value)
    else:
        try:
            finite = type(value) in (int, float) and math.isfinite(value)
        except OverflowError:
            finite = False
        if not finite:
            raise ValueError(f"{key} must be a finite number")
        if field.kind == "int" and type(value) is not int:
            raise ValueError(f"{key} must be an integer")
        if field.minimum is not None and value < field.minimum:
            raise ValueError(f"{key} must be at least {field.minimum:g}")
        if field.maximum is not None and value > field.maximum:
            raise ValueError(f"{key} must be at most {field.maximum:g}")
        if field.kind == "float":
            value = float(value)
    return value


def validate(config, bot_id=None):
    """Complete a partial config with defaults, rejecting unknown/unsafe values."""
    if not isinstance(config, dict):
        raise ValueError("Settings must be a JSON object")
    unknown = set(config) - set(FIELDS)
    if unknown:
        raise ValueError("Unknown settings: " + ", ".join(sorted(map(str, unknown))))
    selected = settings_bot(config, bot_id)
    defaults = dict(DEFAULTS, **PRESETS[selected], robot_id=selected,
                    motion_profile=PROFILE_IDS[selected])
    result = {key: _one_value(key, config.get(key, default)) for key, default in defaults.items()}
    if "turn_ramp_rate" not in config:
        # Older profiles used the straight ramp for turns. Preserve even custom
        # values when adding the independent control, until the user tunes it.
        result["turn_ramp_rate"] = result["ramp_rate"]
    for key in ("auto_secondary_speed", "diagonal_secondary_speed"):
        if result[key] > min(result["left_speed"], result["right_speed"]):
            raise ValueError(f"{key} cannot exceed either motor baseline; that would reverse the intended slowdown")
    for key in ("auto_min_correct_left", "auto_min_correct_right",
                "auto_overshoot_left", "auto_overshoot_right"):
        if result[key] > result["auto_max_correct"]:
            raise ValueError(f"{key} cannot exceed auto_max_correct")
    if result["tof_stop_cm"] >= min(result["tof_resume_cm"], result["tof_slow_cm"]):
        raise ValueError("ToF stop distance must be below both resume and slowdown distances")
    if result["fc_uncertain_duty"] > result["fc_max_duty"]:
        raise ValueError("fc_uncertain_duty cannot exceed fc_max_duty")
    if result["imu_turn_min_duty"] > result["imu_turn_max_duty"]:
        raise ValueError("imu_turn_min_duty cannot exceed imu_turn_max_duty")
    if result["gyro_lane_ref_step"] > result["gyro_lane_ref_max"]:
        raise ValueError("gyro_lane_ref_step cannot exceed gyro_lane_ref_max")
    if result["button_reset"] == result["button_start"]:
        raise ValueError("button_reset cannot use the START pin; choose the other pin or off")
    if result["imu_max_heading_trim"] > result["imu_drive_duty"]:
        raise ValueError("imu_max_heading_trim cannot exceed imu_drive_duty")
    if result["wheel_calibration_enabled"] and not result["wheel_calibration"]:
        raise ValueError("Run PCAL successfully before enabling wheel corrections")
    if result["wheel_calibration"] and not calibration_data.matches(result):
        result["wheel_calibration_enabled"] = False
    return result


def prepare_profile(config, bot_id=None):
    """Apply the starting preset once to legacy data, preserving other settings.

    No writes occur here. Later SET tuning carries the marker and is not reset.
    Validate supplied values before replacing duties so malformed input still fails.
    """
    if not isinstance(config, dict):
        raise ValueError("Settings must be a JSON object")
    for key, value in config.items():
        if key not in FIELDS:
            raise ValueError("Unknown setting: " + str(key))
        _one_value(key, value)
    selected = settings_bot(config, bot_id)
    if "motion_profile" not in config and selected == "500rpm":
        config = dict(config, **PRESETS[selected], motion_profile=PROFILE_IDS[selected])
    return validate(config, selected)


def _bool(text):
    text = text.strip().lower()
    if text in ("true", "on", "yes", "1"):
        return True
    if text in ("false", "off", "no", "0"):
        return False
    raise ValueError("Enter on/off or true/false")


def parse_value(key, text):
    """Parse one editor value; validate the resulting full config before saving."""
    if key not in FIELDS:
        raise ValueError(f"Unknown setting: {key}")
    kind = FIELDS[key].kind
    text = text.strip()
    try:
        if kind == "bool":
            value = _bool(text)
        elif kind == "bool4":
            value = tuple(_bool(item) for item in text.strip("()[]").split(","))
        elif kind == "pins3":
            items = [item.strip().upper() for item in text.strip("()[]").split(",")]
            value = tuple(CONNECTORS[item] if item in CONNECTORS else int(item) for item in items)
        elif kind == "channels4":
            value = tuple(item.strip().upper() for item in text.split(","))
        elif kind == "choice":
            value = text.lower()
        elif kind == "calibration":
            value = json.loads(text, object_pairs_hook=_unique_object)
        elif kind == "int":
            value = int(text)
        else:
            value = float(text)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid {key}: {exc}") from exc
    return _one_value(key, value)


def format_value(key, value):
    value = _one_value(key, value)
    kind = FIELDS[key].kind
    if kind == "calibration":
        return json.dumps(value, sort_keys=True, allow_nan=False)
    if kind == "bool":
        return "ON" if value else "OFF"
    if kind == "bool4":
        return ", ".join("ON" if item else "OFF" for item in value)
    if kind == "channels4":
        return ", ".join(value)
    if kind == "pins3":
        names = {pin: connector for connector, pin in CONNECTORS.items()}
        return ", ".join(names[pin] for pin in value)
    if kind == "float":
        return f"{value:g}"
    return str(value)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate setting: {key}")
        result[key] = value
    return result


def load_settings(path, bot_id=None):
    path = Path(path)
    if not path.exists():
        return validate({}, bot_id)
    try:
        data = json.loads(path.read_text(encoding="utf-8-sig"), object_pairs_hook=_unique_object)
        return prepare_profile(legacy_channels(data, path), bot_id)
    except (UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise ValueError(f"Invalid settings file {path.name}: {exc}") from exc


def save_settings(path, config, bot_id=None, overwrite=True):
    """Validate before writing; atomically replace only after a complete write."""
    data = validate(config, bot_id)
    path = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="\n",
                                         dir=path.parent, prefix="." + path.name + ".",
                                         suffix=".tmp", delete=False) as handle:
            temporary = Path(handle.name)
            json.dump(data, handle, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        if overwrite:
            os.replace(temporary, path)
        else:
            # Linking the complete temporary file is atomic and fails if a
            # concurrent launch has already created the destination.
            os.link(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def _bound_names(target):
    if isinstance(target, ast.Name):
        return {target.id}
    if isinstance(target, (ast.Tuple, ast.List)):
        return set().union(*(_bound_names(item) for item in target.elts))
    if isinstance(target, (ast.Subscript, ast.Attribute)):
        return _bound_names(target.value)
    return set()


def _nested_setting_assignment(node, names):
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda)):
        return False
    targets = (node.targets if isinstance(node, ast.Assign) else
               [node.target] if isinstance(node, (ast.AnnAssign, ast.AugAssign)) else [])
    if any(_bound_names(target) & names for target in targets):
        return True
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
        if _bound_names(node.func.value) & names:
            return True  # A top-level method could mutate a literal list in place.
    return any(_nested_setting_assignment(child, names) for child in ast.iter_child_nodes(node))


def _read_constants(path, mapping):
    try:
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    except (SyntaxError, UnicodeError) as exc:
        raise ValueError(f"Cannot parse {path.name}: {exc}") from exc
    found = {}
    for statement in tree.body:
        if isinstance(statement, (ast.Assign, ast.AnnAssign)):
            targets = statement.targets if isinstance(statement, ast.Assign) else [statement.target]
            bound = set().union(*(_bound_names(target) for target in targets)) & set(mapping)
            if not bound:
                continue
            if any(not isinstance(target, ast.Name) for target in targets):
                raise ValueError(f"{path.name}: settings must use simple literal assignments")
            try:
                value = ast.literal_eval(statement.value)
            except (ValueError, TypeError, SyntaxError) as exc:
                raise ValueError(f"{path.name}: {', '.join(sorted(bound))} is not a literal; import stopped") from exc
            for name in bound:
                found[mapping[name]] = value
        elif _nested_setting_assignment(statement, set(mapping)):
            raise ValueError(f"{path.name}: conditional or augmented setting assignment cannot be imported")
    return found


def legacy_channels(config, settings_path):
    """Recover v8's Python-only motor map when the adjacent source is present.

    JSON channels take precedence. The Python file is parsed, never executed;
    ambiguous or dynamic maps stop migration rather than use invented wiring.
    """
    if not isinstance(config, dict):
        raise ValueError("Settings must be a JSON object")
    source = Path(settings_path).parent / "motors_optimized.py"
    if "motor_channels" in config or not source.is_file():
        return config
    constants = _read_constants(source, {"MOTOR_PINS": "pins"})
    if "pins" not in constants:
        raise ValueError(f"{source} has no literal MOTOR_PINS; record its motor_channels before import")
    return dict(config, motor_channels=channels_from_pins(constants["pins"]))


def import_v7(folder, bot_id=None):
    """Return validated migrated settings and a report; do not save or run code."""
    folder = Path(folder)
    if not (folder / "motors_optimized.py").is_file():
        raise ValueError("Select the v7 folder containing motors_optimized.py")
    migrated = {}
    report = []
    for filename, mapping in _IMPORT_FILES.items():
        path = folder / filename
        if not path.is_file():
            report.append(f"{filename} missing: retained new-package defaults for its settings.")
            continue
        values = _read_constants(path, mapping)
        if "_motor_pins" in values:
            values["motor_channels"] = channels_from_pins(values.pop("_motor_pins"))
        migrated.update(values)
        report.append(f"{filename}: imported {len(values)} literal settings.")
        expected = {"motor_channels" if key == "_motor_pins" else key for key in mapping.values()}
        missing = expected - set(values)
        if missing:
            report.append("Retained defaults for: " + ", ".join(sorted(missing)))
    result = prepare_profile(migrated, bot_id)
    if result["robot_id"] == "500rpm":
        report.append("Applied 500 RPM starting profile: 25% operating maxima; legacy duty/trim/gain values replaced. Timings retained for calibration.")
    else:
        report.append("Imported 100 RPM robot settings: valid legacy duties retained; missing fields use the 104 RPM motor starting settings.")
    report.append("Directions (FL, FR, BL, BR): " + format_value("reversed", result["reversed"]))
    report.append("Motor channels (FL, FR, BL, BR): " + format_value("motor_channels", result["motor_channels"]))
    report.append("IR roles (LEFT, RIGHT, FRONT), Rev C connectors: " + format_value("ir_pins", result["ir_pins"]))
    report.append("Saved-file constants only: old CLI flags, runtime toggles and unsaved changes cannot be inferred. Camera auto-exposure defaults OFF; set it here if previously supplied on the command line.")
    report.append("Only literal Rev C MOTOR_PINS permutations are migrated. Output patterns, IMU/ToF thresholds, detector/image geometry and other unlisted code changes are not migrated. Original files were not executed or changed.")
    return result, report


def apply_settings(config, bot_id=None):
    """Apply at startup BEFORE creating any GPIO driver or sensor sessions.

    CameraOptions must be constructed explicitly from the validated config by
    the caller: dataclass/default-argument snapshots are not rewritten here.
    """
    data = validate(config, bot_id)
    motor = importlib.import_module("motors_optimized")
    vision = importlib.import_module("vision_support")
    confidence = importlib.import_module("ir_confidence")
    single = importlib.import_module("single_motor")
    imu = importlib.import_module("imu_tests")
    lane = importlib.import_module("lane_gyro")
    tof = importlib.import_module("tof_support")
    tof.HOLD_MM = data["tof_stop_cm"] * 10
    tof.RESUME_MM = data["tof_resume_cm"] * 10
    tof.SLOW_MM = data["tof_slow_cm"] * 10
    for constant, key in _MOTOR_KEYS.items():
        setattr(motor, constant, data[key])
    motor.MOTOR_PINS = tuple(CHANNEL_PINS[name] for name in data["motor_channels"])
    factors = data["wheel_calibration"]["factors"] if data["wheel_calibration_enabled"] else ((1.0,) * 4,) * 2
    motor.WHEEL_FACTORS_FORWARD = tuple(factors[0])
    motor.WHEEL_FACTORS_REVERSE = tuple(factors[1])
    for constant, key in _CAMERA_KEYS.items():
        setattr(vision.settings, constant, data[key])
    vision.VISION_START_TIMEOUT = data["camera_startup_timeout"]
    # Read in the main process by vision_support.TrafficLight, so no camera restart is needed.
    vision.settings.LIGHT_ACT_AREA_FRAC = data["camera_light_act_pct"] / 100.0
    vision.settings.STOPSIGN_ACT_AREA_FRAC = data["camera_stop_sign_act_pct"] / 100.0
    motor.START_BTN = BUTTON_PINS[data["button_start"]]
    motor.RESET_BTN = BUTTON_PINS.get(data["button_reset"])  # None: no RESET switch
    confidence.CORRECTION_TIMEOUT = data["auto_max_correct"]
    confidence.MAX_DUTY = data["fc_max_duty"]
    confidence.UNCERTAIN_DUTY = data["fc_uncertain_duty"]
    confidence.FILTER_SECONDS = data["fc_filter_seconds"]
    single.TEST_DUTY = data["single_motor_duty"]
    single.TEST_SECONDS = data["single_motor_seconds"]
    for constant, key in _IMU_KEYS.items():
        setattr(imu, constant, data[key])
    for constant, key in _GYRO_LANE_KEYS.items():
        setattr(lane, constant, data[key])
    return data

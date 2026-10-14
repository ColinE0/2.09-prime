"""Separate saved settings for the two robots, with hardware-free previews.

Tagged per-bot profiles from prior releases migrate independently; older single-profile
automatic migration requires an explicitly identified 500 RPM predecessor.
Imports always name a destination robot, never modify their source, and never
replace an existing destination. Nothing in this module opens robot hardware.
"""

import json
import os
from pathlib import Path
import tempfile

from motion_profile import BOT_IDS, BOT_LABELS, PROFILE_IDS, normalize_bot_id
import robot_settings


_PREVIOUS_500_FOLDERS = (
    "MPU6050_Motor_Test_v8_500rpm_wire_colors",
    "MPU6050_Motor_Test_v8_500rpm",
    "imu-test-v8-500rpm",
)
# Kaleb's bot 2 Pi keeps his v9.6 to v9.8 releases in motor9_x folders (and his zip unpacks as
# MPU6050_Motor_Test_v9_7). They come first: his latest 500 RPM tuning beats an older v9.5 copy.
_KALEB_FOLDERS = ("motor9_8", "motor9_7", "motor9_6", "MPU6050_Motor_Test_v9_8", "MPU6050_Motor_Test_v9_7")
# v9.9 and the 2026-10-06 test copies built on it, newest first: any of them that has run on a
# robot holds tuning at least as new as Kaleb's, since v9.9 itself started from his.
_V9_9_FOLDERS = ("MPU6050_Motor_Test_v9_9_turn_controls", "MPU6050_Motor_Test_v9_9_stop_sign_turn_settle_update",
                 "MPU6050_Motor_Test_v9_9_stop_sign_turn_update", "MPU6050_Motor_Test_v9_9_stop_sign_update",
                 "MPU6050_Motor_Test_v9_9_hardened", "MPU6050_Motor_Test_v9_9")
_PREVIOUS_PROFILE_FOLDERS = _V9_9_FOLDERS + _KALEB_FOLDERS + ("MPU6050_Motor_Test_v9_5", "MPU6050_Motor_Test_v9_4", "MPU6050_Motor_Test_v9_3", "MPU6050_Motor_Test_v9_2", "MPU6050_Motor_Test_v9_1", "MPU6050_Motor_Test_v9")


def _read_json(path):
    try:
        raw = json.loads(path.read_text(encoding="utf-8-sig"),
                         object_pairs_hook=robot_settings._unique_object)
        if not isinstance(raw, dict):
            raise ValueError("Settings must be a JSON object")
        return raw
    except (UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise ValueError(f"Invalid settings file {path}: {exc}") from exc


def _backup(path):
    """Atomically retain original bytes under an unused backup name."""
    original = path.read_bytes()
    counter = 0
    while True:
        suffix = "" if counter == 0 else f"-{counter}"
        backup = path.with_name(path.stem + f".pre-profiles{suffix}.json")
        if backup.exists():
            if backup.read_bytes() == original:
                return backup
            counter += 1
            continue
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".backup-",
                                             suffix=".tmp", delete=False) as handle:
                temporary = Path(handle.name)
                handle.write(original)
                handle.flush()
                os.fsync(handle.fileno())
            try:
                os.link(temporary, backup)
                return backup
            except FileExistsError:
                counter += 1
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()


def _migration_report(raw, bot_id):
    if "motion_profile" not in raw and bot_id == "500rpm":
        return ["Applied the 500 RPM starting duties once (25% operating maxima); "
                "preserved wiring, camera choices and maneuver timings."]
    if "robot_id" not in raw or "motion_profile" not in raw:
        return [f"Identified settings as {BOT_LABELS[bot_id]}; retained saved tuning."]
    return []


def _prepare_json(source, raw, bot_id, report):
    with_channels = robot_settings.legacy_channels(raw, source)
    config = robot_settings.prepare_profile(with_channels, bot_id)
    if "motor_channels" not in raw and "motor_channels" in with_channels:
        report.append(f"Preserved the literal MOTOR_PINS map from {source.parent / 'motors_optimized.py'}.")
    return config


def _automatic_source(package_dir, report, selected="500rpm"):
    for name in _PREVIOUS_PROFILE_FOLDERS:
        source = package_dir.parent / name / "profiles" / f"{selected}.json"
        if source.is_file():
            raw = _read_json(source)
            if "robot_id" not in raw and "motion_profile" not in raw:
                report.append(f"Skipped untagged profile {source}; import it explicitly for the correct robot.")
                continue
            return source, raw, _prepare_json(source, raw, selected, report)
    if selected != "500rpm":
        return None
    candidates = [package_dir / "robot_settings.json"]
    candidates.extend(package_dir.parent / name / "robot_settings.json"
                      for name in _PREVIOUS_500_FOLDERS)
    for source in candidates:
        if not source.is_file():
            continue
        raw = _read_json(source)
        if (raw.get("robot_id") == "500rpm" or
                raw.get("motion_profile") == PROFILE_IDS["500rpm"]):
            # Reject inconsistent identity tags even in a named 500 RPM folder.
            return source, raw, _prepare_json(source, raw, "500rpm", report)
        report.append(f"Skipped automatic import of {source}: no 500 RPM identity. "
                      "Use an explicit import after selecting the correct robot.")
    return None


def load_profile(package_dir, bot_id, settings_path=None, import_v7=None,
                 import_settings=None, persist=True):
    """Return (destination Path, validated config, human-readable report).

    ``bot_id`` is required, including for an untagged legacy import. Defaults
    reside in ``profiles/<bot_id>.json``. A custom settings_path is dedicated to
    the chosen bot: selecting another bot with that file is an error.
    ``persist=False`` validates and previews without creating files or folders.
    """
    selected = normalize_bot_id(bot_id)
    package_dir = Path(package_dir).expanduser().resolve()
    destination = (Path(settings_path).expanduser().resolve() if settings_path is not None
                   else package_dir / "profiles" / f"{selected}.json")
    if import_v7 is not None and import_settings is not None:
        raise ValueError("Choose either import_v7 or import_settings, not both")
    report = []

    if import_v7 is not None or import_settings is not None:
        if destination.exists():
            raise ValueError(f"Settings already exist: {destination}. "
                             "Import into a different --settings path.")
        if import_v7 is not None:
            source = Path(import_v7).expanduser().resolve()
            config, import_report = robot_settings.import_v7(source, selected)
            report.extend(import_report)
        else:
            source = Path(import_settings).expanduser().resolve()
            raw = _read_json(source)
            config = _prepare_json(source, raw, selected, report)
            report.extend(_migration_report(raw, selected))
        if persist:
            destination.parent.mkdir(parents=True, exist_ok=True)
            robot_settings.save_settings(destination, config, selected, overwrite=False)
        report.append(f"{'Imported' if persist else 'Would import'} {source} into {destination} "
                      f"for {BOT_LABELS[selected]}; source unchanged.")
        return destination, config, report

    if destination.exists():
        raw = _read_json(destination)
        config = _prepare_json(destination, raw, selected, report)
        needs_tags = "robot_id" not in raw or "motion_profile" not in raw
        if needs_tags:
            if persist:
                backup = _backup(destination)
                robot_settings.save_settings(destination, config, selected)
                report.append(f"Original settings backed up at {backup}.")
            report.extend(_migration_report(raw, selected))
        return destination, config, report

    previous = (_automatic_source(package_dir, report, selected)
                if settings_path is None else None)
    if previous is not None:
        source, raw, config = previous
        report.extend(_migration_report(raw, selected))
        report.append(f"{'Copied' if persist else 'Would copy'} the saved {BOT_LABELS[selected]} settings "
                      f"from {source}; source unchanged.")
    else:
        config = robot_settings.validate({}, selected)
        report.append(f"Using independent starting settings for {BOT_LABELS[selected]}. "
                      "Motor-channel, direction and IR defaults still require a physical check.")
    if persist:
        destination.parent.mkdir(parents=True, exist_ok=True)
        robot_settings.save_settings(destination, config, selected, overwrite=False)
    return destination, config, report

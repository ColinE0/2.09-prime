"""Profile identity, persistence and migration tests without robot hardware."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from motion_profile import PROFILE_IDS, PRESETS
import robot_profiles as profiles
import robot_settings as settings


class RobotProfilesTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.package = self.root / "MPU6050_Motor_Test_v9"
        self.package.mkdir()

    def write_json(self, path, config):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(config), encoding="utf-8")
        return path

    def test_each_bot_has_complete_independent_starting_settings(self):
        slow_path, slow, _ = profiles.load_profile(self.package, "100")
        fast_path, fast, _ = profiles.load_profile(self.package, "500")
        self.assertEqual(slow_path, self.package / "profiles" / "100rpm.json")
        self.assertEqual(fast_path, self.package / "profiles" / "500rpm.json")
        for bot, config in (("100rpm", slow), ("500rpm", fast)):
            with self.subTest(bot=bot):
                self.assertEqual(config["robot_id"], bot)
                self.assertEqual(config["motion_profile"], PROFILE_IDS[bot])
                self.assertEqual(set(config), set(settings.FIELDS))
                for key, expected in PRESETS[bot].items():
                    self.assertEqual(config[key], expected, key)
        self.assertEqual(slow["left_speed"], 70)
        self.assertEqual(slow["right_speed"], 69)
        self.assertEqual(fast["left_speed"], 25)
        self.assertEqual(slow["motor_channels"], fast["motor_channels"])
        self.assertEqual(slow["ir_pins"], fast["ir_pins"])

    def test_roundtrip_switching_keeps_each_bots_tuning_wiring_and_camera(self):
        fast_path, fast, _ = profiles.load_profile(self.package, "500")
        fast.update(left_speed=31, right_speed=30, imu_drive_duty=27,
                    camera_backend="usb", camera_index=2,
                    motor_channels=("M1A", "M1B", "M2B", "M2A"),
                    ir_pins=(21, 7, 26), reversed=(False, False, True, False))
        settings.save_settings(fast_path, fast)
        fast_bytes = fast_path.read_bytes()
        slow_path, slow, _ = profiles.load_profile(self.package, "100")
        slow.update(left_speed=58, right_speed=57, imu_drive_duty=43,
                    turn_duration_left=0.21)
        settings.save_settings(slow_path, slow)
        slow_bytes = slow_path.read_bytes()
        for _ in range(3):
            self.assertEqual(profiles.load_profile(self.package, "500")[1], fast)
            self.assertEqual(profiles.load_profile(self.package, "100")[1], slow)
        self.assertEqual(fast_path.read_bytes(), fast_bytes)
        self.assertEqual(slow_path.read_bytes(), slow_bytes)

    def test_partial_tagged_100_profile_uses_100_defaults_without_scaling(self):
        path = self.package / "profiles" / "100rpm.json"
        self.write_json(path, {"robot_id": "100rpm", "motion_profile": PROFILE_IDS["100rpm"],
                               "camera_index": 4, "left_speed": 63})
        config = profiles.load_profile(self.package, "100")[1]
        self.assertEqual(config["left_speed"], 63)
        self.assertEqual(config["right_speed"], 69)
        self.assertEqual(config["imu_drive_duty"], 50)
        self.assertEqual(config["auto_secondary_speed"], 40)
        self.assertEqual(config["camera_index"], 4)

    def test_mismatched_identity_is_rejected_without_writes(self):
        variants = [
            {"robot_id": "100rpm"},
            {"motion_profile": PROFILE_IDS["100rpm"]},
            {"robot_id": "500rpm", "motion_profile": PROFILE_IDS["100rpm"]},
            {"robot_id": "100rpm", "motion_profile": PROFILE_IDS["500rpm"]},
        ]
        path = self.package / "profiles" / "500rpm.json"
        for raw in variants:
            with self.subTest(raw=raw):
                self.write_json(path, raw)
                before = path.read_bytes()
                with self.assertRaises(ValueError):
                    profiles.load_profile(self.package, "500")
                self.assertEqual(path.read_bytes(), before)
                self.assertEqual(list(path.parent.iterdir()), [path])

    def test_custom_path_cannot_be_used_for_other_robot(self):
        path = self.root / "custom.json"
        profiles.load_profile(self.package, "500", settings_path=path)
        original = path.read_bytes()
        with self.assertRaises(ValueError):
            profiles.load_profile(self.package, "100", settings_path=path)
        self.assertEqual(path.read_bytes(), original)

    def test_unknown_bot_or_malformed_identity_rejected(self):
        for bot in (None, "", "104rpm", 500, "other"):
            with self.subTest(bot=bot), self.assertRaises(ValueError):
                profiles.load_profile(self.package, bot)
        for raw in ({"robot_id": []}, {"motion_profile": []}, {"motion_profile": "future"}):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                settings.validate(raw)

    def test_preview_creates_neither_files_nor_directories_and_is_quiet(self):
        nonexistent_package = self.root / "not-created"
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            _, config, report = profiles.load_profile(nonexistent_package, "100", persist=False)
        self.assertFalse(nonexistent_package.exists())
        self.assertEqual(config["left_speed"], 70)
        self.assertTrue(report)
        self.assertEqual(output.getvalue(), "")

    def test_custom_legacy_preview_does_not_tag_or_backup(self):
        path = self.write_json(self.root / "custom.json", {"left_speed": 60, "camera_index": 3})
        original = path.read_bytes()
        config = profiles.load_profile(self.package, "100", settings_path=path, persist=False)[1]
        self.assertEqual(config["left_speed"], 60)
        self.assertEqual(path.read_bytes(), original)
        self.assertEqual(list(self.root.glob("*.json")), [path])

    def test_legacy_custom_500_migrates_once_with_exact_backup(self):
        raw = {"left_speed": 70, "right_speed": 69, "auto_secondary_speed": 40,
               "camera_index": 5, "turn_duration_right": 0.19,
               "motor_channels": ["M1A", "M1B", "M2B", "M2A"],
               "ir_pins": [21, 7, 26], "reversed": [True, False, True, False]}
        path = self.write_json(self.root / "custom.json", raw)
        original = path.read_bytes()
        _, config, _ = profiles.load_profile(self.package, "500", settings_path=path)
        self.assertEqual(config["left_speed"], 25)
        for key in ("camera_index", "turn_duration_right"):
            self.assertEqual(config[key], raw[key])
        for key in ("motor_channels", "ir_pins", "reversed"):
            self.assertEqual(config[key], tuple(raw[key]))
        self.assertEqual((self.root / "custom.pre-profiles.json").read_bytes(), original)
        config["left_speed"] = 32
        settings.save_settings(path, config)
        self.assertEqual(profiles.load_profile(self.package, "500", settings_path=path)[1]["left_speed"], 32)
        self.assertEqual(len(list(self.root.glob("*.pre-profiles*.json"))), 1)

    def test_tagging_prior_500_profile_preserves_tuned_duties(self):
        path = self.write_json(self.root / "custom.json",
                               {"motion_profile": PROFILE_IDS["500rpm"], "left_speed": 33})
        original = path.read_bytes()
        config = profiles.load_profile(self.package, "500", settings_path=path)[1]
        self.assertEqual(config["left_speed"], 33)
        self.assertEqual(config["robot_id"], "500rpm")
        self.assertEqual((self.root / "custom.pre-profiles.json").read_bytes(), original)

    def test_tagging_failure_leaves_original_and_backup_intact(self):
        path = self.write_json(self.root / "custom.json", {"left_speed": 60})
        original = path.read_bytes()
        with patch.object(settings.os, "replace", side_effect=OSError("write failure")):
            with self.assertRaises(OSError):
                profiles.load_profile(self.package, "100", settings_path=path)
        self.assertEqual(path.read_bytes(), original)
        self.assertEqual((self.root / "custom.pre-profiles.json").read_bytes(), original)
        self.assertFalse(list(self.root.glob("*.tmp")))

    def test_previous_backup_is_not_overwritten(self):
        path = self.write_json(self.root / "custom.json", {"left_speed": 61})
        backup = self.root / "custom.pre-profiles.json"
        backup.write_bytes(b"earlier backup")
        original = path.read_bytes()
        profiles.load_profile(self.package, "100", settings_path=path)
        self.assertEqual(backup.read_bytes(), b"earlier backup")
        self.assertEqual((self.root / "custom.pre-profiles-1.json").read_bytes(), original)

    def test_auto_import_is_500_only_and_preserves_source(self):
        source = self.write_json(self.root / "MPU6050_Motor_Test_v8_500rpm" / "robot_settings.json",
                                 {"motion_profile": PROFILE_IDS["500rpm"], "left_speed": 34,
                                  "camera_index": 3, "ir_pins": [21, 7, 26]})
        original = source.read_bytes()
        fast = profiles.load_profile(self.package, "500")[1]
        slow = profiles.load_profile(self.package, "100")[1]
        self.assertEqual(fast["left_speed"], 34)
        self.assertEqual(fast["camera_index"], 3)
        self.assertEqual(fast["ir_pins"], (21, 7, 26))
        self.assertEqual(slow["left_speed"], 70)
        self.assertEqual(slow["camera_index"], 0)
        self.assertEqual(source.read_bytes(), original)

    def test_package_marked_500_json_imports_but_untagged_does_not(self):
        source = self.write_json(self.package / "robot_settings.json",
                                 {"camera_index": 6, "left_speed": 60})
        config = profiles.load_profile(self.package, "500", persist=False)[1]
        self.assertEqual(config["camera_index"], 0)
        self.write_json(source, {"motion_profile": PROFILE_IDS["500rpm"], "camera_index": 6})
        config = profiles.load_profile(self.package, "500", persist=False)[1]
        self.assertEqual(config["camera_index"], 6)
        self.assertFalse((self.package / "profiles").exists())

    def test_unmarked_sibling_and_old_v8_never_auto_import(self):
        for name in ("MPU6050_Motor_Test_v8_500rpm", "MPU6050_Motor_Test_v8_reviewed",
                     "MPU6050_Motor_Test_v8", "MPU6050_Motor_Test_v7"):
            self.write_json(self.root / name / "robot_settings.json", {"camera_index": 9})
        config = profiles.load_profile(self.package, "500", persist=False)[1]
        self.assertEqual(config["camera_index"], 0)
        self.assertFalse((self.package / "profiles").exists())

    def test_explicit_json_import_preserves_100_tuning_and_never_source(self):
        raw = {"left_speed": 59, "right_speed": 58, "fc_max_duty": 44,
               "imu_turn_max_duty": 45, "imu_turn_min_duty": 31,
               "camera_index": 5, "turn_duration_left": 0.27,
               "motor_channels": ["M1A", "M1B", "M2B", "M2A"]}
        source = self.write_json(self.root / "legacy.json", raw)
        original = source.read_bytes()
        path, config, _ = profiles.load_profile(self.package, "100", import_settings=source)
        for key, value in raw.items():
            self.assertEqual(config[key], tuple(value) if isinstance(value, list) else value)
        self.assertEqual(source.read_bytes(), original)
        self.assertEqual(json.loads(path.read_text())["robot_id"], "100rpm")

    def test_explicit_500_json_import_migrates_untagged_duties_once(self):
        source = self.write_json(self.root / "legacy.json", {"left_speed": 70, "camera_index": 5})
        original = source.read_bytes()
        path, config, _ = profiles.load_profile(self.package, "500", import_settings=source)
        self.assertEqual(config["left_speed"], 25)
        self.assertEqual(config["camera_index"], 5)
        config["left_speed"] = 29
        settings.save_settings(path, config)
        self.assertEqual(profiles.load_profile(self.package, "500")[1]["left_speed"], 29)
        self.assertEqual(source.read_bytes(), original)

    def test_json_import_recovers_adjacent_legacy_motor_map_without_executing(self):
        source = self.write_json(self.root / "legacy" / "robot_settings.json", {"left_speed": 60})
        motor_source = source.parent / "motors_optimized.py"
        motor_source.write_text(
            "MOTOR_PINS = ((17,27,18),(22,23,25),(16,19,13),(5,6,12))\n"
            "raise RuntimeError('must not execute')\n", encoding="utf-8")
        original = motor_source.read_bytes()
        config = profiles.load_profile(self.package, "100", import_settings=source)[1]
        self.assertEqual(config["motor_channels"], ("M1A", "M1B", "M2B", "M2A"))
        self.assertEqual(settings.load_settings(source, "100")["motor_channels"], config["motor_channels"])
        self.assertEqual(motor_source.read_bytes(), original)

    def test_dynamic_or_missing_adjacent_motor_map_rejects_json_import(self):
        source = self.write_json(self.root / "legacy" / "robot_settings.json", {"left_speed": 60})
        motor_source = source.parent / "motors_optimized.py"
        for text in ("MOTOR_PINS = calculate_pins()\n", "LEFT_SPEED = 60\n"):
            with self.subTest(text=text):
                motor_source.write_text(text, encoding="utf-8")
                with self.assertRaises(ValueError):
                    profiles.load_profile(self.package, "100", import_settings=source)
        self.assertFalse((self.package / "profiles").exists())

    def test_explicit_json_channel_map_takes_precedence_over_adjacent_code(self):
        source = self.write_json(self.root / "legacy" / "robot_settings.json",
                                 {"motor_channels": ["M1B", "M1A", "M2A", "M2B"]})
        (source.parent / "motors_optimized.py").write_text("MOTOR_PINS = calculate_pins()\n",
                                                         encoding="utf-8")
        config = profiles.load_profile(self.package, "100", import_settings=source)[1]
        self.assertEqual(config["motor_channels"], ("M1B", "M1A", "M2A", "M2B"))

    def test_explicit_import_cannot_copy_other_bots_tagged_settings(self):
        source = self.write_json(self.root / "slow.json", settings.validate({}, "100"))
        with self.assertRaises(ValueError):
            profiles.load_profile(self.package, "500", import_settings=source)
        self.assertFalse((self.package / "profiles").exists())

    def test_explicit_import_preview_has_no_filesystem_side_effects(self):
        source = self.write_json(self.root / "legacy.json", {"left_speed": 60})
        original = source.read_bytes()
        profiles.load_profile(self.package, "100", import_settings=source, persist=False)
        self.assertEqual(source.read_bytes(), original)
        self.assertFalse((self.package / "profiles").exists())

    def test_import_refuses_existing_destination_and_ambiguous_sources(self):
        source = self.write_json(self.root / "legacy.json", {"left_speed": 60})
        path, _, _ = profiles.load_profile(self.package, "100")
        original = path.read_bytes()
        for preview in (True, False):
            with self.subTest(persist=preview), self.assertRaises(ValueError):
                profiles.load_profile(self.package, "100", import_settings=source, persist=preview)
        self.assertEqual(path.read_bytes(), original)
        with self.assertRaises(ValueError):
            profiles.load_profile(self.package, "500", import_settings=source, import_v7=self.root)

    def test_atomic_import_cannot_overwrite_file_created_by_another_launch(self):
        source = self.write_json(self.root / "legacy.json", {"left_speed": 60})
        real_link = settings.os.link
        destination = self.package / "profiles" / "100rpm.json"

        def competing_link(temporary, path):
            complete = json.loads(Path(temporary).read_text(encoding="utf-8"))
            self.assertEqual(complete["robot_id"], "100rpm")
            Path(path).write_bytes(b"another launch")
            return real_link(temporary, path)

        with patch.object(settings.os, "link", side_effect=competing_link):
            with self.assertRaises(FileExistsError):
                profiles.load_profile(self.package, "100", import_settings=source)
        self.assertEqual(destination.read_bytes(), b"another launch")
        self.assertEqual(list(destination.parent.iterdir()), [destination])

    def test_explicit_v7_import_preserves_observed_wiring_camera_and_100_tuning(self):
        folder = self.root / "v7"
        folder.mkdir()
        (folder / "motors_optimized.py").write_text(
            "LEFT_SPEED = 61\nRIGHT_SPEED = 60\nAUTO_SECONDARY_SPEED = 38\n"
            "TURN_DURATION_LEFT = 0.22\nIR_PINS = (21, 7, 26)\n"
            "REVERSED = (False, False, True, False)\n"
            "MOTOR_PINS = ((17, 27, 18), (22, 23, 25), (16, 19, 13), (5, 6, 12))\n",
            encoding="utf-8")
        (folder / "vision_settings.py").write_text("CAMERA_BACKEND = 'usb'\nCAMERA_INDEX = 2\n",
                                                     encoding="utf-8")
        originals = {path.name: path.read_bytes() for path in folder.iterdir()}
        config = profiles.load_profile(self.package, "100", import_v7=folder)[1]
        self.assertEqual(config["left_speed"], 61)
        self.assertEqual(config["right_speed"], 60)
        self.assertEqual(config["auto_secondary_speed"], 38)
        self.assertEqual(config["turn_duration_left"], .22)
        self.assertEqual(config["motor_channels"], ("M1A", "M1B", "M2B", "M2A"))
        self.assertEqual(config["ir_pins"], (21, 7, 26))
        self.assertEqual(config["reversed"], (False, False, True, False))
        self.assertEqual(config["camera_backend"], "usb")
        self.assertEqual(config["camera_index"], 2)
        self.assertEqual({path.name: path.read_bytes() for path in folder.iterdir()}, originals)

    def test_independent_caps_and_hard_ceiling_validate_for_both_bots(self):
        for bot in ("100rpm", "500rpm"):
            with self.subTest(bot=bot):
                config = settings.validate({"left_speed": 12, "right_speed": 12,
                                            "auto_secondary_speed": 7,
                                            "diagonal_secondary_speed": 5}, bot)
                self.assertEqual(config["imu_drive_duty"], PRESETS[bot]["imu_drive_duty"])
                self.assertEqual(config["fc_max_duty"], PRESETS[bot]["fc_max_duty"])
                for invalid in ({"left_speed": 70.01}, {"fc_max_duty": 51},
                                {"imu_drive_duty": 51}, {"imu_turn_max_duty": 51},
                                {"imu_turn_min_duty": 40, "imu_turn_max_duty": 30},
                                {"imu_max_heading_trim": 30, "imu_drive_duty": 25}):
                    with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                        settings.validate(invalid, bot)


if __name__ == "__main__":
    unittest.main()

"""Settings/import tests run without Pi modules or sensor access."""

import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

import robot_settings as settings


class SettingsTests(unittest.TestCase):
    def test_defaults_are_complete_and_independent(self):
        data = settings.validate({})
        self.assertEqual(set(data), set(settings.FIELDS))
        self.assertEqual(data["camera_startup_timeout"], 30)
        data["left_speed"] = 80
        self.assertEqual(settings.DEFAULTS["left_speed"], 25)

    def test_unknown_nonfinite_boolean_and_wrong_types_rejected(self):
        for data in ({"typo": 1}, {"left_speed": float("nan")},
                     {"ramp_rate": float("inf")}, {"left_speed": 10 ** 1000}, {"left_speed": True},
                     {"camera_index": 1.0}, {"camera_rotate_180": 1},
                     {"reversed": [1, 0, 0, 0]}, {"ir_pins": [7, 7, 21]},
                     {"ir_pins": [7, 24, 21]}, {"camera_backend": "other"}, []):
            with self.subTest(data=data), self.assertRaises(ValueError):
                settings.validate(data)

    def test_bounds_and_related_settings(self):
        for data in ({"left_speed": 0}, {"camera_startup_timeout": 61},
                     {"single_motor_seconds": 6}, {"single_motor_duty": 51},
                     {"fc_max_duty": 51}, {"fc_max_duty": 19},
                     {"auto_min_correct_left": 1}, {"auto_overshoot_right": 1},
                     {"left_speed": 13}, {"diagonal_secondary_speed": 75},
                     {"imu_turn_min_duty": 26}, {"imu_max_heading_trim": 26}):
            with self.subTest(data=data), self.assertRaises(ValueError):
                settings.validate(data)
        data = settings.validate({"left_speed": 35, "right_speed": 35,
                                  "auto_secondary_speed": 25, "fc_max_duty": 25,
                                  "fc_uncertain_duty": 20})
        self.assertEqual(data["left_speed"], 35)

    def test_editor_parsing_and_roundtrip_format(self):
        self.assertEqual(settings.parse_value("ir_pins", "J7, J10, J9"), (21, 7, 26))
        self.assertEqual(settings.parse_value("ir_pins", "20, 7, 21"), (20, 7, 21))
        self.assertEqual(settings.parse_value("reversed", "on, true, yes, 1"), (True,) * 4)
        self.assertEqual(settings.parse_value("reversed", "off, false, no, 0"), (False,) * 4)
        for key, value in settings.DEFAULTS.items():
            with self.subTest(key=key):
                self.assertEqual(settings.parse_value(key, settings.format_value(key, value)), value)
        for key, text in (("ir_pins", "J7,J7,J9"), ("reversed", "true,false"),
                          ("camera_index", "1.2"), ("camera_auto_exposure", "maybe")):
            with self.subTest(key=key), self.assertRaises(ValueError):
                settings.parse_value(key, text)

    def test_atomic_roundtrip_and_missing_defaults(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "robot_settings.json"
            self.assertEqual(settings.load_settings(path), settings.validate({}))
            data = settings.validate({"reversed": [True] * 4, "ir_pins": [21, 7, 26]})
            settings.save_settings(path, data)
            self.assertEqual(settings.load_settings(path), data)
            self.assertEqual(list(Path(folder).glob("*.tmp")), [])
            original = path.read_bytes()
            with self.assertRaises(ValueError):
                settings.save_settings(path, {"single_motor_duty": 100})
            self.assertEqual(path.read_bytes(), original)

    def test_failed_replace_retains_old_file_and_cleans_temporary(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "settings.json"
            path.write_text('{"camera_auto_exposure": true}', encoding="utf-8")
            original = path.read_bytes()
            with patch.object(settings.os, "replace", side_effect=OSError("denied")):
                with self.assertRaises(OSError):
                    settings.save_settings(path, {})
            self.assertEqual(path.read_bytes(), original)
            self.assertEqual([item.name for item in Path(folder).iterdir()], [path.name])

    def test_malformed_duplicate_unknown_and_nonfinite_json_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "settings.json"
            for text in ('{', '[]', '{"typo": 3}', '{"left_speed": NaN}',
                         '{"camera_index": 0, "camera_index": 1}'):
                path.write_text(text, encoding="utf-8")
                with self.subTest(text=text), self.assertRaises(ValueError):
                    settings.load_settings(path)

    def test_import_preserves_pi_tuning_without_executing_code(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            sentinel = root / "must-not-exist"
            (root / "motors_optimized.py").write_text(
                f"open({str(sentinel)!r}, 'w').write('executed')\n"
                "REVERSED = (True, True, True, True)\nIR_PINS = (21, 7, 26)\n"
                "LEFT_SPEED = 65\nRIGHT_SPEED = 64\nAUTO_SECONDARY_SPEED = 45\n"
                "AUTO_MAX_CORRECT = 1.2\nTURN_DURATION_LEFT = 0.20\n", encoding="utf-8")
            (root / "vision_support.py").write_text("VISION_START_TIMEOUT = 30\n", encoding="utf-8")
            (root / "vision_settings.py").write_text(
                "CAMERA_BACKEND = 'usb'\nCAMERA_INDEX = 1\nCAMERA_ROTATE_180 = False\n",
                encoding="utf-8")
            (root / "ir_confidence.py").write_text("MAX_DUTY = 40\nFILTER_SECONDS = 0.04\n", encoding="utf-8")
            data, report = settings.import_v7(root)
            self.assertEqual(data["reversed"], (True,) * 4)
            self.assertEqual(data["ir_pins"], (21, 7, 26))
            self.assertEqual(data["auto_max_correct"], 1.2)
            self.assertEqual(data["turn_duration_left"], .20)
            self.assertEqual(data["camera_backend"], "usb")
            self.assertFalse(data["camera_rotate_180"])
            self.assertEqual(data["fc_max_duty"], 25)
            self.assertFalse(sentinel.exists())
            self.assertTrue(any("CLI" in line for line in report))

    def test_import_refuses_dynamic_conditional_augmented_and_invalid_pins(self):
        bad_sources = (
            "REVERSED = tuple([True] * 4)\n",
            "REVERSED = (True,) * 4\n",
            "IR_PINS = (7, 24, 21)\n",
            "if True:\n    IR_PINS = (21, 7, 26)\n",
            "LEFT_SPEED = 70\nLEFT_SPEED += 1\n",
            "LEFT_SPEED, RIGHT_SPEED = 70, 69\n",
            "IR_PINS = [7, 26, 21]\nIR_PINS[0] = 20\n",
            "IR_PINS = [7, 26, 21]\nIR_PINS.reverse()\n",
        )
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "motors_optimized.py"
            for source in bad_sources:
                path.write_text(source, encoding="utf-8")
                with self.subTest(source=source), self.assertRaises(ValueError):
                    settings.import_v7(folder)

    def test_import_requires_correct_folder_and_ignores_function_locals(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):
                settings.import_v7(folder)
            (Path(folder) / "motors_optimized.py").write_text(
                "LEFT_SPEED = 70\ndef unrelated():\n    LEFT_SPEED = get_value()\n",
                encoding="utf-8")
            data, _ = settings.import_v7(folder)
            self.assertEqual(data["left_speed"], 25)

    def test_apply_updates_all_runtime_consumers_without_hardware(self):
        modules = {name: types.SimpleNamespace() for name in
                   ("motors_optimized", "vision_support", "ir_confidence", "single_motor", "imu_tests",
                    "lane_gyro", "tof_support")}
        modules["vision_support"].settings = types.SimpleNamespace()
        with patch.object(settings.importlib, "import_module", side_effect=modules.__getitem__):
            settings.apply_settings({"reversed": [True] * 4, "auto_max_correct": 1.2,
                                     "camera_startup_timeout": 45, "fc_max_duty": 40,
                                     "single_motor_seconds": 2, "gyro_lane_ref_step": 1.0,
                                     "tof_stop_cm": 4, "tof_resume_cm": 6, "tof_slow_cm": 10})
        self.assertEqual(modules["motors_optimized"].REVERSED, (True,) * 4)
        self.assertEqual(modules["ir_confidence"].CORRECTION_TIMEOUT, 1.2)
        self.assertEqual(modules["ir_confidence"].MAX_DUTY, 40)
        self.assertEqual(modules["vision_support"].VISION_START_TIMEOUT, 45)
        self.assertEqual(modules["single_motor"].TEST_SECONDS, 2)
        self.assertEqual(modules["vision_support"].settings.CAMERA_BACKEND, "csi")
        self.assertEqual(modules["lane_gyro"].REF_STEP, 1.0)
        self.assertEqual(modules["lane_gyro"].KI, 0.15)
        self.assertEqual(modules["tof_support"].HOLD_MM, 40)
        self.assertEqual(modules["tof_support"].RESUME_MM, 60)
        self.assertEqual(modules["tof_support"].SLOW_MM, 100)

    def test_tof_range_validation_and_saved_profile(self):
        for values in ({"tof_stop_cm": 9}, {"tof_stop_cm": 10, "tof_resume_cm": 11},
                       {"tof_slow_cm": 0}, {"tof_resume_cm": 201}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                settings.validate(values)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "profile.json"
            settings.save_settings(path, {"tof_stop_cm": 4, "tof_resume_cm": 6, "tof_slow_cm": 12})
            data = settings.load_settings(path)
            self.assertEqual([data[k] for k in ("tof_stop_cm", "tof_resume_cm", "tof_slow_cm")], [4, 6, 12])


if __name__ == "__main__":
    unittest.main()

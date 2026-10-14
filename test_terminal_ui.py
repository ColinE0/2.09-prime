"""Menu, settings and terminal rendering tests without GPIO or sensor imports."""

from copy import deepcopy
import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import terminal_ui as ui


class ScriptedIO:
    def __init__(self, *answers):
        self.answers = iter(answers)
        self.messages = []

    def read(self, prompt):
        self.messages.append(prompt)
        return next(self.answers)

    def write(self, text):
        self.messages.append(text)

    @property
    def output(self):
        return "\n".join(self.messages)


class MenuTests(unittest.TestCase):
    def test_global_shortcuts_keep_course_arguments_and_battery_alias(self):
        scripted = ScriptedIO(" c   frl ", "b", "tt", "vt", "set", "q")
        menu = ui.TerminalMenu(scripted.read, scripted.write)
        self.assertEqual([menu.choose({}) for _ in range(6)],
                         ["C FRL", "B", "TT", "VT", "SET", "Q"])

    def test_numeric_category_and_test_remember_category(self):
        scripted = ScriptedIO("1", "2", "4")
        menu = ui.TerminalMenu(scripted.read, scripted.write)
        self.assertEqual(menu.choose({}), "FR")
        self.assertEqual(menu.category, 0)
        self.assertEqual(menu.choose({}), "BR")
        self.assertEqual(menu.category, 0)

    def test_back_changes_category_and_blank_does_not_run_anything(self):
        scripted = ScriptedIO("1", "0", "3", "", "5")
        menu = ui.TerminalMenu(scripted.read, scripted.write)
        self.assertEqual(menu.choose({}), "BAT")
        self.assertEqual(menu.category, 2)

    def test_codes_work_inside_other_categories_and_unknown_reprompts(self):
        scripted = ScriptedIO("4", "999", "not-a-command", "C RLF")
        menu = ui.TerminalMenu(scripted.read, scripted.write)
        self.assertEqual(menu.choose({}), "C RLF")
        self.assertEqual(menu.category, 3)
        self.assertEqual(scripted.output.count("Unknown choice."), 2)

    def test_help_keeps_category_and_covers_every_command(self):
        scripted = ScriptedIO("4", "H", "1")
        menu = ui.TerminalMenu(scripted.read, scripted.write)
        self.assertEqual(menu.choose({}), "RV")
        self.assertEqual(menu.category, 3)
        for command in ui.COMMANDS:
            self.assertIn(command, scripted.output)
        self.assertIn("B is an alias for BAT", scripted.output)

    def test_status_uses_actual_flags_caps_and_saved_settings_banner(self):
        scripted = ScriptedIO("1", "FL")
        menu = ui.TerminalMenu(scripted.read, scripted.write)
        status = dict(vision_enabled=True, tof_enabled=False, imu_status="calibration expired",
                      last_result="Stopped: sensor stale", settings_pending=True,
                      single_duty=23, single_seconds=0.6)
        self.assertEqual(menu.choose(status), "FL")
        self.assertIn("Camera lights (F/FA/C/FG/CG/FC): ON | ToF (FC and CG): OFF", scripted.output)
        self.assertIn("up to 23% for 0.6s", scripted.output)
        self.assertIn("IMU: calibration expired", scripted.output)
        self.assertIn("Last result: Stopped: sensor stale", scripted.output)
        self.assertIn("Saved settings apply when you leave SET", scripted.output)
        self.assertEqual(status["single_duty"], 23)

    def test_optional_status_defaults_and_metadata(self):
        scripted = ScriptedIO("4", "RV", "0", "5", "SET")
        menu = ui.TerminalMenu(scripted.read, scripted.write)
        self.assertEqual(menu.choose(dict(camera_summary="USB 1, upright")), "RV")
        self.assertEqual(menu.choose(dict(config_path="robot_settings.json")), "SET")
        self.assertIn("IMU: not checked", scripted.output)
        self.assertIn("Active camera: USB 1, upright", scripted.output)
        self.assertIn("Settings file: robot_settings.json", scripted.output)
        self.assertNotIn("Saved settings apply when you leave SET", scripted.output)

    def test_lane_limits_and_imu_movement_are_visible(self):
        scripted = ScriptedIO("2", "FC", "0", "5", "IF")
        menu = ui.TerminalMenu(scripted.read, scripted.write)
        self.assertEqual(menu.choose(dict(fc_max_duty=42)), "FC")
        self.assertEqual(menu.choose(), "IF")
        self.assertIn("FC: up to 10s, maximum 42% duty", scripted.output)
        self.assertIn("FA/C use original IR steering", scripted.output)
        self.assertIn("IMU forward 25%, turn maximum 25%", scripted.output)
        self.assertIn("left turn, 90 degrees", scripted.output)

    def test_menu_text_is_ascii(self):
        scripted = ScriptedIO("H", "Q")
        ui.TerminalMenu(scripted.read, scripted.write).choose()
        scripted.output.encode("ascii")


class SettingsEditorTests(unittest.TestCase):
    def setUp(self):
        import robot_settings
        self.settings = robot_settings
        self.active = deepcopy(robot_settings.DEFAULTS)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "robot_settings.json"
        self.settings.save_settings(self.path, self.active)
        self.original_bytes = self.path.read_bytes()

    def choices(self, key):
        group = self.settings.FIELDS[key].group
        groups = list(dict.fromkeys(field.group for field in self.settings.FIELDS.values()))
        keys = [name for name, field in self.settings.FIELDS.items() if field.group == group]
        return str(groups.index(group) + 1), str(keys.index(key) + 1)

    def test_success_saves_new_snapshot_without_applying_or_mutating_active(self):
        group, field = self.choices("single_motor_duty")
        scripted = ScriptedIO(group, field, "23", "0", "0")
        before = deepcopy(self.active)
        with patch.object(self.settings, "apply_settings") as apply:
            saved = ui.edit_settings(self.active, self.path, scripted.read, scripted.write)
        self.assertEqual(saved["single_motor_duty"], 23)
        self.assertEqual(self.settings.load_settings(self.path)["single_motor_duty"], 23)
        self.assertEqual(self.active, before)
        self.assertIsNot(saved, self.active)
        apply.assert_not_called()
        self.assertIn("Saved settings apply when you leave SET", scripted.output)

    def test_full_validation_rejects_inconsistent_cap_without_saving(self):
        group, field = self.choices("fc_max_duty")
        scripted = ScriptedIO(group, field, "19", "0", "0")
        saved = ui.edit_settings(self.active, self.path, scripted.read, scripted.write)
        self.assertEqual(saved, self.active)
        self.assertEqual(self.path.read_bytes(), self.original_bytes)
        self.assertIn("Not saved:", scripted.output)
        self.assertIn("fc_uncertain_duty", scripted.output)

    def test_failed_disk_save_keeps_old_file_and_saved_snapshot(self):
        group, field = self.choices("camera_index")
        scripted = ScriptedIO(group, field, "4", "0", "0")
        with patch.object(self.settings, "save_settings", side_effect=OSError("disk is full")):
            saved = ui.edit_settings(self.active, self.path, scripted.read, scripted.write)
        self.assertEqual(saved, self.active)
        self.assertEqual(self.path.read_bytes(), self.original_bytes)
        self.assertIn("Not saved: disk is full", scripted.output)

    def test_later_failed_edit_retains_first_successful_save(self):
        group, field = self.choices("camera_index")
        scripted = ScriptedIO(group, field, "4", field, "999", "0", "0")
        saved = ui.edit_settings(self.active, self.path, scripted.read, scripted.write)
        self.assertEqual(saved["camera_index"], 4)
        self.assertEqual(self.settings.load_settings(self.path)["camera_index"], 4)
        self.assertEqual(self.active["camera_index"], 0)

    def test_blank_cancels_edit_and_zero_is_valid_value_at_value_prompt(self):
        self.active["camera_index"] = 3
        group, field = self.choices("camera_index")
        scripted = ScriptedIO(group, field, "", field, "0", "0", "")
        saved = ui.edit_settings(self.active, self.path, scripted.read, scripted.write)
        self.assertEqual(saved["camera_index"], 0)
        self.assertEqual(self.active["camera_index"], 3)
        self.assertIn("Canceled; saved settings unchanged.", scripted.output)

    def test_invalid_input_reprompts_with_no_write(self):
        group, field = self.choices("camera_index")
        scripted = ScriptedIO("unknown", group, "999", field, "one", "", "0")
        saved = ui.edit_settings(self.active, self.path, scripted.read, scripted.write)
        self.assertEqual(saved, self.active)
        self.assertEqual(self.path.read_bytes(), self.original_bytes)
        self.assertIn("Unknown settings group", scripted.output)
        self.assertIn("Unknown setting.", scripted.output)
        self.assertIn("Not saved:", scripted.output)

    def test_group_menu_lists_groups_without_entire_field_wall(self):
        scripted = ScriptedIO("0")
        ui.edit_settings(self.active, self.path, scripted.read, scripted.write)
        for group in {field.group for field in self.settings.FIELDS.values()}:
            self.assertIn(group, scripted.output)
        self.assertNotIn("Camera index:", scripted.output)
        self.assertNotIn("Left motor baseline (%):", scripted.output)

    def test_tuple_directions_and_camera_choices_have_usable_inputs(self):
        group, field = self.choices("reversed")
        scripted = ScriptedIO(group, field, "on, off, on, off", "0", "0")
        saved = ui.edit_settings(self.active, self.path, scripted.read, scripted.write)
        self.assertEqual(saved["reversed"], (True, False, True, False))
        group, field = self.choices("camera_backend")
        scripted = ScriptedIO(group, field, "usb", "0", "0")
        saved = ui.edit_settings(saved, self.path, scripted.read, scripted.write)
        self.assertEqual(saved["camera_backend"], "usb")
        self.assertIn("Choices: csi, usb", scripted.output)


class TTYBuffer(io.StringIO):
    def isatty(self):
        return True

    def fileno(self):
        return 1


class LiveStatusTests(unittest.TestCase):
    def test_tty_updates_are_clipped_and_clear_previous_long_line(self):
        output = TTYBuffer()
        status = ui.LiveStatus(output)
        with patch.object(ui.os, "get_terminal_size", return_value=os.terminal_size((30, 24))):
            status("Camera: " + "x" * 100)
            status("Camera: none")
            status.finish()
        lines = output.getvalue().split("\r")
        self.assertEqual(len(lines[1]), 29)
        self.assertTrue(lines[1].endswith("..."))
        self.assertEqual(lines[2], "Camera: none" + " " * 17 + "\n")

    def test_significant_fault_finishes_live_line_and_remains_complete(self):
        output = TTYBuffer()
        status = ui.LiveStatus(output)
        with patch.object(ui.os, "get_terminal_size", return_value=os.terminal_size((25, 24))):
            status("ToF: 42.0 cm")
            status("Stopped: ToF reading is stale; check the sensor wiring")
            status("Camera: error: entire diagnostic must be preserved even when long")
        self.assertEqual(output.getvalue(),
                         "\rToF: 42.0 cm\n"
                         "Stopped: ToF reading is stale; check the sensor wiring\n"
                         "Camera: error: entire diagnostic must be preserved even when long\n")

    def test_non_tty_prints_all_telemetry_without_carriage_returns(self):
        output = io.StringIO()
        status = ui.LiveStatus(output)
        status.log("Accel XYZ [g]: one")
        status.log("Accel XYZ [g]: two")
        status.finish()
        self.assertEqual(output.getvalue(), "Accel XYZ [g]: one\nAccel XYZ [g]: two\n")

    def test_all_existing_telemetry_prefixes_update_one_line(self):
        output = TTYBuffer()
        status = ui.LiveStatus(output)
        for text in ("IR raw L/R/F=1/1/1", "Left IR=1  Right IR=1  Front IR=1",
                     "Battery at sensor: 7.80 V", "Heading +1.2 deg | rate 0.0 deg/s"):
            status(text)
        status.finish()
        self.assertEqual(output.getvalue().count("\r"), 4)
        self.assertEqual(output.getvalue().count("\n"), 1)

    def test_width_fallback_and_finish_are_idempotent(self):
        output = TTYBuffer()
        status = ui.LiveStatus(output)
        with patch.object(ui.os, "get_terminal_size", side_effect=OSError("no terminal")):
            status("Camera: " + "x" * 100)
        status.finish()
        status.finish()
        self.assertEqual(len(output.getvalue()), 1 + 79 + 1)
        self.assertEqual(output.getvalue().count("\n"), 1)

    def test_plain_output_keeps_complete_copyable_lines_even_on_tty(self):
        output = TTYBuffer()
        status = ui.LiveStatus(output, compact=False)
        original = "Camera: " + "x" * 200
        status(original)
        status(original)
        status.finish()
        self.assertEqual(output.getvalue(), original + "\n" + original + "\n")

    def test_ir_readout_stays_on_one_line_under_plain_output_and_pipes(self):
        for output, compact in ((TTYBuffer(), False), (io.StringIO(), True), (io.StringIO(), False)):
            status = ui.LiveStatus(output, compact=compact)
            status("Left IR=1  Right IR=1  Front IR=1")
            status("Left IR=0  Right IR=1  Front IR=1")
            status("IR: stopped.")
            self.assertEqual(output.getvalue(),
                             "\rLeft IR=1  Right IR=1  Front IR=1"
                             "\rLeft IR=0  Right IR=1  Front IR=1\n"
                             "IR: stopped.\n")

    def test_plain_output_still_stacks_other_telemetry(self):
        output = io.StringIO()
        status = ui.LiveStatus(output, compact=False)
        status("IR raw L/R/F=1/1/1")
        status("IR raw L/R/F=1/0/1")
        self.assertEqual(output.getvalue(), "IR raw L/R/F=1/1/1\nIR raw L/R/F=1/0/1\n")

    def test_camera_summary_keeps_action_age_and_processing_with_object(self):
        output = TTYBuffer()
        status = ui.LiveStatus(output)
        original = ("Camera: red_light right center=(220, 42) (2.3% frame)"
                    " | confirmed: red_light | line: wait"
                    " | frame age 0.12s | processing 0.04s")
        with patch.object(ui.os, "get_terminal_size", return_value=os.terminal_size((80, 24))):
            status(original)
        full, live = output.getvalue().split("\n")
        self.assertEqual(full, original)
        self.assertIn("red@(220,42)/2.3%", live)
        self.assertIn("line wait", live)
        self.assertIn("age 0.12s", live)
        self.assertIn("proc 0.04s", live)
        self.assertLessEqual(len(live) - 1, 79)

    def test_camera_changes_keep_full_lines_but_positions_alone_do_not(self):
        output = TTYBuffer()
        status = ui.LiveStatus(output)
        first = ("Camera: red_light right center=(220, 42) (2.3% frame)"
                 " | confirmed: red_light | line: wait"
                 " | frame age 0.12s | processing 0.04s")
        moved = first.replace("220, 42", "219, 43").replace("2.3%", "2.4%").replace("0.12s", "0.10s")
        cleared = ("Camera: none | confirmed: none | line: stop"
                   " | frame age 0.09s | processing 0.03s")
        with patch.object(ui.os, "get_terminal_size", return_value=os.terminal_size((80, 24))):
            status(first)
            status(moved)
            status(cleared)
            status.finish()
        self.assertIn(first + "\n", output.getvalue())
        self.assertNotIn(moved + "\n", output.getvalue())
        self.assertIn(cleared + "\n", output.getvalue())
        self.assertIn("red@(219,43)/2.4%", output.getvalue())


if __name__ == "__main__":
    unittest.main()

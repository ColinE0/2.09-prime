"""v9.5: the merge of the two v9.4 branches (FG/CG gyro lanes and IR wheel calibration).

Checks the seams between them: FG output passes through the saved wheel factors,
auto-balance says which factors it measured under, profiles migrate from v9.4 first,
and SET and help carry both features.
"""

import unittest
from unittest.mock import patch

import motors_optimized as motor
import robot_profiles
import robot_settings as settings
import terminal_ui
from test_v9_4 import Rig, SimulatedLaneCase


def run_fg(case, rig):
    with case.assertRaises(motor.StopRequested):
        rig.control.run("FG")
    rig.assert_off(case)


def balance_line(rig):
    return next(text for _, text in rig.messages if text.startswith("Auto-balance"))


class MergeSeamTests(SimulatedLaneCase):
    def test_auto_balance_reports_no_corrections_by_default(self):
        rig = Rig(events=[(10.0, "s")], right_gain=1.04, margin=10.0)
        run_fg(self, rig)
        self.assertIn("Measured with no wheel corrections applied.", balance_line(rig))

    def test_auto_balance_flags_saved_corrections(self):
        with patch.object(motor, "WHEEL_FACTORS_FORWARD", (1.0, 0.96, 1.0, 0.96)):
            rig = Rig(events=[(10.0, "s")], right_gain=1.04, margin=10.0)
            run_fg(self, rig)
        self.assertIn("Measured with saved wheel corrections applied", balance_line(rig))

    def test_fg_output_passes_through_wheel_factors(self):
        factors = (0.9, 0.8, 0.9, 0.8)
        with patch.object(motor, "WHEEL_FACTORS_FORWARD", factors):
            rig = Rig(events=[(3.0, "s")], margin=10.0)
            seen = []
            original = rig.driver.set_duties

            def spy(targets):
                original(targets)
                seen.append((list(rig.driver.duties), [pwm.duty for pwm in rig.gpio.pwms]))
            rig.driver.set_duties = spy
            run_fg(self, rig)
        driving = [(req, out) for req, out in seen if all(req)]
        self.assertTrue(driving, "FG never drove all four wheels")
        for requested, applied in driving:
            for index, factor in enumerate(factors):
                self.assertAlmostEqual(applied[index], requested[index] * factor, places=6)


class MergeWiringTests(unittest.TestCase):
    def test_profiles_migrate_from_v9_9_then_kalebs_folders_then_v9_5_then_v9_4(self):
        folders = robot_profiles._PREVIOUS_PROFILE_FOLDERS
        v9_9 = robot_profiles._V9_9_FOLDERS
        kaleb = robot_profiles._KALEB_FOLDERS
        self.assertEqual(folders[:len(v9_9)], v9_9)
        self.assertEqual(v9_9[-1], "MPU6050_Motor_Test_v9_9")
        self.assertEqual(folders[len(v9_9):len(v9_9) + len(kaleb)], kaleb)
        self.assertEqual(folders[len(v9_9) + len(kaleb):len(v9_9) + len(kaleb) + 3],
                         ("MPU6050_Motor_Test_v9_5", "MPU6050_Motor_Test_v9_4", "MPU6050_Motor_Test_v9_3"))

    def test_set_menu_and_help_carry_both_features(self):
        groups = {field.group for field in settings.FIELDS.values()}
        self.assertIn("Gyro lane (FG/CG)", groups)
        self.assertIn("Wheel calibration", groups)
        source = open(terminal_ui.__file__, encoding="utf-8").read()
        for command in ("FG", "CG", "PPROBE", "PCAL"):
            self.assertIn(f'("{command}",', source)
        self.assertIn("ROBOT TEST MENU v9.10", source)


if __name__ == "__main__":
    unittest.main()

"""main.py: the one-command launcher and its switch into ~/robot-test-env."""

import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import main


class VenvSwitchTests(unittest.TestCase):
    def setUp(self):
        self.home = Path(tempfile.mkdtemp())
        self.venv = self.home / "robot-test-env"
        patcher = patch.object(main, "VENV", self.venv)
        patcher.start()
        self.addCleanup(patcher.stop)
        env = patch.dict(os.environ, {}, clear=False)
        env.start()
        self.addCleanup(env.stop)
        os.environ.pop("ROBOT_BENCH_NO_REEXEC", None)

    def make_venv(self):
        (self.venv / "bin").mkdir(parents=True)
        (self.venv / "bin" / "python").write_text("")

    def test_runs_in_place_when_there_is_no_venv(self):
        self.assertIsNone(main.venv_python())

    def test_switches_to_the_venv_from_system_python(self):
        self.make_venv()
        with patch.object(sys, "prefix", "/usr"):
            self.assertEqual(main.venv_python(), self.venv / "bin" / "python")

    def test_stays_put_when_already_inside_the_venv(self):
        self.make_venv()
        with patch.object(sys, "prefix", str(self.venv)):
            self.assertIsNone(main.venv_python())

    def test_never_switches_twice(self):
        self.make_venv()
        os.environ["ROBOT_BENCH_NO_REEXEC"] = "1"
        with patch.object(sys, "prefix", "/usr"):
            self.assertIsNone(main.venv_python())


if __name__ == "__main__":
    unittest.main()

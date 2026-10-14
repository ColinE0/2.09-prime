#!/usr/bin/env python3
"""Run the robot test bench: `python3 main.py` from this folder.

Plain `python3` is fine: if ~/robot-test-env exists and is not already the
running interpreter, this restarts itself under that venv, which is where the
ToF driver lives. Any arguments (--bot 500, --show-settings, ...) pass through.
"""

import os
from pathlib import Path
import sys

VENV = Path.home() / "robot-test-env"


def venv_python():
    """Return the venv interpreter to switch to, or None to run as is."""
    python = VENV / "bin" / "python"
    if os.environ.get("ROBOT_BENCH_NO_REEXEC") or not python.exists():
        return None
    if Path(sys.prefix).resolve() == VENV.resolve():
        return None
    return python


if __name__ == "__main__":
    python = venv_python()
    if python is not None:
        os.environ["ROBOT_BENCH_NO_REEXEC"] = "1"  # never bounce twice
        os.execv(str(python), [str(python), str(Path(__file__).resolve()), *sys.argv[1:]])
    from app import main
    raise SystemExit(main())

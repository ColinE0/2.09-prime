"""Small ASCII menus and live readouts for an SSH terminal; no hardware access."""

from copy import deepcopy
import os
import re
import sys


CATEGORIES = (
    ("Motor tests", (
        ("FL", "Front-left motor only"),
        ("FR", "Front-right motor only"),
        ("BL", "Back-left motor only"),
        ("BR", "Back-right motor only"),
        ("F", "Drive forward"),
        ("SR", "Strafe right (timed)"),
        ("SL", "Strafe left (timed)"),
        ("R", "Turn right (timed)"),
        ("L", "Turn left (timed)"),
        ("TR", "Right intersection turn"),
        ("TL", "Left intersection turn"),
        ("DR", "Diagonal right (timed)"),
        ("DL", "Diagonal left (timed)"),
    )),
    ("Lane following and courses", (
        ("FA", "Original IR lane centering"),
        ("FC", "IR confidence lane-following test"),
        ("C", "Original IR course, such as C FRL"),
        ("FG", "IR lanes with gyro heading hold (IC and IV first)"),
        ("CG", "Gyro course: FG lanes and measured turns, such as CG FRL"),
        ("GO", "START button runs the last drive command, timed (or GO CG FRL)"),
        ("VT", "Toggle camera traffic lights at front lines (lane modes)"),
        ("TT", "Toggle ToF obstacle responses for FC and CG"),
    )),
    ("Sensor readouts (motors off)", (
        ("IR", "Raw IR readings"),
        ("IRC", "IR confidence readings"),
        ("IMU", "MPU-6050 live readings"),
        ("TOF", "ToF distance readings"),
        ("BAT", "Battery voltage, current and charge estimate"),
        ("ID", "Identify the IMU"),
        ("TT", "Toggle ToF obstacle responses for FC and CG"),
        ("BTN", "Board START/RESET buttons and status LED"),
    )),
    ("Camera", (
        ("RV", "Ryan's live detections (motors off)"),
        ("RVD", "RV debug: why each colored blob is or isn't a light; 'p' saves a snapshot"),
        ("VT", "Toggle camera traffic lights at front lines (lane modes)"),
    )),
    ("Calibration and settings", (
        ("IC", "Calibrate gyro while resting still"),
        ("IV", "Verify IMU orientation by turning robot by hand"),
        ("IF", "IMU heading hold: forward, 2s, duty from settings"),
        ("ITL", "IMU measured left turn, 90 degrees"),
        ("ITR", "IMU measured right turn, 90 degrees"),
        ("SET", "Edit settings (apply on leaving SET)"),
        ("BOT", "Select robot / reload its saved settings"),
        ("PPROBE", "Check removable FC-51 probe (motors off)"),
        ("PCAL", "Calibrate four wheels with the removable IR probe"),
    )),
)
COMMANDS = {code for _, entries in CATEGORIES for code, _ in entries} | {"B", "Q"}


def choose_robot(read=input, write=print):
    """Select explicitly with motors/hardware closed; never guess on blank input."""
    while True:
        write("\nSELECT ROBOT")
        write("  1. 500 RPM bot")
        write("  2. 100 RPM bot (104 RPM rated)")
        write("Q Quit | Motors are off")
        answer = read("Robot: ").strip().upper()
        if answer in {"1", "500", "500RPM"}:
            return "500"
        if answer in {"2", "100", "100RPM"}:
            return "100"
        if answer in {"Q", "0"}:
            return None
        write("Choose 1 or 2, or Q to quit.")


def _on_off(value):
    return "ON" if value else "OFF"


class TerminalMenu:
    """Return existing controller commands without changing any hardware state.

    ``read`` has the input(prompt) interface; ``write`` accepts a single string.
    The current category remains selected when choose() is called after a test.
    """

    def __init__(self, read=input, write=print):
        self.read, self.write = read, write
        self.category = None

    def _status(self, status):
        self.write("\nROBOT TEST MENU v9.10")
        self.write("Active robot: " + str(status.get("robot_label") or "not selected"))
        if "power_guard_enabled" in status:
            self.write("INA219 power guard: " + _on_off(status["power_guard_enabled"]) + " | BAT diagnoses | SET configures")
        self.write("Camera lights (F/FA/C/FG/CG/FC): " + _on_off(status.get("vision_enabled", False))
                   + " | ToF (FC and CG): " + _on_off(status.get("tof_enabled", False)))
        self.write("IMU: " + str(status.get("imu_status") or "not checked"))
        if "wheel_calibration" in status:
            self.write("Wheel correction: " + status["wheel_calibration"])
        if "go_command" in status:
            self.write("START button (GO): " + (status["go_command"]
                       or "run a drive command first, or GO CG FRL"))
        if status.get("last_result"):
            self.write("Last result: " + str(status["last_result"]))
        if status.get("settings_pending", False):
            self.write("Saved settings apply when you leave SET")

    def _show(self, status):
        self._status(status)
        if self.category is None:
            for index, (name, _) in enumerate(CATEGORIES, 1):
                self.write(f"  {index}. {name}")
        else:
            name, entries = CATEGORIES[self.category]
            self.write("\n" + name)
            if self.category == 0:
                if "motor_channels" in status and "motor_reversed" in status:
                    self.write("Active wiring: " + " | ".join(
                        f"{wheel}={channel} {'reversed' if reverse else 'normal'}"
                        for wheel, channel, reverse in zip(("FL", "FR", "BL", "BR"),
                            status["motor_channels"], status["motor_reversed"])))
                duty = status.get("single_duty", 25)
                seconds = status.get("single_seconds", 1)
                self.write(f"FL/FR/BL/BR: forward, up to {duty:g}% for {seconds:g}s. Lift wheels.")
            if self.category == 1:
                self.write(f"FC: up to 10s, maximum {status.get('fc_max_duty', 25):g}% duty.")
                self.write("FA/C use original IR steering. VT: a green crosses the front line, red/yellow stops "
                           "at it until green. TT applies to FC and CG.")
            if self.category == 3 and status.get("camera_summary"):
                self.write("Active camera: " + str(status["camera_summary"]))
            if self.category == 4 and status.get("config_path"):
                self.write("Settings file: " + str(status["config_path"]))
            if self.category == 4:
                self.write(f"IMU forward {status.get('imu_drive_duty', 25):g}%, "
                           f"turn maximum {status.get('imu_turn_max_duty', 25):g}%.")
            for index, (code, label) in enumerate(entries, 1):
                self.write(f"  {index:2}. {code:<4} {label}")
        self.write("0 Back | BOT Select robot | H All commands | Q Quit | Command codes work anywhere")

    def _help(self):
        self.write("\nCommand reference; use 's' during a test to stop and return.")
        for name, entries in CATEGORIES:
            self.write(name + ":")
            for code, label in entries:
                self.write(f"  {code:<4} {label}")
        self.write("B is an alias for BAT. C FRL runs a course. VT: camera lights at front lines "
                   "(F, FA, C, FG, CG, FC). TT affects FC and CG.")
        self.write("GO arms the last drive command (or GO CG FRL) for START: 5 s countdown, then a run")
        self.write("timer in tenths. RESET (if this robot has one) stops a run and stays armed; 's' leaves GO.")
        self.write("0 returns to the category menu; Q quits.")
        self.write("BOT stops/closes hardware before selecting a robot; saved settings stay separate.")

    def choose(self, status=None):
        status = status or {}
        while True:
            self._show(status)
            answer = self.read("Choose: ").strip().upper()
            if not answer:
                continue
            if answer == "0":
                self.category = None
                continue
            if answer == "H":
                self._help()
                continue
            if answer.isdecimal():
                number = int(answer)
                entries = CATEGORIES if self.category is None else CATEGORIES[self.category][1]
                if 1 <= number <= len(entries):
                    if self.category is None:
                        self.category = number - 1
                        continue
                    return entries[number - 1][0]
            else:
                parts = answer.split(maxsplit=1)
                if parts[0] in COMMANDS:
                    # Keep C's argument for the controller's existing validation.
                    return " ".join(parts)
            self.write("Unknown choice. Enter a listed number, command code, or H for help.")


def edit_settings(config, path, read=input, write=print):
    """Edit a saved snapshot, returning it without mutating the running settings.

    Each accepted field is validated as a complete configuration and saved by
    robot_settings' atomic writer. A failed validation or save leaves the last
    successfully saved snapshot unchanged. The caller applies nothing live.
    """
    import robot_settings as settings

    saved = deepcopy(settings.validate(config))
    groups = {}
    for key, field in settings.FIELDS.items():
        if key in {"motion_profile", "robot_id", "wheel_calibration"}:
            continue
        groups.setdefault(field.group, []).append(key)
    group_names = list(groups)
    group = None
    while True:
        write("\nSAVED ROBOT SETTINGS")
        write("Saved settings apply when you leave SET")
        write("Hardware restarts with motors off; IMU calibration and camera/ToF toggles reset.")
        write("File: " + str(path))
        if group is None:
            for index, name in enumerate(group_names, 1):
                write(f"  {index}. {name}")
            answer = read("Settings group (0 or blank returns): ").strip()
            if not answer or answer == "0":
                return deepcopy(saved)
            if not answer.isdecimal() or not 1 <= int(answer) <= len(group_names):
                write("Unknown settings group. Enter a listed number or 0.")
                continue
            group = group_names[int(answer) - 1]
            continue

        write("\n" + group)
        keys = groups[group]
        for index, key in enumerate(keys, 1):
            field = settings.FIELDS[key]
            current = settings.format_value(key, saved[key])
            write(f"  {index:2}. {field.label}: {current}")
        answer = read("Setting number (0 or blank goes back): ").strip()
        if not answer or answer == "0":
            group = None
            continue
        if not answer.isdecimal() or not 1 <= int(answer) <= len(keys):
            write("Unknown setting. Enter a listed number or 0.")
            continue
        key = keys[int(answer) - 1]
        field = settings.FIELDS[key]
        write(field.label + " - " + field.help)
        if key == "reversed":
            write("Enter FL, FR, BL or BR to flip just that wheel, or four on/off values.")
            write("Current: " + " | ".join(
                f"{wheel}={channel} {'reversed' if reverse else 'normal'}"
                for wheel, channel, reverse in zip(("FL", "FR", "BL", "BR"),
                    saved["motor_channels"], saved["reversed"])))
        if getattr(field, "choices", ()):
            write("Choices: " + ", ".join(map(str, field.choices)))
        elif field.kind == "bool":
            write("Enter ON or OFF.")
        if field.minimum is not None or field.maximum is not None:
            lower = "unbounded" if field.minimum is None else str(field.minimum)
            upper = "unbounded" if field.maximum is None else str(field.maximum)
            write(f"Allowed range: {lower} to {upper}")
        current = settings.format_value(key, saved[key])
        value = read(f"New value (current {current}; blank cancels): ").strip()
        if not value:
            write("Canceled; saved settings unchanged.")
            continue
        try:
            candidate = deepcopy(saved)
            if key == "reversed" and value.upper() in {"FL", "FR", "BL", "BR"}:
                flags = list(saved[key])
                index = ("FL", "FR", "BL", "BR").index(value.upper())
                flags[index] = not flags[index]
                candidate[key] = tuple(flags)
            else:
                candidate[key] = settings.parse_value(key, value)
            candidate = settings.validate(candidate)
            settings.save_settings(path, candidate)
        except (OSError, ValueError) as exc:
            write("Not saved: " + str(exc))
            continue
        if saved["wheel_calibration_enabled"] and not candidate["wheel_calibration_enabled"]:
            write("Wheel corrections disabled. Changes to wiring or reference duty require recalibration.")
        saved = deepcopy(candidate)
        write(f"Saved {field.label}: {settings.format_value(key, saved[key])}")
        write("Saved settings apply when you leave SET")


class LiveStatus:
    """Keep recurring telemetry on one TTY line, preserving significant events."""

    PREFIXES = ("Camera:", "ToF:", "Accel XYZ", "IR raw L/R/F=", "Left IR=",
                "Battery at sensor:", "Heading ")
    # The IR readout stays on one line even under --plain-output or piped through tee (Colin,
    # 2026-09-27): it only prints on a change, so stacked lines were just scroll. A tee'd log
    # still holds every state, separated by carriage returns.
    ALWAYS_LIVE = ("Left IR=",)
    # GO's run timer redraws ten times a second in place, or prints once a second as a
    # plain line under --plain-output or tee.
    REFRESH_IN_PLACE, REFRESH_PLAIN = 0.1, 1.0

    def __init__(self, stream=None, compact=True, status=None):
        self.stream = sys.stdout if stream is None else stream
        self.compact = compact
        self.active = False
        self.previous_width = 0
        self.camera_state = None
        # GO's run timer, a callable returning e.g. "RUN 12.3 s" ("" before it starts). It
        # heads the live line and every printed line; on a TTY the live line stays at the
        # bottom of the screen.
        self.status = status
        self.live_text = ""
        self.next_refresh = None

    def _is_tty(self):
        try:
            return self.stream.isatty()
        except (AttributeError, OSError):
            return False

    def _width(self):
        try:
            columns = os.get_terminal_size(self.stream.fileno()).columns
        except (AttributeError, OSError, ValueError):
            columns = 80
        return max(1, columns - 1)  # Avoid wrapping at the final terminal column.

    @staticmethod
    def _clip(text, width):
        if len(text) <= width:
            return text
        return text[:max(0, width - 3)] + "." * min(3, width)

    def _camera_summary(self, text, width):
        """Summarize known camera output, retaining the line decision/age ahead of verbose boxes."""
        parts = text.split(" | ")
        if len(parts) < 5 or not parts[1].startswith("confirmed:"):
            return text, None
        action = re.search(r"\bline: (cross|wait|end|stop)\b", parts[2])
        age = re.search(r"frame age ([+-]?[0-9.]+s)", parts[3])
        processing = re.search(r"processing ([0-9.]+s)", parts[4])
        if not (action and age and processing):
            return text, None
        labels = "stop_sign|red_light|yellow_light|green_light"
        items = re.findall(r"\b(" + labels + r")\b(.*?)(?=,\s*(?:" + labels + r")\b|$)",
                           parts[0][len("Camera: "):])
        summaries = []
        for label, details in items:
            short = {"stop_sign": "stop", "red_light": "red", "yellow_light": "yellow",
                     "green_light": "green"}[label]
            center = re.search(r"center=(\([^)]*\))", details)
            area = re.search(r"\(([0-9.]+%) frame\)", details)
            if center:
                short += "@" + center.group(1).replace(" ", "")
            if area:
                short += "/" + area.group(1)
            summaries.append(short)
        summary = ", ".join(summaries) if summaries else "none"
        tail = f" | line {action.group(1)} | age {age.group(1)} | proc {processing.group(1)}"
        room = max(1, width - len("Camera: ") - len(tail))
        line = "Camera: " + self._clip(summary, room) + tail
        # Positions, areas and timings fluctuate continuously; preserve a full
        # line when detection/confirmation or the line decision actually changes.
        state = (tuple(sorted(label for label, _ in items)), parts[1], parts[2])
        return line, state

    def _status_text(self):
        return self.status() if self.status is not None else ""

    def _prefix(self):
        heading = self._status_text()
        return heading + " | " if heading else ""

    def _in_place(self):
        return self.compact and self._is_tty()

    def _draw(self, text, width):
        text = self._clip(text, width)
        erase = max(0, min(self.previous_width, width) - len(text))
        self.stream.write("\r" + text + " " * erase)
        self.previous_width = len(text)
        self.active = True

    def _draw_status(self):
        heading = self._status_text()
        if heading:
            self._draw(heading + (" | " + self.live_text if self.live_text else ""), self._width())

    def _break(self):
        """Before a full line: telemetry stays in the scrollback, GO's timer line is erased
        (and redrawn after the line)."""
        if self.active and self._status_text() and self._in_place():
            self.stream.write("\r" + " " * self.previous_width + "\r")
            self.active = False
            self.previous_width = 0
        else:
            self.finish()

    def __call__(self, text):
        text = str(text)
        important = any(marker in text.lower() for marker in ("fault:", "error:", "warning:", "stopped:"))
        live = text.startswith(self.ALWAYS_LIVE) or (
            self.compact and self._is_tty() and text.startswith(self.PREFIXES))
        prefix = self._prefix()
        if live and "\n" not in text and not important:
            width = self._width()
            if text.startswith("Camera:"):
                summary, camera_state = self._camera_summary(text, max(1, width - len(prefix)))
                if camera_state is not None and camera_state != self.camera_state:
                    self._break()
                    self.stream.write(prefix + text + "\n")
                    self.camera_state = camera_state
                text = summary
            self.live_text = text
            self._draw(prefix + text, width)
        else:
            self._break()
            self.stream.write(prefix + text + "\n")
            if prefix and self._in_place():
                self._draw_status()
        self.stream.flush()

    log = __call__

    def refresh(self, now):
        """Redraw GO's run timer; called from every control-loop poll and rate-limited here."""
        heading = self._status_text()
        if not heading or (self.next_refresh is not None and now < self.next_refresh - 1e-9):
            return
        if self._in_place():
            self.next_refresh = now + self.REFRESH_IN_PLACE
            self._draw_status()
        else:
            self.next_refresh = now + self.REFRESH_PLAIN
            self.finish()
            self.stream.write(heading + "\n")
        self.stream.flush()

    def finish(self):
        if self.active:
            self.stream.write("\n")
            self.stream.flush()
            self.active = False
            self.previous_width = 0

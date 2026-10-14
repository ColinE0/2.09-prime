"""GO: run the last drive command from the board's START button, with the PRD countdown and run timer.

The PRD asks for a start button, a 5 second wait with a countdown before moving, and the
elapsed trip time in tenths of a second, starting when the countdown ends and stopping when
the run ends. The time shows in the SSH terminal on the laptop. No hardware on import.

GO arms the last drive command, or the one typed after it (GO CG FRL). Each START press runs
it exactly as the menu would, under the power guard: the LED and the terminal count down
5 seconds, the timer starts, the command runs, and the final time prints in large digits.
The session stays armed for the next press. RESET stops a countdown or a run; 's' does the
same and also leaves GO. F, FA and FG end at their first front line instead of waiting
for 'g' (with the camera on, the first one they stop at for good: a green crosses, a red
waits for green). A power stop disarms GO. A run counts as completed only if the robot drove and the
command reached its end (front line, course end or stop sign); anything else is a DNF.
"""

import math
import time

import motors_optimized as motor

COURSE_COMMANDS = {"C", "CG"}
# Drive commands only: no readouts, settings or lifted single-wheel tests.
GO_COMMANDS = (motor.MODES - {"IR"}) | {"FC", "FG", "CG", "IF", "ITL", "ITR"}
IMU_COMMANDS = {"FG", "CG", "IF", "ITL", "ITR"}  # need a current IC and IV at each press
# FC always calls go_ready(), after its camera and ToF if on; with the camera on (VT), the
# other lane modes start it first too. Either way the clock starts with the wheels.
SENSORS_FIRST = {"FC"}
COUNTDOWN_SECONDS = 5  # PRD: wait 5 s after the start button, with a visible countdown
IMU_MARGIN_SECONDS = 10.0  # IC lifetime a gyro run needs left at the press, past the countdown
START_HOLD_SECONDS = 0.04  # START: a press counts once it holds 40 ms
ARMED_PERIOD, ARMED_ON = 1.0, 0.15  # armed: a short LED blip once a second


def tenths(seconds):
    """Trip time to a tenth, truncated like a stopwatch (never rounded up)."""
    return f"{math.floor(seconds * 10 + 1e-6) / 10:.1f}"


FONT = {
    "0": ("###", "# #", "# #", "# #", "###"),
    "1": ("  #", "  #", "  #", "  #", "  #"),
    "2": ("###", "  #", "###", "#  ", "###"),
    "3": ("###", "  #", "###", "  #", "###"),
    "4": ("# #", "# #", "###", "  #", "  #"),
    "5": ("###", "#  ", "###", "  #", "###"),
    "6": ("###", "#  ", "###", "# #", "###"),
    "7": ("###", "  #", "  #", "  #", "  #"),
    "8": ("###", "# #", "###", "# #", "###"),
    "9": ("###", "# #", "###", "  #", "###"),
    ".": (" ", " ", " ", " ", "#"),
}


def big_text(text):
    """Five rows of block digits, doubled across so they read square in a terminal."""
    return ["  ".join(FONT[char][row].replace("#", "##").replace(" ", "  ") for char in text).rstrip()
            for row in range(5)]


def resolve(argument, last, read=input):
    """The (command, course) GO arms: the one typed after GO, else the last drive command."""
    typed = argument.strip().upper()
    if typed:
        parts = typed.split(maxsplit=1)
        command, sequence = parts[0], parts[1] if len(parts) > 1 else ""
    elif last is not None:
        command, sequence = last
    else:
        raise ValueError("nothing to arm yet. Run a drive command first, or type it after GO, e.g. GO CG FRL.")
    if command not in GO_COMMANDS:
        raise ValueError(f"{command} cannot be armed. GO runs drive commands: "
                         + " ".join(sorted(GO_COMMANDS)) + ".")
    if command not in COURSE_COMMANDS:
        if typed and sequence:
            raise ValueError(f"{command} takes no course. Courses are C or CG, e.g. GO CG FRL.")
        return command, ""
    if not sequence:
        sequence = read("Enter course, e.g. FRL (blank cancels): ").strip()
        if not sequence:
            raise ValueError("course cancelled.")
    course = "".join(sequence.upper().split())
    if any(char not in "FRL" for char in course):
        raise ValueError("Course must contain only F, R, and L, e.g. FRL")
    return command, course


class RunTimer:
    """Trip time from the end of the countdown to the end of the run."""

    def __init__(self, clock=time):
        self.clock = clock
        self.started = self.stopped = None

    def start(self):
        self.started, self.stopped = self.clock.monotonic(), None

    def stop(self, at=None):
        """Stop the clock now, or at ``at``: when the robot stopped, if the run only ended
        after the camera's look at the line."""
        if self.started is not None and self.stopped is None:
            self.stopped = self.clock.monotonic() if at is None else max(self.started, at)

    def elapsed(self):
        if self.started is None:
            return None
        end = self.clock.monotonic() if self.stopped is None else self.stopped
        return max(0.0, end - self.started)

    def status(self):
        """The live line's heading; empty until the countdown ends."""
        elapsed = self.elapsed()
        return "" if elapsed is None else f"RUN {tenths(elapsed)} s"


class Switch:
    """Debounced switch: a change counts after ``reads`` agreeing reads spanning ``hold`` seconds."""

    def __init__(self, hold=START_HOLD_SECONDS, reads=2):
        self.hold, self.reads = hold, reads
        self.down = None  # unknown until the first read, so a switch held at arming is not a press
        self.since = None
        self.count = 0

    def update(self, raw, now):
        """Return True on a debounced press."""
        if self.down is None:
            self.down = raw
            return False
        if raw == self.down:
            self.since, self.count = None, 0
            return False
        if self.since is None:
            self.since = now
        self.count += 1
        if self.count < self.reads or now - self.since < self.hold - 1e-9:
            return False
        self.down, self.since, self.count = raw, None, 0
        return raw


class ButtonKeys:
    """The stop-key reader GO hands to every control loop: RESET held stops like 's'.

    Every loop polls its keys each tick, so RESET takes the keyboard's stop path and the
    power guard, IMU and camera checks run exactly as from the menu. The poll also redraws
    the live run timer and notes whether the wheels have driven.
    """

    go_run = True  # Controller.go_run(): F, FA and FG end at their first front line

    def __init__(self, keys, motors, clock=time):
        self.keys, self.motors, self.clock = keys, motors, clock
        self.start = Switch()
        # RESET stops on its second LOW read in a row, like BTN: a false stop costs a rerun,
        # a missed one does not stop the robot.
        self.reset = Switch(hold=0.0)
        self.begin_run(None, None)

    def begin_run(self, timer, live):
        self.timer, self.live = timer, live
        self.stop_source = None  # "key" or "reset": which stop ended the countdown or run
        self.outcome = None  # why a run that returned did not complete, e.g. "FC time limit"
        self.moved = False  # any wheel duty since the countdown ended
        self.pending = None  # the countdown, waiting until the run's sensors are up

    def ready(self):
        """A run calls this once its camera and ToF are running: the countdown runs now."""
        if self.pending is not None:
            countdown, self.pending = self.pending, None
            countdown()

    def end_run(self, outcome=None, at=None):
        """The command's own end: stop the clock before any sensor shutdown, and note why."""
        if self.timer is not None:
            self.timer.stop(at)
        if outcome and self.outcome is None:
            self.outcome = outcome

    def buttons(self):
        """Read both switches; True on a debounced START press."""
        now = self.clock.monotonic()
        start, reset = self.motors.read_buttons()
        self.reset.update(reset, now)
        return self.start.update(start, now)

    def poll(self):
        keys = self.keys.poll()
        self.buttons()
        if any(self.motors.duties):
            if self.pending is not None:  # nothing may drive before the countdown has run
                self.motors.stop()
                raise motor.LaneFault("GO: a wheel was driven before the countdown; stopped")
            self.moved = True
        if "s" in keys:
            self.stop_source = self.stop_source or "key"
        elif self.reset.down:  # held, not just pressed: a stop button must not be missed
            self.stop_source = self.stop_source or "reset"
            keys += "s"
        if self.live is not None:
            self.live.refresh(self.clock.monotonic())
        return keys


def wait_for_start(keys, motors, clock=time):
    """Armed with the motors off: the LED blips once a second. True on START, False on 's'."""
    begin = clock.monotonic()
    lit = None
    while True:
        now = clock.monotonic()
        if "s" in keys.keys.poll():
            return False
        if keys.buttons():
            return True
        on = (now - begin) % ARMED_PERIOD < ARMED_ON
        if on != lit:
            motors.set_status_led(on)
            lit = on
        delay = motor.LOOP_DT - (clock.monotonic() - now)
        if delay > 0:
            clock.sleep(delay)


def countdown(controller, timer, seconds=COUNTDOWN_SECONDS):
    """The PRD's wait after START: the LED flashes and the terminal counts each second.

    Every tick polls, so 's', RESET and the power guard can end it before anything moves.
    The run timer starts the moment it ends.
    """
    motors, clock, log = controller.motors, controller.clock, controller.log
    begin = clock.monotonic()
    shown = lit = None
    while True:
        now = clock.monotonic()
        controller.poll()
        left = seconds - (now - begin)
        if left <= 1e-9:
            break
        number = math.ceil(left - 1e-9)
        if number != shown:
            log(f"Starting in {number}")
            shown = number
        on = (now - begin) % 1.0 < 0.5
        if on != lit:
            motors.set_status_led(on)
            lit = on
        controller.tick_sleep(now)
    motors.set_status_led(True)  # solid while the run is on
    timer.start()
    log("GO: run timer started")


def armed_session(command, sequence, motors, execute, live_factory, preflight=None,
                  describe=None, keyboard=None, clock=time, write=print, sensors_first=None):
    """Stay armed until 's': each START press counts down, runs the command and times it.

    ``execute(command, sequence, live, keys, before, after)`` is the app's powered-command
    runner. It returns (result line, ending), ending being finished, stopped, fault or
    power. Returns the line for the menu's Last result. ``sensors_first(command)`` is true
    for a run that starts its sensors first and then calls ``Controller.go_ready()``.
    """
    if sensors_first is None:
        sensors_first = SENSORS_FIRST.__contains__
    label = " ".join(part for part in (command, sequence) if part)
    keyboard = motor.Keyboard if keyboard is None else keyboard
    try:
        motors.claim_board_ui()
    except Exception as exc:
        return (f"GO: could not claim the START/RESET/LED pins ({exc}). "
                "GPIO9/10 are SPI0 pins, so SPI must stay off.")
    runs = []  # (seconds, outcome) for every run in which the wheels drove
    try:
        with keyboard() as keyboard_keys:
            keys = ButtonKeys(keyboard_keys, motors, clock)
            while True:
                detail = describe(command) if describe is not None else ""
                write(f"\nARMED: {label}" + (f" ({detail})" if detail else ""))
                write("Set the robot on the start line and press START: 5 s countdown, then the "
                      "run timer. " + ("RESET stops a run; 's' returns to the menu." if motor.RESET_BTN is not None
                                       else "'s' stops a run and returns to the menu (RESET is off)."))
                if not wait_for_start(keys, motors, clock):
                    break
                problem = preflight(command) if preflight is not None else None
                if problem:
                    write("Not started: " + problem)
                    continue
                timer = RunTimer(clock)
                live = live_factory(timer.status)
                keys.begin_run(timer, live)

                def start_countdown():
                    countdown(motor.Controller(motors, keys, clock=clock, log=live), timer)

                def before(run_keys):
                    if sensors_first(command):
                        run_keys.pending = start_countdown  # the run calls ready() once its sensors run
                    else:
                        start_countdown()

                try:
                    result, ending = execute(command, sequence, live, keys,
                                             before=before, after=timer.stop)
                finally:
                    keys.pending = None
                    keys.live = None
                    motors.set_status_led(False)
                write(result)
                run = _report(timer.elapsed(), ending, keys, label, runs, write)
                if run is not None:
                    runs.append(run)
                if ending == "power":
                    write("GO disarmed: power protection stopped the run. Check BAT before arming again.")
                    break
                if keys.stop_source == "key":
                    break
    finally:
        motors.set_status_led(False)
    return _summary(label, runs)


def _report(elapsed, ending, keys, label, runs, write):
    """Print one run's time; return (seconds, outcome), or None when the wheels never drove."""
    if elapsed is None:
        write("Countdown cancelled; the robot did not move." if ending == "stopped"
              else "Not started (see the line above).")
        return None
    if not keys.moved:
        write("Not counted: the run ended before the wheels drove (see the line above).")
        return None
    if ending == "finished":
        outcome = keys.outcome or "completed"
    elif ending == "stopped":
        outcome = "stopped by RESET" if keys.stop_source == "reset" else "stopped with 's'"
    else:
        outcome = {"power": "power stop"}.get(ending, "stopped on a fault")
    number = len(runs) + 1
    if outcome == "completed":
        write("")
        for line in big_text(tenths(elapsed)):
            write("    " + line)
        write("")
        write(f"Run {number}: {tenths(elapsed)} s, completed ({label})")
    else:
        write(f"Run {number}: DNF, {outcome} at {tenths(elapsed)} s ({label})")
    times = [seconds for seconds, result in runs + [(elapsed, outcome)] if result == "completed"]
    if times:
        write(f"Best completed run this session: {tenths(min(times))} s")
    return elapsed, outcome


def _summary(label, runs):
    if not runs:
        return f"GO {label}: no runs."
    times = [seconds for seconds, outcome in runs if outcome == "completed"]
    count = f"{len(runs)} run" + ("" if len(runs) == 1 else "s")
    if not times:
        return f"GO {label}: {count}, none completed."
    return f"GO {label}: {count}, best {tenths(min(times))} s."

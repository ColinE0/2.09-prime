"""BTN: motors-off check of the board's START/RESET buttons and status LED; no hardware on import.

The pins are claimed on first use and stay claimed until the robot session closes.
A press or release counts once it holds for two control ticks (about 40 ms) and is
printed as it happens. The LED blinks once a second and stays on while either button
is held, so one run proves both switches and the LED.
"""

import motors_optimized as motor

BUTTON_COMMANDS = {"BTN"}
NAMES = ("START", "RESET")
DEBOUNCE_TICKS = 2
BLINK_SECONDS = 0.5  # LED half-period while no button is held


def _times(count):
    return f"{count} time" + ("" if count == 1 else "s")


class ButtonTests(motor.Controller):
    def readings(self):
        self.motors.stop()
        try:
            self.motors.claim_board_ui()
        except Exception as exc:
            raise ValueError(f"Could not claim the button/LED pins ({exc}). "
                             "GPIO9/10 are SPI0 pins, so SPI must stay off.") from exc
        held = list(self.motors.read_buttons())
        changes, presses = [0, 0], [0, 0]
        self.log("Board buttons and status LED; motors disabled. 's' returns to menu.")
        if motor.RESET_BTN is None:
            self.log(f"Press START (GPIO{motor.START_BTN}); RESET is off for this robot (SET -> Wiring). "
                     "The LED blinks once a second and stays on while START is held.")
        else:
            self.log(f"Press START (GPIO{motor.START_BTN}) and RESET (GPIO{motor.RESET_BTN}). "
                     "The LED blinks once a second and stays on while a button is held.")
        self.log("At start: " + ", ".join(f"{name} {'DOWN' if down else 'up'}" for name, down in zip(NAMES, held))
                 + (" (reads pressed with nothing touching it: check that switch for a short)" if any(held) else ""))
        led, shown = True, None
        next_toggle = self.clock.monotonic() + BLINK_SECONDS
        try:
            while True:
                started = self.clock.monotonic()
                self.poll()
                raw = self.motors.read_buttons()
                for index, name in enumerate(NAMES):
                    if raw[index] == held[index]:
                        changes[index] = 0
                        continue
                    changes[index] += 1
                    if changes[index] < DEBOUNCE_TICKS:
                        continue
                    held[index], changes[index] = raw[index], 0
                    if held[index]:
                        presses[index] += 1
                        self.log(f"{name} pressed ({presses[index]})")
                    else:
                        self.log(f"{name} released")
                if any(held):
                    led, next_toggle = True, started + BLINK_SECONDS
                elif started >= next_toggle:
                    led, next_toggle = not led, started + BLINK_SECONDS
                if led != shown:
                    self.motors.set_status_led(led)
                    shown = led
                self.tick_sleep(started)
        finally:
            self.motors.set_status_led(False)
            self.log(f"Buttons: START pressed {_times(presses[0])}"
                     + (f", RESET pressed {_times(presses[1])}." if motor.RESET_BTN is not None else "."))

    def run_test(self, command):
        if command not in BUTTON_COMMANDS:
            raise ValueError("Unknown button test")
        try:
            self.readings()
        finally:
            self.motors.stop()

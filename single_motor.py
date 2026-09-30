"""Brief, forward-only tests of one motor channel; no hardware on import."""

import math

import motors_optimized as motor
from motion_profile import PRESET

WHEELS = {"FL": (0, "front left"), "FR": (1, "front right"),
          "BL": (2, "back left"), "BR": (3, "back right")}
SINGLE_MOTOR_COMMANDS = set(WHEELS)
TEST_DUTY = PRESET["single_motor_duty"]
TEST_SECONDS = 1.0  # Includes the ramp; no extra powered hold afterward.
MAX_LOOP_GAP = 0.12


class SingleMotorTests(motor.Controller):
    def run_test(self, command):
        try:
            self.motors.stop(brake=False)
            if command not in WHEELS:
                raise ValueError("Choose FL, FR, BL or BR for a single-motor test")
            index, name = WHEELS[command]
            self.poll()
            self.log(f"{command}: {name} motor only, forward, up to {TEST_DUTY:g}% duty "
                     f"for {TEST_SECONDS:g}s including ramp. Other motors at zero PWM. "
                     "Keep the wheels lifted. 's' stops.")
            self.prepare(motor.FORWARD)
            targets = tuple(TEST_DUTY if wheel == index else 0.0 for wheel in range(4))
            started = previous = self.clock.monotonic()
            if not math.isfinite(started):
                raise motor.LaneFault("Invalid motor-test clock")
            while True:
                self.poll()
                now = self.clock.monotonic()
                if not math.isfinite(now) or not 0 <= now - previous <= MAX_LOOP_GAP:
                    raise motor.LaneFault("Single-motor test loop was interrupted; stopped")
                if now - started >= TEST_SECONDS - 1e-9:
                    break
                self.motors.slew(targets, motor.RAMP_RATE, now - previous)
                previous = now
                self.tick_sleep(now)
            self.motors.stop()  # Disable motion before terminal output can block.
            self.log(f"{command} test complete")
        finally:
            self.motors.stop()

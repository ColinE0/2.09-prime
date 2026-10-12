"""Final code for test day. Made by Marshall Morris.

Detectors by Ryan (traffic light, stop sign). Motor map and presets by Kaleb.

Navigation outline (what main() does)
  1. Setup menu (settings, IR test, wheel/turn/button tests), then the directions;
     the run begins when the START button is pressed
  2. Check all sensors, IMU, motors, camera
  3. Check for the traffic light (wait for green)
  4. While there are directions left:
  5.   Drive straight (IMU heading hold, left AND right IR sensors keep the lane) until the front IR sees the line
  6.   Front IR sees the line: stop
  7.   Check the traffic light, then do the next direction: LEFT / RIGHT turn, or STRAIGHT through
  8.   Drive straight out of the intersection and look for the next line
  9. After the last turn: drive to the final front line, confirm the stop sign, stop
 10. Stop the timer and write the log

Wheel order everywhere: FL, FR, BL, BR. Pins use BCM numbering.
Search for "EDIT" to find the settings you are expected to change.
"""

import collections
import csv
import json
import math
import os
import select
import signal
import sys
import termios
import threading
import time

import cv2
import numpy as np
import RPi.GPIO as GPIO
from smbus2 import SMBus

############ Button set up #####################
start_button = 10               # BCM 10 = physical pin 19 on the header
reset_button = 9                # BCM 9  = physical pin 21 on the header
# A press is any change from the level the pin has at start-up, so only the pull matters:
BUTTON_PULL = GPIO.PUD_UP       # EDIT: PUD_UP = button wired between the pin and GND (most common)
                                #       PUD_DOWN = button wired between the pin and 3.3 V
BUTTON_DEBOUNCE = 0.05

#### set up for pins and directions
BOT_ID = "500rpm"               # EDIT: "500rpm" or "100rpm"
directions = []                 # filled in by wait_for_start(): the program asks you (L or R per intersection)

#### course settings
LINE_DETECTED = 0               # value an IR sensor reads ON a line (your sensors read 1 = nothing, 0 = line)
LINE_CONFIRM_READS = 2          # consecutive reads with the front sensor on the line
USE_RAMP = True                 # gradual speed change: accelerate at RAMP_RATE instead of jumping to speed
USE_HEADING_HOLD = True         # heading correction: the gyro steers the robot back onto its heading
USE_SIDE_LINES = True           # lane correction: stay in the lane using the left AND right IR sensors
USE_WHEEL_SPEEDS = False        # drive each wheel at its own WHEEL_SPEEDS value instead of the left/right speeds
USE_TURN_WHEEL_SPEEDS = False   # turn with each wheel at its own TURN_WHEEL_DUTIES value
SIDE_LINE_TRIM = 3.0            # duty % of steering per sensor that sees a lane line
LANE_HOLD_TIME = 0.3            # s a lane correction is kept after the sensor stops seeing the line
START_DELAY = 3                 # s countdown between entering the directions and moving
STRAIGHT_TIME = 2.5             # s of straight driving to cross an intersection when the direction is STRAIGHT
USE_TRAFFIC_LIGHT = True        # wait for a green light at the stop lines
LIGHT_WAIT_TIMEOUT = 10.0       # s to wait for green before going anyway (0 = wait forever)
RUN_MODE = "full"               # "full" = IR + IMU + camera; "ir_only" = IR sensors only; "imu_only" = IMU only
LEG_TIME = 3.0                  # s to drive to the next intersection when there are no IR sensors (imu_only)
RUN_MODE_LABELS = {"full": "FULL (IR + IMU + camera)",
                   "ir_only": "IR ONLY (no IMU, no camera)",
                   "imu_only": "IMU ONLY (no IR, no camera)"}
LEG_TIMEOUT = 30.0              # s without finding a front line before the run stops itself
NO_LIGHT_FRAMES = 10            # frames with no light in view before the robot goes
STOP_SIGN_CONFIRM_TIME = 1.5    # s spent looking for the stop sign at the final line
MOTOR_TWITCH_CHECK = False      # True: nudge each wheel for 0.15 s during the start-up check
USE_POWER_GUARD = True          # stop the run on overcurrent or a low battery (INA219, power_guard.py)
POWER_OVERCURRENT_A = 4.5       # EDIT: stop when the pack current stays at or above this...
POWER_OVERCURRENT_HOLD = 0.2    #       ...for this many seconds. Starting values until stall current is measured
POWER_LOW_VOLTAGE_V = 6.4       # EDIT: stop when the pack stays at or below this while driving...
POWER_LOW_VOLTAGE_HOLD = 2.0    #       ...for this many seconds (rides out the dip when the motors start)
LOG_FILE = "run_log.txt"
PROFILES_FILE = "profiles.json"   # named sets of settings; edits are saved into the active profile
SETTINGS_FILE = "settings.json"   # old single settings file; moved into the profile "default" on first start
CAMERA_BACKEND = "auto"         # "auto" = try the Pi camera (picamera2), then a USB camera; or force one
                                #   of "picamera2" (ribbon-cable camera) / "v4l2" (USB webcam)
CAMERA_INDEX = 0                # USB camera number (/dev/video0)
CAMERA_REQUIRED = True          # True: stop the run if the camera does not open; False: carry on without it (no light or sign check)
CAMERA_FULL_FOV = True          # Pi camera: use the full sensor view (1640x1232 scaled to the frame size), not the zoomed-in crop
CAMERA_FLIP = True              # Pi camera: rotate the picture 180 degrees (camera mounted upside down)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
MPU6050_BUS = 1
MPU6050_ADDRESS = 0x68
IMU_YAW_SIGN = 1                # EDIT: -1 if turning the robot LEFT gives a negative yaw
IMU_MAX_AGE = 0.25              # s; older IMU data stops the run

############ Faults ############
class CourseFault(Exception):
    """Anything that should stop the run and return to the start screen."""


class IMUFault(CourseFault):
    """IMU data missing or stale."""


class ResetRequested(Exception):
    """The reset button was pressed."""


def mode_uses_ir():
    return RUN_MODE in ("full", "ir_only")


def mode_uses_imu():
    return RUN_MODE in ("full", "imu_only")


def mode_uses_camera():
    return RUN_MODE == "full"


##################pin layout for motor and code from Kaleb################################

STBY = 24  # shared standby pin for both TB6612FNG drivers

# Wheel order FL, FR, BL, BR. Each triple is (IN1, IN2, PWM): the documented rev C map.
# FL M1A (J2), FR M1B (J4), BL M2B (J12), BR M2A (J5).
# MOTOR_PINS = ((17, 27, 18), (22, 23, 25), (16, 19, 13), (5, 6, 12))
FL_AIN1, FL_AIN2, FL_PWM = 17, 27, 18
FR_AIN1, FR_AIN2, FR_PWM = 22, 23, 25
BL_AIN1, BL_AIN2, BL_PWM = 16, 19, 13
BR_AIN1, BR_AIN2, BR_PWM = 5, 6, 12

MOTOR_PINS = ((FL_AIN1, FL_AIN2, FL_PWM), (FR_AIN1, FR_AIN2, FR_PWM),
              (BL_AIN1, BL_AIN2, BL_PWM), (BR_AIN1, BR_AIN2, BR_PWM))
all_pins = [STBY, FL_AIN1, FL_AIN2, FL_PWM, FR_AIN1, FR_AIN2, FR_PWM,
            BL_AIN1, BL_AIN2, BL_PWM, BR_AIN1, BR_AIN2, BR_PWM]

REVERSED = (False, False, False, False)
PWM_FREQ = 1000
MAX_MOTOR_DUTY = 70.0  # Project ceiling; does not provide stall/current protection.
# Logical wheel order FL, FR, BL, BR, independent of electrical REVERSED wiring.
# Attenuation only: a factor can never raise an output above its request.
WHEEL_FACTORS_FORWARD = (1.0, 1.0, 1.0, 1.0)
WHEEL_FACTORS_REVERSE = (1.0, 1.0, 1.0, 1.0)

LOOP_DT = 0.02
DIRECTION_PAUSE = 0.05  # zero output before changing an active direction pattern
BRAKE_ON_STOP = True    # TB6612FNG short brake; False selects coast
TURN_APPROACH_TIME_RIGHT = 0.55   # s of straight driving after the line, before a right turn
TURN_APPROACH_TIME_LEFT = 1.5     # s of straight driving after the line, before a left turn
TURN_EXIT_STRAIGHT_TIME = 2.0     # s of straight driving after a turn (lines ignored)

TURN_DURATION_RIGHT = 0.12  # s of spinning for a right turn when TURN_MODE = "timed"
TURN_DURATION_LEFT = 0.14   # s of spinning for a left turn when TURN_MODE = "timed"

# Kept from the bench code; the course run does not use these yet.
STRAFE_DURATION = 0.6
DIAGONAL_DURATION = 0.6
DIAGONAL_YAW_TRIM = 0.0  # Positive trim rotates toward the slide; negative rotates away.

FORWARD = (True, True, True, True)
# Mecanum strafes and leans, checked on the robot: SR with the older pattern
# (FL/BR reverse) moved the 500 RPM bot LEFT on the mat (2026-09-24). Change both
# strafe patterns together, and re-run SR on each bot.
STRAFE_RIGHT = (True, False, False, True)
STRAFE_LEFT = (False, True, True, False)
TURN_RIGHT = (True, False, True, False)
TURN_LEFT = (False, True, False, True)

#################### motor profile from Kaleb ########################################
"""Independent starting motor/IMU settings for the two 6 V N20 robots.

The 100 RPM label refers to the original motors rated 104 RPM. Neither
preset establishes physical wiring or replaces calibration on the robot.
"""

BOT_IDS = ("500rpm", "100rpm")
BOT_LABELS = {"500rpm": "500 RPM bot", "100rpm": "100 RPM bot (104 RPM rated)"}
PRESETS = {
    "500rpm": {
        "left_speed": 25.0,
        "right_speed": 25.0,
        "diagonal_secondary_speed": 11.0,
        "ramp_rate": 35.0,
        "single_motor_duty": 25.0,
        "imu_drive_duty": 25.0,
        "imu_turn_max_duty": 25.0,
        "imu_turn_min_duty": 16.0,
        "imu_heading_kp": 0.6,
        "imu_heading_kd": 0.075,
        "imu_max_heading_trim": 5.0,
        "imu_turn_slow_gain": 0.4,
    },
    "100rpm": {
        "left_speed": 70.0,
        "right_speed": 69.0,
        "diagonal_secondary_speed": 30.0,
        "ramp_rate": 100.0,
        "single_motor_duty": 35.0,
        "imu_drive_duty": 50.0,
        "imu_turn_max_duty": 50.0,
        "imu_turn_min_duty": 32.0,
        "imu_heading_kp": 1.2,
        "imu_heading_kd": 0.15,
        "imu_max_heading_trim": 10.0,
        "imu_turn_slow_gain": 0.8,
    },
}
PRESET = PRESETS["500rpm"]


def normalize_bot_id(bot_id):
    """Canonical menu/CLI ID; reject unknown robots rather than guessing."""
    aliases = {"500": "500rpm", "100": "100rpm", "1": "500rpm", "2": "100rpm"}
    if not isinstance(bot_id, str):
        raise ValueError("Select a robot: 500rpm or 100rpm")
    bot_id = aliases.get(bot_id.strip().lower(), bot_id.strip().lower())
    if bot_id not in BOT_IDS:
        raise ValueError("Select a robot: 500rpm or 100rpm")
    return bot_id


# Active values; apply_preset() fills these from the selected robot (they used to be strings).
LEFT_SPEED = RIGHT_SPEED = RAMP_RATE = DIAGONAL_SECONDARY_SPEED = 0.0
WHEEL_SPEEDS = (0.0, 0.0, 0.0, 0.0)   # duty % per wheel (FL, FR, BL, BR); used when USE_WHEEL_SPEEDS is on
TURN_WHEEL_DUTIES = (0.0, 0.0, 0.0, 0.0)   # duty % per wheel while turning; used when USE_TURN_WHEEL_SPEEDS is on
TEST_DUTY = IMU_DRIVE_DUTY = TURN_MAX_DUTY = TURN_MIN_DUTY = 0.0
HEADING_KP = HEADING_KD = MAX_HEADING_TRIM = TURN_SLOW_GAIN = 0.0

# Turn and heading limits that are the same for both robots.
MAX_HEADING_ERROR = 12.0   # deg; a straight leg stops if the heading is this far off plan
TURN_ANGLE = 90.0
TURN_TOLERANCE = 2.0       # deg; power is cut this far before the target
TURN_TIMEOUT = 6.0         # s
TURN_STALL_TIME = 1.0      # s without 1 degree of progress stops the turn
TURN_STOP_LEAD = 0.0       # s of yaw rate anticipated before cutting power
TURN_MODE = "timed"        # "timed" = spin for TURN_DURATION_* seconds (default); "imu" = stop on the measured angle


def preset_wheel_values(preset, top_duty):
    """(FL, FR, BL, BR) duties that match the preset's left/right speeds, scaled so the faster side is top_duty."""
    left, right = preset["left_speed"], preset["right_speed"]
    scale = top_duty / max(left, right)
    return tuple(round(speed * scale, 1) for speed in (left, right, left, right))


def apply_preset(bot_id):
    """Point every tunable at the selected robot's preset."""
    global PRESET, LEFT_SPEED, RIGHT_SPEED, RAMP_RATE, DIAGONAL_SECONDARY_SPEED, TEST_DUTY
    global IMU_DRIVE_DUTY, TURN_MAX_DUTY, TURN_MIN_DUTY
    global HEADING_KP, HEADING_KD, MAX_HEADING_TRIM, TURN_SLOW_GAIN, WHEEL_SPEEDS, TURN_WHEEL_DUTIES
    PRESET = PRESETS[normalize_bot_id(bot_id)]
    LEFT_SPEED = PRESET["left_speed"]
    RIGHT_SPEED = PRESET["right_speed"]
    RAMP_RATE = PRESET["ramp_rate"]
    DIAGONAL_SECONDARY_SPEED = PRESET["diagonal_secondary_speed"]
    TEST_DUTY = PRESET["single_motor_duty"]
    IMU_DRIVE_DUTY = PRESET["imu_drive_duty"]
    TURN_MAX_DUTY = PRESET["imu_turn_max_duty"]
    TURN_MIN_DUTY = PRESET["imu_turn_min_duty"]
    HEADING_KP = PRESET["imu_heading_kp"]
    HEADING_KD = PRESET["imu_heading_kd"]
    MAX_HEADING_TRIM = PRESET["imu_max_heading_trim"]
    TURN_SLOW_GAIN = PRESET["imu_turn_slow_gain"]
    WHEEL_SPEEDS = preset_wheel_values(PRESET, PRESET["imu_drive_duty"])         # same as the left/right setting gives
    TURN_WHEEL_DUTIES = preset_wheel_values(PRESET, PRESET["imu_turn_max_duty"])


apply_preset(BOT_ID)

################### end of Kaleb code for motor Profile ##################################


################# motor driver ###################################
def wheel_factor(index, forward):
    """Attenuation for one wheel; never above 1.0."""
    factors = WHEEL_FACTORS_FORWARD if forward else WHEEL_FACTORS_REVERSE
    return min(1.0, factors[index])


class MotorDriver:
    """Four TB6612FNG channels. Duties are 0-100 %; direction comes from a pattern."""

    def __init__(self):
        self.pwm = []
        self.duty = [0.0] * 4
        self.pattern = None

    @property
    def ready(self):
        return bool(self.pwm)

    @property
    def duties(self):
        """Same list as duty; power_guard.py reads it under this name."""
        return self.duty

    def initialize(self):
        for pin in all_pins:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        self.pwm = [GPIO.PWM(pwm_pin, PWM_FREQ) for (_, _, pwm_pin) in MOTOR_PINS]
        for channel in self.pwm:
            channel.start(0)
        GPIO.output(STBY, GPIO.HIGH)  # take both drivers out of standby
        self.stop(brake=False)

    def _apply(self, index, duty, forward):
        in1, in2, _ = MOTOR_PINS[index]
        if REVERSED[index]:
            forward = not forward
        GPIO.output(in1, GPIO.HIGH if forward else GPIO.LOW)
        GPIO.output(in2, GPIO.LOW if forward else GPIO.HIGH)
        self.pwm[index].ChangeDutyCycle(duty)

    def prepare(self, pattern):
        """Zero the output before changing direction pattern."""
        if pattern != self.pattern:
            self.stop()
            time.sleep(DIRECTION_PAUSE)
            self.pattern = pattern

    def slew(self, targets, ramp_rate, dt):
        """Move each wheel's duty toward its target by at most ramp_rate %/s."""
        if self.pattern is None:
            raise CourseFault("prepare() must be called before slew()")
        step = ramp_rate * max(dt, 0.0) if USE_RAMP else 1000.0     # ramp off: go straight to the target
        for index, target in enumerate(targets):
            forward = self.pattern[index]
            target = min(max(target, 0.0), MAX_MOTOR_DUTY) * wheel_factor(index, forward)
            current = self.duty[index]
            current = min(current + step, target) if target > current else max(current - step, target)
            self.duty[index] = current
            self._apply(index, current, forward)

    def stop(self, brake=None):
        if not self.pwm:
            return
        brake = BRAKE_ON_STOP if brake is None else brake
        self.duty = [0.0] * 4
        for index, (in1, in2, _) in enumerate(MOTOR_PINS):
            level = GPIO.HIGH if brake else GPIO.LOW   # both inputs high = short brake
            GPIO.output(in1, level)
            GPIO.output(in2, level)
            self.pwm[index].ChangeDutyCycle(100 if brake else 0)

    def spin(self, indexes, duty, forward=True):
        """Run the chosen wheels (0=FL, 1=FR, 2=BL, 3=BR) until stop() is called."""
        duty = min(duty, MAX_MOTOR_DUTY)
        for index in indexes:
            self._apply(index, duty, forward)
            self.duty[index] = duty

    def twitch(self, duty=None, seconds=0.15):
        """Start-up check: nudge each wheel forward in turn (FL, FR, BL, BR)."""
        duty = min(TEST_DUTY if duty is None else duty, MAX_MOTOR_DUTY)
        for index in range(4):
            self._apply(index, duty, True)
            time.sleep(seconds)
            self.stop()
            time.sleep(0.1)

    def close(self):
        self.stop(brake=False)
        GPIO.output(STBY, GPIO.LOW)
        for channel in self.pwm:
            channel.stop()
        self.pwm = []
        GPIO.cleanup()


motors = MotorDriver()


######################### IR sensor set up (Marshall) ################################
# Three IR sensors: one at the FRONT (stop line), one on the LEFT and one on the RIGHT (lane lines).
# The pins can be changed in Settings > Lanes and lines. BCM numbering.
IR_FRONT_PIN = 26               # EDIT: front sensor, sees the stop line
IR_LEFT_PIN = 21                # EDIT: left sensor, sees the left lane line
IR_RIGHT_PIN = 7                # EDIT: right sensor, sees the right lane line
_ir_configured = None           # the pins last set up as inputs


def ir_pin_problem():
    """A text describing a wrong IR pin choice, or None when the three pins are usable."""
    pins = (IR_FRONT_PIN, IR_LEFT_PIN, IR_RIGHT_PIN)
    if len(set(pins)) != 3:
        return f"Two IR sensors share a pin (front {IR_FRONT_PIN}, left {IR_LEFT_PIN}, right {IR_RIGHT_PIN})"
    taken = {"STBY": STBY, "start button": start_button, "reset button": reset_button, "IMU SDA": 2, "IMU SCL": 3}
    for name, motor in zip(("FL", "FR", "BL", "BR"), MOTOR_PINS):
        for pin, role in zip(motor, ("IN1", "IN2", "PWM")):
            taken[f"{name} motor {role}"] = pin
    for label, pin in (("front", IR_FRONT_PIN), ("left", IR_LEFT_PIN), ("right", IR_RIGHT_PIN)):
        for name, used in taken.items():
            if pin == used:
                return f"The {label} IR sensor pin {pin} is already used by the {name}"
    return None


def setup_sensors():
    """Make the three IR pins inputs. Called again automatically if the pins are changed in Settings."""
    global _ir_configured
    pins = (IR_FRONT_PIN, IR_LEFT_PIN, IR_RIGHT_PIN)
    for pin in pins:
        GPIO.setup(pin, GPIO.IN)
    _ir_configured = pins


def read_sensors():  # sensor read
    """Raw pin values in the order LEFT, FRONT, RIGHT."""
    if _ir_configured != (IR_FRONT_PIN, IR_LEFT_PIN, IR_RIGHT_PIN):
        setup_sensors()
    return GPIO.input(IR_LEFT_PIN), GPIO.input(IR_FRONT_PIN), GPIO.input(IR_RIGHT_PIN)


def read_lines():
    """Same order as read_sensors() (left, front, right), as True/False: line seen under that sensor."""
    return tuple(value == LINE_DETECTED for value in read_sensors())

########################## IR sensor set up done ###########################


######################### IMU set-up and code ##############################
# ----- MPU-6050 register addresses (from the chip's datasheet) -----
SMPLRT_DIV = 0x19                    # sample-rate divider register
CONFIG = 0x1A                        # digital low-pass filter setting
GYRO_CONFIG = 0x1B                   # gyro full-scale range setting
ACCEL_CONFIG = 0x1C                  # accelerometer full-scale range setting
ACCEL_XOUT_H = 0x3B                  # first of 14 data bytes: accel XYZ, temp, gyro XYZ
PWR_MGMT_1 = 0x6B                    # power management: used to wake the chip
WHO_AM_I = 0x75                      # identity register, normally reads 0x68

ACCEL_SCALE = 16384.0                # raw units per g when range is +/-2 g
GYRO_SCALE = 65.5                    # raw units per deg/s when range is +/-500 deg/s


class MPU6050:
    """Minimal driver for the MPU-6050 gyro/accelerometer."""

    def __init__(self, bus=1, address=0x68):
        self.address = address                               # I2C address (0x68, or 0x69 if AD0 high)
        self.bus = SMBus(bus)                                # open /dev/i2c-<bus>
        who = self.bus.read_byte_data(address, WHO_AM_I)     # ask the chip who it is
        if who != 0x68:                                      # clones may answer differently
            print(f"Warning: WHO_AM_I = 0x{who:02X} (expected 0x68); continuing.")
        self.bus.write_byte_data(address, PWR_MGMT_1, 0x01)  # wake from sleep, use gyro X as clock
        time.sleep(0.1)                                      # give the chip time to start up
        self.bus.write_byte_data(address, SMPLRT_DIV, 0x04)  # 1 kHz / (1+4) = 200 samples per second
        self.bus.write_byte_data(address, CONFIG, 0x03)      # low-pass filter at about 44 Hz
        self.bus.write_byte_data(address, GYRO_CONFIG, 0x08) # gyro range +/-500 deg/s
        self.bus.write_byte_data(address, ACCEL_CONFIG, 0x00)# accel range +/-2 g
        self.gyro_bias = (0.0, 0.0, 0.0)                     # zero-rate offset, set by calibrate()

    @staticmethod
    def _signed(high, low):
        value = (high << 8) | low                            # join two bytes into one 16-bit number
        return value - 65536 if value & 0x8000 else value    # negative if the top bit is set

    def read_raw(self):
        data = self.bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 14)  # all 14 bytes at once
        v = [self._signed(data[i], data[i + 1]) for i in range(0, 14, 2)]    # 7 signed values
        accel = tuple(x / ACCEL_SCALE for x in v[0:3])                       # accel X,Y,Z in g
        temp = v[3] / 340.0 + 36.53                                          # temperature in deg C
        gyro = tuple(x / GYRO_SCALE for x in v[4:7])                         # gyro X,Y,Z in deg/s
        return accel, gyro, temp

    def read(self):
        accel, gyro, temp = self.read_raw()
        gyro = tuple(g - b for g, b in zip(gyro, self.gyro_bias))            # subtract the gyro offset
        return accel, gyro, temp

    def calibrate(self, seconds=2.0):
        samples = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            samples.append(self.read_raw()[1])                               # raw gyro only
            time.sleep(0.005)
        n = len(samples)
        self.gyro_bias = tuple(sum(s[i] for s in samples) / n for i in range(3))  # average = offset
        return self.gyro_bias

    def close(self):
        self.bus.close()


class AngleTracker(threading.Thread):
    """Keeps roll/pitch/yaw current in the background and stores every sample.

    Yaw is positive to the LEFT (counter-clockwise) when IMU_YAW_SIGN is right.
    If the IMU stops answering, get() raises IMUFault so the robot stops.
    """

    def __init__(self, imu, csv_path="angles.csv", history=20000, alpha=0.98):
        super().__init__(daemon=True)                          # daemon: dies with the program
        self.imu = imu
        self.alpha = alpha                                     # complementary filter: 0.98 trusts the gyro
        self.lock = threading.Lock()                           # protects data shared with the main thread
        self.running = True
        self.error = None                                      # set if the read loop dies
        self.last_update = time.monotonic()                    # time of the newest sample
        self.roll = self.pitch = self.yaw = 0.0                # degrees
        self.rate = 0.0                                        # yaw rate, deg/s
        self.yaw_zero = 0.0                                    # yaw value treated as 0 degrees
        self.history = collections.deque(maxlen=history)       # in-memory log
        self.csv_file = open(csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["time_s", "roll_deg", "pitch_deg", "yaw_deg", "yaw_rate_dps"])
        self.start_time = time.monotonic()

    def run(self):
        first = True                                           # first loop takes roll/pitch from gravity
        last = time.monotonic()
        try:
            while self.running:
                accel, gyro, _ = self.imu.read()
                now = time.monotonic()
                dt = now - last
                last = now
                ax, ay, az = accel
                accel_roll = math.degrees(math.atan2(ay, az))                    # roll implied by gravity
                accel_pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))  # pitch implied by gravity
                with self.lock:
                    if first:
                        self.roll, self.pitch = accel_roll, accel_pitch
                        first = False
                    else:
                        self.roll = self.alpha * (self.roll + gyro[0] * dt) + (1 - self.alpha) * accel_roll
                        self.pitch = self.alpha * (self.pitch + gyro[1] * dt) + (1 - self.alpha) * accel_pitch
                        self.yaw += gyro[2] * dt * IMU_YAW_SIGN        # gyro only: drifts slowly
                    self.rate = gyro[2] * IMU_YAW_SIGN
                    self.last_update = now
                    row = (now - self.start_time, self.roll, self.pitch, self.yaw - self.yaw_zero, self.rate)
                    self.history.append(row)
                    self.writer.writerow([f"{x:.4f}" for x in row])
                time.sleep(0.005)                              # about 200 updates per second
        except Exception as exc:                               # I2C failure: remember it for get()
            self.error = exc

    def get(self):
        """(roll, pitch, yaw, yaw rate); raises IMUFault if the data is not current."""
        if self.error is not None:
            raise IMUFault(f"IMU stopped: {self.error}")
        if time.monotonic() - self.last_update > IMU_MAX_AGE:
            raise IMUFault("IMU data is stale")
        with self.lock:
            return self.roll, self.pitch, self.yaw - self.yaw_zero, self.rate

    def zero_yaw(self):
        with self.lock:
            self.yaw_zero = self.yaw                           # current heading becomes 0 degrees

    def stop(self):
        self.running = False
        self.join(timeout=1.0)
        self.csv_file.close()


def heading_correction(angle, rate):
    """Steering trim in duty %, clamped to +/- MAX_HEADING_TRIM (PD control).

    angle is the heading error (positive = robot has turned LEFT of the plan).
    Positive trim slows the LEFT side (steers left, used when the robot has
    drifted right); negative trim slows the RIGHT side (steers right).
    """
    trim = -HEADING_KP * angle - HEADING_KD * rate
    return max(-MAX_HEADING_TRIM, min(MAX_HEADING_TRIM, trim))

################## end of imu code ################################


################# Traffic light Detection from Ryan #######################
# ============================================================
# Region of Interest (ROI Bounds as percentages 0.0 to 1.0)
# ============================================================

TOP_CUTOFF = 0.0        # 0.0 = top edge of frame
BOTTOM_CUTOFF = 0.25    # 0.25 = top 25% of frame

LEFT_CUTOFF = 0.2      # 0.2 = removes leftmost 20%
RIGHT_CUTOFF = 0.6     # 0.6 = removes rightmost 40%

# ============================================================
# General Candidate Requirements
# ============================================================

MIN_SATURATION = 50
MIN_VALUE = 150

MIN_AREA_RATIO = 0.00020   # Rejects tiny stray speckles
MAX_AREA_RATIO = 0.04      # Discards large reflections/housings

MIN_COLOR_RATIO = 0.15     # Requires candidate box to contain genuine color density
WINNER_RATIO = 1.30        # Must clearly beat secondary illuminated colors

MAX_PARENT_TO_CORE_RATIO = 10.0  # Prevents housing from swamping the bulb

HOLD_TIME = 0.0

# Minimum compactness (4 * pi * area / perimeter^2) to reject noisy jagged pixels
MIN_COMPACTNESS = 0.35     # Bulbs are round/square; stray noise is thin

# ============================================================
# Blown-Out Core / Overexposure Handling
# ============================================================

# Captures pure white overexposed bulb centers (V >= 235, S <= 95)
WHITE_CORE_MIN_VALUE = 235
WHITE_CORE_MAX_SATURATION = 95

# ============================================================
# Red-Light Thresholds
# ============================================================

RED_CANDIDATE_MIN_SATURATION = 80
RED_CANDIDATE_MIN_VALUE = 180

RED_MIN_MEAN_VALUE = 180
RED_MIN_PEAK_VALUE = 250
RED_MIN_BRIGHT_RATIO = 0.4
RED_BRIGHT_PIXEL_VALUE = 210

# ============================================================
# Yellow-Light Thresholds
# ============================================================

YELLOW_CANDIDATE_MIN_SATURATION = 100
YELLOW_CANDIDATE_MIN_VALUE = 200

YELLOW_MIN_MEAN_VALUE = 210
YELLOW_MIN_PEAK_VALUE = 250
YELLOW_MIN_BRIGHT_RATIO = 0.7
YELLOW_BRIGHT_PIXEL_VALUE = 220

YELLOW_WHITE_MIN_VALUE = 240
YELLOW_WHITE_MAX_SATURATION = 90

# ============================================================
# Green-Light Thresholds
# ============================================================

GREEN_CANDIDATE_MIN_SATURATION = 80
GREEN_CANDIDATE_MIN_VALUE = 180

GREEN_MIN_MEAN_VALUE = 180
GREEN_MIN_PEAK_VALUE = 255
GREEN_MIN_BRIGHT_RATIO = 0.5
GREEN_BRIGHT_PIXEL_VALUE = 210

# ============================================================
# Final Bulb-Box Requirements
# ============================================================

FINAL_MIN_SATURATION = 100
FINAL_MIN_VALUE = 170

YELLOW_FINAL_MIN_SATURATION = 85
YELLOW_FINAL_MIN_VALUE = 170

FINAL_BOX_PADDING = 0.10
YELLOW_BOX_PADDING = 0.12

# ============================================================
# HSV Color Ranges (Halo Friendly)
# ============================================================

RED1_LOWER = np.array([0, 50, 120])
RED1_UPPER = np.array([12, 255, 255])

RED2_LOWER = np.array([155, 50, 120])
RED2_UPPER = np.array([180, 255, 255])

YELLOW_LOWER = np.array([14, 80, 150])
YELLOW_UPPER = np.array([35, 255, 255])

GREEN_LOWER = np.array([38, 60, 85])
GREEN_UPPER = np.array([90, 255, 255])

# ============================================================
# Morphology Kernels (shared with the stop-sign detector)
# ============================================================

OPEN_KERNEL = np.ones((3, 3), np.uint8)
CLOSE_KERNEL = np.ones((5, 5), np.uint8)

YELLOW_DILATE_KERNEL = np.ones((3, 3), np.uint8)
HALO_DILATE_KERNEL = np.ones((5, 5), np.uint8)

# ============================================================
# Detection Hold State
# ============================================================

last_detection = None
last_analysis_time = 0.0


def clean_mask(mask):
    """Remove small noise and reconnect nearby regions (used by both detectors)."""
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, OPEN_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, CLOSE_KERNEL)
    return mask


def get_color_masks(hsv):
    """HSV color masks with white-core inheritance."""
    # 1. Base halo masks (glowing outer ring around overexposed center)
    red1 = cv2.inRange(hsv, RED1_LOWER, RED1_UPPER)
    red2 = cv2.inRange(hsv, RED2_LOWER, RED2_UPPER)
    red_halo = cv2.bitwise_or(red1, red2)

    yellow_halo = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)
    green_halo = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)

    # 2. Detect blown-out core pixels (V >= 235, S <= 95)
    white_core = cv2.inRange(
        hsv,
        np.array([0, 0, WHITE_CORE_MIN_VALUE]),
        np.array([180, WHITE_CORE_MAX_SATURATION, 255])
    )

    # 3. Associate white core pixels with whichever color halo touches them
    red_core = cv2.bitwise_and(white_core, cv2.dilate(red_halo, HALO_DILATE_KERNEL, iterations=2))
    yellow_core = cv2.bitwise_and(white_core, cv2.dilate(yellow_halo, HALO_DILATE_KERNEL, iterations=2))
    green_core = cv2.bitwise_and(white_core, cv2.dilate(green_halo, HALO_DILATE_KERNEL, iterations=2))

    red = cv2.bitwise_or(red_halo, red_core)
    yellow = cv2.bitwise_or(yellow_halo, yellow_core)
    green = cv2.bitwise_or(green_halo, green_core)

    return red, yellow, green


def find_bright_candidates(frame):
    frame_height, frame_width = frame.shape[:2]
    frame_area = frame_width * frame_height

    # Calculate ROI cropping pixels
    y_start = int(frame_height * TOP_CUTOFF)
    y_end = int(frame_height * BOTTOM_CUTOFF)
    x_start = int(frame_width * LEFT_CUTOFF)
    x_end = int(frame_width * RIGHT_CUTOFF)

    # Crop the ROI horizontally and vertically
    roi_frame = frame[y_start:y_end, x_start:x_end]

    blurred = cv2.GaussianBlur(roi_frame, (3, 3), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # ---------------- OVEREXPOSED WHITE CORES ----------------
    white_core = cv2.inRange(
        hsv,
        np.array([0, 0, WHITE_CORE_MIN_VALUE]),
        np.array([180, WHITE_CORE_MAX_SATURATION, 255])
    )

    # ---------------- RED CANDIDATES ----------------
    red1_candidate = cv2.inRange(
        hsv,
        np.array([0, RED_CANDIDATE_MIN_SATURATION, RED_CANDIDATE_MIN_VALUE]),
        np.array([15, 255, 255])
    )
    red2_candidate = cv2.inRange(
        hsv,
        np.array([155, RED_CANDIDATE_MIN_SATURATION, RED_CANDIDATE_MIN_VALUE]),
        np.array([180, 255, 255])
    )
    red_candidate = cv2.bitwise_or(red1_candidate, red2_candidate)

    # Attach white overexposed core to neighboring Red halo
    red_halo = cv2.dilate(red_candidate, HALO_DILATE_KERNEL, iterations=2)
    white_near_red = cv2.bitwise_and(white_core, red_halo)
    red_candidate = cv2.bitwise_or(red_candidate, white_near_red)

    # ---------------- GREEN CANDIDATES ----------------
    green_candidate = cv2.inRange(
        hsv,
        np.array([38, GREEN_CANDIDATE_MIN_SATURATION, GREEN_CANDIDATE_MIN_VALUE]),
        np.array([90, 255, 255])
    )

    # ---------------- YELLOW CANDIDATES ----------------
    yellow_candidate = cv2.inRange(
        hsv,
        np.array([14, YELLOW_CANDIDATE_MIN_SATURATION, YELLOW_CANDIDATE_MIN_VALUE]),
        np.array([35, 255, 255])
    )

    yellow_neighborhood = cv2.dilate(yellow_candidate, HALO_DILATE_KERNEL, iterations=2)
    white_near_yellow = cv2.bitwise_and(white_core, yellow_neighborhood)
    yellow_candidate = cv2.bitwise_or(yellow_candidate, white_near_yellow)

    # ---------------- COMBINE CANDIDATES ----------------
    candidate_mask = cv2.bitwise_or(red_candidate, yellow_candidate)
    candidate_mask = cv2.bitwise_or(candidate_mask, green_candidate)
    candidate_mask = clean_mask(candidate_mask)

    contours, _ = cv2.findContours(candidate_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area <= 0:
            continue

        area_ratio = area / frame_area
        if not (MIN_AREA_RATIO <= area_ratio <= MAX_AREA_RATIO):
            continue

        # Reject irregular / non-compact shapes (filters out stray noise lines/pixels)
        perimeter = cv2.arcLength(contour, True)
        if perimeter > 0:
            compactness = (4.0 * np.pi * area) / (perimeter * perimeter)
            if compactness < MIN_COMPACTNESS:
                continue

        x, y, box_width, box_height = cv2.boundingRect(contour)
        if box_width <= 0 or box_height <= 0:
            continue

        aspect_ratio = float(box_width) / box_height
        if aspect_ratio < 0.65 or aspect_ratio > 1.55:
            continue

        # Map ROI coordinates back to original full frame coordinates
        candidates.append((x + x_start, y + y_start, box_width, box_height))

    return candidates


def validate_light_color(hsv, color_mask, color):
    values = hsv[:, :, 2][color_mask > 0]
    sats = hsv[:, :, 1][color_mask > 0]

    if values.size == 0:
        return False

    if color == "RED":
        min_mean = RED_MIN_MEAN_VALUE
        min_peak = RED_MIN_PEAK_VALUE
        min_bright_ratio = RED_MIN_BRIGHT_RATIO
        bright_pixel_value = RED_BRIGHT_PIXEL_VALUE
    elif color == "YELLOW":
        min_mean = YELLOW_MIN_MEAN_VALUE
        min_peak = YELLOW_MIN_PEAK_VALUE
        min_bright_ratio = YELLOW_MIN_BRIGHT_RATIO
        bright_pixel_value = YELLOW_BRIGHT_PIXEL_VALUE
    elif color == "GREEN":
        min_mean = GREEN_MIN_MEAN_VALUE
        min_peak = GREEN_MIN_PEAK_VALUE
        min_bright_ratio = GREEN_MIN_BRIGHT_RATIO
        bright_pixel_value = GREEN_BRIGHT_PIXEL_VALUE
    else:
        return False

    mean_value = float(np.mean(values))
    peak_value = float(np.percentile(values, 90))
    bright_ratio = float(np.mean(values >= bright_pixel_value))
    mean_sat = float(np.mean(sats)) if sats.size > 0 else 0.0

    # Reject unlit reflected glass: require high saturation ONLY if light isn't blown out
    if peak_value < 240 and sats.size > 0:
        if mean_sat < 80:
            return False

    return (
        mean_value >= min_mean
        and peak_value >= min_peak
        and bright_ratio >= min_bright_ratio
    )


def get_precise_bulb_box(hsv, original_box, color, winner_mask):
    """Refine the bulb box using the HSV ROI and winning mask from analyze_candidate_roi()."""
    x, y, box_width, box_height = original_box

    final_mask = winner_mask.copy()

    if color == "YELLOW":
        final_mask = cv2.dilate(final_mask, YELLOW_DILATE_KERNEL, iterations=1)
    elif color == "RED":
        final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, CLOSE_KERNEL)
    else:
        final_mask = clean_mask(final_mask)

    contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return original_box

    best_contour = None
    best_score = 0.0
    value_channel = hsv[:, :, 2]

    for contour in contours:
        area = cv2.contourArea(contour)
        if area <= 0:
            continue

        contour_mask = np.zeros(final_mask.shape, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, cv2.FILLED)

        values_v = value_channel[contour_mask > 0]
        if values_v.size == 0:
            continue

        score = area * float(np.mean(values_v))
        if score > best_score:
            best_score = score
            best_contour = contour

    if best_contour is None:
        return original_box

    bx, by, bw, bh = cv2.boundingRect(best_contour)
    padding = YELLOW_BOX_PADDING if color == "YELLOW" else FINAL_BOX_PADDING

    pad_x = int(bw * padding)
    pad_y = int(bh * padding)

    new_x1 = max(0, bx - pad_x)
    new_y1 = max(0, by - pad_y)
    new_x2 = min(box_width, bx + bw + pad_x)
    new_y2 = min(box_height, by + bh + pad_y)

    return (x + new_x1, y + new_y1, new_x2 - new_x1, new_y2 - new_y1)


def analyze_candidate_roi(frame, box):
    x, y, box_width, box_height = box
    roi = frame[y:y + box_height, x:x + box_width]

    if roi is None or roi.size == 0:
        return None

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    red_mask, yellow_mask, green_mask = get_color_masks(hsv)

    masks = {"RED": red_mask, "YELLOW": yellow_mask, "GREEN": green_mask}

    # Calculate both pixel count AND brightness power for each color
    color_scores = {}
    for color, mask in masks.items():
        count = cv2.countNonZero(mask)
        if count > 0:
            vals = hsv[:, :, 2][mask > 0]
            avg_v = float(np.mean(vals))
            power_score = count * (avg_v / 255.0) ** 2
        else:
            power_score = 0.0
        color_scores[color] = (count, power_score)

    # Sort colors based on intensity power score rather than raw count
    sorted_colors = sorted(color_scores.items(), key=lambda item: item[1][1], reverse=True)

    winner_color = sorted_colors[0][0]
    winner_count, winner_power_score = sorted_colors[0][1]
    second_count, second_power_score = sorted_colors[1][1]

    if winner_count <= 0 or winner_power_score <= 0:
        return None

    roi_area = max(box_width * box_height, 1)
    winner_ratio = winner_count / float(roi_area)

    if winner_ratio < MIN_COLOR_RATIO:
        return None

    # Compare power scores between candidate colors
    if second_power_score > 0 and winner_power_score < second_power_score * WINNER_RATIO:
        return None

    winner_mask = masks[winner_color]
    if not validate_light_color(hsv, winner_mask, winner_color):
        return None

    values = hsv[:, :, 2][winner_mask > 0]
    if values.size > 0:
        strong_threshold = np.percentile(values, 85)
        strongest = values[values >= strong_threshold]
        mean_strong = float(np.mean(strongest))
        peak = float(np.percentile(values, 95))
        winner_power = 0.65 * mean_strong + 0.35 * peak
    else:
        winner_power = 0.0

    glow_score = winner_power

    precise_box = get_precise_bulb_box(hsv, box, winner_color, winner_mask)
    px, py, pw, ph = precise_box

    parent_area = box_width * box_height
    core_area = pw * ph

    if core_area <= 0:
        return None

    if parent_area / float(core_area) > MAX_PARENT_TO_CORE_RATIO:
        return None

    return {
        "object": f"TRAFFIC_LIGHT_{winner_color}",
        "color": winner_color,
        "box": precise_box,
        "center": (px + pw // 2, py + ph // 2),
        "power": winner_power,
        "glow_score": glow_score
    }


def detect_traffic_light(frame):
    """Main traffic-light detector. Returns a dict with "color" (RED/YELLOW/GREEN) or None."""
    global last_detection
    global last_analysis_time

    current_time = time.time()

    if last_detection is not None and current_time - last_analysis_time < HOLD_TIME:
        return last_detection

    candidates = find_bright_candidates(frame)
    valid_lights = []

    for box in candidates:
        result = analyze_candidate_roi(frame, box)
        if result is not None:
            valid_lights.append(result)

    if not valid_lights:
        last_detection = None
        last_analysis_time = current_time
        return None

    best_light = max(valid_lights, key=lambda light: light.get("glow_score", light["power"]))

    last_detection = best_light
    last_analysis_time = current_time

    return best_light

############################# end of RYANS code for traffic light ###################################


############################# Stop Sign detection from Ryan ###################################
# Red HSV ranges. These used to share the names RED1_LOWER etc. with the traffic-light
# detector, so one silently overwrote the other. The stop-sign ones now start with STOP_.

STOP_RED1_LOWER = np.array([0, 100, 70])
STOP_RED1_UPPER = np.array([22, 255, 255])

STOP_RED2_LOWER = np.array([170, 120, 70])
STOP_RED2_UPPER = np.array([180, 255, 255])

# Basic stop sign requirements
STOP_MIN_AREA = 0.0012
STOP_MAX_AREA = 0.15

STOP_MIN_ASPECT = 0.70
STOP_MAX_ASPECT = 1.40

MIN_EXTENT = 0.40

# Octagon requirements
IDEAL_OCTAGON_ANGLE = 135.0
MAX_ANGLE_ERROR = 24.3

NORMAL_MAX_SIDE_LENGTH_RATIO = 2.00
FAR_MAX_SIDE_LENGTH_RATIO = 2.30

NORMAL_MIN_SIDE_PIXELS = 6.0
NORMAL_MIN_OCTAGON_SCORE = 0.55
NORMAL_MIN_SHAPE_MATCH = 0.94

FAR_OBJECT_AREA_RATIO = 0.004
FAR_MIN_SIDE_PIXELS = 1.3
FAR_MIN_OCTAGON_SCORE = 0.52
FAR_MIN_SHAPE_MATCH = 0.86

# Circle rejection
MAX_CIRCLE_FILL = 0.95

# Red-light rejection
RED_LIGHT_MEAN_VALUE = 190
RED_LIGHT_PEAK_VALUE = 228
RED_LIGHT_BRIGHT_RATIO = 0.18
RED_LIGHT_BRIGHT_PIXEL = 215

RED_LIGHT_VERY_BRIGHT_PIXEL = 242
RED_LIGHT_VERY_BRIGHT_RATIO = 0.08

RED_WHITE_MIN_VALUE = 235
RED_WHITE_MAX_SATURATION = 105
RED_MIN_WHITE_CORE_RATIO = 0.008

RED_CENTER_INNER_FRACTION = 0.55
RED_CENTER_MIN_VALUE_DIFF = 22.0
RED_CENTER_MIN_RATIO = 1.10
RED_CENTER_MIN_INNER_VALUE = 205.0

RED_LIGHT_STRONG_HOT_RATIO = 0.14
RED_LIGHT_STRONG_WHITE_CORE_RATIO = 0.018

RED_WHITE_NEIGHBOR_KERNEL = np.ones((7, 7), np.uint8)

# Octagon outline search: epsilon (as a fraction of the hull perimeter) for approxPolyDP
OCTAGON_EPSILON_RATIOS = (
    0.010, 0.011, 0.012, 0.013, 0.014, 0.015, 0.016, 0.017, 0.018, 0.019, 0.020,
    0.021, 0.022, 0.023, 0.024, 0.025, 0.026, 0.027, 0.028, 0.029, 0.030,
    0.032, 0.034, 0.036, 0.038, 0.040, 0.042, 0.045,
)


def get_red_mask(hsv):
    """Create a cleaned mask containing red pixels."""
    red1 = cv2.inRange(hsv, STOP_RED1_LOWER, STOP_RED1_UPPER)
    red2 = cv2.inRange(hsv, STOP_RED2_LOWER, STOP_RED2_UPPER)
    return clean_mask(cv2.bitwise_or(red1, red2))


def looks_like_red_light(hsv, red_mask):
    """Reject red regions that behave like an illuminated red bulb."""
    if hsv is None or hsv.size == 0:
        return False

    roi_area = hsv.shape[0] * hsv.shape[1]
    if roi_area <= 0:
        return False

    red_values = hsv[:, :, 2][red_mask > 0]
    if red_values.size == 0:
        return False

    mean_value = float(np.mean(red_values))
    peak_value = float(np.percentile(red_values, 90))
    bright_ratio = float(np.mean(red_values >= RED_LIGHT_BRIGHT_PIXEL))
    very_bright_ratio = float(np.mean(red_values >= RED_LIGHT_VERY_BRIGHT_PIXEL))

    # White-hot core next to red pixels
    white_core = cv2.inRange(
        hsv,
        np.array([0, 0, RED_WHITE_MIN_VALUE]),
        np.array([180, RED_WHITE_MAX_SATURATION, 255])
    )
    red_neighborhood = cv2.dilate(red_mask, RED_WHITE_NEIGHBOR_KERNEL, iterations=1)
    white_near_red = cv2.bitwise_and(white_core, red_neighborhood)
    white_core_ratio = cv2.countNonZero(white_near_red) / float(roi_area)

    # Center glow: the inner ellipse is brighter than the rest of the red region
    h, w = hsv.shape[:2]
    yy, xx = np.ogrid[:h, :w]
    cx = (w - 1) / 2.0
    cy = (h - 1) / 2.0
    rx = max(w * RED_CENTER_INNER_FRACTION / 2.0, 1.0)
    ry = max(h * RED_CENTER_INNER_FRACTION / 2.0, 1.0)
    center_region = ((xx - cx) / rx) ** 2 + ((yy - cy) / ry) ** 2 <= 1.0

    red_pixels = red_mask > 0
    inner_pixels = red_pixels & center_region
    outer_pixels = red_pixels & (~center_region)
    value_channel = hsv[:, :, 2]

    inner_mean = float(np.mean(value_channel[inner_pixels])) if np.count_nonzero(inner_pixels) > 0 else 0.0
    outer_mean = float(np.mean(value_channel[outer_pixels])) if np.count_nonzero(outer_pixels) > 0 else mean_value

    center_diff = inner_mean - outer_mean
    center_ratio = inner_mean / outer_mean if outer_mean > 0 else 1.0

    center_glow = (
        inner_mean >= RED_CENTER_MIN_INNER_VALUE
        and center_diff >= RED_CENTER_MIN_VALUE_DIFF
        and center_ratio >= RED_CENTER_MIN_RATIO
    )
    normal_glow = (
        mean_value >= RED_LIGHT_MEAN_VALUE
        and peak_value >= RED_LIGHT_PEAK_VALUE
        and bright_ratio >= RED_LIGHT_BRIGHT_RATIO
    )
    blown_out_glow = (
        peak_value >= RED_LIGHT_PEAK_VALUE
        and (very_bright_ratio >= RED_LIGHT_VERY_BRIGHT_RATIO
             or white_core_ratio >= RED_MIN_WHITE_CORE_RATIO)
    )
    strong_hot_source = (
        very_bright_ratio >= RED_LIGHT_STRONG_HOT_RATIO
        or white_core_ratio >= RED_LIGHT_STRONG_WHITE_CORE_RATIO
    )

    return normal_glow or blown_out_glow or center_glow or strong_hot_source


def get_angle(p1, p2, p3):
    """Calculate the interior angle at p2."""
    v1 = np.array([p1[0] - p2[0], p1[1] - p2[1]], dtype=np.float32)
    v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]], dtype=np.float32)

    norm1 = np.linalg.norm(v1)
    norm2 = np.linalg.norm(v2)

    if norm1 <= 0 or norm2 <= 0:
        return 0.0

    cosine = np.clip(np.dot(v1, v2) / (norm1 * norm2), -1.0, 1.0)
    return math.degrees(math.acos(cosine))


def score_octagon(approx, min_side_pixels, max_side_length_ratio):
    """Score the geometry of an 8-sided polygon. Returns (score, geometry stats)."""
    if len(approx) != 8:
        return 0.0, None

    if not cv2.isContourConvex(approx):
        return 0.0, None

    points = [tuple(point[0]) for point in approx]

    angles = [get_angle(points[(i - 1) % 8], points[i], points[(i + 1) % 8]) for i in range(8)]
    angle_errors = [abs(angle - IDEAL_OCTAGON_ANGLE) for angle in angles]

    mean_angle_error = float(np.mean(angle_errors))
    max_angle_error = float(np.max(angle_errors))
    angle_std = float(np.std(angles))

    side_lengths = [
        float(np.linalg.norm(np.array(points[(i + 1) % 8], dtype=np.float32)
                             - np.array(points[i], dtype=np.float32)))
        for i in range(8)
    ]

    min_side = min(side_lengths)
    max_side = max(side_lengths)

    if min_side <= 0:
        return 0.0, None

    side_ratio = max_side / min_side
    mean_side = float(np.mean(side_lengths))
    side_std = float(np.std(side_lengths))
    side_cv = side_std / mean_side if mean_side > 0 else 1.0

    # Hard geometry rejection
    if mean_angle_error > MAX_ANGLE_ERROR:
        return 0.0, None
    if min_side < min_side_pixels:
        return 0.0, None
    if side_ratio > max_side_length_ratio:
        return 0.0, None

    # Geometry scoring
    mean_angle_score = max(0.0, 1.0 - mean_angle_error / MAX_ANGLE_ERROR)
    consistency_score = max(0.0, 1.0 - angle_std / 35.0)
    worst_angle_score = max(0.0, 1.0 - max_angle_error / 50.0)
    side_score = max(0.0, 1.0 - (side_ratio - 1.0) / (max_side_length_ratio - 1.0))
    side_consistency_score = max(0.0, 1.0 - side_cv / 0.45)

    octagon_score = (
        0.45 * mean_angle_score
        + 0.20 * consistency_score
        + 0.10 * worst_angle_score
        + 0.15 * side_score
        + 0.10 * side_consistency_score
    )

    geometry = {
        "mean_angle_error": mean_angle_error,
        "max_angle_error": max_angle_error,
        "angle_std": angle_std,
        "side_ratio": side_ratio
    }

    return octagon_score, geometry


def find_best_octagon(contour, min_side_pixels, max_side_length_ratio):
    """Find the strongest valid 8-vertex approximation."""
    hull = cv2.convexHull(contour)
    perimeter = cv2.arcLength(hull, True)

    if perimeter <= 0:
        return None, 0.0, None

    best_approx = None
    best_score = 0.0
    best_geometry = None

    for epsilon_ratio in OCTAGON_EPSILON_RATIOS:
        approx = cv2.approxPolyDP(hull, epsilon_ratio * perimeter, True)

        if len(approx) != 8:
            continue

        score, geometry = score_octagon(approx, min_side_pixels, max_side_length_ratio)

        if score > best_score:
            best_score = score
            best_approx = approx
            best_geometry = geometry

    return best_approx, best_score, best_geometry


def detect_stop_sign(frame):
    """Detect the strongest valid stop sign in the right-side ROI. Returns a dict or None."""
    height, width = frame.shape[:2]
    frame_area = width * height

    x_start = int(width * 0.50)      # process only the right half of the image
    y_start = int(height * 0.05)     # original vertical ROI
    y_end = int(height * 0.80)

    roi_frame = frame[y_start:y_end, x_start:]
    if roi_frame.size == 0:
        return None

    blurred = cv2.GaussianBlur(roi_frame, (3, 3), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    red_mask = get_red_mask(hsv)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_detection = None
    best_score = 0.0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area <= 0:
            continue

        area_ratio = area / frame_area
        if not (STOP_MIN_AREA <= area_ratio <= STOP_MAX_AREA):
            continue

        x, y, box_w, box_h = cv2.boundingRect(contour)
        if box_w <= 0 or box_h <= 0:
            continue

        aspect_ratio = box_w / float(box_h)
        if not (STOP_MIN_ASPECT <= aspect_ratio <= STOP_MAX_ASPECT):
            continue

        box_area = box_w * box_h
        if box_area <= 0:
            continue

        extent = area / box_area
        if extent < MIN_EXTENT:
            continue

        # Circle rejection
        _, radius = cv2.minEnclosingCircle(contour)
        if radius <= 0:
            continue

        circle_area = np.pi * radius * radius
        if circle_area <= 0:
            continue

        if area / circle_area > MAX_CIRCLE_FILL:
            continue

        # Reject glowing red traffic lights
        if looks_like_red_light(hsv[y:y + box_h, x:x + box_w], red_mask[y:y + box_h, x:x + box_w]):
            continue

        # Far / normal thresholds
        if area_ratio < FAR_OBJECT_AREA_RATIO:
            min_side_pixels = FAR_MIN_SIDE_PIXELS
            min_octagon_score = FAR_MIN_OCTAGON_SCORE
            min_shape_match = FAR_MIN_SHAPE_MATCH
            max_side_length_ratio = FAR_MAX_SIDE_LENGTH_RATIO
        else:
            min_side_pixels = NORMAL_MIN_SIDE_PIXELS
            min_octagon_score = NORMAL_MIN_OCTAGON_SCORE
            min_shape_match = NORMAL_MIN_SHAPE_MATCH
            max_side_length_ratio = NORMAL_MAX_SIDE_LENGTH_RATIO

        # Find octagon
        octagon, octagon_score, geometry = find_best_octagon(contour, min_side_pixels, max_side_length_ratio)

        if octagon is None or octagon_score < min_octagon_score:
            continue

        octagon_area = cv2.contourArea(octagon)
        if octagon_area / area < min_shape_match:
            continue

        # Final confidence
        square_score = max(0.0, 1.0 - abs(1.0 - aspect_ratio))
        extent_score = min(1.0, extent)

        final_score = float(np.clip(
            0.82 * octagon_score + 0.12 * square_score + 0.06 * extent_score,
            0.0, 1.0
        ))

        final_x, final_y, final_w, final_h = cv2.boundingRect(octagon)

        if final_score > best_score:
            best_score = final_score

            # Translate ROI coordinates back to full-frame coordinates.
            full_x = final_x + x_start
            full_y = final_y + y_start

            best_detection = {
                "object": "STOP_SIGN",
                "box": (full_x, full_y, final_w, final_h),
                "center": (full_x + final_w // 2, full_y + final_h // 2),
                "confidence": final_score
            }

    return best_detection

#################################### end of ryan stop sign Detection code ###############################################


################################### Ryan vision code  ##############################################################
# Ryan's vision code: runs both detectors on one frame and arbitrates between them.
# (His separate stop_sign_detector / traffic_light_detector imports are not needed here: both
# detectors are in this file. His standalone main() is now vision_main(), menu option 11.)

picam2 = None


def make_picamera2_config(picam, width=640, height=480):
    """Ryan's camera configuration.

    IMX219: the 640x480 sensor mode is a cropped, zoomed-in section of the sensor. 1640x1232 uses the
    FULL 3280x2464 sensor field of view; it is scaled down to width x height so the detectors keep
    using the same coordinates and ROIs. hflip + vflip = 180-degree rotation for an upside-down camera.
    "RGB888" arrives as B,G,R bytes, which is the order OpenCV expects.
    """
    main_stream = {"size": (width, height), "format": "RGB888"}
    if not (CAMERA_FULL_FOV or CAMERA_FLIP):
        return picam.create_video_configuration(main=main_stream)
    kwargs = {}
    if CAMERA_FULL_FOV:
        kwargs["sensor"] = {"output_size": (1640, 1232), "bit_depth": 10}
    if CAMERA_FLIP:
        from libcamera import Transform
        kwargs["transform"] = Transform(hflip=1, vflip=1)
    return picam.create_video_configuration(main=main_stream, **kwargs)


def setup_camera():
    """Start the Pi camera once (stand-alone use: vision_main, read_vision)."""
    global picam2

    if picam2 is not None:
        return

    from picamera2 import Picamera2
    picam2 = Picamera2()
    picam2.configure(make_picamera2_config(picam2, CAMERA_WIDTH, CAMERA_HEIGHT))
    picam2.start()


def close_camera():
    """Release the camera that setup_camera() opened."""
    global picam2
    if picam2 is not None:
        try:
            picam2.stop()
            picam2.close()
        finally:
            picam2 = None


def boxes_represent_same_object(box1, box2):
    """Return True if two detection boxes likely refer to the same object."""

    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2

    left = max(x1, x2)
    top = max(y1, y2)
    right = min(x1 + w1, x2 + w2)
    bottom = min(y1 + h1, y2 + h2)

    if right <= left or bottom <= top:
        return False

    overlap_area = (right - left) * (bottom - top)

    area1 = w1 * h1
    area2 = w2 * h2

    if area1 <= 0 or area2 <= 0:
        return False

    smaller_area = min(area1, area2)
    overlap_ratio = overlap_area / smaller_area

    center1_x = x1 + w1 / 2
    center1_y = y1 + h1 / 2

    center2_x = x2 + w2 / 2
    center2_y = y2 + h2 / 2

    center1_inside_box2 = (
        x2 <= center1_x <= x2 + w2
        and y2 <= center1_y <= y2 + h2
    )

    center2_inside_box1 = (
        x1 <= center2_x <= x1 + w1
        and y1 <= center2_y <= y1 + h1
    )

    return (
        overlap_ratio >= 0.50
        or center1_inside_box2
        or center2_inside_box1
    )


def get_horizontal_position(center_x, frame_width):
    """Convert X coordinate into LEFT / CENTER / RIGHT."""

    if center_x is None:
        return None

    left_boundary = frame_width / 3
    right_boundary = 2 * frame_width / 3

    if center_x < left_boundary:
        return "LEFT"

    if center_x > right_boundary:
        return "RIGHT"

    return "CENTER"


def get_vision_data(frame):
    """Run all vision detectors."""

    _, frame_width = frame.shape[:2]

    stop_sign_data = detect_stop_sign(frame)
    traffic_light_data = detect_traffic_light(frame)

    # ---------------------------------------------------------
    # SAME-OBJECT ARBITRATION
    # A red stop sign must not also count as a RED traffic light.
    # ---------------------------------------------------------

    if stop_sign_data is not None and traffic_light_data is not None:

        if boxes_represent_same_object(
            stop_sign_data["box"],
            traffic_light_data["box"]
        ):

            if traffic_light_data["color"] == "RED":
                traffic_light_data = None

    # ---------------------------------------------------------
    # BUILD VISION DATA
    # ---------------------------------------------------------

    vision_data = {
        "stop_sign": False,
        "red_light": False,
        "yellow_light": False,
        "green_light": False,

        "stop_sign_center": None,
        "traffic_light_center": None,

        "stop_sign_position": None,
        "traffic_light_position": None,

        "stop_sign_data": stop_sign_data,
        "traffic_light_data": traffic_light_data
    }

    # ---------------------------------------------------------
    # STOP SIGN
    # ---------------------------------------------------------

    if stop_sign_data is not None:

        center_x, center_y = stop_sign_data["center"]

        vision_data["stop_sign"] = True

        vision_data["stop_sign_center"] = (
            center_x,
            center_y
        )

        vision_data["stop_sign_position"] = (
            get_horizontal_position(
                center_x,
                frame_width
            )
        )

    # ---------------------------------------------------------
    # TRAFFIC LIGHT
    # ---------------------------------------------------------

    if traffic_light_data is not None:

        color = traffic_light_data["color"]

        center_x, center_y = traffic_light_data["center"]

        vision_data["traffic_light_center"] = (
            center_x,
            center_y
        )

        vision_data["traffic_light_position"] = (
            get_horizontal_position(
                center_x,
                frame_width
            )
        )

        if color == "RED":
            vision_data["red_light"] = True

        elif color == "YELLOW":
            vision_data["yellow_light"] = True

        elif color == "GREEN":
            vision_data["green_light"] = True

    return vision_data


def read_vision():
    """Capture one frame from the stand-alone camera and return all vision results."""

    setup_camera()

    frame = picam2.capture_array()

    return get_vision_data(frame)


def print_vision_data(vision_data):
    """Print detected objects and their locations."""

    if vision_data["stop_sign"]:

        print(
            f'STOP_SIGN = 1 | '
            f'Center = {vision_data["stop_sign_center"]} | '
            f'Position = {vision_data["stop_sign_position"]}'
        )

    if vision_data["red_light"]:

        print(
            f'RED_LIGHT = 1 | '
            f'Center = {vision_data["traffic_light_center"]} | '
            f'Position = {vision_data["traffic_light_position"]}'
        )

    elif vision_data["yellow_light"]:

        print(
            f'YELLOW_LIGHT = 1 | '
            f'Center = {vision_data["traffic_light_center"]} | '
            f'Position = {vision_data["traffic_light_position"]}'
        )

    elif vision_data["green_light"]:

        print(
            f'GREEN_LIGHT = 1 | '
            f'Center = {vision_data["traffic_light_center"]} | '
            f'Position = {vision_data["traffic_light_position"]}'
        )


def vision_main():
    """Ryan's stand-alone loop: run vision continuously and print detections (Ctrl+C stops)."""

    setup_camera()

    try:
        while True:
            frame = picam2.capture_array()
            vision_data = get_vision_data(frame)
            print_vision_data(vision_data)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        close_camera()

################################### end Ryan vision code  ##############################################################


####################### TOF SENSOR ###################################
############ not putting in due to not testing
############ will add later

####################### TOF SENSOR END #########################


####################### POWER GUARD (INA219) ###################################
# power_guard.py, battery_support.py and imu_support.py sit next to this file. They come from
# the Motor Test bench; the only change is that mentions of the bench's menu commands and keys
# were taken out, since this program does not have them. The limits are the POWER_* settings at
# the top of this file. Navigation only calls check_power() once per control loop. A trip stops
# the motors (PWM 0, inputs low) before the fault reaches the run.
power = None                    # the running PowerGuard once start_power() has run


def start_power():
    """Start the INA219 guard and wait for its first reading. Off in Settings > Other > Power guard."""
    global power
    if not USE_POWER_GUARD:
        log("Power guard is off")
        return
    log("Checking power guard")
    import power_guard               # imported here so the program still runs with the guard off
    try:
        guard = power_guard.PowerGuard(POWER_OVERCURRENT_A, POWER_OVERCURRENT_HOLD,
                                       POWER_LOW_VOLTAGE_V, POWER_LOW_VOLTAGE_HOLD, clock=time)
    except Exception as exc:
        raise CourseFault(f"Power guard could not start: {exc} "
                          "(Settings > Other > Power guard: off runs without it)")
    try:
        sample = guard.wait_ready()
    except power_guard.PowerFault as exc:
        guard.close()
        cause = exc.__cause__ or "no reading within 1.5 s"
        raise CourseFault(f"Power guard could not read the INA219: {cause} "
                          "(Settings > Other > Power guard: off runs without it)")
    power = guard
    log(guard.describe())
    log(f"Battery {sample.input_voltage:.2f} V, {abs(sample.current):.2f} A")


def check_power():
    """Stop the run at once on overcurrent, a low battery or a failed INA219 reading."""
    if power is None:
        return
    try:
        power.check(motors)
    except Exception as exc:          # a PowerFault, or the guard itself failing
        motors.stop(brake=False)      # a trip has already done this; again is harmless
        raise CourseFault(f"{power.trip_reason or exc} Motors stopped.")


def close_power():
    global power
    if power is not None:
        summary = power.summary()
        if summary:
            log(summary)
        power.close()
    power = None

####################### POWER GUARD END #########################


################# camera ###################################
class CameraStream(threading.Thread):
    """Reads the camera continuously so latest() is always the newest BGR frame.

    Ribbon-cable Pi cameras need picamera2; plain cv2.VideoCapture(0) lets OpenCV pick
    GStreamer, which fails on them ("Failed to allocate required memory"). USB webcams
    use the V4L2 backend directly instead.
    """

    def __init__(self, index=None, width=None, height=None):
        super().__init__(daemon=True)
        index = CAMERA_INDEX if index is None else index
        width = CAMERA_WIDTH if width is None else width
        height = CAMERA_HEIGHT if height is None else height
        self.lock = threading.Lock()
        self.frame = None
        self.running = True
        self.picam = None
        self.capture = None
        self.backend = None
        order = {"auto": ("picamera2", "v4l2"), "picamera2": ("picamera2",), "v4l2": ("v4l2",)}
        for name in order[CAMERA_BACKEND]:
            try:
                if name == "picamera2":
                    self._open_picamera2(width, height)
                else:
                    self._open_v4l2(index, width, height)
                self.backend = name
                break
            except Exception as exc:
                print(f"Camera backend {name} failed: {exc}")

    def _open_picamera2(self, width, height):
        from picamera2 import Picamera2
        picam = Picamera2()
        try:
            try:
                picam.configure(make_picamera2_config(picam, width, height))
            except Exception as exc:                      # e.g. a sensor without the 1640x1232 mode
                print(f"Camera: full-view/flip configuration failed ({exc}); using the plain configuration")
                picam.configure(picam.create_video_configuration(main={"size": (width, height), "format": "RGB888"}))
            picam.start()
        except Exception:
            picam.close()
            raise
        self.picam = picam

    def _open_v4l2(self, index, width, height):
        capture = cv2.VideoCapture(index, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))   # webcams manage more fps in MJPG
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not capture.isOpened() or not capture.read()[0]:
            capture.release()
            raise RuntimeError(f"/dev/video{index} gave no frames")
        self.capture = capture

    def opened(self):
        return self.backend is not None

    def run(self):
        while self.running:
            try:
                if self.picam is not None:
                    frame = self.picam.capture_array()
                    ok = frame is not None
                else:
                    ok, frame = self.capture.read()
            except Exception:
                ok = False
            if ok:
                with self.lock:
                    self.frame = frame
            else:
                time.sleep(0.01)

    def latest(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        self.running = False
        if self.is_alive():
            self.join(timeout=1.0)
        if self.picam is not None:
            self.picam.stop()
            self.picam.close()
        if self.capture is not None:
            self.capture.release()


################# run log and timer ###################################
timer_start = None
run_events = []


def log(message):
    """Print a line with the time since the timer started, and keep it for the log file."""
    elapsed = 0.0 if timer_start is None else time.monotonic() - timer_start
    line = f"[{elapsed:7.2f}s] {message}"
    print(line)
    run_events.append(line)


def write_log():
    try:
        with open(LOG_FILE, "a") as handle:
            handle.write(f"---- run {time.strftime('%Y-%m-%d %H:%M:%S')} ----\n")
            handle.write("\n".join(run_events) + "\n")
    except OSError as exc:
        print(f"Could not write {LOG_FILE}: {exc}")


################# buttons ###################################
button_idle = {}     # level each button pin reads when it is NOT pressed


def setup_buttons():
    for pin in (start_button, reset_button):
        GPIO.setup(pin, GPIO.IN, pull_up_down=BUTTON_PULL)
    time.sleep(0.05)
    for pin in (start_button, reset_button):
        reads = [GPIO.input(pin) for _ in range(5)]
        button_idle[pin] = max(set(reads), key=reads.count)


def button_pressed(pin):
    if GPIO.input(pin) == button_idle[pin]:
        return False
    time.sleep(BUTTON_DEBOUNCE)
    return GPIO.input(pin) != button_idle[pin]


def test_buttons(seconds=8):
    """Type B at the directions prompt: prints the raw level of both buttons as they change."""
    print(f"Button test for {seconds} s. Idle levels: start={button_idle[start_button]} "
          f"reset={button_idle[reset_button]}. Press each button now.")
    end = time.monotonic() + seconds
    last = None
    while time.monotonic() < end:
        levels = (GPIO.input(start_button), GPIO.input(reset_button))
        if levels != last:
            print(f"start={levels[0]} reset={levels[1]}")
            last = levels
        time.sleep(0.02)
    print("No change when pressed? The button is not reaching the pin, or it is wired to 3.3 V "
          "(then set BUTTON_PULL = GPIO.PUD_DOWN).")


def flush_stdin():
    """Throw away keys typed while the robot was running."""
    try:
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except Exception:
        pass


DIRECTION_NAMES = {"L": "LEFT", "LEFT": "LEFT", "R": "RIGHT", "RIGHT": "RIGHT",
                   "S": "STRAIGHT", "STRAIGHT": "STRAIGHT", "F": "STRAIGHT"}


def parse_directions(text):
    """'R S L', 'rslr', 'right,straight,left' -> ['RIGHT', 'STRAIGHT', 'LEFT', ...]. Raises ValueError."""
    result = []
    for token in text.upper().replace(",", " ").split():
        if token in DIRECTION_NAMES:
            result.append(DIRECTION_NAMES[token])
        elif all(ch in "LRSF" for ch in token):          # e.g. "RSLR"
            result.extend(DIRECTION_NAMES[ch] for ch in token)
        else:
            raise ValueError(f"Cannot read {token!r}: use L, R or S for each intersection, e.g. R S L")
    if not result:
        raise ValueError("No directions entered: use L, R or S for each intersection, e.g. R S L")
    return result


def check_reset():
    if button_pressed(reset_button):
        raise ResetRequested("Reset button pressed")


################# navigation helpers ###################################
def normalize_directions(items):
    if not items:
        raise CourseFault("directions is empty")
    result = []
    for item in items:
        name = DIRECTION_NAMES.get(str(item).strip().upper())
        if name is None:
            raise CourseFault(f"Unknown direction {item!r}; use LEFT, RIGHT or STRAIGHT")
        result.append(name)
    return result


def side_line_trim(left, right, front=False):
    """Lane keeping from the LEFT and RIGHT IR sensors. Positive = slow the left side (steer left).

    The left sensor on a line pushes the robot right, the right sensor on a line pushes it left;
    both at once cancel. The front sensor on a line is the stop line, not a lane edge, so that
    gives no steering.
    """
    if not USE_SIDE_LINES or front:
        return 0.0
    return SIDE_LINE_TRIM * (int(right) - int(left))


def forward_targets(angle, rate, side_trim=0.0):
    """Four duties (FL, FR, BL, BR) for straight driving.

    Heading correction (USE_HEADING_HOLD) and lane correction (side_trim) only ever slow one
    side down. With USE_WHEEL_SPEEDS on, each wheel starts from its own WHEEL_SPEEDS value.
    """
    hold = heading_correction(angle, rate) if USE_HEADING_HOLD else 0.0
    trim = max(-MAX_HEADING_TRIM, min(MAX_HEADING_TRIM, hold + side_trim))
    slow_left = max(trim, 0.0)           # positive trim slows the left side
    slow_right = max(-trim, 0.0)         # negative trim slows the right side
    if USE_WHEEL_SPEEDS:
        fl, fr, bl, br = WHEEL_SPEEDS
        return (max(0.0, fl - slow_left), max(0.0, fr - slow_right),
                max(0.0, bl - slow_left), max(0.0, br - slow_right))
    scale = IMU_DRIVE_DUTY / max(LEFT_SPEED, RIGHT_SPEED)
    left = max(0.0, LEFT_SPEED * scale - slow_left)
    right = max(0.0, RIGHT_SPEED * scale - slow_right)
    return (left, right, left, right)


def turn_targets(duty):
    """Four duties (FL, FR, BL, BR) for spinning in place; `duty` = TURN_MAX_DUTY means full turn power.

    With USE_TURN_WHEEL_SPEEDS on, each wheel uses its own TURN_WHEEL_DUTIES value (scaled down
    together when an angle turn slows near its target). The pattern decides each wheel's direction.
    """
    if USE_TURN_WHEEL_SPEEDS:
        factor = duty / TURN_MAX_DUTY if TURN_MAX_DUTY > 0 else 0.0
        return tuple(wheel * factor for wheel in TURN_WHEEL_DUTIES)
    scale = duty / max(LEFT_SPEED, RIGHT_SPEED)
    return (LEFT_SPEED * scale, RIGHT_SPEED * scale) * 2


def drive_to_line(tracker, heading_target):
    """Steps 5 and 6. With IR sensors: drive until the front sensor sees the line.

    IMU-only mode has no IR sensors, so it drives LEG_TIME seconds instead.
    Returns a short name for the log.
    """
    if mode_uses_ir():
        drive_straight(tracker, heading_target, until_line=True)
        return "Front line"
    drive_straight(tracker, heading_target, seconds=LEG_TIME, lanes=False)
    return "Timed leg"


def drive_straight(tracker, heading_target, seconds=None, until_line=False, stop_at_end=True, lanes=True,
                   report_every=None, trace=None):
    """Forward drive with heading hold and lane steering. tracker=None drives without the IMU.

    seconds     : drive for this long and ignore lines (intersection approach and exit)
    until_line  : drive until the front sensor sees the line (steps 5 and 6; needs the IR sensors)
    stop_at_end : brake at the end; False keeps rolling into the next call
    lanes       : steer with the left and right lane sensors (off inside the intersection)
    report_every: log a status line this often, in seconds (default 2 s when looking for a line)
    trace       : optional list; (seconds, heading error) is appended every loop
    """
    if seconds is None and not until_line:
        raise ValueError("give seconds or until_line")
    if until_line and not mode_uses_ir():
        raise ValueError("until_line needs the IR sensors")
    motors.prepare(FORWARD)
    started = previous = time.monotonic()
    armed = False      # a front line only counts after the front sensor has been clear once
    confirm = 0
    lane_trim = 0.0    # latest lane correction, kept for LANE_HOLD_TIME so the robot really moves over
    lane_until = 0.0
    if report_every is None and until_line:
        report_every = 2.0
    next_note = started + (report_every or 0.0)
    shown_trim = 0.0   # lane correction last written to the log
    shown_at = 0.0
    while True:
        check_reset()
        check_power()
        now = time.monotonic()
        if tracker is not None:
            _, _, yaw, rate = tracker.get()
            error = yaw - heading_target
        else:
            error = rate = 0.0                   # no IMU: no heading hold
        if trace is not None:
            trace.append((now - started, error))
        if USE_HEADING_HOLD and abs(error) > MAX_HEADING_ERROR:
            raise CourseFault(f"Heading is {error:+.1f} deg off plan (limit {MAX_HEADING_ERROR:.0f}); "
                              "check IMU orientation and tuning")
        if mode_uses_ir():
            left, front, right = read_lines()
        else:
            left = front = right = False         # no IR sensors in this mode
        if until_line:
            if front:
                if armed:
                    confirm += 1
            else:
                armed = True
                confirm = 0
            if confirm >= LINE_CONFIRM_READS:
                break
            if now - started >= LEG_TIMEOUT:
                raise CourseFault(f"No front line found within {LEG_TIMEOUT:.0f} s")
        elif now - started >= seconds:
            break
        instant = side_line_trim(left, right, front) if lanes else 0.0
        if front:
            lane_trim = 0.0                      # stop line under the front sensor: no steering
        elif instant != 0.0:
            lane_trim, lane_until = instant, now + LANE_HOLD_TIME
        elif now >= lane_until:
            lane_trim = 0.0
        targets = forward_targets(error, rate, lane_trim)
        motors.slew(targets, RAMP_RATE, now - previous)
        previous = now
        if lane_trim == 0.0:
            shown_trim = 0.0
        elif lane_trim != shown_trim and now - shown_at >= 0.5:
            log(f"Lane correction: line on the {'RIGHT' if lane_trim > 0 else 'LEFT'}, steering "
                f"{'LEFT' if lane_trim > 0 else 'RIGHT'} (left/right duty {targets[0]:.0f}/{targets[1]:.0f})")
            shown_trim, shown_at = lane_trim, now
        if report_every and now >= next_note:
            log(f"Driving {now - started:.1f} s: on a line L/F/R = {int(left)}{int(front)}{int(right)} "
                f"(1 = line) | heading error {error:+.1f} deg | left/right duty {targets[0]:.0f}/{targets[1]:.0f}")
            next_note = now + report_every
        time.sleep(LOOP_DT)
    if until_line or stop_at_end:
        motors.stop()


def turn(tracker, direction, heading_target):
    """Gyro-measured 90 degree turn (step 7). Returns the new planned heading."""
    if TURN_MODE == "timed" or tracker is None:         # no IMU means a timed turn
        return turn_timed(tracker, direction, heading_target)
    sign = 1 if direction == "LEFT" else -1
    pattern = TURN_LEFT if sign == 1 else TURN_RIGHT
    log(f"Turning {direction}")
    motors.prepare(pattern)
    start = previous = progress_time = time.monotonic()
    checkpoint = 0.0
    while True:
        check_reset()
        check_power()
        now = time.monotonic()
        _, _, yaw, rate = tracker.get()
        progress = sign * (yaw - heading_target)      # degrees turned so far, in the commanded direction
        if progress < -3:
            raise CourseFault("Turn went the wrong measured direction; check IMU_YAW_SIGN and motor wiring")
        rate_toward = sign * rate
        if progress + max(0.0, rate_toward) * TURN_STOP_LEAD >= TURN_ANGLE - TURN_TOLERANCE:
            break
        if now - start >= TURN_TIMEOUT:
            raise CourseFault("Turn timed out")
        if progress >= checkpoint + 1.0:
            checkpoint, progress_time = progress, now
        elif now - progress_time >= TURN_STALL_TIME:
            raise CourseFault("No measurable turn progress; movement stopped")
        duty = max(TURN_MIN_DUTY, min(TURN_MAX_DUTY, (TURN_ANGLE - progress) * TURN_SLOW_GAIN))
        targets = turn_targets(duty)
        motors.slew(targets, RAMP_RATE, now - previous)
        previous = now
        time.sleep(LOOP_DT)
    motors.stop()
    settle = time.monotonic() + 0.25                   # let the robot finish coasting
    while time.monotonic() < settle:
        check_reset()
        check_power()
        tracker.get()
        time.sleep(LOOP_DT)
    _, _, yaw, _ = tracker.get()
    log(f"Turn done: {sign * (yaw - heading_target):.1f} deg measured (target {TURN_ANGLE:.0f})")
    return heading_target + sign * TURN_ANGLE


def turn_timed(tracker, direction, heading_target):
    """Timed turn: spin at TURN_MAX_DUTY for TURN_DURATION_LEFT/RIGHT seconds.

    With the IMU the gyro only measures it: the measured heading becomes the new plan, so an
    imperfect turn does not trip the heading check; adjust the duration until the log says
    about 90 degrees. Without the IMU (tracker=None) nothing is measured.
    """
    sign = 1 if direction == "LEFT" else -1
    pattern = TURN_LEFT if sign == 1 else TURN_RIGHT
    seconds = TURN_DURATION_LEFT if sign == 1 else TURN_DURATION_RIGHT
    log(f"Turning {direction} (timed, {seconds:.2f} s)")
    yaw_before = tracker.get()[2] if tracker is not None else 0.0
    motors.prepare(pattern)
    targets = turn_targets(TURN_MAX_DUTY)
    start = previous = time.monotonic()
    while time.monotonic() - start < seconds:
        check_reset()
        check_power()
        now = time.monotonic()
        if tracker is not None:
            tracker.get()
        motors.slew(targets, 1000.0, max(now - previous, LOOP_DT))   # no ramp: a short turn needs full duty at once
        previous = now
        time.sleep(LOOP_DT)
    motors.stop()
    settle = time.monotonic() + 0.25
    while time.monotonic() < settle:
        check_reset()
        check_power()
        if tracker is not None:
            tracker.get()
        time.sleep(LOOP_DT)
    if tracker is None:
        log(f"Turn done (timed {seconds:.2f} s; no IMU, so the angle is not measured)")
        return heading_target + sign * TURN_ANGLE
    yaw = tracker.get()[2]
    turned = sign * (yaw - yaw_before)
    log(f"Turn done: {turned:.1f} deg measured (target {TURN_ANGLE:.0f})")
    if turned < -3:
        raise CourseFault("Turn went the wrong measured direction; check IMU_YAW_SIGN and motor wiring")
    return yaw


def wait_for_green(camera):
    """Steps 3 and 9: hold still until the light is green, or until no light is in view.

    Goes anyway after LIGHT_WAIT_TIMEOUT seconds (0 = wait forever), so a light that
    reads red the whole time cannot stall the run. USE_TRAFFIC_LIGHT off skips the check.
    """
    if camera is None:
        log("No camera in this mode: traffic light not checked")
        return
    if not USE_TRAFFIC_LIGHT:
        log("Traffic light check is off: going")
        return
    started = time.monotonic()
    next_note = started + 2.0
    no_light = 0
    last_state = None
    while True:
        check_reset()
        now = time.monotonic()
        if LIGHT_WAIT_TIMEOUT > 0 and now - started >= LIGHT_WAIT_TIMEOUT:
            log(f"Light is still {last_state or 'unreadable'} after {LIGHT_WAIT_TIMEOUT:g} s: going anyway")
            return
        frame = camera.latest()
        if frame is None:
            time.sleep(0.05)
            continue
        light = get_vision_data(frame)["traffic_light_data"]   # a stop sign never counts as a red light
        state = light["color"] if light else "NONE"
        if state != last_state:
            log(f"Traffic light: {state}")
            last_state = state
            next_note = now + 2.0
        elif state != "NONE" and now >= next_note:
            log(f"Still waiting: light is {state} ({now - started:.0f} s)")
            next_note = now + 2.0
        if state == "GREEN":
            return
        if state == "NONE":
            no_light += 1
            if no_light >= NO_LIGHT_FRAMES:
                log("No traffic light in view: going")
                return
        else:
            no_light = 0
        time.sleep(0.05)


def confirm_stop_sign(camera):
    """Look for the stop sign at the final line. True = seen, False = not seen, None = no camera."""
    if camera is None:
        return None
    end = time.monotonic() + STOP_SIGN_CONFIRM_TIME
    while time.monotonic() < end:
        check_reset()
        frame = camera.latest()
        if frame is not None:
            if get_vision_data(frame)["stop_sign"]:
                return True
        time.sleep(0.05)
    return False


def start_imu(parts):
    """Connect to the IMU, calibrate it (robot must be still) and start the angle tracker."""
    log("Checking IMU: keep the robot still")
    try:
        parts["imu"] = MPU6050(MPU6050_BUS, MPU6050_ADDRESS)
        bias = parts["imu"].calibrate()
    except OSError as exc:
        raise CourseFault(f"IMU not responding: {exc}")
    log(f"Gyro bias X/Y/Z: {bias[0]:+.3f}/{bias[1]:+.3f}/{bias[2]:+.3f} deg/s")
    parts["tracker"] = AngleTracker(parts["imu"])
    parts["tracker"].start()
    time.sleep(0.5)                       # let the filter settle
    parts["tracker"].zero_yaw()           # the start pose is heading 0
    parts["tracker"].get()                # raises if the IMU is not producing data


def check_systems(parts):
    """Step 2: check what this run mode uses (IR sensors, IMU, camera) and the motors."""
    log(f"Run mode: {RUN_MODE_LABELS[RUN_MODE]}")
    if mode_uses_ir():
        log("Checking IR sensors")
        problem = ir_pin_problem()
        if problem:
            raise CourseFault(problem + " (Settings > Lanes and lines)")
        log(f"IR pins front/left/right = {IR_FRONT_PIN}/{IR_LEFT_PIN}/{IR_RIGHT_PIN}")
        left, front, right = read_sensors()
        log(f"IR L/F/R = {left}/{front}/{right} (a line reads {LINE_DETECTED})")
    else:
        log("IR sensors are not used in this mode")

    log("Checking motors")
    if not motors.ready:
        raise CourseFault("Motor driver is not initialised")
    start_power()                         # before the twitch check, the first powered move
    if MOTOR_TWITCH_CHECK:
        motors.twitch()

    if mode_uses_imu():
        start_imu(parts)
    else:
        log("IMU is not used in this mode")

    if mode_uses_camera():
        log("Checking camera")
        parts["camera"] = CameraStream()
        problem = None
        if not parts["camera"].opened():
            problem = ("Camera did not open. Test it outside this program with: rpicam-hello --list-cameras "
                       "(no camera listed = reseat the ribbon cable, check its direction, then reboot)")
        else:
            parts["camera"].start()
            deadline = time.monotonic() + 3.0
            while parts["camera"].latest() is None and problem is None:
                if time.monotonic() > deadline:
                    problem = "Camera gave no frames"
                time.sleep(0.05)
        if problem:
            if CAMERA_REQUIRED:
                raise CourseFault(problem + " (Settings > Other > Camera required: off lets the run go without it)")
            log("WARNING: " + problem + ". Carrying on WITHOUT the camera: no traffic light or stop sign check")
            parts["camera"].stop()
            parts["camera"] = None
    else:
        log("Camera is not used in this mode")
    log("All systems OK")


def close_parts(parts):
    close_power()
    if parts.get("camera") is not None:
        parts["camera"].stop()
    if parts.get("tracker") is not None:
        parts["tracker"].stop()
    if parts.get("imu") is not None:
        parts["imu"].close()


def run_course():
    """One full run, from the start button to the stop sign."""
    global timer_start
    timer_start = None
    del run_events[:]
    parts = {"imu": None, "tracker": None, "camera": None}
    try:
        log(f"Starting: checking systems")
        course = normalize_directions(directions)
        check_systems(parts)
        tracker, camera = parts["tracker"], parts["camera"]

        timer_start = time.monotonic()                      # start the timer
        log(f"Timer started. Directions: {', '.join(course)}")
        heading_target = 0.0
        wait_for_green(camera)                              # step 3

        count = 0
        while count < len(course):                          # step 4
            what = drive_to_line(tracker, heading_target)    # steps 5 and 6
            log(f"{what} {count + 1}: stopped")
            wait_for_green(camera)                                           # step 9
            direction = course[count]
            log(f"Intersection {count + 1} of {len(course)}: {direction}")
            if direction == "STRAIGHT":                                      # step 7: straight through
                drive_straight(tracker, heading_target, seconds=STRAIGHT_TIME,
                               stop_at_end=False, lanes=False)
            else:
                approach = TURN_APPROACH_TIME_LEFT if direction == "LEFT" else TURN_APPROACH_TIME_RIGHT
                drive_straight(tracker, heading_target, seconds=approach, lanes=False)  # into the intersection
                heading_target = turn(tracker, direction, heading_target)    # step 7: the turn
                drive_straight(tracker, heading_target, seconds=TURN_EXIT_STRAIGHT_TIME,
                               stop_at_end=False)                            # step 8: clear the intersection
            count += 1

        what = drive_to_line(tracker, heading_target)                        # final line
        log(f"Final {what.lower()}: stopped")
        sign_seen = confirm_stop_sign(camera)
        if sign_seen is None:
            log("Stop sign not checked (no camera in this mode)")
        else:
            log("Stop sign confirmed" if sign_seen else "Stop sign NOT confirmed (stopped at the line anyway)")

        timer = time.monotonic() - timer_start
        log(f"Course finished in {timer:.2f} s")
        print(f"Congratulations, you completed the course in a time of {timer:.2f} s")
    except ResetRequested:
        log("Reset pressed: run aborted")
        while button_pressed(reset_button):                 # wait for release
            time.sleep(0.02)
    except CourseFault as exc:
        log(f"Stopped: {exc}")
    except KeyboardInterrupt:
        log("Interrupted (Ctrl+C, or the terminal was closed)")
        raise
    except Exception as exc:
        log(f"ERROR: {type(exc).__name__}: {exc}")
        raise
    finally:
        motors.stop()
        close_parts(parts)
        write_log()
        timer_start = None


################# setup menu: settings, IR test, tests, directions ###################
# (section, variable, label, kind, low, high). kind: float, int, bool, choice (options in `high`),
# list_float / list_bool (four values in the order FL, FR, BL, BR).
SETTINGS = [
    ("Speed and correction", "USE_RAMP", "Gradual speed change (ramp up): on / off", "bool", None, None),
    ("Speed and correction", "RAMP_RATE", "   ramp: acceleration limit, % per second", "float", 1, 1000),
    ("Speed and correction", "USE_HEADING_HOLD", "Heading correction (gyro steering): on / off", "bool", None, None),
    ("Speed and correction", "USE_SIDE_LINES", "Lane correction (left/right IR sensors): on / off", "bool", None, None),
    ("Speed and correction", "USE_WHEEL_SPEEDS", "Individual wheel speeds: on / off", "bool", None, None),
    ("Speed and correction", "WHEEL_SPEEDS", "   wheel speed, duty % (FL, FR, BL, BR)", "list_float", 0, 100),
    ("Speed and correction", "USE_TURN_WHEEL_SPEEDS", "Individual wheel duty for turns: on / off", "bool", None, None),
    ("Speed and correction", "TURN_WHEEL_DUTIES", "   turn duty per wheel, % (FL, FR, BL, BR)", "list_float", 0, 100),
    ("Mode", "RUN_MODE", "Run mode: full / ir_only / imu_only", "choice", None, ("full", "ir_only", "imu_only")),
    ("Motor speed", "LEFT_SPEED", "Left wheels base duty %", "float", 0, 100),
    ("Motor speed", "RIGHT_SPEED", "Right wheels base duty %", "float", 0, 100),
    ("Motor speed", "IMU_DRIVE_DUTY", "Straight-driving duty % (top of left/right)", "float", 0, 100),
    ("Motor speed", "MAX_MOTOR_DUTY", "Hard duty ceiling %", "float", 0, 100),
    ("Motor speed", "TEST_DUTY", "Wheel-test duty %", "float", 0, 100),
    ("Turning", "TURN_MODE", "Turn type: imu (angle) or timed", "choice", None, ("imu", "timed")),
    ("Turning", "TURN_MAX_DUTY", "Turn duty % (max)", "float", 0, 100),
    ("Turning", "TURN_MIN_DUTY", "Turn duty % (min, imu turns)", "float", 0, 100),
    ("Turning", "TURN_SLOW_GAIN", "Slow-down gain near the target (imu)", "float", 0, 10),
    ("Turning", "TURN_ANGLE", "Turn angle, degrees", "float", 10, 180),
    ("Turning", "TURN_TOLERANCE", "Stop this many degrees early (imu)", "float", 0, 20),
    ("Turning", "TURN_STOP_LEAD", "Stop lead, seconds of spin rate (imu)", "float", 0, 1),
    ("Turning", "TURN_TIMEOUT", "Turn timeout, s (imu)", "float", 1, 30),
    ("Turning", "TURN_STALL_TIME", "No-progress limit, s (imu)", "float", 0.2, 10),
    ("Turning", "TURN_DURATION_RIGHT", "Right turn time, s (timed)", "float", 0, 5),
    ("Turning", "TURN_DURATION_LEFT", "Left turn time, s (timed)", "float", 0, 5),
    ("Timing", "TURN_APPROACH_TIME_RIGHT", "Drive on before a right turn, s", "float", 0, 10),
    ("Timing", "TURN_APPROACH_TIME_LEFT", "Drive on before a left turn, s", "float", 0, 10),
    ("Timing", "TURN_EXIT_STRAIGHT_TIME", "Drive on after a turn, s (lines ignored)", "float", 0, 10),
    ("Timing", "STRAIGHT_TIME", "Cross an intersection going STRAIGHT, s", "float", 0, 20),
    ("Timing", "LEG_TIME", "IMU-only: drive to the next intersection, s", "float", 0.5, 120),
    ("Timing", "LEG_TIMEOUT", "Give up if no front line after, s", "float", 1, 300),
    ("Timing", "START_DELAY", "Countdown after START, s", "int", 0, 30),
    ("Timing", "DIRECTION_PAUSE", "Pause when changing direction, s", "float", 0, 1),
    ("Timing", "LOOP_DT", "Control loop period, s", "float", 0.005, 0.1),
    ("Heading hold", "HEADING_KP", "Heading P gain", "float", 0, 10),
    ("Heading hold", "HEADING_KD", "Heading D gain", "float", 0, 5),
    ("Heading hold", "MAX_HEADING_TRIM", "Largest steering correction, duty %", "float", 0, 50),
    ("Heading hold", "MAX_HEADING_ERROR", "Stop if heading is off by, degrees", "float", 1, 90),
    ("Heading hold", "IMU_YAW_SIGN", "Yaw sign (use -1 if left turns read negative)", "choice", None, (1, -1)),
    ("Lanes and lines", "IR_FRONT_PIN", "Front IR sensor pin (BCM): stops at the line", "int", 2, 27),
    ("Lanes and lines", "IR_LEFT_PIN", "Left IR sensor pin (BCM): lane correction", "int", 2, 27),
    ("Lanes and lines", "IR_RIGHT_PIN", "Right IR sensor pin (BCM): lane correction", "int", 2, 27),
    ("Lanes and lines", "SIDE_LINE_TRIM", "Lane steering per sensor, duty %", "float", 0, 50),
    ("Lanes and lines", "LANE_HOLD_TIME", "Keep a lane correction for, s", "float", 0, 2),
    ("Lanes and lines", "LINE_DETECTED", "IR value on a line", "choice", None, (1, 0)),
    ("Lanes and lines", "LINE_CONFIRM_READS", "Reads needed to confirm a front line", "int", 1, 20),
    ("Light and sign", "USE_TRAFFIC_LIGHT", "Wait for a green light at stop lines", "bool", None, None),
    ("Light and sign", "LIGHT_WAIT_TIMEOUT", "Longest wait for green, s (0 = forever)", "float", 0, 120),
    ("Light and sign", "NO_LIGHT_FRAMES", "Frames with no light before going", "int", 1, 200),
    ("Light and sign", "STOP_SIGN_CONFIRM_TIME", "Look for the stop sign for, s", "float", 0, 30),
    ("Wheels", "REVERSED", "Reverse wheel direction (FL, FR, BL, BR)", "list_bool", None, None),
    ("Wheels", "WHEEL_FACTORS_FORWARD", "Wheel scale going forward (FL, FR, BL, BR)", "list_float", 0, 1),
    ("Wheels", "WHEEL_FACTORS_REVERSE", "Wheel scale going backward (FL, FR, BL, BR)", "list_float", 0, 1),
    ("Other", "BRAKE_ON_STOP", "Short-brake when stopping (off = coast)", "bool", None, None),
    ("Other", "MOTOR_TWITCH_CHECK", "Nudge each wheel during the start-up check", "bool", None, None),
    ("Other", "USE_POWER_GUARD", "Power guard: stop on overcurrent or low battery", "bool", None, None),
    ("Other", "CAMERA_BACKEND", "Camera type", "choice", None, ("auto", "picamera2", "v4l2")),
    ("Other", "CAMERA_INDEX", "USB camera number", "int", 0, 9),
    ("Other", "CAMERA_REQUIRED", "Camera required: stop the run if it will not open", "bool", None, None),
    ("Other", "CAMERA_FULL_FOV", "Pi camera: full sensor view (not zoomed in)", "bool", None, None),
    ("Other", "CAMERA_FLIP", "Pi camera: rotate 180 degrees (upside down)", "bool", None, None),
    ("Other", "CAMERA_WIDTH", "Camera width, pixels", "int", 160, 1920),
    ("Other", "CAMERA_HEIGHT", "Camera height, pixels", "int", 120, 1080),
]

# Settings that come from the robot preset (marked * in the list when they differ from it).
PRESET_NAMES = {
    "LEFT_SPEED": "left_speed", "RIGHT_SPEED": "right_speed", "RAMP_RATE": "ramp_rate",
    "TEST_DUTY": "single_motor_duty", "IMU_DRIVE_DUTY": "imu_drive_duty",
    "TURN_MAX_DUTY": "imu_turn_max_duty", "TURN_MIN_DUTY": "imu_turn_min_duty",
    "HEADING_KP": "imu_heading_kp", "HEADING_KD": "imu_heading_kd",
    "MAX_HEADING_TRIM": "imu_max_heading_trim", "TURN_SLOW_GAIN": "imu_turn_slow_gain",
}


def ask(prompt):
    """Prompt for a line of text. Closed input stops the program cleanly."""
    flush_stdin()
    try:
        return input(prompt).strip()
    except EOFError:
        raise KeyboardInterrupt


def parse_scalar(kind, low, high, text):
    text = text.strip()
    if kind == "bool":
        if text.lower() in ("1", "y", "yes", "true", "on"):
            return True
        if text.lower() in ("0", "n", "no", "false", "off"):
            return False
        raise ValueError("use on or off")
    if kind == "choice":
        for option in high:
            if str(option).lower() == text.lower():
                return option
        raise ValueError("choose one of: " + ", ".join(str(option) for option in high))
    try:
        number = float(text) if kind == "float" else int(text)
    except ValueError:
        raise ValueError(f"{text!r} is not a number")
    if not low <= number <= high:
        raise ValueError(f"must be between {low:g} and {high:g}")
    return number


def parse_setting(kind, low, high, text):
    if kind in ("list_bool", "list_float"):
        tokens = text.replace(",", " ").split()
        if len(tokens) == 1:
            tokens *= 4
        if len(tokens) != 4:
            raise ValueError("give 4 values (FL, FR, BL, BR), or 1 value for all four")
        element = "bool" if kind == "list_bool" else "float"
        return tuple(parse_scalar(element, low, high, token) for token in tokens)
    return parse_scalar(kind, low, high, text)


def format_value(value, kind):
    if kind == "bool":
        return "on" if value else "off"
    if kind == "list_bool":
        return ", ".join("on" if item else "off" for item in value)
    if kind == "list_float":
        return ", ".join(f"{item:g}" for item in value)
    if kind == "float":
        return f"{value:g}"
    return str(value)


def describe_range(kind, low, high):
    if kind == "choice":
        return "one of " + ", ".join(str(option) for option in high)
    if kind in ("bool", "list_bool"):
        return "on or off" + (" for each wheel" if kind == "list_bool" else "")
    return f"{low:g} to {high:g}" + (" for each wheel" if kind == "list_float" else "")


FACTORY = {row[1]: globals()[row[1]] for row in SETTINGS}     # the code's own defaults, captured at start-up
PROFILES = {}                  # profile name -> saved settings (only values that differ from the defaults)
ACTIVE_PROFILE = "default"     # edits in the settings menu are saved into this profile


def default_value(name):
    """What a setting is restored to: the robot preset's value, or the code's default."""
    key = PRESET_NAMES.get(name)
    if key is not None:
        return PRESET[key]
    if name == "WHEEL_SPEEDS":
        return preset_wheel_values(PRESET, PRESET["imu_drive_duty"])
    if name == "TURN_WHEEL_DUTIES":
        return preset_wheel_values(PRESET, PRESET["imu_turn_max_duty"])
    return FACTORY[name]


def settings_snapshot():
    """The robot plus every setting that differs from its default, ready to save."""
    data = {"BOT_ID": BOT_ID}
    for _, name, _, _, _, _ in SETTINGS:
        value = globals()[name]
        if value != default_value(name):
            data[name] = list(value) if isinstance(value, tuple) else value
    return data


def apply_snapshot(data):
    """Set the robot and every setting from a saved dict; anything missing goes to its default."""
    global BOT_ID
    try:
        BOT_ID = normalize_bot_id(data.get("BOT_ID", BOT_ID))
    except ValueError as exc:
        print(f"Saved robot ignored: {exc}")
    apply_preset(BOT_ID)
    for _, name, _, kind, low, high in SETTINGS:
        value = default_value(name)
        if name in data:
            saved = data[name]
            text = ",".join(str(item) for item in saved) if isinstance(saved, list) else str(saved)
            try:
                value = parse_setting(kind, low, high, text)
            except ValueError as exc:
                print(f"Skipped saved {name}: {exc}")
        globals()[name] = value


def write_profiles():
    try:
        with open(PROFILES_FILE, "w") as handle:
            json.dump({"active": ACTIVE_PROFILE, "profiles": PROFILES}, handle, indent=1)
    except OSError as exc:
        print(f"Could not save {PROFILES_FILE}: {exc}")


def save_settings():
    """Save the current settings into the active profile."""
    PROFILES[ACTIVE_PROFILE] = settings_snapshot()
    write_profiles()


def load_settings():
    """Start-up: load the active profile. An old settings.json becomes the profile "default"."""
    global PROFILES, ACTIVE_PROFILE
    try:
        if os.path.exists(PROFILES_FILE):
            with open(PROFILES_FILE) as handle:
                saved = json.load(handle)
            PROFILES = dict(saved.get("profiles", {}))
            ACTIVE_PROFILE = saved.get("active", "default")
        elif os.path.exists(SETTINGS_FILE):
            with open(SETTINGS_FILE) as handle:
                PROFILES = {"default": json.load(handle)}
            ACTIVE_PROFILE = "default"
            print(f"Moved your saved settings from {SETTINGS_FILE} into the profile 'default'")
    except (OSError, ValueError) as exc:
        print(f"Ignoring the saved settings ({exc}); starting from the defaults")
        PROFILES, ACTIVE_PROFILE = {}, "default"
    if PROFILES and ACTIVE_PROFILE not in PROFILES:
        ACTIVE_PROFILE = "default" if "default" in PROFILES else sorted(PROFILES)[0]
    if ACTIVE_PROFILE in PROFILES:
        apply_snapshot(PROFILES[ACTIVE_PROFILE])
        print(f"Loaded profile '{ACTIVE_PROFILE}'")


def parse_item_numbers(words):
    """'5', '3-9', '2 7 12', 'ALL' -> sorted list of setting numbers. Raises ValueError."""
    numbers = set()
    for word in " ".join(words).replace(",", " ").split():
        if word.upper() == "ALL":
            return list(range(1, len(SETTINGS) + 1))
        try:
            first, last = (int(part) for part in word.split("-", 1)) if "-" in word else (int(word),) * 2
        except ValueError:
            raise ValueError(f"Cannot read {word!r}: use item numbers like 5, 3-9 or ALL")
        if not 1 <= first <= last <= len(SETTINGS):
            raise ValueError(f"Item numbers go from 1 to {len(SETTINGS)}")
        numbers.update(range(first, last + 1))
    if not numbers:
        raise ValueError("Say which items: D 5, D 3-9, D 2 7 12 or D ALL")
    return sorted(numbers)


def show_settings():
    print(f"\nSettings for the {BOT_LABELS[normalize_bot_id(BOT_ID)]}   Profile: {ACTIVE_PROFILE}   "
          "(* = differs from the default)")
    section = None
    for number, (sec, name, label, kind, low, high) in enumerate(SETTINGS, start=1):
        if sec != section:
            print(f"\n  -- {sec} --")
            section = sec
        mark = "*" if globals()[name] != default_value(name) else " "
        print(f"  {number:2d}{mark} {label:<46} {format_value(globals()[name], kind):<18} {name}")


def settings_menu():
    """Change or restore any setting while the robot is still. Changes are saved into the active profile."""
    while True:
        show_settings()
        text = ask("\nType a number and a value (e.g. 1 30), or just a number to be asked.\n"
                   "D 5 restores item 5 to its default (also D 3-9, D 2 7 12, D ALL). Enter = back: ")
        if not text:
            return
        words = text.split()
        if words[0].upper() in ("D", "DEFAULT", "RESTORE"):
            try:
                numbers = parse_item_numbers(words[1:])
            except ValueError as exc:
                print(exc)
                continue
            if len(numbers) == len(SETTINGS) and \
                    ask(f"Restore ALL settings of profile '{ACTIVE_PROFILE}' to the defaults? Type Y: ").upper() != "Y":
                continue
            for number in numbers:
                _, name, _, kind, _, _ = SETTINGS[number - 1]
                globals()[name] = default_value(name)
                if len(numbers) <= 6:
                    print(f"{name} = {format_value(globals()[name], kind)} (default)")
            save_settings()
            if len(numbers) > 6:
                print(f"Restored {len(numbers)} settings to their defaults.")
            continue
        parts = text.split(None, 1)
        if not parts[0].isdigit() or not 1 <= int(parts[0]) <= len(SETTINGS):
            print(f"Type a setting number from 1 to {len(SETTINGS)}.")
            continue
        _, name, label, kind, low, high = SETTINGS[int(parts[0]) - 1]
        if len(parts) == 1:
            answer = ask(f"{label} ({name}) is {format_value(globals()[name], kind)} "
                         f"(default {format_value(default_value(name), kind)}). "
                         f"New value ({describe_range(kind, low, high)}), Enter = no change: ")
            if not answer:
                continue
        else:
            answer = parts[1]
        try:
            globals()[name] = parse_setting(kind, low, high, answer)
        except ValueError as exc:
            print(f"Not changed: {exc}")
            continue
        save_settings()
        print(f"{name} = {format_value(globals()[name], kind)}")


def valid_profile_name(name):
    return 1 <= len(name) <= 24 and all(ch.isalnum() or ch in " _-" for ch in name)


def find_profile(text):
    """A profile by list number or by name (any capitals). None if there is no match."""
    names = sorted(PROFILES, key=str.lower)
    if text.isdigit() and 1 <= int(text) <= len(names):
        return names[int(text) - 1]
    for name in names:
        if name.lower() == text.lower():
            return name
    return None


def profiles_menu():
    """Named sets of settings: save the current ones, start a new one, load, delete."""
    global ACTIVE_PROFILE
    save_settings()                       # make sure the active profile exists and is up to date
    while True:
        print("\n===== PROFILES =====")
        for number, name in enumerate(sorted(PROFILES, key=str.lower), start=1):
            saved = PROFILES[name]
            changed = len([key for key in saved if key != "BOT_ID"])
            try:
                robot = BOT_LABELS[normalize_bot_id(saved.get("BOT_ID", BOT_ID))]
            except ValueError:
                robot = "?"
            print(f"  {number:2d}{'*' if name == ACTIVE_PROFILE else ' '} {name:<24} {robot:<28} "
                  f"{changed} changed setting{'' if changed == 1 else 's'}")
        print("  * = active. Every edit in Settings is saved into the active profile automatically.")
        text = ask("S name = save the current settings as a new profile | N name = new profile from the defaults\n"
                   "L name/number = load | X name/number = delete | Enter = back: ")
        if not text:
            return
        parts = text.split(None, 1)
        command, argument = parts[0].upper(), (parts[1].strip() if len(parts) > 1 else "")
        if command in ("S", "N"):
            if not valid_profile_name(argument):
                print("Give a name of 1-24 letters, digits, spaces, - or _, e.g. S slow track")
                continue
            existing = find_profile(argument)
            if existing is not None and existing != ACTIVE_PROFILE and \
                    ask(f"Profile '{existing}' exists. Replace it? Type Y: ").upper() != "Y":
                continue
            save_settings()                                   # keep the profile we are leaving up to date
            if command == "N":
                apply_snapshot({"BOT_ID": BOT_ID})            # every setting back to its default
            ACTIVE_PROFILE = existing or argument
            save_settings()
            print(f"Profile '{ACTIVE_PROFILE}' is now active"
                  + (" (all settings at their defaults)." if command == "N" else "."))
        elif command in ("L", "X"):
            name = find_profile(argument)
            if name is None:
                print("No such profile. Type its number or its name.")
            elif command == "L":
                save_settings()
                ACTIVE_PROFILE = name
                apply_snapshot(PROFILES[name])
                write_profiles()
                print(f"Loaded profile '{name}': {BOT_LABELS[normalize_bot_id(BOT_ID)]}, "
                      f"run mode {RUN_MODE_LABELS[RUN_MODE]}.")
            elif name == ACTIVE_PROFILE:
                print("That profile is active. Load another one first.")
            elif ask(f"Delete profile '{name}'? Type Y: ").upper() == "Y":
                del PROFILES[name]
                write_profiles()
                print(f"Deleted '{name}'.")
        else:
            print("Type S, N, L or X followed by a name or number.")


def ir_test():
    """Live IR readout. The motors stay OFF: slide a line under the sensors by hand."""
    motors.stop(brake=False)
    print(f"\nIR sensor test (no movement). Pins: front {IR_FRONT_PIN}, left {IR_LEFT_PIN}, right {IR_RIGHT_PIN} "
          f"(change them in Settings > Lanes and lines). A sensor on a line should read {LINE_DETECTED}. "
          "Press Enter to finish.")
    problem = ir_pin_problem()
    if problem:
        print("WARNING: " + problem)
    flush_stdin()
    while True:
        if select.select([sys.stdin], [], [], 0.1)[0]:
            sys.stdin.readline()
            break
        left, front, right = read_sensors()
        left_on, front_on, right_on = (value == LINE_DETECTED for value in (left, front, right))
        trim = side_line_trim(left_on, right_on, front_on)
        if front_on:
            verdict = "STOP LINE (front sensor)"
        elif trim < 0:
            verdict = f"left line: steer RIGHT ({trim:+.1f})"
        elif trim > 0:
            verdict = f"right line: steer LEFT ({trim:+.1f})"
        elif left_on or right_on:
            verdict = "no steering (lines cancel, or lane steering is off)"
        else:
            verdict = "no line"
        duties = forward_targets(0.0, 0.0, trim)
        print(f"\r  LEFT={left} FRONT={front} RIGHT={right} | {verdict} | left/right duty "
              f"{duties[0]:.0f}/{duties[1]:.0f}".ljust(110), end="", flush=True)
    print()


def wheel_test():
    print(f"\nWheel test: each wheel spins forward for 0.15 s at {TEST_DUTY:g} %, in the order "
          "FL, FR, BL, BR. LIFT THE ROBOT so the wheels are free.")
    if ask("Type Y to run: ").upper() != "Y":
        return
    motors.twitch()
    print("Done. A wheel that spun the wrong way: flip it in Settings > Reverse wheel direction.")


def turn_test():
    """Spin in place once with the current turn settings and print the measured angle."""
    answer = ask("\nTurn test: the robot spins in place. L or R (Enter cancels): ").upper()
    if answer not in ("L", "R"):
        return
    direction = "LEFT" if answer == "L" else "RIGHT"
    del run_events[:]
    parts = {"imu": None, "tracker": None, "camera": None}
    try:
        start_imu(parts)
        start_power()
        print("Turning in 2 s...")
        time.sleep(2.0)
        turn(parts["tracker"], direction, 0.0)
    except (CourseFault, ResetRequested) as exc:
        print(f"Stopped: {exc}")
    finally:
        motors.stop()
        close_parts(parts)


WHEEL_NAMES = ("FL", "FR", "BL", "BR")


def wheel_run_test():
    """Spin one wheel (or all four) until you press Enter. LIFT THE ROBOT first."""
    text = ask("\nWheel run test. Which wheel: FL, FR, BL, BR or ALL; add B for backward "
               "(e.g. FL or ALL B). Enter cancels: ").upper()
    tokens = text.split()
    if not tokens:
        return
    if tokens[0] == "ALL":
        indexes = range(4)
    elif tokens[0] in WHEEL_NAMES:
        indexes = [WHEEL_NAMES.index(tokens[0])]
    else:
        print("Type FL, FR, BL, BR or ALL.")
        return
    forward = not (len(tokens) > 1 and tokens[1] in ("B", "BACK"))
    duty = min(TEST_DUTY, MAX_MOTOR_DUTY)
    print(f"LIFT THE ROBOT so the wheels are free. {tokens[0]} spinning "
          f"{'forward' if forward else 'backward'} at {duty:g} %. Press Enter (or reset) to stop.")
    flush_stdin()
    try:
        motors.spin(indexes, duty, forward)
        while True:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                sys.stdin.readline()
                break
            if button_pressed(reset_button):
                break
    finally:
        motors.stop()
    print("Stopped.")


def annotate_frame(frame, light, sign):
    """Copy of the frame with both search areas and any detections drawn on it."""
    picture = frame.copy()
    height, width = picture.shape[:2]
    cv2.rectangle(picture, (int(width * LEFT_CUTOFF), int(height * TOP_CUTOFF)),
                  (int(width * RIGHT_CUTOFF), int(height * BOTTOM_CUTOFF)), (255, 255, 0), 1)
    cv2.putText(picture, "traffic light area", (int(width * LEFT_CUTOFF) + 3, int(height * BOTTOM_CUTOFF) + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
    cv2.rectangle(picture, (int(width * 0.50), int(height * 0.05)), (width - 1, int(height * 0.80)), (255, 0, 255), 1)
    cv2.putText(picture, "stop sign area", (int(width * 0.50) + 3, int(height * 0.80) - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)
    if light is not None:
        x, y, w, h = light["box"]
        color = {"RED": (0, 0, 255), "YELLOW": (0, 255, 255), "GREEN": (0, 255, 0)}[light["color"]]
        cv2.rectangle(picture, (x, y), (x + w, y + h), color, 2)
        cv2.putText(picture, light["color"], (x, max(y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    if sign is not None:
        x, y, w, h = sign["box"]
        cv2.rectangle(picture, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(picture, f"STOP {sign['confidence']:.2f}", (x, max(y - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    return picture


def camera_test():
    """Live camera and detector readout. The motors stay OFF.

    Enter finishes. S then Enter saves a picture (camera_HHMMSS.jpg) with the search areas
    and detections drawn on it, so you can aim the camera over SSH.
    """
    motors.stop(brake=False)
    camera = CameraStream()
    if not camera.opened():
        print("Camera did not open. Ribbon camera: sudo apt install -y python3-picamera2 and test with "
              "rpicam-hello. USB camera: check v4l2-ctl --list-devices (see Settings > Camera).")
        camera.stop()
        return
    camera.start()
    try:
        deadline = time.monotonic() + 3.0
        while camera.latest() is None:
            if time.monotonic() > deadline:
                print("The camera opened but gave no frames.")
                return
            time.sleep(0.05)
        print(f"\nCamera test (no movement), camera type: {camera.backend}. "
              "Enter = finish, S then Enter = save a picture.")
        flush_stdin()
        frames = 0
        fps = 0.0
        window = time.monotonic()
        while True:
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if line.strip().upper() != "S":
                    break
                saved = camera.latest()
                if saved is not None:
                    name = time.strftime("camera_%H%M%S.jpg")
                    data = get_vision_data(saved)
                    cv2.imwrite(name, annotate_frame(saved, data["traffic_light_data"], data["stop_sign_data"]))
                    print(f"\nSaved {name}")
            frame = camera.latest()
            if frame is None:
                time.sleep(0.02)
                continue
            data = get_vision_data(frame)
            light, sign = data["traffic_light_data"], data["stop_sign_data"]
            frames += 1
            now = time.monotonic()
            if now - window >= 1.0:
                fps, frames, window = frames / (now - window), 0, now
            height, width = frame.shape[:2]
            light_text = f"{light['color']} {data['traffic_light_position']}" if light else "none"
            sign_text = f"YES ({sign['confidence']:.2f}) {data['stop_sign_position']}" if sign else "no"
            print(f"\r  {width}x{height} | {fps:4.1f} checks/s | traffic light: {light_text} | "
                  f"stop sign: {sign_text}".ljust(90), end="", flush=True)
            time.sleep(0.01)
    finally:
        camera.stop()
    print()


def imu_check():
    """Motors off. Turn the robot LEFT by hand: the yaw must go UP (positive)."""
    motors.stop(brake=False)
    parts = {"imu": None, "tracker": None, "camera": None}
    try:
        print("\nIMU check. Put the robot flat on the table and keep it still for 3 seconds...")
        start_imu(parts)
        tracker = parts["tracker"]
        roll0, pitch0, _, _ = tracker.get()
        print("Now turn the whole robot LEFT by hand, about 45-90 degrees, keeping it flat. "
              "Then press Enter.")
        flush_stdin()
        top = bottom = 0.0
        tilt = 0.0
        while True:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                sys.stdin.readline()
                break
            roll, pitch, yaw, _ = tracker.get()
            top, bottom = max(top, yaw), min(bottom, yaw)
            tilt = max(tilt, abs(roll - roll0), abs(pitch - pitch0))
            print(f"\r  yaw {yaw:+7.1f} deg   roll {roll - roll0:+6.1f}   pitch {pitch - pitch0:+6.1f}".ljust(70),
                  end="", flush=True)
        print()
        if max(top, -bottom) < 15:
            if tilt > 25:
                print("RESULT: roll/pitch moved but yaw did not. The IMU is probably mounted on its side, "
                      "so the Z axis is not the vertical axis. Tell me how it is mounted.")
            else:
                print("RESULT: yaw barely moved. Turn the robot further, flat on the table, and try again.")
        elif top >= -bottom:
            print(f"RESULT: OK. Turning LEFT made yaw go UP (to {top:+.0f}). "
                  f"IMU_YAW_SIGN = {IMU_YAW_SIGN} is right.")
        else:
            print(f"RESULT: WRONG SIGN. Turning LEFT made yaw go DOWN (to {bottom:+.0f}). "
                  f"Set IMU_YAW_SIGN to {-IMU_YAW_SIGN} in Settings > Heading hold. "
                  "With the wrong sign the heading hold pushes the robot further off course.")
    except (CourseFault, ResetRequested) as exc:
        print(f"Stopped: {exc}")
    finally:
        close_parts(parts)


def straight_test():
    """Drive straight with the heading hold only (no lanes) and report how well it holds."""
    text = ask("\nStraight test: the robot drives straight ahead. Clear floor in front, please. "
               "Seconds (Enter = 3, C = cancel): ")
    if text.upper() == "C":
        return
    try:
        seconds = float(text) if text else 3.0
    except ValueError:
        print("Type a number of seconds.")
        return
    parts = {"imu": None, "tracker": None, "camera": None}
    trace = []
    try:
        start_imu(parts)
        start_power()
        print("Driving in 2 s...")
        time.sleep(2.0)
        try:
            drive_straight(parts["tracker"], 0.0, seconds=seconds, lanes=False, report_every=0.5, trace=trace)
        except CourseFault as exc:
            print(f"Stopped: {exc}")
        motors.stop()
        if len(trace) < 10:
            return
        end = trace[-1][1]
        middle = trace[len(trace) * 2 // 3][1]
        if abs(end) < 3:
            print(f"RESULT: straight. Ended {end:+.1f} deg off.")
        elif abs(end) > abs(middle) + 2:
            print(f"RESULT: the error kept growing (ended {end:+.1f} deg). The correction is probably pushing "
                  "the wrong way: run the IMU check (option 9) and the wheel test (option 3), and check "
                  "that FL/BL really are the LEFT wheels.")
        else:
            side = "LEFT" if end > 0 else "RIGHT"
            fix = "raise LEFT_SPEED or lower RIGHT_SPEED" if end > 0 else "raise RIGHT_SPEED or lower LEFT_SPEED"
            print(f"RESULT: it holds but sits {end:+.1f} deg off, drifting {side}. "
                  f"Balance the wheels a little: {fix} in Settings.")
    except (CourseFault, ResetRequested) as exc:
        print(f"Stopped: {exc}")
    finally:
        motors.stop()
        close_parts(parts)


def choose_mode():
    """Pick which sensors a run uses."""
    global RUN_MODE
    print("\n1 = FULL: IR sensors + IMU + camera (lines, heading hold, lights, stop sign)")
    print("2 = IR ONLY: no IMU, no camera (lines and lanes from the IR sensors, timed turns, no light or sign check)")
    print("3 = IMU ONLY: no IR, no camera (heading hold, each leg drives for LEG_TIME seconds, timed turns)")
    text = ask("Mode (Enter cancels): ")
    modes = {"1": "full", "2": "ir_only", "3": "imu_only"}
    if text in modes:
        RUN_MODE = modes[text]
        save_settings()
        print(f"Run mode: {RUN_MODE_LABELS[RUN_MODE]}")
    elif text:
        print("Type 1, 2 or 3.")


def vision_readout():
    """Ryan's vision loop with the Pi camera: prints what is seen and where. Motors stay off."""
    motors.stop(brake=False)
    print("\nVision readout (no movement). Ctrl+C returns to the menu.")
    try:
        vision_main()
    except Exception as exc:
        print(f"Vision readout stopped: {exc}. Ribbon camera: sudo apt install -y python3-picamera2 "
              "and test with rpicam-hello.")
    finally:
        close_camera()


def choose_bot():
    global BOT_ID
    print("\n1 = 500 RPM bot, 2 = 100 RPM bot")
    text = ask("Robot (Enter cancels): ")
    if not text:
        return
    try:
        BOT_ID = normalize_bot_id(text)
    except ValueError as exc:
        print(exc)
        return
    apply_preset(BOT_ID)
    save_settings()
    print(f"Robot: {BOT_LABELS[BOT_ID]}. Its preset speeds and gains are loaded.")


def ask_directions():
    """One L or R per intersection. Returns a list, or None to go back to the menu."""
    last = list(directions)
    while True:
        prompt = "\nDirections, one L, R or S (straight) per intersection (e.g. R S L R)"
        if last:
            prompt += f", Enter keeps {', '.join(last)}"
        text = ask(prompt + ", M = menu: ")
        if text.upper() == "M":
            return None
        if not text:
            if last:
                return last
            print("Type the directions first.")
            continue
        try:
            return parse_directions(text)
        except ValueError as exc:
            print(exc)


def entry_menu():
    """The entry point. True = directions chosen (next: the start button); False = quit."""
    global directions
    while True:
        print("\n===== ROBOT SETUP =====")
        print(f"Robot: {BOT_LABELS[normalize_bot_id(BOT_ID)]}   "
              f"Directions: {', '.join(directions) if directions else 'not set'}")
        print(f"Profile: {ACTIVE_PROFILE}   Run mode: {RUN_MODE_LABELS[RUN_MODE]}")
        print("  1  Settings: speed/correction switches, wheel speeds, turning, timing, lanes, camera")
        print("  2  IR sensor test (no movement)")
        print("  3  Wheel nudge test (each wheel 0.15 s, lift the robot)")
        print("  4  Turn test (spins in place)")
        print("  5  Button test")
        print("  6  Choose robot (500 / 100 RPM preset)")
        print("  7  Wheel run test (spins until you press Enter, lift the robot)")
        print("  8  Camera test (no movement)")
        print("  9  IMU check (no movement): is the yaw sign right?")
        print(" 10  Straight test (drives straight, reports the drift)")
        print(" 11  Vision readout (Ryan's loop: stop sign / light positions, Ctrl+C to stop)")
        print("  M  Run mode: full / IR only / IMU only")
        print("  P  Profiles: save, load or delete named sets of settings")
        print("  G  Enter the directions, then wait for the START button")
        print("  Q  Quit")
        choice = ask("Choose: ").upper()
        if choice == "1":
            settings_menu()
        elif choice == "2":
            ir_test()
        elif choice == "3":
            wheel_test()
        elif choice == "4":
            turn_test()
        elif choice == "5":
            test_buttons()
        elif choice == "6":
            choose_bot()
        elif choice == "7":
            wheel_run_test()
        elif choice == "8":
            camera_test()
        elif choice == "9":
            imu_check()
        elif choice == "10":
            straight_test()
        elif choice == "11":
            vision_readout()
        elif choice == "M":
            choose_mode()
        elif choice == "P":
            profiles_menu()
        elif choice in ("G", ""):
            chosen = ask_directions()
            if chosen is not None:
                directions = chosen
                return True
        elif choice == "Q":
            return False
        else:
            print("Unknown choice")


def wait_for_start():
    """Step 1: wait for the START button. Returns False to go back to the menu."""
    print(f"\nDirections: {', '.join(directions)}")
    print("Put the robot on the start line. Press the START button to go "
          "(or type GO). M = back to the menu.")
    flush_stdin()
    while True:
        if select.select([sys.stdin], [], [], 0.05)[0]:
            line = sys.stdin.readline()
            if line == "":
                raise KeyboardInterrupt
            text = line.strip().upper()
            if text == "GO":
                return True
            if text == "M":
                return False
            print("Type GO, M, or press the START button.")
        elif button_pressed(start_button):
            while button_pressed(start_button):              # wait for release
                time.sleep(0.02)
            return True


def countdown():
    """Short pause so your hand is off the robot before the IMU calibrates."""
    for remaining in range(int(START_DELAY), 0, -1):
        print(f"Going in {remaining}... (reset button or Ctrl+C cancels)")
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            check_reset()
            time.sleep(0.02)


################# safety: closed SSH window / kill still runs the cleanup ###################
STOP_SIGNALS = ("SIGHUP", "SIGTERM", "SIGQUIT")


class stop_on_signals:
    """Turn SIGHUP/SIGTERM/SIGQUIT into KeyboardInterrupt so motors stop and GPIO is released."""

    def __enter__(self):
        self.previous = {}
        for name in STOP_SIGNALS:
            number = getattr(signal, name, None)
            if number is not None:
                try:
                    self.previous[number] = signal.signal(number, self._interrupt)
                except (OSError, ValueError):
                    pass
        return self

    def _interrupt(self, signum, frame):
        for number in self.previous:
            signal.signal(number, signal.SIG_IGN)       # later signals must not cut cleanup short
        raise KeyboardInterrupt(f"signal {signum}")

    def __exit__(self, *exc_info):
        for number, handler in self.previous.items():
            signal.signal(number, handler)
        return False


########################### main function created by Marshall Morris ###################
def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    setup_buttons()
    setup_sensors()
    motors.initialize()
    load_settings()
    try:
        with stop_on_signals():
            while True:
                try:
                    if not entry_menu():         # settings, tests, directions
                        break
                    if not wait_for_start():     # step 1: wait for the START button
                        continue
                    countdown()
                except ResetRequested:
                    print("Cancelled.")
                    continue
                run_course()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        motors.close()                  # stops the wheels and calls GPIO.cleanup()


if __name__ == "__main__":
    main()

    

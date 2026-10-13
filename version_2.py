"""Final code for test day, ALL IN ONE FILE (no project imports). Made by Marshall Morris.

Detectors by Ryan (traffic light, stop sign). Motor map and presets by Kaleb.
Motors, navigation, vision, IMU, IR, INA219 battery readout and power guard, web page and run recording are
all in this file; only Python's standard library and the pip packages (cv2, numpy, RPi.GPIO, smbus2) are imported.

Navigation outline (what main() does)
  1. Setup menu (settings, IR test, wheel/turn/button tests), then the directions;
     the run begins when the START button is pressed
  2. Initialise the IMU, camera and sensors (the motor driver starts with the program)
  3. For each direction (counter = 0, 1, 2 ...):
  4.   Drive forward (IMU heading hold, left AND right IR sensors keep the lane); near the line the
       speed drops to APPROACH_SPEED_PERCENT
  5.   Front IR sees the line:
         - direction STRAIGHT and the light is GREEN: do not stop, keep going for STRAIGHT_TIME
         - otherwise stop, wait for green, then creep into the intersection area at the slow speed
  6.   Turn LEFT / RIGHT, drive out of the intersection, counter + 1
  7. After the last direction: drive on until the front IR sees a line AND the stop sign is seen: stop
  8. Stop the timer, show the time, write the data

Wheel order everywhere: FL, FR, BL, BR. Pins use BCM numbering.
Search for "EDIT" to find the settings you are expected to change.
"""

import collections
import contextlib
import csv
import datetime
import hmac
import http.server
import json
import math
import mimetypes
import os
import queue
import re
import select
import shutil
import signal
import socket
import sys
import termios
import threading
import time
import traceback
import urllib.parse
import zipfile
from dataclasses import dataclass
from pathlib import Path

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
RESET_YAW_AFTER_TURN = True     # zero the IMU yaw after every turn, so each leg starts at heading 0
LANE_HOLD_TIME = 0.3            # s a lane correction is kept after the sensor stops seeing the line
START_DELAY = 3                 # s countdown between entering the directions and moving
STRAIGHT_TIME = 2.5             # s of straight driving to cross an intersection when the direction is STRAIGHT
CHECK_LIGHT_AT_START = False    # look at the traffic light before the first move (off: go straight to driving)
USE_ROLL_THROUGH = True         # STRAIGHT: slow down at the line and keep driving, unless a red/yellow light or a stop sign is seen
ROLL_THROUGH_TURNS = True        # turns and U-turns also keep rolling at a line unless a red/yellow light or a stop sign is seen (off: stop at every line before a turn)
GREEN_FRESH_TIME = 1.0          # s: a red/yellow light seen this recently (and no green since) makes a STRAIGHT stop at the line
USE_APPROACH_SLOW = True        # slow down when approaching a line and while creeping into the intersection
UTURN_SIDE = "LEFT"             # which way a U-turn spins: LEFT or RIGHT
RETURN_EXIT_TIME = 1.0          # s driven straight after turning around, to get off the finish line
AFTER_TURN_FAST_TIME = 0.0      # s at full speed after a turn, then slow down until the next line (0 = off, use APPROACH_SLOW_AFTER)
APPROACH_SLOW_AFTER = 1.5       # s into a drive to a line when the slow-down starts (0 = only the creep after the line)
APPROACH_SPEED_PERCENT = 60.0   # % of the normal driving speed while slowed
FINAL_REQUIRES_SIGN = True      # after the last direction: stop only on a front line AND the stop sign (off: stop at the first line)
STOP_AT_SIGN_MIDCOURSE = True   # a stop sign seen at a line before the end: stop and pause there, STRAIGHT does not drive over it
STOP_SIGN_PAUSE = 1.0           # s to stay stopped at a stop sign before the course goes on
BATTERY_MONITOR = True          # read the INA219 (voltage, current, power); off = no readings and no power guard
INA_BUS = 1                     # EDIT: I2C bus of the INA219
INA_ADDRESS = 0x40              # EDIT: INA219 I2C address (0x40 to 0x4F; the settings screen shows it in decimal)
INA_SHUNT_OHMS = 0.01           # EDIT: shunt resistor on the INA219 board (0.01 for the common 10 milliohm boards)
BATTERY_PERIOD = 0.05           # s between INA219 readings (the power guard and the telemetry use these)
POWER_GUARD = True              # stop the motors on overcurrent / low battery during a run (needs the INA219)
POWER_OVERCURRENT_A = 4.0       # EDIT: A; stop when the battery current stays at or above this
POWER_OVERCURRENT_HOLD_S = 0.5  # s the current must stay over the limit before the guard trips
POWER_LOW_VOLTAGE_V = 6.0       # EDIT: V; stop when the battery voltage stays at or below this (about 3 V per cell)
POWER_LOW_VOLTAGE_HOLD_S = 1.0  # s the voltage must stay under the limit before the guard trips
# EDIT: charge-percentage curve of YOUR pack. PROVISIONAL generic 2-cell Li-ion/LiPo curve: replace with measurements.
BATTERY_PROFILE = {
    "name": "Generic 2S Li-ion/LiPo (PROVISIONAL - replace with your pack's measured curve)",
    "cells": 2,
    "voltage_percent": [(3.30, 0), (3.50, 10), (3.65, 25), (3.75, 45), (3.85, 65), (4.00, 85), (4.20, 100)],
    "low_cell_voltage": 3.30,
    "max_estimate_current": 1.5,
}
YELLOW_WAIT_TIME = 2.0          # s to wait at a YELLOW light, then go anyway (0 = wait for green like a red light)
SIGN_MEMORY_TIME = 2.0          # s: a stop sign seen this recently still counts at the line
VISION_WATCH_FPS = 10.0         # light + sign checks per second in the background during a run
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
LOG_FILE = "run_log.txt"
PROFILES_FILE = "profiles.json"   # named sets of settings; edits are saved into the active profile
SETTINGS_FILE = "settings.json"   # old single settings file; moved into the profile "default" on first start
CAMERA_BACKEND = "auto"         # "auto" = try the Pi camera (picamera2), then a USB camera; or force one
                                #   of "picamera2" (ribbon-cable camera) / "v4l2" (USB webcam)
CAMERA_INDEX = 0                # USB camera number (/dev/video0)
CAMERA_REQUIRED = True          # True: stop the run if the camera does not open; False: carry on without it (no light or sign check)
RECORD_RUNS = True              # save every run's data (sensors, IMU, motors, events) into runs/<date>_<time>_<kind>/ and zip it
RECORD_CAMERA = True            # also save the camera picture of the run as camera.avi (with the run information written on it)
RECORD_DETECT = False          # run the light and stop-sign detectors on EVERY recorded picture (boxes in the video and browser page all run; uses more CPU)
RECORD_CAMERA_FPS = 5.0         # pictures per second saved to camera.avi and sent to the browser page
MAP_CM_PER_SEC_PER_DUTY = 0.8   # map scale: cm travelled per second for each 1 % of wheel duty (calibrate with the straight test)
LIVE_VIEW = True                # during a run, show the camera and status in a browser: http://<robot address>:<port>/
LIVE_VIEW_PORT = 8080
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

    def __init__(self, imu, csv_path="angles.csv", history=20000, alpha=0.98, time_zero=None):
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
        self.log_offset = 0.0                                  # added to logged yaw so the log stays continuous after a re-zero
        self.history = collections.deque(maxlen=history)       # in-memory log
        self.csv_file = open(csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["time_s", "roll_deg", "pitch_deg", "yaw_deg", "yaw_rate_dps"])
        self.start_time = time.monotonic() if time_zero is None else time_zero   # time_zero: the recorder's clock

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
                    row = (now - self.start_time, self.roll, self.pitch, self.yaw - self.yaw_zero + self.log_offset, self.rate)
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

    def zero_yaw(self, absolute=0.0):
        """The current heading becomes 0 degrees. `absolute` is where the robot really points
        (degrees from the start); it only keeps the logs and the route map continuous."""
        with self.lock:
            self.yaw_zero = self.yaw
            self.log_offset = absolute

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


vision_lock = threading.Lock()      # the detectors keep shared state: one caller at a time (run + video thread)


def get_vision_data(frame):
    """Run all vision detectors (safe to call from two threads)."""
    with vision_lock:
        return _get_vision_data(frame)


def _get_vision_data(frame):
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
recent_lines = collections.deque(maxlen=12)    # last log lines, shown on the browser page


def log(message):
    """Print a line with the time since the timer started, and keep it for the log file."""
    elapsed = 0.0 if timer_start is None else time.monotonic() - timer_start
    line = f"[{elapsed:7.2f}s] {message}"
    print(line)
    run_events.append(line)
    recent_lines.append(line)
    kind = "LOG"
    if message.startswith(("Stopped", "ERROR")):
        kind = "ERROR"
    elif message.startswith("WARNING") or "NOT confirmed" in message or "going anyway" in message:
        kind = "WARN"
    rec.event(kind, message)


def write_log():
    try:
        with open(LOG_FILE, "a") as handle:
            handle.write(f"---- run {time.strftime('%Y-%m-%d %H:%M:%S')} ----\n")
            handle.write("\n".join(run_events) + "\n")
    except OSError as exc:
        print(f"Could not write {LOG_FILE}: {exc}")


################# run recorder: data files, camera video and the browser page ###################
# Every run (and the straight and turn tests) gets its own folder  runs/<date>_<time>_<kind>/  holding
#   telemetry.csv   one row per control-loop tick: sensors, IMU, target and real motor duty, light, sign
#   events.csv      everything that happened: log lines, phases, turns, lines, lights, faults, errors
#   imu_200hz.csv   the raw IMU samples (about 200 per second)
#   summary.json    result, total time, time spent in every phase and leg, every turn, every fault
#   meta.json       all settings, pins and directions used for the run
#   camera.avi      the camera picture with the time, phase, sensors and detections written on it
#   camera_frames.csv  the time of every video frame; snap_*.jpg  pictures taken at faults
# and a zip of the folder next to it, ready to copy to another PC (see plot_run.py).
################# INA219 battery sensor (from battery_support.py) ###################
def open_bus(number):
    return SMBus(number)


MAX_BATTERY_AGE = 0.75
INA_CONFIG = 0x2E63  # 32 V range, +/-80 mV, 16 averages per ADC, triggered both.


class BatteryFault(Exception):
    pass


@dataclass(frozen=True)
class BatterySample:
    timestamp: float
    bus_voltage: float     # IN- to GND, load side of the shunt
    shunt_voltage: float   # IN+ minus IN-
    current: float         # positive: IN+ toward IN-

    @property
    def input_voltage(self):
        return self.bus_voltage + self.shunt_voltage

    @property
    def power(self):
        return self.input_voltage * self.current


def check_battery_sample(sample, now):
    if sample is None:
        raise BatteryFault("No battery reading yet")
    if not all(map(math.isfinite, (sample.timestamp, sample.bus_voltage,
                                  sample.shunt_voltage, sample.current))):
        raise BatteryFault("Invalid INA219 reading")
    if not -0.01 <= now - sample.timestamp <= MAX_BATTERY_AGE:
        raise BatteryFault("Battery reading is stale; charge is unavailable")
    if not 0 <= sample.bus_voltage <= 26 or not -0.1 <= sample.input_voltage <= 26:
        raise BatteryFault("INA219 voltage is outside its 0-26 V input range")
    if abs(sample.shunt_voltage) >= 0.0799:
        raise BatteryFault("INA219 shunt reading is at its +/-80 mV limit; current is unreliable")
    return sample


class INA219:
    def __init__(self, bus, address=0x40, shunt_ohms=0.01, clock=time):
        if not 0x40 <= address <= 0x4F:
            raise ValueError("INA219 address must be 0x40 through 0x4F")
        if not math.isfinite(shunt_ohms) or shunt_ohms <= 0:
            raise ValueError("Shunt resistance must be positive and finite")
        self.bus, self.address, self.shunt_ohms, self.clock = bus, address, shunt_ohms, clock

    def read_register(self, register):
        raw = self.bus.read_i2c_block_data(self.address, register, 2)
        if len(raw) != 2:
            raise BatteryFault("Incomplete INA219 register read")
        return (raw[0] << 8) | raw[1]

    def write_register(self, register, value):
        self.bus.write_i2c_block_data(self.address, register, [value >> 8, value & 0xFF])

    def initialize(self):
        # INA219 has no WHO_AM_I register. Only access the configured address;
        # readback checks configuration, not definitive device identity.
        config = self.read_register(0)
        if config & 0xC000:
            raise BatteryFault(f"Unexpected INA219 configuration 0x{config:04X} at 0x{self.address:02X}; "
                               "check device type, address and I2C wiring")
        self.write_register(0, INA_CONFIG & ~7)  # Hold in power-down before first trigger.
        actual = self.read_register(0)
        if actual != INA_CONFIG & ~7:
            raise BatteryFault(f"INA219 configuration readback failed: wrote 0x{INA_CONFIG & ~7:04X}, "
                               f"read 0x{actual:04X} at 0x{self.address:02X}")

    def read_sample(self):
        started = self.clock.monotonic()
        self.write_register(0, INA_CONFIG)  # Each write starts a fresh, paired conversion.
        deadline = started + 0.25
        while True:
            raw_bus = self.read_register(2)
            if self.clock.monotonic() >= deadline:
                raise BatteryFault(f"INA219 conversion timed out at 0x{self.address:02X}; "
                                   f"last bus register 0x{raw_bus:04X}, configuration 0x{INA_CONFIG:04X}")
            if raw_bus & 2:
                break
            self.clock.sleep(0.005)
        raw_shunt = self.read_register(1)
        if raw_shunt & 0x8000:
            raw_shunt -= 0x10000
        shunt_voltage = raw_shunt * 0.00001
        # OVF refers to the chip's calibrated current/power math. We use the
        # raw voltage registers and our own shunt scaling instead.
        sample = BatterySample(started, (raw_bus >> 3) * 0.004, shunt_voltage,
                               shunt_voltage / self.shunt_ohms)
        return check_battery_sample(sample, self.clock.monotonic())


class BatteryStream:
    """All INA219 I/O happens in a worker so the control loop never waits on I2C."""

    def __init__(self, bus_number=1, address=0x40, shunt_ohms=0.01,
                 bus_factory=open_bus, clock=time, period=0.1):
        self.bus_number, self.address, self.shunt_ohms = bus_number, address, shunt_ohms
        if not math.isfinite(period) or period <= 0:
            raise ValueError("INA219 sampling period must be positive and finite")
        self.period = period  # the readout uses 0.1 s; the power guard samples faster
        self.bus_factory, self.clock = bus_factory, clock
        self._sample = self._error = None
        self._lock = threading.Lock()
        self._done = threading.Event()
        self._thread = threading.Thread(target=self._run, name="ina219-reader", daemon=True)
        self._thread.start()

    def _run(self):
        bus = None
        try:
            bus = self.bus_factory(self.bus_number)
            sensor = INA219(bus, self.address, self.shunt_ohms, self.clock)
            # Retry transient bus errors only during initialization, with outputs
            # still off. Mid-motion read errors must reach the guard immediately.
            for attempt in range(3):
                try:
                    sensor.initialize()
                    break
                except OSError:
                    if attempt == 2:
                        raise
                    if self._done.wait(0.05):
                        return
            while not self._done.is_set():
                sample = sensor.read_sample()
                with self._lock:
                    self._sample = sample
                self._done.wait(self.period)
        except Exception as exc:
            with self._lock:
                self._error = (f"INA219 on I2C bus {self.bus_number}, address 0x{self.address:02X}, "
                               f"shunt {self.shunt_ohms:g} ohm: {type(exc).__name__}: {exc}")
        finally:
            if bus is not None:
                try:
                    bus.close()
                except Exception:
                    pass

    def latest(self):
        with self._lock:
            sample, error = self._sample, self._error
        if error is not None:
            raise BatteryFault(error)
        return sample

    def close(self):
        self._done.set()
        self._thread.join(timeout=0.25)
        return not self._thread.is_alive()


@dataclass(frozen=True)
class BatteryProfile:
    name: str
    cells: int
    # Per-cell voltage versus percentage at light load. The bundled generic
    # profile is provisional; replace it with pack measurements for accuracy.
    voltage_percent: tuple
    low_cell_voltage: float
    max_estimate_current: float

    def __post_init__(self):
        if type(self.cells) is not int or not 1 <= self.cells <= 6:
            raise ValueError("Battery cells must be an integer from 1 to 6")
        points = self.voltage_percent
        if len(points) < 2 or any(len(p) != 2 for p in points):
            raise ValueError("Battery profile needs voltage/percentage pairs")
        if not all(math.isfinite(v) and math.isfinite(p) and 0 < v <= 4.5 and 0 <= p <= 100
                   for v, p in points):
            raise ValueError("Battery profile has invalid voltage/percentage values")
        if points[0][1] != 0 or points[-1][1] != 100:
            raise ValueError("Battery profile must run from 0 to 100 percent")
        if any(b[0] <= a[0] or b[1] <= a[1] for a, b in zip(points, points[1:])):
            raise ValueError("Battery profile voltage and percentage must increase")
        if (not math.isfinite(self.low_cell_voltage) or self.low_cell_voltage <= 0
                or not math.isfinite(self.max_estimate_current) or self.max_estimate_current <= 0):
            raise ValueError("Battery low-voltage and estimation-current limits must be positive")

    def estimate(self, voltage):
        if not math.isfinite(voltage):
            raise ValueError("Battery voltage must be finite")
        cell_voltage = voltage / self.cells
        if cell_voltage <= self.voltage_percent[0][0]:
            return 0.0
        for (v0, p0), (v1, p1) in zip(self.voltage_percent, self.voltage_percent[1:]):
            if cell_voltage <= v1:
                return p0 + (p1 - p0) * (cell_voltage - v0) / (v1 - v0)
        return 100.0


def load_battery_profile(path):
    if path is None:
        return None
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return BatteryProfile(data["name"], data["cells"], tuple(map(tuple, data["voltage_percent"])),
                          data["low_cell_voltage"], data["max_estimate_current"])


def battery_status(sample, profile=None):
    voltage = sample.input_voltage
    readings = (f"Battery at sensor: {voltage:.2f} V | Current: {sample.current:+.3f} A | "
                f"Power: {sample.power:+.2f} W")
    if profile is None:
        return readings + " | Charge: unavailable (battery profile not configured)"
    if voltage < profile.cells * 2.0 or voltage > profile.cells * 4.3:
        return readings + " | Charge: unavailable (voltage does not match battery profile)"
    low = " | LOW BATTERY" if voltage <= profile.cells * profile.low_cell_voltage else ""
    if sample.current < -0.02:
        return readings + " | Charge: unavailable (reverse current/charging; voltage estimate unreliable)" + low
    if sample.current > profile.max_estimate_current:
        return readings + " | Charge: unavailable (load too high for voltage estimate)" + low
    percent = profile.estimate(voltage)
    bars = round(percent / 10)
    # Display coarse steps instead of suggesting a precise coulomb-counted SOC.
    percent = int(round(percent / 5) * 5)
    return readings + f" | Charge estimate: ~{percent}% [{'#' * bars}{'-' * (10 - bars)}]" + low


################# power guard (from power_guard.py): stops the motors on overcurrent / low battery ###################
GUARD_PERIOD = 0.02      # worker wait between paired INA219 conversions
MAX_SAMPLE_AGE = 0.5     # a reading older than this while driving trips
START_TIMEOUT = 1.5      # first reading must arrive before any motion
POWERED_HINT = ("check the INA219 wiring and its I2C address. Turning the power guard off in the "
               "settings removes current/voltage protection")


class PowerFault(CourseFault):
    """The power guard stopped the motors."""


def _positive(name, value):
    try:
        ok = math.isfinite(value) and value > 0
    except TypeError:
        ok = False
    if not ok:
        raise ValueError(f"{name} must be a positive finite number")
    return float(value)


class PowerGuard:
    """Overcurrent and low-voltage trip for one powered command."""

    def __init__(self, overcurrent_a, overcurrent_hold_s, low_voltage_v, low_voltage_hold_s,
                 bus=1, address=0x40, shunt_ohms=0.01, stream_factory=None, clock=time):
        self.overcurrent_a = _positive("overcurrent_a", overcurrent_a)
        self.overcurrent_hold_s = _positive("overcurrent_hold_s", overcurrent_hold_s)
        self.low_voltage_v = _positive("low_voltage_v", low_voltage_v)
        self.low_voltage_hold_s = _positive("low_voltage_hold_s", low_voltage_hold_s)
        self.clock = clock
        self.shunt_ohms = _positive("shunt_ohms", shunt_ohms)
        if stream_factory is None:
            def stream_factory():
                return BatteryStream(bus, address, shunt_ohms, clock=clock, period=GUARD_PERIOD)
        self.stream = stream_factory()
        self.peak_current = None
        self.min_voltage = None
        self.drove = False
        self.trip_reason = None
        self._over_since = None
        self._low_since = None

    @classmethod
    def from_settings(cls, settings, bus=1, address=0x40, shunt_ohms=0.01, **kwargs):
        return cls(settings["power_overcurrent_a"], settings["power_overcurrent_hold_s"],
                   settings["power_low_voltage_v"], settings["power_low_voltage_hold_s"],
                   bus=bus, address=address, shunt_ohms=shunt_ohms, **kwargs)

    def describe(self):
        return (f"Power guard ON: stops at {self.overcurrent_a:g} A for {self.overcurrent_hold_s:g} s, "
                f"or {self.low_voltage_v:g} V for {self.low_voltage_hold_s:g} s")

    def wait_ready(self, poll=None):
        """Block motion until the first reading arrives; nothing is powered yet."""
        deadline = self.clock.monotonic() + START_TIMEOUT
        while True:
            if poll is not None:
                poll()
            try:
                sample = self.stream.latest()
                if sample is not None:
                    self._validate(sample, self.clock.monotonic())
            except BatteryFault as exc:
                raise PowerFault(f"Power guard could not read the INA219 ({exc}). "
                                 f"Powered tests are blocked: {POWERED_HINT}.") from exc
            if sample is not None:
                self._record(sample)
                return sample
            if self.clock.monotonic() >= deadline:
                raise PowerFault(f"Power guard got no INA219 reading in {START_TIMEOUT:g} s. "
                                 f"Powered tests are blocked: {POWERED_HINT}.")
            self.clock.sleep(0.01)

    def check(self, motors):
        """Validate before motion as well as during it; never allow a blind first pulse."""
        now = self.clock.monotonic()
        active = any(duty > 0 for duty in motors.duty)
        try:
            sample = self.stream.latest()
            if sample is not None:
                self._validate(sample, now)
        except BatteryFault as exc:
            self._trip(motors, self._failure_reason(exc))
        if sample is None:
            self._trip(motors, "Power guard: no current reading while driving.")
        if active:
            self.drove = True
        self._record(sample)
        if not active:
            self._over_since = self._low_since = None
            return
        if abs(sample.current) >= self.overcurrent_a:
            if self._over_since is None:
                self._over_since = sample.timestamp
            held = sample.timestamp - self._over_since
            if held >= self.overcurrent_hold_s - 1e-9:
                self._trip(motors, f"Overcurrent: {abs(sample.current):.2f} A for {held:.2f} s "
                                   f"(limit {self.overcurrent_a:g} A).")
        else:
            self._over_since = None
        voltage = sample.input_voltage
        if voltage <= self.low_voltage_v:
            if self._low_since is None:
                self._low_since = sample.timestamp
            held = sample.timestamp - self._low_since
            if held >= self.low_voltage_hold_s - 1e-9:
                self._trip(motors, f"Low battery: {voltage:.2f} V for {held:.2f} s "
                                   f"(limit {self.low_voltage_v:g} V). Charge the pack.")
        else:
            self._low_since = None

    def summary(self):
        if self.peak_current is None:
            return None
        text = f"Power: peak {self.peak_current:.2f} A, lowest {self.min_voltage:.2f} V"
        return text + ("" if self.drove else " (motors never driven)")

    def close(self):
        return self.stream.close()

    def _record(self, sample):
        current, voltage = abs(sample.current), sample.input_voltage
        if self.peak_current is None or current > self.peak_current:
            self.peak_current = current
        if self.min_voltage is None or voltage < self.min_voltage:
            self.min_voltage = voltage

    @staticmethod
    def _validate(sample, now):
        age = now - sample.timestamp
        if age > MAX_SAMPLE_AGE:
            raise BatteryFault(f"current reading is {age:.2f} s old; the INA219 read stalled")
        check_battery_sample(sample, now)

    def _failure_reason(self, exc):
        if "80 mV" in str(exc):
            return ("Overcurrent: current beyond the INA219's measurable range "
                    f"(about {0.08 / self.shunt_ohms:g} A for the configured shunt).")
        return f"Power guard: the current monitor failed ({exc})."

    def _trip(self, motors, reason):
        self.trip_reason = reason
        motors.stop(brake=False)  # PWM 0, inputs low before the fault surfaces.
        raise PowerFault(reason + " Motors stopped.")


################# battery and power: INA219 readings, charge estimate, power guard ###################
battery_stream = None           # the ONE INA219 reader of the program (telemetry, web chip, readout, guard)
battery_config = None
battery_retry_at = 0.0
power_guard = None              # set while a run or a powered test is active
run_power = {}                  # peak current / lowest voltage of the last run, for summary.json
_profile_cache = {"key": None, "profile": None, "error": ""}


class SharedStream:
    """Lets the power guard use the program-wide INA219 reader without closing it."""

    def __init__(self, stream):
        self.stream = stream

    def latest(self):
        return self.stream.latest()

    def close(self):
        return True


def battery_profile():
    """The charge curve from BATTERY_PROFILE, or None (charge percentage then shows 'unavailable')."""
    key = repr(BATTERY_PROFILE)
    if key != _profile_cache["key"]:
        try:
            data = BATTERY_PROFILE
            profile = BatteryProfile(data["name"], data["cells"], tuple(map(tuple, data["voltage_percent"])),
                                     data["low_cell_voltage"], data["max_estimate_current"])
            _profile_cache.update(key=key, profile=profile, error="")
        except (ValueError, KeyError, TypeError) as exc:
            _profile_cache.update(key=key, profile=None, error=f"battery profile not usable: {exc}")
    return _profile_cache["profile"]


def close_battery():
    global battery_stream, battery_config
    if battery_stream is not None:
        battery_stream.close()
    battery_stream = battery_config = None


def ensure_battery(force=False):
    """Start (or restart after an error) the INA219 reader. None when BATTERY_MONITOR is off or it will not start."""
    global battery_stream, battery_config, battery_retry_at
    if not BATTERY_MONITOR:
        close_battery()
        return None
    config = (INA_BUS, INA_ADDRESS, INA_SHUNT_OHMS, BATTERY_PERIOD)
    if battery_stream is not None and config == battery_config:
        try:
            battery_stream.latest()
            return battery_stream
        except BatteryFault:
            pass                                          # the reader stopped on an error
    if not force and time.monotonic() < battery_retry_at:
        return None                                       # do not retry a missing sensor more than every 5 s
    close_battery()
    battery_retry_at = time.monotonic() + 5.0
    try:
        battery_stream = BatteryStream(INA_BUS, INA_ADDRESS, INA_SHUNT_OHMS, period=BATTERY_PERIOD)
    except Exception as exc:
        print(f"INA219 reader could not start: {exc}")
        battery_stream = None
        return None
    battery_config = config
    return battery_stream


def battery_sample():
    """The newest INA219 reading, or None. Never raises and never waits (safe inside the control loop)."""
    stream = battery_stream
    if stream is None:
        return None
    try:
        return stream.latest()
    except BatteryFault:
        return None


def battery_error():
    stream = battery_stream
    if stream is None:
        return "INA219 reader not running"
    try:
        stream.latest()
    except BatteryFault as exc:
        return str(exc)
    return ""


def battery_info():
    """For the web page: {'on', 'ok', 'v', 'a', 'w', 'low', 'text'}."""
    if not BATTERY_MONITOR:
        return {"on": False}
    ensure_battery()
    sample = battery_sample()
    if sample is None:
        return {"on": True, "ok": False, "text": battery_error() or "waiting for the first INA219 reading"}
    try:
        check_battery_sample(sample, time.monotonic())
    except BatteryFault as exc:
        return {"on": True, "ok": False, "text": str(exc)}
    voltage = sample.input_voltage
    return {"on": True, "ok": True, "v": round(voltage, 2), "a": round(sample.current, 3),
            "w": round(sample.power, 2), "low": voltage <= POWER_LOW_VOLTAGE_V,
            "text": battery_status(sample, battery_profile())}


def start_power_guard(parts):
    """Before any powered test or run: the INA219 must answer, and the guard watches the motors from then on."""
    global power_guard
    run_power.clear()
    if not BATTERY_MONITOR:
        log("Battery monitor is off: no battery readings, no power guard")
        return
    stream = ensure_battery(force=True)
    profile = battery_profile()
    if not POWER_GUARD:
        log("Power guard is OFF (Settings > Battery and power): no overcurrent or low-battery stop")
        deadline = time.monotonic() + 1.0
        while battery_sample() is None and time.monotonic() < deadline:
            time.sleep(0.02)
        sample = battery_sample()
        log(battery_status(sample, profile) if sample is not None
            else "WARNING: no INA219 reading (" + (battery_error() or "no answer yet") + ")")
        return
    if stream is None:
        raise PowerFault("Power guard: the INA219 reader could not start. " + POWERED_HINT + ".")
    try:
        guard = PowerGuard(POWER_OVERCURRENT_A, POWER_OVERCURRENT_HOLD_S, POWER_LOW_VOLTAGE_V,
                           POWER_LOW_VOLTAGE_HOLD_S, shunt_ohms=INA_SHUNT_OHMS,
                           stream_factory=lambda: SharedStream(stream))
    except ValueError as exc:
        raise CourseFault(f"Power guard setting: {exc}")
    sample = guard.wait_ready(poll=check_reset)           # raises PowerFault before anything is powered
    if sample.input_voltage <= POWER_LOW_VOLTAGE_V:
        raise PowerFault(f"Battery too low to start: {sample.input_voltage:.2f} V "
                         f"(limit {POWER_LOW_VOLTAGE_V:g} V). Charge the pack.")
    parts["guard"] = power_guard = guard
    log(guard.describe())
    log(battery_status(sample, profile))
    rec.event("BATTERY", f"Start: {sample.input_voltage:.2f} V, {sample.current:+.3f} A",
              voltage=round(sample.input_voltage, 3), current=round(sample.current, 3))


def battery_readout():
    """INA219 voltage, current, power and a charge estimate once a second. The motors stay off. Enter ends it."""
    motors.stop(brake=False)
    if not BATTERY_MONITOR:
        print("The battery monitor is off (Settings > Battery and power).")
        return
    stream = ensure_battery(force=True)
    if stream is None:
        print("The INA219 reader could not start. Check the wiring and the address in Settings > Battery and power.")
        return
    profile = battery_profile()
    print(f"\nINA219 on I2C bus {INA_BUS}, address 0x{INA_ADDRESS:02X}, shunt {INA_SHUNT_OHMS:g} ohm. "
          "Motors stay off. Press Enter to return.")
    if profile is not None:
        print(f"Profile: {profile.name}. The charge percentage is a rough voltage estimate; let the battery rest.")
    else:
        print("Voltage and current work. " + (_profile_cache["error"] or "No battery profile") + ".")
    deadline = time.monotonic() + 2.0
    next_print = 0.0
    while True:
        now = time.monotonic()
        try:
            sample = stream.latest()
            if sample is None:
                if now >= deadline:
                    print("The INA219 produced no reading: check the sensor address and wiring.")
                    return
            else:
                check_battery_sample(sample, now)
                if now >= next_print:
                    print(battery_status(sample, profile))
                    next_print = now + 1.0
        except BatteryFault as exc:
            print(f"Battery reading problem: {exc}")
            return
        if select.select([sys.stdin], [], [], 0.1)[0]:
            sys.stdin.readline()
            return


RUNS_DIR = "runs"
TELEMETRY_COLUMNS = [
    "t_s", "timer_s", "clock", "phase", "leg", "pattern",
    "ir_left", "ir_front", "ir_right", "line_left", "line_front", "line_right",
    "roll_deg", "pitch_deg", "yaw_deg", "yaw_rate_dps",
    "heading_target_deg", "heading_error_deg", "lane_trim",
    "target_FL", "target_FR", "target_BL", "target_BR",
    "duty_FL", "duty_FR", "duty_BL", "duty_BR",
    "light", "stop_sign", "video_frame",
    "battery_v", "current_a", "power_w",
]
EVENT_COLUMNS = ["t_s", "timer_s", "clock", "kind", "phase", "leg", "message", "data"]
COLUMN_NOTES = {
    "t_s": "seconds since the recording started (IMU, video and events use the same clock)",
    "timer_s": "seconds on the course timer (blank before the timer starts)",
    "ir_left/front/right": "raw pin value of each IR sensor",
    "line_left/front/right": "1 = that sensor sees a line",
    "yaw_deg": "heading, positive = turned LEFT; heading_error_deg = yaw minus the planned heading",
    "lane_trim": "lane steering, positive = slow the left side (steer left)",
    "target_*": "duty the control loop asked for (before the ramp and the duty ceiling)",
    "duty_*": "duty % actually sent to each wheel (the PWM)",
    "pattern": "direction pattern: FORWARD, TURN_LEFT, TURN_RIGHT or OTHER",
    "battery_v/current_a/power_w": "INA219: battery voltage at the sensor, battery current (+ = discharging) and power",
}


def path_step(x, y, heading, pattern, mean_duty, dt, cm_per_duty=None):
    """Dead reckoning: move the estimated position forward along the heading (positive = turned LEFT)."""
    if pattern != "FORWARD" or dt <= 0:
        return x, y
    speed = (MAP_CM_PER_SEC_PER_DUTY if cm_per_duty is None else cm_per_duty) * mean_duty
    radians = math.radians(heading)
    return x - math.sin(radians) * speed * dt, y + math.cos(radians) * speed * dt


def pattern_name(pattern):
    if pattern == FORWARD:
        return "FORWARD"
    if pattern == TURN_LEFT:
        return "TURN_LEFT"
    if pattern == TURN_RIGHT:
        return "TURN_RIGHT"
    return "OTHER" if pattern is not None else ""


def local_address():
    """The Pi's address on the network (for the copy and browser hints)."""
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("10.255.255.255", 1))
        address = probe.getsockname()[0]
        probe.close()
        return address
    except OSError:
        return socket.gethostname()


class CameraWorker(threading.Thread):
    """Takes the newest camera picture a few times a second, writes the run information on it,
    keeps it for the browser page and (while a run is being recorded) saves it to camera.avi."""

    def __init__(self, camera):
        super().__init__(daemon=True)
        self.camera = camera
        self.running = True
        self.record = rec.active and RECORD_CAMERA
        self.writer = None
        self.jpg_mode = False
        self.index = 0
        self.frames_file = None
        self.frames_writer = None
        if self.record:
            self.frames_file = open(os.path.join(rec.folder, "camera_frames.csv"), "w", newline="")
            self.frames_writer = csv.writer(self.frames_file)
            self.frames_writer.writerow(["frame", "t_s", "clock", "phase"])

    def draw(self, frame):
        stamp, light, sign = rec.vision
        picture = None
        try:
            fresh = time.monotonic() - stamp < 1.0
            picture = annotate_frame(frame, light if fresh else None, sign if fresh else None)
        except Exception:
            picture = frame.copy()
        status = rec.status
        top = f"t={rec.now():6.2f}s  {status.get('phase', '')}  leg {status.get('leg', '')}"
        lines = status.get("lines")
        middle = ("IR L/F/R " + "".join(str(int(v)) for v in lines)) if lines else "IR -"
        if status.get("yaw") is not None:
            middle += f"  yaw {status['yaw']:+.1f}  err {status.get('error') or 0:+.1f}"
        duty = status.get("duty")
        bottom = ("duty FL/FR/BL/BR " + "/".join(f"{d:.0f}" for d in duty)) if duty else ""
        cv2.rectangle(picture, (0, 0), (picture.shape[1], 54), (0, 0, 0), -1)
        for row, text in enumerate((top, middle, bottom)):
            cv2.putText(picture, text, (4, 14 + row * 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return picture

    def run(self):
        period = 1.0 / max(RECORD_CAMERA_FPS, 0.5)
        next_time = time.monotonic()
        while self.running:
            try:
                frame = self.camera.latest()
                if frame is not None:
                    if RECORD_DETECT and watcher is None:
                        data = get_vision_data(frame)
                        rec.note_vision(data["traffic_light_data"], data["stop_sign_data"])
                    picture = self.draw(frame)
                    ok, buffer = cv2.imencode(".jpg", picture, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if ok:
                        rec.latest_jpeg = buffer.tobytes()
                    if self.record and ok:
                        self.save(picture, buffer)
            except Exception as exc:                       # never let the picture take the run down
                print(f"\nCamera recording problem: {exc}")
                self.record = False
            next_time += period
            delay = next_time - time.monotonic()
            if delay < -1.0:
                next_time = time.monotonic()
            time.sleep(max(delay, 0.0))

    def save(self, picture, buffer):
        if self.writer is None and not self.jpg_mode:
            height, width = picture.shape[:2]
            path = os.path.join(rec.folder, "camera.avi")
            self.writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), max(RECORD_CAMERA_FPS, 0.5),
                                          (width, height))
            if not self.writer.isOpened():
                self.writer = None
                self.jpg_mode = True
                os.makedirs(os.path.join(rec.folder, "camera_frames"), exist_ok=True)
                rec.event("WARN", "Could not write camera.avi; saving camera_frames/*.jpg instead")
        if self.writer is not None:
            self.writer.write(picture)
        else:
            with open(os.path.join(rec.folder, "camera_frames", f"frame_{self.index:05d}.jpg"), "wb") as handle:
                handle.write(buffer.tobytes())
        self.frames_writer.writerow([self.index, f"{rec.now():.3f}", rec.clock(), rec.status.get("phase", "")])
        rec.video_frame = self.index
        self.index += 1

    def stop(self):
        self.running = False
        if self.is_alive():
            self.join(timeout=2.0)
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        if self.frames_file is not None:
            self.frames_file.close()
            self.frames_file = None


class RunRecorder:
    """Collects everything about one run into a folder. Every method does nothing while no
    recording is active, so the driving code can call it freely."""

    def __init__(self):
        self.active = False
        self.lock = threading.RLock()
        self.t0 = time.monotonic()
        self.folder = None
        self.name = None
        self.kind = ""
        self.latest_jpeg = None
        self.vision = (0.0, None, None)       # (time, light detection, sign detection) for the picture
        self.status = {}
        self.worker = None
        self.video_frame = ""
        self.last_zip = None
        self._reset()

    def _reset(self):
        self.phase_name = "IDLE"
        self.phase_start = 0.0
        self.phase_leg = 0
        self.leg = 0
        self.phases = []
        self.turns = []
        self.faults = []
        self.snapshots = 0
        self.samples = 0
        self.light = ""
        self.sign = 0
        self.tele_file = self.tele = self.event_file = self.events = None
        self.reset_path()

    def reset_path(self):
        """Start a new estimated path at 0,0 facing forward."""
        self.x = self.y = self.heading = 0.0
        self._ir_prev = (False, False, False)
        self.yaw_offset = 0.0              # absolute heading of the IMU's current zero (see RESET_YAW_AFTER_TURN)
        self.duty_seconds = 0.0
        self._last_tick = None
        self._last_point = 0.0
        self.path_points = []
        self.marks = []

    def live_path(self):
        with self.lock:
            points = self.path_points[-3000:]
            step = max(1, len(points) // 700)
            return {"pts": points[::step], "marks": list(self.marks), "cur": [round(self.x, 1), round(self.y, 1), round(self.heading, 1)]}

    # ---- time ----
    def now(self):
        return time.monotonic() - self.t0

    def timer(self):
        return "" if timer_start is None else f"{time.monotonic() - timer_start:.3f}"

    @staticmethod
    def clock():
        return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def path(self, name):
        return os.path.join(self.folder, name) if self.folder else name

    # ---- start and finish ----
    def start(self, kind):
        """Open a new run folder. Returns True when recording."""
        if self.active:
            self.finish("REPLACED")
        if not RECORD_RUNS:
            return False
        self._reset()
        self.name = time.strftime("%Y%m%d_%H%M%S") + "_" + kind
        self.folder = os.path.join(RUNS_DIR, self.name)
        try:
            os.makedirs(self.folder, exist_ok=True)
            self.tele_file = open(self.path("telemetry.csv"), "w", newline="")
            self.tele = csv.writer(self.tele_file)
            self.tele.writerow(TELEMETRY_COLUMNS)
            self.event_file = open(self.path("events.csv"), "w", newline="")
            self.events = csv.writer(self.event_file)
            self.events.writerow(EVENT_COLUMNS)
        except OSError as exc:
            print(f"Could not start recording ({exc}); the run goes ahead without it.")
            self._reset()
            return False
        self.kind = kind
        self.t0 = time.monotonic()
        self.started_at = time.strftime("%Y-%m-%d %H:%M:%S")
        self.latest_jpeg = None
        self.video_frame = ""
        self.active = True
        self.write_meta()
        self.event("START", f"Recording {kind} to {self.folder}")
        return True

    def write_meta(self):
        try:
            values = {row[1]: globals()[row[1]] for row in SETTINGS}
            meta = {
                "run": self.name, "kind": self.kind, "started": self.started_at,
                "profile": ACTIVE_PROFILE, "run_mode": RUN_MODE, "directions": list(directions),
                "robot": BOT_ID, "settings": values,
                "pins": {"motors_FL_FR_BL_BR_in1_in2_pwm": MOTOR_PINS, "stby": STBY,
                         "ir_front": IR_FRONT_PIN, "ir_left": IR_LEFT_PIN, "ir_right": IR_RIGHT_PIN,
                         "start_button": start_button, "reset_button": reset_button},
                "columns": COLUMN_NOTES,
            }
            with open(self.path("meta.json"), "w") as handle:
                json.dump(meta, handle, indent=2, default=str)
            with open(self.path("README.txt"), "w") as handle:
                handle.write("Robot run data. Copy this folder (or its .zip) to a PC and run:\n"
                             "    python3 plot_run.py <this folder or zip>\n\nColumns:\n")
                handle.write("\n".join(f"  {key}: {text}" for key, text in COLUMN_NOTES.items()) + "\n")
        except OSError as exc:
            print(f"Could not write meta.json: {exc}")

    def finish(self, result="FINISHED", detail=""):
        """Close the recording, write the summary and the zip. Returns the zip path or None."""
        if not self.active:
            return None
        if self.worker is not None:                       # stop the video first: it also writes to the event log
            self.worker.stop()
            frames = self.worker.index
            self.worker = None
        else:
            frames = 0
        with self.lock:
            ended = self.now()
            self._close_phase(ended)
            self.event("END", f"{result} {detail}".strip())
            self.active = False
            for handle in (self.tele_file, self.event_file):
                try:
                    handle.close()
                except (OSError, AttributeError):
                    pass
            legs = {}
            for item in self.phases:
                if item["leg"]:
                    legs.setdefault(item["leg"], 0.0)
                    legs[item["leg"]] += item["duration_s"]
            summary = {
                "run": self.name, "kind": self.kind, "result": result, "detail": detail,
                "started": self.started_at, "recorded_s": round(ended, 3),
                "course_time_s": None if timer_start is None else round(time.monotonic() - timer_start, 3),
                "directions": list(directions), "run_mode": RUN_MODE, "telemetry_rows": self.samples,
                "video_frames": frames,
                "seconds_per_leg": {str(k): round(v, 3) for k, v in legs.items()},
                "phases": self.phases, "turns": self.turns, "faults": self.faults,
                "power": dict(run_power),
            }
            try:
                with open(self.path("summary.json"), "w") as handle:
                    json.dump(summary, handle, indent=2, default=str)
                with open(self.path("log.txt"), "w") as handle:
                    handle.write("\n".join(run_events) + "\n")
            except OSError as exc:
                print(f"Could not write summary.json: {exc}")
        archive = zip_run(self.name)
        self.last_zip = archive
        if archive:
            print(f"\nRun data saved: {self.folder}/  and  {archive}")
            print(f"Copy it to another PC with:  scp {os.environ.get('USER', 'pi')}@{local_address()}:"
                  f"{os.path.abspath(archive)} .")
            if live_server is not None:
                print(f"Or download it in a browser: http://{local_address()}:{LIVE_VIEW_PORT}/runs/")
        return archive

    # ---- writing ----
    def _close_phase(self, ended):
        if self.phase_name != "IDLE":
            self.phases.append({"phase": self.phase_name, "leg": self.phase_leg,
                                "start_s": round(self.phase_start, 3), "end_s": round(ended, 3),
                                "duration_s": round(ended - self.phase_start, 3)})

    def phase(self, name):
        if not self.active or (name == self.phase_name and self.leg == self.phase_leg):
            return
        with self.lock:
            now = self.now()
            self._close_phase(now)
            self.phase_name, self.phase_start, self.phase_leg = name, now, self.leg
            self.event("PHASE", name)

    def set_leg(self, leg):
        if self.active:
            self.leg = leg

    def event(self, kind, message, **data):
        if kind in MARK_KINDS:
            self.marks.append([round(self.x, 1), round(self.y, 1), kind, message])
        if not self.active:
            return
        with self.lock:
            try:
                self.events.writerow([f"{self.now():.3f}", self.timer(), self.clock(), kind, self.phase_name,
                                      self.leg, message, json.dumps(data, default=str) if data else ""])
                self.event_file.flush()
            except (OSError, ValueError):
                pass

    def fault(self, kind, message):
        """A stop, reset or error: recorded with the phase and leg it happened in, plus a picture."""
        if not self.active:
            return
        self.faults.append({"kind": kind, "message": message, "t_s": round(self.now(), 3),
                            "phase": self.phase_name, "leg": self.leg})
        self.event(kind, message)
        self.snapshot(kind.lower())

    def snapshot(self, label):
        if self.active and self.latest_jpeg is not None and self.snapshots < 30:
            self.snapshots += 1
            try:
                with open(self.path(f"snap_{self.now():07.2f}_{label}.jpg"), "wb") as handle:
                    handle.write(self.latest_jpeg)
            except OSError:
                pass

    def note_vision(self, light, sign):
        """Called with the latest detections (dicts or None) whenever the code looks at the camera."""
        self.vision = (time.monotonic(), light, sign)
        color = light["color"] if light else "NONE"
        seen = 1 if sign else 0
        if self.active:
            if color != self.light:
                self.event("LIGHT", color)
            if seen != self.sign:
                self.event("SIGN", "seen" if seen else "gone")
        self.light, self.sign = color, seen

    def turn_done(self, direction, mode, set_seconds, started, ended, measured, target, duty):
        if not self.active:
            return
        record = {"leg": self.leg, "direction": direction, "mode": mode,
                  "set_duration_s": set_seconds, "start_s": round(started, 3), "end_s": round(ended, 3),
                  "took_s": round(ended - started, 3), "measured_deg": measured, "target_deg": target,
                  "duty": duty}
        self.turns.append(record)
        self.event("TURN", f"{direction} {mode} took {ended - started:.2f} s, measured "
                           f"{'n/a' if measured is None else format(measured, '.1f')} deg", **record)

    def attach_camera(self, camera):
        """Start the picture worker (video file and browser page) for an opened camera."""
        if camera is None or self.worker is not None:
            return
        if not (RECORD_CAMERA and self.active) and not LIVE_VIEW:
            return
        self.worker = CameraWorker(camera)
        self.worker.start()

    def tick(self, heading_target=None, error=None, trim=None, targets=None, imu=None, lines=None, tracker=None):
        """One telemetry row. Pass what the loop already knows; the rest is read here."""
        if power_guard is not None:
            power_guard.check(motors)                      # PowerFault stops the run on overcurrent / low battery
        power = battery_sample()
        if lines is None and mode_uses_ir():
            try:
                lines = read_lines()
            except Exception:
                lines = None
        if imu is None and tracker is not None:
            try:
                imu = tracker.get()
            except Exception:
                imu = None
        if error is None and heading_target is not None and imu is not None:
            error = imu[2] - heading_target
        duty = list(motors.duty)
        now_m = time.monotonic()
        dt = 0.0 if self._last_tick is None else min(now_m - self._last_tick, 0.25)
        self._last_tick = now_m
        if imu is not None:
            self.heading = imu[2] + self.yaw_offset
        elif heading_target is not None:
            self.heading = heading_target + self.yaw_offset
        mean_duty = sum(duty) / 4.0
        pattern = pattern_name(motors.pattern)
        self.x, self.y = path_step(self.x, self.y, self.heading, pattern, mean_duty, dt)
        if lines is not None:                                   # a marker on the map each time an IR sensor starts to see a line
            for hit, before, tag in zip(lines, self._ir_prev, ("IR_L", "IR_F", "IR_R")):
                if hit and not before:
                    self.marks.append([round(self.x, 1), round(self.y, 1), tag, ""])
            self._ir_prev = tuple(bool(v) for v in lines)
            if len(self.marks) > 3000:
                del self.marks[:500]
        if pattern == "FORWARD":
            self.duty_seconds += mean_duty * dt
        if now_m - self._last_point >= 0.1:
            self._last_point = now_m
            self.path_points.append([round(self.x, 1), round(self.y, 1)])
            if len(self.path_points) > 20000:
                del self.path_points[:5000]
        self.status = {"phase": self.phase_name, "leg": self.leg, "lines": lines, "duty": duty,
                       "x_cm": round(self.x), "y_cm": round(self.y),
                       "yaw": None if imu is None else imu[2] + self.yaw_offset, "error": error, "t_s": round(self.now(), 2),
                       "light": self.light, "stop_sign": self.sign,
                       **({} if power is None else {"battery_v": round(power.input_voltage, 2),
                                                    "current_a": round(power.current, 3)})}
        if not self.active:
            return
        raw = ("", "", "") if lines is None else tuple(
            LINE_DETECTED if line else 1 - LINE_DETECTED for line in lines)
        flags = ("", "", "") if lines is None else tuple(int(line) for line in lines)
        imu_cells = ("", "", "", "") if imu is None else tuple(
            f"{value + (self.yaw_offset if k == 2 else 0.0):.3f}" for k, value in enumerate(imu))
        aim = ("", "", "", "") if targets is None else tuple(f"{value:.1f}" for value in targets)

        def cell(value, digits=3):
            return "" if value is None else f"{value:.{digits}f}"

        row = [f"{self.now():.3f}", self.timer(), self.clock(), self.phase_name, self.leg,
               pattern_name(motors.pattern), *raw, *flags, *imu_cells,
               cell(None if heading_target is None else heading_target + self.yaw_offset),
               cell(error), cell(trim, 2), *aim,
               *(f"{value:.1f}" for value in duty), self.light, self.sign, self.video_frame,
               *(("", "", "") if power is None else (f"{power.input_voltage:.3f}", f"{power.current:.3f}",
                                                     f"{power.power:.3f}"))]
        with self.lock:
            try:
                self.tele.writerow(row)
                self.samples += 1
                if self.samples % 25 == 0:
                    self.tele_file.flush()
            except (OSError, ValueError):
                pass


rec = RunRecorder()
live_server = None
watcher = None


class VisionWatcher(threading.Thread):
    """Runs the traffic-light and stop-sign detectors on the newest camera picture in the background,
    so the driving code can ask "is the light green right now?" without waiting for a detector."""

    def __init__(self, camera):
        super().__init__(daemon=True)
        self.camera = camera
        self.running = True
        self.lock = threading.Lock()
        self.data = None
        self.data_time = 0.0
        self.green_time = self.nongreen_time = self.sign_time = -1e9
        self.nongreen_color = None
        self.failed = False

    def run(self):
        period = 1.0 / max(VISION_WATCH_FPS, 1.0)
        while self.running:
            started = time.monotonic()
            try:
                frame = self.camera.latest()
                if frame is not None:
                    data = get_vision_data(frame)
                    now = time.monotonic()
                    light = data["traffic_light_data"]
                    with self.lock:
                        self.data, self.data_time = data, now
                        color = light["color"] if light else None
                        if color == "GREEN":
                            self.green_time = now
                        elif color in ("RED", "YELLOW"):
                            self.nongreen_time = now
                            self.nongreen_color = color
                        if data["stop_sign"]:
                            self.sign_time = now
                    rec.note_vision(light, data["stop_sign_data"])
            except Exception as exc:
                if not self.failed:
                    self.failed = True
                    print(f"\nVision watcher problem: {exc}")
            time.sleep(max(period - (time.monotonic() - started), 0.01))

    def latest(self):
        """(newest detections, their age in seconds); (None, big) before the first result."""
        with self.lock:
            return self.data, time.monotonic() - self.data_time if self.data is not None else 1e9

    def green_ok(self, max_age):
        """Green was seen within max_age seconds and nothing red or yellow was seen after it."""
        with self.lock:
            return time.monotonic() - self.green_time <= max_age and self.green_time > self.nongreen_time

    def sign_recent(self, max_age):
        with self.lock:
            return time.monotonic() - self.sign_time <= max_age

    def stop_reason(self, light_age, sign_age):
        """'RED' / 'YELLOW' (seen within light_age s and no green since), 'SIGN' (stop sign within sign_age s) or None."""
        with self.lock:
            now = time.monotonic()
            if now - self.nongreen_time <= light_age and self.nongreen_time > self.green_time:
                return self.nongreen_color
            if now - self.sign_time <= sign_age:
                return "SIGN"
        return None

    def stop(self):
        self.running = False
        if self.is_alive():
            self.join(timeout=1.0)


def start_watcher(camera):
    global watcher
    stop_watcher()
    watcher = VisionWatcher(camera)
    watcher.start()


def stop_watcher():
    global watcher
    if watcher is not None:
        watcher.stop()
        watcher = None


def vision_now(camera):
    """Newest detections: the background watcher's if fresh, otherwise detect on the newest picture."""
    if watcher is not None:
        data, age = watcher.latest()
        if data is not None and age < 0.5:
            return data
    frame = camera.latest() if camera is not None else None
    return get_vision_data(frame) if frame is not None else None


def green_roll_check():
    """True when a STRAIGHT direction may drive over the line: no red/yellow light and no stop sign was just seen.
    Without the camera nothing can be seen, so the answer is yes."""
    if watcher is None:
        return True
    reason = watcher.stop_reason(GREEN_FRESH_TIME if USE_TRAFFIC_LIGHT else -1.0,
                                 SIGN_MEMORY_TIME if STOP_AT_SIGN_MIDCOURSE else -1.0)
    if reason:
        log(f"STRAIGHT: {reason.lower()} seen at the line, stopping")
        rec.event("STOP_REASON", f"STRAIGHT does not pass the line: {reason}")
    return reason is None


def pause_at_stop_sign():
    """Stopped at a line with a stop sign in view: stay still STOP_SIGN_PAUSE seconds."""
    if (STOP_AT_SIGN_MIDCOURSE and STOP_SIGN_PAUSE > 0 and watcher is not None
            and watcher.sign_recent(SIGN_MEMORY_TIME)):
        log(f"Stop sign at this line: waiting {STOP_SIGN_PAUSE:g} s")
        rec.event("SIGN", "Stop sign at an intersection line: pausing")
        rec.phase("STOP_SIGN_PAUSE")
        end = time.monotonic() + STOP_SIGN_PAUSE
        while time.monotonic() < end:
            check_reset()
            rec.tick()
            time.sleep(0.05)


def zip_run(name):
    """Zip runs/<name>/ into runs/<name>.zip. Returns the zip path or None."""
    folder = os.path.join(RUNS_DIR, name)
    archive = folder + ".zip"
    try:
        with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as bundle:
            for file_name in sorted(os.listdir(folder)):
                full = os.path.join(folder, file_name)
                if os.path.isfile(full):
                    bundle.write(full, os.path.join(name, file_name))
                elif os.path.isdir(full):
                    for inner in sorted(os.listdir(full)):
                        bundle.write(os.path.join(full, inner), os.path.join(name, file_name, inner))
        return archive
    except OSError as exc:
        print(f"Could not zip {folder}: {exc}")
        return None


################# the web page: menus, settings, tests, live camera, runs and the map ###################
# Open  http://<robot address>:<port>/  on any PC or phone on the same network. Looking is free; anything
# that changes a setting or moves the robot needs the WEB_PIN. The STOP button never needs it.
WEB_PIN = "robot"                  # EDIT: PIN the page asks for before it changes anything or moves the robot ("" = none)
SAFE_NAME = re.compile(r"^[A-Za-z0-9_.\-]+$")
web_jobs = queue.Queue()           # jobs from the page; the main thread runs them while it waits at a prompt
web_state = {"running": False, "job": "", "queued": "", "started": 0.0, "finished": 0.0, "result": ""}
web_stop = threading.Event()       # set by the page's STOP button; check_reset() turns it into a stop
web_camera = {"camera": None, "worker": None}
last_straight = {"seconds": 0.0, "duty_seconds": 0.0}
MARK_KINDS = ("LINE", "TURN", "FAULT", "ERROR", "RESET", "SIGN", "INTERRUPT")


class Tee:
    """Copies what a job prints to the terminal and keeps it for the web page."""

    def __init__(self, stream):
        self.stream = stream
        self.parts = []

    def write(self, text):
        self.stream.write(text)
        self.parts.append(text)
        return len(text)

    def flush(self):
        self.stream.flush()

    def text(self):
        return "".join(self.parts).replace("\r", "\n")[-6000:]


def process_web_jobs():
    """Run one job from the web page here, in the main thread (called while a prompt is waiting)."""
    if web_state["running"]:
        return
    try:
        name, args = web_jobs.get_nowait()
    except queue.Empty:
        return
    web_state.update(running=True, job=name, queued="", started=time.time(), result="")
    web_stop.clear()
    buffer = Tee(sys.stdout)
    try:
        with contextlib.redirect_stdout(buffer):
            WEB_JOBS[name](**args)
    except ResetRequested as exc:
        print(f"Stopped: {exc}")
    except CourseFault as exc:
        print(f"Stopped: {exc}")
    except ValueError as exc:
        print(f"Not done: {exc}")
    except KeyboardInterrupt:
        raise
    except Exception:
        print("ERROR: " + traceback.format_exc())
    finally:
        motors.stop()
        web_stop.clear()
        web_state.update(running=False, job="", finished=time.time(), result=buffer.text())


def stop_web_camera():
    worker, camera = web_camera["worker"], web_camera["camera"]
    web_camera["worker"] = web_camera["camera"] = None
    stop_watcher()
    if worker is not None:
        worker.stop()
    if camera is not None:
        camera.stop()
    if not rec.active:
        rec.latest_jpeg = None


def start_web_camera():
    if web_camera["camera"] is not None:
        return "The camera is already on"
    camera = CameraStream()
    if not camera.opened():
        camera.stop()
        return "The camera did not open (see the terminal for the reason)"
    camera.start()
    web_camera["camera"] = camera
    web_camera["worker"] = CameraWorker(camera)
    web_camera["worker"].start()
    start_watcher(camera)
    return ""


def apply_calibration(cm):
    """Turn a measured straight-test distance into the map's speed constant."""
    global MAP_CM_PER_SEC_PER_DUTY
    if last_straight["duty_seconds"] <= 0.5:
        raise ValueError("Run the straight test first (it measures the duty-seconds the map needs)")
    if cm <= 0:
        raise ValueError("The distance must be above 0 cm")
    MAP_CM_PER_SEC_PER_DUTY = round(min(max(cm / last_straight["duty_seconds"], 0.05), 20.0), 4)
    save_settings()
    return MAP_CM_PER_SEC_PER_DUTY


# ---- jobs (they run in the main thread) ----
def job_run(directions_text=None):
    stop_web_camera()
    if directions_text:
        directions[:] = parse_directions(directions_text)
    if not directions:
        raise ValueError("Enter the directions first")
    countdown()
    run_course()


def job_wheel_nudge():
    wheel_test(confirmed=True)


def job_wheel_run(wheel="FL", back=False):
    if wheel not in WHEEL_NAMES + ("ALL",):
        raise ValueError("wheel must be FL, FR, BL, BR or ALL")
    indexes = range(4) if wheel == "ALL" else [WHEEL_NAMES.index(wheel)]
    duty = min(TEST_DUTY, MAX_MOTOR_DUTY)
    print(f"{wheel} spinning {'backward' if back else 'forward'} at {duty:g} % (STOP, or 20 s)")
    motors.spin(indexes, duty, not back)
    end = time.monotonic() + 20.0
    while time.monotonic() < end and not web_stop.is_set():
        time.sleep(0.05)
    motors.stop()
    print("Stopped.")


def job_turn_test(direction="L"):
    turn_test(direction)


def job_straight_test(seconds=3.0):
    straight_test(float(seconds))


WEB_JOBS = {"run": job_run, "wheel_nudge": job_wheel_nudge, "wheel_run": job_wheel_run,
            "turn_test": job_turn_test, "straight_test": job_straight_test}


# ---- data for the page ----
def web_row(number):
    section, name, label, kind, low, high = SETTINGS[number - 1]
    value, default = globals()[name], default_value(name)
    return {"n": number, "section": section, "name": name, "label": label.strip(), "sub": label.startswith("  "),
            "kind": kind, "text": format_value(value, kind), "default": format_value(default, kind),
            "changed": value != default, "range": describe_range(kind, low, high),
            "options": [str(o) for o in high] if kind == "choice" else None}


def web_state_json():
    return {"running": web_state["running"], "job": web_state["job"], "queued": web_state["queued"],
            "result": web_state["result"], "finished": web_state["finished"], "recording": rec.active,
            "run": rec.name if rec.active else "", "profile": ACTIVE_PROFILE, "run_mode": RUN_MODE,
            "run_mode_label": RUN_MODE_LABELS[RUN_MODE], "robot": BOT_ID, "directions": list(directions),
            "camera_on": web_camera["camera"] is not None, "profiles": sorted(PROFILES, key=str.lower),
            "pin_needed": bool(WEB_PIN), "cm_per_duty": MAP_CM_PER_SEC_PER_DUTY,
            "duty_seconds": round(last_straight["duty_seconds"], 1), "time": time.strftime("%H:%M:%S"),
            "battery": battery_info()}


def web_ir():
    left, front, right = read_sensors()
    on = tuple(value == LINE_DETECTED for value in (left, front, right))
    trim = side_line_trim(on[0], on[2], on[1])
    duties = forward_targets(0.0, 0.0, trim)
    if on[1]:
        verdict = "STOP LINE (front sensor)"
    elif trim < 0:
        verdict = f"left line: steer RIGHT ({trim:+.1f})"
    elif trim > 0:
        verdict = f"right line: steer LEFT ({trim:+.1f})"
    elif on[0] or on[2]:
        verdict = "no steering (lines cancel, or lane steering is off)"
    else:
        verdict = "no line"
    return {"raw": [left, front, right], "on": list(on), "verdict": verdict, "duty": [round(duties[0]), round(duties[1])],
            "line_value": LINE_DETECTED, "problem": ir_pin_problem(),
            "pins": {"front": IR_FRONT_PIN, "left": IR_LEFT_PIN, "right": IR_RIGHT_PIN}}


def web_profile(action, name, replace=False):
    global ACTIVE_PROFILE
    if action in ("save", "new"):
        if not valid_profile_name(name):
            raise ValueError("Give a name of 1-24 letters, digits, spaces, - or _")
        existing = find_profile(name)
        if existing is not None and existing != ACTIVE_PROFILE and not replace:
            raise ValueError(f"Profile '{existing}' exists; tick replace to overwrite it")
        save_settings()
        if action == "new":
            apply_snapshot({"BOT_ID": BOT_ID})
        ACTIVE_PROFILE = existing or name
        save_settings()
    elif action in ("load", "delete"):
        found = find_profile(name)
        if found is None:
            raise ValueError("No such profile")
        if action == "load":
            save_settings()
            ACTIVE_PROFILE = found
            apply_snapshot(PROFILES[found])
            write_profiles()
        elif found == ACTIVE_PROFILE:
            raise ValueError("That profile is active; load another one first")
        else:
            del PROFILES[found]
            write_profiles()
    else:
        raise ValueError("unknown profile action")


def run_names():
    if not os.path.isdir(RUNS_DIR):
        return []
    return sorted((n for n in os.listdir(RUNS_DIR) if SAFE_NAME.match(n) and os.path.isdir(os.path.join(RUNS_DIR, n))),
                  reverse=True)


def read_json(path):
    try:
        with open(path) as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return {}


def read_rows(path):
    try:
        with open(path, newline="") as handle:
            return list(csv.DictReader(handle))
    except OSError:
        return []


def web_runs():
    runs = []
    for name in run_names():
        summary = read_json(os.path.join(RUNS_DIR, name, "summary.json"))
        archive = os.path.join(RUNS_DIR, name + ".zip")
        runs.append({"name": name, "kind": summary.get("kind", "?"), "result": summary.get("result", "?"),
                     "detail": summary.get("detail", ""), "started": summary.get("started", ""),
                     "course_time_s": summary.get("course_time_s"), "recorded_s": summary.get("recorded_s"),
                     "video": os.path.exists(os.path.join(RUNS_DIR, name, "camera.avi")) or
                              os.path.isdir(os.path.join(RUNS_DIR, name, "camera_frames")),
                     "zip_mb": round(os.path.getsize(archive) / 1e6, 1) if os.path.isfile(archive) else None})
    return runs


def num(text, default=None):
    try:
        return float(text)
    except (TypeError, ValueError):
        return default


def web_run_data(name, cm_per_duty=None):
    """Everything the run viewer needs: summary, a thinned telemetry table with the estimated path, events."""
    folder = os.path.join(RUNS_DIR, name)
    summary = read_json(os.path.join(folder, "summary.json"))
    meta = read_json(os.path.join(folder, "meta.json"))
    rows = read_rows(os.path.join(folder, "telemetry.csv"))
    if cm_per_duty is None:
        cm_per_duty = MAP_CM_PER_SEC_PER_DUTY
    phases = {}
    x = y = 0.0
    heading, last_t = 0.0, None
    table = []
    ir_marks, ir_prev = [], ("0", "0", "0")
    for row in rows:
        t = num(row.get("t_s"), 0.0)
        yaw = num(row.get("yaw_deg"))
        target = num(row.get("heading_target_deg"))
        heading = yaw if yaw is not None else (target if target is not None else heading)
        duty = [num(row.get(f"duty_{w}"), 0.0) for w in WHEEL_NAMES]
        if last_t is not None:
            x, y = path_step(x, y, heading, row.get("pattern", ""), sum(duty) / 4.0, min(t - last_t, 0.25), cm_per_duty)
        last_t = t
        flags = tuple(row.get(f"line_{side}", "") or "0" for side in ("left", "front", "right"))
        for now_flag, before, tag in zip(flags, ir_prev, ("IR_L", "IR_F", "IR_R")):
            if now_flag == "1" and before != "1":
                ir_marks.append([round(x, 1), round(y, 1), tag, round(t, 2)])
        ir_prev = flags
        phase = row.get("phase", "")
        phases.setdefault(phase, len(phases))
        table.append([round(t, 2), round(x, 1), round(y, 1), round(heading, 1), phases[phase],
                      "".join(row.get(f"line_{s}", "") or "-" for s in ("left", "front", "right")),
                      [round(d) for d in duty], row.get("light", ""), row.get("stop_sign", ""),
                      row.get("pattern", ""), num(row.get("heading_error_deg")),
                      num(row.get("battery_v")), num(row.get("current_a"))])
    step = max(1, len(table) // 1500)
    events = [[num(e.get("t_s"), 0.0), e.get("kind"), e.get("phase"), e.get("leg"), e.get("message")]
              for e in read_rows(os.path.join(folder, "events.csv")) if e.get("kind") != "PHASE"]
    return {"name": name, "summary": summary, "settings_cm": meta.get("settings", {}).get("MAP_CM_PER_SEC_PER_DUTY"),
            "cm_per_duty": cm_per_duty, "phase_names": list(phases),
            "cols": ["t", "x", "y", "heading", "phase", "ir", "duty", "light", "sign", "pattern", "error", "volt", "amp"],
            "rows": table[::step], "events": events[:800], "ir_marks": ir_marks[:3000]}


FRAME_INDEX = {}


def frame_index(name):
    if name not in FRAME_INDEX:
        FRAME_INDEX[name] = [(num(r["t_s"], 0.0), int(r["frame"])) for r in
                             read_rows(os.path.join(RUNS_DIR, name, "camera_frames.csv")) if r.get("frame", "").isdigit()]
    return FRAME_INDEX[name]


def run_frame_jpeg(name, t):
    """The camera picture nearest to time t of a saved run, as JPEG bytes (None if there is no video)."""
    folder = os.path.join(RUNS_DIR, name)
    index = frame_index(name)
    if not index:
        return None
    best = min(index, key=lambda item: abs(item[0] - t))[1]
    picture = os.path.join(folder, "camera_frames", f"frame_{best:05d}.jpg")
    if os.path.isfile(picture):
        with open(picture, "rb") as handle:
            return handle.read()
    video = os.path.join(folder, "camera.avi")
    if not os.path.isfile(video):
        return None
    capture = cv2.VideoCapture(video)
    try:
        capture.set(cv2.CAP_PROP_POS_FRAMES, best)
        ok, frame = capture.read()
    finally:
        capture.release()
    if not ok:
        return None
    ok, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return buffer.tobytes() if ok else None


WEB_PAGE = r"""<!doctype html><html lang=en><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>Robot control</title>
<style>
:root{--bg:#14171c;--card:#1d2229;--line:#303845;--tx:#e6e9ee;--mut:#8b95a5;--ac:#4aa3ff;--ok:#3ecf8e;--bad:#ff5d5d;--warn:#f2b84b}
*{box-sizing:border-box}a{color:var(--ac)}body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.45 system-ui,sans-serif}
header{position:sticky;top:0;z-index:5;background:#0e1115;border-bottom:1px solid var(--line);padding:8px 12px;display:flex;gap:8px;align-items:center;flex-wrap:wrap}
h1{font-size:16px;margin:0 8px 0 0}main{padding:12px;max-width:1100px;margin:auto}
nav{display:flex;gap:4px;flex-wrap:wrap;padding:8px 12px 0;max-width:1100px;margin:auto}
nav button{background:var(--card);color:var(--mut);border:1px solid var(--line);border-bottom:0;border-radius:6px 6px 0 0;padding:7px 14px;cursor:pointer}
nav button.on{color:var(--tx);background:#262d37;border-color:var(--ac)}
.tab{display:none}.tab.on{display:block}
.card{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:12px;margin-bottom:12px}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}.grow{flex:1}
button,input,select{font:inherit;color:var(--tx);background:#262d37;border:1px solid var(--line);border-radius:6px;padding:6px 10px}
button{cursor:pointer}button:hover{border-color:var(--ac)}button.go{background:#1f6f4a;border-color:var(--ok)}
button.stop{background:#8a2323;border-color:var(--bad);font-weight:700}button.warn{border-color:var(--warn)}
input[type=checkbox]{width:18px;height:18px}pre{background:#0b0d10;border:1px solid var(--line);border-radius:6px;padding:8px;overflow:auto;max-height:260px;margin:6px 0 0;white-space:pre-wrap}
.chip{padding:2px 9px;border-radius:99px;background:#262d37;border:1px solid var(--line);color:var(--mut)}
.chip.ok{color:var(--ok);border-color:var(--ok)}.chip.bad{color:var(--bad);border-color:var(--bad)}.chip.warn{color:var(--warn);border-color:var(--warn)}
img.cam{max-width:100%;width:640px;border:1px solid var(--line);border-radius:6px;background:#000;min-height:120px}
canvas{background:#0b0d10;border:1px solid var(--line);border-radius:6px;max-width:100%}
table{border-collapse:collapse;width:100%}td,th{padding:4px 8px;border-bottom:1px solid var(--line);text-align:left;vertical-align:top}
th{color:var(--mut);font-weight:600}h2{font-size:15px;margin:0 0 8px}h3{font-size:13px;color:var(--mut);margin:14px 0 4px;text-transform:uppercase;letter-spacing:.04em}
.set{display:grid;grid-template-columns:minmax(180px,1fr) 190px auto;gap:6px 10px;align-items:center;padding:4px 0;border-bottom:1px solid #262d37}
.set.sub .lab{padding-left:18px;color:var(--mut)}.set .lab small{display:block;color:var(--mut)}.set.changed .lab{color:var(--warn)}
.set input[type=text],.set select{width:100%}.dot{display:inline-block;width:46px;height:46px;border-radius:50%;background:#262d37;border:2px solid var(--line);text-align:center;line-height:42px;font-weight:700}
.dot.on{background:#1f6f4a;border-color:var(--ok)}#toast{position:fixed;right:12px;bottom:12px;max-width:420px;z-index:9}
#toast div{background:#0b0d10;border:1px solid var(--ok);border-radius:6px;padding:8px 12px;margin-top:6px}#toast .bad{border-color:var(--bad)}
@media(max-width:640px){.set{grid-template-columns:1fr}}
</style></head><body>
<header><h1>Robot</h1><span class=chip id=cState>...</span><span class=chip id=cProfile></span><span class=chip id=cMode></span><span class=chip id=cBat></span>
<span class=grow></span><input id=pin type=password placeholder="PIN" size=7 title="PIN for changes and moving the robot">
<button class=stop id=bStop>STOP</button></header>
<nav id=tabs></nav>
<main>
<section class=tab id=t-live>
 <div class=card><div class=row><button id=camOn>Camera on</button><button id=camOff>Camera off</button>
  <span style="color:var(--mut)">The camera is on by itself during a run. Switch it on here to aim it or check detections (robot stopped).</span></div>
  <div class=row style="margin-top:10px;align-items:flex-start"><img class=cam id=liveImg alt="camera (switch the camera on, or start a run)">
  <div class=grow><canvas id=liveMap width=420 height=320></canvas><div style="color:var(--mut)">Estimated path (start = 0,0; forward = up). Approximate: it comes from wheel duty and the IMU heading.</div></div></div>
  <pre id=liveStatus>waiting...</pre></div>
</section>
<section class=tab id=t-run>
 <div class=card><h2>Run the course</h2>
  <div class=row><label>Directions (L, R, S or U per intersection; RETURN last to come back)</label><input id=dirs size=24 placeholder="e.g. R S L R">
   <button id=setDirs>Set</button><span id=dirsNow class=chip></span></div>
  <div class=row style="margin-top:10px"><label>Run mode</label><select id=modeSel></select>
   <label>Robot preset</label><select id=botSel><option value=500rpm>500 RPM bot</option><option value=100rpm>100 RPM bot</option></select></div>
  <div class=row style="margin-top:14px"><button class=go id=bStart style="font-size:16px;padding:10px 26px">START</button>
   <span style="color:var(--mut)">Counts down, then runs. The physical START button still works. Put the robot on the start line first.</span></div>
  <pre id=jobOut>No job yet.</pre></div>
</section>
<section class=tab id=t-set>
 <div class=card><h2>Profiles</h2><div class=row><select id=profSel></select><button id=pLoad>Load</button><button id=pDel class=warn>Delete</button>
  <input id=pName placeholder="new profile name" size=18><button id=pSave>Save current as</button><button id=pNew>New from defaults</button>
  <label><input type=checkbox id=pRep> replace if it exists</label></div>
  <div style="color:var(--mut);margin-top:6px">Every change below is saved into the active profile straight away. Yellow = differs from the default; "restore" puts one setting back.</div></div>
 <div class=card id=setBox>loading...</div>
</section>
<section class=tab id=t-test>
 <div class=card><h2>IR sensors (live, no movement)</h2><div class=row id=irRow>
  <div style="text-align:center"><div class=dot id=irL>-</div>LEFT</div><div style="text-align:center"><div class=dot id=irF>-</div>FRONT</div>
  <div style="text-align:center"><div class=dot id=irR>-</div>RIGHT</div><div class=grow><div id=irVerdict></div><div id=irPins style="color:var(--mut)"></div></div></div></div>
 <div class=card><h2>Moving tests <span style="color:var(--mut);font-weight:400">(lift the robot for wheel tests; STOP ends any test)</span></h2>
  <div class=row><button id=tNudge>Wheel nudge (FL FR BL BR)</button></div>
  <div class=row style="margin-top:8px"><label>Spin a wheel</label><select id=wSel><option>FL</option><option>FR</option><option>BL</option><option>BR</option><option>ALL</option></select>
   <label><input type=checkbox id=wBack> backward</label><button id=tWheel>Spin (until STOP, 20 s max)</button></div>
  <div class=row style="margin-top:8px"><label>Turn test</label><button id=tTurnL>Left</button><button id=tTurnR>Right</button></div>
  <div class=row style="margin-top:8px"><label>Straight test, seconds</label><input id=sSec value=3 size=4><button id=tStraight>Drive straight</button></div>
  <div class=row style="margin-top:8px"><label>Map calibration: distance it travelled (cm)</label><input id=calCm size=6><button id=tCal>Set</button>
   <span id=calInfo style="color:var(--mut)"></span></div>
  <pre id=testOut>No test yet.</pre></div>
</section>
<section class=tab id=t-runs>
 <div class=card><h2>Saved runs</h2><div id=runList>loading...</div></div>
 <div class=card id=viewer style="display:none"><h2 id=vTitle></h2>
  <div class=row style="align-items:flex-start"><div><canvas id=runMap width=460 height=360></canvas>
   <div style="color:var(--mut);max-width:460px">Estimated path, from wheel duty and IMU heading (approximate). Drag the slider: the dot, the numbers and the camera picture move together.
   Distance scale <input id=vCm size=5 title="cm of travel per second per duty %"> cm/s per duty %</div></div>
   <div class=grow style="min-width:260px"><img class=cam id=vImg alt="no video"><input id=vSlide type=range min=0 max=0 value=0 style="width:100%;margin-top:8px">
    <pre id=vInfo></pre></div></div>
  <div class=row style="margin-top:8px"><a id=vZip>download .zip</a><a id=vReplay target=_blank>watch the video (replay)</a></div>
  <h3>Phases</h3><div id=vPhases></div><h3>Turns</h3><div id=vTurns></div><h3>Events</h3><div id=vEvents style="max-height:300px;overflow:auto"></div></div>
</section>
</main><div id=toast></div>
<script>
const $=s=>document.querySelector(s);
let pin=sessionStorage.getItem('pin')||'';$('#pin').value=pin;
$('#pin').onchange=e=>{pin=e.target.value;sessionStorage.setItem('pin',pin)};
function toast(m,bad){const d=document.createElement('div');if(bad)d.className='bad';d.textContent=m;$('#toast').appendChild(d);setTimeout(()=>d.remove(),bad?7000:3500)}
async function api(path,body){const o={};if(body!==undefined){o.method='POST';o.headers={'Content-Type':'application/json','X-Pin':pin};o.body=JSON.stringify(body)}
 const r=await fetch(path,o);let j={};try{j=await r.json()}catch(e){}if(!r.ok)throw new Error(j.error||r.statusText);return j}
async function act(path,body,ok){try{const j=await api(path,body);if(ok)toast(ok);return j}catch(e){toast(e.message,true)}}
const TABS=[['live','Live'],['run','Run'],['set','Settings'],['test','Tests'],['runs','Runs']];let tab='live';
TABS.forEach(([k,n])=>{const b=document.createElement('button');b.textContent=n;b.dataset.k=k;b.onclick=()=>show(k);$('#tabs').appendChild(b)});
function show(k){tab=k;document.querySelectorAll('.tab').forEach(e=>e.classList.toggle('on',e.id=='t-'+k));
 document.querySelectorAll('#tabs button').forEach(e=>e.classList.toggle('on',e.dataset.k==k));
 if(k=='set')loadSettings();if(k=='runs')loadRuns();location.hash=k;
 $('#liveImg').src=(k=='live')?'/stream?'+Date.now():''}
/* ---- map drawing ---- */
const MC={IR_F:'#ffd33d',IR_L:'#2ee6d6',IR_R:'#ff7ab2',LINE:'#f2b84b',TURN:'#c77dff',FAULT:'#ff5d5d',ERROR:'#ff5d5d',RESET:'#ff9f43',SIGN:'#ff6bd6',INTERRUPT:'#ff9f43'};
function drawMap(cv,pts,marks,cur){const g=cv.getContext('2d'),W=cv.width,H=cv.height;g.clearRect(0,0,W,H);
 if(!pts.length){g.fillStyle='#8b95a5';g.fillText('no path yet',10,20);return}
 let x0=0,x1=0,y0=0,y1=0;pts.forEach(p=>{x0=Math.min(x0,p[0]);x1=Math.max(x1,p[0]);y0=Math.min(y0,p[1]);y1=Math.max(y1,p[1])});
 const span=Math.max(x1-x0,y1-y0,40),pad=28,s=(Math.min(W,H)-2*pad)/span,cx=(x0+x1)/2,cy=(y0+y1)/2;
 const X=x=>W/2+(x-cx)*s,Y=y=>H/2-(y-cy)*s;
 g.strokeStyle='#222a33';g.lineWidth=1;const grid=span>400?100:span>150?50:10;
 for(let gx=Math.ceil((cx-W/2/s)/grid)*grid;gx<cx+W/2/s;gx+=grid){g.beginPath();g.moveTo(X(gx),0);g.lineTo(X(gx),H);g.stroke()}
 for(let gy=Math.ceil((cy-H/2/s)/grid)*grid;gy<cy+H/2/s;gy+=grid){g.beginPath();g.moveTo(0,Y(gy));g.lineTo(W,Y(gy));g.stroke()}
 g.fillStyle='#8b95a5';g.fillText(grid+' cm grid',6,H-6);
 g.strokeStyle='#4aa3ff';g.lineWidth=2;g.beginPath();pts.forEach((p,i)=>i?g.lineTo(X(p[0]),Y(p[1])):g.moveTo(X(p[0]),Y(p[1])));g.stroke();
 g.fillStyle='#3ecf8e';g.beginPath();g.arc(X(pts[0][0]),Y(pts[0][1]),6,0,7);g.fill();g.fillText('start',X(pts[0][0])+8,Y(pts[0][1])+4);
 (marks||[]).forEach(m=>{g.fillStyle=MC[m[2]]||'#fff';if(m[2].startsWith('IR_')){const r=m[2]=='IR_F'?4:3;g.fillRect(X(m[0])-r,Y(m[1])-r,2*r,2*r);return}g.beginPath();g.arc(X(m[0]),Y(m[1]),5,0,7);g.fill();g.fillStyle='#e6e9ee';g.fillText(m[2],X(m[0])+7,Y(m[1])-6)});
 if(marks&&marks.some(m=>m[2].startsWith('IR_'))){g.font='11px sans-serif';[['IR_F','front IR'],['IR_L','left IR'],['IR_R','right IR']].forEach((k,i)=>{g.fillStyle=MC[k[0]];g.fillRect(W-86,8+i*14,9,9);g.fillStyle='#e6e9ee';g.fillText(k[1],W-72,16+i*14)})}
 if(cur){g.save();g.translate(X(cur[0]),Y(cur[1]));g.rotate(-(cur[2]||0)*Math.PI/180);g.fillStyle='#fff';g.beginPath();g.moveTo(0,-11);g.lineTo(7,8);g.lineTo(-7,8);g.closePath();g.fill();g.restore()}}
/* ---- state, live ---- */
let st={};
async function pollState(){try{st=await api('/api/state');
 const c=$('#cState');c.textContent=st.running?('busy: '+st.job):(st.recording?'recording':'idle');c.className='chip '+(st.running?'warn':'ok');
 $('#cProfile').textContent='profile: '+st.profile;{const b=st.battery||{},c=$('#cBat');c.textContent=!b.on?'battery: off':(b.ok?b.v.toFixed(2)+' V  '+b.a.toFixed(2)+' A':'battery: no reading');c.title=b.text||'';c.className='chip '+(!b.on?'':(b.ok?(b.low?'bad':'ok'):'warn'))}$('#cMode').textContent=st.run_mode_label;$('#dirsNow').textContent=st.directions.length?st.directions.join(' '):'not set';
 if(document.activeElement!==$('#modeSel'))$('#modeSel').value=st.run_mode;
 $('#botSel').value=st.robot;$('#calInfo').textContent='now '+st.cm_per_duty+' cm/s per duty %; last straight test: '+st.duty_seconds+' duty-s';
 if(st.result||st.running){const t=st.running?('Running '+st.job+' ...\n'+(st.result||'')):st.result;$('#jobOut').textContent=t;$('#testOut').textContent=t}
 }catch(e){$('#cState').textContent='offline';$('#cState').className='chip bad'}}
async function pollLive(){if(tab!='live')return;try{const s=await (await fetch('/status')).json();$('#liveStatus').textContent=s.text;
 const p=await api('/api/path');drawMap($('#liveMap'),p.pts,p.marks,p.cur)}catch(e){}}
$('#bStop').onclick=()=>act('/api/stop',{},'Stop sent');
$('#camOn').onclick=()=>act('/api/camera',{on:true},'Camera starting');$('#camOff').onclick=()=>act('/api/camera',{on:false},'Camera off');
/* ---- run tab ---- */
$('#setDirs').onclick=()=>act('/api/directions',{text:$('#dirs').value},'Directions set').then(pollState);
$('#bStart').onclick=async()=>{const d=$('#dirs').value.trim();
 if(!st.directions.length&&!d){toast('Enter the directions first',true);return}
 if(!confirm('Start the run? The robot will move after the countdown.'))return;
 const j=await act('/api/job',{name:'run',args:d?{directions_text:d}:{}},'Run queued: it starts in a moment');pollState()};
$('#modeSel').onchange=async e=>{await act('/api/setting',{name:'RUN_MODE',value:e.target.value},'Run mode set');pollState()};
$('#botSel').onchange=async e=>{await act('/api/bot',{id:e.target.value},'Robot preset loaded');pollState()};
/* ---- settings ---- */
async function loadSettings(){const j=await api('/api/settings');const prof=await api('/api/state');
 const ps=$('#profSel');ps.innerHTML='';prof.profiles.forEach(n=>{const o=document.createElement('option');o.textContent=n;if(n==prof.profile)o.selected=true;ps.appendChild(o)});
 const box=$('#setBox');box.innerHTML='';let sec=null;
 const ms=$('#modeSel');if(!ms.options.length){j.rows.filter(r=>r.name=='RUN_MODE')[0].options.forEach(o=>{const e=document.createElement('option');e.value=e.textContent=o;ms.appendChild(e)})}
 j.rows.forEach(r=>{if(r.section!=sec){sec=r.section;const h=document.createElement('h3');h.textContent=sec;box.appendChild(h)}
  const d=document.createElement('div');d.className='set'+(r.sub?' sub':'');d.dataset.n=r.n;box.appendChild(d);fillRow(d,r)})}
function fillRow(d,r){d.classList.toggle('changed',r.changed);d.innerHTML='';
 const l=document.createElement('div');l.className='lab';l.textContent=r.label;const sm=document.createElement('small');sm.textContent=r.name+' | '+r.range+' | default '+r.default;l.appendChild(sm);d.appendChild(l);
 let inp;if(r.kind=='bool'){inp=document.createElement('input');inp.type='checkbox';inp.checked=r.text=='on'}
 else if(r.kind=='choice'){inp=document.createElement('select');r.options.forEach(o=>{const e=document.createElement('option');e.textContent=o;if(o==r.text)e.selected=true;inp.appendChild(e)})}
 else{inp=document.createElement('input');inp.type='text';inp.value=r.text}
 inp.onchange=async()=>{const v=r.kind=='bool'?(inp.checked?'on':'off'):inp.value;
  try{const j=await api('/api/setting',{name:r.name,value:v});fillRow(d,j.row);toast(r.name+' = '+j.row.text)}catch(e){toast(e.message,true);loadSettings()}};
 d.appendChild(inp);const b=document.createElement('button');b.textContent='restore';b.style.visibility=r.changed?'visible':'hidden';
 b.onclick=async()=>{try{const j=await api('/api/restore',{names:[r.name]});fillRow(d,j.rows[0]);toast(r.name+' restored')}catch(e){toast(e.message,true)}};d.appendChild(b)}
const prof=(action,name,extra)=>act('/api/profile',Object.assign({action,name},extra||{}),'Profile '+action+' done').then(()=>{loadSettings();pollState()});
$('#pLoad').onclick=()=>prof('load',$('#profSel').value);$('#pDel').onclick=()=>confirm('Delete this profile?')&&prof('delete',$('#profSel').value);
$('#pSave').onclick=()=>prof('save',$('#pName').value,{replace:$('#pRep').checked});$('#pNew').onclick=()=>prof('new',$('#pName').value,{replace:$('#pRep').checked});
/* ---- tests ---- */
async function pollIr(){if(tab!='test')return;try{const j=await api('/api/ir');
 [['irL',0],['irF',1],['irR',2]].forEach(([id,i])=>{const e=$('#'+id);e.textContent=j.raw[i];e.classList.toggle('on',j.on[i])});
 $('#irVerdict').textContent=j.verdict+' | left/right duty '+j.duty.join('/');
 $('#irPins').textContent='pins front '+j.pins.front+', left '+j.pins.left+', right '+j.pins.right+'. A sensor on a line reads '+j.line_value+(j.problem?'  WARNING: '+j.problem:'')}catch(e){}}
const job=(name,args,msg)=>()=>{if(!confirm(msg||'The robot will move. Continue?'))return;act('/api/job',{name,args:args?args():{}},'Queued').then(pollState)};
$('#tNudge').onclick=job('wheel_nudge',null,'Each wheel will nudge. Is the robot lifted?');
$('#tWheel').onclick=job('wheel_run',()=>({wheel:$('#wSel').value,back:$('#wBack').checked}),'The wheel will spin until STOP. Is the robot lifted?');
$('#tTurnL').onclick=job('turn_test',()=>({direction:'L'}),'The robot will spin in place.');$('#tTurnR').onclick=job('turn_test',()=>({direction:'R'}),'The robot will spin in place.');
$('#tStraight').onclick=job('straight_test',()=>({seconds:parseFloat($('#sSec').value)||3}),'The robot will drive straight. Clear floor ahead?');
$('#tCal').onclick=async()=>{const j=await act('/api/calibrate',{cm:parseFloat($('#calCm').value)},'Map calibrated');pollState()};
/* ---- runs ---- */
let RUN=null,SEL=0,frameTimer=null;
async function loadRuns(){const j=await api('/api/runs');const box=$('#runList');box.innerHTML='';
 if(!j.runs.length){box.textContent='No runs saved yet.';return}
 const t=document.createElement('table');t.innerHTML='<tr><th>Run<th>Result<th>Course time<th>Video<th></tr>';
 j.runs.forEach(r=>{const tr=document.createElement('tr');
  tr.innerHTML='<td></td><td></td><td></td><td></td><td></td>';tr.children[0].textContent=r.name;tr.children[1].textContent=r.result;
  tr.children[2].textContent=r.course_time_s==null?'':r.course_time_s+' s';tr.children[3].textContent=r.video?'yes':'';
  const b=document.createElement('button');b.textContent='open';b.onclick=()=>openRun(r.name);tr.children[4].appendChild(b);
  const a=document.createElement('a');a.href='/runs/'+r.name+'.zip';a.textContent=' zip'+(r.zip_mb?' ('+r.zip_mb+' MB)':'');tr.children[4].appendChild(a);t.appendChild(tr)});
 box.appendChild(t)}
async function openRun(name,cm){RUN=await api('/api/runs/'+name+'/data'+(cm?'?cm='+cm:''));$('#viewer').style.display='block';
 $('#vTitle').textContent=RUN.name+' - '+(RUN.summary.result||'')+' '+(RUN.summary.detail||'');$('#vCm').value=RUN.cm_per_duty;
 $('#vZip').href='/runs/'+name+'.zip';$('#vReplay').href='/api/runs/'+name+'/replay';
 const sl=$('#vSlide');sl.max=Math.max(RUN.rows.length-1,0);sl.value=0;SEL=0;renderRun();
 const S=RUN.summary,pt=$('#vPhases');pt.innerHTML='<table><tr><th>Start<th>Phase<th>Leg<th>Seconds</tr>'+(S.phases||[]).map(p=>`<tr><td>${p.start_s}<td>${p.phase}<td>${p.leg||''}<td>${p.duration_s}</tr>`).join('')+'</table>';
 $('#vTurns').innerHTML=(S.turns&&S.turns.length)?'<table><tr><th>Leg<th>Direction<th>Mode<th>Took (s)<th>Measured<th>Target</tr>'+S.turns.map(t=>`<tr><td>${t.leg}<td>${t.direction}<td>${t.mode}<td>${t.took_s}<td>${t.measured_deg??'n/a'}<td>${t.target_deg}</tr>`).join('')+'</table>':'none';
 const ev=$('#vEvents');ev.innerHTML='<table><tr><th>t<th>Kind<th>Phase<th>What</tr>'+RUN.events.map(e=>`<tr style="color:${MC[e[1]]||''}"><td>${e[0]}<td>${e[1]}<td>${e[2]}<td></tr>`).join('')+'</table>';
 [...ev.querySelectorAll('tr')].slice(1).forEach((tr,i)=>tr.lastChild.textContent=RUN.events[i][4]);
 $('#viewer').scrollIntoView()}
function runMarks(){const out=(RUN.ir_marks||[]).map(m=>[m[0],m[1],m[2]]);RUN.events.forEach(e=>{if(!MC[e[1]])return;let best=null;for(const r of RUN.rows){if(r[0]>=e[0]){best=r;break}}best=best||RUN.rows[RUN.rows.length-1];if(best)out.push([best[1],best[2],e[1]])});return out}
function renderRun(){if(!RUN||!RUN.rows.length){drawMap($('#runMap'),[],[],null);$('#vInfo').textContent='no telemetry in this run';return}
 const r=RUN.rows[SEL];drawMap($('#runMap'),RUN.rows.map(x=>[x[1],x[2]]),runMarks(),[r[1],r[2],r[3]]);
 $('#vInfo').textContent=`t ${r[0]} s   phase ${RUN.phase_names[r[4]]}\nIR L/F/R ${r[5]} (1 = line)\nheading ${r[3]} deg   error ${r[10]??'-'}\nduty FL/FR/BL/BR ${r[6].join('/')}   ${r[9]}\nlight ${r[7]||'-'}   stop sign ${r[8]||0}\nposition ${r[1]}, ${r[2]} cm\nbattery ${r[11]??'-'} V   ${r[12]??'-'} A`;
 clearTimeout(frameTimer);frameTimer=setTimeout(()=>{$('#vImg').src='/api/runs/'+RUN.name+'/frame?t='+r[0]},120)}
$('#vSlide').oninput=e=>{SEL=+e.target.value;renderRun()};
$('#vCm').onchange=e=>{const v=parseFloat(e.target.value);if(v>0)openRun(RUN.name,v)};
/* ---- start ---- */
show((location.hash||'#live').slice(1)||'live');loadSettings();pollState();setInterval(pollState,1000);setInterval(pollLive,1000);setInterval(pollIr,300);
</script></body></html>"""


_NO_CAMERA = []


def no_camera_jpeg():
    """A grey picture saying the camera is off, shown on the page until the first real frame arrives."""
    if not _NO_CAMERA:
        picture = np.full((240, 640, 3), 40, np.uint8)
        cv2.putText(picture, "Camera is off.", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
        cv2.putText(picture, "Press 'Camera on', or start a run.", (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (160, 160, 160), 2)
        _NO_CAMERA.append(cv2.imencode(".jpg", picture)[1].tobytes())
    return _NO_CAMERA[0]


class LiveHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, body, kind="text/html; charset=utf-8", code=200):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def json(self, data, code=200):
        self.reply(json.dumps(data, default=str), "application/json", code)

    def error(self, message, code=400):
        self.json({"error": message}, code)

    def send_file(self, path):
        size = os.path.getsize(path)
        self.send_response(200)
        self.send_header("Content-Type", mimetypes.guess_type(path)[0] or "application/octet-stream")
        self.send_header("Content-Length", str(size))
        self.send_header("Content-Disposition", f'attachment; filename="{os.path.basename(path)}"')
        self.end_headers()
        with open(path, "rb") as handle:
            shutil.copyfileobj(handle, self.wfile)

    # ---- GET ----
    def do_GET(self):
        try:
            parsed = urllib.parse.urlparse(self.path)
            path, query = urllib.parse.unquote(parsed.path), urllib.parse.parse_qs(parsed.query)
            if path == "/":
                self.reply(WEB_PAGE)
            elif path == "/stream":
                self.stream()
            elif path == "/status":
                s = dict(rec.status)
                info = battery_info()
                text = ("\n".join(f"{k}: {v}" for k, v in s.items()) + "\n\n" + (info.get("text", "") if info.get("on") else "")
                        + "\n\n" + "\n".join(recent_lines))
                self.json({"status": s, "text": text})
            elif path.startswith("/api/"):
                self.api_get(path[5:], query)
            elif path in ("/runs", "/runs/"):
                self.runs_page()
            elif path.startswith("/runs/"):
                self.run_file(path[len("/runs/"):].strip("/"))
            else:
                self.reply("not found", "text/plain", 404)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        except Exception as exc:
            try:
                self.error(f"{type(exc).__name__}: {exc}", 500)
            except OSError:
                pass

    def api_get(self, name, query):
        if name == "state":
            self.json(web_state_json())
        elif name == "settings":
            self.json({"rows": [web_row(n) for n in range(1, len(SETTINGS) + 1)]})
        elif name == "ir":
            self.json(web_ir())
        elif name == "path":
            self.json(rec.live_path())
        elif name == "runs":
            self.json({"runs": web_runs()})
        elif name.startswith("runs/"):
            parts = name.split("/")
            if len(parts) != 3 or not SAFE_NAME.match(parts[1]) or not os.path.isdir(os.path.join(RUNS_DIR, parts[1])):
                self.error("no such run", 404)
            elif parts[2] == "data":
                self.json(web_run_data(parts[1], num((query.get("cm") or [""])[0])))
            elif parts[2] == "frame":
                jpeg = run_frame_jpeg(parts[1], num((query.get("t") or ["0"])[0], 0.0))
                if jpeg is None:
                    self.error("no video in this run", 404)
                else:
                    self.reply(jpeg, "image/jpeg")
            elif parts[2] == "replay":
                self.replay(parts[1])
            else:
                self.error("not found", 404)
        else:
            self.error("not found", 404)

    # ---- POST ----
    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(length) or b"{}")
            name = urllib.parse.urlparse(self.path).path
            if not name.startswith("/api/"):
                return self.error("not found", 404)
            name = name[5:]
            if name != "stop" and WEB_PIN and not hmac.compare_digest(self.headers.get("X-Pin", ""), WEB_PIN):
                time.sleep(0.5)
                return self.error("Wrong or missing PIN (type it in the box at the top)", 403)
            self.api_post(name, body)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        except ValueError as exc:
            self.error(str(exc))
        except Exception as exc:
            self.error(f"{type(exc).__name__}: {exc}", 500)

    def busy(self):
        if web_state["running"] or rec.active:
            self.error("The robot is busy with a run or test; wait for it to finish (or press STOP)", 409)
            return True
        return False

    def api_post(self, name, body):
        if name == "stop":
            web_stop.set()
            motors.stop()
            return self.json({"ok": True})
        if name == "setting":
            if self.busy():
                return
            rows = [i for i, row in enumerate(SETTINGS, 1) if row[1] == body.get("name")]
            if not rows:
                return self.error("unknown setting")
            _, setting, _, kind, low, high = SETTINGS[rows[0] - 1]
            globals()[setting] = parse_setting(kind, low, high, str(body.get("value", "")))
            save_settings()
            return self.json({"row": web_row(rows[0])})
        if name == "restore":
            if self.busy():
                return
            names = body.get("names") or []
            numbers = [i for i, row in enumerate(SETTINGS, 1) if row[1] in names or names == ["ALL"]]
            for number in numbers:
                globals()[SETTINGS[number - 1][1]] = default_value(SETTINGS[number - 1][1])
            save_settings()
            return self.json({"rows": [web_row(n) for n in numbers]})
        if name == "profile":
            if self.busy():
                return
            web_profile(body.get("action", ""), str(body.get("name", "")).strip(), bool(body.get("replace")))
            return self.json({"ok": True})
        if name == "bot":
            if self.busy():
                return
            global BOT_ID
            BOT_ID = normalize_bot_id(str(body.get("id", "")))
            apply_preset(BOT_ID)
            save_settings()
            return self.json({"ok": True})
        if name == "directions":
            directions[:] = parse_directions(str(body.get("text", "")))
            return self.json({"directions": list(directions)})
        if name == "camera":
            if self.busy():
                return
            if body.get("on"):
                problem = start_web_camera()
                if problem:
                    return self.error(problem)
            else:
                stop_web_camera()
            return self.json({"ok": True})
        if name == "calibrate":
            return self.json({"cm_per_duty": apply_calibration(float(body.get("cm", 0)))})
        if name == "job":
            job = body.get("name")
            if job not in WEB_JOBS:
                return self.error("unknown job")
            if web_state["running"] or not web_jobs.empty():
                return self.error("Another job is running or waiting", 409)
            args = body.get("args") or {}
            web_state["queued"] = job
            web_jobs.put((job, args))
            return self.json({"queued": job})
        self.error("not found", 404)

    # ---- streams and files ----
    def stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        last = None
        while True:
            jpeg = rec.latest_jpeg if rec.latest_jpeg is not None else no_camera_jpeg()
            if jpeg is not last:
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                last = jpeg
            time.sleep(0.05)

    def replay(self, name):
        """Play a saved run's video as a picture stream (browsers cannot play the .avi directly)."""
        index = frame_index(name)
        if not index:
            return self.error("no video in this run", 404)
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        started = time.monotonic()
        for t, number in index:
            delay = t - (time.monotonic() - started)
            if delay > 0:
                time.sleep(min(delay, 2.0))
            jpeg = run_frame_jpeg(name, t)
            if jpeg is not None:
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")

    def runs_page(self):
        rows = [f'<li>{name} &nbsp; <a href="/runs/{name}.zip">download .zip</a> &nbsp; '
                f'<a href="/runs/{name}/">files</a></li>' for name in run_names()]
        self.reply("<!doctype html><meta charset=utf-8><body style='font-family:sans-serif;margin:16px'>"
                   "<h2>Saved runs</h2><ul>" + "".join(rows) + "</ul><p><a href='/'>Control page</a></p>")

    def run_file(self, rest):
        parts = rest.split("/")
        if not parts or not all(SAFE_NAME.match(p) and p not in (".", "..") for p in parts) or len(parts) > 3:
            self.reply("not found", "text/plain", 404)
            return
        if len(parts) == 1 and parts[0].endswith(".zip"):
            name = parts[0][:-4]
            archive = os.path.join(RUNS_DIR, parts[0])
            if name == rec.name and rec.active:
                self.reply("This run is still being recorded; its zip is made when it ends.", "text/plain", 409)
            elif os.path.isdir(os.path.join(RUNS_DIR, name)):
                if not os.path.isfile(archive):
                    zip_run(name)
                self.send_file(archive)
            else:
                self.reply("not found", "text/plain", 404)
            return
        target = os.path.join(RUNS_DIR, *parts)
        if os.path.isfile(target):
            self.send_file(target)
        elif os.path.isdir(target):
            links = "".join(f'<li><a href="/runs/{rest}/{n}">{n}</a></li>' for n in sorted(os.listdir(target))
                            if SAFE_NAME.match(n))
            self.reply(f"<!doctype html><meta charset=utf-8><body style='font-family:sans-serif;margin:16px'>"
                       f"<h2>{rest}</h2><ul>{links}</ul><p><a href='/runs/'>All runs</a></p>")
        else:
            self.reply("not found", "text/plain", 404)


class LiveServer(http.server.ThreadingHTTPServer):
    daemon_threads = True


def ensure_live_server():
    """Start the browser page once (live camera, status and run downloads). True when it is up."""
    global live_server
    if live_server is not None:
        return True
    try:
        live_server = LiveServer(("0.0.0.0", LIVE_VIEW_PORT), LiveHandler)
    except OSError as exc:
        print(f"Could not start the browser page on port {LIVE_VIEW_PORT}: {exc}")
        return False
    threading.Thread(target=live_server.serve_forever, daemon=True).start()
    print(f"Browser page: http://{local_address()}:{LIVE_VIEW_PORT}/   (read-only; anyone on this network can open it)")
    return True


def close_live_server():
    global live_server
    if live_server is not None:
        live_server.shutdown()
        live_server.server_close()
        live_server = None


def live_view_menu():
    """Menu option: the camera in a browser (no movement) and the run download page."""
    motors.stop(brake=False)
    if not ensure_live_server():
        return
    camera = CameraStream()
    worker = None
    if camera.opened():
        camera.start()
        worker = CameraWorker(camera)
        worker.start()
        print("Live camera view is on. Open the address above in a browser on your PC.")
    else:
        print("The camera did not open, so there is no picture, but the saved runs can still be downloaded at /runs/.")
        camera.stop()
    ask("Press Enter to stop the camera view (the download page stays up while this program runs): ")
    if worker is not None:
        worker.stop()
        camera.stop()
    rec.latest_jpeg = None


def saved_runs_menu():
    """List the saved runs, make any missing zip and show how to copy them to another PC."""
    if not os.path.isdir(RUNS_DIR):
        print("No runs saved yet (they appear in the runs folder after each run).")
        return
    names = sorted((n for n in os.listdir(RUNS_DIR) if os.path.isdir(os.path.join(RUNS_DIR, n))), reverse=True)
    if not names:
        print("No runs saved yet.")
        return
    address = local_address()
    print(f"\nSaved runs (newest first), in {os.path.abspath(RUNS_DIR)}:")
    for name in names[:15]:
        archive = os.path.join(RUNS_DIR, name + ".zip")
        if not os.path.isfile(archive):
            zip_run(name)
        size = os.path.getsize(archive) / 1e6 if os.path.isfile(archive) else 0.0
        print(f"  {name}   {size:.1f} MB")
    newest = os.path.abspath(os.path.join(RUNS_DIR, names[0] + ".zip"))
    print(f"\nCopy the newest to your PC:  scp {os.environ.get('USER', 'pi')}@{address}:{newest} .")
    print(f"Copy all runs:               scp -r {os.environ.get('USER', 'pi')}@{address}:{os.path.abspath(RUNS_DIR)} .")
    print(f"Or in a browser (menu option 12 starts the page): http://{address}:{LIVE_VIEW_PORT}/runs/")
    print("Then on the PC:  python3 plot_run.py <zip or folder>")


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
                   "S": "STRAIGHT", "STRAIGHT": "STRAIGHT", "F": "STRAIGHT",
                   "U": "UTURN", "UTURN": "UTURN", "U-TURN": "UTURN",
                   "RETURN": "RETURN", "RET": "RETURN", "BACK": "RETURN", "B": "RETURN"}
MIRROR = {"LEFT": "RIGHT", "RIGHT": "LEFT", "STRAIGHT": "STRAIGHT", "UTURN": "UTURN"}
DIRECTION_HELP = "L, R, S (straight) or U (U-turn) for each intersection; RETURN last to come back"


def parse_directions(text):
    """'R S L', 'rslr', 'right,straight,left' -> ['RIGHT', 'STRAIGHT', 'LEFT', ...]. Raises ValueError."""
    result = []
    for token in text.upper().replace(",", " ").split():
        if token in DIRECTION_NAMES:
            result.append(DIRECTION_NAMES[token])
        elif all(ch in "LRSFU" for ch in token):         # e.g. "RSLR"
            result.extend(DIRECTION_NAMES[ch] for ch in token)
        else:
            raise ValueError(f"Cannot read {token!r}: use {DIRECTION_HELP}, e.g. R S L")
    if not result:
        raise ValueError(f"No directions entered: use {DIRECTION_HELP}, e.g. R S L")
    check_return_position(result)
    return result


def check_return_position(items):
    """RETURN may only be the last entry and needs at least one direction before it."""
    if "RETURN" in items:
        if items.index("RETURN") != len(items) - 1:
            raise ValueError("RETURN must be the last entry (the robot finishes, then comes back)")
        if len(items) == 1:
            raise ValueError("RETURN needs at least one direction before it to retrace")


def check_reset():
    if web_stop.is_set():
        raise ResetRequested("Stopped from the web page")
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
            raise CourseFault(f"Unknown direction {item!r}; use LEFT, RIGHT, STRAIGHT, UTURN or RETURN")
        result.append(name)
    try:
        check_return_position(result)
    except ValueError as exc:
        raise CourseFault(str(exc))
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


def forward_targets(angle, rate, side_trim=0.0, speed=1.0):
    """Four duties (FL, FR, BL, BR) for straight driving.

    Heading correction (USE_HEADING_HOLD) and lane correction (side_trim) only ever slow one
    side down. With USE_WHEEL_SPEEDS on, each wheel starts from its own WHEEL_SPEEDS value.
    """
    hold = heading_correction(angle, rate) if USE_HEADING_HOLD else 0.0
    trim = max(-MAX_HEADING_TRIM, min(MAX_HEADING_TRIM, hold + side_trim))
    slow_left = max(trim, 0.0)           # positive trim slows the left side
    slow_right = max(-trim, 0.0)         # negative trim slows the right side
    if USE_WHEEL_SPEEDS:
        fl, fr, bl, br = (wheel * speed for wheel in WHEEL_SPEEDS)
        return (max(0.0, fl - slow_left), max(0.0, fr - slow_right),
                max(0.0, bl - slow_left), max(0.0, br - slow_right))
    scale = IMU_DRIVE_DUTY / max(LEFT_SPEED, RIGHT_SPEED)
    left = max(0.0, LEFT_SPEED * scale * speed - slow_left)
    right = max(0.0, RIGHT_SPEED * scale * speed - slow_right)
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


turn_ended_at = None            # monotonic time the last turn finished; cleared at the next line


def after_turn_slow_after():
    """Seconds from NOW until the slow-down starts after a turn, or None when AFTER_TURN_FAST_TIME is off."""
    if not USE_APPROACH_SLOW or AFTER_TURN_FAST_TIME <= 0 or turn_ended_at is None:
        return None
    return max(0.0, AFTER_TURN_FAST_TIME - (time.monotonic() - turn_ended_at))


def drive_to_line(tracker, heading_target, roll_through=None):
    """Steps 5 and 6. With IR sensors: drive until the front sensor sees the line, slowing down near it.

    roll_through: optional function; if it returns True when the line is reached the robot does NOT stop
    (STRAIGHT on a green light). IMU-only mode has no IR sensors, so it drives LEG_TIME seconds instead.
    Returns (a short name for the log, True if it rolled through without stopping).
    """
    if mode_uses_ir():
        slow = after_turn_slow_after()
        if slow is None:
            slow = APPROACH_SLOW_AFTER if (USE_APPROACH_SLOW and APPROACH_SLOW_AFTER > 0) else None
        global turn_ended_at
        rolled = drive_straight(tracker, heading_target, until_line=True, stop_at_end=False, slow_after=slow,
                                slow_percent=APPROACH_SPEED_PERCENT, roll_through=roll_through)
        turn_ended_at = None                                  # at a line: the "after a turn" timing is over
        return "Front line", rolled
    drive_straight(tracker, heading_target, seconds=LEG_TIME, lanes=False, phase="TIMED_LEG")
    return "Timed leg", False


def drive_to_finish(tracker, heading_target, camera):
    """Last leg: stop when the front IR sees a line AND the stop sign is seen.

    Returns True (sign seen), False (stopped without it) or None (no camera in this mode).
    With FINAL_REQUIRES_SIGN on, a line without the sign is not the end: the robot drives on to the next one.
    """
    started = time.monotonic()
    while True:
        what, _ = drive_to_line(tracker, heading_target)
        log(f"Final {what.lower()}: stopped")
        sign = confirm_stop_sign(camera)
        if sign is None or sign or not FINAL_REQUIRES_SIGN:
            return sign
        log("Front line but no stop sign here: driving on to the next line")
        if time.monotonic() - started > 2 * LEG_TIMEOUT:
            raise CourseFault("No front line with the stop sign found")


def drive_straight(tracker, heading_target, seconds=None, until_line=False, stop_at_end=True, lanes=True,
                   report_every=None, trace=None, phase=None,
                   slow_after=None, slow_percent=100.0, roll_through=None):
    """Forward drive with heading hold and lane steering. tracker=None drives without the IMU.

    seconds     : drive for this long and ignore lines (intersection approach and exit)
    until_line  : drive until the front sensor sees the line (steps 5 and 6; needs the IR sensors)
    stop_at_end : brake at the end; False keeps rolling into the next call
    lanes       : steer with the left and right lane sensors (off inside the intersection)
    report_every: log a status line this often, in seconds (default 2 s when looking for a line)
    trace       : optional list; (seconds, heading error) is appended every loop
    slow_after  : after this many seconds drive at slow_percent of the normal speed (0 = from the start)
    roll_through: function; True at the front line means keep going instead of stopping
    Returns True when it rolled through a front line without stopping.
    """
    if seconds is None and not until_line:
        raise ValueError("give seconds or until_line")
    if until_line and not mode_uses_ir():
        raise ValueError("until_line needs the IR sensors")
    rec.phase(phase or ("DRIVE_TO_LINE" if until_line else "DRIVE_TIMED"))
    motors.prepare(FORWARD)
    started = previous = time.monotonic()
    armed = False      # a front line only counts after the front sensor has been clear once
    rolled = False
    slowed = False
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
        now = time.monotonic()
        if tracker is not None:
            roll, pitch, yaw, rate = tracker.get()
            error = yaw - heading_target
            imu = (roll, pitch, yaw, rate)
        else:
            error = rate = 0.0                   # no IMU: no heading hold
            imu = None
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
                if roll_through is not None and roll_through():
                    rolled = True
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
        speed = 1.0
        if slow_after is not None and now - started >= slow_after:
            speed = max(0.0, slow_percent) / 100.0
            if not slowed:
                slowed = True
                log(f"Slowing to {slow_percent:.0f} % of the driving speed")
        targets = forward_targets(error, rate, lane_trim, speed)
        motors.slew(targets, RAMP_RATE, now - previous)
        previous = now
        rec.tick(heading_target, error, lane_trim, targets, imu=imu,
                 lines=(left, front, right) if mode_uses_ir() else None)
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
    if until_line:
        rec.event("LINE", "Front IR sees the line: " + ("green + STRAIGHT, rolling through" if rolled else "stopping"),
                  front_confirmed_reads=confirm)
    if (until_line and not rolled) or stop_at_end:
        motors.stop()
    return rolled


def turn(tracker, direction, heading_target):
    """Gyro-measured 90 degree turn (step 7). Returns the new planned heading."""
    if TURN_MODE == "timed" or tracker is None:         # no IMU means a timed turn
        return turn_timed(tracker, direction, heading_target)
    sign = 1 if direction == "LEFT" else -1
    pattern = TURN_LEFT if sign == 1 else TURN_RIGHT
    log(f"Turning {direction}")
    rec.phase("TURN")
    turn_began = rec.now()
    motors.prepare(pattern)
    start = previous = progress_time = time.monotonic()
    checkpoint = 0.0
    while True:
        check_reset()
        now = time.monotonic()
        roll, pitch, yaw, rate = tracker.get()
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
        rec.tick(heading_target, yaw - heading_target, None, targets, imu=(roll, pitch, yaw, rate))
        time.sleep(LOOP_DT)
    motors.stop()
    rec.phase("TURN_SETTLE")
    turn_ended = rec.now()
    settle = time.monotonic() + 0.25                   # let the robot finish coasting
    while time.monotonic() < settle:
        check_reset()
        tracker.get()
        rec.tick(heading_target, None, None, None, tracker=tracker)
        time.sleep(LOOP_DT)
    _, _, yaw, _ = tracker.get()
    rec.turn_done(direction, "imu", None, turn_began, turn_ended, round(sign * (yaw - heading_target), 2),
                  TURN_ANGLE, TURN_MAX_DUTY)
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
    rec.phase("TURN")
    turn_began = rec.now()
    yaw_before = tracker.get()[2] if tracker is not None else 0.0
    motors.prepare(pattern)
    targets = turn_targets(TURN_MAX_DUTY)
    start = previous = time.monotonic()
    while time.monotonic() - start < seconds:
        check_reset()
        now = time.monotonic()
        if tracker is not None:
            tracker.get()
        motors.slew(targets, 1000.0, max(now - previous, LOOP_DT))   # no ramp: a short turn needs full duty at once
        previous = now
        rec.tick(heading_target, None, None, targets, tracker=tracker)
        time.sleep(LOOP_DT)
    motors.stop()
    rec.phase("TURN_SETTLE")
    turn_ended = rec.now()
    settle = time.monotonic() + 0.25
    while time.monotonic() < settle:
        check_reset()
        if tracker is not None:
            tracker.get()
        rec.tick(heading_target, None, None, None, tracker=tracker)
        time.sleep(LOOP_DT)
    if tracker is None:
        rec.turn_done(direction, "timed", seconds, turn_began, turn_ended, None, TURN_ANGLE, TURN_MAX_DUTY)
        log(f"Turn done (timed {seconds:.2f} s; no IMU, so the angle is not measured)")
        return heading_target + sign * TURN_ANGLE
    yaw = tracker.get()[2]
    turned = sign * (yaw - yaw_before)
    rec.turn_done(direction, "timed", seconds, turn_began, turn_ended, round(turned, 2), TURN_ANGLE, TURN_MAX_DUTY)
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
    rec.phase("WAIT_LIGHT")
    started = time.monotonic()
    next_note = started + 2.0
    no_light = 0
    last_state = None
    yellow_since = None
    while True:
        check_reset()
        now = time.monotonic()
        if LIGHT_WAIT_TIMEOUT > 0 and now - started >= LIGHT_WAIT_TIMEOUT:
            log(f"Light is still {last_state or 'unreadable'} after {LIGHT_WAIT_TIMEOUT:g} s: going anyway")
            return
        data = vision_now(camera)
        if data is None:
            time.sleep(0.05)
            continue
        light = data["traffic_light_data"]                      # a stop sign never counts as a red light
        rec.note_vision(light, data["stop_sign_data"])
        state = light["color"] if light else "NONE"
        if state != last_state:
            log(f"Traffic light: {state}")
            last_state = state
            next_note = now + 2.0
        elif state != "NONE" and now >= next_note:
            log(f"Still waiting: light is {state} ({now - started:.0f} s)")
            next_note = now + 2.0
        rec.tick()
        if state == "GREEN":
            return
        if state == "YELLOW":
            yellow_since = yellow_since if yellow_since is not None else now
            if YELLOW_WAIT_TIME > 0 and now - yellow_since >= YELLOW_WAIT_TIME:
                log(f"Light has been yellow for {YELLOW_WAIT_TIME:g} s: going")
                rec.event("YELLOW_GO", f"Went after {YELLOW_WAIT_TIME:g} s of yellow")
                return
        else:
            yellow_since = None
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
    rec.phase("CONFIRM_SIGN")
    if watcher is not None and watcher.sign_recent(SIGN_MEMORY_TIME):
        rec.event("SIGN", "Stop sign seen just before the line")
        rec.snapshot("stop_sign")
        return True
    end = time.monotonic() + STOP_SIGN_CONFIRM_TIME
    while time.monotonic() < end:
        check_reset()
        data = vision_now(camera)
        if data is not None:
            rec.note_vision(data["traffic_light_data"], data["stop_sign_data"])
            rec.tick()
            if data["stop_sign"]:
                rec.event("SIGN", "Stop sign confirmed at the final line")
                rec.snapshot("stop_sign")
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
    parts["tracker"] = AngleTracker(parts["imu"],
                                    csv_path=rec.path("imu_200hz.csv") if rec.active else "angles.csv",
                                    time_zero=rec.t0 if rec.active else None)
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
    start_power_guard(parts)                           # INA219 must answer before anything is powered
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
            rec.attach_camera(parts["camera"])        # video file + browser picture
            start_watcher(parts["camera"])            # light + sign checks in the background
    else:
        log("Camera is not used in this mode")
    log("All systems OK")


def close_parts(parts):
    global power_guard
    if parts.get("guard") is not None:
        guard = parts["guard"]
        power_guard = None
        summary = guard.summary()
        if summary:
            log(summary)
        run_power.update(text=summary, peak_current_a=None if guard.peak_current is None else round(guard.peak_current, 3),
                         lowest_voltage_v=None if guard.min_voltage is None else round(guard.min_voltage, 3),
                         tripped=guard.trip_reason)
        guard.close()
        parts["guard"] = None
    power_guard = None
    stop_watcher()
    if parts.get("camera") is not None:
        parts["camera"].stop()
    if parts.get("tracker") is not None:
        parts["tracker"].stop()
    if parts.get("imu") is not None:
        parts["imu"].close()


def do_turn(tracker, direction, heading_target):
    """One 90 degree turn, then (RESET_YAW_AFTER_TURN) the IMU yaw is zeroed so the next leg starts at 0."""
    global turn_ended_at
    heading_target = turn(tracker, direction, heading_target)
    turn_ended_at = time.monotonic()
    if RESET_YAW_AFTER_TURN and tracker is not None:
        absolute = rec.yaw_offset + heading_target
        tracker.zero_yaw(absolute)
        rec.yaw_offset = absolute
        heading_target = 0.0
        rec.event("YAW_ZERO", f"IMU yaw re-zeroed after the turn (robot is at {absolute:.1f} deg from the start)")
    return heading_target


def u_turn(tracker, heading_target, side=None):
    """Turn around: two 90 degree turns the same way (UTURN_SIDE). Uses the timed/IMU turn settings."""
    side = side or ("LEFT" if str(UTURN_SIDE).upper().startswith("L") else "RIGHT")
    log(f"U-turn ({side})")
    rec.event("UTURN", f"U-turn to the {side.lower()}")
    heading_target = do_turn(tracker, side, heading_target)
    return do_turn(tracker, side, heading_target)


def run_leg(tracker, camera, heading_target, number, total, direction):
    """One intersection: drive to its line, wait for green, then go STRAIGHT, turn or U-turn. Returns the heading."""
    rolls = USE_ROLL_THROUGH and (direction == "STRAIGHT" or ROLL_THROUGH_TURNS)
    what, rolled = drive_to_line(tracker, heading_target,
                                 roll_through=green_roll_check if rolls else None)   # forward to the line
    slow = 0.0 if USE_APPROACH_SLOW else None
    if rolled and direction != "STRAIGHT":                           # green, turn: no stop, straight into the turn
        log(f"{what} {number}: no red light or stop sign, not stopping")
    elif rolled:                                                     # green and STRAIGHT: no stop
        log(f"{what} {number}: STRAIGHT, no red light or stop sign, driving over the line")
        log(f"Intersection {number} of {total}: {direction}")
        drive_straight(tracker, heading_target, seconds=STRAIGHT_TIME, stop_at_end=False, lanes=False,
                       phase="STRAIGHT_THROUGH", slow_after=slow, slow_percent=APPROACH_SPEED_PERCENT)
        return heading_target
    else:
        log(f"{what} {number}: stopped")
        pause_at_stop_sign()
        wait_for_green(camera)                                       # red or yellow: wait for green
    log(f"Intersection {number} of {total}: {direction}")
    if direction == "STRAIGHT":
        drive_straight(tracker, heading_target, seconds=STRAIGHT_TIME,
                       stop_at_end=False, lanes=False, phase="STRAIGHT_THROUGH")
        return heading_target
    side = direction if direction in ("LEFT", "RIGHT") else (
        "LEFT" if str(UTURN_SIDE).upper().startswith("L") else "RIGHT")
    approach = TURN_APPROACH_TIME_LEFT if side == "LEFT" else TURN_APPROACH_TIME_RIGHT
    drive_straight(tracker, heading_target, seconds=approach, lanes=False, phase="APPROACH",
                   slow_after=slow, slow_percent=APPROACH_SPEED_PERCENT)             # slow, into the area
    if direction == "UTURN":
        heading_target = u_turn(tracker, heading_target, side)
    else:
        heading_target = do_turn(tracker, direction, heading_target)                 # the turn
    drive_straight(tracker, heading_target, seconds=TURN_EXIT_STRAIGHT_TIME,
                   stop_at_end=False, phase="EXIT", slow_after=after_turn_slow_after(),
                   slow_percent=APPROACH_SPEED_PERCENT)                              # clear the intersection
    return heading_target


def return_path(forward):
    """The way back: the forward directions in reverse order with left and right swapped."""
    return [MIRROR[d] for d in reversed(forward)]


def run_return(tracker, camera, heading_target, forward):
    """After the finish: turn around, retrace every intersection the other way, stop at the start line."""
    back = return_path(forward)
    log("RETURN: " + ", ".join(back))
    rec.event("RETURN", "Returning: " + ", ".join(back))
    rec.phase("RETURN")
    heading_target = u_turn(tracker, heading_target)
    drive_straight(tracker, heading_target, seconds=RETURN_EXIT_TIME, stop_at_end=False,
                   lanes=False, phase="RETURN_EXIT")                                 # get off the finish line
    for count, direction in enumerate(back):
        rec.set_leg(len(forward) + 2 + count)
        heading_target = run_leg(tracker, camera, heading_target, count + 1, len(back), direction)
    rec.set_leg(len(forward) + 2 + len(back))
    what, _ = drive_to_line(tracker, heading_target)                                 # back to the start line
    log(f"Back at the start: {what.lower()} reached, stopped")
    return heading_target


def run_course():
    """One full run, from the start button to the stop sign."""
    global timer_start, turn_ended_at
    turn_ended_at = None
    timer_start = None
    del run_events[:]
    result, detail = "FINISHED", ""
    if not web_state["running"]:
        web_stop.clear()                                    # a STOP pressed while idle must not stop this run
    rec.reset_path()
    parts = {"imu": None, "tracker": None, "camera": None}
    try:
        rec.start("course")
        if rec.active and LIVE_VIEW:
            ensure_live_server()
        rec.phase("CHECK_SYSTEMS")
        log(f"Starting: checking systems")
        course = normalize_directions(directions)
        check_systems(parts)
        tracker, camera = parts["tracker"], parts["camera"]

        timer_start = time.monotonic()                      # start the timer
        log(f"Timer started. Directions: {', '.join(course)}")
        heading_target = 0.0
        rec.set_leg(1)
        if CHECK_LIGHT_AT_START:
            wait_for_green(camera)

        back_after = course[-1] == "RETURN"
        forward = course[:-1] if back_after else course
        for count, direction in enumerate(forward):         # one pass per direction
            rec.set_leg(count + 1)
            heading_target = run_leg(tracker, camera, heading_target, count + 1, len(forward), direction)

        rec.set_leg(len(forward) + 1)
        sign_seen = drive_to_finish(tracker, heading_target, camera)         # last line + stop sign
        if sign_seen is None:
            log("Stop sign not checked (no camera in this mode)")
        else:
            log("Front line and stop sign: finished" if sign_seen
                else "Stop sign NOT confirmed (stopped at the line anyway)")
        if back_after:
            forward_time = time.monotonic() - timer_start
            log(f"Forward part finished in {forward_time:.2f} s; now returning")
            heading_target = run_return(tracker, camera, heading_target, forward)

        timer = time.monotonic() - timer_start
        log(f"Course finished in {timer:.2f} s")
        print(f"Congratulations, you completed the course in a time of {timer:.2f} s")
        detail = f"course time {timer:.2f} s"
        rec.phase("DONE")
    except ResetRequested:
        log("Reset pressed: run aborted")
        result = "RESET"
        rec.fault("RESET", "Reset button pressed")
        while button_pressed(reset_button):                 # wait for release
            time.sleep(0.02)
    except CourseFault as exc:
        log(f"Stopped: {exc}")
        result, detail = "FAULT", str(exc)
        rec.fault("FAULT", str(exc))
    except KeyboardInterrupt:
        log("Interrupted (Ctrl+C, or the terminal was closed)")
        result = "INTERRUPT"
        rec.fault("INTERRUPT", "Ctrl+C or the terminal was closed")
        raise
    except Exception as exc:
        log(f"ERROR: {type(exc).__name__}: {exc}")
        result, detail = "ERROR", f"{type(exc).__name__}: {exc}"
        rec.fault("ERROR", traceback.format_exc())
        raise
    finally:
        motors.stop()
        close_parts(parts)
        rec.finish(result, detail)
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
    ("Heading hold", "RESET_YAW_AFTER_TURN", "Re-zero the IMU yaw after every turn: on / off", "bool", None, None),
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
    ("Navigation", "CHECK_LIGHT_AT_START", "Check the traffic light before the first move", "bool", None, None),
    ("Navigation", "USE_ROLL_THROUGH", "STRAIGHT: slow down, drive over the line, unless red/yellow light or stop sign", "bool", None, None),
    ("Navigation", "GREEN_FRESH_TIME", "   a red/yellow light seen this recently makes STRAIGHT stop, s", "float", 0.1, 10),
    ("Navigation", "STOP_AT_SIGN_MIDCOURSE", "Stop sign at a line before the end: stop and pause (STRAIGHT stops too)", "bool", None, None),
    ("Navigation", "STOP_SIGN_PAUSE", "   pause at a stop sign, s", "float", 0, 20),
    ("Navigation", "YELLOW_WAIT_TIME", "Yellow light: wait this long then go (0 = wait for green)", "float", 0, 30),
    ("Navigation", "USE_APPROACH_SLOW", "Slow down near lines and into the intersection", "bool", None, None),
    ("Navigation", "UTURN_SIDE", "U-turn spins to the: LEFT / RIGHT", "choice", None, ("LEFT", "RIGHT")),
    ("Navigation", "RETURN_EXIT_TIME", "After turning around for RETURN, drive straight for, s", "float", 0, 10),
    ("Navigation", "AFTER_TURN_FAST_TIME", "Full speed for this long after a turn, then slow, s (0 = off)", "float", 0, 60),
    ("Navigation", "APPROACH_SLOW_AFTER", "   start slowing this long into a drive, s (0 = off)", "float", 0, 60),
    ("Navigation", "APPROACH_SPEED_PERCENT", "   slowed speed, % of normal", "float", 10, 100),
    ("Navigation", "ROLL_THROUGH_TURNS", "Turns keep rolling at a line unless red/yellow light or stop sign", "bool", None, None),
    ("Navigation", "FINAL_REQUIRES_SIGN", "After the last direction: stop on front line + stop sign (off = line only)", "bool", None, None),
    ("Navigation", "SIGN_MEMORY_TIME", "   a sign seen this recently still counts, s", "float", 0, 10),
    ("Navigation", "VISION_WATCH_FPS", "Light/sign checks per second while driving", "float", 1, 30),
    ("Wheels", "REVERSED", "Reverse wheel direction (FL, FR, BL, BR)", "list_bool", None, None),
    ("Wheels", "WHEEL_FACTORS_FORWARD", "Wheel scale going forward (FL, FR, BL, BR)", "list_float", 0, 1),
    ("Wheels", "WHEEL_FACTORS_REVERSE", "Wheel scale going backward (FL, FR, BL, BR)", "list_float", 0, 1),
    ("Other", "BRAKE_ON_STOP", "Short-brake when stopping (off = coast)", "bool", None, None),
    ("Other", "MOTOR_TWITCH_CHECK", "Nudge each wheel during the start-up check", "bool", None, None),
    ("Other", "CAMERA_BACKEND", "Camera type", "choice", None, ("auto", "picamera2", "v4l2")),
    ("Other", "CAMERA_INDEX", "USB camera number", "int", 0, 9),
    ("Recording", "RECORD_RUNS", "Save the data of every run (csv files + zip)", "bool", None, None),
    ("Recording", "RECORD_CAMERA", "Save the camera picture of the run (camera.avi)", "bool", None, None),
    ("Recording", "RECORD_DETECT", "   (menu 12 only) detect on every picture; runs always detect", "bool", None, None),
    ("Recording", "RECORD_CAMERA_FPS", "   camera pictures per second (video and browser)", "float", 0.5, 15),
    ("Recording", "LIVE_VIEW", "Web page (menus, settings, camera, runs), starts with the program", "bool", None, None),
    ("Recording", "LIVE_VIEW_PORT", "   web page port (restart to change)", "int", 1024, 65535),
    ("Recording", "MAP_CM_PER_SEC_PER_DUTY", "Map scale: cm per second per 1 % duty", "float", 0.05, 20),
    ("Battery and power", "BATTERY_MONITOR", "INA219 battery monitor: voltage, current, power: on / off", "bool", None, None),
    ("Battery and power", "INA_BUS", "   INA219 I2C bus", "int", 0, 10),
    ("Battery and power", "INA_ADDRESS", "   INA219 address, decimal (64 = 0x40 ... 79 = 0x4F)", "int", 64, 79),
    ("Battery and power", "INA_SHUNT_OHMS", "   shunt resistor, ohm (0.01 = 10 milliohm)", "float", 0.001, 1),
    ("Battery and power", "POWER_GUARD", "Power guard: stop on overcurrent / low battery: on / off", "bool", None, None),
    ("Battery and power", "POWER_OVERCURRENT_A", "   overcurrent limit, A", "float", 0.1, 20),
    ("Battery and power", "POWER_OVERCURRENT_HOLD_S", "   ... for this long before stopping, s", "float", 0.05, 10),
    ("Battery and power", "POWER_LOW_VOLTAGE_V", "   low battery limit, V (about 3 V per cell)", "float", 0.5, 30),
    ("Battery and power", "POWER_LOW_VOLTAGE_HOLD_S", "   ... for this long before stopping, s", "float", 0.05, 30),
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
    """Prompt for a line of text. Jobs sent from the web page run here while this waits."""
    flush_stdin()
    print(prompt, end="", flush=True)
    while True:
        if not web_state["running"] and not web_jobs.empty():
            print()
            process_web_jobs()
            flush_stdin()
            print(prompt, end="", flush=True)
        if select.select([sys.stdin], [], [], 0.2)[0]:
            line = sys.stdin.readline()
            if line == "":
                raise KeyboardInterrupt                       # closed input stops the program cleanly
            return line.strip()


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


def wheel_test(confirmed=False):
    print(f"\nWheel test: each wheel spins forward for 0.15 s at {TEST_DUTY:g} %, in the order "
          "FL, FR, BL, BR. LIFT THE ROBOT so the wheels are free.")
    if not confirmed and ask("Type Y to run: ").upper() != "Y":
        return
    motors.twitch()
    print("Done. A wheel that spun the wrong way: flip it in Settings > Reverse wheel direction.")


def turn_test(direction=None):
    """Spin in place once with the current turn settings and print the measured angle."""
    if direction is None:
        answer = ask("\nTurn test: the robot spins in place. L or R (Enter cancels): ").upper()
        if answer not in ("L", "R"):
            return
        direction = "LEFT" if answer == "L" else "RIGHT"
    else:
        direction = "LEFT" if str(direction).upper().startswith("L") else "RIGHT"
    if not web_state["running"]:
        web_stop.clear()
    rec.reset_path()
    del run_events[:]
    parts = {"imu": None, "tracker": None, "camera": None}
    try:
        rec.start("turn_test")
        start_power_guard(parts)
        start_imu(parts)
        print("Turning in 2 s...")
        time.sleep(2.0)
        turn(parts["tracker"], direction, 0.0)
    except (CourseFault, ResetRequested) as exc:
        print(f"Stopped: {exc}")
        rec.fault("FAULT", str(exc))
    finally:
        motors.stop()
        close_parts(parts)
        rec.finish("DONE")


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


def offer_calibration():
    """After a straight test: turn the measured distance into the map's speed constant."""
    if web_state["running"] or last_straight["duty_seconds"] <= 0.5:
        return
    text = ask("For the map: how far did it travel, in cm? (Enter skips): ")
    if not text:
        return
    try:
        value = apply_calibration(float(text))
        print(f"Map scale set to {value:g} cm per second per 1 % duty (saved).")
    except ValueError as exc:
        print(f"Not changed: {exc}")


def straight_test(seconds=None):
    """Drive straight with the heading hold only (no lanes) and report how well it holds."""
    if seconds is None:
        text = ask("\nStraight test: the robot drives straight ahead. Clear floor in front, please. "
                   "Seconds (Enter = 3, C = cancel): ")
        if text.upper() == "C":
            return
        try:
            seconds = float(text) if text else 3.0
        except ValueError:
            print("Type a number of seconds.")
            return
    if not web_state["running"]:
        web_stop.clear()
    rec.reset_path()
    parts = {"imu": None, "tracker": None, "camera": None}
    trace = []
    try:
        rec.start("straight_test")
        start_power_guard(parts)
        start_imu(parts)
        print("Driving in 2 s...")
        time.sleep(2.0)
        try:
            drive_straight(parts["tracker"], 0.0, seconds=seconds, lanes=False, report_every=0.5, trace=trace)
        except CourseFault as exc:
            print(f"Stopped: {exc}")
        motors.stop()
        last_straight.update(seconds=seconds, duty_seconds=rec.duty_seconds)
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
        offer_calibration()
    except (CourseFault, ResetRequested) as exc:
        print(f"Stopped: {exc}")
        rec.fault("FAULT", str(exc))
    finally:
        motors.stop()
        close_parts(parts)
        rec.finish("DONE")


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
        prompt = "\nDirections, one L, R, S (straight) or U (U-turn) per intersection, RETURN last to come back (e.g. R S L R)"
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
        print(" 12  Live camera view + run download page (open it in a browser on your PC)")
        print(" 13  Saved runs: list them and show how to copy them to your PC")
        print(" 14  Battery readout (INA219 voltage, current, power, charge estimate)")
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
        elif choice == "12":
            live_view_menu()
        elif choice == "13":
            saved_runs_menu()
        elif choice == "14":
            battery_readout()
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
        if not web_state["running"] and not web_jobs.empty():
            process_web_jobs()
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
    ensure_battery(force=True)        # INA219 reader: telemetry, the web chip, the readout and the power guard
    if LIVE_VIEW:
        ensure_live_server()          # the web page: menus, settings, tests, camera, runs
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
        rec.finish("INTERRUPT")         # closes a recording that was still open
        close_live_server()
        close_battery()
        motors.close()                  # stops the wheels and calls GPIO.cleanup()


if __name__ == "__main__":
    main()

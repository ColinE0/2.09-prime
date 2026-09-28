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
# Module defaults; main.apply_preset() switches every module to the selected bot.
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

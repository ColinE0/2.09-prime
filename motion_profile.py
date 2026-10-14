"""Independent starting settings for the two 6 V N20 robots.

The 100 RPM menu label refers to the original motors rated 104 RPM. Neither
preset establishes physical wiring or replaces calibration on the robot.
"""

PROFILE_ID = "n20-500rpm-v1"
PRESET = {
    "left_speed": 25.0,
    "right_speed": 25.0,
    "auto_secondary_speed": 14.0,
    "diagonal_secondary_speed": 11.0,
    "auto_yaw_trim": 0.0,
    "diagonal_yaw_trim": 0.0,
    "ramp_rate": 35.0,
    "turn_ramp_rate": 35.0,
    "auto_slew_rate": 70.0,
    "fc_max_duty": 25.0,
    "fc_uncertain_duty": 20.0,
    "single_motor_duty": 25.0,
    "imu_drive_duty": 25.0,
    "imu_turn_max_duty": 25.0,
    "imu_turn_min_duty": 16.0,
    "imu_heading_kp": 0.6,
    "imu_heading_kd": 0.075,
    "imu_max_heading_trim": 5.0,
    "imu_turn_slow_gain": 0.4,
    "gyro_lane_ki": 0.15,
    # Bot 2's left switch (GPIO10, header pin 19) never reads pressed, so its START is the
    # right switch (GPIO9, pin 21) and RESET is off, as in Kaleb's v9.8.
    "button_start": "gpio9",
    "button_reset": "off",
}

BOT_IDS = ("500rpm", "100rpm")
BOT_LABELS = {"500rpm": "500 RPM bot", "100rpm": "100 RPM bot (104 RPM rated)"}
PROFILE_IDS = {"500rpm": PROFILE_ID, "100rpm": "n20-100rpm-v1"}
PRESETS = {
    "500rpm": PRESET,
    "100rpm": {
        "left_speed": 70.0,
        "right_speed": 69.0,
        "auto_secondary_speed": 40.0,
        "diagonal_secondary_speed": 30.0,
        "auto_yaw_trim": 0.0,
        "diagonal_yaw_trim": 0.0,
        "ramp_rate": 100.0,
        "turn_ramp_rate": 100.0,
        "auto_slew_rate": 200.0,
        "fc_max_duty": 50.0,
        "fc_uncertain_duty": 30.0,
        "single_motor_duty": 35.0,
        "imu_drive_duty": 50.0,
        "imu_turn_max_duty": 50.0,
        "imu_turn_min_duty": 32.0,
        "imu_heading_kp": 1.2,
        "imu_heading_kd": 0.15,
        "imu_max_heading_trim": 10.0,
        "imu_turn_slow_gain": 0.8,
        "gyro_lane_ki": 0.3,
        "button_start": "gpio10",  # bot 1: START on the left switch worked on 2026-10-05
        "button_reset": "gpio9",
    },
}


def normalize_bot_id(bot_id):
    """Canonical menu/CLI ID; reject unknown robots rather than guessing."""
    aliases = {"500": "500rpm", "100": "100rpm"}
    if not isinstance(bot_id, str):
        raise ValueError("Select a robot: 500rpm or 100rpm")
    bot_id = aliases.get(bot_id.strip().lower(), bot_id.strip().lower())
    if bot_id not in BOT_IDS:
        raise ValueError("Select a robot: 500rpm or 100rpm")
    return bot_id


def settings_bot(config, bot_id=None):
    """Infer a tagged robot, and reject contradictory explicit identities."""
    saved_bot = config.get("robot_id")
    saved_profile = config.get("motion_profile")
    if saved_bot is not None and (not isinstance(saved_bot, str) or saved_bot not in BOT_IDS):
        raise ValueError("Unknown robot_id in settings")
    reverse_profiles = {value: key for key, value in PROFILE_IDS.items()}
    if saved_profile is not None and (not isinstance(saved_profile, str) or saved_profile not in reverse_profiles):
        raise ValueError("Unknown motion_profile in settings")
    selected = normalize_bot_id(bot_id) if bot_id is not None else (
        saved_bot or reverse_profiles.get(saved_profile) or "500rpm")
    if saved_bot is not None and saved_bot != selected:
        raise ValueError(f"Settings belong to {saved_bot}, not selected robot {selected}")
    if saved_profile is not None and saved_profile != PROFILE_IDS[selected]:
        raise ValueError(f"motion_profile {saved_profile} does not match selected robot {selected}")
    return selected

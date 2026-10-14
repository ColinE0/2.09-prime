"""Headless camera and action-size settings for Ryan's September 19 rr code."""

# Preserve the existing CSI IMX219 mounting/exposure, using Ryan's 640x480
# resolution because his shape checks include pixel-based minimum sizes.
CAMERA_BACKEND = "csi"
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 20
CAMERA_ROTATE_180 = True
CAMERA_SENSOR_SIZE = (1640, 1232)
CAMERA_EXPOSURE_US = 4000
CAMERA_ANALOGUE_GAIN = 1.0
VISION_PROCESS_HZ = 10.0
DETECT_PROC_WIDTH = 640

# Ryan's detector thresholds and regions live in ryan_vision/*.py, unchanged.
# Do not apply v4's old LIGHT_V_MIN / LAMP_GLOW / area overrides to this code.

# Bounding-box fractions a detection must reach to count, not a distance.
# Profiles override these defaults through SET -> Camera.
# Colin requested 0.04% for lights on 2026-10-06: fraction 0.0004.
STOPSIGN_ACT_AREA_FRAC = 0.012
LIGHT_ACT_AREA_FRAC = 0.0004

"""Rev C board nets. Physical wheel/sensor roles must be checked on the robot."""

CHANNEL_PINS = {
    "M1A": (17, 27, 18),
    "M1B": (22, 23, 25),
    "M2A": (5, 6, 12),
    "M2B": (16, 19, 13),
}
# Documented rev C map, from the fab files and the engraved base plate pockets, in the
# robot's own left/right (stand behind it): FL J2 = M1A, FR J4 = M1B, BL J12 = M2B,
# BR J5 = M2A. IR: LEFT J7 = 21, RIGHT J10 = 7, FRONT J9 = 26. Confirm on each robot.
DOCUMENTED_CHANNELS = ("M1A", "M1B", "M2B", "M2A")
DOCUMENTED_IR_PINS = (21, 7, 26)
DEFAULT_CHANNELS = DOCUMENTED_CHANNELS


def validate_channels(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 4
            or any(type(item) is not str or item not in CHANNEL_PINS for item in value)
            or len(set(value)) != 4):
        raise ValueError("motor_channels needs M1A, M1B, M2A, M2B exactly once, in FL, FR, BL, BR order")
    return tuple(value)


def channels_from_pins(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 4
            or any(not isinstance(pins, (list, tuple)) or len(pins) != 3
                   or any(type(pin) is not int for pin in pins) for pins in value)):
        raise ValueError("MOTOR_PINS must contain four Rev C channel pin triples")
    lookup = {pins: name for name, pins in CHANNEL_PINS.items()}
    try:
        return validate_channels(tuple(lookup[tuple(pins)] for pins in value))
    except KeyError as exc:
        raise ValueError("MOTOR_PINS contains an unknown Rev C channel") from exc

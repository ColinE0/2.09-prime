import time


# ============================================================
# Sensor
# ============================================================

# VL53L0X on the Pi's I2C bus 1 at its default address.
TOF_I2C_BUS = 1
TOF_ADDRESS = 0x29

# About 30 readings per second. One reading blocks for up to
# MEASUREMENT_TIMEOUT, so a stuck sensor cannot freeze the loop.
TIMING_BUDGET_US = 33000
MEASUREMENT_TIMEOUT = 0.20


# ============================================================
# Distance thresholds
# ============================================================

HOLD_MM = 250          # Obstacle: stop at or under 25 cm
RESUME_MM = 300        # Clear again only past 30 cm (hysteresis)
SLOW_MM = 450          # Slow down between 25 and 45 cm
MAX_VALID_MM = 2000    # Farther than this counts as open lane

MIN_SPEED_SCALE = 0.40  # Slowest speed while slowing down


# ============================================================
# Clear confirmation
# ============================================================

# After an obstacle, the path must read clear this long,
# over at least this many readings, before driving again.
CLEAR_SECONDS = 0.75
CLEAR_MIN_READINGS = 3

# A gap longer than this between readings restarts the count.
CLEAR_MAX_GAP = 0.15

# A reading the sensor cannot explain for this long is a fault.
UNKNOWN_FAULT_SECONDS = 1.0


# ============================================================
# VL53L0X range status codes
# ============================================================

# 11 = range complete.
RANGE_COMPLETE = 11

# Nothing returned within range. This is the normal reading
# down an open lane, not an error:
# 4 no target, 6 and 9 phase checks, 14 below threshold.
NO_TARGET_STATUSES = {4, 6, 9, 14}

# Something inside the sensor's minimum range:
# 8 and 10.
TOO_CLOSE_STATUSES = {8, 10}


tof_i2c = None
tof_sensor = None

# Hold / resume state, kept between readings.
tof_holding = True
tof_clear_since = None
tof_clear_count = 0
tof_unknown_since = None
tof_last_time = None


def setup_tof():
    """Start the ToF sensor once."""
    global tof_i2c, tof_sensor

    if tof_sensor is not None:
        return

    # Install on the Pi first:
    # pip install adafruit-circuitpython-vl53l0x adafruit-extended-bus
    import adafruit_vl53l0x
    from adafruit_extended_bus import ExtendedI2C

    tof_i2c = ExtendedI2C(TOF_I2C_BUS)

    try:

        tof_sensor = adafruit_vl53l0x.VL53L0X(
            tof_i2c,
            address=TOF_ADDRESS,
            io_timeout_s=MEASUREMENT_TIMEOUT
        )

        # End any old continuous ranging, and throw away
        # the single reading that stop_continuous() starts.
        tof_sensor.stop_continuous()
        tof_sensor.read_range()

        tof_sensor.measurement_timing_budget = TIMING_BUDGET_US

    except BaseException:

        stop_tof()
        raise


def read_distance():
    """
    Take one reading and return (distance_mm, status).

    The library's range property hides the range status,
    and the status is what separates an open lane from a
    sensor error, so this reads the result registers.
    """

    # Clear the old ready flag before starting a new reading,
    # so an old value is never returned as a fresh one.
    tof_sensor._write_u8(0x0B, 1)
    tof_sensor._data_ready = False

    started = time.monotonic()

    tof_sensor.do_range_measurement()

    while not tof_sensor.data_ready:

        if time.monotonic() - started >= MEASUREMENT_TIMEOUT:
            raise TimeoutError("VL53L0X reading timed out")

        time.sleep(0.002)

    result = bytearray(12)

    with tof_sensor._device as device:
        device.write(bytes((0x14,)))
        device.readinto(result)

    tof_sensor._write_u8(0x0B, 1)
    tof_sensor._data_ready = False

    status = (result[0] & 0x78) >> 3
    distance_mm = (result[10] << 8) | result[11]

    return distance_mm, status


def classify_reading(distance_mm, status):
    """Return RANGE, CLEAR, TOO_CLOSE or UNKNOWN."""

    if status == RANGE_COMPLETE:

        if 0 < distance_mm <= MAX_VALID_MM:
            return "RANGE"

        if distance_mm > MAX_VALID_MM:
            return "CLEAR"

        return "UNKNOWN"

    if status in NO_TARGET_STATUSES:
        return "CLEAR"

    if status in TOO_CLOSE_STATUSES:
        return "TOO_CLOSE"

    return "UNKNOWN"


def reset_tof_state():
    """Start over as if an obstacle was just seen."""
    global tof_holding, tof_clear_since, tof_clear_count
    global tof_unknown_since, tof_last_time

    tof_holding = True
    tof_clear_since = None
    tof_clear_count = 0
    tof_unknown_since = None
    tof_last_time = None


# -----------------------------------------------------------
# WHAT NAVIGATION NEEDS TO KNOW FROM THE TOF
# -----------------------------------------------------------


def get_tof_data(distance_mm, status, now=None):
    """
    Main interface between the ToF sensor and navigation.

    Turns one reading into obstacle / slow-down decisions.
    Works alongside read_vision(): stop if obstacle is True
    OR vision sees a red light, yellow light or stop sign,
    and multiply drive speed by speed_scale.
    """
    global tof_holding, tof_clear_since, tof_clear_count
    global tof_unknown_since, tof_last_time

    if now is None:
        now = time.monotonic()

    reading = classify_reading(distance_mm, status)

    # A long gap between readings restarts the clear count.
    if tof_last_time is not None and now - tof_last_time > CLEAR_MAX_GAP:
        tof_clear_since = None
        tof_clear_count = 0

    tof_last_time = now

    tof_data = {
        "distance_mm": distance_mm,
        "distance_cm": None,
        "status": status,
        "reading": reading,

        "obstacle": True,
        "slow_down": False,
        "speed_scale": 0.0,

        "sensor_fault": False
    }

    # ---------------------------------------------------------
    # UNKNOWN READING
    # ---------------------------------------------------------
    # Never drive on a reading the sensor cannot explain.
    # If it lasts, report a fault so navigation can stop.
    # ---------------------------------------------------------

    if reading == "UNKNOWN":

        tof_holding = True
        tof_clear_since = None
        tof_clear_count = 0

        if tof_unknown_since is None:
            tof_unknown_since = now

        if now - tof_unknown_since >= UNKNOWN_FAULT_SECONDS:
            tof_data["sensor_fault"] = True

        return tof_data

    tof_unknown_since = None

    # ---------------------------------------------------------
    # DISTANCE
    # ---------------------------------------------------------
    # CLEAR means nothing within range: treat it as far away.
    # TOO_CLOSE means something is touching close: distance 0.
    # ---------------------------------------------------------

    if reading == "RANGE":
        distance = distance_mm
        tof_data["distance_cm"] = distance_mm / 10

    elif reading == "TOO_CLOSE":
        distance = 0

    else:
        distance = float("inf")

    if distance <= HOLD_MM:
        tof_holding = True

    # ---------------------------------------------------------
    # RESUME AFTER AN OBSTACLE
    # ---------------------------------------------------------

    if tof_holding:

        if distance > RESUME_MM:

            if tof_clear_since is None:
                tof_clear_since = now

            tof_clear_count += 1

            if (
                tof_clear_count >= CLEAR_MIN_READINGS
                and now - tof_clear_since >= CLEAR_SECONDS
            ):
                tof_holding = False

        else:
            tof_clear_since = None
            tof_clear_count = 0

    if tof_holding:
        return tof_data

    # ---------------------------------------------------------
    # DRIVE, SLOWING DOWN INSIDE SLOW_MM
    # ---------------------------------------------------------

    tof_data["obstacle"] = False

    if distance < SLOW_MM:

        tof_data["slow_down"] = True

        tof_data["speed_scale"] = (
            MIN_SPEED_SCALE
            + (1 - MIN_SPEED_SCALE)
            * (distance - HOLD_MM) / (SLOW_MM - HOLD_MM)
        )

    else:
        tof_data["speed_scale"] = 1.0

    return tof_data


def read_tof():
    """
    Take one reading and return all ToF results.

    Navigation only needs to call this function.
    """

    setup_tof()

    distance_mm, status = read_distance()

    return get_tof_data(distance_mm, status)


def print_tof_data(tof_data):
    """Print the reading and what navigation should do."""

    if tof_data["reading"] == "RANGE":
        distance = f'{tof_data["distance_cm"]:.1f} cm'

    elif tof_data["reading"] == "CLEAR":
        distance = "no target in range"

    elif tof_data["reading"] == "TOO_CLOSE":
        distance = "too close to measure"

    else:
        distance = "unknown reading"

    if tof_data["sensor_fault"]:
        action = "SENSOR FAULT"

    elif tof_data["obstacle"]:
        action = "OBSTACLE = 1"

    elif tof_data["slow_down"]:
        action = f'SLOW = {tof_data["speed_scale"]:.2f}'

    else:
        action = "CLEAR"

    print(
        f'{action} | '
        f'Distance = {distance} | '
        f'Status = {tof_data["status"]}'
    )


def stop_tof():
    """Release the I2C bus."""
    global tof_i2c, tof_sensor

    if tof_i2c is not None:
        tof_i2c.deinit()

    tof_i2c = None
    tof_sensor = None

    reset_tof_state()


def main():
    """Run the ToF sensor continuously."""

    setup_tof()

    print("ToF started. Put a box in front of it, then take it away.")

    try:

        while True:

            tof_data = read_tof()

            print_tof_data(tof_data)

            time.sleep(0.05)

    finally:

        stop_tof()


if __name__ == "__main__":
    main()

import cv2
from libcamera import Transform
from picamera2 import Picamera2

from stop_sign_detector import detect_stop_sign
from traffic_light_detector import detect_traffic_light


picam2 = None


def setup_camera():
    """Start the Pi camera once with full 180-degree rotation (upside down mount)."""
    global picam2

    if picam2 is not None:
        return

    picam2 = Picamera2()

    camera_config = picam2.create_preview_configuration(
        main={
            "size": (640, 480),

            # IMPORTANT:
            # Picamera2 RGB888 gives an array OpenCV can use directly as BGR.
            # Do NOT convert RGB -> BGR after capture_array().
            "format": "RGB888"
        },
        # Full 180-degree rotation (vflip + hflip) for upside-down mounted cameras
        transform=Transform(vflip=1, hflip=1)
    )

    picam2.configure(camera_config)
    picam2.start()


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
    """Convert an X coordinate into LEFT / CENTER / RIGHT."""

    if center_x is None:
        return None

    left_boundary = frame_width / 3
    right_boundary = 2 * frame_width / 3

    if center_x < left_boundary:
        return "LEFT"

    if center_x > right_boundary:
        return "RIGHT"

    return "CENTER"


# -----------------------------------------------------------
# WHAT NAVIGATION NEEDS TO KNOW FROM VISION
# -----------------------------------------------------------


def get_vision_data(frame):
    """
    Main interface between computer vision and navigation.

    Runs the stop-sign and traffic-light detectors and returns
    the detected object, center coordinates, and position.
    """

    _, frame_width = frame.shape[:2]

    stop_sign_data = detect_stop_sign(frame)
    traffic_light_data = detect_traffic_light(frame)

    # ---------------------------------------------------------
    # SAME-OBJECT ARBITRATION
    # ---------------------------------------------------------
    # If both detectors are looking at the same physical object:
    #
    # STOP SIGN wins over RED_LIGHT.
    #
    # This prevents a real stop sign from also being reported
    # as a red traffic light.
    #
    # GREEN and YELLOW are not affected.
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
    """
    Capture one frame and return all vision results.

    Navigation only needs to call this function.
    """

    setup_camera()

    # Capture directly.
    # No RGB -> BGR conversion.
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


def main():
    """Run the vision system continuously."""

    setup_camera()

    print("Camera started with 180-degree rotation (vflip + hflip). Running vision pipeline...")

    try:

        while True:

            # Capture frame.
            frame = picam2.capture_array()

            # Run detectors.
            vision_data = get_vision_data(frame)

            # Print detections.
            print_vision_data(vision_data)

    finally:

        if picam2 is not None:
            picam2.stop()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

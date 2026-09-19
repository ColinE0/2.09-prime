import cv2
import numpy as np
import time


# ============================================================
# Region of Interest
# ============================================================

TOP_CUTOFF = 0.0
BOTTOM_CUTOFF = 0.5


# ============================================================
# General Candidate Requirements
# ============================================================

MIN_SATURATION = 110
MIN_VALUE = 170

MIN_AREA_RATIO = 0.00008
MAX_AREA_RATIO = 0.05

MIN_COLOR_RATIO = 0.08
WINNER_RATIO = 1.20

# Reject a candidate when its parent box is much larger
# than the refined bright bulb/core inside it.
MAX_PARENT_TO_CORE_RATIO = 10.0

HOLD_TIME = 0



# ============================================================
# Red-Light Thresholds
# ============================================================

RED_CANDIDATE_MIN_SATURATION = 75
RED_CANDIDATE_MIN_VALUE = 140

RED_MIN_MEAN_VALUE = 200
RED_MIN_PEAK_VALUE = 240
RED_MIN_BRIGHT_RATIO = 0.7
RED_BRIGHT_PIXEL_VALUE = 220


# ============================================================
# Yellow-Light Thresholds
# ============================================================

YELLOW_CANDIDATE_MIN_SATURATION = 65
YELLOW_CANDIDATE_MIN_VALUE = 170

YELLOW_MIN_MEAN_VALUE = 195
YELLOW_MIN_PEAK_VALUE = 225
YELLOW_MIN_BRIGHT_RATIO = 0.6
YELLOW_BRIGHT_PIXEL_VALUE = 205

YELLOW_WHITE_MIN_VALUE = 230
YELLOW_WHITE_MAX_SATURATION = 90
YELLOW_MIN_WHITE_CORE_RATIO = 0.04

YELLOW_MIN_HOT_RATIO = 0.25
YELLOW_MIN_SUPERHOT_RATIO = 0.04
YELLOW_HOT_PIXEL_VALUE = 235
YELLOW_SUPERHOT_PIXEL_VALUE = 248


# ============================================================
# Green-Light Thresholds
# ============================================================

GREEN_MIN_MEAN_VALUE = 210
GREEN_MIN_PEAK_VALUE = 235
GREEN_MIN_BRIGHT_RATIO = 0.7
GREEN_BRIGHT_PIXEL_VALUE = 205


# ============================================================
# Final Bulb-Box Requirements
# ============================================================

FINAL_MIN_SATURATION = 130
FINAL_MIN_VALUE = 185

YELLOW_FINAL_MIN_SATURATION = 70
YELLOW_FINAL_MIN_VALUE = 140

FINAL_BOX_PADDING = 0.10
YELLOW_BOX_PADDING = 0.18


# ============================================================
# HSV Color Ranges
# ============================================================

RED1_LOWER = np.array([0, 100, 100])
RED1_UPPER = np.array([22, 255, 255])

RED2_LOWER = np.array([168, 100, 100])
RED2_UPPER = np.array([180, 255, 255])

YELLOW_LOWER = np.array([25, 90, 100])
YELLOW_UPPER = np.array([40, 255, 255])

GREEN_LOWER = np.array([40, 80, 80])
GREEN_UPPER = np.array([95, 255, 255])


# ============================================================
# Morphology Kernels
# ============================================================

OPEN_KERNEL = np.ones((3, 3), np.uint8)
CLOSE_KERNEL = np.ones((5, 5), np.uint8)

YELLOW_DILATE_KERNEL = np.ones((5, 5), np.uint8)
YELLOW_WHITE_NEIGHBOR_KERNEL = np.ones((7, 7), np.uint8)


# ============================================================
# Detection Hold State
# ============================================================

last_detection = None
last_analysis_time = 0.0


# ============================================================
# Mask Cleanup
# ============================================================

def clean_mask(mask):

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        OPEN_KERNEL
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        CLOSE_KERNEL
    )

    return mask


# ============================================================
# HSV Color Masks
# ============================================================

def get_color_masks(hsv):

    red1 = cv2.inRange(
        hsv,
        RED1_LOWER,
        RED1_UPPER
    )

    red2 = cv2.inRange(
        hsv,
        RED2_LOWER,
        RED2_UPPER
    )

    red = cv2.bitwise_or(
        red1,
        red2
    )

    yellow = cv2.inRange(
        hsv,
        YELLOW_LOWER,
        YELLOW_UPPER
    )

    green = cv2.inRange(
        hsv,
        GREEN_LOWER,
        GREEN_UPPER
    )

    return red, yellow, green


def get_single_color_mask(hsv, color):

    if color == "RED":

        red1 = cv2.inRange(
            hsv,
            RED1_LOWER,
            RED1_UPPER
        )

        red2 = cv2.inRange(
            hsv,
            RED2_LOWER,
            RED2_UPPER
        )

        return cv2.bitwise_or(
            red1,
            red2
        )

    if color == "YELLOW":

        return cv2.inRange(
            hsv,
            YELLOW_LOWER,
            YELLOW_UPPER
        )

    if color == "GREEN":

        return cv2.inRange(
            hsv,
            GREEN_LOWER,
            GREEN_UPPER
        )

    return np.zeros(
        hsv.shape[:2],
        dtype=np.uint8
    )


# ============================================================
# Candidate Detection
# ============================================================

def find_bright_candidates(frame):

    frame_height, frame_width = frame.shape[:2]
    frame_area = frame_width * frame_height

    # --------------------------------------------------------
    # CROP ROI FIRST
    # --------------------------------------------------------

    y_start = int(
        frame_height * TOP_CUTOFF
    )

    y_end = int(
        frame_height * BOTTOM_CUTOFF
    )

    roi_frame = frame[
        y_start:y_end,
        :
    ]

    # --------------------------------------------------------
    # BLUR ONLY THE ROI
    # --------------------------------------------------------

    blurred = cv2.GaussianBlur(
        roi_frame,
        (3, 3),
        0
    )

    # --------------------------------------------------------
    # HSV ONLY THE ROI
    # --------------------------------------------------------

    hsv = cv2.cvtColor(
        blurred,
        cv2.COLOR_BGR2HSV
    )

    # --------------------------------------------------------
    # CANDIDATE MASKS
    #
    # These use the effective thresholds directly instead of
    # creating a broad color mask and then another full-size
    # gate mask before ANDing them together.
    # --------------------------------------------------------

    # ---------------- RED ----------------

    red1_candidate = cv2.inRange(
        hsv,
        np.array([
            0,
            max(100, RED_CANDIDATE_MIN_SATURATION),
            max(100, RED_CANDIDATE_MIN_VALUE)
        ]),
        np.array([
            8,
            255,
            255
        ])
    )

    red2_candidate = cv2.inRange(
        hsv,
        np.array([
            168,
            max(100, RED_CANDIDATE_MIN_SATURATION),
            max(100, RED_CANDIDATE_MIN_VALUE)
        ]),
        np.array([
            180,
            255,
            255
        ])
    )

    red_candidate = cv2.bitwise_or(
        red1_candidate,
        red2_candidate
    )

    # ---------------- GREEN ----------------

    green_candidate = cv2.inRange(
        hsv,
        np.array([
            40,
            max(80, MIN_SATURATION),
            max(80, MIN_VALUE)
        ]),
        np.array([
            95,
            255,
            255
        ])
    )

    # ---------------- YELLOW ----------------

    yellow_candidate = cv2.inRange(
        hsv,
        np.array([
            9,
            max(90, YELLOW_CANDIDATE_MIN_SATURATION),
            max(100, YELLOW_CANDIDATE_MIN_VALUE)
        ]),
        np.array([
            40,
            255,
            255
        ])
    )

    # White-hot yellow center
    white_core = cv2.inRange(
        hsv,
        np.array([
            0,
            0,
            YELLOW_WHITE_MIN_VALUE
        ]),
        np.array([
            180,
            YELLOW_WHITE_MAX_SATURATION,
            255
        ])
    )

    yellow_neighborhood = cv2.dilate(
        yellow_candidate,
        YELLOW_WHITE_NEIGHBOR_KERNEL,
        iterations=1
    )

    white_near_yellow = cv2.bitwise_and(
        white_core,
        yellow_neighborhood
    )

    yellow_candidate = cv2.bitwise_or(
        yellow_candidate,
        white_near_yellow
    )

    # ---------------- COMBINE ----------------

    candidate_mask = cv2.bitwise_or(
        red_candidate,
        yellow_candidate
    )

    candidate_mask = cv2.bitwise_or(
        candidate_mask,
        green_candidate
    )

    # --------------------------------------------------------
    # MORPHOLOGY ONLY ON CROPPED ROI
    # --------------------------------------------------------

    candidate_mask = clean_mask(
        candidate_mask
    )

    # --------------------------------------------------------
    # CONTOURS
    # --------------------------------------------------------

    contours, _ = cv2.findContours(
        candidate_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    # --------------------------------------------------------
    # FILTER CANDIDATES
    # --------------------------------------------------------

    candidates = []

    for contour in contours:

        area = cv2.contourArea(
            contour
        )

        if area <= 0:
            continue

        # IMPORTANT:
        # Keep area ratio relative to the ORIGINAL frame.
        # This preserves your previous threshold behavior.
        area_ratio = (
            area / frame_area
        )

        if not (
            MIN_AREA_RATIO
            <= area_ratio
            <= MAX_AREA_RATIO
        ):
            continue

        x, y, box_width, box_height = (
            cv2.boundingRect(
                contour
            )
        )

        if (
            box_width <= 0
            or box_height <= 0
        ):
            continue

        # Convert cropped ROI coordinates back to
        # original full-frame coordinates.
        full_frame_y = (
            y + y_start
        )

        candidates.append(
            (
                x,
                full_frame_y,
                box_width,
                box_height
            )
        )

    return candidates


# ============================================================
# Brightness Validation
# ============================================================

def validate_light_color(
    hsv,
    color_mask,
    color
):

    values = hsv[:, :, 2][
        color_mask > 0
    ]

    if values.size == 0:
        return False

    if color == "RED":

        min_mean = (
            RED_MIN_MEAN_VALUE
        )

        min_peak = (
            RED_MIN_PEAK_VALUE
        )

        min_bright_ratio = (
            RED_MIN_BRIGHT_RATIO
        )

        bright_pixel_value = (
            RED_BRIGHT_PIXEL_VALUE
        )

    elif color == "YELLOW":

        min_mean = (
            YELLOW_MIN_MEAN_VALUE
        )

        min_peak = (
            YELLOW_MIN_PEAK_VALUE
        )

        min_bright_ratio = (
            YELLOW_MIN_BRIGHT_RATIO
        )

        bright_pixel_value = (
            YELLOW_BRIGHT_PIXEL_VALUE
        )

    elif color == "GREEN":

        min_mean = (
            GREEN_MIN_MEAN_VALUE
        )

        min_peak = (
            GREEN_MIN_PEAK_VALUE
        )

        min_bright_ratio = (
            GREEN_MIN_BRIGHT_RATIO
        )

        bright_pixel_value = (
            GREEN_BRIGHT_PIXEL_VALUE
        )

    else:

        return False

    mean_value = float(
        np.mean(values)
    )

    peak_value = float(
        np.percentile(
            values,
            90
        )
    )

    bright_ratio = float(
        np.mean(
            values
            >= bright_pixel_value
        )
    )

    return (
        mean_value >= min_mean
        and
        peak_value >= min_peak
        and
        bright_ratio >= min_bright_ratio
    )


# ============================================================
# Precise Bulb Bounding Box
# ============================================================

def get_precise_bulb_box(
    frame,
    original_box,
    color
):

    x, y, box_width, box_height = (
        original_box
    )

    roi = frame[
        y:y + box_height,
        x:x + box_width
    ]

    if (
        roi is None
        or roi.size == 0
    ):
        return original_box

    hsv = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2HSV
    )

    color_mask = get_single_color_mask(
        hsv,
        color
    )

    if color == "YELLOW":

        min_saturation = (
            YELLOW_FINAL_MIN_SATURATION
        )

        min_value = (
            YELLOW_FINAL_MIN_VALUE
        )

    else:

        min_saturation = (
            FINAL_MIN_SATURATION
        )

        min_value = (
            FINAL_MIN_VALUE
        )

    strong_mask = cv2.inRange(
        hsv,
        np.array([
            0,
            min_saturation,
            min_value
        ]),
        np.array([
            180,
            255,
            255
        ])
    )

    final_mask = cv2.bitwise_and(
        color_mask,
        strong_mask
    )

    if color == "YELLOW":

        final_mask = cv2.dilate(
            final_mask,
            YELLOW_DILATE_KERNEL,
            iterations=1
        )

    elif color == "RED":

        final_mask = cv2.morphologyEx(
            final_mask,
            cv2.MORPH_CLOSE,
            CLOSE_KERNEL
        )

    else:

        final_mask = clean_mask(
            final_mask
        )

    contours, _ = cv2.findContours(
        final_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return original_box

    best_contour = None
    best_score = 0.0

    for contour in contours:

        area = cv2.contourArea(
            contour
        )

        if area <= 0:
            continue

        contour_mask = np.zeros(
            final_mask.shape,
            dtype=np.uint8
        )

        cv2.drawContours(
            contour_mask,
            [contour],
            -1,
            255,
            cv2.FILLED
        )

        values_v = hsv[:, :, 2][
            contour_mask > 0
        ]

        values_s = hsv[:, :, 1][
            contour_mask > 0
        ]

        if (
            values_v.size == 0
            or values_s.size == 0
        ):
            continue

        mean_v = float(
            np.mean(values_v)
        )

        mean_s = float(
            np.mean(values_s)
        )

        score = (
            area
            * mean_v
            * (
                0.5
                + 0.5
                * mean_s
                / 255.0
            )
        )

        if score > best_score:

            best_score = score
            best_contour = contour

    if best_contour is None:
        return original_box

    bx, by, bw, bh = (
        cv2.boundingRect(
            best_contour
        )
    )

    if color == "YELLOW":
        padding = YELLOW_BOX_PADDING

    else:
        padding = FINAL_BOX_PADDING

    pad_x = int(
        bw * padding
    )

    pad_y = int(
        bh * padding
    )

    new_x1 = max(
        0,
        bx - pad_x
    )

    new_y1 = max(
        0,
        by - pad_y
    )

    new_x2 = min(
        box_width,
        bx + bw + pad_x
    )

    new_y2 = min(
        box_height,
        by + bh + pad_y
    )

    return (
        x + new_x1,
        y + new_y1,
        new_x2 - new_x1,
        new_y2 - new_y1
    )


# ============================================================
# Candidate Analysis
# ============================================================

def analyze_candidate_roi(frame, box):
    x, y, box_width, box_height = box

    roi = frame[
        y:y + box_height,
        x:x + box_width
    ]

    if roi is None or roi.size == 0:
        return None

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    red_mask, yellow_mask, green_mask = get_color_masks(hsv)

    masks = {
        "RED": red_mask,
        "YELLOW": yellow_mask,
        "GREEN": green_mask
    }

    counts = {
        color: cv2.countNonZero(mask)
        for color, mask in masks.items()
    }

    roi_area = max(box_width * box_height, 1)

    sorted_colors = sorted(
        counts.items(),
        key=lambda item: item[1],
        reverse=True
    )

    winner_color = sorted_colors[0][0]
    winner_count = sorted_colors[0][1]
    second_count = sorted_colors[1][1]

    if winner_count <= 0:
        return None

    winner_ratio = winner_count / roi_area

    if winner_ratio < MIN_COLOR_RATIO:
        return None

    if (
        second_count > 0
        and winner_count < second_count * WINNER_RATIO
    ):
        return None

    winner_mask = masks[winner_color]

    if not validate_light_color(
        hsv,
        winner_mask,
        winner_color
    ):
        return None

    values = hsv[:, :, 2][winner_mask > 0]

    if values.size > 0:
        strong_threshold = np.percentile(values, 85)
        strongest = values[values >= strong_threshold]
        mean_strong = float(np.mean(strongest))
        peak = float(np.percentile(values, 95))

        winner_power = (
            0.65 * mean_strong
            + 0.35 * peak
        )
    else:
        winner_power = 0.0

    glow_score = winner_power

    precise_box = get_precise_bulb_box(
        frame,
        box,
        winner_color
    )

    px, py, pw, ph = precise_box

    # --------------------------------------------------------
    # Reject large parent blobs with tiny bright cores
    # --------------------------------------------------------

    parent_area = (
        box_width
        * box_height
    )

    core_area = (
        pw
        * ph
    )

    if core_area <= 0:
        return None

    parent_to_core_ratio = (
        parent_area
        / float(core_area)
    )

    if (
        parent_to_core_ratio
        > MAX_PARENT_TO_CORE_RATIO
    ):
        return None

    center_x = px + pw // 2
    center_y = py + ph // 2

    return {
        "object": f"TRAFFIC_LIGHT_{winner_color}",
        "color": winner_color,
        "box": precise_box,
        "center": (center_x, center_y),
        "power": winner_power,
        "glow_score": glow_score
    }


# ============================================================
# Main Traffic-Light Detector
# ============================================================

def detect_traffic_light(frame):

    global last_detection
    global last_analysis_time

    current_time = time.time()

    # Detection Hold
    if (
        last_detection is not None
        and current_time - last_analysis_time < HOLD_TIME
    ):
        return last_detection

    candidates = find_bright_candidates(frame)

    valid_lights = []

    for box in candidates:

        result = analyze_candidate_roi(
            frame,
            box
        )

        if result is not None:
            valid_lights.append(result)

    if not valid_lights:

        last_detection = None
        last_analysis_time = current_time

        return None

    best_light = max(
        valid_lights,
        key=lambda light:
            light.get(
                "glow_score",
                light["power"]
            )
    )

    last_detection = best_light
    last_analysis_time = current_time

    return best_light


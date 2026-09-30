import cv2
import numpy as np
import math


# ============================================================
# Red HSV ranges
# ============================================================

RED1_LOWER = np.array([0, 100, 70])
RED1_UPPER = np.array([22, 255, 255])

RED2_LOWER = np.array([170, 120, 70])
RED2_UPPER = np.array([180, 255, 255])


# ============================================================
# Basic stop sign requirements
# ============================================================

STOP_MIN_AREA = 0.0012
STOP_MAX_AREA = 0.15

STOP_MIN_ASPECT = 0.70
STOP_MAX_ASPECT = 1.40

MIN_EXTENT = 0.40


# ============================================================
# Octagon requirements
# ============================================================

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


# ============================================================
# Circle rejection
# ============================================================

MAX_CIRCLE_FILL = 0.95


# ============================================================
# Red-light rejection
# ============================================================

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

RED_WHITE_NEIGHBOR_KERNEL = np.ones(
    (7, 7),
    np.uint8
)


# ============================================================
# Mask-cleaning kernels
# ============================================================

OPEN_KERNEL = np.ones(
    (3, 3),
    np.uint8
)

CLOSE_KERNEL = np.ones(
    (5, 5),
    np.uint8
)


# ============================================================
# Mask cleanup
# ============================================================

def clean_mask(mask):
    """Remove small noise and reconnect nearby red regions."""

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
# Red mask
# ============================================================

def get_red_mask(hsv):
    """Create a mask containing red pixels."""

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

    red_mask = cv2.bitwise_or(
        red1,
        red2
    )

    return clean_mask(red_mask)


# ============================================================
# Red-light rejection
# ============================================================

def looks_like_red_light(hsv, red_mask):
    """
    Reject red regions that behave like an illuminated red bulb.
    """

    if hsv is None or hsv.size == 0:
        return False

    roi_area = hsv.shape[0] * hsv.shape[1]

    if roi_area <= 0:
        return False

    red_values = hsv[:, :, 2][red_mask > 0]

    if red_values.size == 0:
        return False

    mean_value = float(
        np.mean(red_values)
    )

    peak_value = float(
        np.percentile(
            red_values,
            90
        )
    )

    bright_ratio = float(
        np.mean(
            red_values
            >= RED_LIGHT_BRIGHT_PIXEL
        )
    )

    very_bright_ratio = float(
        np.mean(
            red_values
            >= RED_LIGHT_VERY_BRIGHT_PIXEL
        )
    )

    # --------------------------------------------------------
    # White-hot core
    # --------------------------------------------------------

    white_core = cv2.inRange(
        hsv,
        np.array([
            0,
            0,
            RED_WHITE_MIN_VALUE
        ]),
        np.array([
            180,
            RED_WHITE_MAX_SATURATION,
            255
        ])
    )

    red_neighborhood = cv2.dilate(
        red_mask,
        RED_WHITE_NEIGHBOR_KERNEL,
        iterations=1
    )

    white_near_red = cv2.bitwise_and(
        white_core,
        red_neighborhood
    )

    white_core_ratio = (
        cv2.countNonZero(
            white_near_red
        )
        / float(roi_area)
    )

    # --------------------------------------------------------
    # Center glow
    # --------------------------------------------------------

    h, w = hsv.shape[:2]

    yy, xx = np.ogrid[:h, :w]

    cx = (w - 1) / 2.0
    cy = (h - 1) / 2.0

    rx = max(
        w
        * RED_CENTER_INNER_FRACTION
        / 2.0,
        1.0
    )

    ry = max(
        h
        * RED_CENTER_INNER_FRACTION
        / 2.0,
        1.0
    )

    center_region = (
        ((xx - cx) / rx) ** 2
        + ((yy - cy) / ry) ** 2
        <= 1.0
    )

    red_pixels = red_mask > 0

    inner_pixels = (
        red_pixels
        & center_region
    )

    outer_pixels = (
        red_pixels
        & (~center_region)
    )

    if np.count_nonzero(inner_pixels) > 0:

        inner_mean = float(
            np.mean(
                hsv[:, :, 2][
                    inner_pixels
                ]
            )
        )

    else:
        inner_mean = 0.0

    if np.count_nonzero(outer_pixels) > 0:

        outer_mean = float(
            np.mean(
                hsv[:, :, 2][
                    outer_pixels
                ]
            )
        )

    else:
        outer_mean = mean_value

    center_diff = (
        inner_mean
        - outer_mean
    )

    if outer_mean > 0:

        center_ratio = (
            inner_mean
            / outer_mean
        )

    else:
        center_ratio = 1.0

    center_glow = (
        inner_mean
        >= RED_CENTER_MIN_INNER_VALUE

        and center_diff
        >= RED_CENTER_MIN_VALUE_DIFF

        and center_ratio
        >= RED_CENTER_MIN_RATIO
    )

    normal_glow = (
        mean_value
        >= RED_LIGHT_MEAN_VALUE

        and peak_value
        >= RED_LIGHT_PEAK_VALUE

        and bright_ratio
        >= RED_LIGHT_BRIGHT_RATIO
    )

    blown_out_glow = (
        peak_value
        >= RED_LIGHT_PEAK_VALUE

        and (
            very_bright_ratio
            >= RED_LIGHT_VERY_BRIGHT_RATIO

            or

            white_core_ratio
            >= RED_MIN_WHITE_CORE_RATIO
        )
    )

    strong_hot_source = (
        very_bright_ratio
        >= RED_LIGHT_STRONG_HOT_RATIO

        or

        white_core_ratio
        >= RED_LIGHT_STRONG_WHITE_CORE_RATIO
    )

    return (
        normal_glow
        or blown_out_glow
        or center_glow
        or strong_hot_source
    )


# ============================================================
# Octagon geometry
# ============================================================

def get_angle(p1, p2, p3):
    """Calculate the interior angle at p2."""

    v1 = np.array(
        [
            p1[0] - p2[0],
            p1[1] - p2[1]
        ],
        dtype=np.float32
    )

    v2 = np.array(
        [
            p3[0] - p2[0],
            p3[1] - p2[1]
        ],
        dtype=np.float32
    )

    norm1 = np.linalg.norm(v1)
    norm2 = np.linalg.norm(v2)

    if norm1 <= 0 or norm2 <= 0:
        return 0.0

    cosine = (
        np.dot(v1, v2)
        / (norm1 * norm2)
    )

    cosine = np.clip(
        cosine,
        -1.0,
        1.0
    )

    return math.degrees(
        math.acos(cosine)
    )


def score_octagon(
    approx,
    min_side_pixels,
    max_side_length_ratio
):
    """
    Score the geometry of an 8-sided polygon.

    Returns:
        score
        geometry stats
    """

    if len(approx) != 8:
        return 0.0, None

    if not cv2.isContourConvex(approx):
        return 0.0, None

    points = [
        tuple(point[0])
        for point in approx
    ]

    angles = []

    for i in range(8):

        previous_point = points[
            (i - 1) % 8
        ]

        current_point = points[i]

        next_point = points[
            (i + 1) % 8
        ]

        angles.append(
            get_angle(
                previous_point,
                current_point,
                next_point
            )
        )

    angle_errors = [
        abs(
            angle
            - IDEAL_OCTAGON_ANGLE
        )
        for angle in angles
    ]

    mean_angle_error = float(
        np.mean(angle_errors)
    )

    max_angle_error = float(
        np.max(angle_errors)
    )

    angle_std = float(
        np.std(angles)
    )

    side_lengths = []

    for i in range(8):

        p1 = np.array(
            points[i],
            dtype=np.float32
        )

        p2 = np.array(
            points[(i + 1) % 8],
            dtype=np.float32
        )

        side_lengths.append(
            float(
                np.linalg.norm(
                    p2 - p1
                )
            )
        )

    min_side = min(
        side_lengths
    )

    max_side = max(
        side_lengths
    )

    if min_side <= 0:
        return 0.0, None

    side_ratio = (
        max_side
        / min_side
    )

    mean_side = float(
        np.mean(side_lengths)
    )

    side_std = float(
        np.std(side_lengths)
    )

    if mean_side > 0:

        side_cv = (
            side_std
            / mean_side
        )

    else:
        side_cv = 1.0

    # --------------------------------------------------------
    # Hard geometry rejection
    # --------------------------------------------------------

    if (
        mean_angle_error
        > MAX_ANGLE_ERROR
    ):
        return 0.0, None

    if (
        min_side
        < min_side_pixels
    ):
        return 0.0, None

    if (
        side_ratio
        > max_side_length_ratio
    ):
        return 0.0, None

    # --------------------------------------------------------
    # Geometry scoring
    # --------------------------------------------------------

    mean_angle_score = max(
        0.0,
        1.0
        - mean_angle_error
        / MAX_ANGLE_ERROR
    )

    consistency_score = max(
        0.0,
        1.0
        - angle_std
        / 35.0
    )

    worst_angle_score = max(
        0.0,
        1.0
        - max_angle_error
        / 50.0
    )

    side_score = max(
        0.0,
        1.0
        - (side_ratio - 1.0)
        / (
            max_side_length_ratio
            - 1.0
        )
    )

    side_consistency_score = max(
        0.0,
        1.0
        - side_cv
        / 0.45
    )

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


def find_best_octagon(
    contour,
    min_side_pixels,
    max_side_length_ratio
):
    """
    Find the strongest valid
    8-vertex approximation.
    """

    hull = cv2.convexHull(
        contour
    )

    perimeter = cv2.arcLength(
        hull,
        True
    )

    if perimeter <= 0:
        return None, 0.0, None

    # ========================================================
    # EPSILON SEARCH — UNCHANGED
    # ========================================================

    epsilon_ratios = [
        0.010,
        0.011,
        0.012,
        0.013,
        0.014,
        0.015,
        0.016,
        0.017,
        0.018,
        0.019,
        0.020,
        0.021,
        0.022,
        0.023,
        0.024,
        0.025,
        0.026,
        0.027,
        0.028,
        0.029,
        0.030,
        0.032,
        0.034,
        0.036,
        0.038,
        0.040,
        0.042,
        0.045
    ]

    best_approx = None
    best_score = 0.0
    best_geometry = None

    for epsilon_ratio in epsilon_ratios:

        approx = cv2.approxPolyDP(
            hull,
            epsilon_ratio * perimeter,
            True
        )

        if len(approx) != 8:
            continue

        score, geometry = score_octagon(
            approx,
            min_side_pixels,
            max_side_length_ratio
        )

        if score > best_score:

            best_score = score
            best_approx = approx
            best_geometry = geometry

    return (
        best_approx,
        best_score,
        best_geometry
    )


# ============================================================
# Main stop-sign detector
# ============================================================

def detect_stop_sign(frame):
    """
    Detect the strongest valid stop sign
    in the right-side ROI.
    """

    height, width = frame.shape[:2]

    frame_area = (
        width * height
    )

    # Process only right half of image.
    x_start = int(
        width * 0.50
    )

    x_end = width

    # Original vertical ROI.
    y_start = int(
        height * 0.05
    )

    y_end = int(
        height * 0.80
    )

    roi_frame = frame[
        y_start:y_end,
        x_start:x_end
    ]

    if roi_frame.size == 0:
        return None

    blurred = cv2.GaussianBlur(
        roi_frame,
        (3, 3),
        0
    )

    hsv = cv2.cvtColor(
        blurred,
        cv2.COLOR_BGR2HSV
    )

    red_mask = get_red_mask(
        hsv
    )

    contours, _ = cv2.findContours(
        red_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    best_detection = None
    best_score = 0.0

    for contour in contours:

        area = cv2.contourArea(
            contour
        )

        if area <= 0:
            continue

        area_ratio = (
            area
            / frame_area
        )

        if not (
            STOP_MIN_AREA
            <= area_ratio
            <= STOP_MAX_AREA
        ):
            continue

        x, y, box_w, box_h = (
            cv2.boundingRect(
                contour
            )
        )

        if (
            box_w <= 0
            or box_h <= 0
        ):
            continue

        aspect_ratio = (
            box_w
            / float(box_h)
        )

        if not (
            STOP_MIN_ASPECT
            <= aspect_ratio
            <= STOP_MAX_ASPECT
        ):
            continue

        box_area = (
            box_w * box_h
        )

        if box_area <= 0:
            continue

        extent = (
            area / box_area
        )

        if extent < MIN_EXTENT:
            continue

        # ----------------------------------------------------
        # Circle rejection
        # ----------------------------------------------------

        _, radius = (
            cv2.minEnclosingCircle(
                contour
            )
        )

        if radius <= 0:
            continue

        circle_area = (
            np.pi
            * radius
            * radius
        )

        if circle_area <= 0:
            continue

        circle_fill = (
            area
            / circle_area
        )

        if (
            circle_fill
            > MAX_CIRCLE_FILL
        ):
            continue

        candidate_hsv = hsv[
            y:y + box_h,
            x:x + box_w
        ]

        candidate_red_mask = red_mask[
            y:y + box_h,
            x:x + box_w
        ]

        # ----------------------------------------------------
        # Reject glowing red traffic lights
        # ----------------------------------------------------

        if looks_like_red_light(
            candidate_hsv,
            candidate_red_mask
        ):
            continue

        # ----------------------------------------------------
        # Far / normal thresholds
        # ----------------------------------------------------

        if (
            area_ratio
            < FAR_OBJECT_AREA_RATIO
        ):

            min_side_pixels = (
                FAR_MIN_SIDE_PIXELS
            )

            min_octagon_score = (
                FAR_MIN_OCTAGON_SCORE
            )

            min_shape_match = (
                FAR_MIN_SHAPE_MATCH
            )

            max_side_length_ratio = (
                FAR_MAX_SIDE_LENGTH_RATIO
            )

        else:

            min_side_pixels = (
                NORMAL_MIN_SIDE_PIXELS
            )

            min_octagon_score = (
                NORMAL_MIN_OCTAGON_SCORE
            )

            min_shape_match = (
                NORMAL_MIN_SHAPE_MATCH
            )

            max_side_length_ratio = (
                NORMAL_MAX_SIDE_LENGTH_RATIO
            )

        # ----------------------------------------------------
        # Find octagon
        # ----------------------------------------------------

        (
            octagon,
            octagon_score,
            geometry
        ) = find_best_octagon(
            contour,
            min_side_pixels,
            max_side_length_ratio
        )

        if octagon is None:
            continue

        if (
            octagon_score
            < min_octagon_score
        ):
            continue

        contour_area = cv2.contourArea(
            contour
        )

        octagon_area = cv2.contourArea(
            octagon
        )

        if contour_area <= 0:
            continue

        shape_match = (
            octagon_area
            / contour_area
        )

        if (
            shape_match
            < min_shape_match
        ):
            continue

        # ----------------------------------------------------
        # Final confidence
        # ----------------------------------------------------

        square_score = max(
            0.0,
            1.0
            - abs(
                1.0
                - aspect_ratio
            )
        )

        extent_score = min(
            1.0,
            extent
        )

        final_score = (
            0.82 * octagon_score
            + 0.12 * square_score
            + 0.06 * extent_score
        )

        final_score = float(
            np.clip(
                final_score,
                0.0,
                1.0
            )
        )

        final_x, final_y, final_w, final_h = (
            cv2.boundingRect(
                octagon
            )
        )

        if final_score > best_score:

            best_score = final_score

            # Translate ROI coordinates
            # back to full-frame coordinates.
            full_x = (
                final_x
                + x_start
            )

            full_y = (
                final_y
                + y_start
            )

            center_x = (
                full_x
                + final_w // 2
            )

            center_y = (
                full_y
                + final_h // 2
            )

            # Navigation only needs the actual
            # detection information.
            best_detection = {
                "object": "STOP_SIGN",

                "box": (
                    full_x,
                    full_y,
                    final_w,
                    final_h
                ),

                "center": (
                    center_x,
                    center_y
                ),

                "confidence": final_score
            }

    return best_detection
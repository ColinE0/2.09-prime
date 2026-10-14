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

MIN_SATURATION = 50
MIN_VALUE = 150

MIN_AREA_RATIO = 0.00010   # Rejects tiny stray speckles
MAX_AREA_RATIO = 0.04      # Discards large reflections/housings

MIN_COLOR_RATIO = 0.15     # Requires candidate box to contain genuine color density
WINNER_RATIO = 1.30        # Must clearly beat secondary illuminated colors

MAX_PARENT_TO_CORE_RATIO = 10.0  # Prevents housing from swamping the bulb

HOLD_TIME = 0.0

# Minimum compactness (4 * pi * area / perimeter^2) to reject noisy jagged pixels
MIN_COMPACTNESS = 0.35     # Bulbs are round/square; stray noise is thin/jagged


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
RED_MIN_PEAK_VALUE = 220
RED_MIN_BRIGHT_RATIO = 0.45
RED_BRIGHT_PIXEL_VALUE = 210


# ============================================================
# Yellow-Light Thresholds
# ============================================================

YELLOW_CANDIDATE_MIN_SATURATION = 100
YELLOW_CANDIDATE_MIN_VALUE = 200

YELLOW_MIN_MEAN_VALUE = 200
YELLOW_MIN_PEAK_VALUE = 235
YELLOW_MIN_BRIGHT_RATIO = 0.45
YELLOW_BRIGHT_PIXEL_VALUE = 220

YELLOW_WHITE_MIN_VALUE = 240
YELLOW_WHITE_MAX_SATURATION = 90


# ============================================================
# Green-Light Thresholds
# ============================================================

GREEN_CANDIDATE_MIN_SATURATION = 80
GREEN_CANDIDATE_MIN_VALUE = 180

GREEN_MIN_MEAN_VALUE = 180
GREEN_MIN_PEAK_VALUE = 220
GREEN_MIN_BRIGHT_RATIO = 0.45
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
# Morphology Kernels
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


# ============================================================
# Mask Cleanup
# ============================================================

def clean_mask(mask):
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, OPEN_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, CLOSE_KERNEL)
    return mask


# ============================================================
# HSV Color Masks (With White-Core Inheritance)
# ============================================================

def get_color_masks(hsv):
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


# ============================================================
# Candidate Detection
# ============================================================

def find_bright_candidates(frame):
    frame_height, frame_width = frame.shape[:2]
    frame_area = frame_width * frame_height

    y_start = int(frame_height * TOP_CUTOFF)
    y_end = int(frame_height * BOTTOM_CUTOFF)

    roi_frame = frame[y_start:y_end, :]

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

    contours, _ = cv2.findContours(
        candidate_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

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
        if aspect_ratio < 0.4 or aspect_ratio > 2.5:
            continue

        full_frame_y = y + y_start
        candidates.append((x, full_frame_y, box_width, box_height))

    return candidates


# ============================================================
# Brightness & Light Validation
# ============================================================

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

    passed = (
        mean_value >= min_mean
        and peak_value >= min_peak
        and bright_ratio >= min_bright_ratio
    )

    return passed


# ============================================================
# Precise Bulb Bounding Box
# ============================================================

def get_precise_bulb_box(hsv, original_box, color, winner_mask):
    """
    Refine the bulb box using the HSV ROI and winning mask that were
    already calculated in analyze_candidate_roi().

    This avoids converting the same ROI to HSV again and avoids
    regenerating RED/YELLOW/GREEN masks a second time.
    """
    x, y, box_width, box_height = original_box

    final_mask = winner_mask.copy()

    if color == "YELLOW":
        final_mask = cv2.dilate(final_mask, YELLOW_DILATE_KERNEL, iterations=1)
    elif color == "RED":
        final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, CLOSE_KERNEL)
    else:
        final_mask = clean_mask(final_mask)

    contours, _ = cv2.findContours(
        final_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

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

        mean_v = float(np.mean(values_v))
        score = area * mean_v

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
    roi = frame[y:y + box_height, x:x + box_width]

    if roi is None or roi.size == 0:
        return None

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    red_mask, yellow_mask, green_mask = get_color_masks(hsv)

    masks = {
        "RED": red_mask,
        "YELLOW": yellow_mask,
        "GREEN": green_mask
    }


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
    sorted_colors = sorted(
        color_scores.items(),
        key=lambda item: item[1][1],
        reverse=True
    )

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

    parent_to_core_ratio = parent_area / float(core_area)

    if parent_to_core_ratio > MAX_PARENT_TO_CORE_RATIO:
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

    if (
        last_detection is not None
        and current_time - last_analysis_time < HOLD_TIME
    ):
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

    best_light = max(
        valid_lights,
        key=lambda light: light.get("glow_score", light["power"])
    )


    last_detection = best_light
    last_analysis_time = current_time

    return best_light


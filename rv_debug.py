"""RVD, motors-off camera debug: why each bright colored blob is or is not a traffic light.

Runs Ryan's light-detector checks (ryan_vision/traffic_light_detector.py, unchanged) one at a
time on every red, yellow and green blob in a camera frame and names the first check each
blob fails, with its brightness numbers. Also saves snapshots: the frame as the camera saw it
and an annotated copy with each blob boxed and the light search band marked. Runs only in the
camera worker process; nothing here touches the motors. In the release since v9.10.
"""

from dataclasses import dataclass
from pathlib import Path
import time

import cv2
import numpy as np

from ryan_vision import traffic_light_detector as tl

SNAPSHOT_DIR = Path.home() / "rv_snapshots"
MAX_BLOBS = 3  # the largest blobs reported per frame

# Ryan's candidate thresholds per color: (hue ranges, minimum saturation, minimum value).
CANDIDATE = {
    "red": (((0, 15), (155, 180)), tl.RED_CANDIDATE_MIN_SATURATION, tl.RED_CANDIDATE_MIN_VALUE),
    "yellow": (((14, 35),), tl.YELLOW_CANDIDATE_MIN_SATURATION, tl.YELLOW_CANDIDATE_MIN_VALUE),
    "green": (((38, 90),), tl.GREEN_CANDIDATE_MIN_SATURATION, tl.GREEN_CANDIDATE_MIN_VALUE),
}
BRIGHTNESS = {  # validate_light_color: (mean V, 90th-percentile V, bright pixel V, bright share)
    "RED": (tl.RED_MIN_MEAN_VALUE, tl.RED_MIN_PEAK_VALUE, tl.RED_BRIGHT_PIXEL_VALUE, tl.RED_MIN_BRIGHT_RATIO),
    "YELLOW": (tl.YELLOW_MIN_MEAN_VALUE, tl.YELLOW_MIN_PEAK_VALUE, tl.YELLOW_BRIGHT_PIXEL_VALUE,
               tl.YELLOW_MIN_BRIGHT_RATIO),
    "GREEN": (tl.GREEN_MIN_MEAN_VALUE, tl.GREEN_MIN_PEAK_VALUE, tl.GREEN_BRIGHT_PIXEL_VALUE,
              tl.GREEN_MIN_BRIGHT_RATIO),
}


@dataclass
class Blob:
    color: str
    box: tuple          # x, y, width, height in the full frame
    area_frac: float
    stats: str          # brightness numbers for the line
    verdict: str        # "LIGHT ..." when Ryan's detector accepts it, else the first failed check
    near_miss: bool = False  # too dim or too pale to be a candidate at all

    @property
    def accepted(self):
        return self.verdict.startswith("LIGHT")

    def line(self, index):
        x, y, w, h = self.box
        where = f"@({x + w // 2},{y + h // 2})"
        kind = "near miss" if self.near_miss else "blob"
        return f"RVD {kind} {index}: {self.color} {self.area_frac:.2%} {where} | {self.stats} | {self.verdict}"


def _hue_mask(hsv, color, saturation, value):
    ranges = CANDIDATE[color][0]
    mask = None
    for low, high in ranges:
        part = cv2.inRange(hsv, np.array([low, saturation, value]), np.array([high, 255, 255]))
        mask = part if mask is None else cv2.bitwise_or(mask, part)
    return mask


def candidate_masks(hsv):
    """Per-color candidate masks, as find_bright_candidates builds them (white cores attached)."""
    white = cv2.inRange(hsv, np.array([0, 0, tl.WHITE_CORE_MIN_VALUE]),
                        np.array([180, tl.WHITE_CORE_MAX_SATURATION, 255]))
    masks = {}
    for color, (_, saturation, value) in CANDIDATE.items():
        mask = _hue_mask(hsv, color, saturation, value)
        if color in ("red", "yellow"):
            near = cv2.dilate(mask, tl.HALO_DILATE_KERNEL, iterations=2)
            mask = cv2.bitwise_or(mask, cv2.bitwise_and(white, near))
        masks[color] = mask
    return masks


def _brightness(hsv, mask):
    """mean V, 90th-percentile V, mean S over the mask (validate_light_color's numbers)."""
    values = hsv[:, :, 2][mask > 0]
    sats = hsv[:, :, 1][mask > 0]
    if values.size == 0:
        return None
    return float(np.mean(values)), float(np.percentile(values, 90)), float(np.mean(sats)), values


def _roi_verdict(frame, box):
    """analyze_candidate_roi's checks in order: the first one this box fails, or LIGHT."""
    x, y, w, h = box
    roi = frame[y:y + h, x:x + w]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    masks = dict(zip(("RED", "YELLOW", "GREEN"), tl.get_color_masks(hsv)))
    scores = {}
    for color, mask in masks.items():
        count = cv2.countNonZero(mask)
        mean_v = float(np.mean(hsv[:, :, 2][mask > 0])) if count else 0.0
        scores[color] = (count, count * (mean_v / 255.0) ** 2)
    ranked = sorted(scores.items(), key=lambda item: item[1][1], reverse=True)
    (winner, (count, power)), (second, (_, second_power)) = ranked[0], ranked[1]
    if count <= 0 or power <= 0:
        return "rejected: no lamp-colored pixels in its box", ""
    share = count / float(max(w * h, 1))
    if share < tl.MIN_COLOR_RATIO:
        return f"rejected: color fills {share:.0%} of its box (needs {tl.MIN_COLOR_RATIO:.0%})", ""
    if second_power > 0 and power < second_power * tl.WINNER_RATIO:
        return f"rejected: {winner} barely beats {second} (mixed colors)", ""
    mean_v, peak_v, mean_s, values = _brightness(hsv, masks[winner])
    min_mean, min_peak, bright_value, min_bright = BRIGHTNESS[winner]
    bright = float(np.mean(values >= bright_value))
    stats = f"V mean {mean_v:.0f} p90 {peak_v:.0f} bright {bright:.2f} S {mean_s:.0f}"
    if peak_v < 240 and mean_s < 80:
        return "rejected: too pale (mean S under 80 and not blown out)", stats
    if mean_v < min_mean:
        return f"rejected: too dim (mean V {mean_v:.0f} < {min_mean})", stats
    if peak_v < min_peak:
        return f"rejected: too dim (p90 V {peak_v:.0f} < {min_peak})", stats
    if bright < min_bright:
        return f"rejected: too few bright pixels ({bright:.2f} < {min_bright})", stats
    result = tl.analyze_candidate_roi(frame, box)
    if result is None:
        return "rejected: bulb box too small for its candidate box (MAX_PARENT_TO_CORE_RATIO)", stats
    return f"LIGHT {result['color']}", stats


def light_debug(frame):
    """Every red, yellow or green blob big enough to matter, largest first, with its verdict."""
    height, width = frame.shape[:2]
    frame_area = float(width * height)
    band_end = int(height * tl.BOTTOM_CUTOFF)
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (3, 3), 0), cv2.COLOR_BGR2HSV)
    masks = candidate_masks(hsv)
    combined = tl.clean_mask(cv2.bitwise_or(cv2.bitwise_or(masks["red"], masks["yellow"]), masks["green"]))
    blobs = []
    for contour in cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        area = cv2.contourArea(contour)
        if area <= 0 or area / frame_area < tl.MIN_AREA_RATIO / 2:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        inside = np.zeros(combined.shape, np.uint8)
        cv2.drawContours(inside, [contour], -1, 255, cv2.FILLED)
        color = max(masks, key=lambda name: cv2.countNonZero(cv2.bitwise_and(masks[name], inside)))
        ratio = area / frame_area
        perimeter = cv2.arcLength(contour, True)
        compact = (4.0 * np.pi * area) / (perimeter * perimeter) if perimeter > 0 else 1.0
        aspect = w / float(h)
        stats = ""
        if y + h / 2 >= band_end:
            verdict = f"rejected: below the light search band (lights must be in the top half, y < {band_end})"
        elif ratio < tl.MIN_AREA_RATIO:
            verdict = f"rejected: under {tl.MIN_AREA_RATIO:.2%} of frame (a speck)"
        elif ratio > tl.MAX_AREA_RATIO:
            verdict = f"rejected: over {tl.MAX_AREA_RATIO:.0%} of frame (treated as a reflection or housing)"
        elif compact < tl.MIN_COMPACTNESS:
            verdict = f"rejected: ragged shape (compactness {compact:.2f} < {tl.MIN_COMPACTNESS})"
        elif not 0.4 <= aspect <= 2.5:
            verdict = f"rejected: too long and thin (width/height {aspect:.2f}, needs 0.4 to 2.5)"
        else:
            verdict, stats = _roi_verdict(frame, (x, y, w, h))
        if not stats:
            numbers = _brightness(hsv, cv2.bitwise_and(masks[color], inside))
            if numbers:
                stats = f"V mean {numbers[0]:.0f} p90 {numbers[1]:.0f} S {numbers[2]:.0f}"
        blobs.append(Blob(color, (x, y, w, h), ratio, stats, verdict))
    blobs.sort(key=lambda blob: blob.area_frac, reverse=True)
    blobs = blobs[:MAX_BLOBS]
    # Colors with no candidate at all: the biggest patch of that hue at Ryan's looser halo
    # thresholds, to show how far it is from candidate brightness.
    if len(blobs) < MAX_BLOBS:
        found = {blob.color for blob in blobs}
        halos = dict(zip(("red", "yellow", "green"), tl.get_color_masks(hsv)))
        for color, halo in halos.items():
            if color in found or len(blobs) >= MAX_BLOBS:
                continue
            halo = tl.clean_mask(halo)
            contours = cv2.findContours(halo, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            contours = [c for c in contours if cv2.contourArea(c) / frame_area >= tl.MIN_AREA_RATIO]
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(contour)
            inside = np.zeros(halo.shape, np.uint8)
            cv2.drawContours(inside, [contour], -1, 255, cv2.FILLED)
            mean_v, peak_v, mean_s, _ = _brightness(hsv, inside)
            _, min_s, min_v = CANDIDATE[color]
            blobs.append(Blob(color, (x, y, w, h), cv2.contourArea(contour) / frame_area,
                              f"V mean {mean_v:.0f} p90 {peak_v:.0f} S {mean_s:.0f}",
                              f"not a candidate: needs V {min_v}+ and S {min_s}+ (brighter, more saturated)",
                              near_miss=True))
    return blobs


def describe(frame, blobs):
    """The RVD lines for one frame."""
    hsv_v = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 2]
    band_end = int(frame.shape[0] * tl.BOTTOM_CUTOFF)
    lines = [f"RVD frame: brightest V {int(hsv_v.max())} overall, {int(hsv_v[:band_end].max())} in the top half; "
             f"{sum(blob.accepted for blob in blobs)} of {len(blobs)} blobs accepted"]
    if not blobs:
        lines.append("RVD no red, yellow or green patch at all (not even dim)")
    lines += [blob.line(index) for index, blob in enumerate(blobs, 1)]
    lines.extend(stop_debug(frame))
    return tuple(lines)


def stop_debug(frame):
    """Explain the actual stop detector, not a separate approximation of its gates."""
    from ryan_vision.stop_sign_detector import detect_stop_sign
    records = []
    detected = detect_stop_sign(frame, diagnostics=records)
    lines = ["RVD stop: searching WHOLE frame after rotation"]
    if not records:
        lines.append("RVD stop: no red contours; check sign visibility, color, exposure and focus")
    for record in sorted(records, key=lambda item: item["area_frac"], reverse=True)[:3]:
        lines.append(f"RVD stop: box={record['box']} red contour={record['area_frac']:.2%}: {record['reason']}")
    if detected is not None:
        _, _, w, h = detected["box"]
        area = w * h / float(frame.shape[0] * frame.shape[1])
        lines.append(f"RVD stop: selected stop_sign box area={area:.2%}; see RV action size and confirmation")
    return lines


def save_snapshot(frame, blobs, folder=None):
    """Write the frame and an annotated copy; return the annotated file's path."""
    folder = SNAPSHOT_DIR if folder is None else Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S") + f"_{int(time.time() * 1000) % 1000:03d}"
    raw_path, marked_path = folder / f"rv_{stamp}.jpg", folder / f"rv_{stamp}_annotated.jpg"
    cv2.imwrite(str(raw_path), frame)
    marked = frame.copy()
    height, width = marked.shape[:2]
    band_end = int(height * tl.BOTTOM_CUTOFF)
    cv2.line(marked, (0, band_end), (width - 1, band_end), (0, 255, 255), 1)
    cv2.putText(marked, "lights searched above this line", (5, band_end - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    for index, blob in enumerate(blobs, 1):
        x, y, w, h = blob.box
        # Green: accepted. Magenta: rejected (shows on a red lamp). Orange: not even a candidate.
        box_color = (0, 255, 0) if blob.accepted else ((0, 165, 255) if blob.near_miss else (255, 0, 255))
        cv2.rectangle(marked, (x, y), (x + w, y + h), box_color, 2)
        label = f"{index} {blob.color} {blob.area_frac:.2%} {'LIGHT' if blob.accepted else 'no'}"
        cv2.putText(marked, label, (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_color, 1)
    # Include the selected stop sign as well as the traffic-light blobs.
    from ryan_vision.stop_sign_detector import detect_stop_sign
    stop = detect_stop_sign(frame)
    if stop is not None:
        x, y, w, h = stop["box"]
        cv2.rectangle(marked, (x, y), (x + w, y + h), (255, 255, 0), 2)
        cv2.putText(marked, "STOP SIGN", (x, max(12, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
    cv2.imwrite(str(marked_path), marked)
    return marked_path

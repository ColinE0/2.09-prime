"""Adapt Ryan's rr vision interface to the terminal menu's fresh-frame results.

Detector thresholds, image regions and red/sign arbitration come from the
upstream modules. See SOURCE.md for the pinned revision and local changes.
"""

from collections import namedtuple

import cv2

from . import traffic_light_detector, vision

Detection = namedtuple("Detection", ["label", "area_frac", "position", "center"])


class GeometricDetector:
    def __init__(self, cfg):
        self.cfg = cfg

    def detect(self, frame_bgr):
        height, width = frame_bgr.shape[:2]
        if width != self.cfg.DETECT_PROC_WIDTH:
            scale = self.cfg.DETECT_PROC_WIDTH / float(width)
            frame_bgr = cv2.resize(
                frame_bgr, (self.cfg.DETECT_PROC_WIDTH, max(1, round(height * scale))))

        height, width = frame_bgr.shape[:2]
        frame_area = float(height * width)

        # FC confirmation must use a new analysis for every camera frame.
        # Clear upstream's optional wall-clock cache, also covering clock rollback.
        traffic_light_detector.last_detection = None
        traffic_light_detector.last_analysis_time = 0.0
        data = vision.get_vision_data(frame_bgr)
        detections = []
        for label in ("stop_sign", "red_light", "yellow_light", "green_light"):
            if not data[label]:
                continue
            prefix = "stop_sign" if label == "stop_sign" else "traffic_light"
            _, _, box_width, box_height = data[prefix + "_data"]["box"]
            detections.append(Detection(
                label, box_width * box_height / frame_area,
                data[prefix + "_position"], data[prefix + "_center"]))
        return detections

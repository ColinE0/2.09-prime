"""Focused synthetic regressions for Ryan's September 23 detector update.

These exercise the imported algorithms and the menu's camera path. Synthetic
fixtures verify the intended code behavior, not accuracy on physical props.
"""

from queue import Queue
import sys
import types
import unittest
from unittest.mock import patch

import vision_settings as settings
import vision_support as vision

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


@unittest.skipIf(cv2 is None, "OpenCV/numpy are needed for image-based detector tests")
class September23DetectorTests(unittest.TestCase):
    def setUp(self):
        from ryan_vision import traffic_light_detector as lights
        from ryan_vision import vision as ryan
        from ryan_vision.detector_geometric import GeometricDetector
        self.lights, self.ryan = lights, ryan
        self.detector = GeometricDetector(settings)
        self.original_cache = (lights.last_detection, lights.last_analysis_time)
        lights.last_detection, lights.last_analysis_time = None, 0.0
        self.addCleanup(self.restore_cache)

    def restore_cache(self):
        self.lights.last_detection, self.lights.last_analysis_time = self.original_cache

    @staticmethod
    def blank():
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def white_core_lamp(self, color=(0, 255, 0), center=(320, 100)):
        frame = self.blank()
        cv2.circle(frame, center, 25, color, -1)
        # A mostly white bulb with a narrow colored halo. The new algorithm
        # associates nearby V=255/S=0 pixels with the halo's color mask.
        cv2.circle(frame, center, 21, (255, 255, 255), -1)
        return frame

    def test_noncompact_cross_glare_is_rejected_without_rejecting_round_bulbs(self):
        glare = self.blank()
        cv2.rectangle(glare, (270, 95), (370, 105), (0, 255, 0), -1)
        cv2.rectangle(glare, (315, 50), (325, 150), (0, 255, 0), -1)
        # This connected cross has enough area and a square bounding box, so
        # neither the area nor aspect-ratio gate explains its rejection.
        with patch.object(self.lights, "MIN_COMPACTNESS", 0.0):
            self.assertEqual(len(self.lights.find_bright_candidates(glare)), 1)
        self.assertEqual(self.lights.find_bright_candidates(glare), [])
        self.assertEqual(self.detector.detect(glare), [])

        bulb = self.blank()
        cv2.circle(bulb, (320, 100), 25, (0, 255, 0), -1)
        self.assertEqual(len(self.lights.find_bright_candidates(bulb)), 1)
        self.assertEqual([result.label for result in self.detector.detect(bulb)], ["green_light"])

    def test_horizontal_and_vertical_glare_fail_aspect_gate_even_without_compactness_gate(self):
        for start, end in (((270, 85), (370, 115)), ((305, 50), (335, 150))):
            with self.subTest(start=start, end=end):
                glare = self.blank()
                cv2.rectangle(glare, start, end, (0, 255, 0), -1)
                # 101x31 and 31x101 boxes lie beyond the new 0.4..2.5 band.
                # Disabling compactness still leaves the aspect gate active.
                with patch.object(self.lights, "MIN_COMPACTNESS", 0.0):
                    self.assertEqual(self.lights.find_bright_candidates(glare), [])
                self.assertEqual(self.detector.detect(glare), [])

        compact = self.blank()
        cv2.rectangle(compact, (290, 85), (350, 115), (0, 255, 0), -1)
        self.assertEqual(len(self.lights.find_bright_candidates(compact)), 1)
        self.assertEqual([result.label for result in self.detector.detect(compact)], ["green_light"])

    def test_overexposed_core_inherits_each_colored_halo_and_reports_correct_location(self):
        for color_index, label, color in (
                (0, "red_light", (0, 0, 255)),
                (1, "yellow_light", (0, 255, 255)),
                (2, "green_light", (0, 255, 0))):
            with self.subTest(label=label):
                frame = self.white_core_lamp(color)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                self.assertEqual(tuple(hsv[100, 339][1:]), (0, 255))
                masks = self.lights.get_color_masks(hsv)
                self.assertEqual(masks[color_index][100, 339], 255)
                self.assertTrue(all(mask[100, 339] == 0 for i, mask in enumerate(masks) if i != color_index))
                results = self.detector.detect(frame)
                self.assertEqual([result.label for result in results], [label])
                self.assertEqual(results[0].position, "CENTER")
                self.assertEqual(results[0].center, (320, 100))
                self.assertGreater(results[0].area_frac, 0)
                self.assertLess(results[0].area_frac, 0.008)  # under v9.5's light action size

    def test_white_glare_has_no_color_and_cannot_reuse_previous_green_after_clock_rollback(self):
        glare = self.blank()
        cv2.circle(glare, (320, 100), 25, (255, 255, 255), -1)
        with patch.object(self.lights.time, "time", side_effect=(100.0, 99.0)):
            self.assertEqual([result.label for result in self.detector.detect(self.white_core_lamp())],
                             ["green_light"])
            # Fresh white glare must not become a cached green confirmation.
            self.assertEqual(self.detector.detect(glare), [])
        hsv = cv2.cvtColor(glare, cv2.COLOR_BGR2HSV)
        self.assertTrue(all(cv2.countNonZero(mask) == 0 for mask in self.lights.get_color_masks(hsv)))

    def test_menu_worker_rotates_once_without_calling_upstream_camera_transform(self):
        for rotate in (True, False):
            with self.subTest(rotate_180=rotate):
                raw = self.white_core_lamp(center=(119, 379) if rotate else (520, 100))
                devices = []

                class FakePicamera:
                    def __init__(self, index):
                        self.index, self.closed = index, False
                        devices.append(self)

                    def create_video_configuration(self, **kwargs):
                        self.config = kwargs
                        return kwargs

                    def configure(self, config):
                        pass

                    def start(self, **kwargs):
                        self.start_options = kwargs

                    def capture_array(self, name):
                        return raw.copy()

                    def stop(self):
                        pass

                    def close(self):
                        self.closed = True

                finished = []
                done = types.SimpleNamespace(is_set=lambda: bool(finished),
                                             wait=lambda delay: finished.append(True))
                queue = Queue(maxsize=4)
                # Ryan's standalone setup now rotates via libcamera.Transform.
                # The menu owns capture and must never call that setup as well.
                with patch.dict(sys.modules, {"picamera2": types.SimpleNamespace(Picamera2=FakePicamera)}), \
                        patch.object(self.ryan, "setup_camera", side_effect=AssertionError("Upstream camera setup used")) as upstream_setup, \
                        patch.object(cv2, "flip", wraps=cv2.flip) as flip, \
                        patch.object(cv2, "setNumThreads"), \
                        patch.object(vision.time, "monotonic", side_effect=(0.0, 2.0, 2.02)):
                    vision.vision_worker(vision.CameraOptions(backend="csi", rotate_180=rotate), queue, done)
                result = vision.check_vision_sample(queue.get_nowait(), 2.03)
                self.assertEqual(result.detections[0][0], "green_light")
                self.assertEqual(result.locations, (("green_light", "RIGHT", (520, 100)),))
                self.assertEqual(result.sequence, 1)
                self.assertEqual(result.timestamp, 2.0)
                self.assertEqual(flip.call_count, 1 if rotate else 0)
                if rotate:
                    self.assertEqual(flip.call_args.args[1], -1)
                upstream_setup.assert_not_called()
                self.assertEqual(len(devices), 1)
                self.assertNotIn("transform", devices[0].config)
                self.assertEqual(devices[0].start_options, {"show_preview": False})
                self.assertTrue(devices[0].closed)


if __name__ == "__main__":
    unittest.main(verbosity=2)

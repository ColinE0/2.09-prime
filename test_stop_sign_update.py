"""Stop-sign image regressions, with actual detector and action confirmation."""
import types
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import rv_debug
import vision_settings as settings
import vision_support as vision
from ryan_vision import stop_sign_detector as stop
import test_vision


class StopSignUpdateTests(unittest.TestCase):
    def setUp(self):
        self.fixture = test_vision.RyanDetectorTests()
        self.detector = self.fixture.detector()

    def lettered(self, center=(480, 180), value=165, thickness=2):
        frame = self.fixture.sign_frame(center=center, value=value)
        cv2.putText(frame, 'STOP', (center[0]-42, center[1]+10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), thickness, cv2.LINE_AA)
        return frame

    def test_white_lettering_is_a_sign_across_frame_positions_and_red_brightness(self):
        for center in ((150, 180), (320, 180), (480, 180), (320, 400)):
            for value in (100, 150, 165, 200):
                with self.subTest(center=center, value=value):
                    detections = self.detector.detect(self.lettered(center, value))
                    self.assertEqual([d.label for d in detections], ['stop_sign'])

    def test_white_lettering_and_white_border_remain_a_sign(self):
        frame = self.lettered(thickness=3)
        cv2.polylines(frame, [np.array([[480 + int(59*np.cos(np.deg2rad(22.5+45*k))),
                                       180 + int(59*np.sin(np.deg2rad(22.5+45*k)))]
                                      for k in range(8)], np.int32)], True, (255,255,255), 2)
        self.assertEqual([d.label for d in self.detector.detect(frame)], ['stop_sign'])

    def test_bright_lamps_keep_their_light_classification(self):
        for kind in ('circle', 'octagon', 'white_core_circle'):
            with self.subTest(kind=kind):
                frame = self.fixture.blank()
                if kind == 'octagon':
                    frame = self.fixture.sign_frame(radius=45, value=255)
                else:
                    cv2.circle(frame, (480,180), 35, (0,0,255), -1)
                    if kind == 'white_core_circle':
                        cv2.circle(frame, (480,180), 10, (255,255,255), -1)
                self.assertEqual([d.label for d in self.detector.detect(frame)], ['red_light'])

    def test_non_octagonal_red_objects_do_not_become_signs(self):
        for kind in ('circle', 'square', 'triangle'):
            with self.subTest(kind=kind):
                frame = self.fixture.blank()
                if kind == 'circle':
                    cv2.circle(frame, (150,180), 55, (0,0,165), -1)
                elif kind == 'square':
                    cv2.rectangle(frame, (100,130), (200,230), (0,0,165), -1)
                else:
                    cv2.fillPoly(frame, [np.array([(150,120),(90,230),(210,230)], np.int32)], (0,0,165))
                self.assertNotIn('stop_sign', [d.label for d in self.detector.detect(frame)])

    def test_lettered_sign_confirms_after_two_distinct_frames(self):
        detections = self.detector.detect(self.lettered(center=(320,180)))
        stream = types.SimpleNamespace(latest=lambda: None)
        with patch.object(settings, 'STOPSIGN_ACT_AREA_FRAC', 0.012):
            light = vision.TrafficLight(stream, log=lambda text: None)
            for sequence in (1,2):
                sample = vision.VisionSample(sequence, sequence/10, tuple((d.label,d.area_frac) for d in detections), 0.01)
                stream.latest = lambda sample=sample: sample
                light.update(sequence/10)
                self.assertEqual(light.stop_sign, sequence == 2)
            self.assertEqual(light.decision(), 'stop_sign')
            self.assertFalse(light.crossing(0.2))

    def test_small_raw_sign_still_obeys_action_threshold(self):
        detections = self.detector.detect(self.fixture.sign_frame(radius=16))
        self.assertEqual([d.label for d in detections], ['stop_sign'])
        with patch.object(settings, 'STOPSIGN_ACT_AREA_FRAC', 0.012):
            stream = types.SimpleNamespace(latest=lambda: None)
            light = vision.TrafficLight(stream, log=lambda text: None)
            for sequence in (1,2,3):
                sample = vision.VisionSample(sequence, sequence/10, tuple((d.label,d.area_frac) for d in detections), 0.01)
                stream.latest = lambda sample=sample: sample
                light.update(sequence/10)
            self.assertFalse(light.stop_sign)

    def test_rvd_reports_real_rejections_and_raw_acceptance(self):
        self.assertIn('no red contours', '\n'.join(rv_debug.stop_debug(self.fixture.blank())))
        frame = self.fixture.blank()
        cv2.rectangle(frame, (150,100), (200,150), (0,0,165), -1)
        self.assertIn('no valid convex octagon', '\n'.join(rv_debug.stop_debug(frame)))
        self.assertIn('selected stop_sign', '\n'.join(rv_debug.stop_debug(self.lettered())))
        self.assertIn('illuminated/overexposed', '\n'.join(rv_debug.stop_debug(self.fixture.sign_frame(value=255))))

    def test_diagnostics_do_not_change_selected_detection(self):
        for frame in (self.lettered(), self.fixture.sign_frame(value=255), self.fixture.blank()):
            records = []
            self.assertEqual(stop.detect_stop_sign(frame), stop.detect_stop_sign(frame, diagnostics=records))


if __name__ == '__main__':
    unittest.main()

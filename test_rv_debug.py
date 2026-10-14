"""RVD (home testing only): the blob explanations agree with Ryan's detector, and snapshots save."""

from pathlib import Path
from queue import Queue
import tempfile
import types
import unittest

import vision_settings as settings
import vision_support as vision
from test_motors_optimized import FakeClock

try:
    import cv2
    import numpy as np
    import rv_debug
except ImportError:
    cv2 = np = rv_debug = None


@unittest.skipIf(cv2 is None, "OpenCV/numpy are needed for RVD")
class BlobVerdictTests(unittest.TestCase):
    def frame(self, draw):
        frame = np.zeros((480, 640, 3), np.uint8)
        draw(frame)
        return frame

    def verdicts(self, frame):
        return [(blob.color, blob.verdict) for blob in rv_debug.light_debug(frame)]

    def test_accepted_blobs_match_what_ryans_detector_reports(self):
        from ryan_vision.detector_geometric import GeometricDetector
        detector = GeometricDetector(settings)
        drawings = (
            lambda f: cv2.circle(f, (320, 120), 35, (0, 0, 255), -1),
            lambda f: cv2.circle(f, (320, 120), 35, (0, 255, 0), -1),
            lambda f: cv2.circle(f, (320, 120), 35, (0, 255, 255), -1),
            lambda f: cv2.rectangle(f, (200, 20), (440, 230), (0, 0, 255), -1),
            lambda f: cv2.circle(f, (320, 380), 35, (0, 0, 255), -1),
            lambda f: cv2.circle(f, (320, 120), 35, (0, 0, 140), -1),
        )
        for index, draw in enumerate(drawings):
            with self.subTest(index=index):
                frame = self.frame(draw)
                accepted = {blob.color + "_light" for blob in rv_debug.light_debug(frame) if blob.accepted}
                lights = {d.label for d in detector.detect(frame) if d.label != "stop_sign"}
                self.assertEqual(accepted, lights)

    def test_a_phone_screen_up_close_is_too_big(self):
        frame = self.frame(lambda f: cv2.rectangle(f, (200, 20), (440, 230), (0, 0, 255), -1))
        self.assertIn("over 4% of frame", self.verdicts(frame)[0][1])

    def test_a_light_in_the_bottom_half_is_outside_the_search_band(self):
        frame = self.frame(lambda f: cv2.circle(f, (320, 380), 35, (0, 0, 255), -1))
        self.assertIn("below the light search band", self.verdicts(frame)[0][1])

    def test_a_dim_red_is_a_near_miss_with_its_brightness(self):
        frame = self.frame(lambda f: cv2.circle(f, (320, 120), 35, (0, 0, 140), -1))
        blob = rv_debug.light_debug(frame)[0]
        self.assertTrue(blob.near_miss)
        self.assertIn("needs V 180+", blob.verdict)
        self.assertIn("V mean 140", blob.line(1))

    def test_describe_says_when_there_is_no_color_at_all(self):
        frame = self.frame(lambda f: None)
        lines = rv_debug.describe(frame, rv_debug.light_debug(frame))
        self.assertIn("no red, yellow or green patch", lines[1])

    def test_snapshot_writes_the_frame_and_an_annotated_copy(self):
        frame = self.frame(lambda f: cv2.circle(f, (320, 120), 35, (0, 0, 255), -1))
        with tempfile.TemporaryDirectory() as folder:
            marked = rv_debug.save_snapshot(frame, rv_debug.light_debug(frame), folder)
            files = sorted(path.name for path in Path(folder).iterdir())
            self.assertEqual(len(files), 2)
            self.assertTrue(marked.name.endswith("_annotated.jpg"))
            self.assertIsNotNone(cv2.imread(str(marked)))


class StreamSnapshotTests(unittest.TestCase):
    def test_debug_stream_passes_the_snap_event_and_collects_snapshot_paths(self):
        captured = {}

        class FakeProcess:
            def __init__(self, target, args, daemon):
                captured["args"] = args

            def start(self):
                pass

            def is_alive(self):
                return True

        queue = Queue()
        context = types.SimpleNamespace(Queue=lambda maxsize: queue, Event=lambda: types.SimpleNamespace(),
                                        Process=FakeProcess)
        stream = vision.VisionStream(context=context, debug=True)
        self.assertEqual(len(captured["args"]), 5)
        self.assertIs(captured["args"][3], stream.snap)
        queue.put(("snap", "/home/ezra/rv_snapshots/rv_1_annotated.jpg"))
        queue.put(vision.VisionSample(1, 0.0, (), 0.01, (), ("RVD frame: ...",)))
        sample = stream.latest()
        self.assertEqual(stream.snapshots, ["/home/ezra/rv_snapshots/rv_1_annotated.jpg"])
        self.assertEqual(sample.debug, ("RVD frame: ...",))
        plain = vision.VisionStream(context=context)
        self.assertEqual(len(captured["args"]), 3)
        self.assertIsNone(plain.snap)

    def test_rvd_rides_out_a_slow_frame_that_would_stop_rv(self):
        stream = types.SimpleNamespace(latest=lambda: vision.VisionSample(1, 0.0, (), 0.01))
        with self.assertRaisesRegex(vision.VisionFault, "stale"):
            vision.TrafficLight(stream).update(2.0)
        vision.TrafficLight(stream, max_age=vision.RVD_MAX_AGE).update(2.0)

    def test_rvd_prints_debug_lines_and_p_requests_a_snapshot(self):
        clock = FakeClock()
        messages = []
        snap = types.SimpleNamespace(set=lambda: messages.append("SNAP SET"))
        stream = types.SimpleNamespace(snap=snap, snapshots=[], closed=False,
                                       close=lambda: True)
        stream.latest = lambda: vision.VisionSample(round(clock.now * 50) // 5 + 1, (round(clock.now * 50) // 5) / 10,
                                                    (), 0.01, (), ("RVD frame: test",))
        keys = iter(["", "", "p", "", "", "s"])
        control = types.SimpleNamespace(clock=clock, log=messages.append,
                                        motors=types.SimpleNamespace(stop=lambda brake=None: None),
                                        tick_sleep=lambda started: clock.sleep(0.02))

        def poll():
            key = next(keys, "s")
            if key == "s":
                raise StopIteration
            return key

        control.poll = poll
        session = vision.VisionSession(stream_factory=lambda options, debug=False: stream)
        with self.assertRaises(StopIteration):
            session.readings(control, debug=True)
        self.assertIn("SNAP SET", messages)
        self.assertIn("RVD frame: test", messages)


if __name__ == "__main__":
    unittest.main()

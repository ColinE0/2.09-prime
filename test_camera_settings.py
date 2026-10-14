"""Verify saved/CLI camera options survive serialization into the camera worker."""

import pickle
import types
import unittest
from unittest.mock import patch

import app
import robot_settings
import vision_support as vision


class CameraSettingsTests(unittest.TestCase):
    def setUp(self):
        self.devices = []
        devices = self.devices

        class FakePicamera:
            def __init__(self, index):
                self.index = index
                self.stopped = self.closed = False
                devices.append(self)

            def create_video_configuration(self, **kwargs):
                return kwargs

            def configure(self, config):
                self.config = config

            def start(self, **kwargs):
                self.start_options = kwargs

            def stop(self):
                self.stopped = True

            def close(self):
                self.closed = True

        self.fake_modules = {"cv2": types.SimpleNamespace(),
                             "picamera2": types.SimpleNamespace(Picamera2=FakePicamera)}

    def open_options(self, config, argv=()):
        options = app.camera_options(robot_settings.validate(config), app.build_parser().parse_args(argv))
        # spawn transmits CameraOptions by pickle; mutable parent module globals
        # must not be needed by the child to recover these customized values.
        recovered = pickle.loads(pickle.dumps(options))
        self.assertEqual(recovered, options)
        with patch.dict("sys.modules", self.fake_modules):
            camera = vision.HeadlessCamera(recovered)
        device = self.devices[-1]
        camera.close()
        self.assertTrue(device.stopped)
        self.assertTrue(device.closed)
        return recovered, device

    def test_manual_exposure_and_index_survive_serialization(self):
        with patch.object(vision.settings, "CAMERA_EXPOSURE_US", 4000), \
                patch.object(vision.settings, "CAMERA_ANALOGUE_GAIN", 1.0):
            options, device = self.open_options({
                "camera_index": 2, "camera_rotate_180": False,
                "camera_exposure_us": 12345, "camera_analogue_gain": 2.5,
            })
        self.assertEqual(device.index, 2)
        self.assertFalse(options.rotate_180)
        self.assertEqual(device.config["controls"], {
            "FrameRate": 20.0, "AeEnable": False, "ExposureTime": 12345, "AnalogueGain": 2.5,
        })
        self.assertEqual(device.config["main"], {"size": (640, 480), "format": "RGB888"})
        self.assertEqual(device.config["raw"], {"size": (1640, 1232)})
        self.assertFalse(device.config["queue"])
        self.assertEqual(device.start_options, {"show_preview": False})

    def test_auto_exposure_omits_fixed_controls(self):
        for saved, argv in ((True, ()), (False, ("--camera-auto-exposure",))):
            with self.subTest(saved=saved):
                options, device = self.open_options({
                    "camera_auto_exposure": saved, "camera_exposure_us": 20000,
                    "camera_analogue_gain": 3,
                }, argv)
                self.assertTrue(options.auto_exposure)
                self.assertEqual(device.config["controls"], {"FrameRate": 20.0})

    def test_launch_overrides_camera_identity_without_mutating_saved_values(self):
        saved = robot_settings.validate({"camera_backend": "usb", "camera_index": 3,
                                         "camera_rotate_180": True})
        options, device = self.open_options(
            saved, ("--camera-backend", "csi", "--camera-index", "1", "--camera-upright"))
        self.assertEqual((options.backend, options.index, options.rotate_180), ("csi", 1, False))
        self.assertEqual(device.index, 1)
        self.assertEqual((saved["camera_backend"], saved["camera_index"], saved["camera_rotate_180"]),
                         ("usb", 3, True))

    def test_long_startup_does_not_accept_stale_processed_frames(self):
        with patch.object(vision, "VISION_START_TIMEOUT", 60):
            frame = vision.VisionSample(1, 10.0, (), .10)
            self.assertIs(vision.check_vision_sample(frame, 10.14), frame)
            with self.assertRaisesRegex(vision.VisionFault, "stale"):
                vision.check_vision_sample(frame, 10.76)
        self.assertEqual(vision.VISION_MAX_AGE, .75)


if __name__ == "__main__":
    unittest.main()

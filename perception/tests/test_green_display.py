"""Green extrapolation may supply display values but must not authorize motion."""
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from rescue_vision.dual import CameraPerception
from rescue_vision.simple_protocol import observations_from_snapshot
from rescue_vision.types import Detection


class GreenDisplayTests(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[1]
        self.config = json.loads((root / "config/dual_camera.json").read_text(encoding="utf-8"))
        self.calibration = SimpleNamespace(data={}, size=(640, 480), maximum=1000, valid=False,
            matrix=np.eye(3), distortion=np.zeros(5), project=lambda *args, **kwargs: None,
            homography=np.array([[10., 0, -3200], [0, 10., 0], [0, 0, 1.]]))
        self.session = CameraPerception("upper", self.calibration, "local_checked", self.config)
        self.image = np.zeros((480, 640, 3), np.uint8)
        self.detection = Detection("normal_material", .9, (380, 260, 420, 300))

    def test_expands_far_and_lateral_display_range_without_valid_position(self):
        result = self.session.process(self.image, [self.detection], 10., 10.01,
                                      {"expected": "normal_material", "source": "controller"})
        row = result["detections"][0]
        self.assertIsNone(row["position_mm"])
        self.assertFalse(row["position_valid"])
        self.assertFalse(result["motion_valid"])
        self.assertEqual(row["display_position_mm"]["x_right"], 800.)
        self.assertEqual(row["display_position_mm"]["y_forward"], 2990.)
        result.update(sequence=1, age_ms=10, mode="camera")
        observation = observations_from_snapshot({"cameras": {"upper": result, "lower": result}})[0]
        self.assertGreater(observation.objects[0].distance_mm, 3000)
        self.assertFalse(observation.objects[0].distance_valid)
        self.assertIsNotNone(observation.objects[0].angle)

    def test_singular_behind_vehicle_and_wire_overflow_return_unknown(self):
        for matrix in (np.zeros((3, 3)), np.diag([1., -1., 1.]),
                       np.diag([1000., 1000., 1.])):
            with self.subTest(matrix=matrix):
                self.calibration.homography = matrix
                self.assertIsNone(self.session.green_display_position(self.image, self.detection))

    def test_objects_in_gripper_roi_are_not_projected_onto_ground(self):
        self.config["gripper"]["roi_raw_xyxy"] = [100, 100, 300, 400]
        lower = CameraPerception("lower", self.calibration, "fit_only", self.config)
        detection = Detection("normal_material", .9, (150, 150, 200, 200))
        result = lower.process(self.image, [detection], 10., 10.01, {})
        self.assertIsNone(result["detections"][0]["position_mm"])
        self.assertNotIn("display_position_mm", result["detections"][0])


if __name__ == "__main__":
    unittest.main()

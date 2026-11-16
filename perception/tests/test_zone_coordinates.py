"""安全区 ROI 中心地面投影、测距补偿及主控协议回归。"""
from copy import deepcopy
import json
import math
from pathlib import Path
import unittest

import cv2
import numpy as np

from rescue_vision.dual import CameraPerception
from rescue_vision.geometry import GroundCalibration
from rescue_vision.simple_protocol import decode, encode, observations_from_snapshot
from rescue_vision.task_protocol import results_from_snapshot
from rescue_vision.types import Detection


ROOT = Path(__file__).resolve().parents[1]
ZONE_NAMES = ("red_material_zone", "blue_material_zone",
              "red_casualty_zone", "blue_casualty_zone")
BOX = (372., 384., 412., 424.)
TASK = dict(source="controller", contract="rescue_json_v2", session_id="zones",
            task_id="trip-1", seq=1, state_id=1, phase="approach", team_color="red",
            expected="normal_material", expected_kind="green", mechanism_state="open")


class ZoneCoordinatesTests(unittest.TestCase):
    def process(self, role, *, names=ZONE_NAMES, valid=True, box=BOX, distortion=None,
                output_max=None, grip_roi=None):
        cfg = json.loads((ROOT / "config/dual_camera.json").read_text(encoding="utf-8"))
        # r=20 cm 时原始距离为 r+0.01*r²=24 cm，补偿后应为 200 mm。
        cfg["distance_compensation"] = dict(enabled=True, error_coefficients_cm=[.01, 0., 0.],
                                            measured_range_cm=[1., 200.])
        cfg["gripper"] = dict(roi_raw_xyxy=grip_roi, validated=False)
        if output_max is not None:
            cfg["cameras"][role]["output_range_mm"][1] = output_max
        calibration = GroundCalibration(dict(
            validated=valid, image_size=[640, 480], coordinate_system="x_right_y_forward",
            camera_matrix=[[300., 0., 320.], [0., 300., 240.], [0., 0., 1.]],
            dist_coeffs=[0.] * 5 if distortion is None else distortion,
            ground_homography=[[2., 0., -640.], [0., -2., 1000.], [0., 0., 1.]],
            valid_pixel_polygon=[[0., 0.], [639., 0.], [639., 479.], [0., 479.]],
            forward_range_mm=[50., 1000.], bumper_forward_mm=0.,
            collection_center_right_mm=0.))
        session = CameraPerception(role, calibration, "validated" if valid else "fit_only", cfg)
        image = np.zeros((480, 640, 3), np.uint8)
        detections = [Detection(name, .9, box) for name in names]
        result = session.process(image, detections, 10., 10.01, TASK)
        result.update(mode="camera", sequence=1, task_current=True, age_ms=10)
        return result

    def snapshot(self, **changes):
        return dict(cameras={role: self.process(role, **changes) for role in ("upper", "lower")},
                    task=deepcopy(TASK), controller=deepcopy(TASK), model=dict(status="READY"))

    def test_all_four_zones_use_center_and_compensate_once_on_both_cameras(self):
        # ROI 中心 (392,404) 投影为 (144,192)，补偿后为 (120,160)。
        # 底边投影会给出不同距离，因此此断言能识别误用物块贴地点的情况。
        for role in ("upper", "lower"):
            for row in self.process(role)["detections"]:
                with self.subTest(role=role, zone=row["name"]):
                    self.assertEqual(row["position_quality"], "zone_roi_center")
                    self.assertTrue(row["position_valid"])
                    self.assertAlmostEqual(row["position_mm"]["x_right"], 120.)
                    self.assertAlmostEqual(row["position_mm"]["y_forward"], 160.)
                    self.assertAlmostEqual(row["position_mm"]["ground_range"], 200.)
                    self.assertAlmostEqual(row["position_mm"]["bearing_deg"], 36.86989764584402)
                    self.assertAlmostEqual(row["distance_compensation"]["raw_distance_mm"], 240.)
                    self.assertTrue(row["distance_compensation"]["applied"])

    def test_roi_center_is_undistorted_before_ground_projection(self):
        distortion = np.array([.08, -.03, .001, -.002, .005])
        matrix = np.array([[300., 0., 320.], [0., 300., 240.], [0., 0., 1.]])
        # 将已知理想中心正向畸变后作为检测 ROI 中心，检查能还原原地面点。
        pixels, _ = cv2.projectPoints(np.array([[[.24, 164. / 300., 1.]]]),
                                      np.zeros(3), np.zeros(3), matrix, distortion)
        u, v = pixels[0, 0]
        box = (u - 20, v - 20, u + 20, v + 20)
        for role in ("upper", "lower"):
            row = self.process(role, names=ZONE_NAMES[:1], box=box,
                               distortion=distortion.tolist())["detections"][0]
            self.assertAlmostEqual(row["position_mm"]["x_right"], 120., places=4)
            self.assertAlmostEqual(row["position_mm"]["y_forward"], 160., places=4)

    def test_rv2_and_json_return_zone_coordinates_through_existing_object_fields(self):
        snapshot = self.snapshot()
        for observation in observations_from_snapshot(snapshot):
            decoded = decode(encode(observation))
            self.assertEqual([obj.kind for obj in decoded.objects], list(ZONE_NAMES))
            for obj in decoded.objects:
                self.assertEqual(obj.distance_mm, 200)
                self.assertTrue(obj.distance_valid)
                self.assertEqual(obj.angle, 36.87)
                angle = math.radians(obj.angle)
                self.assertAlmostEqual(obj.distance_mm * math.sin(angle), 120., places=2)
                self.assertAlmostEqual(obj.distance_mm * math.cos(angle), 160., places=2)
        observations = [m for m in results_from_snapshot(snapshot) if m["msg_type"] == "observation"]
        self.assertEqual(len(observations), 2)
        for observation in observations:
            self.assertEqual([obj["kind"] for obj in observation["objects"]], list(ZONE_NAMES))
            for obj in observation["objects"]:
                self.assertEqual((obj["distance_mm"], obj["distance_valid"], obj["angle"]), (200, 1, 36.87))
        # 中心坐标只用于对准接近，不能据此声称已入库或投放成功。
        dock = next(m for m in results_from_snapshot(snapshot) if m["msg_type"] == "dock")
        self.assertEqual(dock["dock_valid"], 0)

    def test_unverified_calibration_keeps_numeric_coordinates_invalid(self):
        snapshot = self.snapshot(valid=False)
        for observation in observations_from_snapshot(snapshot):
            for obj in observation.objects:
                self.assertEqual(obj.distance_mm, 200)
                self.assertFalse(obj.distance_valid)
                self.assertEqual(obj.angle, 36.87)

    def test_clipped_roi_or_projection_outside_output_range_has_no_coordinate(self):
        for changes in (dict(box=(0., 384., 412., 424.)), dict(output_max=180.)):
            for observation in observations_from_snapshot(self.snapshot(**changes)):
                with self.subTest(changes=changes, role=observation.source):
                    for obj in observation.objects:
                        self.assertEqual(obj.distance_mm, -1)
                        self.assertFalse(obj.distance_valid)
                        self.assertIsNone(obj.angle)

    def test_zone_on_ground_is_projected_when_overlapping_gripper_roi(self):
        result = self.process("lower", grip_roi=[350, 360, 440, 450])
        for row in result["detections"]:
            self.assertTrue(row["inside_gripper_roi"])
            self.assertAlmostEqual(row["position_mm"]["ground_range"], 200.)

    def test_expired_and_missing_zones_do_not_keep_old_coordinates(self):
        snapshot = self.snapshot()
        for age in (150, 300):
            for camera in snapshot["cameras"].values():
                camera["age_ms"] = age
            for observation in observations_from_snapshot(snapshot):
                if age == 150:
                    self.assertTrue(observation.objects)
                    self.assertTrue(all(not obj.distance_valid for obj in observation.objects))
                else:
                    self.assertEqual((observation.status, observation.objects), ("stale", ()))
        for observation in observations_from_snapshot(self.snapshot(names=())):
            self.assertEqual((observation.status, observation.objects), ("empty", ()))


if __name__ == "__main__":
    unittest.main()

"""安全区 ROI 中心地面投影、测距补偿及主控协议回归。"""
from copy import deepcopy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from rescue_vision.dashboard import annotate_frame
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
    def process(self, role, *, names=ZONE_NAMES[:1], valid=True, box=BOX, distortion=None,
                output_max=None, grip_roi=None, polygon=None, task=TASK):
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
            valid_pixel_polygon=polygon or [[0., 0.], [639., 0.], [639., 479.], [0., 479.]],
            forward_range_mm=[50., 1000.], bumper_forward_mm=0.,
            collection_center_right_mm=0.))
        session = CameraPerception(role, calibration, "validated" if valid else "fit_only", cfg)
        image = np.zeros((480, 640, 3), np.uint8)
        detections = [Detection(name, .9, box) for name in names]
        result = session.process(image, detections, 10., 10.01, task)
        result.update(mode="camera", sequence=1, task_current=True, age_ms=10)
        return result

    def snapshot(self, **changes):
        return dict(cameras={role: self.process(role, **changes) for role in ("upper", "lower")},
                    task=deepcopy(TASK), controller=deepcopy(TASK), model=dict(status="READY"))

    def test_all_four_zones_use_center_and_compensate_once_on_both_cameras(self):
        # ROI 中心 (392,404) 投影为 (144,192)，补偿后为 (120,160)。
        # 底边投影会给出不同距离，因此此断言能识别误用物块贴地点的情况。
        for role in ("upper", "lower"):
            for name in ZONE_NAMES:
                with self.subTest(role=role, zone=name):
                    row, = self.process(role, names=(name,))["detections"]
                    self.assertEqual(row["name"], name)
                    self.assertEqual(row["zone_center_px"], (392., 404.))
                    self.assertEqual(row["position_quality"], "zone_roi_center")
                    self.assertTrue(row["position_valid"])
                    self.assertAlmostEqual(row["position_mm"]["x_right"], 120.)
                    self.assertAlmostEqual(row["position_mm"]["y_forward"], 160.)
                    self.assertAlmostEqual(row["position_mm"]["ground_range"], 200.)
                    self.assertAlmostEqual(row["position_mm"]["bearing_deg"], 36.86989764584402)
                    self.assertAlmostEqual(row["distance_compensation"]["raw_distance_mm"], 240.)
                    self.assertTrue(row["distance_compensation"]["applied"])

    def test_preview_marks_complete_zones_only(self):
        image = np.zeros((480, 640, 3), np.uint8)
        for role in ("upper", "lower"):
            complete = self.process(role, names=ZONE_NAMES[:1], output_max=180.)
            self.assertIsNotNone(complete["detections"][0]["position_mm"])
            marked = annotate_frame(image, complete, None)
            self.assertEqual(tuple(marked[404, 392]), (0, 215, 255))

            clipped = self.process(role, names=ZONE_NAMES[:1], box=(0., 384., 412., 424.))
            marked = annotate_frame(image, clipped, None)
            self.assertEqual(tuple(marked[404, 206]), (0, 0, 0))
            self.assertTrue(clipped["detections"][0]["position_valid"])

            material = self.process(role, names=("normal_material",))
            marked = annotate_frame(image, material, None)
            self.assertEqual(tuple(marked[404, 392]), (0, 0, 0))

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

    def test_rv2_rv3_rv4_and_json_return_zone_coordinates(self):
        for name in ZONE_NAMES:
            with self.subTest(zone=name):
                snapshot = self.snapshot(names=(name,))
                for observation in observations_from_snapshot(snapshot):
                    obj, = decode(encode(observation)).objects
                    self.assertEqual((obj.kind, obj.distance_mm, obj.distance_valid, obj.angle), (name, 200, True, 36.87))
                    angle = math.radians(obj.angle)
                    self.assertAlmostEqual(obj.distance_mm * math.sin(angle), 120., places=2)
                    self.assertAlmostEqual(obj.distance_mm * math.cos(angle), 160., places=2)
                    for version in (3, 4):
                        obj, = decode(encode(observation, version=version)).objects
                        self.assertEqual((obj.kind, obj.x_mm, obj.y_mm), (name, 120, 160))
                messages = results_from_snapshot(snapshot)
                observations = [m for m in messages if m["msg_type"] == "observation"]
                self.assertEqual(len(observations), 2)
                for observation in observations:
                    obj, = observation["objects"]
                    self.assertEqual((obj["kind"], obj["distance_mm"], obj["distance_valid"], obj["angle"]),
                                     (name, 200, 1, 36.87))
                    self.assertEqual((obj["x_mm"], obj["y_mm"]), (120, 160))
                dock = next(m for m in messages if m["msg_type"] == "dock")
                self.assertEqual(dock["dock_valid"], 0)

    def test_unverified_calibration_still_returns_valid_observations(self):
        snapshot = self.snapshot(valid=False)
        for observation in observations_from_snapshot(snapshot):
            obj, = observation.objects
            self.assertEqual(obj.distance_mm, 200)
            self.assertTrue(obj.distance_valid)
            self.assertEqual(obj.angle, 36.87)
        messages = results_from_snapshot(snapshot)
        status = next(msg for msg in messages if msg["msg_type"] == "status")
        self.assertEqual((status["vision_ready"], status["danger_flag"]), (0, 1))
        for message in messages:
            if message["msg_type"] == "observation":
                obj, = message["objects"]
                self.assertEqual(obj["distance_valid"], 1)

    def test_zone_observation_validity_is_independent_from_legacy_navigation(self):
        task = dict(TASK, contract="rescue_json_v1", expected=ZONE_NAMES[0])
        with patch("rescue_vision.dual.Confirmation.update", return_value=True):
            result = self.process("upper", valid=False, task=task)
        self.assertTrue(result["selected"]["position_valid"])
        self.assertFalse(result["motion_valid"])

    def test_orange_blue_and_black_use_box_contact_with_either_calibration_state(self):
        for name, kind in (("casualty", "orange"), ("hazard", "blue"), ("core_material", "black")):
            for valid in (False, True):
                for observation in observations_from_snapshot(self.snapshot(names=(name,), valid=valid)):
                    with self.subTest(kind=kind, calibration_valid=valid, camera=observation.source):
                        obj = decode(encode(observation, version=3)).objects[0]
                        self.assertEqual((obj.kind, obj.distance_valid), (kind, True))
                        # 框底边中点 (392,423) 投影为 (144,154)，随后执行径向补偿。
                        self.assertEqual((obj.x_mm, obj.y_mm), (122, 131))

    def test_first_frame_reports_estimated_centers_outside_projection_limits(self):
        narrow_polygon = [[0., 0.], [350., 0.], [350., 479.], [0., 479.]]
        for changes in (dict(output_max=180.), dict(polygon=narrow_polygon)):
            snapshot = self.snapshot(names=("blue_casualty_zone",), **changes)
            for observation in observations_from_snapshot({"cameras": snapshot["cameras"]}):
                with self.subTest(changes=changes, role=observation.source):
                    decoded = decode(encode(observation, version=4))
                    self.assertEqual(decoded.status, "ok")
                    obj, = decoded.objects
                    self.assertEqual((obj.kind, obj.x_mm, obj.y_mm, obj.distance_mm, obj.angle),
                                     ("blue_casualty_zone", 120, 160, 200, 36.87))
                    self.assertTrue(obj.distance_valid)

    def test_clipped_roi_sends_its_visible_center_as_an_estimate(self):
        expected_angle = round(math.degrees(math.atan2(-228., 192.)), 2)
        snapshot = self.snapshot(box=(0., 384., 412., 424.))
        for observation in observations_from_snapshot(snapshot):
            obj, = decode(encode(observation, version=4)).objects
            self.assertLess(obj.x_mm, 0)
            self.assertGreater(obj.y_mm, 0)
            self.assertEqual(obj.angle, expected_angle)
            self.assertTrue(obj.distance_valid)

    def test_unprojectable_detection_has_no_valid_distance(self):
        with patch.object(CameraPerception, "project_position_outside_calibration", return_value=None):
            snapshot = self.snapshot(output_max=180.)
        for observation in observations_from_snapshot(snapshot):
            obj, = decode(encode(observation, version=4)).objects
            self.assertEqual(obj.kind, ZONE_NAMES[0])
            self.assertEqual((obj.x_mm, obj.y_mm, obj.distance_mm, obj.angle), (None, None, -1, None))
            self.assertFalse(obj.distance_valid)

    def test_zone_on_ground_is_projected_when_overlapping_gripper_roi(self):
        result = self.process("lower", grip_roi=[350, 360, 440, 450])
        row, = result["detections"]
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

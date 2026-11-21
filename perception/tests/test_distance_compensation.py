"""Offline checks for model selection, radial geometry, and actual wire values."""
from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from rescue_vision.dual import CameraPerception, load_dual_config
from rescue_vision.fit_distance import fit_error_samples
from rescue_vision.geometry import RadialDistanceCompensation
from rescue_vision.simple_protocol import decode, encode, observations_from_snapshot
from rescue_vision.task_protocol import results_from_snapshot
from rescue_vision.types import Detection


ROOT = Path(__file__).resolve().parents[1]
REPORT = json.loads((ROOT / "config/dual_calibration/distance_compensation_fit.json").read_text(encoding="utf-8"))
MODEL = REPORT["distance_compensation"]


def measured_mm(true_cm, coefficients):
    return 10*(true_cm + np.polyval(coefficients, true_cm))


class CompensationTests(unittest.TestCase):
    def test_inverse_in_mm_preserves_bearing_on_and_off_axis(self):
        compensation = RadialDistanceCompensation(MODEL)
        for truth in (15., 20., 50., 100., 150.):
            raw = measured_mm(truth, MODEL["error_coefficients_cm"])
            for x_ratio in (-.6, 0., .6):
                point = (raw*x_ratio, raw*math.sqrt(1-x_ratio*x_ratio))
                corrected, diagnostic = compensation.apply(point)
                with self.subTest(truth=truth, x_ratio=x_ratio):
                    self.assertTrue(diagnostic["applied"])
                    self.assertAlmostEqual(math.hypot(*corrected), truth*10, places=9)
                    self.assertAlmostEqual(math.atan2(*corrected), math.atan2(*point), places=12)
                    self.assertAlmostEqual(diagnostic["raw_distance_mm"], raw)

    def test_linear_inverse(self):
        model = dict(enabled=True, error_coefficients_cm=[0., .2, -2.], measured_range_cm=[10., 200.])
        corrected, diagnostic = RadialDistanceCompensation(model).apply((0., 580.))
        self.assertTrue(diagnostic["applied"])
        self.assertAlmostEqual(corrected[1], 500.)

    def test_disabled_outside_and_zero_pass_through(self):
        for config in (None, {"enabled": False}, MODEL):
            compensation = RadialDistanceCompensation(config)
            for point in ((0., 0.), (30., 40.), (0., 2000.)):
                with self.subTest(config=config, point=point):
                    corrected, diagnostic = compensation.apply(point)
                    self.assertEqual(corrected, point)
                    self.assertFalse(diagnostic["applied"])
        raw = (300., 400.)
        self.assertEqual(RadialDistanceCompensation({"enabled": False}).apply(raw)[0], raw)

    def test_observed_range_endpoints_are_included(self):
        compensation = RadialDistanceCompensation(MODEL)
        for endpoint in MODEL["measured_range_cm"]:
            self.assertTrue(compensation.apply((0., endpoint*10))[1]["applied"])

    def test_malformed_or_non_invertible_model_rejected(self):
        bad = [dict(enabled="true"), dict(enabled=True),
               dict(MODEL, error_coefficients_cm=[True, 0., 0.]),
               dict(MODEL, error_coefficients_cm=[float("nan"), 0., 0.]),
               dict(MODEL, measured_range_cm=[100., 10.]),
               dict(MODEL, error_coefficients_cm=[0., -1., 0.]),
               dict(MODEL, error_coefficients_cm=[-.5, 0., 0.])]
        for config in bad:
            with self.subTest(config=config), self.assertRaises(ValueError):
                RadialDistanceCompensation(config)

    def test_both_runtime_configs_match_report(self):
        for name in ("dual_camera.json", "dual_orangepi.json"):
            cfg = load_dual_config(ROOT / "config" / name)
            self.assertEqual(cfg["distance_compensation"], MODEL)


class FittingTests(unittest.TestCase):
    def data(self, quadratic):
        groups = []
        for index, truth in enumerate((20., 40., 60., 80., 100.)):
            error = (.001*truth**2 if quadratic else 0.) + .03*truth - 1.
            groups.append(dict(id=index, x=0., y=truth,
                               samples=[dict(d_mm=10*(truth+error), valid=0)]*(index+1)))
        groups.append(dict(id=99, x=0., y=180., samples=[]))
        return dict(type="vision_error_samples", version=1,
                    units=dict(distance="cm", raw_distance="mm"), groups=groups)

    def test_compare_models_and_ignore_empty_groups(self):
        for quadratic in (False, True):
            result = fit_error_samples(self.data(quadratic))
            self.assertEqual(result["selected_degree"], 2 if quadratic else 1)
            self.assertEqual(result["distance_point_count"], 5)
            self.assertEqual(result["sample_count"], 15)
            self.assertEqual(result["control_valid_sample_count"], 0)
            self.assertEqual(result["excluded_groups"][0]["group_id"], 99)
            self.assertLess(result["evaluation"]["corrected_all_samples_rmse_cm"], 1e-10)

    def test_equal_distance_weight_with_duplicate_positions(self):
        data = self.data(True)
        before = fit_error_samples(data)
        duplicate = deepcopy(data["groups"][0])
        duplicate["id"] = 100
        duplicate["samples"] *= 100
        data["groups"].append(duplicate)
        after = fit_error_samples(data)
        self.assertEqual(after["distance_point_count"], 5)
        np.testing.assert_allclose(after["distance_compensation"]["error_coefficients_cm"],
                                   before["distance_compensation"]["error_coefficients_cm"], atol=1e-12)

    def test_invalid_units_and_insufficient_distances_rejected(self):
        data = self.data(True)
        data["units"]["raw_distance"] = "cm"
        with self.assertRaises(ValueError):
            fit_error_samples(data)
        data = self.data(True)
        data["groups"] = data["groups"][:3]
        with self.assertRaises(ValueError):
            fit_error_samples(data)


class WireIntegrationTests(unittest.TestCase):
    def observation(self, role, *, valid=True, inside_grip=False, model=None, display_only=False,
                    name="normal_material", box=(300., 150., 340., 201.)):
        cfg = json.loads((ROOT / "config/dual_camera.json").read_text(encoding="utf-8"))
        if model is not None:
            cfg["distance_compensation"] = model
        if inside_grip:
            cfg["gripper"]["roi_raw_xyxy"] = [280, 120, 370, 240]
        truth = 120. if display_only else 50. if role == "upper" else 20.
        raw = measured_mm(truth, MODEL["error_coefficients_cm"])
        calibration = SimpleNamespace(data={}, size=(640, 480), maximum=1000., valid=valid,
                                      matrix=np.eye(3), distortion=np.zeros(5),
                                      homography=np.array([[0., 0., raw*.6], [0., 0., raw*.8], [0., 0., 1.]]),
                                      project=lambda pixel, require_valid: None if display_only else (raw*.6, raw*.8))
        session = CameraPerception(role, calibration, "validated" if valid else "local_checked", cfg)
        image = np.zeros((480, 640, 3), np.uint8)
        image[150:201, 300:340] = (0, 255, 0)
        detection = [Detection(name, .9, box)]
        task = dict(source="controller", contract="rescue_json_v2", expected=name,
                    session_id="test", task_id="test", phase="approach", mechanism_state="open",
                    team_color="red")
        for stamp in (10., 10.05, 10.1):
            result = session.process(image, detection, stamp, stamp+.01, task)
        result.update(mode="camera", sequence=3, task_current=True, age_ms=10)
        return result, task, truth

    def test_both_cameras_send_compensated_rv1_rv2_and_json_distances(self):
        cameras, truths = {}, {}
        for role in ("upper", "lower"):
            cameras[role], task, truths[role] = self.observation(role)
            row = cameras[role]["detections"][0]
            self.assertTrue(row["distance_compensation"]["applied"])
            self.assertTrue(row["position_valid"])
            self.assertAlmostEqual(row["position_mm"]["ground_range"], truths[role]*10)
            self.assertAlmostEqual(row["position_mm"]["bearing_deg"], math.degrees(math.atan2(.6, .8)))
        snapshot = dict(cameras=cameras, task=task, controller=task, model=dict(status="READY"))
        for observation in observations_from_snapshot(snapshot):
            self.assertEqual(observation.objects[0].distance_mm, truths[observation.source]*10)
            for version in (1, 2):
                decoded = decode(encode(observation, version=version))
                self.assertEqual(decoded.objects[0].distance_mm, truths[observation.source]*10)
        json_observations = [m for m in results_from_snapshot(snapshot) if m["msg_type"] == "observation"]
        for observation in json_observations:
            self.assertEqual(observation["objects"][0]["distance_mm"], truths[observation["source"]]*10)

    def test_unverified_calibration_keeps_coordinates_separate_from_motion(self):
        result, _, _ = self.observation("upper", valid=False)
        row = result["detections"][0]
        self.assertTrue(row["distance_compensation"]["applied"])
        self.assertTrue(row["position_valid"])
        self.assertFalse(result["motion_valid"])

    def test_held_object_is_not_projected_or_compensated(self):
        result, _, _ = self.observation("lower", inside_grip=True)
        row = result["detections"][0]
        self.assertIsNone(row["position_mm"])
        self.assertNotIn("distance_compensation", row)

    def test_all_four_return_one_position_field_and_valid_coordinates_while_fresh(self):
        colors = (("normal_material", "green"), ("casualty", "orange"),
                  ("hazard", "blue"), ("core_material", "black"))
        for name, kind in colors:
            with self.subTest(kind=kind):
                result, _, truth = self.observation("upper", name=name, display_only=True)
                row = result["detections"][0]
                self.assertTrue(row["position_valid"])
                self.assertEqual(row["position_quality"], "position")
                self.assertNotIn("display_position_mm", row)
                self.assertFalse(result["motion_valid"])
                self.assertTrue(row["distance_compensation"]["applied"])
                self.assertAlmostEqual(row["position_mm"]["ground_range"], truth*10)
                snapshot = {"cameras": {"upper": result, "lower": dict(status="NO_TARGET")}}
                observation = observations_from_snapshot(snapshot)[0]
                for version in (2, 3):
                    obj = decode(encode(observation, version=version)).objects[0]
                    self.assertEqual((obj.kind, obj.distance_mm, obj.distance_valid), (kind, truth*10, True))
                    self.assertAlmostEqual(obj.angle, math.degrees(math.atan2(.6, .8)), places=2)
                    if version == 3:
                        self.assertEqual((obj.x_mm, obj.y_mm), (truth*6, truth*8))
                result["age_ms"] = 150
                self.assertFalse(observations_from_snapshot(snapshot)[0].objects[0].distance_valid)

    def test_clipped_or_held_box_contact_has_no_position(self):
        for name in ("casualty", "hazard", "core_material"):
            for role, changes in (("upper", dict(box=(0., 150., 40., 201.))),
                                  ("lower", dict(inside_grip=True))):
                with self.subTest(name=name, role=role):
                    result, _, _ = self.observation(role, name=name, display_only=True, **changes)
                    row = result["detections"][0]
                    self.assertIsNone(row["position_mm"])
                    self.assertNotIn("display_position_mm", row)
                    self.assertFalse(row["position_valid"])
                    self.assertNotIn("distance_compensation", row)

    def test_json_blue_coordinates_are_valid_alongside_hazard_status(self):
        upper, task, truth = self.observation("upper", name="hazard", display_only=True)
        lower, _, _ = self.observation("lower")
        snapshot = dict(cameras=dict(upper=upper, lower=lower), task=task,
                        controller=task, model=dict(status="READY"))
        self.assertTrue(upper["hazard_blocked"])
        messages = results_from_snapshot(snapshot)
        self.assertEqual(messages[0]["danger_flag"], 1)
        observation = next(m for m in messages if m.get("source") == "upper")
        self.assertEqual(observation["danger_flag"], 1)
        obj = observation["objects"][0]
        self.assertEqual((obj["kind"], obj["distance_mm"], obj["distance_valid"]), ("blue", truth*10, 1))

    def test_disabled_config_sends_original_distance(self):
        result, _, truth = self.observation("upper", model={"enabled": False})
        row = result["detections"][0]
        self.assertFalse(row["distance_compensation"]["applied"])
        self.assertAlmostEqual(row["position_mm"]["ground_range"],
                               measured_mm(truth, MODEL["error_coefficients_cm"]))


if __name__ == "__main__":
    unittest.main()

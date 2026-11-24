"""安全区布局纠正、去重及入仓互锁回归。"""
import json
from pathlib import Path
import unittest

import numpy as np

from rescue_vision.dashboard import annotate_frame
from rescue_vision.dashboard_state import DashboardState
from rescue_vision.dual import CameraPerception
from rescue_vision.geometry import GroundCalibration
from rescue_vision.simple_protocol import decode, encode, observations_from_snapshot
from rescue_vision.types import Detection


ROOT = Path(__file__).resolve().parents[1]
RED_MATERIAL = "red_material_zone"
RED_CASUALTY = "red_casualty_zone"
BLUE_MATERIAL = "blue_material_zone"
BLUE_CASUALTY = "blue_casualty_zone"
LEFT = (30., 190., 170., 230.)
RIGHT = (150., 191., 290., 231.)
LEFT_DUPLICATE = (31., 190., 171., 230.)
FRONT = (32., 240., 172., 280.)


class ZoneLayoutTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((ROOT / "config/dual_camera.json").read_text(encoding="utf-8"))
        self.image = np.zeros((480, 640, 3), np.uint8)
        self.stamp = 10.
        self.task = dict(source="debug", session_id="desktop", task_id="0", state="IDLE")

    def camera(self, role="upper", *, forward=True, kind="camera"):
        cfg = dict(self.cfg, cameras={name: dict(settings) for name, settings in self.cfg["cameras"].items()})
        upper = cfg["cameras"]["upper"]
        upper.pop("zone_layout_forward", None)
        if forward is not None:
            upper["zone_layout_forward"] = forward
        cfg["cameras"][role]["kind"] = kind
        calibration = GroundCalibration.load(ROOT / f"config/dual_calibration/{role}_ground.json")
        return CameraPerception(role, calibration, "fit_only", cfg)

    def frame(self, camera, *detections):
        self.stamp += .05
        result = camera.process(self.image, detections, self.stamp, self.stamp + .01, self.task)
        result.update(mode="camera", sequence=round(self.stamp * 100))
        self.assertEqual(result["status"], "OK")
        return result

    def test_two_casualty_boxes_in_same_half_are_corrected_before_tracking(self):
        camera = self.camera()
        result = self.frame(camera, Detection(RED_CASUALTY, .8, RIGHT), Detection(RED_CASUALTY, .9, LEFT))
        rows = {row["name"]: row for row in result["detections"]}
        self.assertEqual(set(rows), {RED_MATERIAL, RED_CASUALTY})
        self.assertEqual(rows[RED_MATERIAL]["box"], list(LEFT))
        self.assertEqual(rows[RED_CASUALTY]["box"], list(RIGHT))
        self.assertIsNone(rows[RED_MATERIAL]["confidence"])
        self.assertEqual((rows[RED_MATERIAL]["model_name"], rows[RED_MATERIAL]["model_confidence"]),
                         (RED_CASUALTY, .9))
        self.assertEqual(rows[RED_CASUALTY]["confidence"], .8)
        self.assertEqual(result["zone_layout"]["red"]["status"], "paired")
        annotate_frame(self.image, result, None)

        state = DashboardState(self.cfg, clock=lambda: self.stamp + .01)
        state.publish_result("upper", result)
        snapshot = state.snapshot()
        self.assertEqual({row["name"] for row in snapshot["cameras"]["upper"]["detections"]}, set(rows))
        wire = decode(encode(observations_from_snapshot(snapshot)[0], version=4))
        self.assertEqual({obj.kind for obj in wire.objects}, set(rows))

        next_frame = self.frame(camera, Detection(RED_MATERIAL, .95, LEFT), Detection(RED_CASUALTY, .8, RIGHT))
        self.assertEqual({row["name"]: row["target_id"] for row in next_frame["detections"]},
                         {name: row["target_id"] for name, row in rows.items()})

    def test_overlapping_dual_labels_are_deduplicated_before_pairing(self):
        result = self.frame(self.camera(), Detection(RED_CASUALTY, .9, LEFT),
                            Detection(RED_MATERIAL, .85, LEFT_DUPLICATE), Detection(RED_CASUALTY, .7, RIGHT))
        rows = {row["name"]: row for row in result["detections"]}
        self.assertEqual(len(result["detections"]), 2)
        self.assertEqual(rows[RED_MATERIAL]["box"], list(LEFT))
        self.assertEqual(rows[RED_CASUALTY]["box"], list(RIGHT))
        info = result["zone_layout"]["red"]
        self.assertEqual(info["status"], "paired")
        self.assertEqual(info["duplicates_removed"], 1)
        self.assertEqual(len(info["candidates"]), 2)

    def test_single_clean_box_keeps_its_model_class(self):
        camera = self.camera()
        for box in (LEFT, (420., 190., 560., 230.)):
            with self.subTest(box=box):
                result = self.frame(camera, Detection(RED_CASUALTY, .9, box), Detection(RED_MATERIAL, .2, box))
                row, = result["detections"]
                self.assertEqual((row["name"], row["confidence"], row["box"]), (RED_CASUALTY, .9, list(box)))

    def test_single_conflicted_box_is_diagnostic_only(self):
        result = self.frame(self.camera(), Detection(RED_CASUALTY, .9, LEFT),
                            Detection(RED_MATERIAL, .85, LEFT_DUPLICATE))
        self.assertEqual(result["detections"], [])
        info = result["zone_layout"]["red"]
        self.assertEqual((info["status"], info["duplicates_removed"]), ("ambiguous", 1))
        candidate, = info["candidates"]
        self.assertIsNone(candidate["final_name"])
        self.assertEqual((candidate["name"], candidate["confidence"], candidate["box"]),
                         (RED_CASUALTY, .9, list(LEFT)))
        self.assertEqual(candidate["duplicates"], [
            {"name": RED_MATERIAL, "confidence": .85, "box": list(LEFT_DUPLICATE)},
        ])

    def test_front_back_boxes_do_not_infer_a_pair(self):
        result = self.frame(self.camera(), Detection(RED_CASUALTY, .9, LEFT),
                            Detection(RED_CASUALTY, .85, FRONT))
        self.assertEqual(result["detections"], [])
        self.assertEqual(result["zone_layout"]["red"]["status"], "ambiguous")

    def test_multiple_plausible_pairs_are_diagnostic_only(self):
        result = self.frame(self.camera(), Detection(RED_CASUALTY, .9, LEFT),
                            Detection(RED_CASUALTY, .85, RIGHT),
                            Detection(RED_CASUALTY, .8, (270., 192., 410., 232.)))
        self.assertEqual(result["detections"], [])
        self.assertEqual(result["zone_layout"]["red"]["status"], "ambiguous")

    def test_red_and_blue_pairs_are_independent(self):
        blue_left = (350., 280., 490., 320.)
        blue_right = (470., 281., 610., 321.)
        result = self.frame(self.camera(), Detection(BLUE_MATERIAL, .9, blue_right),
                            Detection(RED_CASUALTY, .9, LEFT), Detection(BLUE_MATERIAL, .8, blue_left),
                            Detection(RED_CASUALTY, .8, RIGHT))
        self.assertEqual({row["name"]: tuple(row["box"]) for row in result["detections"]},
                         {RED_MATERIAL: LEFT, RED_CASUALTY: RIGHT,
                          BLUE_MATERIAL: blue_left, BLUE_CASUALTY: blue_right})
        self.assertEqual({team: info["status"] for team, info in result["zone_layout"].items()},
                         {"red": "paired", "blue": "paired"})

    def test_missing_or_disabled_layout_lower_camera_and_video_keep_model_classes(self):
        for camera in (self.camera(forward=None), self.camera(forward=False),
                       self.camera("lower"), self.camera(kind="video")):
            with self.subTest(role=camera.role, settings=camera.cfg["cameras"][camera.role]):
                result = self.frame(camera, Detection(RED_CASUALTY, .9, LEFT),
                                    Detection(RED_CASUALTY, .85, RIGHT))
                self.assertFalse(result["zone_layout_forward"])
                self.assertEqual(result["zone_layout"]["red"]["status"], "ambiguous")
                self.assertEqual(result["detections"], [])
                result = self.frame(camera, Detection(RED_CASUALTY, .9, LEFT),
                                    Detection(RED_MATERIAL, .85, RIGHT))
                self.assertEqual({row["name"]: tuple(row["box"]) for row in result["detections"]},
                                 {RED_CASUALTY: LEFT, RED_MATERIAL: RIGHT})
                self.assertEqual(result["zone_layout"]["red"]["status"], "model")

    def test_trusted_pair_corrects_both_opposite_model_labels(self):
        result = self.frame(self.camera(), Detection(RED_CASUALTY, .9, LEFT), Detection(RED_MATERIAL, .85, RIGHT))
        self.assertTrue(result["zone_layout_forward"])
        rows = {row["name"]: row for row in result["detections"]}
        self.assertEqual(set(rows), {RED_MATERIAL, RED_CASUALTY})
        self.assertEqual((rows[RED_MATERIAL]["box"], rows[RED_MATERIAL]["model_name"]),
                         (list(LEFT), RED_CASUALTY))
        self.assertEqual((rows[RED_CASUALTY]["box"], rows[RED_CASUALTY]["model_name"]),
                         (list(RIGHT), RED_MATERIAL))
        self.assertTrue(all(row["confidence"] is None for row in rows.values()))
        self.assertEqual(result["zone_layout"]["red"]["status"], "paired")

    def test_opposite_model_order_is_preserved_when_geometry_is_untrusted(self):
        result = self.frame(self.camera(), Detection(RED_CASUALTY, .9, LEFT), Detection(RED_MATERIAL, .85, FRONT))
        self.assertEqual({row["name"]: tuple(row["box"]) for row in result["detections"]},
                         {RED_CASUALTY: LEFT, RED_MATERIAL: FRONT})
        info = result["zone_layout"]["red"]
        self.assertEqual((info["status"], info["reason"]), ("model", "opposite_model_order"))

    def test_field_blue_pair_stays_stable_with_or_without_duplicate(self):
        camera = self.camera()
        # 上视现场记录中的模型候选，右框触边，两框中心均位于右半画面。
        frames = (
            (15871, Detection(BLUE_CASUALTY, .8837890625, (212.5, 189.75, 466., 263.75)),
             Detection(BLUE_MATERIAL, .791015625, (417.5, 188.125, 640., 274.875)),
             (Detection(BLUE_MATERIAL, .53564453125, (212.4375, 189.875, 466.0625, 264.125)),)),
            (15873, Detection(BLUE_CASUALTY, .89794921875, (212.5, 190., 466., 263.5)),
             Detection(BLUE_MATERIAL, .78857421875, (417.5, 187.375, 640., 274.625)), ()),
        )
        tracked = set()
        for sequence, left, right, duplicates in frames:
            with self.subTest(sequence=sequence):
                result = self.frame(camera, left, right, *duplicates)
                result["sequence"] = sequence
                info = result["zone_layout"]["blue"]
                self.assertEqual((info["status"], info["duplicates_removed"]), ("paired", len(duplicates)))
                material, casualty = sorted(result["detections"], key=lambda row: row["box"][0])
                self.assertEqual((material["name"], casualty["name"]), (BLUE_MATERIAL, BLUE_CASUALTY))
                self.assertEqual((material["box"], casualty["box"]), (list(left.box), list(right.box)))
                self.assertIn("zone_center_px", material)
                self.assertNotIn("zone_center_px", casualty)
                self.assertIsNotNone(casualty["position_mm"])
                self.assertTrue(casualty["position_valid"])
                tracked.add((material["target_id"], casualty["target_id"]))

                snapshot = {"cameras": {"upper": result, "lower": {"status": "NO_TARGET"}}}
                wire = decode(encode(observations_from_snapshot(snapshot)[0], version=4))
                self.assertEqual(wire.frame_id, sequence)
                self.assertEqual([obj.kind for obj in wire.objects], [BLUE_MATERIAL, BLUE_CASUALTY])
                self.assertTrue(wire.objects[1].distance_valid)
                self.assertGreater(wire.objects[1].distance_mm, 0)
        self.assertEqual(len(tracked), 1)

    def test_close_blue_pair_in_upper_frame_4246_assigns_position_to_material(self):
        left = Detection(BLUE_CASUALTY, .91015625, (284.5, 184.75, 514.5, 261.75))
        right = Detection(BLUE_MATERIAL, .84033203125, (412.875, 172.5625, 640., 239.9375))
        result = self.frame(self.camera(), left, right)
        self.assertEqual(result["zone_layout"]["blue"]["status"], "paired")
        rows = {row["name"]: row for row in result["detections"]}
        self.assertEqual(rows[BLUE_MATERIAL]["box"], list(left.box))
        self.assertIsNotNone(rows[BLUE_MATERIAL]["position_mm"])
        self.assertEqual(rows[BLUE_CASUALTY]["box"], list(right.box))
        self.assertTrue(rows[BLUE_CASUALTY]["position_valid"])

    def test_left_edge_pair_reports_estimated_center_without_confirming_entry(self):
        camera = self.camera()
        left = Detection(RED_CASUALTY, .9, (0., 190., 120., 230.))
        right = Detection(RED_CASUALTY, .85, (110., 191., 250., 231.))
        obj = Detection("normal_material", .92, (52., 199., 69., 221.))
        for _ in range(3):
            result = self.frame(camera, left, right, obj)
            self.assertEqual(result["zone_layout"]["red"]["status"], "paired")
            rows = {row["name"]: row for row in result["detections"]}
            self.assertEqual(rows[RED_MATERIAL]["box"], list(left.box))
            self.assertEqual(rows[RED_CASUALTY]["box"], list(right.box))
            self.assertNotIn("zone_center_px", rows[RED_MATERIAL])
            self.assertEqual(rows[RED_MATERIAL]["position_quality"], "position")
            self.assertTrue(rows[RED_MATERIAL]["position_valid"])
            entry = rows[obj.name]["zone_entry"]
            self.assertEqual(entry["confirm_count"], 0)
            self.assertFalse(entry["object_inside"])
            self.assertFalse(entry["result_valid"])

    def test_edge_pairs_still_require_horizontal_size_and_vertical_alignment(self):
        cases = (
            ((0., 190., 50., 230.), (60., 191., 200., 231.)),  # 剩余宽度不足
            ((0., 190., 120., 230.), (110., 216., 250., 256.)),  # 纵向错位
            ((30., 0., 170., 40.), (150., 1., 290., 41.)),  # 顶边裁剪
            ((30., 439., 170., 479.), (150., 440., 290., 480.)),  # 底边裁剪
        )
        for left, right in cases:
            with self.subTest(left=left, right=right):
                result = self.frame(self.camera(), Detection(RED_CASUALTY, .9, left),
                                    Detection(RED_CASUALTY, .85, right))
                self.assertEqual(result["zone_layout"]["red"]["status"], "ambiguous")
                self.assertEqual(result["detections"], [])

    def test_ambiguity_clears_confirmed_entry_and_preserves_rv4_pickup_filter(self):
        camera = self.camera()
        material = Detection("normal_material", .95, (74., 201., 89., 222.))
        left = Detection(RED_CASUALTY, .9, LEFT)
        right = Detection(RED_CASUALTY, .8, RIGHT)

        def entry(result):
            return next(row["zone_entry"] for row in result["detections"] if row["name"] == material.name)

        for count in (1, 2, 3):
            initial = entry(self.frame(camera, material, left, right))
            self.assertEqual(initial["confirm_count"], count)
            self.assertEqual(initial["zone_kind"], RED_MATERIAL)
        self.assertTrue(initial["object_inside"])

        conflict = Detection(RED_MATERIAL, .86, LEFT)
        for _ in range(4):
            result = self.frame(camera, material, left, conflict)
            ambiguous = entry(result)
            self.assertEqual((ambiguous["reason"], ambiguous["confirm_count"]), ("zone_ambiguous", 0))
            self.assertFalse(ambiguous["result_valid"])
            self.assertFalse(ambiguous["object_inside"])
            snapshot = {"cameras": {"upper": result, "lower": {"status": "NO_TARGET"}}}
            obj, = decode(encode(observations_from_snapshot(snapshot)[0], version=4)).objects
            self.assertEqual(obj.kind, "green")
            self.assertTrue(obj.in_safety_zone)
            self.assertFalse(obj.inside_valid)
            self.assertFalse(obj.object_inside)

        self.assertEqual(entry(self.frame(camera, material, left, right))["confirm_count"], 1)


if __name__ == "__main__":
    unittest.main()

"""入仓三帧确认、错误区域拒绝及RV3跨字段约束回归。"""
from dataclasses import replace
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from rescue_vision.dashboard import ControllerLink
from rescue_vision.dashboard_state import DashboardState
from rescue_vision.dual import CameraPerception
from rescue_vision.geometry import GroundCalibration
from rescue_vision.simple_protocol import SimpleObject, SimpleObservation, decode, encode, observations_from_snapshot
from rescue_vision.task_protocol import results_from_snapshot
from rescue_vision.types import Detection


ROOT = Path(__file__).resolve().parents[1]
OBJECT_BOX = (280., 250., 320., 300.)
ZONE_BOX = (200., 200., 400., 400.)


class ZoneEntryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((ROOT / "config/dual_camera.json").read_text(encoding="utf-8"))
        self.task = dict(source="debug", session_id="entry", task_id="trip", expected=None)
        self.image = np.zeros((480, 640, 3), np.uint8)
        self.image[250:300, 280:320] = (0, 255, 0)
        self.time = 10.

    def session(self, role="upper"):
        calibration = GroundCalibration.load(ROOT / f"config/dual_calibration/{role}_ground.json")
        return CameraPerception(role, calibration, "fit_only", self.cfg)

    def frame(self, session, *, name="normal_material", zone="red_material_zone", box=OBJECT_BOX,
              zone_box=ZONE_BOX, extra=(), age=.01, missing=False):
        self.time += .05
        detections = [] if missing else [Detection(name, .9, box)]
        if zone is not None:
            detections.append(Detection(zone, .9, zone_box))
        detections.extend(extra)
        result = session.process(self.image, detections, self.time, self.time + age, self.task)
        result.update(mode="camera", sequence=round(self.time * 100), age_ms=round(age * 1000))
        return result

    def test_three_frames_for_material_and_casualty_zones_on_both_cameras(self):
        for role in ("upper", "lower"):
            for name in ("normal_material", "core_material", "casualty"):
                for team in ("red", "blue"):
                    session = self.session(role)
                    zone = f"{team}_{'casualty' if name == 'casualty' else 'material'}_zone"
                    for number in (1, 2, 3):
                        entry = self.frame(session, name=name, zone=zone)["detections"][0]["zone_entry"]
                        with self.subTest(role=role, name=name, team=team, frame=number):
                            self.assertEqual(entry["zone_kind"], zone)
                            self.assertEqual(entry["confirm_count"], number)
                            self.assertEqual(entry["object_inside"], number == 3)
                            self.assertEqual(entry["result_valid"], number == 3)

    def test_center_inside_but_roi_crossing_boundary_is_outside(self):
        session = self.session()
        for _ in range(4):
            entry = self.frame(session, box=(190., 250., 260., 300.))["detections"][0]["zone_entry"]
            self.assertTrue(entry["result_valid"])
            self.assertFalse(entry["object_inside"])
            self.assertEqual(entry["reason"], "outside")

    def test_wrong_zone_kind_and_ambiguous_containment_never_confirm(self):
        for name, zone, extra in (
                ("casualty", "red_material_zone", ()),
                ("normal_material", "red_casualty_zone", ()),
                ("normal_material", "red_material_zone", (Detection("blue_material_zone", .9, ZONE_BOX),))):
            session = self.session()
            for _ in range(4):
                entry = self.frame(session, name=name, zone=zone, extra=extra)["detections"][0]["zone_entry"]
                self.assertFalse(entry["result_valid"])
                self.assertFalse(entry["object_inside"])

    def test_missing_or_outside_frame_resets_confirmation(self):
        for interruption in (dict(missing=True), dict(zone=None), dict(box=(190., 250., 260., 300.))):
            session = self.session()
            for _ in range(3):
                self.frame(session)
            self.frame(session, **interruption)
            entry = self.frame(session)["detections"][0]["zone_entry"]
            self.assertEqual(entry["confirm_count"], 1)
            self.assertFalse(entry["object_inside"])

    def test_new_object_identity_or_task_requires_three_new_frames(self):
        session = self.session()
        for _ in range(3):
            self.frame(session)
        entry = self.frame(session, box=(340., 310., 380., 360.))["detections"][0]["zone_entry"]
        self.assertEqual(entry["confirm_count"], 1)
        self.task["task_id"] = "next-trip"
        entry = self.frame(session, box=(340., 310., 380., 360.))["detections"][0]["zone_entry"]
        self.assertEqual(entry["confirm_count"], 1)

    def test_clipped_roi_and_overlapping_objects_do_not_confirm(self):
        for changes in (dict(zone_box=(0., 200., 400., 400.)),
                        dict(extra=(Detection("core_material", .9, OBJECT_BOX),)),
                        dict(box=(0., 250., 320., 300.))):
            session = self.session()
            for _ in range(4):
                result = self.frame(session, **changes)
                self.assertFalse(result["detections"][0]["zone_entry"]["object_inside"])

    def test_slow_frames_do_not_accumulate_and_snapshot_cannot_keep_expired_entry(self):
        session = self.session()
        for _ in range(3):
            result = self.frame(session, age=.16)
            self.assertFalse(result["detections"][0]["zone_entry"]["object_inside"])
        result = self.frame(session)
        self.assertEqual(result["detections"][0]["zone_entry"]["confirm_count"], 1)
        for _ in range(2):
            result = self.frame(session)
        state = DashboardState(self.cfg, clock=lambda: self.time + .15)
        state.publish_result("upper", result)
        row = state.snapshot()["cameras"]["upper"]["detections"][0]
        self.assertFalse(row["zone_entry"]["object_inside"])
        self.assertFalse(row["zone_entry"]["result_valid"])

    def test_rv3_transmits_entry_even_when_ground_calibration_is_unverified(self):
        session = self.session()
        for _ in range(3):
            result = self.frame(session)
        snapshot = dict(cameras=dict(upper=result, lower=dict(status="NO_TARGET")))
        observation = observations_from_snapshot(snapshot)[0]
        obj = decode(encode(observation, version=3)).objects[0]
        self.assertTrue(obj.distance_valid)
        self.assertEqual(obj.zone_kind, "red_material_zone")
        self.assertTrue(obj.inside_valid)
        self.assertTrue(obj.object_inside)
        result["age_ms"] = 150
        obj = observations_from_snapshot(snapshot)[0].objects[0]
        self.assertFalse(obj.distance_valid)
        self.assertFalse(obj.inside_valid)
        self.assertFalse(obj.object_inside)

    def test_json_task_entry_requires_the_locked_object_and_own_correct_zone(self):
        session = self.session()
        for _ in range(3):
            result = self.frame(session)
        result["task_current"] = True
        task = dict(source="controller", contract="rescue_json_v2", task_id="trip", phase="deliver",
                    expected_kind="green", team_color="red", mechanism_state="open",
                    locked_target_id=result["detections"][0]["target_id"])
        snapshot = dict(task=task, controller=task, model=dict(status="READY"), cameras=dict(upper=result,
                        lower=dict(status="OK", task_current=True, mode="camera", age_ms=10, detections=[])))

        def delivery():
            return next(message for message in results_from_snapshot(snapshot) if message["msg_type"] == "delivery")

        message = delivery()
        self.assertEqual((message["result_valid"], message["object_inside"]), (1, 1))
        self.assertEqual((message["entry_allowed"], message["delivery_confirmed"]), (0, 0))
        task["team_color"] = "blue"
        self.assertEqual(delivery()["object_inside"], 0)
        task.update(team_color="red", locked_target_id=0)
        self.assertEqual(delivery()["object_inside"], 0)

    def test_delivery_check_reconfirms_retained_object_after_release(self):
        self.cfg["gripper"] = dict(roi_raw_xyxy=[20, 20, 150, 150], validated=True)
        upper, lower = self.session(), self.session("lower")
        self.task.update(source="controller", contract="rescue_json_v2", phase="release",
                         state_id=1, team_color="red", expected="normal_material",
                         expected_kind="green", mechanism_state="open")
        self.task["locked_target_id"] = self.frame(upper)["detections"][0]["target_id"]

        def capture(*, lower_age=.01, lower_extra=()):
            return dict(upper=dict(self.frame(upper), task_current=True),
                        lower=dict(self.frame(lower, age=lower_age, extra=lower_extra), task_current=True))

        def delivery(cameras):
            snapshot = dict(cameras=cameras, task=self.task, controller=self.task, model=dict(status="READY"))
            return next(message for message in results_from_snapshot(snapshot) if message["msg_type"] == "delivery")

        for _ in range(3):
            released = capture()
        self.assertEqual(delivery(released)["object_inside"], 1)
        self.assertEqual(delivery(released)["delivery_confirmed"], 0)

        self.task["phase"] = "delivery_check"
        for _ in range(2):
            self.assertEqual(delivery(capture())["delivery_confirmed"], 0)
        checked = capture()
        confirmed = delivery(checked)
        self.assertEqual((confirmed["delivery_confirmed"], confirmed["danger_flag"]), (1, 0))
        checked["lower"]["age_ms"] = 150
        self.assertEqual(delivery(checked)["delivery_confirmed"], 0)
        occupied = capture(lower_extra=(Detection("core_material", .9, (25., 25., 75., 75.)),))
        self.assertEqual(delivery(occupied)["danger_reason"], "gripper_empty_not_confirmed")
        self.assertEqual(delivery(capture(lower_age=.151))["delivery_confirmed"], 0)

        self.task["mechanism_state"] = "closed"
        for _ in range(3):
            closed = capture()
        self.assertEqual(delivery(closed)["danger_reason"], "gripper_not_open")


class RV3ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.obj = SimpleObject("green", 500, True, False, -36.87, -300, 400,
                                "blue_material_zone", True, True)
        self.observation = SimpleObservation("upper", 1, 10, "ok", (self.obj,))

    def test_all_coordinates_and_flags_roundtrip_and_legacy_layout_is_preserved(self):
        self.assertEqual(decode(encode(self.observation, version=3)), self.observation)
        for version in (1, 2):
            obj = decode(encode(self.observation, version=version)).objects[0]
            self.assertIsNone(obj.x_mm)
            self.assertIsNone(obj.zone_kind)
            self.assertFalse(obj.object_inside)
            self.assertEqual(obj.angle, None if version == 1 else -36.87)

    def test_inconsistent_or_wrong_kind_entry_is_rejected(self):
        for change in (dict(x_mm=None), dict(inside_valid=False), dict(zone_kind=None),
                       dict(zone_kind="red_casualty_zone"), dict(kind="blue"),
                       dict(x_mm=65536), dict(angle=None)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                encode(replace(self.observation, objects=(replace(self.obj, **change),)), version=3)

    def test_twelve_objects_fit_with_coordinates_and_entry_flags(self):
        obj = SimpleObject("orange", 65535, False, False, -179.99, -65535, -65535,
                           "blue_casualty_zone", True, True)
        observation = replace(self.observation, objects=(obj,) * 12)
        packet = encode(observation, version=3)
        self.assertLessEqual(len(packet) - 1, 1024)
        self.assertEqual(decode(packet), observation)

    def test_controller_link_writes_real_rv3_packets(self):
        class State:
            def snapshot(inner):
                row = dict(name="normal_material", position_mm=dict(x_right=-300., y_forward=400., ground_range=500.),
                           position_valid=True, zone_entry=dict(zone_kind="blue_material_zone", result_valid=True,
                                                                object_inside=True))
                return dict(cameras=dict(upper=dict(status="OK", sequence=1, age_ms=10, detections=[row]),
                                         lower=dict(status="OK", sequence=1, age_ms=10, detections=[])))

        link = ControllerLink(State(), "unused", "vision_simple_v3")
        packets = []

        class Port:
            def write(inner, data):
                packets.append(data)
                if len(packets) == 2:
                    link.stop.set()
                return len(data)

        link.send_simple(Port())
        self.assertEqual(len(packets), 2)
        self.assertTrue(packets[0].startswith(b"RV3,"))
        self.assertEqual(decode(packets[0]).objects[0], self.obj)

    def test_lower_waiting_for_upper_serial_write_is_counted_in_age(self):
        row = dict(name="normal_material", position_mm=dict(x_right=-300., y_forward=400., ground_range=500.),
                   position_valid=True, zone_entry=dict(zone_kind="blue_material_zone", result_valid=True,
                                                        object_inside=True))

        class State:
            def snapshot(inner):
                return dict(cameras={role: dict(status="OK", sequence=1, age_ms=10, detections=[row])
                                     for role in ("upper", "lower")})

        link = ControllerLink(State(), "unused", "vision_simple_v3")
        packets = []

        class Port:
            def write(inner, data):
                packets.append(data)
                if len(packets) == 2:
                    link.stop.set()
                return len(data)

        with patch("rescue_vision.dashboard.time.monotonic", side_effect=[10., 10., 10.151]):
            link.send_simple(Port())
        upper, lower = map(decode, packets)
        self.assertTrue(upper.objects[0].object_inside)
        self.assertGreaterEqual(lower.age_ms, 161)
        self.assertFalse(lower.objects[0].distance_valid)
        self.assertFalse(lower.objects[0].inside_valid)
        self.assertFalse(lower.objects[0].object_inside)


if __name__ == "__main__":
    unittest.main()

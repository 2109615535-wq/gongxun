"""可离线运行的任务协议、安全降级及关联回归测试，不依赖模型或串口硬件。"""
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
import time

import numpy as np

from rescue_vision.dashboard import ControllerLink
from rescue_vision.dashboard_state import DashboardState
from rescue_vision.dual import CameraPerception
from rescue_vision.protocol import crc16
from rescue_vision.simple_protocol import SimpleObject, SimpleObservation, decode as simple_decode, encode as simple_encode
from rescue_vision.task_protocol import decode, encode, parse_task, task_key
from rescue_vision.tracking import TargetTracker
from rescue_vision.types import Detection


ROOT = Path(__file__).resolve().parents[1]


def config():
    cfg = json.loads((ROOT / "config/dual_camera.json").read_text(encoding="utf-8"))
    cfg["gripper"] = {"roi_raw_xyxy": [100, 100, 300, 400], "validated": True}
    return cfg


def task(**changes):
    value = dict(msg_type="task", session_id="boot-1", task_id="trip-1", seq=1, state_id=1,
                 phase="verify_before_transfer", team_color="red", allowed_kinds=["green"],
                 locked_target_id=0, expected_kind="green", expected_count=1,
                 mechanism_state="open", yaw_deg=15)
    value.update(changes)
    return value


def row(name="normal_material", box=(130, 150, 180, 200), target_id=2, **changes):
    value = dict(name=name, box=box, target_id=target_id, inside_gripper_roi=True,
                 intersects_gripper_roi=True, occluded=False, position_mm=None)
    value.update(changes)
    return value


class TaskProtocolTests(unittest.TestCase):
    def test_crc_and_roundtrip(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)
        packet = encode(task())
        self.assertEqual(parse_task(decode(packet))["expected"], "normal_material")
        self.assertEqual(decode(packet.replace(b"\n", b"\r\n"))["task_id"], "trip-1")
        with self.assertRaises(ValueError):
            decode(packet.replace(b"trip-1", b"trip-2"))

    def test_duplicate_keys_and_missing_crc_rejected(self):
        body = b'{"msg_type":"task","seq":1,"seq":2}'
        with self.assertRaises(ValueError):
            decode(body + f"*{crc16(body):04X}".encode())
        with self.assertRaises(ValueError):
            decode(json.dumps(task()).encode())

    def test_task_constraints(self):
        for changes in [dict(state_id=1, allowed_kinds=["black"], expected_kind="black"),
                        dict(state_id=3), dict(expected_count=2), dict(phase="invented"),
                        dict(team_color="green"), dict(seq=True), dict(yaw_deg=float("nan")),
                        dict(expected_kind=None), dict(locked_target_id=-1),
                        dict(mechanism_state="invented")]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                parse_task(task(**changes))
        self.assertEqual(parse_task(task(state_id=0, phase="idle", allowed_kinds=[], expected_kind=None))["state_id"], 0)

    def test_legacy_rv2_roundtrip(self):
        value = SimpleObservation("upper", 12, 10, "ok", (SimpleObject("green", 450, True, False, -3.5),))
        self.assertEqual(simple_decode(simple_encode(value)), value)


class TrackingTests(unittest.TestCase):
    def test_id_stable_then_lost_no_reuse(self):
        tracker = TargetTracker("upper", .2)
        first = [row()]
        tracker.update(first)
        next_rows = [row(box=(132, 150, 182, 200))]
        tracker.update(next_rows)
        self.assertEqual(first[0]["target_id"], next_rows[0]["target_id"])
        tracker.update([])
        tracker.update(first := [row()])
        self.assertNotEqual(first[0]["target_id"], next_rows[0]["target_id"])
        previous = first[0]["target_id"]
        tracker.reset()
        tracker.update(first := [row()])
        self.assertGreater(first[0]["target_id"], previous)

    def test_ambiguous_match_gets_new_ids(self):
        tracker = TargetTracker("lower", .2)
        old = [row()]
        tracker.update(old)
        new = [row(), row(box=(132, 150, 182, 200))]
        tracker.update(new)
        self.assertEqual(len({r["target_id"] for r in new}), 2)
        self.assertNotIn(old[0]["target_id"], [r["target_id"] for r in new])

    def test_unknown_lock_never_assigned(self):
        tracker = TargetTracker("upper", .2)
        new = [row()]
        tracker.update(new, locked_target_id=1)
        self.assertNotEqual(new[0]["target_id"], 1)


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.state = DashboardState(config(), clock=lambda: self.now)
        self.state.model = {"status": "READY"}
        self.link = ControllerLink(self.state, "unused", "rescue_json_v2")
        self.link.receive(encode(task()))

    def publish(self, *, captured=None, key=None, valid=True):
        current = self.state.task()
        for role in ("upper", "lower"):
            self.state.publish_result(role, dict(status="OK", sequence=42, mode="camera",
                captured_monotonic=self.now if captured is None else captured,
                task_key=list(task_key(current)) if key is None else key,
                calibration_quality="validated", detections=[row()] if role == "lower" else [],
                selected=None, motion_valid=False, hazard_blocked=False, gripper_revision=0,
                gripper=dict(status="MATCH", match=True, count=3, roi_count=1,
                             roi_kinds=["normal_material"], result_valid=valid,
                             danger_flag=int(not valid), danger_reason="none" if valid else "occluded")))

    def result(self, kind):
        return next(r for r in self.state.snapshot()["task_results"] if r["msg_type"] == kind)

    def assert_blocked(self):
        result = self.result("verification")
        self.assertEqual((result["result_valid"], result["danger_flag"]), (0, 1))
        delivery = self.result("delivery")
        self.assertEqual([delivery[k] for k in ("result_valid", "entry_allowed", "object_inside", "delivery_confirmed")], [0]*4)

    def test_valid_verification_and_task_binding(self):
        self.publish()
        value = self.result("verification")
        self.assertEqual((value["result_valid"], value["danger_flag"], value["roi_count"]), (1, 0, 1))
        self.assertEqual(value["roi_kinds"], ["green"])
        for message in self.state.snapshot()["task_results"]:
            self.assertEqual((message["task_id"], message["phase"]), ("trip-1", "verify_before_transfer"))
            self.assertEqual(decode(encode(dict(message, seq=2)))["seq"], 2)

    def test_heartbeat_preserves_confirmation_context(self):
        self.publish()
        old_key = task_key(self.state.task())
        self.link.receive(encode(task(seq=2, yaw_deg=16)))
        self.assertEqual(task_key(self.state.task()), old_key)
        self.assertEqual(self.result("status")["ack_seq"], 2)
        self.assertEqual(self.result("verification")["danger_flag"], 0)

    def test_phase_change_rejects_old_and_inflight_frames(self):
        self.publish()
        old_key = list(task_key(self.state.task()))
        self.now += .01
        self.link.receive(encode(task(seq=2, phase="verify_after_transfer")))
        self.assert_blocked()
        self.publish(captured=self.now - .005, key=list(task_key(self.state.task())))
        self.assert_blocked()
        self.publish(key=old_key)
        self.assert_blocked()
        self.publish()
        self.assertEqual(self.result("verification")["danger_flag"], 0)

    def test_crc_error_and_link_timeout_block(self):
        self.publish()
        self.link.receive(encode(task(seq=2)).replace(b"trip-1", b"trip-2"))
        self.assert_blocked()
        self.assertEqual(self.state.link["invalid_lines"], 1)
        self.link.receive(encode(task(seq=3)))
        self.publish()
        self.now += 1.01
        self.assert_blocked()

    def test_duplicate_and_stale_sequences(self):
        self.link.receive(encode(task()))
        self.assertEqual(self.state.controller["status"], "INVALID")
        self.link.receive(encode(task(seq=2)))
        self.assertEqual(self.state.controller["status"], "CONNECTED")
        self.link.receive(encode(task(seq=1)))
        self.assertEqual(self.state.controller["seq"], 2)

    def test_sequence_wrap_and_new_session_guard(self):
        self.state.controller["seq"] = 0xFFFFFFFF
        self.link.receive(encode(task(seq=0)))
        self.assertEqual(self.state.controller["seq"], 0)
        self.link.receive(encode(task(session_id="boot-2")))
        self.assertEqual(self.state.controller["session_id"], "boot-1")
        self.now += 1.01
        self.link.receive(encode(task(session_id="boot-2")))
        self.assertEqual(self.state.controller["session_id"], "boot-2")

    def test_stale_replay_roi_revision_and_calibration_fail_closed(self):
        for change in ("stale", "replay", "roi", "calibration", "model", "mechanism"):
            with self.subTest(change=change):
                self.publish()
                self.state.model = {"status": "READY"}
                if change == "stale":
                    self.state.cameras["lower"]["captured_monotonic"] -= .3
                elif change == "replay":
                    self.state.cameras["lower"]["mode"] = "video"
                elif change == "roi":
                    self.state.cameras["lower"]["gripper_revision"] = -1
                elif change == "calibration":
                    self.state.cameras["upper"]["calibration_quality"] = "local_checked"
                elif change == "model":
                    self.state.model["status"] = "ERROR"
                else:
                    self.link.receive(encode(task(seq=2, mechanism_state="unknown")))
                    self.publish()
                self.assert_blocked()

    def test_docking_delivery_never_infer_success_from_target(self):
        self.link.receive(encode(task(seq=2, phase="near_dock", team_color="blue")))
        self.publish()
        dock = self.result("dock")
        self.assertEqual(dock["zone_kind"], "blue_material_zone")
        self.assertEqual(dock["dock_valid"], 0)
        self.assertIsNone(dock["dock_yaw_deg"])
        self.assert_blocked()

    def test_stream_fragmentation_multiline_and_outgoing_packets(self):
        packets = encode(task(seq=2)) + encode(task(seq=3))
        link = self.link

        class Port:
            chunks = [packets[:30], packets[30:]]
            sent = []

            def read(self, _):
                time.sleep(.001)
                return self.chunks.pop(0) if self.chunks else b""

            def write(self, data):
                self.sent.append(decode(data))
                if len(self.sent) == 4:
                    link.stop.set()
                return len(data)

        port = Port()
        self.link.exchange_legacy(port)
        self.assertEqual(self.state.controller["seq"], 3)
        self.assertEqual(self.state.link["invalid_lines"], 0)
        self.assertEqual([m["seq"] for m in port.sent], [0, 1, 2, 3])
        self.assertEqual([m["msg_type"] for m in port.sent], ["status", "observation", "observation", "verification"])

    def test_old_json_link_rejects_new_task_without_crc(self):
        link = ControllerLink(self.state, "unused", "rescue_json_v1")
        link.receive(json.dumps(task(seq=2)).encode())
        link.receive(b"[]")
        self.assertEqual(self.state.link["invalid_lines"], 2)
        self.assertEqual(self.state.controller["seq"], 1)


class RoiTests(unittest.TestCase):
    def setUp(self):
        self.cfg = config()
        calibration = SimpleNamespace(data={}, size=(640, 480), maximum=1000)
        self.session = CameraPerception("lower", calibration, "validated", self.cfg)
        self.image = np.zeros((480, 640, 3), np.uint8)

    def check(self, rows, **kwargs):
        return self.session.check_gripper(rows, "normal_material", ("test",), self.image, **kwargs)

    def test_requires_three_frames_and_reports_actual_count(self):
        for _ in range(2):
            self.assertFalse(self.check([row()])["result_valid"])
        result = self.check([row()])
        self.assertEqual((result["roi_count"], result["count"], result["danger_flag"]), (1, 3, 0))

    def test_multiple_wrong_hazard_unknown_occlusion(self):
        samples = [([row(), row(box=(200, 220, 240, 260))], "multiple_objects"),
                   ([row(name="hazard")], "hazard"),
                   ([row(name="unclassified")], "unknown_object"),
                   ([row(occluded=True)], "occluded"),
                   ([row(inside_gripper_roi=False)], "occluded")]
        for rows, reason in samples:
            with self.subTest(reason=reason):
                result = self.check(rows)
                self.assertEqual(result["danger_flag"], 1)
                self.assertEqual(result["danger_reason"], reason)
        for _ in range(3):
            result = self.check([row(name="core_material")])
        self.assertEqual(result["danger_reason"], "kind_mismatch")

    def test_roi_unverified_empty_and_lock_mismatch(self):
        for _ in range(3):
            result = self.check([])
        self.assertEqual((result["roi_count"], result["danger_flag"]), (0, 1))
        self.assertEqual(self.check([row()], locked_target_id=99)["danger_reason"], "locked_target_lost")
        self.cfg["gripper"]["validated"] = False
        for _ in range(3):
            result = self.check([row()])
        self.assertFalse(result["result_valid"])
        self.assertEqual(result["danger_reason"], "roi_unverified")

    def test_partial_roi_objects_are_counted_by_real_pipeline(self):
        current = dict(parse_task(task()), source="controller")
        result = self.session.process(self.image, [Detection("normal_material", .9, (80, 150, 160, 200))],
                                      10., 10.01, current)
        self.assertEqual(result["gripper"]["roi_count"], 1)
        self.assertEqual(result["gripper"]["danger_reason"], "occluded")

    def test_pre_phase_frames_never_accumulate_confirmation(self):
        current = dict(parse_task(task()), source="controller", activated_monotonic=10.)
        detection = [Detection("normal_material", .9, (130, 150, 180, 200))]
        for stamp in (9.98, 9.99):
            result = self.session.process(self.image, detection, stamp, 10.01, current)
            self.assertEqual(result["status"], "WAITING_TASK_FRAME")
        result = self.session.process(self.image, detection, 10.02, 10.03, current)
        self.assertEqual(result["gripper"]["count"], 1)
        self.assertFalse(result["gripper"]["result_valid"])


if __name__ == "__main__":
    unittest.main()

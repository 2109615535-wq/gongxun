"""双向任务协议：JSON正文 + 现有CRC16，所有结果按任务和动作阶段隔离。"""
import json
import math
import re

from .protocol import crc16
from .simple_protocol import CLASS_NAMES, observations_from_snapshot


MAX_LINE = 4096
MODEL_NAMES = {wire: model for model, wire in CLASS_NAMES.items()}
PHASES = {"idle", "scan", "search", "align", "approach", "verify_before_transfer",
          "verify_after_transfer", "near_dock", "deliver", "release", "retreat", "delivery_check"}
MECHANISM_STATES = {"unknown", "open", "closed", "opening", "closing", "at_position", "fault"}
STATE_KINDS = {0: set(), 1: {"green"}, 2: {"green", "black"}, 3: {"orange"}}
VERIFY_PHASES = {"verify_before_transfer", "verify_after_transfer"}
DOCK_PHASES = {"near_dock"}
DELIVERY_PHASES = {"deliver", "release", "retreat", "delivery_check"}


def integer(value, low=0, high=0xFFFFFFFF):
    return type(value) is int and low <= value <= high


def encode(message):
    """CRC覆盖原始JSON字节；接收端无需重排或重新序列化JSON。"""
    body = json.dumps({k: v for k, v in message.items() if k != "crc16"},
                      ensure_ascii=True, separators=(",", ":"), allow_nan=False).encode("ascii")
    packet = body + f"*{crc16(body):04X}".encode("ascii")
    if len(packet) > MAX_LINE:
        raise ValueError("Task protocol line too long")
    return packet + b"\n"


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key")
        result[key] = value
    return result


def _reject_constant(_):
    raise ValueError("Nonfinite JSON")


def decode(packet):
    packet = packet.removesuffix(b"\n").removesuffix(b"\r")
    if len(packet) > MAX_LINE or b"*" not in packet:
        raise ValueError("Invalid task protocol line")
    body, checksum = packet.rsplit(b"*", 1)
    if not re.fullmatch(b"[0-9a-fA-F]{4}", checksum) or crc16(body) != int(checksum, 16):
        raise ValueError("CRC mismatch")
    try:
        value = json.loads(body.decode("ascii"), object_pairs_hook=_unique_object,
                           parse_constant=_reject_constant)
    except (UnicodeError, RecursionError) as exc:
        raise ValueError("Invalid task JSON") from exc
    if not isinstance(value, dict) or "crc16" in value:
        raise ValueError("Expected message object without embedded CRC")
    value["crc16"] = checksum.decode("ascii").upper()
    return value


def parse_task(value):
    if not isinstance(value, dict) or value.get("msg_type") != "task":
        raise ValueError("Expected task message")
    for key in ("session_id", "task_id"):
        if not isinstance(value.get(key), str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", value[key]):
            raise ValueError(f"Invalid {key}")
    if not integer(value.get("seq")) or not integer(value.get("state_id"), 0, 3):
        raise ValueError("Invalid sequence/state")
    if (not isinstance(value.get("phase"), str) or value["phase"] not in PHASES or
            value.get("team_color") not in ("red", "blue") or
            not isinstance(value.get("mechanism_state"), str) or value["mechanism_state"] not in MECHANISM_STATES):
        raise ValueError("Invalid phase/team/mechanism")
    kinds = value.get("allowed_kinds")
    if (not isinstance(kinds, list) or any(not isinstance(k, str) for k in kinds) or
            len(kinds) != len(set(kinds)) or not set(kinds) <= STATE_KINDS[value["state_id"]]):
        raise ValueError("Kinds conflict with state restrictions")
    expected = value.get("expected_kind")
    if "expected_kind" not in value or (expected is not None and
                                        (not isinstance(expected, str) or expected not in kinds)):
        raise ValueError("Expected kind must be allowed")
    if value["phase"] in VERIFY_PHASES | DOCK_PHASES | DELIVERY_PHASES and expected is None:
        raise ValueError("This phase requires expected_kind")
    if not integer(value.get("expected_count"), 1, 1) or not integer(value.get("locked_target_id")):
        raise ValueError("Expected one object and a valid target ID (0 means unlocked)")
    yaw = value.get("yaw_deg")
    if yaw is not None and (type(yaw) not in (int, float) or not math.isfinite(yaw) or not -180 <= yaw <= 180):
        raise ValueError("Invalid yaw_deg")
    fields = ("session_id", "task_id", "seq", "state_id", "phase", "team_color", "allowed_kinds",
              "locked_target_id", "expected_kind", "expected_count", "mechanism_state", "yaw_deg")
    if any(key not in value for key in fields):
        raise ValueError("Incomplete task")
    task = {key: value[key] for key in fields}
    # 内部继续复用已有模型类别名和网页state，线上统一使用green/black/orange。
    return dict(task, expected=MODEL_NAMES.get(expected), state=f"{task['state_id']} / {task['phase']}",
                contract="rescue_json_v2")


def task_key(task):
    """心跳seq/yaw变化不重置判断；任务、阶段、选择条件或机构改变则必须重取图像。"""
    return (task.get("session_id"), task.get("task_id"), task.get("expected"), task.get("source"),
            task.get("state"), task.get("phase"), task.get("team_color"),
            tuple(sorted(task.get("allowed_kinds", ()))), task.get("locked_target_id"),
            task.get("expected_count"), task.get("mechanism_state"), task.get("activation_id"))


def results_from_snapshot(snapshot):
    """只读构造结果。尚未实现可靠库口/入仓几何时明确输出无效，绝不猜测成功。"""
    task = snapshot["task"]
    controller = snapshot["controller"]
    cameras = snapshot["cameras"]
    active = task.get("source") == "controller" and task.get("contract") == "rescue_json_v2"
    common = {"session_id": controller.get("session_id"), "task_id": controller.get("task_id"),
              "state_id": controller.get("state_id"), "phase": controller.get("phase")}
    fresh = active and all(c.get("task_current", False) and c.get("status") == "OK" and
                           c.get("mode") != "video" and c.get("age_ms") is not None and
                           0 <= c["age_ms"] < 300 for c in cameras.values())
    fault = "none"
    if not active:
        fault = "controller_lost"
    elif any(c.get("status") == "CALIBRATION_ERROR" for c in cameras.values()):
        fault = "calibration_fault"
    elif snapshot["model"].get("status") != "READY":
        fault = "model_fault"
    elif any(c.get("status") in ("INFERENCE_FAULT", "INVALID_DETECTIONS", "RESOLUTION_MISMATCH") for c in cameras.values()):
        fault = "processing_fault"
    elif any(c.get("status") not in ("OK", "STALE", "WAITING_TASK_FRAME") for c in cameras.values()):
        fault = "camera_fault"
    elif any(c.get("mode") == "video" for c in cameras.values()):
        fault = "replay_input"
    elif not all(c.get("task_current", False) for c in cameras.values()):
        fault = "task_frame_pending"
    elif not fresh:
        fault = "result_stale"
    elif any(c.get("calibration_quality") != "validated" for c in cameras.values()):
        fault = "calibration_unverified"
    ready = fault == "none"
    mechanism_unknown = task.get("mechanism_state") in (None, "unknown", "fault", "opening", "closing")
    blocked = not ready or mechanism_unknown or any(c.get("hazard_blocked") for c in cameras.values())
    messages = [dict(common, msg_type="status", ack_seq=controller.get("seq"),
                     accepted_state_id=controller.get("state_id"), accepted_phase=controller.get("phase"),
                     vision_ready=int(ready), fault_code=fault, danger_flag=int(blocked))]
    for obs in observations_from_snapshot(snapshot):
        camera = cameras[obs.source]
        usable = active and camera.get("task_current", False)
        status = obs.status if usable else "fault"
        rows = [r for r in camera.get("detections", []) if r.get("name") in CLASS_NAMES]
        objects = [dict(target_id=row.get("target_id", 0), kind=obj.kind,
                        distance_mm=obj.distance_mm, distance_valid=int(obj.distance_valid and not blocked),
                        angle="NA" if obj.angle is None else obj.angle, in_roi=int(obj.in_roi))
                   for row, obj in zip(rows, obs.objects)] if usable else []
        messages.append(dict(common, msg_type="observation", source=obs.source, frame_id=obs.frame_id,
                             age_ms=obs.age_ms, status=status, objects=objects,
                             yaw_deg=camera.get("observation_yaw_deg"),
                             danger_flag=int(blocked or status not in ("ok", "empty"))))
    lower = cameras["lower"]
    grip = lower.get("gripper", {})
    valid = bool(ready and not any(c.get("hazard_blocked") for c in cameras.values()) and
                 task.get("phase") in VERIFY_PHASES and grip.get("result_valid") and
                 task.get("mechanism_state") not in ("unknown", "fault", "opening", "closing"))
    reason = (fault if not ready else "wrong_phase" if task.get("phase") not in VERIFY_PHASES else
              "mechanism_unknown" if task.get("mechanism_state") in ("unknown", "fault", "opening", "closing") else
              grip.get("danger_reason", "verification_unavailable"))
    if ready and any(c.get("hazard_blocked") for c in cameras.values()) and reason == "none":
        reason = "hazard_or_unknown_object"
    danger = int(not valid or grip.get("danger_flag", 1))
    messages.append(dict(common, msg_type="verification", source="lower",
                         frame_id=lower.get("sequence", 0), age_ms=lower.get("age_ms"),
                         result_valid=int(valid), roi_count=grip.get("roi_count", 0),
                         roi_kinds=[CLASS_NAMES.get(k, "unknown") for k in grip.get("roi_kinds", [])],
                         danger_flag=danger, danger_reason=reason))
    kind = "casualty" if task.get("expected_kind") == "orange" else "material"
    zone = f"{task['team_color']}_{kind}_zone" if active else None
    messages.append(dict(common, msg_type="dock", zone_kind=zone, dock_valid=0,
                         dock_lateral_mm=None, dock_gap_mm=None, dock_yaw_deg=None,
                         danger_flag=1, danger_reason="dock_geometry_unavailable"))
    messages.append(dict(common, msg_type="delivery", zone_kind=zone, result_valid=0,
                         entry_allowed=0, object_inside=0, delivery_confirmed=0,
                         danger_flag=1, danger_reason="delivery_geometry_unavailable"))
    phase = task.get("phase")
    relevant = messages[3] if phase in VERIFY_PHASES else messages[4] if phase in DOCK_PHASES else messages[5]
    messages[0]["danger_flag"] = (int(blocked) if phase in {"scan", "search", "align", "approach"}
                                  else relevant["danger_flag"])
    return messages

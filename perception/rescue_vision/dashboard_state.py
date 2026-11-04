"""Thread-safe state with read-time expiry, shared by HTTP and the future MCU adapter."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time

from .dual import ROLES, SELECTABLE


def parse_controller_task(value):
    """New, explicitly versioned integration contract, not existing UART5 telemetry."""
    if not isinstance(value, dict) or value.get("type") != "rescue_task_v1":
        raise ValueError("Expected rescue_task_v1")
    for key in ("session_id", "task_id", "state"):
        if not isinstance(value.get(key), str) or not 1 <= len(value[key]) <= 64:
            raise ValueError(f"Invalid {key}")
    if type(value.get("seq")) is not int or not 0 <= value["seq"] <= 0xFFFFFFFF:
        raise ValueError("Invalid seq")
    if value.get("expected") not in SELECTABLE | {None}:
        raise ValueError("Invalid expected class")
    return {key: value[key] for key in ("session_id", "task_id", "state", "seq", "expected")}


class DashboardState:
    def __init__(self, config, clock=time.monotonic, config_path=None):
        self.config, self.clock = config, clock
        self.config_path = Path(config_path) if config_path is not None else None
        self.lock = threading.RLock()
        self.frames = {}
        self.raw_images = {}
        self.cameras = {role: {"role": role, "status": "DISABLED", "detections": [],
                               "selected": None, "motion_valid": False,
                               "gripper": {"status": "UNCONFIGURED", "match": False, "count": 0}}
                        for role in ROLES}
        self.camera_info = {}
        self.controller = {"status": "DISCONNECTED", "state": None, "expected": None}
        self.debug_expected = None
        self.debug_state = "IDLE"
        self.debug_revision = 0
        self.gripper_revision = 0
        self.model = {"status": "NOT_LOADED"}
        self.link = {"status": "DISCONNECTED", "protocol": None, "invalid_lines": 0}

    def set_debug_target(self, expected):
        self.set_debug_task(expected)

    def set_debug_task(self, expected, state=None):
        if expected not in SELECTABLE | {None}:
            raise ValueError("Invalid expected target")
        if state is not None and (not isinstance(state, str) or not 1 <= len(state.strip()) <= 64):
            raise ValueError("任务状态须为1到64个字符")
        with self.lock:
            if self.controller.get("session_id"):
                raise PermissionError("当前会话已接入主控，请通过主控设置任务")
            self.debug_expected = expected
            if state is not None:
                self.debug_state = state.strip()
            self.debug_revision += 1

    def set_gripper_roi(self, roi):
        with self.lock:
            size = self.camera_info.get("lower", {}).get("size")
            if roi is not None:
                if (not isinstance(roi, list) or len(roi) != 4 or
                        not all(type(value) is int for value in roi) or not size or
                        not 0 <= roi[0] < roi[2] <= size[0] or
                        not 0 <= roi[1] < roi[3] <= size[1]):
                    raise ValueError("ROI须位于下视原始图像内，请先启动下视相机")
            gripper = dict(self.config["gripper"], roi_raw_xyxy=deepcopy(roi), validated=False)
            if self.config_path is not None:
                data = json.loads(self.config_path.read_text(encoding="utf-8-sig"))
                data["gripper"] = gripper
                temporary = None
                try:
                    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.config_path.parent,
                                                     prefix=".gripper-", suffix=".json", delete=False) as output:
                        temporary = Path(output.name)
                        json.dump(data, output, ensure_ascii=False, indent=2, allow_nan=False)
                        output.write("\n")
                        output.flush()
                        os.fsync(output.fileno())
                    os.replace(temporary, self.config_path)
                finally:
                    if temporary is not None:
                        temporary.unlink(missing_ok=True)
            self.config["gripper"] = gripper
            self.gripper_revision += 1
            self.frames.pop("lower_annotated", None)
            return deepcopy(gripper)

    def update_controller(self, value):
        task = parse_controller_task(value)
        now = self.clock()
        with self.lock:
            old = self.controller
            fresh = now - old.get("received_monotonic", -math.inf) <= self.config["controller_timeout_ms"] / 1000
            if task["session_id"] == old.get("session_id"):
                if task["seq"] <= old["seq"]:
                    raise ValueError("Duplicate or out-of-order controller sequence")
            elif fresh:
                raise ValueError("A different controller session is still active")
            self.controller = dict(task, source="controller", status="CONNECTED", received_monotonic=now)

    def task(self):
        with self.lock:
            current = self.controller
            if current.get("status") == "CONNECTED" and self.clock() - current.get("received_monotonic", -math.inf) <= self.config["controller_timeout_ms"] / 1000:
                return deepcopy(current)
            # Once a controller has been seen, a link loss must not resume a debug task.
            if current.get("session_id"):
                return {"source": "none", "expected": None, "session_id": None, "task_id": None}
            return {"source": "debug", "expected": self.debug_expected,
                    "session_id": "desktop", "task_id": str(self.debug_revision), "state": self.debug_state}

    def camera_fault(self, role, status, error=None):
        with self.lock:
            self.frames.pop(role, None)
            self.frames.pop(role + "_annotated", None)
            self.raw_images.pop(role, None)
            self.cameras[role] = {"role": role, "status": status, "error": error,
                                  "detections": [], "selected": None, "motion_valid": False,
                                  "gripper": {"status": status, "match": False, "count": 0}}

    def publish_frame(self, role, jpeg, captured, sequence):
        with self.lock:
            self.frames[role] = (jpeg, captured, sequence)

    def publish_raw(self, role, image, captured, sequence):
        with self.lock:
            self.raw_images[role] = (image, captured, sequence)

    def raw_image(self, role):
        with self.lock:
            frame = self.raw_images.get(role)
            if frame and 0 <= self.clock() - frame[1] <= self.config["max_frame_age_ms"] / 1000:
                return frame
            return None

    def publish_result(self, role, result):
        with self.lock:
            self.cameras[role] = deepcopy(result)

    def frame(self, role):
        with self.lock:
            frame = self.frames.get(role)
            if frame and 0 <= self.clock() - frame[1] <= self.config["max_frame_age_ms"] / 1000:
                return frame
            return None

    def snapshot(self):
        with self.lock:
            now = self.clock()
            task = self.task()
            key = [task.get(k) for k in ("session_id", "task_id", "expected", "source", "state")]
            cameras = deepcopy(self.cameras)
            for role, camera in cameras.items():
                captured = camera.get("captured_monotonic")
                age = None if captured is None else (now - captured) * 1000
                camera["age_ms"] = age
                camera["stream_available"] = (self.frame(role + "_annotated") or self.frame(role)) is not None
                camera["info"] = deepcopy(self.camera_info.get(role, {}))
                if age is None or not 0 <= age <= self.config["control_max_age_ms"]:
                    camera["motion_valid"] = False
                if age is not None and not 0 <= age <= self.config["max_frame_age_ms"]:
                    camera.update(status="STALE", detections=[], selected=None, motion_valid=False,
                                  hazard_blocked=False, gripper={"status": "STALE", "match": False, "count": 0})
                if camera.get("task_key") != key:
                    camera.update(selected=None, motion_valid=False,
                                  gripper={"status": "WAITING_TASK_FRAME", "match": False, "count": 0})
                if role == "lower" and camera.get("gripper_revision", 0) != self.gripper_revision:
                    camera.update(selected=None, motion_valid=False,
                                  gripper={"status": "WAITING_TASK_FRAME", "match": False, "count": 0})
            controller = deepcopy(self.controller)
            controller["age_ms"] = (now - controller["received_monotonic"]) * 1000 if "received_monotonic" in controller else None
            if controller["age_ms"] is not None and controller["age_ms"] > self.config["controller_timeout_ms"]:
                controller.update(status="STALE", state=None, expected=None)
            upper, lower = cameras["upper"], cameras["lower"]
            selected = upper.get("selected")
            source = "upper" if selected and selected.get("position_mm") else None
            if source is None:
                selected = lower.get("selected")
                source = "lower" if selected and selected.get("position_mm") else None
            if source is None:
                selected = None
            blocked = any(camera.get("hazard_blocked", False) for camera in cameras.values())
            cameras_ready = all(camera["status"] == "OK" for camera in cameras.values())
            live_sources = all(camera.get("mode") != "video" for camera in cameras.values())
            control_fresh = all(camera.get("age_ms") is not None and
                                0 <= camera["age_ms"] <= self.config["control_max_age_ms"]
                                for camera in cameras.values()) and live_sources
            motion = bool(source == "upper" and upper["motion_valid"] and cameras_ready
                          and control_fresh and not blocked and controller["status"] == "CONNECTED")
            grip = deepcopy(lower["gripper"])
            grip_fresh = (lower.get("age_ms") is not None and
                          0 <= lower["age_ms"] <= self.config["max_frame_age_ms"] and live_sources)
            grip["controller_match"] = bool(grip["match"] and controller["status"] == "CONNECTED"
                                             and cameras_ready and grip_fresh and not blocked)
            return {"schema": "dual_camera_v1", "server_monotonic": now,
                    "coordinate_system": "x_right_y_forward", "unit": "mm",
                    "cameras": cameras, "controller": controller, "task": task,
                    "link": deepcopy(self.link), "model": deepcopy(self.model),
                    "gripper_config": deepcopy(self.config["gripper"]),
                    "gripper_revision": self.gripper_revision,
                    "navigation": {"source": source, "selected": selected, "motion_valid": motion,
                                   "hazard_blocked": blocked, "upper_primary": True},
                    "gripper": grip}

    def controller_observation(self):
        """Compact JSON payload suitable for 115200 baud at 10Hz."""
        state = self.snapshot()
        selected = state["navigation"]["selected"] or {}
        visual = state["cameras"]["upper"].get("selected") or state["cameras"]["lower"].get("selected")
        return {"type": "rescue_observation_v1", "task_id": state["task"].get("task_id"),
                "session_id": state["task"].get("session_id"),
                "tick_ms": int(state["server_monotonic"] * 1000) & 0xFFFFFFFF,
                "source": state["navigation"]["source"], "expected": state["task"].get("expected"),
                "motion_valid": state["navigation"]["motion_valid"],
                "hazard_blocked": state["navigation"]["hazard_blocked"],
                "position_mm": selected.get("position_mm"),
                "position_quality": selected.get("position_quality"),
                "visual_target": None if visual is None else {
                    key: visual[key] for key in ("name", "confidence", "box", "confirmed", "center_offset_normalized")},
                "gripper_status": state["gripper"]["status"],
                "gripper_match": state["gripper"]["controller_match"],
                "upper_status": state["cameras"]["upper"]["status"],
                "lower_status": state["cameras"]["lower"]["status"]}

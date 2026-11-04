"""Dual-camera observations. Geometry and detections stay in original raw pixels."""
import json
import math
from pathlib import Path
import re

import cv2
import numpy as np

from .bench_green import GreenSession, LOWER, UPPER, load_lower_fit, load_vehicle
from .board_detector import validate_uvc_controls
from .detector import box_iou
from .geometry import GroundCalibration, green_contact, touches_border
from .types import TARGET_TYPES


MATERIALS = {"normal_material", "core_material", "casualty"}
ZONES = {"red_material_zone", "red_casualty_zone", "blue_material_zone", "blue_casualty_zone"}
SELECTABLE = MATERIALS | ZONES
KNOWN = set(TARGET_TYPES) | ZONES | {"gripper", "gripper_closed"}
ROLES = ("upper", "lower")


def load_dual_config(path):
    path = Path(path).resolve()
    cfg = json.loads(path.read_text(encoding="utf-8-sig"))
    cfg.setdefault("detector_backend", "desktop_pt")
    cfg.setdefault("cpu_threads", 2)
    cfg.setdefault("nms_iou", .45)
    cfg.setdefault("rknn_fast_path", False)
    cfg.setdefault("rknn_core_mask", 0)
    cfg.setdefault("preview_fps", 20)
    cfg.setdefault("jpeg_quality", 70)
    cfg.setdefault("inference_workers", 1)
    cfg.setdefault("capture_buffers", 2)
    cfg.setdefault("cpu_affinity", [])
    if (type(cfg["rknn_fast_path"]) is not bool or type(cfg["rknn_core_mask"]) is not int or
            cfg["rknn_core_mask"] not in (0, 1, 2, 3, 4, 7) or
            type(cfg["preview_fps"]) is not int or not 1 <= cfg["preview_fps"] <= 30 or
            type(cfg["jpeg_quality"]) is not int or not 30 <= cfg["jpeg_quality"] <= 95):
        raise ValueError("Invalid RKNN or preview options")
    if (type(cfg["inference_workers"]) is not int or cfg["inference_workers"] not in (1, 2) or
            (cfg["inference_workers"] == 2 and (not cfg["rknn_fast_path"] or
                                                 cfg["detector_backend"] != "existing_ultralytics_rknn"))):
        raise ValueError("Two inference workers require the independent RKNN fast path")
    if type(cfg["capture_buffers"]) is not int or not 1 <= cfg["capture_buffers"] <= 4:
        raise ValueError("capture_buffers must be between 1 and 4")
    affinity = cfg["cpu_affinity"]
    if (not isinstance(affinity, list) or any(type(core) is not int or core < 0 for core in affinity) or
            len(affinity) != len(set(affinity))):
        raise ValueError("cpu_affinity must contain unique nonnegative CPU indices")
    if cfg["detector_backend"] not in ("desktop_pt", "existing_ultralytics_rknn"):
        raise ValueError("Unsupported dual-camera detector backend")
    if type(cfg["cpu_threads"]) is not int or not 1 <= cfg["cpu_threads"] <= 4:
        raise ValueError("cpu_threads must be between 1 and 4")
    if not 0 < cfg["nms_iou"] < 1:
        raise ValueError("Invalid NMS IoU")
    for key in ("model",):
        cfg[key] = str((path.parent / cfg[key]).resolve())
    if cfg["detector_backend"] == "existing_ultralytics_rknn":
        cfg["existing_project"] = str((path.parent / cfg["existing_project"]).resolve())
        for key in ("expected_model_sha256", "expected_runtime_helper_sha256"):
            if not re.fullmatch(r"[0-9a-f]{64}", cfg.get(key, "")):
                raise ValueError(f"Invalid {key}")
    for role in ROLES:
        camera = cfg["cameras"][role]
        if ("rknn_core_mask" in camera and (type(camera["rknn_core_mask"]) is not int or
                                             camera["rknn_core_mask"] not in (0, 1, 2, 3, 4, 7))):
            raise ValueError("Invalid camera RKNN core mask")
        if camera["calibration_mode"] not in ("vehicle_check", "lower_fit", "validated"):
            raise ValueError("Unknown calibration_mode")
        for key in ("mapping", "check_report"):
            if camera.get(key):
                camera[key] = str((path.parent / camera[key]).resolve())
        if camera["capture_api"] not in ("auto", "dshow", "msmf", "v4l2"):
            raise ValueError("Invalid capture API")
        controls = camera.get("controls", {})
        validate_uvc_controls(controls)
        if controls and camera["capture_api"] != "v4l2":
            raise ValueError("UVC controls require V4L2 capture")
        source = camera["source"]
        if source is not None and (isinstance(source, bool) or not isinstance(source, (str, int)) or source == ""):
            raise ValueError("Invalid camera source")
        if isinstance(source, int) and source < 0:
            raise ValueError("Invalid camera index")
        if camera.get("kind", "camera") == "video":
            camera["source"] = str((path.parent / source).resolve())
        elif camera.get("kind", "camera") != "camera":
            raise ValueError("kind must be camera or video")
    if cfg["cameras"]["upper"]["source"] is not None and cfg["cameras"]["upper"]["source"] == cfg["cameras"]["lower"]["source"]:
        raise ValueError("上下视必须使用不同视频源")
    for key in ("confidence", "association_iou", "grip_coverage"):
        if not isinstance(cfg[key], (float, int)) or isinstance(cfg[key], bool) or not 0 < cfg[key] <= 1:
            raise ValueError(f"Invalid {key}")
    for key in ("confirm_frames", "max_frame_age_ms", "control_max_age_ms", "controller_timeout_ms", "fps"):
        if type(cfg[key]) is not int or cfg[key] <= 0:
            raise ValueError(f"Invalid {key}")
    if cfg["confirm_frames"] < 3 or cfg["max_frame_age_ms"] > 1000:
        raise ValueError("Require >=3 frames and <=1000ms maximum observation age")
    if not cfg["control_max_age_ms"] <= min(150, cfg["max_frame_age_ms"]):
        raise ValueError("Control observations must expire within 150ms")
    grip = cfg["gripper"]
    roi = grip["roi_raw_xyxy"]
    if type(grip["validated"]) is not bool:
        raise ValueError("gripper.validated must be boolean")
    if roi is not None:
        if (len(roi) != 4 or not all(type(x) in (int, float) and math.isfinite(x) for x in roi)
                or not 0 <= roi[0] < roi[2] or not 0 <= roi[1] < roi[3]):
            raise ValueError("Invalid gripper raw-pixel ROI")
    elif grip["validated"]:
        raise ValueError("Validated gripper requires an ROI")
    guard = grip.get("dark_guard", {})
    if (not isinstance(guard, dict) or type(guard.get("enabled", False)) is not bool or
            type(guard.get("value_max", 55)) is not int or not 0 <= guard.get("value_max", 55) <= 255 or
            type(guard.get("min_area_px", 500)) is not int or guard.get("min_area_px", 500) < 1):
        raise ValueError("Invalid gripper dark-region guard")
    return cfg


def load_camera_calibration(camera):
    mode = camera["calibration_mode"]
    if mode == "vehicle_check":
        calibration, provenance = load_vehicle(camera["check_report"], camera["mapping"])
        quality = "local_checked"
    elif mode == "lower_fit":
        calibration, provenance = load_lower_fit(camera["mapping"])
        quality = "fit_only"
    else:
        calibration = GroundCalibration.load(camera["mapping"])
        if not calibration.valid:
            raise ValueError("正式标定文件必须通过独立验证")
        provenance = {"mapping": camera["mapping"]}
        quality = "validated"
    return calibration, quality, provenance


def roi_coverage(box, roi):
    x1, y1, x2, y2 = box
    width = max(0, min(x2, roi[2]) - max(x1, roi[0]))
    height = max(0, min(y2, roi[3]) - max(y1, roi[1]))
    return width * height / ((x2 - x1) * (y2 - y1))


def unclassified_dark_regions(image, rows, roi, guard):
    """Find interior dark regions left unexplained by recognized material boxes.

    Border-connected components belong to the surrounding jaw/background mask.
    This guard reports uncertainty and never assigns an object class.
    """
    if not guard.get("enabled", False):
        return []
    mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 0, guard.get("value_max", 55))
    for row in rows:
        if row["name"] in TARGET_TYPES:
            x1, y1, x2, y2 = map(round, row["box"])
            cv2.rectangle(mask, (max(0, x1 - 3), max(0, y1 - 3)), (x2 + 3, y2 + 3), 0, -1)
    left, top, right, bottom = map(round, roi)
    mask = cv2.morphologyEx(mask[top:bottom, left:right], cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    _, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    regions = []
    for x, y, width, height, area in stats[1:]:
        if (area >= guard.get("min_area_px", 500) and x > 1 and y > 1 and
                x + width < mask.shape[1] - 1 and y + height < mask.shape[0] - 1):
            regions.append([int(left + x), int(top + y), int(left + x + width), int(top + y + height)])
    return regions


class Confirmation:
    """Only consecutive, associated observations of the same task can accumulate."""

    def __init__(self, frames, iou):
        self.frames, self.iou = frames, iou
        self.reset()

    def reset(self):
        self.key, self.box, self.count = None, None, 0

    def update(self, key, box=None):
        associated = box is None or (self.box is not None and box_iou(self.box, box) >= self.iou)
        self.count = self.count + 1 if key == self.key and associated else 1
        self.key, self.box = key, box
        return self.count >= self.frames


class CameraPerception:
    def __init__(self, role, calibration, quality, cfg):
        self.role, self.calibration, self.quality, self.cfg = role, calibration, quality, dict(cfg)
        declared_role = calibration.data.get("camera_role")
        if declared_role is not None and declared_role != role:
            raise ValueError("标定文件的上下视角色与配置不一致")
        self.selection = Confirmation(cfg["confirm_frames"], cfg["association_iou"])
        self.grip = Confirmation(cfg["confirm_frames"], cfg["association_iou"])
        self.last_stamp = None
        self.task_key = None
        self.projector = GreenSession(
            calibration, scope="vehicle_measurement_only",
            output_range_mm=cfg["cameras"][role]["output_range_mm"],
            allow_near_estimate=True, near_min_mm=0)
        roi = cfg["gripper"]["roi_raw_xyxy"]
        if role == "lower" and roi and (roi[2] > calibration.size[0] or roi[3] > calibration.size[1]):
            raise ValueError("夹爪ROI超出下视标定分辨率")

    def reset(self):
        self.selection.reset()
        self.grip.reset()

    def process(self, image, detections, captured, now, task):
        key = tuple(task.get(k) for k in ("session_id", "task_id", "expected", "source", "state"))
        age = (now - captured) * 1000
        if key != self.task_key or (self.last_stamp is not None and
                                   captured - self.last_stamp > self.cfg["max_frame_age_ms"] / 1000):
            self.reset()
        self.task_key = key
        result = {"role": self.role, "status": "OK", "captured_monotonic": captured,
                  "age_ms": age, "detections": [], "selected": None,
                  "calibration_quality": self.quality, "motion_valid": False,
                  "hazard_blocked": False, "task_key": list(key),
                  "gripper": {"status": "NOT_APPLICABLE", "match": False, "count": 0}}
        if (not math.isfinite(age) or not 0 <= age <= self.cfg["max_frame_age_ms"] or
                (self.last_stamp is not None and captured <= self.last_stamp)):
            self.reset()
            result["status"] = "STALE"
            if not math.isfinite(age):
                result.update(age_ms=None, captured_monotonic=None)
            return result
        self.last_stamp = captured
        if (not isinstance(image, np.ndarray) or image.dtype != np.uint8 or
                image.shape != (self.calibration.size[1], self.calibration.size[0], 3)):
            self.reset()
            result["status"] = "RESOLUTION_MISMATCH"
            return result
        width, height = self.calibration.size
        for d in detections:
            if (len(d.box) != 4 or not np.isfinite(d.box).all() or not math.isfinite(d.confidence)
                    or not 0 <= d.confidence <= 1
                    or not 0 <= d.box[0] < d.box[2] <= width
                    or not 0 <= d.box[1] < d.box[3] <= height):
                self.reset()
                result["status"] = "INVALID_DETECTIONS"
                return result
        detections = [d for d in detections if d.confidence >= self.cfg["confidence"]]
        roi = self.cfg["gripper"]["roi_raw_xyxy"] if self.role == "lower" else None
        for d in detections:
            inside_grip = bool(roi and roi_coverage(d.box, roi) >= self.cfg["grip_coverage"])
            row = {"name": d.name, "confidence": d.confidence, "box": list(d.box),
                   "position_mm": None, "position_quality": "unavailable", "angle": None,
                   "inside_gripper_roi": inside_grip, "position_valid": False,
                   "center_offset_normalized": ((d.box[0] + d.box[2]) / width - 1)}
            # An object held above the ground must never be projected through ground H.
            if d.name in TARGET_TYPES and not inside_grip and not touches_border(d.box, self.calibration.size):
                overlapping = any(other is not d and other.name in TARGET_TYPES and
                                  box_iou(d.box, other.box) > .12 for other in detections)
                contact = None
                if not overlapping:
                    contact = (green_contact(image, d.box, LOWER, UPPER) if d.name == "normal_material"
                               else ((d.box[0] + d.box[2]) / 2, d.box[3] - 1))
                if contact is not None:
                    pixel = cv2.undistortPoints(np.array([[contact]], np.float64), self.calibration.matrix,
                                               self.calibration.distortion, P=self.calibration.matrix)[0, 0]
                    point, near = self.projector.project_contact(pixel)
                    if point is not None:
                        x, y = point
                        row["position_mm"] = {"x_right": x, "y_forward": y,
                                              "ground_range": math.hypot(x, y),
                                              "bearing_deg": math.degrees(math.atan2(x, y))}
                        row["angle"] = row["position_mm"]["bearing_deg"]
                        row["position_quality"] = "near_estimate" if near else self.quality
                        if d.name != "normal_material":
                            row["position_quality"] = "bbox_ground_estimate"
                        row["position_valid"] = (self.calibration.valid and not near and
                                                 d.name == "normal_material")
            result["detections"].append(row)
        result["hazard_blocked"] = any(d.name == "hazard" or d.name not in KNOWN for d in detections)
        expected = task.get("expected")
        candidates = [row for row in result["detections"] if row["name"] == expected
                      and not row["inside_gripper_roi"]]
        if candidates:
            if self.selection.box is None:
                selected = min(candidates, key=lambda r: abs((r["box"][0] + r["box"][2]) / 2 - width / 2))
            else:
                selected = max(candidates, key=lambda r: box_iou(self.selection.box, r["box"]))
            selected = dict(selected)
            selected["confirmed"] = self.selection.update((key, expected), selected["box"])
            if selected["position_mm"] is None and expected in MATERIALS:
                self.selection.reset()
                selected["confirmed"] = False
            selected["confirm_count"] = self.selection.count
            result["selected"] = selected
            result["motion_valid"] = (selected["confirmed"] and selected["position_valid"]
                                      and age <= self.cfg["control_max_age_ms"]
                                      and not result["hazard_blocked"] and task.get("source") == "controller")
        else:
            self.selection.reset()
        if self.role == "lower":
            result["gripper"] = self.check_gripper(result["detections"], expected, key, image)
            if result["gripper"].get("unknown_regions"):
                result.update(hazard_blocked=True, motion_valid=False)
        return result

    def check_gripper(self, rows, expected, task_key, image):
        value = {"status": "UNCONFIGURED", "match": False, "count": 0, "observed": None}
        if self.cfg["gripper"]["roi_raw_xyxy"] is None:
            self.grip.reset()
            return value
        if expected not in MATERIALS:
            self.grip.reset()
            return dict(value, status="WAITING_TASK")
        objects = [row for row in rows if row["inside_gripper_roi"]
                   and row["name"] not in {"gripper", "gripper_closed"}]
        if any(row["name"] == "hazard" for row in objects):
            self.grip.reset()
            return dict(value, status="HAZARD", observed="hazard")
        if len(objects) > 1:
            self.grip.reset()
            return dict(value, status="AMBIGUOUS")
        unexplained = unclassified_dark_regions(image, rows, self.cfg["gripper"]["roi_raw_xyxy"],
                                                self.cfg["gripper"].get("dark_guard", {}))
        if unexplained:
            self.grip.reset()
            return dict(value, status="AMBIGUOUS" if objects else "UNCLASSIFIED_OBJECT",
                        unknown_regions=unexplained)
        candidate = objects[0] if objects else None
        observed = candidate["name"] if candidate else None
        outcome = "EMPTY" if candidate is None else ("MATCH" if observed == expected else "MISMATCH")
        stable = self.grip.update((task_key, outcome, observed), candidate["box"] if candidate else None)
        status = outcome if stable else "CONFIRMING"
        validated = self.cfg["gripper"]["validated"]
        if stable and not validated:
            status += "_UNVERIFIED"
        return dict(value, status=status, observed=observed, count=self.grip.count,
                    match=stable and outcome == "MATCH" and validated)

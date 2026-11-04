"""Reuse the inspected board's Ultralytics RKNN backend and local runtime selector."""
import hashlib
import importlib.util
from pathlib import Path
import re
import sys
import subprocess
import time

import cv2
import numpy as np

from .types import TARGET_TYPES


UVC_CONTROL_VALUES = {"auto_exposure": (1, 3), "exposure_time_absolute": range(1, 5001),
                      "exposure_dynamic_framerate": (0, 1), "backlight_compensation": (0, 1, 2)}


def validate_uvc_controls(controls):
    if not isinstance(controls, dict):
        raise ValueError("UVC controls must be an object")
    for name, value in controls.items():
        if name not in UVC_CONTROL_VALUES or type(value) is not int or value not in UVC_CONTROL_VALUES[name]:
            raise ValueError(f"Invalid UVC control: {name}")


def apply_uvc_controls(camera, controls):
    validate_uvc_controls(controls)
    if not controls:
        return {}
    values = ",".join(f"{name}={value}" for name, value in controls.items())
    result = subprocess.run(["v4l2-ctl", "-d", str(camera), "--set-ctrl=" + values],
                            capture_output=True, text=True, timeout=3)
    if result.returncode:
        raise ValueError("UVC control update failed: " + result.stderr.strip())
    text = subprocess.check_output(["v4l2-ctl", "-d", str(camera), "--get-ctrl=" + ",".join(controls)],
                                   text=True, timeout=3)
    actual = {line.partition(":")[0].strip(): int(line.partition(":")[2].split()[0])
              for line in text.splitlines() if ":" in line}
    if actual != controls:
        raise ValueError(f"Camera rejected UVC controls: {actual}")
    return actual


def decode_existing_output(outputs, class_count, confidence, nms_iou):
    """Decode the inspected [1, 4+classes, 8400] model, preserving class-aware NMS."""
    from .detector import nms
    if len(outputs) != 1:
        raise ValueError("Expected one decoded Ultralytics RKNN output")
    output = np.asarray(outputs[0])
    if (output.ndim != 3 or output.shape[:2] != (1, 4 + class_count) or
            output.shape[2] != 8400 or not np.isfinite(output).all()):
        raise ValueError("Existing RKNN output shape or values differ from metadata")
    predictions = output[0].T.astype(np.float32)
    classes = predictions[:, 4:].argmax(axis=1)
    scores = predictions[:, 4:].max(axis=1)
    selected = (scores > confidence) & (predictions[:, 2] > 0) & (predictions[:, 3] > 0)
    predictions, classes, scores = predictions[selected], classes[selected], scores[selected]
    boxes = np.column_stack((predictions[:, 0] - predictions[:, 2] / 2,
                             predictions[:, 1] - predictions[:, 3] / 2,
                             predictions[:, 0] + predictions[:, 2] / 2,
                             predictions[:, 1] + predictions[:, 3] / 2))
    kept = []
    for category in np.unique(classes):
        subset = np.flatnonzero(classes == category)
        kept.extend(subset[nms(boxes[subset], scores[subset], nms_iou)])
    kept = sorted(kept, key=lambda index: -scores[index])[:300]
    return boxes[kept], classes[kept], scores[kept]


def verify_uvc_device(camera, sys_class=Path("/sys/class/video4linux")):
    """Reject HDMI/meta nodes before opening a stream. Uses only sysfs metadata."""
    node = Path(camera).resolve(strict=True)
    if not re.fullmatch(r"video[0-9]+", node.name):
        raise ValueError("Expected a V4L2 video capture node")
    entry = sys_class/node.name
    name = (entry/"name").read_text().strip()
    driver = (entry/"device/driver").resolve(strict=True).name
    if driver != "uvcvideo" or name == "stream_hdmirx":
        raise ValueError(f"{node} is not the USB UVC camera: {name}, driver={driver}")
    index = entry/"index"
    if index.exists() and index.read_text().strip() != "0":
        raise ValueError("Use the UVC video-index0 capture node, not its metadata node")
    return str(node)


class ExistingUltralyticsDetector:
    def __init__(self, config):
        import yaml
        self.config = config
        self.model = None
        self.rknn_model = None
        self.last_timing = {}
        self.closed = False
        project = Path(config["existing_project"]).resolve()
        helper = project/"rknn_compat.py"
        directory = Path(config["model"]).resolve()
        files = list(directory.glob("*.rknn")) if directory.is_dir() else []
        if len(files) != 1 or not (directory/"metadata.yaml").is_file():
            raise ValueError("Existing RKNN directory must contain one .rknn and metadata.yaml")
        expected = config.get("expected_model_sha256")
        if not expected or hashlib.sha256(files[0].read_bytes()).hexdigest() != expected:
            raise ValueError("Existing RKNN model SHA256 does not match the inspected model")
        if hashlib.sha256(helper.read_bytes()).hexdigest() != config.get("expected_runtime_helper_sha256"):
            raise ValueError("rknn_compat.py differs from the inspected runtime selector")
        data = yaml.safe_load((directory/"metadata.yaml").read_text(encoding="utf-8"))
        mapping = data.get("names")
        if isinstance(mapping, dict):
            if set(mapping) != set(range(len(mapping))):
                raise ValueError("Class IDs in model metadata must be contiguous integers")
            self.names = [mapping[i] for i in range(len(mapping))]
        elif isinstance(mapping, list):
            self.names = mapping
        else:
            raise ValueError("Missing actual model class map")
        if (data.get("task") != "detect" or data.get("imgsz") != [640, 640]
                or not all(isinstance(n, str) for n in self.names)
                or not set(TARGET_TYPES) <= set(self.names)):
            raise ValueError("Unsupported existing model metadata")
        module = sys.modules.get("rknn_compat")
        if module is not None:
            if Path(module.__file__).resolve() != helper:
                raise RuntimeError("Another project's rknn_compat is already imported")
        else:
            spec = importlib.util.spec_from_file_location("rknn_compat", helper)
            module = importlib.util.module_from_spec(spec)
            sys.modules["rknn_compat"] = module
            try:
                spec.loader.exec_module(module)
            except Exception:
                del sys.modules["rknn_compat"]
                raise
        # Match the board's existing run_yolo.py/server.py initialization order.
        # The helper uses verified runtime-2.3.2, not the old /usr/lib runtime.
        module.configure_rknn_runtime()
        if config.get("rknn_fast_path", False):
            from rknnlite.api import RKNNLite
            core_mask = config.get("rknn_core_mask", 0)
            if type(core_mask) is not int or core_mask not in (0, 1, 2, 3, 4, 7):
                raise ValueError("Unsupported RKNN core mask")
            runtime = RKNNLite()
            try:
                if runtime.load_rknn(str(files[0])) != 0 or runtime.init_runtime(core_mask=core_mask) != 0:
                    raise RuntimeError("Existing RKNN load/init failed")
            except Exception:
                runtime.release()
                raise
            self.rknn_model = runtime
            cv2.setNumThreads(1)
            return
        from ultralytics import YOLO
        if "cpu_threads" in config:
            import torch
            from ultralytics.utils import torch_utils
            # select_device('cpu') uses this module value on each prediction.
            # Change only this process; never rewrite installed source or system settings.
            torch_utils.NUM_THREADS = config["cpu_threads"]
            torch.set_num_threads(config["cpu_threads"])
            cv2.setNumThreads(1)
        self.model = YOLO(str(directory), task="detect")

    def predict(self, image):
        from .session import detections_from_arrays
        if self.closed:
            raise RuntimeError("Detector already closed")
        if self.rknn_model is not None:
            from .detector import letterbox, restore_boxes
            started = time.monotonic()
            padded, transform = letterbox(image)
            inputs = np.ascontiguousarray(padded[:, :, ::-1][None])
            prepared = time.monotonic()
            outputs = self.rknn_model.inference(inputs=[inputs])
            inferred = time.monotonic()
            if outputs is None:
                raise RuntimeError("Existing RKNN inference failed")
            boxes, classes, scores = decode_existing_output(outputs, len(self.names),
                                                            self.config["confidence"], self.config["nms_iou"])
            boxes = restore_boxes(boxes, transform, image.shape)
            valid = (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1])
            result = detections_from_arrays(boxes[valid], scores[valid], classes[valid], self.names)
            finished = time.monotonic()
            self.last_timing = {"preprocess_ms": (prepared - started) * 1000,
                                "npu_ms": (inferred - prepared) * 1000,
                                "postprocess_ms": (finished - inferred) * 1000}
            return result
        result = self.model.predict(image, imgsz=640, rect=False, device="cpu",
                                    conf=self.config["confidence"], iou=self.config["nms_iou"],
                                    verbose=False)[0]
        names = [result.names[i] for i in range(len(result.names))]
        if names != self.names:
            raise ValueError("Runtime class map differs from inspected metadata")
        if result.boxes is None:
            return []
        # Ultralytics already restores xyxy into the ORIGINAL input image.
        boxes = result.boxes
        return detections_from_arrays(boxes.xyxy.tolist(), boxes.conf.tolist(),
                                      boxes.cls.tolist(), result.names)

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.rknn_model is not None:
            self.rknn_model.release()
            return
        predictor = getattr(self.model, "predictor", None)
        backend = getattr(predictor, "model", None)
        # Attribute checked in the board's ultralytics/nn/autobackend.py.
        rknn = getattr(backend, "rknn_model", None)
        if rknn is not None:
            rknn.release()

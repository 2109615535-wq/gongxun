"""Reuse the inspected board's Ultralytics RKNN backend and local runtime selector."""
import hashlib
import importlib.util
from pathlib import Path
import re
import sys

from .types import TARGET_TYPES


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
        from ultralytics import YOLO
        if "cpu_threads" in config:
            import cv2
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
        predictor = getattr(self.model, "predictor", None)
        backend = getattr(predictor, "model", None)
        # Attribute checked in the board's ultralytics/nn/autobackend.py.
        rknn = getattr(backend, "rknn_model", None)
        if rknn is not None:
            rknn.release()

"""Consume an already-working board detector without loading/re-exporting its model."""
from dataclasses import replace
import math
import time

import numpy as np

from .config import load_config
from .geometry import GroundCalibration
from .pipeline import Perception
from .types import Detection, Status


def detections_from_arrays(boxes_xyxy, scores, class_ids, class_names):
    """Input boxes must already be restored to the ORIGINAL camera image pixels."""
    if boxes_xyxy is None and scores is None and class_ids is None:
        return []
    if boxes_xyxy is None or scores is None or class_ids is None:
        raise ValueError("Only an entirely empty result may use None arrays")
    boxes = np.asarray(boxes_xyxy, np.float64).reshape(-1, 4)
    scores = np.asarray(scores, np.float64).reshape(-1)
    ids = np.asarray(class_ids, np.float64).reshape(-1)
    if len(boxes) != len(scores) or len(boxes) != len(ids):
        raise ValueError("Box/score/class counts differ")
    if (not np.isfinite(boxes).all() or not np.isfinite(scores).all()
            or not np.isfinite(ids).all() or np.any(scores < 0) or np.any(scores > 1)
            or np.any(ids != np.floor(ids)) or np.any(ids < 0)):
        raise ValueError("Invalid detection arrays")
    result = []
    for box, score, index in zip(boxes, scores, ids):
        try:
            name = class_names[int(index)]
        except (IndexError, KeyError, TypeError) as exc:
            raise ValueError("Class ID is missing from the ACTUAL model's class map") from exc
        if not isinstance(name, str) or not name:
            raise ValueError("Class names must be nonempty strings")
        result.append(Detection(name, float(score), tuple(map(float, box))))
    return result


class DetectionSession:
    """Single-producer API: no camera open, model import, UART open or motor commands."""

    def __init__(self, config):
        self.config = config
        self.calibration = GroundCalibration.load(config["calibration"])
        if self.calibration.size != (config["width"], config["height"]):
            raise ValueError("Image resolution differs from calibration")
        self.pipeline = Perception(self.calibration, config)
        self.last_stamp = None

    @classmethod
    def from_config(cls, path):
        return cls(load_config(path))

    def observe(self, frame, detections, captured_monotonic, *, now_monotonic=None, pixel_space="raw"):
        now = time.monotonic() if now_monotonic is None else now_monotonic
        if not math.isfinite(captured_monotonic) or not math.isfinite(now):
            return self.pipeline.fault(0, Status.STALE)
        tick = int(captured_monotonic*1000) & 0xFFFFFFFF
        if (not 0 <= now-captured_monotonic <= self.config["max_frame_age_ms"]/1000
                or (self.last_stamp is not None and captured_monotonic <= self.last_stamp)):
            return self.pipeline.fault(tick, Status.STALE)
        if self.last_stamp is not None and captured_monotonic-self.last_stamp >= .3:
            self.pipeline.reset()
        self.last_stamp = captured_monotonic
        try:
            if (not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] != 3
                    or frame.dtype != np.uint8 or frame.shape[1::-1] != self.calibration.size):
                raise ValueError("Expected original-resolution uint8 BGR camera frame")
            checked = []
            for d in detections:
                if not isinstance(d, Detection) or not isinstance(d.name, str):
                    raise ValueError("Expected Detection objects")
                coords = np.asarray(d.box, np.float64)
                if (coords.shape != (4,) or not np.isfinite(coords).all()
                        or not math.isfinite(d.confidence) or not 0 <= d.confidence <= 1):
                    raise ValueError("Invalid detection values")
                x1, y1, x2, y2 = coords
                if not (0 <= x1 < x2 <= frame.shape[1] and 0 <= y1 < y2 <= frame.shape[0]):
                    raise ValueError("Boxes must be restored and clipped to original image pixels")
                if d.confidence >= self.config["confidence"]:
                    checked.append(d)
            return self.pipeline.process(frame, checked, tick, pixel_space=pixel_space)
        except (ValueError, TypeError) as exc:
            observation = self.pipeline.fault(tick, Status.TARGET_INVALID)
            return replace(observation, diagnostics={"input_error": str(exc)})

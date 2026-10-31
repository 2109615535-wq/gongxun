import json
import re
from pathlib import Path


def load_config(path):
    path = Path(path).resolve()
    cfg = json.loads(path.read_text(encoding="utf-8"))
    cfg.setdefault("serial_protocol", "motor2_debug_ascii")
    cfg.setdefault("detector_backend", "reference")
    cfg.setdefault("detector_pixel_space", "undistorted")
    for key in ("model", "manifest", "calibration"):
        value = Path(cfg[key])
        cfg[key] = str(value if value.is_absolute() else path.parent/value)
    if "existing_project" in cfg:
        value = Path(cfg["existing_project"])
        cfg["existing_project"] = str(value if value.is_absolute() else path.parent/value)
    if cfg["detector_backend"] == "existing_ultralytics_rknn":
        for key in ("expected_model_sha256", "expected_runtime_helper_sha256"):
            if not re.fullmatch(r"[0-9a-f]{64}", cfg.get(key, "")):
                raise ValueError(f"{key} must be exactly 64 lowercase hexadecimal characters")
    if (cfg["confirm_frames"] < 3 or not 0 < cfg["association_iou"] <= 1
            or not 0 < cfg["smoothing_alpha"] <= 1
            or not 0 < cfg["max_frame_age_ms"] <= 150
            or cfg["baudrate"] not in (115200, 921600) or not 0 < cfg["confidence"] < 1
            or cfg.get("serial_protocol", "vision_v1") not in ("vision_v1", "motor2_debug_ascii")
            or cfg["detector_backend"] not in ("reference", "existing_ultralytics_rknn")
            or cfg["detector_pixel_space"] not in ("raw", "undistorted")
            or ("cpu_threads" in cfg and (isinstance(cfg["cpu_threads"], bool)
                                          or not isinstance(cfg["cpu_threads"], int)
                                          or not 1 <= cfg["cpu_threads"] <= 4))
            or (cfg.get("fourcc") is not None and len(cfg["fourcc"]) != 4)
            or not 0 < cfg["nms_iou"] < 1 or cfg["hazard_corridor_half_width_mm"] <= 0
            or min(cfg["width"], cfg["height"], cfg["fps"]) <= 0):
        raise ValueError("Invalid vision configuration")
    return cfg

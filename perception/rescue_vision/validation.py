from dataclasses import asdict
import csv
import json
from pathlib import Path

import cv2
import numpy as np

from .detector import Detector, box_iou, create_detector
from .geometry import GroundCalibration
from .model_tools import write_json
from .pipeline import Perception
from .protocol import encode
from .runtime import Recorder, annotate
from .types import FrameFlag, Status


def image_files(directory):
    return sorted(p for p in Path(directory).iterdir() if p.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp"))


def compare_models(reference, candidate, manifest, images, output, limit=None):
    files = image_files(images)
    if limit:
        files = files[:limit]
    if not files:
        raise ValueError("No evaluation images")
    first, second = Detector(reference, manifest), Detector(candidate, manifest)
    rows = []
    try:
        for path in files:
            image = cv2.imread(str(path))
            if image is None:
                raise ValueError(f"Cannot read {path}")
            a, b = first.predict(image), second.predict(image)
            unused = set(range(len(b)))
            matches, missing = [], []
            for detection in sorted(a, key=lambda d: -d.confidence):
                options = [(box_iou(detection.box, b[k].box), k) for k in unused
                           if b[k].name == detection.name]
                if not options or max(options)[0] < .85:
                    missing.append(asdict(detection))
                    continue
                overlap, index = max(options)
                unused.remove(index)
                matches.append(dict(name=detection.name, iou=overlap,
                                    confidence_difference=abs(detection.confidence-b[index].confidence)))
            rows.append(dict(image=str(path), matches=matches, missing=missing,
                             extra=[asdict(b[k]) for k in sorted(unused)]))
    finally:
        first.close()
        second.close()
    passed = all(not r["missing"] and not r["extra"] and
                 all(m["confidence_difference"] <= .1 for m in r["matches"]) for r in rows)
    report = dict(passed=passed, images=len(rows), reference=str(reference), candidate=str(candidate),
                  criterion="same classes/counts, matched IoU >= .85, absolute confidence difference <= .1",
                  note="转换一致性检查，不等同于标注集检测精度或危险目标召回率验收。", results=rows)
    write_json(output, report)
    return report


def replay(config, source, output, video=True, limit=None):
    detector = create_detector(config)
    calibration = GroundCalibration.load(config["calibration"])
    perception = Perception(calibration, config)
    recorder = Recorder(output, calibration.size, video)
    capture = cv2.VideoCapture(str(source))
    try:
        if not capture.isOpened():
            raise ValueError(f"Cannot open {source}")
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not fps or not np.isfinite(fps) or fps <= 0:
            raise ValueError("Replay video has no valid frame rate")
        count = 0
        while limit is None or count < limit:
            ok, raw = capture.read()
            if not ok:
                break
            if raw.shape[1::-1] != calibration.size:
                raise ValueError("Replay resolution differs from calibration")
            space = config.get("detector_pixel_space", "undistorted")
            image = raw if space == "raw" else calibration.undistort(raw)
            detections = detector.predict(image)
            observation = perception.process(image, detections, round(count/fps*1000), pixel_space=space)
            recorder.record(observation, encode(observation, count), frame_seq=count,
                            offline_replay=True, transmitted=False, source_video_fps=fps,
                            detections=[asdict(d) for d in detections])
            recorder.frame(raw, annotate(image, detections, observation))
            count += 1
        return dict(frames=count, offline_only=True, output=str(output))
    finally:
        capture.release()
        detector.close()
        recorder.close()


def performance_report(log_path, output):
    rows = [json.loads(line) for line in Path(log_path).read_text(encoding="utf-8").splitlines() if line.strip()]
    if any(r.get("offline_replay") for r in rows):
        raise ValueError("Offline replay cannot establish board real-time performance")
    frames = [r for r in rows if r.get("capture_to_send_ms") is not None]
    valid = [r for r in frames if r["status"] == "OK" and r["flags"] & int(FrameFlag.VISION_VALID)]
    span = max((r["sent_monotonic"] for r in rows), default=0)-min(
        (r["sent_monotonic"] for r in rows), default=0)
    latency = [r["capture_to_send_ms"] for r in frames]
    hz = len(valid)/span if span > 0 else 0
    p95 = float(np.percentile(latency, 95)) if latency else None
    transmitted = bool(rows) and all(r.get("transmitted", False) for r in rows)
    report = dict(passed=bool(len(valid) >= 30 and span >= 2 and hz >= 15 and
                              p95 is not None and p95 <= 150 and transmitted),
                  valid_frames=len(valid), frame_count=len(frames), duration_s=span,
                  valid_observations_hz=hz, capture_to_serial_send_p95_ms=p95,
                  serial_transmitted=transmitted,
                  note="时间起点为USB read返回，终点为串口write返回；不包含曝光/相机内部缓冲与STM32执行延迟。")
    write_json(output, report)
    return report


def trial_report(path, output):
    with Path(path).open(encoding="utf-8-sig", newline="") as source:
        rows = list(csv.DictReader(source))
    approach = [r for r in rows if r["scenario"] == "approach"]
    def success(row):
        return (170 <= float(row["measured_gap_mm"]) <= 230 and
                abs(float(row["measured_lateral_mm"])) <= 20 and row["stopped"] == "1")
    completed = sum(success(r) for r in approach)
    faults = {name: [r for r in rows if r["scenario"] == name]
              for name in ("occlusion", "camera_disconnect", "serial_disconnect", "hazard")}
    fault_passed = all(group and all(r["stopped"] == "1" and
                                   float(r["stop_latency_ms"]) <= 300 for r in group)
                       for group in faults.values())
    report = dict(passed=len(approach) >= 30 and completed/len(approach) >= .9 and bool(fault_passed),
                  approach_trials=len(approach), successful_approaches=completed,
                  fault_tests_passed=bool(fault_passed),
                  note="使用独立尺量和实测停车时间填写；仿真结果不得冒充实车验收。")
    write_json(output, report)
    return report

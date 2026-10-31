"""Desktop-only C100 green-object measurement using a checked bench homography."""

import argparse
from datetime import datetime
import hashlib
import json
from math import hypot
import os
from pathlib import Path
import threading
import time

import cv2
import numpy as np

from .detector import box_iou
from .geometry import GroundCalibration, green_contact, touches_border
from .runtime import LatestFrame
from .types import Detection

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = ROOT / "artifacts/ground_experiment/20261030_180403_198681_check_report.json"
DEFAULT_WEIGHTS = ROOT.parent / "smart-rescue-sim/gazebo/weights/best.pt"
MODEL_SHA256 = "ab67fffaad0bbdac0f915941a328a310d55bc5a45fd5bc60f4815bbd33146d89"
LOWER, UPPER = [35, 65, 40], [90, 255, 255]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_bench(report_path, mapping_path=None):
    """Authorize numerical BENCH measurement without enabling vehicle validity."""
    report_path = Path(report_path)
    report = json.loads(report_path.read_text(encoding="utf-8-sig"))
    errors = np.asarray(report.get("max_abs_error_user_xy_mm", []), np.float64)
    if (report.get("scope") != "independent_bench_check_only"
            or report.get("bench_check_passed") is not True
            or len(report.get("points", [])) < 6
            or not all(p.get("within_error_limits") is True for p in report["points"])
            or errors.shape != (2,) or not np.isfinite(errors).all()
            or (errors > [20, 30]).any()):
        raise ValueError("需要通过独立6点检查的报告，不能使用拟合偏差代替检查结果。")
    mapping_path = Path(mapping_path or report["mapping_source"])
    if sha256(mapping_path) != report.get("mapping_sha256"):
        raise ValueError("地面映射已变化，与检查报告不匹配；请重新检查。")
    data = json.loads(mapping_path.read_text(encoding="utf-8-sig"))
    # Keep the source file and all vehicle-control flags unchanged.
    calibration = GroundCalibration(dict(data, validated=False))
    k, h, d, polygon = (calibration.matrix, calibration.homography,
                        calibration.distortion, calibration.polygon)
    if (k is None or h is None or d is None or k.shape != (3, 3) or h.shape != (3, 3)
            or d.size not in (4, 5, 8, 12, 14) or polygon.ndim != 2
            or polygon.shape[1:] != (2,) or len(polygon) < 3
            or not all(np.isfinite(a).all() for a in (k, h, d, polygon))
            or k[0, 0] <= 0 or k[1, 1] <= 0
            or abs(cv2.determinant(h)) < 1e-12
            or calibration.minimum >= calibration.maximum
            or min(calibration.size) <= 0):
        raise ValueError("地面标定结构无效。")
    return calibration, {
        "check_report": str(report_path.resolve()),
        "check_report_sha256": sha256(report_path),
        "mapping": str(mapping_path.resolve()),
        "mapping_sha256": sha256(mapping_path),
    }


class GreenSession:
    """Fresh-frame observations only; no motor or serial interfaces."""

    def __init__(self, calibration, confirm_frames=3, max_age_ms=300, near_min_mm=0):
        self.calibration = calibration
        self.confirm_frames = confirm_frames
        self.max_age_ms = max_age_ms
        self.near_min_mm = near_min_mm
        measured = np.asarray([p["ground_mm"] for p in calibration.data.get("ground_fit_points", [])],
                              np.float64)
        self.near_left_bounds = None
        if (measured.ndim == 2 and measured.shape[1] == 2 and len(measured) >= 4
                and np.isfinite(measured).all()):
            lo, hi = float(measured[:, 1].min()), float(measured[:, 1].max())
            if lo < hi:
                self.near_left_bounds = (lo, hi)
        self.reset()

    def project_contact(self, rectified):
        """Return a checked-region point or a separately labelled NEAR estimate.

        The original calibration hull and range are never expanded or rewritten.
        Only forward extrapolation towards the origin is allowed, within the
        measured lateral span. Far/lateral extrapolation remains unavailable.
        """
        point = self.calibration.project(rectified, require_valid=False)
        if point is not None:
            return point, False
        if self.near_min_mm is None or self.near_left_bounds is None:
            return None, False
        homogeneous = self.calibration.homography @ np.array([*rectified, 1.], np.float64)
        if not np.isfinite(homogeneous).all() or abs(homogeneous[2]) < 1e-9:
            return None, False
        forward, left = homogeneous[:2]/homogeneous[2]
        lo, hi = self.near_left_bounds
        if (self.near_min_mm <= forward < self.calibration.minimum and lo <= left <= hi):
            return (float(forward), float(left)), True
        return None, False

    def reset(self):
        self.box = None
        self.count = 0

    def process(self, image, detections, age_ms=0, temporal=True):
        if image.shape[1::-1] != self.calibration.size:
            self.reset()
            raise ValueError(f"实际图像尺寸{image.shape[1::-1]}与标定{self.calibration.size}不一致。")
        result = {
            "scope": "bench_only", "motion_valid": False,
            "status": "NO_TARGET", "position_mm": None, "position_quality": None,
            "contact_raw_pixel": None, "contact_undistorted_pixel": None,
            "selected_box": None, "confidence": None, "confirm_count": 0,
            "age_ms": float(age_ms),
        }
        if not np.isfinite(age_ms) or age_ms > self.max_age_ms:
            self.reset()
            result["status"] = "STALE"
            return result
        normals = [d for d in detections if d.name == "normal_material"]
        if not normals:
            self.reset()
            return result
        if self.box is None:
            chosen = min(normals, key=lambda d: abs((d.box[0]+d.box[2])/2-image.shape[1]/2))
        else:
            chosen = max(normals, key=lambda d: box_iou(self.box, d.box))
            if box_iou(self.box, chosen.box) < .2:
                self.reset()
        width, height = self.calibration.size
        box = tuple(float(v) for v in chosen.box)
        if len(box) != 4 or not np.isfinite(box).all():
            self.reset()
            result["status"] = "BAD_BOX"
            return result
        result.update(selected_box=list(box), confidence=float(chosen.confidence))
        invalid = None
        if (box[2] <= box[0] or box[3] <= box[1]
                or box[0] < 0 or box[1] < 0 or box[2] > width or box[3] > height):
            invalid = "BAD_BOX"
        elif touches_border(box, self.calibration.size):
            invalid = "BORDER"
        elif any(other is not chosen and box_iou(box, other.box) > .12
                 for other in detections):
            invalid = "OVERLAP"
        if invalid:
            self.reset()
            result["status"] = invalid
            return result
        contact = green_contact(image, box, LOWER, UPPER)
        if contact is None:
            self.reset()
            result["status"] = "CONTACT_INVALID"
            return result
        rectified = cv2.undistortPoints(np.array([[contact]], np.float64),
                                       self.calibration.matrix, self.calibration.distortion,
                                       P=self.calibration.matrix)[0, 0]
        result.update(contact_raw_pixel=list(contact), contact_undistorted_pixel=rectified.tolist())
        point, near_estimate = self.project_contact(rectified)
        if point is None:
            self.reset()
            result["status"] = "OUTSIDE_CALIBRATION"
            return result
        self.box = box
        self.count += 1
        result["confirm_count"] = self.count
        result["position_quality"] = "near_estimate" if near_estimate else "inside_calibration"
        if temporal and self.count < self.confirm_frames:
            result["status"] = "CONFIRMING"
            return result
        x_right, y_forward = -point[1], point[0]
        result["position_mm"] = {
            "x_right": float(x_right), "y_forward": float(y_forward),
            "ground_range": float(hypot(x_right, y_forward)),
        }
        if near_estimate:
            result["status"] = "NEAR_ESTIMATE" if temporal else "PHOTO_NEAR_ESTIMATE"
        else:
            result["status"] = "OK" if temporal else "PHOTO_OK"
        return result


class YoloDetector:
    def __init__(self, weights, confidence, device, threads, config_dir, expected_sha):
        weights = Path(weights)
        if not weights.is_file():
            raise ValueError(f"模型文件不存在：{weights}")
        if expected_sha and sha256(weights) != expected_sha:
            raise ValueError("模型校验值不匹配；更换模型时需显式提供 --expected-weights-sha256。")
        os.environ.setdefault("YOLO_CONFIG_DIR", str(config_dir))
        try:
            import torch
            from ultralytics import YOLO
        except ImportError as error:
            raise ValueError("当前解释器没有YOLO依赖。请用run_green_distance.cmd，"
                             "或选择miniconda3/envs/yolo/python.exe；"
                             "仅颜色调试可用 --detector color。") from error
        torch.set_num_threads(threads)
        self.model = YOLO(str(weights))
        names = self.model.names
        if "normal_material" not in (names.values() if isinstance(names, dict) else names):
            raise ValueError("模型类别表中没有normal_material，不能套用COCO类别编号。")
        self.confidence, self.device = confidence, device
        self.metadata = {"backend": "YOLO_PT", "weights": str(weights.resolve()),
                         "weights_sha256": sha256(weights), "classes": names, "device": device}

    def __call__(self, image):
        result = self.model.predict(source=image, imgsz=640, conf=self.confidence,
                                    iou=.45, device=self.device, verbose=False, save=False)[0]
        return [Detection(result.names[int(box.cls.item())], float(box.conf.item()),
                          tuple(float(x) for x in box.xyxy.cpu().numpy()[0]))
                for box in result.boxes]


class ColorDetector:
    metadata = {"backend": "COLOR_ONLY", "note": "HSV candidates; not YOLO classification"}

    def __call__(self, image):
        mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV),
                           np.array(LOWER, np.uint8), np.array(UPPER, np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        _, _, stats, _ = cv2.connectedComponentsWithStats(mask)
        height, width = image.shape[:2]
        return [Detection("normal_material", 1., (max(0, int(x)-2), max(0, int(y)-2),
                          min(width, int(x+w)+2), min(height, int(y+h)+2)))
                for x, y, w, h, area in stats[1:] if area >= 40 and w >= 5 and h >= 5]


def render(image, detections, result, backend, fps=0):
    canvas = np.zeros((image.shape[0]+100, image.shape[1], 3), np.uint8)
    canvas[100:] = image
    def text(line, value, color=(230, 230, 230)):
        cv2.putText(canvas, value, (8, 20+line*23), cv2.FONT_HERSHEY_SIMPLEX, .44, color, 1)
    near_estimate = result.get("position_quality") == "near_estimate"
    text(0, ("BENCH | NEAR ESTIMATE: outside checked range" if near_estimate else
             "BENCH | origin: tape cross | camera-facing bottom edge"),
         (0, 180, 255) if near_estimate else (230, 230, 230))
    text(1, f"{backend} | {result['status']} | FPS {fps:.1f} | age {result['age_ms']:.0f} ms")
    position = result["position_mm"]
    if position:
        color = (0, 180, 255) if near_estimate else (0, 230, 0)
        text(2, f"X right: {position['x_right']:+.1f} mm | Y forward: {position['y_forward']:.1f} mm",
             color)
        text(3, f"Ground range from origin: {position['ground_range']:.1f} mm", color)
    else:
        text(2, "X: --   Y: --   ground range: --", (0, 180, 255))
        text(3, "Q: quit | S: save this frame and result")
    for d in detections:
        if d.name != "normal_material":
            continue
        x1, y1, x2, y2 = map(lambda x: int(round(x)), d.box)
        cv2.rectangle(canvas, (x1, y1+100), (x2, y2+100), (0, 200, 0), 1)
        label = f"green {d.confidence:.2f}" if backend != "COLOR_ONLY" else "green (HSV)"
        cv2.putText(canvas, label, (max(0, x1-10), max(110, y1+94)),
                    cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 220, 0), 1)
    if result["contact_raw_pixel"]:
        u, v = result["contact_raw_pixel"]
        cv2.drawMarker(canvas, (round(u), round(v)+100), (0, 0, 255), cv2.MARKER_CROSS, 11, 1)
    return canvas


def save_snapshot(directory, image, preview, result):
    stamp = datetime.now().strftime("%H%M%S_%f")
    for suffix, data in (("raw", image), ("annotated", preview)):
        if not cv2.imwrite(str(directory/f"{stamp}_{suffix}.png"), data):
            raise OSError("Cannot save snapshot")
    (directory/f"{stamp}_result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description="C100绿色物资与地面距离检测（离车实验）")
    parser.add_argument("--camera", default="1")
    parser.add_argument("--capture-backend", choices=["auto", "dshow", "msmf", "v4l2"], default="auto")
    parser.add_argument("--detector", choices=["yolo", "color"], default="yolo")
    parser.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument("--expected-weights-sha256", default=MODEL_SHA256)
    parser.add_argument("--check-report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--mapping", type=Path)
    parser.add_argument("--confidence", type=float, default=.35)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--max-age-ms", type=float, default=300)
    parser.add_argument("--confirm-frames", type=int, default=3)
    parser.add_argument("--near-min-mm", type=float, default=0,
                        help="近处估算的最小前向距离，默认0mm；不表示已通过标定。")
    parser.add_argument("--strict-range", action="store_true",
                        help="关闭近处估算，仅输出原标定区域内坐标。")
    parser.add_argument("--image", type=Path, help="回放一张原始照片；不要传去畸变照片")
    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--output", type=Path, default=ROOT/"artifacts/green_distance_runs")
    args = parser.parse_args(argv)
    if (not 0 < args.confidence <= 1 or args.threads < 1 or args.confirm_frames < 1
            or not np.isfinite(args.max_age_ms) or args.max_age_ms <= 0
            or not np.isfinite(args.near_min_mm) or args.near_min_mm < 0
            or (args.duration is not None and (not np.isfinite(args.duration) or args.duration <= 0))):
        parser.error("置信度、线程数、确认帧数、超时和运行时长必须有效。")
    cv2.setNumThreads(1)
    calibration, provenance = load_bench(args.check_report, args.mapping)
    if not args.strict_range and args.near_min_mm > calibration.minimum:
        parser.error("--near-min-mm不能大于标定的最近前向距离。")
    if args.image and args.image.name.endswith("_undistorted.png"):
        parser.error("--image应传*_raw.png；脚本会校正贴地点，不能重复去畸变。")
    directory = args.output/datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    directory.mkdir(parents=True, exist_ok=False)
    detector = (ColorDetector() if args.detector == "color" else
                YoloDetector(args.weights, args.confidence, args.device, args.threads,
                             directory/"ultralytics_config", args.expected_weights_sha256))
    session = GreenSession(calibration, args.confirm_frames, args.max_age_ms,
                           near_min_mm=None if args.strict_range else args.near_min_mm)
    (directory/"session.json").write_text(json.dumps({
        **provenance, "scope": "bench_only", "motion_valid": False,
        "image_size": calibration.size, "detector": detector.metadata,
        "coordinate_convention": "x right-positive, y forward-positive, mm",
        "point_definition": "visible camera-facing bottom-edge midpoint",
        "max_age_ms": args.max_age_ms, "confirm_frames": args.confirm_frames,
        "near_estimate_min_forward_mm": session.near_min_mm,
        "near_estimate_left_bounds_mm": session.near_left_bounds,
    }, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print("结果目录：", directory, flush=True)
    print("坐标原点为胶带交点；X向右、Y向前，单位毫米。S保存，Q退出。", flush=True)
    print("仅使用相机与场地保持本次标定姿态的640x480画面。", flush=True)
    if not args.strict_range and session.near_left_bounds is not None:
        print(f"近处{args.near_min_mm:g}～{calibration.minimum:g}mm显示橙色NEAR_ESTIMATE；"
              "此区域尚未标定。--strict-range可关闭。", flush=True)
    window = "C100 green distance"

    def evaluate(image, captured_at=None, sequence=0):
        started = time.monotonic()
        detections = detector(image)
        done = time.monotonic()
        age = 0. if captured_at is None else (done-captured_at)*1000
        result = session.process(image, detections, age, temporal=captured_at is not None)
        result.update(sequence=sequence, captured_monotonic=captured_at,
                      processed_monotonic=done, inference_ms=(done-started)*1000,
                      timestamp=datetime.now().astimezone().isoformat(),
                      detections=[{"name":d.name, "confidence":d.confidence, "box":d.box}
                                  for d in detections])
        return detections, result

    if args.image:
        image = cv2.imread(str(args.image))
        if image is None:
            raise ValueError(f"无法读取图片：{args.image}")
        detections, result = evaluate(image)
        result["image_source"] = str(args.image.resolve())
        result["image_sha256"] = sha256(args.image)
        preview = render(image, detections, result, detector.metadata["backend"])
        save_snapshot(directory, image, preview, result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
        if not args.no_gui:
            try:
                cv2.imshow(window, preview)
                cv2.waitKey(0)
            finally:
                cv2.destroyAllWindows()
        return 0

    # Load and warm up the model before opening the camera.
    if args.detector == "yolo":
        detector(np.zeros((calibration.size[1], calibration.size[0], 3), np.uint8))
    source = int(args.camera) if args.camera.isdigit() else args.camera
    api = {"auto": cv2.CAP_ANY, "dshow": cv2.CAP_DSHOW,
           "msmf": cv2.CAP_MSMF, "v4l2": cv2.CAP_V4L2}[args.capture_backend]
    capture = cv2.VideoCapture(source, api)
    if not capture.isOpened():
        capture.release()
        raise ValueError("无法打开相机。请关闭hello10.py、取图脚本等占用摄像头的程序。")
    mailbox, stop, faults = LatestFrame(), threading.Event(), []
    for prop, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")),
                        (cv2.CAP_PROP_FRAME_WIDTH, calibration.size[0]),
                        (cv2.CAP_PROP_FRAME_HEIGHT, calibration.size[1]),
                        (cv2.CAP_PROP_FPS, 30), (cv2.CAP_PROP_BUFFERSIZE, 1)):
        capture.set(prop, value)

    def acquire():
        sequence = 0
        try:
            while not stop.is_set():
                ok, image = capture.read()
                stamp = time.monotonic()
                if stop.is_set():
                    return
                if not ok or image is None:
                    faults.append("CAMERA_FAULT: 摄像头未返回新画面")
                    return
                if image.shape[1::-1] != calibration.size:
                    faults.append(f"RESOLUTION_MISMATCH: 实际{image.shape[1::-1]}，标定{calibration.size}")
                    return
                sequence += 1
                mailbox.publish(sequence, stamp, image)
        except Exception as error:
            faults.append(f"CAMERA_FAULT: {error}")
        finally:
            mailbox.close()

    worker = threading.Thread(target=acquire, daemon=True)
    worker.start()
    start = last_result = time.monotonic()
    sequence, count, printed = -1, 0, 0.
    try:
        with (directory/"observations.jsonl").open("w", encoding="utf-8") as log:
            while args.duration is None or time.monotonic()-start < args.duration:
                if faults:
                    raise RuntimeError(faults[0])
                item = mailbox.next(sequence, timeout=.05)
                if item is None:
                    if time.monotonic()-last_result > 1:
                        raise RuntimeError("CAMERA_TIMEOUT: 超过1秒没有新画面")
                    if not args.no_gui and cv2.waitKey(1) & 255 in (ord("q"), 27):
                        break
                    continue
                sequence, stamp, image = item
                detections, result = evaluate(image, stamp, sequence)
                # Do not display a result if the camera failed while inference ran.
                if faults:
                    raise RuntimeError(faults[0])
                last_result = time.monotonic()
                count += 1
                fps = count/max(last_result-start, .001)
                log.write(json.dumps(result, ensure_ascii=False)+"\n")
                log.flush()
                if last_result-printed >= .5:
                    print(json.dumps({k:result[k] for k in ("status","position_mm","age_ms")},
                                     ensure_ascii=False), flush=True)
                    printed = last_result
                if not args.no_gui:
                    preview = render(image, detections, result, detector.metadata["backend"], fps)
                    cv2.imshow(window, preview)
                    key = cv2.waitKey(1) & 255
                    if key in (ord("q"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                        break
                    if key == ord("s"):
                        save_snapshot(directory, image, preview, result)
    except Exception as error:
        session.reset()
        fault = {"scope":"bench_only", "motion_valid":False, "status":"FAULT",
                 "position_mm":None, "error":str(error)}
        (directory/"fault.json").write_text(json.dumps(fault, ensure_ascii=False, indent=2),
                                           encoding="utf-8")
        raise
    finally:
        stop.set()
        mailbox.close()
        # Let the reader release its read before closing a handle from another thread.
        worker.join(timeout=1)
        capture.release()
        cv2.destroyAllWindows()
    print(f"本次处理{count}帧，记录保存在 {directory}", flush=True)
    return 0

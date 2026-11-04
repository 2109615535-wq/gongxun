"""Desktop/LAN dual-camera service. Start capture only with --cameras."""
import argparse
from collections import deque
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit

import cv2

from .bench_green import YoloDetector
from .board_detector import ExistingUltralyticsDetector, apply_uvc_controls, verify_uvc_device
from .dashboard_state import DashboardState
from .dual import CameraPerception, ROLES, load_camera_calibration, load_dual_config
from .runtime import LatestFrame
# as给导入函数起别名，encode_simple明确表示这里使用的是简易协议编码器。
from .simple_protocol import encode as encode_simple, observations_from_snapshot
from .stm32_status import parse_motor_status


ROOT = Path(__file__).resolve().parents[1]
STATIC = Path(__file__).with_name("web")


def apply_cpu_affinity(cores):
    if not cores:
        return
    if not hasattr(os, "sched_setaffinity") or not set(cores) <= os.sched_getaffinity(0):
        raise ValueError("Configured CPU cores are unavailable on this host")
    os.sched_setaffinity(0, set(cores))


def annotate_frame(image, result, roi):
    image = image.copy()
    for row in result["detections"]:
        x1, y1, x2, y2 = map(round, row["box"])
        color = (60, 70, 245) if row["name"] == "hazard" else (120, 220, 70)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, f"{row['name']} {row['confidence']:.2f}", (x1, max(16, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .42, color, 1)
    if roi:
        x1, y1, x2, y2 = map(round, roi)
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 180, 60), 2)
        cv2.putText(image, "GRIP ROI", (x1, max(16, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 180, 60), 1)
    for box in result["gripper"].get("unknown_regions", []):
        x1, y1, x2, y2 = box
        cv2.rectangle(image, (x1, y1), (x2, y2), (220, 100, 255), 2)
        cv2.putText(image, "UNCLASSIFIED", (x1, max(16, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, .4, (220, 100, 255), 1)
    cv2.putText(image, f"{result['role'].upper()} | {result['status']}", (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    return image


class DualRuntime:
    def __init__(self, state, record_path=None):
        self.state, self.config = state, state.config
        self.stop = threading.Event()
        self.slots = {role: LatestFrame() for role in ROLES}
        self.threads = []
        self.sessions = {}
        self.record_path = record_path
        self.log = None
        self.log_lock = threading.Lock()
        self.processing_times = {role: deque(maxlen=60) for role in ROLES}

    def start(self):
        for role in ROLES:
            try:
                calibration, quality, provenance = load_camera_calibration(self.config["cameras"][role])
                self.sessions[role] = CameraPerception(role, calibration, quality, self.config)
                self.state.camera_info[role] = {"size": calibration.size, "quality": quality,
                                                "forward_range_mm": [calibration.minimum, calibration.maximum],
                                                "source": self.config["cameras"][role]["source"],
                                                "kind": self.config["cameras"][role].get("kind", "camera"),
                                                "provenance": provenance}
            except Exception as exc:
                self.state.camera_fault(role, "CALIBRATION_ERROR", str(exc))
                continue
            self.state.camera_fault(role, "CONNECTING")
            thread = threading.Thread(target=self.capture, args=(role,), daemon=True, name=f"capture-{role}")
            self.threads.append(thread)
            thread.start()
            preview = threading.Thread(target=self.preview, args=(role,), daemon=True, name=f"preview-{role}")
            self.threads.append(preview)
            preview.start()
        inference = threading.Thread(target=self.infer, daemon=True, name="dual-inference")
        self.threads.append(inference)
        inference.start()

    def capture(self, role):
        camera = self.config["cameras"][role]
        size = self.sessions[role].calibration.size
        video = camera.get("kind", "camera") == "video"
        api = {"auto": cv2.CAP_ANY, "dshow": cv2.CAP_DSHOW,
               "msmf": cv2.CAP_MSMF, "v4l2": cv2.CAP_V4L2}[camera["capture_api"]]
        sequence = 0
        captured_times = deque(maxlen=60)
        while not self.stop.is_set():
            cap = cv2.VideoCapture()
            try:
                source = camera["source"]
                if not video and camera.get("require_uvc", False):
                    if isinstance(source, int):
                        source = f"/dev/video{source}"
                    source = verify_uvc_device(source)
                if not cap.open(source, cv2.CAP_ANY if video else api):
                    raise OSError(f"无法打开{role}视频源 {camera['source']}")
                captured_times.clear()
                if not video:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
                    cap.set(cv2.CAP_PROP_FPS, self.config["fps"])
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, self.config.get("capture_buffers", 2))
                    actual_controls = apply_uvc_controls(source, camera.get("controls", {}))
                    with self.state.lock:
                        self.state.camera_info[role]["controls"] = actual_controls
                        self.state.camera_info[role]["capture_buffers"] = cap.get(cv2.CAP_PROP_BUFFERSIZE)
                        self.state.camera_info[role]["reported_fps"] = cap.get(cv2.CAP_PROP_FPS)
                replay_fps = cap.get(cv2.CAP_PROP_FPS)
                if not 0 < replay_fps <= 240:
                    replay_fps = self.config["fps"]
                next_frame = time.monotonic()
                while not self.stop.is_set():
                    ok, image = cap.read()
                    captured = time.monotonic()
                    if not ok:
                        raise EOFError("回放结束" if video else "摄像头读取失败，正在重连")
                    if image.shape[1::-1] != size:
                        raise ValueError(f"视频尺寸{image.shape[1::-1]}与标定{size}不一致")
                    sequence += 1
                    self.slots[role].publish(sequence, captured, image)
                    self.state.publish_raw(role, image, captured, sequence)
                    captured_times.append(captured)
                    if len(captured_times) > 1:
                        with self.state.lock:
                            self.state.camera_info[role]["capture_hz"] = (len(captured_times) - 1) / (captured_times[-1] - captured_times[0])
                    if video:
                        next_frame += 1 / replay_fps
                        self.stop.wait(max(0, next_frame - time.monotonic()))
            except Exception as exc:
                # Clear unread frames so a failed source cannot be revived by delayed inference.
                with self.slots[role].condition:
                    self.slots[role].value = None
                    self.state.camera_fault(role, "REPLAY_ENDED" if video and isinstance(exc, EOFError)
                                            else "CAMERA_FAULT", str(exc))
                if video:
                    return
            finally:
                cap.release()
            self.stop.wait(1)

    def preview(self, role):
        sequence = -1
        next_preview = time.monotonic()
        times = deque(maxlen=60)
        while not self.stop.is_set():
            if self.stop.wait(max(0, next_preview - time.monotonic())):
                break
            item = self.slots[role].next(sequence, timeout=.05)
            if item is None:
                continue
            sequence, captured, image = item
            now = time.monotonic()
            next_preview = now + 1 / self.config.get("preview_fps", 20)
            if now - captured > self.config["max_frame_age_ms"] / 1000:
                continue
            with self.state.lock:
                observation = deepcopy(self.state.cameras[role])
                gripper = deepcopy(self.config["gripper"])
                revision = self.state.gripper_revision
            observed_at = observation.get("captured_monotonic")
            if observed_at is None or now - observed_at > self.config["max_frame_age_ms"] / 1000:
                observation["detections"] = []
                observation["gripper"] = {}
            if observation.get("gripper_revision", 0) != revision:
                observation["gripper"] = {}
            roi = gripper["roi_raw_xyxy"] if role == "lower" else None
            quality = [cv2.IMWRITE_JPEG_QUALITY, self.config.get("jpeg_quality", 70)]
            annotated = annotate_frame(image, observation, roi)
            annotated_ok, annotated_jpeg = cv2.imencode(".jpg", annotated, quality)
            with self.slots[role].condition:
                if self.slots[role].value is None:
                    continue
                with self.state.lock:
                    if annotated_ok:
                        self.state.publish_frame(role + "_annotated", annotated_jpeg.tobytes(), captured, sequence)
                    times.append(now)
                    if len(times) > 1:
                        self.state.camera_info[role]["preview_hz"] = (len(times) - 1) / (times[-1] - times[0])

    def create_detector(self, config):
        if config.get("detector_backend") == "existing_ultralytics_rknn":
            os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / "artifacts/dual_camera/ultralytics_config"))
            detector = ExistingUltralyticsDetector(config)
            return detector, detector.predict, {"backend": "RKNN", "classes": detector.names,
                                               "weights": config["model"], "device": "RK3588 NPU",
                                               "fast_path": config.get("rknn_fast_path", False),
                                               "cpu_affinity": config.get("cpu_affinity", [])}
        detector = YoloDetector(config["model"], config["confidence"], config["device"],
                                config.get("cpu_threads", 2), ROOT / "artifacts/dual_camera/ultralytics_config",
                                config["expected_model_sha256"])
        return detector, detector, detector.metadata

    def infer(self):
        detectors = []
        try:
            with self.state.lock:
                self.state.model = {"status": "LOADING"}
            if self.record_path:
                path = Path(self.record_path)
                path.parent.mkdir(parents=True, exist_ok=True)
                self.log = path.open("x", encoding="utf-8")
            roles = list(self.sessions)
            if not roles:
                raise ValueError("No usable camera calibration")
            if self.config.get("inference_workers", 1) == 2:
                groups = [[role] for role in roles]
            else:
                groups = [roles]
            for group in groups:
                config = dict(self.config)
                if len(groups) > 1:
                    config["rknn_core_mask"] = self.config["cameras"][group[0]].get("rknn_core_mask", config["rknn_core_mask"])
                detector, predict, metadata = self.create_detector(config)
                detectors.append((detector, predict, group))
            with self.state.lock:
                self.state.model = dict(metadata, status="READY", workers=len(detectors))
            if len(detectors) == 1:
                self.infer_frames(*detectors[0])
            else:
                workers = [threading.Thread(target=self.infer_frames, args=worker, daemon=True,
                                            name=f"inference-{worker[2][0]}") for worker in detectors]
                for worker in workers:
                    worker.start()
                for worker in workers:
                    worker.join()
        except Exception as exc:
            with self.state.lock:
                self.state.model = {"status": "ERROR", "error": str(exc)}
            for role in self.sessions:
                self.state.camera_fault(role, "INFERENCE_FAULT", str(exc))
        finally:
            if self.log:
                self.log.close()
                self.log = None
            for detector, _, _ in detectors:
                if isinstance(detector, ExistingUltralyticsDetector):
                    detector.close()

    def infer_frames(self, detector, predict, roles):
        sequences = dict.fromkeys(roles, -1)
        while not self.stop.is_set():
            for role in roles:
                session = self.sessions[role]
                item = self.slots[role].next(sequences[role], timeout=.02)
                if item is None:
                    continue
                sequence, captured, frame = item
                sequences[role] = sequence
                task = self.state.task()
                with self.state.lock:
                    gripper = deepcopy(self.config["gripper"])
                    revision = self.state.gripper_revision
                if session.cfg["gripper"] != gripper:
                    session.cfg["gripper"] = gripper
                    session.reset()
                started = time.monotonic()
                try:
                    detections = predict(frame)
                    result = session.process(frame, detections, captured, time.monotonic(), task)
                    done = time.monotonic()
                    self.processing_times[role].append(done)
                    times = self.processing_times[role]
                    hz = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else None
                    result.update(sequence=sequence, inference_ms=(done - started) * 1000,
                                  processing_hz=hz, **getattr(detector, "last_timing", {}),
                                  mode=self.config["cameras"][role].get("kind", "camera"), gripper_revision=revision)
                    if result["mode"] == "video":
                        result["motion_valid"] = False
                    with self.slots[role].condition:
                        if self.slots[role].value is None:
                            session.reset()
                            continue
                        with self.state.lock:
                            if role == "lower" and revision != self.state.gripper_revision:
                                session.reset()
                                continue
                            self.state.publish_result(role, result)
                    if self.log:
                        with self.log_lock:
                            self.log.write(json.dumps(result, ensure_ascii=False, allow_nan=False) + "\n")
                            self.log.flush()
                except Exception as exc:
                    session.reset()
                    self.state.camera_fault(role, "INFERENCE_FAULT", str(exc))
            self.stop.wait(.001)

    def close(self):
        self.stop.set()
        for slot in self.slots.values():
            slot.close()
        for thread in self.threads:
            thread.join(timeout=2)
        for role in ROLES:
            self.state.camera_fault(role, "STOPPED")


class ControllerLink:
    """负责视觉服务与主控之间的串口连接。

    state保存双摄共享状态，port是串口名称，protocol决定本次使用的协议。
    初学者先看run()如何连接串口，再看send_simple()如何定时发送。
    """
    def __init__(self, state, port, protocol):
        if protocol not in ("motor_debug_ascii", "rescue_json_v1", "vision_simple_v1", "vision_simple_v2"):
            raise ValueError("Unsupported controller protocol")
        self.state, self.port, self.protocol = state, port, protocol
        # Event作为停止开关，thread让串口收发与相机、网页同时运行。
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True, name="controller-link")

    def run(self):
        """连接串口，启动收发流程，出错后每隔1秒重新连接。"""
        import serial
        while not self.stop.is_set():
            try:
                # with在退出该段代码时自动关闭串口，timeout的单位是秒。
                with serial.Serial(self.port, 115200, timeout=.05, write_timeout=.1) as port:
                    # lock保护多个线程共用的状态，防止同时修改时互相干扰。
                    with self.state.lock:
                        self.state.link.update(status="CONNECTED", protocol=self.protocol, error=None)
                    if self.protocol in ("vision_simple_v1", "vision_simple_v2"):
                        self.send_simple(port)
                    else:
                        self.exchange_legacy(port)
            except Exception as exc:
                with self.state.lock:
                    self.state.link.update(status="ERROR", error=str(exc))
                    if self.state.controller.get("session_id"):
                        self.state.controller.update(status="DISCONNECTED", state=None, expected=None)
                self.stop.wait(1)  # 出错后等待1秒重连，停止开关也能提前结束等待。

    def send_simple(self, port):
        """循环执行三步，取得双摄结果、逐路发送、等待0.1秒。"""
        while not self.stop.is_set():
            # 1. 从共享状态取出上视和下视各一条观测。
            observations = observations_from_snapshot(self.state.snapshot())

            # 2. 每条观测编码成一行，交给串口发送。
            for observation in observations:
                data = encode_simple(observation, version=2 if self.protocol == "vision_simple_v2" else 1)
                if port.write(data) != len(data):
                    raise OSError("Incomplete observation write")

            # 3. 等待0.1秒再发送下一组，停止程序时可以立刻结束等待。
            self.stop.wait(.1)

    def exchange_legacy(self, port):
        """处理原有电机遥测和JSON协议的逐行接收与定时发送。"""
        pending = bytearray()
        dropping = False
        next_send = time.monotonic()
        while not self.stop.is_set():
            # 串口一次可能读到半行或多行，用换行符判断一条消息是否收齐。
            for byte in port.read(256):
                if byte in (10, 13):
                    if pending and not dropping:
                        self.receive(bytes(pending))
                    pending.clear()
                    dropping = False
                elif not dropping:
                    if len(pending) >= 2048:
                        pending.clear()
                        dropping = True
                        with self.state.lock:
                            self.state.link["invalid_lines"] += 1
                    else:
                        pending.append(byte)
            if self.protocol == "rescue_json_v1" and time.monotonic() >= next_send:
                payload = json.dumps(self.state.controller_observation(), separators=(",", ":"),
                                     allow_nan=False).encode("ascii") + b"\n"
                if port.write(payload) != len(payload):
                    raise OSError("Incomplete observation write")
                next_send = time.monotonic() + .1

    def receive(self, line):
        """按协议处理一行主控输入，错误行只计数，供网页诊断使用。"""
        try:
            if self.protocol == "rescue_json_v1":
                self.state.update_controller(json.loads(line))
            elif self.protocol in ("vision_simple_v1", "vision_simple_v2"):
                # 本版约定视觉向主控发送观测，收到任务输入时按未支持的消息处理。
                raise ValueError("Simple protocol is observation-only")
            else:
                values = parse_motor_status(line)
                with self.state.lock:
                    self.state.link.update(telemetry=values, telemetry_received_monotonic=time.monotonic(),
                                           speed_unit="normalized_minus1000_to1000", y_body_unit="cm")
        except (ValueError, TypeError, KeyError):
            with self.state.lock:
                self.state.link["invalid_lines"] += 1

    def close(self):
        """通知串口线程退出，并最多等待2秒完成退出。"""
        self.stop.set()
        self.thread.join(timeout=2)


def make_server(state, host, port):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_args):
            pass

        def respond(self, status, data, content_type):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = urlsplit(self.path).path
            try:
                if path == "/api/status":
                    data = json.dumps(state.snapshot(), ensure_ascii=False, allow_nan=False).encode("utf-8")
                    self.respond(200, data, "application/json; charset=utf-8")
                elif path in ("/", "/app.js", "/style.css"):
                    filename, content_type = {"/": ("index.html", "text/html; charset=utf-8"),
                                              "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                                              "/style.css": ("style.css", "text/css; charset=utf-8")}[path]
                    self.respond(200, (STATIC / filename).read_bytes(), content_type)
                elif path in ("/stream/upper.mjpg", "/stream/lower.mjpg"):
                    self.stream(path.split("/")[-1].split(".")[0])
                elif path in ("/snapshot/upper.jpg", "/snapshot/lower.jpg", "/snapshot/lower-raw.jpg"):
                    role = path.split("/")[-1].split(".")[0]
                    raw = state.raw_image("lower") if role == "lower-raw" else None
                    if raw is not None:
                        ok, encoded = cv2.imencode(".jpg", raw[0], [cv2.IMWRITE_JPEG_QUALITY, 95])
                        self.respond(200, encoded.tobytes(), "image/jpeg") if ok else self.respond(503, b"Encoding failed", "text/plain")
                    else:
                        frame = state.frame("lower") if role == "lower-raw" else state.frame(role + "_annotated") or state.frame(role)
                        self.respond(200, frame[0], "image/jpeg") if frame else self.respond(503, b"No fresh frame", "text/plain")
                else:
                    self.respond(404, b"Not found", "text/plain")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, TimeoutError):
                pass

        def stream(self, role):
            self.connection.settimeout(3)
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            last = None
            idle_start = time.monotonic()
            while not self.server.stop.is_set():
                frame = state.frame(role + "_annotated") or state.frame(role)
                if frame and frame[1:] != last:
                    jpeg, stamp, sequence = frame
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                    self.wfile.flush()
                    last = stamp, sequence
                    idle_start = time.monotonic()
                if not frame and time.monotonic() - idle_start > 2:
                    break
                self.server.stop.wait(.01)
            self.close_connection = True

        def do_POST(self):
            # This endpoint selects a labelled desktop test task, never a motor command.
            if self.path not in ("/api/debug-task", "/api/gripper-roi"):
                self.respond(404, b"Not found", "text/plain")
                return
            origin = self.headers.get("Origin")
            if origin and urlsplit(origin).netloc != self.headers.get("Host"):
                self.respond(403, b"Origin rejected", "text/plain")
                return
            try:
                self.connection.settimeout(3)
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 1024:
                    raise ValueError("Invalid body size")
                data = json.loads(self.rfile.read(length))
                if self.path == "/api/gripper-roi":
                    state.set_gripper_roi(data["roi_raw_xyxy"])
                else:
                    state.set_debug_task(data["expected"], data.get("state"))
                self.respond(200, b'{"ok":true}', "application/json")
            except PermissionError as exc:
                self.respond(409, str(exc).encode("utf-8"), "text/plain; charset=utf-8")
            except OSError:
                self.respond(500, "配置保存失败，请检查文件权限".encode("utf-8"), "text/plain; charset=utf-8")
            except (ValueError, KeyError, TypeError, TimeoutError):
                self.close_connection = True
                self.respond(400, b"Invalid debug task", "text/plain")

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    server.stop = threading.Event()
    return server


def main(argv=None):
    parser = argparse.ArgumentParser(description="双摄像头视觉与局域网监看网页")
    parser.add_argument("--config", type=Path, default=ROOT / "config/dual_camera.json")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--cameras", action="store_true", help="开启配置中的两路视频源与YOLO")
    parser.add_argument("--upper-source", help="覆盖上视USB摄像头编号或V4L2路径")
    parser.add_argument("--lower-source", help="覆盖下视USB摄像头编号或V4L2路径")
    parser.add_argument("--record", type=Path, help="新建JSONL检测日志，拒绝覆盖")
    parser.add_argument("--controller-port", help="显式启用串口接入")
    # v2增加angle，v1保持旧格式；电机调试默认仍只收不发。
    parser.add_argument("--controller-protocol", choices=("motor_debug_ascii", "rescue_json_v1", "vision_simple_v1", "vision_simple_v2"), default="motor_debug_ascii")
    parser.add_argument("--duration", type=float, help="限时运行，用于检查")
    args = parser.parse_args(argv)
    if args.duration is not None and not 0 < args.duration < 86400:
        parser.error("duration must be positive and less than one day")
    cfg = load_dual_config(args.config)
    apply_cpu_affinity(cfg["cpu_affinity"])
    for role in ROLES:
        source = getattr(args, role + "_source")
        if source is not None:
            if source.lstrip("-").isdigit():
                source = int(source)
            if isinstance(source, int) and source < 0:
                parser.error("摄像头编号必须非负")
            cfg["cameras"][role].update(source=source, kind="camera")
    if args.cameras and any(cfg["cameras"][role]["source"] is None for role in ROLES):
        parser.error("请配置双摄像头编号，或传入 --upper-source 与 --lower-source")
    if cfg["cameras"]["upper"]["source"] is not None and cfg["cameras"]["upper"]["source"] == cfg["cameras"]["lower"]["source"]:
        parser.error("上下视不能使用相同视频源")
    if args.record and args.record.exists():
        parser.error("日志文件已存在，请选择新文件")
    state = DashboardState(cfg, config_path=args.config.resolve())
    server = make_server(state, args.host, args.port)
    runtime = DualRuntime(state, args.record) if args.cameras else None
    link = ControllerLink(state, args.controller_port, args.controller_protocol) if args.controller_port else None
    timer = None
    try:
        if runtime:
            runtime.start()
        if link:
            link.thread.start()
        if args.duration:
            timer = threading.Timer(args.duration, server.shutdown)
            timer.daemon = True
            timer.start()
        print(f"双视视觉工作台 http://{'127.0.0.1' if args.host == '0.0.0.0' else args.host}:{server.server_port}", flush=True)
        print("视频源已启用" if args.cameras else "网页预览模式，可通过 --cameras 启用视频源", flush=True)
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        if timer:
            timer.cancel()
        server.stop.set()
        if runtime:
            runtime.close()
        if link:
            link.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

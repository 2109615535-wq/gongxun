"""Freshest-frame capture + inference worker + independent communication watchdog."""
from dataclasses import asdict, replace
import json
from pathlib import Path
import threading
import time

import cv2

from .detector import create_detector
from .geometry import GroundCalibration
from .pipeline import Perception
from .protocol import encode
from .types import FrameFlag, Observation, Status


class LatestFrame:
    def __init__(self):
        self.condition = threading.Condition()
        self.value = None
        self.closed = False

    def publish(self, sequence, stamp, image):
        with self.condition:
            self.value = (sequence, stamp, image)
            self.condition.notify_all()

    def next(self, after, timeout=.05):
        with self.condition:
            self.condition.wait_for(lambda: self.closed or
                                    (self.value is not None and self.value[0] != after), timeout)
            if self.value is None or self.value[0] == after:
                return None
            return self.value

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()


def annotate(image, detections, observation):
    image = image.copy()
    for detection in detections:
        x1, y1, x2, y2 = map(int, detection.box)
        color = (0, 0, 255) if detection.name == "hazard" else (0, 220, 0)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, f"{detection.name} {detection.confidence:.2f}",
                    (x1, max(15, y1-5)), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)
    text = f"{observation.status.name} tick={observation.tick_ms} flags={observation.flags}"
    cv2.putText(image, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 255), 1)
    diagnostic = observation.diagnostics
    if "bumper_gap_mm" in diagnostic:
        cv2.putText(image, f"gap={diagnostic['bumper_gap_mm']:.0f}mm lateral={diagnostic['lateral_mm']:.0f}mm",
                    (8, 42), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 1)
    return image


class Recorder:
    def __init__(self, directory, size, video=False):
        self.directory = Path(directory)
        # Refuse overwrite so repeated trials preserve the original evidence.
        self.directory.mkdir(parents=True, exist_ok=False)
        self.log = (self.directory/"observations.jsonl").open("w", encoding="utf-8")
        self.wire = (self.directory/"observations.bin").open("wb")
        self.raw = self.annotated = None
        if video:
            codec = cv2.VideoWriter_fourcc(*"MJPG")
            self.raw = cv2.VideoWriter(str(self.directory/"raw.avi"), codec, 15, size)
            self.annotated = cv2.VideoWriter(str(self.directory/"annotated.avi"), codec, 15, size)
            if not self.raw.isOpened() or not self.annotated.isOpened():
                self.close()
                raise RuntimeError("Cannot open video writers")

    def record(self, observation, packet, **metadata):
        record = asdict(observation)
        record["status"] = observation.status.name
        record.update(metadata)
        self.log.write(json.dumps(record, ensure_ascii=False)+"\n")
        self.log.flush()
        self.wire.write(packet)
        self.wire.flush()

    def frame(self, raw, annotated):
        if self.raw is not None:
            self.raw.write(raw)
            self.annotated.write(annotated)

    def close(self):
        self.log.close()
        self.wire.close()
        for writer in (self.raw, self.annotated):
            if writer is not None:
                writer.release()


def run(config, output, serial_port=None, show=False, video=False, duration=None):
    if serial_port and config.get("serial_protocol") != "vision_v1":
        raise ValueError("Current UART5 is a motor-2 ASCII debug interface, not a visual observation receiver. "
                         "Use stm32-status for read-only status; omit --serial to record perception only.")
    if config.get("require_uvc"):
        from .board_detector import verify_uvc_device
        verify_uvc_device(config["camera"])
    calibration = GroundCalibration.load(config["calibration"])
    if calibration.size != (config["width"], config["height"]):
        raise ValueError("Configured camera resolution differs from calibration")
    detector = create_detector(config)
    pipeline = Perception(calibration, config)
    recorder = Recorder(output, calibration.size, video)
    port = None
    capture = None
    stop = threading.Event()
    mailbox = LatestFrame()
    result_lock = threading.Lock()
    shared = {"capture_stamp": 0., "result": None, "error": None}
    max_age = config["max_frame_age_ms"]/1000

    def capture_loop():
        seq = 0
        try:
            while not stop.is_set():
                ok, image = capture.read()
                stamp = time.monotonic()
                if not ok:
                    with result_lock:
                        shared["error"] = "camera read failed"
                        shared["capture_stamp"] = 0.
                    return
                with result_lock:
                    shared["capture_stamp"] = stamp
                mailbox.publish(seq, stamp, image)
                seq += 1
        except Exception as exc:
            with result_lock:
                shared["error"] = str(exc)
                shared["capture_stamp"] = 0.

    def inference_loop():
        seq, previous_stamp = -1, 0.
        while not stop.is_set():
            item = mailbox.next(seq)
            if item is None:
                continue
            seq, captured, raw = item
            if captured-previous_stamp > .3:
                pipeline.reset()
            previous_stamp = captured
            tick = int(captured*1000) & 0xFFFFFFFF
            detections = []
            image = raw
            try:
                processing_started = time.monotonic()
                space = config.get("detector_pixel_space", "undistorted")
                if raw.shape[1::-1] != calibration.size:
                    raise ValueError("Camera resolution differs from calibration")
                image = raw if space == "raw" else calibration.undistort(raw)
                inference_started = time.monotonic()
                detections = detector.predict(image)
                inference_finished = time.monotonic()
                if time.monotonic()-captured > max_age:
                    observation = pipeline.fault(tick, Status.STALE)
                else:
                    observation = pipeline.process(image, detections, tick, pixel_space=space)
                observation = replace(observation, diagnostics={**observation.diagnostics,
                    "preprocess_ms": (inference_started-processing_started)*1000,
                    "inference_ms": (inference_finished-inference_started)*1000,
                    "geometry_ms": (time.monotonic()-inference_finished)*1000})
            except Exception as exc:
                observation = pipeline.fault(tick, Status.INFERENCE_FAULT)
                with result_lock:
                    shared["error"] = str(exc)
            with result_lock:
                shared["result"] = (seq, captured, raw, image, detections, observation)

    def transmit(observation, sequence, **metadata):
        packet = encode(observation, sequence)
        if port is not None and port.write(packet) != len(packet):
            raise RuntimeError("Partial serial write")
        # Read the clock AFTER bounded serial write, not just after inference.
        sent = time.monotonic()
        captured = metadata.get("capture_monotonic")
        recorder.record(observation, packet, sent_monotonic=sent,
                        capture_to_send_ms=None if captured is None else (sent-captured)*1000,
                        transmitted=port is not None, **metadata)

    workers = []
    started = time.monotonic()
    last_result, sequence, last_sent = -1, 0, 0.
    try:
        if serial_port:
            import serial
            port = serial.Serial(serial_port, config["baudrate"], timeout=.02, write_timeout=.02)
        capture = (cv2.VideoCapture(config["camera"], cv2.CAP_V4L2)
                   if config.get("capture_api") == "v4l2" else cv2.VideoCapture(config["camera"]))
        if not capture.isOpened():
            raise RuntimeError(f"Cannot open camera {config['camera']}")
        if config.get("fourcc"):
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config["fourcc"]))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, config["width"])
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config["height"])
        capture.set(cv2.CAP_PROP_FPS, config["fps"])
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for function in (capture_loop, inference_loop):
            worker = threading.Thread(target=function, daemon=True)
            worker.start()
            workers.append(worker)
        while duration is None or time.monotonic()-started < duration:
            now = time.monotonic()
            with result_lock:
                result, capture_stamp, error = shared["result"], shared["capture_stamp"], shared["error"]
            if result is not None and result[0] != last_result:
                last_result = result[0]
                frame_seq, captured, raw, image, detections, observation = result
                if now-captured > max_age or now-capture_stamp > max_age:
                    observation = Observation(observation.tick_ms, 0, Status.STALE,
                                              diagnostics=observation.diagnostics)
                transmit(observation, sequence, frame_seq=frame_seq, capture_monotonic=captured,
                         detections=[asdict(d) for d in detections], error=error)
                sequence = (sequence+1) & 255
                last_sent = now
                annotated = annotate(image, detections, observation)
                recorder.frame(raw, annotated)
                if show:
                    cv2.imshow("Rescue vision - q to stop", annotated)
                    if cv2.waitKey(1) & 255 == ord("q"):
                        break
            elif now-last_sent >= .05:
                state = Status.CAMERA_FAULT if now-capture_stamp > max_age else Status.STALE
                transmit(Observation(int(now*1000) & 0xFFFFFFFF, 0, state), sequence, error=error)
                sequence = (sequence+1) & 255
                last_sent = now
            stop.wait(.005)
    except KeyboardInterrupt:
        pass
    finally:
        # On normal exit send invalid observation immediately, rather than waiting for receiver timeout.
        try:
            transmit(Observation(int(time.monotonic()*1000) & 0xFFFFFFFF, 0, Status.CAMERA_FAULT), sequence)
        finally:
            stop.set()
            mailbox.close()
            # Never release the NPU while inference is running; a stuck daemon ends with the process.
            for worker in workers:
                worker.join(timeout=.2)
            if not any(worker.is_alive() for worker in workers):
                detector.close()
                if capture is not None:
                    capture.release()
            if port is not None:
                port.close()
            recorder.close()
            if show:
                cv2.destroyAllWindows()

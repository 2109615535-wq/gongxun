"""Record original upper-camera frames without blocking the capture thread."""

from collections import deque
import json
import math
from pathlib import Path
import re
import threading
import time
from uuid import uuid4

import cv2


RECORDING_ID = re.compile(r"[0-9]{19}-[0-9a-f]{8}")


class DatasetRecorder:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.condition = threading.Condition()
        self.frames = deque(maxlen=24)
        self.thread = None
        self.accepting = False
        self.error = None
        self.dropped_frames = 0
        self.receipt_path = self.directory / "last_saved.json"
        self.last_saved = None
        if self.receipt_path.is_file():
            try:
                self.last_saved = json.loads(self.receipt_path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                pass

    def start(self, size, fps):
        with self.condition:
            if self.accepting:
                return
            if self.thread is not None:
                raise PermissionError("上一段录像正在保存")
            if not math.isfinite(fps) or fps <= 0:
                raise ValueError("上视相机帧率无效")
            self.directory.mkdir(parents=True, exist_ok=True)
            recording_id = f"{time.time_ns():019d}-{uuid4().hex[:8]}"
            self.error = None
            self.dropped_frames = 0
            self.accepting = True
            self.thread = threading.Thread(
                target=self.write_video, args=(recording_id, size, fps),
                name="dataset-recording", daemon=True,
            )
            self.thread.start()

    def offer(self, image):
        with self.condition:
            if self.accepting:
                if len(self.frames) == self.frames.maxlen:
                    self.dropped_frames += 1
                else:
                    self.frames.append(image)
                    self.condition.notify()

    def stop(self, reason=None):
        with self.condition:
            if self.accepting:
                self.error = reason
                self.accepting = False
                self.condition.notify()

    def close(self):
        self.stop()
        with self.condition:
            thread = self.thread
        if thread is not None:
            thread.join(timeout=5)

    def write_video(self, recording_id, size, fps):
        video_path = self.video_path(recording_id)
        partial_path = video_path.with_suffix(".partial.mp4")
        failure = None
        try:
            writer = cv2.VideoWriter(str(partial_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
            written = 0
            try:
                if not writer.isOpened():
                    raise OSError("无法打开 MP4 编码器")
                while True:
                    with self.condition:
                        self.condition.wait_for(lambda: self.frames or not self.accepting)
                        if not self.frames:
                            break
                        frame = self.frames.popleft()
                    writer.write(frame)
                    written += 1
            finally:
                writer.release()
            if not written or not partial_path.stat().st_size:
                raise OSError("本段没有录到上视画面")
            partial_path.replace(video_path)
        except (OSError, cv2.error) as exc:
            failure = f"录像保存失败。{exc}"
        finally:
            try:
                partial_path.unlink(missing_ok=True)
            except OSError:
                pass
            with self.condition:
                if failure is not None:
                    self.error = failure
                self.accepting = False
                self.frames.clear()
                self.thread = None

    def video_path(self, recording_id):
        if not isinstance(recording_id, str) or not RECORDING_ID.fullmatch(recording_id):
            raise ValueError("录像编号无效")
        return self.directory / f"{recording_id}.mp4"

    def snapshot(self):
        with self.condition:
            # A video becomes available only after the encoder has closed the file.
            pending = sorted(path.stem for path in self.directory.glob("*.mp4")
                             if RECORDING_ID.fullmatch(path.stem))
            return {
                "recording": self.accepting,
                "finalizing": self.thread is not None and not self.accepting,
                "pending": pending,
                "last_saved": self.last_saved,
                "error": self.error,
                "dropped_frames": self.dropped_frames,
            }

    def acknowledge(self, recording_id, video, images):
        if (not isinstance(video, str) or not re.fullmatch(r"[0-9]{3,}\.mp4", video)
                or type(images) is not int or images < 1):
            raise ValueError("回传结果无效")
        video_path = self.video_path(recording_id)
        receipt = {"id": recording_id, "video": video, "images": images}
        with self.condition:
            if not video_path.is_file() and receipt != self.last_saved:
                raise ValueError("找不到待回传的录像")
            temporary = self.receipt_path.with_suffix(".partial.json")
            temporary.write_text(json.dumps(receipt), encoding="utf-8")
            temporary.replace(self.receipt_path)
            self.last_saved = receipt
            video_path.unlink(missing_ok=True)

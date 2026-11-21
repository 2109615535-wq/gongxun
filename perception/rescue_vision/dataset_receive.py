"""Receive finished camera recordings on Windows through an OpenSSH tunnel."""

import argparse
import json
import math
import os
from pathlib import Path
import random
import re
import shutil
import socket
import subprocess
import time
from urllib.request import ProxyHandler, Request, build_opener

import cv2

from .dataset_recording import RECORDING_ID


DATASET_ROOT = Path(r"F:\Learning coding\deep learning\make_dataset")
RECORDING_ENDPOINT = "/api/dataset-recording"


def next_video_stem(videos, images):
    highest = 0
    for path in videos.iterdir():
        if path.is_file() and path.suffix.lower() == ".mp4" and re.fullmatch(r"[0-9]+", path.stem):
            highest = max(highest, int(path.stem))
    for path in images.iterdir():
        match = re.fullmatch(r"([0-9]+)_[0-9]+", path.stem)
        if match and path.is_file():
            highest = max(highest, int(match[1]))
    return f"{highest + 1:03d}"


def extract_samples(video, output, samples_per_second=3):
    capture = cv2.VideoCapture(str(video))
    count = 0
    frames = []

    def save_samples(amount):
        nonlocal count
        selected = sorted(random.sample(range(len(frames)), min(amount, len(frames))))
        for index in selected:
            ok, jpeg = cv2.imencode(".jpg", frames[index], [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not ok:
                raise OSError("JPEG 编码失败")
            count += 1
            (output / f"{count:06d}.jpg").write_bytes(jpeg.tobytes())

    try:
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not capture.isOpened() or not math.isfinite(fps) or fps <= 0:
            raise OSError(f"无法读取视频 {video}")
        total_frames = capture.get(cv2.CAP_PROP_FRAME_COUNT)
        frame_index = 0
        second = 0
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            frame_second = int(frame_index / fps)
            if frame_second != second:
                save_samples(samples_per_second)
                frames.clear()
                second = frame_second
            frames.append(frame)
            frame_index += 1
        if not frame_index or frame_index < total_frames:
            raise OSError("视频没有完整解码")
        # Scale the last group when the recording ends partway through a second.
        if frames:
            save_samples(min(samples_per_second, math.ceil(len(frames) * samples_per_second / fps)))
        return count
    finally:
        capture.release()


def publish_file(source, destination):
    # A hard link commits the file without overwriting, and makes retries recognizable.
    try:
        os.link(source, destination)
    except FileExistsError:
        if not source.samefile(destination):
            raise


class DatasetReceiver:
    def __init__(self, root):
        self.videos = root / "videos"
        self.images = root / "images2"
        self.staging = self.videos / ".dataset-receive"
        self.staging.mkdir(parents=True, exist_ok=True)
        self.images.mkdir(parents=True, exist_ok=True)
        self.opener = build_opener(ProxyHandler({}))

    def request(self, base_url, endpoint, data=None):
        headers = {"Content-Type": "application/json"} if data is not None else {}
        body = json.dumps(data).encode("utf-8") if data is not None else None
        return self.opener.open(Request(base_url + endpoint, body, headers), timeout=15)

    def receive(self, base_url, recording_id):
        if not isinstance(recording_id, str) or not RECORDING_ID.fullmatch(recording_id):
            raise ValueError("录像编号无效")
        job = self.staging / recording_id
        frames = job / "frames"
        frames.mkdir(parents=True, exist_ok=True)
        source = job / "source.mp4"
        receipt_path = job / "receipt.json"
        if receipt_path.is_file():
            receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        else:
            if not source.is_file():
                partial = job / "source.partial"
                with self.request(base_url, f"{RECORDING_ENDPOINT}/{recording_id}.mp4") as response:
                    expected = int(response.headers["Content-Length"])
                    with partial.open("wb") as destination:
                        shutil.copyfileobj(response, destination, length=1024 * 1024)
                if expected < 1 or partial.stat().st_size != expected:
                    raise OSError("视频传输不完整")
                partial.replace(source)
            for frame in frames.glob("*.jpg"):
                frame.unlink()
            count = extract_samples(source, frames)
            receipt = {"id": recording_id, "video": f"{next_video_stem(self.videos, self.images)}.mp4",
                       "images": count}
            temporary = receipt_path.with_suffix(".partial")
            temporary.write_text(json.dumps(receipt), encoding="utf-8")
            temporary.replace(receipt_path)

        stem = Path(receipt["video"]).stem
        for index in range(1, receipt["images"] + 1):
            frame = f"{index:06d}.jpg"
            publish_file(frames / frame, self.images / f"{stem}_{frame}")
        publish_file(source, self.videos / receipt["video"])
        with self.request(base_url, RECORDING_ENDPOINT + "/saved", receipt):
            pass
        for frame in frames.glob("*.jpg"):
            frame.unlink()
        frames.rmdir()
        source.unlink()
        receipt_path.unlink()
        job.rmdir()
        print(f"已保存 {receipt['video']}，抽取 {receipt['images']} 张照片", flush=True)

    def pending(self, base_url):
        with self.request(base_url, "/api/status") as response:
            status = json.load(response).get("dataset_recording")
        return status["pending"] if status else []


def run_receiver(receiver, ssh_target, identity, board_port):
    while True:
        with socket.socket() as socket_for_port:
            socket_for_port.bind(("127.0.0.1", 0))
            local_port = socket_for_port.getsockname()[1]
        command = ["ssh", "-N", "-o", "BatchMode=yes", "-o", "ExitOnForwardFailure=yes",
                   "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=10",
                   "-o", "ServerAliveCountMax=3", "-L",
                   f"127.0.0.1:{local_port}:127.0.0.1:{board_port}"]
        if identity is not None:
            command.extend(("-o", "IdentitiesOnly=yes", "-i", str(identity)))
        with subprocess.Popen([*command, ssh_target]) as tunnel:
            base_url = f"http://127.0.0.1:{local_port}"
            connected = False
            last_error = None
            try:
                while tunnel.poll() is None:
                    try:
                        pending = receiver.pending(base_url)
                        if not connected:
                            print("SSH 已连接，等待上视录像", flush=True)
                            connected = True
                        for recording_id in pending:
                            receiver.receive(base_url, recording_id)
                        last_error = None
                    except (OSError, ValueError, cv2.error) as exc:
                        message = str(exc)
                        if connected and message != last_error:
                            print(f"等待重试。{message}", flush=True)
                        last_error = message
                    time.sleep(1)
            finally:
                if tunnel.poll() is None:
                    tunnel.terminate()
        time.sleep(3)


def main(argv=None):
    import msvcrt

    parser = argparse.ArgumentParser(description="通过 SSH 接收上视录像并给数据集抽帧")
    parser.add_argument("--dataset-root", type=Path, default=DATASET_ROOT)
    parser.add_argument("--ssh-target", default="orangepi@10.163.0.250")
    parser.add_argument("--board-port", type=int, default=8080)
    parser.add_argument("--identity", type=Path, default=Path.home() / ".ssh" / "gongxun_dataset_ed25519")
    args = parser.parse_args(argv)
    receiver = DatasetReceiver(args.dataset_root.resolve())
    with (receiver.staging / "lock").open("a+b") as lock:
        try:
            msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
        except OSError:
            parser.exit(1, "此数据集目录已有接收程序运行\n")
        try:
            identity = args.identity if args.identity.is_file() else None
            run_receiver(receiver, args.ssh_target, identity, args.board_port)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

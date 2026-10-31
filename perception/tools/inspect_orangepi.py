"""Read board metadata without running a detector, streaming a camera or writing UART."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys


def file_info(path):
    if path is None:
        return None
    path = Path(path).expanduser().resolve()
    result = dict(path=str(path))
    try:
        result.update(bytes=path.stat().st_size, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    except OSError as exc:
        result["error"] = str(exc)
    return result


def read_text(path):
    try:
        return Path(path).read_text(errors="replace").replace("\0", "").strip()
    except OSError as exc:
        return f"unavailable: {exc}"


def collect(model=None, script=None, camera=None):
    versions = {}
    for package in ("rknn-toolkit2", "rknn-toolkit-lite2", "opencv-python", "opencv-python-headless", "numpy"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
    result = dict(python=sys.version, machine=platform.machine(), system=platform.system(),
                  board_model=read_text("/proc/device-tree/model"),
                  npu_driver_version=read_text("/sys/kernel/debug/rknpu/version"),
                  package_versions=versions, model=file_info(model), script=file_info(script),
                  video_nodes=sorted(str(p) for p in Path("/dev").glob("video*")),
                  stable_camera_nodes=sorted(str(p) for p in Path("/dev/v4l/by-id").glob("*")),
                  stable_serial_nodes=sorted(str(p) for p in Path("/dev/serial/by-id").glob("*")),
                  npu_model_executed=False, uart_opened=False, camera_stream_started=False,
                  rknn_output_layout="not inspected; confirm from existing script or runtime output shapes")
    if camera:
        if not re.fullmatch(r"/dev/video[0-9]+", str(camera)):
            raise ValueError("--camera must be an explicit /dev/video* device")
        executable = shutil.which("v4l2-ctl")
        if executable:
            command = [executable, "--device", camera, "--get-fmt-video", "--get-parm", "--list-formats-ext", "--list-ctrls"]
            try:
                run = subprocess.run(command, capture_output=True, text=True, timeout=5)
                result["v4l2_query"] = dict(returncode=run.returncode, stdout=run.stdout[-24000:], stderr=run.stderr[-2000:])
            except (OSError, subprocess.TimeoutExpired) as exc:
                result["v4l2_query"] = dict(error=str(exc))
        else:
            result["v4l2_query"] = dict(error="v4l2-ctl not installed; nothing installed automatically")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model")
    parser.add_argument("--script")
    parser.add_argument("--camera")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = collect(args.model, args.script, args.camera)
    text = json.dumps(result, ensure_ascii=False, indent=2)+"\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as sink:
            sink.write(text)
    else:
        print(text)


if __name__ == "__main__":
    main()

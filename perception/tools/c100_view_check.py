"""Preview/save C100 frames without HSV filtering or changing the existing app.

Close hello10.py before opening this tool. Saved PNGs retain the actual capture
resolution; only the preview is resized. Intrinsics are never silently scaled.
"""

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def pixel_from_preview(x, y, preview_size, image_size):
    """Map resized preview pixel centres back to saved-image pixel centres."""
    pw, ph = preview_size
    width, height = image_size
    return [
        float(np.clip((x + 0.5) * width / pw - 0.5, 0, width - 1)),
        float(np.clip((y + 0.5) * height / ph - 0.5, 0, height - 1)),
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="1")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30)
    parser.add_argument("--backend", choices=["auto", "dshow", "msmf"], default="auto")
    parser.add_argument("--fourcc", choices=["MJPG", "YUY2"], default="MJPG")
    parser.add_argument("--calibration", type=Path,
                        default=ROOT / "artifacts/c100_intrinsics_20261029.json")
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/c100_captures")
    args = parser.parse_args()
    if min(args.width, args.height, args.fps) <= 0:
        parser.error("Width, height and requested FPS must be positive")

    source = int(args.camera) if args.camera.isdigit() else args.camera
    api = {"auto": cv2.CAP_ANY, "dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF}[args.backend]
    cap = cv2.VideoCapture(source, api)
    window = "C100 original preview"
    try:
        if not cap.isOpened():
            raise RuntimeError("Cannot open camera. Close hello10.py and other camera apps first.")
        for prop, value in [
            (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc)),
            (cv2.CAP_PROP_FRAME_WIDTH, args.width),
            (cv2.CAP_PROP_FRAME_HEIGHT, args.height),
            (cv2.CAP_PROP_FPS, args.fps),
        ]:
            cap.set(prop, value)

        ok, frame = cap.read()
        if not ok or frame is None:
            raise RuntimeError("Camera opened but no image arrived. Try --backend dshow or msmf.")
        height, width = frame.shape[:2]
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        info = {
            "camera": str(source), "backend": cap.getBackendName(),
            "requested_size": [args.width, args.height], "actual_size": [width, height],
            "requested_fps": args.fps, "driver_reported_fps": cap.get(cv2.CAP_PROP_FPS),
            "fourcc": "".join(chr((fourcc >> (8 * i)) & 255) for i in range(4)),
            "world_coordinates": "not calibrated; image clicks are pixels only",
        }
        print(json.dumps(info, ensure_ascii=False, indent=2))
        if [width, height] != [args.width, args.height]:
            print("WARNING: requested size was NOT accepted. Use actual_size, not the request.")

        matrix = distortion = maps = None
        if args.calibration.is_file():
            data = json.loads(args.calibration.read_text(encoding="utf-8-sig"))
            if data.get("image_size") == [width, height]:
                matrix = np.asarray(data["camera_matrix"], dtype=np.float64)
                distortion = np.asarray(data["dist_coeffs"], dtype=np.float64).reshape(-1)
                maps = cv2.initUndistortRectifyMap(
                    matrix, distortion, None, matrix, (width, height), cv2.CV_32FC1)
                info["intrinsic_source"] = str(args.calibration.resolve())
                info["intrinsic_sha256"] = hashlib.sha256(args.calibration.read_bytes()).hexdigest()
                print("Matching intrinsics loaded. U toggles full-frame undistortion using the same K.")
            else:
                print("Intrinsics resolution does not match. Undistortion is DISABLED; no scaling.")
        else:
            print("No intrinsics file found. Raw preview and original PNG capture remain available.")

        factor = min(1.0, 1280 / width, 800 / height)
        preview_size = (max(1, round(width * factor)), max(1, round(height * factor)))
        state = {"rectified": False}
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)

        def click(event, x, y, flags, userdata):
            if event != cv2.EVENT_LBUTTONDOWN:
                return
            pixel = pixel_from_preview(x, y, preview_size, (width, height))
            record = {"space": "undistorted" if state["rectified"] else "raw", "pixel": pixel}
            if matrix is not None and not state["rectified"]:
                record["undistorted_pixel"] = cv2.undistortPoints(
                    np.array([[pixel]], dtype=np.float64), matrix, distortion, P=matrix
                )[0, 0].tolist()
            print(json.dumps(record, ensure_ascii=False))

        cv2.setMouseCallback(window, click)
        print("S: save original PNG (+ undistorted PNG when available); U: toggle; Q/ESC: exit.")
        print("Left click: print source-image pixel coordinates, not millimetres.")
        print("Preview is scaled to fit. Saved images are NOT scaled and contain no overlays.")
        count, start, measured_fps = 0, time.monotonic(), 0.0
        while True:
            if frame.shape[:2] != (height, width):
                raise RuntimeError("Camera changed resolution during capture; restart the tool.")
            displayed = cv2.remap(frame, *maps, cv2.INTER_LINEAR) if state["rectified"] else frame
            preview = cv2.resize(displayed, preview_size, interpolation=cv2.INTER_AREA)
            count += 1
            elapsed = time.monotonic() - start
            if elapsed >= 1:
                measured_fps, count, start = count / elapsed, 0, time.monotonic()
            mode = "undistorted" if state["rectified"] else "raw"
            cv2.setWindowTitle(window, f"C100 {width}x{height} {mode} | preview loop {measured_fps:.1f} FPS")
            cv2.imshow(window, preview)
            key = cv2.waitKey(1) & 255
            if key in (ord("q"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord("u"):
                if maps is None:
                    print("Cannot undistort: matching calibration for this capture size is required.")
                else:
                    state["rectified"] = not state["rectified"]
            if key == ord("s"):
                args.output.mkdir(parents=True, exist_ok=True)
                stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
                base = args.output / f"{stamp}_{width}x{height}"
                raw_path = Path(str(base) + "_raw.png")
                if not cv2.imwrite(str(raw_path), frame):
                    raise RuntimeError(f"Cannot save {raw_path}")
                saved = dict(info, saved_at=datetime.now().astimezone().isoformat(),
                             raw_image=str(raw_path.resolve()), preview_loop_fps=measured_fps)
                if maps is not None:
                    corrected = cv2.remap(frame, *maps, cv2.INTER_LINEAR)
                    target = Path(str(base) + "_undistorted.png")
                    if not cv2.imwrite(str(target), corrected):
                        raise RuntimeError(f"Cannot save {target}")
                    saved["undistorted_image"] = str(target.resolve())
                Path(str(base) + ".json").write_text(
                    json.dumps(saved, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                print("Saved:", raw_path)
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError("Camera frame lost; no new image is available.")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

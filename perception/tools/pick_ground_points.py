"""Pick bench ground points from a saved undistorted image, never a live frame."""

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rescue_vision.calibrate import fit_ground
from c100_view_check import pixel_from_preview


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path)
    parser.add_argument("--calibration", type=Path,
                        default=ROOT / "artifacts/c100_intrinsics_20261029.json")
    parser.add_argument("--forward-mm", nargs="+", type=float, default=[150, 400, 600, 800])
    parser.add_argument("--side-mm", type=float, default=200)
    parser.add_argument("--center-x-mm", type=float, default=0,
                        help="Column-centre offset in USER x (right-positive), mm.")
    parser.add_argument("--check-mapping", type=Path,
                        help="Check independent points against an existing bench mapping; never refit it.")
    args = parser.parse_args()
    if (len(args.forward_mm) < 2 or args.side_mm <= 0
            or not np.isfinite([*args.forward_mm, args.side_mm, args.center_x_mm]).all()
            or min(args.forward_mm) <= 0 or args.forward_mm != sorted(set(args.forward_mm))):
        parser.error("Use positive, increasing forward distances and a positive side distance.")
    if args.image is None:
        images = list((ROOT / "artifacts/c100_captures").glob("*_undistorted.png"))
        if not images:
            parser.error("No saved undistorted PNG found. Run c100_view_check.py and press S first.")
        args.image = max(images, key=lambda path: path.stat().st_mtime_ns)
    if not args.image.name.endswith("_undistorted.png"):
        parser.error("Use the saved *_undistorted.png, not a screenshot or raw image.")
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error(f"Cannot read {args.image}")
    height, width = image.shape[:2]
    calibration = json.loads(args.calibration.read_text(encoding="utf-8-sig"))
    if calibration["image_size"] != [width, height]:
        parser.error("Image size and intrinsics do not match.")
    # Check the capture's sidecar to avoid rectified images from a different K.
    base = args.image.name[:-len("_undistorted.png")]
    metadata_path = args.image.with_name(base + ".json")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    intrinsic_hash = hashlib.sha256(args.calibration.read_bytes()).hexdigest()
    if (metadata.get("intrinsic_sha256") != intrinsic_hash
            or metadata.get("actual_size") != [width, height]
            or Path(metadata["undistorted_image"]).resolve() != args.image.resolve()):
        parser.error("Capture metadata does not match this image and calibration.")
    check_mapping = None
    if args.check_mapping:
        check_mapping = json.loads(args.check_mapping.read_text(encoding="utf-8"))
        if (check_mapping.get("image_size") != [width, height]
                or check_mapping.get("ground_homography") is None
                or not np.array_equal(check_mapping.get("camera_matrix"), calibration.get("camera_matrix"))
                or not np.array_equal(check_mapping.get("dist_coeffs"), calibration.get("dist_coeffs"))):
            parser.error("Check mapping size or intrinsics do not match this capture.")
        known_ground = {tuple(point["ground_mm"]) for point in check_mapping["ground_fit_points"]}
        checks_ground = {(y, -(args.center_x_mm + x)) for y in args.forward_mm
                         for x in (-args.side_mm, args.side_mm)}
        if known_ground & checks_ground:
            parser.error("Independent checks must use new measured ground positions, not fitted positions.")

    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(2)
    except (AttributeError, OSError):
        pass
    sides = (-args.side_mm, args.side_mm) if check_mapping else (-args.side_mm, 0, args.side_mm)
    targets = [(args.center_x_mm + x, y) for y in args.forward_mm for x in sides]
    selected = []
    scale = min(2.0, 1280 / width, 960 / height)
    size = (round(width * scale), round(height * scale))
    window = "Ground point picker"
    print("Image:", args.image)
    print("Ground convention: user x=right, y=forward, mm; engine [forward,left]=[y,-x].")
    print("Column x positions, mm:", [args.center_x_mm + x for x in sides])
    print("Click: near to far; at each distance " +
          ("LEFT, RIGHT." if check_mapping else "LEFT, CENTRE, RIGHT.") +
          " Match physical coordinates.")
    if check_mapping:
        print("INDEPENDENT BENCH CHECK: keep the camera and original ground reference unchanged.")
        print("The mapping will not be refitted, validated for vehicle motion, or installed.")
    print("Backspace/Z/right click: undo. R: reset. S: save after all points. Q/ESC: exit without save.")
    print("Pixel enlargement aids clicking only; it does not increase source resolution.")

    def undo():
        if selected:
            selected.pop()

    def click(event, x, y, flags, userdata):
        if event == cv2.EVENT_RBUTTONDOWN:
            undo()
        elif event == cv2.EVENT_LBUTTONDOWN and y < size[1] and len(selected) < len(targets):
            pixel = pixel_from_preview(x, y, size, (width, height))
            user_x, user_y = targets[len(selected)]
            selected.append({"pixel": pixel, "ground_mm": [user_y, -user_x],
                             "user_xy_mm": [user_x, user_y]})
            print(f"P{len(selected):02d}: pixel={pixel}; user_xy_mm={[user_x, user_y]}")

    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window, click)
    try:
        while True:
            canvas = np.full((size[1] + 65, size[0], 3), 35, np.uint8)
            canvas[:size[1]] = cv2.resize(image, size, interpolation=cv2.INTER_NEAREST)
            for i, point in enumerate(selected):
                u, v = point["pixel"]
                position = (round((u + .5) * size[0] / width - .5),
                            round((v + .5) * size[1] / height - .5))
                cv2.drawMarker(canvas, position, (0, 0, 255), cv2.MARKER_CROSS, 13, 1)
                cv2.putText(canvas, str(i + 1), (position[0] + 7, position[1] - 7),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 1)
            if len(selected) < len(targets):
                x, y = targets[len(selected)]
                labels = ("LEFT", "RIGHT") if check_mapping else ("LEFT", "CENTRE", "RIGHT")
                side = labels[len(selected) % len(sides)]
                text = f"NEXT P{len(selected)+1:02d}/{len(targets)}: {side}, user x={x:g}, y={y:g} mm"
            else:
                text = f"All {len(targets)} points selected. Check numbering, then press S to save."
            cv2.putText(canvas, text, (8, size[1] + 24), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
            cv2.putText(canvas, "Z/right-click: undo | R: reset | S: save | Q: quit",
                        (8, size[1] + 50), cv2.FONT_HERSHEY_SIMPLEX, .5, (230, 230, 230), 1)
            cv2.imshow(window, canvas)
            key = cv2.waitKey(40) & 255
            if key in (ord("q"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (8, ord("z")):
                undo()
            elif key == ord("r"):
                selected.clear()
            elif key == ord("s"):
                if len(selected) != len(targets):
                    print(f"Not saved: {len(selected)}/{len(targets)} points selected.")
                    continue
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                folder = ROOT / "artifacts/ground_experiment"
                folder.mkdir(parents=True, exist_ok=True)
                prefix = folder / stamp
                points_path = Path(str(prefix) + "_points.json")
                mapping_path = Path(str(prefix) + "_unvalidated.json")
                data = {
                    "image_size": [width, height], "pixel_space": "undistorted_same_K",
                    "image_source": str(args.image.resolve()),
                    "image_sha256": hashlib.sha256(args.image.read_bytes()).hexdigest(),
                    "intrinsic_sha256": intrinsic_hash,
                    "bumper_forward_mm": 0, "collection_center_left_mm": 0,
                    "forward_range_mm": [min(args.forward_mm), max(args.forward_mm)],
                    "points": selected,
                    "user_grid": {"center_x_mm": args.center_x_mm, "side_mm": args.side_mm,
                                  "forward_mm": args.forward_mm},
                    "note": "离车实验：原点模拟车头；用户x向右/y向前，工程坐标为[y,-x]。尚未独立验证。",
                }
                if check_mapping:
                    from rescue_vision.geometry import GroundCalibration
                    mapping = GroundCalibration(dict(check_mapping, validated=False))
                    results = []
                    for i, point in enumerate(selected):
                        predicted = mapping.project(point["pixel"], require_valid=False)
                        actual = point["ground_mm"]
                        record = dict(point_id=i + 1, **point, inside_valid_region=predicted is not None)
                        if predicted is not None:
                            error = np.array(predicted) - actual
                            record.update(
                                predicted_user_xy_mm=[-predicted[1], predicted[0]],
                                signed_error_user_xy_mm=[-float(error[1]), float(error[0])],
                                within_error_limits=bool(abs(error[0]) <= 30 and abs(error[1]) <= 20))
                        else:
                            record["within_error_limits"] = False
                        results.append(record)
                    valid_errors = [r["signed_error_user_xy_mm"] for r in results
                                    if r["inside_valid_region"]]
                    report = {
                        "scope": "independent_bench_check_only", "validated": False,
                        "mapping_source": str(args.check_mapping.resolve()),
                        "mapping_sha256": hashlib.sha256(args.check_mapping.read_bytes()).hexdigest(),
                        "image_source": data["image_source"], "image_sha256": data["image_sha256"],
                        "error_limits_user_xy_mm": [20, 30],
                        "max_abs_error_user_xy_mm": (
                            np.abs(valid_errors).max(axis=0).tolist() if valid_errors else None),
                        "bench_check_passed": len(results) >= 6 and all(r["within_error_limits"] for r in results),
                        "points": results,
                        "note": "仅检查当前离车实验区域；没有完成原实车>=1m验收，不修改运动标定状态。",
                    }
                    points_path = Path(str(prefix) + "_check_points.json")
                    report_path = Path(str(prefix) + "_check_report.json")
                    data["note"] = "独立检查点，不用于重新拟合映射；用户x向右/y向前。"
                    points_path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                    print("Saved independent check:", report_path)
                    print("Bench check passed:", report["bench_check_passed"])
                    print("Max absolute error [user x/right, user y/forward], mm:",
                          report["max_abs_error_user_xy_mm"])
                    print("This report does NOT validate vehicle calibration or enable motion.")
                    break
                points_path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                fitted = fit_ground(args.calibration, points_path, mapping_path)
                fitted.update(
                    calibration_scope="bench_only", ground_image_source=data["image_source"],
                    ground_image_sha256=data["image_sha256"], intrinsic_config_sha256=intrinsic_hash)
                fitted["note"] = "离车地面映射，仅完成拟合；独立点验收未完成，禁止据此启用运动。"
                mapping_path.write_text(json.dumps(fitted, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                pixels = np.array([p["pixel"] for p in selected], dtype=np.float64).reshape(-1, 1, 2)
                predicted = cv2.perspectiveTransform(pixels, np.array(fitted["ground_homography"])).reshape(-1, 2)
                residual = np.abs(predicted - np.array([p["ground_mm"] for p in selected]))
                print("Saved points:", points_path)
                print("Saved UNVALIDATED bench mapping:", mapping_path)
                print("Fit-point max residual [forward,left], mm:", residual.max(axis=0).tolist())
                print("Fit residual is NOT independent validation accuracy. Active calibration remains unchanged.")
                break
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

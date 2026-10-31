import json
import hashlib
from pathlib import Path

import cv2
import numpy as np

from .geometry import GroundCalibration
from .model_tools import write_json


def import_intrinsics(source_path, output, image_size=None):
    """Import actual OpenCV YAML/XML or native JSON; never validate ground geometry."""
    source_path, output = Path(source_path), Path(output)
    if source_path.resolve() == output.resolve():
        raise ValueError("Import output must differ from the source calibration file")
    metadata_size = None
    if source_path.suffix.lower() == ".json":
        data = json.loads(source_path.read_text(encoding="utf-8-sig"))
        if data.get("reference_only"):
            raise ValueError("Vendor reference values are not a measured camera calibration")
        matrix = data.get("camera_matrix")
        distortion = data.get("dist_coeffs", data.get("distortion_coefficients"))
        metadata_size = data.get("image_size")
        if metadata_size is None and "image_width" in data and "image_height" in data:
            metadata_size = [data["image_width"], data["image_height"]]
    else:
        storage = cv2.FileStorage(str(source_path), cv2.FILE_STORAGE_READ)
        try:
            if not storage.isOpened():
                raise ValueError("Cannot read OpenCV calibration file")
            matrix = storage.getNode("camera_matrix").mat()
            distortion = None
            for name in ("dist_coeffs", "distortion_coefficients", "distortion_coeffs"):
                node = storage.getNode(name)
                if not node.empty():
                    distortion = node.mat()
                    break
            width, height = storage.getNode("image_width"), storage.getNode("image_height")
            if not width.empty() and not height.empty():
                metadata_size = [width.real(), height.real()]
        finally:
            storage.release()
    if image_size is not None and metadata_size is not None and list(image_size) != list(metadata_size):
        raise ValueError("Requested resolution differs from calibration metadata; do not silently scale intrinsics")
    size = metadata_size if metadata_size is not None else image_size
    if size is None or len(size) != 2 or any(not np.isfinite(x) or x <= 0 or x != int(x) for x in size):
        raise ValueError("Calibration resolution is required; supply the ACTUAL --image-size WIDTH HEIGHT")
    if matrix is None or distortion is None:
        raise ValueError("Both measured camera_matrix and distortion coefficients are required")
    matrix, distortion = np.asarray(matrix, np.float64), np.asarray(distortion, np.float64).reshape(-1)
    if (matrix.shape != (3, 3) or distortion.size not in (4, 5, 8, 12, 14)
            or not np.isfinite(matrix).all() or not np.isfinite(distortion).all()
            or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0, 0, 1])):
        raise ValueError("Invalid measured camera intrinsics")
    result = dict(validated=False, image_size=list(map(int, size)), camera_matrix=matrix.tolist(),
                  dist_coeffs=distortion.tolist(), ground_homography=None, valid_pixel_polygon=[],
                  forward_range_mm=[150, 1400], bumper_forward_mm=0,
                  collection_center_left_mm=0, intrinsic_source=str(source_path.resolve()),
                  intrinsic_source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
                  note="导入相机内参与畸变；本车安装外参、地面映射和独立验证仍待完成。")
    write_json(output, result)
    return result


def calibrate_intrinsics(images, pattern, square_mm, output):
    if len(images) < 12 or min(pattern) < 3 or square_mm <= 0:
        raise ValueError("Use >=12 chessboard images, >=3x3 inner corners and a measured square size")
    grid = np.zeros((pattern[0]*pattern[1], 3), np.float32)
    grid[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)*square_mm
    objects, corners, used, size = [], [], [], None
    for path in images:
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f"Cannot read {path}")
        current = image.shape[1::-1]
        if size is not None and current != size:
            raise ValueError("All calibration images must have identical resolution")
        size = current
        ok, pts = cv2.findChessboardCornersSB(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), pattern)
        if ok:
            objects.append(grid.copy())
            corners.append(pts)
            used.append(str(path))
    if len(used) < 12:
        raise ValueError(f"Only {len(used)} chessboards detected; need >=12 with varied positions/tilts")
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(objects, corners, size, None, None)
    if not np.isfinite(rms) or rms > .7:
        raise ValueError(f"Intrinsic calibration RMS {rms:.3f}px exceeds .7px; reshoot images")
    result = dict(validated=False, image_size=list(size), camera_matrix=matrix.tolist(),
                  dist_coeffs=distortion.tolist(), ground_homography=None,
                  valid_pixel_polygon=[], forward_range_mm=[150, 1400],
                  bumper_forward_mm=0, collection_center_left_mm=0,
                  intrinsic_rms_px=float(rms), intrinsic_images=used)
    write_json(output, result)
    return result


def fit_ground(calibration_path, points_path, output):
    data = json.loads(Path(calibration_path).read_text(encoding="utf-8"))
    points = json.loads(Path(points_path).read_text(encoding="utf-8"))
    if data.get("camera_matrix") is None or data.get("dist_coeffs") is None:
        raise ValueError("Calibrate camera intrinsics first")
    if data["image_size"] != points["image_size"]:
        raise ValueError("Ground points and intrinsics have different resolutions")
    entries = points["points"]
    if len(entries) < 6:
        raise ValueError("Use at least six measured, non-collinear ground points")
    pixels = np.asarray([p["pixel"] for p in entries], np.float64)
    ground = np.asarray([p["ground_mm"] for p in entries], np.float64)
    if not np.isfinite(pixels).all() or not np.isfinite(ground).all():
        raise ValueError("Nonfinite calibration points")
    matrix, _ = cv2.findHomography(pixels, ground, method=0)
    if matrix is None or abs(cv2.determinant(matrix)) < 1e-12:
        raise ValueError("Degenerate ground points")
    # Pixel hull defines the permitted interpolation region; do not extrapolate.
    hull = cv2.convexHull(pixels.astype(np.float32)).reshape(-1, 2)
    result = dict(data, validated=False, ground_homography=matrix.tolist(),
                  valid_pixel_polygon=hull.tolist(), ground_fit_points=entries,
                  ground_fit_source=str(Path(points_path).resolve()),
                  forward_range_mm=points["forward_range_mm"],
                  bumper_forward_mm=float(points["bumper_forward_mm"]),
                  collection_center_left_mm=float(points.get("collection_center_left_mm", 0)))
    write_json(output, result)
    return result


def validate_ground(calibration_path, checks_path, output):
    data = json.loads(Path(calibration_path).read_text(encoding="utf-8"))
    checks = json.loads(Path(checks_path).read_text(encoding="utf-8"))
    if checks["image_size"] != data["image_size"]:
        raise ValueError("Validation resolution mismatch")
    points = checks["points"]
    if len(points) < 6:
        raise ValueError("Need >=6 independently measured validation points")
    fitted = {tuple(p["pixel"]) for p in data.get("ground_fit_points", [])}
    if any(tuple(p["pixel"]) in fitted for p in points):
        raise ValueError("Validation must not reuse fitted image points")
    calibration = GroundCalibration(dict(data, validated=False))
    errors = []
    for point in points:
        projected = calibration.project(point["pixel"], require_valid=False)
        if projected is None:
            raise ValueError("Validation point is outside permitted region")
        error = np.abs(np.asarray(projected)-point["ground_mm"])
        errors.append(error)
    errors = np.asarray(errors)
    near = [p for p in points if 170 <= p["ground_mm"][0]-calibration.bumper_mm <= 230]
    far = [p for p in points if p["ground_mm"][0]-calibration.bumper_mm >= 1000]
    if len(near) < 2 or not far:
        raise ValueError("Validation must include >=2 stop-zone points and >=1 point at >=1m bumper gap")
    if errors[:, 0].max() > 30 or errors[:, 1].max() > 20:
        raise ValueError(f"Ground validation failed: max forward/lateral errors {errors.max(axis=0).tolist()} mm")
    data.update(validated=True, validation_points=points,
                validation_max_error_mm=errors.max(axis=0).tolist())
    GroundCalibration(data)  # also checks stop-range coverage and array validity
    write_json(output, data)
    return data

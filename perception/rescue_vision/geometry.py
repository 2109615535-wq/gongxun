import json
from pathlib import Path

import cv2
import numpy as np


class GroundCalibration:
    """Homography maps UNDISTORTED pixels to chassis x-forward/y-left in mm."""

    def __init__(self, data: dict):
        self.data = data
        self.size = tuple(data["image_size"])
        self.valid = bool(data.get("validated", False))
        self.bumper_mm = float(data.get("bumper_forward_mm", 0))
        self.center_left_mm = float(data.get("collection_center_left_mm", 0))
        self.minimum, self.maximum = map(float, data.get("forward_range_mm", [0, 0]))
        self.matrix = self._array("camera_matrix")
        self.distortion = self._array("dist_coeffs")
        self.homography = self._array("ground_homography")
        self.polygon = np.asarray(data.get("valid_pixel_polygon", []), np.float32)
        self.maps = None
        if self.valid:
            self._validate_structure()

    def _array(self, name):
        value = self.data.get(name)
        return None if value is None else np.asarray(value, np.float64)

    def _validate_structure(self):
        if (self.matrix is None or self.matrix.shape != (3, 3)
                or self.distortion is None or self.distortion.size < 4
                or self.homography is None or self.homography.shape != (3, 3)
                or self.polygon.ndim != 2 or self.polygon.shape[1] != 2
                or len(self.polygon) < 3 or self.minimum >= self.maximum
                or self.bumper_mm < 0
                or self.matrix[0, 0] <= 0 or self.matrix[1, 1] <= 0
                or not all(np.isfinite(v).all() for v in
                           (self.matrix, self.distortion, self.homography, self.polygon))
                or abs(np.linalg.det(self.homography)) < 1e-12):
            raise ValueError("Malformed validated calibration")
        if not self.minimum <= self.bumper_mm + 170 < self.bumper_mm + 230 <= self.maximum:
            raise ValueError("Calibration must cover the complete 200 +/- 30 mm stop interval")
        inverse = np.linalg.inv(self.homography)
        for x in (self.bumper_mm+170, self.bumper_mm+230):
            for y in (self.center_left_mm-20, self.center_left_mm+20):
                p = inverse @ np.array([x, y, 1.])
                if abs(p[2]) < 1e-9 or cv2.pointPolygonTest(
                        self.polygon, tuple(p[:2]/p[2]), False) < 0:
                    raise ValueError("Validated pixel region does not cover the complete stop interval")

    @classmethod
    def load(cls, path: str | Path):
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    def undistort(self, image: np.ndarray) -> np.ndarray:
        if image.shape[1::-1] != self.size:
            raise ValueError(f"Camera resolution differs from calibration: {image.shape[1::-1]} != {self.size}")
        if self.matrix is None or self.distortion is None:
            return image
        if self.maps is None:
            self.maps = cv2.initUndistortRectifyMap(
                self.matrix, self.distortion, None, self.matrix, self.size, cv2.CV_32FC1)
        return cv2.remap(image, *self.maps, interpolation=cv2.INTER_LINEAR)

    def project(self, pixel, require_valid=True):
        if require_valid and not self.valid:
            return None
        if self.homography is None or len(self.polygon) < 3:
            return None
        point = tuple(map(float, pixel))
        if cv2.pointPolygonTest(self.polygon, point, False) < 0:
            return None
        p = self.homography @ np.array([*point, 1.0])
        if abs(p[2]) < 1e-9:
            return None
        x, y = p[:2] / p[2]
        if not np.isfinite([x, y]).all() or not self.minimum <= x <= self.maximum:
            return None
        return float(x), float(y)

    def project_raw(self, pixel):
        """Existing detector boxes may be in raw camera pixels. Rectify the point once."""
        if not self.valid or self.matrix is None or self.distortion is None:
            return None
        x, y = map(float, pixel)
        if not (0 <= x < self.size[0] and 0 <= y < self.size[1]):
            return None
        rectified = cv2.undistortPoints(np.array([[[x, y]]], np.float64),
                                       self.matrix, self.distortion, P=self.matrix)[0, 0]
        return self.project(rectified)


def touches_border(box, size, margin=4):
    x1, y1, x2, y2 = box
    width, height = size
    return x1 <= margin or y1 <= margin or x2 >= width-margin or y2 >= height-margin


def green_contact(image, box, lower, upper):
    """Bottom of the main green component, never the box centre.

    This is an estimate of visible ground contact, not an occlusion detector.
    Other detected-object overlaps are rejected separately by the pipeline.
    """
    x1, y1, x2, y2 = map(int, box)
    if x2-x1 < 5 or y2-y1 < 5:
        return None
    roi = image[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.asarray(lower, np.uint8), np.asarray(upper, np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if n <= 1:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[k, cv2.CC_STAT_AREA] < max(20, roi.shape[0]*roi.shape[1]*0.12):
        return None
    ys, xs = np.nonzero(labels == k)
    bottom = int(ys.max())
    if bottom < roi.shape[0]*0.75:
        return None
    edge = xs[ys >= bottom-2]
    if len(edge) < 3:
        return None
    return x1 + float(np.median(edge)), y1 + float(bottom)

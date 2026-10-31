import json
from pathlib import Path

import numpy as np
import pytest

from rescue_vision.geometry import GroundCalibration


@pytest.fixture
def config():
    return json.loads((Path(__file__).parents[1]/"config/vision.json").read_text(encoding="utf-8"))


@pytest.fixture
def calibration_data():
    # Synthetic geometry only; NEVER shipped as an actual robot calibration.
    return dict(validated=True, image_size=[640, 480],
                camera_matrix=[[500, 0, 320], [0, 500, 240], [0, 0, 1]],
                dist_coeffs=[0, 0, 0, 0, 0],
                ground_homography=[[0, -3, 1400], [-1, 0, 320], [0, 0, 1]],
                valid_pixel_polygon=[[5, 5], [635, 5], [635, 415], [5, 415]],
                forward_range_mm=[150, 1400], bumper_forward_mm=120,
                collection_center_left_mm=0)


@pytest.fixture
def calibration(calibration_data):
    return GroundCalibration(calibration_data)


@pytest.fixture
def green_image():
    image = np.full((480, 640, 3), 220, np.uint8)
    image[250:290, 280:340] = (0, 200, 0)
    return image

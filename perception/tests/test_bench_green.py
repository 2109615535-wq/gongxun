import hashlib
import json

import numpy as np
import pytest

from rescue_vision.bench_green import GreenSession, load_bench
from rescue_vision.geometry import GroundCalibration
from rescue_vision.types import Detection


@pytest.fixture
def bench(calibration_data):
    return GroundCalibration(dict(calibration_data, validated=False))


@pytest.fixture
def green():
    return Detection("normal_material", .9, (278, 248, 342, 292))


def test_confirm_coordinates_then_clear_on_loss(bench, green_image, green):
    session = GreenSession(bench)
    assert session.process(green_image, [green])["position_mm"] is None
    assert session.process(green_image, [green])["position_mm"] is None
    result = session.process(green_image, [green])
    assert result["status"] == "OK"
    assert result["position_mm"]["x_right"] == pytest.approx(-10.5)
    assert result["position_mm"]["y_forward"] == pytest.approx(533)
    assert result["position_mm"]["ground_range"] > 533
    assert result["motion_valid"] is False
    assert bench.valid is False
    lost = session.process(green_image, [])
    assert lost["position_mm"] is None
    assert session.process(green_image, [green])["status"] == "CONFIRMING"


def test_stale_frame_discards_position_and_confirmation(bench, green_image, green):
    session = GreenSession(bench, confirm_frames=1)
    assert session.process(green_image, [green])["position_mm"]
    stale = session.process(green_image, [green], age_ms=301)
    assert stale["status"] == "STALE"
    assert stale["position_mm"] is None and session.count == 0


@pytest.mark.parametrize("reason", ["border", "overlap", "blue", "outside"])
def test_invalid_contact_never_returns_distance(bench, green_image, green, reason):
    session = GreenSession(bench)
    detections = [green]
    if reason == "border":
        detections = [Detection("normal_material", .9, (0, 248, 342, 292))]
    elif reason == "overlap":
        detections.append(Detection("hazard", .8, (280, 250, 340, 290)))
    elif reason == "blue":
        green_image[250:290, 280:340] = (200, 0, 0)
    else:
        bench.maximum = 500
    result = session.process(green_image, detections, temporal=False)
    assert result["position_mm"] is None
    assert result["status"] in ("BORDER", "OVERLAP", "CONTACT_INVALID", "OUTSIDE_CALIBRATION")


def test_classes_and_resolution_are_not_silently_reinterpreted(bench, green_image):
    session = GreenSession(bench)
    result = session.process(green_image, [
        Detection("core_material", .99, (278, 248, 342, 292))], temporal=False)
    assert result["status"] == "NO_TARGET"
    with pytest.raises(ValueError, match="尺寸"):
        session.process(np.zeros((720, 1280, 3), np.uint8), [])


def test_checked_hash_required_and_vehicle_calibration_not_enabled(tmp_path, calibration_data):
    mapping = tmp_path/"mapping.json"
    report = tmp_path/"check.json"
    mapping.write_text(json.dumps(dict(calibration_data, validated=False)), encoding="utf-8")
    data = {
        "scope": "independent_bench_check_only", "bench_check_passed": True,
        "points": [{"within_error_limits": True} for _ in range(6)],
        "max_abs_error_user_xy_mm": [5, 10], "mapping_source": str(mapping),
        "mapping_sha256": hashlib.sha256(mapping.read_bytes()).hexdigest(),
    }
    report.write_text(json.dumps(data), encoding="utf-8")
    calibration, _ = load_bench(report)
    assert calibration.valid is False
    assert json.loads(mapping.read_text())["validated"] is False
    mapping.write_text(mapping.read_text()+" ", encoding="utf-8")
    with pytest.raises(ValueError, match="不匹配"):
        load_bench(report)
    data["bench_check_passed"] = False
    report.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="独立6点"):
        load_bench(report)


def measured_near_session(bench, **options):
    bench.maximum = 800
    bench.data["ground_fit_points"] = [
        {"ground_mm": [forward, left]}
        for forward in (150, 800) for left in (-200, 200)]
    return GreenSession(bench, **options)


def test_near_target_gets_estimated_coordinates_without_changing_calibration(bench):
    image = np.full((480, 640, 3), 220, np.uint8)
    image[399:420, 280:340] = (0, 200, 0)
    detection = Detection("normal_material", .9, (278, 397, 342, 422))
    session = measured_near_session(bench)
    assert session.process(image, [detection])["status"] == "CONFIRMING"
    assert session.process(image, [detection])["position_mm"] is None
    result = session.process(image, [detection])
    assert result["status"] == "NEAR_ESTIMATE"
    assert result["position_quality"] == "near_estimate"
    assert result["position_mm"]["y_forward"] == pytest.approx(143)
    assert result["position_mm"]["x_right"] == pytest.approx(-10.5)
    assert result["motion_valid"] is False
    assert bench.valid is False and bench.minimum == 150
    assert bench.project((309.5, 419), require_valid=False) is None


@pytest.mark.parametrize("kind", ["strict", "too_close", "far", "lateral", "behind"])
def test_near_mode_keeps_other_boundaries(bench, kind):
    session = measured_near_session(bench, near_min_mm=None if kind == "strict" else 50)
    pixel = (320., 440.)  # forward80, lateral0
    if kind == "too_close":
        pixel = (320., 460.)  # forward20 < requested minimum50
    elif kind == "far":
        pixel = (320., 40.)  # forward1280 > calibrated maximum800
    elif kind == "lateral":
        pixel = (600., 440.)  # 280mm to the right > measured200
    elif kind == "behind":
        pixel = (320., 475.)  # negative forward distance
    assert session.project_contact(pixel) == (None, False)


def test_near_mode_still_rejects_cropped_contact(bench):
    session = measured_near_session(bench)
    image = np.full((480, 640, 3), 220, np.uint8)
    image[450:480, 280:340] = (0, 200, 0)
    result = session.process(image, [
        Detection("normal_material", .9, (278, 448, 342, 480))], temporal=False)
    assert result["status"] == "BORDER"
    assert result["position_mm"] is None

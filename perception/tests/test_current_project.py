import importlib.util
import json
from pathlib import Path
import time

import cv2
import numpy as np
import pytest
import serial

from rescue_vision.calibrate import import_intrinsics
from rescue_vision.geometry import GroundCalibration
from rescue_vision.model_tools import write_json
from rescue_vision.session import DetectionSession, detections_from_arrays
from rescue_vision.stm32_status import monitor_status
from rescue_vision.types import Detection, FrameFlag, Status


def load_tool(name):
    path = Path(__file__).parents[1]/"tools"/f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("key", ["dist_coeffs", "distortion_coefficients", "distortion_coeffs"])
def test_import_measured_opencv_intrinsics(tmp_path, calibration_data, key):
    path, output = tmp_path/"camera_params.yml", tmp_path/"imported.json"
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    storage.write("camera_matrix", np.asarray(calibration_data["camera_matrix"], np.float64))
    storage.write(key, np.zeros((1, 5)))
    storage.write("image_width", 640)
    storage.write("image_height", 480)
    storage.release()
    before = path.read_bytes()
    result = import_intrinsics(path, output)
    assert not result["validated"] and result["ground_homography"] is None
    assert result["image_size"] == [640, 480]
    assert path.read_bytes() == before
    with pytest.raises(ValueError, match="resolution differs"):
        import_intrinsics(path, output, [1280, 720])


def test_import_requires_resolution_and_distortion_and_rejects_reference(tmp_path, calibration_data):
    path, output = tmp_path/"intrinsics.json", tmp_path/"imported.json"
    data = dict(camera_matrix=calibration_data["camera_matrix"], dist_coeffs=[0]*5)
    write_json(path, data)
    with pytest.raises(ValueError, match="resolution is required"):
        import_intrinsics(path, output)
    assert not import_intrinsics(path, output, [640, 480])["validated"]
    write_json(path, dict(data, image_size=[640, 480], reference_only=True))
    with pytest.raises(ValueError, match="reference values"):
        import_intrinsics(path, output)
    write_json(path, dict(data, image_size=[640, 480], dist_coeffs=None))
    with pytest.raises(ValueError, match="Both measured"):
        import_intrinsics(path, output)


def test_raw_contact_is_rectified_once_before_ground_mapping(calibration_data):
    data = dict(calibration_data, dist_coeffs=[.2, .02, 0, 0, 0])
    calibration = GroundCalibration(data)
    point = (550., 300.)
    rectified = cv2.undistortPoints(np.array([[point]], np.float64),
                                   np.asarray(data["camera_matrix"], np.float64),
                                   np.asarray(data["dist_coeffs"]), P=np.asarray(data["camera_matrix"], np.float64))[0, 0]
    assert calibration.project_raw(point) == pytest.approx(calibration.project(rectified))
    assert abs(calibration.project_raw(point)[1]-calibration.project(point)[1]) > 5


def make_session(tmp_path, calibration_data, config):
    path = tmp_path/"calibration.json"
    write_json(path, calibration_data)
    # The nonexistent model path proves the hook doesn't re-load/re-export a model.
    return DetectionSession(dict(config, calibration=str(path), model="does-not-exist.rknn"))


def test_existing_detector_hook_confirms_then_rejects_duplicates(tmp_path, calibration_data, config, green_image):
    session = make_session(tmp_path, calibration_data, config)
    detections = detections_from_arrays([[280, 250, 340, 290]], [.9], [0], {0: "normal_material"})
    for stamp in (1., 1.067, 1.134):
        result = session.observe(green_image, detections, stamp, now_monotonic=stamp+.01)
    assert result.flags & FrameFlag.VISION_VALID
    assert result.diagnostics["pixel_space"] == "raw"
    duplicate = session.observe(green_image, detections, 1.134, now_monotonic=1.144)
    assert duplicate.status == Status.STALE and not duplicate.flags & FrameFlag.VISION_VALID
    late = session.observe(green_image, detections, 1.201, now_monotonic=1.401)
    assert late.status == Status.STALE


def test_existing_detector_rejects_wrong_coordinates_and_class_map(tmp_path, calibration_data, config, green_image):
    session = make_session(tmp_path, calibration_data, config)
    bad = Detection("normal_material", .9, (-10, 250, 340, 290))
    result = session.observe(green_image, [bad], 1., now_monotonic=1.01)
    assert result.status == Status.TARGET_INVALID and "input_error" in result.diagnostics
    with pytest.raises(ValueError, match="ACTUAL"):
        detections_from_arrays([[0, 0, 10, 10]], [.9], [3], ["normal_material"])
    assert detections_from_arrays(None, None, None, []) == []
    with pytest.raises(ValueError):
        detections_from_arrays(None, [.9], [0], ["normal_material"])


def test_current_uart_cannot_receive_binary_observations(monkeypatch, tmp_path, config):
    import rescue_vision.runtime as runtime
    def forbidden(*args):
        pytest.fail("Must reject incompatible UART before loading a model/opening hardware")
    monkeypatch.setattr(runtime, "create_detector", forbidden)
    with pytest.raises(ValueError, match="ASCII debug"):
        runtime.run(config, tmp_path/"recording", serial_port="not-opened")
    assert not (tmp_path/"recording").exists()


@pytest.mark.parametrize("query", [False, True])
def test_status_monitor_sends_nothing_or_get_only(monkeypatch, tmp_path, query):
    data = dict(speed1=5, target1=5, speed2=5, target2=5,
                speed3=5, target3=5, speed4=5, target4=5, y_body=1.25)
    chunks = [b"OK,GET\r\n"+json.dumps(data).encode()[:20], json.dumps(data).encode()[20:]+b"\r\n"]
    writes = []
    class Port:
        def __init__(self, name, baud, **kwargs):
            assert baud == 115200
        def __enter__(self):
            return self
        def __exit__(self, *args):
            pass
        def write(self, value):
            writes.append(value)
            return len(value)
        def read(self, size):
            time.sleep(.002)
            return chunks.pop(0) if chunks else b""
    monkeypatch.setattr(serial, "Serial", Port)
    output = tmp_path/"telemetry.jsonl"
    result = monitor_status("test", .03, output, query)
    assert writes == ([b"GET\n"] if query else [])
    assert result["samples"] == 1 and not result["chassis_control_available"]
    assert json.loads(output.read_text())["target2"] == 5


def test_source_audit_detects_change_without_writing_project(tmp_path):
    tool = load_tool("verify_stm32_readonly")
    project = tmp_path/"project"
    project.mkdir()
    source = project/"main.c"
    source.write_text("int main(void) {return 0;}\n")
    baseline = tmp_path/"baseline.json"
    write_json(baseline, tool.snapshot(project))
    assert tool.compare(project, baseline)["unchanged"]
    source.write_text("int main(void) {return 1;}\n")
    result = tool.compare(project, baseline)
    assert not result["unchanged"] and result["modified"] == ["main.c"]


def test_board_probe_only_queries_device_metadata(monkeypatch, tmp_path):
    tool = load_tool("inspect_orangepi")
    path = tmp_path/"best.rknn"
    path.write_bytes(b"model-not-executed")
    monkeypatch.setattr(tool.shutil, "which", lambda name: "/usr/bin/v4l2-ctl")
    monkeypatch.setattr(tool.platform, "machine", lambda: "aarch64")
    monkeypatch.setattr(tool.platform, "system", lambda: "Linux")
    class Result:
        returncode, stdout, stderr = 0, "Width/Height: 640/480", ""
    def query(command, **kwargs):
        assert not any(arg.startswith("--set") for arg in command)
        assert "--get-fmt-video" in command and "--list-ctrls" in command
        return Result()
    monkeypatch.setattr(tool.subprocess, "run", query)
    result = tool.collect(model=path, camera="/dev/video0")
    assert not result["uart_opened"] and not result["npu_model_executed"]
    assert not result["camera_stream_started"] and "sha256" in result["model"]
    assert path.read_bytes() == b"model-not-executed"

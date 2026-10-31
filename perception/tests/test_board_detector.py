import hashlib
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from rescue_vision.board_detector import ExistingUltralyticsDetector, verify_uvc_device
from rescue_vision.detector import create_detector
from rescue_vision.config import load_config
from rescue_vision.model_tools import write_json


@pytest.fixture
def board_config(tmp_path, config, monkeypatch):
    root = tmp_path/"existing"
    root.mkdir()
    helper = root/"rknn_compat.py"
    helper.write_text("configured=0\ndef configure_rknn_runtime():\n    global configured\n    configured+=1\n", encoding="utf-8")
    directory = root/"best_rknn_model"
    directory.mkdir()
    model = directory/"best-rk3588.rknn"
    model.write_bytes(b"inspected-model-not-run-in-host-tests")
    names = {0: "normal_material", 1: "core_material", 2: "casualty", 3: "hazard"}
    (directory/"metadata.yaml").write_text(yaml.safe_dump(dict(task="detect", imgsz=[640, 640], names=names)), encoding="utf-8")
    monkeypatch.setitem(sys.modules, "rknn_compat", None)
    return dict(config, detector_backend="existing_ultralytics_rknn", model=str(directory),
                existing_project=str(root),
                expected_model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                expected_runtime_helper_sha256=hashlib.sha256(helper.read_bytes()).hexdigest())


def test_existing_backend_initializes_runtime_before_yolo_and_keeps_original_boxes(board_config, monkeypatch, green_image):
    events = []
    released = []
    class Model:
        def __init__(self, path, task):
            assert sys.modules["rknn_compat"].configured == 1
            events.append((path, task))
            self.predictor = SimpleNamespace(model=SimpleNamespace(rknn_model=SimpleNamespace(release=lambda: released.append(True))))
        def predict(self, image, **kwargs):
            assert image is green_image
            assert kwargs["device"] == "cpu" and kwargs["rect"] is False and kwargs["imgsz"] == 640
            events.append(kwargs)
            return [SimpleNamespace(names={0: "normal_material", 1: "core_material", 2: "casualty", 3: "hazard"},
                                    boxes=SimpleNamespace(xyxy=np.array([[280, 250, 340, 290]]),
                                                          conf=np.array([.9]), cls=np.array([0])))]
    monkeypatch.setitem(sys.modules, "ultralytics", SimpleNamespace(YOLO=Model))
    detector = create_detector(board_config)
    result = detector.predict(green_image)
    assert result[0].box == (280, 250, 340, 290) and result[0].name == "normal_material"
    assert len(events) == 2
    detector.close()
    detector.close()
    assert released == [True]


@pytest.mark.parametrize("key", ["expected_model_sha256", "expected_runtime_helper_sha256"])
def test_backend_rejects_changed_model_or_helper_before_loading(board_config, key):
    board_config[key] = "0"*64
    with pytest.raises(ValueError, match="SHA256|runtime selector"):
        ExistingUltralyticsDetector(board_config)
    assert sys.modules["rknn_compat"] is None


@pytest.mark.parametrize("name,index,driver,allowed", [
    ("HJ USB 2.0 Camera", "0", "uvcvideo", True),
    ("stream_hdmirx", "0", "hdmirx", False),
    ("HJ USB 2.0 Camera", "1", "uvcvideo", False)])
def test_camera_identity_excludes_hdmi_and_metadata(tmp_path, monkeypatch, name, index, driver, allowed):
    camera = tmp_path/"video1"
    camera.touch()
    sysclass = tmp_path/"sys"
    entry = sysclass/"video1"
    entry.mkdir(parents=True)
    (entry/"name").write_text(name)
    (entry/"index").write_text(index)
    original = Path.resolve
    def resolve(path, *args, **kwargs):
        if path == entry/"device/driver":
            return tmp_path/driver
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, "resolve", resolve)
    if allowed:
        assert verify_uvc_device(camera, sysclass) == str(camera)
    else:
        with pytest.raises(ValueError, match="not the USB|metadata"):
            verify_uvc_device(camera, sysclass)


def test_board_profile_hash_format_is_valid_and_rejects_extra_character(tmp_path):
    source = Path(__file__).parents[1]/"config/orangepi_existing.json"
    config = load_config(source)
    config["expected_runtime_helper_sha256"] += "1"
    path = tmp_path/"profile.json"
    write_json(path, config)
    with pytest.raises(ValueError, match="exactly 64"):
        load_config(path)

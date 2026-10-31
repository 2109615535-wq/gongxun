import json
import time

import cv2
import pytest
import serial

from rescue_vision.calibrate import fit_ground, validate_ground
from rescue_vision.model_tools import write_json
from rescue_vision.protocol import StreamDecoder
from rescue_vision.runtime import LatestFrame, run
from rescue_vision.types import Detection, FrameFlag, Status
from rescue_vision.validation import performance_report, trial_report


def entries(pixels):
    return [dict(pixel=[u, v], ground_mm=[1400-3*v, 320-u]) for u, v in pixels]


def test_ground_fit_then_independent_validation(tmp_path, calibration_data):
    original, fit, checks, output = [tmp_path/name for name in ("intrinsics.json", "fit.json", "check.json", "calibration.json")]
    write_json(original, dict(calibration_data, validated=False, ground_homography=None))
    points = entries([(10, 10), (630, 10), (630, 410), (10, 410), (100, 200), (540, 200)])
    write_json(fit, dict(points=points, image_size=[640, 480], forward_range_mm=[150, 1400],
                         bumper_forward_mm=120, collection_center_left_mm=0))
    result = fit_ground(original, fit, output)
    assert result["validated"] is False
    validation = entries([(290, 360), (350, 360), (320, 50), (200, 250), (440, 250), (320, 330)])
    write_json(checks, dict(points=validation, image_size=[640, 480]))
    result = validate_ground(output, checks, output)
    assert result["validated"] is True
    assert max(result["validation_max_error_mm"]) < .01
    write_json(checks, dict(points=points, image_size=[640, 480]))
    with pytest.raises(ValueError, match="reuse"):
        validate_ground(output, checks, tmp_path/"bad.json")
    validation[0]["ground_mm"][1] += 25
    write_json(checks, dict(points=validation, image_size=[640, 480]))
    with pytest.raises(ValueError, match="failed"):
        validate_ground(output, checks, tmp_path/"bad.json")
    assert not (tmp_path/"bad.json").exists()


def test_mailbox_drops_backlog_instead_of_processing_old_frames():
    mailbox = LatestFrame()
    for i in range(100):
        mailbox.publish(i, i/30, None)
    assert mailbox.next(-1)[0] == 99
    assert mailbox.next(99, timeout=.001) is None


@pytest.mark.parametrize("scenario", ["camera_disconnect", "slow_inference", "healthy"])
def test_live_watchdog_keeps_emitting_invalid_frames_independently(
        monkeypatch, tmp_path, calibration_data, config, green_image, scenario):
    import rescue_vision.runtime as runtime
    calibration_path = tmp_path/"calibration.json"
    write_json(calibration_path, calibration_data)
    config = dict(config, calibration=str(calibration_path), model="fake.rknn", manifest="fake.json",
                  serial_protocol="vision_v1")
    writes = []
    class Port:
        def __init__(self, *args, **kwargs):
            pass
        def write(self, packet):
            writes.append((time.monotonic(), packet))
            return len(packet)
        def close(self):
            pass
    class Camera:
        def __init__(self, *args):
            self.n = 0
        def isOpened(self):
            return True
        def set(self, *args):
            return True
        def read(self):
            time.sleep(.025)
            self.n += 1
            if scenario == "camera_disconnect" and self.n > 7:
                return False, None
            return True, green_image.copy()
        def release(self):
            pass
    class Model:
        def __init__(self, *args):
            pass
        def predict(self, image):
            if scenario == "slow_inference":
                time.sleep(.25)
            return [Detection("normal_material", .9, (280, 250, 340, 290))]
        def close(self):
            pass
    monkeypatch.setattr(serial, "Serial", Port)
    monkeypatch.setattr(runtime, "create_detector", lambda config: Model())
    monkeypatch.setattr(cv2, "VideoCapture", Camera)
    run(config, tmp_path/scenario, serial_port="test", duration=.42)
    decoder = StreamDecoder()
    observations = [decoder.feed(packet)[0][1] for _, packet in writes]
    assert len(observations) > 5
    assert not observations[-1].flags & FrameFlag.VISION_VALID
    if scenario == "slow_inference":
        assert not any(o.flags & FrameFlag.VISION_VALID for o in observations)
        assert any(o.status == Status.STALE for o in observations)
    else:
        assert any(o.flags & FrameFlag.VISION_VALID for o in observations)
        valid_ticks = [o.tick_ms for o in observations if o.flags & FrameFlag.VISION_VALID]
        assert len(valid_ticks) == len(set(valid_ticks))
    if scenario == "camera_disconnect":
        assert observations[-2].status == Status.CAMERA_FAULT
    assert max(b[0]-a[0] for a, b in zip(writes, writes[1:])) < .15


def test_real_time_report_rejects_replay_and_missing_serial(tmp_path):
    path, output = tmp_path/"log.jsonl", tmp_path/"report.json"
    rows = [dict(status="OK", flags=3, capture_to_send_ms=50,
                 sent_monotonic=i/20, transmitted=False) for i in range(61)]
    path.write_text("\n".join(json.dumps(row) for row in rows), encoding="utf-8")
    assert performance_report(path, output)["passed"] is False
    rows[0]["offline_replay"] = True
    path.write_text("\n".join(json.dumps(row) for row in rows), encoding="utf-8")
    with pytest.raises(ValueError, match="Offline"):
        performance_report(path, output)


def test_trial_report_demands_30_trials_and_all_fault_tests(tmp_path):
    path, output = tmp_path/"trials.csv", tmp_path/"trials.json"
    header = "trial_id,scenario,measured_gap_mm,measured_lateral_mm,stopped,stop_latency_ms\n"
    rows = [f"{i},approach,{200 if i < 27 else 250},10,1,100\n" for i in range(30)]
    path.write_text(header+"".join(rows), encoding="utf-8")
    assert trial_report(path, output)["passed"] is False
    rows.extend(f"{30+i},{name},0,0,1,250\n" for i, name in enumerate(
        ("occlusion", "camera_disconnect", "serial_disconnect", "hazard")))
    path.write_text(header+"".join(rows), encoding="utf-8")
    assert trial_report(path, output)["passed"] is True

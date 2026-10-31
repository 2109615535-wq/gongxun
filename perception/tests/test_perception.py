import numpy as np
import pytest

from rescue_vision.detector import decode_rk_outputs, letterbox, restore_boxes
from rescue_vision.geometry import GroundCalibration, green_contact
from rescue_vision.pipeline import Perception
from rescue_vision.types import Detection, FrameFlag, Status, TargetFlag


@pytest.mark.parametrize("shape", [(480, 640), (640, 480), (333, 777), (481, 639)])
def test_letterbox_roundtrip(shape):
    image = np.zeros((*shape, 3), np.uint8)
    padded, (sx, sy, left, top) = letterbox(image)
    assert padded.shape == (640, 640, 3)
    box = np.array([[2, 5, shape[1]-3, shape[0]-6]], np.float32)
    transformed = box*np.array([sx, sy, sx, sy])+np.array([left, top, left, top])
    np.testing.assert_allclose(restore_boxes(transformed, (sx, sy, left, top), image.shape), box, atol=1e-4)


def test_decoder_uses_ten_custom_classes_and_dfl():
    outputs = []
    for size in (80, 40, 20):
        outputs.extend((np.zeros((1, 64, size, size), np.float32),
                        np.zeros((1, 10, size, size), np.float32),
                        np.zeros((1, 1, size, size), np.float32)))
    outputs[1][0, 3, 40, 40] = .9
    boxes, classes, scores = decode_rk_outputs(outputs, 10)
    assert classes.tolist() == [3]
    np.testing.assert_allclose(boxes[0], [264, 264, 384, 384], atol=1e-4)
    assert scores[0] == pytest.approx(.9)
    with pytest.raises(ValueError, match="class count"):
        decode_rk_outputs(outputs, 80)
    with pytest.raises(ValueError, match="9-output"):
        decode_rk_outputs(outputs[:1], 10)


def test_ground_contact_is_bottom_not_center(green_image, calibration, config):
    contact = green_contact(green_image, (280, 250, 340, 290),
                            config["green_hsv_lower"], config["green_hsv_upper"])
    assert contact[1] == 289
    assert calibration.project(contact) == pytest.approx((533, 10.5))
    assert calibration.project((320, 270))[0] != calibration.project(contact)[0]
    assert calibration.project((0, 0)) is None


def test_unvalidated_never_produces_valid_position(calibration_data, green_image, config):
    calibration = GroundCalibration(dict(calibration_data, validated=False))
    pipeline = Perception(calibration, config)
    detection = Detection("normal_material", .9, (280, 250, 340, 290))
    for i in range(5):
        result = pipeline.process(green_image, [detection], i*67)
        assert not result.flags & FrameFlag.VISION_VALID
        assert result.status == Status.CALIBRATION_INVALID
        assert result.targets[0].distance_mm == 0


def test_confirmation_loss_and_reacquisition(calibration, green_image, config):
    pipeline = Perception(calibration, config)
    detection = Detection("normal_material", .9, (280, 250, 340, 290))
    for i in range(3):
        result = pipeline.process(green_image, [detection], i*67)
        assert bool(result.flags & FrameFlag.VISION_VALID) == (i == 2)
    assert result.targets[0].flags & TargetFlag.CONFIRMED
    lost = pipeline.process(green_image, [], 201)
    assert lost.status == Status.TARGET_LOST and not lost.targets
    reacquired = pipeline.process(green_image, [detection], 268)
    assert not reacquired.flags & FrameFlag.VISION_VALID


@pytest.mark.parametrize("bad", [Detection("hazard", .9, (350, 250, 390, 290)),
                                  Detection("unexpected", .9, (350, 250, 390, 290)),
                                  Detection("hazard", .9, (0, 10, 30, 50))])
def test_hazard_unknown_or_unlocalized_hazard_stops(calibration, green_image, config, bad):
    pipeline = Perception(calibration, config)
    normal = Detection("normal_material", .9, (280, 250, 340, 290))
    for i in range(4):
        result = pipeline.process(green_image, [normal, bad], i*67)
    assert result.flags & FrameFlag.HAZARD_BLOCKED
    assert not result.flags & FrameFlag.VISION_VALID


def test_hazard_outside_corridor_does_not_block(calibration, green_image, config):
    pipeline = Perception(calibration, config)
    objects = [Detection("normal_material", .9, (280, 250, 340, 290)),
               Detection("hazard", .9, (570, 250, 610, 290))]
    for i in range(3):
        result = pipeline.process(green_image, objects, i*67)
    assert result.flags & FrameFlag.VISION_VALID
    assert not result.flags & FrameFlag.HAZARD_BLOCKED


def test_occlusion_rejects_contact(calibration, green_image, config):
    pipeline = Perception(calibration, config)
    objects = [Detection("normal_material", .9, (280, 250, 340, 290)),
               Detection("core_material", .9, (300, 260, 350, 300))]
    for i in range(4):
        result = pipeline.process(green_image, objects, i*67)
    assert not result.flags & FrameFlag.VISION_VALID
    assert result.targets[0].flags & TargetFlag.OCCLUDED


def test_camera_size_mismatch_and_stop_coverage(calibration, calibration_data):
    with pytest.raises(ValueError, match="resolution"):
        calibration.undistort(np.zeros((720, 1280, 3), np.uint8))
    with pytest.raises(ValueError, match="stop interval"):
        GroundCalibration(dict(calibration_data, forward_range_mm=[400, 1400]))


def test_blank_green_box_not_positioned(calibration, config):
    image = np.full((480, 640, 3), 200, np.uint8)
    pipeline = Perception(calibration, config)
    for i in range(3):
        result = pipeline.process(image, [Detection("normal_material", .9, (280, 250, 340, 290))], i)
    assert result.targets[0].distance_mm == 0
    assert not result.flags & FrameFlag.VISION_VALID

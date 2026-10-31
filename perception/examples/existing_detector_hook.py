"""Callable hook for the board's existing post-processing; not a replacement detector."""
from rescue_vision.session import DetectionSession, detections_from_arrays


def make_session(config_path="config/vision.json"):
    return DetectionSession.from_config(config_path)


def observe_existing_frame(session, original_bgr_frame, boxes_original_xyxy, scores,
                           class_ids, actual_class_names, captured_monotonic,
                           pixel_space="raw"):
    """Called after existing inference. Capture timestamp must be recorded BEFORE inference.

    This hook neither imports RKNN nor opens a camera/UART. The application keeps
    its own capture/model/preprocessing/NMS. Pass its ACTUAL labels and boxes already
    mapped from model coordinates back to the original image. Do not reuse package
    class IDs unless they match your model. Use pixel_space='undistorted' only when
    both frame and boxes already use the calibration's un-cropped K coordinate system.
    """
    detections = detections_from_arrays(boxes_original_xyxy, scores, class_ids, actual_class_names)
    return session.observe(original_bgr_frame, detections, captured_monotonic,
                           pixel_space=pixel_space)

from math import atan2, degrees, hypot

import numpy as np

from .detector import box_iou
from .geometry import green_contact, touches_border
from .types import Detection, FrameFlag, Observation, Status, Target, TargetFlag, TARGET_TYPES


class Perception:
    def __init__(self, calibration, config):
        self.calibration, self.config = calibration, config
        self.last_box = None
        self.filtered = None
        self.count = 0
        self.ever_confirmed = False

    def reset(self):
        self.last_box = self.filtered = None
        self.count = 0

    def fault(self, tick, status):
        self.reset()
        return Observation(tick, int(FrameFlag.CALIBRATION_VALID) if self.calibration.valid else 0, status)

    def _target(self, detection, point=None, confirmed=False, occluded=False):
        flags = TargetFlag.HAZARD if detection.name == "hazard" else TargetFlag(0)
        if occluded:
            flags |= TargetFlag.OCCLUDED
        angle = distance = 0
        if point is not None:
            x, y = point
            angle = int(round(degrees(atan2(-y, x))*100))
            distance = int(round(hypot(x, y)))
            if 0 < distance <= 65535:
                flags |= TargetFlag.POSITION_VALID
            else:
                distance = 0
        if confirmed:
            flags |= TargetFlag.CONFIRMED
        x1, y1, x2, y2 = detection.box
        return Target(TARGET_TYPES.get(detection.name, 255),
                      int(np.clip(round(detection.confidence*100), 0, 100)), angle, distance,
                      min(65535, round(x2-x1)), min(65535, round(y2-y1)), 0, int(flags))

    def process(self, image, detections: list[Detection], tick_ms, pixel_space="undistorted"):
        calibration, cfg = self.calibration, self.config
        if pixel_space not in ("raw", "undistorted"):
            raise ValueError("pixel_space must be raw or undistorted")
        project = calibration.project_raw if pixel_space == "raw" else calibration.project
        flags = int(FrameFlag.CALIBRATION_VALID) if calibration.valid else 0
        targets, normals = [], []
        blocked = False
        objects = [d for d in detections if d.name in TARGET_TYPES]
        for detection in objects:
            x1, y1, x2, y2 = detection.box
            point = None
            occluded = touches_border(detection.box, calibration.size)
            if detection.name == "normal_material":
                contact = green_contact(image, detection.box,
                                        cfg["green_hsv_lower"], cfg["green_hsv_upper"])
                # An overlapping object may hide the true lower edge. Do not infer through it.
                occluded |= any(other is not detection and box_iou(detection.box, other.box) > .12
                                for other in objects)
                if contact is not None and not occluded:
                    point = project(contact)
                normals.append((detection, point, occluded))
            else:
                if not occluded:
                    point = project(((x1+x2)/2, y2-1))
                targets.append(self._target(detection, point, occluded=occluded))
                if detection.name == "hazard":
                    # Invalid hazard depth cannot prove that the corridor is clear.
                    blocked |= (point is None or
                                (0 < point[0]-calibration.bumper_mm <= 1400 and
                                 abs(point[1]-calibration.center_left_mm)
                                 <= cfg["hazard_corridor_half_width_mm"]))
        known_names = set(TARGET_TYPES) | {"gripper", "gripper_closed", "red_material_zone",
                                         "red_casualty_zone", "blue_material_zone", "blue_casualty_zone"}
        unknown = any(d.name not in known_names for d in detections)
        blocked |= unknown
        if blocked:
            flags |= int(FrameFlag.HAZARD_BLOCKED)

        selected = None
        if normals:
            if self.last_box is not None:
                candidates = [(box_iou(self.last_box, d.box), (d, p, o)) for d, p, o in normals]
                overlap, selected = max(candidates, key=lambda item: item[0])
                if overlap < cfg["association_iou"]:
                    self.reset()
                    selected = None  # reacquire on a later fresh frame, never switch while moving
            else:
                selected = min(normals, key=lambda item: abs((item[0].box[0]+item[0].box[2])/2-image.shape[1]/2))

        diagnostic = {"detections": len(detections), "hazard_blocked": bool(blocked),
                      "pixel_space": pixel_space}
        if selected is None:
            self.reset()
            state = Status.TARGET_LOST if self.ever_confirmed else Status.NO_TARGET
        else:
            detection, point, occluded = selected
            diagnostic["selected_box"] = list(detection.box)
            if point is None or blocked:
                self.reset()
                targets.insert(0, self._target(detection, point, occluded=occluded))
                state = Status.CALIBRATION_INVALID if not calibration.valid else Status.TARGET_INVALID
            else:
                if self.filtered is None:
                    self.filtered = np.asarray(point, np.float64)
                else:
                    alpha = cfg["smoothing_alpha"]
                    self.filtered = alpha*np.asarray(point)+(1-alpha)*self.filtered
                self.last_box = detection.box
                self.count += 1
                confirmed = self.count >= cfg["confirm_frames"]
                # Never let filtering move an already too-close target back into a safe interval.
                position = point if point[0]-calibration.bumper_mm < 170 else tuple(self.filtered)
                targets.insert(0, self._target(detection, position, confirmed=confirmed))
                diagnostic.update(x_forward_mm=position[0], y_left_mm=position[1],
                                  bumper_gap_mm=position[0]-calibration.bumper_mm,
                                  lateral_mm=position[1]-calibration.center_left_mm,
                                  confirm_count=self.count)
                state = Status.OK if confirmed else Status.TARGET_INVALID
                if confirmed:
                    flags |= int(FrameFlag.VISION_VALID)
                    self.ever_confirmed = True
        if not calibration.valid:
            state = Status.CALIBRATION_INVALID
        # A truncated list must never hide a hazard. Hazards are listed before other materials.
        if len(targets) > 12:
            flags |= int(FrameFlag.HAZARD_BLOCKED)
            flags &= ~int(FrameFlag.VISION_VALID)
        return Observation(tick_ms & 0xFFFFFFFF, flags, state, tuple(targets[:12]), diagnostic)

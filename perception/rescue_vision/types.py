from dataclasses import dataclass, field
from enum import IntEnum, IntFlag


class Status(IntEnum):
    OK = 0
    NO_TARGET = 1
    CAMERA_FAULT = 2
    CALIBRATION_INVALID = 3
    TARGET_INVALID = 4
    TARGET_LOST = 5
    STALE = 6
    INFERENCE_FAULT = 7


class FrameFlag(IntFlag):
    VISION_VALID = 1
    CALIBRATION_VALID = 2
    HAZARD_BLOCKED = 16  # new v1 meaning for previously unused bit


class TargetFlag(IntFlag):
    HAZARD = 1
    OCCLUDED = 4
    CONFIRMED = 16
    POSITION_VALID = 32


TARGET_TYPES = {"normal_material": 1, "core_material": 2, "casualty": 3, "hazard": 4}


@dataclass(frozen=True)
class Detection:
    name: str
    confidence: float
    box: tuple[float, float, float, float]


@dataclass(frozen=True)
class Target:
    kind: int
    confidence: int
    angle_cdeg: int = 0  # right positive, relative to chassis origin
    distance_mm: int = 0  # ground-plane radial distance from chassis origin
    width_px: int = 0
    height_px: int = 0
    posture: int = 0
    flags: int = 0


@dataclass(frozen=True)
class Observation:
    tick_ms: int
    flags: int
    status: Status
    targets: tuple[Target, ...] = ()
    diagnostics: dict = field(default_factory=dict, compare=False)

from dataclasses import dataclass


@dataclass(frozen=True)
class MotionSegment:
    linear_x: float
    angular_z: float
    duration_sec: float

import math
from typing import List

try:
    from .motion_segments import MotionSegment
except ImportError:
    from motion_segments import MotionSegment


CIRCLE_DURATION_SCALE = 4.0
FIGURE8_SEGMENT_DURATION_SCALE = 1.0
FIGURE8_S_REPEAT_COUNT = 2
_EPSILON = 1e-6


def _positive_duration(value: float) -> float:
    return max(0.0, float(value))


def _base_turn_duration(angular_z: float) -> float:
    angular = abs(float(angular_z))
    if angular <= _EPSILON:
        return 0.0
    return 2.0 * math.pi / angular


def move_preset(speed_mps: float, duration_sec: float) -> List[MotionSegment]:
    return [MotionSegment(float(speed_mps), 0.0, _positive_duration(duration_sec))]


def back_preset(reverse_speed_mps: float, duration_sec: float) -> List[MotionSegment]:
    return [MotionSegment(float(reverse_speed_mps), 0.0, _positive_duration(duration_sec))]


def left_turn_preset(speed_mps: float, angular_z: float, duration_sec: float) -> List[MotionSegment]:
    return [MotionSegment(float(speed_mps), abs(float(angular_z)), _positive_duration(duration_sec))]


def right_turn_preset(speed_mps: float, angular_z: float, duration_sec: float) -> List[MotionSegment]:
    return [MotionSegment(float(speed_mps), -abs(float(angular_z)), _positive_duration(duration_sec))]


def left_circle(
    speed_mps: float,
    angular_z: float,
    duration_scale: float = CIRCLE_DURATION_SCALE,
) -> List[MotionSegment]:
    angular = abs(float(angular_z))
    duration = _base_turn_duration(angular) * max(0.0, float(duration_scale))
    return [MotionSegment(float(speed_mps), angular, duration)]


def right_circle(
    speed_mps: float,
    angular_z: float,
    duration_scale: float = CIRCLE_DURATION_SCALE,
) -> List[MotionSegment]:
    angular = abs(float(angular_z))
    duration = _base_turn_duration(angular) * max(0.0, float(duration_scale))
    return [MotionSegment(float(speed_mps), -angular, duration)]


def _single_s(
    speed_mps: float,
    angular_z: float,
    segment_duration_sec: float,
) -> List[MotionSegment]:
    angular = abs(float(angular_z))
    duration = _positive_duration(segment_duration_sec)
    return [
        MotionSegment(float(speed_mps), angular, duration),
        MotionSegment(float(speed_mps), -angular, duration),
    ]


def figure_eight(
    speed_mps: float,
    angular_z: float,
    segment_duration_scale: float = FIGURE8_SEGMENT_DURATION_SCALE,
    s_repeat_count: int = FIGURE8_S_REPEAT_COUNT,
) -> List[MotionSegment]:
    angular = abs(float(angular_z))
    segment_duration = _base_turn_duration(angular) * max(
        0.0,
        float(segment_duration_scale),
    )
    repeat_count = max(1, int(s_repeat_count))

    segments: List[MotionSegment] = []
    for _ in range(repeat_count):
        segments.extend(_single_s(speed_mps, angular, segment_duration))
    return segments

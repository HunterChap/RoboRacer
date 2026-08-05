#!/usr/bin/env python3
"""Throttle-free adaptive corridor steering test for the RoboRacer.

This ROS 2 node is a safer, track-aware alternative to basic Follow-the-Gap.
It subscribes to the Orbbec SL450 LaserScan and commands ONLY VESC servo
message ID 12.  It never sends duty-cycle, current, RPM, velocity, or motor
commands.

The default behavior is tailored to the current RoboRacer requirements:

* 25 inch (0.635 m) track width.
* Track side walls do not trigger obstacle avoidance.
* Both side walls may create a small centering correction.
* Only a clustered object in the car's projected forward body lane triggers
  avoidance.
* The closer the front object, the sharper the requested turn.
* A front wall plus a clear side opening is treated as a 90-degree corner.
* A 0.324 m wheelbase and 3 ft target radius imply about 19.5 degrees of
  road-wheel steering.  The actual servo-to-wheel-angle relationship must be
  checked with a measured, low-speed floor test before relying on that radius.

The verified on-car steering convention is preserved:

    servo 0.250 = left
    servo 0.500 = center
    servo 0.750 = right

Normal use on the Raspberry Pi (motor remains disabled):

    source /opt/ros/jazzy/setup.bash
    source ~/orbbec_ws/install/setup.bash
    source ~/venv_vesc/bin/activate
    python3 ~/sl450_adaptive_corridor_steering_test.py

Dependency-free planner checks:

    python3 sl450_adaptive_corridor_steering_test.py --self-test
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import statistics
import sys
import time
from typing import Optional, Sequence


INCHES_TO_METERS = 0.0254
FEET_TO_METERS = 0.3048


def clamp(value: float, lower: float, upper: float) -> float:
    """Limit value to the closed interval [lower, upper]."""
    return max(lower, min(value, upper))


def normalize_angle(angle_radians: float) -> float:
    """Return an angle in the interval [-pi, pi)."""
    return (angle_radians + math.pi) % (2.0 * math.pi) - math.pi


def percentile(values: Sequence[float], fraction: float) -> float:
    """Return a linearly interpolated percentile without requiring NumPy."""
    if not values:
        raise ValueError("percentile requires at least one value")
    ordered = sorted(values)
    position = clamp(fraction, 0.0, 1.0) * (len(ordered) - 1)
    lower_index = math.floor(position)
    upper_index = math.ceil(position)
    if lower_index == upper_index:
        return float(ordered[lower_index])
    weight = position - lower_index
    return float(
        ordered[lower_index] * (1.0 - weight)
        + ordered[upper_index] * weight
    )


def steering_angle_for_radius(wheelbase: float, radius: float) -> float:
    """Bicycle-model steering angle for a requested centerline radius."""
    if wheelbase <= 0.0 or radius <= 0.0:
        raise ValueError("wheelbase and radius must be positive")
    return math.atan(wheelbase / radius)


def estimated_turn_radius(wheelbase: float, steering_angle: float) -> Optional[float]:
    """Return bicycle-model radius, or None for effectively straight travel."""
    if wheelbase <= 0.0:
        raise ValueError("wheelbase must be positive")
    tangent = math.tan(abs(steering_angle))
    if tangent < 1e-6:
        return None
    return wheelbase / tangent


def steering_angle_to_servo_position(
    steering_angle: float,
    physical_maximum_steering_angle: float,
    center_position: float,
    left_position: float,
    right_position: float,
) -> float:
    """Map physical steering-angle estimate to verified servo endpoints.

    Positive steering angles mean left.  On this car, servo values below
    center turn left and values above center turn right.
    """
    if physical_maximum_steering_angle <= 0.0:
        raise ValueError("physical_maximum_steering_angle must be positive")

    signed_fraction = clamp(
        steering_angle / physical_maximum_steering_angle,
        -1.0,
        1.0,
    )
    if signed_fraction >= 0.0:
        position = center_position + signed_fraction * (
            left_position - center_position
        )
    else:
        position = center_position + (-signed_fraction) * (
            right_position - center_position
        )
    return clamp(position, 0.0, 1.0)


def rate_limited_position(
    current_position: float,
    target_position: float,
    maximum_change: float,
) -> float:
    """Move toward target by no more than maximum_change."""
    if maximum_change < 0.0:
        raise ValueError("maximum_change cannot be negative")
    return current_position + clamp(
        target_position - current_position,
        -maximum_change,
        maximum_change,
    )


@dataclass(frozen=True)
class ScanPoint:
    """One valid LiDAR return expressed in car-relative coordinates."""

    index: int
    angle: float
    distance: float
    forward: float
    left: float


@dataclass(frozen=True)
class AdaptiveSettings:
    """Planner settings shared by the ROS node and offline tests."""

    forward_raw_angle: float = math.radians(180.0)
    planning_half_fov: float = math.radians(105.0)
    maximum_considered_range: float = 6.0
    minimum_valid_fraction: float = 0.65

    track_width: float = 25.0 * INCHES_TO_METERS
    track_width_tolerance: float = 0.15
    vehicle_width: float = 0.32
    obstacle_safety_margin: float = 0.04
    lidar_lateral_offset: float = 0.0

    side_sector_min_angle: float = math.radians(65.0)
    side_sector_max_angle: float = math.radians(88.0)
    side_wall_max_distance: float = 0.60
    side_minimum_points: int = 8
    centering_deadband: float = 0.025
    centering_full_error: float = 0.12
    maximum_centering_angle: float = math.radians(7.0)

    front_minimum_forward: float = 0.08
    avoidance_start_distance: float = 1.50
    full_corner_distance: float = 0.55
    emergency_stop_distance: float = 0.30
    minimum_obstacle_cluster_points: int = 5
    maximum_cluster_angle_gap: float = math.radians(0.45)
    maximum_cluster_forward_jump: float = 0.20

    opening_sector_min_angle: float = math.radians(8.0)
    opening_sector_max_angle: float = math.radians(55.0)
    opening_score_percentile: float = 0.35
    opening_score_cap: float = 3.0
    direction_score_deadband: float = 0.08
    obstacle_side_deadband: float = 0.025

    wheelbase: float = 0.324
    target_corner_radius: float = 3.0 * FEET_TO_METERS
    physical_maximum_steering_angle: float = math.radians(30.0)


@dataclass(frozen=True)
class AdaptiveDecision:
    """One complete track-aware planner result."""

    safe: bool
    mode: str
    direction: str
    reason: str
    steering_angle: Optional[float]
    front_distance: Optional[float]
    obstacle_lateral_position: Optional[float]
    left_wall_distance: Optional[float]
    right_wall_distance: Optional[float]
    center_error: Optional[float]
    left_opening_score: Optional[float]
    right_opening_score: Optional[float]
    estimated_radius: Optional[float]
    valid_fraction: float


def _scan_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    settings: AdaptiveSettings,
) -> tuple[list[ScanPoint], float, float]:
    """Convert scan readings to car coordinates and report scan quality."""
    if not ranges or not math.isfinite(angle_increment) or angle_increment <= 0.0:
        return [], 0.0, 0.0

    points: list[ScanPoint] = []
    considered_count = 0
    valid_count = 0
    car_angles: list[float] = []

    for index, measured_range in enumerate(ranges):
        raw_angle = angle_min + index * angle_increment
        car_angle = normalize_angle(raw_angle - settings.forward_raw_angle)
        if abs(car_angle) > settings.planning_half_fov:
            continue

        considered_count += 1
        car_angles.append(car_angle)
        is_finite_valid = (
            math.isfinite(measured_range)
            and range_min <= measured_range <= range_max
        )
        is_clear_return = math.isinf(measured_range) and measured_range > 0.0
        if not (is_finite_valid or is_clear_return):
            continue

        valid_count += 1
        distance = (
            min(float(measured_range), settings.maximum_considered_range)
            if is_finite_valid
            else settings.maximum_considered_range
        )
        points.append(
            ScanPoint(
                index=index,
                angle=car_angle,
                distance=distance,
                forward=distance * math.cos(car_angle),
                left=distance * math.sin(car_angle),
            )
        )

    valid_fraction = (
        valid_count / considered_count if considered_count > 0 else 0.0
    )
    if len(car_angles) >= 2:
        car_angles.sort()
        angular_step = statistics.median(
            car_angles[index + 1] - car_angles[index]
            for index in range(len(car_angles) - 1)
        )
    else:
        angular_step = 0.0
    return points, valid_fraction, angular_step


def _estimate_side_wall(
    points: Sequence[ScanPoint],
    side_sign: int,
    settings: AdaptiveSettings,
) -> Optional[float]:
    """Estimate one side-wall lateral distance from a forward-side sector."""
    lateral_distances: list[float] = []
    for point in points:
        signed_angle = side_sign * point.angle
        if not (
            settings.side_sector_min_angle
            <= signed_angle
            <= settings.side_sector_max_angle
        ):
            continue
        if point.forward < 0.0:
            continue
        lateral_distance = side_sign * point.left
        if 0.0 < lateral_distance <= settings.side_wall_max_distance:
            lateral_distances.append(lateral_distance)

    if len(lateral_distances) < settings.side_minimum_points:
        return None
    return float(statistics.median(lateral_distances))


def _track_center_measurement(
    points: Sequence[ScanPoint],
    settings: AdaptiveSettings,
) -> tuple[Optional[float], Optional[float], Optional[float]]:
    """Return left/right wall distance and signed centering error.

    A positive center error means there is more room on the left, so the car
    is too close to the right wall and should steer left.  Centering is used
    only if both observed walls add up to the known track width.
    """
    left_sensor = _estimate_side_wall(points, +1, settings)
    right_sensor = _estimate_side_wall(points, -1, settings)
    if left_sensor is None or right_sensor is None:
        return left_sensor, right_sensor, None

    left_car_center = left_sensor + settings.lidar_lateral_offset
    right_car_center = right_sensor - settings.lidar_lateral_offset
    observed_width = left_car_center + right_car_center
    if abs(observed_width - settings.track_width) > settings.track_width_tolerance:
        return left_car_center, right_car_center, None

    return (
        left_car_center,
        right_car_center,
        left_car_center - right_car_center,
    )


def _front_clusters(
    points: Sequence[ScanPoint],
    angular_step: float,
    settings: AdaptiveSettings,
) -> list[list[ScanPoint]]:
    """Find coherent obstacles inside the car's projected forward lane."""
    half_lane_width = settings.vehicle_width / 2.0 + settings.obstacle_safety_margin
    candidates = sorted(
        (
            point
            for point in points
            if settings.front_minimum_forward
            <= point.forward
            <= settings.avoidance_start_distance
            and abs(point.left) <= half_lane_width
        ),
        key=lambda point: point.angle,
    )
    if not candidates:
        return []

    allowed_angle_gap = max(
        settings.maximum_cluster_angle_gap,
        2.5 * angular_step if angular_step > 0.0 else 0.0,
    )
    clusters: list[list[ScanPoint]] = []
    current = [candidates[0]]
    for point in candidates[1:]:
        previous = current[-1]
        same_cluster = (
            point.angle - previous.angle <= allowed_angle_gap
            and abs(point.forward - previous.forward)
            <= settings.maximum_cluster_forward_jump
        )
        if same_cluster:
            current.append(point)
        else:
            if len(current) >= settings.minimum_obstacle_cluster_points:
                clusters.append(current)
            current = [point]
    if len(current) >= settings.minimum_obstacle_cluster_points:
        clusters.append(current)
    return clusters


def _nearest_front_obstacle(
    points: Sequence[ScanPoint],
    angular_step: float,
    settings: AdaptiveSettings,
) -> tuple[Optional[float], Optional[float]]:
    """Return robust forward distance and lateral center of nearest cluster."""
    clusters = _front_clusters(points, angular_step, settings)
    if not clusters:
        return None, None

    summaries: list[tuple[float, float]] = []
    for cluster in clusters:
        forward_distance = percentile(
            [point.forward for point in cluster],
            0.10,
        )
        near_points = [
            point
            for point in cluster
            if point.forward <= forward_distance + 0.10
        ]
        lateral_center = float(statistics.median(
            point.left for point in near_points
        ))
        summaries.append((forward_distance, lateral_center))
    return min(summaries, key=lambda summary: summary[0])


def _opening_score(
    points: Sequence[ScanPoint],
    side_sign: int,
    settings: AdaptiveSettings,
) -> Optional[float]:
    """Score usable visibility toward one forward-side direction."""
    distances = [
        min(point.distance, settings.opening_score_cap)
        for point in points
        if settings.opening_sector_min_angle
        <= side_sign * point.angle
        <= settings.opening_sector_max_angle
        and point.forward > 0.05
    ]
    if len(distances) < settings.side_minimum_points:
        return None
    return percentile(distances, settings.opening_score_percentile)


def _choose_turn_sign(
    obstacle_lateral_position: float,
    left_score: Optional[float],
    right_score: Optional[float],
    center_error: Optional[float],
    previous_turn_sign: int,
    settings: AdaptiveSettings,
) -> int:
    """Return +1 for left or -1 for right."""
    # If the obstacle is clearly on one side, first turn away from it.
    if obstacle_lateral_position > settings.obstacle_side_deadband:
        return -1
    if obstacle_lateral_position < -settings.obstacle_side_deadband:
        return +1

    # A centered front wall is a corner cue: turn toward the clearer opening.
    if left_score is not None and right_score is not None:
        score_difference = left_score - right_score
        if score_difference > settings.direction_score_deadband:
            return +1
        if score_difference < -settings.direction_score_deadband:
            return -1

    # Avoid direction chatter when a centered obstacle is nearly symmetric.
    if previous_turn_sign in (-1, +1):
        return previous_turn_sign

    # If this is the first ambiguous decision, use the side with more track room.
    if center_error is not None and abs(center_error) > settings.centering_deadband:
        return +1 if center_error > 0.0 else -1
    return +1


def _adaptive_avoidance_magnitude(
    front_distance: float,
    settings: AdaptiveSettings,
) -> float:
    """Increase steering continuously as a front obstacle gets closer."""
    corner_angle = steering_angle_for_radius(
        settings.wheelbase,
        settings.target_corner_radius,
    )
    corner_angle = min(corner_angle, settings.physical_maximum_steering_angle)

    if front_distance >= settings.avoidance_start_distance:
        return 0.0
    if front_distance >= settings.full_corner_distance:
        span = settings.avoidance_start_distance - settings.full_corner_distance
        progress = (
            settings.avoidance_start_distance - front_distance
        ) / span
        # Square-root response makes the first visible avoidance command more
        # noticeable, while remaining continuous and distance-dependent.
        response = math.sqrt(clamp(progress, 0.0, 1.0))
        return corner_angle * response

    urgent_span = settings.full_corner_distance - settings.emergency_stop_distance
    urgent_progress = (
        settings.full_corner_distance - front_distance
    ) / urgent_span
    return corner_angle + clamp(urgent_progress, 0.0, 1.0) * (
        settings.physical_maximum_steering_angle - corner_angle
    )


def _centering_angle(
    center_error: Optional[float],
    settings: AdaptiveSettings,
) -> float:
    """Turn gently toward track center, with a position deadband."""
    if center_error is None or abs(center_error) <= settings.centering_deadband:
        return 0.0
    usable_error = abs(center_error) - settings.centering_deadband
    usable_full_error = max(
        settings.centering_full_error - settings.centering_deadband,
        1e-6,
    )
    fraction = clamp(usable_error / usable_full_error, 0.0, 1.0)
    return math.copysign(
        fraction * settings.maximum_centering_angle,
        center_error,
    )


def plan_adaptive_corridor_steering(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    settings: AdaptiveSettings,
    previous_turn_sign: int = 0,
) -> AdaptiveDecision:
    """Analyze one scan without producing any vehicle command."""
    points, valid_fraction, angular_step = _scan_points(
        ranges,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        settings,
    )
    if angular_step <= 0.0 or not points:
        return AdaptiveDecision(
            False, "STOP", "STOP", "invalid scan geometry", None,
            None, None, None, None, None, None, None, None,
            valid_fraction,
        )
    if valid_fraction < settings.minimum_valid_fraction:
        return AdaptiveDecision(
            False,
            "STOP",
            "STOP",
            f"only {valid_fraction:.0%} of the planning scan is valid",
            None, None, None, None, None, None, None, None, None,
            valid_fraction,
        )

    left_wall, right_wall, center_error = _track_center_measurement(
        points,
        settings,
    )
    front_distance, obstacle_lateral = _nearest_front_obstacle(
        points,
        angular_step,
        settings,
    )
    left_score = _opening_score(points, +1, settings)
    right_score = _opening_score(points, -1, settings)

    if (
        front_distance is not None
        and front_distance <= settings.emergency_stop_distance
    ):
        return AdaptiveDecision(
            False,
            "STOP",
            "STOP",
            (
                f"front obstacle at {front_distance:.2f} m is inside the "
                f"{settings.emergency_stop_distance:.2f} m emergency distance"
            ),
            None,
            front_distance,
            obstacle_lateral,
            left_wall,
            right_wall,
            center_error,
            left_score,
            right_score,
            None,
            valid_fraction,
        )

    if front_distance is not None and obstacle_lateral is not None:
        turn_sign = _choose_turn_sign(
            obstacle_lateral,
            left_score,
            right_score,
            center_error,
            previous_turn_sign,
            settings,
        )
        steering_angle = turn_sign * _adaptive_avoidance_magnitude(
            front_distance,
            settings,
        )
        direction = "LEFT" if turn_sign > 0 else "RIGHT"
        radius = estimated_turn_radius(settings.wheelbase, steering_angle)
        return AdaptiveDecision(
            True,
            "AVOID",
            direction,
            "front-lane obstacle; steering toward clearer side",
            steering_angle,
            front_distance,
            obstacle_lateral,
            left_wall,
            right_wall,
            center_error,
            left_score,
            right_score,
            radius,
            valid_fraction,
        )

    steering_angle = _centering_angle(center_error, settings)
    if steering_angle > math.radians(0.5):
        mode = "CENTER"
        direction = "LEFT"
        reason = "too close to right track wall"
    elif steering_angle < -math.radians(0.5):
        mode = "CENTER"
        direction = "RIGHT"
        reason = "too close to left track wall"
    else:
        mode = "STRAIGHT"
        direction = "STRAIGHT"
        reason = (
            "track centered"
            if center_error is not None
            else "no front obstacle; side walls ignored"
        )
    return AdaptiveDecision(
        True,
        mode,
        direction,
        reason,
        steering_angle,
        None,
        None,
        left_wall,
        right_wall,
        center_error,
        left_score,
        right_score,
        estimated_turn_radius(settings.wheelbase, steering_angle),
        valid_fraction,
    )


def _synthetic_corridor_scan(
    settings: AdaptiveSettings,
    *,
    sensor_offset_left: float = 0.0,
    obstacle_forward: Optional[float] = None,
    obstacle_center_left: float = 0.0,
    obstacle_width: float = 0.16,
) -> tuple[list[float], float, float]:
    """Ray-cast a simple straight corridor for dependency-free tests."""
    angle_min = math.radians(45.0)
    angle_increment = math.radians(0.25)
    sample_count = int(round(270.0 / 0.25)) + 1
    half_track = settings.track_width / 2.0
    left_wall = half_track - sensor_offset_left
    right_wall = -half_track - sensor_offset_left
    scan: list[float] = []

    for index in range(sample_count):
        raw_angle = angle_min + index * angle_increment
        angle = normalize_angle(raw_angle - settings.forward_raw_angle)
        dx = math.cos(angle)
        dy = math.sin(angle)
        intersections: list[float] = []

        if dy > 1e-9:
            intersections.append(left_wall / dy)
        elif dy < -1e-9:
            intersections.append(right_wall / dy)

        if obstacle_forward is not None and dx > 1e-9:
            distance_to_front = obstacle_forward / dx
            lateral_at_front = distance_to_front * dy
            if (
                obstacle_center_left - obstacle_width / 2.0
                <= lateral_at_front
                <= obstacle_center_left + obstacle_width / 2.0
            ):
                intersections.append(distance_to_front)

        positive = [distance for distance in intersections if distance > 0.0]
        scan.append(min(positive) if positive else math.inf)
    return scan, angle_min, angle_increment


def run_self_test() -> None:
    """Exercise side-wall ignoring, centering, avoidance, corner, and STOP."""
    settings = AdaptiveSettings()
    corner_angle = steering_angle_for_radius(
        settings.wheelbase,
        settings.target_corner_radius,
    )
    assert 19.0 <= math.degrees(corner_angle) <= 20.5

    centered, angle_min, increment = _synthetic_corridor_scan(settings)
    decision = plan_adaptive_corridor_steering(
        centered, angle_min, increment, 0.05, 30.0, settings
    )
    assert decision.safe and decision.mode == "STRAIGHT"
    assert decision.steering_angle is not None
    assert abs(decision.steering_angle) < math.radians(0.5)

    shifted_right, angle_min, increment = _synthetic_corridor_scan(
        settings,
        sensor_offset_left=-0.08,
    )
    decision = plan_adaptive_corridor_steering(
        shifted_right, angle_min, increment, 0.05, 30.0, settings
    )
    assert decision.safe and decision.mode == "CENTER"
    assert decision.direction == "LEFT"

    far_right_obstacle, angle_min, increment = _synthetic_corridor_scan(
        settings,
        obstacle_forward=1.20,
        obstacle_center_left=-0.08,
    )
    far_decision = plan_adaptive_corridor_steering(
        far_right_obstacle, angle_min, increment, 0.05, 30.0, settings
    )
    assert far_decision.safe and far_decision.mode == "AVOID"
    assert far_decision.direction == "LEFT"

    close_right_obstacle, angle_min, increment = _synthetic_corridor_scan(
        settings,
        obstacle_forward=0.55,
        obstacle_center_left=-0.08,
    )
    close_decision = plan_adaptive_corridor_steering(
        close_right_obstacle, angle_min, increment, 0.05, 30.0, settings
    )
    assert close_decision.safe and close_decision.direction == "LEFT"
    assert far_decision.steering_angle is not None
    assert close_decision.steering_angle is not None
    assert abs(close_decision.steering_angle) > abs(far_decision.steering_angle)
    assert abs(close_decision.steering_angle) >= corner_angle - math.radians(0.5)

    # Simulate a front wall with a right-side opening.  The side walls remain
    # present, but only the front wall activates avoidance.
    corner_scan, angle_min, increment = _synthetic_corridor_scan(
        settings,
        obstacle_forward=0.70,
        obstacle_center_left=0.0,
        obstacle_width=settings.track_width,
    )
    for index in range(len(corner_scan)):
        car_angle = normalize_angle(
            angle_min + index * increment - settings.forward_raw_angle
        )
        if math.radians(-55.0) <= car_angle <= math.radians(-12.0):
            corner_scan[index] = 2.5
    corner_decision = plan_adaptive_corridor_steering(
        corner_scan, angle_min, increment, 0.05, 30.0, settings
    )
    assert corner_decision.safe and corner_decision.mode == "AVOID"
    assert corner_decision.direction == "RIGHT"

    emergency, angle_min, increment = _synthetic_corridor_scan(
        settings,
        obstacle_forward=0.25,
        obstacle_center_left=0.0,
        obstacle_width=0.18,
    )
    decision = plan_adaptive_corridor_steering(
        emergency, angle_min, increment, 0.05, 30.0, settings
    )
    assert not decision.safe and decision.mode == "STOP"

    assert math.isclose(
        steering_angle_to_servo_position(
            0.0,
            settings.physical_maximum_steering_angle,
            0.500,
            0.250,
            0.750,
        ),
        0.500,
    )
    assert steering_angle_to_servo_position(
        corner_angle,
        settings.physical_maximum_steering_angle,
        0.500,
        0.250,
        0.750,
    ) < 0.500

    print(
        "Self-test passed: 25-inch side walls ignored, centering correction, "
        "distance-adaptive avoidance, 3-foot-radius corner command, opening "
        "selection, emergency STOP, and servo direction mapping."
    )


# Keep the offline test usable without ROS 2, pyserial, or pyvesc installed.
if __name__ == "__main__" and "--self-test" in sys.argv:
    run_self_test()
    raise SystemExit(0)


import pyvesc  # noqa: E402  (after dependency-free self-test)
import rclpy  # noqa: E402
import serial  # noqa: E402
from pyvesc import VESCMessage  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402


class SetServoPosition(metaclass=VESCMessage):
    """VESC servo-position command; the only VESC command in this file."""

    id = 12
    fields = [
        ("servo_pos", "h", 1000),
    ]


class SL450AdaptiveCorridorSteeringTest(Node):
    """Apply adaptive corridor decisions to steering only."""

    def __init__(self) -> None:
        super().__init__("sl450_adaptive_corridor_steering_test")

        self.declare_parameter("scan_topic", "/lidar/scan/points")
        self.declare_parameter("forward_raw_angle_degrees", 180.0)
        self.declare_parameter("planning_half_fov_degrees", 105.0)
        self.declare_parameter("maximum_considered_range_m", 6.0)
        self.declare_parameter("minimum_valid_fraction", 0.65)

        self.declare_parameter("track_width_m", 25.0 * INCHES_TO_METERS)
        self.declare_parameter("track_width_tolerance_m", 0.15)
        self.declare_parameter("vehicle_width_m", 0.32)
        self.declare_parameter("obstacle_safety_margin_m", 0.04)
        self.declare_parameter("lidar_lateral_offset_m", 0.0)
        self.declare_parameter("centering_deadband_m", 0.025)
        self.declare_parameter("centering_full_error_m", 0.12)
        self.declare_parameter("maximum_centering_angle_degrees", 7.0)

        self.declare_parameter("avoidance_start_distance_m", 1.50)
        self.declare_parameter("full_corner_distance_m", 0.55)
        self.declare_parameter("emergency_stop_distance_m", 0.30)
        self.declare_parameter("direction_score_deadband_m", 0.08)
        self.declare_parameter("obstacle_side_deadband_m", 0.025)

        self.declare_parameter("wheelbase_m", 0.324)
        self.declare_parameter("target_corner_radius_m", 3.0 * FEET_TO_METERS)
        self.declare_parameter("physical_max_steering_angle_degrees", 30.0)

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("servo_center", 0.500)
        self.declare_parameter("servo_left", 0.250)
        self.declare_parameter("servo_right", 0.750)
        self.declare_parameter("servo_rate_limit_per_second", 1.00)
        self.declare_parameter("urgent_servo_rate_limit_per_second", 2.00)
        self.declare_parameter("urgent_distance_m", 0.65)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("status_rate_hz", 4.0)
        self.declare_parameter("scan_timeout_seconds", 0.25)

        self.settings = AdaptiveSettings(
            forward_raw_angle=math.radians(float(
                self.get_parameter("forward_raw_angle_degrees").value
            )),
            planning_half_fov=math.radians(float(
                self.get_parameter("planning_half_fov_degrees").value
            )),
            maximum_considered_range=float(
                self.get_parameter("maximum_considered_range_m").value
            ),
            minimum_valid_fraction=float(
                self.get_parameter("minimum_valid_fraction").value
            ),
            track_width=float(self.get_parameter("track_width_m").value),
            track_width_tolerance=float(
                self.get_parameter("track_width_tolerance_m").value
            ),
            vehicle_width=float(self.get_parameter("vehicle_width_m").value),
            obstacle_safety_margin=float(
                self.get_parameter("obstacle_safety_margin_m").value
            ),
            lidar_lateral_offset=float(
                self.get_parameter("lidar_lateral_offset_m").value
            ),
            centering_deadband=float(
                self.get_parameter("centering_deadband_m").value
            ),
            centering_full_error=float(
                self.get_parameter("centering_full_error_m").value
            ),
            maximum_centering_angle=math.radians(float(
                self.get_parameter("maximum_centering_angle_degrees").value
            )),
            avoidance_start_distance=float(
                self.get_parameter("avoidance_start_distance_m").value
            ),
            full_corner_distance=float(
                self.get_parameter("full_corner_distance_m").value
            ),
            emergency_stop_distance=float(
                self.get_parameter("emergency_stop_distance_m").value
            ),
            direction_score_deadband=float(
                self.get_parameter("direction_score_deadband_m").value
            ),
            obstacle_side_deadband=float(
                self.get_parameter("obstacle_side_deadband_m").value
            ),
            wheelbase=float(self.get_parameter("wheelbase_m").value),
            target_corner_radius=float(
                self.get_parameter("target_corner_radius_m").value
            ),
            physical_maximum_steering_angle=math.radians(float(
                self.get_parameter("physical_max_steering_angle_degrees").value
            )),
        )

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.servo_center = float(self.get_parameter("servo_center").value)
        self.servo_left = float(self.get_parameter("servo_left").value)
        self.servo_right = float(self.get_parameter("servo_right").value)
        self.servo_rate_limit = float(
            self.get_parameter("servo_rate_limit_per_second").value
        )
        self.urgent_servo_rate_limit = float(
            self.get_parameter("urgent_servo_rate_limit_per_second").value
        )
        self.urgent_distance = float(
            self.get_parameter("urgent_distance_m").value
        )
        self.scan_timeout = float(
            self.get_parameter("scan_timeout_seconds").value
        )
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        status_rate_hz = float(self.get_parameter("status_rate_hz").value)

        self._validate_parameters(control_rate_hz, status_rate_hz)

        self.latest_decision: Optional[AdaptiveDecision] = None
        self.last_scan_time: Optional[float] = None
        self.planner_fault: Optional[str] = None
        self.serial_fault: Optional[str] = None
        self.previous_turn_sign = 0
        self.current_servo_position = self.servo_center
        self.last_control_time = time.monotonic()
        self._closed = False

        corner_angle = steering_angle_for_radius(
            self.settings.wheelbase,
            self.settings.target_corner_radius,
        )
        self.get_logger().warning(
            "STEERING-ONLY TEST: motor output is absent. Keep the drive wheels "
            "raised until stationary steering behavior is verified."
        )
        self.get_logger().info(
            f"Track {self.settings.track_width:.3f} m (25 in) | target corner "
            f"radius {self.settings.target_corner_radius:.3f} m (3 ft) | "
            f"calculated corner steering {math.degrees(corner_angle):.1f} deg."
        )

        self.connection = serial.Serial(
            port=self.serial_port,
            baudrate=self.baud_rate,
            timeout=0.1,
            write_timeout=1.0,
            exclusive=True,
        )
        time.sleep(1.0)
        self._write_servo(self.servo_center)
        self.get_logger().info("Steering centered; waiting for valid LiDAR data.")

        self.subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.control_timer = self.create_timer(
            1.0 / control_rate_hz,
            self.control_steering,
        )
        self.status_timer = self.create_timer(
            1.0 / status_rate_hz,
            self.report_status,
        )

    def _validate_parameters(
        self,
        control_rate_hz: float,
        status_rate_hz: float,
    ) -> None:
        servo_positions = (self.servo_left, self.servo_center, self.servo_right)
        if any(not 0.0 <= value <= 1.0 for value in servo_positions):
            raise ValueError("servo positions must be between 0.0 and 1.0")
        if not self.servo_left < self.servo_center < self.servo_right:
            raise ValueError("expected servo_left < servo_center < servo_right")
        if self.settings.track_width <= self.settings.vehicle_width:
            raise ValueError("track_width_m must be greater than vehicle_width_m")
        if not (
            0.0 < self.settings.emergency_stop_distance
            < self.settings.full_corner_distance
            < self.settings.avoidance_start_distance
        ):
            raise ValueError(
                "expected emergency_stop_distance_m < full_corner_distance_m "
                "< avoidance_start_distance_m"
            )
        if self.settings.physical_maximum_steering_angle <= 0.0:
            raise ValueError("physical_max_steering_angle_degrees must be positive")
        if self.settings.wheelbase <= 0.0 or self.settings.target_corner_radius <= 0.0:
            raise ValueError("wheelbase_m and target_corner_radius_m must be positive")
        if self.servo_rate_limit <= 0.0 or self.urgent_servo_rate_limit <= 0.0:
            raise ValueError("servo rate limits must be positive")
        if self.urgent_distance <= self.settings.emergency_stop_distance:
            raise ValueError("urgent_distance_m must exceed emergency stop distance")
        if self.scan_timeout <= 0.0:
            raise ValueError("scan_timeout_seconds must be positive")
        if control_rate_hz <= 0.0 or status_rate_hz <= 0.0:
            raise ValueError("control and status rates must be positive")

    def _write_servo(self, position: float) -> bool:
        """Send one servo-only command and latch communication failures."""
        if self.serial_fault is not None:
            return False
        try:
            position = clamp(position, 0.0, 1.0)
            packet = pyvesc.encode(SetServoPosition(position))
            self.connection.write(packet)
            self.connection.flush()
            self.current_servo_position = position
            return True
        except (serial.SerialException, OSError) as error:
            self.serial_fault = str(error)
            self.get_logger().fatal(
                f"VESC serial fault: {error}. Steering commands stopped."
            )
            return False

    def _center_immediately(self) -> None:
        self.last_control_time = time.monotonic()
        self.previous_turn_sign = 0
        self._write_servo(self.servo_center)

    def scan_callback(self, scan: LaserScan) -> None:
        self.last_scan_time = time.monotonic()
        try:
            decision = plan_adaptive_corridor_steering(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                scan.range_min,
                scan.range_max,
                self.settings,
                self.previous_turn_sign,
            )
            self.latest_decision = decision
            self.planner_fault = None
            if decision.safe and decision.mode == "AVOID":
                self.previous_turn_sign = +1 if decision.direction == "LEFT" else -1
            elif decision.front_distance is None:
                self.previous_turn_sign = 0
        except Exception as error:
            self.latest_decision = None
            self.planner_fault = str(error)
            self._center_immediately()
            self.get_logger().error(
                f"Planner error: {error}. Steering centered."
            )

    def _scan_age(self) -> Optional[float]:
        if self.last_scan_time is None:
            return None
        return time.monotonic() - self.last_scan_time

    def control_steering(self) -> None:
        """Command adaptive steering or center immediately on any unsafe state."""
        if self.serial_fault is not None:
            return
        scan_age = self._scan_age()
        if (
            scan_age is None
            or scan_age > self.scan_timeout
            or self.planner_fault is not None
            or self.latest_decision is None
            or not self.latest_decision.safe
            or self.latest_decision.steering_angle is None
        ):
            self._center_immediately()
            return

        decision = self.latest_decision
        target_position = steering_angle_to_servo_position(
            decision.steering_angle,
            self.settings.physical_maximum_steering_angle,
            self.servo_center,
            self.servo_left,
            self.servo_right,
        )
        now = time.monotonic()
        elapsed = min(max(now - self.last_control_time, 0.0), 0.25)
        self.last_control_time = now
        urgent = (
            decision.front_distance is not None
            and decision.front_distance <= self.urgent_distance
        )
        rate_limit = (
            self.urgent_servo_rate_limit if urgent else self.servo_rate_limit
        )
        command = rate_limited_position(
            self.current_servo_position,
            target_position,
            rate_limit * elapsed,
        )
        self._write_servo(command)

    @staticmethod
    def _format_distance(value: Optional[float]) -> str:
        return "--" if value is None else f"{value:.2f} m"

    @staticmethod
    def _format_radius(value: Optional[float]) -> str:
        if value is None:
            return "straight"
        return f"{value:.2f} m/{value / FEET_TO_METERS:.1f} ft"

    def report_status(self) -> None:
        scan_age = self._scan_age()
        if self.serial_fault is not None:
            self.get_logger().fatal(
                f"SERIAL FAULT | {self.serial_fault} | no further commands"
            )
            return
        if scan_age is None:
            self.get_logger().warning("CENTERED | waiting for LiDAR")
            return
        if scan_age > self.scan_timeout:
            self.get_logger().warning(
                f"CENTERED | stale LiDAR ({scan_age:.3f} s old)"
            )
            return
        if self.planner_fault is not None:
            self.get_logger().warning(
                f"CENTERED | planner fault: {self.planner_fault}"
            )
            return
        decision = self.latest_decision
        if decision is None:
            self.get_logger().warning("CENTERED | no planner decision")
            return
        if not decision.safe:
            self.get_logger().warning(
                f"CENTERED | {decision.reason} | front "
                f"{self._format_distance(decision.front_distance)} | "
                "STEERING-ONLY TEST"
            )
            return

        angle_degrees = math.degrees(decision.steering_angle or 0.0)
        center_error = (
            "--"
            if decision.center_error is None
            else f"{decision.center_error:+.3f} m"
        )
        self.get_logger().info(
            f"{decision.mode:<8} {decision.direction:<8} | steer "
            f"{angle_degrees:+5.1f} deg | servo "
            f"{self.current_servo_position:.3f} | front "
            f"{self._format_distance(decision.front_distance)} | walls L/R "
            f"{self._format_distance(decision.left_wall_distance)}/"
            f"{self._format_distance(decision.right_wall_distance)} | "
            f"center error {center_error} | predicted radius "
            f"{self._format_radius(decision.estimated_radius)} | "
            "STEERING-ONLY TEST"
        )

    def close(self) -> None:
        """Best-effort repeated center and close; safe to call repeatedly."""
        if self._closed:
            return
        self._closed = True
        if hasattr(self, "connection") and self.connection.is_open:
            self.serial_fault = None
            for _ in range(5):
                try:
                    packet = pyvesc.encode(SetServoPosition(self.servo_center))
                    self.connection.write(packet)
                    self.connection.flush()
                    self.current_servo_position = self.servo_center
                    time.sleep(0.05)
                except (serial.SerialException, OSError):
                    break
            self.connection.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[SL450AdaptiveCorridorSteeringTest] = None
    try:
        node = SL450AdaptiveCorridorSteeringTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (serial.SerialException, OSError, ValueError) as error:
        print(f"Adaptive steering test could not start: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

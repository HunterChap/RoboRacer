#!/usr/bin/env python3
"""Interactive low-speed autonomous controller for the RoboRacer.

This is the first FLOOR-DRIVING controller for the current car.  It combines:

* Orbbec SL450 ``LaserScan`` data from ``/lidar/scan/points``.
* Track-aware centering and obstacle/corner selection.
* VESC servo command 12 and positive duty-cycle motor commands.
* Interactive quick/advanced tuning, reusable profiles, and CSV logs.
* Pre-arm LiDAR checks, an explicit live-run confirmation, a software watchdog,
  a maximum run timer, and latched fault stops.

The motor is disabled by default.  This first version has immutable limits of
10% forward duty, servo 0.250--0.750, and 60 seconds per run.  It never reverses
or commands active braking.  A safety stop sends zero duty immediately; zero
duty may still allow the car to coast.

Run on the Raspberry Pi:

    source /opt/ros/jazzy/setup.bash
    source ~/orbbec_ws/install/setup.bash
    source ~/venv_vesc/bin/activate
    python3 ~/sl450_interactive_autonomous_drive.py

Dependency-free checks (safe to run on any computer):

    python3 sl450_interactive_autonomous_drive.py --self-test
    python3 sl450_interactive_autonomous_drive.py --simulate

Before any LIVE run, independently configure and raised-wheel-test the VESC
App Settings safety timeout.  Python cannot guarantee a stop after process,
USB, Pi-power, or kernel failure.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass, fields, replace
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import statistics
import sys
import threading
import time
from typing import Any, Optional, Sequence


SCRIPT_VERSION = "1.0.0"

# Immutable first-version safety boundaries.  Interactive values can be made
# more conservative, but cannot exceed these limits without editing the file.
HARD_MINIMUM_SERVO_POSITION = 0.200
HARD_MAXIMUM_SERVO_POSITION = 0.800
HARD_MAXIMUM_FORWARD_DUTY = 10
HARD_MAXIMUM_RUN_SECONDS = 120.0
HARD_MINIMUM_EMERGENCY_DISTANCE_M = 0.05

INCHES_TO_METERS = 0.0254
FEET_TO_METERS = 0.3048


def clamp(value: float, lower: float, upper: float) -> float:
    """Limit value to the closed interval [lower, upper]."""
    return max(lower, min(value, upper))


def normalize_angle(angle_radians: float) -> float:
    """Return an angle in [-pi, pi)."""
    return (angle_radians + math.pi) % (2.0 * math.pi) - math.pi


def percentile(values: Sequence[float], fraction: float) -> float:
    """Return a linearly interpolated percentile without NumPy."""
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


def duty_to_vesc_value(duty: float) -> int:
    """Convert normalized duty to the integer representation pyvesc expects."""
    if not -1.0 <= duty <= 1.0:
        raise ValueError("duty must be between -1.0 and 1.0")
    return int(round(duty * 100_000.0))


def rate_limited_value(
    current: float,
    target: float,
    maximum_change: float,
) -> float:
    """Move current toward target by no more than maximum_change."""
    if maximum_change < 0.0:
        raise ValueError("maximum_change cannot be negative")
    return current + clamp(target - current, -maximum_change, maximum_change)


def ramp_duty(
    current: float,
    target: float,
    elapsed_seconds: float,
    acceleration_rate: float,
    deceleration_rate: float,
) -> float:
    """Slew normal duty changes; safety stops bypass this and command zero."""
    if elapsed_seconds < 0.0:
        raise ValueError("elapsed_seconds cannot be negative")
    if acceleration_rate <= 0.0 or deceleration_rate <= 0.0:
        raise ValueError("duty ramp rates must be positive")
    rate = acceleration_rate if target > current else deceleration_rate
    return rate_limited_value(current, target, rate * elapsed_seconds)


def steering_angle_to_servo_position(
    steering_angle: float,
    physical_maximum_steering_angle: float,
    center_position: float,
    left_position: float,
    right_position: float,
) -> float:
    """Map a road-wheel angle to the verified RoboRacer servo convention.

    Positive road-wheel angles mean left.  Servo values below center turn the
    car left; values above center turn it right.
    """
    if physical_maximum_steering_angle <= 0.0:
        raise ValueError("physical maximum steering angle must be positive")
    fraction = clamp(
        steering_angle / physical_maximum_steering_angle,
        -1.0,
        1.0,
    )
    if fraction >= 0.0:
        return center_position + fraction * (left_position - center_position)
    return center_position + (-fraction) * (right_position - center_position)


def steering_angle_for_radius(wheelbase: float, radius: float) -> float:
    """Return bicycle-model road-wheel angle for a centerline radius."""
    if wheelbase <= 0.0 or radius <= 0.0:
        raise ValueError("wheelbase and radius must be positive")
    return math.atan(wheelbase / radius)


def estimated_turn_radius(
    wheelbase: float,
    steering_angle: float,
) -> Optional[float]:
    """Return bicycle-model radius, or None for effectively straight travel."""
    if wheelbase <= 0.0:
        raise ValueError("wheelbase must be positive")
    tangent = math.tan(abs(steering_angle))
    if tangent < 1e-6:
        return None
    return wheelbase / tangent


@dataclass
class DriveConfig:
    """All operator-tunable values, with conservative first-run defaults."""

    profile_name: str = "conservative_first_run"
    run_mode: str = "DRY"

    # Hardware interface and verified orientation.
    scan_topic: str = "/lidar/scan/points"
    serial_port: str = "/dev/ttyACM0"
    baud_rate: int = 115200
    forward_raw_angle_degrees: float = 180.0

    # Steering calibration and behavior.
    servo_center: float = 0.500
    servo_left_limit: float = 0.400
    servo_right_limit: float = 0.600
    turning_aggression: float = 0.70
    steering_deadband_degrees: float = 1.0
    servo_rate_limit_per_second: float = 0.50
    urgent_servo_rate_limit_per_second: float = 1.00
    turn_commit_seconds: float = 0.40

    # Duty-cycle speed behavior.  Values are normalized: 0.05 means 5%.
    maximum_straight_duty: float = 0.05
    minimum_moving_duty: float = 0.03
    minimum_corner_duty: float = 0.035
    corner_slowdown_strength: float = 0.80
    acceleration_rate_duty_per_second: float = 0.04
    deceleration_rate_duty_per_second: float = 0.08

    # Vehicle and track geometry.
    track_width_m: float = 25.0 * INCHES_TO_METERS
    track_width_tolerance_m: float = 0.15
    vehicle_width_m: float = 0.32
    obstacle_safety_margin_m: float = 0.04
    minimum_usable_gap_m: float = 0.40
    lidar_lateral_offset_m: float = 0.0
    wheelbase_m: float = 0.324
    target_corner_radius_m: float = 3.0 * FEET_TO_METERS
    physical_maximum_steering_angle_degrees: float = 20.0

    # Planner tuning.
    planning_half_fov_degrees: float = 105.0
    maximum_considered_range_m: float = 6.0
    minimum_valid_fraction: float = 0.65
    centering_deadband_m: float = 0.025
    centering_full_error_m: float = 0.12
    maximum_centering_angle_degrees: float = 8.0
    avoidance_start_distance_m: float = 1.50
    slowdown_distance_m: float = 1.40
    full_corner_distance_m: float = 0.90
    urgent_distance_m: float = 0.85
    emergency_stop_distance_m: float = 0.45
    minimum_gap_check_distance_m: float = 0.75
    minimum_obstacle_cluster_points: int = 5
    maximum_cluster_angle_gap_degrees: float = 0.45
    maximum_cluster_forward_jump_m: float = 0.20
    direction_score_deadband_m: float = 0.08
    obstacle_side_deadband_m: float = 0.025

    # Runtime and safety timing.
    scan_timeout_seconds: float = 0.25
    safe_scan_arm_seconds: float = 2.0
    countdown_seconds: float = 3.0
    maximum_run_seconds: float = 15.0
    software_watchdog_seconds: float = 0.35
    control_rate_hz: float = 20.0
    status_rate_hz: float = 4.0


def validate_config(config: DriveConfig) -> list[str]:
    """Return every configuration error instead of failing on only the first."""
    errors: list[str] = []

    if config.run_mode not in {"DRY", "LIVE"}:
        errors.append("run_mode must be DRY or LIVE")
    if not config.scan_topic.startswith("/"):
        errors.append("scan_topic must be an absolute ROS topic beginning with /")
    if not config.serial_port.startswith("/"):
        errors.append("serial_port must be an absolute device path")
    if config.baud_rate <= 0:
        errors.append("baud_rate must be positive")

    if not (
        HARD_MINIMUM_SERVO_POSITION
        <= config.servo_left_limit
        < config.servo_center
        < config.servo_right_limit
        <= HARD_MAXIMUM_SERVO_POSITION
    ):
        errors.append(
            "servo positions must satisfy 0.250 <= left < center < right <= 0.750"
        )
    if not 0.10 <= config.turning_aggression <= 1.50:
        errors.append("turning_aggression must be between 0.10 and 1.50")
    if not 0.0 <= config.steering_deadband_degrees <= 5.0:
        errors.append("steering_deadband_degrees must be between 0 and 5")
    if config.servo_rate_limit_per_second <= 0.0:
        errors.append("servo_rate_limit_per_second must be positive")
    if (
        config.urgent_servo_rate_limit_per_second
        < config.servo_rate_limit_per_second
    ):
        errors.append("urgent servo rate must be at least the normal servo rate")
    if not 0.0 <= config.turn_commit_seconds <= 2.0:
        errors.append("turn_commit_seconds must be between 0 and 2 seconds")

    if not 0.0 < config.maximum_straight_duty <= HARD_MAXIMUM_FORWARD_DUTY:
        errors.append("maximum_straight_duty must be above 0 and no more than 10%")
    if not (
        0.0
        < config.minimum_moving_duty
        <= config.minimum_corner_duty
        <= config.maximum_straight_duty
    ):
        errors.append(
            "duty values must satisfy 0 < minimum moving <= minimum corner "
            "<= maximum straight"
        )
    if not 0.0 <= config.corner_slowdown_strength <= 1.0:
        errors.append("corner_slowdown_strength must be between 0 and 1")
    if config.acceleration_rate_duty_per_second <= 0.0:
        errors.append("acceleration rate must be positive")
    if config.deceleration_rate_duty_per_second <= 0.0:
        errors.append("deceleration rate must be positive")

    if config.vehicle_width_m <= 0.0:
        errors.append("vehicle_width_m must be positive")
    if config.track_width_m <= config.vehicle_width_m:
        errors.append("track_width_m must exceed vehicle_width_m")
    required_gap = (
        config.vehicle_width_m + 2.0 * config.obstacle_safety_margin_m
    )
    if config.minimum_usable_gap_m + 1e-9 < required_gap:
        errors.append(
            "minimum_usable_gap_m must be at least vehicle width plus twice "
            f"the safety margin ({required_gap:.3f} m)"
        )
    if config.minimum_usable_gap_m > config.track_width_m:
        errors.append("minimum_usable_gap_m cannot exceed track_width_m")
    if config.track_width_tolerance_m <= 0.0:
        errors.append("track_width_tolerance_m must be positive")
    if config.wheelbase_m <= 0.0 or config.target_corner_radius_m <= 0.0:
        errors.append("wheelbase and target corner radius must be positive")
    if config.physical_maximum_steering_angle_degrees <= 0.0:
        errors.append("physical maximum steering angle must be positive")

    if not 30.0 <= config.planning_half_fov_degrees <= 130.0:
        errors.append("planning_half_fov_degrees must be between 30 and 130")
    if config.maximum_considered_range_m <= 0.0:
        errors.append("maximum_considered_range_m must be positive")
    if not 0.50 <= config.minimum_valid_fraction <= 1.0:
        errors.append("minimum_valid_fraction must be between 0.50 and 1.00")
    if not 0.0 <= config.centering_deadband_m < config.centering_full_error_m:
        errors.append("centering deadband must be nonnegative and below full error")
    if config.maximum_centering_angle_degrees < 0.0:
        errors.append("maximum centering angle cannot be negative")
    if not (
        HARD_MINIMUM_EMERGENCY_DISTANCE_M
        <= config.emergency_stop_distance_m
        < config.urgent_distance_m
        <= config.full_corner_distance_m
        <= config.slowdown_distance_m
        < config.avoidance_start_distance_m
    ):
        errors.append(
            "distances must satisfy 0.30 <= emergency < urgent <= full corner "
            "<= slowdown < avoidance start"
        )
    if config.minimum_gap_check_distance_m <= config.emergency_stop_distance_m:
        errors.append("minimum gap check distance must exceed emergency distance")
    if config.minimum_obstacle_cluster_points < 3:
        errors.append("minimum obstacle cluster points must be at least 3")
    if config.maximum_cluster_angle_gap_degrees <= 0.0:
        errors.append("maximum cluster angle gap must be positive")
    if config.maximum_cluster_forward_jump_m <= 0.0:
        errors.append("maximum cluster forward jump must be positive")

    if not 0.10 <= config.scan_timeout_seconds <= 1.0:
        errors.append("scan_timeout_seconds must be between 0.10 and 1.0")
    if config.safe_scan_arm_seconds < 1.0:
        errors.append("safe_scan_arm_seconds must be at least 1 second")
    if not 1.0 <= config.countdown_seconds <= 10.0:
        errors.append("countdown_seconds must be between 1 and 10")
    if not 1.0 <= config.maximum_run_seconds <= HARD_MAXIMUM_RUN_SECONDS:
        errors.append("maximum_run_seconds must be between 1 and 60")
    if config.control_rate_hz < 10.0:
        errors.append("control_rate_hz must be at least 10 Hz")
    if not 1.0 <= config.status_rate_hz <= config.control_rate_hz:
        errors.append("status_rate_hz must be from 1 Hz through the control rate")
    if not (
        2.0 / config.control_rate_hz
        <= config.software_watchdog_seconds
        <= 0.50
    ):
        errors.append(
            "software watchdog must be at least two control periods and no "
            "more than 0.50 seconds"
        )
    return errors


@dataclass(frozen=True)
class ScanPoint:
    index: int
    angle: float
    distance: float
    forward: float
    left: float


@dataclass(frozen=True)
class PlannerSettings:
    forward_raw_angle: float
    planning_half_fov: float
    maximum_considered_range: float
    minimum_valid_fraction: float
    track_width: float
    track_width_tolerance: float
    vehicle_width: float
    obstacle_safety_margin: float
    minimum_usable_gap: float
    lidar_lateral_offset: float
    side_sector_min_angle: float
    side_sector_max_angle: float
    side_wall_max_distance: float
    side_minimum_points: int
    centering_deadband: float
    centering_full_error: float
    maximum_centering_angle: float
    front_minimum_forward: float
    avoidance_start_distance: float
    full_corner_distance: float
    emergency_stop_distance: float
    minimum_gap_check_distance: float
    minimum_obstacle_cluster_points: int
    maximum_cluster_angle_gap: float
    maximum_cluster_forward_jump: float
    opening_sector_min_angle: float
    opening_sector_max_angle: float
    gap_sector_forward_overlap: float
    gap_sector_max_angle: float
    opening_score_percentile: float
    opening_score_cap: float
    direction_score_deadband: float
    obstacle_side_deadband: float
    wheelbase: float
    target_corner_radius: float
    physical_maximum_steering_angle: float


@dataclass(frozen=True)
class PlannerDecision:
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
    left_gap_width: Optional[float]
    right_gap_width: Optional[float]
    estimated_radius: Optional[float]
    valid_fraction: float


def planner_settings_from_config(config: DriveConfig) -> PlannerSettings:
    return PlannerSettings(
        forward_raw_angle=math.radians(config.forward_raw_angle_degrees),
        planning_half_fov=math.radians(config.planning_half_fov_degrees),
        maximum_considered_range=config.maximum_considered_range_m,
        minimum_valid_fraction=config.minimum_valid_fraction,
        track_width=config.track_width_m,
        track_width_tolerance=config.track_width_tolerance_m,
        vehicle_width=config.vehicle_width_m,
        obstacle_safety_margin=config.obstacle_safety_margin_m,
        minimum_usable_gap=config.minimum_usable_gap_m,
        lidar_lateral_offset=config.lidar_lateral_offset_m,
        side_sector_min_angle=math.radians(65.0),
        side_sector_max_angle=math.radians(88.0),
        side_wall_max_distance=0.60,
        side_minimum_points=8,
        centering_deadband=config.centering_deadband_m,
        centering_full_error=config.centering_full_error_m,
        maximum_centering_angle=math.radians(
            config.maximum_centering_angle_degrees
        ),
        front_minimum_forward=0.08,
        avoidance_start_distance=config.avoidance_start_distance_m,
        full_corner_distance=config.full_corner_distance_m,
        emergency_stop_distance=config.emergency_stop_distance_m,
        minimum_gap_check_distance=config.minimum_gap_check_distance_m,
        minimum_obstacle_cluster_points=config.minimum_obstacle_cluster_points,
        maximum_cluster_angle_gap=math.radians(
            config.maximum_cluster_angle_gap_degrees
        ),
        maximum_cluster_forward_jump=config.maximum_cluster_forward_jump_m,
        opening_sector_min_angle=math.radians(8.0),
        opening_sector_max_angle=math.radians(55.0),
        gap_sector_forward_overlap=math.radians(5.0),
        gap_sector_max_angle=math.radians(65.0),
        opening_score_percentile=0.35,
        opening_score_cap=3.0,
        direction_score_deadband=config.direction_score_deadband_m,
        obstacle_side_deadband=config.obstacle_side_deadband_m,
        wheelbase=config.wheelbase_m,
        target_corner_radius=config.target_corner_radius_m,
        physical_maximum_steering_angle=math.radians(
            config.physical_maximum_steering_angle_degrees
        ),
    )


def _scan_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    settings: PlannerSettings,
) -> tuple[list[ScanPoint], float, float]:
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
        finite_valid = (
            math.isfinite(measured_range)
            and range_min <= measured_range <= range_max
        )
        clear_return = math.isinf(measured_range) and measured_range > 0.0
        if not (finite_valid or clear_return):
            continue
        valid_count += 1
        distance = (
            min(float(measured_range), settings.maximum_considered_range)
            if finite_valid
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
        valid_count / considered_count if considered_count else 0.0
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
    settings: PlannerSettings,
) -> Optional[float]:
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
    settings: PlannerSettings,
) -> tuple[Optional[float], Optional[float], Optional[float]]:
    left_sensor = _estimate_side_wall(points, +1, settings)
    right_sensor = _estimate_side_wall(points, -1, settings)
    if left_sensor is None or right_sensor is None:
        return left_sensor, right_sensor, None
    left_car_center = left_sensor + settings.lidar_lateral_offset
    right_car_center = right_sensor - settings.lidar_lateral_offset
    observed_width = left_car_center + right_car_center
    if abs(observed_width - settings.track_width) > settings.track_width_tolerance:
        return left_car_center, right_car_center, None
    # Positive means more room on the left, so steer left.
    return left_car_center, right_car_center, left_car_center - right_car_center


def _front_clusters(
    points: Sequence[ScanPoint],
    angular_step: float,
    settings: PlannerSettings,
) -> list[list[ScanPoint]]:
    half_lane_width = (
        settings.vehicle_width / 2.0 + settings.obstacle_safety_margin
    )
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
    settings: PlannerSettings,
) -> tuple[Optional[float], Optional[float]]:
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
        lateral_center = float(
            statistics.median(point.left for point in near_points)
        )
        summaries.append((forward_distance, lateral_center))
    return min(summaries, key=lambda summary: summary[0])


def _opening_score(
    points: Sequence[ScanPoint],
    side_sign: int,
    settings: PlannerSettings,
) -> Optional[float]:
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


def _usable_gap_width(
    points: Sequence[ScanPoint],
    angular_step: float,
    side_sign: int,
    settings: PlannerSettings,
) -> Optional[float]:
    """Estimate the widest visible aperture toward one side.

    Rays must remain clear for ``minimum_gap_check_distance``.  The angular
    aperture is converted to a conservative chord width at that distance.
    This is a screening check, not a full vehicle-dynamics trajectory test.
    """
    candidates = sorted(
        (
            (side_sign * point.angle, point)
            for point in points
            if -settings.gap_sector_forward_overlap
            <= side_sign * point.angle
            <= settings.gap_sector_max_angle
            and point.forward > 0.05
            and point.distance >= settings.minimum_gap_check_distance
        ),
        key=lambda pair: pair[0],
    )
    if not candidates:
        return None
    allowed_gap = max(2.5 * angular_step, math.radians(0.75))
    runs: list[list[tuple[float, ScanPoint]]] = []
    current = [candidates[0]]
    for candidate in candidates[1:]:
        if candidate[0] - current[-1][0] <= allowed_gap:
            current.append(candidate)
        else:
            runs.append(current)
            current = [candidate]
    runs.append(current)

    widths: list[float] = []
    for run in runs:
        if len(run) < settings.side_minimum_points:
            continue
        angular_width = max(0.0, run[-1][0] - run[0][0] + angular_step)
        widths.append(
            2.0
            * settings.minimum_gap_check_distance
            * math.sin(angular_width / 2.0)
        )
    return max(widths) if widths else None


def _choose_turn_sign(
    obstacle_lateral_position: float,
    left_score: Optional[float],
    right_score: Optional[float],
    left_gap_width: Optional[float],
    right_gap_width: Optional[float],
    center_error: Optional[float],
    previous_turn_sign: int,
    settings: PlannerSettings,
) -> Optional[int]:
    left_usable = (
        left_gap_width is not None
        and left_gap_width >= settings.minimum_usable_gap
    )
    right_usable = (
        right_gap_width is not None
        and right_gap_width >= settings.minimum_usable_gap
    )
    if left_usable and not right_usable:
        return +1
    if right_usable and not left_usable:
        return -1
    if not left_usable and not right_usable:
        return None

    # Both sides pass the gap screen.  Prefer turning away from a clearly
    # off-center obstacle, then compare opening depth.
    if obstacle_lateral_position > settings.obstacle_side_deadband:
        return -1
    if obstacle_lateral_position < -settings.obstacle_side_deadband:
        return +1
    if left_score is not None and right_score is not None:
        difference = left_score - right_score
        if difference > settings.direction_score_deadband:
            return +1
        if difference < -settings.direction_score_deadband:
            return -1
    if previous_turn_sign in (-1, +1):
        return previous_turn_sign
    if center_error is not None and abs(center_error) > settings.centering_deadband:
        return +1 if center_error > 0.0 else -1
    return +1


def _avoidance_magnitude(
    front_distance: float,
    settings: PlannerSettings,
) -> float:
    corner_angle = min(
        steering_angle_for_radius(
            settings.wheelbase,
            settings.target_corner_radius,
        ),
        settings.physical_maximum_steering_angle,
    )
    if front_distance >= settings.avoidance_start_distance:
        return 0.0
    if front_distance >= settings.full_corner_distance:
        span = settings.avoidance_start_distance - settings.full_corner_distance
        progress = (settings.avoidance_start_distance - front_distance) / span
        return corner_angle * math.sqrt(clamp(progress, 0.0, 1.0))
    urgent_span = (
        settings.full_corner_distance - settings.emergency_stop_distance
    )
    urgent_progress = (
        settings.full_corner_distance - front_distance
    ) / urgent_span
    return corner_angle + clamp(urgent_progress, 0.0, 1.0) * (
        settings.physical_maximum_steering_angle - corner_angle
    )


def _centering_angle(
    center_error: Optional[float],
    settings: PlannerSettings,
) -> float:
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


def plan_adaptive_drive(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    settings: PlannerSettings,
    previous_turn_sign: int = 0,
) -> PlannerDecision:
    """Analyze one scan; this pure function never sends hardware commands."""
    points, valid_fraction, angular_step = _scan_points(
        ranges,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        settings,
    )
    empty = (None,) * 11
    if angular_step <= 0.0 or not points:
        return PlannerDecision(
            False,
            "STOP",
            "STOP",
            "invalid scan geometry",
            *empty,
            valid_fraction,
        )
    if valid_fraction < settings.minimum_valid_fraction:
        return PlannerDecision(
            False,
            "STOP",
            "STOP",
            f"only {valid_fraction:.0%} of the planning scan is valid",
            *empty,
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
    left_gap = _usable_gap_width(points, angular_step, +1, settings)
    right_gap = _usable_gap_width(points, angular_step, -1, settings)

    if (
        front_distance is not None
        and front_distance <= settings.emergency_stop_distance
    ):
        return PlannerDecision(
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
            left_gap,
            right_gap,
            None,
            valid_fraction,
        )

    if front_distance is not None and obstacle_lateral is not None:
        turn_sign = _choose_turn_sign(
            obstacle_lateral,
            left_score,
            right_score,
            left_gap,
            right_gap,
            center_error,
            previous_turn_sign,
            settings,
        )
        if turn_sign is None:
            left_text = "--" if left_gap is None else f"{left_gap:.2f} m"
            right_text = "--" if right_gap is None else f"{right_gap:.2f} m"
            return PlannerDecision(
                False,
                "STOP",
                "STOP",
                (
                    "no usable opening: estimated gaps L/R "
                    f"{left_text}/{right_text}, need "
                    f"{settings.minimum_usable_gap:.2f} m"
                ),
                None,
                front_distance,
                obstacle_lateral,
                left_wall,
                right_wall,
                center_error,
                left_score,
                right_score,
                left_gap,
                right_gap,
                None,
                valid_fraction,
            )
        steering_angle = turn_sign * _avoidance_magnitude(
            front_distance,
            settings,
        )
        direction = "LEFT" if turn_sign > 0 else "RIGHT"
        return PlannerDecision(
            True,
            "AVOID",
            direction,
            "front-lane obstacle; selected a usable opening",
            steering_angle,
            front_distance,
            obstacle_lateral,
            left_wall,
            right_wall,
            center_error,
            left_score,
            right_score,
            left_gap,
            right_gap,
            estimated_turn_radius(settings.wheelbase, steering_angle),
            valid_fraction,
        )

    steering_angle = _centering_angle(center_error, settings)
    if steering_angle > math.radians(0.5):
        mode, direction, reason = (
            "CENTER",
            "LEFT",
            "too close to right track wall",
        )
    elif steering_angle < -math.radians(0.5):
        mode, direction, reason = (
            "CENTER",
            "RIGHT",
            "too close to left track wall",
        )
    else:
        mode, direction = "STRAIGHT", "STRAIGHT"
        reason = (
            "track centered"
            if center_error is not None
            else "no front obstacle; side walls ignored"
        )
    return PlannerDecision(
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
        left_gap,
        right_gap,
        estimated_turn_radius(settings.wheelbase, steering_angle),
        valid_fraction,
    )


def desired_drive_duty(
    config: DriveConfig,
    servo_target: float,
    front_distance: Optional[float],
) -> float:
    """Schedule forward duty from steering and obstacle proximity."""
    if servo_target <= config.servo_center:
        available_travel = config.servo_center - config.servo_left_limit
    else:
        available_travel = config.servo_right_limit - config.servo_center
    turn_fraction = clamp(
        abs(servo_target - config.servo_center) / max(available_travel, 1e-9),
        0.0,
        1.0,
    )
    proximity_fraction = 0.0
    if front_distance is not None:
        span = (
            config.slowdown_distance_m - config.emergency_stop_distance_m
        )
        proximity_fraction = clamp(
            (config.slowdown_distance_m - front_distance) / span,
            0.0,
            1.0,
        )
    requested_slowdown = max(turn_fraction, proximity_fraction)
    effective_slowdown = requested_slowdown * config.corner_slowdown_strength
    target = config.maximum_straight_duty - effective_slowdown * (
        config.maximum_straight_duty - config.minimum_corner_duty
    )
    return clamp(
        target,
        config.minimum_moving_duty,
        config.maximum_straight_duty,
    )


def _synthetic_corridor_scan(
    settings: PlannerSettings,
    *,
    sensor_offset_left: float = 0.0,
    obstacle_forward: Optional[float] = None,
    obstacle_center_left: float = 0.0,
    obstacle_width: float = 0.16,
) -> tuple[list[float], float, float]:
    """Ray-cast a simple corridor for dependency-free safety tests."""
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
    """Exercise numeric conversion, planner paths, speed, and safety bounds."""
    config = DriveConfig()
    assert validate_config(config) == []
    assert isinstance(duty_to_vesc_value(0.05), int)
    assert duty_to_vesc_value(0.05) == 5_000
    assert duty_to_vesc_value(0.10) == 10_000
    assert math.isclose(
        ramp_duty(0.0, 0.05, 0.5, 0.04, 0.08),
        0.02,
    )
    assert math.isclose(
        ramp_duty(0.05, 0.0, 0.25, 0.04, 0.08),
        0.03,
    )

    settings = planner_settings_from_config(config)
    centered, angle_min, increment = _synthetic_corridor_scan(settings)
    decision = plan_adaptive_drive(
        centered,
        angle_min,
        increment,
        0.05,
        30.0,
        settings,
    )
    assert decision.safe and decision.mode == "STRAIGHT"

    shifted_right, angle_min, increment = _synthetic_corridor_scan(
        settings,
        sensor_offset_left=-0.08,
    )
    decision = plan_adaptive_drive(
        shifted_right,
        angle_min,
        increment,
        0.05,
        30.0,
        settings,
    )
    assert decision.safe and decision.mode == "CENTER"
    assert decision.direction == "LEFT"

    # Full front wall with no side opening must stop rather than guess.
    blocked, angle_min, increment = _synthetic_corridor_scan(
        settings,
        obstacle_forward=0.70,
        obstacle_center_left=0.0,
        obstacle_width=settings.track_width,
    )
    decision = plan_adaptive_drive(
        blocked,
        angle_min,
        increment,
        0.05,
        30.0,
        settings,
    )
    assert not decision.safe and "no usable opening" in decision.reason

    # Same wall with a visible right opening must select the right turn.
    corner = list(blocked)
    for index in range(len(corner)):
        car_angle = normalize_angle(
            angle_min + index * increment - settings.forward_raw_angle
        )
        if math.radians(-60.0) <= car_angle <= math.radians(-8.0):
            corner[index] = 2.5
    decision = plan_adaptive_drive(
        corner,
        angle_min,
        increment,
        0.05,
        30.0,
        settings,
    )
    assert decision.safe and decision.mode == "AVOID"
    assert decision.direction == "RIGHT"
    assert decision.right_gap_width is not None
    assert decision.right_gap_width >= settings.minimum_usable_gap

    emergency, angle_min, increment = _synthetic_corridor_scan(
        settings,
        obstacle_forward=0.30,
        obstacle_center_left=0.0,
        obstacle_width=0.18,
    )
    decision = plan_adaptive_drive(
        emergency,
        angle_min,
        increment,
        0.05,
        30.0,
        settings,
    )
    assert not decision.safe and "emergency distance" in decision.reason

    left_servo = steering_angle_to_servo_position(
        math.radians(20.0),
        math.radians(20.0),
        config.servo_center,
        config.servo_left_limit,
        config.servo_right_limit,
    )
    right_servo = steering_angle_to_servo_position(
        math.radians(-20.0),
        math.radians(20.0),
        config.servo_center,
        config.servo_left_limit,
        config.servo_right_limit,
    )
    assert math.isclose(left_servo, config.servo_left_limit)
    assert math.isclose(right_servo, config.servo_right_limit)
    straight_duty = desired_drive_duty(config, config.servo_center, None)
    corner_duty = desired_drive_duty(config, left_servo, 0.70)
    assert math.isclose(straight_duty, config.maximum_straight_duty)
    assert config.minimum_corner_duty <= corner_duty < straight_duty

    unsafe = replace(config, maximum_straight_duty=0.11)
    assert any("10%" in error for error in validate_config(unsafe))
    unsafe = replace(config, servo_left_limit=0.20)
    assert any("servo positions" in error for error in validate_config(unsafe))
    unsafe = replace(config, emergency_stop_distance_m=0.20)
    assert any("distances" in error for error in validate_config(unsafe))

    encoded = asdict(config)
    round_trip = DriveConfig(**encoded)
    assert round_trip == config

    print(
        "Self-test passed: integer VESC duty encoding, steering direction, "
        "duty ramps, centered corridor, wall centering, no-route STOP, "
        "right-corner selection, emergency STOP, speed scheduling, profile "
        "round-trip, and immutable safety ceilings."
    )


def run_simulation() -> None:
    """Print representative planner outputs without ROS or vehicle hardware."""
    config = DriveConfig()
    settings = planner_settings_from_config(config)
    scenarios: list[tuple[str, list[float], float, float]] = []

    scan, angle_min, increment = _synthetic_corridor_scan(settings)
    scenarios.append(("Centered corridor", scan, angle_min, increment))
    scan, angle_min, increment = _synthetic_corridor_scan(
        settings,
        sensor_offset_left=-0.08,
    )
    scenarios.append(("Car shifted right", scan, angle_min, increment))
    blocked, angle_min, increment = _synthetic_corridor_scan(
        settings,
        obstacle_forward=0.70,
        obstacle_center_left=0.0,
        obstacle_width=settings.track_width,
    )
    scenarios.append(("Blocked front", blocked, angle_min, increment))
    corner = list(blocked)
    for index in range(len(corner)):
        car_angle = normalize_angle(
            angle_min + index * increment - settings.forward_raw_angle
        )
        if math.radians(-60.0) <= car_angle <= math.radians(-8.0):
            corner[index] = 2.5
    scenarios.append(("Right-hand corner", corner, angle_min, increment))

    print("Offline simulation with conservative defaults\n")
    print(
        f"{'Scenario':<22} {'Safe':<6} {'Mode':<9} {'Direction':<10} "
        f"{'Servo':<7} {'Duty':<6} Reason"
    )
    print("-" * 110)
    for name, ranges, angle_min, increment in scenarios:
        decision = plan_adaptive_drive(
            ranges,
            angle_min,
            increment,
            0.05,
            30.0,
            settings,
        )
        if decision.safe and decision.steering_angle is not None:
            angle = clamp(
                decision.steering_angle * config.turning_aggression,
                -settings.physical_maximum_steering_angle,
                settings.physical_maximum_steering_angle,
            )
            servo = steering_angle_to_servo_position(
                angle,
                settings.physical_maximum_steering_angle,
                config.servo_center,
                config.servo_left_limit,
                config.servo_right_limit,
            )
            duty = desired_drive_duty(config, servo, decision.front_distance)
            servo_text = f"{servo:.3f}"
            duty_text = f"{duty:.1%}"
        else:
            servo_text = f"{config.servo_center:.3f}"
            duty_text = "0.0%"
        print(
            f"{name:<22} {str(decision.safe):<6} {decision.mode:<9} "
            f"{decision.direction:<10} {servo_text:<7} {duty_text:<6} "
            f"{decision.reason}"
        )


def _profiles_path() -> Path:
    return Path(__file__).resolve().parent / "roboracer_tuning_profiles.json"


def _logs_directory() -> Path:
    return Path(__file__).resolve().parent / "roboracer_logs"


def load_profiles(path: Path) -> dict[str, dict[str, Any]]:
    if not path.exists():
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        print(f"Warning: could not read profiles: {error}")
        return {}
    if not isinstance(payload, dict) or payload.get("format_version") != 1:
        print("Warning: profile file format is not recognized; using defaults.")
        return {}
    profiles = payload.get("profiles", {})
    return profiles if isinstance(profiles, dict) else {}


def config_from_profile(
    name: str,
    data: dict[str, Any],
) -> Optional[DriveConfig]:
    allowed = {field.name for field in fields(DriveConfig)}
    values = {key: value for key, value in data.items() if key in allowed}
    values["profile_name"] = name
    try:
        config = DriveConfig(**values)
    except (TypeError, ValueError) as error:
        print(f"Profile {name!r} is invalid: {error}")
        return None
    errors = validate_config(config)
    if errors:
        print(f"Profile {name!r} failed validation:")
        for error in errors:
            print(f"  - {error}")
        return None
    return config


def save_profile(config: DriveConfig, path: Path) -> None:
    profiles = load_profiles(path)
    data = asdict(config)
    data.pop("run_mode", None)
    data.pop("profile_name", None)
    profiles[config.profile_name] = data
    payload = {
        "format_version": 1,
        "script_version": SCRIPT_VERSION,
        "profiles": profiles,
    }
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def prompt_choice(prompt: str, choices: dict[str, str], default: str) -> str:
    labels = "/".join(choices)
    while True:
        answer = input(f"{prompt} ({labels}) [{default}]: ").strip().lower()
        answer = answer or default
        if answer in choices:
            return answer
        print("Please choose one of: " + ", ".join(choices))


def prompt_yes_no(prompt: str, default: bool = False) -> bool:
    default_text = "y" if default else "n"
    answer = prompt_choice(prompt, {"y": "yes", "n": "no"}, default_text)
    return answer == "y"


def prompt_float(
    label: str,
    current: float,
    minimum: float,
    maximum: float,
    *,
    unit: str = "",
    decimals: int = 3,
) -> float:
    current = clamp(current, minimum, maximum)
    suffix = f" {unit}" if unit else ""
    while True:
        answer = input(
            f"{label}{suffix} [{current:.{decimals}f}] "
            f"({minimum:g} to {maximum:g}): "
        ).strip()
        if not answer:
            return current
        try:
            value = float(answer)
        except ValueError:
            print("Enter a number, or press Enter to keep the value.")
            continue
        if not math.isfinite(value) or not minimum <= value <= maximum:
            print(f"Value must be from {minimum:g} through {maximum:g}.")
            continue
        return value


def prompt_percent(
    label: str,
    current_fraction: float,
    minimum_percent: float,
    maximum_percent: float,
) -> float:
    value = prompt_float(
        label,
        current_fraction * 100.0,
        minimum_percent,
        maximum_percent,
        unit="%",
        decimals=1,
    )
    return value / 100.0


def tune_quick(config: DriveConfig) -> DriveConfig:
    print("\nQUICK TUNE — press Enter to keep each bracketed value.\n")
    center = prompt_float(
        "Straight steering center",
        config.servo_center,
        0.40,
        0.60,
    )
    left = prompt_float(
        "Left servo limit (smaller number = more left)",
        min(config.servo_left_limit, center - 0.01),
        HARD_MINIMUM_SERVO_POSITION,
        center - 0.01,
    )
    right = prompt_float(
        "Right servo limit (larger number = more right)",
        max(config.servo_right_limit, center + 0.01),
        center + 0.01,
        HARD_MAXIMUM_SERVO_POSITION,
    )
    aggression = prompt_float(
        "Turning aggression multiplier",
        config.turning_aggression,
        0.10,
        1.50,
        decimals=2,
    )
    maximum_duty = prompt_percent(
        "Maximum straight duty",
        config.maximum_straight_duty,
        1.0,
        HARD_MAXIMUM_FORWARD_DUTY * 100.0,
    )
    minimum_moving = prompt_percent(
        "Minimum duty that reliably moves the car",
        min(config.minimum_moving_duty, maximum_duty),
        0.5,
        maximum_duty * 100.0,
    )
    minimum_corner = prompt_percent(
        "Minimum corner duty",
        clamp(config.minimum_corner_duty, minimum_moving, maximum_duty),
        minimum_moving * 100.0,
        maximum_duty * 100.0,
    )
    slowdown_strength = prompt_percent(
        "Corner slowdown strength",
        config.corner_slowdown_strength,
        0.0,
        100.0,
    )
    acceleration = prompt_percent(
        "Acceleration rate (duty added per second)",
        config.acceleration_rate_duty_per_second,
        0.5,
        20.0,
    )
    deceleration = prompt_percent(
        "Normal deceleration rate (duty removed per second)",
        config.deceleration_rate_duty_per_second,
        0.5,
        30.0,
    )
    emergency = prompt_float(
        "Emergency-stop distance",
        config.emergency_stop_distance_m,
        HARD_MINIMUM_EMERGENCY_DISTANCE_M,
        0.70,
        unit="m",
    )
    full_corner = max(config.full_corner_distance_m, emergency + 0.10)
    slowdown = prompt_float(
        "Begin speed reduction at",
        clamp(
            config.slowdown_distance_m,
            full_corner,
            config.avoidance_start_distance_m - 0.05,
        ),
        full_corner,
        config.avoidance_start_distance_m - 0.05,
        unit="m",
    )
    maximum_run = prompt_float(
        "Maximum run time",
        config.maximum_run_seconds,
        1.0,
        HARD_MAXIMUM_RUN_SECONDS,
        unit="s",
        decimals=1,
    )
    return replace(
        config,
        servo_center=center,
        servo_left_limit=left,
        servo_right_limit=right,
        turning_aggression=aggression,
        maximum_straight_duty=maximum_duty,
        minimum_moving_duty=minimum_moving,
        minimum_corner_duty=minimum_corner,
        corner_slowdown_strength=slowdown_strength,
        acceleration_rate_duty_per_second=acceleration,
        deceleration_rate_duty_per_second=deceleration,
        emergency_stop_distance_m=emergency,
        slowdown_distance_m=slowdown,
        maximum_run_seconds=maximum_run,
    )


def tune_advanced(config: DriveConfig) -> DriveConfig:
    print("\nADVANCED TUNE — change one variable at a time when possible.\n")
    vehicle_width = prompt_float(
        "Vehicle width",
        config.vehicle_width_m,
        0.20,
        0.50,
        unit="m",
    )
    safety_margin = prompt_float(
        "Obstacle safety margin on each side",
        config.obstacle_safety_margin_m,
        0.0,
        0.15,
        unit="m",
    )
    required_gap = vehicle_width + 2.0 * safety_margin
    track_width = prompt_float(
        "Nominal track width",
        max(config.track_width_m, required_gap + 0.01),
        required_gap + 0.01,
        3.0,
        unit="m",
    )
    minimum_gap = prompt_float(
        "Minimum usable opening width",
        clamp(config.minimum_usable_gap_m, required_gap, track_width),
        required_gap,
        track_width,
        unit="m",
    )
    avoidance_start = prompt_float(
        "Planner look-ahead / avoidance-start distance",
        max(config.avoidance_start_distance_m, config.slowdown_distance_m + 0.05),
        config.slowdown_distance_m + 0.05,
        4.0,
        unit="m",
    )
    full_corner = prompt_float(
        "Distance for full corner steering",
        clamp(
            config.full_corner_distance_m,
            config.emergency_stop_distance_m + 0.10,
            config.slowdown_distance_m,
        ),
        config.emergency_stop_distance_m + 0.10,
        config.slowdown_distance_m,
        unit="m",
    )
    urgent = prompt_float(
        "Distance for faster steering response",
        clamp(
            config.urgent_distance_m,
            config.emergency_stop_distance_m + 0.05,
            full_corner,
        ),
        config.emergency_stop_distance_m + 0.05,
        full_corner,
        unit="m",
    )
    gap_check = prompt_float(
        "Distance at which opening width is checked",
        max(
            config.minimum_gap_check_distance_m,
            config.emergency_stop_distance_m + 0.05,
        ),
        config.emergency_stop_distance_m + 0.05,
        2.0,
        unit="m",
    )
    target_radius = prompt_float(
        "Target corner radius",
        config.target_corner_radius_m,
        0.40,
        3.0,
        unit="m",
    )
    steering_deadband = prompt_float(
        "Steering deadband",
        config.steering_deadband_degrees,
        0.0,
        5.0,
        unit="degrees",
        decimals=1,
    )
    servo_rate = prompt_float(
        "Normal servo-position change per second",
        config.servo_rate_limit_per_second,
        0.05,
        3.0,
        decimals=2,
    )
    urgent_servo_rate = prompt_float(
        "Urgent servo-position change per second",
        max(config.urgent_servo_rate_limit_per_second, servo_rate),
        servo_rate,
        5.0,
        decimals=2,
    )
    commitment = prompt_float(
        "Turn commitment time",
        config.turn_commit_seconds,
        0.0,
        2.0,
        unit="s",
        decimals=2,
    )
    centering_deadband = prompt_float(
        "Track-centering deadband",
        config.centering_deadband_m,
        0.0,
        0.10,
        unit="m",
    )
    centering_full = prompt_float(
        "Centering error for full correction",
        max(config.centering_full_error_m, centering_deadband + 0.01),
        centering_deadband + 0.01,
        0.30,
        unit="m",
    )
    max_centering = prompt_float(
        "Maximum centering steering angle",
        config.maximum_centering_angle_degrees,
        0.0,
        15.0,
        unit="degrees",
        decimals=1,
    )
    scan_timeout = prompt_float(
        "Maximum LiDAR scan age",
        config.scan_timeout_seconds,
        0.10,
        0.50,
        unit="s",
        decimals=2,
    )
    arm_time = prompt_float(
        "Continuous safe LiDAR time before countdown",
        config.safe_scan_arm_seconds,
        1.0,
        5.0,
        unit="s",
        decimals=1,
    )
    return replace(
        config,
        vehicle_width_m=vehicle_width,
        obstacle_safety_margin_m=safety_margin,
        minimum_usable_gap_m=minimum_gap,
        track_width_m=track_width,
        avoidance_start_distance_m=avoidance_start,
        full_corner_distance_m=full_corner,
        urgent_distance_m=urgent,
        minimum_gap_check_distance_m=gap_check,
        target_corner_radius_m=target_radius,
        steering_deadband_degrees=steering_deadband,
        servo_rate_limit_per_second=servo_rate,
        urgent_servo_rate_limit_per_second=urgent_servo_rate,
        turn_commit_seconds=commitment,
        centering_deadband_m=centering_deadband,
        centering_full_error_m=centering_full,
        maximum_centering_angle_degrees=max_centering,
        scan_timeout_seconds=scan_timeout,
        safe_scan_arm_seconds=arm_time,
    )


def print_configuration_summary(config: DriveConfig) -> None:
    required_gap = config.vehicle_width_m + 2.0 * config.obstacle_safety_margin_m
    print("\nFINAL CONFIGURATION")
    print("=" * 72)
    rows = (
        ("Run mode", config.run_mode),
        ("Profile", config.profile_name),
        (
            "Servo left / center / right",
            (
                f"{config.servo_left_limit:.3f} / {config.servo_center:.3f} / "
                f"{config.servo_right_limit:.3f}"
            ),
        ),
        ("Turning aggression", f"{config.turning_aggression:.2f}x"),
        (
            "Duty min move / min corner / max straight",
            (
                f"{config.minimum_moving_duty:.1%} / "
                f"{config.minimum_corner_duty:.1%} / "
                f"{config.maximum_straight_duty:.1%}"
            ),
        ),
        (
            "Acceleration / normal deceleration",
            (
                f"{config.acceleration_rate_duty_per_second:.1%}/s / "
                f"{config.deceleration_rate_duty_per_second:.1%}/s"
            ),
        ),
        ("Corner slowdown strength", f"{config.corner_slowdown_strength:.0%}"),
        (
            "Emergency / slowdown / avoidance",
            (
                f"{config.emergency_stop_distance_m:.2f} / "
                f"{config.slowdown_distance_m:.2f} / "
                f"{config.avoidance_start_distance_m:.2f} m"
            ),
        ),
        (
            "Vehicle / required / configured gap",
            (
                f"{config.vehicle_width_m:.3f} / {required_gap:.3f} / "
                f"{config.minimum_usable_gap_m:.3f} m"
            ),
        ),
        ("Maximum run time", f"{config.maximum_run_seconds:.1f} s"),
        ("LiDAR topic", config.scan_topic),
        ("VESC port", config.serial_port),
    )
    for label, value in rows:
        print(f"{label:<43} {value}")
    print("=" * 72)
    print(
        "Hard limits: forward duty <= 10%, servo 0.250--0.750, "
        "run <= 60 s, no reverse, no active braking."
    )


def interactive_configuration() -> Optional[DriveConfig]:
    print("\nRoboRacer Interactive Autonomous Drive v" + SCRIPT_VERSION)
    print("Motor output starts DISABLED. Ctrl+C cancels at any prompt.\n")

    profiles = load_profiles(_profiles_path())
    config = DriveConfig()
    valid_names: list[str] = []
    if profiles:
        print("Saved profiles:")
        for name in sorted(profiles):
            candidate = config_from_profile(name, profiles[name])
            if candidate is not None:
                valid_names.append(name)
                print(f"  {len(valid_names)}. {name}")
        print("  0. conservative built-in defaults")
        while True:
            answer = input("Load profile number [0]: ").strip() or "0"
            try:
                selection = int(answer)
            except ValueError:
                print("Enter a listed number.")
                continue
            if selection == 0:
                break
            if 1 <= selection <= len(valid_names):
                name = valid_names[selection - 1]
                loaded = config_from_profile(name, profiles[name])
                if loaded is not None:
                    config = loaded
                    print(f"Loaded profile {name!r}.")
                    break
            print("Enter a listed number.")

    config = tune_quick(config)
    if prompt_yes_no("Open Advanced Tune", default=False):
        config = tune_advanced(config)

    mode = prompt_choice(
        "Run mode: d = steering/duty preview, l = live motor",
        {"d": "dry", "l": "live"},
        "d",
    )
    config = replace(config, run_mode="LIVE" if mode == "l" else "DRY")

    if prompt_yes_no("Save these tuning values as a profile", default=False):
        while True:
            name = input(f"Profile name [{config.profile_name}]: ").strip()
            name = name or config.profile_name
            if len(name) > 60 or any(char in name for char in "\r\n\t"):
                print("Use a short, single-line profile name.")
                continue
            config = replace(config, profile_name=name)
            try:
                save_profile(config, _profiles_path())
            except OSError as error:
                print(f"Could not save profile: {error}")
            else:
                print(f"Saved profile {name!r}.")
            break

    errors = validate_config(config)
    if errors:
        print("\nConfiguration rejected:")
        for error in errors:
            print(f"  - {error}")
        print("No hardware commands were sent. Restart and correct the values.")
        return None

    print_configuration_summary(config)
    if config.run_mode == "LIVE":
        print(
            "\nLIVE-RUN CHECKLIST\n"
            "  - The car passed a raised-wheel DRY run with these servo limits.\n"
            "  - The RIGHT endpoint was physically checked for binding.\n"
            "  - The VESC safety timeout (0.5 s or less) was configured and "
            "independently tested.\n"
            "  - A person is ready at the physical power disconnect.\n"
            "  - The test area is clear, with no people in front of the car.\n"
            "  - Zero-duty coasting distance fits inside the emergency margin.\n"
        )
        if not prompt_yes_no(
            "Have all six LIVE-run checklist items been verified",
            default=False,
        ):
            print("Live run cancelled. Choose DRY mode until the checklist passes.")
            return None
        confirmation = input("Type START LIVE exactly to continue: ").strip()
        if confirmation != "START LIVE":
            print("Confirmation did not match. No hardware commands were sent.")
            return None
    else:
        print(
            "\nDRY mode commands steering but keeps transmitted motor duty at 0%. "
            "Keep the car raised for the first dry run."
        )
        confirmation = input("Type START DRY exactly to continue: ").strip()
        if confirmation != "START DRY":
            print("Confirmation did not match. No hardware commands were sent.")
            return None
    return config


def run_ros_controller(config: DriveConfig) -> None:
    """Load Pi-only dependencies and run the hardware controller."""
    try:
        import pyvesc
        import rclpy
        import serial
        from pyvesc import GetValues, SetDutyCycle, VESCMessage
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
    except ImportError as error:
        raise RuntimeError(
            "ROS 2, pyserial, pyvesc, or sensor_msgs is unavailable. Source "
            "ROS and the Orbbec workspace, then activate venv_vesc."
        ) from error

    class SetServoPosition(metaclass=VESCMessage):
        id = 12
        fields = [("servo_pos", "h", 1000)]

    class InteractiveAutonomousDrive(Node):
        def __init__(self) -> None:
            super().__init__("sl450_interactive_autonomous_drive")
            self.config = config
            self.settings = planner_settings_from_config(config)
            self.latest_decision: Optional[PlannerDecision] = None
            self.last_scan_time: Optional[float] = None
            self.planner_fault: Optional[str] = None
            self.serial_fault: Optional[str] = None
            self.previous_turn_sign = 0
            self.committed_turn_sign = 0
            self.turn_commit_until = 0.0
            self.current_servo_position = config.servo_center
            self.target_servo_position = config.servo_center
            self.current_duty = 0.0
            self.target_duty = 0.0
            self.preview_duty = 0.0
            self.current_state = "STARTUP"
            self.safe_since: Optional[float] = None
            self.countdown_started_at: Optional[float] = None
            self.run_started_at: Optional[float] = None
            self.run_finished_at: Optional[float] = None
            self.stop_latched = False
            self.stop_reason = ""
            self.minimum_front_distance: Optional[float] = None
            self.maximum_commanded_duty = 0.0
            now = time.monotonic()
            self.last_steering_command_time = now
            self.last_duty_command_time = now
            self.last_control_heartbeat = now
            self.watchdog_tripped = False
            self._serial_lock = threading.Lock()
            self._watchdog_stop = threading.Event()
            self._watchdog_thread: Optional[threading.Thread] = None
            self._closed = False
            self._summary_written = False
            self._wall_start = datetime.now(timezone.utc)
            self._telemetry_file = None
            self._telemetry_writer = None

            self.log_directory = _logs_directory()
            self.log_directory.mkdir(parents=True, exist_ok=True)
            stamp = self._wall_start.strftime("%Y%m%dT%H%M%SZ")
            self.run_id = f"{stamp}_{config.run_mode.lower()}"
            self.telemetry_path = self.log_directory / f"{self.run_id}.csv"
            self._open_telemetry_log()

            self.get_logger().warning(
                f"{config.run_mode} MODE | hard duty ceiling "
                f"{HARD_MAXIMUM_FORWARD_DUTY:.0%} | automatic stop after "
                f"{config.maximum_run_seconds:.1f} s."
            )
            self.get_logger().info(f"Opening VESC on {config.serial_port}.")
            self.connection = None
            try:
                self.connection = serial.Serial(
                    port=config.serial_port,
                    baudrate=config.baud_rate,
                    timeout=0.05,
                    write_timeout=0.20,
                    exclusive=True,
                )
                time.sleep(1.0)
                self._send_repeated_stop_and_center(10)
                telemetry = self._request_measurements(1.0)
                if telemetry is None:
                    raise RuntimeError(
                        "VESC telemetry did not respond; control was not armed."
                    )
                self.get_logger().info(
                    "VESC telemetry received. Zero duty confirmed; waiting for "
                    "stable LiDAR decisions."
                )
                self.subscription = self.create_subscription(
                    LaserScan,
                    config.scan_topic,
                    self.scan_callback,
                    qos_profile_sensor_data,
                )
                self.control_timer = self.create_timer(
                    1.0 / config.control_rate_hz,
                    self.control_vehicle,
                )
                self.status_timer = self.create_timer(
                    1.0 / config.status_rate_hz,
                    self.report_status,
                )
                self._watchdog_thread = threading.Thread(
                    target=self._watchdog_loop,
                    name="roboracer-control-watchdog",
                    daemon=True,
                )
                self._watchdog_thread.start()
            except Exception:
                self._constructor_cleanup()
                raise

        def _open_telemetry_log(self) -> None:
            self._telemetry_file = self.telemetry_path.open(
                "w",
                newline="",
                encoding="utf-8",
            )
            fieldnames = [
                "wall_time_utc",
                "elapsed_s",
                "state",
                "planner_mode",
                "direction",
                "reason",
                "front_m",
                "left_wall_m",
                "right_wall_m",
                "left_gap_m",
                "right_gap_m",
                "scan_age_s",
                "servo_actual",
                "servo_target",
                "duty_sent",
                "duty_target_or_preview",
                "motor_enabled",
            ]
            self._telemetry_writer = csv.DictWriter(
                self._telemetry_file,
                fieldnames=fieldnames,
            )
            self._telemetry_writer.writeheader()
            self._telemetry_file.flush()

        def _constructor_cleanup(self) -> None:
            try:
                if self.connection is not None and self.connection.is_open:
                    self._best_effort_repeated_stop(10)
                    self.connection.close()
            except Exception:
                pass
            if self._telemetry_file is not None:
                self._telemetry_file.close()

        def _write_packet(self, packet: bytes) -> bool:
            if self.serial_fault is not None:
                return False
            acquired = self._serial_lock.acquire(timeout=0.10)
            if not acquired:
                self.serial_fault = "serial write lock timeout"
                return False
            try:
                if self.connection is None:
                    self.serial_fault = "serial connection is not open"
                    return False
                written = self.connection.write(packet)
                if written != len(packet):
                    self.serial_fault = (
                        f"partial serial write ({written}/{len(packet)} bytes)"
                    )
                    return False
                self.connection.flush()
                return True
            except (serial.SerialException, OSError) as error:
                self.serial_fault = str(error)
                return False
            finally:
                self._serial_lock.release()

        def _write_servo(self, position: float) -> bool:
            position = clamp(
                position,
                config.servo_left_limit,
                config.servo_right_limit,
            )
            packet = pyvesc.encode(SetServoPosition(position))
            if self._write_packet(packet):
                self.current_servo_position = position
                return True
            return False

        def _write_duty(self, duty: float) -> bool:
            if duty < -1e-12:
                raise ValueError("reverse duty is prohibited")
            if duty > HARD_MAXIMUM_FORWARD_DUTY + 1e-12:
                raise ValueError(
                    f"refusing duty above {HARD_MAXIMUM_FORWARD_DUTY:.0%}"
                )
            packet = pyvesc.encode(SetDutyCycle(duty_to_vesc_value(duty)))
            if self._write_packet(packet):
                self.current_duty = duty
                self.maximum_commanded_duty = max(
                    self.maximum_commanded_duty,
                    duty,
                )
                return True
            return False

        def _send_repeated_stop_and_center(self, repetitions: int) -> None:
            for _ in range(repetitions):
                if self.serial_fault is not None:
                    break
                self._write_duty(0.0)
                self._write_servo(config.servo_center)
                time.sleep(0.05)

        def _best_effort_repeated_stop(self, repetitions: int) -> None:
            """Attempt final zero/center writes even after a latched fault."""
            duty_packet = pyvesc.encode(SetDutyCycle(0))
            servo_packet = pyvesc.encode(SetServoPosition(config.servo_center))
            for _ in range(repetitions):
                try:
                    acquired = self._serial_lock.acquire(timeout=0.05)
                    if not acquired:
                        continue
                    try:
                        self.connection.write(duty_packet)
                        self.connection.write(servo_packet)
                        self.connection.flush()
                    finally:
                        self._serial_lock.release()
                except (serial.SerialException, OSError):
                    break
                time.sleep(0.03)
            self.current_duty = 0.0
            self.current_servo_position = config.servo_center

        def _request_measurements(self, timeout_seconds: float):
            try:
                if self.connection is None:
                    self.serial_fault = "serial connection is not open"
                    return None
                self.connection.reset_input_buffer()
                if not self._write_packet(pyvesc.encode_request(GetValues)):
                    return None
                deadline = time.monotonic() + timeout_seconds
                received = bytearray()
                while time.monotonic() < deadline:
                    waiting = self.connection.in_waiting
                    if waiting:
                        received.extend(self.connection.read(waiting))
                        try:
                            response, _consumed = pyvesc.decode(bytes(received))
                            if response is not None:
                                return response
                        except Exception:
                            pass
                    time.sleep(0.01)
            except (serial.SerialException, OSError) as error:
                self.serial_fault = str(error)
            return None

        def _watchdog_loop(self) -> None:
            while not self._watchdog_stop.wait(0.05):
                if (
                    config.run_mode != "LIVE"
                    or self.run_started_at is None
                    or self.stop_latched
                ):
                    continue
                age = time.monotonic() - self.last_control_heartbeat
                if age <= config.software_watchdog_seconds:
                    continue
                self.watchdog_tripped = True
                self.stop_latched = True
                self.stop_reason = (
                    f"software control watchdog expired after {age:.3f} s"
                )
                self.current_state = "STOP: WATCHDOG"
                self.run_finished_at = time.monotonic()
                self._best_effort_repeated_stop(3)

        def _scan_age(self) -> Optional[float]:
            if self.last_scan_time is None:
                return None
            return max(0.0, time.monotonic() - self.last_scan_time)

        def _apply_turn_commitment(
            self,
            decision: PlannerDecision,
            now: float,
        ) -> PlannerDecision:
            if decision.mode != "AVOID" or decision.steering_angle is None:
                self.committed_turn_sign = 0
                self.turn_commit_until = 0.0
                return decision
            requested_sign = +1 if decision.steering_angle > 0.0 else -1
            if (
                self.committed_turn_sign == 0
                or now >= self.turn_commit_until
            ):
                self.committed_turn_sign = requested_sign
                self.turn_commit_until = now + config.turn_commit_seconds
                return decision
            if requested_sign == self.committed_turn_sign:
                return decision
            committed_gap = (
                decision.left_gap_width
                if self.committed_turn_sign > 0
                else decision.right_gap_width
            )
            if (
                committed_gap is None
                or committed_gap < config.minimum_usable_gap_m
            ):
                self.committed_turn_sign = requested_sign
                self.turn_commit_until = now + config.turn_commit_seconds
                return decision
            held_angle = math.copysign(
                abs(decision.steering_angle),
                self.committed_turn_sign,
            )
            return replace(
                decision,
                direction="LEFT" if self.committed_turn_sign > 0 else "RIGHT",
                steering_angle=held_angle,
                reason=decision.reason + "; holding committed turn",
            )

        def scan_callback(self, scan: Any) -> None:
            now = time.monotonic()
            self.last_scan_time = now
            try:
                decision = plan_adaptive_drive(
                    scan.ranges,
                    scan.angle_min,
                    scan.angle_increment,
                    scan.range_min,
                    scan.range_max,
                    self.settings,
                    self.previous_turn_sign,
                )
                if decision.safe:
                    decision = self._apply_turn_commitment(decision, now)
                self.latest_decision = decision
                self.planner_fault = None
                if decision.safe and decision.mode == "AVOID":
                    self.previous_turn_sign = (
                        +1 if decision.direction == "LEFT" else -1
                    )
                elif decision.front_distance is None:
                    self.previous_turn_sign = 0
                if decision.front_distance is not None:
                    if self.minimum_front_distance is None:
                        self.minimum_front_distance = decision.front_distance
                    else:
                        self.minimum_front_distance = min(
                            self.minimum_front_distance,
                            decision.front_distance,
                        )
                if self.run_started_at is not None and not decision.safe:
                    self._latch_stop(decision.reason)
            except Exception as error:
                self.latest_decision = None
                self.planner_fault = str(error)
                if self.run_started_at is not None:
                    self._latch_stop(f"planner fault: {error}")

        def _unsafe_reason(self) -> Optional[str]:
            if self.serial_fault is not None:
                return f"serial fault: {self.serial_fault}"
            if self.watchdog_tripped:
                return "software control watchdog tripped"
            scan_age = self._scan_age()
            if scan_age is None:
                return "waiting for LiDAR"
            if scan_age > config.scan_timeout_seconds:
                return f"LiDAR data stale ({scan_age:.3f} s old)"
            if self.planner_fault is not None:
                return f"planner fault: {self.planner_fault}"
            if self.latest_decision is None:
                return "no planner decision"
            if not self.latest_decision.safe:
                return self.latest_decision.reason
            if self.latest_decision.steering_angle is None:
                return "planner supplied no steering angle"
            return None

        def _latch_stop(self, reason: str) -> None:
            if self.stop_latched:
                return
            self.stop_latched = True
            self.stop_reason = reason
            self.current_state = "STOP LATCHED"
            self.run_finished_at = time.monotonic()
            self.target_duty = 0.0
            self.preview_duty = 0.0
            # Safety stops bypass the configured normal deceleration ramp.
            self._write_duty(0.0)
            self._write_servo(config.servo_center)
            self.get_logger().error(
                f"STOP LATCHED: {reason}. Restart required; no auto-resume."
            )

        def _safe_steering_target(self) -> float:
            assert self.latest_decision is not None
            assert self.latest_decision.steering_angle is not None
            angle = self.latest_decision.steering_angle * config.turning_aggression
            angle = clamp(
                angle,
                -self.settings.physical_maximum_steering_angle,
                self.settings.physical_maximum_steering_angle,
            )
            if abs(angle) < math.radians(config.steering_deadband_degrees):
                angle = 0.0
            return steering_angle_to_servo_position(
                angle,
                self.settings.physical_maximum_steering_angle,
                config.servo_center,
                config.servo_left_limit,
                config.servo_right_limit,
            )

        def _command_steering(self, now: float) -> bool:
            assert self.latest_decision is not None
            target = self._safe_steering_target()
            self.target_servo_position = target
            elapsed = min(
                max(now - self.last_steering_command_time, 0.0),
                0.25,
            )
            self.last_steering_command_time = now
            urgent = (
                self.latest_decision.front_distance is not None
                and self.latest_decision.front_distance <= config.urgent_distance_m
            )
            rate = (
                config.urgent_servo_rate_limit_per_second
                if urgent
                else config.servo_rate_limit_per_second
            )
            command = rate_limited_value(
                self.current_servo_position,
                target,
                rate * elapsed,
            )
            return self._write_servo(command)

        def _reset_prearm(self, state: str) -> None:
            self.safe_since = None
            self.countdown_started_at = None
            self.current_state = state
            self.target_duty = 0.0
            self.preview_duty = 0.0
            self._write_duty(0.0)
            self._write_servo(config.servo_center)
            now = time.monotonic()
            self.last_steering_command_time = now
            self.last_duty_command_time = now

        def control_vehicle(self) -> None:
            now = time.monotonic()
            self.last_control_heartbeat = now
            if self.stop_latched:
                self._write_duty(0.0)
                return

            unsafe = self._unsafe_reason()
            if self.run_started_at is not None and unsafe is not None:
                self._latch_stop(unsafe)
                return
            if self.run_started_at is None and unsafe is not None:
                self._reset_prearm(f"WAIT: {unsafe}")
                return

            assert self.latest_decision is not None
            assert self.latest_decision.steering_angle is not None
            if not self._command_steering(now):
                self._latch_stop("failed to send steering command")
                return

            if self.run_started_at is None:
                self._write_duty(0.0)
                if self.safe_since is None:
                    self.safe_since = now
                stable_time = now - self.safe_since
                if stable_time < config.safe_scan_arm_seconds:
                    self.current_state = "STABILIZING LIDAR"
                    return
                if self.countdown_started_at is None:
                    self.countdown_started_at = now
                    self.current_state = "COUNTDOWN"
                    self.get_logger().warning(
                        f"Preflight passed. Starting {config.countdown_seconds:.0f} "
                        "second countdown."
                    )
                    return
                if now - self.countdown_started_at < config.countdown_seconds:
                    self.current_state = "COUNTDOWN"
                    return
                self.run_started_at = now
                self.last_duty_command_time = now
                self.current_state = (
                    "RUNNING LIVE" if config.run_mode == "LIVE" else "RUNNING DRY"
                )
                self.get_logger().warning(
                    f"{self.current_state}: automatic stop in "
                    f"{config.maximum_run_seconds:.1f} seconds."
                )

            assert self.run_started_at is not None
            if now - self.run_started_at >= config.maximum_run_seconds:
                self._latch_stop(
                    f"maximum run time {config.maximum_run_seconds:.1f} s reached"
                )
                return

            self.preview_duty = desired_drive_duty(
                config,
                self.target_servo_position,
                self.latest_decision.front_distance,
            )
            self.target_duty = (
                self.preview_duty if config.run_mode == "LIVE" else 0.0
            )
            elapsed = min(max(now - self.last_duty_command_time, 0.0), 0.25)
            self.last_duty_command_time = now
            commanded_duty = ramp_duty(
                self.current_duty,
                self.target_duty,
                elapsed,
                config.acceleration_rate_duty_per_second,
                config.deceleration_rate_duty_per_second,
            )
            if not self._write_duty(commanded_duty):
                self._latch_stop("failed to send motor command")
                return
            self.current_state = (
                f"{self.latest_decision.mode} {self.latest_decision.direction} "
                f"{'LIVE' if config.run_mode == 'LIVE' else 'DRY'}"
            )

        @staticmethod
        def _format_optional(value: Optional[float], suffix: str = "") -> str:
            return "--" if value is None else f"{value:.2f}{suffix}"

        def _record_telemetry(self) -> None:
            if self._telemetry_writer is None or self._telemetry_file is None:
                return
            decision = self.latest_decision
            scan_age = self._scan_age()
            elapsed = (
                0.0
                if self.run_started_at is None
                else time.monotonic() - self.run_started_at
            )
            row = {
                "wall_time_utc": datetime.now(timezone.utc).isoformat(),
                "elapsed_s": f"{elapsed:.3f}",
                "state": self.current_state,
                "planner_mode": "" if decision is None else decision.mode,
                "direction": "" if decision is None else decision.direction,
                "reason": self.stop_reason
                or ("" if decision is None else decision.reason),
                "front_m": ""
                if decision is None or decision.front_distance is None
                else f"{decision.front_distance:.4f}",
                "left_wall_m": ""
                if decision is None or decision.left_wall_distance is None
                else f"{decision.left_wall_distance:.4f}",
                "right_wall_m": ""
                if decision is None or decision.right_wall_distance is None
                else f"{decision.right_wall_distance:.4f}",
                "left_gap_m": ""
                if decision is None or decision.left_gap_width is None
                else f"{decision.left_gap_width:.4f}",
                "right_gap_m": ""
                if decision is None or decision.right_gap_width is None
                else f"{decision.right_gap_width:.4f}",
                "scan_age_s": "" if scan_age is None else f"{scan_age:.4f}",
                "servo_actual": f"{self.current_servo_position:.4f}",
                "servo_target": f"{self.target_servo_position:.4f}",
                "duty_sent": f"{self.current_duty:.5f}",
                "duty_target_or_preview": f"{self.preview_duty:.5f}",
                "motor_enabled": config.run_mode == "LIVE",
            }
            try:
                self._telemetry_writer.writerow(row)
                self._telemetry_file.flush()
            except OSError as error:
                self.get_logger().warning(f"Telemetry log write failed: {error}")
                self._telemetry_writer = None

        def report_status(self) -> None:
            self._record_telemetry()
            decision = self.latest_decision
            scan_age = self._scan_age()
            scan_text = "--" if scan_age is None else f"{scan_age:.3f}s"
            if self.stop_latched:
                self.get_logger().error(
                    f"STOPPED | {self.stop_reason} | duty "
                    f"{self.current_duty:.1%} | servo "
                    f"{self.current_servo_position:.3f} | restart required"
                )
                return
            if decision is None:
                self.get_logger().warning(
                    f"{self.current_state} | duty 0% | servo "
                    f"{self.current_servo_position:.3f} | scan age {scan_text}"
                )
                return
            run_elapsed = (
                0.0
                if self.run_started_at is None
                else time.monotonic() - self.run_started_at
            )
            remaining = max(0.0, config.maximum_run_seconds - run_elapsed)
            duty_text = (
                f"sent {self.current_duty:.1%}"
                if config.run_mode == "LIVE"
                else f"sent 0% / would {self.preview_duty:.1%}"
            )
            self.get_logger().info(
                f"{self.current_state:<24} | {duty_text} | servo "
                f"{self.current_servo_position:.3f}/"
                f"{self.target_servo_position:.3f} | front "
                f"{self._format_optional(decision.front_distance, 'm')} | "
                f"gaps L/R {self._format_optional(decision.left_gap_width, 'm')}/"
                f"{self._format_optional(decision.right_gap_width, 'm')} | "
                f"scan {scan_text} | {remaining:.1f}s left"
            )

        def _write_summary(self) -> None:
            if self._summary_written:
                return
            self._summary_written = True
            summary_path = self.log_directory / "run_summaries.csv"
            fieldnames = [
                "run_id",
                "script_version",
                "start_utc",
                "mode",
                "profile",
                "duration_s",
                "minimum_front_m",
                "maximum_duty_sent",
                "stop_reason",
                "telemetry_file",
                "settings_json",
            ]
            duration = 0.0
            if self.run_started_at is not None:
                end = self.run_finished_at or time.monotonic()
                duration = max(0.0, end - self.run_started_at)
            row = {
                "run_id": self.run_id,
                "script_version": SCRIPT_VERSION,
                "start_utc": self._wall_start.isoformat(),
                "mode": config.run_mode,
                "profile": config.profile_name,
                "duration_s": f"{duration:.3f}",
                "minimum_front_m": ""
                if self.minimum_front_distance is None
                else f"{self.minimum_front_distance:.4f}",
                "maximum_duty_sent": f"{self.maximum_commanded_duty:.5f}",
                "stop_reason": self.stop_reason or "operator shutdown",
                "telemetry_file": self.telemetry_path.name,
                "settings_json": json.dumps(asdict(config), sort_keys=True),
            }
            try:
                exists = summary_path.exists() and summary_path.stat().st_size > 0
                with summary_path.open("a", newline="", encoding="utf-8") as handle:
                    writer = csv.DictWriter(handle, fieldnames=fieldnames)
                    if not exists:
                        writer.writeheader()
                    writer.writerow(row)
            except OSError as error:
                self.get_logger().warning(f"Run summary write failed: {error}")

        def close(self, reason: str = "operator shutdown") -> None:
            if self._closed:
                return
            self._closed = True
            if not self.stop_reason:
                self.stop_reason = reason
            if self.run_finished_at is None:
                self.run_finished_at = time.monotonic()
            self._watchdog_stop.set()
            if self._watchdog_thread is not None:
                self._watchdog_thread.join(timeout=0.50)
            if self.connection is not None and self.connection.is_open:
                self._best_effort_repeated_stop(10)
                self.connection.close()
            self._record_telemetry()
            if self._telemetry_file is not None:
                self._telemetry_file.close()
                self._telemetry_file = None
            self._write_summary()

    def _handle_sigterm(_signum: int, _frame: Any) -> None:
        raise KeyboardInterrupt

    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, _handle_sigterm)
    rclpy.init(args=None)
    node: Optional[InteractiveAutonomousDrive] = None
    exit_reason = "operator shutdown"
    try:
        node = InteractiveAutonomousDrive()
        rclpy.spin(node)
    except KeyboardInterrupt:
        exit_reason = "Ctrl+C or SIGTERM"
    except (serial.SerialException, OSError, RuntimeError, ValueError) as error:
        exit_reason = f"startup/runtime error: {error}"
        print(f"Autonomous drive could not continue: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.close(exit_reason)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_sigterm)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Interactive low-speed SL450/VESC autonomous controller"
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="run dependency-free safety and planner checks",
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="show offline planner decisions using synthetic scans",
    )
    args = parser.parse_args()
    if args.self_test:
        run_self_test()
        return 0
    if args.simulate:
        run_simulation()
        return 0

    try:
        config = interactive_configuration()
    except (EOFError, KeyboardInterrupt):
        print("\nCancelled. No hardware commands were sent.")
        return 1
    if config is None:
        return 1
    try:
        run_ros_controller(config)
    except RuntimeError as error:
        print(str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

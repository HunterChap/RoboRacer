#!/usr/bin/env python3
"""Stationary Follow-the-Gap test for the RoboRacer's Orbbec SL450.

This ROS 2 node subscribes to the SL450 LaserScan topic, analyzes the usable
forward field of view, and reports the direction in which a basic
Follow-the-Gap controller would steer. It deliberately has no publishers and
cannot command the VESC, motor, or steering servo.

The default orientation was measured on this car:

    raw 90 degrees  = car right
    raw 180 degrees = car forward
    raw 270 degrees = car left

Normal use on the Raspberry Pi:

    source /opt/ros/jazzy/setup.bash
    source ~/orbbec_ws/install/setup.bash
    python3 sl450_follow_the_gap_test.py

Offline algorithm check (does not require ROS 2):

    python3 sl450_follow_the_gap_test.py --self-test
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import statistics
import sys
from typing import Optional, Sequence


def normalize_angle(angle_radians: float) -> float:
    """Return an angle in the interval [-pi, pi)."""
    return (angle_radians + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, lower: float, upper: float) -> float:
    """Limit value to the closed interval [lower, upper]."""
    return max(lower, min(value, upper))


@dataclass(frozen=True)
class PlannerSettings:
    """Configuration used by the ROS node and the offline self-test."""

    forward_raw_angle: float = math.radians(180.0)
    planning_half_fov: float = math.radians(100.0)
    max_considered_range: float = 8.0
    median_window_points: int = 5
    max_invalid_run_angle: float = math.radians(1.5)
    vehicle_width: float = 0.32
    safety_margin: float = 0.08
    closest_cluster_tolerance: float = 0.03
    minimum_gap_distance: float = 0.60
    minimum_gap_width: float = math.radians(12.0)
    target_window_width: float = math.radians(8.0)
    max_steering_angle: float = math.radians(30.0)
    emergency_stop_distance: float = 0.30
    front_check_half_width: float = math.radians(15.0)
    minimum_valid_fraction: float = 0.70


@dataclass(frozen=True)
class GapDecision:
    """Result of one complete scan analysis."""

    safe: bool
    reason: str
    direction: str
    target_angle: Optional[float]
    steering_angle: Optional[float]
    gap_width: Optional[float]
    target_clearance: Optional[float]
    nearest_front: Optional[float]
    valid_fraction: float
    bubble_half_width: Optional[float]


def fill_short_invalid_runs(
    values: Sequence[Optional[float]],
    maximum_run_points: int,
) -> list[Optional[float]]:
    """Conservatively fill only short invalid runs between valid readings.

    The lower of the two neighboring distances is used so interpolation never
    invents more clearance than either side of the missing data.
    """
    filled = list(values)
    index = 0

    while index < len(filled):
        if filled[index] is not None:
            index += 1
            continue

        run_start = index
        while index < len(filled) and filled[index] is None:
            index += 1
        run_end = index
        run_length = run_end - run_start

        left_value = filled[run_start - 1] if run_start > 0 else None
        right_value = filled[run_end] if run_end < len(filled) else None

        if (
            run_length <= maximum_run_points
            and left_value is not None
            and right_value is not None
        ):
            conservative_value = min(left_value, right_value)
            for fill_index in range(run_start, run_end):
                filled[fill_index] = conservative_value

    return filled


def median_filter_valid_points(
    values: Sequence[Optional[float]],
    window_points: int,
) -> list[Optional[float]]:
    """Median-filter valid points without bridging long no-data regions."""
    half_window = window_points // 2
    filtered: list[Optional[float]] = []

    for index, center_value in enumerate(values):
        if center_value is None:
            filtered.append(None)
            continue

        window_start = max(0, index - half_window)
        window_end = min(len(values), index + half_window + 1)
        valid_neighbors = [
            value
            for value in values[window_start:window_end]
            if value is not None
        ]
        filtered.append(float(statistics.median(valid_neighbors)))

    return filtered


def find_open_runs(
    values: Sequence[Optional[float]],
    minimum_distance: float,
) -> list[tuple[int, int]]:
    """Return inclusive index ranges that meet the minimum gap distance."""
    runs: list[tuple[int, int]] = []
    run_start: Optional[int] = None

    for index in range(len(values) + 1):
        is_open = (
            index < len(values)
            and values[index] is not None
            and values[index] >= minimum_distance
        )

        if is_open and run_start is None:
            run_start = index
        elif not is_open and run_start is not None:
            runs.append((run_start, index - 1))
            run_start = None

    return runs


def choose_target_index(
    angles: Sequence[float],
    values: Sequence[Optional[float]],
    gap_start: int,
    gap_end: int,
    target_window_points: int,
) -> tuple[int, float]:
    """Choose the clearest window inside a gap, preferring forward on ties."""
    half_window = target_window_points // 2
    gap_length = gap_end - gap_start + 1

    if gap_length > target_window_points:
        first_candidate = gap_start + half_window
        last_candidate = gap_end - half_window
    else:
        first_candidate = gap_start
        last_candidate = gap_end

    best_index = first_candidate
    best_clearance = -math.inf
    tolerance = 1e-9

    for candidate in range(first_candidate, last_candidate + 1):
        window_start = max(gap_start, candidate - half_window)
        window_end = min(gap_end + 1, candidate + half_window + 1)
        window_values = [
            value
            for value in values[window_start:window_end]
            if value is not None
        ]
        mean_clearance = sum(window_values) / len(window_values)

        if mean_clearance > best_clearance + tolerance:
            best_index = candidate
            best_clearance = mean_clearance
        elif (
            abs(mean_clearance - best_clearance) <= tolerance
            and abs(angles[candidate]) < abs(angles[best_index])
        ):
            best_index = candidate

    return best_index, best_clearance


def plan_follow_the_gap(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    settings: PlannerSettings,
) -> GapDecision:
    """Analyze one LaserScan without producing any vehicle command."""
    if not ranges or not math.isfinite(angle_increment) or angle_increment <= 0.0:
        return GapDecision(
            False, "invalid scan geometry", "STOP", None, None, None, None,
            None, 0.0, None,
        )

    samples: list[tuple[float, Optional[float], bool]] = []

    for index, measured_range in enumerate(ranges):
        raw_angle = angle_min + index * angle_increment
        car_angle = normalize_angle(raw_angle - settings.forward_raw_angle)
        if abs(car_angle) > settings.planning_half_fov:
            continue

        is_valid = (
            math.isfinite(measured_range)
            and range_min <= measured_range <= range_max
        )

        if is_valid:
            processed_range: Optional[float] = min(
                float(measured_range), settings.max_considered_range
            )
        elif math.isinf(measured_range) and measured_range > 0.0:
            # Positive infinity normally means no return inside range_max.
            processed_range = settings.max_considered_range
            is_valid = True
        else:
            processed_range = None

        samples.append((car_angle, processed_range, is_valid))

    samples.sort(key=lambda sample: sample[0])
    if len(samples) < 2:
        return GapDecision(
            False, "not enough scan coverage", "STOP", None, None, None, None,
            None, 0.0, None,
        )

    angles = [sample[0] for sample in samples]
    values = [sample[1] for sample in samples]
    valid_fraction = sum(sample[2] for sample in samples) / len(samples)

    angular_step = statistics.median(
        angles[index + 1] - angles[index]
        for index in range(len(angles) - 1)
    )
    if not math.isfinite(angular_step) or angular_step <= 0.0:
        return GapDecision(
            False, "invalid angular spacing", "STOP", None, None, None, None,
            None, valid_fraction, None,
        )

    maximum_run_points = max(
        1, round(settings.max_invalid_run_angle / angular_step)
    )
    values = fill_short_invalid_runs(values, maximum_run_points)
    values = median_filter_valid_points(values, settings.median_window_points)

    front_values = [
        value
        for angle, value in zip(angles, values)
        if abs(angle) <= settings.front_check_half_width and value is not None
    ]
    nearest_front = min(front_values) if front_values else None

    if valid_fraction < settings.minimum_valid_fraction:
        return GapDecision(
            False,
            f"only {valid_fraction:.0%} of planning scan is valid",
            "STOP",
            None,
            None,
            None,
            None,
            nearest_front,
            valid_fraction,
            None,
        )

    valid_obstacles = [
        (index, value)
        for index, value in enumerate(values)
        if value is not None
    ]
    if not valid_obstacles:
        return GapDecision(
            False, "no valid obstacle data", "STOP", None, None, None, None,
            nearest_front, valid_fraction, None,
        )

    closest_index, closest_range = min(
        valid_obstacles, key=lambda indexed_value: indexed_value[1]
    )
    cluster_limit = closest_range + settings.closest_cluster_tolerance
    cluster_start = closest_index
    cluster_end = closest_index
    while (
        cluster_start > 0
        and values[cluster_start - 1] is not None
        and values[cluster_start - 1] <= cluster_limit
    ):
        cluster_start -= 1
    while (
        cluster_end + 1 < len(values)
        and values[cluster_end + 1] is not None
        and values[cluster_end + 1] <= cluster_limit
    ):
        cluster_end += 1

    clearance_radius = settings.vehicle_width / 2.0 + settings.safety_margin
    bubble_half_width = math.atan2(clearance_radius, closest_range)
    closest_angle = (angles[cluster_start] + angles[cluster_end]) / 2.0

    bubbled_values = list(values)
    for index, angle in enumerate(angles):
        if abs(angle - closest_angle) <= bubble_half_width:
            bubbled_values[index] = None

    minimum_gap_points = max(2, math.ceil(settings.minimum_gap_width / angular_step))
    candidate_gaps = [
        gap
        for gap in find_open_runs(
            bubbled_values, settings.minimum_gap_distance
        )
        if gap[1] - gap[0] + 1 >= minimum_gap_points
    ]

    if not candidate_gaps:
        return GapDecision(
            False,
            "no opening meets the minimum distance and width",
            "STOP",
            None,
            None,
            None,
            None,
            nearest_front,
            valid_fraction,
            bubble_half_width,
        )

    # Standard Follow-the-Gap first chooses the widest contiguous opening.
    # Clearance and forward alignment break near-ties deterministically.
    def gap_rank(gap: tuple[int, int]) -> tuple[int, float, float]:
        start, end = gap
        gap_values = [
            value
            for value in bubbled_values[start:end + 1]
            if value is not None
        ]
        center_angle = (angles[start] + angles[end]) / 2.0
        return (
            end - start + 1,
            sum(gap_values) / len(gap_values),
            -abs(center_angle),
        )

    maximum_gap_points = max(end - start + 1 for start, end in candidate_gaps)
    near_tie_points = max(1, round(math.radians(1.0) / angular_step))
    effectively_widest_gaps = [
        gap
        for gap in candidate_gaps
        if gap[1] - gap[0] + 1 >= maximum_gap_points - near_tie_points
    ]
    gap_start, gap_end = max(
        effectively_widest_gaps,
        key=lambda gap: gap_rank(gap)[1:],
    )
    target_window_points = max(
        1, round(settings.target_window_width / angular_step)
    )
    target_index, target_clearance = choose_target_index(
        angles,
        bubbled_values,
        gap_start,
        gap_end,
        target_window_points,
    )

    target_angle = angles[target_index]
    steering_angle = clamp(
        target_angle,
        -settings.max_steering_angle,
        settings.max_steering_angle,
    )
    gap_width = angles[gap_end] - angles[gap_start] + angular_step

    direction_threshold = math.radians(3.0)
    if steering_angle > direction_threshold:
        direction = "LEFT"
    elif steering_angle < -direction_threshold:
        direction = "RIGHT"
    else:
        direction = "STRAIGHT"

    safe = True
    reason = "usable gap found"
    if nearest_front is None:
        safe = False
        reason = "no valid data directly ahead"
        direction = "STOP"
    elif nearest_front <= settings.emergency_stop_distance:
        safe = False
        reason = (
            f"front obstacle at {nearest_front:.2f} m is inside the "
            f"{settings.emergency_stop_distance:.2f} m emergency distance"
        )
        direction = "STOP"

    return GapDecision(
        safe,
        reason,
        direction,
        target_angle,
        steering_angle,
        gap_width,
        target_clearance,
        nearest_front,
        valid_fraction,
        bubble_half_width,
    )


def run_self_test() -> None:
    """Exercise left, right, blocked, and emergency decisions without ROS."""
    settings = PlannerSettings()
    angle_min = math.radians(45.0)
    angle_increment = math.radians(0.1)
    reading_count = 2700

    def synthetic_scan(open_side: str) -> list[float]:
        scan = [0.90] * reading_count
        for index in range(reading_count):
            raw_degrees = 45.0 + index * 0.1
            if 170.0 <= raw_degrees <= 190.0:
                scan[index] = 0.50
            if open_side == "left" and 220.0 <= raw_degrees <= 270.0:
                scan[index] = 4.0
            if open_side == "right" and 90.0 <= raw_degrees <= 140.0:
                scan[index] = 4.0
        return scan

    left_decision = plan_follow_the_gap(
        synthetic_scan("left"), angle_min, angle_increment, 0.05, 30.0,
        settings,
    )
    assert left_decision.safe and left_decision.direction == "LEFT"

    right_decision = plan_follow_the_gap(
        synthetic_scan("right"), angle_min, angle_increment, 0.05, 30.0,
        settings,
    )
    assert right_decision.safe and right_decision.direction == "RIGHT"

    blocked_decision = plan_follow_the_gap(
        [0.40] * reading_count, angle_min, angle_increment, 0.05, 30.0,
        settings,
    )
    assert not blocked_decision.safe and blocked_decision.direction == "STOP"

    emergency_scan = synthetic_scan("left")
    for index in range(reading_count):
        raw_degrees = 45.0 + index * 0.1
        if 178.0 <= raw_degrees <= 182.0:
            emergency_scan[index] = 0.20
    emergency_decision = plan_follow_the_gap(
        emergency_scan, angle_min, angle_increment, 0.05, 30.0, settings,
    )
    assert not emergency_decision.safe
    assert emergency_decision.direction == "STOP"

    print("Self-test passed: LEFT, RIGHT, blocked, and emergency cases.")


# Keep offline testing usable on a computer that does not have ROS 2 installed.
if __name__ == "__main__" and "--self-test" in sys.argv:
    run_self_test()
    raise SystemExit(0)


import rclpy  # noqa: E402  (intentionally after the offline self-test)
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402


class SL450FollowTheGapTest(Node):
    """Report Follow-the-Gap decisions without publishing drive commands."""

    def __init__(self) -> None:
        super().__init__("sl450_follow_the_gap_test")

        self.declare_parameter("scan_topic", "/lidar/scan/points")
        self.declare_parameter("forward_raw_angle_degrees", 180.0)
        self.declare_parameter("planning_half_fov_degrees", 100.0)
        self.declare_parameter("max_considered_range_m", 8.0)
        self.declare_parameter("median_window_points", 5)
        self.declare_parameter("max_invalid_run_degrees", 1.5)
        self.declare_parameter("vehicle_width_m", 0.32)
        self.declare_parameter("safety_margin_m", 0.08)
        self.declare_parameter("closest_cluster_tolerance_m", 0.03)
        self.declare_parameter("minimum_gap_distance_m", 0.60)
        self.declare_parameter("minimum_gap_width_degrees", 12.0)
        self.declare_parameter("target_window_width_degrees", 8.0)
        self.declare_parameter("max_steering_angle_degrees", 30.0)
        self.declare_parameter("emergency_stop_distance_m", 0.30)
        self.declare_parameter("front_check_half_width_degrees", 15.0)
        self.declare_parameter("minimum_valid_fraction", 0.70)
        self.declare_parameter("output_rate_hz", 2.0)
        self.declare_parameter("scan_timeout_seconds", 0.25)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.settings = PlannerSettings(
            forward_raw_angle=math.radians(float(
                self.get_parameter("forward_raw_angle_degrees").value
            )),
            planning_half_fov=math.radians(float(
                self.get_parameter("planning_half_fov_degrees").value
            )),
            max_considered_range=float(
                self.get_parameter("max_considered_range_m").value
            ),
            median_window_points=int(
                self.get_parameter("median_window_points").value
            ),
            max_invalid_run_angle=math.radians(float(
                self.get_parameter("max_invalid_run_degrees").value
            )),
            vehicle_width=float(self.get_parameter("vehicle_width_m").value),
            safety_margin=float(self.get_parameter("safety_margin_m").value),
            closest_cluster_tolerance=float(
                self.get_parameter("closest_cluster_tolerance_m").value
            ),
            minimum_gap_distance=float(
                self.get_parameter("minimum_gap_distance_m").value
            ),
            minimum_gap_width=math.radians(float(
                self.get_parameter("minimum_gap_width_degrees").value
            )),
            target_window_width=math.radians(float(
                self.get_parameter("target_window_width_degrees").value
            )),
            max_steering_angle=math.radians(float(
                self.get_parameter("max_steering_angle_degrees").value
            )),
            emergency_stop_distance=float(
                self.get_parameter("emergency_stop_distance_m").value
            ),
            front_check_half_width=math.radians(float(
                self.get_parameter("front_check_half_width_degrees").value
            )),
            minimum_valid_fraction=float(
                self.get_parameter("minimum_valid_fraction").value
            ),
        )
        output_rate_hz = float(self.get_parameter("output_rate_hz").value)
        self.scan_timeout_seconds = float(
            self.get_parameter("scan_timeout_seconds").value
        )
        self._validate_parameters(output_rate_hz)

        self.latest_decision: Optional[GapDecision] = None
        self.last_scan_time = None
        self.have_logged_geometry = False
        self.timeout_warning_active = False

        self.subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.output_timer = self.create_timer(
            1.0 / output_rate_hz,
            self.report_decision,
        )

        self.get_logger().warning(
            "STATIONARY TEST ONLY: this node analyzes LiDAR data but publishes "
            "no motor or steering commands."
        )
        self.get_logger().info(
            f"Listening for SL450 scans on {self.scan_topic}."
        )

    def _validate_parameters(self, output_rate_hz: float) -> None:
        settings = self.settings
        if not 0.0 < settings.planning_half_fov <= math.pi:
            raise ValueError("planning_half_fov_degrees must be in (0, 180]")
        if settings.max_considered_range <= 0.0:
            raise ValueError("max_considered_range_m must be greater than zero")
        if settings.median_window_points < 1:
            raise ValueError("median_window_points must be at least one")
        if settings.median_window_points % 2 == 0:
            raise ValueError("median_window_points must be odd")
        if settings.max_invalid_run_angle < 0.0:
            raise ValueError("max_invalid_run_degrees cannot be negative")
        if settings.vehicle_width <= 0.0:
            raise ValueError("vehicle_width_m must be greater than zero")
        if settings.safety_margin < 0.0:
            raise ValueError("safety_margin_m cannot be negative")
        if settings.closest_cluster_tolerance < 0.0:
            raise ValueError("closest_cluster_tolerance_m cannot be negative")
        if settings.minimum_gap_distance <= 0.0:
            raise ValueError("minimum_gap_distance_m must be greater than zero")
        if settings.minimum_gap_width <= 0.0:
            raise ValueError("minimum_gap_width_degrees must be greater than zero")
        if settings.target_window_width <= 0.0:
            raise ValueError("target_window_width_degrees must be greater than zero")
        if settings.max_steering_angle <= 0.0:
            raise ValueError("max_steering_angle_degrees must be greater than zero")
        if settings.emergency_stop_distance <= 0.0:
            raise ValueError("emergency_stop_distance_m must be greater than zero")
        if settings.front_check_half_width <= 0.0:
            raise ValueError(
                "front_check_half_width_degrees must be greater than zero"
            )
        if not 0.0 < settings.minimum_valid_fraction <= 1.0:
            raise ValueError("minimum_valid_fraction must be in (0, 1]")
        if output_rate_hz <= 0.0:
            raise ValueError("output_rate_hz must be greater than zero")
        if self.scan_timeout_seconds <= 0.0:
            raise ValueError("scan_timeout_seconds must be greater than zero")

    def scan_callback(self, scan: LaserScan) -> None:
        self.last_scan_time = self.get_clock().now()
        self.latest_decision = plan_follow_the_gap(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self.settings,
        )

        if self.timeout_warning_active:
            self.get_logger().info("LiDAR scans resumed.")
            self.timeout_warning_active = False

        if not self.have_logged_geometry:
            self.get_logger().info(
                "Scan geometry: "
                f"{len(scan.ranges)} readings, "
                f"{math.degrees(scan.angle_min):.2f} deg to "
                f"{math.degrees(scan.angle_max):.2f} deg, "
                f"{math.degrees(scan.angle_increment):.3f} deg increments."
            )
            self.have_logged_geometry = True

    @staticmethod
    def _format_optional_distance(value: Optional[float]) -> str:
        return "NO DATA" if value is None else f"{value:.2f} m"

    def report_decision(self) -> None:
        if self.latest_decision is None or self.last_scan_time is None:
            if not self.timeout_warning_active:
                self.get_logger().warning(
                    f"Waiting for LaserScan data on {self.scan_topic}."
                )
                self.timeout_warning_active = True
            return

        scan_age_seconds = (
            self.get_clock().now() - self.last_scan_time
        ).nanoseconds / 1_000_000_000.0

        if scan_age_seconds > self.scan_timeout_seconds:
            if not self.timeout_warning_active:
                self.get_logger().error(
                    f"STOP | LiDAR data is stale ({scan_age_seconds:.2f} s old)."
                )
                self.timeout_warning_active = True
            return

        decision = self.latest_decision
        if not decision.safe:
            self.get_logger().error(
                "STOP | "
                f"{decision.reason} | "
                "front: "
                f"{self._format_optional_distance(decision.nearest_front)} | "
                f"valid scan: {decision.valid_fraction:.0%} | "
                "STATIONARY TEST"
            )
            return

        assert decision.target_angle is not None
        assert decision.steering_angle is not None
        assert decision.gap_width is not None
        assert decision.target_clearance is not None

        raw_target_angle = normalize_angle(
            decision.target_angle + self.settings.forward_raw_angle
        )
        raw_target_degrees = math.degrees(raw_target_angle) % 360.0

        self.get_logger().info(
            f"PROPOSED: {decision.direction:<8} | "
            f"steering {math.degrees(decision.steering_angle):+5.1f} deg | "
            f"gap target {math.degrees(decision.target_angle):+5.1f} deg "
            f"(raw {raw_target_degrees:5.1f} deg) | "
            f"gap width {math.degrees(decision.gap_width):5.1f} deg | "
            f"target clearance {decision.target_clearance:.2f} m | "
            f"front {decision.nearest_front:.2f} m | "
            "STATIONARY TEST"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = SL450FollowTheGapTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

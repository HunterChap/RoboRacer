#!/usr/bin/env python3
"""Safely inspect left, front, and right clearances from an Orbbec SL450.

This is a LiDAR-only ROS 2 node. It subscribes to the SL450 LaserScan topic
and prints human-readable clearance estimates. It does not publish motor,
steering, or Ackermann drive commands.

The defaults match the orientation measured on the RoboRacer:

    raw 90 degrees  = car right
    raw 180 degrees = car forward
    raw 270 degrees = car left

Example:
    source /opt/ros/jazzy/setup.bash
    source ~/orbbec_ws/install/setup.bash
    python3 sl450_clearance_inspector.py
"""

from __future__ import annotations

import math
from typing import Iterable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def normalize_angle(angle_radians: float) -> float:
    """Return an angle in the interval [-pi, pi)."""
    return (angle_radians + math.pi) % (2.0 * math.pi) - math.pi


def percentile(values: Iterable[float], percent: float) -> Optional[float]:
    """Return a linearly interpolated percentile, or None for no values."""
    ordered = sorted(values)
    if not ordered:
        return None

    position = (len(ordered) - 1) * percent / 100.0
    lower_index = math.floor(position)
    upper_index = math.ceil(position)

    if lower_index == upper_index:
        return ordered[lower_index]

    fraction = position - lower_index
    return (
        ordered[lower_index] * (1.0 - fraction)
        + ordered[upper_index] * fraction
    )


class SL450ClearanceInspector(Node):
    """Report robust clearance estimates from three LiDAR sectors."""

    def __init__(self) -> None:
        super().__init__("sl450_clearance_inspector")

        self.declare_parameter("scan_topic", "/lidar/scan/points")
        self.declare_parameter("sector_half_width_degrees", 7.5)
        self.declare_parameter("clearance_percentile", 10.0)
        self.declare_parameter("output_rate_hz", 2.0)
        self.declare_parameter("scan_timeout_seconds", 0.5)
        self.declare_parameter("minimum_valid_points", 5)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.sector_half_width = math.radians(
            float(self.get_parameter("sector_half_width_degrees").value)
        )
        self.clearance_percentile = float(
            self.get_parameter("clearance_percentile").value
        )
        output_rate_hz = float(self.get_parameter("output_rate_hz").value)
        self.scan_timeout_seconds = float(
            self.get_parameter("scan_timeout_seconds").value
        )
        self.minimum_valid_points = int(
            self.get_parameter("minimum_valid_points").value
        )

        self._validate_parameters(output_rate_hz)

        # These raw SL450 angles were measured with the LiDAR installed on the car.
        self.direction_angles = {
            "LEFT": math.radians(270.0),
            "FRONT": math.radians(180.0),
            "RIGHT": math.radians(90.0),
        }

        self.latest_scan: Optional[LaserScan] = None
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
            self.report_clearances,
        )

        self.get_logger().info(
            f"Listening for SL450 scans on {self.scan_topic}. "
            "This node cannot command the car."
        )

    def _validate_parameters(self, output_rate_hz: float) -> None:
        if not 0.0 < self.sector_half_width <= math.pi:
            raise ValueError("sector_half_width_degrees must be between 0 and 180")
        if not 0.0 <= self.clearance_percentile <= 100.0:
            raise ValueError("clearance_percentile must be between 0 and 100")
        if output_rate_hz <= 0.0:
            raise ValueError("output_rate_hz must be greater than zero")
        if self.scan_timeout_seconds <= 0.0:
            raise ValueError("scan_timeout_seconds must be greater than zero")
        if self.minimum_valid_points < 1:
            raise ValueError("minimum_valid_points must be at least one")

    def scan_callback(self, scan: LaserScan) -> None:
        self.latest_scan = scan
        self.last_scan_time = self.get_clock().now()

        if self.timeout_warning_active:
            self.get_logger().info("LiDAR scans resumed.")
            self.timeout_warning_active = False

        if not self.have_logged_geometry:
            self._log_scan_geometry(scan)
            self.have_logged_geometry = True

    def _log_scan_geometry(self, scan: LaserScan) -> None:
        self.get_logger().info(
            "Scan geometry: "
            f"{len(scan.ranges)} readings, "
            f"{math.degrees(scan.angle_min):.2f} deg to "
            f"{math.degrees(scan.angle_max):.2f} deg, "
            f"{math.degrees(scan.angle_increment):.3f} deg increments, "
            f"valid range {scan.range_min:.2f}-{scan.range_max:.2f} m."
        )

    def _valid_sector_ranges(
        self,
        scan: LaserScan,
        center_angle: float,
    ) -> list[float]:
        valid_ranges: list[float] = []

        for index, measured_range in enumerate(scan.ranges):
            point_angle = scan.angle_min + index * scan.angle_increment
            angular_error = abs(normalize_angle(point_angle - center_angle))

            if angular_error > self.sector_half_width:
                continue
            if not math.isfinite(measured_range):
                continue
            if measured_range < scan.range_min or measured_range > scan.range_max:
                continue

            valid_ranges.append(float(measured_range))

        return valid_ranges

    def _sector_clearance(
        self,
        scan: LaserScan,
        center_angle: float,
    ) -> tuple[Optional[float], int]:
        valid_ranges = self._valid_sector_ranges(scan, center_angle)

        if len(valid_ranges) < self.minimum_valid_points:
            return None, len(valid_ranges)

        return (
            percentile(valid_ranges, self.clearance_percentile),
            len(valid_ranges),
        )

    @staticmethod
    def _format_clearance(clearance: Optional[float], point_count: int) -> str:
        if clearance is None:
            return f"NO DATA ({point_count} valid)"
        return f"{clearance:5.2f} m ({point_count} valid)"

    def report_clearances(self) -> None:
        if self.latest_scan is None or self.last_scan_time is None:
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
                    f"LiDAR data is stale ({scan_age_seconds:.2f} s old). "
                    "Do not drive the car."
                )
                self.timeout_warning_active = True
            return

        results = {
            name: self._sector_clearance(self.latest_scan, angle)
            for name, angle in self.direction_angles.items()
        }

        output = " | ".join(
            f"{name}: {self._format_clearance(*results[name])}"
            for name in ("LEFT", "FRONT", "RIGHT")
        )
        self.get_logger().info(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = SL450ClearanceInspector()
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

#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String


@dataclass
class VehicleState:
    requested_speed: Optional[float] = None
    requested_turn: Optional[float] = None
    filtered_speed: Optional[float] = None
    filtered_turn: Optional[float] = None
    final_speed: Optional[float] = None
    final_turn: Optional[float] = None
    drive_speed: Optional[float] = None
    steering_angle_rad: Optional[float] = None
    actual_speed: Optional[float] = None
    actual_yaw_rate: Optional[float] = None
    front_distance: Optional[float] = None
    safety_scale: Optional[float] = None
    safety_state: str = "waiting"
    drive_mode: str = "unknown"
    command_source: str = "unknown"


class SimVehicleMonitor(Node):
    def __init__(self) -> None:
        super().__init__("sim_vehicle_monitor")
        self.state = VehicleState()

        self.create_subscription(Twist, "/cmd_vel_requested", self._requested_callback, 10)
        self.create_subscription(Twist, "/cmd_vel_safety_filtered", self._filtered_callback, 10)
        self.create_subscription(Twist, "/cmd_vel_safe", self._final_callback, 10)
        self.create_subscription(AckermannDriveStamped, "/drive", self._drive_callback, 10)
        self.create_subscription(Odometry, "/ego_racecar/odom", self._odom_callback, 10)
        self.create_subscription(Float32, "/front_distance", self._front_distance_callback, 10)
        self.create_subscription(Float32, "/safety/speed_scale", self._safety_scale_callback, 10)
        self.create_subscription(String, "/safety/state", self._safety_state_callback, 10)
        self.create_subscription(String, "/drive_switch_state", self._drive_mode_callback, 10)
        self.create_subscription(String, "/command_source", self._command_source_callback, 10)

        self.create_timer(0.25, self._render)

    def _requested_callback(self, msg: Twist) -> None:
        self.state.requested_speed = msg.linear.x
        self.state.requested_turn = msg.angular.z

    def _filtered_callback(self, msg: Twist) -> None:
        self.state.filtered_speed = msg.linear.x
        self.state.filtered_turn = msg.angular.z

    def _final_callback(self, msg: Twist) -> None:
        self.state.final_speed = msg.linear.x
        self.state.final_turn = msg.angular.z

    def _drive_callback(self, msg: AckermannDriveStamped) -> None:
        self.state.drive_speed = msg.drive.speed
        self.state.steering_angle_rad = msg.drive.steering_angle

    def _odom_callback(self, msg: Odometry) -> None:
        self.state.actual_speed = msg.twist.twist.linear.x
        self.state.actual_yaw_rate = msg.twist.twist.angular.z

    def _front_distance_callback(self, msg: Float32) -> None:
        self.state.front_distance = msg.data

    def _safety_scale_callback(self, msg: Float32) -> None:
        self.state.safety_scale = msg.data

    def _safety_state_callback(self, msg: String) -> None:
        self.state.safety_state = msg.data

    def _drive_mode_callback(self, msg: String) -> None:
        self.state.drive_mode = msg.data

    def _command_source_callback(self, msg: String) -> None:
        self.state.command_source = msg.data

    @staticmethod
    def _number(value: Optional[float], digits: int = 3) -> str:
        if value is None or not math.isfinite(value):
            return "   --   "
        return f"{value: .{digits}f}"

    @staticmethod
    def _degrees(value: Optional[float]) -> str:
        if value is None or not math.isfinite(value):
            return "   --   "
        return f"{math.degrees(value): .1f}"

    def _render(self) -> None:
        s = self.state
        print("\033[2J\033[H", end="")
        print("╔════════════════ RoboRacer Simulator Monitor ════════════════╗")
        print(f"  Drive mode        : {s.drive_mode}")
        print(f"  Command source    : {s.command_source}")
        print(f"  Safety state      : {s.safety_state}")
        print(f"  Safety speed scale: {self._number(s.safety_scale, 2)}")
        print(f"  Front distance    : {self._number(s.front_distance, 2)} m")
        print("├──────────────────── Command Pipeline ───────────────────────┤")
        print(
            "  Requested         : "
            f"speed {self._number(s.requested_speed)} m/s   "
            f"turn-z {self._number(s.requested_turn)} rad/s"
        )
        print(
            "  After AEB         : "
            f"speed {self._number(s.filtered_speed)} m/s   "
            f"turn-z {self._number(s.filtered_turn)} rad/s"
        )
        print(
            "  Final command     : "
            f"speed {self._number(s.final_speed)} m/s   "
            f"turn-z {self._number(s.final_turn)} rad/s"
        )
        print("├──────────────────── Vehicle Output ─────────────────────────┤")
        print(
            "  Ackermann target  : "
            f"speed {self._number(s.drive_speed)} m/s   "
            f"steer {self._number(s.steering_angle_rad)} rad "
            f"({self._degrees(s.steering_angle_rad)} deg)"
        )
        print(
            "  Actual odometry   : "
            f"speed {self._number(s.actual_speed)} m/s   "
            f"yaw-rate {self._number(s.actual_yaw_rate)} rad/s"
        )
        print("╚══════════════════════════════════════════════════════════════╝")
        print("Refresh: 4 Hz   Exit: Ctrl+C")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimVehicleMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

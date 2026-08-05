import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class CmdVelToAckermannNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann_node')

        # Vehicle geometry and command-conversion limits.
        self.declare_parameter('wheelbase_m', 0.33)
        self.declare_parameter('max_speed_mps', 2.0)
        self.declare_parameter('max_reverse_speed_mps', 2.0)
        self.declare_parameter('max_steering_angle_rad', 0.65)
        self.declare_parameter('min_speed_for_steering_mps', 0.05)
        self.declare_parameter('low_speed_turn_speed_mps', 0.20)

        self.wheelbase_m = float(self.get_parameter('wheelbase_m').value)
        self.max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self.max_reverse_speed_mps = float(
            self.get_parameter('max_reverse_speed_mps').value
        )
        self.max_steering_angle_rad = float(
            self.get_parameter('max_steering_angle_rad').value
        )
        self.min_speed_for_steering_mps = float(
            self.get_parameter('min_speed_for_steering_mps').value
        )
        self.low_speed_turn_speed_mps = float(
            self.get_parameter('low_speed_turn_speed_mps').value
        )

        if self.wheelbase_m <= 0.0:
            raise ValueError('wheelbase_m must be greater than zero.')
        if self.max_speed_mps <= 0.0:
            raise ValueError('max_speed_mps must be greater than zero.')
        if self.max_reverse_speed_mps <= 0.0:
            raise ValueError('max_reverse_speed_mps must be greater than zero.')
        if self.max_steering_angle_rad <= 0.0:
            raise ValueError('max_steering_angle_rad must be greater than zero.')
        if self.min_speed_for_steering_mps < 0.0:
            raise ValueError('min_speed_for_steering_mps must not be negative.')

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive_target',
            10,
        )

        self.get_logger().info('cmd_vel_to_ackermann_node started.')
        self.get_logger().info('Subscribing: /cmd_vel')
        self.get_logger().info('Publishing: /drive_target')
        self.get_logger().info(
            'Speed limits: '
            f'forward={self.max_speed_mps:.2f} m/s, '
            f'reverse={self.max_reverse_speed_mps:.2f} m/s'
        )

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def convert_cmd_vel(self, linear_x: float, angular_z: float):
        raw_speed = float(linear_x)
        raw_yaw_rate = float(angular_z)

        if not math.isfinite(raw_speed) or not math.isfinite(raw_yaw_rate):
            self.get_logger().error(
                'Non-finite cmd_vel detected; publishing zero command.'
            )
            return 0.0, 0.0

        speed = self.clamp(
            raw_speed,
            -self.max_reverse_speed_mps,
            self.max_speed_mps,
        )

        # Preserve the requested path curvature when a speed limit clamps the
        # command. Without this scaling, clipping -2.0 m/s to -1.0 m/s while
        # leaving angular.z unchanged doubles reverse steering curvature.
        yaw_rate = raw_yaw_rate
        if abs(raw_speed) > 1e-6 and abs(speed - raw_speed) > 1e-9:
            yaw_rate *= speed / raw_speed

        # A zero command produces zero speed and steering.
        if abs(speed) < 1e-6 and abs(yaw_rate) < 1e-6:
            return 0.0, 0.0

        # Ackermann vehicles cannot rotate in place. Ignore angular commands
        # while translational speed is too close to zero.
        if abs(speed) < self.min_speed_for_steering_mps:
            return 0.0, 0.0

        # Bicycle model:
        # angular_z = v / L * tan(delta)
        # delta = atan(L * angular_z / v)
        steering_angle = math.atan(
            self.wheelbase_m * yaw_rate / speed
        )

        steering_angle = self.clamp(
            steering_angle,
            -self.max_steering_angle_rad,
            self.max_steering_angle_rad,
        )

        return speed, steering_angle

    def cmd_vel_callback(self, msg: Twist) -> None:
        speed, steering_angle = self.convert_cmd_vel(
            msg.linear.x,
            msg.angular.z,
        )

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'

        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle

        self.drive_pub.publish(drive_msg)

        self.get_logger().info(
            f'/cmd_vel linear.x={msg.linear.x:.2f}, '
            f'angular.z={msg.angular.z:.2f} '
            f'-> /drive_target speed={speed:.2f}, '
            f'steering_angle={steering_angle:.2f}',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermannNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

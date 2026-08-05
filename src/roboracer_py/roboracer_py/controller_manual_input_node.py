import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class ControllerManualInputNode(Node):
    """Convert standardized game-controller input into a dedicated ROS command.

    This node does not select Auto, Manual, or Stop Mode. It only publishes the
    current controller command and connection/button state. Final authority is
    decided by controller_priority_mux_node.

    The steering and throttle axes use separate deadbands. This is important
    when one physical stick provides both axes: pulling the stick straight back
    can otherwise create a small unintended steering command.
    """

    def __init__(self):
        super().__init__('controller_manual_input_node')

        self.declare_parameter('steering_axis', 0)
        self.declare_parameter('throttle_axis', 1)
        self.declare_parameter('stop_button', 1)
        self.declare_parameter('release_button', 10)

        self.declare_parameter('max_speed_mps', 2.0)
        self.declare_parameter('max_turn_angular_z', 2.20)
        self.declare_parameter('reverse_steering_scale', 0.60)

        # Keep the combined deadband parameter for launch compatibility.
        # Current launch files should use the axis-specific parameters below.
        self.declare_parameter('deadband', 0.08)
        self.declare_parameter('throttle_deadband', 0.08)
        self.declare_parameter('steering_deadband', 0.10)
        self.declare_parameter('reverse_steering_deadband', 0.25)

        self.declare_parameter('invert_throttle', False)
        self.declare_parameter('invert_steering', False)

        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('joy_timeout_sec', 0.50)

        self.steering_axis = int(self.get_parameter('steering_axis').value)
        self.throttle_axis = int(self.get_parameter('throttle_axis').value)
        self.stop_button = int(self.get_parameter('stop_button').value)
        self.release_button = int(self.get_parameter('release_button').value)

        self.max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self.max_turn_angular_z = float(
            self.get_parameter('max_turn_angular_z').value
        )
        self.reverse_steering_scale = float(
            self.get_parameter('reverse_steering_scale').value
        )
        self.throttle_deadband = float(
            self.get_parameter('throttle_deadband').value
        )
        self.steering_deadband = float(
            self.get_parameter('steering_deadband').value
        )
        self.reverse_steering_deadband = float(
            self.get_parameter('reverse_steering_deadband').value
        )
        self.invert_throttle = bool(
            self.get_parameter('invert_throttle').value
        )
        self.invert_steering = bool(
            self.get_parameter('invert_steering').value
        )

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.joy_timeout_sec = float(
            self.get_parameter('joy_timeout_sec').value
        )

        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero.')
        if self.joy_timeout_sec <= 0.0:
            raise ValueError('joy_timeout_sec must be greater than zero.')
        if self.max_speed_mps <= 0.0:
            raise ValueError('max_speed_mps must be greater than zero.')
        if self.max_turn_angular_z < 0.0:
            raise ValueError('max_turn_angular_z must not be negative.')
        if not 0.0 < self.reverse_steering_scale <= 1.0:
            raise ValueError('reverse_steering_scale must be in (0.0, 1.0].')
        if not 0.0 <= self.throttle_deadband < 1.0:
            raise ValueError('throttle_deadband must be in [0.0, 1.0).')
        if not 0.0 <= self.steering_deadband < 1.0:
            raise ValueError('steering_deadband must be in [0.0, 1.0).')
        if not 0.0 <= self.reverse_steering_deadband < 1.0:
            raise ValueError(
                'reverse_steering_deadband must be in [0.0, 1.0).'
            )

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10,
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            '/controller_cmd_vel',
            10,
        )
        self.stop_pub = self.create_publisher(
            Bool,
            '/controller_stop',
            10,
        )
        self.release_pub = self.create_publisher(
            Bool,
            '/controller_release',
            10,
        )
        self.connected_pub = self.create_publisher(
            Bool,
            '/controller_connected',
            10,
        )

        self.last_joy: Joy | None = None
        self.last_joy_time: Time | None = None

        self.previous_stop_pressed = False
        self.previous_release_pressed = False
        self.previous_connected: bool | None = None
        self.warned_axis_indices: set[int] = set()
        self.warned_button_indices: set[int] = set()

        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.publish_control,
        )

        self.get_logger().info(
            'Controller input started: /joy -> /controller_cmd_vel'
        )
        self.get_logger().info(
            'Controller limits: '
            f'speed=+/-{self.max_speed_mps:.2f} m/s, '
            f'max_yaw_rate={self.max_turn_angular_z:.2f} rad/s, '
            f'reverse_steering_scale={self.reverse_steering_scale:.2f}, '
            f'throttle_deadband={self.throttle_deadband:.2f}, '
            f'forward_steering_deadband={self.steering_deadband:.2f}, '
            f'reverse_steering_deadband={self.reverse_steering_deadband:.2f}'
        )
        self.get_logger().info(
            'Neutral stick and stale /joy data always publish zero velocity.'
        )

    def joy_callback(self, msg: Joy) -> None:
        self.last_joy = msg
        self.last_joy_time = self.get_clock().now()

    @staticmethod
    def clamp_axis(value: float) -> float:
        return max(-1.0, min(1.0, float(value)))

    @staticmethod
    def apply_deadband(value: float, deadband: float) -> float:
        """Apply a deadband and linearly rescale the remaining axis range.

        The output remains linear outside the deadband and still reaches
        exactly -1.0 or +1.0 at full stick deflection.
        """
        value = ControllerManualInputNode.clamp_axis(value)
        magnitude = abs(value)

        if magnitude <= deadband:
            return 0.0

        rescaled = (magnitude - deadband) / (1.0 - deadband)
        return math.copysign(rescaled, value)

    def get_axis(self, msg: Joy, index: int) -> float:
        if 0 <= index < len(msg.axes):
            return self.clamp_axis(msg.axes[index])

        if index not in self.warned_axis_indices:
            self.get_logger().error(
                f'Axis index {index} is invalid; axes length={len(msg.axes)}.'
            )
            self.warned_axis_indices.add(index)
        return 0.0

    def get_button(self, msg: Joy, index: int) -> int:
        if index < 0:
            return 0
        if 0 <= index < len(msg.buttons):
            return int(msg.buttons[index])

        if index not in self.warned_button_indices:
            self.get_logger().error(
                f'Button index {index} is invalid; '
                f'buttons length={len(msg.buttons)}.'
            )
            self.warned_button_indices.add(index)
        return 0

    def publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())

    def publish_connection_state(self, connected: bool) -> None:
        msg = Bool()
        msg.data = connected
        self.connected_pub.publish(msg)

        if connected == self.previous_connected:
            return

        if connected:
            self.get_logger().info('Controller connected and ready.')
        elif self.previous_connected is True:
            self.get_logger().warn(
                'Controller connection lost; zero velocity is being published.'
            )
        else:
            self.get_logger().info('Waiting for controller input...')

        self.previous_connected = connected

    def publish_button_edges(self, msg: Joy) -> None:
        stop_pressed = self.get_button(msg, self.stop_button) == 1
        release_pressed = self.get_button(msg, self.release_button) == 1

        if stop_pressed and not self.previous_stop_pressed:
            stop_msg = Bool()
            stop_msg.data = True
            self.stop_pub.publish(stop_msg)
            self.get_logger().warn('Controller STOP requested.')

        if release_pressed and not self.previous_release_pressed:
            release_msg = Bool()
            release_msg.data = True
            self.release_pub.publish(release_msg)
            self.get_logger().info('Controller override release requested.')

        self.previous_stop_pressed = stop_pressed
        self.previous_release_pressed = release_pressed

    def joy_is_fresh(self, now: Time) -> bool:
        if self.last_joy is None or self.last_joy_time is None:
            return False
        joy_age = (now - self.last_joy_time).nanoseconds / 1e9
        return joy_age <= self.joy_timeout_sec

    def publish_control(self) -> None:
        now = self.get_clock().now()

        if not self.joy_is_fresh(now):
            self.publish_connection_state(False)
            self.publish_zero()
            return

        self.publish_connection_state(True)
        msg = self.last_joy
        if msg is None:
            self.publish_zero()
            return

        self.publish_button_edges(msg)

        throttle = self.get_axis(msg, self.throttle_axis)
        steering = self.get_axis(msg, self.steering_axis)

        if self.invert_throttle:
            throttle = -throttle
        if self.invert_steering:
            steering = -steering

        throttle = self.apply_deadband(throttle, self.throttle_deadband)

        # Pulling one physical stick backward can create substantial sideways
        # cross-axis input. Use a larger deadband only in reverse, so forward
        # steering remains responsive while straight reversing stays stable.
        steering_deadband = (
            self.reverse_steering_deadband
            if throttle < 0.0
            else self.steering_deadband
        )
        steering = self.apply_deadband(steering, steering_deadband)

        cmd = Twist()
        cmd.linear.x = float(throttle * self.max_speed_mps)

        # The downstream converter uses:
        #   steering_angle = atan(L * angular_z / linear_x)
        # Keep angular.z proportional to throttle so forward and reverse retain
        # the same steering sign. Reverse steering is deliberately reduced to
        # avoid unstable tight reverse arcs at 2.0 m/s.
        if abs(throttle) < 1e-6 or abs(steering) < 1e-6:
            cmd.angular.z = 0.0
        else:
            direction_scale = (
                self.reverse_steering_scale
                if throttle < 0.0
                else 1.0
            )
            cmd.angular.z = float(
                steering
                * throttle
                * self.max_turn_angular_z
                * direction_scale
            )

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerManualInputNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Controller manual input interrupted.')
    finally:
        if rclpy.ok():
            node.publish_connection_state(False)
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

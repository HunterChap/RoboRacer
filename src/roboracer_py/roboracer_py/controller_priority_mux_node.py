import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class ControllerPriorityMuxNode(Node):
    """Final priority mux for controller temporary override.

    Base mode remains Stop / Auto / Manual. Controller permission is a
    separate enable switch published by terminal_command_node.

    Behavior:
      - Stop Mode always outputs zero.
      - Auto Active: controller overrides while the stick is moved; neutral
        immediately returns control to autonomous driving.
      - Auto Hold: controller overrides while moved; neutral returns to hold.
      - Manual: controller takeover cancels the old terminal motion. Neutral
        leaves Manual Mode stopped and never restores the old keyboard command.
      - Controller disconnect during an active override stops and holds until
        the controller reconnects with the stick centered.
    """

    STOP_STATES = {'stop', 'stop_mode', 'disarm', 'emergency_stop', 'emergency_hold'}
    AUTO_STATES = {'auto_hold', 'auto_active'}
    MANUAL_STATES = {'manual_hold', 'manual_active'}

    def __init__(self):
        super().__init__('controller_priority_mux_node')

        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('controller_timeout_sec', 0.50)
        self.declare_parameter('safe_command_timeout_sec', 0.50)
        self.declare_parameter('takeover_linear_threshold', 0.02)
        self.declare_parameter('takeover_angular_threshold', 0.02)

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.controller_timeout_sec = float(self.get_parameter('controller_timeout_sec').value)
        self.safe_command_timeout_sec = float(self.get_parameter('safe_command_timeout_sec').value)
        self.takeover_linear_threshold = float(self.get_parameter('takeover_linear_threshold').value)
        self.takeover_angular_threshold = float(self.get_parameter('takeover_angular_threshold').value)

        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero.')

        self.create_subscription(Twist, '/cmd_vel_safety_filtered', self.safe_callback, 10)
        self.create_subscription(Twist, '/controller_cmd_vel', self.controller_callback, 10)
        self.create_subscription(String, '/drive_switch_state', self.switch_state_callback, 10)
        self.create_subscription(String, '/drive_mode', self.drive_mode_callback, 10)
        self.create_subscription(Bool, '/controller_enable', self.controller_enable_callback, 10)
        self.create_subscription(Bool, '/controller_connected', self.controller_connected_callback, 10)
        self.create_subscription(Bool, '/controller_release', self.controller_release_callback, 10)
        self.create_subscription(Bool, '/controller_stop', self.controller_stop_callback, 10)

        self.final_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.source_pub = self.create_publisher(String, '/command_source', 10)
        self.status_pub = self.create_publisher(String, '/controller_override_state', 10)
        self.manual_hold_pub = self.create_publisher(Bool, '/controller_manual_hold_request', 10)

        self.last_safe_cmd = Twist()
        self.last_safe_time: Time | None = None
        self.last_controller_cmd = Twist()
        self.last_controller_time: Time | None = None

        self.current_drive_state = 'stop'
        self.controller_enabled = False
        self.controller_connected = False
        self.controller_override_active = False
        self.manual_hold_pending = False
        self.disconnect_hold = False
        self.release_inhibit_until_neutral = False
        self.stop_latched = False

        self.last_source: str | None = None
        self.last_status: str | None = None

        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_final_command)

        self.get_logger().info('Controller priority mux started.')
        self.get_logger().warn('Controller override bypasses safety-distance/AEB filtering.')

    def safe_callback(self, msg: Twist) -> None:
        self.last_safe_cmd = msg
        self.last_safe_time = self.get_clock().now()

    def switch_state_callback(self, msg: String) -> None:
        self.current_drive_state = msg.data.strip().lower()
        if self.current_drive_state in self.STOP_STATES:
            self.controller_override_active = False
            self.manual_hold_pending = False
        elif self.current_drive_state == 'manual_hold':
            self.manual_hold_pending = False
        elif self.current_drive_state in self.AUTO_STATES:
            self.manual_hold_pending = False

    def drive_mode_callback(self, msg: String) -> None:
        mode = msg.data.strip().lower()
        if mode not in self.STOP_STATES:
            self.stop_latched = False

    def controller_enable_callback(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        if self.controller_enabled and not enabled:
            if self.controller_override_active and self.current_drive_state in self.MANUAL_STATES:
                self.request_manual_hold()
            self.controller_override_active = False
            self.disconnect_hold = False
            self.release_inhibit_until_neutral = False
        self.controller_enabled = enabled

    def controller_connected_callback(self, msg: Bool) -> None:
        connected = bool(msg.data)
        if self.controller_connected and not connected and self.controller_override_active:
            self.controller_override_active = False
            self.disconnect_hold = True
            if self.current_drive_state in self.MANUAL_STATES:
                self.request_manual_hold()
            self.get_logger().warn('Controller disconnected during override; vehicle held stopped.')
        self.controller_connected = connected

    def command_requests_takeover(self, msg: Twist) -> bool:
        return (
            abs(msg.linear.x) > self.takeover_linear_threshold
            or abs(msg.angular.z) > self.takeover_angular_threshold
        )

    def request_manual_hold(self) -> None:
        if self.manual_hold_pending:
            return
        pulse = Bool()
        pulse.data = True
        self.manual_hold_pub.publish(pulse)
        self.manual_hold_pending = True
        self.get_logger().info('Requested Manual Hold to cancel the old terminal motion.')

    def controller_callback(self, msg: Twist) -> None:
        self.last_controller_cmd = msg
        self.last_controller_time = self.get_clock().now()
        moving = self.command_requests_takeover(msg)

        if self.disconnect_hold:
            if self.controller_connected and not moving:
                self.disconnect_hold = False
                self.get_logger().info('Controller reconnected at neutral; disconnect hold cleared.')
            return

        if self.release_inhibit_until_neutral:
            if not moving:
                self.release_inhibit_until_neutral = False
            return

        if (
            not self.controller_enabled
            or not self.controller_connected
            or self.stop_latched
            or self.current_drive_state in self.STOP_STATES
        ):
            return

        if moving:
            if not self.controller_override_active:
                if self.current_drive_state in self.MANUAL_STATES:
                    self.request_manual_hold()
                self.get_logger().info(
                    f'Controller override started over {self.current_drive_state}.'
                )
            self.controller_override_active = True
        else:
            if self.controller_override_active:
                self.get_logger().info('Controller stick returned to neutral; base mode resumes.')
            self.controller_override_active = False

    def controller_release_callback(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self.current_drive_state in self.MANUAL_STATES:
            self.request_manual_hold()
        self.controller_override_active = False
        self.release_inhibit_until_neutral = True
        self.get_logger().info('Controller override released; center the stick before reuse.')

    def controller_stop_callback(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.stop_latched = True
        self.controller_override_active = False
        self.disconnect_hold = False
        self.get_logger().warn('Global STOP latched by controller stop button.')

    def is_fresh(self, stamp: Time | None, timeout_sec: float) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age <= timeout_sec

    def publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        if status != self.last_status:
            self.get_logger().info(f'Controller state -> {status}')
            self.last_status = status

    def publish_with_source(self, cmd: Twist, source: str, status: str) -> None:
        self.final_pub.publish(cmd)

        source_msg = String()
        source_msg.data = source
        self.source_pub.publish(source_msg)
        self.publish_status(status)

        if source != self.last_source:
            self.get_logger().info(f'Final command source -> {source}')
            self.last_source = source

    def publish_final_command(self) -> None:
        zero = Twist()

        if self.stop_latched or self.current_drive_state in self.STOP_STATES:
            self.publish_with_source(zero, 'stop', 'stop_mode')
            return

        if self.disconnect_hold:
            self.publish_with_source(zero, 'controller_disconnect_hold', 'disconnect_hold')
            return

        if self.controller_enabled and self.controller_override_active:
            if (
                self.controller_connected
                and self.is_fresh(self.last_controller_time, self.controller_timeout_sec)
            ):
                mode_label = 'auto' if self.current_drive_state in self.AUTO_STATES else 'manual'
                self.publish_with_source(
                    self.last_controller_cmd,
                    'controller_manual_override',
                    f'active_override_{mode_label}',
                )
                return

            self.controller_override_active = False
            self.disconnect_hold = True
            self.publish_with_source(zero, 'controller_timeout_hold', 'disconnect_hold')
            return

        if self.manual_hold_pending and self.current_drive_state != 'manual_hold':
            self.publish_with_source(zero, 'waiting_manual_hold', 'waiting_manual_hold')
            return

        if self.is_fresh(self.last_safe_time, self.safe_command_timeout_sec):
            status = 'enabled_idle' if self.controller_enabled else 'disabled'
            if self.controller_enabled and not self.controller_connected:
                status = 'enabled_disconnected'
            self.publish_with_source(
                self.last_safe_cmd,
                'safety_filtered_terminal_or_auto',
                status,
            )
            return

        status = 'enabled_idle' if self.controller_enabled else 'disabled'
        self.publish_with_source(zero, 'no_fresh_command', status)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerPriorityMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Controller priority mux interrupted.')
    finally:
        if rclpy.ok():
            node.final_pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

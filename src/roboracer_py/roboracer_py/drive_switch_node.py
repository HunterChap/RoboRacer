import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class DriveSwitchNode(Node):
    """
    Final /cmd_vel safety gate.

    Inputs:
      /auto_cmd_vel          geometry_msgs/Twist
      /transmitter_cmd_vel   geometry_msgs/Twist
      /drive_mode            std_msgs/String

    Outputs:
      /cmd_vel               geometry_msgs/Twist
      /drive_switch_state    std_msgs/String

    Supported drive_mode strings:
      s, stop, emergency_stop, disarm, 5
      0, soft_stop, hold_current
          stop inside the current selected mode:
          auto_* -> auto_hold, manual_* -> manual_hold, stop -> stop
      a, auto
          toggle: auto_hold -> auto_active, otherwise auto_hold
      auto_hold
      auto_active, auto_start, start_auto
      auto_now, auto_active_now, rescue_auto
          enter auto_active immediately and skip transition stop
      manual, m, manual_hold
      manual_active

    Note:
      a1/a2/a3/a4 are terminal commands, not drive_switch states.
      terminal_command_node translates them into auto_active + /manual_command.
    """

    STOP = 'stop'
    AUTO_HOLD = 'auto_hold'
    AUTO_ACTIVE = 'auto_active'
    MANUAL_HOLD = 'manual_hold'
    MANUAL_ACTIVE = 'manual_active'

    VALID_STATES = {
        STOP,
        AUTO_HOLD,
        AUTO_ACTIVE,
        MANUAL_HOLD,
        MANUAL_ACTIVE,
    }

    def __init__(self):
        super().__init__('drive_switch_node')

        self.declare_parameter('default_mode', 'stop')
        self.declare_parameter('start_armed', False)
        self.declare_parameter('transition_stop_sec', 0.1)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('mode_timeout', 0.5)  # kept for launch compatibility
        self.declare_parameter('publish_period', 0.05)

        self.default_mode = str(self.get_parameter('default_mode').value).strip().lower()
        self.start_armed = bool(self.get_parameter('start_armed').value)
        self.transition_stop_sec = float(self.get_parameter('transition_stop_sec').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.mode_timeout = float(self.get_parameter('mode_timeout').value)
        self.publish_period = float(self.get_parameter('publish_period').value)

        self.current_state = self.resolve_start_state(self.default_mode, self.start_armed)
        self.transition_stop_until = self.get_clock().now()

        self.last_auto_cmd = None
        self.last_manual_cmd = None
        self.last_auto_time = None
        self.last_manual_time = None
        self.last_mode_time = self.get_clock().now()

        self.auto_sub = self.create_subscription(
            Twist,
            '/auto_cmd_vel',
            self.auto_callback,
            10
        )

        self.manual_sub = self.create_subscription(
            Twist,
            '/transmitter_cmd_vel',
            self.manual_callback,
            10
        )

        self.mode_sub = self.create_subscription(
            String,
            '/drive_mode',
            self.mode_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.state_pub = self.create_publisher(
            String,
            '/drive_switch_state',
            10
        )

        self.timer = self.create_timer(
            self.publish_period,
            self.switch_loop
        )

        self.get_logger().info('Drive switch node started.')
        self.get_logger().info('Subscribing: /auto_cmd_vel, /transmitter_cmd_vel, /drive_mode')
        self.get_logger().info('Publishing final command: /cmd_vel')
        self.get_logger().info('Publishing state: /drive_switch_state')
        self.get_logger().info(f'Initial state: {self.current_state}')

    def resolve_start_state(self, default_mode, start_armed):
        mode = self.normalize_mode(default_mode)

        # Start in Stop Mode unless the launch configuration explicitly arms the system.
        if not start_armed:
            return self.STOP

        if mode in [self.AUTO_ACTIVE, self.MANUAL_ACTIVE]:
            return mode

        if mode in [self.AUTO_HOLD, self.MANUAL_HOLD, self.STOP]:
            return mode

        if mode == 'auto':
            return self.AUTO_ACTIVE

        if mode == 'manual':
            return self.MANUAL_HOLD

        return self.STOP

    def normalize_mode(self, raw_mode):
        mode = str(raw_mode).strip().lower()

        aliases = {
            's': self.STOP,
            '5': self.STOP,
            'stop': self.STOP,
            'e_stop': self.STOP,
            'estop': self.STOP,
            'emergency_stop': self.STOP,
            'disarm': self.STOP,

            '0': 'hold_current',
            'soft_stop': 'hold_current',
            'hold': 'hold_current',
            'hold_current': 'hold_current',
            'current_hold': 'hold_current',

            'auto_hold': self.AUTO_HOLD,
            'ah': self.AUTO_HOLD,

            'auto_active': self.AUTO_ACTIVE,
            'auto_start': self.AUTO_ACTIVE,
            'start_auto': self.AUTO_ACTIVE,

            'auto_now': 'auto_now',
            'auto_active_now': 'auto_now',
            'rescue_auto': 'auto_now',
            'auto_rescue': 'auto_now',
            'aa': 'auto_now',

            'manual': self.MANUAL_HOLD,
            'm': self.MANUAL_HOLD,
            'manual_hold': self.MANUAL_HOLD,
            'mh': self.MANUAL_HOLD,

            'manual_active': self.MANUAL_ACTIVE,
            'ma': self.MANUAL_ACTIVE,
        }

        if mode in aliases:
            return aliases[mode]

        # The a/auto command uses a two-step Auto enable sequence.
        if mode == 'a' or mode == 'auto':
            return 'auto'

        return mode

    def auto_callback(self, msg):
        self.last_auto_cmd = msg
        self.last_auto_time = self.get_clock().now()

    def manual_callback(self, msg):
        self.last_manual_cmd = msg
        self.last_manual_time = self.get_clock().now()

    def mode_callback(self, msg):
        requested = self.normalize_mode(msg.data)

        if requested == 'auto':
            # Two-step Auto start:
            # First command selects Auto Hold; the second enables Auto Active.
            if self.current_state == self.AUTO_HOLD:
                new_state = self.AUTO_ACTIVE
            else:
                new_state = self.AUTO_HOLD
        elif requested == 'auto_now':
            # Direct Auto Active command.
            # Skip the transition delay so autonomous control can take over immediately.
            self.set_state(self.AUTO_ACTIVE, skip_transition_stop=True)
            return

        elif requested == 'hold_current':
            # Soft stop keeps the selected mode while commanding zero motion.
            # Stop Mode additionally disarms the output.
            if self.current_state in [self.AUTO_HOLD, self.AUTO_ACTIVE]:
                new_state = self.AUTO_HOLD
            elif self.current_state in [self.MANUAL_HOLD, self.MANUAL_ACTIVE]:
                new_state = self.MANUAL_HOLD
            else:
                new_state = self.STOP
        elif requested in self.VALID_STATES:
            new_state = requested
        else:
            self.get_logger().warn(f'Unknown drive mode: {msg.data}')
            return

        self.set_state(new_state)

    def set_state(self, new_state, skip_transition_stop=False):
        if new_state not in self.VALID_STATES:
            self.get_logger().warn(f'Refusing invalid state: {new_state}')
            return

        now = self.get_clock().now()
        self.last_mode_time = now

        if new_state != self.current_state:
            old_state = self.current_state
            self.current_state = new_state
            if skip_transition_stop:
                self.transition_stop_until = now
                self.get_logger().info(
                    f'Drive state changed: {old_state} -> {new_state}. '
                    'No transition stop requested.'
                )
            else:
                self.transition_stop_until = now + Duration(
                    seconds=max(0.0, self.transition_stop_sec)
                )
                self.get_logger().info(
                    f'Drive state changed: {old_state} -> {new_state}. '
                    f'Holding stop for {self.transition_stop_sec:.2f}s.'
                )

    def is_fresh(self, last_time, timeout):
        if last_time is None:
            return False

        age = (self.get_clock().now() - last_time).nanoseconds / 1e9
        return age <= timeout

    def in_transition_stop(self):
        return self.get_clock().now() < self.transition_stop_until

    def make_stop_cmd(self):
        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        return cmd

    def publish_state(self):
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)

    def choose_output_cmd(self):
        if self.in_transition_stop():
            return self.make_stop_cmd(), 'transition_stop'

        if self.current_state == self.STOP:
            return self.make_stop_cmd(), self.STOP

        if self.current_state == self.AUTO_HOLD:
            return self.make_stop_cmd(), self.AUTO_HOLD

        if self.current_state == self.MANUAL_HOLD:
            return self.make_stop_cmd(), self.MANUAL_HOLD

        if self.current_state == self.AUTO_ACTIVE:
            if self.is_fresh(self.last_auto_time, self.cmd_timeout):
                return self.last_auto_cmd, self.AUTO_ACTIVE

            self.get_logger().warn(
                'AUTO_ACTIVE but /auto_cmd_vel timed out. Publishing stop.',
                throttle_duration_sec=1.0
            )
            return self.make_stop_cmd(), 'auto_timeout_stop'

        if self.current_state == self.MANUAL_ACTIVE:
            if self.is_fresh(self.last_manual_time, self.cmd_timeout):
                return self.last_manual_cmd, self.MANUAL_ACTIVE

            self.get_logger().warn(
                'MANUAL_ACTIVE but /transmitter_cmd_vel timed out. Publishing stop.',
                throttle_duration_sec=1.0
            )
            return self.make_stop_cmd(), 'manual_timeout_stop'

        return self.make_stop_cmd(), 'unknown_state_stop'

    def switch_loop(self):
        output_cmd, active_reason = self.choose_output_cmd()
        self.cmd_pub.publish(output_cmd)
        self.publish_state()

        self.get_logger().info(
            f'state={self.current_state}, reason={active_reason}, '
            f'linear.x={output_cmd.linear.x:.2f}, angular.z={output_cmd.angular.z:.2f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = DriveSwitchNode()

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

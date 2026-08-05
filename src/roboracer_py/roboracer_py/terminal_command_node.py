import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String

try:
    from .terminal_command_parser import (
        CommandKind,
        parse_terminal_command,
    )
    from .auto_intervention_commands import (
        AutoInterventionCommandPublisher,
    )
    from .manual_motion_commands import ManualMotionController
except ImportError:
    from terminal_command_parser import (
        CommandKind,
        parse_terminal_command,
    )
    from auto_intervention_commands import (
        AutoInterventionCommandPublisher,
    )
    from manual_motion_commands import ManualMotionController


class TerminalCommandNode(Node):
    def __init__(self):
        super().__init__('terminal_command_node')

        self.declare_parameter('publish_period', 0.05)
        self.declare_parameter('manual_speed_mps', 0.50)
        self.declare_parameter('manual_reverse_speed_mps', -0.20)
        self.declare_parameter('manual_turn_speed_mps', 0.30)
        self.declare_parameter('manual_turn_angular_z', 1.0)
        self.declare_parameter('preset_move_duration_sec', 1.0)
        self.declare_parameter('preset_back_duration_sec', 1.0)
        self.declare_parameter('preset_turn_duration_sec', 2.4)
        self.declare_parameter('circle_speed_mps', 0.25)
        self.declare_parameter('circle_angular_z', 0.8)
        self.declare_parameter('max_abs_speed_mps', 2.0)
        self.declare_parameter('max_abs_angular_z', 2.0)
        self.declare_parameter('wheelbase_m', 0.33)
        self.declare_parameter('max_steering_angle_rad', 0.65)
        self.declare_parameter('turn_speed_follows_forward', False)

        # Scripted avoidance-maneuver settings. Both directions share
        # one forward-distance value, while the turn stage is time calibrated.
        self.declare_parameter('avoid_forward_distance_m', 1.0)
        self.declare_parameter('avoid_turn_duration_sec', 2.4)

        # Controller nodes are launched with the control stack. This
        # parameter determines whether temporary override is permitted.
        self.declare_parameter('controller_enabled_on_start', True)

        # Declared for launch compatibility. control_node handles automatic
        # turn completion because it has distance and safety context.
        self.declare_parameter('auto_turn_target_deg', 90.0)

        # Distance-feedback source selection:
        #   odom      -> accumulated odometry path
        #   wheel_rpm -> distance integrated from measured wheel RPM
        #   auto      -> prefer odometry, then fall back to wheel RPM
        self.declare_parameter('distance_source', 'odom')
        self.declare_parameter(
            'distance_odom_topic',
            '/ego_racecar/odom',
        )
        self.declare_parameter(
            'distance_wheel_rpm_topic',
            '/vehicle_feedback/wheel_rpm',
        )
        self.declare_parameter('wheel_diameter_m', 0.103)
        self.declare_parameter('distance_feedback_timeout_sec', 0.75)

        self.publish_period = float(
            self.get_parameter('publish_period').value
        )
        self.auto_turn_target_rad = math.radians(
            abs(
                float(
                    self.get_parameter(
                        'auto_turn_target_deg'
                    ).value
                )
            )
        )

        self.distance_source = str(
            self.get_parameter('distance_source').value
        ).strip().lower()
        self.distance_feedback_timeout_sec = max(
            0.05,
            float(
                self.get_parameter(
                    'distance_feedback_timeout_sec'
                ).value
            ),
        )
        self.wheel_diameter_m = max(
            1e-6,
            abs(
                float(
                    self.get_parameter(
                        'wheel_diameter_m'
                    ).value
                )
            ),
        )
        self.wheel_circumference_m = (
            math.pi * self.wheel_diameter_m
        )

        self.manual_command_pub = self.create_publisher(
            String,
            '/manual_command',
            10,
        )
        self.drive_mode_pub = self.create_publisher(
            String,
            '/drive_mode',
            10,
        )
        self.transmitter_cmd_pub = self.create_publisher(
            Twist,
            '/transmitter_cmd_vel',
            10,
        )
        self.controller_enable_pub = self.create_publisher(
            Bool,
            '/controller_enable',
            10,
        )
        self.terminal_stop_pub = self.create_publisher(
            Bool,
            '/terminal_stop',
            10,
        )

        retained_qos = QoSProfile(depth=1)
        retained_qos.reliability = ReliabilityPolicy.RELIABLE
        retained_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.shared_forward_speed_pub = self.create_publisher(
            Float32,
            '/terminal_settings/forward_speed_mps',
            retained_qos,
        )
        self.shared_reverse_speed_pub = self.create_publisher(
            Float32,
            '/terminal_settings/reverse_speed_mps',
            retained_qos,
        )
        self.shared_turn_speed_pub = self.create_publisher(
            Float32,
            '/terminal_settings/turn_speed_mps',
            retained_qos,
        )
        self.shared_steering_angle_pub = self.create_publisher(
            Float32,
            '/terminal_settings/steering_angle_rad',
            retained_qos,
        )
        self.shared_turn_follow_pub = self.create_publisher(
            Bool,
            '/terminal_settings/turn_speed_follows_forward',
            retained_qos,
        )

        self.switch_state_sub = self.create_subscription(
            String,
            '/drive_switch_state',
            self.switch_state_callback,
            10,
        )
        self.controller_status_sub = self.create_subscription(
            String,
            '/controller_override_state',
            self.controller_status_callback,
            10,
        )
        self.controller_manual_hold_sub = self.create_subscription(
            Bool,
            '/controller_manual_hold_request',
            self.controller_manual_hold_callback,
            10,
        )
        self.controller_stop_sub = self.create_subscription(
            Bool,
            '/controller_stop',
            self.controller_stop_callback,
            10,
        )

        odom_topic = str(
            self.get_parameter('distance_odom_topic').value
        )
        wheel_rpm_topic = str(
            self.get_parameter(
                'distance_wheel_rpm_topic'
            ).value
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )
        self.wheel_rpm_sub = self.create_subscription(
            Float32,
            wheel_rpm_topic,
            self.wheel_rpm_callback,
            10,
        )

        self.current_switch_state = 'stop'
        self.selected_mode = 'stop'
        self.controller_enabled = bool(
            self.get_parameter(
                'controller_enabled_on_start'
            ).value
        )
        self.controller_status = 'disabled'
        self.running = True

        self.odom_path_total_m = 0.0
        self.last_odom_xy = None
        self.last_odom_monotonic = None

        self.yaw_unwrapped_rad = None
        self.last_raw_yaw_rad = None

        self.wheel_path_total_m = 0.0
        self.last_wheel_rpm = None
        self.last_wheel_rpm_monotonic = None

        self.auto_intervention = AutoInterventionCommandPublisher(
            self,
            self.manual_command_pub,
            self.drive_mode_pub,
        )

        self.manual_motion = ManualMotionController(
            self,
            self.drive_mode_pub,
            self.transmitter_cmd_pub,
            manual_speed_mps=float(
                self.get_parameter('manual_speed_mps').value
            ),
            manual_reverse_speed_mps=float(
                self.get_parameter(
                    'manual_reverse_speed_mps'
                ).value
            ),
            manual_turn_speed_mps=float(
                self.get_parameter(
                    'manual_turn_speed_mps'
                ).value
            ),
            manual_turn_angular_z=float(
                self.get_parameter(
                    'manual_turn_angular_z'
                ).value
            ),
            preset_move_duration_sec=float(
                self.get_parameter(
                    'preset_move_duration_sec'
                ).value
            ),
            preset_back_duration_sec=float(
                self.get_parameter(
                    'preset_back_duration_sec'
                ).value
            ),
            preset_turn_duration_sec=float(
                self.get_parameter(
                    'preset_turn_duration_sec'
                ).value
            ),
            circle_speed_mps=float(
                self.get_parameter(
                    'circle_speed_mps'
                ).value
            ),
            circle_angular_z=float(
                self.get_parameter(
                    'circle_angular_z'
                ).value
            ),
            max_abs_speed_mps=float(
                self.get_parameter(
                    'max_abs_speed_mps'
                ).value
            ),
            max_abs_angular_z=float(
                self.get_parameter(
                    'max_abs_angular_z'
                ).value
            ),
            wheelbase_m=float(
                self.get_parameter('wheelbase_m').value
            ),
            max_steering_angle_rad=float(
                self.get_parameter(
                    'max_steering_angle_rad'
                ).value
            ),
            # Start with an independent turn-speed setting. The runtime
            # `tsauto` command is the only way to enable speed following.
            turn_speed_follows_forward=False,
            avoid_forward_distance_m=float(
                self.get_parameter(
                    'avoid_forward_distance_m'
                ).value
            ),
            avoid_turn_duration_sec=float(
                self.get_parameter(
                    'avoid_turn_duration_sec'
                ).value
            ),
        )

        self.timer = self.create_timer(
            self.publish_period,
            self.update,
        )

        self.command_thread = threading.Thread(
            target=self.command_loop,
        )
        self.command_thread.daemon = True
        self.command_thread.start()

        # Startup enters Stop Mode without latching an operator emergency
        # stop. The operator must select Manual or Auto before motion.
        self.publish_terminal_stop(False)
        self.publish_controller_enable()
        self.publish_shared_settings()

        self.get_logger().info(
            'Terminal command node started in Stop Mode. '
            'Choose m or a. Controller permission is '
            f'{"enabled" if self.controller_enabled else "disabled"}.'
        )
        self.print_help()

    def switch_state_callback(self, msg):
        self.current_switch_state = msg.data.strip().lower()

    def controller_status_callback(self, msg):
        self.controller_status = msg.data.strip().lower()

    def quaternion_to_yaw(self, orientation) -> float:
        x = float(orientation.x)
        y = float(orientation.y)
        z = float(orientation.z)
        w = float(orientation.w)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        now = time.monotonic()
        position = msg.pose.pose.position
        current_xy = (
            float(position.x),
            float(position.y),
        )

        if self.last_odom_xy is not None:
            dx = current_xy[0] - self.last_odom_xy[0]
            dy = current_xy[1] - self.last_odom_xy[1]
            step = math.hypot(dx, dy)
            if math.isfinite(step) and step < 5.0:
                self.odom_path_total_m += step

        self.last_odom_xy = current_xy
        self.last_odom_monotonic = now

        raw_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )
        if self.last_raw_yaw_rad is None:
            self.yaw_unwrapped_rad = raw_yaw
        else:
            yaw_delta = math.atan2(
                math.sin(raw_yaw - self.last_raw_yaw_rad),
                math.cos(raw_yaw - self.last_raw_yaw_rad),
            )
            self.yaw_unwrapped_rad += yaw_delta

        self.last_raw_yaw_rad = raw_yaw

    def wheel_rpm_callback(self, msg):
        now = time.monotonic()
        rpm = float(msg.data)

        if (
            self.last_wheel_rpm_monotonic is not None
            and self.last_wheel_rpm is not None
        ):
            dt = now - self.last_wheel_rpm_monotonic
            if 0.0 < dt < 1.0:
                average_abs_rpm = (
                    abs(rpm) + abs(self.last_wheel_rpm)
                ) / 2.0
                revolutions = average_abs_rpm * dt / 60.0
                self.wheel_path_total_m += (
                    revolutions
                    * self.wheel_circumference_m
                )

        self.last_wheel_rpm = rpm
        self.last_wheel_rpm_monotonic = now

    def feedback_is_fresh(self, timestamp) -> bool:
        if timestamp is None:
            return False
        return (
            time.monotonic() - float(timestamp)
            <= self.distance_feedback_timeout_sec
        )

    def get_distance_total_m(self):
        odom_ready = self.feedback_is_fresh(
            self.last_odom_monotonic
        )
        wheel_ready = self.feedback_is_fresh(
            self.last_wheel_rpm_monotonic
        )

        if self.distance_source == 'odom':
            return self.odom_path_total_m if odom_ready else None

        if self.distance_source == 'wheel_rpm':
            return self.wheel_path_total_m if wheel_ready else None

        if self.distance_source == 'auto':
            if odom_ready:
                return self.odom_path_total_m
            if wheel_ready:
                return self.wheel_path_total_m
            return None

        self.get_logger().warn(
            f'Unknown distance_source="{self.distance_source}". '
            'Use odom, wheel_rpm, or auto.'
        )
        return None

    def get_yaw_unwrapped_rad(self):
        if not self.feedback_is_fresh(
            self.last_odom_monotonic
        ):
            return None
        return self.yaw_unwrapped_rad

    def publish_controller_enable(self):
        msg = Bool()
        msg.data = self.controller_enabled
        self.controller_enable_pub.publish(msg)

    def publish_terminal_stop(self, active: bool):
        msg = Bool()
        msg.data = bool(active)
        self.terminal_stop_pub.publish(msg)

    def release_terminal_stop(self):
        self.publish_terminal_stop(False)

    def publish_float(self, publisher, value: float):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    def publish_shared_settings(self):
        self.publish_float(
            self.shared_forward_speed_pub,
            self.manual_motion.forward_speed_mps,
        )
        self.publish_float(
            self.shared_reverse_speed_pub,
            self.manual_motion.reverse_speed_mps,
        )
        self.publish_float(
            self.shared_turn_speed_pub,
            self.manual_motion.get_effective_turn_speed(),
        )
        self.publish_float(
            self.shared_steering_angle_pub,
            self.manual_motion.steering_angle_rad,
        )

        follow_msg = Bool()
        follow_msg.data = (
            self.manual_motion.turn_speed_follows_forward
        )
        self.shared_turn_follow_pub.publish(follow_msg)

    def controller_manual_hold_callback(self, msg):
        if not msg.data:
            return

        if self.selected_mode != 'manual':
            return

        self.manual_motion.set_manual_hold()
        self.current_switch_state = 'manual_hold'
        print(
            'Controller override cancelled the previous '
            'Terminal Manual action.'
        )
        self.print_status()

    def controller_stop_callback(self, msg):
        if not msg.data:
            return

        self.selected_mode = 'stop'
        self.publish_terminal_stop(True)
        self.manual_motion.emergency_stop()
        self.current_switch_state = (
            self.auto_intervention.handle_hard_stop()
        )
        print('Controller Stop button entered Stop Mode.')
        self.print_status()

    def is_manual_state(self):
        return self.selected_mode == 'manual'

    def is_auto_state(self):
        return self.selected_mode == 'auto'

    def state_label(self):
        if self.selected_mode == 'stop':
            return 'Stop Mode'

        if self.selected_mode == 'manual':
            if self.current_switch_state == 'manual_active':
                return (
                    'Manual Mode / '
                    f'{self.manual_motion.get_status_label()}'
                )
            return 'Manual Mode / STOPPED'

        if self.selected_mode == 'auto':
            if self.manual_motion.is_temporary_auto_motion():
                return (
                    'Auto Mode / INTERVENTION / '
                    f'{self.manual_motion.get_status_label()}'
                )
            if self.current_switch_state == 'auto_hold':
                return 'Auto Mode / HOLD'
            if self.current_switch_state == 'auto_active':
                return 'Auto Mode / ACTIVE'
            return 'Auto Mode / TRANSITION'

        return f'Unknown Mode: {self.selected_mode}'

    def controller_label(self):
        if not self.controller_enabled:
            return 'PERMISSION DISABLED'

        labels = {
            'disabled': 'PERMISSION ENABLED / SYNCING',
            'enabled_idle': 'PERMISSION ENABLED / IDLE',
            'enabled_disconnected':
                'PERMISSION ENABLED / CONTROLLER DISCONNECTED',
            'active_override_auto':
                'PERMISSION ENABLED / ACTIVE AUTO OVERRIDE',
            'active_override_manual':
                'PERMISSION ENABLED / ACTIVE MANUAL OVERRIDE',
            'waiting_manual_hold':
                'PERMISSION ENABLED / RETURNING TO MANUAL HOLD',
            'disconnect_hold':
                'PERMISSION ENABLED / CONNECTION LOST / STOPPED',
            'stop_mode':
                'PERMISSION ENABLED / BLOCKED BY STOP MODE',

        }
        return labels.get(
            self.controller_status,
            'PERMISSION ENABLED / '
            + self.controller_status.upper().replace('_', ' '),
        )

    def distance_feedback_label(self):
        ready = self.get_distance_total_m() is not None
        state = 'READY' if ready else 'NOT READY'
        return (
            f'{self.distance_source.upper()} / {state} / '
            f'wheel diameter={self.wheel_diameter_m:.3f}m'
        )

    def print_status(self):
        print(f'Current mode: {self.state_label()}')
        print(f'Controller: {self.controller_label()}')

    def print_settings(self):
        print('Persistent shared settings:')
        print(f'  {self.manual_motion.get_settings_text()}')
        print(
            'Distance feedback: '
            f'{self.distance_feedback_label()}'
        )
        print(
            'Avoidance demo: '
            f'{self.manual_motion.get_avoidance_settings_text()}'
        )

    def print_help(self):
        def command_row(
            label: str,
            command: str,
            description: str = '',
        ):
            command_block = f'[{command}]'
            if description:
                print(
                    f'  {label:<18}'
                    f'{command_block:<12}'
                    f': {description}'
                )
            else:
                print(
                    f'  {label:<18}'
                    f'{command_block}'
                )

        print('')
        print('Terminal command node')
        print('=====================')
        print('')

        print('Mode')
        command_row(
            'Manual',
            'm',
            'switch to Manual Mode, stopped',
        )
        command_row(
            'Auto',
            'a',
            'switch to Auto Hold; press again to run',
        )
        command_row(
            'Auto now',
            'aa',
            'switch directly to Auto Active',
        )
        command_row(
            'Hold',
            '0',
            'stop now, keep selected mode',
        )
        command_row(
            'Stop',
            's',
            'Stop Mode / disarm',
        )
        print('')

        print('Input authority')
        command_row(
            'Controller',
            'j',
            'enable / disable temporary override',
        )
        print(
            f'  {"Startup":<18}'
            f'{"":<12}'
            f': controller permission is enabled'
        )
        print('')

        print('Number actions')
        command_row('Move', '1', 'forward command')
        command_row(
            'Left',
            '2',
            'manual hold; auto forced correction, then resumes',
        )
        command_row(
            'Right',
            '3',
            'manual hold; auto forced correction, then resumes',
        )
        command_row('Back', '4', 'reverse command')
        print(
            f'  {"Rule":<18}'
            f'{"":<12}'
            f': auto 2/3 force a turn, then hand back when committed'
        )
        print('')

        print('Shortcuts')
        command_row(
            'Manual action',
            'm#',
            'switch to manual and hold #',
        )
        command_row(
            'Auto action',
            'a#',
            'switch, run temporary #, then auto',
        )
        print(
            f'  {"# values":<18}'
            f'{"":<12}'
            f': 1, 2, 3, or 4'
        )
        print('')

        print('Speed / steering / trajectory')
        command_row(
            'Forward speed',
            'v#',
            'save shared forward speed; keep mode',
        )
        command_row(
            'Reverse speed',
            'v-#',
            'save shared reverse speed; keep mode',
        )
        command_row(
            'Turn speed',
            'ts#',
            'save shared turn speed; keep mode',
        )
        command_row(
            'Turn follow',
            'tsauto',
            'turn speed follows latest positive v#',
        )
        command_row(
            'Steering rad',
            'sa#',
            'save steering angle in rad',
        )
        command_row(
            'Steering deg',
            'sad#',
            'save steering angle in deg',
        )
        command_row(
            'Distance',
            '#m / #ft',
            'measured distance; negative = reverse',
        )
        command_row(
            'Settings',
            'p',
            'show saved settings and feedback',
        )
        command_row(
            'Cancel',
            'c',
            'stop action/profile; keep settings',
        )
        command_row('Left circle', 'lc')
        command_row('Right circle', 'rc')
        command_row('Figure-eight', 'f8')
        print('')

        print('Avoidance demo')
        command_row(
            'Avoid left',
            'l',
            'forward, timed left turn, then keep moving',
        )
        command_row(
            'Avoid right',
            'r',
            'forward, timed right turn, then keep moving',
        )
        command_row(
            'Avoid distance',
            'avd#',
            'shared forward distance in m, e.g. avd1.0',
        )
        command_row(
            'Avoid turn time',
            'avt#',
            'turn duration in s, e.g. avt2.4',
        )
        print('')

        print('Utility')
        command_row('Help', 'h')
        command_row(
            'Quit',
            'q',
            'publish stop and quit',
        )
        print('')

        self.print_status()
        print('')
        print('Choose mode: [m] Manual or [a] Auto.')
        print('')

    def update(self):
        self.manual_motion.update()
        self.publish_controller_enable()

    def command_loop(self):
        while self.running:
            try:
                raw = input(f'[{self.state_label()}] > ')
                self.handle_raw_command(raw)
            except KeyboardInterrupt:
                self.handle_quit()
                break

    def handle_controller_toggle(self):
        self.controller_enabled = not self.controller_enabled
        self.publish_controller_enable()
        state = (
            'ENABLED'
            if self.controller_enabled
            else 'DISABLED'
        )
        print(f'Controller permission changed: {state}')
        self.print_status()

    def handle_soft_stop(self):
        if self.is_manual_state():
            self.manual_motion.set_manual_hold()
            self.current_switch_state = 'manual_hold'
            self.print_status()
            return

        if self.is_auto_state():
            self.manual_motion.clear_active_motion()
            self.current_switch_state = (
                self.auto_intervention.handle_auto_hold()
            )
            self.print_status()
            return

        self.manual_motion.clear_active_motion()
        self.current_switch_state = 'stop'
        print(
            'Hold requested in Stop Mode. '
            'Choose m or a first.'
        )
        self.print_status()

    def start_auto_temporary_action(
        self,
        action: str,
        *,
        shortcut: bool = False,
    ):
        """Route Auto interventions through control_node rather than Manual Mode."""
        self.release_terminal_stop()
        self.selected_mode = 'auto'
        self.manual_motion.clear_active_motion()

        if shortcut:
            new_state = self.auto_intervention.handle_auto_preset(action)
        else:
            new_state = self.auto_intervention.handle_intervention(
                action,
                self.current_switch_state,
            )

        if new_state is not None:
            self.current_switch_state = new_state
        self.print_status()

    def handle_number_action(self, action: str):
        if self.is_manual_state():
            self.release_terminal_stop()
            self.manual_motion.start_manual_action(action)
            self.current_switch_state = 'manual_active'
            self.print_status()
            return

        if self.is_auto_state():
            self.start_auto_temporary_action(action)
            return

        self.get_logger().warn(
            f'Number command "{action}" ignored in Stop Mode.'
        )
        print('Choose a mode first: m = manual, a = auto.')

    def handle_auto_toggle(self):
        self.release_terminal_stop()
        self.manual_motion.clear_active_motion()

        if self.selected_mode != 'auto':
            self.selected_mode = 'auto'
            self.current_switch_state = (
                self.auto_intervention.handle_auto_hold()
            )
            self.print_status()
            return

        if self.current_switch_state == 'auto_hold':
            self.current_switch_state = (
                self.auto_intervention.handle_auto_active()
            )
        else:
            self.current_switch_state = (
                self.auto_intervention.handle_auto_hold()
            )
        self.print_status()

    def handle_distance_command(self, distance_m: float):
        if self.selected_mode == 'stop':
            print('Choose a mode first: m = manual, a = auto.')
            return

        self.release_terminal_stop()
        completion_mode = (
            'manual_hold'
            if self.selected_mode == 'manual'
            else 'auto_hold'
        )

        if self.selected_mode == 'auto':
            self.auto_intervention.publish_manual_command('auto')

        started = self.manual_motion.start_distance(
            distance_m,
            distance_provider=self.get_distance_total_m,
            completion_mode=completion_mode,
        )
        if started:
            self.current_switch_state = 'manual_active'
        self.print_status()

    def handle_avoidance_command(self, direction: str):
        if self.selected_mode == 'stop':
            print('Choose a mode first: m = manual, a = auto.')
            return

        self.release_terminal_stop()
        completion_mode = (
            'manual_hold'
            if self.selected_mode == 'manual'
            else 'auto_active'
        )

        if self.selected_mode == 'auto':
            self.auto_intervention.publish_manual_command('auto')

        started = self.manual_motion.start_avoidance_demo(
            direction,
            completion_mode=completion_mode,
        )
        if started:
            self.current_switch_state = 'manual_active'
        self.print_status()

    def handle_trajectory_command(self, trajectory: str):
        if self.selected_mode == 'stop':
            print('Choose a mode first: m = manual, a = auto.')
            return

        self.release_terminal_stop()
        completion_mode = (
            'manual_hold'
            if self.selected_mode == 'manual'
            else 'auto_active'
        )

        if self.selected_mode == 'auto':
            self.auto_intervention.publish_manual_command('auto')

        started = self.manual_motion.start_trajectory(
            trajectory,
            completion_mode=completion_mode,
        )
        if started:
            self.current_switch_state = 'manual_active'
        self.print_status()

    def handle_raw_command(self, raw_command: str):
        parsed = parse_terminal_command(raw_command)

        if parsed.kind == CommandKind.UNKNOWN:
            if parsed.raw.strip():
                print(f'Unknown command: {parsed.raw}')
                self.print_help()
            return

        if parsed.kind == CommandKind.HELP:
            self.print_help()
            return

        if parsed.kind == CommandKind.SETTINGS:
            self.print_settings()
            return

        if parsed.kind == CommandKind.QUIT:
            self.handle_quit()
            return

        if parsed.kind == CommandKind.CONTROLLER_TOGGLE:
            self.handle_controller_toggle()
            return

        if parsed.kind == CommandKind.SOFT_STOP:
            self.handle_soft_stop()
            return

        if parsed.kind == CommandKind.STOP:
            self.selected_mode = 'stop'
            self.publish_terminal_stop(True)
            self.manual_motion.emergency_stop()
            self.current_switch_state = (
                self.auto_intervention.handle_hard_stop()
            )
            self.print_status()
            return

        if parsed.kind == CommandKind.AUTO_TOGGLE:
            self.handle_auto_toggle()
            return

        if parsed.kind == CommandKind.AUTO_HOLD:
            self.release_terminal_stop()
            self.selected_mode = 'auto'
            self.manual_motion.clear_active_motion()
            self.current_switch_state = (
                self.auto_intervention.handle_auto_hold()
            )
            self.print_status()
            return

        if parsed.kind == CommandKind.AUTO_ACTIVE:
            self.release_terminal_stop()
            self.selected_mode = 'auto'
            self.manual_motion.clear_active_motion()
            self.current_switch_state = (
                self.auto_intervention.handle_auto_active()
            )
            self.print_status()
            return

        if parsed.kind == CommandKind.AUTO_NOW:
            self.release_terminal_stop()
            self.selected_mode = 'auto'
            self.manual_motion.clear_active_motion()
            self.current_switch_state = (
                self.auto_intervention.handle_auto_now()
            )
            self.print_status()
            return

        if parsed.kind == CommandKind.AUTO_PRESET:
            self.start_auto_temporary_action(
                str(parsed.value),
                shortcut=True,
            )
            return

        if parsed.kind == CommandKind.NUMBER_ACTION:
            self.handle_number_action(str(parsed.value))
            return

        if parsed.kind == CommandKind.MANUAL_HOLD:
            self.release_terminal_stop()
            self.selected_mode = 'manual'
            self.manual_motion.set_manual_hold()
            self.current_switch_state = 'manual_hold'
            self.print_status()
            return

        if parsed.kind == CommandKind.MANUAL_PRESET:
            self.release_terminal_stop()
            self.selected_mode = 'manual'
            self.manual_motion.start_manual_action(
                str(parsed.value)
            )
            self.current_switch_state = 'manual_active'
            self.print_status()
            return

        if parsed.kind == CommandKind.SPEED:
            self.manual_motion.set_linear_speed(
                float(parsed.value)
            )
            self.publish_shared_settings()
            self.print_settings()
            self.print_status()
            return

        if parsed.kind == CommandKind.TURN_SPEED:
            self.manual_motion.set_turn_speed(
                float(parsed.value)
            )
            self.publish_shared_settings()
            self.print_settings()
            self.print_status()
            return

        if parsed.kind == CommandKind.TURN_SPEED_FOLLOW:
            self.manual_motion.set_turn_speed_follow()
            self.publish_shared_settings()
            self.print_settings()
            self.print_status()
            return

        if parsed.kind == CommandKind.STEERING_ANGLE:
            self.manual_motion.set_steering_angle(
                float(parsed.value)
            )
            self.publish_shared_settings()
            self.print_settings()
            self.print_status()
            return

        if parsed.kind == CommandKind.DISTANCE:
            self.handle_distance_command(
                float(parsed.value)
            )
            return

        if parsed.kind == CommandKind.AVOIDANCE_DISTANCE:
            self.manual_motion.set_avoidance_distance(
                float(parsed.value)
            )
            self.print_settings()
            self.print_status()
            return

        if parsed.kind == CommandKind.AVOIDANCE_TURN_TIME:
            self.manual_motion.set_avoidance_turn_duration(
                float(parsed.value)
            )
            self.print_settings()
            self.print_status()
            return

        if parsed.kind == CommandKind.AVOIDANCE:
            self.handle_avoidance_command(
                str(parsed.value)
            )
            return

        if parsed.kind == CommandKind.TRAJECTORY:
            self.handle_trajectory_command(
                str(parsed.value)
            )
            return

        if parsed.kind == CommandKind.CANCEL:
            if self.selected_mode == 'manual':
                self.manual_motion.cancel('manual_hold')
                self.current_switch_state = 'manual_hold'
            elif self.selected_mode == 'auto':
                self.auto_intervention.publish_manual_command(
                    'auto'
                )
                self.manual_motion.cancel('auto_active')
                self.current_switch_state = 'auto_active'
            else:
                self.manual_motion.clear_active_motion()
                print('No active motion to cancel in Stop Mode.')
            self.print_status()
            return

        self.get_logger().warn(
            f'Unhandled parsed command: {parsed}'
        )

    def handle_quit(self):
        self.get_logger().warn(
            'Quitting terminal command node. '
            'Publishing stop first.'
        )
        self.selected_mode = 'stop'
        self.publish_terminal_stop(True)
        self.manual_motion.emergency_stop()
        self.auto_intervention.handle_hard_stop()
        self.current_switch_state = 'stop'
        self.controller_enabled = False
        self.publish_controller_enable()
        self.running = False
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TerminalCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.handle_quit()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

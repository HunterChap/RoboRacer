from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Run the Pi side of Dual mode.

    The Raspberry Pi owns the complete command, safety, controller, and
    physical-vehicle pipeline. It publishes the final Twist command on
    /cmd_vel_safe. The PC-side companion launch receives that topic over ROS 2
    DDS and converts it to /drive for the F1TENTH simulator.

    terminal_command_node remains a separate interactive process and should be
    started by the Pi-side script or in another terminal/tmux window.
    """

    hardware_output_enable = LaunchConfiguration('hardware_output_enable')
    enable_auto_stack = LaunchConfiguration('enable_auto_stack')
    enable_controller = LaunchConfiguration('enable_controller')
    enable_lidar_validator = LaunchConfiguration('enable_lidar_validator')
    require_distance_data = LaunchConfiguration('require_distance_data')
    auto_scan_topic = LaunchConfiguration('auto_scan_topic')
    controller_max_speed_mps = LaunchConfiguration(
        'controller_max_speed_mps'
    )

    launch_arguments = [
        DeclareLaunchArgument(
            'hardware_output_enable',
            default_value='false',
            description=(
                'Enable physical output in vehicle_driver_node. Keep false '
                'until VESC and steering-servo communication is implemented '
                'and calibrated.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_auto_stack',
            default_value='true',
            description='Start perception_node and control_node.',
        ),
        DeclareLaunchArgument(
            'enable_controller',
            default_value='true',
            description=(
                'Start the ROS 2 controller driver and controller input node. '
                'The final priority mux always runs.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_lidar_validator',
            default_value='true',
            description='Start lidar_scan_validator_node.',
        ),
        DeclareLaunchArgument(
            'require_distance_data',
            default_value='false',
            description=(
                'Stop when front-distance data is missing or stale. Set true '
                'before autonomous physical driving.'
            ),
        ),
        DeclareLaunchArgument(
            'auto_scan_topic',
            default_value='/lidar/scan/points',
            description=(
                'LaserScan topic used by the autonomous stack. Change this '
                'to the topic published by the real LiDAR driver.'
            ),
        ),
        DeclareLaunchArgument(
            'controller_max_speed_mps',
            default_value='0.35',
            description='Maximum controller speed for initial Dual testing.',
        ),
    ]

    lidar_scan_validator_node = Node(
        package='roboracer_py',
        executable='lidar_scan_validator_node',
        name='lidar_scan_validator_node',
        output='screen',
        condition=IfCondition(enable_lidar_validator),
        parameters=[{
            'scan_topic': auto_scan_topic,
            'expected_frame_id': '',
            'min_valid_ratio': 0.20,
            'min_reasonable_range_m': 0.02,
            'warn_timeout_sec': 1.0,
            'publish_period': 0.5,
            'front_min_deg': -10.0,
            'front_max_deg': 10.0,
            'left_min_deg': 30.0,
            'left_max_deg': 90.0,
            'right_min_deg': -90.0,
            'right_max_deg': -30.0,
        }],
    )

    perception_node = Node(
        package='roboracer_py',
        executable='perception_node',
        name='perception_node',
        output='screen',
        condition=IfCondition(enable_auto_stack),
        remappings=[
            ('/scan', auto_scan_topic),
        ],
    )

    control_node = Node(
        package='roboracer_cpp',
        executable='control_node',
        name='control_node',
        output='screen',
        condition=IfCondition(enable_auto_stack),
        remappings=[
            ('/cmd_vel', '/auto_cmd_vel'),
        ],
        parameters=[{
            'safe_distance': 1.7,
            'forward_speed': 0.5,
            'turn_speed': 0.6,
            'turn_forward_speed': 0.2,
            'auto_min_steering_scale': 0.30,
            'auto_full_steering_distance_m': 0.90,
            'auto_steering_curve_exponent': 1.0,
            'emergency_stop_distance': 0.0,
            'data_timeout': 0.5,
            'wheelbase_m': 0.33,
            'max_steering_angle_rad': 0.50,
            'safe_cmd_feedback_topic': '/cmd_vel_safety_filtered',
        }],
    )

    drive_switch_node = Node(
        package='roboracer_py',
        executable='drive_switch_node',
        name='drive_switch_node',
        output='screen',
        remappings=[
            ('/cmd_vel', '/cmd_vel_requested'),
        ],
        parameters=[{
            'default_mode': 'stop',
            'start_armed': False,
            'transition_stop_sec': 0.10,
            'cmd_timeout': 0.50,
            'mode_timeout': 0.50,
            'publish_period': 0.05,
        }],
    )

    safety_brake_node = Node(
        package='roboracer_cpp',
        executable='safety_brake_node',
        name='safety_brake_node',
        output='screen',
        remappings=[
            ('/cmd_vel', '/cmd_vel_safety_filtered'),
        ],
        parameters=[{
            'distance_safety_enabled': True,
            'require_distance_data': ParameterValue(
                require_distance_data,
                value_type=bool,
            ),
            'command_timeout_sec': 0.30,
            'distance_timeout_sec': 0.50,
            'minimum_clearance_m': 0.35,
            'reaction_time_sec': 0.25,
            'braking_deceleration_mps2': 1.00,
            'slowdown_margin_m': 0.50,
            'estop_release_margin_m': 0.10,
            'slowdown_brake_max_request': 0.50,
            'emergency_brake_request': 1.00,
            'command_stop_brake_enabled': True,
            'command_stop_brake_request': 0.60,
            'command_stop_brake_hold_sec': 0.50,
            'publish_period_sec': 0.05,
        }],
    )

    game_controller_node = Node(
        package='joy',
        executable='game_controller_node',
        name='game_controller_node',
        output='screen',
        condition=IfCondition(enable_controller),
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
    )

    controller_manual_input_node = Node(
        package='roboracer_py',
        executable='controller_manual_input_node',
        name='controller_manual_input_node',
        output='screen',
        condition=IfCondition(enable_controller),
        parameters=[{
            'steering_axis': 0,
            'throttle_axis': 1,
            'stop_button': 1,
            'release_button': 10,
            'max_speed_mps': ParameterValue(
                controller_max_speed_mps,
                value_type=float,
            ),
            'max_turn_angular_z': 0.70,
            'deadband': 0.08,
            'invert_throttle': False,
            'invert_steering': False,
            'publish_rate_hz': 20.0,
            'joy_timeout_sec': 0.50,
        }],
    )

    controller_priority_mux_node = Node(
        package='roboracer_py',
        executable='controller_priority_mux_node',
        name='controller_priority_mux_node',
        output='screen',
        parameters=[{
            'publish_rate_hz': 30.0,
            'controller_timeout_sec': 0.50,
            'safe_command_timeout_sec': 0.50,
            'takeover_linear_threshold': 0.02,
            'takeover_angular_threshold': 0.02,
        }],
    )

    real_cmd_converter = Node(
        package='roboracer_py',
        executable='cmd_vel_to_ackermann_node',
        name='real_cmd_vel_to_ackermann_node',
        output='screen',
        remappings=[
            ('/cmd_vel', '/cmd_vel_safe'),
        ],
        parameters=[{
            'wheelbase_m': 0.33,
            'max_speed_mps': 1.0,
            'max_reverse_speed_mps': 0.5,
            'max_steering_angle_rad': 0.50,
            'min_speed_for_steering_mps': 0.05,
            'low_speed_turn_speed_mps': 0.20,
        }],
    )

    vehicle_driver_node = Node(
        package='roboracer_py',
        executable='vehicle_driver_node',
        name='vehicle_driver_node',
        output='screen',
        parameters=[{
            'wheel_diameter_m': 0.102,
            'total_gear_ratio': 8.0,
            'max_speed_mps': 1.0,
            'max_reverse_speed_mps': 0.5,
            'max_steering_angle_rad': 0.50,
            'throttle_neutral_pwm': 1500,
            'throttle_forward_max_pwm': 1600,
            'throttle_reverse_max_pwm': 1400,
            'steering_center_pwm': 1500,
            'steering_left_max_pwm': 2000,
            'steering_right_max_pwm': 1000,
            'command_timeout_sec': 0.50,
            'brake_command_timeout_sec': 0.30,
            'debug_max_brake_current_a': 5.0,
            'publish_period': 0.05,
            'publish_debug': True,
            'hardware_output_enable': ParameterValue(
                hardware_output_enable,
                value_type=bool,
            ),
        }],
    )

    return LaunchDescription(
        launch_arguments + [
            lidar_scan_validator_node,
            perception_node,
            control_node,
            drive_switch_node,
            safety_brake_node,
            game_controller_node,
            controller_manual_input_node,
            controller_priority_mux_node,
            real_cmd_converter,
            vehicle_driver_node,
        ]
    )

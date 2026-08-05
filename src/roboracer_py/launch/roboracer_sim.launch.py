from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # start_rc_sim launches this file without arguments, so controller input
    # is enabled by default. The final priority mux always runs because it is
    # also the bridge between AEB-filtered commands and /cmd_vel_safe.
    enable_controller = LaunchConfiguration('enable_controller')

    enable_controller_arg = DeclareLaunchArgument(
        'enable_controller',
        default_value='true',
        description=(
            'Start game_controller_node and controller_manual_input_node.'
        ),
    )

    perception_node = Node(
        package='roboracer_py',
        executable='perception_node',
        name='perception_node',
        output='screen',
    )

    control_node = Node(
        package='roboracer_cpp',
        executable='control_node',
        name='control_node',
        output='screen',
        # Remap any control-node /cmd_vel output to the autonomous command topic.
        remappings=[
            ('/cmd_vel', '/auto_cmd_vel'),
        ],
        parameters=[{
            'safe_distance': 1.7,
            'forward_speed': 0.5,
            'turn_speed': 0.6,
            'turn_forward_speed': 0.2,
            # Normal Auto steering starts gently at safe_distance and grows
            # smoothly to the configured maximum steering angle as the front
            # obstacle approaches auto_full_steering_distance_m.
            'auto_min_steering_scale': 0.00,
            'auto_full_steering_distance_m': 0.90,
            'auto_steering_curve_exponent': 1.0,
            'auto_distance_filter_alpha': 0.25,
            'auto_max_steering_rate_rad_s': 1.20,
            'auto_direction_switch_deadband_m': 0.25,
            'control_period_sec': 0.02,
            # Disable the control-node fixed E-stop; downstream AEB handles safety.
            'emergency_stop_distance': 0.0,
            'data_timeout': 0.5,
        }],
    )

    drive_switch_node = Node(
        package='roboracer_py',
        executable='drive_switch_node',
        name='drive_switch_node',
        output='screen',
        # Remap the drive-switch output into the requested-command stage.
        remappings=[
            ('/cmd_vel', '/cmd_vel_requested'),
        ],
        parameters=[{
            'default_mode': 'stop',
            'start_armed': False,
            'transition_stop_sec': 0.1,
            'cmd_timeout': 0.5,
            'mode_timeout': 0.5,
            'publish_period': 0.02,
        }],
    )

    safety_brake_node = Node(
        package='roboracer_cpp',
        executable='safety_brake_node',
        name='safety_brake_node',
        output='screen',
        # Route AEB output through the controller-priority mux.
        remappings=[
            ('/cmd_vel', '/cmd_vel_safety_filtered'),
        ],
        parameters=[{
            'distance_safety_enabled': True,
            'require_distance_data': True,
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
            'publish_period_sec': 0.02,
        }],
    )

    # Start the ROS 2 controller driver as part of the simulator stack.
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
            # Confirm the actual mapping with: ros2 topic echo /joy
            'steering_axis': 0,
            'throttle_axis': 1,
            'stop_button': 1,
            'release_button': 10,
            'max_speed_mps': 0.35,
            'max_turn_angular_z': 0.70,
            'deadband': 0.08,
            'invert_throttle': False,
            'invert_steering': False,
            'publish_rate_hz': 20.0,
            'joy_timeout_sec': 0.50,
        }],
    )

    # Always run the mux. With controller input disabled, it forwards
    # AEB-filtered terminal or autonomous commands unchanged.
    controller_priority_mux_node = Node(
        package='roboracer_py',
        executable='controller_priority_mux_node',
        name='controller_priority_mux_node',
        output='screen',
        parameters=[{
            'publish_rate_hz': 50.0,
            'controller_timeout_sec': 0.50,
            'safe_command_timeout_sec': 0.50,
            'takeover_linear_threshold': 0.02,
            'takeover_angular_threshold': 0.02,
            # The startup Stop state must not permanently lock the mux.
            'accept_stop_from_drive_mode': False,
        }],
    )

    cmd_vel_to_ackermann_node = Node(
        package='roboracer_py',
        executable='cmd_vel_to_ackermann_node',
        name='cmd_vel_to_ackermann_node',
        output='screen',
        remappings=[
            ('/cmd_vel', '/cmd_vel_safe'),
            ('/drive_target', '/drive'),
        ],
    )

    return LaunchDescription([
        enable_controller_arg,
        perception_node,
        control_node,
        drive_switch_node,
        safety_brake_node,
        game_controller_node,
        controller_manual_input_node,
        controller_priority_mux_node,
        cmd_vel_to_ackermann_node,
    ])

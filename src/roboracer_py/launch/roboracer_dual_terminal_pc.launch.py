from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Run the PC/VM side of Dual mode.

    This launch intentionally does not start perception, control, AEB,
    controller, terminal, or vehicle-driver nodes. Those run only on the Pi.

    The PC receives /cmd_vel_safe from the Pi, converts it to the simulator's
    Ackermann /drive command, and optionally starts the F1TENTH gym bridge.
    """

    enable_simulator = LaunchConfiguration('enable_simulator')
    enable_sim_converter = LaunchConfiguration('enable_sim_converter')
    sim_input_topic = LaunchConfiguration('sim_input_topic')
    sim_drive_topic = LaunchConfiguration('sim_drive_topic')
    sim_max_speed_mps = LaunchConfiguration('sim_max_speed_mps')
    sim_max_reverse_speed_mps = LaunchConfiguration(
        'sim_max_reverse_speed_mps'
    )
    sim_max_steering_angle_rad = LaunchConfiguration(
        'sim_max_steering_angle_rad'
    )

    launch_arguments = [
        DeclareLaunchArgument(
            'enable_simulator',
            default_value='true',
            description='Start f1tenth_gym_ros gym_bridge_launch.py.',
        ),
        DeclareLaunchArgument(
            'enable_sim_converter',
            default_value='true',
            description=(
                'Convert the Pi final command /cmd_vel_safe into the '
                'simulator Ackermann /drive topic.'
            ),
        ),
        DeclareLaunchArgument(
            'sim_input_topic',
            default_value='/cmd_vel_safe',
            description='Final Twist command received from the Pi.',
        ),
        DeclareLaunchArgument(
            'sim_drive_topic',
            default_value='/drive',
            description='Ackermann command topic subscribed by gym_bridge.',
        ),
        DeclareLaunchArgument(
            'sim_max_speed_mps',
            default_value='1.0',
            description='Simulator forward speed limit.',
        ),
        DeclareLaunchArgument(
            'sim_max_reverse_speed_mps',
            default_value='0.5',
            description='Simulator reverse speed limit.',
        ),
        DeclareLaunchArgument(
            'sim_max_steering_angle_rad',
            default_value='0.65',
            description='Simulator steering-angle limit.',
        ),
    ]

    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('f1tenth_gym_ros'),
                'launch',
                'gym_bridge_launch.py',
            ])
        ),
        condition=IfCondition(enable_simulator),
    )

    sim_cmd_converter = Node(
        package='roboracer_py',
        executable='cmd_vel_to_ackermann_node',
        name='sim_cmd_vel_to_ackermann_node',
        output='screen',
        condition=IfCondition(enable_sim_converter),
        remappings=[
            ('/cmd_vel', sim_input_topic),
            ('/drive_target', sim_drive_topic),
        ],
        parameters=[{
            'wheelbase_m': 0.33,
            'max_speed_mps': ParameterValue(
                sim_max_speed_mps,
                value_type=float,
            ),
            'max_reverse_speed_mps': ParameterValue(
                sim_max_reverse_speed_mps,
                value_type=float,
            ),
            'max_steering_angle_rad': ParameterValue(
                sim_max_steering_angle_rad,
                value_type=float,
            ),
            'min_speed_for_steering_mps': 0.05,
            'low_speed_turn_speed_mps': 0.15,
        }],
    )

    return LaunchDescription(
        launch_arguments + [
            simulator_launch,
            sim_cmd_converter,
        ]
    )

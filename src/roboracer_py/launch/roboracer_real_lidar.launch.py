from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_topic = LaunchConfiguration('scan_topic')
    expected_frame_id = LaunchConfiguration('expected_frame_id')
    start_perception = LaunchConfiguration('start_perception')

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LaserScan topic provided by the real LiDAR driver.',
    )

    expected_frame_id_arg = DeclareLaunchArgument(
        'expected_frame_id',
        default_value='',
        description=(
            'Expected LaserScan frame_id. Leave empty to disable the check.'
        ),
    )

    start_perception_arg = DeclareLaunchArgument(
        'start_perception',
        default_value='true',
        description='Start perception_node with the LiDAR validator.',
    )

    lidar_scan_validator_node = Node(
        package='roboracer_py',
        executable='lidar_scan_validator_node',
        name='lidar_scan_validator_node',
        output='screen',
        parameters=[{
            'scan_topic': scan_topic,
            'expected_frame_id': expected_frame_id,
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
        condition=IfCondition(start_perception),
        remappings=[
            ('/scan', scan_topic),
        ],
    )

    return LaunchDescription([
        scan_topic_arg,
        expected_frame_id_arg,
        start_perception_arg,
        lidar_scan_validator_node,
        perception_node,
    ])

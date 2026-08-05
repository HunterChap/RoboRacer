from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'roboracer_py'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='jojokia05',
    maintainer_email='jojokia05@todo.todo',
    description=(
        'RoboRacer Python nodes for perception, command routing, controller '
        'override, Ackermann conversion, and vehicle-output integration.'
    ),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'perception_node = roboracer_py.perception_node:main',
            'drive_switch_node = roboracer_py.drive_switch_node:main',
            'cmd_vel_to_ackermann_node = '
            'roboracer_py.cmd_vel_to_ackermann_node:main',
            'vehicle_driver_node = roboracer_py.vehicle_driver_node:main',
            'lidar_scan_validator_node = '
            'roboracer_py.lidar_scan_validator_node:main',
            'terminal_command_node = '
            'roboracer_py.terminal_command_node:main',
            'controller_manual_input_node = '
            'roboracer_py.controller_manual_input_node:main',
            'controller_priority_mux_node = '
            'roboracer_py.controller_priority_mux_node:main',
        ],
    },
)

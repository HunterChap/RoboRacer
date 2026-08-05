#!/usr/bin/env bash
# Stop the standalone PC simulator and its local RoboRacer sim control nodes.

set -o pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
TMUX_SESSION="${TMUX_SESSION:-roboracer_sim}"

pkill -f "ros2 launch roboracer_py roboracer_sim.launch.py" 2>/dev/null || true
pkill -f "ros2 launch f1tenth_gym_ros gym_bridge_launch.py" 2>/dev/null || true
pkill -f "f1tenth_gym_ros.*gym_bridge" 2>/dev/null || true
pkill -f "rviz2/rviz2" 2>/dev/null || true
pkill -f "robot_state_publisher/robot_state_publisher" 2>/dev/null || true
pkill -f "ros2 run roboracer_py terminal_command_node" 2>/dev/null || true
pkill -f "roboracer_py.*terminal_command_node" 2>/dev/null || true
pkill -f "roboracer_py.*perception_node" 2>/dev/null || true
pkill -f "roboracer_cpp.*control_node" 2>/dev/null || true
pkill -f "roboracer_cpp.*safety_brake_node" 2>/dev/null || true
pkill -f "roboracer_py.*drive_switch_node" 2>/dev/null || true
pkill -f "roboracer_py.*controller_manual_input_node" 2>/dev/null || true
pkill -f "roboracer_py.*controller_priority_mux_node" 2>/dev/null || true
pkill -f "joy.*game_controller_node" 2>/dev/null || true
pkill -f "roboracer_py.*cmd_vel_to_ackermann_node" 2>/dev/null || true
pkill -f "sim_vehicle_monitor.py" 2>/dev/null || true

if command -v tmux >/dev/null 2>&1 && tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true
fi

if [[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
    set +u
    source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
    ros2 daemon stop 2>/dev/null || true
fi

echo "Standalone simulator stopped."

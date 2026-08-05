#!/usr/bin/env bash
# Stop the standalone real-car stack on the Pi.

set -o pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
ROBORACER_WS="${ROBORACER_WS:-$HOME/roboracer_ws}"
TMUX_SESSION="${TMUX_SESSION:-roboracer_real}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
USE_DISCOVERY_SERVER="${USE_DISCOVERY_SERVER:-false}"
ROS_DISCOVERY_SERVER="${ROS_DISCOVERY_SERVER:-100.83.24.46:11811}"

set +u
[[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]] && source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
[[ -f "${ROBORACER_WS}/install/setup.bash" ]] && source "${ROBORACER_WS}/install/setup.bash"
export ROS_DOMAIN_ID RMW_IMPLEMENTATION
if [[ "$USE_DISCOVERY_SERVER" == "true" ]]; then
    export ROS_DISCOVERY_SERVER
else
    unset ROS_DISCOVERY_SERVER
fi

ros2 topic pub --once /drive_mode std_msgs/msg/String "{data: 's'}" >/dev/null 2>&1 || true
sleep 0.3

pkill -f "ros2 launch roboracer_py roboracer_real_car.launch.py" 2>/dev/null || true
pkill -f "roboracer_real_car.launch.py" 2>/dev/null || true
pkill -f "ros2 run roboracer_py terminal_command_node" 2>/dev/null || true
pkill -f "roboracer_py.*terminal_command_node" 2>/dev/null || true
pkill -f "roboracer_py.*perception_node" 2>/dev/null || true
pkill -f "roboracer_py.*lidar_scan_validator_node" 2>/dev/null || true
pkill -f "roboracer_cpp.*control_node" 2>/dev/null || true
pkill -f "roboracer_cpp.*safety_brake_node" 2>/dev/null || true
pkill -f "roboracer_py.*drive_switch_node" 2>/dev/null || true
pkill -f "roboracer_py.*controller_manual_input_node" 2>/dev/null || true
pkill -f "roboracer_py.*controller_priority_mux_node" 2>/dev/null || true
pkill -f "joy.*game_controller_node" 2>/dev/null || true
pkill -f "roboracer_py.*cmd_vel_to_ackermann_node" 2>/dev/null || true
pkill -f "roboracer_py.*vehicle_driver_node" 2>/dev/null || true

if command -v tmux >/dev/null 2>&1 && tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true
fi

echo "Standalone real-car stack stopped."

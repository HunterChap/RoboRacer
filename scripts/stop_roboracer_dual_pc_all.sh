#!/usr/bin/env bash
# Stop only the PC/VM side of split Dual mode.
# This intentionally does NOT publish Stop Mode to the Pi or stop the real car.

set -o pipefail

TMUX_SESSION="${TMUX_SESSION:-roboracer_dual_pc}"

pkill -f "ros2 launch roboracer_py roboracer_dual_terminal_pc.launch.py" 2>/dev/null || true
pkill -f "roboracer_dual_terminal_pc.launch.py" 2>/dev/null || true
pkill -f "sim_cmd_vel_to_ackermann_node" 2>/dev/null || true
pkill -f "ros2 launch f1tenth_gym_ros gym_bridge_launch.py" 2>/dev/null || true
pkill -f "f1tenth_gym_ros.*gym_bridge" 2>/dev/null || true
pkill -f "rviz2/rviz2" 2>/dev/null || true
pkill -f "robot_state_publisher/robot_state_publisher" 2>/dev/null || true
pkill -f "sim_vehicle_monitor.py" 2>/dev/null || true

if command -v tmux >/dev/null 2>&1 && tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true
fi

echo "PC Dual companion stopped. The Pi control/real-car stack was not stopped."

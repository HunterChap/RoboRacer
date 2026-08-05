#!/usr/bin/env bash
# Start the standalone real-car stack on the Raspberry Pi.
# This mode does not require the PC, simulator, Tailscale, or Discovery Server.

set -o pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
ROBORACER_WS="${ROBORACER_WS:-$HOME/roboracer_ws}"
TMUX_SESSION="${TMUX_SESSION:-roboracer_real}"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
USE_DISCOVERY_SERVER="${USE_DISCOVERY_SERVER:-false}"
ROS_DISCOVERY_SERVER="${ROS_DISCOVERY_SERVER:-100.83.24.46:11811}"

ENABLE_AUTO_STACK="${ENABLE_AUTO_STACK:-true}"
ENABLE_CONTROLLER="${ENABLE_CONTROLLER:-true}"
ENABLE_LIDAR_VALIDATOR="${ENABLE_LIDAR_VALIDATOR:-true}"
HARDWARE_OUTPUT_ENABLE="${HARDWARE_OUTPUT_ENABLE:-false}"
REQUIRE_DISTANCE_DATA="${REQUIRE_DISTANCE_DATA:-false}"
SCAN_TOPIC="${SCAN_TOPIC:-/lidar/scan/points}"
CONTROLLER_MAX_SPEED_MPS="${CONTROLLER_MAX_SPEED_MPS:-0.35}"
START_TERMINAL_ON_PI="${START_TERMINAL_ON_PI:-true}"
SLEEP_AFTER_REAL_LAUNCH="${SLEEP_AFTER_REAL_LAUNCH:-4}"

source_or_fail() {
    local setup_file="$1"
    if [[ ! -f "$setup_file" ]]; then
        echo "ERROR: setup file not found: $setup_file" >&2
        exit 1
    fi
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
}

source_or_fail "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source_or_fail "${ROBORACER_WS}/install/setup.bash"
export ROS_DOMAIN_ID RMW_IMPLEMENTATION
if [[ "$USE_DISCOVERY_SERVER" == "true" ]]; then
    export ROS_DISCOVERY_SERVER
else
    unset ROS_DISCOVERY_SERVER
fi

node_exists() {
    ros2 node list 2>/dev/null | grep -qx "$1"
}

safe_window_name() {
    printf '%s' "$1" | tr '[:upper:]' '[:lower:]' | tr -c 'a-z0-9_-' '_' | cut -c1-20
}

open_terminal() {
    local title="$1"
    local body="$2"
    local window_name runner
    window_name="$(safe_window_name "$title")"
    runner="/tmp/roboracer_${USER}_${window_name}_$$.sh"

    cat > "$runner" <<RUNNER_EOF
#!/usr/bin/env bash
set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source "${ROBORACER_WS}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}"
if [[ "${USE_DISCOVERY_SERVER}" == "true" ]]; then
    export ROS_DISCOVERY_SERVER="${ROS_DISCOVERY_SERVER}"
else
    unset ROS_DISCOVERY_SERVER
fi
${body}
status=\$?
echo
echo "[${title}] exited with status \${status}."
echo "Press Ctrl+D or close this terminal."
exec bash
RUNNER_EOF
    chmod +x "$runner"

    if [[ -n "${DISPLAY:-}" ]] && command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="$title" -- bash -lc "$runner"
    elif command -v tmux >/dev/null 2>&1; then
        if ! tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
            tmux new-session -d -s "$TMUX_SESSION" -n launcher "bash"
        fi
        tmux new-window -t "$TMUX_SESSION" -n "$window_name" "$runner"
        echo "Opened tmux window: ${TMUX_SESSION}:${window_name}"
    else
        echo "ERROR: tmux is required on a headless Pi." >&2
        return 1
    fi
}

if node_exists "/drive_switch_node" || node_exists "/controller_priority_mux_node" || node_exists "/vehicle_driver_node"; then
    echo "ERROR: a RoboRacer control stack is already running." >&2
    echo "Run stop_rc_real or stop_rc_dual first." >&2
    exit 1
fi

cat <<EOF
Starting standalone real-car system...
  Hardware output:       ${HARDWARE_OUTPUT_ENABLE}
  Auto stack:            ${ENABLE_AUTO_STACK}
  Controller input:      ${ENABLE_CONTROLLER}
  LiDAR validator:       ${ENABLE_LIDAR_VALIDATOR}
  Scan topic:            ${SCAN_TOPIC}
  Require distance data: ${REQUIRE_DISTANCE_DATA}
  Discovery server used: ${USE_DISCOVERY_SERVER}
EOF

echo "SAFETY: starts in Stop Mode. Keep the vehicle lifted during initial hardware-output tests."

launch_command="ros2 launch roboracer_py roboracer_real_car.launch.py \
enable_auto_stack:=${ENABLE_AUTO_STACK} \
enable_controller:=${ENABLE_CONTROLLER} \
enable_lidar_validator:=${ENABLE_LIDAR_VALIDATOR} \
hardware_output_enable:=${HARDWARE_OUTPUT_ENABLE} \
require_distance_data:=${REQUIRE_DISTANCE_DATA} \
scan_topic:=${SCAN_TOPIC} \
controller_max_speed_mps:=${CONTROLLER_MAX_SPEED_MPS}"

open_terminal "RoboRacer Real Car" "$launch_command"
sleep "$SLEEP_AFTER_REAL_LAUNCH"

if [[ "$START_TERMINAL_ON_PI" == "true" ]]; then
    open_terminal "RoboRacer Terminal Pi" \
        "ros2 run roboracer_py terminal_command_node --ros-args \
-p max_abs_speed_mps:=1.0 \
-p max_steering_angle_rad:=0.50 \
-p wheelbase_m:=0.33"
fi

open_terminal "RoboRacer Real Check" "
sleep 3

echo '===== Required nodes ====='
for node in \
  /drive_switch_node \
  /safety_brake_node \
  /controller_priority_mux_node \
  /real_cmd_vel_to_ackermann_node \
  /vehicle_driver_node; do
    if ros2 node list 2>/dev/null | grep -qx \"\$node\"; then
        echo \"OK      \$node\"
    else
        echo \"MISSING \$node\"
    fi
done

echo
echo '===== Command and vehicle output ====='
for topic in /cmd_vel_requested /cmd_vel_safety_filtered /cmd_vel_safe /drive_target; do
    echo
    echo \"--- \$topic ---\"
    ros2 topic info \"\$topic\" -v || true
done
ros2 param get /vehicle_driver_node hardware_output_enable || true
ros2 topic info '${SCAN_TOPIC}' -v || true
"

echo
echo "Standalone real-car stack started in Stop Mode."
if command -v tmux >/dev/null 2>&1; then
    echo "Attach with: tmux attach -t ${TMUX_SESSION}"
fi

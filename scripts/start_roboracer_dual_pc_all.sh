#!/usr/bin/env bash
# Start the PC/VM side of split Dual mode:
#   - F1TENTH gym bridge / simulator / RViz
#   - simulator cmd_vel_to_ackermann converter
#   - optional monitor and optional terminal_command_node
# The Pi owns the real control, safety, and vehicle-output pipeline.

set -o pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
F1TENTH_WS="${F1TENTH_WS:-$HOME/f1tenth_ws}"
ROBORACER_WS="${ROBORACER_WS:-$HOME/roboracer_ws}"
TMUX_SESSION="${TMUX_SESSION:-roboracer_dual_pc}"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
USE_DISCOVERY_SERVER="${USE_DISCOVERY_SERVER:-false}"
ROS_DISCOVERY_SERVER="${ROS_DISCOVERY_SERVER:-100.83.24.46:11811}"

ENABLE_SIMULATOR="${ENABLE_SIMULATOR:-true}"
ENABLE_SIM_CONVERTER="${ENABLE_SIM_CONVERTER:-true}"
START_MONITOR="${START_MONITOR:-true}"
START_TERMINAL_ON_PC="${START_TERMINAL_ON_PC:-true}"
SLEEP_AFTER_LAUNCH="${SLEEP_AFTER_LAUNCH:-6}"

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

apply_network_env() {
    export ROS_DOMAIN_ID RMW_IMPLEMENTATION
    if [[ "$USE_DISCOVERY_SERVER" == "true" ]]; then
        export ROS_DISCOVERY_SERVER
    else
        unset ROS_DISCOVERY_SERVER
    fi
}

source_or_fail "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source_or_fail "${F1TENTH_WS}/install/setup.bash"
source_or_fail "${ROBORACER_WS}/install/setup.bash"
apply_network_env

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
source "${F1TENTH_WS}/install/setup.bash"
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
        echo "ERROR: install gnome-terminal or tmux." >&2
        echo "Command that would have run:" >&2
        echo "$body" >&2
        return 1
    fi
}

if pgrep -af "roboracer_dual_terminal_pc.launch.py" >/dev/null 2>&1; then
    echo "ERROR: the PC Dual launch is already running." >&2
    echo "Run stop_rc_dual first or attach to the existing terminal/tmux session." >&2
    exit 1
fi

cat <<EOF
Starting PC/VM Dual companion...
  ROS_DOMAIN_ID:          ${ROS_DOMAIN_ID}
  RMW implementation:    ${RMW_IMPLEMENTATION}
  Discovery server used: ${USE_DISCOVERY_SERVER}
  Discovery server:      ${ROS_DISCOVERY_SERVER}
  Simulator:             ${ENABLE_SIMULATOR}
  Sim converter:         ${ENABLE_SIM_CONVERTER}
  Terminal on PC:        ${START_TERMINAL_ON_PC}
EOF

launch_command="export NUMBA_DISABLE_JIT=1; \
export NUMBA_JIT_COVERAGE=0; \
ros2 launch roboracer_py roboracer_dual_terminal_pc.launch.py \
enable_simulator:=${ENABLE_SIMULATOR} \
enable_sim_converter:=${ENABLE_SIM_CONVERTER}"

open_terminal "RoboRacer Dual PC" "$launch_command"
sleep "$SLEEP_AFTER_LAUNCH"

if [[ "$START_MONITOR" == "true" ]]; then
    open_terminal "RoboRacer Dual Monitor" \
        "python3 \"${ROBORACER_WS}/scripts/sim_vehicle_monitor.py\""
fi

open_terminal "RoboRacer Dual PC Check" "
sleep 3

echo '===== PC-local nodes ====='
for node in /bridge /sim_cmd_vel_to_ackermann_node; do
    if ros2 node list 2>/dev/null | grep -qx \"\$node\"; then
        echo \"OK      \$node\"
    else
        echo \"MISSING \$node\"
    fi
done

echo
echo '===== Cross-device command input ====='
ros2 topic info /cmd_vel_safe -v || true
ros2 topic echo /cmd_vel_safe --once --timeout 3 || true

echo
echo '===== Simulator output ====='
ros2 topic info /drive -v || true
ros2 topic echo /drive --once --timeout 3 || true
ros2 topic info /ego_racecar/odom -v || true

echo
echo 'Expected: Pi publishes /cmd_vel_safe; PC converter publishes /drive.'
"

if [[ "$START_TERMINAL_ON_PC" == "true" ]]; then
    echo "WARNING: Do not also run terminal_command_node on the Pi." >&2
    open_terminal "RoboRacer Terminal PC" \
        "ros2 run roboracer_py terminal_command_node --ros-args \
-p max_abs_speed_mps:=1.0 \
-p max_steering_angle_rad:=0.50 \
-p wheelbase_m:=0.33"
fi

echo
echo "PC Dual companion started."
if [[ "$USE_DISCOVERY_SERVER" == "true" ]]; then
    echo "The Pi Discovery Server must be running at ${ROS_DISCOVERY_SERVER}."
fi
if command -v tmux >/dev/null 2>&1; then
    echo "Headless/tmux attach command: tmux attach -t ${TMUX_SESSION}"
fi

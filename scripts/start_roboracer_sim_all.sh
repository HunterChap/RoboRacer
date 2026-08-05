#!/usr/bin/env bash
# Start the standalone PC simulator stack:
# gym bridge + roboracer_sim.launch.py + monitor + terminal command UI.

set -o pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
F1TENTH_WS="${F1TENTH_WS:-$HOME/f1tenth_ws}"
ROBORACER_WS="${ROBORACER_WS:-$HOME/roboracer_ws}"
TMUX_SESSION="${TMUX_SESSION:-roboracer_sim}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
USE_DISCOVERY_SERVER="${USE_DISCOVERY_SERVER:-false}"
ROS_DISCOVERY_SERVER="${ROS_DISCOVERY_SERVER:-100.83.24.46:11811}"
SLEEP_AFTER_BRIDGE="${SLEEP_AFTER_BRIDGE:-5}"
SLEEP_AFTER_ROBORACER="${SLEEP_AFTER_ROBORACER:-3}"

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
source_or_fail "${F1TENTH_WS}/install/setup.bash"
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
        return 1
    fi
}

echo "Starting standalone RoboRacer simulator..."

if ! node_exists "/bridge"; then
    open_terminal "F1TENTH Bridge" \
        "export NUMBA_DISABLE_JIT=1; export NUMBA_JIT_COVERAGE=0; \
ros2 launch f1tenth_gym_ros gym_bridge_launch.py"
    sleep "$SLEEP_AFTER_BRIDGE"
else
    echo "Existing /bridge detected; not starting a duplicate."
fi

if ! node_exists "/drive_switch_node"; then
    open_terminal "RoboRacer Sim Nodes" \
        "ros2 launch roboracer_py roboracer_sim.launch.py"
    sleep "$SLEEP_AFTER_ROBORACER"
else
    echo "Existing /drive_switch_node detected; not starting duplicate sim control nodes."
fi

open_terminal "RoboRacer Vehicle Monitor" \
    "python3 \"${ROBORACER_WS}/scripts/sim_vehicle_monitor.py\""

if ! node_exists "/terminal_command_node"; then
    open_terminal "RoboRacer Terminal Command" \
        "ros2 run roboracer_py terminal_command_node"
else
    echo "Existing /terminal_command_node detected; not starting a duplicate."
fi

echo
echo "Standalone simulator startup completed."
if command -v tmux >/dev/null 2>&1; then
    echo "If tmux was used: tmux attach -t ${TMUX_SESSION}"
fi

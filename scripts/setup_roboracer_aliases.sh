#!/usr/bin/env bash
# Install role-appropriate RoboRacer command shortcuts in ~/.bashrc.
# Usage:
#   bash setup_roboracer_aliases.sh pc
#   bash setup_roboracer_aliases.sh pi
#
# An alias is only a short command name. It does not start anything on boot.
# Example: after installing the PC aliases, typing start_rc_dual runs the
# longer start_roboracer_dual_pc_all.sh command.

set -e

ROLE="${1:-${RC_DEVICE_ROLE:-}}"
case "$ROLE" in
    pc|PC|vm|VM) ROLE="pc" ;;
    pi|PI|raspberrypi) ROLE="pi" ;;
    *)
        echo "Usage: bash $0 pc" >&2
        echo "   or: bash $0 pi" >&2
        exit 2
        ;;
esac

BASHRC="${HOME}/.bashrc"
START_MARKER="# >>> RoboRacer aliases >>>"
END_MARKER="# <<< RoboRacer aliases <<<"
TEMP_FILE="$(mktemp)"
SCRIPTS_DIR='"$HOME/roboracer_ws/scripts"'

touch "$BASHRC"
awk -v start="$START_MARKER" -v end="$END_MARKER" '
    $0 == start {inside=1; next}
    $0 == end {inside=0; next}
    !inside {print}
' "$BASHRC" > "$TEMP_FILE"

{
    echo
    echo "$START_MARKER"
    echo "# Device role: ${ROLE}"
    echo "alias rcbuild='cd \"\$HOME/roboracer_ws\" && colcon build --symlink-install --packages-select roboracer_py roboracer_cpp --event-handlers console_direct+'"
    echo "alias rcsource='source /opt/ros/jazzy/setup.bash && source \"\$HOME/roboracer_ws/install/setup.bash\"'"
    echo "alias rcnodes='source /opt/ros/jazzy/setup.bash && source \"\$HOME/roboracer_ws/install/setup.bash\" && ros2 node list'"

    if [[ "$ROLE" == "pc" ]]; then
        echo "alias start_rc_sim='bash \"\$HOME/roboracer_ws/scripts/start_roboracer_sim_all.sh\"'"
        echo "alias stop_rc_sim='bash \"\$HOME/roboracer_ws/scripts/stop_roboracer_sim_all.sh\"'"
        echo "alias start_rc_dual='bash \"\$HOME/roboracer_ws/scripts/start_roboracer_dual_pc_all.sh\"'"
        echo "alias stop_rc_dual='bash \"\$HOME/roboracer_ws/scripts/stop_roboracer_dual_pc_all.sh\"'"
        echo "alias start_rc_dual_pc='bash \"\$HOME/roboracer_ws/scripts/start_roboracer_dual_pc_all.sh\"'"
        echo "alias stop_rc_dual_pc='bash \"\$HOME/roboracer_ws/scripts/stop_roboracer_dual_pc_all.sh\"'"
    else
        echo "alias start_rc_real='bash \"\$HOME/roboracer_ws/scripts/start_roboracer_real_car_all.sh\"'"
        echo "alias stop_rc_real='bash \"\$HOME/roboracer_ws/scripts/stop_roboracer_real_car_all.sh\"'"
        echo "alias start_rc_dual='bash \"\$HOME/roboracer_ws/scripts/start_roboracer_dual_pi_all.sh\"'"
        echo "alias stop_rc_dual='bash \"\$HOME/roboracer_ws/scripts/stop_roboracer_dual_pi_all.sh\"'"
        echo "alias start_rc_dual_pi='bash \"\$HOME/roboracer_ws/scripts/start_roboracer_dual_pi_all.sh\"'"
        echo "alias stop_rc_dual_pi='bash \"\$HOME/roboracer_ws/scripts/stop_roboracer_dual_pi_all.sh\"'"
        echo "alias start_rc_discovery='bash \"\$HOME/roboracer_ws/scripts/start_roboracer_discovery_server.sh\"'"
        echo "alias stop_rc_discovery='bash \"\$HOME/roboracer_ws/scripts/stop_roboracer_discovery_server.sh\"'"
    fi
    echo "$END_MARKER"
} >> "$TEMP_FILE"

mv "$TEMP_FILE" "$BASHRC"

echo "Installed RoboRacer aliases for role: ${ROLE}"
if [[ "$ROLE" == "pc" ]]; then
    echo "  start_rc_sim / stop_rc_sim"
    echo "  start_rc_dual / stop_rc_dual   (PC companion)"
else
    echo "  start_rc_real / stop_rc_real"
    echo "  start_rc_dual / stop_rc_dual   (Pi control stack)"
    echo "  start_rc_discovery / stop_rc_discovery"
fi
echo "  rcbuild / rcsource / rcnodes"
echo
echo "Activate now with: source ${BASHRC}"

#!/usr/bin/env bash
# Start a standalone Fast DDS Discovery Server on the Raspberry Pi.
# Keep this separate from Dual/Real sessions so stopping a control stack does
# not unexpectedly remove discovery for the other device.

set -euo pipefail

TMUX_SESSION="${DISCOVERY_TMUX_SESSION:-roboracer_discovery}"
PI_TAILSCALE_IP="${PI_TAILSCALE_IP:-100.83.24.46}"
DISCOVERY_PORT="${DISCOVERY_PORT:-11811}"

command -v fastdds >/dev/null 2>&1 || {
    echo "ERROR: fastdds was not found." >&2
    echo "Install it with: sudo apt update && sudo apt install -y fastdds-tools" >&2
    exit 1
}
command -v tmux >/dev/null 2>&1 || {
    echo "ERROR: tmux was not found: sudo apt install -y tmux" >&2
    exit 1
}

if tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    echo "Discovery Server session already exists: ${TMUX_SESSION}"
    echo "Attach with: tmux attach -t ${TMUX_SESSION}"
    exit 0
fi

command="source /opt/ros/jazzy/setup.bash; exec fastdds discovery --server-id 0 --udp-address '${PI_TAILSCALE_IP}' --udp-port '${DISCOVERY_PORT}'"
tmux new-session -d -s "$TMUX_SESSION" -n discovery "bash -lc \"$command\""

echo "Fast DDS Discovery Server started at ${PI_TAILSCALE_IP}:${DISCOVERY_PORT}."
echo "Attach with: tmux attach -t ${TMUX_SESSION}"

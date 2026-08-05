#!/usr/bin/env bash
set -o pipefail
TMUX_SESSION="${DISCOVERY_TMUX_SESSION:-roboracer_discovery}"
if command -v tmux >/dev/null 2>&1 && tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true
fi
pkill -f "fastdds discovery" 2>/dev/null || true
echo "Fast DDS Discovery Server stopped."

#!/usr/bin/env bash
# Compatibility wrapper for the old single-machine Dual script name.
# Prefer the role-aware aliases installed by setup_roboracer_aliases.sh.

set -e
ROLE="${1:-${RC_DEVICE_ROLE:-}}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

case "$ROLE" in
    pc|PC|vm|VM)
        exec bash "${SCRIPT_DIR}/start_roboracer_dual_pc_all.sh"
        ;;
    pi|PI|raspberrypi)
        exec bash "${SCRIPT_DIR}/start_roboracer_dual_pi_all.sh"
        ;;
    *)
        echo "The old Dual script has been split into PC and Pi versions." >&2
        echo "Run one of:" >&2
        echo "  bash ${SCRIPT_DIR}/start_roboracer_dual_pc_all.sh" >&2
        echo "  bash ${SCRIPT_DIR}/start_roboracer_dual_pi_all.sh" >&2
        echo "Or pass a role: $0 pc | $0 pi" >&2
        exit 2
        ;;
esac

#!/usr/bin/env bash
# Source this file before starting PX4/Gazebo with custom map-consensus assets.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTO_WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
PROFILE_FILE="$AUTO_WS/config/workspace_profile.env"
if [[ -f "$PROFILE_FILE" ]]; then
    # shellcheck disable=SC1090
    source "$PROFILE_FILE"
fi

WS="${WS:-${MAP_CONSENSUS_WS:-$AUTO_WS}}"
export MAP_CONSENSUS_WS="$WS"
PX4_ROOT="${PX4_DIR:-$HOME/PX4-Autopilot}"

append_resource_path() {
    local candidate="$1"
    [[ -d "${candidate}" ]] || return 0

    case ":${GZ_SIM_RESOURCE_PATH:-}:" in
        *":${candidate}:"*) ;;
        *)
            if [[ -n "${GZ_SIM_RESOURCE_PATH:-}" ]]; then
                export GZ_SIM_RESOURCE_PATH="${candidate}:${GZ_SIM_RESOURCE_PATH}"
            else
                export GZ_SIM_RESOURCE_PATH="${candidate}"
            fi
            ;;
    esac
}

append_resource_path "${WS}/src/multi_uav_sim/models"
append_resource_path "${WS}/src/multi_uav_sim/worlds"

append_resource_path "${PX4_ROOT}/Tools/simulation/gz/models"
append_resource_path "${PX4_ROOT}/Tools/simulation/gz/worlds"
append_resource_path "${PX4_ROOT}/Tools/simulation/gz"

append_resource_path "${HOME}/.simulation-gazebo/models"
append_resource_path "${HOME}/.simulation-gazebo/worlds"
append_resource_path "${HOME}/.simulation-gazebo"

export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-0,0,0,0,0,0}"

echo "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"

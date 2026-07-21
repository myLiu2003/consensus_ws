#!/usr/bin/env bash
# Source this file before starting PX4/Gazebo with custom map-consensus assets.

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

append_resource_path "${HOME}/consensus_ws/src/multi_uav_sim/models"
append_resource_path "${HOME}/consensus_ws/src/multi_uav_sim/worlds"

append_resource_path "${HOME}/PX4-Autopilot/Tools/simulation/gz/models"
append_resource_path "${HOME}/PX4-Autopilot/Tools/simulation/gz/worlds"
append_resource_path "${HOME}/PX4-Autopilot/Tools/simulation/gz"

append_resource_path "${HOME}/.simulation-gazebo/models"
append_resource_path "${HOME}/.simulation-gazebo/worlds"
append_resource_path "${HOME}/.simulation-gazebo"

export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-0,0,0,0,0,0}"

echo "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"

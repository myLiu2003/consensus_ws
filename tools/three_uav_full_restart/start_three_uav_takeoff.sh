#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTO_WS="$(cd "$SCRIPT_DIR/../.." && pwd)"
PROFILE_FILE="$AUTO_WS/config/workspace_profile.env"
if [[ -f "$PROFILE_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$PROFILE_FILE"
fi

WS="${WS:-${MAP_CONSENSUS_WS:-$AUTO_WS}}"
export MAP_CONSENSUS_WS="$WS"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
exec ros2 run multi_uav_offboard three_uav_waypoints

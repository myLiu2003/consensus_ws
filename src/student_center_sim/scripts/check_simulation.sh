#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTO_WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
PROFILE_FILE="$AUTO_WS/config/workspace_profile.env"
if [[ -f "$PROFILE_FILE" ]]; then
    # shellcheck disable=SC1090
    source "$PROFILE_FILE"
fi

WS="${WS:-${MAP_CONSENSUS_WS:-$AUTO_WS}}"
export MAP_CONSENSUS_WS="$WS"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${WS}/install/setup.bash" ]]; then
    source "${WS}/install/setup.bash"
fi
set -u

echo "================ Gazebo world ================"
gz service -l \
    | grep -E "/world/student_center" \
    | head -n 20 \
    || true

echo
echo "================ PX4 processes ================"
pgrep -af "build/px4_sitl_default/bin/px4" || true

echo
echo "================ PX4 ROS 2 topics ================"
ros2 topic list \
    | grep -E "^/px4_[123]/fmu/(in|out)/" \
    | head -n 100 \
    || true

echo
echo "================ Sensor topics ================"
ros2 topic list \
    | grep -E "^/uav[123]/sensors/(lidar|imu)$|^/clock$" \
    || true

echo
echo "================ Recent errors ================"
grep -RniE \
    "error|failed|not found|abort|segmentation" \
    "${HOME}/logs/student_center" \
    2>/dev/null \
    | tail -n 50 \
    || true

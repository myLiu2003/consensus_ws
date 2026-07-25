#!/usr/bin/env bash
set -Eeuo pipefail

LOG_DIR="${HOME}/logs/student_center"
PID_FILE="${LOG_DIR}/bridge_pids.txt"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTO_WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
PROFILE_FILE="$AUTO_WS/config/workspace_profile.env"
if [[ -f "$PROFILE_FILE" ]]; then
    # shellcheck disable=SC1090
    source "$PROFILE_FILE"
fi

WS="${WS:-${MAP_CONSENSUS_WS:-$AUTO_WS}}"
export MAP_CONSENSUS_WS="$WS"

mkdir -p "$LOG_DIR"
: > "$PID_FILE"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${WS}/install/setup.bash" ]]; then
    source "${WS}/install/setup.bash"
fi
set -u

find_topic_by_type() {
    local instance="$1"
    local required_type="$2"

    local topic
    while IFS= read -r topic; do
        local info
        info="$(gz topic -i -t "$topic" 2>&1 || true)"

        if grep -q "$required_type" <<< "$info"; then
            printf '%s\n' "$topic"
            return 0
        fi
    done < <(
        gz topic -l \
            | grep -E "/model/[^/]*_${instance}(/|$)" \
            | sort
    )

    return 1
}

start_clock_bridge() {
    local clock_topic=""

    if gz topic -l | grep -qx "/clock"; then
        clock_topic="/clock"
    else
        clock_topic="$(
            gz topic -l \
                | grep -E "/world/student_center/clock$" \
                | head -n 1 \
                || true
        )"
    fi

    if [[ -z "$clock_topic" ]]; then
        echo "未找到 Gazebo clock 话题。"
        return
    fi

    echo "启动 clock bridge：${clock_topic} -> /clock"

    nohup ros2 run ros_gz_bridge parameter_bridge \
        "${clock_topic}@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
        --ros-args \
        -r "${clock_topic}:=/clock" \
        > "${LOG_DIR}/bridge_clock.log" 2>&1 &

    echo "$!" >> "$PID_FILE"
}

start_sensor_bridge() {
    local instance="$1"

    local lidar_topic
    local imu_topic

    lidar_topic="$(
        find_topic_by_type "$instance" "gz.msgs.PointCloudPacked" || true
    )"

    imu_topic="$(
        find_topic_by_type "$instance" "gz.msgs.IMU" || true
    )"

    if [[ -z "$lidar_topic" ]]; then
        echo "UAV${instance} 未找到 PointCloudPacked 话题。" >&2
    else
        echo "UAV${instance} LiDAR："
        echo "  GZ : ${lidar_topic}"
        echo "  ROS: /uav${instance}/sensors/lidar"

        nohup ros2 run ros_gz_bridge parameter_bridge \
            "${lidar_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
            --ros-args \
            -r "${lidar_topic}:=/uav${instance}/sensors/lidar" \
            > "${LOG_DIR}/bridge_uav${instance}_lidar.log" 2>&1 &

        echo "$!" >> "$PID_FILE"
    fi

    if [[ -z "$imu_topic" ]]; then
        echo "UAV${instance} 未找到 IMU 话题。" >&2
    else
        echo "UAV${instance} IMU："
        echo "  GZ : ${imu_topic}"
        echo "  ROS: /uav${instance}/sensors/imu"

        nohup ros2 run ros_gz_bridge parameter_bridge \
            "${imu_topic}@sensor_msgs/msg/Imu[gz.msgs.IMU" \
            --ros-args \
            -r "${imu_topic}:=/uav${instance}/sensors/imu" \
            > "${LOG_DIR}/bridge_uav${instance}_imu.log" 2>&1 &

        echo "$!" >> "$PID_FILE"
    fi
}

start_clock_bridge

for instance in 1 2 3; do
    start_sensor_bridge "$instance"
done

sleep 2

echo
echo "桥接进程已启动。"
echo "ROS 2 传感器话题："
ros2 topic list \
    | grep -E "^/uav[123]/sensors/(lidar|imu)$|^/clock$" \
    || true

#!/usr/bin/env bash
set -Eeuo pipefail

LOG_DIR="${HOME}/logs/student_center"
PID_FILE="${LOG_DIR}/bridge_pids.txt"
MODEL_PREFIX="consensus_x500_lidar_3d"

mkdir -p "$LOG_DIR"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${HOME}/consensus_ws/install/setup.bash" ]]; then
    source "${HOME}/consensus_ws/install/setup.bash"
fi
set -u

# 清理本用户上一次遗留的桥接，防止重复发布。
pkill -u "${USER}" -f "ros_gz_bridge.*parameter_bridge" 2>/dev/null || true
sleep 1
: > "$PID_FILE"

start_bridge() {
    local argument="$1"
    local gz_topic="$2"
    local ros_topic="$3"
    local log_file="$4"

    echo "启动桥接：${gz_topic} -> ${ros_topic}"

    nohup ros2 run ros_gz_bridge parameter_bridge \
        "$argument" \
        --ros-args \
        -r "${gz_topic}:=${ros_topic}" \
        > "${LOG_DIR}/${log_file}" 2>&1 &

    echo "$!" >> "$PID_FILE"
}

# Gazebo 时钟。
start_bridge \
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
    "/clock" \
    "/clock" \
    "bridge_clock.log"

# 三机 LiDAR 与 IMU。
for instance in 1 2 3; do
    MODEL="${MODEL_PREFIX}_${instance}"

    LIDAR_TOPIC="/world/student_center/model/${MODEL}/link/lidar_link/sensor/lidar_3d/scan/points"
    IMU_TOPIC="/world/student_center/model/${MODEL}/link/base_link/sensor/imu_sensor/imu"

    start_bridge \
        "${LIDAR_TOPIC}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
        "${LIDAR_TOPIC}" \
        "/uav${instance}/sensors/lidar" \
        "bridge_uav${instance}_lidar.log"

    start_bridge \
        "${IMU_TOPIC}@sensor_msgs/msg/Imu[gz.msgs.IMU" \
        "${IMU_TOPIC}" \
        "/uav${instance}/sensors/imu" \
        "bridge_uav${instance}_imu.log"
done

sleep 3

echo
echo "桥接进程已启动："
ros2 topic list \
    | grep -E "^/clock$|^/uav[123]/sensors/(lidar|imu)$" \
    | sort \
    || true
#!/usr/bin/env bash
set -Eeuo pipefail

echo "Gazebo IMU、LiDAR 与点云话题："
echo

mapfile -t TOPICS < <(
    gz topic -l \
        | grep -i -E "imu|lidar|scan|point|cloud" \
        | sort
)

if [[ "${#TOPICS[@]}" -eq 0 ]]; then
    echo "未找到传感器话题。"
    exit 1
fi

for topic in "${TOPICS[@]}"; do
    echo "------------------------------------------------------------"
    echo "Topic: ${topic}"
    gz topic -i -t "$topic" 2>&1 \
        | grep -E "Message Type|Publishers|Subscribers" \
        || true
done

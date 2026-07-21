#!/usr/bin/env bash
set -o pipefail

SESSION="${SESSION:-map_consensus_integrated}"
source /opt/ros/humble/setup.bash
source "$HOME/consensus_ws/install/setup.bash"

echo "========== TMUX WINDOWS =========="
tmux list-windows -t "$SESSION" 2>/dev/null || true

echo
echo "========== REQUIRED TOPICS =========="
TOPICS=(
  /px4_1/fmu/out/vehicle_status_v1
  /px4_2/fmu/out/vehicle_status_v1
  /px4_3/fmu/out/vehicle_status_v1
  /uav1/sensors/lidar
  /uav1/sensors/imu
  /uav2/sensors/lidar
  /uav2/sensors/imu
  /uav3/sensors/lidar
  /uav3/sensors/imu
  /uav1/fast_lio/odometry
  /uav2/fast_lio/odometry
  /uav3/fast_lio/odometry
)
CURRENT_TOPICS="$(ros2 topic list 2>/dev/null || true)"
for topic in "${TOPICS[@]}"; do
  if grep -Fxq "$topic" <<< "$CURRENT_TOPICS"; then
    echo "[OK]      $topic"
  else
    echo "[MISSING] $topic"
  fi
done

echo
echo "========== LATEST MISSION LOG =========="
LOG_ROOT="$HOME/consensus_ws/logs/integrated_stack"
LATEST="$(find "$LOG_ROOT" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort | tail -n 1)"
if [[ -n "${LATEST:-}" && -f "$LATEST/mission.log" ]]; then
  tail -n 100 "$LATEST/mission.log"
else
  echo "[WARN] mission.log not found"
fi

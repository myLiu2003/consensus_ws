#!/usr/bin/env bash
set -o pipefail

BUNDLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_WS="${WS:-${MAP_CONSENSUS_WS:-$HOME/consensus_lcgo_ws}}"
PROFILE_FILE="$BUNDLE_DIR/workspace_profile.env"
if [[ ! -f "$PROFILE_FILE" && -f "$DEFAULT_WS/config/workspace_profile.env" ]]; then
  PROFILE_FILE="$DEFAULT_WS/config/workspace_profile.env"
fi
[[ -f "$PROFILE_FILE" ]] || { echo "[ERROR] Missing workspace_profile.env near $BUNDLE_DIR or in $DEFAULT_WS/config"; exit 1; }
# shellcheck disable=SC1090
source "$PROFILE_FILE"

DEFAULT_WS="${MAP_CONSENSUS_WS:-$DEFAULT_WS}"
WS="${WS:-$DEFAULT_WS}"
export MAP_CONSENSUS_WS="$WS"
SESSION="${SESSION:-map_consensus_integrated}"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

RUN_ROOT="$WS/logs/integrated_stack"
LATEST="$(find "$RUN_ROOT" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort | tail -n 1)"

echo "========== TMUX WINDOWS =========="
tmux list-windows -t "$SESSION" 2>/dev/null || true

echo
echo "========== MAP CONSENSUS NODES =========="
NODES=(
  /multi_uav_tf_manager
  /loop_consensus_node
  /gicp_verifier_node
  /keyframe_frontend_uav1
  /keyframe_frontend_uav2
  /keyframe_frontend_uav3
)
CURRENT_NODES="$(ros2 node list 2>/dev/null || true)"
for node_name in "${NODES[@]}"; do
  if grep -Fxq "$node_name" <<< "$CURRENT_NODES"; then
    echo "[OK]      $node_name"
  else
    echo "[MISSING] $node_name"
  fi
done

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
  /uav1/consensus/keyframe
  /uav2/consensus/keyframe
  /uav3/consensus/keyframe
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
echo "========== LOOP CONSENSUS TOPICS =========="
LOOP_TOPICS=(
  /map_consensus/loop_candidates
  /map_consensus/loop_candidate_markers
  /map_consensus/loop_verifications
  /map_consensus/accepted_loops
  /map_consensus/rejected_loops
)
for topic in "${LOOP_TOPICS[@]}"; do
  if grep -Fxq "$topic" <<< "$CURRENT_TOPICS"; then
    echo "[OK]      $topic"
  else
    echo "[WAITING] $topic"
  fi
done

echo
echo "========== SUBMAP SERVICES =========="
SERVICES=(
  /uav1/consensus/get_submap
  /uav2/consensus/get_submap
  /uav3/consensus/get_submap
)
CURRENT_SERVICES="$(ros2 service list 2>/dev/null || true)"
for service_name in "${SERVICES[@]}"; do
  if grep -Fxq "$service_name" <<< "$CURRENT_SERVICES"; then
    echo "[OK]      $service_name"
  else
    echo "[MISSING] $service_name"
  fi
done

echo
echo "========== LATEST RUN =========="
if [[ -n "${LATEST:-}" ]]; then
  echo "$LATEST"
else
  echo "[WARN] no integrated_stack run directory found"
fi

echo
echo "========== LATEST MISSION LOG =========="
if [[ -n "${LATEST:-}" && -f "$LATEST/mission.log" ]]; then
  tail -n 40 "$LATEST/mission.log"
else
  echo "[WARN] mission.log not found"
fi

echo
echo "========== LATEST MAP CONSENSUS LOG =========="
MAP_LOG=""
if [[ -n "${LATEST:-}" && -f "$LATEST/map_consensus.log" ]]; then
  MAP_LOG="$LATEST/map_consensus.log"
elif [[ -n "${LATEST:-}" && -f "$LATEST/loop_consensus.log" ]]; then
  MAP_LOG="$LATEST/loop_consensus.log"
fi

if [[ -n "$MAP_LOG" ]]; then
  tail -n 60 "$MAP_LOG"
else
  echo "[WARN] map_consensus.log not found"
fi

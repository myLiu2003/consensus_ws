#!/usr/bin/env bash
set -euo pipefail

SESSION="${SESSION:-map_consensus_integrated}"

echo "[INFO] Stopping tmux sessions..."
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux kill-session -t map_consensus_3uav 2>/dev/null || true

PATTERNS=(
  'uav1_fastlio_mission'
  'uav1_fastlio_motion_test'
  'three_uav_waypoints'
  'three_uav_staggered_common_area'
  'multi_uav_tf_manager'
  'keyframe_frontend_node'
  'loop_consensus_node'
  'fastlio_mapping'
  'ros_gz_bridge.*parameter_bridge'
  'MicroXRCEAgent.*udp4.*8888'
  'micro-xrce-dds-agent.*udp4.*8888'
  'QGroundControl'
  'rviz2'
  'px4_sitl_default/bin/px4'
  'gz sim'
  'gz-server'
  'gz-gui'
  'gzserver'
  'gzclient'
)

for pattern in "${PATTERNS[@]}"; do
  pkill -TERM -f "$pattern" 2>/dev/null || true
done
sleep 4
for pattern in "${PATTERNS[@]}"; do
  pkill -KILL -f "$pattern" 2>/dev/null || true
done

rm -rf /tmp/px4-* /tmp/px4io* 2>/dev/null || true
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
ros2 daemon start >/dev/null 2>&1 || true

echo "[INFO] Remaining related processes:"
ps -eo pid=,args= | grep -E \
    'px4_sitl_default/bin/px4|gz sim|gz-server|gz-gui|MicroXRCEAgent|micro-xrce-dds-agent|fastlio_mapping|multi_uav_tf_manager|keyframe_frontend_node|loop_consensus_node|parameter_bridge|rviz2|QGroundControl|uav1_fastlio_mission' \
  | grep -v grep || true

echo "[OK] Previous stack stopped."

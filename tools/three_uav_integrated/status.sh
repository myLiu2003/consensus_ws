#!/usr/bin/env bash
# =============================================================================
# 三无人机地图共识系统状态检查脚本
#
# 检查内容：
#   1. 当前tmux会话和组件窗口
#   2. ROS包是否来自正确的consensus_ws
#   3. 地图共识相关节点
#   4. 三机传感器、FAST-LIO和关键帧话题
#   5. 回环候选和Marker话题
#   6. 相关进程是否混入consensus_lcgo_ws
#   7. 最近一次运行日志
#
# 该脚本只读检查，不会启动或停止任何程序。
# =============================================================================

set -o pipefail


# =============================================================================
# 1. 加载ROS 2环境
# =============================================================================

WS="${WS:-$HOME/consensus_ws}"
TOOLS_DIR="$WS/tools/three_uav_integrated"
SESSION="${SESSION:-map_consensus_integrated}"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"


# =============================================================================
# 2. tmux窗口状态
# =============================================================================

echo
echo "========== TMUX WINDOWS =========="

if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux list-windows -t "$SESSION"
else
  echo "[MISSING] tmux session: $SESSION"
fi


# =============================================================================
# 3. 检查ROS包来源
# =============================================================================

echo
echo "========== ROS PACKAGE PREFIX =========="

PACKAGES=(
  student_center_sim
  fast_lio
  keyframe_frontend
  loop_consensus
  multi_uav_tf_manager
  map_consensus_msgs
)

for package in "${PACKAGES[@]}"; do
  PREFIX="$(ros2 pkg prefix "$package" 2>/dev/null || true)"

  if [[ -z "$PREFIX" ]]; then
    echo "[MISSING] $package"
  elif [[ "$PREFIX" == "$WS/install/"* ]]; then
    echo "[OK]      $package -> $PREFIX"
  else
    echo "[WRONG]   $package -> $PREFIX"
    echo "          expected: $WS/install/..."
  fi
done


# =============================================================================
# 4. 地图共识节点状态
# =============================================================================

echo
echo "========== MAP CONSENSUS NODES =========="

EXPECTED_NODES=(
  /multi_uav_tf_manager
  /keyframe_frontend_uav1
  /keyframe_frontend_uav2
  /keyframe_frontend_uav3
  /loop_consensus_node
)

CURRENT_NODES="$(ros2 node list 2>/dev/null || true)"

for node in "${EXPECTED_NODES[@]}"; do
  if grep -Fxq "$node" <<< "$CURRENT_NODES"; then
    echo "[OK]      $node"
  else
    echo "[MISSING] $node"
  fi
done


# =============================================================================
# 5. 三机传感器和FAST-LIO话题
# =============================================================================

echo
echo "========== REQUIRED DATA TOPICS =========="

REQUIRED_TOPICS=(
  /clock

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

  /uav1/consensus/submap_cloud
  /uav2/consensus/submap_cloud
  /uav3/consensus/submap_cloud
)

CURRENT_TOPICS="$(ros2 topic list 2>/dev/null || true)"

for topic in "${REQUIRED_TOPICS[@]}"; do
  if grep -Fxq "$topic" <<< "$CURRENT_TOPICS"; then
    echo "[OK]      $topic"
  else
    echo "[MISSING] $topic"
  fi
done


# =============================================================================
# 6. 地图共识输出话题
# =============================================================================

echo
echo "========== CONSENSUS OUTPUT TOPICS =========="

CONSENSUS_TOPICS=(
  /map_consensus/loop_candidates
  /map_consensus/loop_candidate_markers
)

for topic in "${CONSENSUS_TOPICS[@]}"; do
  if grep -Fxq "$topic" <<< "$CURRENT_TOPICS"; then
    echo "[OK]      $topic"

    # 显示发布者数量，辅助判断节点是否真正发布。
    ros2 topic info "$topic" 2>/dev/null \
      | grep -E "Type:|Publisher count:|Subscription count:" \
      | sed 's/^/          /' \
      || true
  else
    echo "[MISSING] $topic"
  fi
done


# =============================================================================
# 7. 检查实际运行进程路径
# =============================================================================

echo
echo "========== RELATED PROCESSES =========="

PROCESS_OUTPUT="$(
  ps -u "$USER" -eo pid=,args= \
    | grep -E \
      'px4_sitl_default/bin/px4|gz sim|MicroXRCEAgent|micro-xrce-dds-agent|parameter_bridge|fastlio_mapping|keyframe_frontend_node|loop_consensus_node|multi_uav_tf_manager' \
    | grep -v grep \
    || true
)"

if [[ -z "$PROCESS_OUTPUT" ]]; then
  echo "[WARN] No related processes found."
else
  echo "$PROCESS_OUTPUT"

  if grep -q "consensus_lcgo_ws" <<< "$PROCESS_OUTPUT"; then
    echo
    echo "[WARN] consensus_lcgo_ws process detected."
    echo "[WARN] Please stop it before formal testing."
  else
    echo
    echo "[OK] No consensus_lcgo_ws process detected."
  fi
fi


# =============================================================================
# 8. 最近一次运行日志
# =============================================================================

echo
echo "========== LATEST RUN LOG =========="

LOG_ROOT="$WS/logs/integrated_stack"

LATEST="$(
  find "$LOG_ROOT" \
    -mindepth 1 \
    -maxdepth 1 \
    -type d \
    2>/dev/null \
    | sort \
    | tail -n 1
)"

if [[ -z "${LATEST:-}" ]]; then
  echo "[WARN] No integrated stack log directory found."
else
  echo "[INFO] Latest log directory: $LATEST"
  echo

  for log in \
    world.log \
    agent.log \
    px4.log \
    sensors.log \
    fastlio.log \
    consensus.log \
    mission.log
  do
    if [[ -f "$LATEST/$log" ]]; then
      echo "----- $log -----"
      tail -n 8 "$LATEST/$log"
    fi
  done
fi


# =============================================================================
# 9. 使用提示
# =============================================================================

echo
echo "========== COMMANDS =========="
echo "进入tmux：tmux attach -t $SESSION"
echo "停止系统：$TOOLS_DIR/stop_all.sh"
echo "重新启动：$TOOLS_DIR/restart_all.sh"
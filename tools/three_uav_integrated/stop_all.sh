#!/usr/bin/env bash
# =============================================================================
# 三无人机地图共识系统一键停止脚本
#
# 作用：
#   1. 停止一键启动创建的tmux会话
#   2. 停止飞行任务、地图共识、FAST-LIO
#   3. 停止传感器桥接、PX4和DDS Agent
#   4. 停止Gazebo、RViz和QGroundControl
#   5. 刷新ROS 2 daemon中的旧节点发现记录
#
# 说明：
#   所有pkill命令均限制为当前用户，避免影响服务器上的其他用户。
# =============================================================================

set -Eeuo pipefail


# =============================================================================
# 1. 基础配置
# =============================================================================

# 新版一键启动使用的tmux会话。
SESSION="${SESSION:-map_consensus_integrated}"

# student_center_sim脚本产生的PID记录目录。
STUDENT_LOG_DIR="$HOME/logs/student_center"


# =============================================================================
# 2. 停止tmux会话
# =============================================================================

echo "[INFO] Stopping tmux sessions..."

# 当前新版一键启动会话。
tmux kill-session \
  -t "$SESSION" \
  2>/dev/null || true

# 兼容历史脚本创建的旧会话。
tmux kill-session \
  -t map_consensus_3uav \
  2>/dev/null || true


# =============================================================================
# 3. 定义需要停止的进程
# =============================================================================

# 顺序按“上层应用 → 底层仿真”排列。
# 先停飞行任务和算法，再停传感器、PX4和Gazebo。
PATTERNS=(
  # 飞行任务和运动测试。
  'uav1_fastlio_mission'
  'uav1_fastlio_motion_test'
  'three_uav_waypoints'
  'three_uav_staggered_common_area'

  # 地图共识系统。
  'loop_consensus_node'
  'keyframe_frontend_node'
  'multi_uav_tf_manager.*tf_manager'

  # 三机FAST-LIO。
  'fastlio_mapping'

  # 传感器自动桥接脚本及其子进程。
  'auto_bridge_sensors.sh'
  'ros_gz_bridge.*parameter_bridge'

  # PX4与ROS 2通信Agent。
  'MicroXRCEAgent.*udp4.*8888'
  'micro-xrce-dds-agent.*udp4.*8888'

  # 可选图形界面。
  'QGroundControl'
  'rviz2'

  # 三架PX4 SITL。
  'px4_sitl_default/bin/px4'

  # Gazebo Sim相关进程。
  'student_center_world.launch.py'
  'gz sim'
  'gz-server'
  'gz-gui'
  'gzserver'
  'gzclient'
)


# =============================================================================
# 4. 优雅停止进程
# =============================================================================

echo "[INFO] Sending SIGTERM to related processes..."

# SIGTERM允许节点执行正常退出和资源释放。
for pattern in "${PATTERNS[@]}"; do
  pkill \
    -TERM \
    -u "$USER" \
    -f "$pattern" \
    2>/dev/null || true
done

# 给各节点、Gazebo和PX4最多4秒正常退出。
sleep 4


# =============================================================================
# 5. 强制清理未退出进程
# =============================================================================

echo "[INFO] Force-stopping remaining processes..."

# 对仍未退出的相关进程发送SIGKILL。
for pattern in "${PATTERNS[@]}"; do
  pkill \
    -KILL \
    -u "$USER" \
    -f "$pattern" \
    2>/dev/null || true
done


# =============================================================================
# 6. 清理本项目的PID记录
# =============================================================================

# 这里只删除student_center_sim生成的PID文本，
# 不使用rm -rf /tmp/px4-*，避免误删其他用户的临时文件。
rm -f \
  "$STUDENT_LOG_DIR/px4_pids.txt" \
  "$STUDENT_LOG_DIR/bridge_pids.txt" \
  2>/dev/null || true


# =============================================================================
# 7. 刷新ROS 2 daemon
# =============================================================================

# 清除已经退出但暂时仍显示在ROS图中的旧节点和旧话题。
ros2 daemon stop \
  >/dev/null 2>&1 || true

sleep 1

ros2 daemon start \
  >/dev/null 2>&1 || true


# =============================================================================
# 8. 检查是否仍有相关进程
# =============================================================================

echo
echo "[INFO] Remaining related processes:"

REMAINING="$(
  ps -u "$USER" -eo pid=,args= \
    | grep -E \
      'px4_sitl_default/bin/px4|gz sim|gz-server|gz-gui|MicroXRCEAgent|micro-xrce-dds-agent|fastlio_mapping|keyframe_frontend_node|loop_consensus_node|multi_uav_tf_manager|parameter_bridge|rviz2|QGroundControl|uav1_fastlio_mission' \
    | grep -v grep \
    || true
)"

if [[ -n "$REMAINING" ]]; then
  echo "$REMAINING"
  echo "[WARN] Some related processes are still present."
else
  echo "[OK] No related processes remain."
fi

echo
echo "[OK] Previous integrated stack stopped."
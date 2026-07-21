#!/usr/bin/env bash
# =============================================================================
# restart_all.sh — 三无人机集成栈一键启动脚本（无需编译）
# =============================================================================
# 职责：
#   在已编译完成的工作空间基础上，依次启动完整的三无人机仿真+感知+控制栈：
#     QGroundControl → XRCE-DDS Agent → 三台 PX4 SITL → 
#     传感器桥接(ros_gz_bridge) → FAST-LIO(激光里程计) → RViz → 任务节点
#
# 与 install_and_restart.sh 的关系：
#   - install_and_restart.sh = 完整流程：配置生成 + colcon编译 + 调用本脚本
#   - restart_all.sh         = 快速重启：跳过编译，直接启动各组件
#
# 每个组件在独立 tmux 窗口中运行，stdout/stderr 分别记录到
# $RUN_DIR/<组件名>.log，方便事后排查。
#
# 前提条件：
#   1. 工作空间已完成 colcon build（否则缺少可执行文件）
#   2. PX4 SITL 已编译（$PX4_DIR/build/px4_sitl_default/bin/px4）
#   3. QGroundControl AppImage 已下载
# =============================================================================
set -eo pipefail

# ---- 路径与工作目录定义 ----
WS="${WS:-$HOME/consensus_ws}"                   # ROS2 工作空间
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"        # PX4 源码目录
TOOLS_DIR="$WS/tools/three_uav_integrated"        # 辅助脚本目录
SESSION="${SESSION:-map_consensus_integrated}"    # tmux 会话名
STAMP="$(date +%Y%m%d_%H%M%S)"                   # 本次运行时间戳
RUN_DIR="$WS/logs/integrated_stack/$STAMP"        # 本次日志输出目录
mkdir -p "$RUN_DIR"

# ---- 第一步：停止所有旧进程 ----
"$TOOLS_DIR/stop_all.sh"

# ---- 加载 ROS2 环境 ----
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# ---- 校验关键文件是否存在 ----
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"       # PX4 SITL 可执行文件
BRIDGE_CFG="$WS/config/three_uav_sensor_bridges.yaml"    # Gz↔ROS2 传感器桥接配置
RVIZ_CFG="$WS/config/uav1_fastlio.rviz"                  # RViz 可视化配置
for required in "$PX4_BIN" "$BRIDGE_CFG" "$RVIZ_CFG"; do
  [[ -e "$required" ]] || { echo "[ERROR] Missing: $required"; exit 1; }
done

# ---- 查找 XRCE-DDS Agent（PX4 与 ROS2 通信桥梁） ----
AGENT_BIN="$(command -v MicroXRCEAgent 2>/dev/null || command -v micro-xrce-dds-agent 2>/dev/null || true)"
[[ -n "$AGENT_BIN" ]] || { echo "[ERROR] XRCE-DDS Agent not found"; exit 1; }

# ---- 查找 QGroundControl 地面站 ----
QGC=""
for candidate in "$HOME/下载/QGroundControl-x86_64.AppImage" "$HOME/Downloads/QGroundControl-x86_64.AppImage"; do
  [[ -x "$candidate" ]] && { QGC="$candidate"; break; }
done
[[ -n "$QGC" ]] || { echo "[ERROR] QGroundControl AppImage not found"; exit 1; }

# ---- 辅助函数：为每个组件生成独立的启动脚本并写入日志 ----
# 将组件的启动命令包装成独立 .sh 文件，stdout+stderr 通过 tee 同时输出到终端和日志文件
make_component_script() {
  local name="$1"       # 组件名（用于 tmux 窗口名和日志文件名）
  local command="$2"    # 实际要执行的命令
  cat > "$RUN_DIR/$name.sh" <<SCRIPT
#!/usr/bin/env bash
set -o pipefail
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
$command 2>&1 | tee "$RUN_DIR/$name.log"
rc=\${PIPESTATUS[0]}
echo "[EXIT] component=$name status=\${rc}" | tee -a "$RUN_DIR/$name.log"
exec bash
SCRIPT
  chmod +x "$RUN_DIR/$name.sh"
}

# 在 tmux 会话中创建新窗口并运行组件脚本
start_component() {
  local name="$1"
  local command="$2"
  make_component_script "$name" "$command"
  tmux new-window -t "$SESSION" -n "$name" "bash '$RUN_DIR/$name.sh'"
}

# 轮询等待指定 ROS2 topic 出现，超时则报错退出
wait_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local started="$(date +%s)"
  while true; do
    ros2 topic list 2>/dev/null | grep -Fxq "$topic" && { echo "[OK] $topic"; return 0; }
    (( $(date +%s) - started < timeout_sec )) || { echo "[ERROR] Timeout: $topic"; return 1; }
    sleep 1
  done
}

# =========================================================================
# 创建 tmux 会话并依次启动所有组件
# =========================================================================
echo "[INFO] Run logs: $RUN_DIR"

# 创建后台 tmux 会话，设置 remain-on-exit 使窗口在进程退出后保留（便于查看日志）
tmux new-session -d -s "$SESSION" -n bootstrap "bash"
tmux set-option -t "$SESSION" remain-on-exit on >/dev/null
# 传递图形环境变量，确保 QGC、Gazebo、RViz 等 GUI 程序能正常显示
for var in DISPLAY XAUTHORITY WAYLAND_DISPLAY DBUS_SESSION_BUS_ADDRESS; do
  tmux set-environment -t "$SESSION" "$var" "${!var:-}"
done

# ---- 1. 启动 QGroundControl 地面站 ----
start_component qgc "'$QGC'"
sleep 2

# ---- 2. 启动 XRCE-DDS Agent（PX4↔ROS2 通信桥梁，UDP 端口 8888） ----
start_component agent "'$AGENT_BIN' udp4 -p 8888"
sleep 2

# ---- 3. 启动三台 PX4 SITL 实例 ----
# UAV1: 主实例（不设 PX4_GZ_STANDALONE），位置 (-6,-12,0)，先启动以创建 Gazebo 世界
# UAV2: 从实例，位置 (6,-12,0)
# UAV3: 从实例，位置 (0,-12,0)
# 使用 gz_x500_lidar_3d 模型，student_center 世界
start_component uav1 "cd '$PX4_DIR' && env PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='-6,-12,0,0,0,0' PX4_SIM_MODEL=gz_x500_lidar_3d PX4_GZ_WORLD=student_center PX4_UXRCE_DDS_NS=px4_1 '$PX4_BIN' -i 1"
sleep 12   # 等待 UAV1 初始化 Gazebo 世界
start_component uav2 "cd '$PX4_DIR' && env PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='6,-12,0,0,0,0' PX4_SIM_MODEL=gz_x500_lidar_3d PX4_UXRCE_DDS_NS=px4_2 '$PX4_BIN' -i 2"
sleep 4
start_component uav3 "cd '$PX4_DIR' && env PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='0,-12,0,0,0,0' PX4_SIM_MODEL=gz_x500_lidar_3d PX4_UXRCE_DDS_NS=px4_3 '$PX4_BIN' -i 3"

# 等待三台 PX4 的 vehicle_status 话题就绪（确认飞控已启动）
wait_topic /px4_1/fmu/out/vehicle_status_v1 60
wait_topic /px4_2/fmu/out/vehicle_status_v1 60
wait_topic /px4_3/fmu/out/vehicle_status_v1 60

# ---- 4. 启动 Gazebo↔ROS2 传感器桥接 ----
# 将 Gazebo 中的 LiDAR 点云和 IMU 数据转发到 ROS2 topic
start_component sensors "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:='$BRIDGE_CFG'"
for topic in \
  /uav1/sensors/lidar /uav1/sensors/imu \
  /uav2/sensors/lidar /uav2/sensors/imu \
  /uav3/sensors/lidar /uav3/sensors/imu; do
  wait_topic "$topic" 60
done

# ---- 5. 启动 FAST-LIO（三台无人机激光里程计） ----
start_component fastlio "ros2 launch fast_lio three_uav_fastlio.launch.py"
wait_topic /uav1/fast_lio/odometry 90
wait_topic /uav2/fast_lio/odometry 90
wait_topic /uav3/fast_lio/odometry 90

# ---- 6. 启动 RViz 可视化 ----
start_component rviz "rviz2 -d '$RVIZ_CFG' --ros-args -p use_sim_time:=true"
sleep 3

# ---- 7. 启动任务节点 ----
# 默认：三机解锁+起飞 → UAV1 4×4m 方形轨迹演示 → 悬停
# 如需仅解锁+悬停（跳过方形轨迹），编辑 uav1_fastlio_mission.py 中 SETTLE 阶段末尾：
#   将 self.set_phase("SQUARE") 改为 self.set_phase("FINAL_HOLD")
start_component mission "ros2 run multi_uav_offboard uav1_fastlio_mission --ros-args -p use_sim_time:=true"

# ---- 清理辅助窗口并输出摘要 ----
# 删除初始的 bootstrap 占位窗口
tmux kill-window -t "$SESSION:bootstrap" 2>/dev/null || true

echo "============================================================"
echo "[OK] Integrated stack started"
echo "[INFO] tmux attach -t $SESSION    # 进入 tmux 会话查看各组件运行状态"
echo "[INFO] $TOOLS_DIR/status.sh      # 检查各 topic 是否就绪"
echo "[INFO] Logs: $RUN_DIR            # 各组件日志存放目录"
echo "============================================================"
sleep 10
"$TOOLS_DIR/status.sh"

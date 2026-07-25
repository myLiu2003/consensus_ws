#!/usr/bin/env bash
# =============================================================================
# 三无人机地图共识系统一键启动脚本
#
# 启动顺序：
#   Gazebo世界
#   → Micro XRCE-DDS Agent
#   → 三台PX4 SITL
#   → 三机LiDAR/IMU桥接
#   → 三机FAST-LIO2
#   → 地图共识（TF + 关键帧 + Scan Context）
#   → 可选RViz/QGroundControl/飞行任务
#
# 每个组件运行在独立的tmux窗口中，因此SSH断开或电脑锁屏后，
# 服务器上的程序仍会继续运行。
# =============================================================================

# -E：函数内部错误也触发ERR行为
# -e：任一命令失败后停止脚本
# -u：使用未定义变量时报错
# pipefail：管道中任一命令失败，整条管道视为失败
set -Eeuo pipefail


# =============================================================================
# 1. 基础路径和启动选项
# =============================================================================

# ROS 2主工作空间。
# 所有地图共识相关包必须从这个工作空间加载，不能混入consensus_lcgo_ws。
WS="${WS:-$HOME/consensus_ws}"

# 一键启动、停止和状态检查脚本所在目录。
TOOLS_DIR="$WS/tools/three_uav_integrated"

# 所有组件所在的tmux会话名称。
SESSION="${SESSION:-map_consensus_integrated}"

# 是否启动QGroundControl。
# 远程SSH环境默认关闭，实验室本地需要时设为true。
START_QGC="${START_QGC:-false}"

# 是否启动RViz。
# 远程SSH环境默认关闭，实验室有图形界面时设为true。
START_RVIZ="${START_RVIZ:-false}"

# 是否启动自动飞行任务。
# 默认关闭，避免一键启动后无人机自动解锁起飞。
START_MISSION="${START_MISSION:-false}"

# 是否以无图形界面方式启动Gazebo。
# 远程服务器默认为true；实验室本地显示Gazebo时设为false。
HEADLESS="${HEADLESS:-true}"

# 每次启动创建独立日志目录，避免覆盖之前的运行记录。
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$WS/logs/integrated_stack/$STAMP"
mkdir -p "$RUN_DIR"


# =============================================================================
# 2. 停止上一次运行的系统
# =============================================================================

# 先调用统一停止脚本，防止旧PX4、桥接、FAST-LIO和共识节点残留。
# 如果旧进程没有清理，可能出现：
#   - 同名ROS节点
#   - 重复话题发布者
#   - UDP 8888端口占用
#   - Gazebo模型重复
echo "[INFO] Stopping previous stack..."
"$TOOLS_DIR/stop_all.sh"


# =============================================================================
# 3. 加载ROS 2环境
# =============================================================================

# ROS 2的setup.bash可能访问未定义环境变量，
# 因此加载环境时暂时关闭set -u。
set +u

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# ROS 2环境加载完成后重新开启未定义变量检查。
set -u


# =============================================================================
# 4. 检查关键ROS包的来源
# =============================================================================

# 这些包是一键启动所必需的。
required_packages=(
  student_center_sim
  fast_lio
  keyframe_frontend
  loop_consensus
  multi_uav_tf_manager
  map_consensus_msgs
)

# 除了检查包是否存在，还检查包是否来自~/consensus_ws。
# 该检查可以防止再次混入~/consensus_lcgo_ws中的同名包。
for package in "${required_packages[@]}"; do
  prefix="$(ros2 pkg prefix "$package" 2>/dev/null || true)"

  if [[ -z "$prefix" ]]; then
    echo "[ERROR] ROS package not found: $package"
    echo "[ERROR] Please build the workspace first:"
    echo "        cd $WS"
    echo "        colcon build --symlink-install"
    exit 1
  fi

  if [[ "$prefix" != "$WS/install/"* ]]; then
    echo "[ERROR] Package comes from the wrong workspace:"
    echo "        package  = $package"
    echo "        prefix   = $prefix"
    echo "        expected = $WS/install/..."
    exit 1
  fi

  echo "[OK] package $package -> $prefix"
done


# =============================================================================
# 5. tmux组件启动辅助函数
# =============================================================================

# 为每个组件生成一个独立的临时启动脚本。
#
# 例如，world组件会生成：
#   $RUN_DIR/world.sh
#
# 组件输出同时：
#   1. 显示在tmux窗口中
#   2. 保存到$RUN_DIR/world.log
#
# 组件退出后执行exec bash，使tmux窗口继续保留，
# 方便进入窗口查看退出原因和历史日志。
make_component_script() {
  local name="$1"
  local command="$2"

  cat > "$RUN_DIR/$name.sh" <<SCRIPT
#!/usr/bin/env bash
set -o pipefail

# 每个tmux窗口都独立加载正确工作空间，
# 防止继承其他终端中consensus_lcgo_ws的环境。
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "[START] component=$name"
echo "[START] time=\$(date --iso-8601=seconds)"
echo "[START] command=$command"

# tee同时将输出显示在终端并保存到日志。
$command 2>&1 | tee "$RUN_DIR/$name.log"

# PIPESTATUS[0]表示实际组件命令的退出状态，
# 而不是tee命令的退出状态。
rc=\${PIPESTATUS[0]}

echo "[EXIT] component=$name status=\${rc}" \
  | tee -a "$RUN_DIR/$name.log"

# 保留tmux窗口，便于检查日志。
exec bash
SCRIPT

  chmod +x "$RUN_DIR/$name.sh"
}


# 在当前tmux会话中新建窗口并启动指定组件。
#
# 参数1：tmux窗口名称
# 参数2：实际启动命令
start_component() {
  local name="$1"
  local command="$2"

  make_component_script "$name" "$command"

  tmux new-window \
    -t "$SESSION" \
    -n "$name" \
    "bash '$RUN_DIR/$name.sh'"
}


# =============================================================================
# 6. 数据就绪检查函数
# =============================================================================

# 等待ROS话题收到至少一条真实消息。
#
# 这里只检查“话题名称存在”是不够的，因为桥接进程可能已经创建发布者，
# 但Gazebo传感器实际没有输出数据。
#
# 参数1：ROS话题名称
# 参数2：最大等待时间，单位秒
wait_topic_data() {
  local topic="$1"
  local timeout_sec="$2"
  local started

  started="$(date +%s)"
  echo "[WAIT] ROS data: $topic"

  while true; do
   # 使用best_effort订阅，兼容FAST-LIO可能使用的传感器数据QoS。
# best_effort订阅可以接收reliable发布者，也可以接收best_effort发布者。
    if timeout 5 ros2 topic echo "$topic" \
        --once \
        --qos-reliability best_effort \
        >/dev/null 2>&1; then
      echo "[OK] $topic"
      return 0
    fi

    # 超过总等待时间则停止一键启动，并明确指出缺失话题。
    if (( $(date +%s) - started >= timeout_sec )); then
      echo "[ERROR] Timeout waiting for data: $topic"
      return 1
    fi

    sleep 1
  done
}


# 等待Gazebo student_center世界服务出现。
#
# 三架PX4必须在Gazebo世界启动完成后再生成，
# 否则PX4可能无法找到world或模型资源。
wait_gz_world() {
  local timeout_sec="$1"
  local started

  started="$(date +%s)"
  echo "[WAIT] Gazebo student_center world"

  while true; do
    if gz service -l 2>/dev/null \
        | grep -q "/world/student_center/control"; then
      echo "[OK] Gazebo student_center world"
      return 0
    fi

    if (( $(date +%s) - started >= timeout_sec )); then
      echo "[ERROR] Timeout waiting for Gazebo world"
      return 1
    fi

    sleep 1
  done
}


# =============================================================================
# 7. 创建后台tmux会话
# =============================================================================

echo "[INFO] Run logs: $RUN_DIR"
echo "[INFO] tmux session: $SESSION"

# bootstrap是临时占位窗口。
# 后续组件窗口全部创建完成后会删除该窗口。
tmux new-session \
  -d \
  -s "$SESSION" \
  -n bootstrap \
  "bash"

# 即使组件退出，也保留对应tmux窗口，方便排错。
tmux set-option \
  -t "$SESSION" \
  remain-on-exit on \
  >/dev/null

# 将图形界面和ROS Domain环境传入tmux。
# START_RVIZ或START_QGC为true时需要图形环境变量。
for var in \
  DISPLAY \
  XAUTHORITY \
  WAYLAND_DISPLAY \
  DBUS_SESSION_BUS_ADDRESS \
  ROS_DOMAIN_ID
do
  tmux set-environment \
    -t "$SESSION" \
    "$var" \
    "${!var:-}"
done


# =============================================================================
# 8. 启动Gazebo student_center世界
# =============================================================================

# 必须首先启动世界。
#
# student_center.sdf中已经显式加载：
#   - Physics系统
#   - UserCommands系统
#   - SceneBroadcaster系统
#   - Sensors系统（负责GPU LiDAR）
#   - Imu系统（负责IMU数据）
#
# HEADLESS=true：
#   无Gazebo窗口，适合远程SSH服务器。
#
# HEADLESS=false：
#   显示Gazebo窗口，适合实验室本地查看。
start_component world \
  "ros2 launch student_center_sim student_center_world.launch.py headless:=$HEADLESS"

# 等待Gazebo世界真正启动，而不是使用固定sleep时间。
wait_gz_world 60


# =============================================================================
# 9. 启动Micro XRCE-DDS Agent
# =============================================================================

# Agent负责PX4 uXRCE-DDS客户端与ROS 2之间的通信，
# 默认监听UDP端口8888。
start_component agent \
  "ros2 run student_center_sim start_agent.sh"

# 给Agent少量初始化时间。
sleep 2


# =============================================================================
# 10. 启动三台PX4 SITL
# =============================================================================

# start_three_px4.sh完成以下工作：
#   1. 检查student_center世界是否存在
#   2. 设置Gazebo模型搜索路径
#   3. 启动PX4实例1、2、3
#   4. 设置三架无人机的初始位置
#
# 必须使用当前已经验证成功的模型：
#   consensus_x500_lidar_3d
#
# Gazebo生成的实际实体名称为：
#   consensus_x500_lidar_3d_1
#   consensus_x500_lidar_3d_2
#   consensus_x500_lidar_3d_3
start_component px4 \
  "ros2 run student_center_sim start_three_px4.sh consensus_x500_lidar_3d"

# 等待三台PX4的状态话题收到真实数据，
# 确认飞控与ROS 2通信正常。
wait_topic_data /px4_1/fmu/out/vehicle_status_v1 90
wait_topic_data /px4_2/fmu/out/vehicle_status_v1 90
wait_topic_data /px4_3/fmu/out/vehicle_status_v1 90


# =============================================================================
# 11. 启动三机传感器快速桥接
# =============================================================================

# auto_bridge_sensors.sh采用固定Gazebo话题路径，不再遍历所有话题，
# 因此启动时间由原来的几十秒缩短到数秒。
#
# 该脚本启动：
#   1个/clock桥接
#   3个LiDAR桥接
#   3个IMU桥接
#
# 预期频率：
#   LiDAR约10 Hz
#   IMU约250 Hz
start_component sensors \
  "ros2 run student_center_sim auto_bridge_sensors.sh"

# 等待六个传感器话题收到真实数据。
for topic in \
  /uav1/sensors/lidar \
  /uav1/sensors/imu \
  /uav2/sensors/lidar \
  /uav2/sensors/imu \
  /uav3/sensors/lidar \
  /uav3/sensors/imu
do
  wait_topic_data "$topic" 60
done


# =============================================================================
# 12. 启动三机FAST-LIO2
# =============================================================================

# FAST-LIO必须在LiDAR和IMU均有数据后启动。
# three_uav_fastlio.launch.py会分别启动三套FAST-LIO。
start_component fastlio \
  "ros2 launch fast_lio three_uav_fastlio.launch.py"

# 等待三台FAST-LIO里程计收到真实数据。
# 正常频率约为10 Hz。
wait_topic_data /uav1/fast_lio/odometry 120
wait_topic_data /uav2/fast_lio/odometry 120
wait_topic_data /uav3/fast_lio/odometry 120


# =============================================================================
# 13. 启动地图共识系统
# =============================================================================

# multi_uav_consensus.launch.py统一启动：
#   1. multi_uav_tf_manager
#   2. keyframe_frontend_uav1
#   3. keyframe_frontend_uav2
#   4. keyframe_frontend_uav3
#   5. loop_consensus_node
#
# start_fastlio=false：
#   FAST-LIO已经在上一步启动，避免重复启动。
#
# start_rviz由START_RVIZ控制：
#   远程SSH默认为false
#   实验室本地可设为true
start_component consensus \
  "ros2 launch loop_consensus multi_uav_consensus.launch.py start_fastlio:=false start_rviz:=$START_RVIZ"

# 等待三架无人机的关键帧数据。
# 当前关键帧频率约为0.8 Hz，因此等待时间设置为60秒。
wait_topic_data /uav1/consensus/keyframe 60
wait_topic_data /uav2/consensus/keyframe 60
wait_topic_data /uav3/consensus/keyframe 60

# 等待Scan Context输出至少一个候选。
# 候选生成需要积累一定数量的关键帧，因此等待时间比普通话题更长。
wait_topic_data /map_consensus/loop_candidates 90


# =============================================================================
# 14. 按需启动QGroundControl
# =============================================================================

# QGroundControl默认关闭。
# 使用START_QGC=true运行脚本时才会进入该分支。
if [[ "$START_QGC" == "true" ]]; then
  QGC=""

  # 同时兼容中文“下载”目录和英文Downloads目录。
  for candidate in \
    "$HOME/下载/QGroundControl-x86_64.AppImage" \
    "$HOME/Downloads/QGroundControl-x86_64.AppImage"
  do
    if [[ -x "$candidate" ]]; then
      QGC="$candidate"
      break
    fi
  done

  if [[ -n "$QGC" ]]; then
    start_component qgc "'$QGC'"
  else
    # QGC缺失不影响地图共识主链路，因此这里只警告，不停止系统。
    echo "[WARN] QGroundControl not found; skipping"
  fi
fi


# =============================================================================
# 15. 按需启动飞行任务
# =============================================================================

# 自动飞行任务默认关闭。
# 只有明确设置START_MISSION=true时才自动解锁和执行任务。
if [[ "$START_MISSION" == "true" ]]; then
  start_component mission \
    "ros2 run multi_uav_offboard uav1_fastlio_mission --ros-args -p use_sim_time:=true"
fi


# =============================================================================
# 16. 清理占位窗口并输出启动摘要
# =============================================================================

# 所有正式组件窗口均已建立，可以删除bootstrap占位窗口。
tmux kill-window \
  -t "$SESSION:bootstrap" \
  2>/dev/null || true

echo
echo "============================================================"
echo "[OK] Integrated stack started"
echo "[INFO] Session: $SESSION"
echo "[INFO] Logs: $RUN_DIR"
echo
echo "[INFO] 查看后台组件："
echo "       tmux attach -t $SESSION"
echo
echo "[INFO] 退出tmux但保持程序运行："
echo "       Ctrl+B，然后按D"
echo
echo "[INFO] 检查系统状态："
echo "       $TOOLS_DIR/status.sh"
echo
echo "[INFO] 停止整个系统："
echo "       $TOOLS_DIR/stop_all.sh"
echo "============================================================"

# 启动完成后自动执行一次状态检查。
"$TOOLS_DIR/status.sh"
#!/usr/bin/env bash
set -Eeuo pipefail

MODEL="${1:-gz_x500}"

PX4_ROOT="${HOME}/PX4-Autopilot"
PX4_BIN="${PX4_ROOT}/build/px4_sitl_default/bin/px4"
LOG_DIR="${HOME}/logs/student_center"
PID_FILE="${LOG_DIR}/px4_pids.txt"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${HOME}/consensus_ws/install/setup.bash" ]]; then
    source "${HOME}/consensus_ws/install/setup.bash"
fi
set -u

PKG_PREFIX="$(ros2 pkg prefix student_center_sim)"
PKG_SHARE="${PKG_PREFIX}/share/student_center_sim"

PX4_MODELS_DIR="$(
    find "${PX4_ROOT}/Tools" \
        -type d \
        -path "*/simulation/gz/models" \
        -print -quit
)"

PX4_WORLDS_DIR="$(
    find "${PX4_ROOT}/Tools" \
        -type d \
        -path "*/simulation/gz/worlds" \
        -print -quit
)"

export GZ_SIM_RESOURCE_PATH="$(
    printf '%s:%s:%s:%s:%s' \
        "${PKG_SHARE}/models" \
        "${PKG_SHARE}/worlds" \
        "${PX4_MODELS_DIR}" \
        "${PX4_WORLDS_DIR}" \
        "${GZ_SIM_RESOURCE_PATH:-}"
)"

[[ -x "$PX4_BIN" ]] || {
    echo "PX4 SITL 二进制不存在：$PX4_BIN" >&2
    exit 1
}

mkdir -p "$LOG_DIR"
: > "$PID_FILE"

echo "等待 student_center Gazebo world 启动……"

WORLD_READY=0
for _ in $(seq 1 30); do
    if gz service -l 2>/dev/null | grep -q "/world/student_center/control"; then
        WORLD_READY=1
        break
    fi
    sleep 1
done

if [[ "$WORLD_READY" -ne 1 ]]; then
    echo "未检测到 student_center world。" >&2
    echo "请先运行：ros2 launch student_center_sim student_center_world.launch.py" >&2
    exit 1
fi

start_px4() {
    local instance="$1"
    local pose="$2"
    local log_file="${LOG_DIR}/px4_${instance}.log"

    echo "启动 PX4 实例 ${instance}"
    echo "模型：${MODEL}"
    echo "初始位姿：${pose}"

    (
        cd "$PX4_ROOT"

        nohup env \
            PX4_GZ_STANDALONE=1 \
            PX4_SYS_AUTOSTART=4001 \
            PX4_SIM_MODEL="$MODEL" \
            PX4_GZ_MODEL_POSE="$pose" \
            "$PX4_BIN" -i "$instance" \
            > "$log_file" 2>&1 &

        echo "$!" >> "$PID_FILE"
    )

    sleep 4
}

start_px4 1 "-2.0,-12.5,0.4,0,0,0"
start_px4 2 "2.0,-12.5,0.4,0,0,0"
start_px4 3 "0.0,-10.8,0.4,0,0,0"

echo
echo "三架 PX4 已启动。"
echo "模型：${MODEL}"
echo "日志目录：${LOG_DIR}"

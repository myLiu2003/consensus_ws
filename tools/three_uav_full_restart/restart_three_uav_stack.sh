#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="${SESSION_NAME:-map_consensus_3uav}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
MODEL="${MODEL:-gz_x500_lidar_3d}"
WORLD="${WORLD:-student_center}"

QGC_BIN="${QGC_BIN:-$(command -v QGroundControl 2>/dev/null || true)}"
if [[ -z "$QGC_BIN" ]]; then
  QGC_BIN="$(find "$HOME" /opt -maxdepth 5 -type f \( -iname 'QGroundControl*.AppImage' -o -iname 'QGroundControl' \) -print -quit 2>/dev/null)"
fi

command -v tmux >/dev/null
command -v micro-xrce-dds-agent >/dev/null
[[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]
[[ -n "$QGC_BIN" ]]
[[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]

chmod +x "$QGC_BIN" 2>/dev/null || true

tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
pkill -x px4 2>/dev/null || true
pkill -f '[g]z sim' 2>/dev/null || true
pkill -f '[Q]GroundControl' 2>/dev/null || true
pkill -x micro-xrce-dds-agent 2>/dev/null || true
pkill -x MicroXRCEAgent 2>/dev/null || true
sleep 3

tmux new-session -d -s "$SESSION_NAME" -n qgc \
  "bash -lc 'exec \"${QGC_BIN}\"'"

sleep 3

tmux new-window -t "$SESSION_NAME" -n agent \
  "bash -lc 'source /opt/ros/humble/setup.bash; exec micro-xrce-dds-agent udp4 -p 8888'"

sleep 2

tmux new-window -t "$SESSION_NAME" -n uav1 \
  "bash -lc 'cd \"${PX4_DIR}\"; PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"-6,0,0,0,0,0\" PX4_SIM_MODEL=\"${MODEL}\" PX4_GZ_WORLD=\"${WORLD}\" PX4_UXRCE_DDS_NS=px4_1 exec ./build/px4_sitl_default/bin/px4 -i 1'"

sleep 12

tmux new-window -t "$SESSION_NAME" -n uav2 \
  "bash -lc 'cd \"${PX4_DIR}\"; PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"6,0,0,0,0,0\" PX4_SIM_MODEL=\"${MODEL}\" PX4_UXRCE_DDS_NS=px4_2 exec ./build/px4_sitl_default/bin/px4 -i 2'"

sleep 5

tmux new-window -t "$SESSION_NAME" -n uav3 \
  "bash -lc 'cd \"${PX4_DIR}\"; PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"0,6,0,0,0,0\" PX4_SIM_MODEL=\"${MODEL}\" PX4_UXRCE_DDS_NS=px4_3 exec ./build/px4_sitl_default/bin/px4 -i 3'"

echo "[OK] QGC、Agent、Gazebo和三套PX4实例已启动"
echo "查看：tmux attach -t $SESSION_NAME"

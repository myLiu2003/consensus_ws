#!/usr/bin/env bash
set -euo pipefail
SESSION_NAME="${SESSION_NAME:-map_consensus_3uav}"
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
pkill -f '[m]ulti_uav_offboard.*three_uav_waypoints' 2>/dev/null || true
pkill -x px4 2>/dev/null || true
pkill -f '[g]z sim' 2>/dev/null || true
pkill -f '[Q]GroundControl' 2>/dev/null || true
pkill -x micro-xrce-dds-agent 2>/dev/null || true
pkill -x MicroXRCEAgent 2>/dev/null || true
echo "[OK] 全部停止"

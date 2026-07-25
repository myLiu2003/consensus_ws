#!/usr/bin/env bash
set -eo pipefail

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
MONITOR_INTERVAL="${MONITOR_INTERVAL:-2}"
RUN_ROOT="$WS/logs/integrated_stack"
RUN_DIR="${1:-$(find "$RUN_ROOT" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort | tail -n 1)}"
[[ -n "${RUN_DIR:-}" ]] || { echo "[ERROR] No integrated_stack run directory found"; exit 1; }
[[ -d "$RUN_DIR" ]] || { echo "[ERROR] Run directory does not exist: $RUN_DIR"; exit 1; }

MAP_LOG="$RUN_DIR/map_consensus.log"
if [[ ! -f "$MAP_LOG" && -f "$RUN_DIR/loop_consensus.log" ]]; then
  MAP_LOG="$RUN_DIR/loop_consensus.log"
fi
MISSION_LOG="$RUN_DIR/mission.log"
[[ -f "$MAP_LOG" ]] || { echo "[ERROR] map consensus log not found in $RUN_DIR"; exit 1; }

STAMP="$(date +%Y%m%d_%H%M%S)"
MONITOR_DIR="$WS/logs/consensus_monitor/$STAMP"
MONITOR_LOG="$MONITOR_DIR/consensus_monitor.log"
mkdir -p "$MONITOR_DIR"
ln -sfn "$MONITOR_DIR" "$WS/logs/consensus_monitor/latest"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

log_line() {
  local line="$1"
  printf '%s\n' "$line" | tee -a "$MONITOR_LOG"
}

topic_state() {
  local topic_name="$1"
  if ros2 topic list 2>/dev/null | grep -Fxq "$topic_name"; then
    printf 'OK'
  else
    printf 'MISS'
  fi
}

service_state() {
  local service_name="$1"
  if ros2 service list 2>/dev/null | grep -Fxq "$service_name"; then
    printf 'OK'
  else
    printf 'MISS'
  fi
}

node_state() {
  local node_name="$1"
  if ros2 node list 2>/dev/null | grep -Fxq "$node_name"; then
    printf 'OK'
  else
    printf 'MISS'
  fi
}

emit_snapshot() {
  local snapshot
  snapshot=$(
    printf '[SNAPSHOT] nodes(tf=%s loop=%s gicp=%s kf1=%s kf2=%s kf3=%s) topics(kf1=%s kf2=%s kf3=%s cand=%s marker=%s verify=%s accept=%s reject=%s) services(s1=%s s2=%s s3=%s)' \
      "$(node_state /multi_uav_tf_manager)" \
      "$(node_state /loop_consensus_node)" \
      "$(node_state /gicp_verifier_node)" \
      "$(node_state /keyframe_frontend_uav1)" \
      "$(node_state /keyframe_frontend_uav2)" \
      "$(node_state /keyframe_frontend_uav3)" \
      "$(topic_state /uav1/consensus/keyframe)" \
      "$(topic_state /uav2/consensus/keyframe)" \
      "$(topic_state /uav3/consensus/keyframe)" \
      "$(topic_state /map_consensus/loop_candidates)" \
      "$(topic_state /map_consensus/loop_candidate_markers)" \
      "$(topic_state /map_consensus/loop_verifications)" \
      "$(topic_state /map_consensus/accepted_loops)" \
      "$(topic_state /map_consensus/rejected_loops)" \
      "$(service_state /uav1/consensus/get_submap)" \
      "$(service_state /uav2/consensus/get_submap)" \
      "$(service_state /uav3/consensus/get_submap)"
  )

  if [[ "$snapshot" != "$LAST_SNAPSHOT" ]]; then
    log_line "$(date '+[%F %T]') $snapshot"
    LAST_SNAPSHOT="$snapshot"
  fi
}

emit_new_lines() {
  local source_name="$1"
  local file_path="$2"
  local start_line="$3"
  local end_line="$4"

  [[ -f "$file_path" ]] || return 0
  (( end_line >= start_line )) || return 0

  while IFS= read -r line; do
    [[ -n "$line" ]] || continue
    case "$source_name" in
      MAP)
        if [[ "$line" =~ loop_consensus\ started|gicp_verifier\ started|SC\ stored:|Polling\ round|INTER\ query:|INTRA\ query:|No\ eligible\ candidate:|Cannot\ transform|Built\ on-demand\ submap|\[GICP_START\]|\[GICP_SUBMAP\]|\[GICP_RESULT\]|\[CHECK_RESULT\] ]]; then
          log_line "$(date '+[%F %T]') [MAP] $line"
        fi
        ;;
      MISSION)
        if [[ "$line" =~ Mission\ started|MISSION\ PHASE|Sent\ OFFBOARD|Captured\ local\ origins ]]; then
          log_line "$(date '+[%F %T]') [MISSION] $line"
        fi
        ;;
    esac
  done < <(sed -n "${start_line},${end_line}p" "$file_path")
}

LAST_SNAPSHOT=""
MAP_LAST_LINE=0
MISSION_LAST_LINE=0
LOOP_COUNTER=0

log_line "============================================================"
log_line "[INFO] monitor started"
log_line "[INFO] run_dir=$RUN_DIR"
log_line "[INFO] map_log=$MAP_LOG"
if [[ -f "$MISSION_LOG" ]]; then
  log_line "[INFO] mission_log=$MISSION_LOG"
else
  log_line "[WARN] mission.log not found in $RUN_DIR"
fi
log_line "[INFO] monitor_log=$MONITOR_LOG"
log_line "============================================================"

trap 'log_line "[INFO] monitor stopped"; exit 0' INT TERM

while true; do
  emit_snapshot

  MAP_LINES="$(wc -l < "$MAP_LOG")"
  if (( MAP_LINES < MAP_LAST_LINE )); then
    MAP_LAST_LINE=0
  fi
  if (( MAP_LINES > MAP_LAST_LINE )); then
    emit_new_lines "MAP" "$MAP_LOG" "$((MAP_LAST_LINE + 1))" "$MAP_LINES"
    MAP_LAST_LINE="$MAP_LINES"
  fi

  if [[ -f "$MISSION_LOG" ]]; then
    MISSION_LINES="$(wc -l < "$MISSION_LOG")"
    if (( MISSION_LINES < MISSION_LAST_LINE )); then
      MISSION_LAST_LINE=0
    fi
    if (( MISSION_LINES > MISSION_LAST_LINE )); then
      emit_new_lines "MISSION" "$MISSION_LOG" "$((MISSION_LAST_LINE + 1))" "$MISSION_LINES"
      MISSION_LAST_LINE="$MISSION_LINES"
    fi
  fi

  LOOP_COUNTER=$((LOOP_COUNTER + 1))
  if (( LOOP_COUNTER % 10 == 0 )); then
    log_line "$(date '+[%F %T]') [INFO] monitor heartbeat"
  fi

  sleep "$MONITOR_INTERVAL"
done

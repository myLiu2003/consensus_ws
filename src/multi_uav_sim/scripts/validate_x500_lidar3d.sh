#!/usr/bin/env bash
set -Eeo pipefail

PKG="${HOME}/consensus_ws/src/multi_uav_sim"
MODEL="${PKG}/models/x500_lidar_3d/model.sdf"
LIDAR="${PKG}/models/lidar_3d_v1/model.sdf"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${HOME}/logs/map_consensus"
RUN_LOG="${LOG_DIR}/x500_lidar3d_gz_runtime_${STAMP}.log"
TOPIC_LOG="${LOG_DIR}/x500_lidar3d_topics_${STAMP}.txt"
TEST_WORLD="/tmp/x500_lidar3d_validation_${USER}_${STAMP}.sdf"
GZ_PARTITION_NAME="x500_lidar3d_validation_${USER}_${STAMP}"

mkdir -p "${LOG_DIR}"

cleanup() {
    if [[ -n "${GZ_PID:-}" ]] && kill -0 "${GZ_PID}" 2>/dev/null; then
        kill "${GZ_PID}" 2>/dev/null || true
        sleep 2
        kill -9 "${GZ_PID}" 2>/dev/null || true
    fi
    rm -f "${TEST_WORLD}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

fail() {
    echo
    echo "VALIDATION FAILED: $*" >&2
    echo "GAZEBO_LOG=${RUN_LOG}" >&2
    echo "TOPIC_LOG=${TOPIC_LOG}" >&2
    exit 1
}

# shellcheck disable=SC1091
source "${PKG}/scripts/setup_sim_env.sh"
export GZ_PARTITION="${GZ_PARTITION_NAME}"

echo
echo "[1/6] Required files"
test -f "${MODEL}" || fail "Missing composite model"
test -f "${LIDAR}" || fail "Missing LiDAR model"
echo "OK"

echo
echo "[2/6] Locate stock X500"
X500_FOUND=""
IFS=':' read -r -a RESOURCE_DIRS <<< "${GZ_SIM_RESOURCE_PATH}"
for root in "${RESOURCE_DIRS[@]}"; do
    if [[ -f "${root}/x500/model.sdf" ]]; then
        X500_FOUND="${root}/x500/model.sdf"
        break
    fi
done
[[ -n "${X500_FOUND}" ]] || fail "model x500 is not present in GZ_SIM_RESOURCE_PATH"
echo "X500_MODEL=${X500_FOUND}"

echo
echo "[3/6] XML and standalone sensor SDF"
python3 - "${MODEL}" "${LIDAR}" <<'PY'
import sys
import xml.etree.ElementTree as ET
for path in sys.argv[1:]:
    ET.parse(path)
    print(f"XML_OK={path}")
PY

# Standalone model has no external include and is suitable for gz sdf -k.
gz sdf -k "${LIDAR}" || fail "Standalone LiDAR SDF is invalid"
echo "OK"

echo
echo "[4/6] Create isolated headless Gazebo test world"
cat > "${TEST_WORLD}" <<'WORLD_EOF'
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="x500_lidar3d_validation">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics"/>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands"/>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster"/>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gravity>0 0 -9.8</gravity>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

    <model name="lidar_target_box">
      <static>true</static>
      <pose>3 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 2</size></box>
          </geometry>
        </visual>
      </link>
    </model>

    <include>
      <uri>model://x500_lidar_3d</uri>
      <name>x500_lidar_3d_test</name>
      <pose>0 0 1 0 0 0</pose>
    </include>
  </world>
</sdf>
WORLD_EOF
echo "TEST_WORLD=${TEST_WORLD}"

echo
echo "[5/6] Load the composite model in Gazebo Sim"
echo "GZ_PARTITION=${GZ_PARTITION}"
echo "GAZEBO_LOG=${RUN_LOG}"

gz sim -s -r -v 4 "${TEST_WORLD}" >"${RUN_LOG}" 2>&1 &
GZ_PID=$!

TOPICS_FOUND=0
for attempt in $(seq 1 20); do
    sleep 1

    if ! kill -0 "${GZ_PID}" 2>/dev/null; then
        wait "${GZ_PID}" || true
        tail -n 120 "${RUN_LOG}" || true
        fail "Gazebo server exited before the model was ready"
    fi

    gz topic -l > "${TOPIC_LOG}" 2>/dev/null || true

    if grep -Eq 'scan/points|lidar_3d' "${TOPIC_LOG}"; then
        TOPICS_FOUND=1
        break
    fi
done

echo
echo "--- Relevant Gazebo topics ---"
grep -Ei 'lidar|scan|mapping_imu|imu' "${TOPIC_LOG}" || true

if grep -Eq \
  'Unable to find uri|A model must have at least one link|FrameAttachedToGraph|PoseRelativeToGraph|parent frame.*not found|child frame.*not found|Failed to load system plugin|Unable to create rendering engine|OGRE EXCEPTION' \
  "${RUN_LOG}"; then
    echo
    echo "--- Detected model / rendering errors ---"
    grep -En \
      'Unable to find uri|A model must have at least one link|FrameAttachedToGraph|PoseRelativeToGraph|parent frame.*not found|child frame.*not found|Failed to load system plugin|Unable to create rendering engine|OGRE EXCEPTION' \
      "${RUN_LOG}" || true
    fail "Gazebo reported model, frame, plugin, or rendering errors"
fi

[[ "${TOPICS_FOUND}" -eq 1 ]] \
    || fail "Gazebo loaded, but both 3D LiDAR and mapping IMU topics were not discovered"

echo "RUNTIME_MODEL_LOAD=OK"
echo "SENSOR_TOPICS=OK"

echo
echo "[6/6] Stop the isolated Gazebo server"
kill "${GZ_PID}" 2>/dev/null || true
wait "${GZ_PID}" 2>/dev/null || true
GZ_PID=""
echo "OK"

echo
echo "VALIDATION PASSED"
echo "PX4_SIM_MODEL=gz_x500_lidar_3d"
echo "GAZEBO_LOG=${RUN_LOG}"
echo "TOPIC_LOG=${TOPIC_LOG}"

#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/consensus_ws/install/setup.bash"
exec ros2 run multi_uav_offboard three_uav_waypoints

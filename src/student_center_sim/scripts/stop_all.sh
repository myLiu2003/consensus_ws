#!/usr/bin/env bash
set +e

echo "停止 ros_gz_bridge……"
pkill -f "ros_gz_bridge.*parameter_bridge"

echo "停止 PX4 SITL……"
pkill -f "build/px4_sitl_default/bin/px4"

echo "停止 Micro XRCE-DDS Agent……"
pkill -f "micro-xrce-dds-agent"
pkill -f "MicroXRCEAgent"

echo "停止 Gazebo Sim……"
pkill -f "gz sim"
pkill -f "ruby.*gz_sim"

sleep 2
echo "清理完成。"

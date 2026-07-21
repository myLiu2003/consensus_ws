# keyframe_frontend

`keyframe_frontend` 负责订阅三机 `FAST-LIO` 的局部里程计与注册点云，按阈值选取关键帧，并发布轻量化局部子图供后续回环与图优化模块使用。

## 功能概览

- 订阅 `/<uav>/fast_lio/odometry` 与 `/<uav>/fast_lio/cloud_registered`
- 按距离、偏航角、时间三类条件联合触发关键帧
- 为每个关键帧保存预处理后的单帧点云
- 以当前关键帧为中心，构造 `±3` 帧局部子图
- 发布关键帧消息、关键帧轨迹、局部子图和关键帧 marker

## 主要输出

- `/<uav>/consensus/keyframe`
  - 自定义 `Keyframe.msg`，包含关键帧位姿和单帧点云
- `/<uav>/consensus/keyframe_path`
  - `nav_msgs/Path`，显示关键帧轨迹
- `/<uav>/consensus/submap_cloud`
  - `sensor_msgs/PointCloud2`，显示当前关键帧对应的局部子图
- `/<uav>/consensus/keyframe_markers`
  - `visualization_msgs/Marker`，以红色球点显示所有关键帧位置

## 启动方法

先确保基础系统已经启动，例如：

```bash
source /opt/ros/humble/setup.bash
cd ~/consensus_ws
source install/setup.bash
./tools/three_uav_integrated/restart_all.sh
```

再单独启动关键帧前端：

```bash
source /opt/ros/humble/setup.bash
cd ~/consensus_ws
source install/setup.bash
ros2 launch keyframe_frontend keyframe_frontend.launch.py
```

如果希望直接带 RViz 一起启动，并自动显示关键帧相关话题：

```bash
source /opt/ros/humble/setup.bash
cd ~/consensus_ws
source install/setup.bash
ros2 launch keyframe_frontend keyframe_frontend.launch.py start_rviz:=true
```

默认 RViz 配置文件位于：

```text
src/keyframe_frontend/rviz/keyframe_frontend_uav1.rviz
```

也可以手动打开：

```bash
rviz2 -d ~/consensus_ws/install/keyframe_frontend/share/keyframe_frontend/rviz/keyframe_frontend_uav1.rviz
```

## 参数在哪里改

默认参数文件：

```text
src/keyframe_frontend/config/keyframe_frontend.yaml
```

重点参数如下：

- 关键帧触发
  - `d_thresh`
  - `yaw_thresh_deg`
  - `t_max`
  - `t_min`
  - `d_min`
  - `yaw_min_deg`
- 局部子图
  - `submap_window`
  - `range_min`
  - `range_max`
  - `body_exclusion_radius`
  - `voxel_size`
  - `max_points`
- 可视化
  - `keyframe_marker_topic`
  - `keyframe_marker_scale`

## 修改参数后的编译

如果只改 `yaml`，通常不需要重新编译，重新 launch 即可。

如果改了 C++ 源码，则重新编译：

```bash
source /opt/ros/humble/setup.bash
cd ~/consensus_ws
colcon build --symlink-install --packages-select map_consensus_msgs keyframe_frontend --event-handlers console_direct+
source install/setup.bash
```

## 当前实现说明

- `keyframe_markers` 目前发布为 `SPHERE_LIST`
- 每个红色球表示一个关键帧位置
- `submap_cloud` 表示当前关键帧的轻量局部子图，不是单个关键帧位置点

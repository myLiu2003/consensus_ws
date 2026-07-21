# Swarm-LIO2 ROS2 Port

ROS2 Humble port of [HKU-MaRS Swarm-LIO2](https://github.com/hku-mars/Swarm-LIO2).

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/comp_ws
colcon build --symlink-install --packages-select swarm_msgs udp_bridge swarm_lio
source install/setup.bash
```

**Dependencies:**
- ROS2 Humble (`rclcpp`, `tf2_ros`, `pcl_ros`, `visualization_msgs`)
- [GTSAM](https://github.com/borglab/gtsam) (installed to `/usr/local`)
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) (compiled in workspace)
- PCL >= 1.12, Eigen3

## Run (single drone, MID-360)

```bash
# Launch everything: IMU → Livox driver → Swarm-LIO2 → UDP bridge
ros2 launch swarm_lio swarm_lio2.launch.py \
    drone_id:=1 namespace:=dp550_1 config_file:=mid360.yaml
```

To override individual parameters:
```bash
ros2 launch swarm_lio swarm_lio2.launch.py \
    drone_id:=1 namespace:=dp550_1 \
    ros_args:='--ros-args -p common/drone_id:=1 -p preprocess/lidar_type:=1'
```

## Configuration

YAML configs in `config/`:
- `mid360.yaml` — Livox MID-360 (default)
- `avia.yaml` — Livox AVIA
- `simulation.yaml` — simulation mode

Key parameters to tune per drone:
| Parameter | Default | Description |
|-----------|---------|-------------|
| `common/drone_id` | 1 | Unique ID per drone |
| `common/imu_topic` | `/livox/imu` | IMU data topic |
| `common/lid_topic` | `/livox/lidar` | Livox CustomMsg topic |
| `mapping/LI_extrinsic_T` | — | LiDAR→IMU translation |
| `mapping/LI_extrinsic_R` | — | LiDAR→IMU rotation |
| `calibration/gravity_align_time` | 10.0 | Seconds for gravity init |

## Multi-Drone Setup

Each drone runs its own swarm_lio_node + udp_bridge on UDP port 8821.
The UDP bridge handles time sync, state exchange, and global extrinsic calibration
automatically over the local network.

Cross-drone topics (absolute, shared across namespaces):
- `/quadstate_to_teammate`
- `/quadstate_from_teammate`
- `/global_extrinsic_to_teammate`
- `/global_extrinsic_from_teammate`
- `/teammate_id_with_traj_matching`

## Visualization

```bash
rviz2 -d install/swarm_lio/share/swarm_lio/rviz_cfg/mid360.rviz
```

Then set the Fixed Frame to `dp550_1/world` (or your drone's namespace + `/world`).

## Port Changes from ROS1

- Build system: catkin_make → ament_cmake + colcon
- ROS API: roscpp → rclcpp, tf → tf2_ros
- Messages: custom msg/ → swarm_msgs rosidl package
- Config: rosparam YAML → ROS2 parameter YAML (with namespace)
- Topics: absolute paths → relative (namespace-aware) for local topics
- Launch: .launch XML → .launch.py Python
- Imu: /mavros/imu/data → /livox/imu (configurable)
- mavros pose: disabled by default (`publish/mavros_pose_en: false`)

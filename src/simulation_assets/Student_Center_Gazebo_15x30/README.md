# Student Center 15 x 30 m Gazebo Environment

`15 x 30 m` 长方形 Gazebo 场地、三机 PX4/MAVROS
启动文件，以及场景引用的 `grey_wall` 自定义模型。

## 文件

- `PX4_Firmware/Tools/sitl_gazebo/worlds/Student_Center.world`
- `PX4_Firmware/launch/student_center_three_uav.launch`
- `gazebo_models/grey_wall/`


## 安装

在已有 PX4/XTDrone 仿真环境的电脑上执行：

```bash
cp PX4_Firmware/Tools/sitl_gazebo/worlds/Student_Center.world \
  ~/PX4_Firmware/Tools/sitl_gazebo/worlds/
cp PX4_Firmware/launch/student_center_three_uav.launch \
  ~/PX4_Firmware/launch/
mkdir -p ~/.gazebo/models
cp -a gazebo_models/grey_wall ~/.gazebo/models/
```

如果 PX4 工程不在 `~/PX4_Firmware`，将前两条命令的目标路径替换为实际路径。

## 启动

```bash
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=~/PX4_Firmware:$ROS_PACKAGE_PATH
roslaunch px4 student_center_three_uav.launch
```

默认出生点为：

```text
iris_0: (-2.0, -12.5, 0.4)
iris_1: ( 2.0, -12.5, 0.4)
iris_2: ( 0.0, -10.8, 0.4)
```


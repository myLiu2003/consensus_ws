# imu_publisher ROS2

ROS2 driver node for publishing ICM45686 IMU data.

## Topic Convention

This driver publishes raw accelerometer and gyroscope data without fused orientation.

Default topic:

```bash
imu/data_raw
```

Default frame:

```bash
imu_link
```

Message type:

```bash
sensor_msgs/msg/Imu
```

The node sets `orientation_covariance[0] = -1.0` because ICM45686 output here does not provide an orientation estimate.

## Build On NanoPi

Copy this package to:

```bash
~/comp_ws/src/imu_publisher
```

Then build:

```bash
cd ~/comp_ws
colcon build --packages-select imu_publisher
source install/setup.bash
```

## Run

Single UAV:

```bash
ros2 launch imu_publisher imu_publisher.launch.py
```

Three-UAV naming examples:

```bash
ros2 launch imu_publisher imu_publisher.launch.py namespace:=uav_0
ros2 launch imu_publisher imu_publisher.launch.py namespace:=uav_1
ros2 launch imu_publisher imu_publisher.launch.py namespace:=uav_2
```

The corresponding topics are:

```bash
/uav_0/imu/data_raw
/uav_1/imu/data_raw
/uav_2/imu/data_raw
```

To keep the old ROS1-style local topic name:

```bash
ros2 launch imu_publisher imu_publisher.launch.py publish_topic:=imu/data
```

## Parameters

- `i2c_bus`: default `/dev/i2c-8`
- `i2c_addr`: default `105` (`0x69`)
- `frame_id`: default `imu_link`
- `publish_topic`: default `imu/data_raw`
- `publish_rate_hz`: default `300.0`
- `accel_covariance`: default `0.04`
- `gyro_covariance`: default `0.02`

## Check

```bash
ros2 topic list | grep imu
ros2 topic info /imu/data_raw
ros2 topic hz /imu/data_raw
ros2 topic echo /imu/data_raw --once
```

For a namespaced UAV:

```bash
ros2 topic hz /uav_0/imu/data_raw
ros2 topic echo /uav_0/imu/data_raw --once
```

At rest, the acceleration vector norm should be close to `9.8 m/s^2`, and angular velocity should be close to `0 rad/s`.

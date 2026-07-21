cmake_minimum_required(VERSION 3.8)
project(swarm_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  msg/ConnectedTeammateList.msg
  msg/GlobalExtrinsic.msg
  msg/GlobalExtrinsicStatus.msg
  msg/ObserveTeammate.msg
  msg/QuadStatePub.msg
  msg/SpatialTemporalOffset.msg
  msg/SpatialTemporalOffsetStatus.msg
  msg/TeamStatus.msg
  msg/TeammateInfo.msg
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files}
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()

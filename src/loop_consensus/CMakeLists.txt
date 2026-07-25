cmake_minimum_required(VERSION 3.8)
project(loop_consensus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(map_consensus_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(
  loop_consensus_node
  src/loop_consensus_node.cpp
)

target_compile_features(
  loop_consensus_node
  PUBLIC cxx_std_17
)

ament_target_dependencies(
  loop_consensus_node
  rclcpp
  map_consensus_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(
  TARGETS loop_consensus_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

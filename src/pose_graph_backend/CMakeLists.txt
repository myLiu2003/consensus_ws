cmake_minimum_required(VERSION 3.8)
project(pose_graph_backend)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree registration)
find_package(Eigen3 REQUIRED)
find_package(map_consensus_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(pose_graph_backend_core
  src/submap_client.cpp
  src/gicp_aligner.cpp
  src/consistency_checker.cpp
)

target_include_directories(pose_graph_backend_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(pose_graph_backend_core
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(pose_graph_backend_core
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  map_consensus_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(gicp_verifier_node
  src/gicp_verifier_node.cpp
)

target_link_libraries(gicp_verifier_node
  pose_graph_backend_core
)

ament_target_dependencies(gicp_verifier_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  map_consensus_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  pose_graph_backend_core
  gicp_verifier_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_consistency_checker test/test_consistency_checker.cpp)
  target_link_libraries(test_consistency_checker pose_graph_backend_core)
endif()

ament_package()

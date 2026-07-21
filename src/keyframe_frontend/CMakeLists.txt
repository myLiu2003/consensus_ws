cmake_minimum_required(VERSION 3.8)
project(keyframe_frontend)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED)
find_package(map_consensus_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
add_executable(keyframe_frontend_node
  src/keyframe_frontend_node.cpp
  src/keyframe_manager.cpp
  src/submap_builder.cpp
)

target_include_directories(keyframe_frontend_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(keyframe_frontend_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(keyframe_frontend_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  map_consensus_msgs
    visualization_msgs

install(TARGETS keyframe_frontend_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

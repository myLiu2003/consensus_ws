cmake_minimum_required(VERSION 3.8)
project(udp_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(fmt REQUIRED)

include_directories(
  include
  src
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(udp_online_node src/udp_online.cpp)

# DEBUG_FILE_DIR 宏需要 ROOT_DIR，设为当前包路径方便调试日志
target_compile_definitions(udp_online_node PRIVATE ROOT_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/")

ament_target_dependencies(udp_online_node
  rclcpp
  swarm_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  ament_index_cpp
)

target_link_libraries(udp_online_node fmt pthread ${EIGEN3_LIBRARIES})

install(TARGETS udp_online_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()

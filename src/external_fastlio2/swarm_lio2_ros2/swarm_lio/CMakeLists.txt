cmake_minimum_required(VERSION 3.8)
project(swarm_lio)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# GTSAM installed to /usr/local (source build), not default cmake path
list(INSERT CMAKE_PREFIX_PATH 0 "/usr/local")
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search io)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

include_directories(include src ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

# ---- MultiUAV library ----
add_library(multi_uav_lib src/MultiUAV.cpp)
ament_target_dependencies(multi_uav_lib rclcpp swarm_msgs sensor_msgs nav_msgs std_msgs tf2_ros visualization_msgs pcl_ros pcl_conversions)
target_link_libraries(multi_uav_lib ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES} gtsam)
target_compile_definitions(multi_uav_lib PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

# ---- Swarm-LIO2 main node ----
add_executable(swarm_lio_node src/laserMapping.cpp src/preprocess.cpp include/ikd-Tree/ikd_Tree.cpp)
ament_target_dependencies(swarm_lio_node rclcpp swarm_msgs sensor_msgs nav_msgs std_msgs tf2_ros visualization_msgs pcl_ros pcl_conversions livox_ros_driver2)
target_link_libraries(swarm_lio_node multi_uav_lib ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES} gtsam pthread)
target_compile_definitions(swarm_lio_node PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

# ---- Install targets ----
install(TARGETS swarm_lio_node multi_uav_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config OPTIONAL)
install(DIRECTORY rviz_cfg/ DESTINATION share/${PROJECT_NAME}/rviz_cfg OPTIONAL)
install(FILES README.md DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()

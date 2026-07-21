cmake_minimum_required(VERSION 3.8)
project(multi_uav_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    models
    worlds
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/setup_sim_env.sh
    scripts/validate_x500_lidar3d.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

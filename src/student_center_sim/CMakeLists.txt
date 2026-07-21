cmake_minimum_required(VERSION 3.8)
project(student_center_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    worlds
    models
    config
    tools
    legacy
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/start_agent.sh
    scripts/start_three_px4.sh
    scripts/list_sensor_topics.sh
    scripts/auto_bridge_sensors.sh
    scripts/check_simulation.sh
    scripts/stop_all.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

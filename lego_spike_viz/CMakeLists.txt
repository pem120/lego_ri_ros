cmake_minimum_required(VERSION 3.25.0)
project(lego_spike_viz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
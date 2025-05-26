cmake_minimum_required(VERSION 3.25.0)
project(lego_spike_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY instructions launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
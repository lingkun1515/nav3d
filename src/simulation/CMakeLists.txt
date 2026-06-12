cmake_minimum_required(VERSION 3.8)
project(simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  worlds
  launch
  config
  meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pcd_to_world.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

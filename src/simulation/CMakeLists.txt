cmake_minimum_required(VERSION 3.8)
project(simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  worlds
  launch
  meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pcd_to_world.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()

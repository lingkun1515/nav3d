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

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
endif()

install(PROGRAMS
  scripts/pcd_to_world.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()

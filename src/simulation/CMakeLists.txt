cmake_minimum_required(VERSION 3.8)
project(simulation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# ROS 2 dependencies
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ONNX Runtime (local)
set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/onnxruntime")
set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_ROOT}/include")
set(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_ROOT}/lib")
find_library(ONNXRUNTIME_LIB onnxruntime PATHS "${ONNXRUNTIME_LIB_DIR}" NO_DEFAULT_PATH REQUIRED)

# A1RLController plugin
add_library(A1RLController SHARED
  src/a1_rl_controller.cpp
)

target_include_directories(A1RLController PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
)

target_link_libraries(A1RLController
  ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
  ${ONNXRUNTIME_LIB}
  rclcpp::rclcpp
)

target_compile_features(A1RLController PUBLIC cxx_std_17)

ament_target_dependencies(A1RLController
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

# RPATH so the plugin finds libonnxruntime.so at runtime
set_target_properties(A1RLController PROPERTIES
  INSTALL_RPATH "${ONNXRUNTIME_LIB_DIR}"
)

# Install plugin
install(TARGETS A1RLController
  LIBRARY DESTINATION lib
)

# Install ONNX Runtime shared libs alongside plugin
install(DIRECTORY "${ONNXRUNTIME_LIB_DIR}/"
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.so" PATTERN "*.so.*"
)

# Install URDF, worlds, launch, meshes, models
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

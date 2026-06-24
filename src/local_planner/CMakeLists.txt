cmake_minimum_required(VERSION 3.10)
project(local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# serial is optional — only needed for direct motor controller connection
find_package(serial QUIET)
if(serial_FOUND)
  add_compile_definitions(SERIAL_ENABLED)
  message(STATUS "serial found — motor controller support enabled")
else()
  message(STATUS "serial not found — motor controller via ROS topic only")
endif()

# --- latticePlanner executable ---
add_executable(latticePlanner src/local_planner_node.cpp)
ament_target_dependencies(latticePlanner
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  message_filters
  pcl_ros
  pcl_conversions
)
target_include_directories(latticePlanner PRIVATE ${PCL_INCLUDE_DIRS})

# --- pathFollower executable ---
add_executable(pathFollower src/path_follower_node.cpp)
ament_target_dependencies(pathFollower
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
if(serial_FOUND)
  target_link_libraries(pathFollower ${serial_LIBRARIES})
  target_include_directories(pathFollower PRIVATE ${serial_INCLUDE_DIRS})
endif()

install(TARGETS
  latticePlanner
  pathFollower
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    config
    launch
    paths
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

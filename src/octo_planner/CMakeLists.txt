cmake_minimum_required(VERSION 3.8)
project(octo_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(PCL REQUIRED COMPONENTS io common)

# --- OctoPlanner3D external library integration ---
set(OCTOPLANNER3D_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/OctoPlanner3D")

# Build OctoPlanner3D's sub-libraries directly.
# We override the vendored octomap/octomath targets with system versions
# before adding subdirectories so the library links against system octomap.
if(NOT TARGET octomap_vendored)
  add_library(octomap_vendored INTERFACE)
  target_include_directories(octomap_vendored INTERFACE
    "${OCTOPLANNER3D_DIR}/octomap/include"
  )
  target_link_libraries(octomap_vendored INTERFACE ${OCTOMAP_LIBRARIES})
endif()

# Build the PCD→OctoMap converter library
add_library(OctoPlanner3D_Octomap STATIC
  "${OCTOPLANNER3D_DIR}/octomap/src/pcd2octomap_converter.cpp"
)
target_include_directories(OctoPlanner3D_Octomap PUBLIC
  "${OCTOPLANNER3D_DIR}/octomap/include"
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(OctoPlanner3D_Octomap PUBLIC
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_compile_definitions(OctoPlanner3D_Octomap PRIVATE ${PCL_DEFINITIONS})
set_target_properties(OctoPlanner3D_Octomap PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Build the GlobalPlanner library
add_library(OctoPlanner3D_Planner STATIC
  "${OCTOPLANNER3D_DIR}/planner/src/global_planner.cpp"
)
target_include_directories(OctoPlanner3D_Planner PUBLIC
  "${OCTOPLANNER3D_DIR}/planner/include"
  "${OCTOPLANNER3D_DIR}/octomap/include"
  ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(OctoPlanner3D_Planner PUBLIC
  ${OCTOMAP_LIBRARIES}
)
set_target_properties(OctoPlanner3D_Planner PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- Main ROS 2 node ---
add_executable(octo_planner_node src/octo_planner_node.cpp)

target_include_directories(octo_planner_node PRIVATE
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(octo_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  octomap_msgs
)

target_link_libraries(octo_planner_node
  OctoPlanner3D_Planner
  OctoPlanner3D_Octomap
  ${OCTOMAP_LIBRARIES}
)

install(TARGETS octo_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

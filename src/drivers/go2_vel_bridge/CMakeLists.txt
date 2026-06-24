cmake_minimum_required(VERSION 3.10)
project(go2_vel_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/sport_request.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  unitree_api
)

add_executable(go2_vel_bridge_node src/go2_vel_bridge_node.cpp)
target_link_libraries(go2_vel_bridge_node ${PROJECT_NAME}_lib)
ament_target_dependencies(go2_vel_bridge_node
  rclcpp
  geometry_msgs
  unitree_api
)

install(TARGETS ${PROJECT_NAME}_lib go2_vel_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(rclcpp geometry_msgs unitree_api)

ament_package()

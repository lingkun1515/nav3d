cmake_minimum_required(VERSION 3.10)
project(global_reloc LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# PCL precompiles its templated algorithms only for the built-in point types.
# Our custom PointXYZRN type needs the templates instantiated in this TU.
# Must be visible before any PCL header is included.
add_compile_definitions(PCL_NO_PRECOMPILE)

find_package(PCL REQUIRED COMPONENTS common io kdtree features registration filters surface)
find_package(Eigen3 REQUIRED)

option(GLOBAL_RELOC_BUILD_ROS "Build ROS2 nodes (requires ament_cmake)" ON)
# GUI is a non-ROS standalone demo (needs glfw3+ImGui). Default OFF so a plain
# `colcon build` of the workspace never depends on glfw3; standalone GUI users
# pass -DGLOBAL_RELOC_BUILD_GUI=ON.
option(GLOBAL_RELOC_BUILD_GUI "Build the reloc_gui demo (requires glfw3+ImGui)" OFF)
option(GLOBAL_RELOC_BUILD_TESTS "Build unit tests (requires gtest)" OFF)
option(GLOBAL_RELOC_ENABLE_TBB "Use TBB backend for small_gicp parallelism" OFF)
option(GLOBAL_RELOC_ENABLE_TEASER "Enable TEASER++ coarse matcher (optional)" OFF)
option(GLOBAL_RELOC_BUILD_ROSBAG_TOOLS "Build bag_to_query/imu_yaw_delta (needs rosbag2_cpp; off — replay via `ros2 bag play` instead)" OFF)

# --- small_gicp: bundled under third_party. NOT purely header-only — its
# preprocess_points/align helpers are compiled into libsmall_gicp. We build it
# via add_subdirectory (minimal static config) and link the target.
set(SMALL_GICP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/small_gicp/include CACHE PATH "small_gicp include dir")
if(NOT EXISTS ${SMALL_GICP_INCLUDE_DIR}/small_gicp)
  message(WARNING "small_gicp not found under third_party/. "
    "Fine-matching falls back to PCL GICP.\n")
  set(GLOBAL_RELOC_HAVE_SMALL_GICP OFF)
else()
  set(GLOBAL_RELOC_HAVE_SMALL_GICP ON)
  # Configure small_gicp as a minimal static helper lib (no tests/examples/etc).
  set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
  set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
  set(BUILD_WITH_TBB OFF CACHE BOOL "" FORCE)
  set(BUILD_WITH_PCL OFF CACHE BOOL "" FORCE)
  set(BUILD_WITH_FAST_GICP OFF CACHE BOOL "" FORCE)
  set(BUILD_WITH_IRIDESCENCE OFF CACHE BOOL "" FORCE)
  set(BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  add_subdirectory(third_party/small_gicp EXCLUDE_FROM_ALL)
endif()

add_library(global_reloc_lib STATIC
  src/params.cpp
  src/feature.cpp
  src/map_builder.cpp
  src/coarse_matching.cpp
  src/bev_matching.cpp
  src/ndt_matching.cpp
  src/ndt_gicp_matcher.cpp
  src/fast_global_matcher.cpp
  src/scoring.cpp
  src/relocalizer.cpp
  src/nonros/reloc_facade.cpp
)
if(GLOBAL_RELOC_HAVE_SMALL_GICP)
  target_sources(global_reloc_lib PRIVATE src/fine_matching.cpp)
endif()

target_include_directories(global_reloc_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  PRIVATE
    $<BUILD_INTERFACE:${SMALL_GICP_INCLUDE_DIR}>
)
target_link_libraries(global_reloc_lib PUBLIC ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})
if(GLOBAL_RELOC_HAVE_SMALL_GICP)
  target_link_libraries(global_reloc_lib PUBLIC small_gicp)
endif()
if(OPENMP_FOUND OR OpenMP_FOUND)
  target_link_libraries(global_reloc_lib PUBLIC OpenMP::OpenMP_CXX)
endif()
if(GLOBAL_RELOC_ENABLE_TBB)
  find_package(TBB REQUIRED)
  target_link_libraries(global_reloc_lib PUBLIC TBB::tbb)
  target_compile_definitions(global_reloc_lib PUBLIC SMALL_GICP_USE_TBB)
endif()
if(GLOBAL_RELOC_HAVE_SMALL_GICP)
  target_compile_definitions(global_reloc_lib PUBLIC GLOBAL_RELOC_HAVE_SMALL_GICP)
endif()

# --- CLI tools (no ROS) ---
add_executable(build_map_cli src/cli/build_map_cli.cpp)
target_link_libraries(build_map_cli PRIVATE global_reloc_lib)
target_include_directories(build_map_cli PRIVATE ${SMALL_GICP_INCLUDE_DIR})

add_executable(reloc_cli src/cli/reloc_cli.cpp)
target_link_libraries(reloc_cli PRIVATE global_reloc_lib)
target_include_directories(reloc_cli PRIVATE ${SMALL_GICP_INCLUDE_DIR})

add_executable(diag_cands src/cli/diag_cands.cpp)
target_link_libraries(diag_cands PRIVATE global_reloc_lib)
target_include_directories(diag_cands PRIVATE ${SMALL_GICP_INCLUDE_DIR})

# --- GUI (no ROS): ImGui vendored under third_party/imgui, glfw via find_package ---
if(GLOBAL_RELOC_BUILD_GUI)
  set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui)
  if(NOT EXISTS ${IMGUI_DIR}/imgui.h)
    message(WARNING "ImGui not found at ${IMGUI_DIR}. Skipping reloc_gui.\n"
      "Run: git clone --depth 1 https://github.com/ocornut/imgui.git third_party/imgui")
  else()
    find_package(glfw3 QUIET)
    if(NOT glfw3_FOUND)
      message(WARNING "glfw3 not found (apt install libglfw3-dev / vcpkg install glfw3 "
        "or set GLFW_DIR). Skipping reloc_gui.")
    else()
      add_library(imgui_backend STATIC
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/imgui_demo.cpp
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
      )
      target_include_directories(imgui_backend PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
      target_link_libraries(imgui_backend PUBLIC glfw)

      add_executable(reloc_gui
        src/nonros/reloc_gui.cpp
        src/nonros/point_cloud_viewer.cpp)
      target_link_libraries(reloc_gui PRIVATE global_reloc_lib imgui_backend)
      target_include_directories(reloc_gui PRIVATE ${SMALL_GICP_INCLUDE_DIR})
    endif()
  endif()
endif()

# --- ROS2 nodes (optional) ---
if(GLOBAL_RELOC_BUILD_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)

  add_executable(reloc_node src/ros/reloc_node.cpp)
  target_link_libraries(reloc_node global_reloc_lib)
  target_include_directories(reloc_node PRIVATE ${SMALL_GICP_INCLUDE_DIR})
  ament_target_dependencies(reloc_node
    rclcpp sensor_msgs geometry_msgs std_msgs pcl_conversions)
  install(TARGETS reloc_node DESTINATION lib/${PROJECT_NAME})

  # Optional offline bag tools (need rosbag2_cpp). OFF by default — the ROS node
  # reads live /livox/lidar and replay uses `ros2 bag play`, so programmatic bag
  # access isn't needed. Enable with -DGLOBAL_RELOC_BUILD_ROSBAG_TOOLS=ON.
  if(GLOBAL_RELOC_BUILD_ROSBAG_TOOLS)
    find_package(rosbag2_cpp REQUIRED)
    add_executable(bag_to_query src/tools/bag_to_query.cpp)
    target_link_libraries(bag_to_query global_reloc_lib)
    target_include_directories(bag_to_query PRIVATE ${SMALL_GICP_INCLUDE_DIR})
    ament_target_dependencies(bag_to_query rclcpp rosbag2_cpp sensor_msgs std_msgs)
    add_executable(imu_yaw_delta src/tools/imu_yaw_delta.cpp)
    target_link_libraries(imu_yaw_delta global_reloc_lib)
    ament_target_dependencies(imu_yaw_delta rclcpp rosbag2_cpp sensor_msgs std_msgs)
    install(TARGETS bag_to_query imu_yaw_delta DESTINATION lib/${PROJECT_NAME})
  endif()
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
endif()

install(TARGETS global_reloc_lib build_map_cli reloc_cli diag_cands
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(GLOBAL_RELOC_BUILD_ROS)
  ament_package()
endif()

# --- tests ---
if(GLOBAL_RELOC_BUILD_TESTS)
  enable_testing()
  find_package(ament_cmake_gtest QUIET)
  add_subdirectory(test)
endif()

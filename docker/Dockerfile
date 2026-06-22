# Dog3DNav - Ubuntu 20.04 + ROS 2 Foxy + NVIDIA GPU
FROM nvidia/cuda:11.8.0-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Ensure ROS libraries are discoverable by Gazebo plugins loaded via
# GAZEBO_PLUGIN_PATH. Without this, libgazebo_ros_factory.so fails to load
# its dependencies (libgazebo_msgs__rosidl_typesupport_cpp.so etc.) when
# launched via ros2 launch, because the gzserver subprocess may not inherit
# the full runtime LD_LIBRARY_PATH from the sourced setup.bash.
ENV LD_LIBRARY_PATH=/opt/ros/foxy/lib:${LD_LIBRARY_PATH}

# ROS 2 Foxy setup
# Retry-friendly install: NVIDIA/cuda repo can be flaky, so loop apt-get install
# with --fix-missing to tolerate transient single-package fetch failures.
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
  && echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2.list \
  && apt-get update \
  && for i in 1 2 3 4 5; do \
       apt-get install -y --fix-missing \
         ros-foxy-desktop \
         ros-foxy-rosbridge-server \
         ros-foxy-octomap \
         ros-foxy-octomap-msgs \
         ros-foxy-gazebo-ros-pkgs \
         ros-foxy-pcl-conversions \
         ros-foxy-pcl-ros \
         ros-foxy-xacro \
         ros-foxy-joint-state-publisher \
         ros-foxy-robot-state-publisher \
         ros-foxy-tf2-sensor-msgs \
         ros-foxy-rviz2 \
         python3-colcon-common-extensions \
         python3-rosdep \
         python3-vcstool \
       && break \
       || (echo "=== apt install attempt $i failed, retrying in 10s ===" && sleep 10 && apt-get update); \
     done \
  && rm -rf /var/lib/apt/lists/*

# Build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    liboctomap-dev \
    libpcl-dev \
    libeigen3-dev \
    libboost-all-dev \
  && rm -rf /var/lib/apt/lists/*

# Workspace setup
ENV ROS_WS=/ros2_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# NOTE: source build is intentionally NOT done here.
# The project is designed to mount the host workspace at /ros2_ws at runtime
# (see docker/run.sh), so colcon build runs against live source during
# development. This keeps the image as a pure environment image and avoids
# baking a stale source snapshot into the layer cache.

# Web frontend
COPY web/ ${ROS_WS}/web/

# Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 9090

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

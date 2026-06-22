# Dog3DNav - Ubuntu 20.04 + ROS 2 Foxy + NVIDIA GPU
FROM nvidia/cuda:11.8.0-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# ROS 2 Foxy setup
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
  && echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2.list \
  && apt-get update && apt-get install -y \
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

# Copy project source (maps live under src/bringup/maps/)
COPY src/ ${ROS_WS}/src/

# Build workspace
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Web frontend
COPY web/ ${ROS_WS}/web/

# Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 9090

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

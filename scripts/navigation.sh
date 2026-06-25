#!/bin/bash
# 启动导航栈（在 dog3dnav-foxy 容器内）
# 用法: ./scripts/navigation.sh [launch_rosbridge] [launch_rviz]
# 示例: ./scripts/navigation.sh
#       ./scripts/navigation.sh true false

ROSBRIDGE="${1:-true}"
RVIZ="${2:-true}"

CMD="source /opt/ros/foxy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch bringup navigation.launch.py launch_rosbridge:=$ROSBRIDGE launch_rviz:=$RVIZ"

docker exec -it dog3dnav-foxy bash -c "$CMD"

#!/bin/bash
# 启动导航栈（在 dog3dnav-foxy 容器内）
# 用法: ./scripts/navigation.sh [mode] [launch_rosbridge] [launch_rviz]
# 示例:
#   ./scripts/navigation.sh                         # 默认完整导航
#   ./scripts/navigation.sh default true false      # 完整导航 + rosbridge
#   ./scripts/navigation.sh no_avoidance             # 无避障版（跳过 latticePlanner）
#   ./scripts/navigation.sh no_avoidance true true   # 无避障 + rosbridge + rviz

MODE="${1:-no_avoidance}"
ROSBRIDGE="${2:-false}"
RVIZ="${3:-false}"

if [ "$MODE" = "no_avoidance" ]; then
  CMD="source ~/.bashrc && ros2 launch bringup navigation_no_avoidance.launch.py launch_rosbridge:=$ROSBRIDGE launch_rviz:=$RVIZ"
else
  CMD="source ~/.bashrc && ros2 launch bringup navigation.launch.py launch_rosbridge:=$ROSBRIDGE launch_rviz:=$RVIZ"
fi

docker exec -it dog3dnav-foxy bash -i -c "$CMD"

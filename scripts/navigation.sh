#!/bin/bash
# 启动导航栈（本机 ROS2 Humble）
# 用法: ./scripts/navigation.sh [mode] [launch_rosbridge] [launch_rviz]
# 示例:
#   ./scripts/navigation.sh                         # 默认完整导航
#   ./scripts/navigation.sh default true false      # 完整导航 + rosbridge
#   ./scripts/navigation.sh no_avoidance             # 无避障版（跳过 latticePlanner）
#   ./scripts/navigation.sh no_avoidance true true   # 无避障 + rosbridge + rviz

MODE="${1:-no_avoidance}"
ROSBRIDGE="${2:-true}"
RVIZ="${3:-false}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"

if [ "$MODE" = "no_avoidance" ]; then
  ros2 launch bringup navigation_no_avoidance.launch.py launch_rosbridge:=$ROSBRIDGE launch_rviz:=$RVIZ
else
  ros2 launch bringup navigation.launch.py launch_rosbridge:=$ROSBRIDGE launch_rviz:=$RVIZ
fi

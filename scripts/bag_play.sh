#!/bin/bash
# 播放 rosbag（本机 ROS2 Humble）
# 用法: ./scripts/bag_play.sh <bag_path> [--loop]
# 示例: ./scripts/bag_play.sh /path/to/bag
#       ./scripts/bag_play.sh /path/to/bag --loop

if [ -z "$1" ]; then
  echo "用法: $0 <bag_path> [--loop]"
  echo "示例: $0 /home/lenovo/Documents/rosbag/lidar_with_pc2"
  exit 1
fi

BAG="$1"
LOOP="${2:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"

if [ "$LOOP" = "--loop" ]; then
  ros2 bag play --loop "$BAG"
else
  ros2 bag play "$BAG"
fi

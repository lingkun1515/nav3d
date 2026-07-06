#!/bin/bash
# 播放 rosbag（本机 ROS2 Humble）
# 用法: ./scripts/bag_play.sh [bag_path] [--loop]
# 示例: ./scripts/bag_play.sh                        # 默认播放 docker/rosbag/lidar_with_pc2
#       ./scripts/bag_play.sh /path/to/bag            # 指定 bag
#       ./scripts/bag_play.sh /path/to/bag --loop     # 循环播放

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# 未指定 bag 则默认用项目内 docker/rosbag/lidar_with_pc2
DEFAULT_BAG="$PROJECT_DIR/docker/rosbag/lidar_with_pc2"
BAG="${1:-$DEFAULT_BAG}"
LOOP="${2:-}"

# 兼容 --loop 作为第一个参数的情况
if [ "$BAG" = "--loop" ]; then
  BAG="$DEFAULT_BAG"
  LOOP="--loop"
fi

if [ ! -d "$BAG" ]; then
  echo "错误: bag 路径不存在: $BAG" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"

if [ "$LOOP" = "--loop" ]; then
  echo "播放 (循环): $BAG"
  ros2 bag play --loop "$BAG"
else
  echo "播放: $BAG"
  ros2 bag play "$BAG"
fi

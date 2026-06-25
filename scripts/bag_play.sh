#!/bin/bash
# 在容器内播放 rosbag（在 dog3dnav-foxy 容器内）
# 用法: ./scripts/bag_play.sh <bag_path> [--loop]
# 示例: ./scripts/bag_play.sh /ros2_ws/docker/rosbag/lidar_with_pc2
#       ./scripts/bag_play.sh /path/to/bag --loop

if [ -z "$1" ]; then
  echo "用法: $0 <bag_path> [--loop]"
  echo "示例: $0 /ros2_ws/docker/rosbag/lidar_with_pc2"
  exit 1
fi

BAG="$1"
LOOP="${2:-}"

CMD="source ~/.bashrc"

if [ "$LOOP" = "--loop" ]; then
  CMD="$CMD && ros2 bag play --loop $BAG"
else
  CMD="$CMD && ros2 bag play $BAG"
fi

docker exec -it dog3dnav-foxy bash -i -c "$CMD"

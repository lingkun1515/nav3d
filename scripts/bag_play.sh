#!/bin/bash
# 在容器内播放 rosbag（在 dog3dnav-foxy 容器内）
# 用法: ./scripts/bag_play.sh <bag_path> [rate] [loop]
# 示例: ./scripts/bag_play.sh /ros2_ws/docker/rosbag/lidar_with_pc2
#       ./scripts/bag_play.sh /path/to/bag 0.5         # 半速播放
#       ./scripts/bag_play.sh /path/to/bag 1.0 --loop   # 循环播放

if [ -z "$1" ]; then
  echo "用法: $0 <bag_path> [rate] [--loop]"
  echo "示例: $0 /ros2_ws/docker/rosbag/lidar_with_pc2"
  exit 1
fi

BAG="$1"
RATE="${2:-1.0}"
LOOP="${3:-}"

CMD="source /opt/ros/foxy/setup.bash"

if [ "$LOOP" = "--loop" ]; then
  CMD="$CMD && ros2 bag play -r $RATE --loop $BAG"
else
  CMD="$CMD && ros2 bag play -r $RATE $BAG"
fi

docker exec -it dog3dnav-foxy bash -c "$CMD"

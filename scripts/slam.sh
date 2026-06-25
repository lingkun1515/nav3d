#!/bin/bash
# 启动 SLAM（在 dog3dnav-foxy 容器内）
# 用法: ./scripts/slam.sh [mode]
# 示例: ./scripts/slam.sh              # 默认 mapping
#       ./scripts/slam.sh relocation   # 重定位模式

MODE="${1:-mapping}"

CMD="source /opt/ros/foxy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch bringup slam.launch.py mode:=$MODE"

docker exec -it dog3dnav-foxy bash -c "$CMD"

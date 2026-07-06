#!/bin/bash
# 启动 SLAM（本机 ROS2 Humble）
# 用法: ./scripts/slam.sh [mode] [rviz]
# 示例:
#   ./scripts/slam.sh                       # 默认 mapping，无 rviz
#   ./scripts/slam.sh mapping true          # 建图 + rviz
#   ./scripts/slam.sh relocation false      # 重定位，无 rviz

MODE="${1:-mapping}"
RVIZ="${2:-false}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"

ros2 launch bringup slam.launch.py mode:=$MODE rviz:=$RVIZ

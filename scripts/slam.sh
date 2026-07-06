#!/bin/bash
# 启动 SLAM（本机 ROS2 Humble）
# 用法: ./scripts/slam.sh [mode] [use_sim_time]
# 示例:
#   ./scripts/slam.sh                    # 默认 mapping，真实时间
#   ./scripts/slam.sh mapping true       # 建图 + 仿真时间
#   ./scripts/slam.sh relocation true    # 重定位 + 仿真时间

MODE="${1:-mapping}"
USE_SIM_TIME="${2:-true}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"

ros2 launch bringup slam.launch.py mode:=$MODE use_sim_time:=$USE_SIM_TIME

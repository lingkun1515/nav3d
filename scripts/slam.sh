#!/bin/bash
# 启动 SLAM（本机 ROS2 Humble）
# 用法: ./scripts/slam.sh [mode] [rviz] [额外 launch 参数...]
# 示例:
#   ./scripts/slam.sh                              # 默认 mapping，无 rviz
#   ./scripts/slam.sh mapping true                 # 建图 + rviz
#   ./scripts/slam.sh relocation false             # 重定位，无 rviz
#   # 重定位 + global_reloc 全局重定位 (super_lio 订阅 /initial_pose 作为初始预估):
#   ./scripts/slam.sh relocation false \
#       global_reloc:=true map_key_path:=/tmp/reloc_map/map.gkey

MODE="${1:-mapping}"
RVIZ="${2:-false}"
[ $# -ge 1 ] && shift
[ $# -ge 1 ] && shift

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"

ros2 launch bringup slam.launch.py mode:=$MODE rviz:=$RVIZ "$@"

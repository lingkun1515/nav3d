#!/bin/bash
# 启动 SLAM（本机 ROS2 Humble）
# 用法: ./scripts/slam.sh [mode] [rviz] [global_reloc] [额外 launch 参数...]
# 示例:
#   ./scripts/slam.sh                              # 默认 relocation + global_reloc
#   ./scripts/slam.sh relocation false true        # 重定位 + global_reloc 全局重定位
#   ./scripts/slam.sh relocation false false       # 重定位（只用配置 init_pose）
#   ./scripts/slam.sh mapping true                 # 建图 + rviz
#   # 覆盖 .gkey 路径:
#   ./scripts/slam.sh relocation false true map_key_path:=/other/map.gkey

MODE="${1:-relocation}"
RVIZ="${2:-true}"
GLOBAL_RELOC="${3:-false}"
[ $# -ge 1 ] && shift
[ $# -ge 1 ] && shift
[ $# -ge 1 ] && shift

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"

ros2 launch bringup slam.launch.py mode:=$MODE rviz:=$RVIZ global_reloc:=$GLOBAL_RELOC "$@"

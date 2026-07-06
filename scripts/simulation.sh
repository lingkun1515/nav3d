#!/bin/bash
# 启动 Gazebo 仿真（本机 ROS2 Humble）
# 用法: ./scripts/simulation.sh [world] [model] [slam_mode] [x] [y] [yaw] [gui]

WORLD="${1:-}"
MODEL="${2:-a1}"
SLAM_MODE="${3:-true}"
X="${4:-0.0}"
Y="${5:--0.0}"
YAW="${6:-0.0}"
GUI="${7:-true}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"

echo "[simulation.sh] 清理残留进程..."
killall -9 gzserver gzclient rosbridge_websocket robot_state_publisher 2>/dev/null || true
sleep 1

CMD="ros2 launch simulation gazebo.launch.py"
if [ -n "$WORLD" ]; then
  CMD="$CMD world:=\$(ros2 pkg prefix simulation)/share/simulation/worlds/$WORLD"
fi
CMD="$CMD robot_model:=$MODEL slam_mode:=$SLAM_MODE x:=$X y:=$Y yaw:=$YAW gui:=$GUI"
eval "$CMD"

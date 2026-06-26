#!/bin/bash
# 启动 Gazebo 仿真（在 dog3dnav-foxy 容器内）
# 用法: ./scripts/simulation.sh [world] [model] [slam_mode] [x] [y] [yaw]
# 示例:
#   ./scripts/simulation.sh                          # 默认 urban2_story + a1
#   ./scripts/simulation.sh empty.world car           # 空地 + 小车
#   ./scripts/simulation.sh "" a1 true               # a1 + SLAM 闭环模式
#   ./scripts/simulation.sh "" a1 true 1.0 0.0 1.57  # 指定初始位姿

WORLD="${1:-}"
MODEL="${2:-a1}"
SLAM_MODE="${3:-true}"
X="${4:-0.0}"
Y="${5:--0.0}"
YAW="${6:-0.0}"

CMD="source ~/.bashrc && ros2 launch simulation gazebo.launch.py"

if [ -n "$WORLD" ]; then
  CMD="$CMD world:=\$(ros2 pkg prefix simulation)/share/simulation/worlds/$WORLD"
fi
CMD="$CMD robot_model:=$MODEL slam_mode:=$SLAM_MODE x:=$X y:=$Y yaw:=$YAW"

docker exec -it dog3dnav-foxy bash -i -c "$CMD"

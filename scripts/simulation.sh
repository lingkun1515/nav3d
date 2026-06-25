#!/bin/bash
# 启动 Gazebo 仿真（在 dog3dnav-foxy 容器内）
# 用法: ./scripts/simulation.sh [world_file] [robot_model]
# 示例: ./scripts/simulation.sh                          # 默认 urban2_story + a1
#       ./scripts/simulation.sh empty.world car           # 空地 + 小车

WORLD="${1:-}"
MODEL="${2:-a1}"

CMD="source ~/.bashrc"

if [ -n "$WORLD" ]; then
  CMD="$CMD && ros2 launch simulation gazebo.launch.py world:=\$(ros2 pkg prefix simulation)/share/simulation/worlds/$WORLD robot_model:=$MODEL"
else
  CMD="$CMD && ros2 launch simulation gazebo.launch.py robot_model:=$MODEL"
fi

docker exec -it dog3dnav-foxy bash -i -c "$CMD"

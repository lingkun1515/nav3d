#!/bin/bash
# 启动 Gazebo 仿真（在 dog3dnav-foxy 容器内）
# 用法: ./scripts/simulation.sh [world] [model] [slam_mode] [x] [y] [yaw] [gui]
# 示例:
#   ./scripts/simulation.sh                          # 默认 hospital + a1 + SLAM，楼梯旁
#   ./scripts/simulation.sh empty.world car false    # 空地 + 小车 + 非 SLAM
#   ./scripts/simulation.sh "" a1 true               # a1 + SLAM 闭环模式
#   ./scripts/simulation.sh "" a1 true 0 0 0         # 原点生成
#   ./scripts/simulation.sh "" a1 true 0 0 0 false   # 无 GUI (headless))

WORLD="${1:-}"
MODEL="${2:-a1}"
SLAM_MODE="${3:-true}"
X="${4:-0.0}"
Y="${5:--0.0}"
YAW="${6:-0.0}"
GUI="${7:-true}"

# ---- 同步 X11 cookie（防止 MIT-MAGIC-COOKIE 过期导致 gzclient 崩溃） ----
if [ "$GUI" = "true" ] && [ -n "$DISPLAY" ]; then
  HEX_KEY=$(xauth list "$DISPLAY" 2>/dev/null | head -1 | awk '{print $NF}')
  if [ -n "$HEX_KEY" ]; then
    docker exec -i dog3dnav-foxy xauth add "$DISPLAY" MIT-MAGIC-COOKIE-1 "$HEX_KEY" 2>/dev/null
  fi
fi

# ---- 清理容器内残留的 Gazebo/rosbridge 进程 ----
echo "[simulation.sh] 清理残留进程..."
docker exec -i dog3dnav-foxy bash -c '
  killall -9 gzserver gzclient rosbridge_websocket robot_state_publisher 2>/dev/null
  sleep 1
' 2>/dev/null

# ---- 构建 launch 命令 ----
CMD="source /opt/ros/foxy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch simulation gazebo.launch.py"

if [ -n "$WORLD" ]; then
  CMD="$CMD world:=\$(ros2 pkg prefix simulation)/share/simulation/worlds/$WORLD"
fi
CMD="$CMD robot_model:=$MODEL slam_mode:=$SLAM_MODE x:=$X y:=$Y yaw:=$YAW gui:=$GUI"

# ---- 执行 ----
if [ -t 0 ]; then
  docker exec -it dog3dnav-foxy bash -i -c "$CMD"
else
  docker exec -i dog3dnav-foxy bash -c "$CMD"
fi

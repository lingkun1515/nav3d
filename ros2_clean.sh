#!/usr/bin/env bash
# ros2_clean.sh — 彻底清理 ROS 2 残留进程 + DDS 幽灵端点
#
# 用途: 每次重启 launch 前运行，避免僵尸进程堆积和 DDS 订阅端点残留。
# 背景: 本容器 PID 1 是 `sleep infinity`，不会 reap 孤儿子进程，
#       所以 Ctrl+C 后 ros2 launch 的子进程若未被正常回收会变僵尸；
#       FastDDS 的共享内存端点也会残留，导致 topic info 显示虚假订阅者。

set -u

echo "[ros2_clean] 停止 ROS 2 daemon..."
ros2 daemon stop 2>/dev/null || true

echo "[ros2_clean] 杀掉所有 ROS 2 相关进程..."
PATTERNS=(
  "ros2 launch"
  "ros2 run"
  "super_lio"
  "relocation_node"
  "octo_planner"
  "local_planner"
  "latticePlanner"
  "pathFollower"
  "go2_vel_bridge"
  "static_transform_publisher"
  "rosbag"
  "rviz2"
  "rosbridge"
  "_ros2_daemon"
)
for pat in "${PATTERNS[@]}"; do
  pkill -9 -f "$pat" 2>/dev/null && echo "  killed: $pat" || true
done

sleep 1

echo "[ros2_clean] 清理 FastDDS 共享内存端点（消除幽灵订阅者）..."
rm -f /dev/shm/fastrtps* 2>/dev/null
rm -f /dev/shm/sem.fastrtps* 2>/dev/null
rm -f /dev/shm/sem.* 2>/dev/null

echo "[ros2_clean] 清理 ros2cli daemon 数据 + launch 临时文件..."
rm -rf /root/.ros/*daemon* 2>/dev/null
rm -f /tmp/launch_params_* 2>/dev/null

sleep 1

echo "[ros2_clean] 重启 ROS 2 daemon..."
ros2 daemon start 2>/dev/null || true
sleep 2

# 报告状态
ZOMBIES=$(ps -eo stat | grep -c "^Z")
ACTIVE=$(ps -eo stat,comm | grep -v "^Z" | grep -cE "ros2|lio|octo_planner|rosbag|transform")
echo ""
echo "[ros2_clean] 完成。"
echo "  历史僵尸进程（不影响运行）: $ZOMBIES"
echo "  活跃 ROS 进程: $ACTIVE"
echo ""
echo "  注: 历史僵尸是 PID 1 (sleep infinity) 未回收的，无法从用户空间清除，"
echo "      但不影响 ROS 2 运行。重启容器可清零。"

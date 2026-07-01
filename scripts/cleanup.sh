#!/usr/bin/env bash
# cleanup.sh — 清理 SLAM + Navigation 栈残留进程 & FastDDS 幽灵端点
# 用法: ./scripts/cleanup.sh
# 注意: 不会动 ros2 daemon / 系统信号量，安全可反复跑

set -u

# ---------- 1. 杀节点进程 ----------
# 使用正则自排除技巧（[x] 写法），防止 pkill 匹配到自身
PROCS=(
  "[s]uper_lio"
  "[o]cto_planner"
  "[p]athFollower"
  "[l]atticePlanner"
  "[g]o2_vel_bridge"
  "[r]viz2"
  "[r]osbridge"
  "[r]elocation_node"
  "[s]tatic_transform_publisher"
)

killed=0
for pat in "${PROCS[@]}"; do
  if pkill -9 -f "$pat" 2>/dev/null; then
    echo "  killed: $pat"
    killed=1
  fi
done
[[ $killed -eq 0 ]] && echo "  没有需要清理的活跃节点"

# ---------- 2. 清 FastDDS SHM 端点（消除幽灵订阅者） ----------
SHM_FILES=(/dev/shm/fastrtps* /dev/shm/sem.fastrtps*)
shopt -s nullglob
cleaned=0
for f in "${SHM_FILES[@]}"; do
  rm -f "$f" 2>/dev/null && cleaned=1
done
shopt -u nullglob
[[ $cleaned -eq 1 ]] && echo "  已清理 FastDDS 共享内存端点"
[[ $cleaned -eq 0 ]] && echo "  FastDDS 共享内存干净，无需清理"

# ---------- 3. 清 launch 临时文件 ----------
rm -f /tmp/launch_params_* 2>/dev/null

# ---------- 4. 报告 ----------
ZOMBIES=$(ps -eo stat 2>/dev/null | grep -c "^Z" || echo 0)
echo ""
echo "  [cleanup] 完成。僵尸进程（历史残留，不影响运行）: $ZOMBIES"
echo "  提示: 僵尸只能重启容器清零，不影响 ROS 2 运行"

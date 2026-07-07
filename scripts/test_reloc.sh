#!/bin/bash
# 测试 global_reloc → super_lio 重定位 单次启动
# 用法:
#   # 1) 首先生成 .gkey（只需一次, 与 super_lio 的 map.pcd 同源）
#   ./scripts/test_reloc.sh build-map
#   #    然后重新安装 bringup（把 .gkey 装到 share 目录, launch 默认路径才能找到）:
#   colcon build --packages-select bringup
#
#   # 2) 从 rosbag 第 0 秒开始测试（不需要写 map_key_path, 用默认值）
#   ./scripts/test_reloc.sh 0
#
#   # 3) 从第 120 秒开始
#   ./scripts/test_reloc.sh 120
#
#   # 循环测多个时刻:
#   for off in $(seq 0 20 360); do ./scripts/test_reloc.sh $off; sleep 2; done
#
# 监控方式（另开终端）:
#   tail -f /tmp/reloc_test_launch.log | grep -E --color \
#       'INIT|PUBLISHED|GET Initial|ICP Converged|coarse_strategy|waiting'
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_DIR"

source /opt/ros/humble/setup.bash
source install/setup.bash
export PATH="$PROJECT_DIR/install/global_reloc/lib/global_reloc:$PATH"

# ---- config ----
BAG="rosbag/lidar_with_pc2"
MAP_PCD="src/slam/src/super_lio/map/map.pcd"    # super_lio 先验 pcd
MAP_DIR="src/bringup/maps"                       # 中间产物统一放在 bringup/maps/
MAP_GKEY="$MAP_DIR/reloc_map.gkey"               # global_reloc 的 .gkey（与 MAP_PCD 同源）

case "${1:-help}" in
  build-map)
    echo "=== 从 super_lio 先验地图生成 global_reloc 的 .gkey ==="
    mkdir -p "$MAP_DIR"
    build_map_cli --pcd "$MAP_PCD" --out "$MAP_DIR" --name reloc_map --voxel 0.4
    ls -la "$MAP_GKEY" "$MAP_DIR/reloc_map.pcd" "$MAP_DIR/reloc_map.fpfh.bin" 2>/dev/null
    echo ""
    echo "完成。记得 colcon build bringup（把 .gkey 装到 share 目录, launch 才能用默认路径找到）："
    echo "  colcon build --packages-select bringup"
    echo "然后就可以跑了:  ./scripts/test_reloc.sh <start_offset_sec>"
    ;;
  help|-h|--help)
    echo "用法:"
    echo "  $0 build-map              # 生成 .gkey (只需一次)"
    echo "  $0 <start_offset_sec>     # 从 bag 的 offset 秒开始测试"
    echo "  $0 help                   # 本帮助"
    echo ""
    echo "loop 示例:  for off in \$(seq 0 20 360); do $0 \$off; sleep 3; done"
    echo "监控日志:  tail -f /tmp/reloc_test_launch.log | grep -E --color 'INIT|PUBLISHED|GET Initial|ICP Converged|waiting'"
    ;;
  *)
    OFF="$1"
    if [ ! -f "$MAP_GKEY" ]; then
      echo "错误: .gkey 不存在: $MAP_GKEY" >&2
      echo "请先运行: $0 build-map" >&2
      exit 1
    fi
    echo "========== start_offset=${OFF}s =========="
    # 清理上次残留
    pkill -x reloc_node 2>/dev/null || true
    pkill -x relocation_node 2>/dev/null || true
    sleep 1

    # 启动 SLAM（后台）, 日志写文件
    ros2 launch bringup slam.launch.py \
        mode:=relocation global_reloc:=true \
        map_key_path:="$MAP_GKEY" \
        use_sim_time:=false rviz:=false \
        reloc_gate_publish:=false reloc_strategy:=fast \
        accumulate_frames:=10 accumulate_max_dt:=1.0 \
        > /tmp/reloc_test_launch.log 2>&1 &
    LP=$!
    sleep 6

    # 播放 rosbag（前台, 你 Ctrl-C 退出）
    echo "--- 播放 rosbag (Ctrl-C 停止) ---"
    ros2 bag play --start-offset "$OFF" "$BAG" 2>/dev/null || true

    # 摘取关键日志
    echo ""
    echo "========== 结果 =========="
    grep -aE "coarse_strategy override|global_reloc ready|/initial_pose PUBLISHED|waiting for external|GET Initial guess|ICP Converged|INIT DONE" \
        /tmp/reloc_test_launch.log | tail -10

    # 清理
    kill $LP 2>/dev/null || true
    pkill -x reloc_node 2>/dev/null || true
    pkill -x relocation_node 2>/dev/null || true
    echo "========== done =========="
    ;;
esac

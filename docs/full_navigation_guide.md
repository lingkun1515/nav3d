# 全闭环导航调试指南

本文档是 [快速入门](quickstart.md) 的下一步，介绍如何在 Gazebo 仿真中运行完整的导航闭环：
**Web 选点 → 全局规划 → 局部避障 → 速度控制 → 小车移动**。

系统架构与模块详解参见 [系统架构](architecture.md)，本文不再重复。

---

## 2. 安装仿真依赖

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-rosbridge-server
```

---

## 3. 编译

```bash
cd ~/Projects/NavProject/Dog3DNav
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 4. 端到端 Launch 方式

### 4.1 完整导航闭环（一键启动）

```bash
ros2 launch bringup navigation.launch.py \
  pcd_file:=$HOME/Projects/NavProject/Dog3DNav/maps/building_map.pcd
```

此命令同时启动：Gazebo（自动从 PCD 生成世界场景）+ octo_planner + localPlanner + pathFollower + rosbridge + RViz2。
所有逻辑均在 C++ 节点内闭环，无 Python 中继节点。

若不需障碍物场景（仅测试运动控制），省略 pcd_file 参数即可在空地启动：

```bash
ros2 launch bringup navigation.launch.py
```

不启动 RViz2 时：

```bash
ros2 launch bringup navigation.launch.py launch_rviz:=false
```

**启动参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pcd_file` | `""` | 地图文件路径（格式详见 quickstart） |
| `launch_rviz` | `true` | 是否启动 RViz2（使用 `bringup/config/navigation.rviz`） |
| `use_sim_time` | `true` | 使用仿真时间 |

首次提供 `pcd_file` 时自动执行 `pcd_to_world` 生成 `from_pcd.world`（缓存在 `worlds/` 目录），后续启动直接复用。删除 `from_pcd.world` 可强制重新生成。

### 4.2 分步启动（调试用）

如果需要分步调试单个组件：

```bash
# 终端 1: Gazebo 仿真
ros2 launch simulation gazebo.launch.py

# 终端 2: 全局规划器
ros2 run octo_planner octo_planner_node --ros-args \
  -p pcd_file:=$HOME/Projects/NavProject/Dog3DNav/maps/building_map.pcd \
  -p resolution:=0.2 -p robot_radius:=0.05

# 终端 3: 局部规划 + 轨迹跟踪
ros2 run local_planner localPlanner --ros-args \
  -p autonomyMode:=true -p use_laser_scan:=true -p use_planned_path:=true &
ros2 run local_planner pathFollower --ros-args \
  -p autonomyMode:=true &
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 &
```

### 4.3 使用 PCD 地图生成 Gazebo 场景（离线）

也可以预先生成世界文件：

```bash
# 离线生成带障碍物的 Gazebo 世界
python3 src/simulation/scripts/pcd_to_world.py \
  maps/building_map.pcd \
  src/simulation/worlds/from_pcd.world \
  --resolution 0.3 --max-boxes 3000

# 用生成的世界启动 Gazebo
ros2 launch simulation gazebo.launch.py \
  world:=$(ros2 pkg prefix simulation)/share/simulation/worlds/from_pcd.world
```

> 注意：`pcd_to_world.py` 默认去除最低 Z 层（地面），将剩余体素合并为碰撞 box。
> `--resolution 0.3` 控制体素大小（越小越精细），`--max-boxes 3000` 限制最大 box 数（按体积排序保留最大的）。
> 使用 `navigation.launch.py` 时此步骤自动执行，生成结果缓存为 `from_pcd.world`。

---

## 5. Web 前端操作流程

1. 浏览器打开 `http://localhost:8080`
2. 等待连接状态显示「已连接 ROSBridge」
3. 等待地图数据接收完成（日志显示「地图数据全部接收完成」）
4. 点击 **「导航目标」** 按钮
5. 在绿色可通行层上 **点击** 设置目标点，**拖拽** 设置目标航向角
6. 弹出路径确认框 → 点击 **「开始导航」**
7. 小车开始移动，日志面板显示导航过程信息
8. 随时点击 **「停止导航」** 紧急停止

---

## 6. 关键参数调优

### 6.1 局部规划器 (`local_planner_params.yaml`)

| 参数 | 默认值 | 仿真建议值 | 说明 |
|------|--------|-----------|------|
| `vehicleLength` | 0.4 | 0.5 | 小车碰撞包络长 (m) |
| `vehicleWidth` | 0.4 | 0.4 | 小车碰撞包络宽 (m) |
| `adjacentRange` | 3.5 | 3.0 | 感知范围半径 (m) |
| `obstacleHeightThre` | 0.2 | 0.15 | 障碍物高度阈值 (m) |
| `checkObstacle` | true | true | 是否启用避障 |
| `autonomyMode` | false | **true** | 自主导航模式（必须开启） |
| `autonomySpeed` | 1.0 | 0.5 | 自主导航速度 (m/s) |
| `maxSpeed` | 1.0 | 0.5 | 最大速度 (m/s) |
| `use_laser_scan` | false | **true** | 订阅 /scan (LaserScan) |
| `use_planned_path` | false | **true** | 订阅 /planned_path，内部管理航点 |
| `global_frame_id` | "odom" | "odom" | 点云 TF 转换的目标坐标系 |
| `waypoint_lookahead` | 2.5 | 2.5 | 航点前视距离 (m) |
| `waypoint_tolerance` | 0.5 | 0.5 | 航点到达容差 (m) |

### 6.2 路径跟踪器 (pathFollower)

| 参数 | 默认值 | 仿真建议值 | 说明 |
|------|--------|-----------|------|
| `lookAheadDis` | 0.5 | 0.4 | 前视距离 (m) |
| `maxSpeed` | 1.0 | 0.5 | 最大速度 (m/s) |
| `maxYawRate` | 45.0 | 2.0 | 最大角速度 (rad/s) |
| `yawRateGain` | 7.5 | 5.0 | 航向比例增益 |
| `stopDisThre` | 0.2 | 0.2 | 到达目标距离阈值 (m) |
| `noRotAtGoal` | true | true | 到达目标后停止旋转 |

---

## 7. 调试工具

### 7.1 RViz2 可视化

一键启动时 RViz2 已自动打开，配置文件 `bringup/config/navigation.rviz` 预置了所有显示：

- **TF** — map/odom/base_link/base_footprint/lidar_link 坐标系
- **RobotModel** — 差速小车 3D 模型
- **Odometry** — 里程计轨迹（箭头样式）
- **GlobalPath** — `/planned_path`（青色）
- **LocalPath** — `/path`（绿色）
- **LaserScan** — `/scan`（红色点）
- **OctoMap** 组 — Occupied/Traversable/Preblocked 体素 + CostCloud

手动启动（如需）：
```bash
rviz2 -d src/bringup/config/navigation.rviz
```

### 7.2 话题监控

```bash
# 查看全局路径
ros2 topic echo /planned_path --once

# 查看局部规划路径
ros2 topic echo /path --once

# 查看速度指令输出
ros2 topic echo /cmd_vel

# 查看里程计
ros2 topic echo /odom --once

# 查看激光扫描
ros2 topic echo /scan --once

# 检查 TF 树
ros2 run tf2_tools view_frames

# 查看话题频率
ros2 topic hz /odom
ros2 topic hz /scan
```

### 7.3 手动发布目标测试

```bash
# 手动设置起点
ros2 topic pub /start_point geometry_msgs/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 0.0, y: 0.0, z: 0.0}}" --once

# 手动发布全局目标
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}" --once

# 手动发布局部目标（给 local_planner）
ros2 topic pub /way_point geometry_msgs/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 1.0, y: 0.0, z: 0.0}}" --once
```

---

## 8. 常见问题

| 问题 | 排查 |
|------|------|
| Gazebo 中小车不动 | 检查 `/cmd_vel` 话题是否有数据 (`ros2 topic echo /cmd_vel`) |
| 小车原地打转 | `localPlanner` 的 `autonomyMode` 未设为 `true` |
| 局部规划器不输出路径 | 检查 `/scan` 是否有数据；`adjacentRange` 是否覆盖到障碍物 |
| 小车撞障碍物 | 减小 `vehicleLength`/`vehicleWidth`，或增大 `vehicleWidthMargin` |
| 全局路径在 Gazebo 中不对齐 | 确认 PCD 地图与 Gazebo 世界坐标原点一致；检查 `map→odom` TF |
| 里程计无数据 | 检查 Gazebo 是否正常运行；`ros2 topic hz /odom` |
| 激光雷达无数据 | 确认 URDF 中 `lidar_link` 传感器配置正确；`ros2 topic hz /scan` |
| 场景无障碍物 | 确认传入了 `pcd_file` 参数，launch 会自动生成世界；或手动运行 `pcd_to_world.py` |
| rosbridge 连不上 | 确认 9090 端口未被占用；检查防火墙 |

---

## 9. 话题完整对照表

```
话题名称                    类型                      来源          目标
─────────────────────────────────────────────────────────────────────────────
/goal_pose                  PoseStamped               Web UI     → octo_planner
/start_point                PointStamped              Web UI     → octo_planner
/start_navigation           Bool                      Web UI     → localPlanner
/stop_navigation            Bool                      Web UI     → pathFollower
/planned_path               Path                      octo_planner → localPlanner
/odom                       Odometry                  Gazebo      → localPlanner, pathFollower
/scan                       LaserScan                 Gazebo      → localPlanner
/path                       Path                      localPlanner → pathFollower
/slow_down                  Int8                      localPlanner → pathFollower
/surrounding_block          Int8                      localPlanner → pathFollower
/cmd_vel                    Twist                     pathFollower → Gazebo
/tf                         TFMessage                 Gazebo      → Web UI / 各节点
/web_cmd_vel                Twist                     Web UI      → pathFollower
```

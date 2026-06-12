# 全闭环导航调试指南

本文档是 [快速入门](quickstart.md) 的下一步，介绍如何在 Gazebo 仿真中运行完整的导航闭环：
**Web 选点 → 全局规划 → 局部避障 → 速度控制 → 小车移动**。

---

## 1. 系统架构总览

```
┌─────────┐    /goal_pose     ┌──────────────┐   /planned_path    ┌────────────────┐
│ Web UI  │ ─────────────────▶│ octo_planner │───────────────────▶│ path_bridge    │
│(选点导航)│                   │  (全局 A*)   │                    │(全局路径→waypoint)│
└─────────┘                   └──────────────┘                    └───────┬────────┘
     │                                                                      │
     │ /tf, odom                                                  /way_point│
     │                                                                      ▼
     │                                                              ┌──────────────┐
     │                                                              │localPlanner  │
     │                                                              │(格栅避障规划) │
     │                                                              └──────┬───────┘
     │                                                           /path     │
     │                                                           /slow_down │
     │                                                                     ▼
     │                                                              ┌──────────────┐
     │                                                              │pathFollower  │
     │                                                              │(纯跟踪控制)  │
     │                                                              └──────┬───────┘
     │                                                           /cmd_vel(TwistStamped)
     │                                                                     │
     │                                                          ┌──────────▼──────────┐
     │                                                          │  cmd_vel_adapter    │
     │                                                          │(TwistStamped→Twist) │
     │                                                          └──────────┬──────────┘
     │                                                           /cmd_vel(Twist)
     │                                                                     │
     │  /tf (odom→base_link)     ┌──────────────┐                         ▼
     │◀──────────────────────────│    Gazebo     │◀────────────── diff_drive_robot
     │                           │ (仿真物理引擎) │
     │                           └──────────────┘
     │                                    │
     │                           /registered_scan (激光点云)
     │                                    │
     │                                    ▼
     │                            localPlanner 感知输入
```

### 数据流说明

| 阶段 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 全局目标 | `/goal_pose` | PoseStamped | Web 发布目标位姿 |
| 全局路径 | `/planned_path` | Path | octo_planner 的 A* 路径（map 坐标系） |
| 局部目标 | `/way_point` | PointStamped | path_bridge 从全局路径提取的当前子目标 |
| 里程计 | `/state_estimation` | Odometry | Gazebo 差速驱动发布的里程计真值 |
| 障碍点云 | `/registered_scan` | Point2 | Gazebo 激光雷达发布的点云 |
| 局部路径 | `/path` | Path | localPlanner 选中的无碰撞格栅路径（base 坐标系） |
| 速度指令 | `/cmd_vel` | Twist | pathFollower 输出（经适配器转为 Gazebo 格式） |

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

### 4.1 仿真 + 全局规划（简易模式，使用 waypoint_follower）

适合初次验证全局路径规划是否正确：

```bash
ros2 launch simulation navigation.launch.py \
  pcd_file:=$HOME/Projects/NavProject/Dog3DNav/maps/building_map.pcd
```

此模式下 `waypoint_follower.py` 直接跟踪 `/planned_path`，不经过局部规划器。
适合验证地图加载、全局规划、Web 交互的基本功能。

### 4.2 全闭环模式（全局 + 局部规划）

需要 4 个组件同时运行：

```bash
# 终端 1: Gazebo 仿真 + 机器人
ros2 launch simulation gazebo.launch.py \
  world:=$(ros2 pkg prefix simulation)/share/simulation/worlds/obstacles.world

# 终端 2: 全局规划器
ros2 run octo_planner octo_planner_node --ros-args \
  -p pcd_file:=$HOME/Projects/NavProject/Dog3DNav/maps/building_map.pcd \
  -p resolution:=0.2 -p robot_radius:=0.25

# 终端 3: 局部规划器
ros2 launch local_planner local_planner.launch.py

# 终端 4: 路径桥接 + 速度适配 + rosbridge
# （见 4.3 节，需要运行 bridge 节点）
```

### 4.3 话题桥接

由于各模块间存在话题名称和类型差异，需要运行桥接节点。在编译好的 install 目录下提供 `nav_bridge.py`：

```bash
# 终端 4: 桥接节点 + rosbridge
ros2 run simulation nav_bridge.py &
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 &
```

桥接节点负责：
1. **全局→局部目标转换**：订阅 `/planned_path`，沿路径等间距提取子目标发布到 `/way_point`
2. **速度指令适配**：订阅 `/cmd_vel`（TwistStamped），转为 Twist 发布到 Gazebo 的 `/cmd_vel`
3. **里程计转发**：订阅 Gazebo 的 `/odom`，发布到 local_planner 需要的 `/state_estimation`
4. **点云转发**：订阅 Gazebo 的 `/scan`（LaserScan），转为 PointCloud2 发布到 `/registered_scan`

### 4.4 使用 PCD 地图生成 Gazebo 场景

如果已有 PCD 点云地图，可直接生成 Gazebo 世界文件：

```bash
# 生成带障碍物的 Gazebo 世界
python3 src/simulation/scripts/pcd_to_world.py \
  maps/building_map.pcd \
  src/simulation/worlds/from_pcd.world \
  --resolution 0.2

# 用生成的世界启动 Gazebo
ros2 launch simulation gazebo.launch.py \
  world:=$(ros2 pkg prefix simulation)/share/simulation/worlds/from_pcd.world
```

> 注意：`pcd_to_world.py` 默认去除最低 Z 层（地面），将剩余体素合并为碰撞 box。
> `--max-boxes 5000` 限制最大 box 数，过大会影响 Gazebo 性能。

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
| `autonomySpeed` | 1.0 | 0.3 | 自主导航速度 (m/s) |
| `maxSpeed` | 1.0 | 0.5 | 最大速度 (m/s) |

### 6.2 路径跟踪器 (pathFollower)

| 参数 | 默认值 | 仿真建议值 | 说明 |
|------|--------|-----------|------|
| `lookAheadDis` | 0.5 | 0.4 | 前视距离 (m) |
| `maxSpeed` | 1.0 | 0.5 | 最大速度 (m/s) |
| `maxYawRate` | 45.0 | 2.0 | 最大角速度 (rad/s) |
| `yawRateGain` | 7.5 | 5.0 | 航向比例增益 |
| `stopDisThre` | 0.2 | 0.2 | 到达目标距离阈值 (m) |
| `noRotAtGoal` | true | true | 到达目标后停止旋转 |

### 6.3 waypoint_follower（简易模式专用）

编辑 `src/simulation/config/sim_params.yaml`：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `lookahead_distance` | 0.4 | 前视距离 (m) |
| `linear_speed` | 0.3 | 线速度 (m/s) |
| `max_angular_speed` | 1.0 | 最大角速度 (rad/s) |
| `goal_tolerance` | 0.2 | 到达判定距离 (m) |

---

## 7. 调试工具

### 7.1 RViz2 可视化

```bash
rviz2 -d src/simulation/config/debug.rviz
# 或手动添加：
# - Map 显示 /octomap_binary (OctoMap)
# - Path 显示 /planned_path (全局路径, 紫色)
# - Path 显示 /path (局部路径, 绿色)
# - PointCloud2 显示 /registered_scan (激光)
# - Odometry 显示 /odom (里程计)
# - TF 显示坐标树
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

# 检查 TF 树
ros2 run tf2_tools view_frames

# 查看话题频率
ros2 topic hz /state_estimation
ros2 topic hz /registered_scan
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
| 小车原地打转 | `local_planner` 的 `autonomyMode` 未设为 `true` |
| 局部规划器不输出路径 | 检查 `/registered_scan` 是否有数据；`adjacentRange` 是否覆盖到障碍物 |
| 小车撞障碍物 | 减小 `vehicleLength`/`vehicleWidth`，或增大 `vehicleWidthMargin` |
| 全局路径在 Gazebo 中不对齐 | 确认 PCD 地图与 Gazebo 世界坐标原点一致；检查 `map→odom` TF |
| `/state_estimation` 无数据 | 确认 bridge 节点在运行，将 `/odom` 转发到 `/state_estination` |
| 激光雷达无数据 | 确认 URDF 中 `lidar_link` 传感器配置正确；`ros2 topic hz /scan` |
| rosbridge 连不上 | 确认 9090 端口未被占用；检查防火墙 |

---

## 9. 话题完整对照表

```
话题名称                    类型                      来源          目标
─────────────────────────────────────────────────────────────────────────────
/goal_pose                  PoseStamped               Web UI     → octo_planner
/start_point                PointStamped              Web UI     → octo_planner
/start_navigation           Bool                      Web UI     → path_bridge
/stop_navigation            Bool                      Web UI     → path_bridge
/planned_path               Path                      octo_planner → path_bridge
/way_point                  PointStamped              path_bridge → localPlanner
/state_estimation           Odometry                  path_bridge → localPlanner
/registered_scan            PointCloud2               path_bridge → localPlanner
/path                       Path                      localPlanner → pathFollower
/slow_down                  Int8                      localPlanner → pathFollower
/surrounding_block          Int8                      localPlanner → pathFollower
/cmd_vel (TwistStamped)     TwistStamped              pathFollower → path_bridge
/cmd_vel (Twist)            Twist                     path_bridge  → Gazebo
/odom                       Odometry                  Gazebo      → path_bridge
/scan                       LaserScan                 Gazebo      → path_bridge
/tf                         TFMessage                 Gazebo      → Web UI / 各节点
```

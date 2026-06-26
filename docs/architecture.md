# Dog3DNav 系统架构

## 总览

```
┌─────────────────────────────────────────────────────────────┐
│  浏览器 (localhost:8000)                                    │
│  Three.js 3D 渲染 ←→ rosbridge WebSocket :9090             │
└──────────┬────────────────────────────────────┬─────────────┘
           │ 发布: /start_point, /goal_point,    │ 订阅: TF, OctoMap
           │       /web_cmd_vel, /start_navigation│       markers, /planned_path
           │       /stop_navigation               │
           ▼                                    ▼
┌──────────────────────┐    ┌──────────────────────────────────┐
│  octo_planner         │    │  local_planner                    │
│  (全局3D路径规划)      │    │  (局部实时避障 + 轨迹跟踪)         │
│                       │    │                                   │
│  PCD → OctoMap → A*  │    │  localPlanner   pathFollower      │
│                       │    │  ┌──────────┐   ┌────────────┐   │
│  入: /start_point     │    │  │343路径×36│   │PurePursuit │   │
│      /goal_point      │    │  │旋转角搜索 │──→│P控制器     │──→ /cmd_vel
│                       │    │  │           │   │            │   │
│  出: /planned_path    │    │  │O(1)碰撞表 │   │/cmd_vel    │   │
│      OctoMap Markers  │    │  └──────────┘   └────────────┘   │
└──────────────────────┘    └──────────────────────────────────┘
           ▲                           ▲
           │    ┌──────────────────────┴────────────────┐
           │    │  仿真 / 传感器                         │
           └────┤  /odom (Odometry)                     │
                │  /scan (LaserScan, C++内部转PC2+TF)   │
                │  /tf (odom→base_footprint→base_link)  │
                └────────────────────────────────────────┘
```

## 模块

### octo_planner — 全局3D规划

节点 `octo_planner_node`，源码 `src/octo_planner/src/octo_planner_node.cpp`。

**流程：**
1. 启动时加载地图（多格式自动检测：`.pcd/.bt/.ot/.world/.sdf`，首次 `.bt` 自动缓存）
2. OctoMap 可通行性分析：地面支撑检测、代价膨胀、禁行区标记
3. 等待 `/goal_point`（或 `/goal_pose`），用 `/start_point`（优先）或 `/odom` 作为起点
4. 3D A* 搜索（`GlobalPlanner::makePlan`）→ `/planned_path`

**关键话题：** 入 `/start_point`、`/goal_point`、`/goal_pose`、`/odom`、`/pcd_file_cmd`；出 `/planned_path`、OctoMap Marker 系列

**规划触发条件（`try_plan()`）：** ① 地图已加载 ② 起点可用（显式 `/start_point` > `/odom` 自动） ③ 终点已设置。三者同时满足才执行 A*。

参数见 `src/octo_planner/config/planner_params.yaml` 和 `src/bringup/config/navigation_config.yaml`。

### local_planner — 局部规划与跟踪

两个独立节点，同一 package（`src/local_planner/`）。

**latticePlanner**（100Hz）：
- 订阅 `/scan`（LaserScan，内部转 PC2+TF）或 `/registered_scan`（PC2）
- 343 条预生成路径 × 36 旋转角 = 12,348 组合评分，O(1) 碰撞查找（`paths/correspondences.txt`）
- 选最优无碰路径 → `/path`（vehicle 帧），发布 `/slow_down`、`/surrounding_block`
- 订阅 `/planned_path` 管理航点推进（`/start_navigation` 激活）

**pathFollower**（100Hz）：
- Pure-pursuit 跟踪 `/path` → `/cmd_vel`（Twist）
- 含安全停车、倾角减速、侧向避障、双向行驶模式

参数见 `src/local_planner/config/local_planner_params.yaml` 和 `src/bringup/config/navigation_config.yaml`。

预生成路径文件：`paths/startPaths.ply`、`paths/paths.ply`、`paths/pathList.ply`、`paths/correspondences.txt`。

### simulation — Gazebo 仿真

**两种模型：**

| 模型 | 传感器 | 里程计 | TF |
|------|--------|--------|-----|
| **car** (差速小车) | LiDAR → `/lidar_points` (lidar_link) | diff_drive 插件 → `/odom` (frame_id=map) | map→base_footprint→base_link |
| **a1** (四足机器人) | LiDAR → `/livox/lidar`, IMU → `/livox/imu` (livox_frame) | A1RLController → `/odom` (frame_id=odom)；slam_mode 时抑制 | 无 SLAM: odom→base_link→trunk→legs + base_link→livox_frame；SLAM: SLAM 提供全链 |

**A1 slam_mode**：A1RLController 的 `publish_odom`/`publish_tf` 设为 false，SLAM（独立启动）提供 `/odom` + TF 全链 `map→odom→livox_frame→base_link`。LiDAR/IMU 位姿对齐实机（x=+0.15 前方, z=+0.13 上方, pitch=-15°）。

离线脚本 `pcd_to_world.py` / `bt_to_world.py` 将地图转为 `.world` 文件（占据体素 → 贪婪合并 → SDF box）。launch 不动态生成世界。

### bringup — 启动配置

纯配置包（无 C++/Python 节点），统一管理 launch 和 config：
- `launch/navigation.launch.py` — 完整导航栈（octo_planner + latticePlanner + pathFollower + rosbridge + RViz2）
- `launch/navigation_no_avoidance.launch.py` — 无避障导航（octo_planner + pathFollower `use_global_path:=true`，跳过 latticePlanner）
- `launch/slam.launch.py` — SLAM 启动（super_lio mapping/relocation + TF 补齐 livox_frame→base_link + map→odom）
- `config/navigation_config.yaml` — 导航栈统一参数（所有节点的 ros__parameters）

### web — 前端

Three.js + ROSBridge（`ws://localhost:9090`）。3D 体素渲染（占据/可通行/禁行/代价四层）、选点导航、地图编辑（笔刷 ± 体素 + 200ms debounce 同步）、虚拟摇杆。

源码 `web/`，操作详见 [开发指南](development.md#web-ui)。

## 数据流

### SLAM 闭环（A1 仿真 → SLAM → 导航）

```
Gazebo (A1)                    SLAM (super_lio)              导航栈
  │── /livox/lidar ──────────────►│                             │
  │── /livox/imu ────────────────►│                             │
  │                                │── /odom ──────────────────►│ (latticePlanner + pathFollower)
  │                                │── /lidar_points ──────────►│ (latticePlanner /registered_scan)
  │                                │── TF: odom→livox_frame ──►│
  │                                │                             │── /cmd_vel ──► A1RLController
```

TF 链：`robot_state_publisher: base_link→trunk→legs` + `slam.launch.py: map→odom(identity) + livox_frame→base_link(static)` + `SLAM: odom→livox_frame(dynamic)` → 完整单链 `map→odom→livox_frame→base_link→trunk→legs`

### 无避障导航（跳过 latticePlanner）

```
octo_planner  /planned_path (map) → pathFollower (use_global_path)  /cmd_vel
                                        │ TF: map→base_link
                                        │ 航点管理 + pure pursuit
```

**pathFollower `use_global_path` 模式：** 订阅 `/planned_path`→TF 变换到 vehicle 帧→去 Z→2D pure pursuit。复用现有 turn-in-place 逻辑（|dirDiff|>0.1rad 时 speed=0 原地转向）。

### Web → 规划 → 控制闭环

```
浏览器                     rosbridge                octo_planner           localPlanner
  │                           │                         │                      │
  │── 订阅 Marker话题 ────────│── 定时推送(5s) ─────────│                      │
  │── /start_point ──────────►│────────────────────────►│                      │
  │── /goal_point ───────────►│────────────────────────►│ 触发A*              │
  │                           │◄─── /planned_path ──────│                      │
  │◄─ /planned_path ─────────│                         │                      │
  │── /start_navigation ─────►│─────────────────────────│─────────────────────►│
  │                           │                         │    /planned_path ───►│
  │── /stop_navigation ───────│─────────────────────────│─────────────────────►│
  │                           │                         │                pathFollower
  │                           │                         │                安全停车
```

### local_planner 闭环

```
Gazebo                          localPlanner              pathFollower
  │── /odom ─────────────────────►│                         │
  │── /scan ──────────────────────►│ (LaserScan→PC2+TF)     │
  │── /planned_path ──────────────►│ (航点管理+lookahead)    │
  │                          100Hz:│ 点云→障碍物→路径评分    │
  │                               │── /path ────────────────►│
  │                               │── /slow_down ───────────►│
  │── /odom ──────────────────────│─────────────────────────►│
  │                               │                   100Hz:│ pure-pursuit
  │                               │                         │── /cmd_vel ──►
```

### 话题对照

完整话题列表见 [开发指南 - 话题对照表](development.md#话题对照表)。

## 坐标系约定

| 帧 | 说明 | 来源 |
|----|------|------|
| `map` | 全局固定坐标系 | SLAM 输出 / 静态 identity→odom |
| `odom` | 里程计坐标系 | SLAM 动态 odom→livox_frame；无 SLAM 时 A1RLController odom→base_link |
| `livox_frame` | LiDAR/IMU 传感器帧 | 不在 URDF 内；SLAM 模式由 slam.launch.py 发布 livox_frame→base_link，非 SLAM 模式由 gazebo.launch.py 发布 base_link→livox_frame |
| `base_link` | 机器人本体根帧 | A1 URDF 根链接（无 base_footprint）；car URDF 中为 base_footprint 子帧 |
| `trunk` | 机身物理中心 | URDF floating_base 固定关节（base_link 子帧） |

## 关键机制

### 在线增量 OctoMap 更新

**流程：** 实时点云 `/lidar_points` → TF 变换到 map 帧 → `updateNode()` + `updateInnerOccupancy()` 写入 OcTree → 定时（默认 60s）`reanalyze()` 重分析可通行区 → `republish_all()` 更新 Marker。

**实现位置：**
- `octo_planner_node.cpp` → `on_online_cloud()`：TF 变换 + 体素过滤（空间范围 + 保守模式 Z 补偿）+ 写入 OcTree
- `octo_planner_node.cpp` → `on_online_reanalyze()`：后台线程 `reanalyze()` + 重新发布
- 支持两种模式：raycasting（`insertPointCloud`）和手动 `updateNode`

**配置：** `navigation_config.yaml` 中 `online_update_enabled`（默认 `false`）+ `online_update_period_s/occupied_prob/use_raycasting/conservative_mode/min_interval_ms/downsample_step/max_xy_distance/max_z_above/max_z_below`

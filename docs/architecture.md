# Dog3DNav 系统架构

## 总览

```
┌─────────────────────────────────────────────────────────────┐
│  浏览器 (localhost:8080)                                    │
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
           │                           │
           │    ┌──────────────────────┴────────────────┐
           │    │  仿真 / 传感器                         │
           └────┤  /odom (Odometry, remap→/state_estimation) │
                │  /scan (LaserScan, C++内部转PointCloud2)  │
                │  /tf (odom→base_footprint→base_link)     │
                └────────────────────────────────────────┘
```

## 模块详解

### 1. 地图预处理 — `src/bringup/maps/map_preprocessor.py`

**定位：** 离线工具，不参与在线导航。

**输入：** 原始 SLAM 点云 (PCD)

**处理流水线：**
```
原始PCD → [体素降采样] → RANSAC地面提取 → 重力对齐旋转
→ 法向量统计墙面方向 → XY轴旋转对齐 → 平移原点 → [地面补全] → 输出PCD
```

**关键输出：** 对齐后的 `map_nav3d.pcd`，坐标系满足：
- XY 平面平行于建筑墙面
- Z 轴垂直于地面
- 原点 = 地面中心点

**与 octo_planner 的衔接：** 预处理后的 PCD 直接作为 `octo_planner_node` 的地图输入。首次加载自动生成 `.bt` 缓存，后续启动直接加载 `.bt`。

---

### 2. 全局规划 — `octo_planner`

**节点：** `octo_planner_node`

**工作逻辑：**

1. 启动时加载地图文件（多格式自动检测），转为 OctoMap（3D 占据栅格）
2. 基于 OctoMap 构建可通行性 map：地面支撑检测、代价膨胀、禁行区标记
3. 等待 Web 前端下发起点 `/start_point` 和终点 `/goal_point`
4. 收到终点后触发 `GlobalPlanner::makePlan()`（3D A* 搜索）
5. 将规划结果 `/planned_path` 发送到 Web 前端展示

**话题接口：**

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 入 | `/start_point` | PointStamped | 起点(map帧) |
| 入 | `/goal_point` | PointStamped | 终点(map帧) |
| 入 | `/goal_pose` | PoseStamped | 终点(含朝向) |
| 入 | `/pcd_file_cmd` | String | 动态切换地图文件 |
| 出 | `/planned_path` | Path | 全局路径(map帧) |
| 出 | `/octomap` | Octomap | 完整OctoMap(transient_local) |
| 出 | `/octomap_occupied_markers` | Marker | 占据体素(橘色) |
| 出 | `/traversable_cells_markers` | Marker | 可通行体素(绿色) |
| 出 | `/preblocked_cells_markers` | Marker | 禁行体素(蓝色) |
| 出 | `/risk_cost_cells` | PointCloud2 | 代价云(intensity=代价) |

**关键参数：**

| 参数 | 默认 | 含义 |
|------|------|------|
| `resolution` | 0.2 | OctoMap体素分辨率(m) |
| `robot_radius` | 0.05 | 碰撞检测半径(m) |
| `max_iterations` | 500000 | A*搜索上限 |
| `require_ground_support` | true | 要求地面支撑 |
| `enable_preblocked_costmap` | true | 代价膨胀 |
| `octomap_publish_period_s` | 5.0 | 定时重发周期(s) |

---

### 3. 局部规划与跟踪 — `local_planner`

包含两个独立节点，共用同一 package。

#### 3.1 localPlanner — Lattice 局部规划

**节点：** `localPlanner`

**工作逻辑（100Hz循环）：**

1. 将传感器点云变换到 vehicle 坐标系
2. 检测周围6个方向的障碍物遮挡位掩码 → `/surrounding_block`
3. 对 343 条预生成路径 × 36 个旋转角 = 12,348 种组合逐一评分：
   - 通过预计算的 72,611 体素对应表 (`correspondences.txt`) 实现 O(1) 碰撞查找
   - 每条路径得分 = 方向差异权重 × 旋转角权重 × 分组权重
4. 选择得分最高的无碰路径 → `/path`
5. 统计可通行路径数 + 地面代价 → `/slow_down`（0~3级慢行）
6. 无满意结果时缩小路径缩放/范围重试

**话题接口：**

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 入 | `/state_estimation` | Odometry | 机器人位姿（通过 remap 订阅 `/odom`） |
| 入 | `/scan` | LaserScan | 激光扫描（use_laser_scan=true 时） |
| 入 | `/registered_scan` | PointCloud2 | 激光点云（use_laser_scan=false 时） |
| 入 | `/planned_path` | Path | 全局路径（use_planned_path=true 时） |
| 入 | `/start_navigation` | Bool | 激活航点推进 |
| 入 | `/way_point` | PointStamped | 导航目标点（use_planned_path=false 时） |
| 入 | `/terrain_map` | PointCloud2 | 地形分析云 |
| 入 | `/joy` | Joy | 手柄控制 |
| 入 | `/speed` | Float32 | 速度覆盖 |
| 入 | `/navigation_boundary` | PolygonStamped | 虚拟边界 |
| 入 | `/added_obstacles` | PointCloud2 | 手动障碍物 |
| 入 | `/check_obstacle` | Bool | 避障开关 |
| 出 | `/path` | Path | 局部路径(vehicle帧) |
| 出 | `/slow_down` | Int8 | 减速等级(0~3) |
| 出 | `/surrounding_block` | Int8 | 6-bit遮挡掩码 |
| 出 | `/free_paths` | PointCloud2 | 无碰路径可视化 |

**关键参数（输入模式）：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_laser_scan` | `false` | true=订阅 `/scan`(LaserScan)，C++内部转 PointCloud2+TF |
| `use_planned_path` | `false` | true=订阅 `/planned_path`(Path)，自行管理航点推进 |
| `global_frame_id` | `"odom"` | 障碍物点云 TF 转换的目标坐标系 |
| `waypoint_lookahead` | `2.5` | 航点前视距离 (m) |
| `waypoint_tolerance` | `0.5` | 航点到达容差 (m) |

**预生成路径文件（`paths/`）：**

| 文件 | 内容 |
|------|------|
| `startPaths.ply` | 7 组种子路径（按曲率/方向分组） |
| `paths.ply` | 343 条完整路径几何点 |
| `pathList.ply` | 每条路径的终点位姿 + 所属组 |
| `correspondences.txt` | 72,611 个网格体素 → 被阻挡路径ID的映射表 |

#### 3.2 pathFollower — Pure-Pursuit 轨迹跟踪

**节点：** `pathFollower`

**工作逻辑（100Hz循环）：**

1. 接收 `/path`（vehicle 帧），记录发布时刻的参考位姿
2. 将当前位姿转换为相对参考位姿的增量位移
3. 从路径起点扫描，找到第一个距离 > `lookAheadDis` 的航点
4. 计算航向角误差 `dirDiff = 当前yaw - 参考yaw - pathDir`
5. 双向行驶模式：当误差 > 90° 时自动切换前进/后退方向
6. P 控制器输出横摆角速度：
   - 行驶中：`yawRate = -yawRateGain × dirDiff`
   - 静止时：`yawRate = -stopYawRateGain × dirDiff`
7. 速度 ramping：加速度限制平滑逼近目标速度
8. 多级减速：倾角减速、`/slow_down` 指令、路径末端减速
9. 应急停止：`/stop` + 倾角超限

**话题接口：**

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 入 | `/state_estimation` | Odometry | 机器人位姿（通过 remap 订阅 `/odom`） |
| 入 | `/path` | Path | 跟随路径(vehicle帧) |
| 入 | `/joy` | Joy | 手柄 |
| 入 | `/speed` | Float32 | 速度覆盖 |
| 入 | `/stop` | Int8 | 急停(1=停车,2=也停转向) |
| 入 | `/stop_navigation` | Bool | 导航停止（Web UI 下发，安全停车） |
| 入 | `/slow_down` | Int8 | 慢行等级 |
| 入 | `/surrounding_block` | Int8 | 周边遮挡位掩码 |
| 出 | `/cmd_vel` | Twist | 速度指令(vehicle帧) |

---

### 4. Web 前端 — `web/`

**定位：** 用户交互 + 3D 可视化

**通信：** rosbridge WebSocket (默认 `ws://localhost:9090`)

**用户操作 → ROS 指令映射：**

| 用户操作 | 发布话题 | 说明 |
|----------|----------|------|
| 点击「设置起点」→ 地图选点 | `/start_point` | 发送到 octo_planner |
| 点击「设置终点」→ 地图选点 | `/goal_point` + `/goal_pose` | 发送到 octo_planner |
| 点击「导航目标」→ 选点 → 弹窗确认 | `/goal_point` + `/start_navigation` | 触发全局规划 |
| 点击「停止导航」 | `/stop_navigation` | 中止导航 |
| 拖拽虚拟摇杆 | `/web_cmd_vel` | 手动控制(80ms间隔) |
| 滑动旋转条 | `/web_cmd_vel` | 原地旋转 |

**ROS 数据 → 3D 渲染映射：**

| 订阅话题 | 渲染内容 | 颜色 |
|----------|----------|------|
| `/octomap_occupied_markers` | 占据体素立方体 | 橘色 |
| `/traversable_cells_markers` | 可通行体素(可选点) | 绿色半透明 |
| `/preblocked_cells_markers` | 禁行区体素 | 蓝色半透明 |
| `/risk_cost_cells` | 代价风险云 | 蓝色(按cost调不透明度) |
| `/planned_path` | 全局路径管状线 | 紫色 |
| `/tf` + `/tf_static` | 机器人3D模型(狗) | 青色 |

**选点交互：**
- 射线检测 (`Raycaster`) 与可通行体素的 InstancedMesh 求交
- 确定选中点后，通过拖拽方向计算朝向 (yaw)
- 起点以绿球标记、终点以红球标记

---

## 典型工作流

### 离线建图流程

```
SLAM原始点云
    │
    ▼
map_preprocessor.py  ──→  map_nav3d.pcd (位于 src/bringup/maps/)
(对齐+降采样+补全)
```

### 在线导航流程（Web → 规划 → 仿真闭环）

```
终端1: ros2 launch bringup navigation.launch.py pcd_file:=/path/to/map.pcd
       (一键启动: Gazebo + octo_planner + localPlanner + pathFollower + rosbridge + RViz2)
```

```
浏览器                     rosbridge                octo_planner           localPlanner
  │                           │                         │                      │
  │── 订阅 Marker话题 ────────│── 定时推送(5s) ─────────│                      │
  │   地图体素渲染             │                         │                      │
  │                           │                         │                      │
  │── /start_point ──────────►│────────────────────────►│ 记录起点              │
  │── /goal_point ───────────►│────────────────────────►│ 记录终点 + 触发A*    │
  │                           │                         │                      │
  │                           │◄─── /planned_path ──────│                      │
  │◄─ /planned_path ─────────│                         │                      │
  │   紫色路径线渲染           │                         │                      │
  │                           │                         │                      │
  │── /start_navigation ─────►│─────────────────────────│─────────────────────►│
  │                           │                         │      激活航点推进     │
  │                           │                         │                      │
  │                           │                         │    /planned_path ───►│
  │                           │                         │    (航点管理+避障)   │
  │                           │                         │                      │
  │── /stop_navigation ───────│─────────────────────────│──(停止)──────────────►│
  │                           │                         │                pathFollower
  │                           │                         │                安全停车
```

### local_planner 闭环控制流程

```
Gazebo/Odometry                  localPlanner              pathFollower        机器人
    │                               │                         │                 │
    │── /odom ──(remap)────────────►│                         │                 │
    │── /scan ──────────────────────►│                         │                 │
    │   (LaserScan→PC2+TF 内部转换)  │                         │                 │
    │                               │                         │                 │
    │── /planned_path ──────────────►│(航点管理+lookahead)     │                 │
    │── /start_navigation ──────────►│                         │                 │
    │                               │                         │                 │
    │                         100Hz:│ 点云→障碍物grid          │                 │
    │                         路径评分→选最优                 │                 │
    │                               │                         │                 │
    │                               │── /path ────────────────►│                 │
    │                               │── /slow_down ───────────►│                 │
    │                               │── /surrounding_block ────►│                 │
    │                               │                         │                 │
    │── /odom ──(remap)─────────────│─────────────────────────►│                 │
    │── /stop_navigation ───────────│─────────────────────────►│                 │
    │                               │                   100Hz:│ pure-pursuit    │
    │                               │                   P控制 │                 │
    │                               │                         │── /cmd_vel ────►│
```

---

## 当前集成状态

| 链路 | 状态 | 说明 |
|------|------|------|
| Web → octo_planner 规划 | ✅ 已打通 | `/start_point` + `/goal_point` → `/planned_path` |
| Web 地图可视化 | ✅ 已打通 | Marker 体素分层渲染 |
| Web 机器人位姿显示 | ✅ 已打通 | TF 解析 + 3D模型 |
| Web 手动控制 → 机器人 | ✅ 已打通 | Web 发 `/web_cmd_vel`(Twist)，pathFollower 订阅后透传 |
| localPlanner → pathFollower | ✅ 已打通 | `/path` + `/slow_down` + `/surrounding_block` |
| octo_planner → localPlanner | ✅ 已打通 | `/planned_path`(Path) 直接订阅，localPlanner 内部管理航点推进 |
| localPlanner 感知 | ✅ 已打通 | `/scan`(LaserScan) 内部转 PointCloud2 + TF→odom |
| pathFollower → Gazebo | ✅ 已打通 | `/cmd_vel`(Twist) 直接驱动 diff_drive 插件 |
| pathFollower → 真实机器人 | ⚠️ 待对接 | `/cmd_vel` 到真实电机驱动接口（串口或 DDS） |

> **注意：** 项目中不存在 Python 中继节点。所有里程计转发、点云格式转换、航点管理、速度适配等逻辑已下沉到 localPlanner/pathFollower (C++) 或通过 launch remap 解决。

## 坐标系约定

| 帧 | 说明 | 使用者 |
|----|------|--------|
| `map` | 全局固定坐标系 | octo_planner 输入/输出 |
| `odom` | 里程计坐标系 | TF 链中间帧 |
| `vehicle` | 机器人本体坐标系 | localPlanner 路径输出、pathFollower 指令输出 |
| `base_link` | 机器狗本体帧 | Web 前端 3D 模型定位 |

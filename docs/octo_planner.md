# OctoPlanner — 全局 3D 路径规划器

> 基于 OctoMap 的 3D A* 全局路径规划。负责地图加载/转换、可通行性分析、路径搜索和在线增量更新。

## 1. 整体架构

```
octo_planner_node (ROS 2 Node)
├── Pcd2OctomapConverter   ←  PCD → OctoMap 转换管线
├── GlobalPlanner          ←  可通行性分析 + 3D A* 搜索
├── world_loader           ←  .world / .sdf → OctoMap
└── 在线更新模块           ←  实时点云 → OctoMap 增量更新
```

**底层库**: `OctoPlanner3D/`（以 STATIC 库形式编译进节点）

**源文件结构**:

| 文件 | 职责 |
|------|------|
| `src/octo_planner_node.cpp` | ROS 2 封装：参数声明、话题/服务、生命周期、在线更新 |
| `thirdparty/.../pcd2octomap_converter.cpp` | PCD 点云 → OctoMap 转换 |
| `thirdparty/.../global_planner.cpp` | 可通行性分析 + 3D A* 规划 |
| `src/world_loader.cpp` | Gazebo `.world` / `.sdf` → OctoMap |

---

## 2. 地图加载管线

### 2.1 多格式输入

节点通过 `load_map_auto()` 自动检测文件扩展名并路由到对应加载器：

```
.pcd  ──→ load_pcd_map()     → Pcd2OctomapConverter 转换管线
.bt   ──→ load_bt_map()      → octomap::OcTree 直接反序列化
.ot   ──→ load_ot_map()      → octomap::AbstractOcTree::read()
.world/.sdf ──→ load_world_map()  → world_loader 解析 SDF 几何体
```

### 2.2 .bt 缓存机制

对于 `.pcd` / `.world` / `.sdf` 源文件，优先检查同目录下是否存在**更新的** `.bt` 缓存：
- 存在且比源文件新 → 直接加载 `.bt`，跳过转换
- 不存在或比源文件旧 → 执行完整转换，完成后自动保存 `.bt`（`auto_save_bt: true`）

### 2.3 在线模式（无初始地图）

当 `pcd_file` 为空且 `online_update_enabled: true` 时，创建一个**空白 OctoMap**，完全依赖 lidar 点云在线填充。此时只执行可通行性分析（已分析过的区域），不执行路径规划（因为没有初始占据信息）。

---

## 3. PCD → OctoMap 转换算法

`Pcd2OctomapConverter::convert()` 执行五阶段管线：

```
PCD 文件
  │
  ▼
① loadPointCloud()         — PCL 读取点云
  │
  ▼
② buildVoxelCounts()       — 按 resolution 离散化，统计每体素点数
  │
  ▼
③ filterByPointCount()     — 点数 < min_points_per_voxel 的体素丢弃（去噪）
  │
  ▼
④ filterByConnectedClusters() — 26 连通域分析，体素数 < min_cluster_voxels 的簇丢弃（去除孤立噪点）
  │
  ▼
⑤ fillOcTree()             — 剩余体素写入 OcTree（updateNode → occupied）
  │
  ▼
⑥ groundInfill() (可选)    — 地面补全（填补地面空洞）
  │
  ▼
OctoMap (.bt)
```

### 3.1 体素计数（buildVoxelCounts）

遍历所有 PCD 点，对每个点计算 OctoMap key (`coordToKeyChecked`)，在 `unordered_map<Key, count>` 中累加。跳过 NaN 点。

### 3.2 体素点数过滤（filterByPointCount）

仅保留点数 ≥ `min_points_per_voxel`（默认 3）的体素。低密度体素视为传感器噪声或树叶等动态物体，直接丢弃。

### 3.3 连通域过滤（filterByConnectedClusters）

对通过的体素做 **26-邻域 BFS 连通域分析**。体素数 < `min_cluster_voxels`（默认 2）的孤立小簇被丢弃。这一步去除落在空中的离散噪点簇。

### 3.4 地面补全（groundInfill）

PCD 中地面可能因采样稀疏出现空洞。groundInfill 在 **每一列** (x,y) 上从最低的 occupied 体素开始向上检查：如果某体素 Z 下方有足够密度的 occupied 邻居（`ground_infill_density_threshold`），则该体素也被标记为 occupied。

**参数**:
| 参数 | 默认 | 说明 |
|------|------|------|
| `enable_ground_infill` | `true` | 是否启用地面补全 |
| `ground_infill_density_threshold` | `0.02` | 邻居密度阈值 |
| `ground_infill_neighbor_threshold` | `3` | 邻居数量阈值 |

---

## 4. 可通行性分析

`GlobalPlanner::setOctomap()` 和 `reanalyze()` 都会触发完整的场景分析流水线。在线更新期间通过 `rebuildFromSnapshot()` 入口，先持有 `octree_mutex_` 快照 `occupied_set_` 再释放锁，后续步骤只持 `derived_mutex_`：

```
setOctomap() / reanalyze() / rebuildFromSnapshot()
  │
  ├──⓪ 快照 occupied_set_ ← octree 叶子节点（仅 octree_mutex_）
  ├──① rebuildPreblockedCells()    — 障碍物贴身空体素标记（查 occupied_set_）
  ├──② rebuildDerivedLayers()      — 可通行体素提取（查 occupied_set_）
  ├──③ radicalInfill() (可选)      — 激进填充桥接
  ├──④ flattenTraversable() (可选) — 可通行面平滑
  └──⑤ rebuildPreblockedCostmap()  — 软代价地图构建
```

> **`occupied_set_`** 是 A* 与点云写入解耦的核心：所有占据查询走快照，A* 不再持有 `octree_mutex_`，与 online cloud 零等待并发。详见 [11. 线程模型](#11-线程模型与锁设计)。

### 4.1 可通行体素提取（rebuildDerivedLayers）

**核心思想**：可通行体素 = 位于占据体素**正上方**的空体素（有地面支撑）。

```
对每个 occupied 体素 O:
  检查 O 上方第 1 层的空体素 C (x+dx, y+dy, z+1):
    条件 1: C 在 bounds 内
    条件 2: C 本身不是 occupied
    条件 3: C 满足 isCellTraversable() → 可通行
```

**`isCellTraversable()` 检查项**：

| 检查 | 说明 |
|------|------|
| **边界检查** | 在 metric bounds 内 |
| **地面支撑** (`require_ground_support`) | 正下方必须有 occupied 体素 |
| └ 严格模式 (`strict_direct_ground_support`) | 仅检查正下方 (0,0,-1) |
| └ 宽松模式 | 在 `support_depth_cells × support_xy_radius_cells` 范围内检查 |
| **硬 Preblocked** (`preblocked_hard_obstacle`) | 下方不能有 preblocked cell（贴边障碍物） |
| **机器人碰撞检测** | 以体素为中心，`robot_radius` 半径的半球（dz≥0）内无 occupied 体素 |

**`lowest_traversable_only: false`**（默认）：每个 (x,y) 列上可能存在多层可通行体素（如桥梁的上层和下层）。

**`lowest_traversable_only: true`**：每列 (x,y) 仅保留 Z 最低的可通行体素（适合平面导航）。

**两种地面支撑模式**：
- `strict_direct_ground_support: true`（默认）— 仅检查正下方 (x,y,z-1)，快速严格
- `strict_direct_ground_support: false` — 在 Z 方向 `support_depth_cells` 层、XY 方向 `support_xy_radius_cells` 格范围内寻找支撑

### 4.2 Preblocked Cells（贴身障碍标记）

详见 `rebuildPreblockedCells()`（第 449 行）。两个独立开关：

| 开关 | 作用 |
|------|------|
| `preblocked_hard_obstacle: true` | 硬阻挡：如果体素下方有 preblocked cell，则体素不可通行 |
| `enable_preblocked_costmap: true` | 软代价：preblocked 周围衰减代价（`preblocked_costmap_weight=2.5`），A* 倾向于绕行 |

**运算**：遍历 occupied 叶子，取其 XY 8 邻域空体素 → 判别是否为障碍物"贴身"区域 → 标记为 preblocked。

**代价地图**：以每个 preblocked 为中心，在 `preblocked_costmap_radius_cells`（默认 3）半径内，给可通行体素赋衰减代价 `(radius+1 - d) / (radius+1)`。A* 中：
```
tentative_g += preblocked_costmap_weight × preblocked_cost
```

### 4.3 Radical Infill（激进填充）

**目的**：桥接被小间隙分隔的可通行区域。当两片可通行区域之间存在窄缝（非 occupied 的空隙），且间隙尺寸 < `radical_infill_radius_m`，则将间隙填充为可通行。

**参数**:
| 参数 | 默认 | 说明 |
|------|------|------|
| `radical_infill_enabled` | `true` | 是否启用 |
| `radical_infill_radius_m` | `1.0` | 桥接半径 |
| `radical_infill_clearance_m` | `1.0` | 填充体素与障碍物的最小间距 |
| `radical_infill_half_height_m` | `0.1` | 填充半高 |

**算法**：
1. 对可通行体素做 26-连通域分析，找出所有独立"可通行岛屿"
2. 检查各岛屿之间的间距，间距 < `radial_infill_radius_m` 的生成桥接候选
3. 桥接候选与障碍物保持 `clearance_m` 间距
4. 将通过的候选加入 `traversable_cells_`

### 4.4 Flatten（可通行面平滑）

**目的**：中值滤波去除可通行面的高频噪声，同时保留楼梯/坡道。

**算法**：对每个 (x,y) 列，在 `flatten_window_cells × flatten_window_cells` 窗口内取可通行体素 Z 的中值。若原始 Z 与中值之差 ≤ `flatten_max_delta_cells` 格，则用中值替换。差异超过阈值的保留原值（如楼梯踏步）。

---

## 5. 3D A* 路径搜索

### 5.1 搜索空间

- **节点**: GridIndex (x, y, z) 整数坐标
- **邻域**: 26 方向（3×3×3 不含自身）—— 体素中心间距 = `resolution`
- **启发函数**: 欧几里得距离 `h(n) = ‖n - goal‖`
- **代价函数**: `f(n) = g(n) + h(n)`，其中 g 为累计欧几里得距离 + preblocked 软代价

### 5.2 搜索过程（startPlan）

```
① 世界坐标 → GridIndex (worldToGrid: floor(x/res))
② 起/终点 Snap: 若在 occupied 上 → findNearestFreeCell() 在 snap_search_radius_cells 内搜索最近可通行格
③ A* 主循环:
   - 优先队列（f 值最小堆）
   - 展开当前节点的 26 邻域
   - 对每个邻居: isCellTraversable() → g + step_cost + preblocked_cost → 更新
   - 到达 goal 或超过 max_iterations 或 cancel_flag 触发 → 退出
④ 路径重建: came_from 回溯 → 世界坐标序列
⑤ 发布 nav_msgs/Path
```

### 5.3 取消机制

A* 循环每轮检查 `cancel_flag_`。新规划请求到达时，ROS 节点设置 `cancel_planning_ = true`，当前 A* 在微秒级内退出。

### 5.4 持久化 Worker 线程

`planning_worker_loop()` 在独立线程中运行，通过 `condition_variable` 等待规划请求。避免阻塞 ROS spin 线程。

**定时重规划**（`replan_period_s` > 0）: 创建 WallTimer 周期性触发 A*，适用于机器人移动中需要不断更新路径的场景。定时重规划不清空旧路径（避免打断 latticePlanner 的 freeze 恢复状态）。

---

## 6. 在线增量更新

### 6.1 概述

通过 lidar 实时点云增量更新 OctoMap，使地图随环境和机器人移动而更新。与一次性加载的静态地图互补。

### 6.2 数据处理流程

```
/lidar_points (PointCloud2, lidar 帧)
  │
  ▼  TF 变换 → map 帧
  │
  ▼  去采样 (downsample_step)
  │
  ├──→ Phase 1: 射线清空 (仅 raycasting 模式)
  │      computeRayKeys(sensor, point) → 体素列表 (不含终点)
  │      updateNode(每个体素, prob_miss_log) → 标记 free
  │
  └──→ Phase 2: 占据标记 (两种模式共用)
        过滤: z_above / z_below / xy_distance
        通过 → updateNode(point, prob_hit_log) → 标记 occupied
```

### 6.3 两阶段设计理由

**Phase 1（射线清空）**: 激光束从传感器到每个点之间穿过的空间 → 沿途体素标记为 free。使用**全部点**（包括远处/高处），因为即使终点超出范围，射线本身仍包含有效的 free-space 信息。

**Phase 2（占据标记）**: 只对范围内（`max_z_above` / `max_z_below` / `max_xy_distance`）的点做占据。屋顶/远处点只参与清空，不参与占据。

### 6.4 Raycasting 模式 vs Manual 模式

| 模式 | `use_raycasting` | Phase 1 | Phase 2 | 适用场景 |
|------|-----------------|---------|---------|---------|
| Raycasting | `true` | ✓ 射线清空 free space | ✓ 占据范围内点 | 推荐，清空+占据解耦 |
| Manual | `false` | ✗ 跳过 | ✓ 占据范围内点 | 简单场景，不清空 |

### 6.5 Conservative 模式

`online_update_conservative_mode: true` 将占据点 Z 坐标下移半个分辨率（`0.5 × resolution`），使障碍物标记在"根部"而非精确命中点。适用于激光点恰好打在障碍物边缘的场景。

### 6.6 定时重分析

`on_online_reanalyze()` 每 `online_update_period_s`（10s）触发一次，在后台线程执行 `planner_->reanalyze()` 并重新发布所有可视化数据。

### 6.7 相关参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `online_update_enabled` | `true` | 启用在线更新 |
| `online_update_cloud_topic` | `/lidar_points` | 输入点云话题 |
| `online_update_period_s` | `10.0` | 重分析周期 |
| `online_update_occupied_prob` | `0.8` | 占据概率 |
| `online_update_use_raycasting` | `true` | 启用射线清空 |
| `online_update_conservative_mode` | `true` | 保守模式 |
| `online_update_min_interval_ms` | `500` | 最小处理间隔（防抖） |
| `online_update_downsample_step` | `1` | 降采样步长（1=不降采样） |
| `online_update_max_xy_distance` | `10.0` | XY 最大距离 |
| `online_update_max_z_above` | `1.0` | 传感器上方最大高度 |
| `online_update_max_z_below` | `5.0` | 传感器下方最大深度 |

---

## 7. Web 地图编辑接口

### 7.1 增删占据体素

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/add_occupied_voxels` | `PointCloud2` | Web → 节点 | 添加占据体素（`setNodeValue(1.5)`） |
| `/remove_occupied_voxels` | `PointCloud2` | Web → 节点 | 删除占据体素（`setNodeValue(-1.5)`） |

操作后自动调用 `updateInnerOccupancy()` 和 `republish_all()`。

### 7.2 地图保存/加载

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/save_octomap_path` | `String` | Web → 节点 | 保存当前 OctoMap 为 .bt |
| `/load_map_file` | `String` | Web → 节点 | 加载地图文件 |
| `/pcd_file_cmd` | `String` | Web → 节点 | 别名，同 `/load_map_file` |

### 7.3 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/request_map` | `Trigger` | 触发 reanalyze + republish，返回当前地图状态 |

---

## 8. 可视化输出

### 8.1 发布话题

| 话题 | 类型 | 颜色 | 说明 |
|------|------|------|------|
| `/octomap` | `Octomap` | — | 完整 OctoMap 二进制 |
| `/octomap_occupied_markers` | `Marker(CUBE_LIST)` | 红色 | 占据体素 |
| `/traversable_cells_markers` | `Marker(CUBE_LIST)` | 绿色 | 可通行体素 |
| `/preblocked_cells_markers` | `Marker(CUBE_LIST)` | 蓝紫色 | Preblocked 体素 |
| `/risk_cost_cells` | `PointCloud2` | 热度图 | 软代价云（preblocked 代价） |
| `/planned_path` | `Path` | — | A* 规划路径 |

### 8.2 Chunked 发布

大场景下 Marker 点数量巨大，采用 **chunked 分片** 方式发布，每片最多 `5000` 个点，通过 `marker.id` 区分 chunk。

---

## 9. 话题/服务完整对照

### 9.1 订阅（输入）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/start_point` | `PointStamped` | 显式设置起点 |
| `/goal_point` | `PointStamped` | 设置目标点（位置） |
| `/goal_pose` | `PoseStamped` | 设置目标点（位姿，Web UI 下发含航向） |
| `/odom` | `Odometry` | 里程计（在线更新传感器位置 + 自动起点） |
| `/add_occupied_voxels` | `PointCloud2` | 添加占据体素 |
| `/remove_occupied_voxels` | `PointCloud2` | 删除占据体素 |
| `/save_octomap_path` | `String` | 保存地图 |
| `/load_map_file` | `String` | 加载地图 |
| `/pcd_file_cmd` | `String` | 加载地图（别名） |
| `/lidar_points` | `PointCloud2` | 在线增量点云（话题名可配） |

### 9.2 发布（输出）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/octomap` | `Octomap` | 完整 OctoMap |
| `/octomap_occupied_markers` | `Marker` | 占据体素可视化 |
| `/traversable_cells_markers` | `Marker` | 可通行体素可视化 |
| `/preblocked_cells_markers` | `Marker` | Preblocked 体素可视化 |
| `/risk_cost_cells` | `PointCloud2` | 软代价云 |
| `/planned_path` | `Path` | 规划路径 |
| `/tf` (通过 `tf2_ros::TransformListener`) | — | 在线更新中激光点云 TF 变换 |

---

## 10. 坐标系

| 坐标系 | 来源 | 说明 |
|--------|------|------|
| `map` | 参数 `frame_id` | 全局固定坐标系，所有输出话题的 `frame_id` |
| `odom` | `/odom` 话题 | 里程计，online update 中用于确定传感器位置 |
| `cloud_frame` | `/lidar_points` header | 激光点云原始帧，在线更新时通过 TF 转到 map |

---

## 11. 线程模型与锁设计

### 11.1 线程架构

```
Spin 线程 (ROS 2 主循环):
  ├── 话题回调: on_start, on_goal, on_goal_pose, on_odom
  ├── 在线更新: on_online_cloud (Phase 1 ray-clearing + Phase 2 occupy)
  ├── 地图编辑: on_add_voxels, on_remove_voxels
  ├── Timer: replan_timer (每 1.5s → 通知 Worker)
  └── Timer: online_update_timer (每 10s → 启动 detached reanalyze)

Worker 线程 (planning_worker_loop):
  └── 等待 condition_variable → makePlan() → publish path

Reanalyze detached 线程 (每次 on_online_reanalyze 新开):
  ├── Phase A: 持有 octree_mutex_ → 快照 occupied_set_ → 释放 (~10ms)
  ├── Phase B: 持有 derived_mutex_ → rebuildFromSnapshot() → 释放 (~1-5s)
  └── Phase C: 持有双锁 → republish_all() → 释放 (~10ms)
```

### 11.2 锁设计

两把 `std::mutex`，替代原来的一把 `recursive_mutex`：

| 锁 | 保护对象 | 持有者 |
|----|---------|--------|
| `octree_mutex_` | `octree_` 树结构读写 | online cloud（写）、add/remove voxels（写）、reanalyze Phase A（快照读）、reanalyze Phase C（republish 读）、configure_planner（初始化写） |
| `derived_mutex_` | `occupied_set_`, `traversable_cells_`, `preblocked_cells_`, `preblocked_costmap_` | A* Worker（读快照 + derived）、reanalyze Phase B（写 derived）、reanalyze Phase C（republish 读）、configure_planner（初始化写） |

**锁序**: 需要同时持有时，始终先 `octree_mutex_` 后 `derived_mutex_`，避免死锁。

### 11.3 Occupied 快照 —— A* 与点云解耦的关键

A* 路径搜索中需要查询"某个体素是否被占据"来做碰撞检测和地面支撑检查。如果直接查 OctoMap 树（`octree_->search()`），就必须持有 `octree_mutex_`，与点云写入互斥。

**方案**: `GlobalPlanner` 维护一个 `occupied_set_`（`unordered_set<GridIndex>`），由 reanalyze 从 OctoMap 叶子节点快照生成。A* 中所有占据查询（`isOccupiedCell`、`hasGroundSupport`、机器人碰撞检测）全部改为查 `occupied_set_`，不再碰 OctoMap。

```
reanalyze (每 10s):             A* 搜索 (实时):
  octree 叶子 ──快照──→            occupied_set_
  occupied_set_           ←────────── 读 ── isOccupiedCell()
                            ←────── 读 ── hasGroundSupport()
  traversable_cells_        ←────── 读 ── 碰撞检测
  preblocked_cells_
  preblocked_costmap_       ←────── 读 ── getPreblockedCost()
```

读/写分离：

| 操作 | 读 octree | 写 octree | 读 occupied_set_ | 写 occupied_set_ |
|------|----------|----------|------------------|------------------|
| A* 搜索 | ✗ | ✗ | ✓ | ✗ |
| Online cloud | ✗ | ✓ | ✗ | ✗ |
| Reanalyze | ✓ (仅快照阶段) | ✗ | ✗ | ✓ (rebuildFromSnapshot) |

### 11.4 并发效果

```
改造前 (1 把 recursive_mutex):
  A* ────→ 点云阻塞，等 A* 完成
  点云 ──→ A* 阻塞，等点云完成

改造后 (2 把 mutex，快照解耦):
  A* ────→ 点云零等待 ✓
  点云 ──→ A* 零等待 ✓
  reanalyze Phase A (~10ms) ──→ 点云等待（可接受）
  reanalyze Phase B (~1-5s) ──→ A* 等待（可接受，10s 一次）
```

| 场景 | 改造前 | 改造后 |
|------|--------|--------|
| A* 运行中 → lidar 点云到达 | 阻塞，等 A* 完成 | **零等待，立即处理** |
| lidar 点云处理中 → 新 goal 到达 | A* 被锁阻塞 | **A* 立即启动**（用快照） |
| reanalyze 运行中 → lidar 点云 | 全程阻塞 | 只阻塞快照阶段 ~10ms |
| reanalyze 运行中 → A* | 全程阻塞 | 阻塞 Phase B 重建阶段 ~1-5s |

### 11.5 精度说明

A* 使用的 `occupied_set_` 快照最旧 10 秒（reanalyze 周期），但：
- Timer 每 1.5s 触发 A* 重规划，路径随快照刷新而更新
- `online_update_period_s` 可调小加快快照刷新频率
- 机器人移动缓慢，碰撞检测用稍旧的占据信息风险极低
- 改造前 A* 阻塞点云 → 新障碍物无法及时入图 → 实际上比现在更差

---

## 12. 全部参数参考

### 12.1 地图加载

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `pcd_file` | string | `""` | 地图文件路径 |
| `frame_id` | string | `"map"` | 全局坐标系 ID |
| `resolution` | double | `0.2` | OctoMap 分辨率 (m) |
| `auto_save_bt` | bool | `true` | 转换后自动保存 .bt 缓存 |
| `crop_box_enabled` | bool | `false` | 是否启用统一 box 裁剪（所有格式载入后） |
| `crop_box_min` | double[3] | `[0,0,0]` | 裁剪盒最小角（map/world 系，x/y/z） |
| `crop_box_max` | double[3] | `[0,0,0]` | 裁剪盒最大角（map/world 系，x/y/z） |

### 12.2 PCD 转换过滤

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `min_points_per_voxel` | int | `3` | 体素最小点数（去噪） |
| `min_cluster_voxels` | int | `2` | 连通域最小体素数（去孤立噪点） |
| `enable_ground_infill` | bool | `true` | 地面补全 |
| `ground_infill_neighbor_threshold` | int | `3` | 地面补全邻居阈值 |
| `ground_infill_density_threshold` | double | `0.02` | 地面补全密度阈值 |

### 12.3 规划器

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `robot_radius` | double | `0.05` | 机器人碰撞半径 (m) |
| `max_iterations` | int | `500000` | A* 最大迭代次数 |
| `snap_search_radius_cells` | int | `8` | 起/终点 Snap 搜索半径 |
| `require_ground_support` | bool | `true` | 要求地面支撑 |
| `strict_direct_ground_support` | bool | `true` | 严格正下方支撑 |
| `ground_support_xy_radius_cells` | int | `1` | 地面支撑 XY 搜索半径 |
| `ground_support_depth_cells` | int | `2` | 地面支撑深度搜索 |
| `lowest_traversable_only` | bool | `false` | 仅保留最低可通行层 |
| `replan_period_s` | double | `1.5` | 定时重规划周期（0=禁用） |

### 12.4 Preblocked

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `preblocked_hard_obstacle` | bool | `true` | 硬障碍物开关 |
| `enable_preblocked_costmap` | bool | `true` | 软代价地图开关 |
| `preblocked_costmap_radius_cells` | int | `3` | 代价扩散半径 |
| `preblocked_costmap_weight` | double | `2.5` | 代价权重 |

### 12.5 Radical Infill

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `radical_infill_enabled` | bool | `true` | 启用激进填充 |
| `radical_infill_radius_m` | double | `1.0` | 桥接半径 |
| `radical_infill_clearance_m` | double | `1.0` | 障碍物间距 |
| `radical_infill_half_height_m` | double | `0.1` | 填充半高 |

### 12.6 Flatten

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `flatten_enabled` | bool | `true` | 启用可通行面平滑 |
| `flatten_window_cells` | int | `5` | 中值滤波窗口 |
| `flatten_max_delta_cells` | int | `1` | 最大 Z 差异（保留楼梯） |

### 12.7 发布

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `auto_publish_enabled` | bool | `false` | 是否定时自动重发布 |
| `octomap_publish_period_s` | double | `5.0` | 定时发布周期 |

### 12.8 在线更新

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `online_update_enabled` | bool | `true` | 启用在线更新 |
| `online_update_cloud_topic` | string | `"/lidar_points"` | 输入点云话题 |
| `online_update_period_s` | double | `10.0` | 重分析周期 |
| `online_update_occupied_prob` | double | `0.8` | 占据概率 |
| `online_update_use_raycasting` | bool | `true` | 射线清空 |
| `online_update_conservative_mode` | bool | `true` | 保守模式 |
| `online_update_min_interval_ms` | int | `500` | 最小处理间隔 |
| `online_update_downsample_step` | int | `1` | 降采样步长 |
| `online_update_max_xy_distance` | double | `10.0` | XY 最大距离 |
| `online_update_max_z_above` | double | `1.0` | 传感器上方最大高度 |
| `online_update_max_z_below` | double | `5.0` | 传感器下方最大深度 |
| `online_update_ray_clear_prob_miss` | double | `0.15` | 射线清空 miss 概率（越小清空越激进） |

---

## 13. 典型使用流程

### 13.1 静态地图规划

```bash
# 启动导航栈（加载 PCD/BT 地图）
ros2 launch bringup navigation.launch.py \
  pcd_file:=/path/to/map.pcd \
  launch_rosbridge:=true

# Web UI 设置目标 → octo_planner 返回路径 → localPlanner 执行
```

### 13.2 在线更新 + 增量建图

```bash
# 不加载初始地图，完全依赖 lidar
ros2 launch bringup navigation.launch.py \
  pcd_file:="" \
  launch_rosbridge:=true
```

### 13.3 闭环导航（仿真）

```bash
# 终端 1: Gazebo
ros2 launch simulation gazebo.launch.py world:=.../worlds/map_nav3d.world

# 终端 2: 导航栈
ros2 launch bringup navigation.launch.py launch_rosbridge:=true
```

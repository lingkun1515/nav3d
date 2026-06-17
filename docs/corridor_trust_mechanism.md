# 3D 楼梯攀爬：全局路径走廊信任机制

## 问题背景

octo_planner 能从完整 OctoMap 生成跨楼层的 3D 全局路径（`/planned_path` 含 Z 坐标），但 localPlanner 用独立的 2D 碰撞检测重新判断可通行性——楼梯台阶的 LaserScan 点云被当作障碍物，导致无路可走。

**根因**：localPlanner 的碰撞检测是纯 2D 的（343 条路径 Z=0，Z 过滤 `[-0.5, 0.25]`，2D 对应表查表），无法区分"楼梯台阶（已知静态几何，应该通过）"和"动态障碍物（需要避开）"。

**解决思路**：不是替换 localPlanner，而是在其碰撞检测中增加一个"信任全局路径走廊"的甄别层——走廊内且 OctoMap 已知的体素 → 放过；走廊外的或 OctoMap 未知的 → 保留原碰撞检测。

## 关键洞察

LaserScan 中落在全局路径走廊内的点，如果在 OctoMap 原始占据体素中能找到对应，说明它是全局规划器已知的静态几何（楼梯台阶），应该放过。如果找不到对应，说明是后来出现的动态物体，必须保留避障。

## 架构概览

```
octo_planner                          localPlanner                        pathFollower
    │                                      │                                    │
    ├──/octomap_occupied_cloud────────────>│  OctoMap 哈希表                      │
    │                                      │                                    │
    ├──/planned_path──────────────────────>│  corridor_segments_ 检测            │
    │                                      │                                    │
    │                                      ├──/near_corridor───────────────────>│  放宽倾斜阈值
    │                                      │                                    │
    │                                      │  碰撞检测:                          │
    │                                      │    if 走廊内 && OctoMap已知 → 跳过  │
    │                                      │    else → 正常避障                  │
```

## 修改的文件

| 文件 | 改动量 | 说明 |
|------|--------|------|
| `src/octo_planner/src/octo_planner_node.cpp` | ~40 行 | 新增 `/octomap_occupied_cloud` publisher |
| `src/local_planner/src/local_planner_node.cpp` | ~100 行 | 走廊检测 + OctoMap 查表 + 碰撞跳过 |
| `src/local_planner/src/path_follower_node.cpp` | ~20 行 | 楼梯段倾斜停车阈值调整 |
| `src/bringup/launch/navigation.launch.py` | 1 行 | 启用 `corridor_trust_mode` |

## Phase 1：octo_planner 发布原始占据体素

**文件**：`src/octo_planner/src/octo_planner_node.cpp`

### 新增 publisher

```cpp
occupied_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
  "/octomap_occupied_cloud", qos_tl);  // transient_local, reliable
```

### `publish_occupied_cloud()` 方法

遍历 `octree_->begin_leafs()` 到 `end_leafs()`，收集所有占据叶子节点的中心坐标（含 pruned node 分解为单个体素），发布为 `PointCloud2`。

实现方式：与 `publish_risk_cost_cloud()` 相同模式——两遍遍历（第一遍计数，第二遍用 `PointCloud2Modifier` + `PointCloud2Iterator` 填充），无需 `pcl_conversions` 依赖。

### 调用点

- `configure_planner()` 末尾 → 首次加载地图时发布
- `republish_all()` 中 → Web 编辑后 reanalyze 时更新

### 成员变量

```cpp
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_cloud_pub_;
```

## Phase 2：localPlanner 走廊信任机制

**文件**：`src/local_planner/src/local_planner_node.cpp`

### 新增参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `corridor_trust_mode` | `false` | 总开关 |
| `corridor_slope_threshold` | `0.15` | Z梯度/XY距离 > 此值视为楼梯段 (~8.5°) |
| `corridor_xy_margin` | `0.5` | 走廊 XY 膨胀半宽 (m) |
| `corridor_z_margin` | `0.3` | 走廊 Z 膨胀容差 (m) |
| `octomap_voxel_resolution` | `0.1` | 查表量化精度 (m) |

### OctoMap 占据体素哈希表

```cpp
std::unordered_set<std::string> octomap_occupied_set_;
bool has_octomap_cloud_ = false;
```

体素坐标量化到 `octomap_voxel_resolution_` 避免浮点误差：

```cpp
std::string encode_voxel(double x, double y, double z) {
  int ix = static_cast<int>(std::round(x / octomap_voxel_resolution_));
  int iy = static_cast<int>(std::round(y / octomap_voxel_resolution_));
  int iz = static_cast<int>(std::round(z / octomap_voxel_resolution_));
  return std::to_string(ix) + "_" + std::to_string(iy) + "_" + std::to_string(iz);
}
```

订阅 `/octomap_occupied_cloud`（QoS: `transient_local`），回调中将点云每个点编码后 insert 到 `unordered_set`。

### 楼梯段检测

`detect_corridor_segments()` 在 `planned_path_callback` 末尾调用：

1. 遍历 `planned_waypoints_` 相邻点对
2. 计算 `dz / dxy`，若 > `corridor_slope_threshold` → 标记为坡道段
3. 为每段连续坡道计算 AABB + 膨胀（`corridor_xy_margin`、`corridor_z_margin`）
4. 存入 `corridor_segments_`

```cpp
struct CorridorSegment {
  double min_x, max_x, min_y, max_y;
  double min_z, max_z;
  bool active;
};
```

### 碰撞检测循环修改

在 `process_loop()` 的障碍物点变换循环中（世界坐标系下），对每个点做甄别：

```cpp
bool use_corridor = corridor_trust_mode_ && has_octomap_cloud_ && !corridor_segments_.empty();

// 在变换循环内：
if (use_corridor && in_corridor(pt.x, pt.y, pt.z)) {
  if (octomap_occupied_set_.count(encode_voxel(pt.x, pt.y, pt.z)) > 0) {
    continue;  // 已知静态几何 → 跳过碰撞检测
  }
  // 不在 OctoMap 中 → 动态物体 → 正常碰撞检测
}
```

被信任的点根本不进入后续碰撞检测循环，既正确又高效。

### `in_corridor()` 辅助函数

遍历 `corridor_segments_`，检查世界坐标点是否落在任一活跃段 AABB 内。

### `/near_corridor` 话题

在 `process_loop()` 末尾，判断当前车辆位置是否在走廊内，发布 `std_msgs::msg::Bool`。pathFollower 用此信号切换倾斜阈值。

## Phase 3：pathFollower 倾斜停车适配

**文件**：`src/local_planner/src/path_follower_node.cpp`

### 新增参数

```cpp
declare_parameter("corridor_tilt_threshold", 60.0);  // 楼梯段放宽到 60°
```

### 新增订阅

订阅 `/near_corridor`（`std_msgs::msg::Bool`），回调更新 `near_corridor_` 标志。

### 修改倾斜停车逻辑

```cpp
if (use_incl_to_stop_) {
  double effective_thre = (near_corridor_ && corridor_tilt_threshold_ > 0)
    ? corridor_tilt_threshold_ : incl_thre_;
  if (std::abs(roll) > effective_thre * PI / 180.0 ||
      std::abs(pitch) > effective_thre * PI / 180.0) {
    stop_init_time_ = rclcpp::Time(odom->header.stamp).seconds();
  }
}
```

## Phase 4：launch 配置

**文件**：`src/bringup/launch/navigation.launch.py`

localPlanner 节点参数添加：

```python
{'corridor_trust_mode': True},
```

## 新增话题

| 话题 | 方向 | 类型 | QoS |
|------|------|------|-----|
| `/octomap_occupied_cloud` | octo_planner → localPlanner | PointCloud2 | transient_local |
| `/near_corridor` | localPlanner → pathFollower | Bool | best_effort (rclcpp::QoS(5)) |

## 对平地导航的影响

| 场景 | `corridor_segments_` | 行为 |
|------|---------------------|------|
| 平地（全局路径无 Z 梯度） | 空 | 甄别逻辑完全跳过 → **原行为 100% 不变** |
| 全局路径经过楼梯但机器人还在平地 | 非空，但车辆不在走廊内 | `in_corridor() = false` → 正常避障 |
| 机器人进入楼梯走廊 | 非空，车辆在走廊内 | 走廊内 OctoMap 已知体素放过，陌生点保留 |
| `corridor_trust_mode = false` | 不检测 | 全局路径 Z 被忽略，原行为不变 |

## 验证清单

### 编译

```bash
colcon build --symlink-install --packages-select octo_planner local_planner
```

### 平地导航回归

- 启动 `ros2 launch bringup navigation.launch.py`（默认 map_nav3d.bt，平地地图）
- 设置目标点 → 确认 localPlanner 正常规划路径、pathFollower 正常跟踪
- `corridor_trust_mode=false` 与 `true` 行为一致（无楼梯段 → corridor_segments 为空）

### 楼梯地图功能测试

- 准备含楼梯的多层 .bt 地图
- 启动导航栈，设置跨层目标
- 验证 `/octomap_occupied_cloud` 有数据发布
- 验证 localPlanner 日志显示检测到 corridor segments
- 验证路径搜索成功（不被楼梯障碍物阻塞）

### 动态障碍物安全测试

- 在楼梯走廊内放置临时障碍物（如 box 模型）
- 验证 localPlanner 正确检测并避开（不在 OctoMap 中 → 保留碰撞检测）

### pathFollower 倾斜测试

- 仿真中机器人爬楼梯 → 倾角变化
- 验证 `/near_corridor = true` 时使用放宽阈值，不会误停车

## 配置参数汇总

### localPlanner 新增 (`local_planner_params.yaml`)

```yaml
localPlanner:
  ros__parameters:
    # ... existing ...
    # Corridor trust — trust known static geometry near global path (stairs)
    corridor_trust_mode: true        # 总开关，false 完全恢复原行为
    corridor_slope_threshold: 0.15   # Z梯度/XY距离 > 此值视为楼梯段 (~8.5°)
    corridor_xy_margin: 0.5          # 走廊 XY 膨胀半宽 (m)
    corridor_z_margin: 0.3           # 走廊 Z 膨胀容差 (m)
    octomap_voxel_resolution: 0.1    # OctoMap 查表量化精度 (m)
```

### pathFollower 新增 (`local_planner_params.yaml`)

```yaml
pathFollower:
  ros__parameters:
    # ... existing ...
    corridor_tilt_threshold: 60.0    # 楼梯段倾斜停车放宽阈值 (度)
```

### octo_planner 新增 (`planner_params.yaml`)

```yaml
octo_planner_node:
  ros__parameters:
    # ... existing ...
    occupied_cloud_radius: 0.0       # 0 = 不限，设 30.0 则只发布机器人 30m 内的体素
```

### 禁止后退（`local_planner_params.yaml` 修改）

```yaml
# localPlanner 和 pathFollower 两处均改为 false：
twoWayDrive: false
```

效果：
- localPlanner: 后向路径组不参与评选，`joyDir` 钳制在前方 ±95°
- pathFollower: `nav_fwd_` 永不切换，`cmd_vel.linear.x` 永不为负

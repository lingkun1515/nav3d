# 在线增量 OctoMap 更新

## 背景

当前 octo_planner 通过 Web 编辑界面手动增删体素，或一次性加载 .bt/.pcd 地图。如果希望导航过程中自动发现新障碍物并更新全局地图，需要一个在线更新通道。

## 架构

```
Real-time LiDAR / depth camera
        │
        ▼
/lidar_points (PointCloud2, sensor frame)
        │
        ▼
octo_planner::on_online_cloud()
  ├── TF → map frame
  ├── for each point: octree_->updateNode(x, y, z, log_odds)
  └── octree_->updateInnerOccupancy()

        ... (持续累积) ...

        │  每 60s (可配置)
        ▼
octo_planner::on_online_reanalyze()
  ├── planner_->reanalyze()
  └── republish_all()
        ├── /octomap
        ├── /octomap_occupied_cloud   ← localPlanner 更新 octomap_occupied_set_
        ├── /traversable_cells_markers
        ├── /preblocked_cells_markers
        └── /risk_cost_cells
```

## 新增参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `online_update_enabled` | `false` | 总开关 |
| `online_update_cloud_topic` | `"/lidar_points"` | 订阅的点云话题 |
| `online_update_period_s` | `60.0` | reanalyze + republish 周期 |
| `online_update_occupied_prob` | `0.7` | updateNode 的占据概率（对数几率累积） |
| `online_update_use_raycasting` | `false` | 启用 `insertPointCloud` 射线追踪模式：标记占用端点的同时清空传感器到端点之间的 free space |
| `online_update_conservative_mode` | `false` | 保守更新模式：标记占据时将点 z 下移半个 voxel（落到命中点下一格），仅在 updateNode 路径生效；raycasting 清除仍用原始坐标 |
| `online_update_min_interval_ms` | `500` | 两次云处理最短间隔 (ms)。0 = 每帧都处理；500 = 最多 2 Hz |
| `online_update_downsample_step` | `1` | 点云抽稀步长。2 = 隔 1 取 1；3 = 每 3 点取 1；1 = 不抽稀 |

全部在 `planner_params.yaml` 中配置。

## 实现概要

### 订阅注册（`setup_pub_sub`）

```cpp
if (get_parameter("online_update_enabled").as_bool()) {
  online_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    online_update_cloud_topic, rclcpp::QoS(5).best_effort(),
    [this](auto msg) { on_online_cloud(msg); });

  double period = get_parameter("online_update_period_s").as_double();
  online_update_timer_ = create_wall_timer(
    std::chrono::duration<double>(period),
    [this]() { on_online_reanalyze(); });
}
```

### `on_online_cloud(msg)` — 实时体素插入

支持两种模式，由 `online_update_use_raycasting` 切换：

**Raycasting 模式** (`use_raycasting=true`)：
1. TF 变换点云到 map 系，同时获取传感器原点
2. 构建 `octomap::Pointcloud`，调用 `octree_->insertPointCloud(points, sensor_origin, -1, false, false)`
3. `insertPointCloud` 内部从传感器原点到每个端点做射线追踪：射线经过的体素标记为 free，端点标记为占据（log-odds 累积）
4. 保守模式在此模式下自动跳过（射线清空与保守外推互斥）

**手动模式** (`use_raycasting=false`)：
1. TF 变换点云到 map 系
2. 遍历每个点，调用 `octree_->updateNode(x, y, z, log_odds)`
3. 可选保守下移：开启时将点 z 减去半个 voxel（标记落到命中点下一格）
4. `octree_->updateInnerOccupancy()`

### `on_online_reanalyze()` — 定时重分析

1. `planner_->reanalyze()`
2. `republish_all()`（含 `publish_occupied_cloud()`）

### 新增成员

```cpp
// 参数
double online_update_period_s_{60.0};
double online_update_occupied_prob_{0.7};
std::string online_update_cloud_topic_{"/lidar_points"};

// TF
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

// 订阅与定时器
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr online_cloud_sub_;
rclcpp::TimerBase::SharedPtr online_update_timer_;
```

## 关键设计决策

### 1. 使用 `updateNode` 而非 `setNodeValue`

`updateNode` 以对数几率累积证据，单次观测不会立即标记为占据。概率 0.7 对应 log-odds ≈ 0.85，一次观测即可越过高阈值，但与 `setNodeValue(1.5)` 不同，后续负观测可以抵消（`setNodeValue` 直接覆盖，不支持证据消退）。

### 2. Raycasting 模式（`insertPointCloud`）

启用 `online_update_use_raycasting` 后，使用 OctoMap 的 `insertPointCloud` API，从传感器原点向每个点云端点投射射线：

- 射线穿过的体素 → `updateNode(log_odds_free)` 标记为空闲（log_odds 递减）
- 射线端点 → `updateNode(log_odds_occupied)` 标记为占据（log_odds 递增）
- 多次观测累积后，空闲证据会抵消占据证据，实现自然的动态障碍物消退

**与手动模式的区别**：
| | 手动模式 | Raycasting 模式 |
|---|---|---|
| 空闲空间更新 | 无（仅标记占据端点） | 有（射线轨迹清空） |
| 动态障碍物消退 | 需依赖 reanalyze 后占据云更新 | 射线自带负证据累积 |
| 性能 | O(N) 遍历+updateNode | O(N×R) 射线遍历，R≈射线长度/分辨率 |
| 保守模式兼容 | ✓ 可选外推 | ✓ 外推后射线追踪（端点后移 → 占据+清空同步后移） |

### 2.5. 保守模式（Voxel drop）

当 `online_update_conservative_mode=true` 时，标记占据格前把命中点 **z 下移半个 voxel**（`0.5 × resolution`），让占据标记落到光束击中点的下一格（更接近地面/障碍物根部）。

- **手动模式**：下移后的位置直接 `updateNode` 标记占据
- **Raycasting 模式**：清除自由空间仍用**原始坐标**（必须反映光束实际经过的位置）；只有占据端点标记时才下移——注意当前实现中 raycasting 分支不做下移，下移仅在 updateNode 路径生效

```
正常模式： [hit at voxel A]           → 标记 voxel A 为占据
保守模式： [hit at voxel A] ↓½ voxel → 标记 voxel A 下方一格为占据
```

### 3. Reanalyze 周期 60s

过于频繁会导致：

- 全局路径频繁变动（机器人行为不稳定）
- reanalyze 计算量大（可通行性分析 + 膨胀 + 代价地图）
- 大量 transient_local 消息重发

60s 足以应对"新发现一堵墙 / 一扇门"的场景，又不会过于频繁。

### 4. TF 变换

与 localPlanner 的 `laser_cloud_callback` 相同模式：

```cpp
auto transform = tf_buffer_->lookupTransform(
    global_frame_id_, cloud_frame, header.stamp,
    rclcpp::Duration::from_seconds(0.1));
tf2::doTransform(in_cloud, out_cloud, transform);
```

## 与走廊信任机制的兼容性

| 场景 | 行为 | 是否安全 |
|------|------|----------|
| 新静态障碍物被在线更新加入 OctoMap | 60s 后 reanalyze → republish → localPlanner 收到新的 `/octomap_occupied_cloud` → 障碍物在走廊内查表命中 → 被信任跳过 | ✓ 正确 |
| 动态障碍物（如行人）暂时停留在原地 | 在线更新积累占据证据 → 60s 后可能被加入 OctoMap → 变成"已知几何" | ⚠️ 潜在风险，但动态物体通常会在 60s 内移动 |
| 在线更新尚未触发 reanalyze | 新障碍物不在 `octomap_occupied_set_` 中 → 走廊内查表未命中 → normal 避障 | ✓ 安全保守 |
| `online_update_enabled=false` | 无在线更新订阅，行为完全不变 | ✓ 零影响 |

**结论：无冲突。** 在线更新 → reanalyze → republish → `octomap_occupied_set_` 同步更新是闭环的。延迟（最多 60s）期间新障碍物不被信任，属于安全保守策略。

## 推荐搭配

启用在线更新时建议同时设置 `occupied_cloud_radius: 30.0`，避免每次 republish 把整个地图的占据体素都推送给 localPlanner——机器人只需要周边 30m 内的体素用于走廊查表。

## 依赖

- `tf2_ros`（ROS 2 TF 库）—— octo_planner 当前未依赖，需在 CMakeLists.txt 和 package.xml 中新增
- `sensor_msgs` —— 已有依赖

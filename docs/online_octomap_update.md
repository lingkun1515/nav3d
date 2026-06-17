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

1. TF 查询（`tf_buffer_->lookupTransform(map_frame, cloud_frame, ...)`）→ `tf2::doTransform`
2. 遍历变换后的点云，对每个点调用 `octree_->updateNode(x, y, z, prob_to_log_odds)`
3. `octree_->updateInnerOccupancy()`

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

### 2. 不做 Raycasting

简化的体素插入（不追踪传感器射线清空中间空间）可能产生"拖尾"——传感器穿越的空白区域不会被 mark as free。这在以下场景中无影响：

- 标记占据体素本身已经足够用于走廊信任的查表
- 全局路径规划依赖 OctoMap 占据体素，free space 由可通行性分析推断
- 如需完整 free-space 更新，后续可扩展为 `insertPointCloud(sensor_origin, point_cloud)`

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

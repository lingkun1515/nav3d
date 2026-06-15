# ROS 2 参数参考手册

本文档列出 Dog3DNav 项目中所有模块的全部 ROS 参数及其物理含义。

---

## 1. octo_planner — 全局 3D 路径规划

**节点:** `octo_planner_node`  
**配置文件:** `src/octo_planner/config/planner_params.yaml`

### 1.1 地图加载

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `pcd_file` | string | `""` | 地图文件路径（多格式支持详见 quickstart）。空字符串表示启动时不加载，等待 `/pcd_file_cmd` 话题动态指定 |
| `frame_id` | string | `"map"` | 全局固定坐标系名称，所有发布的话题（OctoMap、Markers、Path）的 `header.frame_id` |

### 1.2 PCD→OctoMap 转换

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `resolution` | double | `0.2` | OctoMap 体素分辨率 (m)。值越小精度越高，但计算量呈 O(1/res³) 增长。推荐范围 0.1~0.2 |
| `min_points_per_voxel` | int | `1` | 体素过滤阈值：至少需要多少个原始点云落入同一体素才将该体素标记为占据。值越大噪声越少，但可能丢失薄墙 |
| `min_cluster_voxels` | int | `1` | 连通域过滤阈值：剔除体素数少于该值的孤立占据簇（噪声）。至少为 1，设为 1 不剔除任何簇 |
| `enable_ground_infill` | bool | `true` | 是否启用地面补全：在占据体素稀疏的地面层上，基于邻域填充缺失的地面体素。有助于提高遍历性分析的连续性 |
| `ground_infill_neighbor_threshold` | int | `3` | 地面补全的 8 邻域阈值：空洞格周围 8 个方向至少有该数量的占据邻居时，才将该格补为地面（范围 1~8） |
| `ground_infill_density_threshold` | double | `0.02` | 地面补全的 Z 层密度门限：某 Z 层占据格占全局 XY 面积的比例低于此值则认为该层不是地面层，跳过不补。设为 0.0 禁用密度门限，对所有 Z 层做邻域补全 |

### 1.3 全局路径规划（A*）

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `robot_radius` | double | `0.05` | 机器人碰撞检测半径 (m)。在网格中生成以该值为半径的球形碰撞体，检查占据体素。应与机器狗实际投影半径匹配 |
| `max_iterations` | int | `500000` | A* 搜索最大迭代次数。搜索超过此值仍未找到路径则返回失败 |
| `snap_search_radius_cells` | int | `8` | 起点/终点吸附半径（格数）。当用户指定的起点/终点落在占据格上时，在此半径内搜索最近的可通行格作为实际起点/终点 |
| `require_ground_support` | bool | `true` | 是否要求可通行格下方有占据格支撑（地面检测）。四足机器人必须开启，无人机可关闭 |
| `strict_direct_ground_support` | bool | `true` | 地面支撑的严格模式。`true` 时要求格子正下方紧邻格为占据；`false` 时在下方一定深度和半径范围内搜索占据格 |
| `ground_support_xy_radius_cells` | int | `1` | 非严格地面支撑的 XY 搜索半径（格数）。仅在 `strict_direct_ground_support=false` 时生效 |
| `ground_support_depth_cells` | int | `2` | 非严格地面支撑的 Z 向下搜索深度（格数）。仅在 `strict_direct_ground_support=false` 时生效 |
| `enable_preblocked_costmap` | bool | `true` | 是否启用禁行区代价膨胀。禁行格（如狭窄缝隙、悬空区下方）周围生成距离衰减的代价值，使 A* 倾向于绕行而非贴边 |
| `preblocked_costmap_radius_cells` | int | `3` | 代价膨胀半径（格数）。每个禁行格周围该范围内的可通行格都会被赋上代价 |
| `preblocked_costmap_weight` | double | `2.5` | 代价膨胀权重。A* 搜索时额外代价 = `weight × cost`。值越大越倾向于绕行；0 禁用代价影响 |
| `lowest_traversable_only` | bool | `false` | 是否仅保留每 (X,Y) 列最低的可通行格。高层建筑多楼层时关闭（保留所有层）；单层导航时开启可减少搜索空间 |
| `preblocked_hard_obstacle` | bool | `true` | 禁行格是否作为硬障碍阻断可通行性。`true` 时禁行格在碰撞检测中等同占据格；`false` 时禁行格仅影响代价地图（软惩罚），障碍物间窄缝变为可通行 |

### 1.4 平坦化（Flatten）

用于平滑因点云噪声导致的地面可通行格 Z 向坑洼，不影响楼梯/坡道区域。

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `flatten_enabled` | bool | `false` | 是否启用可通行格平坦化 |
| `flatten_window_cells` | int | `5` | 中值滤波窗口半径（格数）。窗口越大越平滑，但计算量增加 |
| `flatten_max_delta_cells` | int | `1` | 最大 Z 调整量（格数）。当前格 Z 与邻域中位数的差值 ≤ 此值时才调整。设小值（1）仅去噪不伤坡道；设大值可能将缓坡压平 |

### 1.6 激进补全（Radical Infill）

用于桥接因建图扫描遗漏而断开的多块可通行区域。

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `radical_infill_enabled` | bool | `true` | 是否启用激进补全。开启后对可通行格连通分量之间的空洞进行填充 |
| `radical_infill_radius_m` | double | `1.0` | 桥接搜索的扁圆柱半径 (m)。每个边缘可通行格在此半径的圆柱区域内搜索其他分量的可通行格 |
| `radical_infill_clearance_m` | double | `1.0` | 上方清空距离 (m)。候选填充格正上方该距离内若无占据体素，则判定为可填充 |
| `radical_infill_half_height_m` | double | `0.1` | 桥接搜索扁圆柱的半高 (m)。控制跨层搜索的 Z 向宽容度 |

### 1.7 自动保存

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `auto_save_bt` | bool | `true` | 是否在 PCD/World/OT 转换后自动保存 `.bt` 缓存（与源文件同目录同名）。二次启动时若 `.bt` 比源文件新则直接加载跳过转换 |
| `world_xy_window_size_m` | double | `24.0` | .world/.sdf 加载时的 XY 平面裁剪窗口半边长 (m)。0 表示不裁剪，加载世界全部内容 |

### 1.8 发布控制

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `auto_publish_enabled` | bool | `false` | 是否启用定时自动重发地图数据。`false` 时 Web 前端需通过 `/request_map` 服务手动获取或点击「获取地图」按钮 |
| `octomap_publish_period_s` | double | `5.0` | 自动发布周期 (s)。仅在 `auto_publish_enabled=true` 时生效 |

---

## 2. local_planner — 局部规划与实时避障

**Package:** `local_planner`  
**配置文件:** `src/local_planner/config/local_planner_params.yaml`

### 2.1 localPlanner 节点 — Lattice 局部规划

#### 2.1.1 路径文件

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `pathFolder` | string | `""` | 预生成路径文件目录。目录中需包含 `startPaths.ply`, `paths.ply`, `pathList.ply`, `correspondences.txt`。由 launch 文件自动注入，通常无需手动设置 |

#### 2.1.2 车辆几何

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `vehicleLength` | double | `0.4` | 车辆/机器人长度 (m)，用于碰撞检测的投影矩形 |
| `vehicleWidth` | double | `0.4` | 车辆/机器人宽度 (m)，用于碰撞检测的投影矩形 |
| `sensorOffsetX` | double | `0.0` | 传感器相对 vehicle 坐标系原点的 X 偏移 (m) |
| `sensorOffsetY` | double | `0.0` | 传感器相对 vehicle 坐标系原点的 Y 偏移 (m) |
| `vehicleLengthSlot` | double | `0.05` | 碰撞检测时车身 XY 投影的格化步长 (m)。更小的值更精确但计算量更大 |
| `vehicleWidthMargin` | double | `0.1` | 车身宽度安全余量 (m)。车身投影两侧各扩展该值，防止贴边碰撞 |
| `marginYawRateRatio` | double | `0.0` | 横摆角速度对安全余量的放大系数。高速旋转时增大有效宽度 |
| `twoWayDrive` | bool | `true` | 是否支持双向行驶。开启后路径方向与当前朝向相反时自动切换前进/后退 |

#### 2.1.3 感知

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `laserVoxelSize` | double | `0.05` | 激光点云体素降采样分辨率 (m)。点云格化后的体素边长 |
| `terrainVoxelSize` | double | `0.2` | 地形分析点云体素分辨率 (m)。仅在 `useTerrainAnalysis=true` 时生效 |
| `useTerrainAnalysis` | bool | `false` | 是否启用地形分析。开启后使用 `/terrain_map` 点云评估地面可通过性 |
| `checkObstacle` | bool | `true` | 是否启用障碍物检测。关闭后所有路径视为无碰（仅调试用） |
| `checkRotObstacle` | bool | `false` | 是否在旋转时也做障碍物检测 |
| `adjacentRange` | double | `3.5` | 障碍物检测的最大范围 (m)。超过此距离的点云不做碰撞判断 |
| `obstacleHeightThre` | double | `0.2` | 障碍物判定高度阈值 (m)。点高于地面此值视为障碍物 |
| `groundHeightThre` | double | `0.1` | 地面判定高度阈值 (m)。点低于地面此值视为地面（可通行） |
| `costHeightThre1` | double | `0.15` | 代价地形高度阈值一 (m)。地面以上该范围内为低代价地形 |
| `costHeightThre2` | double | `0.1` | 代价地形高度阈值二 (m)。地面以上该范围内为更低代价地形 |
| `useCost` | bool | `false` | 是否启用路径代价评估。开启后对地形高度变化附加代价 |

#### 2.1.4 路径评分

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `slowPathNumThre` | int | `5` | 慢行等级 1 的路径数阈值：可通行路径数低于此值时触发减速等级 1 |
| `slowGroupNumThre` | int | `1` | 慢行等级 2 的路径组数阈值：可通行路径组数低于此值时触发减速等级 2 |
| `surPointThre` | int | `2` | 周围点阈值：用于判断前进方向是否有充足空间 |
| `pointPerPathThre` | int | `2` | 每条路径最少通过点数阈值：路径上满足可通过条件的点数低于此值则该路径被淘汰 |
| `minRelZ` | double | `-0.5` | 障碍物点 Z 向最小相对高度 (m)。低于此高度的点忽略 |
| `maxRelZ` | double | `0.25` | 障碍物点 Z 向最大相对高度 (m)。高于此高度的点忽略 |
| `maxSpeed` | double | `1.0` | 目标最大速度 (m/s)。用于速度相关的路径缩放 |
| `dirWeight` | double | `0.02` | 方向偏差权重。路径方向与目标方向偏差的惩罚系数，越小越不敏感 |
| `dirThre` | double | `90.0` | 方向偏差阈值 (deg)。路径方向与目标方向偏差超过此值则被淘汰 |
| `dirToVehicle` | bool | `false` | 方向参考系。`true` 时以机器人朝向为参考，`false` 时以目标点方向为参考 |
| `pathScale` | double | `1.0` | 路径初始缩放因子。1.0 为原始尺寸，小于 1.0 缩小路径范围（密集障碍物场景适用） |
| `minPathScale` | double | `0.75` | 路径最小缩放因子。无满意路径时逐级缩小的下限 |
| `pathScaleStep` | double | `0.25` | 路径缩放步长。每次缩小路径范围的变化量 |
| `pathScaleBySpeed` | bool | `true` | 是否根据速度动态调整路径缩放。高速时缩小范围，低速时扩大 |
| `minPathRange` | double | `1.0` | 最小路径范围 (m)。限制路径缩短的下限 |
| `pathRangeStep` | double | `0.5` | 路径范围调整步长 (m) |
| `pathRangeBySpeed` | bool | `true` | 是否根据速度动态调整路径范围 |
| `pathCropByGoal` | bool | `true` | 是否根据目标点距离裁剪路径：目标在路径范围内时截断超出部分 |

#### 2.1.5 自主导航

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `autonomyMode` | bool | `false` | 是否启用自主导航模式。开启后忽略手柄输入，由目标点驱动 |
| `autonomySpeed` | double | `1.0` | 自主导航目标速度 (m/s) |
| `joyToSpeedDelay` | double | `2.0` | 手柄释放后延迟切换回手柄速度的时间 (s) |
| `joyToCheckObstacleDelay` | double | `5.0` | 手柄释放后延迟恢复障碍物检测的时间 (s) |
| `freezeAng` | double | `90.0` | 静止转向角度阈值 (deg)。超过此值触发冻结计数器 |
| `freezeTime` | double | `2.0` | 冻结时间 (s)。姿态估计不稳定时原地等待此时间 |
| `omniDirGoalThre` | double | `1.0` | 全向目标判定距离阈值 (m)。距离目标在此范围内时不做方向限制 |
| `goalClearRange` | double | `0.5` | 目标清空范围 (m)。目标点周围此范围内无障碍物即视为到达 |
| `goalBehindRange` | double | `0.8` | 目标后方距离 (m)。目标在机器人后方此距离内有效（用于双向行驶） |
| `goalX` | double | `0.0` | 固定导航目标 X 坐标 (vehicle 帧)。`autonomyMode=true` 且无外部 waypoint 时使用 |
| `goalY` | double | `0.0` | 固定导航目标 Y 坐标 (vehicle 帧) |

#### 2.1.6 TF / 输入配置

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `global_frame_id` | string | `"odom"` | 障碍物点云 TF 转换的目标坐标系。所有输入点云（无论原始 frame_id）都会转换到此坐标系 |
| `use_planned_path` | bool | `false` | 是否订阅 `/planned_path` (nav_msgs::Path) 自行管理航点。`true` 时从全局路径提取 lookahead 航点作为局部目标，并在到达后自动推进；`false` 时使用 `/way_point` 话题接收单个目标点 |
| `use_laser_scan` | bool | `false` | 是否订阅 `/scan` (sensor_msgs::LaserScan)。`true` 时在回调中将 LaserScan 转为 PointCloud2（XYZI 格式），再经 TF 转换后走原有避障逻辑；`false` 时直接订阅 `/registered_scan` (PointCloud2) |
| `waypoint_lookahead` | double | `2.5` | 航点前视距离 (m)。从 `/planned_path` 中提取距离机器人此距离的航点作为局部目标。值越大路径越平滑但可能错过转弯 |
| `waypoint_tolerance` | double | `0.5` | 航点到达判定距离 (m)。机器人当前位置距当前目标航点小于此值时，自动推进到下一个航点 |

---

### 2.2 pathFollower 节点 — Pure-Pursuit 轨迹跟踪

**话题接口补充：**

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 入 | `/stop_navigation` | Bool | Web UI 下发，收到 `true` 时触发安全停车（`safety_stop_ = 1`），停止线速度 |

#### 2.2.1 硬件

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `realRobot` | bool | `false` | 是否使用真实机器人串口通信。`false` 时仅通过 ROS topic `/cmd_vel` 输出（Twist 类型，非 TwistStamped） |
| `serialPort` | string | `"/dev/ttyACM0"` | 串口设备路径。仅 `realRobot=true` 时生效 |
| `baudrate` | int | `115200` | 串口波特率 |
| `sensorOffsetX` | double | `0.0` | 传感器 X 偏移 (m)，用于坐标补偿 |
| `sensorOffsetY` | double | `0.0` | 传感器 Y 偏移 (m)，用于坐标补偿 |
| `pubSkipNum` | int | `1` | 指令发布间隔（帧数）。设为 2 则隔帧发布，降低指令频率 |

#### 2.2.2 驱动模式

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `twoWayDrive` | bool | `true` | 是否支持双向行驶。当前朝向与路径方向夹角 > 90° 时自动切换前进/后退 |

#### 2.2.3 Pure-Pursuit 控制器

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `lookAheadDis` | double | `0.5` | 前视距离 (m)。从路径起点扫描，取第一个距离 > 此值的目标点作为跟踪点。越大路径越平滑但延迟越大 |
| `yawRateGain` | double | `7.5` | 行驶中横摆角速度 P 控制器增益。`yaw_rate = -gain × heading_error` |
| `stopYawRateGain` | double | `7.5` | 静止时横摆角速度 P 控制器增益。用于原地旋转对准目标方向 |
| `maxYawRate` | double | `45.0` | 最大横摆角速度 (deg/s)。控制器输出的硬限幅 |
| `maxSpeed` | double | `1.0` | 最大行驶速度 (m/s) |
| `maxAccel` | double | `1.0` | 最大加速度 (m/s²)。速度 ramp 的上升斜率 |
| `switchTimeThre` | double | `1.0` | 方向切换时间阈值 (s)。双向行驶时，需在此时间内方向持续相反才切换 |
| `dirDiffThre` | double | `0.1` | 方向误差收敛阈值 (rad)。误差小于此值视为已对准 |
| `omniDirGoalThre` | double | `1.0` | 全向目标距离阈值 (m)。距离目标在此范围内时不做方向约束 |
| `omniDirDiffThre` | double | `1.5` | 全向方向差阈值 (rad)。目标距离小于 `omniDirGoalThre` 时方向误差阈值放宽至此值 |
| `stopDisThre` | double | `0.2` | 停车距离阈值 (m)。距目标小于此值时发布零速度 |
| `slowDwnDisThre` | double | `1.0` | 减速起始距离 (m)。距目标小于此值时开始线性减速 |

#### 2.2.4 倾角与应急处理

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `useInclRateToSlow` | bool | `false` | 是否根据倾角变化率减速。机器人在不平坦地面俯仰/侧倾变化过快时减速 |
| `inclRateThre` | double | `120.0` | 倾角变化率阈值 (deg/s)。超过此值触发减速 |
| `slowRate1` | double | `0.25` | 减速等级 1 的速度比例。倾角变化超过阈值时速度降至 `maxSpeed × slowRate1` |
| `slowRate2` | double | `0.5` | 减速等级 2 的速度比例 |
| `slowRate3` | double | `0.75` | 减速等级 3 的速度比例 |
| `slowTime1` | double | `2.0` | 减速等级 1 持续最短时间 (s) |
| `slowTime2` | double | `2.0` | 减速等级 2 持续最短时间 (s) |
| `useSideAvoid` | bool | `false` | 是否启用侧向避让。检测侧向倾角防止翻倒 |
| `useInclToStop` | bool | `false` | 是否根据绝对倾角触发急停。姿态倾角超过阈值时停车 |
| `inclThre` | double | `45.0` | 倾角急停阈值 (deg)。超过此值触发急停 |
| `stopTime` | double | `5.0` | 急停最短持续时间 (s) |
| `noRotAtStop` | bool | `false` | 停车后是否禁止原地旋转 |
| `noRotAtGoal` | bool | `true` | 到达目标后是否禁止原地旋转 |

#### 2.2.5 自主导航

| 参数 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `autonomyMode` | bool | `false` | 是否启用自主导航模式 |
| `autonomySpeed` | double | `1.0` | 自主导航目标速度 (m/s) |
| `joyToSpeedDelay` | double | `2.0` | 手柄释放后延迟切换回手柄速度的时间 (s) |

---

## 3. 参数调优建议

### 分辨率与精度权衡
- `resolution` 是影响整体性能的最关键参数。0.2m 适用于中小型建筑（< 5000 m²）；大型场地考虑 0.3~0.5m
- 减小 `resolution` 需要同步调大 `max_iterations`（A* 搜索格数随 1/res³ 增长）

### 可通行性
- 四足机器人：保持 `require_ground_support=true`、`strict_direct_ground_support=true`
- 无人机：关闭所有地面支撑相关参数
- `robot_radius` 应大于实际机身半径约 20%，保留安全余量

### 激进补全
- 仅在建图质量较差、可通行区域明显断裂时启用
- 先小范围试探：`radical_infill_radius_m=0.5`, `radical_infill_clearance_m=0.5`，确认效果后逐步放大
- 注意：过大的参数可能将不同楼层的可通行区桥接，产生错误的跨层路径

### 局部规划
- `lookAheadDis`: 速度越快应设置越大（一般设为 `maxSpeed × 0.5` ~ `maxSpeed × 1.0`）
- `yawRateGain`: 过大导致震荡，过小导致转弯迟缓。典型范围 5~15
- `pathScale`: 密集障碍物场景降至 0.75~0.5

# local_planner 算法原理与楼梯攀爬扩展分析

## 目录

1. [当前 local_planner 算法详解](#1-当前-local_planner-算法详解)
2. [楼梯攀爬可行性评估](#2-楼梯攀爬可行性评估)
3. [FAR Planner 深度分析](#3-far-planner-深度分析)
4. [octo_planner vs FAR Planner 对比](#4-octo_planner-vs-far-planner-对比)
5. [原 autonomy_stack 的 FAR Planner 方案](#5-原-autonomystack-的-far-planner-方案)
6. [实际解决方案：全局路径走廊信任机制](#6-实际解决方案全局路径走廊信任机制)
   - 6.1 [问题重述](#61-问题重述)
   - 6.2 [关键洞察](#62-关键洞察octomap-原始占据体素--全局路径的证据)
   - 6.3 [方案：走廊信任 + OctoMap 甄别](#63-方案走廊信任--octomap-静态动态甄别)
   - 6.4 [详细实现](#64-详细实现)
   - 6.5 [对平地导航的影响](#65-对平地导航的影响)
   - 6.6 [话题与参数总览](#66-新增话题与参数总览)
7. [实施步骤](#7-实施步骤)

---

## 1. 当前 local_planner 算法详解

### 1.1 架构概览

local_planner 包包含两个独立的 ROS 2 节点，编译自两个 `.cpp` 文件，无共享头文件：

| 节点 | 可执行文件 | 源文件 | 行数 | 频率 |
|------|-----------|--------|------|------|
| **localPlanner** | `localPlanner` | `local_planner_node.cpp` | ~1283 | 100Hz（仅新数据到达时处理） |
| **pathFollower** | `pathFollower` | `path_follower_node.cpp` | ~558 | 100Hz（持续运行） |

两者通过 ROS 话题通信：`/path`、`/slow_down`、`/surrounding_block`。

### 1.2 localPlanner — 基于预计算路径库的碰撞检测规划器

#### 1.2.1 核心思想

**非在线搜索规划器**——不执行 A*、RRT、DWA 等在线搜索算法。所有候选路径在 MATLAB 中离线生成，运行时仅做碰撞检测 + 评分选择。

#### 1.2.2 离线路径生成（MATLAB `path_generator.m`）

**三层嵌套循环生成 343 条路径：**

```
外层 (7 组):   shift1 = -27° ~ +27°, 步长 9°
  中层 (7 子组): shift2 = shift1 ± angle*scale 范围内, 步长 deltaAngle*scale
    内层 (7 子子组): shift3 = 类似变化, 步长 angle*scale²
```

- 每条路径由 4 个控制点定义的三次样条曲线：(0,0) → (1.0, shift1) → (2.0, shift2) → (3.0, shift3)
- 曲线以 0.01m 分辨率重采样，总长 3.0m
- **所有路径的 Z 坐标均为 0**（纯 2D 平面路径）
- 7 个组定义了 7 个"转向族"（group 3 = 直行中心组）

**输出文件：**

| 文件 | 内容 | 点数 |
|------|------|------|
| `startPaths.ply` | 每组首段 0~1m 路径点（用于输出路径） | ~707（~101/组） |
| `paths.ply` | 全部 343 条完整路径点（用于可视化） | ~103,243（~301/路径） |
| `pathList.ply` | 每条路径的终点坐标 + 组 ID | 343 |
| `correspondences.txt` | 体素→路径碰撞映射表 | 72,611 条 |

**关键数据：对应表（Correspondence Table）**

```
网格: 161 × 451 = 72,611 个体素
体素大小: 0.02m
搜索半径: 0.45m
覆盖范围: X 方向 3.2m, Y 方向 4.5m（车辆前方右侧象限）

对每个体素中心，rangesearch() 查找半径 0.45m 内的所有路径点
→ 输出: voxelID → [pathID1, pathID2, ..., -1]
```

这是规划器的核心加速结构：给定障碍物点的网格坐标，O(1) 查表即可获知哪些路径经过该区域。

#### 1.2.3 在线碰撞检测流程（`process_loop()`, 100Hz）

**Step 1: 障碍物点云装配**
- 激光模式：从 `laserCloudDwz_`（VoxelGrid 降采样，leaf=0.05m）复制
- 地形模式：从 `terrainCloudDwz_`（VoxelGrid 降采样，leaf=0.2m）复制
- 附加边界虚拟障碍物 + 手动添加障碍物

**Step 2: 坐标变换到车辆系**
- 所有障碍物点平移到车辆位置，旋转到车辆朝向
- Z 轴过滤：仅保留 `minRelZ_`(-0.5m) < point.z < `maxRelZ_`(0.25m) 的点
- 超出 `adjacentRange_`(3.5m) 的点丢弃

**Step 3: 周围阻挡检测**
- 将车辆周围划分为 6 个区域（前左/前右/后左/后右/左侧/右侧）
- 边界框使用 `vehicleLength_/2` + `vehicleWidth_/2` + 安全余量
- 横摆角速度相关余量：`marginYawRateRatio_ * x * vehicleYawRate_`（转弯时动态扩展）
- 发布 `/surrounding_block`（Int8 位掩码）

**Step 4: 航点管理**（当 `use_planned_path_=true`）
- 沿 `planned_waypoints_` 向前搜索，找到距离超过 `waypoint_lookahead_`(2.5m) 的航点
- 设为当前目标 `(goalX_, goalY_)`
- 到达容差内（`waypoint_tolerance_`=0.5m）推进索引到下一航点

**Step 5: 期望方向计算**
- 手动模式：摇杆方向
- 自主模式：车辆到目标的方向角
- 冻结逻辑：目标在后方且角度超过 `freezeAng_`(90°) → 零方向 → 状态机计时后解除

**Step 6: 迭代路径搜索（多尺度重试）**

外层 while 循环：`pathScale` 从预设值（默认 1.0）逐步降至 `minPathScale_`(0.75)，`pathRange` 降至 `minPathRange_`(1.0m)：

```
对每个缩放级别：
  1. 清空评分数组
  2. 计算旋转障碍物角度边界（如启用）
  3. 对每个障碍物点：
     a. 按 1/pathScale 缩放 XY
     b. 对 36 个旋转方向（0~350°, 步长 10°）：
        - 跳过超出 dirThre_(90°) 约束的方向
        - 旋转障碍物点到路径坐标系
        - 计算非均匀 Y 缩放（补偿路径的锥形几何）
        - 查 correspondences_ 表获取受影响的路径 ID
        - 递增 clearPathList_[rotDir*343 + pathID] 碰撞计数
        - 或更新 pathPenaltyList_（地形代价模式）
  4. 评分每个 (旋转, 路径) 对：
     - 碰撞计数 < pointPerPathThre_(2) 才算通畅
     - score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW^4
     - dirDiff: 期望方向与路径终点方向的偏差
     - rotDirW: 旋转惩罚（rotDir=9 向前、rotDir=27 向后 → 惩罚最小）
  5. 选择最佳组：
     - 按 clearPathPerGroupScore_ 最大选组
     - 满足旋转障碍物角度约束
  6. 计算缓行等级：
     - Level 1/2: 地形代价超阈值
     - Level 3: 通畅路径数 < slowPathNumThre_(5) 且组偏离中心
  7. 构建输出路径：
     - 提取 startPaths_[groupID] 点（组首段路径）
     - 旋转 + 缩放
     - 裁剪到 pathRange 和目标距离
     - 发布 nav_msgs::Path（base_link 帧）
  8. 如无路径：缩 scale → 缩 range → 重试
```

**Step 7: 若全部尺度都无路径，发布原点单车长路径（零速效果）**

#### 1.2.4 评分公式详解

```
dirDiff = |joyDir - endDirPathList_[pathID] - (rotDir*10 - 180)|
rotDirW = | |rotDir - 9| + 1 |    // rotDir < 18 (前半圆)
        = | |rotDir - 27| + 1 |   // rotDir >= 18 (后半圆)
groupDirW = 4.0 - |pathList_[pathID] - 3|

// 非全向模式:
score = (1.0 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW^4

// 全向模式 (relativeGoalDis < omniDirGoalThre):
score = (1.0 - sqrt(sqrt(dirWeight * dirDiff))) * groupDirW^2
```

- `rotDirW^4`: 大幅惩罚需要大角度旋转的路径（rotDir=9 是正前方，rotDir=27 是正后方）
- `groupDirW`: 组 3（中心直行组）权重最高
- `1 - sqrt(sqrt(...))`: 方向偏差惩罚呈次线性增长

#### 1.2.5 Z 轴/高度处理总结

| 处理层 | 机制 | 默认值 |
|--------|------|--------|
| 障碍物过滤 | 仅保留 minRelZ ~ maxRelZ 的点 | -0.5m ~ 0.25m |
| 障碍物分类 | 地形 intensity > obstacleHeightThre → 硬障碍 | 0.1~0.2m |
| 地形代价 | intensity 介于 groundThre 和 obstacleThre → 代价惩罚 | — |
| 路径 Z | 所有 343 条路径 Z=0（纯 2D） | 0 |
| 输出路径 Z | 保留 startPaths_ 中的 Z（即 0），乘以 pathScale | 0 |
| cmd_vel Z | pathFollower 发布 Twist，无 Z 分量 | 0 |

### 1.3 pathFollower — 纯追踪 + 比例横摆控制

#### 1.3.1 核心算法

**相对里程计记录**：收到路径时记录车辆位姿 `(x_rec, yaw_rec)`，后续在记录的坐标系中计算相对位移，消除里程计漂移影响。

```
vehicleXRel = cos(yaw_rec)*(x - x_rec) + sin(yaw_rec)*(y - y_rec)
vehicleYRel = -sin(yaw_rec)*(x - x_rec) + cos(yaw_rec)*(y - y_rec)
```

**前瞻点选择**：沿路径从 `path_point_id_` 开始推进，找到首个距离超过 `look_ahead_dis_`(0.5m) 的点。

**方向偏差**：
```
pathDir = atan2(disY, disX)
dirDiff = vehicle_yaw - vehicle_yaw_rec - pathDir
```

**双向驾驶**：若 `|dirDiff| > 90°` 且切换计时器到期，翻转 `nav_fwd_`。倒车时 `dirDiff += PI`，速度取反。

**横摆角速度 P 控制**：
```
vehicleYawRate = -gain * dirDiff
// 停止时 gain = stopYawRateGain(7.5)
// 运动中 gain = yawRateGain(7.5)
// 钳制到 ±maxYawRate(45°/s)
```

**速度曲线**：
- 距终点距离 < `slowDwnDisThre * joySpeed` → 线性减速
- 规划器缓行信号 → 乘以 slowRate1/2/3
- 加速度限制：每周期 ±(maxAccel/100) m/s（100Hz → maxAccel m/s²）

**侧向避障**：当某侧被阻挡且对侧通畅时，发布横向速度指令（`linear.y`）。

**安全停车**：
- 倾角超过 `inclThre_`(45°) 持续 `stopTime_`(5s) → 零速
- `/stop` 话题 → 紧急停车
- `/stop_navigation` → 导航停止

#### 1.3.2 关键约束

- **无 Z 轴速度控制**：`Twist.linear.z = 0`，`Twist.angular.x = 0`，`Twist.angular.y = 0`
- **路径跟随在 2D 平面**：整个 pure pursuit 逻辑假设地平面导航
- **倾角仅用于停车**：不用于爬坡速度调节（有 `useInclRateToSlow` 参数但默认 false）

---

## 2. 楼梯攀爬可行性评估

### 2.1 当前系统为何无法处理楼梯

#### 根本限制 1：路径库是纯 2D 的

所有 343 条路径在 MATLAB 中生成时 Z=0，构成 2D 平面路径集。无论传感器检测到什么 3D 地形，`startPaths_` 输出的路径点 Z 恒为 0。**楼梯需要路径具有 Z 分量**——机器人需要"知道"要向上或向下移动。

#### 根本限制 2：Z 轴障碍物过滤会裁掉楼梯

```
minRelZ_ = -0.5   // 低于车辆 0.5m 以上的点被丢弃
maxRelZ_ = 0.25   // 高于车辆 0.25m 以上的点被丢弃
```

典型楼梯（每级高 15-20cm）在车辆前方 1-2m 处时，上级台阶相对车辆的高度可能超过 0.25m，从而被过滤器丢弃——楼梯变成了"不可见"的障碍物。即使不被丢弃，也会被标记为障碍物（高度 > `obstacleHeightThre_`=0.1m），阻塞所有路径。

#### 根本限制 3：碰撞检测无法区分楼梯和墙壁

`correspondences_` 表是 2D 网格（161×451），仅存储 "体素 XY → 哪些路径经过"。没有 Z 维度的碰撞推理——一个位于 Z=0.3m 的楼梯台阶（刚好在车辆上方）和位于 Z=0.0m 的墙壁对路径的阻塞效果完全相同。

#### 根本限制 4：pathFollower 无 Z 轴控制

`/cmd_vel` 发布 `geometry_msgs/Twist`，其 `linear.z` 恒为 0。对于真实机器狗，爬楼梯需要身体姿态控制（俯仰角适应）和 Z 轴运动，这些都不在 pathFollower 的控制范围内。

### 2.2 如强行扩展会怎样？

假设我们修改 local_planner 以支持楼梯：

| 改动 | 影响 | 风险 |
|------|------|------|
| 生成 3D 路径（Z 变化） | MATLAB 生成器需重写，路径空间从 2D 爆炸到 3D | 343 条变 343×N 层 = 数千条，碰撞检测性能大幅下降 |
| 3D 对应表 | 网格从 161×451 变为 161×451×K | 内存从 ~1MB 变为 ~100MB+，查表从 O(1) 变 O(layers) |
| 放宽 Z 过滤 | 更多障碍物点进入计算 | 碰撞检测循环的障碍物点数增加，100Hz 可能无法维持 |
| 楼梯检测 | 需要新增模块判断"这是楼梯而非墙壁" | 误判导致路径穿墙，安全问题严重 |
| pathFollower Z 控制 | 增加 Z 轴速度/姿态指令 | 与现有差速驱动模型冲突，机器狗的运动学完全不同 |
| 代价模型 | 楼梯应有别于平地，需要新的代价函数 | 影响平地导航的路径质量 |

**结论：不可行。** 该规划器的核心设计假设（2D 路径库 + 2D 对应表 + Z 过滤 + 差速驱动模型）与 3D 楼梯攀爬的需求根本矛盾。强行扩展不仅是工程量的问题，更会**不可避免地对平地导航产生负面影响**——放宽 Z 过滤会让低矮障碍物大量涌入，降低路径质量；3D 碰撞检测的计算开销会破坏 100Hz 实时性；Z 轴速度指令与现有差速模型冲突。

### 2.3 可行性判定

> **不可行。** local_planner 的 2D 路径库架构与楼梯攀爬所需的 3D 空间推理根本矛盾。任何有意义的扩展都会（1）大幅增加计算量、（2）改变平地路径选择行为、（3）引入难以调试的安全边界问题。应当寻找架构上原生支持 3D/多层导航的替代方案。

---

## 3. FAR Planner 深度分析

### 3.1 概述

FAR Planner（Fast and Autonomous Route Planner）是 CMU autonomy_stack 中的**在线路径规划器**，基于**增量构建的 3D 可见图（Visibility Graph）**。

**关键定性：FAR Planner 不是传统意义上的"全局规划器"。**

传统全局规划器（Nav2 GlobalPlanner、octo_planner）的共同特征是**预先拥有完整地图**——规划器知道整个环境的所有障碍物和可通行区域，可以在任意两点之间找到全局最优路径。FAR Planner **不具备这个能力**——它只能规划**已经观测到的区域**，对未探索区域一无所知。

更准确的描述是：FAR Planner 是一个**带持久化图记忆的在线增量式路径规划器**。它的图从零开始，随机器人移动而增长（`globalGraphNodes_` 持久化所有历史节点），但"没见过就是没见过"。

**核心特性：**

| 特性 | 说明 |
|------|------|
| 算法类型 | 3D 可见图（Visibility Graph）在线增量规划 |
| 建图方式 | **纯在线**——从 LiDAR 点云流增量构建，无离线地图加载能力 |
| 全局知识 | **无**——仅知道已观测区域，未探索区域无图节点，无法规划 |
| 运行频率 | 5Hz（主循环 `MainLoopCallBack` + 规划回调 `PlanningCallBack`） |
| 多层支持 | 原生 `is_multi_layer` 标志，支持跨层（上下楼）导航 |
| 输出接口 | `/way_point` (PointStamped) + `/navigation_boundary` (PolygonStamped) |
| 依赖 | OpenCV（轮廓检测）、PCL（点云处理）、Eigen3 |
| 适用场景 | 未知环境探索、动态环境导航、边建图边规划 |

**关键源码文件（autonomy_stack 原版）：**

| 文件 | 行数 | 职责 |
|------|------|------|
| `far_planner.cpp` | 913 | 主节点：ROS 接口、主循环、航点投影 |
| `dynamic_graph.cpp` | 854 | 图结构管理：节点/边增删、连接验证、投票系统 |
| `dynamic_graph.h` | 699 | NavNode 数据结构、静态图存储、TrajectoryType 枚举 |
| `graph_planner.cpp` | 489 | 图搜索：Dijkstra 两遍遍历、路径重建、目标管理 |
| `contour_detector.cpp` | — | 轮廓检测：点云→2D 图像→OpenCV 轮廓提取 |
| `contour_graph.cpp` | — | 轮廓图：CTNode 管理、轮廓匹配 |
| `map_handler.cpp` | — | 地图处理：地形高度调整、节点高度对齐 |
| `far_planner.h` | — | FARMaster 类定义、参数结构、ROS 接口声明 |

### 3.2 云输入机制：为什么 FAR Planner 没有"全局"视野

FAR Planner 的"眼睛"完全来自三个 ROS 2 话题订阅。**没有离线地图加载——源码中没有任何 `pcl::io::loadPCDFile` 调用、没有地图导入服务、没有文件路径参数。**

**订阅架构：**

```
/terrain_cloud        (sensor_msgs/PointCloud2) → TerrainCallBack()
/scan_cloud           (sensor_msgs/PointCloud2) → ScanCallBack()
/terrain_local_cloud  (sensor_msgs/PointCloud2) → TerrainLocalCallBack()
```

**TerrainCallBack 是核心数据入口**，流程如下：

```
TerrainCallBack(pc):
│
├─ CropBoxCloud(pc, robot_pos, terrain_range × terrain_range × kTolerZ)
│   裁切到机器人周围 terrain_range（默认 15m）范围内
│
├─ ExtractFreeAndObsCloud() → 按 intensity 分离自由/占据点云
│   intensity < kFreeZ(0.1) → free, >= kFreeZ → obs
│
├─ UpdateObsCloudGrid(temp_obs) / UpdateFreeCloudGrid(temp_free)
│   将点分配到 MapHandler 的固定世界网格单元中
│   网格: 1000m × 1000m × 100m，单元大小 cell_length(5m) × cell_height
│
├─ GetSurroundFreeCloud(surround_free_cloud)
│   GetSurroundObsCloud(surround_obs_cloud)
│   只提取机器人周围邻域单元的云（邻域范围 = sensor_range / cell_length）
│
└─ 静态环境模式 (is_static_env=true):
    - ScanCallBack 直接返回（跳过动态障碍）
    - TerrainLocalCallBack 直接返回
    - 但 TerrainCallBack 仍正常运行
```

**关键限制**：

1. **网格原点固定**：`SetMapOrigin()` 在首次收到里程计时锁死——网格以机器人起始位置为中心，永不移动。机器人走出网格范围后，新点云被丢弃。
2. **邻域裁剪**：`GetSurroundObsCloud` 只返回机器人周围邻域单元的云，即使网格中存了更远的点，也不会被图构建使用。
3. **图节点来自轮廓检测**：`BuildTerrainImgAndExtractContour` 只处理 `surround_obs_cloud_`——即机器人周围的占据点云。远处区域无点云 → 无轮廓 → 无图节点。

**这意味着什么？** 假设机器人站在大楼一楼大厅的起始位置，目标设在三楼走廊。FAR Planner 只知道一楼大厅周围 15-30m 范围内的区域。三楼走廊对它来说完全未知——图中没有任何三楼的节点，也找不到从起始位置到目标的路径。它必须**边走边建图**，逐渐发现上行路径。

这就是 FAR Planner 的设计哲学：它是为**未知环境探索**场景设计的，不是为**已知地图导航**设计的。autonomy_stack 的典型使用场景是：机器人被放入一个未知建筑物，自行探索并导航到目标——边建图边规划。

### 3.3 可见图是在线构建的（不需要预跑）

**直接回答：FAR Planner 的可见图是 100% 在线构建的，不需要预加载任何地图，也不需要"先跑一遍"。**

图构建在 `MainLoopCallBack()` 中每 200ms（5Hz）执行一次，完整流程如下：

```
MainLoopCallBack() @ 5Hz
│
├─ Step 1: 轮廓检测（ContourDetector）
│   BuildTerrainImgAndExtractContour(odom_node, surround_obs_cloud, realworld_contour)
│   将障碍物点云投影到 2D 图像 → OpenCV findContours → 提取真实世界轮廓顶点
│
├─ Step 2: 更新轮廓图（ContourGraph）
│   UpdateContourGraph(odom_node, realworld_contour)
│   将轮廓顶点转为 CTNode，更新轮廓图结构
│
├─ Step 3: 地形高度调整（MapHandler）
│   AdjustCTNodeHeight(ContourGraph::contour_graph_)
│   AdjustNodesHeight(nav_graph_)
│   将 CTNode 和 NavNode 的 Z 坐标吸附到实际地形高度
│
├─ Step 4: 更新全局近邻节点
│   graph_manager_.UpdateGlobalNearNodes()
│
├─ Step 5: 轮廓-导航图匹配
│   contour_graph_.MatchContourWithNavGraph(nav_graph_, near_nav_graph_, new_ctnodes_)
│   找出轮廓图中"新出现"的 CTNode（尚不存在于导航图中）
│
├─ Step 6: 提取新图节点
│   graph_manager_.ExtractGraphNodes(new_ctnodes_)
│   将新 CTNode 转为 NavNode，并检查是否需要中间导航点
│
├─ Step 7: 更新导航图（边/连接）
│   graph_manager_.UpdateNavGraph(new_nodes_, is_stop_update_, clear_nodes_)
│   添加新节点 → 连接 odom 节点 → 重连近邻节点 → 边界检测
│
└─ Step 8: 图搜索（GraphPlanner）
    PlanningCallBack() → 从当前位置搜索到目标的最短路径
```

**核心要点：**
- 图在机器人移动过程中**增量构建**——新探索到的区域自动添加节点和边
- 全局图节点存储在 `DynamicGraph::globalGraphNodes_`（`std::vector<NavNodePtr>`）
- 节点 ID 由 `id_tracker_` 全局递增分配，保证唯一性
- `ResetCurrentGraph()` 可完全清空图并重新开始（切换地图时使用）

### 3.4 多层节点如何产生

**直接回答：多层节点不是"专门生成"的，而是从轮廓检测中自然产生的。不同高度的障碍物轮廓会产生不同 Z 坐标的节点。**

#### 3.3.1 节点生成链路

```
LiDAR 点云（含 Z 坐标）
    │
    ▼
ContourDetector::BuildTerrainImgAndExtractContour()
    │ 点云投影到 2D 图像（俯视图），每个像素存储该列点云的特征
    │ OpenCV findContours() 提取自由空间与障碍物的边界轮廓
    │
    ▼
realworld_contour = 轮廓顶点的真实世界 3D 坐标（XY 来自投影反算，Z 来自点云）
    │
    ▼
ContourGraph::UpdateContourGraph()
    │ 为每个轮廓顶点创建 CTNode（Contour Node）
    │ CTNode 有 position(x, y, z)——z 来自原始点云
    │
    ▼
MapHandler::AdjustCTNodeHeight()
    │ 将 CTNode 的 Z 调整为"最近地形高度 + vehicle_height"
    │ 两层高度的地形（如楼梯平台）会在不同 Z 产生不同 CTNode
    │
    ▼
ContourGraph::MatchContourWithNavGraph()
    │ 找出尚不存在于导航图中的"新"CTNode
    │
    ▼
DynamicGraph::ExtractGraphNodes()
    │ CreateNavNodeFromPoint(ctnode->position)
    │ → NavNode 继承 CTNode 的 3D 坐标
    │ → 检查是否需要中间导航点（is_need_inter_nav_node）
    │
    ▼
DynamicGraph::UpdateNavGraph()
    │ 新 NavNode 加入 globalGraphNodes_
    │ 评估与已有节点的边连接
```

**关键：** 如果环境有两层（如 1 楼和 2 楼），LiDAR 扫描到两层不同高度的障碍物边界，轮廓检测会在**不同 Z 高度**生成两套 CTNode，进而产生两套 NavNode——自然地形成了多层图结构。

#### 3.3.2 terrainAnalysisExt 的作用

FAR Planner 依赖 `terrainAnalysisExt` 节点提供高质量的地形点云：

- **terrainAnalysisExt** 维护滚动体素网格（半径 4m），累积多帧点云（`voxelPointUpdateThre=100`），输出 `/terrain_map_ext`（全局地形）
- **terrainAnalysis** 提供局部地形 `/terrain_map`，intensity 通道编码高度变化
- 这些地形点云含有准确的 Z 信息，使 `AdjustCTNodeHeight` 能正确将节点吸附到地面/楼梯表面

### 3.5 图边如何连接（含跨层边）

**直接回答：边连接由 `IsValidConnect()` 中的多准则投票系统决定。跨层连接需要满足：多边形连通性 + 方向约束 + 地形坡度和高度验证 + 投票计数达标。**

#### 3.4.1 边的四种类型

每个 NavNode 维护四类连接列表：

| 连接类型 | 存储字段 | 来源 |
|----------|----------|------|
| 多边形连接 | `poly_connects` | 两个节点在自由空间中"可见"（IsConvexConnect 多边形凸性检查） |
| 轮廓连接 | `contour_connects` | 沿轮廓边界相邻的节点（IsBoundaryConnect 或 contour_match） |
| 地形连接 | `terrain_connects` | 两节点间地形可通过（IsOnTerrainConnect 坡度+高度检查） |
| 轨迹连接 | `trajectory_connects` | 机器人已实际走过的轨迹（行走证明可通行） |

#### 3.4.2 投票系统（Voting System）

连接不是一次通过就永久有效，而是需要**累积信任**：

```cpp
// dynamic_graph.h - NavNode 投票队列
std::deque<int> edge_votes;        // 多边形边的投票历史（大小 = votes_size, 默认 10）
std::deque<int> contour_votes;     // 轮廓边的投票历史
std::deque<int> terrain_votes;     // 地形边的投票历史（用于 IsOnTerrainConnect）
std::deque<int> trajectory_votes;  // 轨迹边的投票历史
```

- 每次 `IsValidConnect()` 检查通过 → push 1，否则 push 0
- 队列满了 → pop_front 最老的一票
- **边被信任的条件**：连续 N 票为 1（`votes_size` 默认 10，即需要连续 10 次检查通过）
- 任何一次检查失败 → 边**立即失效**（可以从图中移除）

这个机制关键地防止了**动态障碍物导致的虚假连接**：移动的人/车阻挡视线时，边的投票会逐渐降低直到被移除；障碍物消失后，连接重新建立。

#### 3.4.3 IsValidConnect() 完整逻辑

```
IsValidConnect(node1, node2):
│
├─ 检查 1: 多边形连通性（仅限 PILLAR/CONVEX 类型节点）
│   IsConvexConnect(node1, node2)  → 两节点在自由多边形中相互"可见"
│   IsInDirectConstraint(...)      → 连接方向不穿过障碍物边缘
│   IsNavNodesConnectFreePolygon() → 连接线段经过自由空间多边形
│   IsOnTerrainConnect()           → 地形坡度和高度验证
│   → 任一失败则进入 contour 检查
│
├─ 检查 2: 轮廓连通性
│   IsBoundaryConnect(node1, node2)  → 两节点沿同一轮廓边界相邻
│   或: contour_match + IsOnTerrainConnect → 轮廓匹配 + 地形可通行
│   → 任一失败则进入 trajectory 检查
│
├─ 检查 3: 轨迹连通性
│   检查 traj_connects 中是否已有历史连接记录
│   → 失败则尝试额外的 tight-space contour 连接（仅 odom 节点）
│
└─ 检查 4: 地形坡度（IsOnTerrainConnect, 跨层关键）
    │
    ├─ 坡度检查: abs(diff_p.z) / hypot(diff_p.x, diff_p.y) > 1.0 → 拒绝
    │   即斜率 > 45° 的连接被拒绝
    │
    ├─ 高度检查: 连线中点处的地形高度与两端节点高度的差异 < kTolerZ
    │
    └─ terrain_votes 投票累计
```

**跨层边的关键在 `IsOnTerrainConnect`**：
- 如果楼梯坡度 < 45°，且连线中点处有地形支撑 → 边通过
- 如果两节点之间有墙壁/悬崖（坡度 > 45° 或中间无地形）→ 边拒绝
- 投票累计确保只有**持续观测到的可通行连接**才被信任

#### 3.4.4 跨层边代价缩放（graph_planner.cpp）

当 `is_multi_layer=true` 时，图搜索中对跨层边做代价调整：

```cpp
// graph_planner.cpp - UpdateGraphTraverability()
if (neighbor == goal_ptr && !IsAtSameLayer(neighbor, current)) {
    // 跨层连接：用 3D/2D 距离比缩放代价
    float factor = hypot(diff_p.x, diff_p.y) / edist;  // <= 1.0
    edist /= factor;  // 代价放大（因为 factor < 1）
}
// 注：edist = hypot(diff_p.x, diff_p.y, diff_p.z)  即 3D 距离
//      hypot(diff_p.x, diff_p.y) 即 2D 水平距离
//      当有 Z 分量时 edist > 水平距离，factor < 1, edist/=factor 增大代价
```

`IsAtSameLayer()` 逻辑：
```cpp
// Z 差 > kTolerZ → 视为不同层
bool IsAtSameLayer(node1, node2) {
    return abs(node1->position.z - node2->position.z) <= kTolerZ;
}
```

**跨层边代价更大的意义**：规划器会**偏好同层路径**，只有在没有同层路径可达目标时才走跨层连接。这避免了机器人"无缘无故上下楼"。

#### 3.4.5 多层模式下的目标点处理

```cpp
// graph_planner.cpp - UpdateGoal()
if (!FARUtil::IsMultiLayer) {
    // 单层模式：目标 Z 强制吸附到地形高度
    goal_node_ptr_->position.z = NearestTerrainHeightofNavPoint(...) + vehicle_height;
} else {
    // 多层模式：保持用户指定的目标 Z（如二楼的高度）
    // goal_node_ptr_->position.z 不变
}
```

这是多层导航的关键设计：用户设定目标在 2 楼（Z=3.0m），FAR Planner 不会将其"拉回"1 楼地面高度。

### 3.6 图搜索（GraphPlanner）

#### 3.5.1 两遍 Dijkstra 遍历

```cpp
UpdateGraphTraverability():
│
├─ Pass 1: 从 odom 节点展开
│   Dijkstra 传播，只走"已信任"的边
│   → 标记每个节点的 is_traversable（是否可达）
│   → 记录 gscore（到 odom 的距离）
│
└─ Pass 2: 从 odom 节点再次展开
    只走 is_traversable=true 且 is_covered=true 的节点
    → 标记 is_free_traversable（是否在自由空间中可达）
    → 记录 fgscore 和 free_parent
```

- `is_traversable`: 节点在物理上可达（不考虑是否被自由空间覆盖）
- `is_free_traversable`: 节点在自由空间中可达（更高置信度）
- 路径搜索时优先使用 `free_parent`（置信度高），回退到 `parent`（置信度低）

#### 3.5.2 路径重建

```
PathToGoal():
  if goal 是 is_free_traversable:
      从 goal 沿 free_parent 回溯到 origin → 路径 A
  else:
      找 is_free_traversable 且离 goal 最近的节点 → 路径 B

  对路径做后处理：
    - 航点投影（ProjectNavWaypoint）：沿自由空间射线延伸到最远可见点
    - 动量导航：方向一致的短步移动
    - 自动切换 free-nav ↔ attemptable 模式
```

### 3.7 `is_static_env` 静态环境模式

FAR Planner 支持 `is_static_env=true` 参数：

- **跳过**动态障碍物检测（`surround_obs_cloud` 处理）
- **跳过**scan_cloud 更新（不做逐帧点云差分）
- 仅用 `/terrain_cloud`（全局地形）和 `/terrain_local_cloud`（原始扫描）构建图

对于已知静态地图的导航场景（如 octo_planner 当前的使用场景），开启此模式可减少计算开销。

---

## 4. octo_planner vs FAR Planner 对比

### 4.0 根本性差异：已知地图 vs 在线探索

在进行逐项对比之前，必须明确两者最根本的差异：

| | octo_planner | FAR Planner |
|---|---|---|
| **世界观** | **全知**——加载 .bt 后拥有整个环境的完整 3D 模型 | **未知**——启动时对环境一无所知，边走边建图 |
| **规划范围** | 地图内任意两点之间，即刻可规划 | 仅限已观测区域，未探索区域无法规划 |
| **设计场景** | 已知地图的精确导航 | 未知环境的在线探索+导航 |
| **是否需要预跑** | 不需要，加载即用 | **需要**——必须先"看过"一个区域才能规划穿过它 |

这个差异意味着两者不是简单的"谁更好"的关系，而是**解决不同问题**的工具。

### 4.1 总览

| 维度 | octo_planner | FAR Planner |
|------|-------------|-------------|
| **算法** | OctoMap 占据栅格 + 3D A* 搜索 | 3D 可见图（Visibility Graph）+ Dijkstra/A* |
| **地图表示** | 八叉树（OcTree），概率占据，**全量加载** | 图结构（NavNode + 四种边类型），**增量构建** |
| **全局知识** | **有**——全地图立即可用，任意点间可规划 | **无**——仅已知已观测区域，未见过则不可规划 |
| **建图方式** | **离线预加载**（.bt/.pcd/.ot 文件） | **纯在线**（LiDAR 点云流 → MapHandler 滚动网格） |
| **是否需要预跑** | 不需要，加载文件即用 | **需要**——机器人必须"看"过区域才能建立图节点 |
| **动态环境** | **不支持**（静态地图，体素编辑后需手动 reanalyze） | **原生支持**（投票系统 + 动态障碍过滤） |
| **多层/跨层** | **不支持**（3D A* 在单连通空间中搜索，无层概念） | **原生支持**（`is_multi_layer` + 跨层边代价缩放） |
| **更新频率** | 按需（路径请求时计算一次） | 5Hz 持续更新（图结构 + 图搜索） |
| **路径输出** | `/planned_path`（nav_msgs/Path，完整路径点序列） | `/way_point`（PointStamped，下一个航点）+ `/navigation_boundary`（PolygonStamped） |
| **分辨率/精度** | 体素分辨率（如 0.1m），精度高 | 轮廓顶点精度，依赖点云密度 |
| **未知空间** | 视为可通行（A* 可穿越未知） | 无图节点 = 未知，不规划穿越 |
| **计算复杂度** | O(N³) A* on 3D grid（受地图尺寸影响大） | O(V²) per node（受图节点数影响，通常远小于体素数） |
| **内存占用** | 大（数百万体素） | 中等（数千图节点） |
| **依赖** | octomap、PCL | OpenCV、PCL、Eigen3 |
| **成熟度** | OctoPlanner3D 库封装，经 ROS 2 适配 | CMU 原版，需 ROS 2 移植 |

### 4.2 详细优劣分析

#### octo_planner 优势

1. **即开即用**：加载 .bt 文件即可规划，不需要传感器数据积累
2. **精度高**：体素级分辨率（典型的 0.1m），规划路径精确到每个栅格
3. **A* 保证最优**：在离散栅格空间中，A* 保证找到最短路径（在分辨率精度内）
4. **体素语义丰富**：OctoMap 有占据/空闲/未知三态，代价地图可精细建模
5. **与当前系统深度集成**：已有完整的 ROS 2 封装、地图编辑、Web 可视化
6. **离线可运行**：不需要 LiDAR，适合纯仿真和已知地图场景

#### octo_planner 劣势

1. **静态地图**：加载后不会自动更新，动态障碍物无法反映
2. **无多层概念**：3D A* 在统一空间中搜索，无"层"的语义——二楼和三楼的导航无区别，但也不区分"可站立的平面"和"空中"
3. **地图文件依赖**：必须预先生成 .bt/.pcd 文件，不能从零开始探索
4. **大场景性能**：A* 在百万级体素上的搜索时间不可忽略
5. **无边界约束**：只输出路径，无 `/navigation_boundary` 等安全边界信息

#### FAR Planner 优势

1. **纯在线**：不需要任何预加载地图，从 LiDAR 数据实时构建
2. **多层原生支持**：`is_multi_layer` 模式下的层判断、跨层边代价缩放、目标 Z 保持
3. **动态环境适应**：投票系统自然处理动态障碍（出现→投票降低→边移除；消失→重新投票→边恢复）
4. **输出更丰富**：`/way_point` + `/navigation_boundary`，后者可指导局部规划器不偏离安全区域
5. **轻量图结构**：数千节点 vs 数百万体素，搜索更快
6. **轨迹边学习**：机器人实际走过的路径自动成为可信边
7. **边界检测**：自动检测自由空间边界（frontier），可用于探索
8. **适配 autonomy_stack 原生方案**：与 localPlanner 天然协同

#### FAR Planner 劣势

1. **依赖传感器**：必须有 LiDAR 持续输入，纯离线仿真场景受限
2. **需要 terrainAnalysisExt**：依赖地形分析节点提供高质量地形点云
3. **依赖 OpenCV**：增加编译和运行时依赖
4. **ROS 2 移植成本**：CMU 原版为 ROS 1，需适配 ROS 2 Humble（CMakeLists、package.xml、话题名称、NodeHandle → rclcpp）
5. **5Hz 低频率**：全局规划频率低，适合做 waypoint 级引导，不能替代局部避障
6. **精度不如栅格**：轮廓检测精度依赖点云密度，稀疏点云可能导致图粗糙
7. **可见图限制**：非凸环境中可能遗漏某些可行路径（图边不能穿过障碍物边界）
8. **无路径平滑**：输出 waypoint 而非完整路径，路径平滑由下游 localPlanner 负责

### 4.3 关键场景对比

| 场景 | octo_planner | FAR Planner |
|------|-------------|-------------|
| 已知单层建筑物 | 优（直接加载 .bt，精确 A*） | 良（需在线建图，首次需探索） |
| 已知多层建筑物 | 可规划但无层概念（3D A* 穿越楼层间空隙） | **优**（层感知 + 跨层边代价） |
| 动态环境（人/车移动） | **差**（静态地图，无法反映） | **优**（投票系统自适应） |
| 未知环境探索 | **不支持** | **优**（在线建图 + frontier 检测） |
| 纯仿真（无 LiDAR） | **优**（加载 .bt 即可） | 受限（需模拟 LiDAR 或加载 PCD 回放） |
| 大范围室外 | 性能依赖地图尺寸 | 优（图节点数增长慢于体素数） |
| 精度要求高（窄门/缝隙） | 优（体素级精度） | 中（轮廓级精度） |

### 4.4 替换可行性结论

> **FAR Planner 不能直接平替 octo_planner 作为已知地图的全局规划器。**
>
> 根本原因：octo_planner 的核心价值是"预知全局"——加载 .bt 后瞬间拥有整个建筑的完整 3D 模型，可以在任意两点间做精确的 A* 全局寻路。FAR Planner 不具备这个能力——它的图是从 LiDAR 点云流中在线增量构建的，没有"预知"机制。
>
> **如果一定要让 FAR Planner 在已知地图场景中工作**，需要额外的适配层：
> - **方案**：写一个"全局地图回放节点"——启动时从 .bt/.pcd 加载完整 OctoMap，将其转换为 PointCloud2，作为 `/terrain_cloud` 发布给 FAR Planner。FAR Planner 的 `MapHandler` 会将整个地图摄入网格，`MainLoopCallBack` 从中提取轮廓并构建全局可见图。
> - **问题**：`GetSurroundObsCloud` 只提取机器人邻域单元的云，远处区域即使存入了网格也不会被图构建使用。需要修改 FAR Planner 源码，添加"全局加载模式"——在初始化阶段让图构建器遍历全部网格单元而非仅邻域单元。
> - **结论**：可行，但需要非平凡的修改，不是"开箱即用"的。

**因此，正确的定位是：**

| 场景 | 推荐方案 |
|------|----------|
| 已知地图 + 单层平面导航 | **octo_planner**（已完美适配，不需要改） |
| 已知地图 + 多层跨楼导航 | **octo_planner + 层感知增强**（在 OctoMap 上增加层分割 + 跨层连接逻辑）或 **FAR Planner + 全局地图适配** |
| 未知环境 + 在线探索导航 | **FAR Planner**（这正是它的设计目标） |
| 动态环境 + 实时避障 | **FAR Planner**（投票系统天然适应） |

**核心建议**：不要替换 octo_planner。对于你的场景（已知建筑 + .bt 地图），octo_planner 提供了 FAR Planner 无法替代的"全局预知"能力。多层导航的需求应该通过在 octo_planner 基础上**增加层感知**来解决，而非引入一个根本设计目标不同的规划器。

---

## 5. 原 autonomy_stack 的 FAR Planner 方案

### 5.1 官方架构

CMU autonomy_stack 的标准导航管线是：

```
传感器层:
  LiDAR → /registered_scan (sensor_msgs/PointCloud2)
       → terrainAnalysis → /terrain_map (局部地形, intensity=高度变化)
       → terrainAnalysisExt → /terrain_map_ext (全局地形, 滚动体素网格)
       → ARISE SLAM → /state_estimation (里程计, nav_msgs/Odometry)

规划层:
  FAR Planner (5Hz 全局规划)
    输入:
      /odom_world          ← /state_estimation (remap)
      /terrain_cloud       ← /terrain_map_ext
      /scan_cloud          ← /terrain_map
      /terrain_local_cloud ← /registered_scan
      /goal_point          ← 用户设定 (geometry_msgs/PointStamped)
    输出:
      /way_point           → localPlanner (geometry_msgs/PointStamped)
      /navigation_boundary → localPlanner (geometry_msgs/PolygonStamped)
      /far_reach_goal_status → 监控 (std_msgs/Bool)

执行层:
  localPlanner (100Hz 局部碰撞避免)
    输入:
      /way_point           ← FAR Planner
      /navigation_boundary ← FAR Planner
      /registered_scan     ← LiDAR
      /state_estimation    ← SLAM
    输出:
      /path                → pathFollower
      /slow_down           → pathFollower
      /surrounding_block   → pathFollower

  pathFollower (100Hz 运动控制)
    输入:
      /path                ← localPlanner
      /slow_down           ← localPlanner
      /surrounding_block   ← localPlanner
      /state_estimation    ← SLAM
    输出:
      /cmd_vel             → 机器人/仿真
```

### 5.2 关键设计要点

1. **FAR Planner 只做全局引导**：输出 waypoint（单一航点），不做速度控制。路径的平滑、障碍物回避、速度决策完全由 localPlanner 负责。

2. **localPlanner 是必须保留的**：FAR Planner 5Hz 的频率不足以处理动态避障，localPlanner 的 100Hz 碰撞检测 + 343 条路径是不可替代的安全层。

3. **`/navigation_boundary` 的作用**：PolygonStamped 定义了可通行的凸多边形边界，localPlanner 在边界内做规划，防止机器人偏离安全区域（如驶出楼梯平台边缘）。

4. **话题名称灵活性**：autonomy_stack 大量使用 remap，FAR Planner 内部使用通用话题名（`/odom_world`、`/terrain_cloud` 等），实际部署时通过 launch 文件 remap 到实际话题。

5. **terrainAnalysisExt 不是可选的**：FAR Planner 严重依赖全局地形图来调整节点高度和验证地形可通行性。没有它，`AdjustCTNodeHeight` 和 `IsOnTerrainConnect` 将无法工作。

### 5.3 源码关键参数速查

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `is_multi_layer` | false | **多层/楼梯模式开关** |
| `is_static_env` | false | 静态环境模式（跳过动态障碍处理） |
| `voxel_dim` | 0.1m | PCL 降采样体素大小 |
| `robot_dim` | 0.5m | 机器人半径（用于碰撞检测和 free-space 判断） |
| `vehicle_height` | 0.6m | 车辆高度（节点 Z = 地形 + vehicle_height） |
| `sensor_range` | 15.0m | 最大感知范围 |
| `local_planner_range` | 2.5m | 导航边界范围 |
| `main_run_freq` | 5.0Hz | 主循环频率 |
| `g_planner/converge_distance` | 0.4m | 到达目标判定距离 |
| `g_planner/votes_size` | 10 | 边投票队列长度（需连续 10 次通过） |
| `kTolerZ` | — | 同层 Z 容差阈值 |
| `kAddNodeThre` | — | 节点间最小距离阈值 |

---

## 6. 实际解决方案：全局路径走廊信任机制

### 6.1 问题重述

**核心矛盾**：octo_planner 已经从完整 OctoMap 中生成了穿越楼梯的 3D 全局路径（`/planned_path` 含 Z 坐标），但 localPlanner 用自己独立的 2D 碰撞检测重新判断"可通行性"——楼梯台阶的 LaserScan 点云在 localPlanner 看来就是障碍物。

**问题不是规划，是执行。** 不需要改 octo_planner 的算法，也不需要换 FAR Planner。只需让 localPlanner 在全局路径已规划的走廊内"信任" octo_planner 的判断，同时对走廊内的陌生物体（动态障碍）保持警惕。

### 6.2 关键洞察：OctoMap 原始占据体素 = 全局路径的"证据"

全局路径是基于 OctoMap 的 `traversable_cells_` 搜索出来的，而 `traversable_cells_` 又来源于原始占据体素的地面支撑分析。因此：

> LaserScan 中落在全局路径走廊内的点，如果在 OctoMap 原始占据体素中**能找到对应**，说明它是全局规划器已知的静态几何（楼梯台阶），应该放过。如果**找不到对应**，说明是后来出现的动态物体（人、移动障碍物），必须保留避障。

### 6.3 方案：走廊信任 + OctoMap 静态/动态甄别

```
LaserScan 障碍物点
  │
  ├─ 该点在全局路径走廊外？
  │   → 正常碰撞检测（原逻辑不变）
  │
  ├─ 该点在走廊内，且在 OctoMap 占据体素集中？
  │   → 已知静态几何（楼梯）→ 跳过碰撞检测
  │
  └─ 该点在走廊内，但不在 OctoMap 占据体素集中？
      → 新出现的物体（人/动态障碍）→ 保留，正常碰撞检测
```

**数据需要跨节点传递**：localPlanner 需要知道 OctoMap 的原始占据体素位置。通过在 octo_planner 中新增一个 PointCloud2 publisher（`/octomap_occupied_cloud`），localPlanner 订阅后构建 `std::unordered_set` 哈希表，碰撞检测时 O(1) 查表。

### 6.4 详细实现

#### 6.4.1 楼梯段检测（localPlanner `planned_path_callback` 中）

```cpp
struct StairSegment {
  size_t start_idx, end_idx;
  double min_x, max_x, min_y, max_y;  // XY 包围盒（膨胀后）
  double min_z, max_z;                // Z 范围
  bool active;
};

std::vector<StairSegment> stair_segments_;

void detect_stair_segments() {
  stair_segments_.clear();
  const double slope_threshold = 0.15;   // Z梯度/XY距离 > 0.15 → 楼梯 (~8.5°)
  const double corridor_xy_margin = 0.5; // 走廊半宽（m）
  const double corridor_z_margin = 0.3;  // Z 容差（m）

  StairSegment current_seg;
  bool in_stair = false;

  for (size_t i = 1; i < planned_waypoints_.size(); i++) {
    double dx = std::get<0>(planned_waypoints_[i]) - std::get<0>(planned_waypoints_[i-1]);
    double dy = std::get<1>(planned_waypoints_[i]) - std::get<1>(planned_waypoints_[i-1]);
    double dz = std::get<2>(planned_waypoints_[i]) - std::get<2>(planned_waypoints_[i-1]);
    double dxy = std::hypot(dx, dy);

    bool is_sloped = (dxy > 0.01 && std::abs(dz) / dxy > slope_threshold);

    if (is_sloped && !in_stair) {
      // 开始新楼梯段
      in_stair = true;
      current_seg.start_idx = i - 1;
      current_seg.min_x = current_seg.max_x = std::get<0>(planned_waypoints_[i-1]);
      current_seg.min_y = current_seg.max_y = std::get<1>(planned_waypoints_[i-1]);
      current_seg.min_z = current_seg.max_z = std::get<2>(planned_waypoints_[i-1]);
    }

    if (in_stair) {
      double wx = std::get<0>(planned_waypoints_[i]);
      double wy = std::get<1>(planned_waypoints_[i]);
      double wz = std::get<2>(planned_waypoints_[i]);
      current_seg.min_x = std::min(current_seg.min_x, wx);
      current_seg.max_x = std::max(current_seg.max_x, wx);
      current_seg.min_y = std::min(current_seg.min_y, wy);
      current_seg.max_y = std::max(current_seg.max_y, wy);
      current_seg.min_z = std::min(current_seg.min_z, wz);
      current_seg.max_z = std::max(current_seg.max_z, wz);

      if (!is_sloped || i == planned_waypoints_.size() - 1) {
        // 结束楼梯段
        current_seg.end_idx = i;
        // 膨胀
        current_seg.min_x -= corridor_xy_margin;
        current_seg.max_x += corridor_xy_margin;
        current_seg.min_y -= corridor_xy_margin;
        current_seg.max_y += corridor_xy_margin;
        current_seg.min_z -= corridor_z_margin;
        current_seg.max_z += corridor_z_margin;
        current_seg.active = true;
        stair_segments_.push_back(current_seg);
        in_stair = false;
      }
    }
  }
}
```

#### 6.4.2 OctoMap 占据体素发布（octo_planner 新增，~25 行）

在 `octo_planner_node.cpp` 的 `configure_planner()` 和 `reanalyze()` 末尾各加一行 `publish_occupied_cloud()`：

```cpp
void publish_occupied_cloud() {
  if (!octree_) return;
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = now();
  cloud.header.frame_id = get_parameter("frame_id").as_string();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
    if (octree_->isNodeOccupied(*it)) {
      pcl_cloud.push_back(pcl::PointXYZ(
        static_cast<float>(it.getX()),
        static_cast<float>(it.getY()),
        static_cast<float>(it.getZ())));
    }
  }
  pcl::toROSMsg(pcl_cloud, cloud);
  occupied_cloud_pub_->publish(cloud);
}
```

在 `setup_pub_sub()` 中注册：
```cpp
occupied_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
  "/octomap_occupied_cloud", rclcpp::QoS(1).transient_local());
```

#### 6.4.3 localPlanner 订阅 + 哈希表构建（~30 行）

```cpp
// 新增成员
std::unordered_set<std::string> octomap_occupied_set_;  // key = "x_y_z"
bool has_octomap_cloud_ = false;
double octomap_resolution_ = 0.1;  // voxel snap tolerance

std::string encode_voxel(double x, double y, double z) {
  // 量化到 resolution 精度，避免浮点误差
  int ix = static_cast<int>(std::round(x / octomap_resolution_));
  int iy = static_cast<int>(std::round(y / octomap_resolution_));
  int iz = static_cast<int>(std::round(z / octomap_resolution_));
  return std::to_string(ix) + "_" + std::to_string(iy) + "_" + std::to_string(iz);
}

void octomap_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  octomap_occupied_set_.clear();
  for (const auto & pt : cloud) {
    octomap_occupied_set_.insert(encode_voxel(pt.x, pt.y, pt.z));
  }
  has_octomap_cloud_ = true;
  RCLCPP_INFO(get_logger(), "OctoMap occupied set: %zu voxels", octomap_occupied_set_.size());
}
```

#### 6.4.4 走廊内静态/动态甄别（`process_loop` 碰撞循环中，~15 行）

```cpp
// 辅助函数：判断一个世界坐标点是否在楼梯走廊内
bool in_stair_corridor(double wx, double wy, double wz) const {
  for (const auto & seg : stair_segments_) {
    if (!seg.active) continue;
    if (wx >= seg.min_x && wx <= seg.max_x &&
        wy >= seg.min_y && wy <= seg.max_y &&
        wz >= seg.min_z && wz <= seg.max_z) {
      return true;
    }
  }
  return false;
}

// 在碰撞检测循环（第 931 行 for）内部，障碍物点处理前插入：
bool near_stair = !stair_segments_.empty();

// 在点过滤循环内（约第 726 行附近），对每个通过 XY 距离检查的点：
if (near_stair && has_octomap_cloud_) {
  if (in_stair_corridor(world_x, world_y, world_z)) {
    bool in_octomap = octomap_occupied_set_.count(encode_voxel(world_x, world_y, world_z)) > 0;
    if (in_octomap) {
      continue;  // 已知静态几何 → 跳过碰撞检测
    }
    // 不在 OctoMap 中 → 新出现的物体 → 正常做碰撞检测
  }
}
// 走廊外的点 → 正常碰撞检测（原逻辑不变）
```

#### 6.4.5 pathFollower 倾斜停止适配（~5 行）

楼梯攀爬过程中机器人会倾斜，pathFollower 默认在倾角 > 45° 且持续 5s 时停车。需要感知当前是否在楼梯段来切换阈值：

```cpp
// 新建话题：/near_stair_segment (std_msgs/Bool)
// localPlanner 在 process_loop 中发布当前是否在楼梯走廊附近

// pathFollower 订阅后：
double effective_tilt_thre = near_stair_segment_ ? stair_tilt_threshold_ : inclThre_;
// stair_tilt_threshold_ 默认 60°（楼梯上放宽）
// 或直接禁用楼梯段上的倾斜停车：if (near_stair_segment_) skip_tilt_check = true;
```

### 6.5 对平地导航的影响

| 场景 | `stair_segments_` | 行为 |
|------|-------------------|------|
| 平地（全局路径无 Z 梯度） | 空 | `near_stair = false` → 全部逻辑跳过 → **原行为 100% 不变** |
| 全局路径经过楼梯 | 非空，但机器人还在平地上 | `in_stair_corridor = false`（机器人位置不在走廊内）→ 正常避障 |
| 机器人进入楼梯走廊 | 非空，机器人在走廊内 | 走廊内的 OctoMap 已知体素被放过，陌生点保留避障 |
| `stair_aware_mode = false` | 不检测楼梯段 | 全局路径 Z 被忽略，原行为不变 |

### 6.6 新增话题与参数总览

**话题：**

| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/octomap_occupied_cloud` | octo_planner → localPlanner | PointCloud2 | OctoMap 原始占据体素（transient_local QoS） |
| `/near_stair_segment` | localPlanner → pathFollower | Bool | 当前机器人是否在楼梯走廊附近 |

**localPlanner 新增参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `stair_aware_mode` | `false` | 全局路径走廊信任模式开关 |
| `stair_slope_threshold` | 0.15 | Z 梯度 / XY 距离 > 此值视为楼梯段 |
| `stair_corridor_xy_margin` | 0.5 | 走廊 XY 膨胀半宽（m） |
| `stair_corridor_z_margin` | 0.3 | 走廊 Z 膨胀容差（m） |
| `octomap_resolution` | 0.1 | 占据体素查找表量化精度（m） |

**pathFollower 新增参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `stair_tilt_threshold` | 60.0 | 楼梯段上的倾斜停车阈值（度，0=禁用） |

### 6.7 方案总结

| 维度 | 说明 |
|------|------|
| **改什么** | localPlanner（主要）+ octo_planner（加 1 个 publisher）+ pathFollower（加 1 个参数） |
| **不改什么** | GlobalPlanner 算法、path 生成逻辑、碰撞检测核心、octo_planner 规划逻辑 |
| **对平地影响** | **零**——`stair_segments_` 空时全部逻辑跳过 |
| **安全性** | 已知几何放过，未知物体保留——不牺牲动态避障能力 |
| **代码量** | ~80 行新增 + ~15 行修改，分布在 3 个文件中 |
| **工期** | 1-2 天 |
| **新增依赖** | 无 |

---

## 7. 实施步骤

### Phase 1：octo_planner 发布占据体素（0.5 天）

1. `octo_planner_node.cpp` 新增 `publish_occupied_cloud()` 方法
2. 在 `setup_pub_sub()` 中注册 publisher（`transient_local` QoS）
3. 在 `configure_planner()` 末尾调用（首次发布）
4. 在 `reanalyze()` 末尾调用（编辑后更新）
5. `colcon build` 验证

### Phase 2：localPlanner 适配（1 天）

1. 新增参数声明（`declare_parameter`）：`stair_aware_mode` 等
2. 新增订阅：`/octomap_occupied_cloud` → 构建哈希表
3. 新增 `detect_stair_segments()` 方法 → `planned_path_callback` 末尾调用
4. 新增 `in_stair_corridor()` 辅助函数
5. 修改 `process_loop` 碰撞检测循环：走廊内点 → OctoMap 查表 → 决策跳过/保留
6. 新增 publisher `/near_stair_segment` → `process_loop` 末尾发布
7. `colcon build` + 启动验证

### Phase 3：pathFollower 适配（0.5 天）

1. 新增参数 `stair_tilt_threshold`
2. 新增订阅 `/near_stair_segment`
3. 倾斜停车逻辑中：楼梯段上使用 `stair_tilt_threshold` 替代 `inclThre_`
4. `colcon build` 验证

### Phase 4：集成测试（0.5 天）

1. 启动完整导航栈（`navigation.launch.py` + 多层地图）
2. 验证平地导航回归（`stair_aware_mode=false`）
3. 开启 `stair_aware_mode=true`：
   - 发送跨层目标 → 确认 localPlanner 不阻挡楼梯走廊
   - 在楼梯走廊内放置动态障碍物（如临时 box）→ 确认被正常检测
   - 走廊外障碍物 → 确认正常避障
4. Web UI 验证：OctoMap 占据云可视化

### 预计总工期：2-3 天

---

## 附录 A：本方案新增参数速查

### localPlanner 新增

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `stair_aware_mode` | `false` | 全局路径走廊信任模式开关 |
| `stair_slope_threshold` | 0.15 | Z 梯度 / XY 距离 > 此值视为楼梯段（≈8.5°） |
| `stair_corridor_xy_margin` | 0.5 | 走廊 XY 膨胀半宽（m） |
| `stair_corridor_z_margin` | 0.3 | 走廊 Z 膨胀容差（m） |
| `octomap_resolution` | 0.1 | 占据体素查找表量化精度（m） |

### localPlanner 现有参数（楼梯相关）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `minRelZ` | -0.5 | 障碍物下方可见范围（相对车辆） |
| `maxRelZ` | 0.25 | 障碍物上方可见范围 |
| `obstacleHeightThre` | 0.1~0.2 | 硬障碍高度阈值 |
| `groundHeightThre` | 0.1 | 地面高度变化阈值 |
| `useTerrainAnalysis` | false | 启用 intensity 地形分析 |

### pathFollower 新增

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `stair_tilt_threshold` | 60.0 | 楼梯段上的倾斜停车阈值（度，0=禁用） |

### 新增话题

| 话题 | 方向 | 类型 | QoS | 说明 |
|------|------|------|-----|------|
| `/octomap_occupied_cloud` | octo_planner → localPlanner | PointCloud2 | transient_local | OctoMap 原始占据体素 |
| `/near_stair_segment` | localPlanner → pathFollower | Bool | best_effort | 当前是否在楼梯走廊附近 |

## 附录 B：FAR Planner 关键参数（参考）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `is_multi_layer` | false | 多层/楼梯模式开关 |
| `is_static_env` | false | 静态环境（跳过动态障碍） |
| `voxel_dim` | 0.1m | PCL 降采样分辨率 |
| `robot_dim` | 0.5m | 机器人半径 |
| `vehicle_height` | 0.6m | 车辆高度 |
| `sensor_range` | 15.0m | 最大感知范围 |
| `local_planner_range` | 2.5m | 局部导航边界范围 |
| `main_run_freq` | 5.0Hz | 主循环频率 |
| `g_planner/converge_distance` | 0.4m | 到达目标判定距离 |
| `g_planner/votes_size` | 10 | 边投票队列长度 |

## 附录 C：原 autonomy_stack 完整架构

```
传感器层:
  LiDAR → /registered_scan (sensor_msgs/PointCloud2)
       → terrainAnalysis → /terrain_map (局部地形, intensity=高度变化)
       → terrainAnalysisExt → /terrain_map_ext (全局地形, 滚动体素网格)
       → ARISE SLAM → /state_estimation (里程计, nav_msgs/Odometry)

全局规划层:
  FAR Planner (5Hz, 在线可见图构建+搜索)
    输入:
      /odom_world          ← /state_estimation
      /terrain_cloud       ← /terrain_map_ext
      /scan_cloud          ← /terrain_map
      /terrain_local_cloud ← /registered_scan
      /goal_point          ← 用户设定
    输出:
      /way_point           → localPlanner
      /navigation_boundary → localPlanner
      /far_reach_goal_status → 监控

局部规划 + 执行层:
  localPlanner (100Hz, 预计算路径库碰撞检测)
    输入: /way_point + /navigation_boundary + /registered_scan + /state_estimation
    输出: /path + /slow_down + /surrounding_block
  pathFollower (100Hz, pure pursuit + P横摆控制)
    输入: /path + /slow_down + /surrounding_block + /state_estimation
    输出: /cmd_vel (geometry_msgs/Twist)

仿真层:
  vehicleSimulator ← /cmd_vel → /state_estimation (仿真里程计闭环)
```

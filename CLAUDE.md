# Dog3DNav - 机器狗3D导航框架

ROS 2 Humble 工作空间，面向四足机器狗平台的3D导航系统。

## 项目架构

```
Dog3DNav/
├── src/
│   ├── octo_planner/    # 全局3D路径规划器（ROS 2 package）
│   ├── local_planner/   # 局部规划 + 实时避障 + waypoint following（ROS 2 package）
│   ├── embodied/        # 具身感知理解模块（预留，暂不开发）
│   └── slam/            # 建图定位（git submodule，暂不管）
└── web/                 # Web 前端交互页面
```

## 模块说明

### octo_planner — 全局3D规划

基于 OctoPlanner3D 库（`~/Projects/NavProject/OctoPlanner3D/`）封装的 ROS 2 节点。

**核心能力：**
- 加载预建 PCD 点云地图，转为 OctoMap（`Pcd2OctomapConverter`）
- 基于 OctoMap 的 3D A* 路径搜索（`GlobalPlanner`）
- 可通行性分析：地面支撑检测、膨胀禁行区、代价地图

**开发原则：**
- OctoPlanner3D 作为外部库引用，尽量不改其源码
- 功能开发集中在 ROS 2 封装层：话题/服务接口、参数化配置、生命周期管理
- 如必须扩展库接口，在库层面做最小改动

**参考实现：** `~/Projects/NavProject/jie_3d_nav/jie_octomap/`
- Web 可视化方案（Three.js + roslib）
- PCD/OctoMap 转换节点
- 地图编辑交互

**OctoPlanner3D 库接口摘要：**
```cpp
// pcd2octomap::Pcd2OctomapConverter
bool convert();                              // PCD → OctoMap 主流程
std::shared_ptr<octomap::OcTree> getOctomap();
bool isPointFree(const octomap::point3d& p);
bool isSpaceFree(const octomap::point3d& min, const octomap::point3d& max);

// global_planner::GlobalPlanner
void setOctomap(std::shared_ptr<octomap::OcTree> map);
void makePlan(const PointPose start, const PointPose goal);
void getPlannerResults(std::vector<PointPose>& results);
```

### local_planner — 局部规划与避障

移植自 CMU autonomy_stack（`~/Projects/NavProject/autonomy_stack_mecanum_wheel_platform/src/base_autonomy/local_planner`）。

**原始系统功能：**
- `localPlanner.cpp` — 基于预生成路径集的局部规划，接收里程计 + 障碍物点云，输出速度指令
- `pathFollower.cpp` — waypoint following，跟踪全局路径下发的 waypoint 序列
- 预生成路径集（paths/*.ply）用于快速轨迹采样

**移植要点：**
- 原始为 ROS 2 节点，需适配本系统的话题/坐标系约定
- 与 octo_planner 的数据流对齐方案待确认（waypoint 格式、坐标系、更新频率等）
- 机器狗运动学适配（原始面向麦轮平台，需调整运动约束）

### slam — 建图与定位

Git submodule，由外部仓库导入。当前状态：预留。

**已知输出接口：**
- 全局一致性里程计（nav_msgs/Odometry）
- base_link → map 的 TF 变换
- 后处理点云（具体话题待定）

### embodied — 具身感知

预留模块，待具身智能技术方案确定后开发。

### web — 前端交互

独立前端应用，提供：
- 3D 地图可视化（OctoMap 体素渲染）
- 实时机器人位姿显示
- 路径可视化与导航目标设置
- 地图编辑（禁行区标注等）
- 手动运动控制（虚拟摇杆）
- 导航指令下发与状态监控

**通信方案：** 待确认（候选：rosbridge_suite WebSocket / 独立后端 API）

## 外部依赖路径

| 资源 | 路径 | 用途 |
|------|------|------|
| OctoPlanner3D 库 | `~/Projects/NavProject/OctoPlanner3D/` | 全局规划核心算法 |
| jie_octomap 参考 | `~/Projects/NavProject/jie_3d_nav/jie_octomap/` | Web可视化与地图管理参考实现 |
| autonomy_stack local_planner | `~/Projects/NavProject/autonomy_stack_mecanum_wheel_platform/src/base_autonomy/local_planner` | 局部规划移植源 |

## 构建与运行

```bash
# 构建（标准 colcon 工作流）
cd ~/Projects/NavProject/Dog3DNav
colcon build --symlink-install

# Source 环境
source install/setup.bash
```

## 坐标系约定

- `map` — 全局固定坐标系（SLAM 输出）
- `odom` — 里程计坐标系
- `base_link` — 机器人本体坐标系

## 待确认设计决策

1. **Web 通信方案**：rosbridge WebSocket vs 独立后端（FastAPI/Node.js）+ REST/WebSocket
2. **local_planner 数据流**：与 octo_planner 的 waypoint 传递格式、更新策略
3. **地图管理**：预建地图的存储格式、加载方式、运行时更新机制
4. **机器狗适配**：速度指令接口格式（Twist vs 自定义）、运动约束参数

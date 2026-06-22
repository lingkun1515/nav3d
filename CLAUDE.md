# Dog3DNav - 机器狗3D导航框架

ROS 2 Humble 工作空间，面向四足机器狗平台的3D导航系统。

## 项目架构

```
Dog3DNav/
├── src/
│   ├── octo_planner/    # 全局3D路径规划器（ROS 2 package）
│   ├── local_planner/   # 局部规划 + 实时避障 + waypoint following（ROS 2 package）
│   ├── simulation/      # Gazebo 仿真包（差速轮小车 URDF + PCD→世界场景生成）
│   ├── bringup/         # 启动配置汇总（launch 文件 + RViz2 配置 + 地图文件）
│   ├── embodied/        # 具身感知理解模块（预留，暂不开发）
│   └── slam/            # 建图定位（git submodule，暂不管）
├── maps/                # 地图预处理脚本（实际位置 src/bringup/maps/）
├── docs/                # 项目文档
├── web/                 # Web 前端交互页面
└── docker/              # Docker 配置
```

## 模块说明

### octo_planner — 全局3D规划

基于 OctoPlanner3D 库（`~/Projects/NavProject/OctoPlanner3D/`）封装的 ROS 2 节点。

**核心能力：**
- 多格式地图加载（`.pcd` / `.bt` / `.ot` / `.world` / `.sdf`）与自动缓存，详见 [quickstart](docs/quickstart.md)
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

**已实现功能：**
- `localPlanner` — 基于预生成路径集的局部规划 + 实时避障
  - 输入：`/scan`(LaserScan→PC2+TF) 或 `/registered_scan`(PointCloud2+TF)，`/odom`(通过 remap)，`/planned_path`(航点管理) 或 `/way_point`(直设目标)
  - 内部 TF 转换任意输入帧到 `global_frame_id` 参数指定的坐标系
  - 订阅 `/start_navigation`(Bool) 激活航点推进
  - 输出：`/path`(局部路径，vehicle 帧)，`/slow_down`，`/surrounding_block`
- `pathFollower` — pure-pursuit 路径跟踪 + Twist 指令发布
  - 订阅 `/path`(局部路径)、`/odom`(通过 remap)、`/stop_navigation`(Bool)、`/stop`(Int8)
  - 输出：`/cmd_vel`(Twist)，含安全停车、侧向避障、下坡减速逻辑
- 预生成路径集（paths/*.ply）用于快速轨迹采样

**关键参数（数据流控制）：**
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_laser_scan` | `false` | true=订阅 `/scan`(LaserScan)，内部转 PC2+TF |
| `use_planned_path` | `false` | true=订阅 `/planned_path`(Path)，自行管理航点 |
| `global_frame_id` | `"odom"` | 障碍物点云的目标坐标系 |

### simulation — Gazebo 仿真

ROS 2 package，提供仿真环境用于闭环导航调试。

**核心能力：**
- 差速驱动小车 URDF（Xacro），含 Gazebo 插件（diff_drive + lidar + joint_states）
- 发布里程计真值（`/odom`）、TF（odom→base_footprint→base_link）、激光扫描（`/scan`）
- 接受 `/cmd_vel`（Twist）控制小车移动
- bt_to_world / pcd_to_world：离线脚本，将 `.bt`/`.pcd` 转为 Gazebo `.world`（读取占据体素 → 贪婪合并 → SDF box）。手动运行，生成结果放在 `worlds/map_nav3d.world`。launch 文件不再动态生成世界
- nav\_bridge / waypoint\_follower 已删除：所有中继/控制逻辑已下沉到 localPlanner / pathFollower (C++)

**启动方式：**
```bash
# 仅仿真（空地）
ros2 launch simulation gazebo.launch.py
# 带障碍物（预生成的 world 文件）
ros2 launch simulation gazebo.launch.py world:=.../worlds/map_nav3d.world
```

### bringup — 启动与配置

ament_cmake package，集中管理所有 launch 文件和 RViz2 配置，无 C++/Python 节点。

**内容：**
- `launch/navigation.launch.py` — 导航栈启动（octo_planner + latticePlanner + pathFollower + rosbridge + RViz2），不含 Gazebo 仿真
- `config/navigation.rviz` — 导航栈 RViz2 可视化配置

**启动参数：**
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pcd_file` | `""` | 地图文件（支持 .pcd/.bt/.ot/.world/.sdf）。Gazebo 世界需离线生成（bt_to_world.py / pcd_to_world.py） |
| `launch_rviz` | `true` | 是否启动 RViz2 |
| `launch_rosbridge` | `false` | 是否启动 rosbridge WebSocket |
| `use_sim_time` | `true` | 使用仿真时间 |

```bash
# 仅导航栈（需另外启动 Gazebo 仿真）
ros2 launch bringup navigation.launch.py launch_rosbridge:=true
# 指定地图文件
ros2 launch bringup navigation.launch.py pcd_file:=/path/to/map.pcd
# 不启动 RViz2
ros2 launch bringup navigation.launch.py launch_rviz:=false
```

### slam — 建图与定位

Git submodule，由外部仓库导入。当前状态：预留。

**已知输出接口：**
- 全局一致性里程计（nav_msgs/Odometry）
- base_link → map 的 TF 变换
- 后处理点云（具体话题待定）

### embodied — 具身感知

预留模块，待具身智能技术方案确定后开发。

### web — 前端交互

独立前端应用（Three.js + ROSBridge），提供：
- 3D 地图可视化（OctoMap 体素渲染：占据/可通行/禁行/代价四层可切换）
- 实时机器人位姿显示（TF 解析 + 6DOF 坐标轴 + 狗模型）
- 点击可通行体素设置起/终点，拖拽设定航向角（箭头可视化）
- 导航目标下发 → 接收规划路径（亮青色发光管线）→ 弹窗确认执行/停止
- 事件日志面板（起终点坐标+航向、导航开始/停止、地图加载进度等）
- 地图编辑交互：笔刷添加/擦除占据体素，支持多尺寸笔刷 + Z 平面调节 + 即时本地渲染，200ms debounce 批量同步到 octo_planner（PointCloud2 → updateNode → reanalyze → republish），支持 .bt 保存/加载
- 手动运动控制（虚拟摇杆 + 旋转滑块，发布 `/web_cmd_vel`）

**通信方案：** rosbridge WebSocket（`ws://localhost:9090`），话题发布/订阅均在 main.js 中管理。

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

## 开发原则

1. **C++ 优先**：所有部署运行的算法包必须用 C++。Python 仅允许用于简单测试脚本（如 launch 文件、单元测试、一次性数据预处理脚本）。杜绝用 Python 写中继节点（remap 可替代的话题转发、应下沉到 C++ 节点的逻辑转换等）。
2. **代码位置**：功能应尽可能放入已有 C++ 节点（localPlanner / pathFollower），而非新增中继节点。新增 ROS 参数来控制行为切换。
3. **禁止主动 commit**：除非用户明确要求 commit，否则永远不要执行 git commit。先改代码、验证、等用户确认再提交。

## 已确定设计决策

1. **Web 通信方案**：rosbridge WebSocket（`ws://localhost:9090`），使用 roslib.min.js 客户端库
2. **地图管理**：octo_planner 支持多格式输入与自动缓存（详见 quickstart）；预处理脚本 `src/bringup/maps/map_preprocessor.py` 负责对齐/降采样/补全
3. **仿真机器人**：差速驱动小车（Gazebo diff_drive 插件），发布 `/odom` + TF + `/scan`，接收 `/cmd_vel`（Twist）

## 待解决

1. ~~**cmd_vel 类型适配**~~ ✓ 已修复：pathFollower 改为发布 `Twist`（原 `TwistStamped`）
2. ~~**nav_bridge.py**~~ ✓ 已删除：功能通过 launch remap + localPlanner C++ 内部实现（TF 坐标变换 + LaserScan→PointCloud2 转换 + `/planned_path` 航点管理）
3. ~~**local_planner 数据流**~~ ✓ 全链路已打通：localPlanner 直接订阅 `/scan`(LaserScan) 或 `/registered_scan`(PointCloud2)，通过 TF 转全局系；支持 `/planned_path`(航点管理) 或 `/way_point`(直设目标)；`/odom` 通过 remap 替代 `/state_estimation`
4. ~~**导航生命周期**~~ ✓ 已实现：`/start_navigation`(Bool) 由 Web UI 和 localPlanner 管理；`/stop_navigation`(Bool) 由 pathFollower 处理（安全停车）
5. ~~**waypoint_follower.py**~~ ✓ 已删除：`navigation.launch.py` 现已使用 localPlanner + pathFollower (C++) 做全闭环控制，PCD→world 场景自动生成
6. **机器狗运动学适配**：真实机器狗的速度指令接口（Twist vs 自定义）、运动约束参数

## 闭环导航测试

### 启动命令
```bash
# 仿真和导航栈需分开启动

# 终端 1: Gazebo 仿真
ros2 launch simulation gazebo.launch.py world:=.../worlds/map_nav3d.world

# 终端 2: 导航栈（octo_planner + latticePlanner + pathFollower + rosbridge + RViz2）
ros2 launch bringup navigation.launch.py launch_rosbridge:=true

# 空地模式（无 world 文件）
ros2 launch simulation gazebo.launch.py
ros2 launch bringup navigation.launch.py launch_rosbridge:=true

# 不启动 RViz2（仅终端）
ros2 launch bringup navigation.launch.py launch_rviz:=false launch_rosbridge:=true
```

### 数据流
```
完整管线 (仿真 + 导航栈):
  Web UI ─/goal_pose→ octo_planner ─/planned_path→ localPlanner (航点管理+TF)
         ─/start_navigation→ localPlanner
         ─/stop_navigation→ pathFollower
  Gazebo ─/odom→ (remap)→ /state_estimation→ localPlanner + pathFollower
         ─/scan→ localPlanner (内部 LaserScan→PointCloud2 + TF→odom)
  localPlanner ─/path→ pathFollower ─/cmd_vel→ Gazebo
  RViz2 可视化: TF + RobotModel + Odometry + /planned_path + /path + /scan + OctoMap
```

# Dog3DNav - 机器狗3D导航框架

面向四足机器狗平台的 3D 导航系统（ROS 2）。仓库同时维护两条分支：

| 分支 | 平台 | 状态 |
|------|------|------|
| `main` | Ubuntu 22.04 + ROS 2 Humble | 主开发线 |
| `foxy` | Ubuntu 20.04 + ROS 2 Foxy（运行于 `dog3dnav:foxy-gpu` Docker 容器，CUDA 11.8 + NVIDIA GPU） | Humble→Foxy 移植分支 |

> 当前所在分支：`foxy`。两条分支源码组织一致，本文档对二者均适用。容器化运行请见 [开发指南](docs/development.md)。

## 快速开始

```bash
cd ~/Projects/NavProject/Dog3DNav
colcon build --symlink-install
source install/setup.bash
```

> Foxy 分支推荐在容器内构建运行，避免本机环境差异。详见 [开发指南](docs/development.md)。

## 项目结构

```
Dog3DNav/
├── src/
│   ├── octo_planner/          # 全局 3D 路径规划器（OctoMap + A*），节点 octo_planner_node
│   │   └── thirdparty/        #   └─ 内置 OctoPlanner3D 库（直接编译，非外部安装）
│   ├── local_planner/         # 局部规划 + 实时避障 + waypoint following
│   │                          #   节点 latticePlanner（规划/避障）+ pathFollower（跟踪 → /cmd_vel）
│   ├── bringup/               # 全栈启动与配置中心：navigation.launch.py / slam.launch.py
│   │   ├── config/            #   ├─ navigation_config.yaml（导航栈统一参数）、slam_config.yaml、navigation.rviz
│   │   └── maps/              #   └─ 地图文件 + 预处理脚本（map_nav3d.bt/.pcd、map_preprocessor.py、npz_to_bt.py）
│   ├── simulation/            # Gazebo 仿真（差速轮小车 URDF + PCD/BT→.world 场景生成），launch gazebo.launch.py
│   ├── drivers/               # 外部驱动模块
│   ├── slam/                  # 建图/重定位（嵌套 git 仓库，Super-LIO），节点 super_lio_node / relocation_node
│   └── embodied/              # 具身感知理解模块（预留占位，暂未开发）
├── web/                       # Web 前端（Three.js + ROSBridge）：3D 可视化、选点导航、地图编辑
├── scripts/                   # 顶层工具脚本（rosbag 转换、清理脚本等）
├── docker/                    # Docker 配置（Dockerfile / build.sh / run.sh / entrypoint.sh，Foxy GPU 镜像）
└── docs/                      # 项目文档
```

> **关于嵌套仓库**：`src/drivers/livox_ros_driver2` 和 `src/slam`（Super-LIO）各自是独立的 git 仓库（有自己的 `.git` 与外部 GitHub 远程），暂未登记为 git submodule（仓库内无 `.gitmodules`）。它们由外部独立管理、克隆到本地即可参与 colcon 编译，不纳入本仓库版本控制。

## 文档

- [开发指南](docs/development.md) — 容器构建、运行、调试（Foxy 环境）
- [系统架构](docs/architecture.md) — 模块设计、数据流、关键机制

AI 开发上下文见 [CLAUDE.md](CLAUDE.md)。

## 核心组件

Dog3DNav 的核心组件基于开源项目集成、组装与改进：

| 组件 | 来源 | 节点 | 说明 |
|------|------|------|------|
| `octo_planner` | [OctoPlanner3D](https://github.com/JackJu-HIT/OctoPlanner3D)（内置 `thirdparty/`，直接编译） | `octo_planner_node` | 加载 PCD/OctoMap → 可通行性分析 → 3D A\* 全局路径规划 |
| `local_planner` | 移植自 CMU [autonomy_stack](https://github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform) | `latticePlanner` + `pathFollower` | 预生成路径集局部规划 + pure-pursuit 跟踪 → `/cmd_vel` |
| `web` | 参考 [jie_3d_nav](https://github.com/6-robot/jie_3d_nav) | — | Three.js + ROSBridge 3D 可视化、选点导航、地图编辑 |

在集成基础上本项目进行了大量功能改进与系统完善，详细设计见上方 [文档](#文档) 链接。

未来将逐步集成具身智能相关基础能力，敬请期待。

## 坐标系约定

导航栈统一以 **`map`** 作为全局固定坐标系（`octo_planner` / `local_planner` / SLAM 的 `frame_id` 均为 `map`，见 `navigation_config.yaml` 与 `slam_config.yaml`）。

- `map` — 全局固定坐标系（SLAM 建图/重定位输出，`super_lio` 配置 `lio.global.frame_id: "map"`）
- `odom` — 里程计坐标系（仿真时 Gazebo 真值 / 实机时由 SLAM 经 remap 提供 `/odom`）
- `imu` — IMU 坐标系（`super_lio` 动态发布 TF `map → imu`）
- `base_footprint` — 机器人足底投影（静态 TF，由 `slam.launch.py` 补齐）
- `base_link` — 机器人本体坐标系

## 实际部署

### 仿真全流程

```bash
# 终端 1 — Gazebo
ros2 launch simulation gazebo.launch.py

# 终端 2 — 导航(仿真已自带 /odom, 不需要 SLAM)
ros2 launch bringup navigation.launch.py
```

### 实机全流程

```bash
# 终端 1 — 启动 Livox 驱动(Mid360) + 确保 /livox/lidar /livox/imu 正常
# （按 Livox 官方文档配置，话题名需为 /livox/lidar 和 /livox/imu）

# 终端 2 — SLAM 建图
ros2 launch bringup slam.launch.py mode:=mapping
# 或 重定位(加载已有地图后使用)
ros2 launch bringup slam.launch.py mode:=relocation \
    init_pose:="[x, y, z, roll, pitch, yaw]"

# 终端 3 — 导航
ros2 launch bringup navigation.launch.py launch_rosbridge:=true
# go2_vel_bridge 包存在时自动启动，将 /cmd_vel 转发至 Unitree Go2 实机 API
```

> `slam.launch.py` 已完成话题/TF 适配，输出导航栈所需的 `/odom`、`/lidar_points` 与 `map → base_link` TF 链，详见下节。

### SLAM launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | `mapping` | `mapping`(建图) 或 `relocation`(重定位) |
| `rviz` | `false` | 是否启动 RViz2 可视化 |
| `use_sim_time` | `true` | 使用仿真/GPS 时间 |
| `init_pose` | `[0,0,0,0,0,0]` | 重定位初值 `[x,y,z,roll,pitch,yaw]` |

### Navigation launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pcd_file` | `bringup/maps/map_nav3d.bt` | 地图文件路径（`.bt` / `.pcd` / `.ot` / `.world` / `.sdf`） |
| `launch_rviz` | `true` | 是否启动 RViz2 可视化 |
| `launch_rosbridge` | `false` | 是否启动 rosbridge WebSocket（Web UI 需要） |
| `use_sim_time` | `false` | 使用仿真/GPS 时间 |

> `go2_vel_bridge` 包如存在则自动启动，无需额外参数。

### 话题与 TF 对齐

`slam.launch.py` 做了完整的话题/TF 适配，使 `navigation.launch.py` 无需任何改动即可与 SLAM 对接：

| 导航栈期望 | SLAM 原生输出 | 对齐方式 |
|-----------|--------------|---------|
| `/odom` | `/lio/odom` | `slam.launch.py` 内 remap |
| `/lidar_points` | `/lio/cloud_world` | `slam.launch.py` 内 remap |
| TF: `map → base_link` | `map → imu`（SLAM 动态发布） | `slam.launch.py` 补静态 TF `imu → base_footprint → base_link` |

> 注：`imu → base_footprint → base_link` 的静态外参当前为占位全零值，实机部署时需在 `slam.launch.py` 中填入机器人模型实际值。

### Web端

详见 [开发指南 - Web UI](docs/development.md#web-ui)。

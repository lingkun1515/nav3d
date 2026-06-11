# Dog3DNav - 机器狗3D导航框架

ROS 2 Humble 工作空间，面向四足机器狗平台的 3D 导航系统。

## 快速开始

```bash
cd ~/Projects/NavProject/Dog3DNav
colcon build --symlink-install
source install/setup.bash
```

## 项目结构

```
Dog3DNav/
├── src/
│   ├── octo_planner/    # 全局3D路径规划器（OctoMap + A*）
│   ├── local_planner/   # 局部规划 + 实时避障 + waypoint following
│   ├── simulation/      # Gazebo 仿真环境
│   └── slam/            # 建图定位（git submodule）
├── web/                 # Web 前端交互页面（Three.js + ROSBridge）
├── maps/                # 地图预处理工具
└── docs/                # 项目文档
```

## 文档

- [快速入门指南](docs/quickstart.md)
- [ROS 2 参数参考手册](docs/params_reference.md)
- [系统架构设计](docs/architecture.md)

## 模块概览

| 模块 | 说明 |
|------|------|
| `octo_planner` | 加载 PCD 点云 → OctoMap → 3D A* 全局路径规划 |
| `local_planner` | 基于预生成路径集的局部规划，接收里程计+障碍物，输出速度指令 |
| `simulation` | Gazebo 差速轮仿真小车，支持从点云生成仿真场景 |
| `web` | 3D 可视化、选点导航、手动控制、地图编辑 |

## 坐标系约定

- `map` — 全局固定坐标系（SLAM 输出）
- `odom` — 里程计坐标系
- `base_link` — 机器人本体坐标系

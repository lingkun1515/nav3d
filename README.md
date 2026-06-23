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
├── maps/                # 地图预处理工具（路径: src/bringup/maps/）
└── docs/                # 项目文档
```

## 文档

- [快速入门指南](docs/quickstart.md)
- [全闭环导航调试指南](docs/full_navigation_guide.md)
- [ROS 2 参数参考手册](docs/params_reference.md)
- [系统架构设计](docs/architecture.md)

## 核心组件

Dog3DNav 的三大核心组件基于开源项目集成、组装与改进：

| 组件 | 来源 | 说明 |
|------|------|------|
| `octo_planner` | [OctoPlanner3D](https://github.com/JackJu-HIT/OctoPlanner3D) | 加载 PCD/OctoMap → 可通行性分析 → 3D A\* 全局路径规划 |
| `local_planner` | [autonomy_stack](https://github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform) | 预生成路径集局部规划 + pure-pursuit 跟踪 → `/cmd_vel` |
| `web` | [jie_3d_nav](https://github.com/6-robot/jie_3d_nav) | Three.js + ROSBridge 3D 可视化、选点导航、地图编辑 |

在集成基础上本项目进行了大量功能改进与系统完善：

**建图与感知**
- 多格式地图加载（`.pcd` / `.bt` / `.ot` / `.world` / `.sdf`），自动 `.bt` 缓存
- 纯 Python `.bt` 解析器，无需 OctoMap 库依赖
- 确定性占据体素分解，loose 节点展开为叶节点
- 地面高度修复与中值滤波展平（保留楼梯/坡道梯度）
- 在线增量 OctoMap 更新设计（实时点云 → updateNode → 定时 reanalyze）

**导航规划**
- 全 C++ 闭环管线，消除所有 Python 中继节点
- 3D 楼梯攀爬：全局路径走廊信任机制（corridor trust）
- `/octomap_occupied_cloud` 发布 + `transient_local` QoS 同步
- pathFollower 楼梯段倾斜停车阈值自适应

**仿真打通**
- Gazebo 差速轮小车 URDF（含 lidar + diff_drive + joint_states 插件）
- 离线 PCD/BT → Gazebo `.world` 场景生成（占据体素 → 贪婪合并 → SDF box）
- 分辨率自适应解析、地面体素化、原生分辨率 world 构建
- `launch_sim` 开关，仿真/实车一键切换
- 仿真摄像头跟随机器人、`map` 帧里程计发布
- 完整导航闭环：Gazebo ↔ octo_planner ↔ localPlanner ↔ pathFollower

**控制与避障**
- 传感器高度感知过滤（`minRelZ` / `maxRelZ`），匹配雷达安装位置
- 狭窄通道容错优化（点云密度偏置消除）
- 贴墙距离与自由路径分布调优
- `twoWayDrive` / `pathCropByGoal` 行为修正
- `map` 帧全局坐标系统一

**Web 交互**
- rosbridge WebSocket 通信方案（`roslib.min.js`，话题发布/订阅 + 服务调用）
- Three.js 3D 可视化：OctoMap 四层可切换渲染（占据/可通行/禁行/代价）
- 点击选点设定起/终点 + 拖拽航向角
- 地面刷平笔刷：一键将区域内体素顶面平整为统一高度
- `.bt` 地图在线保存/加载
- 虚拟摇杆手动控制
- 事件日志面板（起终点坐标、导航状态、地图加载进度）

详细设计见 [文档目录](docs/)。

未来将逐步集成具身智能相关基础能力，敬请期待。

## 坐标系约定

- `world` — 全局固定坐标系（SLAM 输出, `super_lio` 发布）
- `odom` — 里程计坐标系（仿真时 Gazebo 真值 / 实机时由 SLAM remap 提供）
- `imu` — IMU 坐标系（`super_lio` 动态 TF 发布 `world→imu`）
- `base_footprint` — 机器人足底投影（静态 TF）
- `base_link` — 机器人本体坐标系

## 实际部署

### 仿真全流程

```bash
# 终端 1 — Gazebo
ros2 launch simulation gazebo.launch.py

# 终端 2 — 导航(仿真已自带 /odom, 不需要 SLAM)
ros2 launch bringup navigation.launch.py launch_rosbridge:=true
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

# 终端 4(宿主机) — Web UI
cd web && python3 -m http.server 8000
```

`slam.launch.py` 做了完整的话题适配, 输出 `/odom` 和 `/lidar_points`, `navigation.launch.py` 无需任何改动即可与 SLAM 对接。

### SLAM launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | `mapping` | `mapping`(建图) 或 `relocation`(重定位) |
| `rviz` | `false` | 是否启动 RViz2 可视化 |
| `use_sim_time` | `true` | 使用仿真/GPS 时间 |
| `init_pose` | `[0,0,0,0,0,0]` | 重定位初值 `[x,y,z,roll,pitch,yaw]` |

### 话题对齐

| 导航栈期望 | SLAM 原生 | 对齐方式 |
|-----------|----------|---------|
| `/odom` | `/lio/odom` | `slam.launch.py` 内 remap |
| `/lidar_points` | `/lio/cloud_world` | `slam.launch.py` 内 remap |
| TF: `world→base_link` | `world→imu`(SLAM 动态) | `slam.launch.py` 补静态 `imu→base_footprint→base_link` |

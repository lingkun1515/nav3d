# Dog3DNav 快速启动指南

## 环境依赖

- Ubuntu 22.04
- ROS 2 Humble
- Python 3 (含 Open3D)
- ros-humble-rosbridge-server
- PCL, liboctomap-dev

```bash
# 安装 ROS 依赖（如未安装）
sudo apt install ros-humble-rosbridge-server ros-humble-xacro \
  ros-humble-gazebo-ros-pkgs ros-humble-robot-state-publisher \
  ros-humble-octomap-msgs liboctomap-dev libpcl-all-dev \
  libtinyxml2-dev libeigen3-dev
```

## 编译

```bash
cd ~/Projects/NavProject/Dog3DNav
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 地图格式与加载

octo_planner 支持以下地图格式，通过文件扩展名自动检测加载器：

| 格式 | 扩展名 | 说明 |
|------|--------|------|
| PCD 点云 | `.pcd` | SLAM 原始点云，通过 `Pcd2OctomapConverter` 转为 OctoMap |
| OctoMap 二进制 | `.bt` | OctoMap 紧凑二进制格式，直接构造 `OcTree`，加载最快 |
| OctoMap 完整 | `.ot` | OctoMap 完整概率树格式，通过 `AbstractOcTree::read` 加载 |
| Gazebo 世界 | `.world` / `.sdf` | SDF 场景描述，通过 `world_loader`（tinyxml2 解析 + 体素化采样）转为 OctoMap |

### .bt 自动缓存

`auto_save_bt` 参数（默认 `true`）控制自动缓存行为：

```
首次加载 .pcd/.world/.ot
       │
       ▼
  格式转换 → OctoMap
       │
       ▼
  writeBinary("同目录同名.bt")   ← 自动保存缓存
       │
       ▼
  二次启动，检测到 .bt 存在且比源文件新
       │
       ▼
  OcTree(.bt) 直接构造  ← 跳过转换，秒级加载
```

- 缓存文件与源文件同目录、同名（仅扩展名不同）
- 删除 `.bt` 缓存可强制重新转换
- 参数 `world_xy_window_size_m`（默认 24.0m）控制 `.world`/`.sdf` 加载时的 XY 裁剪窗口

### PCD 点云预处理

将原始 SLAM 点云转为对齐后的导航地图：

```bash
conda activate dimos

# 仅坐标系校正（默认：对齐开、补全关、降采样关）
python3 src/bringup/maps/map_preprocessor.py <输入.pcd> maps/map_nav3d.pcd

# 启用体素降采样（稀疏大场景推荐）
python3 src/bringup/maps/map_preprocessor.py <输入.pcd> maps/map_nav3d.pcd --voxel_size 0.1

# 完整处理：对齐 + 补全 + 降采样 + 预览
python3 src/bringup/maps/map_preprocessor.py <输入.pcd> maps/map_nav3d.pcd \
    --voxel_size 0.1 --infill --visualize
```

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `--visualize` | 关 | Open3D 3D 预览 |
| `--no-align` | 关（对齐开） | 跳过坐标系校正 |
| `--infill` | 关 | 多楼层地面空洞补全 |
| `--voxel_size N` | 禁用 | 体素降采样网格大小 (m) |

处理步骤：体素降采样（可选）→ RANSAC 地面提取 + 重力对齐 → 法向量统计墙面方向 → XY 旋转对齐 → 平移原点 → 地面补全（可选）。

### jie_3d_nav NPZ 地图转换

将 jie_3d_nav 的 NPZ 地图包转为 `.bt` 格式后可直接加载：

```bash
python3 src/bringup/maps/npz_to_bt.py <地图包>/octomap_msg.npz maps/map.bt
# 或直接指定包目录
python3 src/bringup/maps/npz_to_bt.py --package-dir <地图包目录> maps/map.bt
```

## 启动系统

需要 3 个终端（或用一个终端后台启动）：

### 方式一：分终端启动

```bash
# 终端1：rosbridge WebSocket (供 Web 前端连接)
source /opt/ros/humble/setup.bash
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090

# 终端2：全局路径规划节点（支持 .pcd/.bt/.ot/.world/.sdf 任意格式）
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run octo_planner octo_planner_node --ros-args \
  -p pcd_file:=$HOME/Projects/NavProject/Dog3DNav/src/bringup/maps/map_nav3d.pcd \
  -p resolution:=0.2 \
  -p robot_radius:=0.05

# 终端3：Web 前端静态服务
cd ~/Projects/NavProject/Dog3DNav/web
python3 -m http.server 8080
```

### 方式二：后台一键启动

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

# 启动 rosbridge
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 &

# 启动 octo_planner
ros2 run octo_planner octo_planner_node --ros-args \
  -p pcd_file:=$HOME/Projects/NavProject/Dog3DNav/src/bringup/maps/map_nav3d.pcd \
  -p resolution:=0.2 -p robot_radius:=0.05 &

# 启动 Web 服务
cd web && python3 -m http.server 8080 &
```

### 方式三：launch 文件（含 rosbridge）

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch octo_planner planner.launch.py \
  pcd_file:=$HOME/Projects/NavProject/Dog3DNav/src/bringup/maps/map_nav3d.pcd

# 另开终端启动 Web
cd web && python3 -m http.server 8080
```

## 打开 Web 前端

浏览器访问：**http://localhost:8080**

首次加载后约 5 秒内，地图体素会出现（定时推送机制）。

## Web 前端操作

| 功能 | 操作 |
|------|------|
| 旋转视角 | 鼠标左键拖拽 |
| 平移视角 | 鼠标右键拖拽 / 中键拖拽 |
| 缩放 | 滚轮 |
| 重置视角 | 点击「重置视角」按钮 |
| 设置起点 | 点击「设置起点」→ 点击绿色可通行体素 → 拖拽设定朝向 |
| 设置终点 | 点击「设置终点」→ 同上 |
| 导航目标 | 点击「导航目标」→ 选点后自动触发规划 → 弹窗确认执行 |
| 停止导航 | 点击红色「停止导航」按钮 |
| 手动控制 | 拖拽虚拟摇杆 / 滑动旋转条 |
| 层级切换 | 勾选/取消 占据/可通行/禁行/代价 复选框 |

> **提示：** 设置起点/终点前，需要先勾选「可通行」层使其可见，才能点击选点。

## 常用 ROS 2 话题

```bash
# 查看所有话题
ros2 topic list

# 手动发布起点/终点测试规划
ros2 topic pub /start_point geometry_msgs/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 0.0, y: 0.0, z: 0.0}}" --once

ros2 topic pub /goal_point geometry_msgs/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 5.0, y: 3.0, z: 0.0}}" --once

# 查看规划结果
ros2 topic echo /planned_path --once
```

## 参数调节

编辑 `src/octo_planner/config/planner_params.yaml` 或在启动时通过 `--ros-args -p` 覆盖：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pcd_file` | `""` | 地图文件路径（支持 .pcd/.bt/.ot/.world/.sdf） |
| `resolution` | `0.2` | OctoMap 体素分辨率 (m) |
| `robot_radius` | `0.05` | 碰撞检测半径 (m) |
| `max_iterations` | `500000` | A* 最大迭代次数 |
| `require_ground_support` | `true` | 是否要求地面支撑 |
| `enable_preblocked_costmap` | `true` | 是否启用代价膨胀 |
| `preblocked_costmap_weight` | `2.5` | 代价权重 |
| `auto_save_bt` | `true` | 转换后自动保存 .bt 缓存 |
| `octomap_publish_period_s` | `5.0` | 数据重发周期 (s) |

## 故障排除

| 问题 | 解决 |
|------|------|
| Web 页面无地图显示 | 等待 5s 定时推送；检查 rosbridge 是否运行；F12 控制台看 WebSocket 连接状态 |
| 连接状态显示「已断开」 | 确认 rosbridge 在 9090 端口运行；检查防火墙 |
| 规划失败 | 确认起终点在可通行区域内；尝试增大 `snap_search_radius_cells` |
| 编译报错找不到 octomap | `sudo apt install liboctomap-dev ros-humble-octomap-msgs` |

## 下一步

- [全闭环导航调试指南](full_navigation_guide.md) — 在 Gazebo 仿真中运行完整的导航闭环（全局规划 + 局部避障 + 速度控制）
- [参数参考手册](params_reference.md) — 全部 ROS 参数详细说明

# Dog3DNav 开发指南

所有开发在 Docker 容器中进行，宿主机仅用于代码编辑和 Web 服务。

## 环境

| 项目 | 值 |
|------|-----|
| 容器名 | `dog3dnav-foxy` |
| 镜像 | `dog3dnav:foxy-gpu`（Ubuntu 20.04 + ROS 2 Foxy + CUDA 11.8） |
| 工作空间 | 宿主机 `~/Projects/NavProject/Dog3DNav` ↔ 容器 `/ros2_ws`（bind-mount） |
| 网络 | host 模式（rosbridge 9090、X11 直通） |
| 重启 | `unless-stopped`（开机自动启动） |

宿主机改代码容器内立即可见，**无需重启容器**。

## 进入容器

```bash
docker exec -it dog3dnav-foxy bash
source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.bash  # 首次构建前可忽略
```

## 构建

```bash
cd /ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

注意事项：
- **不要用 `--symlink-install`**（容器内 symlink 路径不兼容）
- 单包构建：`colcon build --packages-select <包名> --cmake-args -DCMAKE_BUILD_TYPE=Release`
- 改 launch / yaml / rviz 文件不需要重新 build

## 运行仿真

需要两个终端（各 `docker exec` 进入容器）：

```bash
# 终端 1: Gazebo（默认 urban2_story.world，含楼梯/平台）
ros2 launch simulation gazebo.launch.py
# 空地模式:
ros2 launch simulation gazebo.launch.py world:=$(ros2 pkg prefix simulation)/share/simulation/worlds/empty.world

# 终端 2: 导航栈
ros2 launch bringup navigation.launch.py launch_rosbridge:=true
```

Gazebo 启动参数：`robot_model:=car|a1`、`world:=...`、`x/y/z/yaw` 初始位姿。

## 运行实机

```bash
# 终端 1: Livox 驱动（输出 /livox/lidar、/livox/imu）
# 终端 2: SLAM
ros2 launch bringup slam.launch.py mode:=mapping
# 终端 3: 导航
ros2 launch bringup navigation.launch.py launch_rosbridge:=true
```

SLAM 已做话题/TF 适配（详见 `src/bringup/launch/slam.launch.py`），导航栈无需任何改动。

## Web UI

在**宿主机**启动：

```bash
cd ~/Projects/NavProject/Dog3DNav/web
python3 -m http.server 8000
# 浏览器 → http://localhost:8000
```

| 功能 | 操作 |
|------|------|
| 旋转/平移/缩放 | 鼠标左键拖拽 / 右键拖拽 / 滚轮 |
| 设置起点 | 点击「设置起点」→ 点击绿色可通行体素 → 拖拽设朝向 |
| 设置终点 | 点击「设置终点」→ 同上 |
| 导航目标 | 点击「导航目标」→ 选点 → 弹窗确认执行 |
| 停止导航 | 点击红色「停止导航」 |
| 手动控制 | 拖拽虚拟摇杆 / 滑动旋转条 |
| 层级切换 | 勾选 占据/可通行/禁行/代价 复选框 |
| 编辑地图 | 点击「编辑地图」→ 选添加/擦除 → 拖拽刷地形 → Q/E 升降笔刷 |
| 保存/加载地图 | 编辑模式下点击对应按钮 → 浏览目录 → 确认 |

> 选点前需勾选「可通行」层使其可见。笔刷 1x1/3x3/5x5 控制大小，200ms debounce 批量同步到 octo_planner。

## Web UI 操作流程

1. 浏览器打开 `http://localhost:8000`
2. 等待连接状态显示「已连接 ROSBridge」
3. 等待地图数据接收完成
4. 点击「导航目标」→ 在绿色可通行层上选点 → 拖拽设航向角
5. 弹窗确认 → 点击「开始导航」
6. 随时点击「停止导航」紧急停止

## 常用调试命令

```bash
# 查看所有节点/话题
ros2 node list
ros2 topic list

# 查看话题内容（Foxy 不支持 --once，用 timeout）
timeout 3 ros2 topic echo /odom | head -15
timeout 3 ros2 topic echo /planned_path | head -20

# 查看话题频率
ros2 topic hz /odom
ros2 topic hz /scan

# 手动发目标测试规划
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}" -1

# 检查 TF 树
ros2 run tf2_tools view_frames

# 手动控制（测试控制链路）
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" &
sleep 3; kill %1
```

## 话题对照表

| 话题 | 类型 | 来源 → 目标 |
|------|------|-------------|
| `/goal_pose` | PoseStamped | Web UI → octo_planner |
| `/start_point` | PointStamped | Web UI → octo_planner |
| `/planned_path` | Path | octo_planner → latticePlanner |
| `/odom` | Odometry | Gazebo/SLAM → latticePlanner, pathFollower |
| `/scan` | LaserScan | Gazebo → latticePlanner |
| `/path` | Path | latticePlanner → pathFollower |
| `/slow_down` | Int8 | latticePlanner → pathFollower |
| `/cmd_vel` | Twist | pathFollower → Gazebo/实机 |
| `/stop_navigation` | Bool | Web UI → pathFollower |
| `/web_cmd_vel` | Twist | Web UI → pathFollower |
| `/lidar_points` | PointCloud2 | SLAM → octo_planner（online update） |

## 容器管理

```bash
docker stop dog3dnav-foxy      # 停止
docker start dog3dnav-foxy     # 启动
docker restart dog3dnav-foxy   # 重启
docker rm -f dog3dnav-foxy     # 删除容器
```

### 重建容器

```bash
xhost +local:docker 2>/dev/null
docker run -d \
  --gpus all --net=host --privileged \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/lenovo/Projects/NavProject/Dog3DNav:/ros2_ws \
  --name dog3dnav-foxy \
  --restart unless-stopped \
  dog3dnav:foxy-gpu sleep infinity
```

### 重建镜像

```bash
cd ~/Projects/NavProject/Dog3DNav
bash docker/build.sh
# 重建后需重建容器
```

# Dog3DNav Humble → Foxy 移植文档

本分支（`foxy`）将主分支 `master`（Ubuntu 22.04 + ROS 2 Humble）移植到 Ubuntu 20.04 + ROS 2 Foxy，运行于 `dog3dnav:foxy-gpu` Docker 容器（CUDA 11.8 + NVIDIA GPU）。

## 快速开始

```bash
# 1. 确认在 foxy 分支
git checkout foxy

# 2. 重建镜像（依赖变更后需要）
bash docker/build.sh

# 3. 启动容器（GPU + X11 + 挂载工作空间）
bash docker/run.sh
# 或手动：
xhost +local:docker
docker run -it --rm --gpus all --net=host --privileged \
  -e DISPLAY=$DISPLAY -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/lenovo/Projects/NavProject/Dog3DNav:/ros2_ws \
  --name dog3dnav dog3dnav:foxy-gpu

# 4. 容器内构建（首次或代码变更后）
source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 5. 运行（两个终端/两个容器实例）
#    终端 1: Gazebo 仿真
ros2 launch simulation gazebo.launch.py
#    终端 2: 导航栈
ros2 launch bringup navigation.launch.py launch_rosbridge:=true
```

> **注意**：`colcon build` 不要加 `--symlink-install`。`src/simulation/models/` 下有指向宿主机绝对路径的 symlink，在容器内无效，symlink install 会因无法创建这些符号链接而失败。普通 build（copy 模式）正常工作。

---

## Humble → Foxy 关键差异与修复

### 1. tf2 头文件后缀：`.hpp` vs `.h`（5 处）

| 类型 | Foxy | Humble | 规律 |
|------|------|--------|------|
| **消息包**（如 `tf2_msgs`） | 同时有 `.h`（C 接口）和 `.hpp`（C++ 命名空间） | 统一 `.hpp` | C++ 代码**必须用 `.hpp`** |
| **库包**（如 `tf2_sensor_msgs`、`tf2_geometry_msgs`） | 只有 `.h` | `.hpp` | Foxy **必须用 `.h`** |

**修改的文件**：
- `src/local_planner/src/local_planner_node.cpp`：`tf2_geometry_msgs.hpp` → `.h`，`tf2_sensor_msgs.hpp` → `.h`
- `src/local_planner/src/path_follower_node.cpp`：`tf2_geometry_msgs.hpp` → `.h`
- `src/octo_planner/src/octo_planner_node.cpp`：`tf2_sensor_msgs.hpp` → `.h`
- `src/simulation/src/a1_rl_controller.hpp`：`tf2_msgs/msg/tf_message.hpp` **保持 .hpp**（消息包）

> **教训**：不能无脑把所有 `.hpp` 改成 `.h`。消息包（`*_msgs`）的 `.h` 是纯 C 接口，不含 C++ 命名空间声明；只有 `.hpp` 才有 `namespace::msg::Type`。

### 2. simulation 包 tf2_msgs 依赖缺失

`a1_rl_controller.hpp` 使用 `tf2_msgs::msg::TFMessage`，但 simulation 的 `CMakeLists.txt` / `package.xml` 没有声明 `tf2_msgs` 依赖。Humble 下靠间接包含侥幸通过，Foxy 下 `find_package` 找不到。

**修复**：
- `CMakeLists.txt`：`find_package(tf2_msgs REQUIRED)` + `ament_target_dependencies` + 显式 `target_include_directories`
- `package.xml`：`<depend>tf2_msgs</depend>`
- `a1_rl_controller.hpp`：显式 `#include <geometry_msgs/msg/transform_stamped.hpp>`

### 3. Gazebo factory 插件服务注册失败（最关键的运行时问题）

**症状**：`gzserver` 启动正常，`libgazebo_ros_factory.so` 加载到内存，但 `/spawn_entity` 服务永不出现，`spawn_entity.py` 30 秒后超时。

**根因**：Foxy 的 `gzserver.launch.py`（`/opt/ros/foxy/share/gazebo_ros/launch/gzserver.launch.py`）这样计算插件路径：
```python
model, plugin, media = GazeboRosPaths.get_paths()  # plugin = '' (无包导出)
if 'GAZEBO_PLUGIN_PATH' in environ:
    plugin += pathsep + environ['GAZEBO_PLUGIN_PATH']  # '':'+ourpath' = ':/ourpath'
env['GAZEBO_PLUGIN_PATH'] = plugin  # 传给 gzserver: ':/ourpath' ← 冒号开头!
```
`GAZEBO_PLUGIN_PATH` 以冒号开头，使 Gazebo 把**当前工作目录**加入插件搜索路径，破坏了 `libgazebo_ros_factory.so` 的服务注册机制。

**修复**（3 处协同）：
1. **`simulation/package.xml`**：新增 `<gazebo_ros plugin_path="${prefix}/lib"/>` 导出 → `GazeboRosPaths.get_paths()` 返回非空路径 → 拼接结果无冒号开头
2. **`gazebo.launch.py`**：不再设 `os.environ['GAZEBO_PLUGIN_PATH']`，改为通过 `SetEnvironmentVariable` 传播完整路径
3. **`Dockerfile`**：`ENV LD_LIBRARY_PATH=/opt/ros/foxy/lib:${LD_LIBRARY_PATH}` 确保 Gazebo 插件的 ROS 库依赖可被发现

### 4. 硬编码 Humble 路径

`gazebo.launch.py` 原有 `ros_gazebo_plugins = '/opt/ros/humble/lib'` 硬编码。改为按 `$ROS_DISTRO` 动态解析：
```python
ros_gazebo_plugins = os.path.join('/opt', 'ros', os.environ.get('ROS_DISTRO', 'foxy'), 'lib')
```
两个分支天然兼容，合并无冲突。

### 5. Dockerfile 依赖补全

原 Dockerfile 缺少关键依赖（镜像能构建但无法运行仿真）：
- `ros-foxy-gazebo-ros-pkgs`（Gazebo 11 + ROS 桥）
- `ros-foxy-pcl-conversions` / `ros-foxy-pcl-ros`（local_planner 依赖）
- `ros-foxy-xacro`（URDF 处理）
- `ros-foxy-joint-state-publisher` / `ros-foxy-robot-state-publisher`
- `ros-foxy-tf2-sensor-msgs` / `ros-foxy-rviz2`

### 6. Dockerfile 架构调整

- **移除镜像内的 `colcon build`**：原 Dockerfile 在构建镜像时编译源码快照，但开发时源码已变更。改为纯环境镜像，源码在运行时挂载本地目录后编译。
- **apt 重试机制**：NVIDIA CUDA 仓库偶发 DNS 抖动，添加 `--fix-missing` + 5 次重试循环。
- **entrypoint 容错**：`install/setup.bash` 不存在时跳过（首次启动未编译的容器可正常 boot）。

### 7. 版本控制

`.gitignore` 原有 `docker/` 项导致 Dockerfile 等不入库。已移除，让 `docker/` 目录纳入版本控制。

---

## 已验证的功能

容器内（`dog3dnav:foxy-gpu`）完整验证通过：

- ✅ **4 个包全部编译**：octo_planner、local_planner、simulation、bringup（共 18.9s）
- ✅ **Gazebo 11.11.0 启动**：A1 机器狗模型加载，URDF/xacro 解析正常
- ✅ **A1RLController 插件**：12 关节识别、ONNX RL 模型加载、站立→RL_RUNNING 切换
- ✅ **话题发布**：`/odom`、`/cmd_vel`、`/lidar_points`、`/joint_states`、`/tf`、`/trunk_imu`
- ✅ **导航栈全节点**：octo_planner + latticePlanner + pathFollower + rosbridge 全在线
- ✅ **rosbridge WebSocket**：9090 端口监听，Web 客户端可连接订阅话题

## 已知限制

1. **`--symlink-install` 不可用**：`src/simulation/models/` 有宿主机绝对路径的 symlink，容器内无效。用普通 `colcon build`（copy 模式）。
2. **`ros2 topic echo --once` / `--timeout`**：Foxy 不支持这些参数（Humble 新增）。用 `timeout 3 ros2 topic echo /topic | head -N` 替代。
3. **GPU 驱动版本**：宿主机 CUDA 13.0 驱动，容器用 CUDA 11.8 runtime，向前兼容正常。RTX 5060 显卡验证通过。

## 双分支同步策略

`foxy` 分支的改动设计为与 `master`（humble）兼容，便于双向 cherry-pick：
- tf2 头文件后缀：Humble/Foxy 用不同后缀，cherry-pick 时需手动调整这几行
- `gazebo.launch.py` 的 `ROS_DISTRO` 动态路径：两个分支代码一致
- `package.xml` 的 `gazebo_ros` 导出：对 Humble 无害（Humble 也支持），可直接合并
- Dockerfile 改动：仅影响 foxy 分支的镜像构建

## 移植提交历史

```
bdfa7aa port(foxy): fix Gazebo factory plugin registration, A1 sim fully works
dadca57 port(foxy): fix tf_message header suffix, all 4 packages build
186d8a4 port(foxy): fix simulation tf2_msgs dep, decouple image build from source
<hash> port(foxy): fix tf2 headers, humble path, Dockerfile deps
```

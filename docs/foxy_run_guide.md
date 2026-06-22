# Dog3DNav Foxy 容器使用指南

持久容器 `dog3dnav-foxy` 已创建，开机自动启动。本文档说明如何进入容器、构建代码、运行全流程仿真。

---

## 一、容器概况

| 项目 | 值 |
|------|-----|
| 容器名 | `dog3dnav-foxy` |
| 镜像 | `dog3dnav:foxy-gpu`（Ubuntu 20.04 + ROS 2 Foxy + CUDA 11.8） |
| 重启策略 | `unless-stopped`（开机自动启动，除非手动 `docker stop`） |
| 工作空间 | 宿主机 `/home/lenovo/Projects/NavProject/Dog3DNav` ↔ 容器 `/ros2_ws` |
| GPU | 全部透传（`--gpus all`） |
| 网络 | host 模式（rosbridge 9090、X11 直通） |

> **重要**：宿主机 `/home/lenovo/Projects/NavProject/Dog3DNav` 目录已挂载到容器 `/ros2_ws`。你在宿主机用编辑器改代码，容器内立即可见，**无需重启容器**。改完代码只需在容器内重新 `colcon build`。

---

## 二、开机后进入容器

开机后 Docker 服务会自动启动该容器（因为 `--restart unless-stopped`）。进入容器：

```bash
docker exec -it dog3dnav-foxy bash
```

进去后先 source 环境：

```bash
source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.bash
```

> 如果提示 `install/setup.bash` 不存在（首次或清理过），先执行第三节"构建代码"。

---

## 三、构建代码（代码变更后执行）

在容器内：

```bash
cd /ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**注意事项**：
- **不要用 `--symlink-install`**。`src/simulation/models/` 下有指向宿主机绝对路径的 symlink，在容器内无效，symlink install 会失败。普通 build（copy 模式）正常。
- 如果只改了一个包，可单独构建加速：`colcon build --packages-select <包名> --cmake-args -DCMAKE_BUILD_TYPE=Release`
- 改了 launch 文件或配置（yaml/rviz）通常不需要重新 build（运行时直接读源文件）。

---

## 四、运行全流程仿真

需要**两个终端**（可以是两个 `docker exec` 窗口，或 tmux 两个 pane）。

### 终端 1：Gazebo 仿真

```bash
docker exec -it dog3dnav-foxy bash
source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DISTRO=foxy

# 带场景（楼梯/平台，推荐）
ros2 launch simulation gazebo.launch.py

# 空地（无障碍物，快速测试）
ros2 launch simulation gazebo.launch.py world:=$(ros2 pkg prefix simulation)/share/simulation/worlds/empty.world
```

等待直到看到：
```
[a1_rl_controller]: A1RLController loaded successfully
[a1_rl_controller]: Stand-up complete, switching to RL_RUNNING
```
说明 A1 机器狗已站立并进入 RL 控制状态。

### 终端 2：导航栈

```bash
docker exec -it dog3dnav-foxy bash
source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DISTRO=foxy

ros2 launch bringup navigation.launch.py launch_rosbridge:=true
```

启动后所有节点在线：`octo_planner_node`、`latticePlanner`、`pathFollower`、`rosbridge_websocket`。

### 终端 3（可选）：Web UI

在**宿主机**（不是容器内）启动 Web 服务：

```bash
cd ~/Projects/NavProject/Dog3DNav/web
python3 -m http.server 8000
```

浏览器打开 `http://localhost:8000`，即可看到 3D 地图、机器人位姿，点击下目标导航。

---

## 五、常用验证命令

在容器内（已 source 环境）：

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看 A1 位姿（Foxy 不支持 --once，用 timeout）
timeout 3 ros2 topic echo /odom | head -15

# 查看话题发布频率
ros2 topic hz /odom

# 手动控制机器狗前进（测试控制链路）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" &
sleep 3; kill %1

# 查看导航栈是否收到目标
ros2 topic echo /planned_path
```

---

## 六、停止 / 重启 / 清理

```bash
# 手动停止容器（停止后开机不会自动启动，除非再 docker start）
docker stop dog3dnav-foxy

# 重新启动已停止的容器
docker start dog3dnav-foxy

# 重启容器（清理运行中的进程，保留文件系统）
docker restart dog3dnav-foxy

# 进入容器
docker exec -it dog3dnav-foxy bash
```

**彻底清理（删除容器，下次需重新创建）**：
```bash
docker rm -f dog3dnav-foxy
```

---

## 七、重新创建容器（如被删除）

如果容器被误删，用以下命令重建（参数已固化，直接复制）：

```bash
xhost +local:docker 2>/dev/null
docker run -d \
  --gpus all \
  --net=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/lenovo/Projects/NavProject/Dog3DNav:/ros2_ws \
  --name dog3dnav-foxy \
  --restart unless-stopped \
  dog3dnav:foxy-gpu sleep infinity
```

---

## 八、镜像重建（依赖变更后）

如果 Dockerfile 或依赖发生变化，重建镜像：

```bash
cd ~/Projects/NavProject/Dog3DNav
bash docker/build.sh
```

重建后需要重启容器以使用新镜像：
```bash
docker rm -f dog3dnav-foxy
# 然后按第七节重新创建容器
```

---

## 九、故障排查

### 问题：开机后容器没自动启动
```bash
# 检查 Docker 服务是否开机自启
systemctl is-enabled docker
# 如未启用
sudo systemctl enable docker

# 检查容器重启策略
docker inspect -f '{{.HostConfig.RestartPolicy.Name}}' dog3dnav-foxy
# 应输出 "unless-stopped"
```

### 问题：Gazebo 窗口不显示（X11 权限）
```bash
# 宿主机执行
xhost +local:docker
# 然后重启容器
docker restart dog3dnav-foxy
```

### 问题：`/spawn_entity` 服务不可用
这是 Foxy 的已知坑点（见 `docs/foxy_porting.md` 第 3 节）。确保：
- `simulation/package.xml` 有 `<gazebo_ros plugin_path="${prefix}/lib"/>` 导出
- `gazebo.launch.py` 不设置 `os.environ['GAZEBO_PLUGIN_PATH']`
- 启动 launch 前 `export ROS_DISTRO=foxy`

### 问题：GPU 不可用
```bash
# 容器内检查
nvidia-smi
# 如失败，宿主机检查 NVIDIA 驱动
nvidia-smi
# 重启容器
docker restart dog3dnav-foxy
```

### 问题：仿真跑起来但机器狗掉到地下
empty world 没有地面。改用带场景的 world：
```bash
ros2 launch simulation gazebo.launch.py  # 默认用 urban2_story.world
```

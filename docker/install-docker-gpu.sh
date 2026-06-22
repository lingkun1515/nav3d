#!/bin/bash
# Docker Engine + NVIDIA Container Toolkit 安装脚本（国内镜像）
# 适用于 Ubuntu 22.04 (jammy)
# 使用方法: sudo bash docker/install-docker-gpu.sh

set -e

echo "=== 1. 安装 Docker Engine（使用阿里云镜像）==="

# 卸载旧版本（如有）
apt-get remove -y docker docker-engine docker.io containerd runc 2>/dev/null || true

# 安装依赖
apt-get update
apt-get install -y ca-certificates curl gnupg

# 添加 Docker GPG key（阿里云镜像）
install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://mirrors.aliyun.com/docker-ce/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
chmod a+r /etc/apt/keyrings/docker.gpg

# 添加 Docker 仓库（阿里云镜像）
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://mirrors.aliyun.com/docker-ce/linux/ubuntu \
  $(lsb_release -cs) stable" > /etc/apt/sources.list.d/docker.list

# 安装 Docker
apt-get update
apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# 将当前用户加入 docker 组（免 sudo）
usermod -aG docker ${SUDO_USER:-$USER}

# 配置 Docker 镜像加速器
mkdir -p /etc/docker
cat > /etc/docker/daemon.json <<EOF
{
  "registry-mirrors": [
    "https://docker.1ms.run",
    "https://docker.xuanyuan.me"
  ]
}
EOF

systemctl daemon-reload
systemctl restart docker

echo "=== 2. 安装 NVIDIA Container Toolkit ==="

# 添加 GPG key（如果还没有）
if [ ! -f /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg ]; then
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
fi

# 直接写入 apt 源（避免 curl list 文件为空的问题）
echo "deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://nvidia.github.io/libnvidia-container/stable/deb/$(dpkg --print-architecture) /" > /etc/apt/sources.list.d/nvidia-container-toolkit.list

apt-get update
apt-get install -y nvidia-container-toolkit

# 配置 Docker runtime 支持 GPU
nvidia-ctk runtime configure --runtime=docker
systemctl restart docker

echo "=== 3. 验证安装 ==="
docker --version
echo ""
echo "验证 GPU 支持（需要重新登录后或使用 newgrp docker）:"
echo "  docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu20.04 nvidia-smi"
echo ""
echo "=== 安装完成！请重新登录终端以使 docker 组生效 ==="

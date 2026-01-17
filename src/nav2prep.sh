#!/bin/bash
# 在ros2 foxy 安装最新版本的nav2用于no-cov fuzz
# install_nav2_ustc.sh

echo "=== 安装 Nav2 - 使用国内镜像源（自动回退） ==="

SUDO=""
if [ "$(id -u)" -ne 0 ]; then
  SUDO="sudo"
fi

# 1. Ubuntu 源（中科大）
echo "配置中科 Ubuntu 源..."
$SUDO tee /etc/apt/sources.list > /dev/null << 'EOF'
deb https://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse
EOF

# 2. ROS 2 源（国内镜像回退）
$SUDO apt install -y curl gnupg2 lsb-release
$SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
$SUDO rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2-latest.list /etc/apt/sources.list.d/ros2-testing.list

ros2_mirrors=(
  "https://mirrors.aliyun.com/ros2/ubuntu/"
  "https://mirrors.bfsu.edu.cn/ros2/ubuntu/"
  "https://mirrors.cloud.tencent.com/ros2/ubuntu/"
  "https://mirrors.iscas.ac.cn/ros2/ubuntu/"
  "https://mirror.sjtu.edu.cn/ros2/ubuntu/"
  "https://repo.huaweicloud.com/ros2/ubuntu/"
  "https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/"
  "https://mirrors.ustc.edu.cn/ros2/ubuntu/"
)

# 3. 更新（清理旧索引）
install_ok=0
for mirror in "${ros2_mirrors[@]}"; do
  echo "配置 ROS 2 源: $mirror"
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] $mirror $(lsb_release -cs) main" | $SUDO tee /etc/apt/sources.list.d/ros2.list

  $SUDO apt clean
  $SUDO rm -rf /var/lib/apt/lists/*
  if ! $SUDO apt update; then
    echo "apt update 失败，尝试下一个镜像..."
    continue
  fi

  echo "安装 ROS Foxy 桌面版..."
  if $SUDO apt install -y --fix-missing ros-foxy-desktop; then
    install_ok=1
    break
  fi

  echo "安装失败，尝试下一个镜像..."
done

if [ "$install_ok" -ne 1 ]; then
  echo "所有 ROS2 镜像都失败，请检查网络或更换镜像。"
  exit 1
fi

# 5. 安装 Nav2（使用循环处理失败情况）
nav2_packages=(
  ros-foxy-navigation2
  ros-foxy-nav2-bringup
  ros-foxy-nav2-common
  ros-foxy-nav2-core
  ros-foxy-nav2-msgs
  ros-foxy-nav2-controller
  ros-foxy-nav2-planner
  ros-foxy-nav2-navfn-planner
  ros-foxy-nav2-dwb-controller
  ros-foxy-nav2-amcl
  ros-foxy-nav2-behavior-tree
  ros-foxy-nav2-bt-navigator
  ros-foxy-nav2-costmap-2d
  ros-foxy-nav2-lifecycle-manager
  ros-foxy-nav2-map-server
  ros-foxy-nav2-recoveries
  ros-foxy-nav2-rviz-plugins
  ros-foxy-nav2-util
  ros-foxy-nav2-voxel-grid
  ros-foxy-nav2-waypoint-follower
  ros-foxy-slam-toolbox
)

echo "安装 Nav2 相关包..."
for pkg in "${nav2_packages[@]}"; do
  echo "正在安装: $pkg"
  $SUDO apt install -y --fix-missing "$pkg" || echo "跳过 $pkg，继续安装其他包..."
done

echo "安装完成！"

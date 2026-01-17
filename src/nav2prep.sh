#!/bin/bash
# 在ros2 foxy 安装最新版本的nav2用于no-cov fuzz
# install_nav2_ustc.sh

echo "=== 安装 Nav2 - 使用中科大源 ==="

# 1. Ubuntu 源（中科大）
echo "配置中科 Ubuntu 大源..."
sudo tee /etc/apt/sources.list > /dev/null << 'EOF'
deb https://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse
EOF

# 2. ROS 2 源（中科大）
echo "配置中科大 ROS 2 源..."
sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.ustc.edu.cn/ros2/ubuntu/ $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list

# 3. 更新
sudo apt update

# 4. 安装（使用 --fix-missing 自动修复）
echo "安装 ROS Foxy 桌面版..."
sudo apt install -y --fix-missing ros-foxy-desktop

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
  sudo apt install -y --fix-missing "$pkg" || echo "跳过 $pkg，继续安装其他包..."
done

echo "安装完成！"
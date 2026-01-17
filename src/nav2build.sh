# 创建 workspace
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws/src

# 克隆 nav2 源码
git clone https://github.com/ros-planning/navigation2.git -b foxy

# 安装依赖
cd ..
rosdep install -y --from-paths src --ignore-src --rosdistro foxy

# 编译
colcon build --symlink-install --packages-select nav2_controller nav2_recoveries
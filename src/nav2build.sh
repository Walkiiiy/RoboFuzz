# sudo apt-get update
# 1. 安装所有系统依赖
echo "安装所有系统依赖..."
sudo apt-get update


# 1. 安装Ceres Solver
echo "安装Ceres Solver..."
sudo apt-get install -y \
    libceres-dev \
    libsuitesparse-dev \
    libcxsparse3 \
    libgflags-dev \
    libgoogle-glog-dev \
    libatlas-base-dev

sudo apt-get install -y \
    libgraphicsmagick++-dev \
    libgraphicsmagick1-dev \
    graphicsmagick-libmagick-dev-compat \
    graphicsmagick \
    libzmq3-dev \
    libsqlite3-dev \
    libboost-all-dev \
    libssl-dev \
    libyaml-cpp-dev \
    libeigen3-dev \
    libopencv-dev \
    libcurl4-openssl-dev \
    libbullet-dev \
    libsdl2-dev \
    libtinyxml2-dev \
    libgtest-dev \
    libpoco-dev \
    libflann-dev \
    libqhull-dev \
    libpcap-dev \
    libusb-1.0-0-dev \
    libgps-dev \
    libdw-dev

echo "安装ROS包依赖..."
sudo apt-get install -y \
    ros-foxy-behaviortree-cpp-v3 \
    ros-foxy-test-msgs \
    ros-foxy-graphicsmagick-cpp \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport \
    ros-foxy-vision-opencv \
    ros-foxy-laser-geometry \
    ros-foxy-tf2-sensor-msgs \
    ros-foxy-tf2-geometry-msgs \
    ros-foxy-pcl-ros \
    ros-foxy-octomap \
    ros-foxy-octomap-msgs \
    ros-foxy-rclcpp \
    ros-foxy-rclcpp-action \
    ros-foxy-geometry-msgs \
    ros-foxy-tf2 \
    ros-foxy-tf2-ros \
    ros-foxy-nav-msgs \
    ros-foxy-sensor-msgs \
    ros-foxy-visualization-msgs


# 创建 workspace
mkdir -p /robofuzz/targets/nav2
cd /robofuzz/targets/nav2

# 克隆 nav2 源码
git clone https://github.com/ros-planning/navigation2.git -b foxy-devel

# 安装依赖
# cd ..
rosdep install -y --from-paths src --ignore-src --rosdistro foxy

# 编译
colcon build \
    --symlink-install \
    --packages-up-to nav2_amcl nav2_bringup
    # --cmake-args \
    #     -DBUILD_TESTING=OFF
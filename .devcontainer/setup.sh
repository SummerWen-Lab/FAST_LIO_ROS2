#!/bin/bash

# 设置工作目录
cd /workspace

# 克隆 FAST_LIO 仓库
if [ ! -d "src/FAST_LIO" ]; then
    git clone --recursive https://github.com/Ericsiii/FAST_LIO.git src/FAST_LIO
fi

# 安装依赖
sudo rosdep install --from-paths src --ignore-src -y

# 构建工作区
source /opt/ros/humble/setup.sh
colcon build --symlink-install
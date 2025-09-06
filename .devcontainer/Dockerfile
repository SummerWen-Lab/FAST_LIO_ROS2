# 使用官方 ROS2 Humble 镜像
# FROM osrf/ros:humble-desktop-full\
FROM summerwen612/fast_lio_wxm:250906-stable

# 安装基本工具和依赖
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    libpcl-dev \
    libeigen3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 安装 VSCode 扩展依赖
RUN apt-get update && apt-get install -y \
    gdb \
    openssh-server \
    && rm -rf /var/lib/apt/lists/*

# 创建非 root 用户 (VSCode 要求)
ARG USERNAME=SummerWen
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 设置工作目录
WORKDIR /workspace
RUN chown $USER_UID:$USER_GID /workspace

# 切换到非 root 用户
USER $USERNAME

# 初始化 ROS 环境
RUN sudo rosdep init || true && rosdep update
# 使用 Ubuntu 20.04 作為基礎映像
FROM ubuntu:20.04

# 設置非交互模式，避免在安裝過程中要求輸入
ENV DEBIAN_FRONTEND=noninteractive

# 設置 ROS 環境變數
ENV ROS_PACKAGE_PATH=/opt/ros/noetic/share

# 設置環境變量
ENV CERES_VERSION="1.12.0"
ENV CATKIN_WS=/root/catkin_ws

# 安裝基本的系統依賴並安裝 curl
RUN apt-get update && apt-get install -y curl

# 安裝必要的系統依賴
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    build-essential \
    cmake \
    git \
    libopencv-dev \
    libsuitesparse-dev \
    libglew-dev \
    libtiff5-dev \
    qt5-qmake \
    qt5-default \
    qtbase5-dev \
    python3 \
    python3-pip \
    wget 

# 添加 ROS 軟體源和 GPG 密鑰
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-get update

# 安裝 ROS Noetic
RUN apt-get install -y ros-noetic-desktop-full

# 設置 ROS 環境變數
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
ENV ROS_PACKAGE_PATH=/opt/VINS-Mono:${ROS_PACKAGE_PATH}

# 安裝 rosdep 並初始化
RUN apt-get install -y python3-rosdep && \
    rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update

# 設置線程數以加快構建速度
RUN if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
    else export USE_PROC=$(($(nproc)/2)) ; fi && \
    apt-get update && apt-get install -y \
    libatlas-base-dev \
    libeigen3-dev \
    libgoogle-glog-dev \
    python3-catkin-tools \
    python3-opencv \
    x11-apps \  
    # ros-${ROS_DISTRO}-rviz \
    # ros-${ROS_DISTRO}-cv-bridge \
    # ros-${ROS_DISTRO}-image-transport \
    # ros-${ROS_DISTRO}-message-filters \
    # ros-${ROS_DISTRO}-tf && \
    rm -rf /var/lib/apt/lists/* && \
    # 構建和安裝Ceres
    git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    git checkout tags/${CERES_VERSION} && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(USE_PROC) install && \
    rm -rf ../../ceres-solver && \
    mkdir -p $CATKIN_WS/src/VINS-Mono/

# 複製VINS-Mono
COPY ./ $CATKIN_WS/src/VINS-Mono/
# 如果只有這個Dockerfile，使用以下行
# RUN git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git

# 構建VINS-Mono
WORKDIR $CATKIN_WS
ENV TERM=xterm
ENV PYTHONIOENCODING=UTF-8
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release && \
    catkin build && \
    sed -i '/exec "$@"/i \
            source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
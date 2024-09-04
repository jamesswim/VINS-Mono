# 使用ROS Noetic Desktop Full作為基礎映像
FROM osrf/ros:noetic-desktop-full

# 設置環境變量
ENV CERES_VERSION="1.12.0"
ENV CATKIN_WS=/root/catkin_ws

# 設置線程數以加快構建速度
RUN if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
    else export USE_PROC=$(($(nproc)/2)) ; fi && \
    apt-get update && apt-get install -y \
    git \ 
    cmake \
    libatlas-base-dev \
    libeigen3-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    python3-catkin-tools \
    python3-opencv \
    x11-apps \  
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-tf && \
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


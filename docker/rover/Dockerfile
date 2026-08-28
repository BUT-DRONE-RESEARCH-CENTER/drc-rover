#FROM dustynv/ros:humble-desktop-l4t-r36.4.0
#FROM osrf/ros:humble-desktop
FROM osrf/ros:humble-desktop-jammy
#FROM dustynv/ros:humble-ros-base
#FROM ubuntu:22.04
#FROM --platform=linux/arm64 osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# # Odstranění původního listu s expirovaným klíčem, aby prošel úvodní update
# RUN rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros1-latest.list && \
#     apt-get update || true && \
#     apt-get install -y curl && \
#     curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
#     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list

# Instalace systémových nástrojů, MAVROS a SLAM (BEZ balíčků závislých na OpenCV)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    tmux \
    nano \
    micro \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    python3-colcon-common-extensions \
    v4l-utils \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-slam-toolbox \
    geographiclib-tools \
    && /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*


# RUN mkdir /workspace && mkdir /workspace/src && cd /workspace/src && git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git 

# Nastavení prostředí a aliasů
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc && \
    echo "alias source_install='source /workspace/install/setup.bash'" >> /root/.bashrc && \
    echo "alias tkill='tmux kill-session'" >> /root/.bashrc

WORKDIR /workspace

RUN mkdir -p src && \
    git clone -b ros2 \
    https://github.com/Slamtec/rplidar_ros.git \
    src/rplidar_ros && \
    git clone \
    https://github.com/MAPIRlab/rf2o_laser_odometry.git \
    src/rf2o_laser_odometry && \
    git clone \
    https://github.com/ros-perception/vision_opencv \
    src/opencv_vision
    

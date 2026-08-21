FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Instalace systémových nástrojů, MAVROS a OpenCV pro C++
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    tmux \
    nano \
    python3-colcon-common-extensions \
    libopencv-dev \
    v4l-utils \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-slam-toolbox \
    geographiclib-tools \
    ros-humble-v4l2-camera \
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
    src/rf2o_laser_odometry
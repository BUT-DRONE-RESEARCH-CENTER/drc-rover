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
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    geographiclib-tools \
    && /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*

# Nastavení prostředí a aliasů
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc && \
    echo "alias source_install='source /workspace/install/setup.bash'" >> /root/.bashrc && \
    echo "alias tkill='tmux kill-session'" >> /root/.bashrc

WORKDIR /workspace
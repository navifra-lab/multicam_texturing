# ===== Base: Ubuntu 20.04 + ROS Noetic =====
FROM osrf/ros:noetic-desktop-full

# Non-interactive apt
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# 기본 유틸 + 개발 툴 (여기서는 gtsam 제외)
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo curl wget git vim nano \
    build-essential cmake pkg-config \
    software-properties-common \
    python3-pip python3-venv python3-empy \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-vcstool \
    libboost-all-dev libeigen3-dev libtbb-dev \
 && rm -rf /var/lib/apt/lists/*

# --- GTSAM 4.0.3 PPA 추가 및 설치 ---
# PPA: borglab/gtsam-release-4.0.3  (focal용 4.0.3 빌드 제공)
RUN apt-get update && \
    add-apt-repository -y ppa:borglab/gtsam-release-4.0 && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        libgtsam-dev libgtsam-unstable-dev && \
    # (선택) 이후 업그레이드로 버전이 바뀌지 않도록 고정
    apt-mark hold libgtsam-dev libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*

# rosdep 초기화 (컨테이너 안)
RUN rosdep init || true && rosdep update

# (옵션) catkin tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-catkin python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*
    
# 편의 alias
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc


# 이후 레이어는 쉘을 일반 형태로 전환해도 OK
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws


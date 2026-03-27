# =============================== 
# Dockerfile for CASPER_MAS Experiments
# CUDA + ROS 2 Humble + Gazebo + Python + CASPER Adapters
# FR3 (simulation + physical via Franky) + Tiago (simulation only)
# No MoveIt — uses ikpy for lightweight IK in simulation
# ===============================

# Base image with CUDA + cuDNN
FROM nvidia/cuda:13.0.1-cudnn-devel-ubuntu22.04

# ------------------------------
# Environment for noninteractive installs and GPU
# ------------------------------
ENV DEBIAN_FRONTEND=noninteractive 
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV ROS_DISTRO=humble
SHELL ["/bin/bash", "-c"]

# ------------------------------
# System dependencies: Python + OpenGL/EGL + general tools
# ------------------------------
# Step 1: install software-properties-common first
RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common

# Step 2: now universe repository is available and git can be installed
RUN add-apt-repository universe && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        python3 \
        python3-venv \
        python3-pip \
        python3-dev \
        python3-full \
        graphviz \
        graphviz-dev \
        libgraphviz-dev \
        pkg-config \
        locales \
        build-essential \
        libgl1 \
        libgl1-mesa-dri \
        libegl1 \
        mesa-utils \
        libx11-6 \
        libxext6 \
        libxrender1 \
        libsm6 \
        libglib2.0-0 \
        xvfb \
        ffmpeg \
        curl \
        gnupg2 \
        lsb-release \
        git \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------
# Set locales
# ------------------------------
RUN locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

# ------------------------------
# Graphviz headers for pip
# ------------------------------
ENV CFLAGS="-I/usr/include/graphviz"
ENV LDFLAGS="-L/usr/lib/x86_64-linux-gnu"

# ------------------------------
# ROS 2 Humble + Gazebo
# ------------------------------
# Add ROS apt repo
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME})_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb 

# Install ROS + Gazebo packages
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-ros-gz \
        ros-dev-tools \
        ros-${ROS_DISTRO}-ros-ign-bridge \
        ros-${ROS_DISTRO}-xacro \
        # Ignition Fortress specific (replaces gazebo-ros-pkgs for Ignition)
        ros-${ROS_DISTRO}-ros-gz-sim \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-gz-ros2-control \
        ros-${ROS_DISTRO}-ign-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Set Gazebo Version — must be ENV not export (export doesn't persist between layers)
ENV GZ_VERSION=fortress
ENV IGN_VERSION=fortress

# ------------------------------
# Source ROS automatically
# ------------------------------
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# ------------------------------
# ros2_control + controllers
# JointTrajectoryController — arm motion
# GripperActionController   — gripper open/close
# No MoveIt needed.
# ------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-joint-trajectory-controller \
        ros-${ROS_DISTRO}-joint-state-broadcaster \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-gripper-controllers \
        ros-${ROS_DISTRO}-control-msgs \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------
# Navigation2 — for Tiago mobile base transport
# ------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-nav2-simple-commander \
        ros-${ROS_DISTRO}-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*
 
# ------------------------------
# Rendering / headless configuration
# ------------------------------
ENV PYOPENGL_PLATFORM=egl
ENV DISPLAY=:0
ENV SDL_AUDIODRIVER=dummy
ENV ALSA_CARD=none

# ------------------------------
# ROS workspace
# ------------------------------ 
WORKDIR /ros2_ws

RUN mkdir src

# Copy ROS pacakages
COPY ros2_packages/ ./src

# ------------------------------
# Clone franka_description on humble branch only if it doesn't exist. 
# It clones the specific version used to develop the simulation, which is compatible with Gazebo Fortress. 
# This avoids potential issues with newer versions of franka_description that may not be compatible with the current setup. 
# If want the latest version, simply delete "&& cd franka_description && git checkout 2c4610f4df7e736b44882483598856819cd6b6f6"
# If the directory already exists, it skips cloning to save time and bandwidth. 
# ------------------------------
RUN cd src && \
    if [ ! -d franka_description ]; then \
        git clone --branch humble --single-branch https://github.com/frankarobotics/franka_description.git && cd franka_description && git checkout 2c4610f4df7e736b44882483598856819cd6b6f6; \
    else \
        echo "franka_description already exists, skipping clone"; \
    fi
# ------------------------------ 
# Tiago source clone
# ------------------------------
RUN cd src && \
    if [ ! -d tiago_robot ]; then \
        git clone --branch humble-devel --single-branch \
            https://github.com/pal-robotics/tiago_robot.git; \
    fi && \
    if [ ! -d tiago_simulation ]; then \
        git clone --branch humble-devel --single-branch \
            https://github.com/pal-robotics/tiago_simulation.git; \
    fi

RUN cd src && \
    if [ ! -d pmb2_robot ]; then \
        git clone --branch humble-devel --single-branch \
            https://github.com/pal-robotics/pmb2_robot.git; \
    fi && \
    if [ ! -d pmb2_simulation ]; then \
        git clone --branch humble-devel --single-branch \
            https://github.com/pal-robotics/pmb2_simulation.git; \
    fi

RUN apt-get update && rosdep install -i --from-path src --ignore-src --rosdistro $ROS_DISTRO -y --skip-keys="diagnostic_aggregator urdf_test"

# ------------------------------
# Python dependencies 
# ------------------------------
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt && rm requirements.txt

# ------------------------------
# Build workspace
# ------------------------------
RUN . /opt/ros/$ROS_DISTRO/setup.sh && rm -rf build install log && colcon build --symlink-install 
#--cmake-args -DBUILD_TESTING=OFF

# ------------------------------
# Workspace
# ------------------------------
WORKDIR /app

# ------------------------------
# Always source ROS, workspace
# ------------------------------
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc 

# ------------------------------
# Default command
# ------------------------------
CMD ["/bin/bash"]
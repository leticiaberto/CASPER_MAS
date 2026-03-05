# ===============================
# Dockerfile for CASPER_MAS Experiments
# CUDA + ROS 2 Humble + Gazebo + MoveIt 2 + Python + CASPER Adapters
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
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Set Gazebo Version
RUN export GZ_VERSION=fortress

# ------------------------------
# Source ROS automatically
# ------------------------------
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" 

# ------------------------------
# MoveIt 2 (ROS 2 Humble)
# ------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-moveit-resources \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------
# Rendering / headless configuration
# ------------------------------
ENV PYOPENGL_PLATFORM=egl
ENV DISPLAY=:0
ENV SDL_AUDIODRIVER=dummy
ENV ALSA_CARD=none

# ------------------------------
# ROS workspace for CASPER adapters
# ------------------------------
WORKDIR /ros2_ws

# Copy ROS adapters package 
COPY ros2_ws/src ./src

#COPY ros_adapters ./src/ros_adapters
RUN rosdep install -i --from-path src --ignore-src --rosdistro $ROS_DISTRO -y

# ------------------------------
# Python dependencies for adapters 
# ------------------------------
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt && rm requirements.txt

# ------------------------------
# Build workspace
# ------------------------------
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install

# ------------------------------
# Workspace
# ------------------------------
WORKDIR /app

# ------------------------------
# Always source ROS, workspace, and venv in all shells
# ------------------------------
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc 
 #  &&\ echo "source /venv/bin/activate" >> /etc/bash.bashrc

# ------------------------------
# Default command
# ------------------------------
CMD ["/bin/bash"]
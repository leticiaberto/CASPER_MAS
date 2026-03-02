# ===============================
# Dockerfile for Experiments
# CUDA + ROS 2 Jazzy + Gazebo (GUI) + Python + ROS Adapters
# ===============================

# Base image with CUDA + cuDNN
FROM nvidia/cuda:13.0.1-cudnn-devel-ubuntu24.04

# ------------------------------
# Environment for noninteractive installs and GPU
# ------------------------------
ENV DEBIAN_FRONTEND=noninteractive 
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV ROS_DISTRO=jazzy
SHELL ["/bin/bash", "-c"]

# ------------------------------
# System dependencies: Python + OpenGL/EGL + general tools
# ------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 \
        python3-venv \
        python3-pip \
        python3-dev \
        graphviz \
        graphviz-dev \
        libgraphviz-dev \
        pkg-config \
        python3-full \
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
        software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------
# Set locales
# ------------------------------
RUN locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

# ------------------------------
# Tell pip where to find Graphviz headers
# ------------------------------
ENV CFLAGS="-I/usr/include/graphviz"
ENV LDFLAGS="-L/usr/lib/x86_64-linux-gnu"

# ------------------------------
# Python virtual environment
# ------------------------------
RUN python3 -m venv /venv
ENV VIRTUAL_ENV=/venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# ------------------------------
# ROS 2 Jazzy + Gazebo (GUI) installation
# ------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && sh -c 'echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list' \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \    
    ros-jazzy-ros-gz \          
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------
# Source ROS automatically
# ------------------------------
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# ------------------------------
# Rendering configuration
# ------------------------------
ENV PYOPENGL_PLATFORM=egl
ENV DISPLAY=:0
# Audio dummy for headless compatibility
ENV SDL_AUDIODRIVER=dummy 
ENV ALSA_CARD=none

# ------------------------------
# ROS workspace
# ------------------------------
WORKDIR /ros2_ws
RUN mkdir -p src

# ------------------------------
# Copy ROS adapters package 
# ------------------------------
COPY ros_adapters ./src/ros_adapters

# ------------------------------
# Build workspace
# ------------------------------
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build

# ------------------------------
# Workspace
# ------------------------------
WORKDIR /app 

# Copy Python requirements and install
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt && rm requirements.txt

# ------------------------------
# Always source ROS and workspace for all shells
# ------------------------------
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc
RUN echo "source /venv/bin/activate" >> /etc/bash.bashrc

# Default command
CMD ["/bin/bash"]

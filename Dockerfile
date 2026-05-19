# ===============================
# Dockerfile for CASPER_MAS Experiments
# CUDA + ROS 2 Humble + Gazebo Fortress + Python + CASPER Adapters
# FR3 (simulation + physical via Franky) + Tiago (simulation only)
# No MoveIt — uses ikpy for lightweight IK in simulation
# ===============================

FROM nvidia/cuda:13.0.1-cudnn-devel-ubuntu22.04

# ------------------------------
# Environment for noninteractive installs and GPU
# ------------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
# Must include 'display' and 'graphics' explicitly — 'all' alone sometimes
# omits them, which prevents Gazebo from accessing the GPU renderer.
ENV NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility,display
ENV ROS_DISTRO=humble
SHELL ["/bin/bash", "-c"]

# ------------------------------
# Step 1: software-properties-common first
# ------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common

# ------------------------------
# Step 2: system dependencies
# ------------------------------
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
        ufw \
        # ── GPU / EGL rendering stack ──────────────────────────────────
        # Required for Ignition Fortress to use the NVIDIA GPU.
        # Without these, Gazebo falls back to llvmpipe (CPU software
        # renderer) which runs at ~1/70 real time.
        libgl1 \
        libgl1-mesa-dri \
        libgl1-mesa-glx \
        libegl1 \
        libegl1-mesa \
        libegl-mesa0 \
        libgles2 \
        libglvnd0 \
        libglvnd-dev \
        libglx0 \
        libglx-mesa0 \
        mesa-utils \
        mesa-utils-extra \
        libvulkan1 \
        vulkan-tools \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------
# Locales
# ------------------------------
RUN locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

# ------------------------------
# Graphviz headers for pip
# ------------------------------
ENV CFLAGS="-I/usr/include/graphviz"
ENV LDFLAGS="-L/usr/lib/x86_64-linux-gnu"

# ------------------------------
# OSRF apt source — must come before ROS apt source so OSRF packages
# take priority over Ubuntu universe ignition variants. Both ros-humble-
# ros-gz and the Fortress dev headers come from this repo.
# ------------------------------
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list

# ------------------------------
# ROS 2 Humble apt source
# ------------------------------
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME})_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb

# Install ROS + Gazebo runtime + Fortress dev headers in one layer.
# With the OSRF repo present first, all ignition packages resolve from
# the same source and there are no Ubuntu universe conflicts.
# Dev package names use the gz- prefix (OSRF renamed ignition- to gz-
# for Fortress onward on jammy): libignition-gazebo6-dev, libignition-transport11-dev,
# libignition-msgs8-dev, libignition-math6-dev.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-ros-gz \
        ros-dev-tools \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-ros-gz-sim \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-gz-ros2-control \
        ros-${ROS_DISTRO}-ign-ros2-control \
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
        libignition-gazebo6-dev \
        libignition-transport11-dev \
        libignition-msgs8-dev \
        libignition-math6-dev \
    && rm -rf /var/lib/apt/lists/*

# rosdep init is not idempotent — guard against repeated calls in cached layers
RUN rosdep init 2>/dev/null || true && rosdep update

ENV GZ_VERSION=fortress
ENV IGN_VERSION=fortress

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# ------------------------------
# ros2_control + controllers
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
# GPU / EGL rendering environment for Ignition Fortress
# ------------------------------
# Do NOT set PYOPENGL_PLATFORM=egl — it interferes with Gazebo's own EGL
# initialisation. Gazebo handles EGL device selection internally.
#
# DISPLAY is intentionally omitted: EGL_PLATFORM=device makes Gazebo use
# NVIDIA's EGL device path directly, bypassing X11 entirely. Setting
# DISPLAY=:0 with no running X server can cause Gazebo to attempt an X11
# fallback and fail. Start Xvfb in your entrypoint/run script only if a
# specific tool in your stack requires an X display.
ENV SDL_AUDIODRIVER=dummy
ENV ALSA_CARD=none

# Force EGL to use the NVIDIA device specifically.
# - EGL_PLATFORM=device   : use EGL device enumeration (not DRI2/X11)
# - __EGL_VENDOR_LIBRARY_FILENAMES : point directly to NVIDIA's EGL ICD,
#   bypassing Mesa/Intel. This works without __NV_PRIME_RENDER_OFFLOAD
#   so the monitor (driven by iGPU) is not affected.
# GLX will still show Intel (monitor is on iGPU) — that is correct.
# Gazebo uses EGL, not GLX, so glxinfo showing Intel is irrelevant.
ENV EGL_PLATFORM=device
ENV __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

# ------------------------------
# ROS workspace
# ------------------------------
WORKDIR /ros2_ws
RUN mkdir src

COPY ros2_packages/ ./src

# ------------------------------
# franka_description (pinned commit, Gazebo Fortress compatible)
# ------------------------------
RUN cd src && \
    if [ ! -d franka_description ]; then \
        git clone --branch humble --single-branch https://github.com/frankarobotics/franka_description.git \
        && cd franka_description \
        && git checkout 2c4610f4df7e736b44882483598856819cd6b6f6; \
    else \
        echo "franka_description already exists, skipping clone"; \
    fi

# ------------------------------
# Tiago source clone
# ------------------------------
RUN cd src && \
    UPDATED=0 && \
    if [ ! -d tiago_robot ]; then \
        git clone --branch humble-devel --single-branch \
            https://github.com/pal-robotics/tiago_robot.git && \
        cd tiago_robot && \
        git checkout 8384f3adb21ee7b80ee665b0091c4c8eebe24f94 && \
        cd .. && \
        UPDATED=1; \
    fi && \
    if [ ! -d pal_gripper ]; then \
        git clone --branch humble-devel --single-branch \
            https://github.com/pal-robotics/pal_gripper.git && \
        cd pal_gripper && \
        git checkout 0e41b4f3d7e15c16711846e510dcf2071275d759 && \
        cd .. && \
        UPDATED=1; \
    fi && \
    if [ ! -d pal_urdf_utils ]; then \
        git clone --branch humble-devel --single-branch \
            https://github.com/pal-robotics/pal_urdf_utils.git && \
        cd pal_urdf_utils && \
        git checkout 4320a981a75c2eef17b5615aadfde8c1c949e504 && \
        cd .. && \
        UPDATED=1; \
    fi && \
    if [ ! -d pmb2_robot ]; then \
        git clone --branch humble-devel --single-branch \
            https://github.com/pal-robotics/pmb2_robot.git && \
        cd pmb2_robot && \
        git checkout e7b8f1a0d1b88650364b56103f1f71dc80f9e9b4 && \
        cd .. && \
        UPDATED=1; \
    fi && \
    if [ "$UPDATED" = "1" ]; then \
        apt-get update; \
    fi && \
    apt-get update && rosdep install -i --from-path . --ignore-src \
        --rosdistro $ROS_DISTRO -y \
        --skip-keys="diagnostic_aggregator urdf_test pal_gazebo_worlds"

# ------------------------------
# Python dependencies
# ------------------------------ 
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt && rm requirements.txt

RUN pip3 install --no-cache-dir \
        ikpy \
        scipy \
        numpy \
        colcon-common-extensions

# ------------------------------
# Build workspace
# ------------------------------
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rm -rf build install log && \
    colcon build --symlink-install --event-handlers console_direct+

# ------------------------------
# Workspace
# ------------------------------
WORKDIR /app

# ------------------------------
# Always source ROS and workspace
# ------------------------------
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc

# ------------------------------
# Default command
# ------------------------------
CMD ["/bin/bash"]

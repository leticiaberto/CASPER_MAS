FROM nvidia/cuda:13.0.1-cudnn-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# System dependencies: Python + OpenGL/EGL
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3 \
        python3-venv \
        python3-pip \
        graphviz \
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
    && rm -rf /var/lib/apt/lists/*

# Set locales
RUN locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

# Create Python virtual environment
RUN python3 -m venv /venv
ENV VIRTUAL_ENV=/venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Install Python packages from requirements.txt
COPY requirements.txt .
RUN pip3 install -r requirements.txt && rm requirements.txt

# Rendering configuration
ENV PYOPENGL_PLATFORM=egl
ENV DISPLAY=:0

ENV SDL_AUDIODRIVER=dummy 
ENV ALSA_CARD=none

WORKDIR /app 

CMD ["/bin/bash"]

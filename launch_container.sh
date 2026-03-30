#!/bin/bash

# ===========================
# Launch CASPER_MAS container with NVIDIA GPU
# ===========================

IMAGE_NAME="casper_mas"
DEFAULT_CONTAINER_NAME="casper_mas-container"

CONTAINER_NAME="${1:-$DEFAULT_CONTAINER_NAME}"

DOCKERFILE_PATH="Dockerfile"
REQUIREMENTS_PATH="requirements.txt"
CHECKSUM_FILE=".dockerfile_checksum"

# ---------------------------
# Allow Docker to access X server
# ---------------------------
xhost +SI:localuser:$(whoami)

# ---------------------------
# Setup Xauthority for container access
# ---------------------------
XAUTH_FILE=/tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH_FILE nmerge -
chmod 777 $XAUTH_FILE

# ---------------------------
# Rebuild image if Dockerfile / requirements / ros2_packages changed
# ---------------------------
ROS2_DIR="ros2_packages"

CURRENT_HASH=$(
{
    sha256sum "$DOCKERFILE_PATH"
    sha256sum "$REQUIREMENTS_PATH"
    find "$ROS2_DIR" -type f ! -path "*/build/*" ! -path "*/install/*" ! -path "*/log/*"
} | sha256sum | awk '{print $1}'
)

OLD_HASH=""
if [[ -f $CHECKSUM_FILE ]]; then
    OLD_HASH=$(cat $CHECKSUM_FILE)
fi

if [[ "$CURRENT_HASH" != "$OLD_HASH" ]] || [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
    echo "🔨 Changes detected or image missing. Building $IMAGE_NAME..."
    docker build -t $IMAGE_NAME -f $DOCKERFILE_PATH .
    echo "$CURRENT_HASH" > $CHECKSUM_FILE
else
    echo "✅ No relevant changes detected. Skipping build."
fi

# ---------------------------
# Stop and remove any existing container
# ---------------------------
if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
  echo "🛑 Stopping existing $CONTAINER_NAME..."
  docker stop $CONTAINER_NAME >/dev/null 2>&1 || true
  docker rm $CONTAINER_NAME >/dev/null 2>&1 || true
fi

# ---------------------------
# GL performance
# ---------------------------
export LIBGL_ALWAYS_INDIRECT=0

# ---------------------------
# Launch container
# ---------------------------
docker run -it \
    --gpus all \
    --device=/dev/dri \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility,display \
    -e DISPLAY=$DISPLAY \
    -e GAZEBO_DISPLAY=$DISPLAY \
    -e IGN_RENDERING_ENGINE=ogre2 \
    -e LIBGL_ALWAYS_SOFTWARE=0 \
    -e XAUTHORITY=$XAUTH_FILE \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_INDIRECT=0 \
    -e EGL_PLATFORM=device \
    -e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $XAUTH_FILE:$XAUTH_FILE \
    --ipc=host \
    --mount type=bind,source=/home/e10738lb/Development/CASPER_MAS,target=/app/CASPER_MAS,readonly=false,bind-propagation=rslave \
    -v $(pwd)/ros2_packages:/ros2_ws/src \
    -w /app/CASPER_MAS \
    --name $CONTAINER_NAME \
    $IMAGE_NAME

docker stop $CONTAINER_NAME

# ---------------------------
# Cleanup on exit
# ---------------------------
xhost -SI:localuser:$(whoami)
rm -f $XAUTH_FILE
#!/bin/bash

# ===========================
# Launch ADIRL container with NVIDIA GPU Offload
# ===========================

IMAGE_NAME="casper_mas"
DEFAULT_CONTAINER_NAME="casper_mas-container"

# If a command-line argument is provided, use it as container name
CONTAINER_NAME="${1:-$DEFAULT_CONTAINER_NAME}"

DOCKERFILE_PATH="Dockerfile"
REQUIREMENTS_PATH="requirements.txt"
CHECKSUM_FILE=".dockerfile_checksum"

# ---------------------------
# Allow Docker (root) to access X server
# ---------------------------
xhost +SI:localuser:$(whoami)

# ---------------------------
# Setup Xauthority for container access
# ---------------------------
XAUTH_FILE=/tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH_FILE nmerge -
chmod 777 $XAUTH_FILE

# ---------------------------
# Check Dockerfile / requirements / ros2_packages changes and rebuild image if needed
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
    echo "🔨 Dockerfile, requirements.txt, or ros2_packages changed, or image missing. Building $IMAGE_NAME..."
    docker build -t $IMAGE_NAME -f $DOCKERFILE_PATH .
    echo "$CURRENT_HASH" > $CHECKSUM_FILE
else
    echo "✅ No relevant changes detected. Skipping build."
fi

# ---------------------------
# Stop and remove any existing container (if still running)
# ---------------------------
if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
  echo "🛑 Stopping existing $CONTAINER_NAME..."
  docker stop $CONTAINER_NAME >/dev/null 2>&1 || true
  docker rm $CONTAINER_NAME >/dev/null 2>&1 || true
fi

# ---------------------------
# Enable NVIDIA GPU offload (in Docker or native)
# Removed because it was freezing the screen and causing performance issues. It may be worth revisiting in the future if we can find a way to mitigate those issues.
# ---------------------------
#export __NV_PRIME_RENDER_OFFLOAD=1
#export __GLX_VENDOR_LIBRARY_NAME=nvidia
#export __VK_LAYER_NV_optimus=NVIDIA_only
#export __NV_PRIME_RENDER_OFFLOAD_PROVIDER=NVIDIA-G0

# ---------------------------
# Optional for better GL performance
# ---------------------------
export LIBGL_ALWAYS_INDIRECT=0

# ---------------------------
# Launch container:
# ---------------------------
docker run -it \
    --gpus all \
    --device=/dev/dri \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTH_FILE \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e QT_X11_NO_MITSHM=1 \
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
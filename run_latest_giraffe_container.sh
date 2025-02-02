#!/bin/bash

# Define container name and base image name
CONTAINER_NAME="giraffe_container"
IMAGE_NAME="giraffe_ros2_backup"

# Find the latest committed image
LATEST_IMAGE=$(sudo docker images --format "{{.Repository}}:{{.Tag}}" | grep "$IMAGE_NAME" | sort -r | head -n 1)

# Check if an image exists
if [ -z "$LATEST_IMAGE" ]; then
    echo "❌ No committed images found for $IMAGE_NAME. Please run commit_giraffe_container.sh first."
    exit 1
fi

# Check if the container already exists and remove it
if sudo docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "🛑 Stopping and removing existing container: $CONTAINER_NAME..."
    sudo docker stop $CONTAINER_NAME >/dev/null 2>&1
    sudo docker rm $CONTAINER_NAME >/dev/null 2>&1
fi

echo "🚀 Running the latest committed image: $LATEST_IMAGE"

# Run the latest container in the background with restart policy
sudo docker run -it -d --restart unless-stopped \
    --runtime=nvidia \
    --network host \
    --privileged \
    --ipc=host \
    --env DISPLAY=$DISPLAY \
    --env ROS_DOMAIN_ID=30 \
    --env FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp.xml \
    --device /dev/video0 \
    --device /dev/video1 \
    --device /dev/ttyUSB0 \
    --volume /mnt/nvme/app:/app \
    --volume $HOME/.ssh:/root/.ssh:ro \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume /dev/bus/usb:/dev/bus/usb \
    --volume /etc/localtime:/etc/localtime:ro \
    --volume /home/$USER:/home/$USER \
    --name $CONTAINER_NAME \
    $LATEST_IMAGE

echo "✅ Giraffe container is now running in the background."
echo "🔗 Run ./attach_giraffe_container.sh to attach to it."
echo "🔄 This container will automatically restart after a reboot unless manually stopped."

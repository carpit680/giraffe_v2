#!/bin/bash

# Define container name and image name
CONTAINER_NAME="giraffe_container"
IMAGE_NAME="giraffe_ros2_backup"

# Get the current date and time for versioning
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

# Commit the container to a new image
echo "📦 Committing the running container '$CONTAINER_NAME' to image '$IMAGE_NAME:$TIMESTAMP'..."
sudo docker commit $CONTAINER_NAME $IMAGE_NAME:$TIMESTAMP

# List available images to verify
echo "✅ Docker images after commit:"
sudo docker images | grep "$IMAGE_NAME"

echo "🚀 Committed successfully! You can run the new image with:"
echo "    sudo docker run -it $IMAGE_NAME:$TIMESTAMP"

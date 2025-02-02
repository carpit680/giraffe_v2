#!/bin/bash

# Check if the container is running
if sudo docker ps | grep -q "giraffe_container"; then
    echo "🔗 Attaching to giraffe_container..."
    sudo docker exec -it giraffe_container zsh
else
    echo "❌ giraffe_container is not running. Start it first using ./start_giraffe_container.sh"
fi

#!/bin/bash

# Viro Cobot Kill Script (Cleanup)
# Stops and removes all project-related containers and images.

PROJECT_ROOT=$(pwd)
DOCKER_DIR="${PROJECT_ROOT}/docker"

echo "Stopping and removing viro_cobot_container..."
docker compose -f "${DOCKER_DIR}/docker-compose.yml" down --remove-orphans

# Clean up any lingering ROS bridge or Gazebo processes (esp. for host network mode)
echo "Cleaning up lingering ROS/Gazebo processes..."
pkill -9 -f rosbridge_websocket || true
pkill -9 -f parameter_bridge || true
pkill -9 -f cobot_gui_client || true
pkill -9 -f ign-gazebo-server || true
pkill -9 -f gz-sim-server || true

# Optional: Force-remove container if it's lingering
if [[ $(docker ps -a -f name=viro_cobot_container -q) ]]; then
    echo "Forcing removal of lingering container..."
    docker rm -f viro_cobot_container
fi

# Optional: Remove the image as well if user wants a full cleanup
if [[ "$1" == "--purge" ]]; then
    echo "Purging viro_cobot images..."
    docker rmi viro_cobot_env:latest || true
fi

echo "Cleanup complete."

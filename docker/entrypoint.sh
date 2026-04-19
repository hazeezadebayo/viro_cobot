#!/bin/bash
set -e

# Source ROS 2 Humble setup
source "/opt/ros/humble/setup.bash"

# Build workspace if setup.bash doesn't exist (first time)
if [ ! -f "/ros2_ws/install/setup.bash" ]; then
    echo "Installing dependencies and building the project for the first time..."
    cd /ros2_ws
    
    # rosdep update for the current user (if needed, but usually done in Dockerfile)
    rosdep update
    
    # dependencies installation
    rosdep install --from-paths . --ignore-src -r -y
    
    colcon build --symlink-install
fi

# Source local workspace
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source "/ros2_ws/install/setup.bash"
fi

# Finally execute whatever was passed
exec "$@"

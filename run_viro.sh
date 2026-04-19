#!/bin/bash

# Viro Cobot Run Script
PROJECT_ROOT=$(pwd)
DOCKER_DIR="${PROJECT_ROOT}/docker"

# Identity Detection for Portability
export USER_UID=$(id -u)
export USER_GID=$(id -g)
export USERNAME=$(whoami)

case "$1" in
    "build")
        docker compose -f "${DOCKER_DIR}/docker-compose.yml" build
        ;;
    "clean-build")
        docker compose -f "${DOCKER_DIR}/docker-compose.yml" build --no-cache
        ;;
    "up")
        # Ensure X11 access for Docker
        xhost +local:docker > /dev/null
        docker compose -f "${DOCKER_DIR}/docker-compose.yml" up -d
        ;;
    "launch")
        echo "Launching Viro Cobot Interface..."

        # Wait for container's workspace build to complete (entrypoint runs colcon on first start)
        echo "Waiting for workspace build to complete (max 5 mins)..."
        TIMEOUT=150 # 150 * 2s = 5mins
        COUNT=0
        while [ $COUNT -lt $TIMEOUT ]; do
            # Check if container is even alive
            STATUS=$(docker inspect -f '{{.State.Running}}' viro_cobot_container 2>/dev/null)
            if [ "$STATUS" != "true" ]; then
                echo -e "\nERROR: viro_cobot_container is not running. Check 'docker ps -a'."
                exit 1
            fi

            if docker exec viro_cobot_container test -f /ros2_ws/install/setup.bash 2>/dev/null; then
                if ! docker exec viro_cobot_container pgrep -x colcon > /dev/null 2>&1; then
                    break
                fi
            fi

            printf "."
            sleep 2
            COUNT=$((COUNT + 1))
        done

        if [ $COUNT -ge $TIMEOUT ]; then
            echo -e "\nERROR: Build synchronization timed out. Check container logs."
            exit 1
        fi
        echo -e "\n Build ready."

        # Clear logs from previous session so `tail -f` shows only current session
        > "${PROJECT_ROOT}/rosbridge.log"
        > "${PROJECT_ROOT}/gui_bridge.log"
        > "${PROJECT_ROOT}/http.log"

        # 1. Start ROS WebSocket Bridge
        echo "Starting ROS Bridge (Logs: rosbridge.log)..."
        docker exec -d viro_cobot_container bash -c \
            "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml >> /ros2_ws/src/viro_cobot/rosbridge.log 2>&1"

        # 2. Start Web App Server (must cd into the web dir first)
        echo "Starting HTTP Server (Logs: http.log)..."
        docker exec -d viro_cobot_container bash -c \
            "cd /ros2_ws/src/viro_cobot/cobot_gui/ros2_websocket && python3 serve_quiet.py 8080 >> /ros2_ws/src/viro_cobot/http.log 2>&1"

        # 3. Start GUI Command Bridge Node
        echo "Starting Command Bridge (Logs: gui_bridge.log)..."
        docker exec -d viro_cobot_container bash -c \
            "export PYTHONUNBUFFERED=1 && source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run cobot_api cobot_gui_client.py >> /ros2_ws/src/viro_cobot/gui_bridge.log 2>&1"

        echo "Interface launched!"
        echo "- Web GUI: http://localhost:8080"
        echo "- Monitor logs: tail -f rosbridge.log gui_bridge.log"
        echo "- Click 'Power On' in the web interface to start the Robot."
        ;;
    "shell")
        # Enter the container with a sourced ROS 2 environment
        docker exec -it viro_cobot_container bash
        ;;
    "stop")
        docker compose -f "${DOCKER_DIR}/docker-compose.yml" stop
        ;;
    "clean-workspace")
        echo "Cleaning ROS 2 workspace (build, install, log)..."
        docker exec -it viro_cobot_container rm -rf /ros2_ws/build /ros2_ws/install /ros2_ws/log
        echo "Workspace cleaned."
        ;;
    *)
        echo "Usage: ./run_viro.sh {build|clean-build|up|launch|shell|stop}"
        echo "  build:       Quick rebuild using Docker cache"
        echo "  clean-build: Force rebuild from scratch (fixes cache corruption)"
        echo "  up:          Start the containerized environment"
        echo "  launch:      Launch Web UI and Command Bridge"
        echo "  shell:       Enter the running container"
        echo "  stop:        Stop all running containers"
        ;;
esac

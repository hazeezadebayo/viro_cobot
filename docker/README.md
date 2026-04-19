# Docker Setup for Viro Cobot

This folder contains the Docker configuration for the `viro_cobot` project.

## Quick Start (Manual)

If you prefer to run `docker compose` manually instead of using the scripts in the project root:

1. **Build image**:
    ```bash
    docker compose build
    ```

2. **Run container**:
    ```bash
    docker compose up -d
    ```

3. **Open interactive shell**:
    ```bash
    docker exec -it viro_cobot_container bash
    ```

## Architecture

- **Base Image**: `ros:humble-desktop` (Supports GUIs like RViz and MoveIt Setup Assistant).
- **Network**: Host mode for ROS 2 discovery.
- **Display**: Automatically forwards X11 for GUI applications.
- **Hardware**: Enabled with `--privileged` and device mapping pointers for EtherCAT.

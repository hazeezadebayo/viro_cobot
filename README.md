# Viro Cobot Control System

This repository contains the simulation and control stack for the Viro robotic arm.

## Directory Structure
```text
viro_cobot/
├── cobot_api/               # Launch logic and GUI-ROS bridge
├── cobot_description/       # URDF, meshes, and hardware config
├── cobot_gui/               # Web interface (HTML/JS)
├── cobot_moveit_config/     # MoveIt 2 configuration (SRDF, kinematics)
├── cobot_gravity_compensation/ # [Advanced] RNE-based Free-drive logic
├── cobot_admittance_control/  # [Advanced] Force-based target generation
├── cobot_impedance_control/   # [Advanced] Jacobian compliance
├── docker/                  # Dockerfile and compose for isolated execution
├── run_viro.sh              # Unified entrypoint script
└── project_report.md        # Technical architecture and status
```

## Getting Started

### Prerequisites
- Docker and Docker Compose
- NVIDIA Container Toolkit (for GPU acceleration in Gazebo, optional)

### Build and Run
1.  **Build the environment**:
    ```bash
    ./run_viro.sh build
    ```
2.  **Start the container**:
    ```bash
    ./run_viro.sh up
    ```
3.  **Launch the System**:
    ```bash
    ./run_viro.sh launch
    ```
    This starts the `rosbridge`, `http.server`, and the command bridge.

### Operating the Robot
- Open your browser to `http://localhost:8080`.
- Ensure the status says **Connection: successful**.
- Click **Power On** to initialize the backend (Gazebo & MoveIt).
- Use the **Jogging** buttons to move individual joints or Cartesian coordinates.
- Monitor real-time feedback in the **Feedback** section.

## Verification
To verify the system is working, ensure that after clicking **Power On**, the joint values update from `N/A` to actual numeric values (e.g., `0.0000`).

## Operation Modes
The Viro Cobot now supports selectable operation modes via the Web GUI:

1. **Simulation (Gazebo)**: Full physics engine (Ignition Gazebo). Best for testing environment interactions and sensor data.
2. **Headless (Fake)**: High-speed, in-memory kinematics simulation using `ros2_control`'s fake components. No physics, zero GPU overhead.
3. **Real Robot (Ethercat)**: Direct control of hardware using the `EthercatDriver`. 
4. **Real Robot (SOEM)**: Infrastructure for SOEM-based control.

### Mode Selection Workflow
- Select the desired mode from the dropdown menu in the Web Interface.
- Click **Power On**.
- The backend bridge dynamically launches the appropriate ROS 2 nodes and hardware interfaces.

### SOEM Integration Requirements
To utilize the SOEM option:
- Install a compatible SOEM hardware interface (e.g., `soem_ros2_control`) in your workspace.
- The URDF is configured to look for `soem_hardware_interface/SoemHWInterface`.






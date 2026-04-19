# cobot_scan_n_plan (ROS 2)

![Project Status](https://img.shields.io/badge/Status-Migrated-success)
![Version](https://img.shields.io/badge/Version-1.0.0-blue)

A surgical, high-performance ROS 2 migration of the **Scan-and-Plan** workflow. This package enables a robot to scan a 3D surface, generate a geometric coverage path, and parameterize it into a time-scaled trajectory.

## 🛸 Architecture: Library + Node

This package follows a strict decoupling pattern for maximum reliability and ease of testing.

- **`scripts/`**: Contains the persistent ROS 2 Nodes (Orchestrators).
- **`cobot_scan_n_plan/`**: Contains the pure algorithmic libraries (Math & Geometric processing).

## 🚀 Key Components

### 1. Vision Driver (Optional / Generic)
- **Node**: Any node providing a `sensor_msgs/PointCloud2` stream.
- **Example**: `rs_point_cloud` (via `scripts/rs_point_cloud.py`) for RealSense hardware.
- **Topics**: `/kinect_camera/points` (Default).

### 2. Scanning Orchestrator (`surface_scan_node`)
- **Node**: `surface_scan_node`
- **Role**: Captures the transient vision stream into a persistent `.ply` file.
- **Services**: `/mcfly_scan_n_plan/scan` (`SetBool`) -> Start/Stop buffering and save.

### 3. Planning Orchestrator (`surface_plan_node`)
- **Node**: `surface_plan_node`
- **Role**: Handles geometric path generation and time-parameterization.
- **Services**: `/mcfly_scan_n_plan/plan` (`Trigger`) -> Reads PLY and generates CSV trajectory.

### 4. Trajectory Executor (`surface_traj_node`)
- **Node**: `surface_traj_node`
- **Role**: Bridges the gap between planning and motion.
- **Services**: `/mcfly_scan_n_plan/execute` (`Trigger`) -> Loads Joint CSV and publishes to JTC.

## 🛠️ Usage

### Workflow Execution
Start the nodes (they will remain active and wait for triggers):
```bash
ros2 run cobot_scan_n_plan surface_scan_node
ros2 run cobot_scan_n_plan surface_plan_node
ros2 run cobot_scan_n_plan surface_traj_node
```

**1. Scan**
```bash
ros2 service call /mcfly_scan_n_plan/scan std_srvs/srv/SetBool "{data: true}"
# Wait...
ros2 service call /mcfly_scan_n_plan/scan std_srvs/srv/SetBool "{data: false}"
```

**2. Plan**
```bash
ros2 service call /mcfly_scan_n_plan/plan std_srvs/srv/Trigger "{}"
```

**3. Execute**
```bash
ros2 service call /mcfly_scan_n_plan/execute std_srvs/srv/Trigger "{}"
```

## 📂 Structure
```
cobot_scan_n_plan/
├── scripts/                # ROS 2 Executable Nodes (Persistent)
│   ├── surface_plan_node.py
│   ├── surface_scan_node.py
│   ├── surface_traj_node.py
│   └── rs_point_cloud.py
├── cobot_scan_n_plan/      # Algorithmic Libraries (Pure Logic)
│   ├── surface_plan.py
│   ├── surface_scan.py
│   └── surface_traj.py
├── launch/                 # Python Launch System
├── config/                 # Assets (.ply, .csv, and parameters)
├── package.xml             # Metadata & Dependencies
└── setup.py                # Build Configuration
```

## 📦 Dependencies
- **Core**: `rclpy`, `sensor_msgs`, `std_srvs`, `cv_bridge`
- **Math/CV**: `numpy`, `open3d`, `opencv-python`, `pyrealsense2`, `scipy`, `scikit-learn`

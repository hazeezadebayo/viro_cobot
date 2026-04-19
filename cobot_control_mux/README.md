# Cobot Control Mux

The `cobot_control_mux` serves as the central coordination layer for the Viro Cobot's modular control architecture. It implements a **Unified Effort** strategy, allowing multiple control agents (Gravity, Admittance, Impedance) to contribute to the robot's hardware commands without collision.

## 📐 Philosophy: Active Transparency

Standard robotic controllers either provide Position OR Torque. This Mux provides **Both** simultaneously:
1.  **Direct Effort**: It sums the torques from all modular nodes and sends them to the `/effort_direct_controller`.
2.  **Adaptive Setpoint**: It continuously feeds the robot's current position ($q_{actual}$) back into the `/effort_joint_trajectory_controller`. This ensures that even when the motors are moving the arm due to gravity/assist forces, the internal PID never fights against the motion.

## 🛰️ Interface

### Subscribed Topics
| Topic | Type | Source |
| :--- | :--- | :--- |
| `/gravity/effort` | `std_msgs/Float64MultiArray` | Gravity Compensator |
| `/admittance/effort` | `std_msgs/Float64MultiArray` | Admittance/Assist Node |
| `/impedance/effort` | `std_msgs/Float64MultiArray` | Impedance/Compliance Node |
| `/joint_states` | `sensor_msgs/JointState` | Robot Feedback |

### Published Topics
| Topic | Type | Target |
| :--- | :--- | :--- |
| `/effort_direct_controller/commands` | `std_msgs/Float64MultiArray` | Raw Motor Torques |
| `/effort_joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | JTC Transparency Setpoint |

## 🚀 Launch

```bash
ros2 launch cobot_control_mux control_mux.launch.py
```

## 🛠️ Parameters
- `update_rate_hz`: Frequency of the summation loop (Default: `100.0`).
- `joint_names`: List of joints to control.

# Cobot Impedance Control

The `cobot_impedance_control` package provides **Virtual Stiffness and Damping** (Compliance) for the Viro Cobot.

## 📐 Philosophy: Safety & Stiffness (MVL)

Operating on the "Minimum Viable Logic" principle, this node acts as a **Virtual Spring**. It calculates the joint torques required to pull the robot back toward a specified Cartesian setpoint.

This node is the "elastic heart" of the control stack:
- It maintains a target pose (Setpoint).
- It generates torques proportional to the Cartesian error (Stiffness).
- It generates torques proportional to the Cartesian velocity (Damping).
- It ensures the robot feels stable and "returns to center" when external forces are removed.

## 🛰️ Interface

### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/cobot/setpoint_pose` | `geometry_msgs/PoseStamped` | The target Cartesian goal for the virtual spring. |
| `/joint_states` | `sensor_msgs/JointState` | Current robot configuration for error calculation. |

### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/impedance/effort` | `std_msgs/Float64MultiArray` | Stiffness torques sent to the Multiplexer. |

## 🛠️ Parameters
- `stiffness`: 6D vector $[X, Y, Z, Rx, Ry, Rz]$ defining the spring strength.
- `damping`: 6D vector for energy dissipation (to prevent oscillation).
- `setpoint_topic`: Reconfigurable topic for the target pose.

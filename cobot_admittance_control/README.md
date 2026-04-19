# Cobot Admittance Control

The `cobot_admittance_control` package provides **Force assistance and Lead-through** capabilities for the Viro Cobot. 

## 📐 Philosophy: Reactive Assistance (MVL)

Following the principle of "Minimum Viable Logic," this node acts as a pure **Force-to-Torque Mapper**. It interprets external wrenches (detected at the wrist) as a desire for motion and generates the corresponding assistance torques using the Jacobian Transpose.

This node is strictly reactive:
- It does **not** simulate mass-damper systems.
- It does **not** maintain a virtual trajectory.
- It provides pure "Gravity Assist" to make the robot feel light during user interaction.

## 🛰️ Interface

### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/cobot/wrist_ft` | `geometry_msgs/WrenchStamped` | Raw force/torque sensor data. |
| `/joint_states` | `sensor_msgs/JointState` | Current robot configuration for Jacobian calculation. |

### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/admittance/effort` | `std_msgs/Float64MultiArray` | Assistance torques sent to the Multiplexer. |

## 🛠️ Parameters
- `assist_gain`: Sensitivity of the lead-through feeling (Default: `1.0`).
- `base_link`: Mounting frame of the robot.
- `end_effector`: Sensor attachment frame.
- `robot_description`: URDF XML content for kinematics.

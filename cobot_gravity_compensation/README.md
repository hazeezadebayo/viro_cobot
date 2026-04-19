# Cobot Gravity Compensation

The `cobot_gravity_compensation` package provides **Real-time Dynamics and Friction compensation** for the Viro Cobot.

## 📐 Philosophy: Pure Dynamics (MVL)

Operating on the "Minimum Viable Logic" principle, this node is the "foundation" of the control stack. It calculates the joint torques required to neutralize the effects of gravity and joint-level friction, effectively making the robot "massless" in its default state.

This node is strictly focused on physics:
- It maintains zero interaction logic (No F/T sensor dependencies).
- It focus purely on Joint Space dynamics.
- It enables all other controllers (Admittance, Impedance) to work in a "Gravity-Free" environment.

## 🛰️ Interface

### Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/joint_states` | `sensor_msgs/JointState` | Current robot configuration for Inverse Dynamics. |

### Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/gravity/effort` | `std_msgs/Float64MultiArray` | Compensation torques sent to the Multiplexer. |

## 🛠️ Parameters
- `friction.<joint>.f_v`: Viscous friction coefficient.
- `friction.<joint>.f_c`: Coulomb friction coefficient (Static).
- `base_link`: Mounting frame (used for gravity vector orientation).
- `robot_description`: URDF XML content for mass properties.

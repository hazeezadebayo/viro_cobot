# cobot_force_estimation

### TL;DR
This package provides a **Virtual Force-Torque Sensor**. It calculates the external forces and torques (Wrench) acting on the end-effector without needing a physical $2,000 wrist sensor. It does this by observing "torque discrepancies" in the motors.

---

### How to Run
1. **Build** the package:
   ```bash
   colcon build --packages-select cobot_force_estimation
   ```
2. **Launch** the estimator:
   ```bash
   ros2 launch cobot_force_estimation force_estimation.launch.py
   ```

---

### Input and Output 

| Direction | Topic | Type | Description |
| :--- | :--- | :--- | :--- |
| **Input** | `/joint_states` | `sensor_msgs/JointState` | Current positions ($q$) and measured efforts ($\tau_{meas}$). |
| **Input** | `robot_description` | `Parameter` | URDF for Jacobian and Dynamics. |
| **Output** | `/cobot/estimated_wrench` | `geometry_msgs/WrenchStamped` | The estimated 6D force/torque at the wrist. |

---

### Mathematical Justification (The "Nervous System")

The estimator calculates the **External Torque** ($\tau_{ext}$) by subtracting the modeled gravity and friction from the measured motor efforts:

$$ \tau_{ext} = \tau_{measured} - \underbrace{\tau_{G}(q)}_{\text{Gravity}} $$

Once we have the joint torques caused by the external push, we map them to Cartesian Force ($F$) using the **Jacobian Transpose** ($J^T$):

$$ \tau_{ext} = J^T F $$
$$ F = (J^T)^\dagger \tau_{ext} $$

Where $(J^T)^\dagger$ is the pseudo-inverse of the Jacobian Transpose. This allows the robot to "feel" your hand even if it only has standard motor encoders.

> [!NOTE]
> Because this calculation relies on motor current, it can be noisy. This node includes a **Low-Pass Filter (LPF)** to smooth the data for use in Admittance control.

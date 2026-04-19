# Viro Cobot High-Fidelity Test Suite (ZeroErr Emulation)

This directory contains a high-fidelity emulation environment for the 6-joint Viro Cobot. It allows for full-stack verification of MoveIt, RViz, and SOEM-based control logic by simulating the CiA 402 state machine and specific ZeroErr drive behaviors.

## 🛠 Prerequisites

- **Python Libraries**: `scapy` (installed in the container).
- **Network Permissions**: Root/Sudo privileges are required for `veth` interface management and raw socket communication.

---

## 🐳 Dockerized Emulation (Recommended)

The easiest way to run the emulator with all its dependencies is via Docker. This ensures that `scapy` and other network tools are correctly configured.

### 1. Run the Emulator
This will automatically build the image, set up the virtual network (`enp3_sim` <-> `enp3_sim_v`), and start the emulator.

```bash
chmod +x run_eth_test.sh kill_eth_test.sh
./run_eth_test.sh
```

### 2. Stop and Cleanup
To stop the emulator and remove the virtual network interfaces:

```bash
./kill_eth_test.sh
```

### 3. Running on a Raspberry Pi (Physical Interface)
If you want to run the emulator on a Raspberry Pi connected to your laptop via a physical cable (e.g., `eth0`), pass the interface name as an argument:

```bash
./run_eth_test.sh eth0
```

---

## 🚀 Legacy Workflow (Manual)

If you prefer to run things manually on your host system:

### 1. Initialize Virtual "Cable"
The `enp3_sim` interface represents the Master port, and `enp3_sim_v` represents the Robot port.

```bash
sudo ./setup_network.sh
```

### 2. Start ZeroErr Slave Emulator
In a dedicated terminal, start the simulated 6-joint robot. This mock reports `Vendor ID: 0x5A65726F` and handles `0x6040/0x6041` state transitions.

```bash
sudo python3 robot_slave_emulator.py
```

### 3. Run Self-Diagnostic (Optional)
Verify the Master (`enp3_sim`) can find the Slaves on the virtual wire.

```bash
python3 test_soem_driver.py
```

---

## 🤖 Launching the ROS 2 Stack

By default, the system is now configured to look for the **Pseudo** configuration for testing.

```bash
ros2 launch cobot_api system.launch.py use_soem:=true sim_gazebo:=false use_fake_hardware:=false
```

### ⚙️ Switching Between Simulation and Real Hardware

To maintain a clean architecture, we have eliminated launch flags. To swap between the Emulator and the Physical Robot:

1. Open **[start_controller.launch.py](file:///home/azeez/ws/dev_env/py_code/projects/ROS/cobot/viro_cobot/cobot_moveit_config/launch/start_controller.launch.py)**.
2. At line 111, change `"pseudo_soem_config.yaml"` to `"soem_config.yaml"` (or vice-versa).
3. If using real hardware, ensure your Ethernet cable is plugged in and recognized as `enp3s0`.

---

## 🦾 Mock Fidelity Details

- **CiA 402 State Machine**: The emulator responds to Control Word commands to transition from `Switch On Disabled` -> ... -> `Operation Enabled`.
- **PDO Layout**: Uses the exact 8-byte (Rx) and 16-byte (Tx) layout defined in `ethercat_manager.cpp`.
- **Interleaved Feedback**: Echoes `Target Position` back as `Actual Position` to satisfy MoveIt's feedback loops.

---

## 🏗 Physical Robot Configuration (Hardware Contract) -> (2+ Joints)
For robots with multiple joints, each slave on the wire must follow the same contract. Below is a sample XML structure representing a network with two identical ZeroErr slaves:

```xml
<EtherCATInfoList>
    <!-- Joint 1 -->
    <EtherCATInfo>
        <Vendor><Id>1516597871</Id></Vendor>
        <Descriptions>
            <Devices>
                <Device>
                    <Type ProductCode="#x00029252" RevisionNo="#x00000001">ZeroErr Driver</Type>
                    <Name>ZeroErr_J1</Name>
                    <Sm Enable="1" StartAddress="#x1000" ControlByte="#x26" DefaultSize="128" />
                    <Sm Enable="1" StartAddress="#x1080" ControlByte="#x22" DefaultSize="128" />
                    <Sm Enable="1" StartAddress="#x1100" ControlByte="#x64" DefaultSize="0" />
                    <Sm Enable="1" StartAddress="#x1400" ControlByte="#x20" DefaultSize="0" />
                </Device>
            </Devices>
        </Descriptions>
    </EtherCATInfo>
    
    <!-- Joint 2 -->
    <EtherCATInfo>
        <Vendor><Id>1516597871</Id></Vendor>
        <Descriptions>
            <Devices>
                <Device>
                    <Type ProductCode="#x00029252" RevisionNo="#x00000001">ZeroErr Driver</Type>
                    <Name>ZeroErr_J2</Name>
                    <Sm Enable="1" StartAddress="#x1000" ControlByte="#x26" DefaultSize="128" />
                    <Sm Enable="1" StartAddress="#x1080" ControlByte="#x22" DefaultSize="128" />
                    <Sm Enable="1" StartAddress="#x1100" ControlByte="#x64" DefaultSize="0" />
                    <Sm Enable="1" StartAddress="#x1400" ControlByte="#x20" DefaultSize="0" />
                </Device>
            </Devices>
        </Descriptions>
    </EtherCATInfo>
</EtherCATInfoList>
```

> [!IMPORTANT]
> The `StartAddress` and `ControlByte` values are universal for ZeroErr drivers on this robot. If you add more joints, simply duplicate the `<EtherCATInfo>` block and ensure the robot's physical firmware is uploaded with matching SM addresses.

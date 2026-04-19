#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from cobot_msgs.srv import TaskParams
from cobot_msgs.srv import CtrlParam
import subprocess, signal, re, os, json, time

# Force unbuffered output for ALL prints in this script
def print_flush(*args, **kwargs):
    print(*args, **kwargs)
    sys.stdout.flush()

class MySubscriber(Node):

    def __init__(self):
        super().__init__('my_subscriber')
        print_flush("Initializing MySubscriber Node...")
        
        self.system_launch_process = None
        self.command = None
        self.home_goal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Absolute path for container
        # self.cli_cmd = "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
        #     ros2 launch cobot_api system.launch.py sim_gazebo:=true use_fake_hardware:=false move_group_demo:=api"

        self.ctrlparam_req = CtrlParam.Request()
        self.taskparams_req = TaskParams.Request()

        # Clients
        self.joint_tel_cli = self.create_client(CtrlParam, '/cobot_api/joint_teleop')
        self.cart_tel_cli = self.create_client(CtrlParam, '/cobot_api/cart_teleop')
        self.stop_cli = self.create_client(CtrlParam, '/cobot_api/stop_motion')
        self.vary_speed_cli = self.create_client(TaskParams, '/cobot_api/vary_speed')
        self.vary_speed_teleop_cli = self.create_client(TaskParams, '/cobot_api/vary_speed_teleop')
        self.go_to_joint_goal_client = self.create_client(TaskParams, '/cobot_api/go_to_joint_goal')
        self.go_to_cart_goal_client = self.create_client(TaskParams, '/cobot_api/go_to_cart_goal')

        # Subscriptions & Publishers
        self.subscription = self.create_subscription(JointState, 'joint_states', self.joint_states_cb, 10)
        self.gui_cmd_sub = self.create_subscription(String, 'gui_command', self.gui_cmd_cb, 10)
        self.publisher_ = self.create_publisher(String, 'gui_joint_states', 10)

        # Heartbeat timer
        self.timer = self.create_timer(0.03, self.timer_callback)
        print_flush("Node initialized and listening for /gui_command.")

    def joint_states_cb(self, msg):
        expected_order = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        joint_dict = dict(zip(msg.name, msg.position))
        if all(j in joint_dict for j in expected_order):
            sorted_positions = [joint_dict[joint] for joint in expected_order]
            joint_positions_str = ','.join(map(str, sorted_positions))
            out_msg = String()
            out_msg.data = joint_positions_str
            self.publisher_.publish(out_msg)

    def gui_cmd_cb(self, msg):
        print_flush(f"Incoming GUI Command: {msg.data}")
        self.command = msg.data

    def handle_future(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service call finished.')
        except Exception as e:
            print_flush(f"Service call failed: {e}")

    def timer_callback(self):
        if self.command is None:
            return

        cmd_str = self.command
        self.command = None # Clear immediately
        
        try:
            if ',' in cmd_str:
                parts = cmd_str.split(',')
                type = parts[0]
                val = parts[1]
                
                print_flush(f"Processing action: {type} with value {val}")

                if type == "power":
                    state = float(val)
                    if state == 1.0 and self.system_launch_process is None:
                        # Extract mode if provided (format: power,1,mode)
                        mode = parts[2] if len(parts) > 2 else "simulation"
                        
                        sim_gazebo = "true" if (mode == "simulation" or mode == "free_mode") else "false"
                        use_fake = "true" if mode == "headless" else "false"
                        use_soem = "true" if mode == "real_soem" else "false"
                        
                        # Add control_mode and interaction flags
                        control_mode = "effort" if mode == "free_mode" else "position"
                        enable_interaction = "true" if mode == "free_mode" else "false"

                        launch_cmd = f"source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
                            ros2 launch cobot_api system.launch.py sim_gazebo:={sim_gazebo} \
                            use_fake_hardware:={use_fake} use_soem:={use_soem} \
                            control_mode:={control_mode} enable_interaction:={enable_interaction} \
                            move_group_demo:=api"
                        
                        print_flush(f"Starting Cobot Backend in {mode} mode...")
                        print_flush(f"Command: {launch_cmd}")
                        
                        self.system_launch_process = subprocess.Popen(
                            launch_cmd, 
                            shell=True, 
                            preexec_fn=os.setpgrp, 
                            executable="/bin/bash"
                        )
                        print_flush(f"Process PID: {self.system_launch_process.pid}")
                    elif state == 0.0 and self.system_launch_process:
                        print_flush("Stopping Cobot Backend...")
                        os.killpg(os.getpgid(self.system_launch_process.pid), signal.SIGTERM)
                        self.system_launch_process = None
                
                elif type == "joint":
                    self.send_joint_teleop_request(int(val))
                elif type == "cartesian":
                    self.send_cart_teleop_request(int(val))
                elif type == "speed_val":
                    self.send_vary_speed_teleop_request(float(val))
                elif type == "stop":
                    self.send_stop_request(int(val))
                elif type == "home":
                    self.send_go_to_joint_goal_request(self.home_goal)
                elif type == "interaction_gain":
                    # Dynamic parameter update
                    set_cmd = f"ros2 param set /gravity_comp_node interaction_gain {val}"
                    subprocess.Popen(set_cmd, shell=True, executable="/bin/bash")
                elif type == "zero_sensor" and val == "1":
                    # Trigger zeroing bias in gravity comp node
                    set_cmd = "ros2 param set /gravity_comp_node zero_bias true"
                    subprocess.Popen(set_cmd, shell=True, executable="/bin/bash")
                    # Wait 500ms and set back to false
                    time.sleep(0.5)
                    reset_cmd = "ros2 param set /gravity_comp_node zero_bias false"
                    subprocess.Popen(reset_cmd, shell=True, executable="/bin/bash")
                elif type == "manual_force":
                    # manual_force,fx,fy,fz,tx,ty,tz
                    if len(parts) >= 7:
                        fx, fy, fz = parts[1], parts[2], parts[3]
                        tx, ty, tz = parts[4], parts[5], parts[6]
                        
                        print_flush(f"holding action: {type} with value wrench: force: x: {fx}, y: {fy}, z: {fz}")

                        # Use 'ign topic' with /persistent to ensure the force stays active and visible
                        # We use 'cobot_arm::link_6' because Gazebo namespaces spawned links.
                        ign_cmd = (
                            f"ign topic -t \"/world/empty/wrench/persistent\" -m ignition.msgs.EntityWrench "
                            f"-p \"entity: {{name: 'cobot_arm::link_6', type: LINK}}, "
                            f"wrench: {{force: {{x: {fx}, y: {fy}, z: {fz}}}, "
                            f"torque: {{x: {tx}, y: {ty}, z: {tz}}}}}\""
                        )
                        subprocess.Popen(ign_cmd, shell=True, executable="/bin/bash")
                        print_flush(f"Persistent wrench applied via ign: F[{fx},{fy},{fz}]")
        except Exception as e:
            print_flush(f"Error in command processing: {e}")

    def send_joint_teleop_request(self, number):
        if not self.joint_tel_cli.wait_for_service(timeout_sec=1.0):
            print_flush("Joint teleop service not available")
            return
        self.ctrlparam_req.command = number
        self.joint_tel_cli.call_async(self.ctrlparam_req).add_done_callback(self.handle_future)

    def send_cart_teleop_request(self, number):
        if not self.cart_tel_cli.wait_for_service(timeout_sec=1.0):
            print_flush("Cart teleop service not available")
            return
        self.ctrlparam_req.command = number
        self.cart_tel_cli.call_async(self.ctrlparam_req).add_done_callback(self.handle_future)

    def send_vary_speed_teleop_request(self, speed):
        if not self.vary_speed_teleop_cli.wait_for_service(timeout_sec=1.0):
            print_flush("Vary speed teleop service not available")
            return
        self.taskparams_req.cmd1 = speed
        self.vary_speed_teleop_cli.call_async(self.taskparams_req).add_done_callback(self.handle_future)

    def send_stop_request(self, number):
        if not self.stop_cli.wait_for_service(timeout_sec=1.0):
            print_flush("Stop service not available")
            return
        self.ctrlparam_req.command = number
        self.stop_cli.call_async(self.ctrlparam_req).add_done_callback(self.handle_future)

    def send_go_to_joint_goal_request(self, joint_goal):
        if not self.go_to_joint_goal_client.wait_for_service(timeout_sec=1.0):
            print_flush("GoToJoint service not available")
            return
        self.taskparams_req.cmd1, self.taskparams_req.cmd2, self.taskparams_req.cmd3 = joint_goal[0:3]
        self.taskparams_req.cmd4, self.taskparams_req.cmd5, self.taskparams_req.cmd6 = joint_goal[3:6]
        self.go_to_joint_goal_client.call_async(self.taskparams_req).add_done_callback(self.handle_future)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MySubscriber()
        print_flush("GUI Bridge is SPINNING...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

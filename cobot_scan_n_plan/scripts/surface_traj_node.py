#!/usr/bin/env python3

import os
import sys
import rclpy
import numpy as np
import pandas as pd
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from builtin_interfaces.msg import Duration
from std_srvs.srv import Trigger

# -----------------------------------------------------------------------------
# Library Class (Consolidated)
# -----------------------------------------------------------------------------
class JointTrajectoryExecuter:
    """
    A class to execute joint trajectories on a robotic arm using ROS 2.
    Responsible for loading, validating, and publishing joint trajectory messages.
    """
    
    def __init__(self, node, controller_name, joint_names):
        """ Initialize the Executer with a ROS 2 node, controller name, and joint names. """
        self.node = node
        self.controller_name = controller_name
        self.joint_names = joint_names
        
        self.pos_error = [0.0 for _ in range(len(joint_names))]
        self.vel_error = [0.0 for _ in range(len(joint_names))]
        
        self.q_array = []
        self.qd_array = []
        self.qdd_array = []
        self.time_array = np.array([])
        self.total_traj_time = 0.0
        self.time_between_points = 0.0
        
        self.round_digit = 5
        
        self.columns = ["time", 
                        "q1", "q2", "q3", "q4", "q5", "q6", 
                        "qd1", "qd2", "qd3", "qd4", "qd5", "qd6", 
                        "qdd1", "qdd2", "qdd3", "qdd4", "qdd5", "qdd6"]
        self.traj_df = pd.DataFrame(columns=self.columns)
        self.len_df = 0
        
        self.is_traj_ok = False
        self.traj_msg = JointTrajectory()
        self.traj_msg.joint_names = self.joint_names

        # Setup ROS 2 Publisher and Subscriber
        self.traj_publisher = self.node.create_publisher(
            JointTrajectory, 
            f"/{self.controller_name}/joint_trajectory", 
            10
        )
        self.node.create_subscription(
            JointTrajectoryControllerState, 
            f"/{self.controller_name}/state", 
            self.state_callback, 
            10
        )
        
    def state_callback(self, msg):
        """ Callback to update position and velocity errors from the controller state. """
        for i, name in enumerate(self.joint_names):
            if name in msg.joint_names:
                idx = msg.joint_names.index(name)
                if len(msg.error.positions) > idx:
                    self.pos_error[i] = msg.error.positions[idx]
                if len(msg.error.velocities) > idx:
                    self.vel_error[i] = msg.error.velocities[idx]
        
    def go_to_position(self, position, traj_time=5.0):
        """ Publish a single trajectory point to move the arm to the target position. """
        t_msg = JointTrajectory()
        t_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = list(position)
        point.velocities = [0.0 for _ in range(len(self.joint_names))]
        
        secs = int(traj_time)
        nanosecs = int((traj_time - secs) * 1e9)
        point.time_from_start = Duration(sec=secs, nanosec=nanosecs)
        
        t_msg.points.append(point)
        t_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.traj_publisher.publish(t_msg)
        self.node.get_logger().info(f"Published go_to_position target. Time: {traj_time}s")
            
    def go_first_traj_point(self, traj_time=5.0):
        """ Move the robot to the first point of the loaded trajectory. """
        if self.is_traj_ok and len(self.q_array) > 0:
            self.go_to_position(self.q_array[0], traj_time)
        else:
            self.node.get_logger().error("Cannot move to first point: Trajectory not loaded.")
        
    def load_trajectory(self, traj_path: str):
        """ Load a joint trajectory from a CSV file into internal arrays. """
        try:
            input_csv_df = pd.read_csv(traj_path)
            
            if input_csv_df.columns[0] != "time":
                input_csv_df = pd.read_csv(traj_path, header=None)
                input_csv_df.columns = self.columns

            self.traj_df = input_csv_df
    
            self.time_array = self.traj_df["time"].astype(float).values
            if round(self.time_array[0], self.round_digit) == 0.0:
                self.traj_df = self.traj_df.drop(self.traj_df.index[0])
                self.time_array = self.traj_df["time"].astype(float).values
            
            self.len_df = len(self.traj_df)
            self.total_traj_time = self.time_array[-1]
            self.time_between_points = self.time_array[1] - self.time_array[0] if self.len_df > 1 else 0
            
            q_cols = ["q1", "q2", "q3", "q4", "q5", "q6"]
            qd_cols = ["qd1", "qd2", "qd3", "qd4", "qd5", "qd6"]
            qdd_cols = ["qdd1", "qdd2", "qdd3", "qdd4", "qdd5", "qdd6"]

            self.q_array = self.traj_df[q_cols].values.tolist()
            self.qd_array = self.traj_df[qd_cols].values.tolist()
            self.qdd_array = self.traj_df[qdd_cols].values.tolist()
            
            self.is_traj_ok = True
            self.node.get_logger().info(f"Trajectory loaded: {self.len_df} points, {self.total_traj_time:.2f}s total.")
        
        except Exception as e:
            self.node.get_logger().error(f"Failed to load trajectory CSV: {e}")
            self.is_traj_ok = False
            self.clear_trajectory()
    
    def generate_traj_msg(self):
        """ Maps the loaded internal arrays to a ROS 2 JointTrajectory message. """
        if not self.is_traj_ok:
            raise ValueError("Trajectory is not loaded!")
        
        self.traj_msg.points.clear()
            
        for i in range(self.len_df):
            point = JointTrajectoryPoint()
            
            point.positions = self.q_array[i]
            point.velocities = self.qd_array[i]
            point.accelerations = self.qdd_array[i]
            
            time_from_start = self.time_array[i]
            secs = int(time_from_start)
            nsecs = int((time_from_start - secs) * 1e9)
            point.time_from_start = Duration(sec=secs, nanosec=nsecs)

            self.traj_msg.points.append(point)
        
    def run_trajectory(self):
        """ Publish the full multi-point trajectory to the JTC. """
        try:
            self.generate_traj_msg()
            self.traj_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.traj_publisher.publish(self.traj_msg)
            self.node.get_logger().info("Published full joint trajectory to controller.")
        except Exception as e:
            self.node.get_logger().error(f"Error publishing trajectory: {e}")
    
    def stop_trajectory(self):
        """ Publish an empty trajectory to halt motion. """
        t_msg = JointTrajectory()
        t_msg.joint_names = self.joint_names
        t_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.traj_publisher.publish(t_msg)
        self.node.get_logger().info("Stop command published.")
    
    def clear_trajectory(self):
        """ Reset local data buffers. """
        self.traj_msg.points.clear()
        self.q_array = []
        self.qd_array = []
        self.qdd_array = []
        self.time_array = np.array([])
        self.total_traj_time = 0.0
        self.time_between_points = 0.0

# -----------------------------------------------------------------------------
# Node Class
# -----------------------------------------------------------------------------
class SurfaceTrajNode(Node):
    """
    A persistent ROS 2 Node orchestrating the execution of Joint Trajectories.
    """
    def __init__(self):
        super().__init__('surface_traj_node')
        
        # Declare Parameters
        self.declare_parameter('controller_name', 'orion_arm_controller')
        self.declare_parameter('joint_names', [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
        ])
        
        controller_name = self.get_parameter('controller_name').value
        joint_names = self.get_parameter('joint_names').value
        
        # Setup Executer Abstraction
        self.executer = JointTrajectoryExecuter(self, controller_name, joint_names)
        
        # Resolve path
        try:
            pkg_share = get_package_share_directory('cobot_scan_n_plan')
            self.csv_path = os.path.join(pkg_share, "config", "joint_trajectory.csv")
        except Exception:
            self.csv_path = "joint_trajectory.csv"
            
        # Services
        self.execute_srv = self.create_service(Trigger, '/mcfly_scan_n_plan/execute', self.execute_srv_cb)
            
        self.get_logger().info(f"Surface Traj Node started against /{controller_name}.")

    def execute_srv_cb(self, request, response):
        """
        Trigger service to run the trajectory execution workflow.
        """
        self.get_logger().info(">>> TRIGGERED TRAJECTORY EXECUTION TASK <<<")
        
        if not os.path.exists(self.csv_path):
            error_msg = f"Cannot find {self.csv_path}! Ensure the Cartesian Trajectory was passed through IK first."
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            return response
            
        self.get_logger().info(f"Loading Inverse Kinematics trajectory from {self.csv_path}")
        self.executer.load_trajectory(self.csv_path)
        
        if not self.executer.is_traj_ok:
            error_msg = "Trajectory failed validation. Aborting."
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            return response
            
        self.get_logger().info("Moving to starting point...")
        self.executer.go_first_traj_point(traj_time=5.0)
        
        # In a real async scenario, you might wait for reach. 
        # For now, we fire the trajectory.
        self.get_logger().info("Executing full tracking trajectory...")
        self.executer.run_trajectory()
        
        response.success = True
        response.message = "Trajectory execution started."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SurfaceTrajNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupt received, stopping robot.")
        node.executer.stop_trajectory()
    except Exception as e:
        node.get_logger().error(f"Node Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

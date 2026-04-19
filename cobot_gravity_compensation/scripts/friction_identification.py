#!/usr/bin/env python3

import rclpy
from rclcpp.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time

class FrictionIdentifier(Node):
    def __init__(self):
        super().__init__('friction_identifier')
        
        self.joint_name = self.declare_parameter('joint', 'joint_1').value
        self.velocities = [0.1, 0.2, 0.4, 0.6, 0.8] # rad/s
        
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        self.current_pos = 0.0
        self.current_effort = 0.0
        self.data_points = []
        
        self.get_logger().info(f"Starting Friction Identification for {self.joint_name}")
        self.run_calibration()

    def joint_callback(self, msg):
        if self.joint_name in msg.name:
            idx = msg.name.index(self.joint_name)
            self.current_pos = msg.position[idx]
            self.current_effort = msg.effort[idx]

    def move_to_start(self):
        self.get_logger().info("Moving to start position...")
        msg = JointTrajectory()
        msg.joint_names = [self.joint_name]
        point = JointTrajectoryPoint()
        point.positions = [0.0]
        point.effort = [0.0]
        point.time_from_start.sec = 2
        msg.points.append(point)
        self.pub.publish(msg)
        time.sleep(3)

    def run_calibration(self):
        # We'll run this in a thread or simple loop for the script
        # Note: In a real educational script, we use a simple synchronous approach for clarity
        time.sleep(1) # Wait for joint states
        self.move_to_start()
        
        results = []
        
        for v in self.velocities:
            self.get_logger().info(f"Capturing data at velocity: {v} rad/s")
            
            # Command trajectory
            msg = JointTrajectory()
            msg.joint_names = [self.joint_name]
            
            # Point 1: Move for 2 seconds
            p1 = JointTrajectoryPoint()
            p1.positions = [self.current_pos + v * 2.0]
            p1.velocities = [v]
            p1.time_from_start.sec = 2
            msg.points.append(p1)
            
            self.pub.publish(msg)
            
            # Collect data during the middle of the move
            time.sleep(0.5)
            efforts = []
            for _ in range(20):
                efforts.append(self.current_effort)
                time.sleep(0.05)
            
            avg_effort = np.mean(efforts)
            results.append((v, avg_effort))
            time.sleep(1.0) # Finish move

        # 5. Math: Linear Fit y = mx + c
        vels = [r[0] for r in results]
        effs = [abs(r[1]) for r in results]
        
        m, c = np.polyfit(vels, effs, 1)
        
        self.get_logger().info("--- CALIBRATION RESULTS ---")
        self.get_logger().info(f"Joint: {self.joint_name}")
        self.get_logger().info(f"Viscous Friction (f_v): {m:.4f}")
        self.get_logger().info(f"Coulomb Friction (f_c): {c:.4f}")
        self.get_logger().info(f"Static Friction (f_s): Estimating as {c*1.2:.4f} (1.2x Coulomb)")
        self.get_logger().info("---------------------------")
        
        # Self-destruct
        rclpy.shutdown()

def main():
    rclpy.init()
    node = FrictionIdentifier()
    # Node manages its own loop in run_calibration for script mode
    # For a real node we would use spin(), but for a script this is clearer for students

if __name__ == '__main__':
    main()

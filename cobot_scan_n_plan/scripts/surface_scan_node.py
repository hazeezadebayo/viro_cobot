#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool
from ament_index_python.packages import get_package_share_directory

# Add parent directory for development imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Library Imports
from cobot_scan_n_plan.surface_scan import read_points, PointCloudScanner

class SurfaceScanNode(Node):
    def __init__(self):
        super().__init__('surface_scan_node')

        # File path for saving the scan
        try:
            pkg_share = get_package_share_directory("cobot_scan_n_plan")
            output_path = os.path.join(pkg_share, "config", "point_cloud.ply")
        except Exception:
            output_path = "point_cloud.ply"

        self.scanner = PointCloudScanner(output_path=output_path)

        # Parameters
        self.declare_parameter('pointcloud_topic', '/kinect_camera/points')
        topic = self.get_parameter('pointcloud_topic').value

        # Subscription
        self.pc_subscriber = self.create_subscription(
            PointCloud2,
            topic,
            self.pc_callback,
            10
        )

        # Service
        self.srv = self.create_service(SetBool, '/mcfly_scan_n_plan/scan', self.scan_srv_cb)

        self.get_logger().info(f"SurfaceScan Orchestrator Node initialized. Topic: {topic}")

    def scan_srv_cb(self, request, response):
        if request.data:
            self.scanner.start_session()
            self.get_logger().info("Scan session started.")
        else:
            self.scanner.stop_session()
            self.get_logger().info("Scan session stopped and saved.")
        
        response.success = True
        response.message = f"Scan state: {request.data}"
        return response

    def pc_callback(self, msg):
        if self.scanner.is_active:
            try:
                # Extract points using library
                points = list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
                # Update scanner
                self.scanner.update(points)
            except Exception as e:
                self.get_logger().error(f"Error processing point cloud: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SurfaceScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.scanner.stop_session()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

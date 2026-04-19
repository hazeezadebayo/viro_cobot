#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger
import csv

# Add the parent directory to path so we can import from the package during development
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Library Imports
from cobot_scan_n_plan.surface_plan import plan
from cobot_scan_n_plan.surface_traj import process_csv_to_trajectory

class SurfacePlanNode(Node):
    """
    A persistent ROS 2 Node orchestrating the geometric path generation.
    """
    def __init__(self):
        super().__init__('surface_plan_node')

        # Centralized Parameters (Defaulted to User Snippet Values)
        self.declare_parameter('dist_threshold', 0.55)
        self.declare_parameter('downsample_voxel_size', 0.015)
        self.declare_parameter('path_choice', 0)     # 0 for spiral
        self.declare_parameter('max_search_length', 0.6)
        self.declare_parameter('neighbourhood_radius', 0.2)
        self.declare_parameter('grid_x_offset', 0.03)
        self.declare_parameter('grid_y_offset', 0.03)
        self.declare_parameter('connectedness_radius', 0.0001)
        self.declare_parameter('stretch_limit', 0.2)
        self.declare_parameter('skip_count', 1)
        self.declare_parameter('show_frames', True)
        self.declare_parameter('show_visualization', True)

        # Service
        self.plan_srv = self.create_service(Trigger, '/mcfly_scan_n_plan/plan', self.plan_srv_cb)

        self.get_logger().info("Persistent Surface Planning Node initialized.")

    def get_plan_params(self):
        return {
            'dist_threshold': self.get_parameter('dist_threshold').value,
            'downsample_voxel_size': self.get_parameter('downsample_voxel_size').value,
            'path_choice': self.get_parameter('path_choice').value,
            'max_search_length': self.get_parameter('max_search_length').value,
            'neighbourhood_radius': self.get_parameter('neighbourhood_radius').value,
            'grid_x_offset': self.get_parameter('grid_x_offset').value,
            'grid_y_offset': self.get_parameter('grid_y_offset').value,
            'connectedness_radius': self.get_parameter('connectedness_radius').value,
            'stretch_limit': self.get_parameter('stretch_limit').value,
            'skip_count': self.get_parameter('skip_count').value,
            'show_frames': self.get_parameter('show_frames').value,
            'show_visualization': self.get_parameter('show_visualization').value
        }

    def plan_srv_cb(self, request, response):
        """
        Trigger service to run the geometric planning workflow.
        """
        self.get_logger().info(">>> TRIGGERED PLANNING TASK <<<")

        # RESOLVE PATHS
        try:
            pkg_share = get_package_share_directory('cobot_scan_n_plan')
            ply_path = os.path.join(pkg_share, "config", "point_cloud.ply")
            csv_path = os.path.join(pkg_share, "config", "scan_n_plan_path.csv")
            traj_path = os.path.join(pkg_share, "config", "scan_n_plan_traj.csv")
        except Exception as e:
            error_msg = f"Failed to resolve package paths: {e}"
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            return response

        if not os.path.exists(ply_path):
            error_msg = f"Input file not found: {ply_path}. Please scan first."
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            return response

        # EXECUTE PLANNING
        try:
            self.get_logger().info(f"Executing Geometric Planning on {ply_path}...")
            params = self.get_plan_params()
            
            # Call the library function
            path, orientations = plan(ply_path, **params)

            # Validation logic
            assert path is not None and len(path) > 0, "Path is empty"
            assert orientations is not None and len(orientations) > 0, "Orientations are empty"
            assert len(path) == len(orientations), f"Length mismatch: path={len(path)}, orientations={len(orientations)}"

            # SAVE PATH CSV
            self.get_logger().info(f"Saving spatial path to {csv_path}...")
            with open(csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['y1', 'y2', 'y3', 'y4', 'y5', 'y6', 'y7'])
                for p, q in zip(path, orientations):
                    writer.writerow([p[0], p[1], p[2], q[0], q[1], q[2], q[3]])

            # GENERATE TRAJECTORY
            self.get_logger().info(f"Generating Time-Parameterized Trajectory -> {traj_path}")
            process_csv_to_trajectory(csv_path, traj_path, velocity=0.1, acceleration=0.05)
            
            self.get_logger().info(">>> PLANNING TASK COMPLETE <<<")
            response.success = True
            response.message = "Planning and trajectory generation complete."
        except Exception as e:
            error_msg = f"Planning failed: {e}"
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg

        return response

def main(args=None):
    rclpy.init(args=args)
    node = SurfacePlanNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Node Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

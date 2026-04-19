import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('cobot_scan_n_plan')
    
    # Declare arguments
    pc_ply_arg = DeclareLaunchArgument(
        'pc_ply',
        default_value=os.path.join(pkg_share, 'config', 'point_cloud.ply'),
        description='Path to the input .ply file'
    )
    
    return LaunchDescription([
        pc_ply_arg,
        Node(
            package='cobot_scan_n_plan',
            executable='surface_plan_node',
            name='surface_plan',
            output='screen',
            parameters=[{
                'pointcloud_ply_path': LaunchConfiguration('pc_ply'),
                'dist_threshold': 1.5,
                'downsample_voxel_size': 0.2,
                'path_choice': 1,
                'max_search_length': 0.6,
                'neighbourhood_radius': 0.2,
                'grid_x_offset': 0.03,
                'grid_y_offset': 0.03,
                'connectedness_radius': 0.0001,
                'stretch_limit': 0.2,
                'skip_count': 1,
                'show_frames': True,
                'show_visualization': True
            }]
        )
    ])

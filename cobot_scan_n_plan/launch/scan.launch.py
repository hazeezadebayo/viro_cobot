from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cobot_scan_n_plan',
            executable='surface_scan',
            name='surface_scan',
            output='screen',
            parameters=[
                {'pointcloud_topic': '/kinect_camera/points'}
            ]
        )
    ])

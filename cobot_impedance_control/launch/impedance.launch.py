import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('cobot_impedance_control')
    default_config_path = os.path.join(pkg_path, 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config_path),

        Node(
            package='cobot_impedance_control',
            executable='impedance_node',
            name='impedance_node',
            parameters=[LaunchConfiguration('config_file')],
            output='screen'
        )
    ])

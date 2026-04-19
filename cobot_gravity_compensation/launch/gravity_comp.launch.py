import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('cobot_gravity_compensation')
    
    # Path to YAML config
    default_config_path = os.path.join(pkg_path, 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config_path),
        DeclareLaunchArgument('base_link', default_value='base_link'),
        DeclareLaunchArgument('end_effector', default_value='wrist_3_link'),

        Node(
            package='cobot_gravity_compensation',
            executable='gravity_comp_node',
            name='gravity_comp_node',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'base_link': LaunchConfiguration('base_link'),
                    'end_effector': LaunchConfiguration('end_effector'),
                }
            ],
            output='screen'
        )
    ])

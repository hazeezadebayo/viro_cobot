from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_description', default_value=''),
        DeclareLaunchArgument('assist_gain', default_value='2.0'),
        DeclareLaunchArgument('base_link', default_value='base_link'),
        DeclareLaunchArgument('end_effector', default_value='link_6'),

        Node(
            package='cobot_admittance_control',
            executable='admittance_node',
            name='admittance_node',
            output='screen',
            parameters=[{
                'robot_description': LaunchConfiguration('robot_description'),
                'assist_gain': LaunchConfiguration('assist_gain'),
                'base_link': LaunchConfiguration('base_link'),
                'end_effector': LaunchConfiguration('end_effector'),
            }]
        )
    ])

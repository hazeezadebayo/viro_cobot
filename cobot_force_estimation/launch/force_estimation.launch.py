import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('cobot_force_estimation')

    estimator_node = Node(
        package='cobot_force_estimation',
        executable='wrench_estimator_node',
        name='wrench_estimator',
        output='screen',
        parameters=[{
            'base_link': 'base_link',
            'end_effector': 'wrist_3_link', # link_6 usually
            'lpf_alpha': 0.1, # Smoothing factor
        }]
    )

    return LaunchDescription([
        estimator_node
    ])

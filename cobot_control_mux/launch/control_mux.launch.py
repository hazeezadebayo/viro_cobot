import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Configuration
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    
    return LaunchDescription([
        DeclareLaunchArgument('update_rate_hz', default_value='100.0'),

        Node(
            package='cobot_control_mux',
            executable='control_mux_node',
            name='control_mux_node',
            output='screen',
            parameters=[{
                'update_rate_hz': LaunchConfiguration('update_rate_hz'),
                'joint_names': joint_names,
                'gravity_effort_topic': '/gravity/effort',
                'admittance_effort_topic': '/admittance/effort',
                'impedance_effort_topic': '/impedance/effort',
                'joint_state_topic': '/joint_states',
                'jtc_sync_topic': '/joint_trajectory_controller/joint_trajectory',
                'direct_effort_topic': '/effort_direct_controller/commands',
                'moveit_effort_topic': '/moveit/effort',
            }]
        ),

        Node(
            package='cobot_control_mux',
            executable='moveit_effort_bridge',
            name='moveit_effort_bridge',
            output='screen',
            parameters=[{
                'joint_names': joint_names,
                'effort_topic': '/moveit/effort',
                # Gains are default in code, can be overridden here if needed
            }]
        )
    ])

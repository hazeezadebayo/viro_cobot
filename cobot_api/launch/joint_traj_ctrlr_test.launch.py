#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ros2 launch cobot_api joint_traj_ctrlr_test.launch.py

def generate_launch_description():

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("cobot_api"),
            "config",
            "joint_trajectory_positions.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="cobot_api",
                executable="joint_traj_ctrlr_test.py",
                name="joint_traj_ctrlr_test",
                parameters=[position_goals],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )
        ]
    )

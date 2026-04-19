import os
from launch import LaunchDescription
from launch_ros.descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():

    # ARGUMENTS ---------------------------------------------------------------

    rviz_arg = DeclareLaunchArgument("rviz_gui", default_value="false",
                                       description="Flag to enable rviz")

    # current package path
    pkg_share_path = get_package_share_directory("cobot_description")

    # Rviz config path
    rviz_config_path = PathJoinSubstitution(
        [pkg_share_path, "rviz", "display.rviz"]
    )

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_share_path, "urdf", "cobot.xacro"]
            ),
            " ",
            "name:=cobot_arm",
            " ",
            "use_fake_hardware:=true",
            " ",
            "sim_gazebo:=false"
        ]
    )

    # NODES -----------------------------------------------------------------

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": ParameterValue(robot_description, value_type=str)}]
    )
    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz_gui"))
    )

    # GAZEBO -----------------------------------------------------------------

    description_share = os.path.join(get_package_prefix("cobot_description"), "share")
    gazebo_env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", description_share)

    gazebo_server = ExecuteProcess(
        cmd=[
              "gzserver",
              "--verbose",
              "-s", "libgazebo_ros_factory.so",
              os.path.join(pkg_share_path, "worlds", "empty.world"),
        ],
        output="screen"
    )
    
    gazebo_client = ExecuteProcess(cmd=["gzclient"], output="screen")

    spawn_robot = Node(
        name="spawn_robot",
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-entity", "cobot_arm", 
                   "-topic", "/robot_description",
                   "-x", "0.0",
                   "-y", "0.0",
                   "-z", "0.1"]
    )

    # ROS 2 CONTROL ----------------------------------------------------------

    joint_state_broadcaster = Node(
        name="joint_state_broadcaster",
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"       
        ]
    )

    joint_trajectory_controller = Node(
        name="joint_trajectory_controller",
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager"       
        ]
    )

    # Return Launch Function -------------------------------------------------

    return LaunchDescription(
        [
            rviz_arg,
            rviz_node,
            robot_state_publisher_node,
            
            gazebo_env_var,
            gazebo_server,
            gazebo_client,
            spawn_robot,

            joint_state_broadcaster,
            joint_trajectory_controller,
        ]
    )
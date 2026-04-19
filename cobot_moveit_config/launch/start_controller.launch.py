import os
from launch import LaunchDescription
from launch_ros.descriptions import ParameterValue
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch.actions import Shutdown, DeclareLaunchArgument
from launch.conditions import UnlessCondition, LaunchConfigurationEquals

from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():

    # Declare the launch arguments
    sim_gazebo_arg = DeclareLaunchArgument(
        'sim_gazebo',
        default_value='false',
        description='Simulation in Gazebo'
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use fake hardware driver'
    )

    use_soem_arg = DeclareLaunchArgument(
        'use_soem',
        default_value='false',
        description='Use SOEM hardware driver'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='cobot_arm',
        description='Name of the robot'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='position',
        description='Control mode: position or effort'
    )

    # Get the launch arguments
    sim_gazebo = LaunchConfiguration('sim_gazebo')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_soem = LaunchConfiguration('use_soem')
    robot_name = LaunchConfiguration('robot_name')
    control_mode = LaunchConfiguration('control_mode')

    # Map control_mode to mode_of_operation
    # Position: 8, Torque(Effort): 10
    mode_of_operation = PythonExpression([
        "'10' if '", control_mode, "' == 'effort' else '8'"
    ])

    # package paths
    pkg_share_path = get_package_share_directory("cobot_description")

    # ROS 2 CONTROL ----------------------------------------------------------

    joint_state_broadcaster = TimerAction(
        period=1.0,
        actions=[
            Node(
                name="joint_state_broadcaster",
                package="controller_manager",
                executable="spawner",
                output="screen",
                parameters=[{"use_sim_time": sim_gazebo}],
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager"
                ]
            )
        ]
    )

    # Choose which controller to spawn based on mode
    selected_controller = PythonExpression([
        "'effort_direct_controller' if '", control_mode, "' == 'effort' else 'joint_trajectory_controller'"
    ])

    joint_trajectory_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                name="joint_trajectory_controller_spawner",
                package="controller_manager",
                executable="spawner",
                output="screen",
                parameters=[{"use_sim_time": sim_gazebo}],
                arguments=[
                    selected_controller,
                    "--controller-manager",
                    "/controller_manager"
                ]
            )
        ]
    )

    ros2_controllers_path = os.path.join(
        pkg_share_path,
        "config",
        "cobot_controllers.yaml",
    )

    xacro_file = os.path.join(get_package_share_directory('cobot_description'),
                             'urdf',
                             'cobot.xacro')
    
    gripper_controller_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                name="gripper_controller_spawner",
                package="controller_manager",
                executable="spawner",
                output="screen",
                parameters=[{"use_sim_time": sim_gazebo}],
                arguments=[
                    "gripper_controller",
                    "--controller-manager",
                    "/controller_manager"       
                ]
            )
        ]
    )

    soem_config_path = os.path.join(
        pkg_share_path,
        "config",
        "pseudo_soem_config.yaml", # "soem_config.yaml",
    )

    # ROS 2 CONTROL ----------------------------------------------------------
    robot_description_config = Command(
        [FindExecutable(name='xacro'),
         ' ',
         xacro_file,
         ' hand:=true',
         ' name:=',
         robot_name,
         ' use_fake_hardware:=',
         use_fake_hardware,
         ' sim_gazebo:=',
         sim_gazebo,
         ' use_soem:=',
         use_soem,
         ' initial_mode_of_operation:=',
         mode_of_operation])
    robot_description = {'robot_description': robot_description_config}

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": sim_gazebo}]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, 
            ros2_controllers_path, 
            soem_config_path,
            {"use_sim_time": sim_gazebo}
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
        condition=UnlessCondition(sim_gazebo)
    )

    # Return Launch Function -------------------------------------------------

    return LaunchDescription(
        [
            sim_gazebo_arg,
            use_fake_hardware_arg,
            use_soem_arg,
            robot_name_arg,
            control_mode_arg,

            robot_state_publisher_node,
            ros2_control_node,

            joint_state_broadcaster,
            joint_trajectory_controller,
            gripper_controller_spawner,
        ]
    )
# import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """ generate_launch_description """
    # Declare the launch arguments
    move_group_demo_arg = DeclareLaunchArgument(
        'move_group_demo',
        default_value='x',
        description='Options: joint, api, cart, job, x'
    )

    sim_gazebo_arg = DeclareLaunchArgument(
        'sim_gazebo',
        default_value='false',
        description='Simulation in Gazebo'
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware driver'
    )

    use_soem_arg = DeclareLaunchArgument(
        'use_soem',
        default_value='false',
        description='Use SOEM hardware driver'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='position',
        description='Control mode: position or effort'
    )

    enable_interaction_arg = DeclareLaunchArgument(
        'enable_interaction',
        default_value='false',
        description='Enable interactive force assistance'
    )

    # Get the launch arguments
    sim_gazebo = LaunchConfiguration('sim_gazebo')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_soem = LaunchConfiguration('use_soem')
    move_group_demo = LaunchConfiguration('move_group_demo')
    control_mode = LaunchConfiguration('control_mode')
    enable_interaction = LaunchConfiguration('enable_interaction')

    # Define the 'start_controller' launch
    start_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("cobot_moveit_config"),
                    "launch",
                    "start_controller.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("sim_gazebo", sim_gazebo),
            ("use_fake_hardware", use_fake_hardware),
            ("use_soem", use_soem),
            ("control_mode", control_mode),
        ],
    )

    # Define the 'cobot_api' launch
    cobot_api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("cobot_api"),
                    "launch",
                    "cobot_api.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("move_group_demo", move_group_demo),
            ("sim_gazebo", sim_gazebo),
            ("use_fake_hardware", use_fake_hardware),
            ("use_soem", use_soem),
            ("enable_interaction", enable_interaction),
            ("control_mode", control_mode),
        ],
    )

    return LaunchDescription([
        move_group_demo_arg,
        sim_gazebo_arg,
        use_fake_hardware_arg,
        use_soem_arg,
        control_mode_arg,
        enable_interaction_arg,
        cobot_api_launch,
        start_controller_launch,
    ])

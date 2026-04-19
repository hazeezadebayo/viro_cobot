import os
import yaml, xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,Command, FindExecutable
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    rviz_config_file = os.path.join(get_package_share_directory("cobot_moveit_config"), "rviz", "moveit.rviz")
    cobot_urdf_xacro = os.path.join(get_package_share_directory("cobot_description"), "urdf", "cobot.xacro") 
    cobot_srdf_xacro = os.path.join( get_package_share_directory("cobot_moveit_config"), "srdf", "cobot.srdf" )
    kinematics_yaml = load_yaml("cobot_moveit_config", "config/kinematics.yaml")
    ompl_planning_yaml = load_yaml("cobot_moveit_config", "config/ompl_planning.yaml")
    moveit_simple_controllers_yaml = load_yaml( "cobot_moveit_config", "config/moveit_controllers.yaml" )
    joint_limits_yaml = { "robot_description_planning": load_yaml( "cobot_moveit_config", "config/joint_limits.yaml") }   

    # context = LaunchContext()
    robot_doc = xacro.process_file(
        cobot_urdf_xacro,
        mappings={},
    )
    robot_urdf = robot_doc.toprettyxml(indent="  ")

    robot_description = {"robot_description": robot_urdf}
    robot_description_semantic_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            cobot_srdf_xacro,
            # " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/ResolveConstraintFrames "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {"use_sim_time": use_sim_time }, 
        ],)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            joint_limits_yaml,
        ],)


    launch_sequence = [
       rviz_node,
       move_group_node,
    ]
  
    return launch_sequence


def generate_launch_description():

    # Declare command-line arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulator time flag",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])




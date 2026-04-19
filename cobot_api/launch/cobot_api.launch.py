import os
import yaml, xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import OpaqueFunction, DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction


# ros2 launch cobot_api cobot_api.launch.py move_group_demo:=x
# [joint, api, cart, job, x]


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def launch_setup(context, *args, **kwargs):

    rviz_config_file = os.path.join(get_package_share_directory("cobot_moveit_config"), "rviz", "moveit.rviz")
    cobot_urdf_xacro = os.path.join(get_package_share_directory("cobot_description"), "urdf", "cobot.xacro")
    cobot_srdf_xacro = os.path.join( get_package_share_directory("cobot_moveit_config"), "srdf", "cobot.srdf" )
    kinematics_yaml = load_yaml("cobot_moveit_config", "config/kinematics.yaml")
    ompl_planning_yaml = load_yaml("cobot_moveit_config", "config/ompl_planning.yaml")
    moveit_simple_controllers_yaml = load_yaml( "cobot_moveit_config", "config/moveit_controllers.yaml" )
    joint_limits_yaml = { "robot_description_planning": load_yaml( "cobot_moveit_config", "config/joint_limits.yaml") }

    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_gazebo_val = context.perform_substitution(sim_gazebo)
    use_sim_time = True if sim_gazebo_val.lower() == "true" else False
    move_group_demo = LaunchConfiguration("move_group_demo")
    move_group_demo = context.perform_substitution(move_group_demo)
    world = LaunchConfiguration("world")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_gazebo_val = context.perform_substitution(sim_gazebo)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_fake_hardware_val = context.perform_substitution(use_fake_hardware)
    use_soem = LaunchConfiguration("use_soem")
    use_soem_val = context.perform_substitution(use_soem)
    enable_interaction = LaunchConfiguration("enable_interaction")
    enable_interaction_val = context.perform_substitution(enable_interaction)
    control_mode = LaunchConfiguration("control_mode")
    control_mode_val = context.perform_substitution(control_mode)

    # Map control_mode to numerical initial_mode_of_operation
    # 8 = Position, 10 = Effort (ZeroErr/CANopen standards)
    operational_mode = "8" if control_mode_val.lower() == "position" else "10"

    job_program = PathJoinSubstitution(
        [
            FindPackageShare("cobot_api"),
            "config",
            "cobot_job_itinerary.yaml",
        ]
    )

    # Process Xacro
    robot_doc = xacro.process_file(
        cobot_urdf_xacro,
        mappings={
            "sim_gazebo": sim_gazebo_val,
            "use_fake_hardware": use_fake_hardware_val,
            "use_soem": use_soem_val,
            "initial_mode_of_operation": operational_mode,
        },
    )
    robot_urdf = robot_doc.toprettyxml(indent="  ")
    robot_description = {"robot_description": robot_urdf}

    # ------------ IGNITION GAZEBO
    # Ignition Gazebo Launch
    
    # Mesh resolution fix
    # Add both the package share and the directory above it to GZ paths
    pkg_share_path = get_package_share_directory("cobot_description")
    parent_share_path = os.path.join(pkg_share_path, '..')
    
    resource_paths = [pkg_share_path, parent_share_path]
    resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[':'.join(resource_paths)]
    )
    ign_resource_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[':'.join(resource_paths)]
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments=[('gz_args', ['-r -v 4 ', world])]
    )

    gazebo_nodes = []
    if sim_gazebo_val.lower() == 'true':
        # Spawn Robot in Ignition with a delay to ensure Gazebo is ready
        robot_spawn_entity = TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'cobot_arm',
                        '-string', robot_urdf,
                        '-world', 'empty',
                        '-x', '-1.5',
                        '-y', '-1.5',
                        '-z', '0.2',
                    ],
                    output='screen',
                )
            ]
        )

        ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/cobot/wrist_ft_raw@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
                '/kinect_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/kinect_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/kinect_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            ],
            remappings=[
                ('/cobot/wrist_ft_raw', '/cobot/wrist_ft'),
            ],
            output='screen',
        )

        gazebo_nodes = [resource_env, ign_resource_env, gazebo_server, robot_spawn_entity, ros_gz_bridge]

    robot_description_semantic_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            cobot_srdf_xacro,
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
        "joint_state_topic": "/joint_states",
    }

    # -----------------------------------------------------------------------------------
    # Start the actual move_group node/action server
    # -----------------------------------------------------------------------------------
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

    # -----------------------------------------------------------------------------------
    # Rviz
    # -----------------------------------------------------------------------------------
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
            {"use_sim_time": use_sim_time},
        ],)

    # -----------------------------------------------------------------------------------
    # Teleop cart and joint jog demo executable
    # -----------------------------------------------------------------------------------
    cobot_group_teleop_api = Node(
        name="cobot_group_teleop_api",
        package="cobot_api",
        executable="cobot_group_teleop_api",
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
            {"use_sim_time": use_sim_time }],
    )

    launch_sequence_ = []
    # -----------------------------------------------------------------------------------
    # MoveGroup joint space goals demo executable
    # -----------------------------------------------------------------------------------
    if move_group_demo == "joint":
        move_group_demo_joint = Node(
            name="move_group_interface_tutorial_joint",
            package="cobot_api",
            executable="move_group_interface_tutorial_joint",
            output="screen",
            parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": use_sim_time}],)
        launch_sequence_.append(move_group_demo_joint)

    # -----------------------------------------------------------------------------------
    # MoveGroup cartesian goals demo executable
    # -----------------------------------------------------------------------------------
    if move_group_demo == "cart":
        move_group_demo_cart = Node(
            name="move_group_interface_tutorial_cart",
            package="cobot_api",
            executable="move_group_interface_tutorial_cart",
            output="screen",
            parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": use_sim_time}],)
        launch_sequence_.append(move_group_demo_cart)

    # -----------------------------------------------------------------------------------
    # MoveGroup single/multi/fkin/invkin/speedchange etc. demo executable
    # -----------------------------------------------------------------------------------
    if move_group_demo == "api":
        move_group_service_node = Node(
            name="move_group_service_node",
            package="cobot_api",
            executable="move_group_service_node",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                {'group_name': 'cobot_arm',
                'joint_state_topic_name': 'joint_states',
                'planning_frame': 'base_link',
                },
                {"use_sim_time": use_sim_time},
            ],)
        launch_sequence_.append(move_group_service_node)

    # -----------------------------------------------------------------------------------
    # if you want to plan a program with the yaml file [ movel | movej | wait | gripper ]
    # -----------------------------------------------------------------------------------
    if move_group_demo == "job":
        move_group_service_node = Node(
            name="move_group_service_node",
            package="cobot_api",
            executable="move_group_service_node",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                {'group_name': 'cobot_arm',
                'joint_state_topic_name': 'joint_states',
                'planning_frame': 'base_link',
                },
            ],)
        cobot_job_client_node = TimerAction(
            period=30.0,
            actions=[
                Node(
                package="cobot_api",
                executable="cobot_job_client.py",
                name="cobot_job_client",
                parameters=[job_program],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                    },
                )
            ]
        )
        launch_sequence_.append(move_group_service_node)
        launch_sequence_.append(cobot_job_client_node)

    virtual_force_estimator = Node(
        package="cobot_force_estimation",
        executable="wrench_estimator_node",
        name="wrench_estimator_node",
        output="screen",
        parameters=[
            {"robot_description": robot_urdf}, 
            {"use_sim_time": use_sim_time}
        ],
    )

    admittance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("cobot_admittance_control"),
                    "launch",
                    "admittance.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("robot_description", robot_urdf),
            ("assist_gain", "2.0"),
        ],
    )

    control_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("cobot_control_mux"),
                    "launch",
                    "control_mux.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("update_rate_hz", "100.0"),
        ],
    )

    gravity_compensation_node = Node(
        package="cobot_gravity_compensation",
        executable="gravity_comp_node",
        name="gravity_comp_node",
        output="screen",
        parameters=[
            {"robot_description": robot_urdf},
            {
                "use_sim_time": use_sim_time,
                "joint_names": ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
                "base_link": "base_link",
                "end_effector": "gripper_tcp",
                "direct_torque_topic": "/gravity/effort",
            }
        ],
    )

    # Modular nodes should only run if interaction is enabled or effort mode is active
    modular_stack = []
    if control_mode_val.lower() == "effort":
        modular_stack.append(control_mux_launch)
        
    if enable_interaction_val.lower() == "true":
        modular_stack.append(admittance_launch)
        # Ensure mux is added if not already there
        if control_mux_launch not in modular_stack:
            modular_stack.append(control_mux_launch)

    nodes_to_start = gazebo_nodes + [
        move_group_node, 
        rviz_node, 
        cobot_group_teleop_api, 
        virtual_force_estimator, 
        gravity_compensation_node
    ] + modular_stack + launch_sequence_

    return nodes_to_start


def generate_launch_description():

    # Declare command-line arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "move_group_demo",
            default_value="api",
            choices=["joint","cart","api","job","x"],
            description="watch go to cartesian or joint space goals.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(get_package_share_directory("cobot_description"),
                                       "worlds",
                                       "empty.world"),
            description="Full path to world model file to load",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="false",
            description="Whether to use Ignition Gazebo simulation",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Whether to use fake hardware",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_soem",
            default_value="false",
            description="Whether to use SOEM hardware",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_interaction",
            default_value="false",
            description="Enable wrist interaction for Free Mode",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "control_mode",
            default_value="position",
            description="Control mode: position or effort",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
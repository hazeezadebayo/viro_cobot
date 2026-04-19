import gym
gym.logger.set_level(40) # hide warnings
import time
import numpy as np
import copy
import math
import os
import psutil
import signal
import sys
from scipy.stats import skew
from gym import utils, spaces
from gym_gazebo2.utils import ut_generic, ut_launch, ut_cobot, ut_math, ut_gazebo, tree_urdf, general_utils
from gym.utils import seeding
from gazebo_msgs.srv import SpawnEntity
import subprocess
import argparse
import transforms3d as tf3d

# ROS 2
import rclpy


from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # Used for publishing joint angles.
from control_msgs.msg import JointTrajectoryControllerState
# from gazebo_msgs.srv import SetEntityState, DeleteEntity
from gazebo_msgs.msg import ContactState, ModelState#, GetModelList
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from ros2pkg.api import get_prefix_path
from builtin_interfaces.msg import Duration

# Algorithm specific
from PyKDL import ChainJntToJacSolver # For KDL Jacobians

import PyKDL
from PyKDL import Chain, Frame, Vector, Rotation
from pykdl_utils import kdl_kinematics
from PyKDL import ChainFkSolverPos_recursive




class COBOTEnv(gym.Env):
    """
    TODO. Define the environment.
    """

    def __init__(self):
        """
        Initialize the environemnt
        """
        # Manage command line args
        args = ut_generic.getArgsParser().parse_args()
        self.gzclient = args.gzclient
        self.velocity = args.velocity
        self.multiInstance = args.multiInstance
        self.port = args.port

        # Set the path of the corresponding URDF file
        urdf = "app/ros2_ws/viro_cobot/src/cobot_gui/ros2_websocket/src/cobot_description/urdf/cobot_arm.urdf"
        urdfPath = urdf # get_prefix_path("cobot_description")

        # Launch cobot in a new Process
        self.launch_subp = ut_launch.startLaunchServiceProcess(
            ut_launch.generateLaunchDescription(
                self.gzclient, self.multiInstance, self.port, urdfPath))

        # Create the node after the new ROS_DOMAIN_ID is set in generate_launch_description()
        rclpy.init()
        self.node = rclpy.create_node(self.__class__.__name__)

        # class variables
        self._observation_msg = None
        self.max_episode_steps = 1024 #default value, can be updated from baselines
        self.iterator = 0
        self.reset_jnts = True
        self._collision_msg = None

        #############################
        #   Environment hyperparams
        #############################
        # Target, where should the agent reach
        self.targetPosition = np.asarray([-0.40028, 0.095615, 0.25]) # close to the table
        # self.targetPosition =  np.asarray([round(np.random.uniform(-0.615082, -0.35426), 5), round(np.random.uniform( -0.18471, 0.1475), 5), 0.35])
        self.target_orientation = np.asarray([0., 0.7071068, 0.7071068, 0.]) # arrow looking down [w, x, y, z]
        # self.targetPosition = np.asarray([-0.386752, -0.000756, 1.40557]) # easy point
        # self.target_orientation = np.asarray([-0.4958324, 0.5041332, 0.5041331, -0.4958324]) # arrow looking opposite to cobot [w, x, y, z]

        EE_POINTS = np.asmatrix([[0, 0, 0]])
        EE_VELOCITIES = np.asmatrix([[0, 0, 0]])

        # Initial joint position
        INITIAL_JOINTS = np.array([0., 0., 0., 0., 0., 0.])

        # # Topics for the robot publisher and subscriber.
        controller_name = "joint_trajectory_controller"
        JOINT_PUBLISHER = "/" + controller_name + "/" + "joint_trajectory" # '/x_controller/command'
        JOINT_SUBSCRIBER = "/" + controller_name + "/" + "state" # '/x_controller/state'

        # joint names:
        JOINT1 = 'joint_1'
        JOINT2 = 'joint_2'
        JOINT3 = 'joint_3'
        JOINT4 = 'joint_4'
        JOINT5 = 'joint_5'
        JOINT6 = 'joint_6'
        EE_J = 'eef_tcp'

        # Set constants for links
        WORLD = 'world'
        BASE = 'base_link'
        LINK1 = 'link_1'
        LINK2 = 'link_2'
        LINK3 = 'link_3'
        LINK4 = 'link_4'
        LINK5 = 'link_5'
        LINK6 = 'link_6'
        EE_LINK = 'eef_tcp_link'

        JOINT_ORDER = [JOINT1, JOINT2, JOINT3,
                        JOINT4, JOINT5, JOINT6]

        LINK_NAMES = [ WORLD, BASE,
                        LINK1, LINK2,
                        LINK3, LINK4,
                        LINK5, LINK6, EE_LINK]

        reset_condition = {
            'initial_positions': INITIAL_JOINTS,
             'initial_velocities': []
        }
        #############################

        m_jointOrder = copy.deepcopy(JOINT_ORDER)
        m_linkNames = copy.deepcopy(LINK_NAMES)

        # Initialize target end effector position
        self.environment = {
            'jointOrder': m_jointOrder,
            'linkNames': m_linkNames,
            'reset_conditions': reset_condition,
            'tree_path': urdfPath,
            'end_effector_points': EE_POINTS,
        }

        # Subscribe to the appropriate topics, taking into account the particular robot
        self._pub = self.node.create_publisher(JointTrajectory, JOINT_PUBLISHER, qos_profile=qos_profile_sensor_data)
        self._sub = self.node.create_subscription(JointTrajectoryControllerState, JOINT_SUBSCRIBER, self.observation_callback, qos_profile=qos_profile_sensor_data)
        # self._sub_coll = self.node.create_subscription(ContactState, '/gazebo_contacts', self.collision_callback, qos_profile=qos_profile_sensor_data)
        self.reset_sim = self.node.create_client(Empty, '/reset_simulation')

        # Initialize a tree structure from the robot urdf.
        #   note that the xacro of the urdf is updated by hand.
        # The urdf must be compiled.
        _, self.ur_tree = tree_urdf.treeFromFile(self.environment['tree_path'])
        # Retrieve a chain structure between the base and the start of the end effector.
        self.cobot_chain = self.ur_tree.getChain(self.environment['linkNames'][0], self.environment['linkNames'][-1])
        self.numJoints = self.cobot_chain.getNrOfJoints()
        # Initialize a KDL Jacobian solver from the chain.
        self.jacSolver = ChainJntToJacSolver(self.cobot_chain)

        # --------------------
        # Initialize KDL chain
        self.jnt_pos = PyKDL.JntArray(self.numJoints)
        # Create a forward kinematics solver
        self.fk_solver = ChainFkSolverPos_recursive(self.cobot_chain)
        # Define bounding volumes for links
        self.link_bounding_volumes = {
            LINK1: {'type': 'cylinder', 'radius': 0.1, 'length': 0.2},
            LINK2: {'type': 'cylinder', 'radius': 0.1, 'length': 0.7},
            LINK3: {'type': 'cylinder', 'radius': 0.1, 'length': 0.7},
            LINK4: {'type': 'cylinder', 'radius': 0.1, 'length': 0.1},
            LINK5: {'type': 'cylinder', 'radius': 0.1, 'length': 0.1},
            LINK6: {'type': 'cylinder', 'radius': 0.1, 'length': 0.05},
        }
        # --------------------

        self.obs_dim = self.numJoints + 6 + 12

        # # Here idially we should find the control range of the robot. Unfortunatelly in ROS/KDL there is nothing like this.
        # # I have tested this with the mujoco enviroment and the output is always same low[-1.,-1.], high[1.,1.]

        low = -np.pi * np.ones(self.numJoints)
        high = np.pi * np.ones(self.numJoints)

        self.action_space = spaces.Box(low, high)

        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)

        # --- simulation associated:
        spawn_cli = self.node.create_client(SpawnEntity, '/spawn_entity')

        while not spawn_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('/spawn_entity service not available, waiting again...')

        modelXml = ut_gazebo.getTargetSdf()
        pose = Pose()
        pose.position.x = self.targetPosition[0]
        pose.position.y = self.targetPosition[1]
        pose.position.z = self.targetPosition[2]
        pose.orientation.x = self.target_orientation[1]
        pose.orientation.y= self.target_orientation[2]
        pose.orientation.z = self.target_orientation[3]
        pose.orientation.w = self.target_orientation[0]

        #override previous spawn_request element.
        self.spawn_request = SpawnEntity.Request()
        self.spawn_request.name = "target"
        self.spawn_request.xml = modelXml
        self.spawn_request.robot_namespace = ""
        self.spawn_request.initial_pose = pose
        self.spawn_request.reference_frame = "world"

        # #ROS2 Spawn Entity
        target_future = spawn_cli.call_async(self.spawn_request)
        rclpy.spin_until_future_complete(self.node, target_future)

        # Seed the environment
        self.seed()
        self.buffer_dist_rewards = []
        self.buffer_tot_rewards = []
        self.collided = 0



    def observation_callback(self, message):
        """
        Callback method for the subscriber of JointTrajectoryControllerState
        """
        self._observation_msg = message

    # old logic
    # def collision_callback(self, message):
    #     """
    #     Callback method for the subscriber of Collision data
    #     """
    #     collision_messages = ["cobot::base_robot::base_robot_collision", "ground_plane::link::collision"]
    #     if message.collision1_name != message.collision2_name:
    #         if not ((message.collision1_name in collision_messages) and (message.collision2_name in collision_messages)):
    #             self._collision_msg = message


    # new collision logic starts here ---------------------------------------

    def get_link_transforms(self, joint_angles):
        """ A method computes the transformation of each link in the
        robot's kinematic chain based on the given joint angles """
        transforms = []
        jnt_array = PyKDL.JntArray(len(joint_angles))
        for i, angle in enumerate(joint_angles):
            jnt_array[i] = angle

        # Compute transforms for each segment
        frame = PyKDL.Frame()
        for i in range(self.cobot_chain.getNrOfJoints()):
            segment = self.cobot_chain.getSegment(i)
            segment_frame = segment.getFrameToEnd()
            # Forward kinematics computation
            self.fk_solver.JntToCart(jnt_array, frame)
            transforms.append(frame * segment_frame)

        return transforms


    def check_collision(self, joint_angles):
        """ A method checks for collisions between pairs of links by transforming
        the links' bounding volumes to the global frame """
        transforms = self.get_link_transforms(joint_angles)

        # Check for collisions between each pair of links
        for i, (link1, trans1) in enumerate(zip(self.environment['linkNames'], transforms)):
            for j, (link2, trans2) in enumerate(zip(self.environment['linkNames'], transforms)):
                if i >= j:
                    continue
                if self.detect_collision(link1, trans1, link2, trans2):
                    return True
        return False


    def detect_collision(self, link1, trans1, link2, trans2):
        """this example uses axis-aligned bounding boxes (AABBs)."""
        vol1 = self.link_bounding_volumes.get(link1, None)
        vol2 = self.link_bounding_volumes.get(link2, None)

        if vol1 is None or vol2 is None:
            return False

        # Collision detection logic (e.g., AABB)
        def check_aabb_collision(vol1, vol2, trans1, trans2):
            size1 = vol1['size']
            size2 = vol2['size']
            center1 = np.array([trans1.p.x(), trans1.p.y(), trans1.p.z()])
            center2 = np.array([trans2.p.x(), trans2.p.y(), trans2.p.z()])

            half_size1 = np.array(size1) / 2
            half_size2 = np.array(size2) / 2

            collision_x = abs(center1[0] - center2[0]) < (half_size1[0] + half_size2[0])
            collision_y = abs(center1[1] - center2[1]) < (half_size1[1] + half_size2[1])
            collision_z = abs(center1[2] - center2[2]) < (half_size1[2] + half_size2[2])

            return collision_x and collision_y and collision_z

        if vol1['type'] == 'box' and vol2['type'] == 'box':
            return check_aabb_collision(vol1, vol2, trans1, trans2)

        return False

    def collision(self, joint_angles):
        """
        Check for self-collisions and Reset if there is a collision
        """
        if self.check_collision(joint_angles):
            while not self.reset_sim.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/reset_simulation service not available, waiting again...')

            reset_future = self.reset_sim.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, reset_future)
            self.collided += 1
            return True
        else:
            return False

    # collision code ends here...
    # ---------------------------------------
    # ---------------------------------------

    def set_episode_size(self, episode_size):
        self.max_episode_steps = episode_size

    def take_observation(self):
        """
        Take observation from the environment and return it.
        :return: state.
        """
        # # # # Take an observation
        rclpy.spin_once(self.node)
        obs_message = self._observation_msg

        # Check that the observation is not prior to the action
        # obs_message = self._observation_msg
        while obs_message is None or int(str(self._observation_msg.header.stamp.sec)+(str(self._observation_msg.header.stamp.nanosec))) < self.ros_clock:
            # print("I am in obs_message is none")
            rclpy.spin_once(self.node)
            obs_message = self._observation_msg

        # Collect the end effector points and velocities in cartesian coordinates for the processObservations state.
        # Collect the present joint angles and velocities from ROS for the state.
        lastObservations = ut_cobot.processObservations(obs_message, self.environment)
        #Set observation to None after it has been read.
        self._observation_msg = None

        # Get Jacobians from present joint angles and KDL trees
        # The Jacobians consist of a 6x6 matrix getting its from from
        # (joint angles) x (len[x, y, z] + len[roll, pitch, yaw])
        ee_link_jacobians = ut_cobot.getJacobians(lastObservations, self.numJoints, self.jacSolver)
        if self.environment['linkNames'][-1] is None:
            print("End link is empty!!")
            return None
        else:
            translation, rot = general_utils.forwardKinematics(self.cobot_chain,
                                                self.environment['linkNames'],
                                                lastObservations[:self.numJoints],
                                                baseLink=self.environment['linkNames'][0], # use the base_robot coordinate system
                                                endLink=self.environment['linkNames'][-1])

            current_eePos_tgt = np.ndarray.flatten(general_utils.getEePoints(self.environment['end_effector_points'], translation, rot).T)
            eePos_points = current_eePos_tgt - self.targetPosition

            eeVelocities = ut_cobot.getEePointsVelocities(ee_link_jacobians, self.environment['end_effector_points'], rot, lastObservations)

            # Concatenate the information that defines the robot state
            # vector, typically denoted asrobot_id 'x'.
            state = np.r_[np.reshape(lastObservations, -1),
                          np.reshape(eePos_points, -1),
                          np.reshape(eeVelocities, -1),
                          np.reshape(current_eePos_tgt,-1),
                          np.reshape(rot.reshape(1, 9),-1)]

            return state

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """
        Implement the environment step abstraction. Execute action and returns:
            - action
            - observation
            - reward
            - done (status)
        """
        self.iterator+=1
        # Execute "action"
        self._pub.publish(ut_cobot.getTrajectoryMessage(
            action[:self.numJoints],
            self.environment['jointOrder'],
            self.velocity))

        self.ros_clock = rclpy.clock.Clock().now().nanoseconds

        # Take an observation
        obs = self.take_observation()

        # Fetch the positions of the end-effector which are nr_dof:nr_dof+3
        rewardDist = ut_math.rmseFunc(obs[self.numJoints:(self.numJoints+3)])

        collided = self.collision(obs[:self.numJoints])

        reward = ut_math.computeReward(rewardDist)

        # Calculate if the env has been solved
        done = bool(self.iterator == self.max_episode_steps)

        self.buffer_dist_rewards.append(rewardDist)
        self.buffer_tot_rewards.append(reward)
        info = {}
        if self.iterator % self.max_episode_steps == 0:

            max_dist_tgt = max(self.buffer_dist_rewards)
            mean_dist_tgt = np.mean(self.buffer_dist_rewards)
            std_dist_tgt = np.std(self.buffer_dist_rewards)
            min_dist_tgt = min(self.buffer_dist_rewards)
            skew_dist_tgt = skew(self.buffer_dist_rewards)

            max_tot_rew = max(self.buffer_tot_rewards)
            mean_tot_rew = np.mean(self.buffer_tot_rewards)
            std_tot_rew = np.std(self.buffer_tot_rewards)
            min_tot_rew = min(self.buffer_tot_rewards)
            skew_tot_rew = skew(self.buffer_tot_rewards)

            num_coll = self.collided

            info = {"infos":{"ep_dist_max": max_dist_tgt,"ep_dist_mean": mean_dist_tgt,"ep_dist_min": min_dist_tgt,\
                "ep_rew_max": max_tot_rew,"ep_rew_mean": mean_tot_rew,"ep_rew_min": min_tot_rew,"num_coll": num_coll,\
                "ep_dist_skew": skew_dist_tgt,"ep_dist_std": std_dist_tgt, "ep_rew_std": std_tot_rew, "ep_rew_skew":skew_tot_rew}}
            self.buffer_dist_rewards = []
            self.buffer_tot_rewards = []
            self.collided = 0

        # Return the corresponding observations, rewards, etc.
        return obs, reward, done, info

    def reset(self):
        """
        Reset the agent for a particular experiment condition.
        """
        self.iterator = 0

        if self.reset_jnts is True:
            # reset simulation
            while not self.reset_sim.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('/reset_simulation service not available, waiting again...')

            reset_future = self.reset_sim.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, reset_future)

        self.ros_clock = rclpy.clock.Clock().now().nanoseconds

        # Take an observation
        obs = self.take_observation()

        # Return the corresponding observation
        return obs

    def close(self):
        print("Closing " + self.__class__.__name__ + " environment.")
        self.node.destroy_node()
        parent = psutil.Process(self.launch_subp.pid)
        for child in parent.children(recursive=True):
            child.kill()
        rclpy.shutdown()
        parent.kill()












    # 'link_1': {'type': 'box', 'size': (0.1, 0.1, 0.5)},
    # def get_link_frame(self, joint_angles):
    #     """
    #     Get the transformation frames for each link given the joint angles.
    #     """
    #     frames = {}
    #     kdl_jnt_array = kdl.JntArray(self.numJoints)
    #     for i, angle in enumerate(joint_angles):
    #         kdl_jnt_array[i] = angle
    #     for i in range(self.numJoints + 1):
    #         frame = kdl.Frame()
    #         self.fk_solver.JntToCart(kdl_jnt_array, frame)
    #         link_name = self.environment['linkNames'][i]
    #         frames[link_name] = frame
    #         if i < self.numJoints:
    #             self.fk_solver.JntToCart(kdl_jnt_array, frame)
    #             self.fk_solver.JntToCart(kdl_jnt_array, frame)
    #     return frames

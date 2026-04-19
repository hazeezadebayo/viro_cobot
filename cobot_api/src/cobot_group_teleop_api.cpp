#include "cobot_api/cobot_group_teleop_api.h"

// Terminal 1
// ros2 launch cobot_api cobot_api.launch.py move_group_demo:=x

// Terminal 2
// # ros2 service call /cobot_api/joint_teleop cobot_msgs/srv/CtrlParam "{command: 1}"
// # ros2 service call /cobot_api/cart_teleop cobot_msgs/srv/CtrlParam "{command: -1}"





MoveGroupTeleopAPI::MoveGroupTeleopAPI(rclcpp::Node::SharedPtr node)
     : teleop_nh_(node)
{
    std::cerr << "debug: starting teleop api." << std::endl;

    // Create a reentrant callback group to allow concurrent execution of callbacks
    callback_group_ = teleop_nh_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    PLANNING_GROUP = teleop_nh_->declare_parameter<std::string>("group_name", "cobot_arm");
    action_name = teleop_nh_->declare_parameter<std::string>("follow_joint_trajectory_name", "joint_trajectory_controller/follow_joint_trajectory");

    // Initialize the move group interface
    group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(teleop_nh_, PLANNING_GROUP);

    // Initialize the planning scene monitor
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(teleop_nh_, "robot_description");
    // Start the state monitor
    planning_scene_monitor_->startStateMonitor("/joint_states");
    // Start the world geometry monitor
    planning_scene_monitor_->startWorldGeometryMonitor();
    // Set the planning scene publishing frequency
    planning_scene_monitor_->setPlanningScenePublishingFrequency(25.0);
    // Start publishing the planning scene
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
    // Start the scene monitor
    planning_scene_monitor_->startSceneMonitor();

    action_client_=rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(teleop_nh_, action_name);

    // std::cerr << "debug: 00.." << std::endl;
    goal_.trajectory.joint_names = group_->getJointNames();
    goal_.trajectory.header.stamp.sec = 0;
    goal_.trajectory.header.stamp.nanosec = 0;

    joint_teleop_server_=teleop_nh_->create_service<cobot_msgs::srv::CtrlParam>("/cobot_api/joint_teleop",
        std::bind(&MoveGroupTeleopAPI::jointTeleop_cb,this,std::placeholders::_1,std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_);
    
    cart_teleop_server_=teleop_nh_->create_service<cobot_msgs::srv::CtrlParam>("/cobot_api/cart_teleop", 
        std::bind(&MoveGroupTeleopAPI::cartTeleop_cb,this,std::placeholders::_1,std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_);
        
    vary_speed_teleop_service_=teleop_nh_->create_service<cobot_msgs::srv::TaskParams>("/cobot_api/vary_speed_teleop", 
        std::bind(&MoveGroupTeleopAPI::handleVarySpeed,this,std::placeholders::_1,std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    js_sub_ = teleop_nh_->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&MoveGroupTeleopAPI::js_callback, this, std::placeholders::_1), sub_options);

    // Initialize current_js with 6 elements
    current_js = std::vector<double>(6, 0.0);

    // parameter for normal teleop
    joint_speed_limit_ = JOINT_SPEED_LIMIT_CONST;
    joint_speed_ = JOINT_SPEED_DEFAULT_CONST;
    cart_duration_ = CART_DURATION_DEFAULT_CONST;
    group_->setMaxVelocityScalingFactor(0.5);

    resolution_angle_ = 0.02;
    resolution_linear_ = 0.002; // 0.005;
    tfBuffer = std::make_unique<tf2_ros::Buffer>(teleop_nh_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    end_link_=group_->getEndEffectorLink();
    reference_link_=group_->getPlanningFrame();

    default_tip_link_=group_->getEndEffectorLink();
    root_link_=group_->getPlanningFrame();

    robot_in_motion = false;
    // Create a timer that triggers every 5 seconds
    timer_ = teleop_nh_->create_wall_timer(std::chrono::seconds(1), 
        std::bind(&MoveGroupTeleopAPI::timerCallback, this),
        callback_group_);

    std::cerr << "debug: teleop api started successfully..." << std::endl;
}

void MoveGroupTeleopAPI::js_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Create a map of joint names and positions
    std::map<std::string, double> joint_map;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        joint_map[msg->name[i]] = msg->position[i];
    }

    // Define the expected order
    // std::vector<std::string> expected_order = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    // Get the expected order from the MoveGroup
    std::vector<std::string> expected_order = group_->getJointNames();

    // Create a new joint state vector in the expected order
    current_js.clear();
    for (const auto& joint_name : expected_order) {
        // Now current_js is in the order defined in the MoveGroup
        current_js.push_back(joint_map[joint_name]);
    }
}

void MoveGroupTeleopAPI::timerCallback() {
    if (robot_in_motion) {
        // Send the clear trajectory goal
        // std::cerr << "debug: 0x0..." << std::endl;

        trajectory_msgs::msg::JointTrajectoryPoint point_tmp;
        std::vector<double> position_tmp = current_js; // group_->getCurrentJointValues();

        point_tmp.positions = position_tmp;

        rclcpp::Duration duration = rclcpp::Duration::from_seconds(0.1);
        point_tmp.time_from_start.sec = duration.seconds();
        point_tmp.time_from_start.nanosec = duration.nanoseconds() % 1000000000;

        goal_.trajectory.points.push_back(point_tmp);
        action_client_->async_send_goal(goal_);
        goal_.trajectory.points.clear();

        robot_in_motion = false;
    }
}

bool MoveGroupTeleopAPI::handleVarySpeed(
    const std::shared_ptr<cobot_msgs::srv::TaskParams::Request> req,
    std::shared_ptr<cobot_msgs::srv::TaskParams::Response> resp)
{
    resp->status = 1;
    group_->setMaxVelocityScalingFactor(req->cmd1);
    group_->setMaxAccelerationScalingFactor(req->cmd1); 
    // std::cerr << "debug: req->cmd1: " << req->cmd1 << std::endl;
    joint_speed_ = req->cmd1 * JOINT_SPEED_LIMIT_CONST;
    cart_duration_ = req->cmd1 * CART_DURATION_DEFAULT_CONST;
    return true;
}

bool MoveGroupTeleopAPI::jointTeleop_cb(
    const std::shared_ptr<cobot_msgs::srv::CtrlParam::Request> req,
    const std::shared_ptr<cobot_msgs::srv::CtrlParam::Response> resp)
{
    // std::cerr << "debug: 1" << std::endl;

    // if(req->command==0 || abs(req->command)>goal_.trajectory.joint_names.size())
    if(req->command==0 || abs(req->command) > static_cast<int>(goal_.trajectory.joint_names.size()))
    {
        // std::cerr << "debug: 2" << std::endl;
        resp->status = 0;
        // resp->success=false;
        // resp->message="wrong joint teleop req->command";
        return true;
    }

    // std::cerr << "debug: 3" << std::endl;
    int joint_num=abs(req->command);
    std::vector<double> position_current = current_js; // group_->getCurrentJointValues();
    std::vector<double> position_goal=position_current;
    double joint_current_position=position_current[joint_num-1];
    std::string direction=goal_.trajectory.joint_names[joint_num-1];
    double sign;

    // std::cerr << "debug: 4" << std::endl;
    if(joint_num==req->command)
    {
        // std::cerr << "debug: 5" << std::endl;
        position_goal[joint_num-1]=group_->getRobotModel()->getURDF()->getJoint(goal_.trajectory.joint_names[joint_num-1])->limits->upper;
        direction.append("+");
        sign=1;
    }
    else
    {
        // std::cerr << "debug: 6" << std::endl;
        position_goal[joint_num-1]=group_->getRobotModel()->getURDF()->getJoint(goal_.trajectory.joint_names[joint_num-1])->limits->lower;
        direction.append("-");
        sign=-1;
    }

    // std::cerr << "debug: 7" << std::endl;
    double duration_from_speed=fabs(position_goal[joint_num-1]-joint_current_position)/joint_speed_;
    if(duration_from_speed<=0.1)
    {
        // std::cerr << "debug: 8" << std::endl;
        resp->status = 0;
        // resp->success=false;
        std::string result="robot can't move in ";
        result.append(direction);
        result.append(" direction any more");
        // resp->message=result;
        return true;
    }

    // std::cerr << "debug: 9" << std::endl;
    trajectory_msgs::msg::JointTrajectoryPoint point_tmp;

    // std::cerr << "debug: 10" << std::endl;
    moveit::core::RobotStatePtr kinematic_state_ptr = group_->getCurrentState();
    if (!kinematic_state_ptr) {
        std::cerr << "debug: getCurrentState() returned null, not ready yet." << std::endl;
        resp->status = 0;
        return true;
    }
    moveit::core::RobotState kinematic_state = *kinematic_state_ptr;
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());

    // std::cerr << "debug: 11" << std::endl;
    planning_scene_monitor_->updateFrameTransforms();
    planning_scene::PlanningSceneConstPtr plan_scene = planning_scene_monitor_->getPlanningScene();

    // std::cerr << "debug: 12" << std::endl;
    std::vector<double> position_tmp = position_current;
    bool collision_flag=false;

    // std::cerr << "debug: 13" << std::endl;
    int loop_num=1;
    while(fabs(position_goal[joint_num-1] - position_tmp[joint_num-1]) / joint_speed_>0.1)
    {
        // std::cerr << "debug: 14" << std::endl;
        position_tmp[joint_num-1] += joint_speed_ * 0.1 * sign;
        ///////////
        // kinematic_state.setJointGroupPositions(joint_model_group, position_tmp);
        // if (plan_scene->isStateColliding(kinematic_state, group_->getName()))
        // {
        //     if (loop_num == 1)
        //     {
        //         // resp.success = false;
        //         resp->status = 0;
        //         std::string result = "robot can't move in ";
        //         result.append(direction);
        //         result.append(" direction any more");
        //         // resp.message = result;
        //         return true;
        //     }
        //     collision_flag = true;
        //     break;
        // }
        ///////////
        point_tmp.time_from_start=rclcpp::Duration::from_seconds(0.1*loop_num);
        point_tmp.positions=position_tmp;
        goal_.trajectory.points.push_back(point_tmp);
        loop_num++;
    }

    if(!collision_flag)
    {
        // std::cerr << "debug: 15" << std::endl;
        kinematic_state.setJointGroupPositions(joint_model_group, position_goal);
        // if (!plan_scene->isStateColliding(kinematic_state, group_->getName()))
        // {
            point_tmp.positions=position_goal;
            point_tmp.time_from_start=rclcpp::Duration::from_seconds(duration_from_speed);
            goal_.trajectory.points.push_back(point_tmp);
        // }
    }

    // std::cerr << "debug: 16" << std::endl;
    robot_in_motion = true;

    action_client_->async_send_goal(goal_);
    goal_.trajectory.points.clear();

    // resp->success=true;
    resp->status = 1;
    std::string result="robot is moving in ";
    result.append(direction);
    result.append(" direction");
    // std::cerr << "debug: 17" << std::endl;
    // resp->message=result;

    return true;
}

bool MoveGroupTeleopAPI::cartTeleop_cb(
    const std::shared_ptr<cobot_msgs::srv::CtrlParam::Request> req,
    const std::shared_ptr<cobot_msgs::srv::CtrlParam::Response> resp)
{
    // std::cerr << "debug: c1" << std::endl;
    if(req->command==0 || abs(req->command)>6)
    {
        // std::cerr << "debug: c2" << std::endl;
        resp->status = 0;
        // resp->success=false;
        // resp->message="wrong cart. teleop command";
        return true;
    }

    // std::cerr << "debug: c3" << std::endl;
    int operation_num = abs(req->command);
    int symbol;
    if(operation_num == req->command)
        symbol = 1;
    else
        symbol = -1;

    // std::cerr << "debug: c4" << std::endl;
    geometry_msgs::msg::PoseStamped current_pose = group_->getCurrentPose(end_link_);
    rclcpp::Rate r(100);

    // std::cerr << "debug: c5" << std::endl;
    int counter = 0;
    while(rclcpp::ok())
    {
        // std::cerr << "debug: c6" << std::endl;
        try{
            // std::cerr << "debug: c7" << std::endl;
            rclcpp::Duration duration(std::chrono::seconds(10));
            tfBuffer -> canTransform(reference_link_, root_link_, rclcpp::Time(0), duration);
            transform_rootToRef_ = tfBuffer->lookupTransform(reference_link_, root_link_, tf2::TimePointZero);
            break;
        }
        catch (tf2::TransformException &ex) {
          // std::cerr << "debug: c8" << std::endl;
          r.sleep();
          counter++;
          if(counter>200)
          {
            // std::cerr << "debug: c9" << std::endl;
            RCLCPP_ERROR(teleop_nh_->get_logger(),"%s",ex.what());
            resp->status = 0;
            // resp->success=false;
            // resp->message="can't get pose of reference frame";
            return true;
          }
          continue;
        }
    }
    counter=0;
    while(rclcpp::ok())
    {
        try{
            // std::cerr << "debug: c10" << std::endl;
            // tfBuffer -> canTransform(end_link_, default_tip_link_, rclcpp::Time(0), rclcpp::Duration(10.0));
            rclcpp::Duration duration(std::chrono::seconds(10));
            tfBuffer->canTransform(end_link_, default_tip_link_, rclcpp::Time(0), duration);
            transform_tipToEnd_ = tfBuffer ->lookupTransform(end_link_, default_tip_link_, rclcpp::Time(0));
            break;
        }
        catch (tf2::TransformException &ex) {
            // std::cerr << "debug: c11" << std::endl;
            r.sleep();
            counter++;
            if(counter>200)
            {
                // std::cerr << "debug: c12" << std::endl;
                RCLCPP_ERROR(teleop_nh_->get_logger(),"%s",ex.what());
                resp->status = 0;
                // resp->success=false;
                // resp->message="can't get pose of teleop frame";
                return true;
            }
            continue;
        }
    }

    // std::cerr << "debug: c13" << std::endl;
    Eigen::Isometry3d affine_rootToRef, affine_refToRoot;
    affine_rootToRef = tf2::transformToEigen(transform_rootToRef_);
    affine_refToRoot=affine_rootToRef.inverse();

    Eigen::Isometry3d affine_tipToEnd;
    affine_tipToEnd = tf2::transformToEigen(transform_tipToEnd_);

    geometry_msgs::msg::TransformStamped tf_pose_tmp;
    Eigen::Isometry3d affine_pose_tmp;
    tf2::fromMsg(current_pose.pose, affine_pose_tmp);

    Eigen::Isometry3d affine_current_pose=affine_rootToRef * affine_pose_tmp;
    current_pose.pose = tf2::toMsg(affine_current_pose);

    std::vector<double> current_joint_states = current_js; // group_->getCurrentJointValues();

    tf2::Vector3 x_axis(1, 0, 0);
    tf2::Vector3 y_axis(0, 1, 0);
    tf2::Vector3 z_axis(0, 0, 1);
    double resolution_alpha=resolution_angle_;
    double resolution_delta=resolution_linear_;

    moveit::core::RobotStatePtr kinematic_state_ptr = group_->getCurrentState();
    if (!kinematic_state_ptr) {
        std::cerr << "debug: getCurrentState() returned null, not ready yet." << std::endl;
        resp->status = 0;
        return true;
    }
    moveit::core::RobotState kinematic_state=*kinematic_state_ptr;
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());

    planning_scene_monitor_->updateFrameTransforms();
    planning_scene::PlanningSceneConstPtr plan_scene = planning_scene_monitor_->getPlanningScene();

    trajectory_msgs::msg::JointTrajectoryPoint point_tmp;

    std::string direction;

    bool ik_have_result=true;

    for(size_t i=0; i<100; i++)
    {
        // std::cerr << "debug: c14 " << operation_num << std::endl;
        switch (operation_num) {
        case 1:
            current_pose.pose.position.x+=symbol*resolution_delta;
            if(symbol==1)
                direction="X+";
            else
                direction="X-";
            break;
        case 2:
            current_pose.pose.position.y+=symbol*resolution_delta;
            if(symbol==1)
                direction="Y+";
            else
                direction="Y-";
            break;
        case 3:
            current_pose.pose.position.z+=symbol*resolution_delta;
            if(symbol==1)
                direction="Z+";
            else
                direction="Z-";
            break;
        case 4:
            PoseStampedRotation(current_pose, x_axis, symbol*resolution_alpha);
            if(symbol==1)
                direction="Rx+";
            else
                direction="Rx-";
            break;
        case 5:
            PoseStampedRotation(current_pose, y_axis, symbol*resolution_alpha);
            if(symbol==1)
                direction="Ry+";
            else
                direction="Ry-";
            break;
        case 6:
            PoseStampedRotation(current_pose, z_axis, symbol*resolution_alpha);
            if(symbol==1)
                direction="Rz+";
            else
                direction="Rz-";
            break;
        default:
            break;
        }

        // std::cerr << "debug: c16" << std::endl;
        tf2::fromMsg(current_pose.pose, affine_pose_tmp);
        // std::cerr << "debug: c16.1" << std::endl;
        affine_current_pose = affine_refToRoot * affine_pose_tmp * affine_tipToEnd;
        // std::cerr << "debug: c16.2" << std::endl;
        // ik_have_result = kinematic_state.setFromIK(joint_model_group, affine_current_pose, default_tip_link_);
        ik_have_result = kinematic_state.setFromIK(joint_model_group, affine_current_pose, 0.1);
        // std::cerr << "debug: c16.3 " << ik_have_result << std::endl;
        if(ik_have_result)
        {
            // std::cerr << "debug: c17" << std::endl;
            if(goal_.trajectory.points.size() != i)
                break;

            point_tmp.positions.resize(goal_.trajectory.joint_names.size());
            double biggest_shift=0;
            for(unsigned int j=0; j<goal_.trajectory.joint_names.size(); j++)
            {
                // std::cerr << "debug: c18" << std::endl;
                point_tmp.positions[j]=*kinematic_state.getJointPositions(goal_.trajectory.joint_names[j]);
                if(i==0)
                {
                    // std::cerr << "debug: c19" << std::endl;
                    double shift_tmp=fabs(current_joint_states[j]-point_tmp.positions[j]);
                    if(shift_tmp>biggest_shift)
                        biggest_shift=shift_tmp;
                }
                else {
                    // std::cerr << "debug: c20" << std::endl;
                    double shift_tmp=fabs(goal_.trajectory.points[i-1].positions[j]-point_tmp.positions[j]);
                    if(shift_tmp>biggest_shift)
                        biggest_shift=shift_tmp;
                }
            }
            try{
                // std::cerr << "debug: c21" << std::endl;
                if(biggest_shift>cart_duration_*joint_speed_limit_ )
                    break;
            }
            catch(...)
            {
                // std::cerr << "debug: c22" << std::endl;
                break;
            }
            // std::cerr << "debug: c23" << std::endl;
            point_tmp.time_from_start=rclcpp::Duration::from_seconds((i+1)*cart_duration_);
            goal_.trajectory.points.push_back(point_tmp);
            // std::cerr << "debug: c24" << std::endl;
        }
        else
        {
            break;
        }
    }
    if(goal_.trajectory.points.size() == 0)
    {
        resp->status = 0;
        // resp->success=false;
        std::string result="robot can't move in ";
        result.append(direction);
        result.append(" direction any more");
        // resp->message=result;
        // std::cerr << "debug: c25 " << result << std::endl;
        return true;
    }

    robot_in_motion = true;

    // std::cerr << "debug: c26" << std::endl;
    action_client_->async_send_goal(goal_);
    goal_.trajectory.points.clear();

    // resp->success=true;
    resp->status = 1;
    std::string result="robot is moving in ";
    result.append(direction);
    result.append(" direction");
    // resp->message=result;
    return true;

}

void MoveGroupTeleopAPI::PoseStampedRotation(geometry_msgs::msg::PoseStamped &pose_stamped, const tf2::Vector3 &axis, double angle)
{
    tf2::Quaternion q_1(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                                 pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w);
    tf2::Quaternion q_2(axis, angle);
    tf2::Matrix3x3 m(q_1);
    tf2::Matrix3x3 m_2(q_2);
    m_2.operator *=(m);
    double r, p, y;
    m_2.getRPY(r,p,y);

    q_2.setRPY(r, p, y);
    pose_stamped.pose.orientation.x=q_2.getX();
    pose_stamped.pose.orientation.y=q_2.getY();
    pose_stamped.pose.orientation.z=q_2.getZ();
    pose_stamped.pose.orientation.w=q_2.getW();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create a node for the move group
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto cobot_group_nh = rclcpp::Node::make_shared("cobot_group_teleop_api", node_options);

    // Create an instance of the MoveGroupTeleopAPI class
    MoveGroupTeleopAPI api(cobot_group_nh);

    // Use a MultiThreadedExecutor to allow concurrent callback execution
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(cobot_group_nh);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}

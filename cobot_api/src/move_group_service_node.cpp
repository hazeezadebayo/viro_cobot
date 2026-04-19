#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/dynamics_solver/dynamics_solver.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <chrono>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp> 
#include <sensor_msgs/msg/joint_state.hpp>

// Include the generated service message header
// TaskParams task_params | // CtrlParam ctrl_param | // TaskParamsinv task_paramsinv
#include "cobot_msgs/srv/task_params.hpp"
#include "cobot_msgs/srv/ctrl_param.hpp"      
#include <cobot_msgs/srv/task_paramsinv.hpp>

#include <iostream>
#include <thread>
#include <chrono>



// ros2 launch cobot_api cobot_api.launch.py move_group_demo:=api

// ros2 run cobot_api move_group_service_node
// ros2 service call /cobot_api/go_to_joint_goal cobot_msgs/srv/TaskParams "{cmd1: 2.0,cmd2: 0.5,cmd3: 1.2,cmd4: 0.0,cmd5: 0.0,cmd6: 0.0}"
// ros2 service call /cobot_api/go_to_cart_goal cobot_msgs/srv/TaskParams "{cmd1: 1.0,cmd2: 0.5,cmd3: 1.0,cmd4: 0.0,cmd5: 0.0,cmd6: 0.0}"
// ros2 service call /cobot_api/stop_motion cobot_msgs/srv/CtrlParam "{command: 0}"
// ros2 service call /cobot_api/vary_speed cobot_msgs/srv/TaskParams "{cmd1: 1.0}"     # 0 --> 1 scale

// cobot_msgs.srv.TaskParamsinv_Response(pos1=0.725131094455719, pos2=-0.06371435523033142, pos3=0.20013514161109924, pos4=0.021665425971150398, pos5=0.021718084812164307, pos6=0.9999134540557861)
// ros2 service call /cobot_api/inverse_kin cobot_msgs/srv/TaskParamsinv "{cmd1: 0.73,cmd2: -0.07,cmd3: 0.2,cmd4: 0.0,cmd5: 0.0,cmd6: -1.57}"
// ros2 service call /cobot_api/forward_kin cobot_msgs/srv/TaskParamsinv "{cmd1: 0.94,cmd2: 2.18,cmd3: -0.09,cmd4: 1.19,cmd5: -1.99,cmd6: -2.03}"


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class MoveGroupDemo
{
   
public:
    MoveGroupDemo() : node_(rclcpp::Node::make_shared("move_group_interface_tutorial"))
    {
        PLANNING_GROUP = node_->declare_parameter<std::string>("group_name", "cobot_arm");
        joint_state_topic_name = node_->declare_parameter<std::string>("joint_state_topic_name", "");
        planning_frame = node_->declare_parameter<std::string>("planning_frame", "");

        // Initialize the MoveIt! planning interface
        move_group.reset(new moveit::planning_interface::MoveGroupInterface(node_, PLANNING_GROUP)); // You can change the planning group here

        // Initialize the planning scene interface
        planning_scene_interface.reset(new moveit::planning_interface::PlanningSceneInterface);

        robot_model_loader.reset(new robot_model_loader::RobotModelLoader(node_)); // robot_model_loader::RobotModelLoader robot_model_loader(node);

        kinematic_model = robot_model_loader->getModel();

        kinematic_state.reset(new moveit::core::RobotState(kinematic_model)); // moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));

        kinematic_state->setToDefaultValues();

        joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP); // const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
        // joint_names = joint_model_group->getVariableNames(); 
        // Get Joint Values: We can retrieve the current set of joint values stored in the state for the Panda arm.

        // Get Joint Values: We can retrieve the current set of joint values stored in the state for the Panda arm.
        std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

        stop_motion = false;

        // Create the subscriber // "joint_states" // "joint_states"
        joint_states_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(joint_state_topic_name, 10, std::bind(&MoveGroupDemo::jointStatesCallback, this, std::placeholders::_1));

        // Publish to goal pose topic
        goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_state", 10);

        // Create services
        go_to_cart_goal_service_ = node_->create_service<cobot_msgs::srv::TaskParams>("/cobot_api/go_to_cart_goal", std::bind(&MoveGroupDemo::handleGoToCartGoal, this, std::placeholders::_1, std::placeholders::_2));
        go_to_joint_goal_service_ = node_->create_service<cobot_msgs::srv::TaskParams>("/cobot_api/go_to_joint_goal", std::bind(&MoveGroupDemo::handleGoToJointGoal, this, std::placeholders::_1, std::placeholders::_2)); 
        stop_motion_service_ = node_->create_service<cobot_msgs::srv::CtrlParam>("/cobot_api/stop_motion", std::bind(&MoveGroupDemo::handleStopMotion, this, std::placeholders::_1, std::placeholders::_2));
        vary_speed_service_ = node_->create_service<cobot_msgs::srv::TaskParams>("/cobot_api/vary_speed", std::bind(&MoveGroupDemo::handleVarySpeed, this, std::placeholders::_1, std::placeholders::_2));
        get_for_kin_service_ = node_->create_service<cobot_msgs::srv::TaskParamsinv>("/cobot_api/forward_kin", std::bind(&MoveGroupDemo::handleGetForKin, this, std::placeholders::_1, std::placeholders::_2));
        get_inv_kin_service_ = node_->create_service<cobot_msgs::srv::TaskParamsinv>("/cobot_api/inverse_kin", std::bind(&MoveGroupDemo::handleGetInvKin, this, std::placeholders::_1, std::placeholders::_2));

        // Sleep for a while and then start spinning
        std::this_thread::sleep_for(std::chrono::seconds(5));
        rclcpp::spin(node_);
    }




    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint_values_ = msg->position; // {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model_));
        // kinematic_state->setVariablePositions(msg->name, msg->position);

        // const robot_state::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup("manipulator");
        // const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");

        // RCLCPP_INFO(this->get_logger(), "Translation: %f, %f, %f", end_effector_state.translation()[0], end_effector_state.translation()[1], end_effector_state.translation()[2]);
        // Eigen::Quaterniond quaternion(end_effector_state.rotation());
        // RCLCPP_INFO(this->get_logger(), "Quaternion: %f, %f, %f, %f", quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
    
        // joint_values_ = {end_effector_state.translation()[0], end_effector_state.translation()[1], end_effector_state.translation()[2],
        //     quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()}
    }



    bool handleGoToCartGoal(const std::shared_ptr<cobot_msgs::srv::TaskParams::Request> request,
                        std::shared_ptr<cobot_msgs::srv::TaskParams::Response> response)
    {
        // response->status = 1; // the service was called succesfully, Set the response status to great.
        stop_motion = false;
        move_group->stop();
        // Start a new thread to execute the handleGoToCartGoal motion asynchronously
        if (executehandleGoToCartGoal(request)) {
            response->status = 1;
            return true;
        } else { response->status = 0; 
                return false;}
        // return true;
    }


    bool executehandleGoToCartGoal(std::shared_ptr<cobot_msgs::srv::TaskParams::Request> request)
    { 
        // Show target pose: Create a PoseStamped message to be published
        geometry_msgs::msg::PoseStamped pose_st;
        pose_st.header.frame_id = planning_frame; // "Tending"; // base_link // Set the frame id
        pose_st.header.stamp = node_->now();
        pose_st.pose.position.x = request->cmd1;
        pose_st.pose.position.y = request->cmd2;
        pose_st.pose.position.z = request->cmd3;
        pose_st.pose.orientation.x = request->cmd4;
        pose_st.pose.orientation.y = request->cmd5;
        pose_st.pose.orientation.z = request->cmd6;
        pose_st.pose.orientation.w = request->cmd7;
        goal_pub_->publish(pose_st);

        std::shared_ptr<cobot_msgs::srv::TaskParamsinv::Request> requ = std::make_shared<cobot_msgs::srv::TaskParamsinv::Request>();
        std::shared_ptr<cobot_msgs::srv::TaskParamsinv::Response> resp = std::make_shared<cobot_msgs::srv::TaskParamsinv::Response>();
        bool status;
     
        requ->cmd1 = joint_values_[0];
        requ->cmd2 = joint_values_[1];
        requ->cmd3 = joint_values_[2];
        requ->cmd4 = joint_values_[3];
        requ->cmd5 = joint_values_[4];
        requ->cmd6 = joint_values_[5];

        status = handleGetForKin(requ,resp);
        if (!status) { return false; }

        Eigen::Affine3d start_tool_pose;
        Eigen::Vector3d tc(resp->pos1, resp->pos2, resp->pos3); // x y z 
        Eigen::Quaterniond qc(resp->pos7, resp->pos4, resp->pos5, resp->pos6); // w x y z 
        start_tool_pose = Eigen::Translation3d(tc) * qc;

        Eigen::Affine3d final_tool_pose; // Convert geometry_msgs::msg::Pose to Eigen::Affine3d // poseMsgToEigen(pose_st,final_tool_pose); 
        Eigen::Vector3d t(pose_st.pose.position.x, pose_st.pose.position.y, pose_st.pose.position.z);
        Eigen::Quaterniond q(pose_st.pose.orientation.w, pose_st.pose.orientation.x, pose_st.pose.orientation.y, pose_st.pose.orientation.z);
        final_tool_pose = Eigen::Translation3d(t) * q;

        auto interpolate = [](const Eigen::Affine3d& p0, const Eigen::Affine3d& pf,
            int num_poses) -> std::vector<geometry_msgs::msg::Pose>
        {
            std::vector<geometry_msgs::msg::Pose> poses;
            double increment = 1.0/double(num_poses-1);
            Eigen::Vector3d t0 = p0.translation();
            Eigen::Vector3d tf = pf.translation();
            Eigen::Quaterniond q0(p0.rotation());
            Eigen::Quaterniond qf(pf.rotation());

            Eigen::Vector3d t_new;
            Eigen::Quaterniond q_new;
            Eigen::Affine3d p_new;
            geometry_msgs::msg::Pose pose_msg;
            
            for(std::size_t i = 0; i < static_cast<std::size_t>(num_poses); i++) // for(std::size_t i = 0; i < num_poses; i++)
            {
                double t = i*increment;
                t_new = t0 + (tf - t0) * (t);
                q_new = q0.slerp(t,qf);
                p_new = Eigen::Translation3d(t_new) * q_new;
                // Convert Eigen::Affine3d to geometry_msgs::msg::Pose |  tf2::poseEigenToMsg(p_new,pose_msg);
                pose_msg.position.x = p_new.translation().x();
                pose_msg.position.y = p_new.translation().y();
                pose_msg.position.z = p_new.translation().z();
                Eigen::Quaterniond q(p_new.rotation());
                pose_msg.orientation.x = q.x();
                pose_msg.orientation.y = q.y();
                pose_msg.orientation.z = q.z();
                pose_msg.orientation.w = q.w();
                poses.push_back(pose_msg);
            }
            return poses;
        };
        
        int cartesian_num_points_ = 3; // 40;
        std::vector<geometry_msgs::msg::Pose> waypoints = interpolate(start_tool_pose, final_tool_pose, cartesian_num_points_);

        const double jump_threshold = 0.0; // 0.0; //  ("cartesian_jump_threshold",cartesian_jump_threshold_,2.0);
        const double eef_step = 0.01; // 1cm resolution   ("cartesian_eef_max_step",cartesian_eef_max_step_,0.1);
        moveit_msgs::msg::RobotTrajectory trajectory;

        double res = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        RCLCPP_INFO(LOGGER, "\n Cartesian path plan (%.2f%% res acheived) and  %lu points", res * 100.0, waypoints.size());
        if(res<0.8) // 0.8 // 0.9
        {
            RCLCPP_ERROR(LOGGER, "Cartesian plan only solved %f of the %lu points", 100.0 * res, waypoints.size());
            return false;
        }

        // Set the time_from_start for each point in the trajectory
        double time_step = 0.1; // Set your desired time step here
        for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i)
        {
            // trajectory.joint_trajectory.points[i].time_from_start = rclcpp::Duration(i * time_step);
            trajectory.joint_trajectory.points[i].time_from_start = rclcpp::Duration::from_nanoseconds(i * time_step * 1e9);
        }

        // Execute the trajectory as before
        // galactic
        // bool success = (move_group->execute(trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // humble
        bool success = (move_group->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            // std::this_thread::sleep_for(std::chrono::seconds(2)); // Adjust sleep duration
            return true;}
        else {
            return false;}
    }



    bool handleGoToJointGoal(const std::shared_ptr<cobot_msgs::srv::TaskParams::Request> request,
                        std::shared_ptr<cobot_msgs::srv::TaskParams::Response> response)
    {
        // response->status = 1;
        stop_motion = false;
        move_group->stop();
        // Start a new thread to execute the go_to_home motion asynchronously
        if (executehandleGoToJointGoal(request)) {
            response->status = 1;
            return true;
        } else { response->status = 0; 
                return false;}
        // return true;
    }
    bool executehandleGoToJointGoal(std::shared_ptr<cobot_msgs::srv::TaskParams::Request> request)
    {
        // Your go_to_joint_goal logic here, using move_group_
        std::vector<std::vector<double>> joint_group_positions_list = {{request->cmd1, request->cmd2, request->cmd3, request->cmd4, request->cmd5, request->cmd6},}; // {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},};
        for (auto &joint_group_positions : joint_group_positions_list) {
            move_group->setJointValueTarget(joint_group_positions);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            // galactic
            // bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            // humble
            bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success)
                { move_group->move();}
            else {return false; }
                
        }    
        return true;
    }



    bool handleStopMotion(const std::shared_ptr<cobot_msgs::srv::CtrlParam::Request> /*request*/,
                        std::shared_ptr<cobot_msgs::srv::CtrlParam::Response> response)
    {
        response->status = 1;
        stop_motion = true;
        move_group->stop();
        return true;
    }



    bool handleVarySpeed(const std::shared_ptr<cobot_msgs::srv::TaskParams::Request> request,
                        std::shared_ptr<cobot_msgs::srv::TaskParams::Response> response)
    {
        response->status = 1;      
        move_group->setMaxVelocityScalingFactor(request->cmd1); // 0.05
        move_group->setMaxAccelerationScalingFactor(request->cmd1); // 0.05
        return true;
    }


    // void Forward_Kinematics(std::vector<double> const &joint_positions)
    bool handleGetForKin(const std::shared_ptr<cobot_msgs::srv::TaskParamsinv::Request> request,
                        std::shared_ptr<cobot_msgs::srv::TaskParamsinv::Response> response)
    {
      std::vector<double> joint_positions_ = {request->cmd1,request->cmd2,request->cmd3,request->cmd4,request->cmd5,request->cmd6};

      moveit::core::RobotState n_kinematic_state(kinematic_model);

      n_kinematic_state.setVariablePositions(joint_model_group->getActiveJointModelNames(), joint_positions_);
      
      // Now, we can use moveit to compute forward kinematics to find the pose of the "any_link"
      const Eigen::Isometry3d& end_effector_state = n_kinematic_state.getGlobalLinkTransform("link_6");
      
      /* Print end-effector pose. Remember that this is in the model frame */
      RCLCPP_INFO_STREAM(LOGGER, "Fwd: Translation: \n" << end_effector_state.translation() << "\n");
      RCLCPP_INFO_STREAM(LOGGER, "Fwd: Rotation: \n" << end_effector_state.rotation() << "\n");

      response->pos1 = end_effector_state.translation()[0];
      response->pos2 = end_effector_state.translation()[1];
      response->pos3 = end_effector_state.translation()[2];

      Eigen::Matrix3d rotation_matrix = end_effector_state.rotation();
      Eigen::Quaterniond quaternion(rotation_matrix);

      response->pos4 = quaternion.x();
      response->pos5 = quaternion.y();
      response->pos6 = quaternion.z();
      response->pos7 = quaternion.w(); 

      return true;
    }



    //bool handleGetInvKin(const Eigen::Isometry3d& end_effector_state)
    bool handleGetInvKin(const std::shared_ptr<cobot_msgs::srv::TaskParamsinv::Request> request,
                        std::shared_ptr<cobot_msgs::srv::TaskParamsinv::Response> response)
    {
      moveit::core::RobotState n_kinematic_state(kinematic_model);

      Eigen::Isometry3d end_effector_state;
      Eigen::Isometry3d end_effector_state_new;
      end_effector_state_new.translation() = Eigen::Vector3d(request->cmd1, request->cmd2, request->cmd3);

      double roll, pitch, yaw;
      
      double qx = request->cmd4;
      double qy = request->cmd5;
      double qz = request->cmd6;
      double qw = request->cmd7;

      roll = atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);
      pitch = asin(-2.0*(qx*qz - qw*qy));
      yaw = atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz);

      Eigen::Matrix3d rotation_matrix;
      rotation_matrix = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

      end_effector_state_new.linear() = rotation_matrix;
      end_effector_state = end_effector_state_new;

      /* Print end-effector pose. Remember that this is in the model frame */
      RCLCPP_INFO_STREAM(LOGGER, "Inv: Translation: \n" << end_effector_state.translation() << "\n");
      RCLCPP_INFO_STREAM(LOGGER, "Inv: Rotation: \n" << end_effector_state.rotation() << "\n");

      std::vector<double> joint_values;

      double timeout = 0.1;
      bool found_ik = n_kinematic_state.setFromIK(joint_model_group, end_effector_state, timeout); // (iiwa_modelgroup, target_eig_pose1, iiwa_ee_link, 10, 0.1)
      if (found_ik) // Now, we can print out the IK solution (if found):
      {
        n_kinematic_state.copyJointGroupPositions(joint_model_group, joint_values);

        response->pos1 = joint_values[0];
        response->pos2 = joint_values[1];
        response->pos3 = joint_values[2];
        response->pos4 = joint_values[3];
        response->pos5 = joint_values[4];
        response->pos6 = joint_values[5];
        return true;
      }
      else
      {
        response->pos1 = 10000;
        RCLCPP_INFO(LOGGER, "Did not find IK solution");
        return false;
      }
    }


    std::tuple<Eigen::MatrixXd, bool> Jacobian(std::string const & /*link_name*/, std::vector<double> const &joint_positions)
    { // Get the Jacobian
      moveit::core::RobotState n_kinematic_state(kinematic_model);
      Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
      Eigen::MatrixXd jacobian;
      n_kinematic_state.setVariablePositions(joint_model_group->getActiveJointModelNames(), joint_positions);
      auto const success = n_kinematic_state.getJacobian(joint_model_group,
                                  n_kinematic_state.getLinkModel(joint_model_group->getLinkModelNames().back()), // kinematic_state->getLinkModel(link_name)
                                  reference_point_position, 
                                  jacobian);
      RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");
      return std::tuple{jacobian, success};
    }



private:
    bool stop_motion;
    std::string PLANNING_GROUP;
    std::string planning_frame;
    std::string joint_state_topic_name;

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;

    std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader;
    std::shared_ptr<moveit::core::RobotModel> kinematic_model;
    std::shared_ptr<moveit::core::RobotState> kinematic_state; 

    const moveit::core::JointModelGroup* joint_model_group;

    rclcpp::Service<cobot_msgs::srv::TaskParamsinv>::SharedPtr get_for_kin_service_;
    rclcpp::Service<cobot_msgs::srv::TaskParamsinv>::SharedPtr get_inv_kin_service_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Service<cobot_msgs::srv::TaskParams>::SharedPtr go_to_joint_goal_service_;
    rclcpp::Service<cobot_msgs::srv::TaskParams>::SharedPtr go_to_cart_goal_service_;
    rclcpp::Service<cobot_msgs::srv::TaskParams>::SharedPtr vary_speed_service_;
    rclcpp::Service<cobot_msgs::srv::CtrlParam>::SharedPtr stop_motion_service_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
    std::vector<double> joint_values_;
    std::vector<double> eff_values_;

};




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    MoveGroupDemo demo;
    rclcpp::shutdown();
    return 0;
}




#ifndef COBOT_GROUP_TELEOP_API_H
#define COBOT_GROUP_TELEOP_API_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/int64.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

// Include the generated service message header
// TaskParams task_params | // CtrlParam ctrl_param | // TaskParamsinv task_paramsinv
#include "cobot_msgs/srv/ctrl_param.hpp"
#include "cobot_msgs/srv/task_params.hpp"




class MoveGroupTeleopAPI
{
public:
    // MoveGroupTeleopAPI(const rclcpp::Node::SharedPtr& node,moveit::planning_interface::MoveGroupInterfacePtr& group, std::string action_name, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);
    MoveGroupTeleopAPI(rclcpp::Node::SharedPtr node);

    void timerCallback();
    void js_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    // bool performIK(const geometry_msgs::msg::PoseStamped& pose, const std::shared_ptr<cobot_msgs::srv::CtrlParam::Response>& resp);
    // std::string getDirectionString(int operation_num, int symbol);

    // void teleopJointCmdCB(const std_msgs::Int64ConstPtr &msg);
    void teleopJointCmdCB(const std_msgs::msg::Int64 &msg);
    // void teleopCartCmdCB(const std_msgs::Int64ConstPtr &msg);
    void teleopCartCmdCB(const std_msgs::msg::Int64 &msg);

    bool jointTeleop_cb(const std::shared_ptr<cobot_msgs::srv::CtrlParam::Request> req,const std::shared_ptr<cobot_msgs::srv::CtrlParam::Response> resp);
    bool cartTeleop_cb(const std::shared_ptr<cobot_msgs::srv::CtrlParam::Request> req, const std::shared_ptr<cobot_msgs::srv::CtrlParam::Response> resp);
    bool handleVarySpeed(const std::shared_ptr<cobot_msgs::srv::TaskParams::Request> req, const std::shared_ptr<cobot_msgs::srv::TaskParams::Response> resp);

    void PoseStampedRotation(geometry_msgs::msg::PoseStamped &pose_stamped, const tf2::Vector3 &axis, double angle);

private:

    moveit::planning_interface::MoveGroupInterfacePtr group_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    // ros::NodeHandle teleop_nh_;
    rclcpp::Node::SharedPtr teleop_nh_;

    // actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client_;
    // rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory> action_client_;
    // std::shared_ptr<control_msgs::action::FollowJointTrajectory> action_client_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;
    // control_msgs::FollowJointTrajectoryGoal goal_;
    control_msgs::action::FollowJointTrajectory::Goal goal_;

    rclcpp::Service<cobot_msgs::srv::CtrlParam>::SharedPtr joint_teleop_server_;
    rclcpp::Service<cobot_msgs::srv::CtrlParam>::SharedPtr cart_teleop_server_;
    rclcpp::Service<cobot_msgs::srv::TaskParams>::SharedPtr vary_speed_teleop_service_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    double joint_speed_limit_;
    double velocity_scaling_;
    double joint_speed_default_;
    double cart_duration_default_;
    double resolution_angle_;
    double resolution_linear_;

    double joint_speed_;
    double cart_duration_; // in second

    const double JOINT_SPEED_LIMIT_CONST=1.57; // rad per second
    const double JOINT_SPEED_DEFAULT_CONST=0.78; // rad per second
    const double CART_DURATION_DEFAULT_CONST=0.1; // 0.04 // second

    std::string end_link_;
    std::string reference_link_;
    std::string default_tip_link_;
    std::string root_link_;
    std::string action_name;
    std::string PLANNING_GROUP;

    std::vector<double> current_js;

    bool robot_in_motion;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    geometry_msgs::msg::TransformStamped transform_rootToRef_;

    geometry_msgs::msg::TransformStamped transform_tipToEnd_;

};


#endif // COBOT_GROUP_TELEOP_API_H

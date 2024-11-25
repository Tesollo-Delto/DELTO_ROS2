#ifndef DELTO_EXTERNAL_DRIVER_HPP
#define DELTO_EXTERNAL_DRIVER_HPP


#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"

#include <thread>
#include <mutex>
#include <vector>

#include "delto_external_TCP.hpp"


using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;


class DeltoExternalDriver : public rclcpp::Node {
public:
    DeltoExternalDriver();
    virtual ~DeltoExternalDriver();

private:
    void joint_state_publisher();
    void timer_callback();
    void targetjoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  
    std::vector<double> Torque2Duty(std::vector<double> tq_u);
    std::vector<double> JointControl(std::vector<double> target_joint_state,
                                        std::vector<double> current_joint_state,
                                        std::vector<double> joint_dot,
                                        std::vector<double> kp,
                                        std::vector<double> kd);
                                        
    std::vector<double> get_position();
    void execute_callback(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
    void execute_callback_test(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr grasp_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr grasp_mode_sub;
    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_trajectory_server;


    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
    
    void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

    rclcpp::TimerBase::SharedPtr joint_state_timer;

    std::unique_ptr<DeltoTCP::Communication> delto_client;
    std::mutex mutex_;

    std::vector<double> kp;
    std::vector<double> kd;

    std::vector<double> current_joint_state;
    std::vector<double> electric_current_state;
    std::vector<double> target_joint_state;
    std::vector<double> joint_dot;
    std::vector<double> pre_joint_state;
    
    std::string delto_ip;
    int delto_port;

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Time prev_time;

    double publish_rate;
    int joint_publish_count;
    sensor_msgs::msg::JointState joint_state;

   // void connect();
};

#endif // DELTO_EXTERNAL_DRIVER_HPP

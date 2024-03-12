#include <rclcpp/rclcpp.hpp>
#include <universal_robot_ign/joint_position_controller.hpp>
#include <universal_robot_ign/joint_trajectory_controller.hpp>

int main(int argc, char* argv[])
{
    //creat ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("joint_controller");
    // variable
    std::vector<std::string> joint_names;
    std::vector<std::string> ign_joint_topics;
    int update_rate;
    // parameters
    ros_node->declare_parameter<std::vector<std::string>>("joint_names");
    ros_node->declare_parameter<std::vector<std::string>>("ign_joint_topics");
    ros_node->declare_parameter<int>("rate", 200);
    joint_names = ros_node->get_parameter("joint_names").as_string_array();
    ign_joint_topics = ros_node->get_parameter("ign_joint_topics").as_string_array();
    update_rate = ros_node->get_parameter("rate").as_int();
    // create controller
    auto joint_trajectory_controller = std::make_shared<universal_robot_ign::jointTrajectoryController>(ros_node,
        joint_names, "set_joint_trajectory", ign_joint_topics ,update_rate);
    // create controller 
    auto joint_position_controller = std::make_shared<universal_robot_ign::JointPositionController>(ros_node,
        joint_names, "set_joint_state", ign_joint_topics);
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}

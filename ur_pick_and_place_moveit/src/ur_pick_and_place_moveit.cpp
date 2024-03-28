#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#define PI 3.14159265358979

int main(int argc, char *argv[])
{

    // Create a ROS logger
    auto const LOGGER = rclcpp::get_logger("ur_pick_and_place_moveit");

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node =
        rclcpp::Node::make_shared("ur_pick_and_place_moveit", node_options);
    //         self.grasp_sub = self.create_subscription(Int32,'gripper/cmd',callback=self.grasp_callback, qos_profile=qos_profile)
    
    // auto gripper_pub = rclcpp::create_publisher<std_msgs::msg::Int32>(move_group_node, "gripper/cmd", 10);
    auto gripper_pub = move_group_node->create_publisher<std_msgs::msg::Int32>("gripper/cmd", 10);
    auto gripper_joint_pub = move_group_node->create_publisher<std_msgs::msg::Float32MultiArray>("gripper/target_joint", 10);
    
    rclcpp::sleep_for(std::chrono::nanoseconds(100000000));


    auto grpper_joint_msg = std_msgs::msg::Float32MultiArray();
    grpper_joint_msg.data = std::vector<float>{             0.0, 0.0, 40.0* PI/180.0, 70.0 * PI/180.0,
                                            0.0 * PI/180, 0.0, 40.0 * PI/180.0, 70.0 * PI/180.0,
                                            0.0 * PI/180, 0.0, 40.0 * PI/180.0, 70.0 * PI/180.0};
    gripper_joint_pub->publish(grpper_joint_msg);

    rclcpp::sleep_for(std::chrono::nanoseconds(100000000));
    
    auto gripper_cmd = std_msgs::msg::Int32();
    gripper_cmd.data = 0;
    gripper_pub->publish(gripper_cmd);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

    moveit::planning_interface::MoveGroupInterface move_group_arm(
        move_group_node, PLANNING_GROUP_ARM);

    const moveit::core::JointModelGroup *joint_model_group_arm =
        move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

    move_group_arm.setEndEffectorLink("tool0");

    for (auto name : move_group_arm.getLinkNames())
    {
        RCLCPP_INFO(LOGGER, "Link: %s", name.c_str());
    }

    // Make plane to appropriate path planning

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    moveit_msgs::msg::CollisionObject collision_object;
    moveit_msgs::msg::CollisionObject collision_object2;

    collision_object.header.frame_id = move_group_arm.getPlanningFrame();
    collision_object2.header.frame_id = move_group_arm.getPlanningFrame();

    collision_object.id = "ground";

    shape_msgs::msg::Plane plane;
    plane.coef = {0, 0, 1, 0}; // Equation of plane z = 0

    geometry_msgs::msg::Pose ground_pose;
    ground_pose.orientation.w = 1.0;
    ground_pose.position.z = -0.01; // z position

    collision_object.planes.push_back(plane);
    collision_object.plane_poses.push_back(ground_pose);
    collision_object.operation = collision_object.ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 1.5;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.48;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;

    collision_object2.primitives.push_back(primitive);
    collision_object2.primitive_poses.push_back(box_pose);
    collision_object2.operation = collision_object2.ADD;


    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    collision_objects.push_back(collision_object);
    collision_objects.push_back(collision_object2);


    planning_scene_interface.addCollisionObjects(collision_objects);

    // Get Current State


    moveit::core::RobotStatePtr current_state_arm =
        move_group_arm.getCurrentState(10);

    std::vector<double> joint_group_positions_arm;
    current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                               joint_group_positions_arm);

    move_group_arm.setStartStateToCurrentState();

//     // Set UR3e to Home State
//     RCLCPP_INFO(LOGGER, "Going Home");

//     joint_group_positions_arm[0] = 0.00;    // Shoulder Pan
//     joint_group_positions_arm[1] = -0.5 *PI; // Shoulder Lift
//     joint_group_positions_arm[2] = 0.00;    // Elbow
//     joint_group_positions_arm[3] = 0.00;    // Wrist 1
//     joint_group_positions_arm[4] = 0.00;    // Wrist 2
//     // joint_group_positions_arm[5] = 1.5 * PI;  // Wrist 3

//     move_group_arm.setJointValueTarget(joint_group_positions_arm);

//     moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm1;
//     bool success = (move_group_arm.plan(my_plan_arm1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     move_group_arm.execute(my_plan_arm1);
    
//     rclcpp::sleep_for(std::chrono::seconds(3));

//     // RCLCPP_INFO(LOGGER, "Going Ready");

//     // joint_group_positions_arm[0] = 78.6 * PI / 180;   // Shoulder Pan
//     // joint_group_positions_arm[1] = -74.36 * PI / 180; // Shoulder Lift
//     // joint_group_positions_arm[2] = 63.37 * PI / 180;  // Elbow
//     // joint_group_positions_arm[3] = -79.08 * PI / 180; // Wrist 1
//     // joint_group_positions_arm[4] = -89.93 * PI / 180; // Wrist 2
//     // // joint_group_positions_arm[5] = 0.00;  // Wrist 3

//     // move_group_arm.setJointValueTarget(joint_group_positions_arm);

//     // moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm3;
//     // success = (move_group_arm.plan(my_plan_arm3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     // move_group_arm.execute(my_plan_arm3);


//     // auto &last_point = my_plan_arm3.trajectory_.joint_trajectory.points.back();
//     // auto last_time = last_point.time_from_start;
//     // // auto duration_time = last_time.toSec();
//     // int duration_time = last_time.sec;
//     // duration_time += 1;
//     // rclcpp::sleep_for(std::chrono::seconds(duration_time));

//     // Pregrasp - Move to Picking Point
//     RCLCPP_INFO(LOGGER, "Pregrasp Position");
//     current_state_arm = move_group_arm.getCurrentState(10);
//     current_state_arm->copyJointGroupPositions(joint_model_group_arm,
//                                                joint_group_positions_arm);

//     tf2::Quaternion orientation;
//     orientation.setRPY(0, -PI, 0); // Set Rotation Roll Pitch Yaw
//     geometry_msgs::msg::Quaternion ros_orientation;

//     ros_orientation = tf2::toMsg(orientation); // tf2 Quaternion -> ROS msg

//     // Assign Quaternion
//     geometry_msgs::msg::Pose target_pose1;
//     target_pose1.orientation = ros_orientation;

//     target_pose1.position.x = -0.1;
//     target_pose1.position.y = 0.27;
//     target_pose1.position.z = 0.450;

//     move_group_arm.setPoseTarget(target_pose1);
//     success = (move_group_arm.plan(my_plan_arm1) == moveit::core::MoveItErrorCode::SUCCESS);
//     move_group_arm.execute(my_plan_arm1);

//     rclcpp::sleep_for(std::chrono::seconds(3));

//     // Approach - Get close to object along the Cartesian Path

//     RCLCPP_INFO(LOGGER, "Approach to object!");

//     std::vector<geometry_msgs::msg::Pose> approach_waypoints1;
//     target_pose1.position.z -= 0.049;
//     approach_waypoints1.push_back(target_pose1);

//     target_pose1.position.z -= 0.049;
//     approach_waypoints1.push_back(target_pose1);

//     target_pose1.position.z -= 0.049;
//     approach_waypoints1.push_back(target_pose1);


//     moveit_msgs::msg::RobotTrajectory trajectory_approach1;

//     const double jump_threshold = 0.0;
//     const double eef_step = 0.01;

//     double fraction = move_group_arm.computeCartesianPath(
//         approach_waypoints1, eef_step, jump_threshold, trajectory_approach1);

//     RCLCPP_INFO(LOGGER, "approach1 fraction: %f", fraction);

//     move_group_arm.execute(trajectory_approach1);

//     rclcpp::sleep_for(std::chrono::seconds(2));

// //////////////////////////////////////////////////
//     // Grasp
//     gripper_cmd.data = 1;
//     gripper_pub->publish(gripper_cmd);
//     rclcpp::sleep_for(std::chrono::seconds(3));

// //////////////////////////////////////////////////

//     // Retreat - Lift up the object along the Cartesian Path
//     RCLCPP_INFO(LOGGER, "Retreat from object!");

//     std::vector<geometry_msgs::msg::Pose> retreat_waypoints1;
//     target_pose1.position.z += 0.049;
//     retreat_waypoints1.push_back(target_pose1);
 

//     target_pose1.position.z += 0.049;
//     retreat_waypoints1.push_back(target_pose1);

//     target_pose1.position.z += 0.049;
//     retreat_waypoints1.push_back(target_pose1);

//     moveit_msgs::msg::RobotTrajectory trajectory_retreat1;

//     // eef_step = 0.005;
//     fraction = move_group_arm.computeCartesianPath(
//         retreat_waypoints1, eef_step, jump_threshold, trajectory_retreat1);
//     RCLCPP_INFO(LOGGER, "retreat1 fraction: %f", fraction);

//     move_group_arm.execute(trajectory_retreat1);

//     rclcpp::sleep_for(std::chrono::seconds(4));


// //////////////////////////////////////////////////

//     // Carry - Move to Placing Point
//     RCLCPP_INFO(LOGGER, "Move to Placing Point");

//     std::vector<geometry_msgs::msg::Pose> carry_waypoints;
//     target_pose1.position.x -= 0.200;
//     target_pose1.position.y -= 0.100;
//     carry_waypoints.push_back(target_pose1);

//     moveit_msgs::msg::RobotTrajectory trajectory_carry;

//     fraction = move_group_arm.computeCartesianPath(
//     carry_waypoints, eef_step, jump_threshold, trajectory_carry);

//     move_group_arm.execute(trajectory_carry);
//     RCLCPP_INFO(LOGGER, "carry_waypoints fraction: %f", fraction);

//     rclcpp::sleep_for(std::chrono::seconds(4));

// //////////////////////////////////////////////////
//     // Approach - Get close to object along the Cartesian Path
//     RCLCPP_INFO(LOGGER, "Approach to object!");

//     std::vector<geometry_msgs::msg::Pose> approach_waypoints2;
    
//     target_pose1.position.z -= 0.049;
//     approach_waypoints2.push_back(target_pose1);

//     target_pose1.position.z -= 0.049;
//     approach_waypoints2.push_back(target_pose1);
    
//     target_pose1.position.z -= 0.049;
//     approach_waypoints2.push_back(target_pose1);
    

//     moveit_msgs::msg::RobotTrajectory trajectory_approach2;
//     fraction = move_group_arm.computeCartesianPath(
//         approach_waypoints2, eef_step, jump_threshold, trajectory_approach2);
//     move_group_arm.execute(trajectory_approach2);

//         RCLCPP_INFO(LOGGER, "approach_waypoints2 fraction: %f", fraction);

//     rclcpp::sleep_for(std::chrono::seconds(4));


// //////////////////////////////////////////////////
//     // Dropping - Add Dropping action here
//     gripper_cmd.data = 0;
//     gripper_pub->publish(gripper_cmd);
//     rclcpp::sleep_for(std::chrono::seconds(3));

    
//     // Retreat - Lift up the object along the Cartesian Path
//     RCLCPP_INFO(LOGGER, "Retreat from object!");

//     std::vector<geometry_msgs::msg::Pose> retreat_waypoints3;
//     target_pose1.position.z += 0.049;
//     retreat_waypoints3.push_back(target_pose1);

//     target_pose1.position.z += 0.049;
//     retreat_waypoints3.push_back(target_pose1);

//     target_pose1.position.z += 0.049;
//     retreat_waypoints3.push_back(target_pose1);

//     moveit_msgs::msg::RobotTrajectory trajectory_retreat3;

//     fraction = move_group_arm.computeCartesianPath(
//         retreat_waypoints3, eef_step, jump_threshold, trajectory_retreat3);

//         RCLCPP_INFO(LOGGER, "retreat2 fraction: %f", fraction);

//     move_group_arm.execute(trajectory_retreat3);

//     rclcpp::sleep_for(std::chrono::seconds(3));

//     // Get back UR3e to Home State
//     RCLCPP_INFO(LOGGER, "Going Home");

//     current_state_arm = move_group_arm.getCurrentState(10);

//     current_state_arm->copyJointGroupPositions(joint_model_group_arm,
//                                                joint_group_positions_arm);

//     move_group_arm.setStartStateToCurrentState();

//     joint_group_positions_arm[0] = 0.00;    // Shoulder Pan
//     joint_group_positions_arm[1] = -0.5 * PI; // Shoulder Lift
//     joint_group_positions_arm[2] = 0.00;    // Elbow
//     joint_group_positions_arm[3] = 0.00;    // Wrist 1
//     joint_group_positions_arm[4] = 0.00;    // Wrist 2
//     // joint_group_positions_arm[5] = 1.5 * PI;  // Wrist 3

//     move_group_arm.setJointValueTarget(joint_group_positions_arm);

//     moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm2;
//     success = (move_group_arm.plan(my_plan_arm2) == moveit::core::MoveItErrorCode::SUCCESS);
//     move_group_arm.execute(my_plan_arm2);

    rclcpp::sleep_for(std::chrono::seconds(10));

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}

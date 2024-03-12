from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("delto_gripper_3f", package_name="delto_3f_moveit").to_moveit_configs()
   
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))

    # Given the published joint states, publish tf for the robot links and the robot description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        remappings=[("/joint_states", "/gripper/joint_states")],
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
        ],
    )
    ld.add_action(rsp_node)

    return ld
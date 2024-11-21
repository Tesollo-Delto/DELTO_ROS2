from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "delto_ip", 
            default_value="169.254.186.72", 
            description="IP address for gripper"
        )
    )
    delto_ip = LaunchConfiguration("delto_ip")

    declared_arguments.append(
        DeclareLaunchArgument(
            "delto_port", 
            default_value="502", 
            description="Port for gripper"
        )
    )
    
    delto_port = LaunchConfiguration("delto_port")

    # Get paths to config files
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("delto_5f_driver"), "urdf", "delto_5f_driver.xacro"]
            ),
            " ",
            "delto_ip:=",
            delto_ip,
            " ",
            "delto_port:=",
            delto_port,
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("delto_5f_driver"), "cfg", "delto_5f_driver.yaml"]
    )

    # ROS2 Control Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Delto 5F Controller
    delto_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["delto_5f_controller"],
        output="screen",
    )
    
    rqt_joint_trajectory_controller = Node(
    package="rqt_joint_trajectory_controller",
    executable="rqt_joint_trajectory_controller",
    name="rqt_joint_trajectory_controller",
    output="screen",
    parameters=[
        robot_description,
        robot_controllers,
        {"delto_ip": delto_ip, "delto_port": delto_port},
    ],
)
    
    # List all nodes to start
    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delto_controller_spawner,
        # rqt_joint_trajectory_controller
    ]

    return LaunchDescription(declared_arguments + nodes)
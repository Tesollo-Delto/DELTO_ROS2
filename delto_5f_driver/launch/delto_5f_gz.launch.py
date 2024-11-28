from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # Get package paths
    pkg_delto_description = FindPackageShare("delto_description").find("delto_description")
    model_path = os.path.join(pkg_delto_description, "meshes")
    
    # Set Gazebo model path
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':' + model_path
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = model_path

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        # launch_arguments={"gz_args": " -r  empty.sdf"}.items(),
        launch_arguments={
    "gz_args": " -r empty.sdf -v 0"
}.items(),
    )

    # Get URDF via xacro
    robot_description_content1 = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("delto_5f_driver"), "urdf", "delto_5f_gazebo.xacro"]
            ),
        ]
    )
    robot_description_content2 = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("delto_5f_driver"), "urdf", "delto_5f_gazebo.urdf.xacro"]
            ),
        ]
    )
    
    robot_description1 = {"robot_description": robot_description_content1}
    robot_description2 = {"robot_description": robot_description_content2}
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("delto_5f_driver"),
            "cfg",
            "gazebo_controller.yaml",
        ]
    )

    # Node for control
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description1, robot_controllers],
        # remappings=[
        #     ("~/robot_description", "/robot_description"),
        # ],
        output="screen",
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description1],
    )
    node_robot_state_other_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description2],
        remappings=[("/robot_description", "/robot_description_other")],
    )
        
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            # "-file", file_path, 
            "-name", "delto_5f_gazebo",
            "-allow_renaming", "true",
            # "-self_collide", "false",
            # "-param", "{self_collide': False}",
        ],
    )

    # Delay start of robot controllers after spawn
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller",
                   '--param-file',
                   robot_controllers,
                    ],
    )

    rqt_joint_trajectory_controller = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller",
        name="rqt_joint_trajectory_controller",
        output="screen",
        parameters=[
            robot_description1,
            robot_controllers,
        ],
    )
    nodes = [
        gazebo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),
        node_robot_state_publisher,
        gz_spawn_entity,
        #control_node,
        # rqt_joint_trajectory_controller,
        node_robot_state_other_publisher

    ]

    return LaunchDescription(nodes)
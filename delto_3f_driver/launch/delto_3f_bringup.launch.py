import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    description_package = FindPackageShare('delto_description')
    delto_driver_package = FindPackageShare('delto_3f_driver')

    # Initialize Arguments
    name = LaunchConfiguration("name")
    delto_ip = LaunchConfiguration("delto_ip")
    delto_port = LaunchConfiguration("delto_port")
    delto_id = LaunchConfiguration("delto_id")
    fake_mode = LaunchConfiguration("fake_mode")

    launch_rviz = LaunchConfiguration("launch_rviz")
    
    initial_joint_controllers = PathJoinSubstitution(
            [delto_driver_package, "controller", "delto_3f_controller.yaml"])


    share_dir = get_package_share_directory('delto_description')

    urdf_file = os.path.join(share_dir, 'urdf', 'delto_gripper_3f.urdf')

    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [description_package, "config", "dg3f_display.rviz"]
    )
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    delto_3f_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, initial_joint_controllers],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        remappings=[("/robot_description", "/gripper_description"),
                    ("/joint_states", "/gripper/joint_states")],
        parameters=[robot_description],
    )

    # show_gui = LaunchConfiguration('gui')

    joint_state_publisher_gui_node = Node(
        # condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='delto_joint_state_publisher_gui',
        output='screen',


        )


    delto_3f_driver = Node(
        package="delto_3f_driver",
        executable="delto_3f_driver.py",
        name="delto_3f_driver",
        output="screen",
        emulate_tty=True,
        parameters=[
            {'ip': delto_ip.perform(context)},
            {'port': int(delto_port.perform(context))},
            {'slaveID': int(delto_id.perform(context))},
            {'dummy': bool(fake_mode.perform(context))}    

        ],
    )

    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Delay rviz

    delay_rviz2_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_publisher_node,
            on_exit=[rviz_node],
        )
    )

    
    nodes_to_start = [
        # gui_arg,
        delto_3f_driver,
        robot_state_publisher_node,
        delay_rviz2_spawner,
        rviz_node,
        # joint_state_publisher_gui_node
    ]
    
    # if bool(fake_mode.perform(context)):
    #     nodes_to_start.append(joint_state_publisher_gui_node)

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value="delto_3f"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "delto_ip", 
            default_value="169.254.186.72",
            description="IP address for gripper"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "delto_port", 
            default_value="10000", 
            description="Port for gripper"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "delto_id", 
            default_value="1", 
            description="ID for gripper"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", 
            default_value="true", 
            description="Launch RViz?"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_mode", 
            default_value="false", 
            description="simulate fake gripper"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
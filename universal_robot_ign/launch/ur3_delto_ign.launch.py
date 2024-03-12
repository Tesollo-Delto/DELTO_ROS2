'''Launch ignition_simulator with ros joint position controller and state publisher'''

import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from xmacro.xmacro4sdf import XMLMacro4sdf
from sdformat_tools.urdf_generator import UrdfGenerator

def generate_launch_description():
    ld = LaunchDescription()
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_universal_robot_ign = get_package_share_directory('universal_robot_ign')
    #data
    world_sdf_path=os.path.join(pkg_universal_robot_ign, 'resource', 'worlds', 'test_world.sdf') 
    robot_sdf_path=os.path.join(pkg_universal_robot_ign, 'resource', 'ur3_delto.sdf') 
    ign_config_path=os.path.join(pkg_universal_robot_ign, 'ign', 'gui.config')
    # ignition_simulator launch
    ignition_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
        launch_arguments={
            'ign_args': world_sdf_path + ' -r -v 2 --gui-config ' + ign_config_path,
        }.items()
    )
    ld.add_action(ignition_simulator)
    
    # Spawn robot
    robot_macro = XMLMacro4sdf()
    robot_macro.set_xml_file(robot_sdf_path)
    robot_macro.generate()
    robot_xml = robot_macro.to_string()

    spawn_robot = Node(package='ros_ign_gazebo', executable='create',
        arguments=['-name', 'ur3' ,'-z', '1.4', '-file', robot_sdf_path],
        output='screen')
    ld.add_action(spawn_robot)

    # parameter for ur10 controller
    joint_names_list=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                    "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
    ign_joint_topics_list=[]
    for joint_name in joint_names_list:
        ign_joint_topics_list.append("/model/ur3/joint/%s/0/cmd_pos"%joint_name)
    
    # ros<-ign, joint state publisher for ur10
    joint_state_publisher=Node(package='universal_robot_ign', 
                executable='joint_state_publisher',
                name="ur3_joint_state_publisher",
                parameters=[{"joint_names": joint_names_list},
                            {"ign_topic": "/world/default/model/ur3/joint_state"}
                        ],
                remappings=[("/joint_states","/ign_joint_states")],
                output='screen')
    ld.add_action(joint_state_publisher)

    #  ros->ign,  joint controller for ur10
    joint_controller=Node(package='universal_robot_ign', 
                executable='joint_controller',
                name="ur3_joint_controller",
                parameters=[{"joint_names": joint_names_list},
                            {"ign_joint_topics": ign_joint_topics_list},
                            {"rate":200},             
                           ],
                output='screen')
    ld.add_action(joint_controller)
    # ros->ign, ign bridge to control Gripper

    
    # bridge_arguments=['/model/ur3e/gripper/cmd@std_msgs/msg/Int32@ignition.msgs.Int32']
    # bridge_remappings=[("/model/ur3e/gripper/cmd","/gripper/cmd")]

    # for i in range(12):
    #     bridge_arguments.append('/model/ur3e/gripper/target_joint'+str(i)+'/@std_msgs/msg/Float32@ignition.msgs.Float')
    #     bridge_remappings.append(("/model/ur3e/gripper/target_joint"+str(i),"/gripper/target_joint"+str(i)))
    
    # print(bridge_arguments)
    # print(bridge_remappings)

    ros_ign_bridge = Node(package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/ur3/gripper/cmd@std_msgs/msg/Int32@ignition.msgs.Int32',
                '/model/ur3/gripper/target_joint@ros_gz_interfaces/msg/Float32Array@ignition.msgs.Float_V',
                ],
            remappings=[("/model/ur3/gripper/cmd","/gripper/cmd"),
                        ("/model/ur3/gripper/target_joint","/gripper/target_joint"),
                        ],
            output='screen'
    )

    ld.add_action(ros_ign_bridge)

    move_group=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_universal_robot_ign,"launch", "ur3_move_group_server.launch.py")
        ),
        # Simulation time does not function properly (as of Nov 2020), see https://github.com/AndrejOrsula/ign_moveit2/issues/4
        )
    
    ld.add_action(move_group)

    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_file(robot_sdf_path)
    urdf_generator.remove_joint('world_ur3_joint')
    robot_urdf_xml = urdf_generator.to_string()

    # robot_state_publisher = Node(package="robot_state_publisher",
    #          executable="robot_state_publisher",
    #          name="robot_state_publisher",
    #          parameters=[{'robot_description': robot_urdf_xml}],
    #         remappings=[("/joint_states","/ign_joint_states")],
    #          output="screen")
    
    # ld.add_action(robot_state_publisher)

    return ld

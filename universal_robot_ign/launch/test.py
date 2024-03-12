'''Launch ur10 ignition_simulator with ros joint position controller and state publisher'''

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


pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
pkg_universal_robot_ign = get_package_share_directory('universal_robot_ign')
#data
world_sdf_path=os.path.join(pkg_universal_robot_ign, 'resource', 'worlds', 'test_world.sdf') 
robot_xmacro_path=os.path.join(pkg_universal_robot_ign, 'resource', 'xmacro', 'ur10_robotiq140.sdf.xmacro') 
ign_config_path=os.path.join(pkg_universal_robot_ign, 'ign', 'gui.config')
# ignition_simulator launch

robot_macro = XMLMacro4sdf()
robot_macro.set_xml_file(robot_xmacro_path)
robot_macro.generate()
robot_xml = robot_macro.to_string()
robot_macro.to_file(os.path.join(pkg_universal_robot_ign, 'resource', 'models', 'ur10_robotiq140.sdf'))
#robot xml file save
with open(os.path.join(pkg_universal_robot_ign, 'resource', 'models', 'ur10_robotiq140.sdf'), 'w') as f:
    f.write(robot_xml)
    f.close()
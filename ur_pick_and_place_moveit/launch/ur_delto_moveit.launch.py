from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # ExecuteProcess(
        #     cmd=['ros2', 'launch', 'delto_3f_driver', 'ur_delto_3f_bringup.launch.py','launch_rviz:=false'],
        #     output='screen'
        # ),
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ur_robot_driver', 'ur3.launch.py', 'robot_ip:=192.168.0.150', 'launch_rviz:=false'],
            output='screen'
        ),
        # ExecuteProcess(
        #     cmd=['ros2', 'launch', 'ur_moveit_config', 'ur_moveit.launch.py', 'ur_type:=ur3', 'description_package:=delto_3f_description', 'moveit_config_package:=ur_delto_moveit_config'],
        #     output='screen'
        # ),

        ExecuteProcess(
            cmd=['ros2', 'launch', 'ur_moveit_config', 'ur_moveit.launch.py', 'ur_type:=ur3'],
            output='screen'
        ),

    ])
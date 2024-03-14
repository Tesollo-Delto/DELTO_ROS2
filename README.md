# Delto Gripper

<center><img src="https://cdn.imweb.me/thumbnail/20240102/cc620fcfefe42.png" style="width: 80%;"/></center>

This is Delto Gripper ROS 2 package.

Some of the new features are enabled by ROS2 and include 
decreased latency, improved security, and more flexibility regarding middleware configuration. 


Check also [Delto Gripper Manual](https://www.tesollo.com/Community/?q=YToyOntzOjEyOiJrZXl3b3JkX3R5cGUiO3M6MzoiYWxsIjtzOjQ6InBhZ2UiO2k6MTt9&bmode=view&idx=18137982&t=board).


## Build Status

<table width="100%">
  <tr>
    <th>ROS Distro</th>
    <th>ROS2 (22.04)</th>
  </tr>
  <tr>
   <th> Branch </th>
   <th>  Humble </th> 
  </tr>
</table>




## Packages in the Repository:

  - `delto_3f_description` - URDF, mesh file

  - `delto_3f_driver` - Delto Gripper ROS2 driver

  - `delto_3f_moveit` - example MoveIt configuration for Delto Gripper.

  - `universal_robot_ign` - example Gazebo Ignition (Simulation) for Delto, UR3.

  - `ur_pick_and_place_moveit` - example MoveIt! pick and place application for Delto Gripper and UR3.

Some physical measurements (like PID Gain, inertia) may not be accurate. Adjustments may be necessary for a perfect simulation or operation.



## Getting Started


1. **ROS Install** 

- Check this  [ROS 2 Humble installation site](https://docs.ros.org/en/humble/Installation.html).

  
2. **Gazebo Ign Install**

- Check this  [Gazebo installation site](https://gazebosim.org/docs/latest/ros_installation).
```
sudo apt-get install ros-humble-ros-gz

```
3. **Moveit Install**
```
sudo apt install ros-humble-moveit
```

3. **Create a new ROS2 workspace**:

  ```bash
  
  cd your_ros2_ws/src
  git clone https://github.com/Tesollo-Delto/DELTO_ROS2
  coclon build 
  ```

## How to use Delto Gripper

Delto_3f_driver subscribes to the topic /gripper_cmd and /target_joint.

- Check this [Delto Driver](https://github.com/Tesollo-Delto/DELTO_ROS2/tree/main/delto_3f_driver)

  
## How to Connect Delto Gripper with Robot Arm

 **Connecting Gripper URDF file with robot arm**

make new joint (robot_arm_end_link - gripper_base_link)


**example Delto with robot-arm (Universal Robot)

```xacro

...

</joint>
  <joint name="ee_joint" type="fixed">
    <origin rpy="0 1.57 0" xyz="0 0 0"/>
    <parent link="wrist_3_link"/>
    <child link="delto_base_link"/>
    <!-- <axis xyz="0.0 1.0 0.0"/> -->
 </joint>
 
 ...
  
```
## visualizing the robot arm with delto gripper

```bash
ros2 launch delto_3f_description display.launch.py
```


## Ignition Gazebo Simulation

```bash
ros2 launch inivation_gazebo delto_3f_ign.launch.py
```

## MoveIt! ur3 with Delto Gripper

It is a simple example of how to use the MoveIt! with Delto Gripper and ur3 (robot arm).

before run the moveit, you should run the delto driver, ur_robot_driver.


```bash
ros2 launch delto_3f_driver ur_delto_bringup.launch.py delto_ip:=X.X.X.X
```
open your robot driver launch file and change the robot_ip to your robot ip address.

```bash
ros2 launch ur_robot_driver ur3.launch.py
  robot_ip:=x.x.x.x
```

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 description_package:=delto_3f_description moveit_config_package:=ur_delto_moveit_config
```

```bash
ros2 launch ur_pick_and_place pick_and_place.launch.py
```

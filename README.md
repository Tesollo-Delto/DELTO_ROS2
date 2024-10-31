# Delto Gripper

<center><img src="https://cdn.imweb.me/thumbnail/20240102/cc620fcfefe42.png" style="width: 80%;"/></center>

<center><img src="https://cdn.imweb.me/thumbnail/20240102/4404e0bb0eae6.png" style="width: 80%;"/></center>

This is Delto Gripper ROS 2 package.

Some of the new features are enabled by ROS2 and include 
decreased latency, improved security, and more flexibility regarding middleware configuration. 


Check also [Delto 3F Gripper](https://en.tesollo.com/DG-3F)


Check also [Delto 2F Gripper](https://en.tesollo.com/DG-2F)



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

  - `delto_3f_driver` - Delto Gripper 3F ROS2 driver

  - `delto_3f_moveit` - example MoveIt configuration for Delto Gripper.

  - `delto_2f_driver` - Delto Gripper 2F ROS2 driver

Some physical measurements (like PID Gain, inertia) may not be accurate. Adjustments may be necessary for a perfect simulation or operation.



## Getting Started


1. **ROS2 Install** 

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


[![DG3F](https://img.youtube.com/vi/cFbdHVstmg4/0.jpg)](https://www.youtube.com/watch?v=cFbdHVstmg4)

  
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
## visualizing delto gripper

```bash
ros2 launch delto_description dg3f_display.launch.py
```

```bash
ros2 launch delto_description dg2f_display.launch.py
```

```bash
ros2 launch delto_description dg2f_display.launch.py
```

```bash
ros2 launch delto_3f_driver delto_3f_bringup.launch.py delto_ip:=192.168.0.100 delto_port:=502
```

## Feature Requests and Bug Reports

If you have any feature requests, modification suggestions, or encounter any bugs, please create an issue on GitHub or contact the maintainer at khc@tesollo.com.


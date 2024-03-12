#!/usr/bin/env python3

import threading
import copy
import rclpy
from moveit2 import MoveIt2Interface
from ros_gz_interfaces.msg import Float32Array
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Pose , Point
import time
from math import sin, cos, pi


#refer https://index.ros.org/doc/ros2/Tutorials/URDF/Using-URDF-with-Robot-State-Publisher/
def euler_to_quaternion(roll, pitch, yaw):
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    # pub = rclpy.create_publisher(Int32, '/gripper', 10)
    pub_node = rclpy.create_node('gripper_test')
    gripper_grasp_pub = pub_node.create_publisher(Int32, '/gripper/cmd', 10)
    gripper_pose_pub = pub_node.create_publisher(Float32Array, '/gripper/target_joint', 10)
    # Initialise MoveIt2
    moveit2 = MoveIt2Interface()

    # Spin MoveIt2 node in the background
    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(moveit2)
    # executor.add_node(pub_node)
    msg=Int32()

    #gripper initial position
    gripper_pose= Float32Array()
    data = [0.0, 0.0, 50.0, 80.0,
                         -30.0, -0.0, 50.0, 80.0,
                         30.0, 0.0, 50.0, 80.0]

    gripper_pose.data = [degree*pi/180 for degree in data]
    gripper_pose_pub.publish(gripper_pose)
    time.sleep(1)

    
    thread = threading.Thread(target=executor.spin)
    thread.start()

    joint_positions=[11.63 * pi/180, -67.16 * pi/180, 108.88 * pi/180,
                     -134.13 * pi/180, -90.80*pi / 180 , 5.43 * pi/180] 
    moveit2.set_joint_goal(joint_positions)
    # Plan and execute
    res = moveit2.plan_kinematic_path()
    # pose = moveit2.compute_fk(fk_link_names=['ee_link'])
    # print(pose)
    moveit2.execute()
    # moveit2.robot_move()
    print("joint goal1")
    time.sleep(5)
    


    response = moveit2.compute_fk(fk_link_names=['ee_link'])
    pose1 = response.pose_stamped[0].pose
    pose2 = copy.deepcopy(pose1) 
    pose2.position.z -= 0.2

    pose1.orientation = Quaternion(x=0.0, y=1.0,z=0.0, w=0.0)
    pose2.orientation = Quaternion(x=0.0, y=1.0,z=0.0, w=0.0)
    
    open = 0
    grasp = 1

    while(1): 
       
        msg.data = open
        gripper_grasp_pub.publish(msg)
        time.sleep(0.1)

        # position2 = [pose2.position.x, pose2.position.y, pose2.position.z]
        # moveit2.set_pose_goal(position2, pose2.orientation)

        response = moveit2.compute_fk(fk_link_names=['ee_link'])
        pose1 = response.pose_stamped[0].pose
        pose1.orientation = Quaternion(x=0.0, y=1.0,z=0.0, w=0.0)
        pose2 = copy.deepcopy(pose1) 
        pose2.position.z -= 0.15
        pose1.orientation = Quaternion(x=0.0, y=1.0,z=0.0, w=0.0)


        # moveit2.plan_kinematic_path()
        # moveit2.execute()

        # Create a list of Pose objects
        waypoints = [pose1, pose2]

        joint=[joint for joint in moveit2.get_joint_state().position]
        moveit2.plan_cartesian_path(waypoints, joint)
        moveit2.execute()
        moveit2.robot_move()
        print("cartesian path")

        time.sleep(5)
        msg.data = grasp
        gripper_grasp_pub.publish(msg)
        time.sleep(1)

        response = moveit2.compute_fk(fk_link_names=['ee_link'])
        pose1 = response.pose_stamped[0].pose
        pose1.orientation = Quaternion(x=0.0, y=1.0,z=0.0, w=0.0)
        pose2 = copy.deepcopy(pose1) 
        pose2.position.z += 0.15
        pose1.orientation = Quaternion(x=0.0, y=1.0,z=0.0, w=0.0)

        # # print(response.pose_stamped[0].pose)

        # position2 = [pose2.position.x, pose2.position.y, pose2.position.z]
        # moveit2.set_pose_goal(position2, pose2.orientation)

        # moveit2.plan_kinematic_path()
        # moveit2.execute()

        waypoints = [pose2, pose1]
        joint=[joint for joint in moveit2.get_joint_state().position]
        moveit2.plan_cartesian_path(waypoints, joint)
        moveit2.execute()

        # moveit2.robot_move()   
        time.sleep(5)
        msg.data = open
        gripper_grasp_pub.publish(msg)
        time.sleep(1)
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
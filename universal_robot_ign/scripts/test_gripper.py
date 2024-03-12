#!/usr/bin/python3
import sys

import rclpy
import math
import time
from std_msgs.msg import Int32,Float32MultiArray
from ros_gz_interfaces.msg import Float32Array
# from control_msgs.msg import FollowJointTrajectory

input_msg='''
#0 : open gripper.
#1 : close gripper to grasp.
#2 : close gripper to grasp2.
#3 : close gripper to grasp3.

input:
'''

def main():
    rclpy.init()
    node = rclpy.create_node('gripper_test')
    pub = node.create_publisher(Int32, '/gripper/cmd', 10)
    gripper_pose_pub = node.create_publisher(Float32Array, '/gripper/target_joint', 10)    

    # while(rclpy.ok()):
        # start detect
        # mode = input(input_msg)
        # mode = int(mode)
        # msg=Int32()
    
    gripper_pose= Float32Array()
    data = [0.0, 0.0, 50.0, 80.0,
                         0.0, 0.0, 50.0, 80.0,
                         0.0, 0.0, 50.0, 80.0]

    gripper_pose.data = [degree*math.pi/180 for degree in data]
    gripper_pose_pub.publish(gripper_pose)

    time.sleep(1)

        
    gripper_pose= Float32Array()
    data = [0.0, 0.0, 40.0, 90.0,
                         -30.0, 0.0, 40.0, 90.0,
                         30.0, 0.0, 40.0, 90.0]

    gripper_pose.data = [degree*math.pi/180 for degree in data]
    gripper_pose_pub.publish(gripper_pose)

    time.sleep(1)

    gripper_pose= Float32Array()
    data = [0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0]

    gripper_pose.data = [degree*math.pi/180 for degree in data]
    gripper_pose_pub.publish(gripper_pose)

    time.sleep(1)
    
    # while(rclpy.ok()):
    #     # start detect
    #     mode = input(input_msg)
    #     mode = int(mode)
    #     msg=Int32()
    #     if mode == 0:
    #         msg.data = 0
    #     elif mode == 1:
    #         msg.data = 1
    #     elif mode == 2:
    #         msg.data = 2
    #     elif mode == 3:
    #         msg.data = 3
    #     else:
    #         print("invalid input")
    #         continue
        
    #     pub.publish(msg)
        
if __name__ == '__main__':
    main()
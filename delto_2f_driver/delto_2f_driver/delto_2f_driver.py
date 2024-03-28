#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import threading
import sys
import os

import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Int32
from ros_gz_interfaces.msg import Float32Array

from sensor_msgs.msg import JointState

import rclpy

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from delto_utility import delto_2f_modbus_TCP as delto_TCP

class DeltoROSDriver(Node):

    def __init__(self):

        # ROS2 Node Initialize
        super().__init__('delto_2f_driver')

        qos_profile = QoSProfile( depth = 2, reliability = QoSReliabilityPolicy.RELIABLE)

        self.declare_parameter('ip', "192.254.186.62") #192.254.186.62
        self.declare_parameter('port', 10000)
        self.declare_parameter('slaveID', 0)
        self.declare_parameter('dummy', False)

        self.current_position = 0
        self.electric_current = 0
        self.low_force = 0
        self.high_force = 0

        self.delto_client = delto_TCP.Communication()
        self.stop_thread = False

        self.lock = threading.Lock()

        #Too high frequency will cause blocking sub/pub 
        self.publish_rate = 20

        self.jointstate_pub = self.create_publisher(JointState,'gripper/joint_states',qos_profile)
        # self.command_sub = self.create_subscription(Float32Array,'gripper/command',self.command_callback, qos_profile)

        self.low_force_sub = self.create_subscription(Int32,'gripper/low_force',self.low_force_callback,qos_profile)
        self.high_force_sub = self.create_subscription(Int32,'gripper/high_force',self.high_force_callback,qos_profile)

        self.open_position_sub = self.create_subscription(Int32,'gripper/open_position',self.open_position_callback,qos_profile)
        self.close_position_sub = self.create_subscription(Int32,'gripper/close_position',self.close_position_callback,qos_profile)
        
        self.low_force_pub = self.create_publisher(Int32,'gripper/status/low_force',qos_profile)
        self.high_force_pub = self.create_publisher(Int32,'gripper/status/high_force',qos_profile)
        self.open_position_pub = self.create_publisher(Int32,'gripper/status/open_position',qos_profile)
        self.close_position_pub = self.create_publisher(Int32,'gripper/status/close_position',qos_profile)

        self.grasp_sub = self.create_subscription(Int32,'gripper/grasp',self.grasp_callback,qos_profile)
        
        self.get_data_timer = self.create_timer(1.0 / self.publish_rate, self.get_data_timer_callback)

        self.joint_state_msg = JointState()
    
    # Connect to the delto gripper
    def connect(self)->bool:

        print("Connecting to the delto gripper...")
        return self.delto_client.connect(self.get_parameter('ip').value,
                            self.get_parameter('port').value,
                             self.get_parameter('slaveID').value)
        

    # Publish joint state
    def joint_state_publisher(self):

        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        self.joint_state_msg.name = ['Distance']

        self.joint_state_msg.position = [self.current_position * 0.001]

        self.jointstate_pub.publish(self.joint_state_msg)

    # Get current position
    def get_open_position(self):       
        return self.open_position
    
    def get_close_position(self):
       return self.close_position 
    
    def get_current_position(self):
        return self.current_position
    
    def get_electric_current(self):
        return self.electric_current

    def set_ip(self, ip):
        self.delto_client.set_ip(ip)

    def grasp(self, is_grasp:int):
        self.delto_client.grasp(bool(is_grasp));

    def set_open_position(self, position):
       
         self.open_position = position
         self.delto_client.set_open_position(self.open_position)

    def set_close_position(self, position):
       
        self.close_position = position
        self.delto_client.set_close_position(self.close_position)

    def set_high_force(self, force):
        self.high_force = force
        self.delto_client.set_high_force(self.high_force)
        

    def set_low_force(self, force):
        self.low_force = force
        self.delto_client.set_low_force(self.low_force)


    def high_force_callback(self,msg):

        self.set_high_force(msg.data)

    def low_force_callback(self,msg):

        self.set_low_force(msg.data)

    def open_position_callback(self,msg):
        
        self.set_open_position(msg.data)

    def close_position_callback(self,msg):
            
        self.set_close_position(msg.data)

    def grasp_callback(self,msg):
            
            self.grasp(msg.data)

    def get_data_timer_callback(self):
            
    
            self.lock.acquire()
            data = self.delto_client.get_data()
            self.lock.release()
  
            # if len(data) != 5:
            #     return
            
            self.current_position = float(data[0])
            self.electric_current = float(data[1])

            self.high_force = self.delto_client.get_high_force()
            self.low_force = self.delto_client.get_low_force()
            # self.low_force = data[3]

            force_msg =Int32()
            force_msg.data = self.low_force
            self.low_force_pub.publish(force_msg)
            force_msg.data = self.high_force
            self.high_force_pub.publish(force_msg)

            self.joint_state_publisher()

def main(args=None):
    
    rclpy.init(args=args)
    
    delto_driver = DeltoROSDriver()
    connect = delto_driver.connect()

    if connect == False:
        delto_driver.get_logger().error("network connection failed.")
        return
    
    time.sleep(0.1)

    delto_driver.get_logger().info("delto_2f_driver initialized")
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(delto_driver)
    executor.spin()

    executor.shutdown()

if __name__ == '__main__':
    main()
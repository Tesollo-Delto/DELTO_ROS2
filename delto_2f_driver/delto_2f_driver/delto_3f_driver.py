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
from delto_utility import delto_modbus_TCP as delto_TCP

class DeltoROSDriver(Node):

    def __init__(self):

        # ROS2 Node Initialize
        super().__init__('delto_3f_driver')

        qos_profile = QoSProfile( depth = 2, reliability = QoSReliabilityPolicy.RELIABLE)

        self.declare_parameter('ip', "169.254.186.62")
        self.declare_parameter('port', 10000)
        self.declare_parameter('slaveID', 1)
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

        self.position_pub = self.create_publisher(JointState,'gripper/joint_states',qos_profile)
        self.command_sub = self.create_subscription(Float32Array,'gripper/command',self.command_callback, qos_profile)

        self.low_force_sub = self.create_subscription(Int32,'gripper/low_force',self.low_force_callback,qos_profile)
        self.high_force_sub = self.create_subscription(Int32,'gripper/high_force',self.high_force_callback,qos_profile)

        self.open_position_sub = self.create_subscription(Int32,'gripper/open_position',self.open_position_callback,qos_profile)
        self.close_position_sub = self.create_subscription(Int32,'gripper/close_position',self.close_position_callback,qos_profile)

        self.get_data_timer = self.create_timer(1.0 / self.publish_rate, self.get_data)

    
    # Connect to the delto gripper
    def connect(self)->bool:

        if self.is_dummy:
            print("Dummy mode")
            return True
        
        print("Connecting to the delto gripper...")
        return self.delto_client.connect(self.get_parameter('ip').value,
                            self.get_parameter('port').value,
                             self.get_parameter('slaveID').value)
        

    # Publish joint state
    def joint_state_publisher(self):

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        joint_state_msg.name = ['Distance']

        position = self.get_position()

        joint_state_msg.position = position

        self.joint_state_pub.publish(joint_state_msg)

    # Get current position
    def get_open_position(self):

       pass
    
    def get_close_position(self):

        if self.is_dummy:
            return self.current_joint_state
        
        pass
    
    def set_open_position(self, position):
       
        pass

    def set_close_position(self, position):
       
        pass


    def high_force_callback(self,msg):

        self.high_force = msg.data 

    def low_force_callback(self,msg):

        self.low_force = msg.data
        pass

    def open_position_callback(self,msg):
        
        self.open_position = msg.data
        pass

    def close_position_callback(self,msg):
            
        self.close_position = msg.data
        pass

def main(args=None):
    rclpy.init(args=args)
    
    delto_driver = DeltoROSDriver()
    connect = delto_driver.connect()

    if connect == False:
        delto_driver.get_logger().error("network connection failed.")
        return
    
    if not(delto_driver.is_dummy):  
        delto_driver.delto_client.SetFree(False)

    time.sleep(0.1)

    delto_driver.get_logger().info("delto_driver initialized")
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(delto_driver)
    executor.spin()

    executor.shutdown()

if __name__ == '__main__':
    main()
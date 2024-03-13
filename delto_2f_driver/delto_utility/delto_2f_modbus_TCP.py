from pymodbus.client.tcp import ModbusTcpClient 
import rclpy
import sys
import os
import threading
import struct

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from delto_2f_driver.delto_utility.delto_2f_enum import Delto2FCoils, Delto2FHoldingRegisters, Delto2FInputRegisters    

'''
default values
IP = '169.254.186.72'
PORT = 10000
SUBNET_MASK = '255.255.255.0'
DEFAULT_GATEWAY = '192.168.1.1'
'''

class Communication:
    '''
    Communication sends command and receives data from Delto Gripper
    '''
    
    def __init__(self, dummy=False):
        self.client = None
        self.slaveID = 0
        self.dummy = dummy
        self.lock = threading.Lock()

    def __del__(self):
        self.disconnect()

    def connect(self, ip, port, slaveID = 1):
        '''
        Connect to Delto Gripper
        '''

        self.slaveID = slaveID
        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)
            
            return
        
        self.client = ModbusTcpClient(ip, port)
        return self.client.connect()
    
    def disconnect(self):
        '''
        Disconnect from Delto Gripper
        '''
        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)            
            return
        
        self.client.close()

    def send_data(self,address,count,data):

        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)          
            return
        
        if data != []:
            with self.lock:
                #singe write register
                if(count == 1):
                    self.client.write_register(address = address, count = count, values = data)
                #multiple write register
                elif(count > 1):
                    self.client.write_registers(address = address, count = count, values = data)

    def get_data(self,address,count):
        '''
        Get data from Delto Gripper
        '''
        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)            
            return
        

        ## Position and Current
        status = self.client.read_input_registers(
            address = Delto2FInputRegisters.CURRENT_POSITION, count = 2).registers
        
        return status
    
    def set_open_position(self, value):

        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)            
            return
        
        self.send_data(Delto2FHoldingRegisters.OPEN_POSITION, 1, [value]) 

    def set_close_position(self, value):
            
            if self.dummy:
                rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                            ": " +
                                            sys._getframe().f_code.co_name)            
                return
            
            self.send_data(Delto2FHoldingRegisters.CLOSE_POSITION, 1, [value])

    def set_low_force(self, value):

        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)            
            return
        
        self.send_data(Delto2FHoldingRegisters.LOW_FORCE, 1, [value])

    def set_high_force(self, value):

        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)            
            return
        
        self.send_data(Delto2FHoldingRegisters.HIGH_FORCE, 1, [value])

    def set_ip(self,ip :str):

        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)            
            return
        
        ip = ip.split('.')

        if len(ip) != 4:

            rclpy.Node.get_logger().error(rclpy.Node.get_name() +
                                          ": " +
                                          "Invalid IP address")
            return
        
        ip = list(map(int,ip))
        self.send_data(Delto2FHoldingRegisters.ETHERNET_IP_CLASS_A, 4, ip)

    def set_subnet_mask(self,subnet_mask :str):

        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": "
                                         sys._getframe().f_code.co_name)            
            return
        
        subnet_mask = subnet_mask.split('.')

        if len(subnet_mask) != 4:

            rclpy.Node.get_logger().error(rclpy.Node.get_name() +
                                          ": " +
                                          "Invalid subnet mask")
            return
        
        subnet_mask = list(map(int,subnet_mask))
        self.send_data(Delto2FHoldingRegisters.ETHERNET_SUBNET_MASK_A, 4, subnet_mask)

    def set_gate_way(self,gateway :str):

        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)            
            return
        
        gateway = gateway.split('.')

        if len(gateway) != 4:

            rclpy.Node.get_logger().error(rclpy.Node.get_name() +
                                          ": " +
                                          "Invalid gateway")
            return
        
        gateway = list(map(int,gateway))
        self.send_data(Delto2FHoldingRegisters.ETHERNET_GATEWAY_A, 4, gateway)
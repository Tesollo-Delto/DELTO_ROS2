from pymodbus.client.tcp import ModbusTcpClient 
# from pymodbus.diag_message import DiagnosticRequest
import rclpy
import sys
import os
import threading
import struct

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from delto_utility.delto_2f_enum import Delto2FCoils, Delto2FHoldingRegisters, Delto2FInputRegisters    

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
        self.slaveID = 1
        self.dummy = dummy
        self.lock = threading.Lock()

    def __del__(self):
        self.disconnect()

    def connect(self, ip, port, slaveID = None):
        '''
        Connect to Delto Gripper
        '''
        if slaveID is None:
            self.slaveID = slaveID
       
        
        self.client = ModbusTcpClient(host=ip, port=port)
        return self.client.connect()
    
    def disconnect(self):
        '''
        Disconnect from Delto Gripper
        '''
        if self.client is not None:
            self.client.close()
    
    def send_data(self,address,count,data,slave=None):

        if data != []:
            with self.lock:
                if(count >= 1):
                    self.client.write_registers(address = address, values = data, slave=slave)

    def get_data(self):
        '''
        Get data from Delto Gripper
        '''
        address_ =Delto2FInputRegisters.CURRENT_POSITION.value
        ## Position and Current
        status = self.client.read_input_registers(
            address = address_, count = 2, slave=self.slaveID).registers

        return status
    
    def set_open_position(self, value):

        
        self.send_data(Delto2FHoldingRegisters.OPEN_POSITION.value, 1, [value],slave=self.slaveID) 

    def set_close_position(self, value):
            
            
            self.send_data(Delto2FHoldingRegisters.CLOSE_POSITION.value, 1, [value],slave=self.slaveID)

    def set_low_force(self, value):

        
        self.send_data(Delto2FHoldingRegisters.LOW_FORCE.value, 1, [value],slave=self.slaveID)

    def set_high_force(self, value):

        response = self.client.write_registers(Delto2FHoldingRegisters.HIGH_FORCE.value,values= [value], slave=self.slaveID)
        if response.isError():
            print("Failed to set high force")
        # # else:
        # self.send_data(Delto2FHoldingRegisters.HIGH_FORCE.value, 1, [value],slave=self.slaveID)

    def get_high_force(self):
            
            return self.client.read_holding_registers(Delto2FHoldingRegisters.HIGH_FORCE.value,1,slave=self.slaveID).registers[0]
        
    def get_low_force(self):
                
                return self.client.read_holding_registers(Delto2FHoldingRegisters.LOW_FORCE.value,1,slave=self.slaveID).registers[0]
    
    def set_ip(self,ip :str):


        ip = ip.split('.')

        if len(ip) != 4:

            rclpy.Node.get_logger().error(rclpy.Node.get_name() +
                                          ": " +
                                          "Invalid IP address")
            return
        
        ip = list(map(int,ip))
        self.send_data(Delto2FHoldingRegisters.ETHERNET_IP_CLASS_A.value, 4, ip,slave=self.slaveID)

    def set_subnet_mask(self,subnet_mask :str):

        #TO DO
        # if self.dummy:
        #     rclpy.Node.get_logger().info(rclpy.Node.get_name() +
        #                                  ": "
        #                                  sys._getframe().f_code.co_name)            
            # return
        
        subnet_mask = subnet_mask.split('.')

        if len(subnet_mask) != 4:

            rclpy.Node.get_logger().error(rclpy.Node.get_name() +
                                          ": " +
                                          "Invalid subnet mask")
            return
        
        subnet_mask = list(map(int,subnet_mask))
        self.send_data(Delto2FHoldingRegisters.ETHERNET_SUBNET_MASK_A.value, 4, subnet_mask,slave=self.slaveID)

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
        self.send_data(Delto2FHoldingRegisters.ETHERNET_GATEWAY_A.value, 4, gateway,)

    def grasp(self, is_grasp: bool):

        self.lock = threading.Lock();
        
        with self.lock:
            # self.client.write_coil(Delto2FCoils.REVERSE_MODE.value,
            #                         value= True,
            #                         slave=1)
            response= self.client.write_coil(Delto2FCoils.GRASP.value,
                                    value= is_grasp,
                                    slave=self.slaveID)
            if response.isError():
                print("Failed to send TCP grasp command")

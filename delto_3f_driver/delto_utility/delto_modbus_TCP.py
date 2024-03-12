from pymodbus.client.tcp import ModbusTcpClient 
import rclpy
import sys
import os
import threading
import struct

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from delto_utility.delto_3f_enum import Delto3F, Delto3FCoils, Delto3FHoldingRegisters, Delto3FInputRegisters

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

    def sendData(self,address,count,data):

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

    def getPosition(self):
        
        if self.dummy:
            status = [0]*12
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)
            
            return status
        
        status =[]

        for i in range(Delto3F.MOTOR_NUM.value):
            stats = self.client.read_input_registers(
                address = Delto3FInputRegisters.MOTOR1_CURRENT_POSITION.value + i,
                count=1,
                slave=self.slaveID).registers
            
            #unsigned 8bit to singed 8 bit
            status.append(struct.unpack('h', struct.pack('H',stats[0]))[0]/10)

        return status
    
    def SetPosition(self,position : list[float]):

        if(self.dummy):
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)
            
            return
        
        if(len(position) != 12):
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name +
                                         " position size is not 12")
            return
        
        with self.lock:

            intPosion = list(map(lambda x : struct.unpack('H', struct.pack('h',int((x*10))))[0],position))
            self.client.write_registers(address = 72, values = intPosion,slave=self.slaveID)

    def GetPGain(self):
        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)
            
            return
        
        with self.lock:
            pGain = self.client.read_holding_registers(address=Delto3FHoldingRegisters.MOTOR1_PGAIN.value,
                                                        count= Delto3F.MOTOR_NUM.value,
                                                        slave=self.slaveID).registers
        return pGain
    
    def SetPGain(self, pGain : list[int]):
        print("setPGain", pGain)
        self.client.write_registers(address=Delto3FHoldingRegisters.MOTOR1_PGAIN.value,
                                    values=pGain,slave=self.slaveID)

    def GetDGain(self):
        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)
            
            return
        
        with self.lock:
            dGain = self.client.read_holding_registers(address=Delto3FHoldingRegisters.MOTOR1_DGAIN.value,
                                                        count= Delto3F.MOTOR_NUM.value, 
                                                        slave=self.slaveID).registers
        return dGain
    
    def SetDGain(self, dGain: list[int]):
        self.client.write_registers(address= Delto3FHoldingRegisters.MOTOR1_DGAIN,
                                     values= dGain,
                                     slave=self.slaveID)
        
    def SetFree(self, isFree : bool):
        
        self.client.write_coil(address = Delto3FCoils.JOINT_CUSTOM_MODE.value
                               ,value = isFree
                               ,slave = self.slaveID)

    def GraspMode(self,mode):
        if(mode == 0):
            self.Grasp(False)
        else:
            self.client.write_register(address=int(Delto3FHoldingRegisters.GRASP_MODE.value),
                                    value=mode,
                                    slave=self.slaveID)
            
            self.Grasp(True)

    def GetGraspMode(self):

        with self.lock:
            mode = self.client.read_input_registers(address=Delto3FHoldingRegisters.GRASP_MODE.value,
                                                     count=1,
                                                     slave=self.slaveID).registers
        
        return mode

    def Grasp(self,isGrasp : bool):
        with self.lock:
            print("Grasp", isGrasp)
            self.client.write_coil(address=Delto3FCoils.GRASP.value,
                                value=isGrasp,
                                slave=self.slaveID)
        
    def SetStep(self,step):
        
        if(step < 1):
            step = 1
        elif(step > 32767):
            step = 32767

        self.client.write_register(address = Delto3FHoldingRegisters.MOTION_STEP.value,
                                    value = step, slave = self.slaveID)

    def test(self):
        self.client.write_register(address=Delto3FHoldingRegisters.ETHERNET_GATEWAY_C.value,
                                   value=0,
                                   slave=self.slaveID)
        self.RomWrite()

    def RomWrite(self):
        self.client.write_coil(address = Delto3FCoils.EEPROM_WRITE.value,
                               value = True,
                               slave = self.slaveID)
        


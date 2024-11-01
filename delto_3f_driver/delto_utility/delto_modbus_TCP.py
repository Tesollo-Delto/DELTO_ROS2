from delto_utility.delto_3f_enum import Delto3F, Delto3FCoils, Delto3FHoldingRegisters, Delto3FInputRegisters
from pymodbus.client.tcp import ModbusTcpClient
import rclpy
import sys
import os
import threading
import struct

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


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

    def connect(self, ip, port, slaveID=1):
        '''
        Connect to Delto Gripper
        '''

        self.slaveID = slaveID
        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)

            return

        self.client = ModbusTcpClient(host=ip, port=port)
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

        # self.client.close()

    def get_position(self):

        if self.dummy:
            status = [0]*12
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)

            return status

        #status = []
        status = self.client.read_input_registers(
            address=Delto3FInputRegisters.MOTOR1_CURRENT_POSITION.value,
            count=Delto3F.MOTOR_NUM.value,
            slave=self.slaveID).registers
                
        for i in range(Delto3F.MOTOR_NUM.value):
            # stats = self.client.read_input_registers(
            #     address=Delto3FInputRegisters.MOTOR1_CURRENT_POSITION.value + i,
            #     count=1,
            #     slave=self.slaveID).registers
            status[i] = (status[i] if status[i] < 32768 else status[i] - 65536)/10.0
            # unsigned 8bit to singed 8 bit
            # stats = (stats[0] if stats[0] < 32768 else stats[0] - 65536)/10.0
            # status.append(struct.unpack('h', struct.pack('H', stats[0]))[0]/10)
            # status.append(stats)
        return status

    def get_high_force(self):

        with self.lock:
            high_force = self.client.read_input_registers(address=Delto3FInputRegisters.HIGH_FORCE.value,
                                                          count=1,
                                                          slave=self.slaveID).registers
        return high_force[0]

    def get_low_force(self):

        with self.lock:
            low_force = self.client.read_input_registers(address=Delto3FInputRegisters.LOW_FORCE.value,
                                                         count=1,
                                                         slave=self.slaveID).registers
        return low_force[0]

    def set_position(self, position: list[float]):

        if (self.dummy):
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)

            return

        if (len(position) != 12):
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name +
                                         " position size is not 12")
            return

        with self.lock:

            intPosion = list(map(lambda x: struct.unpack(
                'H', struct.pack('h', int((x*10))))[0], position))
            self.client.write_registers(
                address=72, values=intPosion, slave=self.slaveID)

    def get_pgain(self):
        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)

            return

        with self.lock:
            pGain = self.client.read_holding_registers(address=Delto3FHoldingRegisters.MOTOR1_PGAIN.value,
                                                       count=Delto3F.MOTOR_NUM.value,
                                                       slave=self.slaveID).registers
        return pGain

    def set_pgain(self, pGain: list[int]):
        print("setPGain", pGain)
        self.client.write_registers(address=Delto3FHoldingRegisters.MOTOR1_PGAIN.value,
                                    values=pGain, slave=self.slaveID)

    def get_dgain(self):
        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)

            return

        with self.lock:
            dGain = self.client.read_holding_registers(address=Delto3FHoldingRegisters.MOTOR1_DGAIN.value,
                                                       count=Delto3F.MOTOR_NUM.value,
                                                       slave=self.slaveID).registers
        return dGain

    def set_dgain(self, dGain: list[int]):
        self.client.write_registers(address=Delto3FHoldingRegisters.MOTOR1_DGAIN,
                                    values=dGain,
                                    slave=self.slaveID)

    def set_free(self, isFree: bool):

        self.client.write_coil(
            address=Delto3FCoils.JOINT_CUSTOM_MODE.value, value=isFree, slave=self.slaveID)

    def grasp_mode(self, mode):
        if (mode == 0):
            self.grasp(False)
        else:
            self.client.write_register(address=Delto3FHoldingRegisters.GRASP_MODE.value,
                                       value=mode,
                                       slave=self.slaveID)

            self.grasp(True)

    def get_grasp_mode(self):

        with self.lock:
            mode = self.client.read_input_registers(address=Delto3FHoldingRegisters.GRASP_MODE.value,
                                                    count=1,
                                                    slave=self.slaveID).registers

        return mode

    def grasp(self, isGrasp: bool):
        with self.lock:
            print("Grasp", isGrasp)
            self.client.write_coil(address=Delto3FCoils.GRASP.value,
                                   value=isGrasp,
                                   slave=self.slaveID)

    def set_step(self, step):

        if (step < 1):
            step = 1
        elif (step > 32767):
            step = 32767

        self.client.write_register(address=Delto3FHoldingRegisters.MOTION_STEP.value,
                                   value=step, slave=self.slaveID)

    def rom_write(self):
        self.client.write_coil(address=Delto3FCoils.EEPROM_WRITE.value,
                               value=True,
                               slave=self.slaveID)

    def set_ip(self, ip: str):

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

        ip = list(map(int, ip))

        self.client.write_registers(address=Delto3FHoldingRegisters.ETHERNET_IP_CLASS_A.value,
                                    values=ip,
                                    slave=self.slaveID)

    def set_subnet_mask(self, subnet_mask: str):

        if self.dummy:
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)
            return

        subnet_mask = subnet_mask.split('.')

        if len(subnet_mask) != 4:

            rclpy.Node.get_logger().error(rclpy.Node.get_name() +
                                          ": " +
                                          "Invalid subnet mask")
            return

        subnet_mask = list(map(int, subnet_mask))
        self.client.write_registers(Delto3FHoldingRegisters.ETHERNET_SUBNET_MASK_A.value,
                                    values=subnet_mask,
                                    slave=self.slaveID)

    def set_gate_way(self, gateway: str):

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

        gateway = list(map(int, gateway))
        self.client.write_registers(Delto3FHoldingRegisters.ETHERNET_GATEWAY_A.value,
                                    values=gateway,
                                    slave=self.slaveID)

    def fix_position(self, position: list):
        """_summary_
        accessible from firmware version 1.5 or higher
        Args:
            position (list): [0, 1] 0: free, 1: fix
        """
        if (self.dummy):
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name)
            return

        if (len(position) != 12):
            rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                         ": " +
                                         sys._getframe().f_code.co_name +
                                         " position size is not 12")
            return

        for i in list:
            if i != 0 or i != 1:
                rclpy.Node.get_logger().info(rclpy.Node.get_name() +
                                             ": " +
                                             sys._getframe().f_code.co_name +
                                             "value is not 0 or 1")
                return

        with self.lock:

            self.client.write_registers(
                address=Delto3FHoldingRegisters.MOTOR1_FIXED_POSITION.value, values=position, slave=self.slaveID)


'''
change ip Example 

    comm = Communication()
    comm.connect('169.254.186.72',10000)
    comm.set_ip('169.254.186.73')
    comm.rom_write()

    The changed IP is applied only after restarting the power.

'''

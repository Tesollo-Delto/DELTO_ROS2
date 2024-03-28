from enum import Enum

class GraspMode(Enum):
    """Enumeration of the grasp mode"""
    OPEN = 0
    GRASP = 1

class Delto2FCoils(Enum):
    """Enumeration of coils addresses of the Delto 2f"""
    REVERSE_MODE = 0
    GRASP = 1
    TORQUE = 2
    EEPROM_WRITE = 3

class Delto2FHoldingRegisters(Enum):
    """Enumeration of the holding registers addresses of the Delto 2f"""
    RS485_BAUDRATE = 0

    MODBUS_SLAVE_ID = 1

    ETHERNET_IP_CLASS_A = 2
    ETHERNET_IP_CLASS_B = 3
    ETHERNET_IP_CLASS_C = 4
    ETHERNET_IP_CLASS_D = 5

    ETHERNET_SUBNET_MASK_A = 6
    ETHERNET_SUBNET_MASK_B = 7
    ETHERNET_SUBNET_MASK_C = 8
    ETHERNET_SUBNET_MASK_D = 9

    ETHERNET_GATEWAY_A = 10
    ETHERNET_GATEWAY_B = 11
    ETHERNET_GATEWAY_C = 12
    ETHERNET_GATEWAY_D = 13

    ETHERNET_PORT = 14

    MOTOR_PGAIN = 15
    MOTOR_DGAIN = 16

    OPEN_POSITION = 18
    CLOSE_POSITION = 19
    LOW_FORCE = 20
    HIGH_FORCE = 21

   

class Delto2FInputRegisters(Enum):
    """Enumeration of the input registers addresses of the Delto 2f"""
    PRODUCT_ID = 0
    FIRMWARE_VERSION = 1

    CURRENT_POSITION = 2
    MOTOR_ELECTRIC_CURRENT = 3

    TARGET_POSITION_REACHED = 4
    TARGET_CURRENT_REACHED = 5
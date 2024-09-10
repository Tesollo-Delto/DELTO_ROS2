import socket
import numpy as np
import math
import crcmod.predefined

dg_ip = '169.254.186.72'
dg_port = 10000
crc16 = crcmod.predefined.mkCrcFun('crc-16')

q = [0]*12 
q_dot = [0]*12
q_d = [0]*12 #deired q 
pre_q = [0]*12 

#pd controller gain
kp = [1]*12
kd = [0]*12

tq_u = [0]*12
duty = [0]*12


tcp_data_send = bytearray(b'\x03\x28\x01\x00\x00\x02\x00\x00\x03\x00\x00\x04\x00\x00\x05\x00\x00\x06\x00\x00\x07\x00\x00\x08\x00\x00\x09\x00\x00\x0A\x00\x00\x0B\x00\x00\x0C\x00\x00')
tcp_crc_send = bytearray(b'\x50\xee')

class Communication:
    
    def __init__(self) -> None:
        self.delto_socket =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
    def connect(self,
                ip : str = '169.254.186.72',
                port : int = 10000):
        self.delto_socket.connect((ip, port))


    def get_position_rad(self):
        
        raw_finger_pos = [[0 for j in range(4)] for i in range(3)]

        self.delto_socket.send(b'\x01\x05\xEE\xD2\xDC') #CALL position, current position & current data
        msg = self.delto_socket.recv(1024)
        
        #(int(duty[i]) >> 8) & 0x00FF
    
        #finger-1
        raw_finger_pos[0][0] = np.int16(self._combine_msg(msg[3],msg[4]))/10
        raw_finger_pos[0][1] = np.int16(self._combine_msg(msg[8],msg[9]))/10
        raw_finger_pos[0][2] = np.int16(self._combine_msg(msg[13],msg[14]))/10
        raw_finger_pos[0][3] = np.int16(self._combine_msg(msg[18],msg[19]))/10

        #finger-2
        raw_finger_pos[1][0] = np.int16(self._combine_msg(msg[23],msg[24]))/10
        raw_finger_pos[1][1] = np.int16(self._combine_msg(msg[28],msg[29]))/10
        raw_finger_pos[1][2] = np.int16(self._combine_msg(msg[33],msg[34]))/10
        raw_finger_pos[1][3] = np.int16(self._combine_msg(msg[38],msg[39]))/10
       
        #finger-3
        raw_finger_pos[2][0] = np.int16(self._combine_msg(msg[43],msg[44]))/10
        raw_finger_pos[2][1] = np.int16(self._combine_msg(msg[48],msg[49]))/10
        raw_finger_pos[2][2] = np.int16(self._combine_msg(msg[53],msg[54]))/10
        raw_finger_pos[2][3] = np.int16(self._combine_msg(msg[58],msg[59]))/10

        #raw data(degree) to radian
        #raw_finger_pos = 16bit int -180~+180 (smallest unit 0.1)
        #radian(q) = 0 is zero position

        tmp_q = [None]*12
        for i in range(3):
            for j in range(4):
                #print(i,j)
                tmp_q[(i*4)+j] = ((raw_finger_pos[i][j]/180)*math.pi)

        return tmp_q

    def _combine_msg(self,data1, data2):
        combined_msgs = data1 << 8 | data2  
        if combined_msgs >= 0x8000:  
            combined_msgs -= 0x10000 
        return combined_msgs

    def send_duty(self,duty : list):

        for i in range(4) :

            tcp_data_send[i*3+3] = (int(duty[i]) >> 8) & 0x00FF
            tcp_data_send[i*3+4] = int(duty[i]) & 0x00FF

            tcp_data_send[i*3+15] = (int(duty[i+4]) >> 8) & 0x00FF
            tcp_data_send[i*3+16] = int(duty[i+4]) & 0x00FF

            tcp_data_send[i*3+27] = (int(duty[i+8]) >> 8) & 0x00FF
            tcp_data_send[i*3+28] = int(duty[i+8]) & 0x00FF  
                
        crc = crc16(tcp_data_send)

        tcp_crc_send[0] = crc & 0x00FF #crc_l
        tcp_crc_send[1] = (crc >> 8) & 0x00FF #crc_h
        
        self.delto_socket.send(tcp_data_send + tcp_crc_send)

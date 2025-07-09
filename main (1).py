import sys
import signal
import os
import time
import struct
import serial
import serial.tools.list_ports
from apscheduler.schedulers.blocking import BlockingScheduler
import socket
import struct
import select
import threading
from time import sleep

import math
from typing import Tuple, Optional
import numpy as np
from gym import core
import scipy.integrate
from ctrl import utils

import math
from typing import Any
from scipy.spatial.transform import Rotation
from numpy import sin, cos, arcsin, arctan2, pi, ndarray, dtype, bool_
from numpy.linalg import norm
from ctrl.ball import Ball
from ctrl.fsm import FSM
from ctrl.Controller import Controller

DEG = math.pi / 180

FRAME_HEAD = b'\xaa'    # 帧头
FRAME_TAIL = b'\x55'        # 帧尾
DATA_LENGTH = 14             # 数据长度（不含帧头帧尾）
TOTAL_FRAME_LENGTH = 16      # 总帧长 = 2(帧头) + 6(数据) + 1(帧尾)

current_state = {
    'w': np.zeros((3, 1)),  # 三轴角速度 [gx, gy, gz]
    'q': np.array([[1.0,0.0,0.0,0.0]]).T     # 姿态四元数 [qw, qx, qy, qz]
}

class SerialPort:
    def __init__(self, port=None, baudrate=115200):
        self.serial_port = None
        self.alive = False
        self.port = port
        self.baudrate = baudrate
        
    def start(self):
        """打开串口并开始监听"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1 
            )
            self.alive = True
            self.thread = threading.Thread(target=self._read_loop)
            self.thread.daemon = True
            self.thread.start()
            print(f"串口 {self.port} 已打开，开始监听...")
        except serial.SerialException as e:
            print(f"无法打开串口 {self.port}: {e}")

    def calculate_checksum(self,data):
        """计算数据的异或校验和"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum.to_bytes(1, byteorder='big')

    def generate_test_array(self,T1,T2,T3):
        test_array = bytearray(14)
        #print(T1,T2,T3)
        byte_T1 = struct.pack('<f', T1) 
        byte_T2 = struct.pack('<f', T2) 
        byte_T3 = struct.pack('<f', T3) 

        test_array[0] = 0xaa         
        test_array[1] = byte_T1[0]
        test_array[2] = byte_T1[1]
        test_array[3] = byte_T1[2]
        test_array[4] = byte_T1[3]
        test_array[5] = byte_T2[0]
        test_array[6] = byte_T2[1]
        test_array[7] = byte_T2[2]
        test_array[8] = byte_T2[3]
        test_array[9] = byte_T3[0]
        test_array[10] = byte_T3[1]
        test_array[11] = byte_T3[2]
        test_array[12] = byte_T3[3]
        test_array[13] = 0x55
        #print(test_array)
        return bytes(test_array)

    def _read_loop(self):
        global current_state
        received_buffer = b''  # 接收缓冲区    
        try:
            cnt = 0
            while True:
                # 延时以保证不会一直消耗CPU算力
                # 已确定帧率正常
                sleep(0.001)

                # 读取串口数据（非阻塞形式）
                if self.serial_port.in_waiting:

                    raw_data = self.serial_port.read(self.serial_port.in_waiting)
                    received_buffer += raw_data
                    #print(f"\n收到 {len(raw_data)} 字节，缓冲区总长度: {len(received_buffer)} 字节")
                    #print(f"原始数据: {raw_data.hex(' ')}")

                    # 持续解析，直到缓冲区数据不足一帧
                    while len(received_buffer) >= TOTAL_FRAME_LENGTH:
                        # 查找帧头
                        head_pos = received_buffer.find(FRAME_HEAD)

                        if head_pos == -1:
                            # 没找到帧头，丢弃全部数据
                            #print("未找到帧头，清空缓冲区")
                            received_buffer = b''
                            break

                        # 丢弃帧头前的数据
                        if head_pos > 0:
                            #print(f"丢弃帧头前的 {head_pos} 字节数据")
                            received_buffer = received_buffer[head_pos:]
                            continue

                        # 检查缓冲区是否包含完整帧
                        if len(received_buffer) < TOTAL_FRAME_LENGTH:
                            break

                        # 提取完整帧
                        current_frame = received_buffer[:TOTAL_FRAME_LENGTH]

                        # 提取数据部分
                        payload = current_frame[1:1+DATA_LENGTH]

                        # 提取帧尾
                        tail = current_frame[-1:]  # 帧尾是最后一个字节

                        # 验证帧尾
                        if tail != FRAME_TAIL:
                            #print(f"帧尾验证失败，期望: {FRAME_TAIL.hex()}, 实际: {tail.hex()}")
                            received_buffer = received_buffer[1:]  # 丢弃一个字节，尝试重新同步
                            continue

                        # 解析数据
                        q0 = ((payload[0] << 8) | payload[1])*0.001
                        q1 = ((payload[2] << 8) | payload[3])*0.001
                        q2 = ((payload[4] << 8) | payload[5])*0.001
                        q3 = ((payload[6] << 8) | payload[7])*0.001
                        gx = ((payload[8] << 8) | payload[9])*0.001
                        gy = ((payload[10] << 8) | payload[11])*0.001
                        gz = ((payload[12] << 8) | payload[13])*0.001

                        current_state['q'][0, 0] = q0  
                        current_state['q'][1, 0] = q1  
                        current_state['q'][2, 0] = q2  
                        current_state['q'][3, 0] = q3  
                        current_state['w'][0, 0] = gx
                        current_state['w'][1, 0] = gy
                        current_state['w'][2, 0] = gz

                        # 输出解析结果
                        cnt +=1
                        # print(f"完整帧: {current_frame.hex(' ')}")
                        # print(f"数据部分: {payload.hex(' ')}")
                        # print(cnt, current_state['q'].flatten(), current_state['w'].flatten() )


                        # 从缓冲区移除已处理的帧
                        received_buffer = received_buffer[TOTAL_FRAME_LENGTH:]


        except KeyboardInterrupt:
            pass
        finally:
            self.serial_port.close()
            print("Serial port closed")
            self.stop()
        return 0

    def stop(self):
        """停止并关闭串口"""
        self.alive = False

    
class UDPreceicer:
    def __init__(self):
        self.alive = False
        self.speed = 0.0
        self.angle = 0.0

    def start(self):
        self.alive = True
        self.thread = threading.Thread(target=self._read_udp)
        self.thread.daemon = True
        self.thread.start()

    def _read_udp(self):
        client_socket =socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    
        client_socket.bind(('',7799))
        while True:
            #print("udp_read")

            data, addr=client_socket.recvfrom(1024)
            angle_str = data.decode('utf-8')
            parts = angle_str.strip().split()
            self.speed = float(parts[0])/6000.0
            self.angle = float(parts[1])

            #print("speed =", self.speed)
            #print("angle =", self.angle)

            


    def stop(self):
        self.alive = False

cnt1 = 0

def timer_handler(signum, frame):
    global cnt1
    print(cnt1)
    cnt1 += 1

    # ctrl
    # 暂时给成常
    A_psi = 0 * DEG  # A_psi 目前只支持是定值
    # alpha_des = udp_receive.angle
    # speed_des = udp_receive.speed
    #print(udp_receive.angle,udp_receive.speed)
    

    # A_phi = Controller.vx2A_phi(r=ball.radius, w=controller.w, A_psi=A_psi, vx=speed_des)
    A_phi = 30 * DEG
    alpha_des = 0

    # print(A_phi)
    R_des, w_des, dw_des = controller.gaitPlanner(alpha=alpha_des, A_psi=A_psi, A_phi=A_phi)

    # w是跟着体坐标系动的，所以体坐标系怎么转都不影响期望 w的值，不用转换到w_des_w，否则效果不对
    state_des_att = {"q": utils.R2quat(R_des),
                     "w": w_des,
                     "dw": dw_des}
    T_body = controller.gaitTracker(current_state, state_des_att)
    T_wheel = controller.touqueDistribute(T_body, ball.D)
    controller.updateT() 
    # T_wheel = np.array([[0, 0, 0]]).T
    T1 = -T_wheel[0,0]*500
    T2 = -T_wheel[1,0]*500
    T3 = -T_wheel[2,0]*500

    #print(T1,T2,T3)
    # 2. 封装并发送帧
    # frame = serial_port.generate_test_array(T1,T2,T3)
    # write_num = serial_port.serial_port.write(frame)
    #print(f"Sent frame (length: {len(frame)}B):")
    #print(f"  Hex: {frame.hex(' ')}")

if __name__ == "__main__":

    # ctrl
    t_step = 0.01

    ball = Ball(t_step=t_step)
    # circle trajectory has different initial position
    # TODO:研究怎么加进去初值传参

    fsm = FSM()
    controller = Controller(ball = ball, t_step=t_step)


    # 创建实例
    serial_port = SerialPort(port='/dev/ttyS1', baudrate=115200)
    udp_receive = UDPreceicer()
    try:
        # 开始监听
        serial_port.start()
        udp_receive.start()

        # 这个定时器非常准
        signal.signal(signal.SIGALRM, timer_handler)
        signal.setitimer(signal.ITIMER_REAL, 0.01, 0.01)  # 首次触发 0.01s，之后每 10ms 触发一次
        # 主线程继续执行其他任务
        while True:
            sleep(0.01)
            pass
            
    except KeyboardInterrupt:
        print("用户中断程序")
    finally:
        # 确保关闭串口
        serial_port.stop()

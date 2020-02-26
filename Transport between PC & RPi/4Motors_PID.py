#!/usr/bin/env python
# coding: utf-8
'''
This file is for testing the brushless DC motor: DJI M2006 P36 and its controller: DJI C610. Raspberry Pi 4 is used to send to and receive message from controller. Since Raspberrypi has no CAN bus interface,  a CANable pro chip is used to transfer message between CAN and serial. To know more information of DJI motor and its controller: 
https://www.robomaster.com/zh-CN/products/components/general/M2006
To understand more about script: https://python-can.readthedocs.io/en/master/
For more information of CANable: https://canable.io/getting-started.html
'''

from __future__ import print_function
import socket
import time
import threading
import can
from can import Message
from binascii import hexlify
from collections import deque
import msgpack
import numpy as np
np.set_printoptions(suppress=True)

k_p = 0.07
k_i = 0.9
k_d = 0.25
# k_p = 0.07
# k_i = 1.65
# k_d = 0.3
speedDirectionBoundary = 32768
maxBoundary = 65536
drive_ratio = 36
PID_H = 1000
PID_L = -1000
desire_speed = np.array([0, 0, 0, 0])
command_que = deque(maxlen=4)
speed_que = deque(maxlen=4)

def udp_Receive_FromPC(udp_socket):
    global desire_speed
    while True:
        recv_data = udp_socket.recv(1024)
        command = np.array(msgpack.unpackb(recv_data))
        desire_speed = command[[0, 1, 2, 3]]
        # command_que.append(desire_speed)
    
    
def receive_send(bus, msg):
    global desire_speed, speed_que
    ID_set = set()
    former_msg = []
    i = 0
    while True:
        recv_msg = bus.recv()
        if recv_msg.arbitration_id - 513 == i:
            former_msg.append(recv_msg)
            i += 1
            if len(former_msg) == 4:
                break
    former_speed = np.zeros(4,dtype=np.float64)
    former_time = np.zeros(4,dtype=np.float64)
    
    i = 0
    bytes_to_int = np.dtype(np.uint16)
    bytes_to_int = bytes_to_int.newbyteorder('>')
    for mesg in former_msg:
        f_speed = np.frombuffer(mesg.data, dtype=bytes_to_int)[1]
        # f_speed = int(str(hexlify(mesg.data), "utf-8")[4:8],16)
        if f_speed >= speedDirectionBoundary:
            f_speed = -(maxBoundary - f_speed) / drive_ratio
        else:
            f_speed /= drive_ratio
        former_speed[i] = f_speed
        former_time[i] = mesg.timestamp
        i += 1
    former_error = desire_speed - former_speed
    error = former_error
    
    new_speed = np.zeros(4,dtype=np.float64)
    new_time = np.zeros(4,dtype=np.float64)
    new_msg = former_msg.copy()
    while True:
        ID_set.clear()
        while True:
            recv_msg = bus.recv()
            ID_set.add(recv_msg.arbitration_id)
            new_msg[recv_msg.arbitration_id - 513] = recv_msg
            if len(ID_set) == 4:
                break
            
        i = 0
        for mesg in new_msg:
            n_speed = np.frombuffer(mesg.data, dtype=bytes_to_int)[1]
            if n_speed >= speedDirectionBoundary:
                n_speed = -(maxBoundary - n_speed) / drive_ratio
            else:
                n_speed /= drive_ratio
            new_speed[i] = n_speed
            new_time[i] = mesg.timestamp
            i += 1
        
        speed_que.append(new_speed)
        dt = new_time - former_time
        print(dt)
        # print(dt)
        new_error = desire_speed - new_speed
        error += new_error
        
        v_command = k_p * (new_error + error * dt / k_i + k_d * (new_error - former_error) / dt)
        
        former_time = new_time[[0, 1, 2, 3]]
        former_error = new_error[[0, 1, 2, 3]]
        
        
        v_command[v_command > PID_H] = PID_H
        v_command[v_command < PID_L] = PID_L
        v_command[v_command > 0] = (v_command[v_command > 0] / PID_H) * (speedDirectionBoundary - 1)
        v_command[v_command < 0] = 65535 - (v_command[v_command < 0] / PID_L) * (speedDirectionBoundary - 1)
        
        i = 0
        for r_speed in v_command:
            msg.data[i], msg.data[i + 1] = divmod(int(r_speed), 0x100)
            i += 2
            
        bus.send(msg)

def main():
    global desire_speed, speed_que
    
    ip_local = '192.168.0.77'
    port_local = 1001

    ip_remote = '192.168.0.67'
    port_remote = 1000

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((ip_local, port_local))
    
    """Control the sender and receiver."""
    with can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=1000000) as bus:
        tx_msg: Message = can.Message(
            # controller can id 0x200
            arbitration_id=0x200,
            data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            is_extended_id=False
        )

        # Thread for sending and receiving messages

        t_receive_send = threading.Thread(target=receive_send, args=(bus, tx_msg))
        t_receive_udp = threading.Thread(target=udp_Receive_FromPC, args=(udp_socket,))
        t_receive_send.start()
        t_receive_udp.start()
        
        # Send msg to PC:
        '''
        while True:
            if len(speed_que) > 3:
                send_data = speed_que.popleft()
                send_data = msgpack.packb(send_data.tolist())
                udp_socket.sendto(send_data, (ip_remote, port_remote))    
        '''
        try:
            while True:
                time.sleep(0)  # yield
        except KeyboardInterrupt:
            pass  # exit normally

    # stop_event.set()
     #time.sleep(0.5)

    print("Stopped script")


if __name__ == "__main__":
    main()


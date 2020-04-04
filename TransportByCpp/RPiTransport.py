#!/usr/bin/env python
# coding: utf-8


from __future__ import print_function
import socket
import time
import serial
import msgpack
# import threading
from binascii import hexlify
from collections import deque
import msgpack
import numpy as np
from multiprocessing import Process,Queue
import pickle
np.set_printoptions(suppress=True)
    
command = [0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

def recvUdp(ser, udp_socket):
    while True:
        time_now = time.time()
        recv_data = udp_socket.recv(40)
        #np.array(msgpack.unpackb(recv_data))
        send_data = list(eval(recv_data.decode()))
        if (len(send_data) == 4):
            for i in range(4):
                command[2 + i * 2], command[3 + i * 2]= divmod(send_data[i], 0x100)
            # print(command)
            ser.write(command)
        #print(recv_data.decode())
        # command_que.append(desire_speed)
        
def sendUdp(ser, udp_socket, ip_remote, port_remote):
    while True:
        send_data = str(list(ser.read(16))).replace(" ","")
        #send_Teensy = pickle.dumps(send_data[0:18])
        print(send_data[1:-1] + ",")
        #print(str(list(send_Teensy)))s
        udp_socket.sendto((send_data[1:-1] + ",").encode(), (ip_remote, port_remote)) 
        #print(list(send_Teensy)[10:18])
    

def main():

    ip_local = '192.168.0.77'
    port_local = 1001

    ip_remote = '192.168.0.82'
    port_remote = 1000

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((ip_local, port_local))

    """Control the sender and receiver."""
    
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        #s = ser.read(50)
        # Thread for sending and receiving messages
    t_recvUdp_sendToTeensy = Process(target = recvUdp, args = (ser, udp_socket))
    t_sendUdp_recvfromTeensy = Process(target = sendUdp, args = (ser, udp_socket, ip_remote, port_remote))
    t_recvUdp_sendToTeensy.start()
    t_sendUdp_recvfromTeensy.start()

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

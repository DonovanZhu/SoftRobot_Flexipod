#!/usr/bin/env python
# coding: utf-8
from __future__ import print_function
import socket
import threading
import msgpack
import time
from collections import deque
import can
from can import Message
from binascii import hexlify
import numba as nb
motors_que1 = deque(maxlen=4)
motors_que2 = deque(maxlen=4)
motors_que3 = deque(maxlen=4)
motors_que4 = deque(maxlen=4)
# motor_q = deque(maxlen=4)
speed = ['250', '200', '150', '100'] 
# Not using Socket, so I set the speed of 4 motors in program. Unit: Rpm
k_p = 0.07
k_i = 1.65
k_d = 0.3
speedDirectionBoundary = 32768 
'''
Which is 0x8000, if the speed is between 0x0000 to 0x7fff, the motor rotates in clockwise.
If the speed is between 0x8000 to 0xffff, the motor rotates in counterclockwise.
Besides, if the current is between 0x0000 to 0x7fff, the motor rotates in clockwise.
If the current is between 0x8000 to 0xffff, the motor rotates in counterclockwise.

When speed or current is 0x0000 or 0xffff, the motor stops.
'''
maxBoundary = 65536
#Maxboundary is 0xffff
drive_ratio = 36
#Drive ratio of motor between rotor and output shaft is 36
PID_H = 1000
#Max PID output
PID_L = -1000
#Minimun PID output
wait_time = 0.007
'''
The speed of receiving data from Encoder and processing data are different.
To make sure each motor receives the data from CAN bus insead of receive an empty msg,
it will wait 0.007s.
I tried to use a while loop to check the data continuously, but the program seems very slow.
'''
lock = threading.Lock()


# Disable UDP modle：
'''
def UdpSendToPC(udp_socket, ip_remote, port_remote):
    global motor_q
    while True:
        if motor_q:
            motor_data = motor_q.popleft()
            send_data = msgpack.packb(motor_data)
            udp_socket.sendto(send_data, (ip_remote, port_remote))
'''
'''
def UdpRecvFromPC(udp_socket):
    # In this function, it receives velocity command from PC.py.
    global speed
    while True:
        recv_data = udp_socket.recv(1024)
        speed_msg = bytes.decode(msgpack.unpackb(recv_data))
        motor_index = 0
        speed = ['','','','']
        for speed_index in speed_msg:
            if speed_index != ',':
                speed[motor_index] += speed_index
            else:
                motor_index += 1
        print(speed)
'''

def receive(bus, stop_event):
'''
This function keep receiving the data in CAN bus. It appends data to
the ralated deques (which are motors_que1, motors_que2, motors_que3, motors_que4), 
so PID processing modle reads data from it.
Since I disable the UDP module, I comment out all the 'motor_q' and msg_sending.
'''
    global motors_que1, motors_que2, motors_que3, motors_que4 # , motor_q

    while not stop_event.is_set():
        lock.acquire()
        rx_msg = bus.recv()
        # msg_sending = str(hexlify(rx_msg.data), "utf-8")

        if rx_msg.arbitration_id == 513:
            motors_que1.append(rx_msg)
            # msg_sending += '1'
        elif rx_msg.arbitration_id == 514:
            motors_que2.append(rx_msg)
            # msg_sending += '2'
        elif rx_msg.arbitration_id == 515:
            motors_que3.append(rx_msg)
            # msg_sending += '3'
        elif rx_msg.arbitration_id == 516:
            motors_que4.append(rx_msg)
            # msg_sending += '4'
        lock.release()
        # motor_q.append(msg_sending)


#Motor1:
def send_cyclic_motor1(bus, msg):
    global speed, motors_que1
    desire_speed = int(speed[0])
    former_msg = motors_que1.popleft()
    
    '''
    If directly reading the data from bus, the data is hex type like: 
    \x12\x32\x12
    So I need to transfer it to Dec:
    '''
    former_speed = str(hexlify(former_msg.data), "utf-8") 
    former_speed = int(former_speed[4:8], 16)
    # Check the command is rotating clockwise or counter-clockwise:
    if former_speed >= speedDirectionBoundary:
        former_speed = -(maxBoundary - former_speed) / drive_ratio
    else:
        former_speed /= drive_ratio
     
        
    former_error = desire_speed - former_speed
    error = former_error
    former_time = former_msg.timestamp

    while True:
        # check command everytime:
        desire_speed = int(speed[0])
        '''
        #To avoid receiving Empty data:
        while True:
            if len(motors_que1) > 0:
                new_msg = motors_que1.popleft()
                break
        '''
        new_msg = motors_que1.popleft()
        new_time = new_msg.timestamp
        dt = new_time - former_time
        
        new_speed = str(hexlify(new_msg.data), "utf-8")
        new_speed = int(new_speed[4:8], 16)
        
        # print(new_speed)
        if new_speed >= speedDirectionBoundary:
            new_speed = - (maxBoundary - new_speed) / drive_ratio
        else:
            new_speed /= drive_ratio

        new_error = desire_speed - new_speed
        error += new_error
        # PID:
        v_command = k_p * (new_error + error * dt / k_i + k_d * (new_error - former_error) / dt)
        # PID output a Dec number. Transfer it into Hex then devide it into High and Low
        # msg.data[0] and msg.data[1] are current of motor 1
        if v_command >= 0:
            if v_command > PID_H:
                v_command = PID_H
            msg.data[0], msg.data[1] = divmod(int((v_command / PID_H) * (speedDirectionBoundary - 1)), 0x100)

        elif v_command < 0:
            if v_command < PID_L:
                v_command = PID_L
            current_command = int((v_command / PID_L) * (speedDirectionBoundary - 1))
            current_command = 65535 - current_command
            msg.data[0], msg.data[1] = divmod(current_command, 0x100)
            
        former_time, former_error = new_time, new_error
        # print(dt)
        # send message to bus
        bus.send(msg)
        time.sleep(wait_time)

# Motor 2:
def send_cyclic_motor2(bus, msg):
    global speed, motors_que2
    while True:
        if speed[1] != '':
            desire_speed = int(speed[1])
            break
    former_msg = motors_que2.popleft()
    former_speed = str(hexlify(former_msg.data), "utf-8")
    former_speed = int(former_speed[4:8], 16)
    if former_speed >= speedDirectionBoundary:
        former_speed = -(maxBoundary - former_speed) / drive_ratio
    else:
        former_speed /= drive_ratio
    former_error = desire_speed - former_speed
    error = former_error
    former_time = former_msg.timestamp

    while True:
        desire_speed = int(speed[1])
        '''
        while True:
            if len(motors_que2) > 0:
                new_msg = motors_que2.popleft()
                break
        '''
        new_msg = motors_que2.popleft()
        new_time = new_msg.timestamp
        dt = new_time - former_time
        
        new_speed = str(hexlify(new_msg.data), "utf-8")
        new_speed = int(new_speed[4:8], 16)
        # print(new_speed)
        if new_speed >= speedDirectionBoundary:
            new_speed = - (maxBoundary - new_speed) / drive_ratio
        else:
            new_speed /= drive_ratio
        new_error = desire_speed - new_speed
        error += new_error
        v_command = k_p * (new_error + error * dt / k_i + k_d * (new_error - former_error) / dt)

        if v_command >= 0:
            if v_command > PID_H:
                v_command = PID_H
            msg.data[2], msg.data[3] = divmod(int((v_command / PID_H) * (speedDirectionBoundary - 1)), 0x100)

        elif v_command < 0:
            if v_command < PID_L:
                v_command = PID_L
            current_command = int((v_command / PID_L) * (speedDirectionBoundary - 1))
            current_command = 65535 - current_command
            msg.data[2], msg.data[3] = divmod(current_command, 0x100)
            
        former_time, former_error = new_time, new_error
        # print(dt)
        # send message to bus
        bus.send(msg)
        time.sleep(wait_time)

# Motor 3:
def send_cyclic_motor3(bus, msg):
    global speed, motors_que3
    while True:
        if speed[2] != '':
            desire_speed = int(speed[2])
            break
    former_msg = motors_que3.popleft()
    former_speed = str(hexlify(former_msg.data), "utf-8")
    former_speed = int(former_speed[4:8], 16)
    if former_speed >= speedDirectionBoundary:
        former_speed = -(maxBoundary - former_speed) / drive_ratio
    else:
        former_speed /= drive_ratio
    former_error = desire_speed - former_speed
    error = former_error
    former_time = former_msg.timestamp

    while True:
        desire_speed = int(speed[2])
        '''
        while True:
            if len(motors_que3) > 0:
                new_msg = motors_que2.popleft()
                break
        '''
        new_msg = motors_que3.popleft()
        new_time = new_msg.timestamp
        dt = new_time - former_time
        
        new_speed = str(hexlify(new_msg.data), "utf-8")
        new_speed = int(new_speed[4:8], 16)
        # print(new_speed)
        if new_speed >= speedDirectionBoundary:
            new_speed = - (maxBoundary - new_speed) / drive_ratio
        else:
            new_speed /= drive_ratio
        print(new_speed)
        new_error = desire_speed - new_speed
        error += new_error
        v_command = k_p * (new_error + error * dt / k_i + k_d * (new_error - former_error) / dt)

        if v_command >= 0:
            if v_command > PID_H:
                v_command = PID_H
            msg.data[4], msg.data[5] = divmod(int((v_command / PID_H) * (speedDirectionBoundary - 1)), 0x100)

        elif v_command < 0:
            if v_command < PID_L:
                v_command = PID_L
            current_command = int((v_command / PID_L) * (speedDirectionBoundary - 1))
            current_command = 65535 - current_command
            msg.data[4], msg.data[5] = divmod(current_command, 0x100)
            
        former_time, former_error = new_time, new_error
        # print(dt)
        # msg.data[0], msg.data[1], former_msg, former_error = pid(new_msg, desire_speed, former_error, error, dt)
        # send message to bus
        bus.send(msg)
        time.sleep(wait_time)

# Motor 4:
def send_cyclic_motor4(bus, msg):
    global speed, motors_que4
    while True:
        if speed[3] != '':
            desire_speed = int(speed[3])
            break
    former_msg = motors_que4.popleft()
    former_speed = str(hexlify(former_msg.data), "utf-8")
    former_speed = int(former_speed[4:8], 16)
    if former_speed >= speedDirectionBoundary:
        former_speed = -(maxBoundary - former_speed) / drive_ratio
    else:
        former_speed /= drive_ratio
    former_error = desire_speed - former_speed
    error = former_error
    former_time = former_msg.timestamp

    while True:
        desire_speed = int(speed[3])
        '''
        while True:
            if len(motors_que3) > 0:
                new_msg = motors_que2.popleft()
                break
        '''
        new_msg = motors_que4.popleft()
        new_time = new_msg.timestamp
        dt = new_time - former_time
        
        new_speed = str(hexlify(new_msg.data), "utf-8")
        new_speed = int(new_speed[4:8], 16)
        # print(new_speed)
        if new_speed >= speedDirectionBoundary:
            new_speed = - (maxBoundary - new_speed) / drive_ratio
        else:
            new_speed /= drive_ratio
        print(new_speed)
        new_error = desire_speed - new_speed
        error += new_error
        v_command = k_p * (new_error + error * dt / k_i + k_d * (new_error - former_error) / dt)

        if v_command >= 0:
            if v_command > PID_H:
                v_command = PID_H
            msg.data[6], msg.data[7] = divmod(int((v_command / PID_H) * (speedDirectionBoundary - 1)), 0x100)

        elif v_command < 0:
            if v_command < PID_L:
                v_command = PID_L
            current_command = int((v_command / PID_L) * (speedDirectionBoundary - 1))
            current_command = 65535 - current_command
            msg.data[6], msg.data[7] = divmod(current_command, 0x100)
            
        former_time, former_error = new_time, new_error
        # print(dt)
        # msg.data[0], msg.data[1], former_msg, former_error = pid(new_msg, desire_speed, former_error, error, dt)
        # send message to bus
        bus.send(msg)
        time.sleep(wait_time)


def main():
    '''
    ip_local = '192.168.0.77'
    port_local = 1001

    ip_remote = '192.168.0.67'
    port_remote = 1000

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((ip_local, port_local))
    '''
    """Control the sender and receiver."""
    with can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=1000000) as bus:
        tx_msg: Message = can.Message(
            # controller can id 0x200
            arbitration_id=0x200,
            data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            is_extended_id=False
        )

        # Thread for sending and receiving messages
        stop_event = threading.Event()

        t_send_cyclic1 = threading.Thread(target=send_cyclic_motor1, args=(bus, tx_msg))
        t_send_cyclic2 = threading.Thread(target=send_cyclic_motor2, args=(bus, tx_msg))
        t_send_cyclic3 = threading.Thread(target=send_cyclic_motor3, args=(bus, tx_msg))
        t_send_cyclic4 = threading.Thread(target=send_cyclic_motor4, args=(bus, tx_msg))

        t_receive = threading.Thread(target=receive, args=(bus, stop_event))
        # thread_sender = threading.Thread(target=UdpSendToPC, args=(udp_socket, ip_remote, port_remote))
        # thread_receiver = threading.Thread(target=UdpRecvFromPC, args=(udp_socket,))
        t_receive.start()
        time.sleep(2)
        # thread_sender.start()
        # thread_receiver.start()
        t_send_cyclic1.start()
        t_send_cyclic2.start()
        t_send_cyclic3.start()
        t_send_cyclic4.start()

        try:
            while True:
                time.sleep(0)  # yield
        except KeyboardInterrupt:
            pass  # exit normally

    stop_event.set()
    time.sleep(0.5)


if __name__ == '__main__':
    main()


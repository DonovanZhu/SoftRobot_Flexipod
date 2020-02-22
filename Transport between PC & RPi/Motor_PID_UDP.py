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

motors_que1 = deque(maxlen=2)
motors_que2 = deque(maxlen=2)
motors_que3 = deque(maxlen=2)
motors_que4 = deque(maxlen=2)
motor_q = deque()
speed = ['', '', '', '']
k_p = 0.07
k_i = 1.65
k_d = 0.3
speedDirectionBoundary = 32768
maxBoundary = 65536
drive_ratio = 36
PID_H = 1000
PID_L = -1000


def UdpSendToPC(udp_socket, ip_remote, port_remote):
    global motor_q
    while True:
        if motor_q:
            motor_data = motor_q.popleft()
            send_data = msgpack.packb(motor_data)
            udp_socket.sendto(send_data, (ip_remote, port_remote))


def UdpRecvFromPC(udp_socket):
    # In this function, it receives velocity command from PC.py.
    global speed
    while True:
        recv_data = udp_socket.recv(1024)
        speed_msg = bytes.decode(msgpack.unpackb(recv_data))
        motor_index = 0
        speed = ['', '', '', '']
        for speed_index in speed_msg:
            if speed_index != ',':
                speed[motor_index] += speed_index
            else:
                motor_index += 1
                continue


def receive(bus, stop_event):
    global motor_q
    while not stop_event.is_set():
        rx_msg = bus.recv()
        msg_sending = str(hexlify(rx_msg.data), "utf-8")
        if rx_msg is not None:
            if rx_msg.arbitration_id == 513:
                motors_que1.append(rx_msg)
                msg_sending += '1'
            elif rx_msg.arbitration_id == 514:
                motors_que2.append(rx_msg)
                msg_sending += '2'
            elif rx_msg.arbitration_id == 515:
                motors_que3.append(rx_msg)
                msg_sending += '3'
            elif rx_msg.arbitration_id == 516:
                motors_que4.append(rx_msg)
                msg_sending += '4'
            # msg_sending = hex(int(msg_sending, 16))
            motor_q.append(msg_sending)


def pid(new_msg, desire_speed, former_error, error, dt):
    new_speed = str(hexlify(new_msg.data), "utf-8")
    new_speed = int(new_speed[4:8], 16)
    current_command_H = 0
    current_command_L = 0
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
        current_command_H, current_command_L = divmod(int((v_command / PID_H) * (speedDirectionBoundary - 1)), 0x100)

    elif v_command < 0:
        if v_command < PID_L:
            v_command = PID_L
        current_command = int((v_command / PID_L) * (speedDirectionBoundary - 1))
        current_command = 0xffff - current_command
        current_command_H, current_command_L = divmod(current_command, 0x100)

    return current_command_H, current_command_L, new_msg, new_error


def send_cyclic_motor1(bus, msg):
    global motors_que1
    desire_speed = int(speed[0])
    start_time = time.time()
    while True:
        if len(motors_que1) > 1:
            former_msg = motors_que1.popleft()
            break
    former_speed = str(hexlify(former_msg.data), "utf-8")
    former_speed = int(former_speed[4:8], 16)
    if former_speed >= speedDirectionBoundary:
        former_speed = -(maxBoundary - former_speed) / drive_ratio
    else:
        former_speed /= drive_ratio
    former_error = desire_speed - former_speed
    error = former_error

    while True:
        while True:
            if len(motors_que1) > 1:
                new_msg = motors_que1.popleft()
                break
        dt = new_msg.timestamp - former_msg.timestamp
        print(dt)
        msg.data[0], msg.data[1], former_msg, former_error = pid(new_msg, desire_speed, former_error, error, dt)

        msg.timestamp = time.time() - start_time
        # send message to bus
        bus.send(msg)


def send_cyclic_motor2(bus, msg):
    global motors_que2
    desire_speed = int(speed[1])
    start_time = time.time()
    while True:
        if len(motors_que2) > 1:
            former_msg = motors_que2.popleft()
            break
    former_speed = str(hexlify(former_msg.data), "utf-8")
    former_speed = int(former_speed[4:8], 16)
    if former_speed >= speedDirectionBoundary:
        former_speed = -(maxBoundary - former_speed) / drive_ratio
    else:
        former_speed /= drive_ratio
    former_error = desire_speed - former_speed
    error = former_error

    while True:
        while True:
            if len(motors_que2) > 1:
                new_msg = motors_que2.popleft()
                break
        dt = new_msg.timestamp - former_msg.timestamp
        print(dt)
        msg.data[2], msg.data[3], new_msg, new_error = pid(new_msg, desire_speed, former_error, error, dt)
        msg.timestamp = time.time() - start_time
        # send message to bus
        bus.send(msg)


def send_cyclic_motor3(bus, msg):
    global motors_que3
    desire_speed = int(speed[2])
    start_time = time.time()
    while True:
        if len(motors_que3) > 1:
            former_msg = motors_que3.popleft()
            break
    former_speed = str(hexlify(former_msg.data), "utf-8")
    former_speed = int(former_speed[4:8], 16)
    if former_speed >= speedDirectionBoundary:
        former_speed = -(maxBoundary - former_speed) / drive_ratio
    else:
        former_speed /= drive_ratio
    former_error = desire_speed - former_speed
    error = former_error

    while True:
        while True:
            if len(motors_que3) > 1:
                new_msg = motors_que3.popleft()
                break
        dt = new_msg.timestamp - former_msg.timestamp
        print(dt)
        msg.data[4], msg.data[5], new_msg, new_error = pid(new_msg, desire_speed, former_error, error, dt)
        msg.timestamp = time.time() - start_time
        # send message to bus
        bus.send(msg)


def send_cyclic_motor4(bus, msg):
    global motors_que4
    desire_speed = int(speed[3])
    start_time = time.time()
    while True:
        if len(motors_que4) > 1:
            former_msg = motors_que4.popleft()
            break
    former_speed = str(hexlify(former_msg.data), "utf-8")
    former_speed = int(former_speed[4:8], 16)
    if former_speed >= speedDirectionBoundary:
        former_speed = -(maxBoundary - former_speed) / drive_ratio
    else:
        former_speed /= drive_ratio
    former_error = desire_speed - former_speed
    error = former_error

    while True:
        while True:
            if len(motors_que4) > 1:
                new_msg = motors_que4.popleft()
                break
        dt = new_msg.timestamp - former_msg.timestamp
        print(dt)
        msg.data[6], msg.data[7], new_msg, new_error = pid(new_msg, desire_speed, former_error, error, dt)
        msg.timestamp = time.time() - start_time
        # send message to bus
        bus.send(msg)


def main():
    ip_local = '160.39.159.127'
    port_local = 1001

    ip_remote = '160.39.159.127'
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
        stop_event = threading.Event()

        t_send_cyclic1 = threading.Thread(target=send_cyclic_motor1, args=(bus, tx_msg))
        t_send_cyclic2 = threading.Thread(target=send_cyclic_motor2, args=(bus, tx_msg))
        t_send_cyclic3 = threading.Thread(target=send_cyclic_motor3, args=(bus, tx_msg))
        t_send_cyclic4 = threading.Thread(target=send_cyclic_motor4, args=(bus, tx_msg))

        t_receive = threading.Thread(target=receive, args=(bus, stop_event))
        thread_sender = threading.Thread(target=UdpSendToPC, args=(udp_socket, ip_remote, port_remote))
        thread_receiver = threading.Thread(target=UdpRecvFromPC, args=(udp_socket,))
        thread_sender.start()
        thread_receiver.start()
        t_receive.start()
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

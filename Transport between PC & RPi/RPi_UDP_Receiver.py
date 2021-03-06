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

motors_que = deque()
motor_q = deque()


def UdpSendToPC(udp_socket, ip_remote, port_remote):
    global motor_q
    while True:
        if motor_q:
            motor_data = motor_q.popleft()
            send_data = msgpack.packb(motor_data)
            udp_socket.sendto(send_data, (ip_remote, port_remote))


def UdpRecvFromPC(udp_socket):
    # In this function, it receives velocity command from PC.py.
    global motors_que
    while True:
        recv_data = udp_socket.recv(1024)
        motors_que.append(msgpack.unpackb(recv_data))
        #print(msgpack.unpackb(recv_data))


def send_cyclic(bus, msg, stop_event):
    """The loop for sending."""
    global motors_que
    start_time = time.time()
    while not stop_event.is_set():
        msg.timestamp = time.time() - start_time
        # send message to bus
        if len(motors_que) != 0:
            motor_command = motors_que.popleft()
            motor_command = bytes.decode(motor_command)
            print(motor_command)
            for i in range(0, len(motor_command), 2):
                #msg.data[int(i / 2)] = bytes.fromhex(motor_command[i:i+2])
                msg.data[int(i / 2)] = int(motor_command[i:i+2], 16)
            #print(msg.data)

        bus.send(msg)

        # improve the current 1 unit every one second
        #print(f"tx: {msg.data}")
        time.sleep(0.5)


def receive(bus, stop_event):
    """The loop for receiving."""
    global motor_q
    while not stop_event.is_set():
        rx_msg = bus.recv()
        #data = bytes.decode(data)
        msg_sending = str(hexlify(rx_msg.data), "utf-8")
        if rx_msg is not None and len(msg_sending) == 16:
            if rx_msg.arbitration_id == 513:
                msg_sending += '1'
            elif rx_msg.arbitration_id == 514:
                msg_sending += '2'
            elif rx_msg.arbitration_id == 515:
                msg_sending += '3'
            elif rx_msg.arbitration_id == 516:
                msg_sending += '4'
            print(msg_sending)
            #msg_sending = hex(int(msg_sending, 16))
            motor_q.append(msg_sending)
            #print("rx: {}".format(rx_msg.data))


def main():
    ip_local = '192.168.0.27'
    port_local = 1001

    ip_remote = '192.168.0.231'
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

        t_send_cyclic = threading.Thread(target=send_cyclic, args=(bus, tx_msg, stop_event))
        t_receive = threading.Thread(target=receive, args=(bus, stop_event))
        thread_sender = threading.Thread(target=UdpSendToPC, args=(udp_socket, ip_remote, port_remote))
        thread_receiver = threading.Thread(target=UdpRecvFromPC, args=(udp_socket,))
        thread_sender.start()
        thread_receiver.start()
        t_receive.start()
        t_send_cyclic.start()

        try:
            while True:
                time.sleep(0)  # yield
        except KeyboardInterrupt:
            pass  # exit normally

    stop_event.set()
    time.sleep(0.5)


if __name__ == '__main__':
    main()

#!/usr/bin/env python
# coding: utf-8
"""
This file is for testing the brushless DC motor: DJI M2006 P36 and its controller: DJI C610. Raspberry Pi 4 is used to send to and receive message from controller. Since Raspberrypi has no CAN bus interface,  a CANable pro chip is used to transfer message between CAN and serial. To know more information of DJI motor and its controller:
https://www.robomaster.com/zh-CN/products/components/general/M2006
To understand more about script: https://python-can.readthedocs.io/en/master/
For more information of CANable: https://canable.io/getting-started.html
"""

from __future__ import print_function
from binascii import hexlify
import time
import threading
import can
from can import Message
from collections import deque

k_p = 1.0
k_i = 0.0
k_d = 0.0
motors_que = deque(maxlen=2)
speedDirectionBoundary = 32768
maxBoundary = 65536
drive_ratio = 36
PID_H = 100
PID_L = -100


def send_cyclic(bus, msg, stop_event):
    """The loop for sending."""
    global motors_que
    desire_speed = 60
    start_time = time.time()
    while True:
        if len(motors_que) > 1:
            break
    former_msg = motors_que.popleft()
    former_speed = str(hexlify(former_msg.data), "utf-8")
    former_speed = int(former_speed[4:8], 16)
    if former_speed >= speedDirectionBoundary:
        former_speed = (maxBoundary - former_speed) / drive_ratio
    former_error = desire_speed - former_speed
    error = former_error

    while not stop_event.is_set():
        new_msg = motors_que.popleft()
        dt = new_msg.timestamp - former_msg.timestamp

        new_speed = str(hexlify(new_msg.data), "utf-8")
        new_speed = int(new_speed[4:8], 16)
        if new_speed >= speedDirectionBoundary:
            new_speed = (maxBoundary - former_speed) / drive_ratio
        new_error = desire_speed - new_speed

        error += new_error
        v_command = k_p * new_error + k_i * error * dt + k_d * (new_error - former_error) / dt

        former_msg = new_msg
        former_error = new_error
        former_speed = new_speed

        if v_command > 0:
            if v_command > PID_H:
                v_command = PID_H
            current_command_H, current_command_L = divmod(int((v_command / PID_H) * (speedDirectionBoundary - 1)), 0x100)
            msg.data[0] = current_command_H
            msg.data[1] = current_command_L
        elif v_command < 0:
            if v_command < PID_L:
                v_command = PID_L
            current_command = int((v_command / PID_L) * (speedDirectionBoundary - 1))
            current_command += 0x8000
            current_command_H, current_command_L = divmod(current_command, 0x100)
            msg.data[0] = current_command_H
            msg.data[1] = current_command_L

        msg.timestamp = time.time() - start_time
        # send message to bus
        bus.send(msg)

        # improve the current 1 unit every one second
        time.sleep(0.1)


def receive(bus, stop_event):
    global motors_que
    """The loop for receiving."""
    while not stop_event.is_set():
        Motor_msg = bus.recv()
        motors_que.append(Motor_msg)


def pid(dt, desired_speed, msg, former_error, error):
    msg_data = str(hexlify(msg.data), "utf-8")
    msg_data = int(msg_data[4:8], 16)
    if msg_data >= 32768:
        msg_data = 65536 - msg_data
    v_error = desired_speed - msg_data / 36
    error += v_error
    v_command = k_p * v_error + k_i * error * dt + k_d * (v_error - former_error) / dt

    return v_command


def main():
    """Control the sender and receiver."""
    with can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=1000000) as bus:
        tx_msg: Message = can.Message(
            # controller can id 0x200
            arbitration_id=0x200,
            data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            # Every two hex number stand for the high and low byte of current
            # data[0] + data[1] = 0x0000, which means the current is 0x0000
            # Smaller and equal to 0x7FFF, rotating counter clockwise
            # Larger and equal to 0x8000,rotating clockwise
            # When current is 0x0000 or 0xffff, the motor is stop
            # from -10A ~ 10A
            is_extended_id=False
        )

        # Thread for sending and receiving messages
        stop_event = threading.Event()

        t_send_cyclic = threading.Thread(target=send_cyclic, args=(bus, tx_msg, stop_event))

        t_receive = threading.Thread(target=receive, args=(bus, stop_event))
        t_receive.start()
        t_send_cyclic.start()

        try:
            while True:
                time.sleep(0)  # yield
        except KeyboardInterrupt:
            pass  # exit normally

    stop_event.set()
    time.sleep(0.5)


if __name__ == "__main__":
    main()

#!/usr/bin/env python
# coding: utf-8
"""
This file is for testing the brushless DC motor: DJI M2006 P36 and its controller: DJI C610. Raspberry Pi 4 is used to send to and receive message from controller. Since Raspberrypi has no CAN bus interface,  a CANable pro chip is used to transfer message between CAN and serial. To know more information of DJI motor and its controller:
https://www.robomaster.com/zh-CN/products/components/general/M2006
To understand more about script: https://python-can.readthedocs.io/en/master/
For more information of CANable: https://canable.io/getting-started.html
"""

from __future__ import print_function
import time
import threading
import can
from can import Message
from collections import deque
from binascii import hexlify

k_p = 0.07
k_i = 1.65
k_d = 0.3
motors_que = deque()
speedDirectionBoundary = 32768
maxBoundary = 65536
drive_ratio = 36
PID_H = 1000
PID_L = -1000


def send_cyclic(bus, msg):
    """The loop for sending."""
    desire_speed = 300
    start_time = time.time()
    motors_que.append(bus.recv())
    former_msg = motors_que.popleft()
    former_speed = str(hexlify(former_msg.data), "utf-8")
    former_speed = int(former_speed[4:8], 16)
    if former_speed >= speedDirectionBoundary:
        former_speed = -(maxBoundary - former_speed) / drive_ratio
    else:
        former_speed /= drive_ratio
    former_error = desire_speed - former_speed
    error = former_error

    while True:
        motors_que.append(bus.recv())
        new_msg = motors_que.popleft()
        dt = new_msg.timestamp - former_msg.timestamp
        new_speed = str(hexlify(new_msg.data), "utf-8")
        new_speed = int(new_speed[4:8], 16)
        if new_speed >= speedDirectionBoundary:
            new_speed = - (maxBoundary - new_speed) / drive_ratio
        else:
            new_speed /= drive_ratio
        print("speed:",new_speed)
        new_error = desire_speed - new_speed
        error += new_error
        v_command = k_p * (new_error + error * dt / k_i + k_d * (new_error - former_error) / dt)
        print(dt)
        former_msg = new_msg
        former_error = new_error
        former_speed = new_speed

        if v_command >= 0:
            if v_command > PID_H:
                v_command = PID_H
            current_command_H, current_command_L = divmod(int((v_command / PID_H) * (speedDirectionBoundary - 1)), 0x100)
            msg.data[0] = current_command_H
            msg.data[1] = current_command_L
        elif v_command < 0:
            if v_command < PID_L:
                v_command = PID_L
            current_command = int((v_command / PID_L) * (speedDirectionBoundary - 1))
            current_command = 0xffff - current_command
            current_command_H, current_command_L = divmod(current_command, 0x100)
            msg.data[0] = current_command_H
            msg.data[1] = current_command_L

        msg.timestamp = time.time() - start_time
        # send message to bus
        bus.send(msg)


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
        send_cyclic(bus, tx_msg)

        try:
            while True:
                time.sleep(0)  # yield
        except KeyboardInterrupt:
            pass  # exit normally

    stop_event.set()
    time.sleep(0.5)


if __name__ == "__main__":
    main()

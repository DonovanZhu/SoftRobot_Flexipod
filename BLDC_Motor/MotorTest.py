#!/usr/bin/env python
# coding: utf-8
'''
This file is for testing the brushless DC motor: DJI M2006 P36 and its controller: DJI C610. Raspberry Pi 4 is used to send to and receive message from controller. Since Raspberrypi has no CAN bus interface,  a CANable pro chip is used to transfer message between CAN and serial. To know more information of DJI motor and its controller: 
https://www.robomaster.com/zh-CN/products/components/general/M2006
To understand more about script: https://python-can.readthedocs.io/en/master/
For more information of CANable: https://canable.io/getting-started.html
'''

from __future__ import print_function
import time
import threading
import can
from can import Message


def send_cyclic(bus, msg, stop_event):
    """The loop for sending."""
    print("Start to send a message every 1s")
    start_time = time.time()
    while not stop_event.is_set():
        msg.timestamp = time.time() - start_time
        # send message to bus
        bus.send(msg)

        # improve the current 1 unit every one second
        if msg.data[1] == 0xff:
            msg.data[1] = 0x00
            msg.data[0] += 1
        msg.data[1] += 1
        print(str(msg.data[0]) + str(msg.data[1]))
        time.sleep(1)
    print("Stopped sending messages")


def receive(bus, stop_event):
    """The loop for receiving."""
    print("Start receiving messages")
    while not stop_event.is_set():
        rx_msg = bus.recv()
        if rx_msg is not None:
            print("rx: {}".format(rx_msg))
    print("Stopped receiving messages")


def main():
    """Control the sender and receiver."""
    with can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=1000000) as bus:
        tx_msg: Message = can.Message(
            # controller can id 0x200
            arbitration_id=0x200,
            data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            #Every two hex number stand for the high and low byte of current
            # data[0] + data[1] = 0x0000, which means the current is 0x0000
            #Smaller and equal to 0x7FFF, rotating counter clockwise
            #Larger and equal to 0x8000,rotating clockwise
            #When current is 0x0000 or 0xffff, the motor is stop
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

    print("Stopped script")


if __name__ == "__main__":
    main()

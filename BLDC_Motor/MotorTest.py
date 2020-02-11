#!/usr/bin/env python
# coding: utf-8
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
        msg.data[1] += 1
        # avoiding the current too large
        if msg.data[1] > 0x11:
            msg.data[1] = 0x00
        print(f"tx: {msg}")
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

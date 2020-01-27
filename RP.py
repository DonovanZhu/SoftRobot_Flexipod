import socket
import threading
import msgpack
import time
from collections import deque

motors_que = deque(maxlen=4)
motor_q = deque()


def UdpSendToPC(udp_socket, ip_remote, port_remote):
    i = 0
    # In each second, this function sends a rotation angle data to PC.py.
    while True:
        send_data = msgpack.packb(i)
        udp_socket.sendto(send_data, (ip_remote, port_remote))
        i += 1
        time.sleep(1)


def UdpRecvFromPC(udp_socket):
    # In this function, it receives velocity command from PC.py.
    while True:
        recv_data = udp_socket.recv(1024)
        motors_que.append(msgpack.unpackb(recv_data))
        print('Deque:%s' % motors_que)


def main():
    ip_local = '160.39.222.206'
    port_local = 1001

    ip_remote = '160.39.222.206'
    port_remote = 1000

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((ip_local, port_local))

    thread_sender = threading.Thread(target=UdpSendToPC, args=(udp_socket, ip_remote, port_remote))
    thread_receiver = threading.Thread(target=UdpRecvFromPC, args=(udp_socket,))
    thread_sender.start()
    thread_receiver.start()


if __name__ == '__main__':
    main()

import socket
import threading
import msgpack
from collections import deque

motors_que = deque(maxlen=4)
motor_q = deque()


def UdpSendToRP(udp_socket, ip_remote, port_remote):
    while True:
        time_period = 1.0
        # Every 1 second, the data of a motor's rotation angle is sent from RP.py
        # This function calculates the velocity of the motor then transports the velocity command to motor
        if motors_que:
            motor_data = motors_que.popleft()
            send_data = msgpack.packb(motor_data / time_period)
            udp_socket.sendto(send_data, (ip_remote, port_remote))
        else:
            continue


def UdpRecvFromRP(udp_socket):
    while True:
        recv_data = udp_socket.recv(1024)
        motors_que.append(msgpack.unpackb(recv_data))
        print('Deque:%s' % motors_que)


def main():
    # put argument at top
    ip_local = '160.39.222.206'
    port_local = 1000

    ip_remote = '160.39.222.206'
    port_remote = 1001

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((ip_local, port_local))

    thread_sender = threading.Thread(target=UdpSendToRP, args=(udp_socket, ip_remote, port_remote))
    thread_receiver = threading.Thread(target=UdpRecvFromRP, args=(udp_socket,))
    thread_sender.start()
    thread_receiver.start()


if __name__ == '__main__':
    main()

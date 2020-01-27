import socket
import threading
import msgpack
from collections import deque

motors_que = deque(maxlen=4)
motor_q = deque()


def udp_send(udp_socket):
    while True:
        num1 = '160.39.222.206'
        num2 = 1001
        if motors_que:
            motor_data = motors_que.popleft()
            send_data = msgpack.packb(motor_data / 2.0)
            udp_socket.sendto(send_data, (num1, num2))
        else:
            continue


def udp_recv(udp_socket):
    while True:
        recv_data = udp_socket.recv(1024)
        motors_que.append(msgpack.unpackb(recv_data))
        print('Deque:%s' % motors_que)


def main():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ip = '160.39.222.206'
    port = 1000
    udp_socket.bind((ip, port))
    t = threading.Thread(target=udp_recv, args=(udp_socket,))
    t1 = threading.Thread(target=udp_send, args=(udp_socket,))
    t.start()
    t1.start()


if __name__ == '__main__':
    main()

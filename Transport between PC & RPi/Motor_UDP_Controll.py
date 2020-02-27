import socket
import threading
import msgpack
from collections import deque
import tkinter as tk
from tkinter import ttk
from tkinter import scrolledtext
import numpy as np

motors_que = deque()
speed_command = ''
is_write = False


def UdpSendToRP(udp_socket, ip_remote, port_remote):
    global speed_command
    global is_write
    while True:
        speed_msg = ['', '', '', '']
        msg_index = 0
        if len(speed_command) != 0 and is_write:
            for i in speed_command:
                if i != ',':
                    speed_msg[msg_index] += i
                else:
                    msg_index += 1
            msg_index = 0
            for j in speed_msg:
                speed_msg[msg_index] = int(j)
                msg_index += 1

            print(speed_msg)
            send_data = msgpack.packb(speed_msg)
            udp_socket.sendto(send_data, (ip_remote, port_remote))
            is_write = False


def UdpRecvFromRP(udp_socket):
    global motors_que
    while True:
        recv_data = udp_socket.recv(1024)
        motors_que.append(msgpack.unpackb(recv_data))


def RecvData():
    global motors_que
    while True:
        if len(motors_que) != 0:
            Display_data = motors_que.popleft()
            Receive_Window_1.insert("end", "Rpm" + str(Display_data[0]) + '\n')
            Receive_Window_1.see("end")
            Receive_Window_2.insert("end", "Rpm" + str(Display_data[1]) + '\n')
            Receive_Window_2.see("end")
            Receive_Window_3.insert("end", "Rpm" + str(Display_data[2]) + '\n')
            Receive_Window_3.see("end")
            Receive_Window_4.insert("end", "Rpm" + str(Display_data[3]) + '\n')
            Receive_Window_4.see("end")
            '''
            # Display_data = str(Display_data, encoding="gbk")
            Display_data = str(Display_data, encoding="utf-8")
            Angle = int(Display_data[0:4], 16)
            Rpm = int(Display_data[4:8], 16)
            Current = int(Display_data[8:12], 16)
            Angle = Angle / 0xffff * 360
            if Rpm < 0x8000:
                Rpm = Rpm / 36
            else:
                Rpm = - (0xffff - Rpm) / 36
            if Current < 0x8000:
                Current = Current / 0x7fff * 10
            else:
                Current = - (0xffff - Current) / 0x7fff * 10
            if Display_data[-1] == '1':
                Receive_Window_1.insert("end", "Rotation Angle:" + str(Angle)
                                        + "  " + "Rpm:" + str(Rpm)
                                        + "  " + "Torque:" + str(Current) + '\n')
                Receive_Window_1.see("end")
            elif Display_data[-1] == '2':
                Receive_Window_2.insert("end", "Rotation Angle:" + str(Angle)
                                        + "  " + "Rpm:" + str(Rpm)
                                        + "  " + "Torque:" + str(Current) + '\n')
                Receive_Window_2.see("end")
            elif Display_data[-1] == '3':
                Receive_Window_3.insert("end", "Rotation Angle:" + str(Angle)
                                        + "  " + "Rpm:" + str(Rpm)
                                        + "  " + "Torque:" + str(Current) + '\n')
                Receive_Window_3.see("end")
            elif Display_data[-1] == '4':
                Receive_Window_4.insert("end", "Rotation Angle:" + str(Angle)
                                        + "  " + "Rpm:" + str(Rpm)
                                        + "  " + "Torque:" + str(Current) + '\n')
                Receive_Window_4.see("end")
            '''
        else:
            pass


def WriteData(InputData, Info):
    global speed_command
    global is_write
    speed_command = InputData.get()
    is_write = True
    Info.insert("end", 'Command:' + str(speed_command) + "Rpm" + '\n')
    Info.see("end")


def main():
    # put argument at top
    ip_local = '160.39.222.46'
    port_local = 1000

    ip_remote = '192.168.0.77'
    # ip_remote = '192.168.0.231'
    port_remote = 1001

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((ip_local, port_local))

    thread_RECV = threading.Thread(target=RecvData)
    thread_sender = threading.Thread(target=UdpSendToRP, args=(udp_socket, ip_remote, port_remote))
    thread_receiver = threading.Thread(target=UdpRecvFromRP, args=(udp_socket,))
    thread_sender.start()
    thread_receiver.start()
    thread_RECV.start()


if __name__ == '__main__':
    main()
    MotorCommandGui = tk.Tk()
    MotorCommandGui.title("Motor Control")
    MotorCommandGui.geometry("680x800")

    # Motor One
    ReceiveMotor1 = tk.LabelFrame(MotorCommandGui, text="Motor1", padx=10, pady=10)
    ReceiveMotor1.place(x=20, y=135)
    Receive_Window_1 = scrolledtext.ScrolledText(ReceiveMotor1, width=72, height=6, padx=8, pady=10, wrap=tk.WORD)
    Receive_Window_1.grid()

    # Motor Two
    ReceiveMotor2 = tk.LabelFrame(MotorCommandGui, text="Motor2", padx=10, pady=10)
    ReceiveMotor2.place(x=20, y=300)
    Receive_Window_2 = scrolledtext.ScrolledText(ReceiveMotor2, width=72, height=6, padx=8, pady=10, wrap=tk.WORD)
    Receive_Window_2.grid()

    # Motor Three
    ReceiveMotor3 = tk.LabelFrame(MotorCommandGui, text="Motor3", padx=10, pady=10)
    ReceiveMotor3.place(x=20, y=465)
    Receive_Window_3 = scrolledtext.ScrolledText(ReceiveMotor3, width=72, height=6, padx=8, pady=10, wrap=tk.WORD)
    Receive_Window_3.grid()

    # Motor Four
    ReceiveMotor4 = tk.LabelFrame(MotorCommandGui, text="Motor4", padx=10, pady=10)
    ReceiveMotor4.place(x=20, y=630)
    Receive_Window_4 = scrolledtext.ScrolledText(ReceiveMotor4, width=72, height=6, padx=8, pady=10, wrap=tk.WORD)
    Receive_Window_4.grid()

    Information = tk.LabelFrame(MotorCommandGui, text="Sending Information", padx=10, pady=10)
    Information.place(x=20, y=20)
    Information_Window = scrolledtext.ScrolledText(Information, width=50, height=3, padx=8, pady=10, wrap=tk.WORD)
    Information_Window.grid()

    Send = tk.LabelFrame(MotorCommandGui, text="Send Command", padx=10, pady=10)
    Send.place(x=487, y=20)

    EntrySend = tk.StringVar()
    Send_Window = ttk.Entry(Send, textvariable=EntrySend, width=23)
    Send_Window.grid()

    tk.Button(Send, text="Send", command=lambda: WriteData(EntrySend, Information_Window)).grid(pady=10, sticky=tk.E)

    MotorCommandGui.mainloop()

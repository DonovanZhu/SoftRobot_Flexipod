#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <vector>
#include <netdb.h>
#include <limits.h>
#include <dirent.h>
#include <iomanip>
#include <iostream>
#include <vector>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/param.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <linux/serial.h>
#include <linux/input.h>
#include <msgpack.hpp>
#include "host.h"

#include <chrono>
#include <ctime>

using namespace std;

char buf_UDP_recv[200]; 			// for holding UDP data

double desire_speed[NB_ESC] = {0.0, 0.0, 0.0, 0.0};		// The deisred speed

// Receiving the desired speed data by UDP and hold it in this class
class MotorSpeed {
public:
	double rpm[NB_ESC];
	MSGPACK_DEFINE(rpm);
};

//
// Class for sending command to PC if using msgpack
//
class MsgToPC {
public:
	double angle[NB_ESC]; 		// Rotation angle, unit degree
	double rpm[NB_ESC]; 		// Rotation speed, unit rpm
	double torque[NB_ESC]; 		// Rotation torque, unit N*m
	double command[NB_ESC];  	// Desired speed, unit rpm
	double acc[3];				// Acceleration of IMU, unit m/s^2
	double gyr[3];				// Gyroscope, unit deg/s
	MSGPACK_DEFINE(angle, rpm, torque, command, acc, gyr);
};
// set a object for sending UDP through msgpack
MsgToPC SendMotorData;



double x = 0.0;

int main(int argc, char *argv[]) {

/********************UDP_Receiving_Initializing********************/
	int sock_recv, length_recv;
	socklen_t fromlen;
	struct sockaddr_in server_recv;
	struct sockaddr_in hold_recv;

	sock_recv = socket(AF_INET, SOCK_DGRAM, 0);
	
	length_recv = sizeof(server_recv);
	bzero(&server_recv,length_recv);
	
	server_recv.sin_family = AF_INET;
	server_recv.sin_addr.s_addr = INADDR_ANY;
	server_recv.sin_port = htons(1000);	// Setting port of this program
	bind(sock_recv,(struct sockaddr *)&server_recv,length_recv);
	fromlen = sizeof(struct sockaddr_in);
	
	vector<MotorSpeed> recv;		// For holding the received class
/******************************************************************/
/*********************UDP_Sending_Initializing*********************/
 	int sock_send, length_send;
	struct sockaddr_in client_send;

	sock_send = socket(AF_INET, SOCK_DGRAM, 0);
	
	length_send = sizeof(client_send);
	bzero(&client_send,length_send);
	
	client_send.sin_family = AF_INET;
	client_send.sin_port = htons(2000); // Port of aim program
	inet_pton(AF_INET, "192.168.0.67", &client_send.sin_addr); // Address
	bind(sock_send,(struct sockaddr *)&client_send,length_send);
	
	vector<MsgToPC> send;			// For holding the sent class
/******************************************************************/

	int ret;
	double RPM[NB_ESC] = {0.0, 0.0, 0.0, 0.0};

	while (1) {
		
		auto start = std::chrono::system_clock::now();
		int datalength = recvfrom(sock_recv, buf_UDP_recv, 200, 0, (struct sockaddr *)&hold_recv, &fromlen);
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		// If using joystick, which sends a list of speed:
		#ifdef UDP_LIST
		int j = 0;
		int pos = 0;
		char num[10];
		for (int i = 0; i < datalength; ++i) {
			if (buf_UDP_recv[i] == ',') {
				memset(num, 0, sizeof(num));
				strncpy(num, buf_UDP_recv + pos, i - pos);
				desire_speed[j] = atoi(num);
				pos = i + 1;
				j++;
			}
		}
		#endif
		
		#ifdef UDP_MSGPACK
		// If using PC to send a msgpack, unpack data into class MotorSpeed type:
		msgpack::object_handle oh = msgpack::unpack(buf_UDP_recv, datalength);
		
		msgpack::object obj = oh.get();
		recv.clear();
		obj.convert(recv);	// Save data into the vector<MotorSpeed> recv
		#endif
		
		// Get the desired speed
		for (int i = 0; i < NB_ESC; ++i)
			desire_speed[i] = recv[0].rpm[i];

		printf("%f\n", elapsed_seconds.count());
		if (elapsed_seconds.count() > 0.01) {
			x = elapsed_seconds.count();
		}
		printf("%f\n", x);
		// Pack class into msgpack and send it to PC
		send.clear();
		send.push_back(SendMotorData);
		msgpack::sbuffer sbuf;
		msgpack::pack(sbuf, send);
		sendto(sock_send, sbuf.data(), sbuf.size(), 0, (struct sockaddr *)&client_send, sizeof(client_send));
		          
	}

	return 0;
}


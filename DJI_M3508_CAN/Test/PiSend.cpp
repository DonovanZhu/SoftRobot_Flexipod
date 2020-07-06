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


#include <chrono>
#include <ctime>
#define NB_ESC 4
using namespace std;

char buf_UDP_recv[200]; 			// for holding UDP data

double desire_speed[NB_ESC] = {0.0, 0.0, 0.0, 0.0};		// The deisred speed


//
// Class for sending command to PC if using msgpack
//
class MotorData {
public:
	double rpm[NB_ESC];
	MSGPACK_DEFINE(rpm);
};
// set a object for sending UDP through msgpack
MotorData SendMotorData;



double x = 0.0;

int main(int argc, char *argv[]) {


/*********************UDP_Sending_Initializing*********************/
 	int sock_send, length_send;
	struct sockaddr_in client_send;

	sock_send = socket(AF_INET, SOCK_DGRAM, 0);
	
	length_send = sizeof(client_send);
	bzero(&client_send,length_send);
	
	client_send.sin_family = AF_INET;
	client_send.sin_port = htons(1000); // Port of aim program
	inet_pton(AF_INET, "192.168.0.87", &client_send.sin_addr); // Address
	bind(sock_send,(struct sockaddr *)&client_send,length_send);
	
	vector<MotorData> send;			// For holding the sent class
/******************************************************************/


	while (1) {
		
		
		for (int k = 0; k < 4; k++) {
			SendMotorData.rpm[k] = 0.0;
		}
		// Pack class into msgpack and send it to PC
		send.clear();
		send.push_back(SendMotorData);
		msgpack::sbuffer sbuf;
		msgpack::pack(sbuf, send);
		auto start = std::chrono::system_clock::now();
		sendto(sock_send, sbuf.data(), sbuf.size(), 0, (struct sockaddr *)&client_send, sizeof(client_send));
		
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		// If using joystick, which sends a list of speed:

		printf("%f\n", elapsed_seconds.count());
		if (elapsed_seconds.count() > 0.01) {
			x = elapsed_seconds.count();
		}
		printf("%f\n", x);
		          
	}

	return 0;
}


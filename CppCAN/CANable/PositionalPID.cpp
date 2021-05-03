#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <msgpack.hpp>
#include <vector>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <iomanip>
#include <iostream>

#define PID_H 10000.0
#define PID_L -10000.0
#define speedDirectionBoundary 32768.0
#define maxBoundary 65535.0
#define drive_ratio 36.0
#define k_p 65.0
#define k_i 1.0
#define k_d 0.00
/*
#define k_p 52.0
#define k_i 1.0
#define k_d 0.00
*/
using namespace std;

/***********************Global***********************/
double time_step_former[4] = {0.0, 0.0, 0.0, 0.0};
double time_step_later[4] = {0.0, 0.0, 0.0, 0.0};
int can_received[4] = {0, 0, 0, 0};
double speed_meas[4] = {0.0, 0.0, 0.0, 0.0};
double dt[4] = {0.0, 0.0, 0.0, 0.0};
double error[4] = {0.0, 0.0, 0.0, 0.0};
double error_former[4] = {0.0, 0.0, 0.0, 0.0};
double error_sum[4] = {0.0, 0.0, 0.0, 0.0};
double command[4] = {0.0, 0.0, 0.0, 0.0};
double desire_speed[4] = {0.0, 0.0, 0.0, 0.0};
int angle;
int rpm;
int torque;
struct can_frame frame_recv;
struct can_frame frame_send;

char buf_send[200];
char buf_recv[200];

struct timespec time1;
struct timespec time2;

double x = 0.0;

class MotorSpeed {
public:
	int rpm[4];
	MSGPACK_DEFINE(rpm);
};

class MsgToPC {
public:
	double angle[4];
	double rpm[4];
	double torque[4];
	MSGPACK_DEFINE(angle, rpm, torque);
};

MsgToPC SendMotorData;

void CAN_control(int s)
{
	int i = 0;
	while(1)
	{
		read(s, &frame_recv, sizeof(struct can_frame));

		if (!can_received[frame_recv.can_id - 513])
		{
			clock_gettime(CLOCK_REALTIME, &time1);
			time_step_later[frame_recv.can_id - 513] = time1.tv_sec + time1.tv_nsec / 1000000000.0;
			rpm = 0;
			rpm |= (int16_t)(unsigned char)frame_recv.data[2] << 8;
			rpm |= (int16_t)(unsigned char)frame_recv.data[3];
			if (rpm >= speedDirectionBoundary)
				speed_meas[frame_recv.can_id - 513] = -(maxBoundary - rpm) / drive_ratio;
			else
				speed_meas[frame_recv.can_id - 513] = rpm / drive_ratio;
			can_received[frame_recv.can_id - 513] = true;
			
			angle = 0;
			angle |= (int16_t)(unsigned char)frame_recv.data[0] << 8;
			angle |= (int16_t)(unsigned char)frame_recv.data[1];
			SendMotorData.angle[i] = (double)angle / 8191.0 * 360.0;		
			SendMotorData.rpm[i] = speed_meas[i];		
			torque = 0;
			torque |= (int16_t)(unsigned char)frame_recv.data[4] << 8;
			torque |= (int16_t)(unsigned char)frame_recv.data[5];
			SendMotorData.torque[i] = (double)torque;
			i++;
		}
		if (i == 4)
		{
			for (int k = 0; k < 4; ++k)
				can_received[k] = 0;
			break;
		}
	}
	
	for (i = 0; i < 4; ++i)
	{
		dt[i] = time_step_later[i] - time_step_former[i];
		printf("v = %f  ",speed_meas[i]);
		printf("t = %f  ", dt[i]);
		/***********************PID***********************/
		time_step_former[i] = time_step_later[i];
		error[i] = desire_speed[i] - speed_meas[i];
		
		
		if (dt[i] < 0.002 && abs(error[i]) < 100.0)
		{
			error_sum[i] += error[i];
			command[i] = k_p * (error[i] + error_sum[i] * 0.001 + k_d * (error[i] - error_former[i]) / dt[i]);
		}
		else if (dt[i] < 0.002 && abs(error[i]) >= 100.0)
		{
			error_sum[i] += error[i];
			command[i] = 0.5 * k_p * error[i];
		}
		error_former[i] = error[i];
		
		if(command[i] >= 0)
		{
			if(command[i] > PID_H)
				command[i] = PID_H;
			int v = int(command[i]);
			// int v = int((command[i] / PID_H) * speedDirectionBoundary);
			frame_send.data[2 * i + 1] = v & 0x00ff;
			frame_send.data[2 * i] = (v >> 8) & 0x00ff;
			
		}
		else
		{
			if(command[i] < PID_L)
				command[i] = PID_L;
			int v = 0xffff + int(command[i]);
			// int v = 0xffff - int((command[i] / PID_L) * speedDirectionBoundary);
			frame_send.data[2 * i + 1] = v & 0x00ff;
			frame_send.data[2 * i] = (v >> 8) & 0x00ff;
		}
	}
	printf("\n");
	
	write(s, &frame_send, sizeof(can_frame)); 

}

void control_init(int s)
{
	int i = 0;	
	while(1)
	{

		read(s, &frame_recv, sizeof(struct can_frame));
		if (!can_received[frame_recv.can_id - 513])
		{
			clock_gettime(CLOCK_REALTIME, &time1);
			time_step_former[frame_recv.can_id - 513] = time1.tv_sec + time1.tv_nsec / 1000000000.0;
			rpm = 0;
			rpm |= (int16_t)(unsigned char)frame_recv.data[2] << 8;
			rpm |= (int16_t)(unsigned char)frame_recv.data[3];
			if (rpm >= speedDirectionBoundary)
				speed_meas[frame_recv.can_id - 513] = -(maxBoundary - rpm) / drive_ratio;
			else
				speed_meas[frame_recv.can_id - 513] = rpm / drive_ratio;
			can_received[frame_recv.can_id - 513] = true;
			i++;
		}
		if (i == 4)
		{
			for (int k = 0; k < 4; ++k)
			{
				can_received[k] = 0;
				error_former[k] = desire_speed[k] - speed_meas[k];
			}
			break;
		}
		
	}
}

int main(int argc, char **argv)
{
/***********************CAN_Bus_Initializing***********************/
	int s; 
	struct sockaddr_can addr;
	struct ifreq ifr;

	system("sudo slcand -o -c -s8 /dev/ttyACM0");
	system("sudo ip link set up slcan0");

	s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	strcpy(ifr.ifr_name, "slcan0" );
	ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	
	bind(s, (struct sockaddr *)&addr, sizeof(addr));
/******************************************************************/

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
	server_recv.sin_port = htons(1000);
	bind(sock_recv,(struct sockaddr *)&server_recv,length_recv);
	fromlen = sizeof(struct sockaddr_in);
	vector<MotorSpeed> recv;
/******************************************************************/

/*********************UDP_Sending_Initializing*********************/
 	int sock_send, length_send;
	struct sockaddr_in client_send;

	sock_send = socket(AF_INET, SOCK_DGRAM, 0);
	
	length_send = sizeof(client_send);
	bzero(&client_send,length_send);
	
	client_send.sin_family = AF_INET;
	client_send.sin_port = htons(2000);
	inet_pton(AF_INET, "192.168.0.67", &client_send.sin_addr);
	bind(sock_send,(struct sockaddr *)&client_send,length_send);
	vector<MsgToPC> send;
/******************************************************************/
	control_init(s);
	
	frame_send.can_id = 0x200;
	frame_send.can_dlc = 8;
	char num[10];
	int pos;
	int k;
	int j;

	while(1)
	{
		pos = 0;
		j = 0;
		int datalength = recvfrom(sock_recv, buf_recv, 200, 0, (struct sockaddr *)&hold_recv, &fromlen);
		// Char array:
		for (int i = 0; i < datalength; ++i)
		{
			if (buf_recv[i] == ',')
			{
				memset(num, 0, sizeof(num));
				strncpy(num, buf_recv + pos, i - pos);
				k = atoi(num);
				desire_speed[j] = (double)(k - 0x4000) / (double)(0x4000) * 400.0;
				//desire_speed[0] = 200.0;
				pos = i + 1;
				//printf("%f ", desire_speed[j]);
				j++;
			}
		}
		
		// Msgpack:
		/*
		msgpack::object_handle oh = msgpack::unpack(buf_recv, datalength);
		
		msgpack::object obj = oh.get();
		recv.clear();
		obj.convert(recv);
		for (int i = 0; i < 4; i++)
		{
			desire_speed[i] = recv[0].rpm[i];
			cout << desire_speed[i] << " ";
		}
		cout << endl;
		*/

		CAN_control(s);

		send.clear();
		send.push_back(SendMotorData);
		msgpack::sbuffer sbuf;
		msgpack::pack(sbuf, send);
		sendto(sock_send, sbuf.data(), sbuf.size(), 0, (struct sockaddr *)&client_send, sizeof(client_send));
		//usleep(1000);
	}
	
	printf("\r\n");

	if (close(s) < 0) 
	{
		perror("Close");
		return 1;
	}

	return 0;
}

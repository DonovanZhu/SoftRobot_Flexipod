#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
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



#define PID_H 1000.0
#define PID_L -1000.0
#define speedDirectionBoundary 32768.0
#define maxBoundary 65535.0
#define drive_ratio 36.0


using namespace std;

double time_step_former[4] = {0.0, 0.0, 0.0, 0.0};
double time_step_later[4] = {0.0, 0.0, 0.0, 0.0};
int can_received[4] = {0, 0, 0, 0};
double speed_meas[4] = {0.0, 0.0, 0.0, 0.0};

int rpm;
struct can_frame frame_recv;
struct can_frame frame_send;


double dt[4] = {0.0, 0.0, 0.0, 0.0};
double error[4] = {0.0, 0.0, 0.0, 0.0};
double error_former[4] = {0.0, 0.0, 0.0, 0.0};
double error_sum[4] = {0.0, 0.0, 0.0, 0.0};
double command[4] = {0.0, 0.0, 0.0, 0.0};

static double k_p = 1.0;
static double k_i = 0.00000;
static double k_d = 0.0;


double desire_speed[4] = {0.0, 0.0, 0.0, 0.0};

class MotorSpeed {
public:
	double rpm[4];
	MSGPACK_DEFINE(rpm);
};

void CAN_control(int s)
{
	int i = 0;	
	while(1)
	{
		read(s, &frame_recv, sizeof(struct can_frame));
		if (!can_received[frame_recv.can_id - 513])
		{
			time_step_later[frame_recv.can_id - 513] = clock();
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
				can_received[k] = 0;
			break;
		}
	}
	
	for (i = 0; i < 4; ++i)
	{
		dt[i] = (double)(time_step_later[i] - time_step_former[i]);
		printf("v = %f  ",speed_meas[i]);
		printf("t = %f  ", dt[i] / CLOCKS_PER_SEC);
		
		/***********************PID***********************/
		time_step_former[i] = time_step_later[i];
		error[i] = desire_speed[i] - speed_meas[i];
		error_sum[i] += error[i];
		command[i] = k_p * (error[i] + error_sum[i] * dt[i] * k_i + k_d * (error[i] - error_former[i]) / dt[i]);
		error_former[i] = error[i];
		
		if(command[i] >= 0)
		{
			if(command[i] > PID_H)
				command[i] = PID_H;
			int v = int((command[i] / PID_H) * speedDirectionBoundary);
			frame_send.data[2 * i + 1] = v & 0x00ff;
			frame_send.data[2 * i] = (v >> 8) & 0x00ff;
			
		}
		else
		{
			if(command[i] < PID_L)
				command[i] = PID_L;
			int v = 0xffff - int((command[i] / PID_L) * speedDirectionBoundary);
			frame_send.data[2 * i + 1] = v & 0x00ff;
			frame_send.data[2 * i] = (v >> 8) & 0x00ff;
		}

	}

	int nbytes = write(s, &frame_send, sizeof(frame_send)); 
	if(nbytes != sizeof(frame_send)) 
	{
		printf("Send Error frame[0]!\r\n");
		system("sudo ifconfig slcan0 down");
	}

	printf("\n");
	
}

void control_init(int s)
{
	int i = 0;	
	while(1)
	{
		read(s, &frame_recv, sizeof(struct can_frame));
		if (!can_received[frame_recv.can_id - 513])
		{
			time_step_later[frame_recv.can_id - 513] = clock();
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

/*************************UDP_Initializing*************************/
	int sock_recv, length_recv;
	socklen_t fromlen;
	struct sockaddr_in server_recv;
	struct sockaddr_in from;

	sock_recv = socket(AF_INET, SOCK_DGRAM, 0);
	
	length_recv = sizeof(server_recv);
	bzero(&server_recv,length_recv);
	
	server_recv.sin_family = AF_INET;
	server_recv.sin_addr.s_addr = INADDR_ANY;
	server_recv.sin_port = htons(10000);
	bind(sock_recv,(struct sockaddr *)&server_recv,length_recv);
		   
	fromlen = sizeof(struct sockaddr_in);
	
	char buf[200];
	vector<MotorSpeed> recv;
/******************************************************************/
	
	control_init(s);
	
	frame_send.can_id = 0x200;
	frame_send.can_dlc = 8;
	while(1)
	{
		int datalength = recvfrom(sock_recv, buf, 200, 0, (struct sockaddr *)&from, &fromlen);
		msgpack::object_handle oh = msgpack::unpack(buf, datalength);
		
		msgpack::object obj = oh.get();
		recv.clear();
		obj.convert(recv);
		for (int i = 0; i < 4; i++)
		{
			desire_speed[i] = recv[0].rpm[i];
		}
		
		CAN_control(s);
	}
	
	printf("\r\n");

	if (close(s) < 0) 
	{
		perror("Close");
		return 1;
	}

	return 0;
}

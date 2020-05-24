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

#define PID_H 10000.0 //Upper limit of PID output
#define PID_L -10000.0 //lower limit of PID output
#define drive_ratio 36.0 // Drive ratio of motor gear box
#define speedDirectionBoundary 32768.0 //32768(dec) = 0x 8000
// if speed command higher than 32768, the motor rotates clockwise
// if speed command lower than 32768, the motor rotates counter-clockwise
#define maxBoundary 65535.0 // 0xffff, command upper limit
// PID:
#define k_p 24.5
#define k_i 0.035
#define k_d 0.00

using namespace std;

/***********************Global***********************/
double time_step_former[4] = {0.0, 0.0, 0.0, 0.0}; //time of former step
double time_step_later[4] = {0.0, 0.0, 0.0, 0.0}; // time of current step
int can_received[4] = {0, 0, 0, 0}; // Bool array to check if the CAN signal of 4 motors is received
double speed_meas[4] = {0.0, 0.0, 0.0, 0.0}; // measured speed
double dt[4] = {0.0, 0.0, 0.0, 0.0}; // time step, the difference between time_step_later and time_step_former
double error[4] = {0.0, 0.0, 0.0, 0.0}; // current step error
double error_former[4] = {0.0, 0.0, 0.0, 0.0}; // former step error

double command[4] = {0.0, 0.0, 0.0, 0.0}; //control loop output
double desire_speed[4] = {0.0, 0.0, 0.0, 0.0};



int angle;
int rpm;
int torque;
struct can_frame frame_recv; // for holding CAN data received from motors
struct can_frame frame_send; // for holding CAN data sent to motors

char buf_send[200]; // for holding UDP data send to PC
char buf_recv[200]; // for holding UDP data received from Joystick

struct timespec time1; //for timer
struct timespec time2;

// class for receiving command from Joystick if using msgpack
class MotorSpeed {
public:
	int rpm[4];
	MSGPACK_DEFINE(rpm);
};


// class for sending command to PC if using msgpack
class MotorData {
public:
	double angle[4]; //degree
	double rpm[4]; //rpm
	double torque[4]; //N*m
	MSGPACK_DEFINE(angle, rpm, torque);
};

// set a object for sending UDP through msgpack
MotorData SendMotorData;

// PID control
void CAN_control(int s)
{
	int i = 0;
	// loop while 4 motors' CAN data received
	while(1)
	{
		// read from CAN bus
		read(s, &frame_recv, sizeof(struct can_frame));
		// check if the specific motor's data is received
		// 513(dec) = 0x201(hex) whcih is the CAN id of motor1
		// the ID of motor 1 to 4 is : 0x201(513)/0x202(514)/0x203(515)/0x204(516)
		if (!can_received[frame_recv.can_id - 513])
		{
			// timer
			clock_gettime(CLOCK_REALTIME, &time1);
			time_step_later[frame_recv.can_id - 513] = time1.tv_sec + time1.tv_nsec / 1000000000.0;
			
			// the CAN data is a 8 length char array.
			// data[0]/data[1]: high/low byte of motor angle
			// data[2]/data[3]: high/low byte of motor speed
			// data[4]/data[5]: high/low byte of motor torque
			// data[6]/data[7]: NULL
			
			rpm = 0;
			rpm |= (int16_t)(unsigned char)frame_recv.data[2] << 8;
			rpm |= (int16_t)(unsigned char)frame_recv.data[3];
			
			// check the rotation direction of motor
			if (rpm >= speedDirectionBoundary)
				speed_meas[frame_recv.can_id - 513] = -(maxBoundary - rpm) / drive_ratio;
			else
				speed_meas[frame_recv.can_id - 513] = rpm / drive_ratio;
			can_received[frame_recv.can_id - 513] = true;
			
			//receive angle and torque data, put the data into object SendMotorData
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
			// reset bool array can_received
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
		
		//PID controller
		command[i] += k_p * (error[i] - error_former[i]) + k_i * error[i];
		error_former[i] = error[i];
		
		/*
		set command accorrding to the direction
		the command is the current value of motor
		data is stored in a 8 length array frame_send.data
		data[0]/data[1]: high/low byte of motor1 current command
		data[2]/data[3]: high/low byte of motor2 current command
		data[4]/data[5]: high/low byte of motor3 current command
		data[6]/data[7]: high/low byte of motor4 current command
		*/
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
	
	// write command to CAN bus
	write(s, &frame_send, sizeof(can_frame)); 

}

// Initialize control loop
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
	
	// Open CAN bus
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
	vector<MotorData> send;
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
		/*********************UDP_Receiving*********************/
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
				desire_speed[0] = 200.0;
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
		/*******************************************************/
		CAN_control(s);
		/*********************UDP_Sending**********************/
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

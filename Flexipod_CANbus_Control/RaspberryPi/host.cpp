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

using namespace std;

//#define  HOST_MODEMDEVICE		"/dev/serial/by-id/usb-Teensyduino_USB_Serial_6582050-if00"
#define HOST_MODEMDEVICE    	"/dev/ttyACM0"	// Name of USB port
#define HOST_DEV_SERIALNB		6582050			// Serial number of the teensy, check this number by using terminal
#define HOST_DEV_SERIALLG		10				// Max length of a serial number
#define HOST_SERIAL_DEV_DIR		"/dev/serial/by-id/"
#define HOST_BAUDRATE       	B115200			// Serial baudrate

#define drive_ratio 			36.0 			// Drive ratio of motor gear box
#define speedDirectionBoundary 	32768.0 		// 32768(dec) = 0x 8000
// if speed command higher than 32768, the motor rotates clockwise
// if speed command lower than 32768, the motor rotates counter-clockwise
#define maxBoundary 			65535.0 		// 0xffff, command upper limit


// Globals
int Host_fd = HOST_ERROR_FD;        // Serial port file descriptor
                            
char Host_devname[PATH_MAX] = ""; 	// Serial port devname used to get fd with open
                            
struct termios Host_oldtio;  		// Backup of initial tty configuration

Teensycomm_struct_t Teensy_comm;	// A data struct received from Teensy
RPicomm_struct_t	RPi_comm;		// A data struct sent to Teensy

char buf_UDP_recv[200]; 			// for holding UDP data


double desire_speed[4] = {0.0, 0.0, 0.0, 0.0};		// The deisred speed
double angle_former[4] = {0.0, 0.0, 0.0, 0.0};		// Rotation angle of each motor's rotor in the former step
double angle_cur[4]    = {0.0, 0.0, 0.0, 0.0};		// Rotation angle of each motor's rotor in the current step
double angle_sum[4]    = {0.0, 0.0, 0.0, 0.0};		// Rotation angle of each motor's shaft in the current step
bool   angle_init[4]   = {true, true, true, true};	// To mark the first step of rotation angle

// Receiving the desired speed data by UDP and hold it in this class
class MotorSpeed {
public:
	double rpm[4];
	MSGPACK_DEFINE(rpm);
};

//
// Class for sending command to PC if using msgpack
//
class MotorData {
public:
	double angle[4]; 	// Rotation angle, unit degree
	double rpm[4]; 		// Rotation speed, unit rpm
	double torque[4]; 	// Rotation torque, unit N*m
	double command[4];  // Desired speed, unit rpm
	double acc[3];		// Acceleration of IMU, unit m/s^2
	double gyr[3];		// Gyroscope, unit deg/s
	MSGPACK_DEFINE(angle, rpm, torque, acc, gyr);
};
// set a object for sending UDP through msgpack
MotorData SendMotorData;

//
// Get the device name from the device serial number
//
char *Host_name_from_serial(uint32_t serial_nb) {
	DIR				*d;
	struct dirent	*dir;
	char			serial_nb_char[HOST_DEV_SERIALLG];
	static char		portname[PATH_MAX];
  
	// Convert serial number into string
	snprintf(serial_nb_char, HOST_DEV_SERIALLG, "%d", serial_nb);
  
	// Open directory where serial devices can be found
	d = opendir(HOST_SERIAL_DEV_DIR);

	// Look for a device name contining teensy serial number
	if (d) {
		// Scan each file in the directory
		while ((dir = readdir(d)) != NULL) {
		if (strstr(dir->d_name, serial_nb_char)) {

			// A match is a device name containing the serial number
			snprintf(portname, PATH_MAX, "%s%s", HOST_SERIAL_DEV_DIR, dir->d_name);
			return portname;
			}
		}
	closedir( d );
	}
	return NULL;
}

//
// Get the file descriptor index which device name contains
// specified serial number. 
// Returns -1 if no matching fd is found.
//
int Host_get_fd(uint32_t serial_nb) {
	int   i;
	char  serial_nb_char[HOST_DEV_SERIALLG];
  
	// Convert serial number into string
	snprintf(serial_nb_char, HOST_DEV_SERIALLG, "%d", serial_nb);
    
	if (Host_fd != HOST_ERROR_FD)
		if (strstr(Host_devname, serial_nb_char))
			return 0;

	return HOST_ERROR_FD;
}

//
// Initialize serial port
//
int Host_init_port(uint32_t serial_nb) {
	struct	termios newtio;
	int     check_fd;
	int     i;
	char    *portname;
  
	// Check if device plugged in
	portname = Host_name_from_serial(serial_nb);
	if (!portname)
		return HOST_ERROR_DEV;

	// Open device
	check_fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
  
	if (check_fd < 0) {
		perror(portname);
		return HOST_ERROR_DEV;
	}
  
	// Store the fd and the corresponding devname
	Host_fd = check_fd;
	strncpy(Host_devname, portname, PATH_MAX);
  
	// Initialize corresponding data structure
	for (i = 0; i < NB_ESC; i++) {
		RPi_comm.magic = COMM_MAGIC;
		RPi_comm.RPM[i] =  0;
	}

	/* Save current port settings */
	tcgetattr(check_fd, &Host_oldtio);

	/* Define new settings */
	bzero(&newtio, sizeof(newtio));
	cfmakeraw(&newtio);

	newtio.c_cflag 	   =  HOST_BAUDRATE | CS8 | CLOCAL | CREAD;
	newtio.c_iflag 	   =  IGNPAR;
	newtio.c_oflag     =  0;
	newtio.c_lflag     =  0;
	newtio.c_cc[VTIME] =  0;
	newtio.c_cc[VMIN]  =  0;

	/* Apply the settings */
	tcflush(check_fd, TCIFLUSH);
	tcsetattr(check_fd, TCSANOW, &newtio);

	return 0;
}

//
// Release serial port
//
void Host_release_port(uint32_t serial_nb) {
	int fd_idx;
    
	// Get fd index from serial number
	fd_idx = Host_get_fd(serial_nb);
  
	if (fd_idx != HOST_ERROR_FD) {
		// Restore initial settings if needed
		tcsetattr(Host_fd, TCSANOW, &Host_oldtio);
		close(Host_fd);
    
		// Clear fd and corresponding devname
		Host_fd = HOST_ERROR_FD;
		strncpy(Host_devname, "", PATH_MAX);
	}
}

//
// Manage communication with the teensy connected to portname
//
int Host_comm_update( uint32_t            serial_nb,
                      double              *RPM,
                      Teensycomm_struct_t **comm ) {
                      
	int                 i, ret, res = 0;
	uint8_t             *pt_in;
  
	// Get fd index
	Host_get_fd(serial_nb);
  
	// Update output data structue
	for (i = 0; i < NB_ESC; i++) {
		RPi_comm.RPM[i] = desire_speed[i];
	}

	// Send output structure
	res = write(Host_fd, &RPi_comm, sizeof(RPi_comm));
	if (res < 0) {
		perror("write RPi_comm");
		return HOST_ERROR_WRITE_SER;
	}
  
	// Flush output buffer
	fsync(Host_fd);

	// Reset byte counter and magic number
	res = 0;
	Teensy_comm.magic = 0;
	pt_in = (uint8_t*)(&Teensy_comm);

	do {
		ret = read(Host_fd, &pt_in[res], 1);

		// Data received
		if (ret > 0) 
			res += ret;	

		// Read error
		if (ret < 0)
			break;
	} while (res < sizeof(Teensy_comm));

	// Check response size
	if (res != sizeof(Teensy_comm)) {
		fprintf(stderr, "Packet with bad size received.\n");

		// Flush input buffer
		while ((ret = read(Host_fd, pt_in, 1)))
			if (ret <= 0)
				break;
        
		return HOST_ERROR_BAD_PK_SZ;
	}

	// Check magic number
	if (Teensy_comm.magic !=  COMM_MAGIC) {
		fprintf( stderr, "Invalid magic number.\n" );
		return HOST_ERROR_MAGIC;
	}
  
	// Return pointer to Teensy_comm structure
	*comm = &Teensy_comm;

	return 0;
}

//
// The angle data sent from motor is the rotation angle
// of the rotor. This function is for transfering the
// rotor angle to shaft angle. Inputs are rotor angle,
// index of motor and speed of motor. Results are saved
// in list "angle_sum".
// 
void angle_calculate(double angle, int i, double speed) {
	// Check the first step and initialize
	if (angle_init[i]) {
		angle_former[i] = angle;
		angle_init[i] = false;
	}
	else {
		// Difference between current angle and the former one
		double d_angle = angle - angle_former[i];
		
		// When motor rotates in clockwise, if current angle
		// acrosses 0 degree line, which means difference is
		// minus, the difference should be added 360 degree to
		// compensate. Vice versa, when motor moving in counter
		// clockwise, the difference needs to be subtracted 360.
		if (speed >= 0.0 && d_angle < 0.0) 
			d_angle += 360.0;
		
		else if (speed < 0.0 && d_angle > 0.0) 
			d_angle -= 360.0;
		
		// Angle of rotor to angle of shaft
		angle_sum[i] += d_angle / drive_ratio;
		
		// Set the shaft angle between 0 - 360
		if (angle_sum[i] >= 360.0)
			angle_sum[i] -= 360.0;
		else if (angle_sum[i] < 0.0)
			angle_sum[i] += 360.0;

		angle_former[i] = angle;
	}
}


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
	
	vector<MotorData> send;			// For holding the sent class
/******************************************************************/

	int data_num, k, ret, j, pos;
	char num[10];
	double RPM[NB_ESC] = {0.0, 0.0, 0.0, 0.0};
	Teensycomm_struct_t *comm;
  
	// Initialize serial port
	if (Host_init_port(HOST_DEV_SERIALNB)) {
		fprintf(stderr, "Error initializing serial port.\n");
		exit(-1);
	}

	while (1) {
		pos = 0;
		j = 0;
		int datalength = recvfrom(sock_recv, buf_UDP_recv, 200, 0, (struct sockaddr *)&hold_recv, &fromlen);
		// If using joystick, which sends a list of speed:
		/*
		for (int i = 0; i < datalength; ++i) {
			if (buf_UDP_recv[i] == ',') {
				memset(num, 0, sizeof(num));
				strncpy(num, buf_UDP_recv + pos, i - pos);
				desire_speed[j] = atoi(num);
				pos = i + 1;
				j++;
			}
		}
		*/
		
		// If using PC to send a msgpack, unpack data into class MotorSpeed type:
		msgpack::object_handle oh = msgpack::unpack(buf_UDP_recv, datalength);
		
		msgpack::object obj = oh.get();
		recv.clear();
		obj.convert(recv);	// Save data into the vector<MotorSpeed> recv
		
		// Get the desired speed
		for (int i = 0; i < 4; ++i)
			desire_speed[i] = recv[0].rpm[i];

		// Serial exchange with teensy
		if ((ret = Host_comm_update(HOST_DEV_SERIALNB, RPM, &comm))) {
			fprintf(stderr, "Error %d in Host_comm_update.\n", ret);
			break;
		}

		// After receiving the data from Teensy, save it into class SendMotorData
		for (k = 0; k < NB_ESC; ++k) {
			
			// Speed of motors:
			if (comm->rpm[k] >= speedDirectionBoundary)
				SendMotorData.rpm[k] = -(double)(maxBoundary - comm->rpm[k]) / drive_ratio;
			else
				SendMotorData.rpm[k] = (double)comm->rpm[k] / drive_ratio;
			
			// Angle of motors:
			angle_calculate((double)comm->deg[k] / 8191.0 * 360.0, k, SendMotorData.rpm[k]);
			SendMotorData.angle[k] = angle_sum[k];
			
			// Torque of motors:
			SendMotorData.torque[k] = (double)comm->torque[k];
			SendMotorData.command[k] = desire_speed[k];
		}
		//printf("%f\n", SendMotorData.rpm[0]);
		// Data from IMU:
		for (k = 0; k < 3; ++k) {
			SendMotorData.acc[k] = (double)comm->acc[k];
			SendMotorData.gyr[k] = (double)comm->gyr[k];
		}
		
		// Pack class into msgpack and send it to PC
		send.clear();
		send.push_back(SendMotorData);
		msgpack::sbuffer sbuf;
		msgpack::pack(sbuf, send);
		sendto(sock_send, sbuf.data(), sbuf.size(), 0, (struct sockaddr *)&client_send, sizeof(client_send));
		          
	}

	// Restoring serial port initial configuration
	Host_release_port(HOST_DEV_SERIALNB);

	return 0;
}


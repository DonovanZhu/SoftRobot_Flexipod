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
#include <sys/mman.h>
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

// Globals
int Host_fd = HOST_ERROR_FD;        // Serial port file descriptor
                            
char Host_devname[PATH_MAX] = ""; 	// Serial port devname used to get fd with open
                            
struct termios Host_oldtio;  		// Backup of initial tty configuration

Teensycomm_struct_t Teensy_comm;	// A data struct received from Teensy
RPicomm_struct_t	RPi_comm;		// A data struct sent to Teensy

double 	desire_speed = 0.0;		// The deisred speed

// Receiving the desired speed data by UDP and hold it in this class
class MotorSpeed {
public:
	double 	robot_command;
	MSGPACK_DEFINE(robot_command);
};

//
// Class for sending command to PC if using msgpack
//
class MotorData {
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
	closedir(d);
	}
	return NULL;
}

//
// Get the file descriptor index which device name contains
// specified serial number. 
// Returns -1 if no matching fd is found.
//
int Host_get_fd(uint32_t serial_nb) {
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
	RPi_comm.speedcommand =  0.0;

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
                      
	int                 ret, res = 0;
	uint8_t             *pt_in;
  
	// Get fd index
	Host_get_fd(serial_nb);
  
	// Update output data structue
	RPi_comm.speedcommand = desire_speed;
	

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
  
	// Return pointer to Teensy_comm structure
	*comm = &Teensy_comm;

	return 0;
}

void* create_shared_memory(size_t size) {
  // Our memory buffer will be readable and writable:
  int protection = PROT_READ | PROT_WRITE;

  // The buffer will be shared (meaning other processes can access it), but
  // anonymous (meaning third-party processes cannot obtain an address for it),
  // so only this process and its children will be able to use it:
  int visibility = MAP_SHARED | MAP_ANONYMOUS;

  // The remaining parameters to `mmap()` are not important for this use case,
  // but the manpage for `mmap` explains their purpose.
  return mmap(NULL, size, protection, visibility, -1, 0);
}

int main( ) {
	int init_command[2] = {0, 0};
	int* shmem = (int*)create_shared_memory(sizeof(int) * 2);
	memcpy(shmem, init_command, sizeof(init_command));
	
	int pid = fork();

	if (pid == 0) {
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
		server_recv.sin_port = htons(8);	// Setting port of this program
		bind(sock_recv,(struct sockaddr *)&server_recv,length_recv);
		fromlen = sizeof(struct sockaddr_in);
		vector<MotorSpeed> recv;		// For holding the received class
	/******************************************************************/
		char buf_UDP_recv[100]; 			// for holding UDP data
		int command_recv[1] = {0};
		while (1) {
			
			int datalength = recvfrom(sock_recv, buf_UDP_recv, 100, 0, (struct sockaddr *)&hold_recv, &fromlen);
			msgpack::object_handle oh = msgpack::unpack(buf_UDP_recv, datalength);
			msgpack::object obj = oh.get();
			recv.clear();
			obj.convert(recv);
			command_recv[0] = (int)(recv[0].robot_command * 100000.0);
			memcpy(shmem, command_recv, sizeof(command_recv));
		}
	} 
	
	else {
	/*********************UDP_Sending_Initializing*********************/
		int sock_send, length_send;
		struct sockaddr_in client_send;

		sock_send = socket(AF_INET, SOCK_DGRAM, 0);
		
		length_send = sizeof(client_send);
		bzero(&client_send,length_send);
		
		client_send.sin_family = AF_INET;
		client_send.sin_port = htons(2000); // Port of aim program
		inet_pton(AF_INET, "192.168.8.109", &client_send.sin_addr); // Address
		bind(sock_send,(struct sockaddr *)&client_send,length_send);
		vector<MotorData> send;			// For holding the sent class
	/******************************************************************/
	
		Teensycomm_struct_t *comm;
		double RPM[NB_ESC] = {0.0, 0.0, 0.0, 0.0};
		int ret;
		
		// Initialize serial port
		if (Host_init_port(HOST_DEV_SERIALNB)) {
			fprintf(stderr, "Error initializing serial port.\n");
			exit(-1);
		}


		while(1) {
			
			desire_speed = (double) *shmem / 100000.0 ;

			// Serial exchange with teensy
			if ((ret = Host_comm_update(HOST_DEV_SERIALNB, RPM, &comm))) {
				fprintf(stderr, "Error %d in Host_comm_update.\n", ret);
				break;	
			}
			// After receiving the data from Teensy, save it into class SendMotorData
			for (int j = 0; j < NB_ESC; ++j) {
				
				// Speed of motors:
				SendMotorData.rpm[j] = comm -> rspeed[j];
				
				// Angle of motors:
				SendMotorData.angle[j] = comm -> angle[j];
				
				// Torque of motors:
				SendMotorData.torque[j] = comm -> torque[j];
				// Desired speed:
				SendMotorData.command[j] = comm -> motor_command[j];
			}
			
			// Data from IMU:
			for (int k = 0; k < 3; ++k) {
				SendMotorData.acc[k] = (double)comm->acc[k];
				SendMotorData.gyr[k] = (double)comm->gyr[k];
			}
			//if (SendMotorData.acc[0] > 1050)
				printf("%f\t%f\n", SendMotorData.acc[0], SendMotorData.acc[1]);
			// printf("%f\t%f\t%f\t%f\n", SendMotorData.angle[0], SendMotorData.angle[1], SendMotorData.angle[2], SendMotorData.angle[3]);
			// Pack class into msgpack and send it to PC
			send.clear();
			send.push_back(SendMotorData);
			msgpack::sbuffer sbuf;
			msgpack::pack(sbuf, send);
			sendto(sock_send, sbuf.data(), sbuf.size(), 0, (struct sockaddr *)&client_send, sizeof(client_send));

		}
		Host_release_port(HOST_DEV_SERIALNB);
	}
	return 0;
}


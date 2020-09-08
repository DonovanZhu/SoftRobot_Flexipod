#ifndef __HOST_H
#define __HOST_H

// Defines
#define UDP_LIST				0
#define UDP_MSGPACK				1
#define HOST_ERROR_FD         	-1			// Non existant file descriptor
#define HOST_ERROR_DEV        	-2			// Non existant serial device
#define HOST_ERROR_MAX_DEV    	-3			// Maximum devices limit 
#define HOST_ERROR_WRITE_SER  	-4			// Write error on serial
#define HOST_ERROR_BAD_PK_SZ  	-5			// Bad incoming packet size error

#define NB_ESC				   	4			// Number of ESCs

#define USB_UART_SPEED     		1000000		// Baudrate of the teeensy USB serial link

//#define  HOST_MODEMDEVICE		"/dev/serial/by-id/usb-Teensyduino_USB_Serial_6581650-if00"
#define HOST_MODEMDEVICE    	"/dev/ttyACM0"	// Name of USB port
#define HOST_DEV_SERIALNB		6245540			// Serial number of the teensy, check this number by using terminal
#define HOST_DEV_SERIALLG		10				// Max length of a serial number
#define HOST_SERIAL_DEV_DIR		"/dev/serial/by-id/"
#define HOST_BAUDRATE       	B1000000		// Serial baudrate
#define DRIVE_RATIO 			19.20321 		// Drive ratio of motor gear box

// if speed command higher than 32768, the motor rotates clockwise
// if speed command lower than 32768, the motor rotates counter-clockwise
#define DirectionBoundary 		32768.0 		// 32768(dec) = 0x 8000
#define maxBoundary 			65535.0 		// 0xffff, command upper limit

// Teensy->host communication data structure
// sizeof(ESCPID_comm)=64 to match USB 1.0 buffer size
typedef struct {
  double    angle[NB_ESC];      // Motors rotation angle
  double    rspeed[NB_ESC];     // Motors rpm
  double	  torque[NB_ESC];     // Motors torque
  double    motor_command[NB_ESC];    // Motors speed command
  double    acc[3];						  // Acceleration in X Y Z direction, m/s^2
  double    gyr[3];						  // Gyroscope in X Y Z direction, deg/s
  double    mag[3];             // Magnetometer in X Y Z, uT
  double    eular[3];
  double    timestamps;
} Teensycomm_struct_t;

// Host->teensy communication data structure
// sizeof(RPi_comm)=64 to match USB 1.0 buffer size
typedef struct {
  double    speedcommand[4];        		// Desired Speed, rpm
} RPicomm_struct_t;

// Prototypes
char *Host_name_from_serial(uint32_t);
int   Host_get_fd(uint32_t);
int   Host_init_port(uint32_t);
void  Host_release_port(uint32_t);
int   Host_comm_update(uint32_t, double*, Teensycomm_struct_t**);

#endif

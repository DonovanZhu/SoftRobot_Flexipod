#ifndef __HOST_H
#define __HOST_H

// Defines
#define HOST_ERROR_FD         -1        // Non existant file descriptor
#define HOST_ERROR_DEV        -2        // Non existant serial device
#define HOST_ERROR_MAX_DEV    -3        // Maximum devices limit 
#define HOST_ERROR_WRITE_SER  -4        // Write error on serial
#define HOST_ERROR_BAD_PK_SZ  -5        // Bad incoming packet size error
#define HOST_ERROR_MAGIC      -6        // Bad magic number received
#define NB_ESC				   4                 // Number of ESCs
#define MAX_ESC                4                 // Max number of ESCs

#define USB_UART_SPEED     115200            // Baudrate of the teeensy USB serial link

#define COMM_MAGIC         0x43305735        // Magic number: "teensy35" in leet speech

// Teensy->host communication data structure
// sizeof(ESCPID_comm)=64 to match USB 1.0 buffer size
typedef struct {
	uint32_t	magic;                        // Magic number
	uint16_t	deg[MAX_ESC];          // ESC temperature (°C)
	uint16_t	rpm[MAX_ESC];          // Motor rpm (10 rpm)
	uint16_t	amp[MAX_ESC];          // ESC current (0.01A)
} Teensycomm_struct_t;

// Host->teensy communication data structure
// sizeof(Host_comm)=64 to match USB 1.0 buffer size
typedef struct {
	uint32_t	magic;                        // Magic number
	int16_t		RPM[MAX_ESC];        // Velocity reference (10 rpm)
} RPicomm_struct_t;

// Prototypes
char *Host_name_from_serial(uint32_t);
int   Host_get_fd(uint32_t);
int   Host_init_port(uint32_t);
void  Host_release_port(uint32_t);
int   Host_comm_update(uint32_t, int16_t*, Teensycomm_struct_t**);

#endif

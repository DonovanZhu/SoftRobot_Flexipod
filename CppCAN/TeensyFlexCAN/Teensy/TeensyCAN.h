#ifndef __TeensyCAN_H
#define __TeensyCAN_H

// Defines
#define NB_ESC             4                 // Number of ESCs
#define MAX_ESC            4                 // Max number of ESCs

#define USB_UART_SPEED     115200            // Baudrate of the teeensy USB serial link

#define COMM_MAGIC         0x43305735        // Magic number: "teensy35" in leet speech
#define RESET_GAIN         0xffff            // PIDf gain value that triggers teensy reset

#define ERROR_MAGIC        -1                // Magic number error code

// Teensy->host communication data structure
// sizeof(Teensy_comm)=64 to match USB 1.0 buffer size
typedef struct {
  uint32_t      magic;                        // Magic number
  int       deg[MAX_ESC];          // Motors rotation angle(°C)
  int       rpm[MAX_ESC];          // Motors rpm
  int       amp[MAX_ESC];          // Motors current
} Teensycomm_struct_t;

// Host->teensy communication data structure
// sizeof(RPi_comm)=64 to match USB 1.0 buffer size
typedef struct {
  uint32_t      magic;                        // Magic number
  double        RPM[MAX_ESC];        // Velocity reference (10 rpm)
} RPicomm_struct_t;

#endif

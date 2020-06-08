#ifndef __TeensyCAN_H
#define __TeensyCAN_H

// Defines
#define NB_ESC             4                 // Number of ESCs

#define USB_UART_SPEED     115200            // Baudrate of the teeensy USB serial link

#define COMM_MAGIC         0x43305735        // Magic number: "teensy35" in leet speech
#define RESET_GAIN         0xffff            // PIDf gain value that triggers teensy reset

#define ERROR_MAGIC        -1                // Magic number error code

// Teensy->host communication data structure
typedef struct {
  uint32_t      magic;                // Magic number
  int           deg[NB_ESC];          // Motors rotation angle(°C)
  int           rpm[NB_ESC];          // Motors rpm
  int           torque[NB_ESC];       // Motors current
  double        acc[3];               // Acceleration in X Y Z direction, m/s^2
  double        gyr[3];               // Gyroscope in X Y Z direction, deg/s
} Teensycomm_struct_t;

// Host->teensy communication data structure
typedef struct {
  uint32_t      magic;                // Magic number
  double        RPM[NB_ESC];          // Desired Speed, rpm
} RPicomm_struct_t;

#endif

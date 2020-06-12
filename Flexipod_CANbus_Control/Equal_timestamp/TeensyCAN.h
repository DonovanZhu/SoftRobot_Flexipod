#ifndef __TeensyCAN_H
#define __TeensyCAN_H

// Defines
#define NB_ESC             4         // Number of ESCs
#define USB_UART_SPEED     4000000   // Baudrate of the teeensy USB serial link

#define NUM_TX_MAILBOXES 1
#define NUM_RX_MAILBOXES 4

//
// if speed command higher than 32768, the motor rotates clockwise
// if speed command lower than 32768, the motor rotates counter-clockwise
//
#define DirectionBoundary  32768.0   // 32768(dec) = 0x8000(hex)
#define maxBoundary        65535.0   // 0xffff, command upper limit
#define PID_H              10000.0   // PID controller output upper limit, corresponding to 10A
#define PID_L              -10000.0  // PID controller output lower limit,
#define DRIVE_RATIO        36.0      // Drive ratio of motor gear box
#define k_p                8.5      // Proportional parameter
#define k_i                0.015     // Integral parameter


// Teensy->host communication data structure
typedef struct {
  double angle[NB_ESC];   // Motors rotation angle, deg
  double rspeed[NB_ESC];  // Motors rotation speed, rpm
  double torque[NB_ESC];  // Motors torque, N*m
  double acc[3];          // Acceleration in X Y Z direction, m/s^2
  double gyr[3];          // Gyroscope in X Y Z direction, deg/s
} Teensycomm_struct_t;

// Host->teensy communication data structure
typedef struct {
  double        desired_speed[NB_ESC];          // Desired Speed, rpm
} RPicomm_struct_t;

#endif

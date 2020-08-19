#ifndef __TeensyCAN_H
#define __TeensyCAN_H

// Defines
#define NB_ESC             4                 // Number of ESCs
#define USB_UART_SPEED     1000000            // Baudrate of the teeensy USB serial link
//
// if speed command higher than 32768, the motor rotates clockwise
// if speed command lower than 32768, the motor rotates counter-clockwise
//
#define DirectionBoundary  32768.0   // 32768(dec) = 0x8000(hex)
#define maxBoundary        65535.0   // 0xffff, command upper limit
#define PID_H              12000.0   // PID controller output upper limit, corresponding to 10A
#define PID_L              -12000.0  // PID controller output lower limit,
#define PID_ANG_H          60.0
#define PID_ANG_L          0.0
#define DRIVE_RATIO        19.20320856  // Drive ratio of motor gear box
#define k_p                65.0       // Proportional parameter
#define k_i                0.15       // Integral parameter
#define k_p_ang            5.0



// Teensy->host communication data structure
typedef struct {
  double  angle[NB_ESC];      // Motors rotation angle
  double  rspeed[NB_ESC];     // Motors rpm
  double  torque[NB_ESC];     // Motors torque
  double  motor_command[NB_ESC];    // Motors speed command
  double  acc[3];             // Acceleration in X Y Z direction, m/s^2
  double  gyr[3];             // Gyroscope in X Y Z direction, deg/s
} Teensycomm_struct_t;

// Host->teensy communication data structure
typedef struct {
  double  speedcommand;            // Desired Speed, rpm
} RPicomm_struct_t;

#endif

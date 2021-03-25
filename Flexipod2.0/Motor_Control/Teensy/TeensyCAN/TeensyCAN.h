#ifndef __TeensyCAN_H
#define __TeensyCAN_H

// Defines
#define MOTOR_NUM          3                 // Number of ESCs
#define USB_UART_SPEED     1000000            // Baudrate of the teeensy USB serial link

#define DRIVE_RATIO        8.0  // Drive ratio of motor gear box

// -0.0192715 -0.2800474 10.0537329 0.0136186 0.0011329 -0.0076148
#define GRAVITY 9.802 // The gravity acceleration in New York City

// Calibration outcomes
#define GYRO_X_OFFSET        0.0131094
#define GYRO_Y_OFFSET        0.0012340
#define GYRO_Z_OFFSET       -0.0056364

#define ACCEL_X_OFFSET       0.0426984
#define ACCEL_Y_OFFSET      -0.3458546
#define ACCEL_Z_OFFSET       10.0628948

// Teensy->host communication data structure
typedef struct {
  double    angle[MOTOR_NUM];      // Motors rotation angle
  double    rspeed[MOTOR_NUM];     // Motors rpm
  double    torque[MOTOR_NUM];     // Motors torque
  double    comd[MOTOR_NUM];    // Motors speed command
  double    acc[3];             // Acceleration in X Y Z direction, m/s^2
  double    gyr[3];             // Gyroscope in X Y Z direction, deg/s
  double    mag[3];             // Magnetometer in X Y Z, uT
  double    eular[3];
  double    timestamps;
} Teensycomm_struct_t;

// Host->teensy communication data structure
typedef struct {
  double    comd[MOTOR_NUM];            // Desired Speed, rpm
} Jetson_comm_struct_t;
#endif

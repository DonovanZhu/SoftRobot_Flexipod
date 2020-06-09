#include <Wire.h>
#include <Arduino.h>
#include <FlexCAN_T4.h> // FlexCAN library for Teensy4.x
#include "MPU9250.h"    // IMU MPU9250 / MPU9255 for Teensy3.x or 4.x
#include "TeensyCAN.h"

//
// Choose CAN2 as the CAN port. In Teensy4.0, CAN2 pins are
// PIN0(CRX) and PIN1(CTX). Setting RX and TX queue size as
// 256 and 16.
//
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN;

//
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
// The MPU-9250 I2C address will be 0x68 if the AD0 pin is grounded or 0x69
// if the AD0 pin is pulled high. Check: https://github.com/tonton81/FlexCAN_T4
//
MPU9250 IMU(Wire,0x68);   
int status; // Use for start IMU

// Globals
bool    can_received[NB_ESC]     = {0,   0,   0,   0};     // For checking the data receiving status of each motor
double  shaft_angle_meas[NB_ESC] = {0.0, 0.0, 0.0, 0.0};   // Angle measured of each motor's shaft, unit degree, 0 - 360
double  rotor_angle_meas[NB_ESC] = {0.0, 0.0, 0.0, 0.0};   // Angle measured of each motor's rotor, unit 1, 0 - 0x1fff, corresponding to 0 - 360
double  torque_meas[NB_ESC]      = {0.0, 0.0, 0.0, 0.0};   // Torque measured from encoder of each motor
double  speed_meas[NB_ESC]       = {0.0, 0.0, 0.0, 0.0};   // The speed of motor shaft
double  desired_speed[NB_ESC]    = {0.0, 0.0, 0.0, 0.0};   // The desired speed of motors
double  speed_command[NB_ESC]    = {0.0, 0.0, 0.0, 0.0};   // Command output of PID controller
double  error[NB_ESC]            = {0.0, 0.0, 0.0, 0.0};   // Error between input and feedback
double  error_former[NB_ESC]     = {0.0, 0.0, 0.0, 0.0};   // Error in former time step

Teensycomm_struct_t Teensy_comm  = {{}, {}, {}, {}, {}};   // For holding data sent to RPi
RPicomm_struct_t    RPi_comm     = {{}};                   // For holding data received from RPi

// Set CAN bus message structure as CAN2.0
CAN_message_t msg_recv; // For receiving data on CAN bus
CAN_message_t msg_send; // For sending data on CAN bus

// Manage communication with RPi
void Teensy_comm_update(void) {
  static uint8_t  *ptin  = (uint8_t*)(&RPi_comm);
  static uint8_t  *ptout = (uint8_t*)(&Teensy_comm);
  static int      in_cnt = 0;

  // Read all incoming bytes available until incoming structure is complete
  while((Serial.available() > 0) && (in_cnt < (int)sizeof(RPi_comm)))
    ptin[in_cnt++] = Serial.read();

  // Check if a complete incoming packet is available
  if (in_cnt == (int)sizeof(RPi_comm)) {

    // Clear incoming bytes counter
    in_cnt = 0;

    for (int i = 0; i < NB_ESC; i++)
      desired_speed[i] = RPi_comm.desired_speed[i];

    // Read data from motors
    Read_CAN();

    // Do PID contol, then sending command to motor through CAN bus
    Control_loop();

    // Save angle, speed and torque into struct Teensy_comm
    for (int i = 0; i < NB_ESC; i++) {
      Teensy_comm.angle[i]  = shaft_angle_meas[i];
      Teensy_comm.rspeed[i] = speed_meas[i];
      Teensy_comm.torque[i] = torque_meas[i];
    }

    // Read data from IMU(MPU 9255)
    IMU.readSensor();

    // Save acceleration (m/s^2) of IMU into struct Teensy_comm
    Teensy_comm.acc[0] = IMU.getAccelX_mss();
    Teensy_comm.acc[1] = IMU.getAccelY_mss();
    Teensy_comm.acc[2] = IMU.getAccelZ_mss();

    // Save gyroscope (deg/s) of IMU into struct Teensy_comm
    Teensy_comm.gyr[0] = IMU.getGyroX_rads();
    Teensy_comm.gyr[1] = IMU.getGyroY_rads();
    Teensy_comm.gyr[2] = IMU.getGyroZ_rads();

    // Send data structure Teensy_comm to RPi
    Serial.write(ptout, sizeof(Teensy_comm));

    // Force immediate transmission
    Serial.send_now();
  }
}

//
// This function is for doing PID control. In practical using, PI control 
// is already enough fast and stable, thus in this function, differential
// part is omitted.
//
void Control_loop() {

  // Set the CAN message ID as 0x200
  msg_send.id = 0x200;

  // In this function, a incremental PI controller is used
  for (int i = 0; i < NB_ESC; ++i) {
    error[i] = desired_speed[i] - speed_meas[i];

    // For motors' ESCs, this command output is current value.
    speed_command[i] += k_p * (error[i] - error_former[i]) + k_i * error[i];
    error_former[i] = error[i];

    // If the output is positive, then the output shall be smaller than PID_H limit.
    if(speed_command[i] >= 0)
    {
      if(speed_command[i] > PID_H)
        speed_command[i] = PID_H;
      int v = int(speed_command[i]);

      // msg_send.buf is a list of 8 bytes.
      // buf[0] and buf[1] are the high byte and low byte of command sent to motor1
      // buf[2] and buf[3] are the high byte and low byte of command sent to motor2
      // buf[4] and buf[5] are the high byte and low byte of command sent to motor3
      // buf[6] and buf[7] are the high byte and low byte of command sent to motor4
      msg_send.buf[2 * i + 1] = v & 0x00ff;
      msg_send.buf[2 * i] = (v >> 8) & 0x00ff;
      
    }
    // If the output is negative, then the output shall be larger than PID_L limit.
    else
    {
      if(speed_command[i] < PID_L)
        speed_command[i] = PID_L;
        
      // 0xffff is the max Boundary of command. If motor moves in clockwise,
      // the command increases from 0, which is the stay point. When moving
      // in counter clockwise, the command decreases from 0xffff, which is
      // the other stay point.
      int v = (int)maxBoundary + int(speed_command[i]);
      msg_send.buf[2 * i + 1] = v & 0x00ff;
      msg_send.buf[2 * i] = (v >> 8) & 0x00ff;
    }
  }
  CAN.write(msg_send); // Write the message on CAN bus
}

//
// Read message from CAN bus
//
void Read_CAN() {
  int rpm, angle, torque;
  int i = 0;

  // To control 4 motors synchronously, the data from each motor is supposed
  // to be received.
  while (true) {

    // Reading one message
    if (CAN.read(msg_recv)) {

      // The ID of 4 motors are 0x201, 0x202, 0x203 and 0x204.
      // To make sure all the data from each motor is received,
      // a bool list "can_received" is used. When one corresponding
      // data is received, the bool in the list of that motor is turn true.
      
      if (!can_received[msg_recv.id - 0x201]) {

        // The received message data "msg_recv.buf" is a list of 8 bytes.
        // buf[0] and buf[1] are the high byte and low byte of rotor angle
        // buf[2] and buf[3] are the high byte and low byte of rotor speed
        // buf[4] and buf[5] are the high byte and low byte of torque
        // buf[6] and buf[7] are NULL
        rpm = 0;
        rpm |= (int16_t)(unsigned char)msg_recv.buf[2] << 8;
        rpm |= (int16_t)(unsigned char)msg_recv.buf[3];

        // If speed is between 0 (0 rpm point) to 0x7fff, then the motor rotates in clockwise.
        // If speed is between 0xffff (0 rpm point) to 0x8000, then the motor rotates in counter-clockwise.
        // For doing PI control, the rpm data is transformed into the speed  of motor shaft.
        if (rpm < DirectionBoundary)
          speed_meas[msg_recv.id - 0x201] = (double)rpm / DRIVE_RATIO;
        else
          speed_meas[msg_recv.id - 0x201] = - (double)(maxBoundary - rpm) / DRIVE_RATIO;
        
        angle = 0;
        angle |= (int16_t)(unsigned char)msg_recv.buf[0] << 8;
        angle |= (int16_t)(unsigned char)msg_recv.buf[1];

        angle_calculate((double)angle / 8191.0 * 360.0, msg_recv.id - 0x201);
        
        torque = 0;
        torque |= (int16_t)(unsigned char)msg_recv.buf[4] << 8;
        torque |= (int16_t)(unsigned char)msg_recv.buf[5];
        torque_meas[msg_recv.id - 0x201] = (double)torque;
        
        can_received[msg_recv.id - 0x201] = true;
        i++;
      }
    }

    // After all the data of all motors is revceived, turn bool list in all false
    if (i == NB_ESC)
    {
      for (int k = 0; k < NB_ESC; ++k)
        can_received[k] = false;
      break;
    }
  }
}

// Read the initial data from motors
void CAN_init() {
  int i = 0;
  int angle;
  while (true) {
    if (CAN.read(msg_recv)) {
      if (!can_received[msg_recv.id - 0x201]) {
        
        angle = 0;
        angle |= (int16_t)(unsigned char)msg_recv.buf[0] << 8;
        angle |= (int16_t)(unsigned char)msg_recv.buf[1];
        
        rotor_angle_meas[msg_recv.id - 0x201] = (double)angle / 8191.0 * 360.0;
        
        can_received[msg_recv.id - 0x201] = true;
        i++;
      }
    }
    if (i == NB_ESC) {
      for (int k = 0; k < NB_ESC; ++k) {
        can_received[k] = false;
      }
      break;
    }
  }
}


//
// The angle data sent from motor is the rotation angle of the rotor.
// This function is for transfering the rotor angle to shaft angle. 
// Inputs are rotor angle, index of motor and speed of motor. 
// Results are saved in list "angle_sum".
// 
void angle_calculate(double angle, int i) {

  // Difference between current angle and the former one
  double delta_angle = angle - rotor_angle_meas[i];
    
  // When motor rotates in clockwise, if current angle
  // acrosses 0 degree line, which means difference is
  // minus, the difference should be added 360 degree to
  // compensate. Vice versa, when motor moving in counter
  // clockwise, the difference needs to be subtracted 360.
  if (speed_meas[i] >= 0.0 && delta_angle < 0.0) 
    delta_angle += 360.0;
    
  else if (speed_meas[i] < 0.0 && delta_angle > 0.0) 
    delta_angle -= 360.0;
 
  // Angle of rotor to angle of shaft
  shaft_angle_meas[i] += delta_angle / DRIVE_RATIO;

  // Set the shaft angle between 0 - 360
  if (shaft_angle_meas[i] >= 360.0)
    shaft_angle_meas[i] -= 360.0;
  else if (shaft_angle_meas[i] < 0.0)
    shaft_angle_meas[i] += 360.0;

  rotor_angle_meas[i] = angle;
}


void setup() {
  // Switch on IMU, for checking the IMU setting detail:
  // https://github.com/bolderflight/MPU9250
  IMU.begin();
  // Setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // Setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // Setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // Setting SRD to 4 for a 200 Hz update rate
  IMU.setSrd(4);

  // Switch on CAN bus
  CAN.begin();
  CAN.setBaudRate(1000000);
  CAN_init();

  // Open Serial port in speed 115200 Baudrate
  Serial.begin(USB_UART_SPEED);
}

void loop() {
  
  Teensy_comm_update();

}

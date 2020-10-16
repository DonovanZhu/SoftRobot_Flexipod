#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_INA260.h>
#include <FlexCAN_T4.h>
#include "MadgwickAHRS.h"
#include "TeensyCAN.h"


//
// Choose CAN2 as the CAN port. In Teensy4.0, CAN2 pins are
// PIN0(CRX) and PIN1(CTX). Setting RX and TX queue size as
// 256 and 16.
//
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN;

Adafruit_INA260 ina260 = Adafruit_INA260();

Adafruit_LSM6DS sox;
Adafruit_LIS3MDL lis;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mage;


const double magn_ellipsoid_center[3] = {0.262689, -6.89484, 4.0776};
const double magn_ellipsoid_transform[3][3] = {{0.899993, 0.0341615, -0.000181209}, {0.0341615, 0.988324, -0.000514259}, { -0.000181209, -0.000514259, 0.952566}};

// Globals
bool    can_received[NB_ESC]       = {  0,   0,   0,   0};   // For checking the data receiving status of each motor
double  shaft_angle_meas[NB_ESC]   = {0.0, 0.0, 0.0, 0.0};   // Angle measured of each motor's shaft, unit degree, 0 - 360
double  rotor_angle_meas[NB_ESC]   = {0.0, 0.0, 0.0, 0.0};   // Angle measured of each motor's rotor, unit 1, 0 - 0x1fff, corresponding to 0 - 360
double  rotor_init_angle[NB_ESC]   = {0.0, 0.0, 0.0, 0.0};
int     rotation_number[NB_ESC]    = {  0,   0,   0,   0};
double  torque_meas[NB_ESC]        = {0.0, 0.0, 0.0, 0.0};   // Torque measured from encoder of each motor
double  speed_meas[NB_ESC]         = {0.0, 0.0, 0.0, 0.0};   // The speed of motor shaft
double  desired_speed[NB_ESC]      = {0.0, 0.0, 0.0, 0.0};   // The desired speed of motors
double  speed_command[NB_ESC]      = {0.0, 0.0, 0.0, 0.0};   // Command output of PID controller
double  speed_error[NB_ESC]        = {0.0, 0.0, 0.0, 0.0};   // Error between input and feedback
double  speed_error_former[NB_ESC] = {0.0, 0.0, 0.0, 0.0};   // Error in former time step
double  ang_error[NB_ESC]          = {0.0, 0.0, 0.0, 0.0};
double  ang_command[NB_ESC]        = {0.0, 0.0, 0.0, 0.0};
double  desire_angle[NB_ESC]       = {0.0, 0.0, 0.0, 0.0};
double  command_angle[NB_ESC]      = {0.0, 0.0, 0.0, 0.0};
double  freq                       = 0.0;
double  former_base                = 0.6;
double  acc[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
double  mag[3];
double  mag_tmp[3];
double  gyr[3];

double time_now;
double time_former;
double dt = 0.0;
double deltat;

double contact_time = 0.6

double timestamp_1;
double timestamp_2;

int    former_gait = 5; // reset legs
bool   gait_transfer = false;
double next_gait_angle[4] = {0.0, 0.0, 0.0, 0.0};
double former_roll_angle = 0.0;

Teensycomm_struct_t Teensy_comm = {{}, {}, {}, {}, {}, {}, {}, {}, {}};   // For holding data sent to RPi
RPicomm_struct_t    RPi_comm    = {{}, {}, {}};                   // For holding data received from RPi

static uint8_t  *ptin  = (uint8_t*)(&RPi_comm);
static uint8_t  *ptout = (uint8_t*)(&Teensy_comm);
static int      in_cnt = 0;

// Set CAN bus message structure as CAN2.0
CAN_message_t msg_recv; // For receiving data on CAN bus
CAN_message_t msg_send; // For sending data on CAN bus

Quaternion qua;
EulerAngles eul;


void read_sensors() {
  sox.getEvent(&accel, &gyro);
  lis.getEvent(&mage);
  acc[0] = accel.acceleration.x;
  acc[1] = accel.acceleration.y;
  acc[2] = accel.acceleration.z;

  mag[0] = mage.magnetic.x;
  mag[1] = mage.magnetic.y;
  mag[2] = mage.magnetic.z;

  gyr[0] = gyro.gyro.x;
  gyr[1] = gyro.gyro.y;
  gyr[2] = gyro.gyro.z;
}

void compensate_sensor_errors() {
  // Compensate accelerometer error
  acc[0] -= ACCEL_X_OFFSET;
  acc[1] -= ACCEL_Y_OFFSET;
  acc[2] -= ACCEL_Z_OFFSET - GRAVITY;

  // Compensate magnetometer error
  for (int i = 0; i < 3; i++)
    mag_tmp[i] = mag[i] - magn_ellipsoid_center[i];
  Matrix_Vector_Multiply(magn_ellipsoid_transform, mag_tmp, mag);

  // Compensate gyroscope error
  gyr[0] -= GYRO_X_OFFSET;
  gyr[1] -= GYRO_Y_OFFSET;
  gyr[2] -= GYRO_Z_OFFSET;
}

void Angle_Sync_Loop() {
  for (int i = 0; i < NB_ESC; ++i) {
    double delta_angle = command_angle[i] - shaft_angle_meas[i];
    if (delta_angle >= 0 && delta_angle < 180.0)
      ang_error[i] = delta_angle;
    else if (delta_angle < - 180.0)
      ang_error[i] = 360.0 + delta_angle;
    else if (delta_angle >= -180 && delta_angle < 0.0)
      ang_error[i] = delta_angle;
    else if (delta_angle >= 180.0)
      ang_error[i] = delta_angle - 360.0;

    ang_command[i] = k_p_ang * ang_error[i];

    constrain(ang_command[i], -500.0, 500.0);
    desired_speed[i] = ang_command[i];
  }

}

//
// This function is for doing PID control. In practical using, PI control
// is already enough fast and stable, thus in this function, differential
// part is omitted.
//
void Speed_Control_Loop() {

  // Set the CAN message ID as 0x200
  msg_send.id = 0x200;

  // In this function, a incremental PI controller is used
  for (int i = 0; i < NB_ESC; ++i) {
    speed_error[i] = desired_speed[i] - speed_meas[i];

    // For motors' ESCs, this command output is current value.
    speed_command[i] += k_p * (speed_error[i] - speed_error_former[i]) + k_i * speed_error[i];
    speed_error_former[i] = speed_error[i];

    // If the output is positive, then the output shall be smaller than PID_H limit.
    // If the output is negative, then the output shall be larger than PID_L limit.
    constrain(speed_command[i], PID_L, PID_H);
    int v = int(speed_command[i]);
    //v = 0;
    // msg_send.buf is a list of 8 bytes.
    // buf[0] and buf[1] are the high byte and low byte of command sent to motor1
    // buf[2] and buf[3] are the high byte and low byte of command sent to motor2
    // buf[4] and buf[5] are the high byte and low byte of command sent to motor3
    // buf[6] and buf[7] are the high byte and low byte of command sent to motor4
    msg_send.buf[2 * i + 1] = v & 0x00ff;
    msg_send.buf[2 * i] = (v >> 8) & 0x00ff;
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
        rotor_init_angle[msg_recv.id - 0x201] = rotor_angle_meas[msg_recv.id - 0x201];
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
  double angle_now;
  double angle_former;
  if (angle - rotor_init_angle[i] >= 0.0)
    angle_now = angle - rotor_init_angle[i];
  else
    angle_now = 360.0 + (angle - rotor_init_angle[i]);

  if (rotor_angle_meas[i] - rotor_init_angle[i] >= 0.0)
    angle_former = rotor_angle_meas[i] - rotor_init_angle[i];
  else
    angle_former = 360.0 + (rotor_angle_meas[i] - rotor_init_angle[i]);

  if (fabs(speed_meas[i]) > 130.0) {
    if ((angle_now < angle_former && speed_meas[i] > 0.0) || (angle_now < angle_former && ((360.0 - angle_former) + angle_now < 180.0)))
      rotation_number[i]++;
    else if ((angle_now > angle_former && speed_meas[i] < 0.0) || (angle_now > angle_former && ((360.0 - angle_now) + angle_former < 180.0)))
      rotation_number[i]--;
  }
  else {
    if (angle_now < angle_former && ((360.0 - angle_former) + angle_now < 180.0))
      rotation_number[i]++;
    else if (angle_now > angle_former && ((360.0 - angle_now) + angle_former < 180.0))
      rotation_number[i]--;
  }

  shaft_angle_meas[i] = ((double)rotation_number[i] * 360.0 + angle_now) / DRIVE_RATIO ;

  // Set the shaft angle between 0 - 360
  while (shaft_angle_meas[i] > 360.0) {
    shaft_angle_meas[i] -= 360.0;
  }
  while (shaft_angle_meas[i] < 0.0) {
    shaft_angle_meas[i] += 360.0;
  }

  rotor_angle_meas[i] = angle;
}

void trot_gait() {
  double k_1_trot = (360.0 - CONTACT_ANGLE) / (1.0 - contact_time);
  double k_2_trot = CONTACT_ANGLE / contact_time;
  double b_1_trot = 360.0 - k_1_trot;
  double b_2_trot = (90.0 + 0.5 * CONTACT_ANGLE) - k_2_trot * ((90.0 + 0.5 * CONTACT_ANGLE) - b_1_trot) / k_1_trot;
  
  double T = fabs(1.0 / RPi_comm.speedcommand);

  if (fabs(RPi_comm.speedcommand - freq) > 0.0 || fabs(RPi_comm.s_base - former_base) > 0.0) {
    if (desire_angle[0] >= 0 && desire_angle[0] < 90.0 - CONTACT_ANGLE * 0.5)
      dt = desire_angle[0] * T / k_1_trot;
    else if (desire_angle[0] >= 90.0 - CONTACT_ANGLE * 0.5 && desire_angle[0] < 90.0 + CONTACT_ANGLE * 0.5)
      dt = (desire_angle[0] - b_2_trot) * T / k_2_trot;
    else
      dt = (desire_angle[0] - b_1_trot) * T / k_1_trot;
  }
  if (RPi_comm.speedcommand > 0)
    dt += deltat;
  else
    dt -= deltat;

  freq = RPi_comm.speedcommand;
  former_base = RPi_comm.s_base;
  
  while (dt > T) {
    dt -= T;
  }
  while (dt < 0) {
    dt += T;
  }

  double dt_2 = dt - 0.5 * T;
  while (dt_2 > T) {
    dt_2 -= T;
  }
  while (dt_2 < 0) {
    dt_2 += T;
  }

  timestamp_1 = (90.0 - CONTACT_ANGLE * 0.5) / k_1_trot;
  timestamp_2 = (90.0 + CONTACT_ANGLE * 0.5 - b_1_trot) / k_1_trot ;

  if (dt >= 0 && dt < timestamp_1 * T) {
    desire_angle[0] =  k_1_trot / T * dt;
    desire_angle[2] =  360.0 - k_1_trot / T * dt;
  }
  else if (dt >= timestamp_1 * T && dt < timestamp_2 * T) {
    desire_angle[0] =  k_2_trot / T * dt + b_2_trot;
    desire_angle[2] =  360.0 - (k_2_trot / T * dt + b_2_trot);
  }
  else {
    desire_angle[0] = k_1_trot / T * dt + b_1_trot;
    desire_angle[2] = 360.0 - (k_1_trot / T * dt + b_1_trot);
  }

  if (dt_2 >= 0 && dt_2 < timestamp_1 * T) {
    desire_angle[1] =  k_1_trot / T * dt_2;
    desire_angle[3] =  360.0 - k_1_trot / T * dt_2;
  }
  else if (dt_2 >= timestamp_1 * T && dt_2 < timestamp_2 * T) {
    desire_angle[1] =  k_2_trot / T * dt_2 + b_2_trot;
    desire_angle[3] =  360.0 - (k_2_trot / T * dt_2 + b_2_trot);
  }
  else {
    desire_angle[1] = k_1_trot / T * dt_2 + b_1_trot;
    desire_angle[3] = 360.0 - (k_1_trot / T * dt_2 + b_1_trot);
  }
}

void crawl_gait() {
  double k_1_crawl = (360.0 - CONTACT_ANGLE) / (1.0 - contact_time);
  double k_2_crawl = CONTACT_ANGLE / contact_time;
  double b_1_crawl = 360.0 - k_1_crawl;
  double b_2_crawl = (90.0 + 0.5 * CONTACT_ANGLE) - k_2_crawl * ((90.0 + 0.5 * CONTACT_ANGLE) - b_1_crawl) / k_1_crawl;
  
  double T = fabs(1.0 / RPi_comm.speedcommand );

  if (fabs(RPi_comm.speedcommand - freq) > 0.0 || fabs(RPi_comm.s_base - former_base) > 0.0) {
    if (desire_angle[0] >= 0 && desire_angle[0] < 90.0 - CONTACT_ANGLE * 0.5)
      dt = desire_angle[0] * T / k_1_crawl;
    else if (desire_angle[0] >= 90.0 - CONTACT_ANGLE * 0.5 && desire_angle[0] < 90.0 + CONTACT_ANGLE * 0.5)
      dt = (desire_angle[0] - b_2_crawl) * T / k_2_crawl;
    else
      dt = (desire_angle[0] - b_1_crawl) * T / k_1_crawl;
  }

  if (RPi_comm.speedcommand > 0.0)
    dt += deltat;
  else
    dt -= deltat;

  freq = RPi_comm.speedcommand;
  former_base = RPi_comm.s_base;
  
  while (dt > T) {
    dt -= T;
  }
  while (dt < 0) {
    dt += T;
  }

  double dt_2 = dt - 0.75 * T;
  while (dt_2 > T) {
    dt_2 -= T;
  }
  while (dt_2 < 0) {
    dt_2 += T;
  }

  double dt_3 = dt - 0.25 * T;
  while (dt_3 > T) {
    dt_3 -= T;
  }
  while (dt_3 < 0) {
    dt_3 += T;
  }

  double dt_4 = dt - 0.5 * T;
  while (dt_4 > T) {
    dt_4 -= T;
  }
  while (dt_4 < 0) {
    dt_4 += T;
  }
  timestamp_1 = (90.0 - CONTACT_ANGLE * 0.5) / k_1_crawl;
  timestamp_2 = (90.0 + CONTACT_ANGLE * 0.5 - b_1_crawl) / k_1_crawl;

  if (dt >= 0 && dt < timestamp_1 * T)
    desire_angle[0] =  k_1_crawl / T * dt;
  else if (dt >= timestamp_1 * T && dt < timestamp_2 * T)
    desire_angle[0] =  k_2_crawl / T * dt + b_2_crawl;
  else
    desire_angle[0] = k_1_crawl / T * dt + b_1_crawl;

  if (dt_2 >= 0 && dt_2 < timestamp_1 * T)
    desire_angle[1] =  k_1_crawl / T * dt_2;
  else if (dt_2 >= timestamp_1 * T && dt_2 < timestamp_2 * T)
    desire_angle[1] =  k_2_crawl / T * dt_2 + b_2_crawl;
  else
    desire_angle[1] = k_1_crawl / T * dt_2 + b_1_crawl;

  if (dt_3 >= 0 && dt_3 < timestamp_1 * T)
    desire_angle[2] =  360.0 - k_1_crawl / T * dt_3;
  else if (dt_3 >= timestamp_1 * T && dt_3 < timestamp_2 * T)
    desire_angle[2] =  360.0 - (k_2_crawl / T * dt_3 + b_2_crawl);
  else
    desire_angle[2] = 360.0 - (k_1_crawl / T * dt_3 + b_1_crawl);


  if (dt_4 >= 0 && dt_4 < timestamp_1 * T)
    desire_angle[3] =  360.0 - k_1_crawl / T * dt_4;
  else if (dt_4 >= timestamp_1 * T && dt_4 < timestamp_2 * T)
    desire_angle[3] =  360.0 - (k_2_crawl / T * dt_4 + b_2_crawl);
  else
    desire_angle[3] = 360.0 - (k_1_crawl / T * dt_4 + b_1_crawl);
}

void bound_gait() {
  
  double k_1_bound = (360.0 - CONTACT_ANGLE) / (1.0 - contact_time);
  double k_2_bound = CONTACT_ANGLE / contact_time;
  double b_1_bound = 360.0 - k_1_bound;
  double b_2_bound = (90.0 + 0.5 * CONTACT_ANGLE) - k_2_bound * ((90.0 + 0.5 * CONTACT_ANGLE) - b_1_bound) / k_1_bound;
  double T = fabs(1.0 / RPi_comm.speedcommand);

  if (fabs(RPi_comm.speedcommand - freq) > 0.0 || fabs(RPi_comm.s_base - former_base) > 0.0) {
    if (desire_angle[0] >= 0 && desire_angle[0] < 90.0 - CONTACT_ANGLE * 0.5)
      dt = desire_angle[0] * T / k_1_bound;
    else if (desire_angle[0] >= 90.0 - CONTACT_ANGLE * 0.5 && desire_angle[0] < 90.0 + CONTACT_ANGLE * 0.5)
      dt = (desire_angle[0] - b_2_bound) * T / k_2_bound;
    else
      dt = (desire_angle[0] - b_1_bound) * T / k_1_bound;
  }
  if (RPi_comm.speedcommand > 0)
    dt += deltat;
  else
    dt -= deltat;

  freq = RPi_comm.speedcommand;
  former_base = RPi_comm.s_base;
  
  while (dt > T) {
    dt -= T;
  }
  while (dt < 0) {
    dt += T;
  }

  double dt_2 = dt - 0.5 * T;
  while (dt_2 > T) {
    dt_2 -= T;
  }
  while (dt_2 < 0) {
    dt_2 += T;
  }

  timestamp_1 = (90.0 - CONTACT_ANGLE * 0.5) / k_1_bound;
  timestamp_2 = (90.0 + CONTACT_ANGLE * 0.5 - b_1_bound) / k_1_bound;

  if (dt >= 0 && dt < timestamp_1 * T) {
    desire_angle[0] =  k_1_bound / T * dt;
    desire_angle[3] =  360.0 - k_1_bound / T * dt;
  }
  else if (dt >= timestamp_1 * T && dt < timestamp_2 * T) {
    desire_angle[0] =  k_2_bound / T * dt + b_2_bound;
    desire_angle[3] =  360.0 - (k_2_bound / T * dt + b_2_bound);
  }
  else {
    desire_angle[0] = k_1_bound / T * dt + b_1_bound;
    desire_angle[3] = 360.0 - (k_1_bound / T * dt + b_1_bound);
  }

  if (dt_2 >= 0 && dt_2 < timestamp_1 * T) {
    desire_angle[1] =  k_1_bound / T * dt_2;
    desire_angle[2] =  360.0 - k_1_bound / T * dt_2;
  }
  else if (dt_2 >= timestamp_1 * T && dt_2 < timestamp_2 * T) {
    desire_angle[1] =  k_2_bound / T * dt_2 + b_2_bound;
    desire_angle[2] =  360.0 - (k_2_bound / T * dt_2 + b_2_bound);
  }
  else {
    desire_angle[1] = k_1_bound / T * dt_2 + b_1_bound;
    desire_angle[2] = 360.0 - (k_1_bound / T * dt_2 + b_1_bound);
  }
}

void flip_gait() {
  double front_leg_step = 0.4;
  double back_leg_step = 1.0;

  if (fabs(120.0 - desire_angle[0]) < 1.0)
    desire_angle[0] = 120.0;
  else
    desire_angle[0] += front_leg_step;

  if (fabs(240.0 - desire_angle[3]) < 1.0)
    desire_angle[3] = 240.0;
  else
    desire_angle[3] -= front_leg_step;

  if (fabs(300.0 - desire_angle[1]) < 1.0)
    desire_angle[1] = 300.0;
  else
    desire_angle[1] += back_leg_step;
  if (fabs(60.0 - desire_angle[2]) < 1.0)
    desire_angle[2] = 60.0;
  else
    desire_angle[2] -= back_leg_step;

}

void turn_left() {
  double turn_step = 0.18;
  for (int i = 0; i < NB_ESC; ++i) {
    desire_angle[i] = command_angle[i];
    desire_angle[i] -= turn_step;
  }
}

void turn_right() {
  double turn_step = 0.18;
  for (int i = 0; i < NB_ESC; ++i) {
    desire_angle[i] = command_angle[i];
    desire_angle[i] += turn_step;
  }
}

void legs_reset() {
  double reset_step = 0.18;
  for (int i = 0; i < NB_ESC; ++i) {
    desire_angle[i] = command_angle[i];
    if (desire_angle[i] < 1.0 || (360.0 - desire_angle[i]) < 1.0) {
      desire_angle[i] = 0.0;
    }
    else {
      if (desire_angle[i] < 180.0) {
        desire_angle[i] -= reset_step;
      }
      else {
        desire_angle[i] += reset_step;
      }
    }
  }
}

void legs_sync() {
  double reset_step = 0.18;
  for (int i = 0; i < 2; ++i) {
    desire_angle[i] = command_angle[i];
    if (desire_angle[i] < 1.0 || (360.0 - desire_angle[i]) < 1.0) {
      desire_angle[i] = 0.0;
    }
    else {
      if (desire_angle[i] < 180.0) {
        desire_angle[i] -= reset_step;
      }
      else {
        desire_angle[i] += reset_step;
      }
    }
  }
  for (int i = 2; i < NB_ESC; ++i) {
    desire_angle[i] = command_angle[i];
    if (fabs(180.0 - desire_angle[i]) < 1.0) {
      desire_angle[i] = 180.0;
    }
    else {
      if (desire_angle[i] < 180.0) {
        desire_angle[i] += reset_step;
      }
      else {
        desire_angle[i] -= reset_step;
      }
    }
  }
}

void setup() {
  time_former = micros();

  sox.begin_I2C();
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  lis.begin_I2C();          // hardware I2C mode, can pass in address & alt Wire
  lis.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis.setIntThreshold(500);
  lis.configInterrupt(false, false, true, true, false, true); // enabled!

  ina260.begin();

  // Switch on CAN bus
  CAN.begin();
  CAN.setBaudRate(1000000);
  CAN.setClock(CLK_60MHz);
  CAN_init();

  // Open Serial port in speed 115200 Baudrate
  Serial.begin(USB_UART_SPEED);
  pinMode(13, OUTPUT);
}

void loop() {

  in_cnt = 0;

  read_sensors();

  compensate_sensor_errors();
  time_now = (double)micros();
  deltat = (time_now - time_former) / 1000000.0;
  time_former = time_now;

  MadgwickQuaternionUpdate(acc[0], acc[1], acc[2],
                           gyr[0], gyr[1], gyr[2],
                           mag[0], mag[1], mag[2], deltat);
  qua.w = q[0];
  qua.x = q[1];
  qua.y = q[2];
  qua.z = q[3];
  eul = ToEulerAngles(qua);
  eul.yaw_e += 0.8;
  if (eul.yaw_e > PI) {
    eul.yaw_e -= 2 * PI;
  }

  // Read all incoming bytes available until incoming structure is complete
  while ((Serial.available() > 0) && (in_cnt < (int)sizeof(RPi_comm)))
    ptin[in_cnt++] = Serial.read();

  // Check if a complete incoming packet is available
  if (in_cnt == (int)sizeof(RPi_comm)) {
    
    contact_time = RPi_comm.s_base;
    
    // Clear incoming bytes counter
    in_cnt = 0;

    // Save angle, speed and torque into struct Teensy_comm
    for (int i = 0; i < NB_ESC; i++) {
      Teensy_comm.angle[i]  = shaft_angle_meas[i];
      Teensy_comm.rspeed[i] = speed_meas[i];
      Teensy_comm.torque[i] = torque_meas[i];
      Teensy_comm.motor_command[i] = desired_speed[i];
    }
    
    // Save acceleration (m/s^2) of IMU into struct Teensy_comm
    Teensy_comm.acc[0] = accel.acceleration.x;
    Teensy_comm.acc[1] = accel.acceleration.y;
    Teensy_comm.acc[2] = accel.acceleration.z;

    // Save gyroscope (deg/s) of IMU into struct Teensy_comm
    Teensy_comm.gyr[0] = gyro.gyro.x;
    Teensy_comm.gyr[1] = gyro.gyro.y;
    Teensy_comm.gyr[2] = gyro.gyro.z;

    Teensy_comm.mag[0] = mage.magnetic.x;
    Teensy_comm.mag[1] = mage.magnetic.y;
    Teensy_comm.mag[2] = mage.magnetic.z;

    Teensy_comm.eular[0] = eul.roll_e;
    Teensy_comm.eular[1] = eul.pitch_e;
    Teensy_comm.eular[2] = eul.yaw_e;

    Teensy_comm.timestamps = time_now / 1000000.0;
    // Send data structure Teensy_comm to RPi
    Serial.write(ptout, sizeof(Teensy_comm));

    // Force immediate transmission
    Serial.send_now();
  }

  if (RPi_comm.speedcommand != 0.0) {
    if (RPi_comm.gait != former_gait && RPi_comm.gait != 4 && RPi_comm.gait != 5 && RPi_comm.gait != 8) {
      
      gait_transfer = true;
      if (RPi_comm.gait == 1)
        trot_gait();
      else if (RPi_comm.gait == 2)
        crawl_gait();
      else if (RPi_comm.gait == 3)
        bound_gait();
      else if (RPi_comm.gait == 6)
        turn_left();
      else if (RPi_comm.gait == 7)
        turn_right();

      if (fabs(eul.roll_e) > PI / 2.0 && RPi_comm.gait != 6 && RPi_comm.gait != 7) {
        for (int i = 0; i < NB_ESC; ++i) {
          next_gait_angle[i] = 360.0 - desire_angle[i];
        }
      }
      else {
        for (int i = 0; i < NB_ESC; ++i) {
          next_gait_angle[i] = desire_angle[i];
        }
      }

      if (former_gait == 1)
        trot_gait();
      else if (former_gait == 2)
        crawl_gait();
      else if (former_gait == 3)
        bound_gait();
      else if (former_gait == 4)
        flip_gait();
      else if (former_gait == 5)
        legs_reset();
      else if (former_gait == 6)
        turn_left();
      else if (former_gait == 7)
        turn_right();
      else if (former_gait == 8)
        legs_sync();
      if (fabs(eul.roll_e) > PI / 2.0 && RPi_comm.gait != 5 && RPi_comm.gait != 6 && RPi_comm.gait != 7 && RPi_comm.gait != 8) {
        for (int i = 0; i < NB_ESC; ++i) {
          desire_angle[i] = 360.0 - desire_angle[i];
        }
      }   
    }

    if (!gait_transfer) {
      if (RPi_comm.gait == 1)
        trot_gait();
      else if (RPi_comm.gait == 2)
        crawl_gait();
      else if (RPi_comm.gait == 3)
        bound_gait();
      else if (RPi_comm.gait == 4)
        flip_gait();
      else if (RPi_comm.gait == 5)
        legs_reset();
      else if (RPi_comm.gait == 6)
        turn_left();
      else if (RPi_comm.gait == 7)
        turn_right();
      else if (RPi_comm.gait == 8)
        legs_sync();
    }

    if (gait_transfer) {
      double gait_transfer_step = 0.18;
      for (int i = 0; i < NB_ESC; ++i) {
             
        
        if (fabs(desire_angle[i] - next_gait_angle[i]) < 1.0) {
          desire_angle[i] = next_gait_angle[i];
        }
        else {
          if (desire_angle[i] < next_gait_angle[i]) {
            desire_angle[i] += gait_transfer_step;
          }
          else {
            desire_angle[i] -= gait_transfer_step;
          }
        }
      }
    }

    former_gait = RPi_comm.gait;
    former_roll_angle = eul.roll_e;
    
    if (fabs(eul.roll_e) > PI / 2.0 && RPi_comm.gait != 4 && RPi_comm.gait != 5 && RPi_comm.gait != 6 && RPi_comm.gait != 7 && RPi_comm.gait != 8) {
      for (int i = 0; i < NB_ESC; ++i) {
        command_angle[i] = 360.0 - desire_angle[i];
      }
    }
    else {
      for (int i = 0; i < NB_ESC; ++i) {
        command_angle[i] = desire_angle[i];
      }
    }

    if (gait_transfer) {
      for (int i = 0; i < NB_ESC; ++i) 
        command_angle[i] = desire_angle[i];
      double transfer_angle_error = 0.0;
      for (int i = 0; i < NB_ESC; ++i)
        transfer_angle_error += fabs(desire_angle[i] - next_gait_angle[i]);
      if (transfer_angle_error / 4.0 < 1.0)
        gait_transfer = false;
    }

  }

  for (int i = 0; i < NB_ESC; ++i) {
    while (desire_angle[i] >= 360.0) {
      desire_angle[i] -= 360.0;
    }
    while (desire_angle[i] < 0.0) {
      desire_angle[i] += 360.0;
    }
    while (command_angle[i] >= 360.0) {
      command_angle[i] -= 360.0;
    }
    while (command_angle[i] < 0.0) {
      command_angle[i] += 360.0;
    }
  }
  // Read data from motors
  Read_CAN();

  // Do PI angle synchronizing control
  Angle_Sync_Loop();
  // Do PI speed contol, then sending command to motor through CAN bus
  Speed_Control_Loop();


  if (ina260.readBusVoltage() < 19000.0) {
    msg_send.id = 0x200;
    for (int i = 0; i < 8; i++) {
      msg_send.buf[i] = 0;
    }
    CAN.write(msg_send);

    while (1) {
      digitalWrite(13, HIGH);
      delay(500);
      digitalWrite(13, LOW);
      delay(500);
    }

  }
}

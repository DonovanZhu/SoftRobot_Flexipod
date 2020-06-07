#include <Wire.h>
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "TeensyCAN.h"
#include "MPU9250.h"
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN;

#define PID_H 10000.0
#define PID_L -10000.0
#define speedDirectionBoundary 32768.0
#define maxBoundary 65535.0
#define drive_ratio 36.0
#define k_p 31.0
#define k_i 0.075

MPU9250 IMU(Wire,0x68);
int status;

// Globals
bool  can_received[4]     = {0, 0, 0, 0};
int angle_meas[4]         = {0, 0, 0, 0};
int torque_meas[4]        = {0, 0, 0, 0};
int w_meas[4]             = {0, 0, 0, 0};

double speed_meas[4]         = {0.0, 0.0, 0.0, 0.0};
double desired_speed[NB_ESC] = {0.0, 0.0, 0.0, 0.0};
double speed_command[NB_ESC] = {0.0, 0.0, 0.0, 0.0};
double dt[4]                 = {0.0, 0.0, 0.0, 0.0};
double error[4]              = {0.0, 0.0, 0.0, 0.0};
double error_former[4]       = {0.0, 0.0, 0.0, 0.0};
double error_sum[4]          = {0.0, 0.0, 0.0, 0.0};



Teensycomm_struct_t Teensy_comm = {COMM_MAGIC, {}, {}, {}};
RPicomm_struct_t    RPi_comm = {COMM_MAGIC, {}};
CAN_message_t msg_recv;
CAN_message_t msg_send;

// Manage communication with the host
int Teensy_comm_update(void) {
  static int          i;
  static uint8_t      *ptin  = (uint8_t*)(&RPi_comm),
                      *ptout = (uint8_t*)(&Teensy_comm);
  static int          ret;
  static int          in_cnt = 0;
  ret = 0;

  // Read all incoming bytes available until incoming structure is complete
  while((Serial.available() > 0) && (in_cnt < (int)sizeof(RPi_comm)))
    ptin[in_cnt++] = Serial.read();

  // Check if a complete incoming packet is available
  if (in_cnt == (int)sizeof(RPi_comm)) {

    // Clear incoming bytes counter
    in_cnt = 0;

    // Check the validity of the magic number
    if (RPi_comm.magic != COMM_MAGIC) {

      // Flush input buffer
      while (Serial.available())
        Serial.read();

      ret = ERROR_MAGIC;
    }
    else {
      for (i = 0; i < NB_ESC; i++)
        desired_speed[i] = RPi_comm.RPM[i];

      readCAN();
      
      PID();
      //imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      //imu::Quaternion quat = bno.getQuat();
      
      for (i = 0; i < NB_ESC; i++) {
        Teensy_comm.deg[i] = angle_meas[i];
        Teensy_comm.rpm[i] = w_meas[i];
        Teensy_comm.amp[i] = torque_meas[i];
      }
      IMU.readSensor();

      Teensy_comm.acc[0] = IMU.getAccelX_mss();
      Teensy_comm.acc[1] = IMU.getAccelY_mss();
      Teensy_comm.acc[2] = IMU.getAccelZ_mss();

      Teensy_comm.gyr[0] = IMU.getGyroX_rads();
      Teensy_comm.gyr[1] = IMU.getGyroY_rads();
      Teensy_comm.gyr[2] = IMU.getGyroZ_rads();

      // Send data structure to host
      Serial.write(ptout, sizeof(Teensy_comm));

      // Force immediate transmission
      Serial.send_now();
    }
  }

  return ret;
}

void PID() {
  int i;

  msg_send.id = 0x200;
  for (i = 0; i < NB_ESC; ++i) {
    /*
    if (time_step[i] - time_step_former[i] < 0)
      dt[i] = (double)(time_step[i] - time_step_former[i] + 65536) / 1000000.0;
    else
      dt[i] = (double)(time_step[i] - time_step_former[i]) / 1000000.0;
    */
    /***********************PID***********************/
    //time_step_former[i] = time_step[i];
    error[i] = desired_speed[i] - speed_meas[i];
    
    //error_sum[i] += error[i];
    speed_command[i] += (k_p * (error[i] - error_former[i]) + k_i * error[i]);
    error_former[i] = error[i];
    
    if(speed_command[i] >= 0)
    {
      if(speed_command[i] > PID_H)
        speed_command[i] = PID_H;
      int v = int(speed_command[i]);
      //int v = int((speed_command[i] / PID_H) * speedDirectionBoundary);
      msg_send.buf[2 * i + 1] = v & 0x00ff;
      msg_send.buf[2 * i] = (v >> 8) & 0x00ff;
      
    }
    else
    {
      if(speed_command[i] < PID_L)
        speed_command[i] = PID_L;
      int v = 0xffff + int(speed_command[i]);
      //int v = 0xffff - int((speed_command[i] / PID_L) * speedDirectionBoundary);
      msg_send.buf[2 * i + 1] = v & 0x00ff;
      msg_send.buf[2 * i] = (v >> 8) & 0x00ff;
    }
  }
  CAN.write(msg_send);
}

void readCAN() {
  int rpm, angle, torque;
  int i = 0;
  while (true) {
    if (CAN.read(msg_recv)) {
      if (!can_received[msg_recv.id - 513]) {
        //time_step[msg_recv.id - 513] = msg_recv.timestamp;
        rpm = 0;
        rpm |= (int16_t)(unsigned char)msg_recv.buf[2] << 8;
        rpm |= (int16_t)(unsigned char)msg_recv.buf[3];
        w_meas[msg_recv.id - 513] = rpm;
        if (rpm < speedDirectionBoundary)
          speed_meas[msg_recv.id - 513] = (double)rpm / drive_ratio;
        else
          speed_meas[msg_recv.id - 513] = - (double)(maxBoundary - rpm) / drive_ratio;
        angle = 0;
        angle |= (int16_t)(unsigned char)msg_recv.buf[0] << 8;
        angle |= (int16_t)(unsigned char)msg_recv.buf[1];
        angle_meas[msg_recv.id - 513] = angle;
        torque = 0;
        torque |= (int16_t)(unsigned char)msg_recv.buf[4] << 8;
        torque |= (int16_t)(unsigned char)msg_recv.buf[5];
        torque_meas[msg_recv.id - 513] = torque;
        can_received[msg_recv.id - 513] = true;
        i++;
      }
    }
    if (i == NB_ESC)
    {
      for (int k = 0; k < NB_ESC; ++k)
        can_received[k] = false;
      break;
    }
  }
}

void CAN_init() {
  int i = 0;
  while (true) {
    if (CAN.read(msg_recv)) {
      if (!can_received[msg_recv.id - 513]) {
        //time_step_former[msg_recv.id - 513] = msg_recv.timestamp;
        can_received[msg_recv.id - 513] = true;
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


void setup() {
  IMU.begin();
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(4);
  CAN.begin();
  CAN.setBaudRate(1000000);
  CAN_init();
  Serial.begin(USB_UART_SPEED);
}

void loop() {
  Teensy_comm_update();
}

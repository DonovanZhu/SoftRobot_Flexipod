#include <FlexCAN_T4.h>
#include "TeensyCAN.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN;

// Globals
double  speed_command[MOTOR_NUM];   // Command output of PID controller

double  acc[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
double  mag[3];
double  mag_tmp[3];
double  gyr[3];

double time_now;
double time_former;


Teensycomm_struct_t   Teensy_comm = {{}, {}, {}, {}, {}, {}, {}, {}, {}};   // For holding data sent to Jetson
Jetson_comm_struct_t  Jetson_comm    = {{}};                   // For holding data received from Jetson

static uint8_t  *ptin  = (uint8_t*)(&Jetson_comm);
static uint8_t  *ptout = (uint8_t*)(&Teensy_comm);
static int      in_cnt = 0;

// Set CAN bus message structure as CAN2.0
CAN_message_t msg_recv; // For receiving data on CAN bus
CAN_message_t msg_send; // For sending data on CAN bus





void Speed_Control_Loop(int motor_id, double desired_speed) {
  int direc = 0;
  if (desired_speed < 0) {
    desired_speed = (int)(256 + desired_speed);
    direc = 0xff;
  }
  // Set the CAN message ID as 0x200
  msg_send.id = motor_id;
  msg_send.buf[0] = 0xA2;
  msg_send.buf[1] = 0x00;
  msg_send.buf[2] = 0x00;
  msg_send.buf[3] = 0x00;
  msg_send.buf[4] = 0x00;
  msg_send.buf[5] = 0x00;
  msg_send.buf[6] = desired_speed;
  msg_send.buf[7] = direc;
  CAN.write(msg_send); // Write the message on CAN bus
}

//
// Read message from CAN bus
//
void Read_CAN() {
}



void setup() {
  // Switch on CAN bus
  CAN.begin();
  CAN.setBaudRate(1000000);
  CAN.setClock(CLK_60MHz);

  // Open Serial port in speed 115200 Baudrate
  Serial.begin(USB_UART_SPEED);
  time_now = micros();
}

void loop() {

  in_cnt = 0;


  // Read all incoming bytes available until incoming structure is complete
  while ((Serial.available() > 0) && (in_cnt < (int)sizeof(Jetson_comm)))
    ptin[in_cnt++] = Serial.read();

  // Check if a complete incoming packet is available
  if (in_cnt == (int)sizeof(Jetson_comm)) {

    // Clear incoming bytes counter
    in_cnt = 0;

    // Save angle, speed and torque into struct Teensy_comm
    for (int i = 0; i < MOTOR_NUM; i++) {
      Teensy_comm.angle[i]  = 0;
      Teensy_comm.rspeed[i] = 0;
      Teensy_comm.torque[i] = 0;
      Teensy_comm.comd[i] = Jetson_comm.comd[i];
    }

    // Read data from IMU(MPU9250)

    // Save acceleration (m/s^2) of IMU into struct Teensy_comm
    Teensy_comm.acc[0] = 0;
    Teensy_comm.acc[1] = 0;
    Teensy_comm.acc[2] = 0;

    // Save gyroscope (deg/s) of IMU into struct Teensy_comm
    Teensy_comm.gyr[0] = 1;
    Teensy_comm.gyr[1] = 1;
    Teensy_comm.gyr[2] = 1;

    Teensy_comm.mag[0] = 2;
    Teensy_comm.mag[1] = 2;
    Teensy_comm.mag[2] = 2;

    Teensy_comm.eular[0] = 3;
    Teensy_comm.eular[1] = 3;
    Teensy_comm.eular[2] = 3;

    Teensy_comm.timestamps = (micros() - time_now) / 1000000.0;
    // Send data structure Teensy_comm to Jetson
    Serial.write(ptout, sizeof(Teensy_comm));

    // Force immediate transmission
    Serial.send_now();
  }
  for (int i = 0; i < MOTOR_NUM; ++i)
    speed_command[i] = (double)Jetson_comm.comd[i];
  Speed_Control_Loop(0x141, speed_command[0]);
  Speed_Control_Loop(0x142, speed_command[1]);
  Speed_Control_Loop(0x143, speed_command[2]);
//  if (CAN.read(msg_recv)) {
//    deg_d = 0;
//    deg_d |= (int32_t)(unsigned char)msg_send.buf[7] << 24;
//    deg_d |= (int32_t)(unsigned char)msg_send.buf[6] << 16;
//    deg_d |= (int32_t)(unsigned char)msg_send.buf[5] << 8;
//    deg_d |= (int32_t)(unsigned char)msg_send.buf[4];
//    angle = deg_d * 0.01;
//    Serial.print(angle / 48);
//    Serial.print(" ");
//    
//    deg = 0;
//    deg |= (int16_t)(unsigned char)msg_recv.buf[5] << 8;
//    deg |= (int16_t)(unsigned char)msg_recv.buf[4];
//    Serial.println(deg / 48.0);
//  }
}

#include <FlexCAN_T4.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include "MadgwickAHRS.h"
#include "Teensy.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN_F;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN_B;
// Globals
float angle_command[MOTOR_NUM]  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   // Command output of PID controller
float deg_now[MOTOR_NUM];
float deg_former[MOTOR_NUM];
int   r_num[MOTOR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float time_now;
float time_former;

float ang_recv[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float vel_recv[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float cur_recv[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float upper_limit[MOTOR_NUM] = { 2.4,  1.93,  1.6,  4.7,  1.93,  1.6,  4.7,  1.93,  1.6,  2.4,  1.93,  1.6};
float lower_limit[MOTOR_NUM] = {-4.7, -1.93, -1.6, -2.4, -1.93, -1.6, -2.4, -1.93, -1.6, -4.7, -1.93, -1.6};
float acc[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float mag[3];
float mag_tmp[3];
float gyr[3];

float deltat;
Teensycomm_struct_t   Teensy_comm = {{}, {}, {}, {}, {}, {}, {}, {}, {}};   // For holding data sent to Jetson
Jetson_comm_struct_t  Jetson_comm    = {{}};                   // For holding data received from Jetson

static uint8_t  *ptin  = (uint8_t*)(&Jetson_comm);
static uint8_t  *ptout = (uint8_t*)(&Teensy_comm);
static int      in_cnt = 0;

// Set CAN bus message structure as CAN2.0
CAN_message_t msg_recv; // For receiving data on CAN bus
CAN_message_t msg_send; // For sending data on CAN bus


Adafruit_LSM6DS sox;
Adafruit_LIS3MDL lis;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;
sensors_event_t mage;


const float magn_ellipsoid_center[3] = {0.262689, -6.89484, 4.0776};
const float magn_ellipsoid_transform[3][3] = {{0.899993, 0.0341615, -0.000181209}, {0.0341615, 0.988324, -0.000514259}, { -0.000181209, -0.000514259, 0.952566}};


Quaternion qua;
EulerAngles eul;

void read_sensors() {
  sox.getEvent(&accel, &gyro, &temp);
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

void Motor_Init() {
  msg_send.buf[0] = 0xA3;
  msg_send.buf[1] = 0x00;
  msg_send.buf[2] = 0x00;
  msg_send.buf[3] = 0x00;
  msg_send.buf[4] = 0x00;
  msg_send.buf[5] = 0x00;
  msg_send.buf[6] = 0x00;
  msg_send.buf[7] = 0x00;

  for (int i = 0; i < MOTOR_NUM / 2; ++i) {
      msg_send.id = 0x141 + i;
      CAN_F.write(msg_send);
      CAN_B.write(msg_send);
  }
}

void Motor_Data(int id) {
  int deg = 0;
  deg |= (int16_t)(unsigned char)msg_recv.buf[7] << 8;
  deg |= (int16_t)(unsigned char)msg_recv.buf[6];
  deg_now[id] = (float)deg / 65535.0 * 2 * PI;
  if (deg_now[id] - deg_former[id] < -PI)
    r_num[id] += 1;
  else if (deg_now[id] - deg_former[id] > PI)
    r_num[id] -= 1;

  deg_former[id] = deg_now[id];
  ang_recv[id] = (deg_now [id]+ r_num[id] * 2 * PI) / 8.0;

  int vel = 0;
  vel |= (int16_t)(unsigned char)msg_recv.buf[5] << 8;
  vel |= (int16_t)(unsigned char)msg_recv.buf[4];
  if (vel > 0x8000)
    vel -= 0x10000;
  vel_recv[id] = (float)vel * PI / 180.0;

  int cur = 0;
  cur |= (int16_t)(unsigned char)msg_recv.buf[3] << 8;
  cur |= (int16_t)(unsigned char)msg_recv.buf[2];
  if (cur > 0x8000)
    cur -= 0x10000;
  cur_recv[id] = (float)cur * 33.0 / 2048.0;
}

void Angle_Control_Loop(int motor_id, float pos_command, bool front_legs) {
  int pos = round(pos_command / 0.01);
  
  if (pos < 0)
    pos = 0x100000000 + pos;

  unsigned int pos_1 = pos & 0xff;
  unsigned int pos_2 = (pos >> 8) & 0xff;
  unsigned int pos_3 = (pos >> 16) & 0xff;
  unsigned int pos_4 = (pos >> 24) & 0xff;

  // Set the CAN message ID as 0x200
  msg_send.id = motor_id;
  msg_send.buf[0] = 0xA3;
  msg_send.buf[1] = 0x00;
  msg_send.buf[2] = 0x00;
  msg_send.buf[3] = 0x00;
  msg_send.buf[4] = pos_1;
  msg_send.buf[5] = pos_2;
  msg_send.buf[6] = pos_3;
  msg_send.buf[7] = pos_4;


  if (front_legs) {
    int id = motor_id - 0x141;
    CAN_F.write(msg_send); // Write the message on CAN bus
    while (true) {
      if (CAN_F.read(msg_recv)) {
        Motor_Data(id);
        break;
      }
    }
  }
    
  else {
    int id = motor_id - 0x141 + (int)(MOTOR_NUM / 2);
    CAN_B.write(msg_send); // Write the message on CAN bus
    while (true) {
      if (CAN_B.read(msg_recv)) {
        Motor_Data(id);
        break;
      }
    }
  }
}

void Jetson_Teensy () {
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
      Teensy_comm.angle[i]  = ang_recv[i];
      Teensy_comm.rspeed[i] = vel_recv[i];
      Teensy_comm.torque[i] = cur_recv[i];
      Teensy_comm.comd[i] = angle_command[i];
    }

    // Read data from IMU(MPU9250)
   // Save acceleration (m/s^2) of IMU into struct Teensy_comm
    Teensy_comm.acc[0] = accel.acceleration.x;
    Teensy_comm.acc[1] = accel.acceleration.y;
    Teensy_comm.acc[2] = accel.acceleration.z;

    // Save gyroscope (rad/s) of IMU into struct Teensy_comm
    Teensy_comm.gyr[0] = gyro.gyro.x / 180.0 * PI;
    Teensy_comm.gyr[1] = gyro.gyro.y / 180.0 * PI;
    Teensy_comm.gyr[2] = gyro.gyro.z / 180.0 * PI;

    Teensy_comm.mag[0] = mage.magnetic.x;
    Teensy_comm.mag[1] = mage.magnetic.y;
    Teensy_comm.mag[2] = mage.magnetic.z;

    Teensy_comm.eular[0] = eul.roll_e;
    Teensy_comm.eular[1] = eul.pitch_e;
    Teensy_comm.eular[2] = eul.yaw_e;

    Teensy_comm.timestamps = time_now / 1000000.0;
    // Send data structure Teensy_comm to Jetson
    Serial.write(ptout, sizeof(Teensy_comm));

    // Force immediate transmission
    Serial.send_now();
  }
}



void setup() {
  // Switch on CAN bus
  Serial.begin(USB_UART_SPEED);
  delay(1000);

  
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
  
  CAN_F.begin();
  CAN_F.setBaudRate(1000000);
  CAN_F.setClock(CLK_60MHz);

  CAN_B.begin();
  CAN_B.setBaudRate(1000000);
  CAN_B.setClock(CLK_60MHz);
  // Open Serial port in speed 1000000 Baudrate
  Motor_Init();
  delay(400);
  time_former = (float)micros();
}

void loop() {

  read_sensors();

  compensate_sensor_errors();
  time_now = (float)micros();
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
  
  Jetson_Teensy ();

  for (int i = 0; i < MOTOR_NUM; ++i) {
    angle_command[i] = Jetson_comm.comd[i];
    if (angle_command[i] > upper_limit[i])
      angle_command[i] = upper_limit[i];
    else if (angle_command[i] < lower_limit[i])
      angle_command[i] = lower_limit[i];
    angle_command[i] = angle_command[i] * 180.0 * DRIVE_RATIO / PI;


  }
  
  for (int i = 0; i < MOTOR_NUM / 2; ++i){
      Angle_Control_Loop(0x141 + i, angle_command[i], true);
      Angle_Control_Loop(0x141 + i, angle_command[i + (int)(MOTOR_NUM / 2)], false);
  }
}

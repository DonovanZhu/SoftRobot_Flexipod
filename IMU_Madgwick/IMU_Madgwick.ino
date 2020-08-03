#include "MPU9250.h"
#include "MadgwickAHRS.h"
MPU9250 IMU(Wire,0x68);

float GYRO_X_OFFSET;
float GYRO_Y_OFFSET;
float GYRO_Z_OFFSET;

// 0.0954546 0.2109326 0.3001436
float ACCEL_X_OFFSET;
float ACCEL_Y_OFFSET;
float ACCEL_Z_OFFSET;

const float magn_ellipsoid_center[3] = {-1.22362, -3.49591, -28.3068};
const float magn_ellipsoid_transform[3][3] = {{0.936683, -0.0120599, -0.00747369}, {-0.0120599, 0.997691, -5.88781e-05}, {-0.00747369, -5.88781e-05, 0.846255}};
// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float magnetom[3];
float magnetom_tmp[3];
float gyro[3];

float time_1;
float time_2;
float deltat;

struct Quaternion {
    float w, x, y, z;
};

struct EulerAngles {
    float roll_e, pitch_e, yaw_e;
};

Quaternion qua;
EulerAngles eul;
  
void read_sensors() {
  IMU.readSensor();
  accel[0] = IMU.getAccelX_mss();
  accel[1] = IMU.getAccelY_mss();
  accel[2] = IMU.getAccelZ_mss();

  magnetom[0] = IMU.getMagX_uT(); // Receiving data's unit is uT, times 10 to transfer it to mGauss
  magnetom[1] = IMU.getMagY_uT();
  magnetom[2] = IMU.getMagZ_uT();

  gyro[0] = IMU.getGyroX_rads();
  gyro[1] = IMU.getGyroY_rads();
  gyro[2] = IMU.getGyroZ_rads();
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
    // Compensate accelerometer error
    // 0.0954546 0.2109326 0.3001436
    accel[0] = accel[0] - 0.0954546;
    accel[1] = accel[1] - 0.2109326;
    accel[2] = accel[2] + 0.3001436;

    // Compensate magnetometer error
    for (int i = 0; i < 3; i++)
      magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);

    // Compensate gyroscope error
    gyro[0] -= GYRO_X_OFFSET;
    gyro[1] -= GYRO_Y_OFFSET;
    gyro[2] -= GYRO_Z_OFFSET;
}

/* This file is part of the Razor AHRS Firmware */
// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3])
{
  for(int x = 0; x < 3; x++)
  {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}

void sendToPC(float* data1, float* data2, float* data3)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3]};
  Serial.write(buf, 12);
}



EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll_e = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch_e = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch_e = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw_e = atan2(siny_cosp, cosy_cosp);

    return angles;
}

void setup() {
  Serial.begin(1000000);
  while (!Serial) yield();

  IMU.begin();

  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(9);
  int j = 0;
  for (int i = 0; i < 20000; i++) {
    IMU.readSensor();
    GYRO_X_OFFSET += IMU.getGyroX_rads();
    GYRO_Y_OFFSET += IMU.getGyroY_rads();
    GYRO_Z_OFFSET += IMU.getGyroZ_rads();
    j++;
  }
  GYRO_X_OFFSET /= j;
  GYRO_Y_OFFSET /= j;
  GYRO_Z_OFFSET /= j;
  time_2 = micros();
}


// -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz
void loop() {

  // put your main code here, to run repeatedly:
  read_sensors();

  compensate_sensor_errors();
  
  //for (int i = 0; i < 10; i++) {
    time_1 = micros();
    deltat = (float)(time_1 - time_2) / 1000000.0f;
    time_2 = time_1;
    MadgwickQuaternionUpdate(accel[0], accel[1], accel[2],
                             gyro[0], gyro[1], gyro[2],
                             magnetom[0], magnetom[1], magnetom[2], deltat);
    //delayMicroseconds(10);
  //}

  //Serial.print(time_2 - time_1);
  //Serial.print(" ");
  qua.w = q[0];
  qua.x = q[1];
  qua.y = q[2];
  qua.z = q[3];
  eul = ToEulerAngles(qua);
  eul.yaw_e += 0.8;
  if (eul.yaw_e > PI) {
    eul.yaw_e -= 2 * PI;
  }
  //Serial.print(eul.roll * 180.0 / PI);
  //Serial.print(" ");
  //Serial.print(eul.pitch * 180.0 / PI);
  //Serial.print(" ");
  //Serial.print(eul.yaw * 180.0 / PI);
  //Serial.println(" ");
  sendToPC(&eul.roll_e, &eul.pitch_e, &eul.yaw_e);
  delayMicroseconds(500);
}

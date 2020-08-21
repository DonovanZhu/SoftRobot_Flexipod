#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>

Adafruit_LSM6DS sox;
Adafruit_LIS3MDL lis;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mage;
sensors_event_t temp;

int sample_times = 1;

float acc_cal[3] = {0.0, 0.0, 0.0};
float gyro_cal[3] = {0.0, 0.0, 0.0};

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

void setup() {
  Serial.begin(1000000);
  while (!Serial) yield();

  sox.begin_I2C();

  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);

  sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );

  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);

  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  lis.begin_I2C();          // hardware I2C mode, can pass in address & alt Wire

  lis.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);

  lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  lis.setDataRate(LIS3MDL_DATARATE_155_HZ);

  lis.setRange(LIS3MDL_RANGE_4_GAUSS);

  lis.setIntThreshold(500);
  lis.configInterrupt(false, false, true, // enable z axis
                      true, // polarity
                      false, // don't latch
                      true); // enabled!

}

//-0.0159352 -0.2338077 10.0549326 0.0127355 0.0005773 -0.0068255
//  -0.0162423 -0.2332105 10.0582304 0.0127815 0.0005184 -0.0067737

void loop() {
  //sox.getEvent(&accel, &gyro, &temp);
  lis.getEvent(&mage);
  /*
  acc_cal[0] += accel.acceleration.x;
  acc_cal[1] += accel.acceleration.y;
  acc_cal[2] += accel.acceleration.z;

  gyro_cal[0] += gyro.gyro.x;
  gyro_cal[1] += gyro.gyro.y;
  gyro_cal[2] += gyro.gyro.z;
  */
  /*
  Serial.print(acc_cal[0] / sample_times, 7);
  Serial.print(" ");
  Serial.print(acc_cal[1] / sample_times, 7);
  Serial.print(" ");
  Serial.print(acc_cal[2] / sample_times, 7);
  Serial.print(" ");
  Serial.print(gyro_cal[0] / sample_times, 7);
  Serial.print(" ");
  Serial.print(gyro_cal[1] / sample_times, 7);
  Serial.print(" ");
  Serial.println(gyro_cal[2] / sample_times, 7);
  */
  sendToPC(&mage.magnetic.x, &mage.magnetic.y, &mage.magnetic.z);
  
}

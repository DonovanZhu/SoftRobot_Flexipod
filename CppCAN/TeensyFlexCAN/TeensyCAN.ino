#include "FlexCAN_T4.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can;
CAN_message_t msg_recv;
CAN_message_t msg_send;

#define PID_H 10000.0
#define PID_L -10000.0
#define speedDirectionBoundary 32768.0
#define maxBoundary 65535.0
#define drive_ratio 36.0
#define k_p 65.0
#define k_i 1.0
#define k_d 0.00

double time_step_former[4] = {0.0, 0.0, 0.0, 0.0};
double time_step_later[4] = {0.0, 0.0, 0.0, 0.0};
int can_received[4] = {0, 0, 0, 0};
double speed_meas[4] = {0.0, 0.0, 0.0, 0.0};
double dt[4] = {0.0, 0.0, 0.0, 0.0};
double error[4] = {0.0, 0.0, 0.0, 0.0};
double error_former[4] = {0.0, 0.0, 0.0, 0.0};
double error_sum[4] = {0.0, 0.0, 0.0, 0.0};
double control_output[4] = {0.0, 0.0, 0.0, 0.0};
double desire_speed[4] = {200.0, 0.0, 0.0, 0.0};
int angle;
int rpm;
int torque;


void setup(void) {
  can.begin();
  can.setBaudRate(1000000);
  msg_send.id = 0x200;
  int i = 0;
  while(1) {
    if (can.read(msg_recv)) {
      if (!can_received[msg_recv.id - 0x201]) {
        time_step_later[msg_recv.id - 0x201] = msg_recv.timestamp;
        rpm = 0;
        rpm |= (int16_t)(unsigned char)msg_recv.buf[2] << 8;
        rpm |= (int16_t)(unsigned char)msg_recv.buf[3];
        if (rpm >= speedDirectionBoundary)
          speed_meas[msg_recv.id - 0x201] = -(maxBoundary - rpm) / drive_ratio;
        else
          speed_meas[msg_recv.id - 0x201] = rpm / drive_ratio;
        can_received[msg_recv.id - 0x201] = true;

        i++;
      }
      if (i == 4) {
        for (int k = 0; k < 4; ++k) {
          can_received[k] = 0;
          error_former[k] = desire_speed[k] - speed_meas[k];
        }
        break;
      }
    }
  }
}

void loop() {
  int i = 0;
  while(1) {
    if ( can.read(msg_recv) ) {
      if (!can_received[msg_recv.id - 0x201]) {
        time_step_later[msg_recv.id - 0x201] = msg_recv.timestamp;
        rpm = 0;
        rpm |= (int16_t)(unsigned char)msg_recv.buf[2] << 8;
        rpm |= (int16_t)(unsigned char)msg_recv.buf[3];
        if (rpm >= speedDirectionBoundary)
          speed_meas[msg_recv.id - 0x201] = -(maxBoundary - rpm) / drive_ratio;
        else
          speed_meas[msg_recv.id - 0x201] = rpm / drive_ratio;
        can_received[msg_recv.id - 0x201] = true;

        i++;
      }
      if (i == 4) {
        for (int k = 0; k < 4; ++k)
          can_received[k] = 0;
        break;
      }
    }
  }
  
  for (i = 0; i < 4; ++i) {
    dt[i] = (double)(time_step_later[i] - time_step_former[i]);
    if (dt[i] < 0)
      dt[i] = (double)(0xffff + dt[i]) / 1000000.0;
    else
      dt[i] /= 1000000.0;
    Serial.print(" v = ");
    Serial.print(speed_meas[i]);
    Serial.print(" t = ");
    Serial.print(dt[i]);
    /***********************PID***********************/
    time_step_former[i] = time_step_later[i];
    error[i] = desire_speed[i] - speed_meas[i];
  
    error_sum[i] += error[i];
    control_output[i] = k_p * (error[i] + error_sum[i] * 0.001 + k_d * (error[i] - error_former[i]) / dt[i]);
    error_former[i] = error[i];
    
    if(control_output[i] >= 0) {
      if(control_output[i] > PID_H)
        control_output[i] = PID_H;
      int v = int(control_output[i]);
      // int v = int((command[i] / PID_H) * speedDirectionBoundary);
      msg_send.buf[2 * i + 1] = v & 0x00ff;
      msg_send.buf[2 * i] = (v >> 8) & 0x00ff;  
    }
    else {
      if(control_output[i] < PID_L)
        control_output[i] = PID_L;
      int v = 0xffff + int(control_output[i]);
      // int v = 0xffff - int((command[i] / PID_L) * speedDirectionBoundary);
      msg_send.buf[2 * i + 1] = v & 0x00ff;
      msg_send.buf[2 * i] = (v >> 8) & 0x00ff;
    }
  }
  Serial.println(" ");

  can.write(msg_send);
}

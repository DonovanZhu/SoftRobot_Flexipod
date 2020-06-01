// Includes
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "TeensyCAN.h"

#define PID_H 10000.0
#define PID_L -10000.0
#define speedDirectionBoundary 32768.0
#define maxBoundary 65535.0
#define drive_ratio 36.0
#define k_p 1.8
#define k_i 0.00000
#define k_d 0.0


FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN;

// Globals

bool  can_received[4] = {0, 0, 0, 0};
int   time_step[4] = {0, 0, 0, 0};
int   time_step_former[4] = {0, 0, 0, 0};

double desired_speed[NB_ESC] = {0.0, 0.0, 0.0, 0.0};
double angle_meas[4] = {0.0, 0.0, 0.0, 0.0};
double torque_meas[4] = {0.0, 0.0, 0.0, 0.0};
double speed_meas[4] = {0.0, 0.0, 0.0, 0.0};
double dt[4] = {0.0, 0.0, 0.0, 0.0};
double error[4] = {0.0, 0.0, 0.0, 0.0};
double error_former[4] = {0.0, 0.0, 0.0, 0.0};
double error_sum[4] = {0.0, 0.0, 0.0, 0.0};
double speed_command[4] = {0.0, 0.0, 0.0, 0.0};


Teensycomm_struct_t Teensy_comm = { COMM_MAGIC, {}, {}, {}};
RPicomm_struct_t   RPi_comm =   { COMM_MAGIC, {} };

CAN_message_t msg_recv;


// Manage communication with the host
int Teensy_comm_update( void ) {
  static int          i;
  static uint8_t      *ptin   = (uint8_t*)(&RPi_comm),
                      *ptout  = (uint8_t*)(&Teensy_comm);
  static int          ret;
  static int          in_cnt = 0;
  
  ret = 0;
  
  // Read all incoming bytes available until incoming structure is complete
  while(( Serial.available( ) > 0 ) && ( in_cnt < (int)sizeof( RPi_comm )))
    ptin[in_cnt++] = Serial.read( );
  
  // Check if a complete incoming packet is available
  if ( in_cnt == (int)sizeof( RPi_comm ) ) {
  
    // Clear incoming bytes counter
    in_cnt = 0;
    
    
    // Check the validity of the magic number
    if ( RPi_comm.magic != COMM_MAGIC ) {
    
      // Flush input buffer
      while ( Serial.available( ) )
        Serial.read( );
    
      ret = ERROR_MAGIC;
    }
    else {
      for ( i = 0; i < NB_ESC; i++ ) {
        desired_speed[i] = RPi_comm.RPM[i];
      }

      readCAN();
      PID();

      for ( i = 0; i < NB_ESC; i++ ) {
        Teensy_comm.deg[i] = angle_meas[i];
        Teensy_comm.rpm[i] = speed_meas[i];
        Teensy_comm.amp[i] = torque_meas[i];
      }
      
      // Send data structure to host
      Serial.write( ptout, sizeof( Teensy_comm ) );
      
      // Force immediate transmission
      Serial.send_now( );
    }
  }

  return ret;
}

void PID() {
  int i;
  CAN_message_t msg_send;
  msg_send.id = 0x200;
  for (i = 0; i < 4; ++i)
  {
    if (time_step[i] - time_step_former[i] < 0)
      dt[i] = (double)(time_step[i] - time_step_former[i] + 65536) / 1000000.0;
    else
      dt[i] = (double)(time_step[i] - time_step_former[i]) / 1000000.0;

    /***********************PID***********************/
    time_step_former[i] = time_step[i];
    error[i] = desired_speed[i] - speed_meas[i];
    
    //error_sum[i] += error[i];
    speed_command[i] += k_p * (error[i] - error_former[i]) + k_i * error[i];
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
  //CAN.write(msg_send); 
}

void readCAN() {
  int rpm, angle, torque;
  int i = 0;
  while (true) {
    if (CAN.read(msg_recv)) {
      if (!can_received[msg_recv.id - 513]) {
        time_step[msg_recv.id - 513] = msg_recv.timestamp;
        rpm = 0;
        rpm |= (int16_t)(unsigned char)msg_recv.buf[2] << 8;
        rpm |= (int16_t)(unsigned char)msg_recv.buf[3];
        if (rpm < speedDirectionBoundary)
          speed_meas[msg_recv.id - 513] = (double)rpm / drive_ratio;
        else
          speed_meas[msg_recv.id - 513] = - (double)(maxBoundary - rpm) / drive_ratio;
        angle = 0;
        angle |= (int16_t)(unsigned char)msg_recv.buf[0] << 8;
        angle |= (int16_t)(unsigned char)msg_recv.buf[1];
        angle_meas[msg_recv.id - 513] = (double)angle / 8191.0 * 360.0;;
        torque = 0;
        torque |= (int16_t)(unsigned char)msg_recv.buf[4] << 8;
        torque |= (int16_t)(unsigned char)msg_recv.buf[5];
        torque_meas[msg_recv.id - 513] = (double)torque;
        can_received[msg_recv.id - 513] = true;
        i++;
      }
    }
    if (i == 4)
    {
      for (int k = 0; k < 4; ++k)
        can_received[k] = false;
      break;
    }
  }
}

void CAN_init( ) {
  int rpm;
  int i = 0;
  while (true) {
    if (CAN.read(msg_recv)) {
      if (!can_received[msg_recv.id - 513]) {
        time_step_former[msg_recv.id - 513] = msg_recv.timestamp;
        rpm = 0;
        rpm |= (int16_t)(unsigned char)msg_recv.buf[2] << 8;
        rpm |= (int16_t)(unsigned char)msg_recv.buf[3];
        if (rpm < speedDirectionBoundary)
          speed_meas[msg_recv.id - 513] = (double)rpm / drive_ratio;
        else
          speed_meas[msg_recv.id - 513] = - (double)(maxBoundary - rpm) / drive_ratio;
        can_received[msg_recv.id - 513] = true;
        i++;
      }
    }
    if (i == 4) {
      for (int k = 0; k < 4; ++k) {
        can_received[k] = false;
        error_former[k] = desired_speed[k] - speed_meas[k];
      }
      break;
    }
  }
}

void setup() {

  CAN.begin();
  CAN.setBaudRate(1000000);

  Serial.begin( USB_UART_SPEED );

}

void loop( ) {
  CAN_init( );
  Teensy_comm_update(  );
}

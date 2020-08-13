#include <FlexCAN_T4.h>

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN;
CAN_message_t msg;
CAN_message_t msg_send;

int time_1 = 0;
int time_2 = 0;
int delta_t = 0;
int rec[4] = {0, 0, 0, 0};
void setup(void) {
  CAN.begin();
  CAN.setBaudRate(1000000);
  //CAN.setClock(CLK_60MHz);
}

void loop() {
  
  time_1 = micros();
  while(1) {
    if (CAN.read(msg)) {
      if (!rec[msg.id - 0x201]) {
        rec[msg.id - 0x201] = 1;
      }
      if (rec[0] && rec[1] && rec[2] && rec[3]) {
        for (int i = 0; i < 4; i++)
          rec[i] = 0;
        break;
      }
    }
  }

  msg_send.id = 0x200;
  msg_send.buf[1] = 200;
  CAN.write(msg_send);
  
  time_2 = micros();
  delta_t = time_2 - time_1;
  
  if (delta_t < 1150) {
    int delay_t = 1150 - delta_t;
    delayMicroseconds(delay_t);
  }
  int t = micros();
  Serial.print(" ");
  Serial.println(t - time_1);
  
}

#include <WiFi.h>
#include <WiFiUdp.h>
#include <MegunoLink.h>
#include <Filter.h>

int VEncoder = 35;
int HEncoder = 34;
ExponentialFilter<long> filterV(5, 0);
ExponentialFilter<long> filterH(5, 0);

int VValue = 0;
int HValue = 0;

const int V_LOW = 460;
const int V_HIGH = 2000;
const int V_MID = 1212;

const int H_LOW = 290;
const int H_HIGH = 2035;
const int H_MID = 1140;
/* WiFi network name and password */
const char * ssid = "NETGEAR97";
const char * pwd = "classysheep299";

// IP address to send UDP data to.
const char * sendRPiAddress = "192.168.0.77";
const int sendRPiPort = 1000;

const char * sendPCAddress = "192.168.0.67";
const int sendPCPort = 2000;

const int localPort = 1000;

//create UDP instance
WiFiUDP udp;

unsigned long time_previous;
unsigned long time_now;
unsigned long time_delta;

void setup()
{
  Serial.begin(115200);

  //Connect to the WiFi network
  WiFi.begin(ssid, pwd);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
  //This initializes udp and transfer buffer
  udp.begin(localPort);
  Serial.println(WiFi.localIP());
  time_previous = micros();

}

int error = 12;
int dataSize;
int sendPCinterval = 0;
int sendRPiinterval = 0;

char num1[4];
char num2[4];
int pos1 = 0;
int pos2 = 0;
int j = 0;
int motorSpeed;
char motorsData[64]; //buffer to hold incoming packet
int k;
bool f = true;
bool s = false;

double k_p = 0.01;
double k_i = 1000000000.0;
//double k_d = 0.0;

double k_p2 = 0.01;
double k_i2 = 1000000000.0;

double angle0;
double angle[4] = {0.0, 0.0, 0.0, 0.0};
double former_error[4] = {0.0, 0.0, 0.0, 0.0};
double new_error[4] = {0.0, 0.0, 0.0, 0.0};
double new_error2[4] = {0.0, 0.0, 0.0, 0.0};
double error_sum[4] = {0.0, 0.0, 0.0, 0.0};
double error_sum2[4] = {0.0, 0.0, 0.0, 0.0};

double average_angle;
int speed_error[4] = {0, 0, 0, 0};
double absolute_angle[4] = {0.0, 0.0, 0.0, 0.0};
bool first_loop = true;

void loop()
{

  VValue = analogRead(VEncoder);
  HValue = analogRead(HEncoder);
  filterV.Filter(VValue);
  filterH.Filter(HValue);
  VValue = filterV.Current();
  HValue = filterH.Current();
  VValue = constrain(VValue, V_LOW, V_HIGH);
  HValue = constrain(HValue, H_LOW, H_HIGH);
  Serial.print(VValue,HValue);

  // stay
  if (sendRPiinterval % 2 == 0)
  {
    if (abs(HValue - H_MID) < error && abs(VValue - V_MID) < error)
    {
      motorSpeed = 0x4000;
      udp.beginPacket(sendRPiAddress, sendRPiPort);
      udp.printf("%u,%u,%u,%u,", motorSpeed, motorSpeed, motorSpeed, motorSpeed);
      udp.endPacket();
    }
    else
    {
      time_delta = micros() - time_previous;
      time_previous = micros();

      // 90 degree
      if (abs(HValue - H_MID) < error && VValue > V_MID + error)
      {
        double per_error_2;

        motorSpeed = (int)((double)(VValue - V_MID) / (double)(V_HIGH - V_MID) * 0x4000) + 0x4000;

        udp.beginPacket(sendRPiAddress, sendRPiPort);
        udp.printf("%u,%u,%u,%u,", motorSpeed, motorSpeed, 0x8000 - motorSpeed, 0x8000 - motorSpeed);
        udp.endPacket();
      }

      // 270 degree
      if (abs(HValue - H_MID) < error && VValue < V_MID - error)
      {
        double per_error_2;

        motorSpeed = (int)((double)(VValue - V_MID) / (double)(V_MID - V_LOW) * 0x4000) + 0x4000;

        udp.beginPacket(sendRPiAddress, sendRPiPort);
        udp.printf("%u,%u,%u,%u,", motorSpeed, motorSpeed, 0x8000 - motorSpeed, 0x8000 - motorSpeed);
        udp.endPacket();
      }
      /*
      // 180 degree
      else if (abs(HValue - H_MID) > error && abs(VValue - V_MID) < error)
      {
        double per_error_4;
        average_angle = (absolute_angle[0] + absolute_angle[1] + absolute_angle[2] + absolute_angle[3]) / 4.0;
        if (HValue > H_MID)
        {
          motorSpeed = (int)((double)(HValue - H_MID) / (double)(H_HIGH - H_MID) * 0x2000) + 0x2000;
          for (int Id = 0; Id < 4; ++Id)
          {
            per_error_4 = constrain(anglePID_four(Id, absolute_angle[Id], time_delta), -0.3, 0.3 );
            if (per_error_4 < 0.0)
              speed_error[Id] = (int)(per_error_4 * (0x4000 - motorSpeed));
            if (per_error_4 >= 0.0)
              speed_error[Id] = (int)(per_error_4 * (motorSpeed - 0x2000));
          }
        }
        else
        {
          motorSpeed = (int)((double)(HValue - H_MID) / (double)(H_MID - H_LOW) * 0x2000) + 0x2000;
          for (int Id = 0; Id < 4; ++Id)
          {
            per_error_4 = constrain(anglePID_four(Id, absolute_angle[Id], time_delta), -0.3, 0.3);
            if (per_error_4 < 0.0)
              speed_error[Id] = (int)(per_error_4 * motorSpeed);
            if (per_error_4 >= 0.0)
              speed_error[Id] = (int)(per_error_4 * (0x2000 - motorSpeed));
          }
        }
        udp.beginPacket(sendRPiAddress, sendRPiPort);
        udp.printf("%u,%u,%u,%u,", motorSpeed - speed_error[0], motorSpeed - speed_error[1], motorSpeed - speed_error[2], motorSpeed - speed_error[3]);
        udp.endPacket();
      }

      // 135 degree
      else if (HValue <= (H_MID - error) && VValue >= (V_MID + error) && (VValue - V_MID) >= (H_MID - HValue))
      {
        int distance = (int)sqrt(abs(VValue - V_MID) * abs(VValue - V_MID) + abs(HValue - H_MID) * abs(HValue - H_MID));
        distance = constrain(distance, 0, V_HIGH - V_MID);
        motorSpeed = (int)((double)distance / (double)(V_HIGH - V_MID) * 0x2000) + 0x2000;
        double per = 1.0 - (double)(H_MID - HValue) / (double)(VValue - V_MID);
        udp.beginPacket(sendRPiAddress, sendRPiPort);
        udp.printf("%u,%u,%u,%u,", (int)((motorSpeed - 0x2000) * per + 0x2000), (int)((motorSpeed - 0x2000) * per + 0x2000),\
        0x4000 - motorSpeed, 0x4000 - motorSpeed);
        udp.endPacket();
        for (int m = 0; m < 4; ++m)
        {
          absolute_angle[m] = angle[m];
        }
      }
      
        // 225 degree
      else if (HValue <= (H_MID - error) && VValue <= (V_MID - error))
      {
        int distance = (int)sqrt(abs(VValue - V_MID) * abs(VValue - V_MID) + abs(HValue - H_MID) * abs(HValue - H_MID));
        distance = constrain(distance, 0, V_MID - V_LOW);
        motorSpeed = (int)((double)(-distance) / (double)(V_MID - V_LOW) * 0x2000) + 0x2000;
        double per = 1.0 - (double)(H_MID - HValue) / (double)(V_MID-VValue);
        udp.beginPacket(sendRPiAddress, sendRPiPort);
        udp.printf("%u,%u,%u,%u,", (int)((motorSpeed - 0x2000) * per + 0x2000), (int)((motorSpeed - 0x2000) * per + 0x2000),\
        0x4000 - motorSpeed, 0x4000 - motorSpeed);
        udp.endPacket();
        for (int m = 0; m < 4; ++m)
        {
          absolute_angle[m] = angle[m];
        }
      }
      // 315 degree
      else if (HValue >= (H_MID + error) && VValue < (V_MID - error))
      {
        int distance = (int)sqrt(abs(VValue - V_MID) * abs(VValue - V_MID) + abs(HValue - H_MID) * abs(HValue - H_MID));
        distance = constrain(distance, 0, V_MID - V_LOW);
        motorSpeed = (int)((double)(-distance) / (double)(V_MID - V_LOW) * 0x2000) + 0x2000;
        double per = 1.0 - (double)(HValue - H_MID) / (double)(V_MID-VValue);
        udp.beginPacket(sendRPiAddress, sendRPiPort);
        udp.printf("%u,%u,%u,%u,", motorSpeed, motorSpeed,\
        (int)(0x4000 - ((motorSpeed - 0x2000) * per + 0x2000)), (int)(0x4000 - ((motorSpeed - 0x2000) * per + 0x2000)));
        udp.endPacket();
        for (int m = 0; m < 4; ++m)
        {
          absolute_angle[m] = angle[m];
        }
      }
      // 45 degree
      else if (HValue > (H_MID + error) && VValue > (V_MID + error))
      {
        int distance = (int)sqrt(abs(VValue - V_MID) * abs(VValue - V_MID) + abs(HValue - H_MID) * abs(HValue - H_MID));
        distance = constrain(distance, 0, V_HIGH - V_MID);
        motorSpeed = (int)((double)distance / (double)(V_HIGH - V_MID) * 0x2000) + 0x2000;
        double per = 1.0 - (double)(HValue - H_MID) / (double)(VValue - V_MID);
        udp.beginPacket(sendRPiAddress, sendRPiPort);
        udp.printf("%u,%u,%u,%u,", motorSpeed, motorSpeed,\
        (int)((0x4000 - ((motorSpeed - 0x2000) * per + 0x2000))), (int)((0x4000 - ((motorSpeed - 0x2000) * per + 0x2000))));
        udp.endPacket();
        for (int m = 0; m < 4; ++m)
        {
          absolute_angle[m] = angle[m];
        }
      }
      */
    }
    sendRPiinterval = 0;
  }
  sendRPiinterval++;
}

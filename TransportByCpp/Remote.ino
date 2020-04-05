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
const int sendRPiPort = 1001;

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
double k_i = 6000000.0;
//double k_d = 0.0;

double k_p2 = 0.01;
double k_i2 = 8000000.0;

double angle0;
double angle[4] = {0.0, 0.0, 0.0, 0.0};
double former_error[4] = {0.0, 0.0, 0.0, 0.0};
double new_error[4] = {0.0, 0.0, 0.0, 0.0};
double error_sum[4] = {0.0, 0.0, 0.0, 0.0};
double error_sum2[4] = {0.0, 0.0, 0.0, 0.0};

double average_angle;
int speed_error[4] = {0, 0, 0, 0};
double absolute_angle[4] = {0.0, 0.0, 0.0, 0.0};
bool first_loop = true;
bool motor_dir = true;

double anglePID_four(int Id, double Motor_angle, double dt)
{
  new_error[Id] = Motor_angle - average_angle;
  error_sum[Id] += new_error[Id];
  double v_command = (k_p * (new_error[Id] + error_sum[Id] * dt / k_i));
  return v_command;
}

double anglePID_two(int Id, double Motor_angle, double dt, double avg_angle)
{
  new_error[Id] = Motor_angle - avg_angle;
  error_sum2[Id] += new_error[Id];
  double v_command = (k_p2 * (new_error[Id] + error_sum2[Id] * dt / k_i2));
  return v_command;
}

void loop()
{

  VValue = analogRead(VEncoder);
  HValue = analogRead(HEncoder);
  filterV.Filter(VValue);
  filterH.Filter(HValue);
  VValue = filterV.Current();
  HValue = filterH.Current();
  dataSize = udp.parsePacket();

  if (dataSize)
  {
    // read the packet into packetBufffer
    int len = udp.read(motorsData, 64);

    if (len > 0)
    {
      motorsData[len] = 0;
      pos1 = 0;
      pos2 = 0;
      j = 0;
      // Serial.println(motorsData);

      for (int i = 0; i < len; ++i)
      {
        if (motorsData[i] == ',' && f)
        {
          memset(num1, 0, sizeof(num1));
          strncpy(num1, motorsData + pos1, i - pos1);
          k = (int16_t)atoi(num1) << 8;
          pos2 = i + 1;
          f = false;
          s = true;
        }
        else if (motorsData[i] == ',' && s)
        {
          memset(num2, 0, sizeof(num2));
          strncpy(num2, motorsData + pos2, i - pos2);
          k |= (int16_t)atoi(num2);
          //****************absolute angle******************//
          angle0 = (double)k / 16384.0 * 360.0;
          if (first_loop)
          {
            angle[j] = angle0;
            absolute_angle[j] = angle0;
          }
          else
          {
            if (abs(angle0 - angle[j]) < 180.0)
            {
              if (angle0 - angle[j] > 0)
              {
                absolute_angle[j] += (angle0 - angle[j]);
              }
              else
              {
                absolute_angle[j] += (angle0 - angle[j]);
              }
            }
            else
            {
              if (angle0 < angle[j])
              {
                absolute_angle[j] += 360.0 - (angle[j] - angle0);
              }
              else
              {
                absolute_angle[j] += (angle0 - angle[j]) - 360.0;
              }
            }
            angle[j] = angle0;
          }

          //***************************************//
          pos1 = i + 1;
          f = true;
          s = false;
          if (j == 3)
            break;
          j++;
        }
      }
      first_loop = false;
      if (abs(absolute_angle[0]) > 360.0 && abs(absolute_angle[1]) > 360.0 \
          && abs(absolute_angle[2]) > 360.0 && abs(absolute_angle[3]) > 360.0)
      {
        for (int m = 0; m < 4; ++m)
        {
          if (absolute_angle[m] > 0.0)
            absolute_angle[m] -= 360.0;
          else
            absolute_angle[m] += 360.0;
        }
      }
    }

    if (sendPCinterval % 4 == 0)
    {
        udp.beginPacket(sendPCAddress, sendPCPort);
        udp.printf("%s",motorsData);
        udp.endPacket();
        sendPCinterval = 0;
    }
    sendPCinterval++;
  }

  // stay
  if (sendRPiinterval % 30 == 0)
  {
    if (abs(HValue - H_MID) < error && abs(VValue - V_MID) < error)
    {
      motorSpeed = 0x2000;
      udp.beginPacket(sendRPiAddress, sendRPiPort);
      udp.printf("%u,%u,%u,%u,", motorSpeed, motorSpeed, motorSpeed, motorSpeed);
      udp.endPacket();
      for (int m = 0; m < 4; ++m)
      {
        absolute_angle[m] = angle[m];
      }
    }
    else
    {
      time_delta = micros() - time_previous;
      time_previous = micros();

      // 90 degree
      if (abs(HValue - H_MID) < error && abs(VValue - V_MID) > error)
      {
        double per_error_2;
        double average_angle1 = (absolute_angle[0] + absolute_angle[1]) / 2.0;
        double average_angle2 = (absolute_angle[2] + absolute_angle[3]) / 2.0;
        if (VValue > V_MID)
        {
          motorSpeed = (int)((double)(VValue - V_MID) / (double)(V_HIGH - V_MID) * 0x2000) + 0x2000;
          for (int Id = 0; Id < 4; ++Id)
          {
            if (Id < 2)
            {
              per_error_2 = constrain(anglePID_two(Id, absolute_angle[Id], time_delta, average_angle1), -0.4, 0.4);
              if (per_error_2 < 0.0)
                speed_error[Id] = (int)(per_error_2 * (0x4000 - motorSpeed));
              if (per_error_2 >= 0.0)
                speed_error[Id] = (int)(per_error_2 * (motorSpeed - 0x2000));
            }
            else
            {
              per_error_2 = constrain(anglePID_two(Id, absolute_angle[Id], time_delta, average_angle2), -0.4, 0.4);
              if (per_error_2 < 0.0)
                speed_error[Id] = (int)(per_error_2 * (motorSpeed - 0x2000));
              if (per_error_2 >= 0.0)
                speed_error[Id] = (int)(per_error_2 * (0x4000 - motorSpeed));
            }
          }
        }
        else
        {
          motorSpeed = (int)((double)(VValue - V_MID) / (double)(V_MID - V_LOW) * 0x2000) + 0x2000;
          for (int Id = 0; Id < 4; ++Id)
          {
            if (Id < 2)
            {
              per_error_2 = constrain(anglePID_two(Id, absolute_angle[Id], time_delta, average_angle1), -0.4, 0.4);
              if (per_error_2 < 0.0)
                speed_error[Id] = (int)(per_error_2 * motorSpeed);
              if (per_error_2 >= 0.0)
                speed_error[Id] = (int)(per_error_2 * (0x2000 - motorSpeed));
            }
            else
            {
              per_error_2 = constrain(anglePID_two(Id, absolute_angle[Id], time_delta, average_angle2), -0.4, 0.4);
              if (per_error_2 < 0.0)
                speed_error[Id] = (int)(per_error_2 * (0x2000 - motorSpeed));
              if (per_error_2 >= 0.0)
                speed_error[Id] = (int)(per_error_2 * motorSpeed);
            }
          }
        }
        udp.beginPacket(sendRPiAddress, sendRPiPort);
        udp.printf("%u,%u,%u,%u,", motorSpeed - speed_error[0], motorSpeed - speed_error[1], \
                   0x4000 - motorSpeed - speed_error[2], 0x4000 - motorSpeed - speed_error[3]);
        udp.endPacket();
      }

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
            per_error_4 = constrain(anglePID_four(Id, absolute_angle[Id], time_delta), -0.4, 0.4);
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
            per_error_4 = constrain(anglePID_four(Id, absolute_angle[Id], time_delta), -0.4, 0.4);
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
      else if (HValue < H_MID && VValue > V_MID)
      {

      }
      // 225 degree
      else if (HValue < H_MID && VValue < V_MID)
      {

      }
      // 315 degree
      else if (HValue > H_MID && VValue < V_MID)
      {

      }
      // 45 degree
      else if (HValue > H_MID && VValue > V_MID)
      {

      }
    }
    sendRPiinterval = 0;
  }
  sendRPiinterval++;
}

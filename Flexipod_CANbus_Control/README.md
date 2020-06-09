# CAN bus motor Control

## Hardware

Brushless DC motor: DJI M2006 P36
Electronic Speed Controller: DJI C610
Controller: Teensy 4.0
CAN Transceiver: CJMCU-2551 MCP2551
Voltage Stepdown Regulator: Pololu 5V, 5A Step-Down Voltage Regulator D24V50F5
IMU: MPU9255
Data transport: Rapsberry Pi 4 Model B and PC

## Software
### Teensy
Two libraries are used in Teensy program: FlexCAN_T4 and MPU9250.For more detail of these two libraries, please check the following repositories:
https://github.com/tonton81/FlexCAN_T4
https://github.com/bolderflight/MPU9250

Include this header to use FlexCAN_T4:

`#include <FlexCAN_T4.h>`

To use MPU9250 library, download MPU9250.h and MPU9250.cpp from the [repository](https://github.com/bolderflight/MPU9250)

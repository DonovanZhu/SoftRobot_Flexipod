#include <math.h>       /* atan2 */

const int PWM_LOW = 2016;
const int PWM_HIGH = 4032;
const int PWM_MID = (PWM_LOW + PWM_HIGH) / 2;

union BytesToInt16
{
  byte bytes[2];
  int16_t integer;
};

class MotorM2006
{
    /*
       pwm using oneshot42 42us-84us
       analogwrite resolutoin:12bit (4096)
       min: 42us-> 2016 , max: 84 us-> 4032
    */

    static constexpr double RPM_TO_RAD_PER_S = TWO_PI / 60.0;
  public:
    double valSin, valCos, angle, angleDelta, angle0;

    // the angle of the motor end measured on last loop
    double angleLastLoop;

    double angleShaft;
    // the angle of the shaft end measured on last RPM loop
    double angleShaftLastRPMLoop;

    // the desired_speed input from the last RPM loop
    double desiredSpeedLastRPMloop = 0;
    // the desired angle shaft position from the last RPM Loop
    double desiredPositionLastRPMLoop = 0;
    double speedCorrection;

    double angleShaftDesired = 0;

    double positionError;
    // simple an running sum (numerical integration) of the desired RPM * dt
    double desiredRPMPosition = 0;

    // measured speed of the shaft revolution per minute [rpm], calculated from encoder
    // in the SpeedLoop()
    double measuredSpeed;

    /****************/
    double valMax = 137;
    double valMin = 17;
    /****************/

    //
    //    int valMax = 0;
    //    int valMin = pow(2, 8);

    double valMid = (valMax + valMin) / 2;
    long nEncoder = 0; // this will overflow after 21304 hours with 400rpm
    bool ccw = false;

    int pwmVal = 3024;
    int pwmPin, encoderPin1, encoderPin2;

    MotorM2006(int pwm_pin, int encoder_pin1, int encoder_pin2)
    {
      this->pwmPin = pwm_pin;
      this-> encoderPin1 = encoder_pin1;
      this->encoderPin2 = encoder_pin2;
    }

    void Arm()
    {
      analogReadAveraging(2);
      analogWriteResolution(12);
      analogWriteFrequency(this->pwmPin, 11718.75);// Oneshot42 PWM frequency
      analogWrite(this->pwmPin, 3000);

      analogReadRes(8);// use default 8 bit for faster read rate
      this->valSin = analogRead(this->encoderPin1);  // read the input pin
      this->valCos = analogRead(this->encoderPin2);  // read the input pin
      // initial angle !![rad]!!
      this->angle0 = atan2 (this->valSin - this->valMid, this->valCos - this->valMid);
      this->angle = 0;
    }

    inline void SetPWM(int pwm_val)
    {
      analogWrite(this->pwmPin, pwm_val);
      this->pwmVal = pwm_val;
    }

    void TestRange()
    {
      /* For debugging the analog input range
         Rotate the motor to see the max and min input
      */
      this->valSin = analogRead(this->encoderPin1);  // read the input pin
      this->valCos = analogRead(this->encoderPin2);  // read the input pin
      this->valMax = max(this->valMax, max(this->valSin, this->valCos));
      this->valMin = min(this->valMin, min(this->valSin, this->valCos));
      Serial.print(this->valMin);
      Serial.print(',');
      Serial.print(this->valMax);
      Serial.print(',');
      Serial.print(this->valSin);          // debug value
      Serial.print(',');
      Serial.println(this->valCos);          // debug value
    }

    double pwmValZeroBalanced = 0.0; //pwmValZeroBalanced = pwmVal - PWM_MID
    double errorSpeed = 0.0;
    double errorSpeedPrevious = 0.0;
    /*Speed closed loop P-I control
       input:
            dt: time_delta (long)
            desired_speed: between -400~400 [rpm] (double)
       output:
            measuredSpeed: shaft speed calculated from encoder [rpm] (double)
    */
    static constexpr double k_p = 400.0;
    static constexpr double k_i = 800.0;


    //    void PositionLoop(long dt, double desired_speed){
    //      this->angleShaftDesired += desired_speed/60.0 * TWO_PI /1000000.0* double(dt);
    //
    //
    //      double position_error = this->angleShaftDesired - this->angleShaftLastRPMLoop;
    //
    //      double
    //
    //    }

    void SpeedLoop(long dt, double desired_speed)
    {
      // for position close-loop
      //      this->desiredRPMPosition = this->desiredRPMPosition + desired_speed * double(dt);
      this->desiredPositionLastRPMLoop = this->desiredPositionLastRPMLoop + \
                                         this->desiredSpeedLastRPMloop * TWO_PI / 60.0 * double(dt) / 1000000.0;
      this->positionError = this->desiredPositionLastRPMLoop - this->angleShaft;
      // if the error is too large, give up the larger chuncks
      if (this->positionError > TWO_PI)
      {
        this->positionError = this-> positionError - TWO_PI;
        this->desiredPositionLastRPMLoop = this->desiredPositionLastRPMLoop -  TWO_PI;
      }
      else if (this->positionError < (-TWO_PI))
      {
        this->positionError = this-> positionError + TWO_PI;
        this->desiredPositionLastRPMLoop = this->desiredPositionLastRPMLoop +  TWO_PI;
      }

      //      speedCorrection = positionError / double(dt) * 1000000.0 / TWO_PI * 60; //RPM

      // revolution per minute
      this->measuredSpeed = (this->angleShaft - this->angleShaftLastRPMLoop) / double(dt) * (1000000.0 / TWO_PI * 60.0);
      this->angleShaftLastRPMLoop = this->angleShaft;

      //      Serial.print(angleShaft - angleShaftLastRPMLoop);

      if (desired_speed == 0)
      {
        this->SetPWM(PWM_MID);
      }

      else
      {

        /******************************/
        this->errorSpeed = desired_speed  - this->measuredSpeed + positionError * 5;
        /******************************/

        // PI loop
        pwmValZeroBalanced += (k_i * (errorSpeed - errorSpeedPrevious) + k_p * errorSpeed) / double(dt);
        errorSpeedPrevious = errorSpeed;

        SetPWM(constrain(this->pwmValZeroBalanced + PWM_MID, PWM_LOW, PWM_HIGH));
      }

      this->desiredSpeedLastRPMloop = desired_speed;

    }

    /* EncoderLoop() measures the encoder position
       The encoder is a analog sin-cos encoder
       it uses atan2(sin,cos) to find the encoder position
       gear ratio between encoder and the motor =  7:1
       gear ratio between motor and gearbox = 36:1
    */
    void EncoderLoop()
    {
      valSin = analogRead(encoderPin1);  // read the input pin
      valCos = analogRead(encoderPin2);  // read the input pin
      angleLastLoop = angle;
      angle = atan2 (valSin - valMid, valCos - valMid) - angle0;// angle of the encoder !![rad]!!
      angleDelta = angle - angleLastLoop;
      /*if (angleDelta > -TWO_PI) {*/
      if (angleDelta < -PI)
      {
        nEncoder++;
        ccw = true;
      }
      else if (angleDelta < 0)
      {
        ccw = false;
      }
      else if (angleDelta < PI)
      {
        ccw = true;
      }
      else if (angleDelta < TWO_PI)
      {
        nEncoder--;
        ccw = false;
      }
      /*}
        else {
        //ERROR!
        }*/
      // angle of the shaft !![rad]!!
      //(7.0 encoder-motor ratio * 36.0 gear ratio)

      angleShaft = (double(nEncoder) * TWO_PI + angle) / 252.0;

    }
};



#define NUMBER_MOTORS 4
MotorM2006 motors[NUMBER_MOTORS] =
{
  MotorM2006(12, A4, A5),//motor_back_left
  MotorM2006(11, A3, A2),//motor_front_left
  MotorM2006(10, A1, A0),//motor_back_right
  MotorM2006(9, A8, A9)//motor_front_right
};

unsigned long time_previous;
unsigned long time_now;
unsigned long time_delta;

void setup()
{
  for (int k = 0; k < NUMBER_MOTORS; k++)
  {
    motors[k].Arm();
  }
  delay(3000);
  // put your setup code here, to run once:
  Serial.begin(2000000);           //  setup serial
  // while (! Serial);
  //sets the maximum milliseconds to wait for serial data
  Serial.setTimeout(1);
  Serial.flush();
  // remote control from serial5
  time_previous = micros();
}

//double desired_speed = 0;


double SPEED_HIGH = 400; //rpm
double SPEED_LOW = -400; //rpm

double speed_cmd_arr[NUMBER_MOTORS]; // -400 ~ +400

/**************************************/
const int16_t D_SPEED_HIGH = 16384; // rpm
const int16_t D_SPEED_LOW = 0; // rpm
const int16_t D_POSITION_HIGH = 16384;
const int16_t D_POSITION_LOW = 0;
/**************************************/

// digitized speed and position information
int16_t d_speed_cmd[NUMBER_MOTORS]; // D_SPEED_LOW ~ D_SPEED_HIGH //rpm
int16_t d_speed_measured[NUMBER_MOTORS]; // D_SPEED_LOW ~ D_SPEED_HIGH //rpm
int16_t d_position_measured[NUMBER_MOTORS]; // D_POSITION_LOW ~ D_POSITION_HIGH

/*constrain the angle to 0-2PI [rad] then to D_POSITION_LOW~D_POSITION_HIGH*/
inline int16_t ConstrainAngle(double x)
{
  x = fmod(x, TWO_PI);
  if (x < 0)
    x += TWO_PI;
  return (int16_t)round(map(x, 0, TWO_PI, D_POSITION_LOW, D_POSITION_HIGH));
}

// byte message[26];
byte message[24];
const int16_t MESSAGE_START_TOKEN = 1;
int issend = 0;
byte input_message[10];

void loop()
{

  // delay(1000);

  time_delta = micros() - time_previous;
  if (time_delta >= 2500)
  {
    time_previous = micros();

    // Serial.print(time_delta);
    // Serial.print("\t");

    // Serial.print('\n');
    // Serial.println(Serial.available());
    if (Serial.available())
    { // Check receive buffer.
      Serial.readBytes(input_message, 10);
      // Serial.write(input_message, 10);
      // -32767
      if ((input_message[0] == 0x80) && (input_message[1] == 0x01))
      {
        for (int i = 0; i < 4; i++)
        {
          d_speed_cmd[i] = (int16_t) input_message[2 + 2 * i] << 8;
          d_speed_cmd[i] |= (int16_t) input_message[3 + 2 * i];
          // Serial.print(d_speed_cmd[i]);
          // Serial.print(',');

          speed_cmd_arr[i] = map( d_speed_cmd[i], D_SPEED_LOW, D_SPEED_HIGH, SPEED_LOW, SPEED_HIGH); //rpm
          // Serial.println(speed_cmd_arr[i]);
        }
        // Serial.println();
      }
    }

    for (int k = 0; k < NUMBER_MOTORS; k++)
    {
      motors[k].SpeedLoop(time_delta, speed_cmd_arr[k]);
      //Serial.println(time_delta);
    }

    // to serial message
    for (int k = 0; k < 4; k++)
    {
      // D_SPEED_LOW ~ D_SPEED_HIGH
      d_speed_cmd[k] = (int16_t)map(speed_cmd_arr[k], SPEED_LOW, SPEED_HIGH, D_SPEED_LOW, D_SPEED_HIGH);
      // D_SPEED_LOW ~ D_SPEED_HIGH
      d_speed_measured[k] = (int16_t)map(motors[k].measuredSpeed, SPEED_LOW, SPEED_HIGH, D_SPEED_LOW, D_SPEED_HIGH);
      // D_POSITION_LOW ~ D_POSITION_HIGH
      d_position_measured[k] = ConstrainAngle(motors[k].angleShaft);
    }
    /*
    message[0] = highByte(MESSAGE_START_TOKEN);
    message[1] = lowByte(MESSAGE_START_TOKEN);
    for (uint8_t i = 0; i < 4; i++)
    {
      message[i * 2 + 2] = highByte(d_speed_measured[i]);
      message[i * 2 + 3] = lowByte(d_speed_measured[i]);

      message[i * 2 + 10] = highByte(d_position_measured[i]);
      message[i * 2 + 11] = lowByte(d_position_measured[i]);

      message[i * 2 + 18] = highByte(d_speed_cmd[i]);
      message[i * 2 + 19] = lowByte(d_speed_cmd[i]);
    }
    */
    if (issend % 12 ==0)
    {
      Serial.println(String(((double)d_speed_measured[0] - 8192.0) / 8192.0 * 400.0) + \
      " " + String(((double)d_speed_measured[1] - 8192.0) / 8192.0 * 400.0) + \
      " " + String(((double)d_speed_measured[2] - 8192.0) / 8192.0 * 400.0) + \
      " " + String(((double)d_speed_measured[3] - 8192.0) / 8192.0 * 400.0) + \
      " " + String((double)d_position_measured[0] / 16384.0 * 360.0) + \
      " " + String((double)d_position_measured[1] / 16384.0 * 360.0) + \
      " " + String((double)d_position_measured[2] / 16384.0 * 360.0) + \
      " " + String((double)d_position_measured[3] / 16384.0 * 360.0));
      issend = 0;
    }
    issend++;
    // byte mes[2];
    // mes[0] = message[2];
    // mes[1] = message[3];
    // Serial.println((int16_t)message[0]);
    // Serial.write(mes, sizeof(mes));
    //Serial.write(message, sizeof(message));
    // Serial.println(Serial.baud());
    Serial.flush();
  }

  // motor_front_left.TestRange();

  for (int k = 0; k < 4; k++)
  {
    motors[k].EncoderLoop();
  }
}

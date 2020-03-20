int led = 13;
int ledMode;
char incoming[32];
float data;

void setup()
{
  pinMode(led, OUTPUT);
  Serial.begin(9600); // USB is always 12 Mbit/sec
  /*
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
    }
  */
} 

void loop()
{
  while (Serial.available() == 0);
  int i = 0;

  while (Serial.available() > 0)
  {
    incoming[i] = Serial.read();
    i++;
  }
  //ledMode = digitalRead(led);
  data = atof(incoming);

  // Serial.println(incoming);

  Serial.println(incoming);
  if (data >= 5.0)
  {
    // Serial.println("Led is off");
    // delay(1000);               // wait for a second
    digitalWrite(led, LOW);   // turn the LED on (HIGH is the voltage level)
  }
  else if (data < 5.0)
  {
    // Serial.println("Led is on");
    // delay(1000);               // wait for a second
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  }

}

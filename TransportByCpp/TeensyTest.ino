//only work when opening Serial Monitor
byte mes[2] = {0x01, 0x02};

void setup()
{
  delay(3000);
  // put your setup code here, to run once:
  Serial.begin(2000000);           //  setup serial
  // while (! Serial);
  //sets the maximum milliseconds to wait for serial data
  Serial.setTimeout(1);
  Serial.flush();

}
void loop()
{
  Serial.write(mes, sizeof(mes));
  Serial.flush();
  delay(200);
}

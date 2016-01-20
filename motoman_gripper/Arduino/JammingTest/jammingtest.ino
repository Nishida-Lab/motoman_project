int switch_pin = 8;

void setup()
{
  pinMode(switch_pin, OUTPUT);
  digitalWrite(switch_pin, LOW);
  delay(2000);
}

void loop()
{
  digitalWrite(switch_pin,HIGH); // switch ON
  delay(5000);
  digitalWrite(switch_pin,LOW); // switch OFF
  delay(5000);
}

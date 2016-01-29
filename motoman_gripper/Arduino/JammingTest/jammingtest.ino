int switch_pin = 8;

void setup()
{
  pinMode(switch_pin, OUTPUT);
  digitalWrite(switch_pin, LOW);
}

void loop()
{
  digitalWrite(switch_pin, HIGH); // switch on
  delay(5000);
  digitalWrite(switch_pin, LOW); // switch off
  delay(5000);
}

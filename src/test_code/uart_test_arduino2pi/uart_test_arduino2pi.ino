const int buttonPin = 13;
bool buttonState = LOW;

void setup()
{
  pinMode(buttonPin, INPUT);
  Serial.begin(115200);
}

void loop()
{
  buttonState = digitalRead(buttonPin);

  Serial.write(buttonState);

  if (Serial.available() > 0) {
    char response = Serial.read();
    Serial.print("Received: ");
    Serial.println(response);
  }

  delay(500);
}

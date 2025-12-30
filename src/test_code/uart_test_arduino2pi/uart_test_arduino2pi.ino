const int ledPin = 10;
const int buttonPin = 13;
bool buttonState = LOW;

void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  Serial.begin(115200);

  digitalWrite(ledPin, LOW);
}

void loop()
{
  buttonState = digitalRead(buttonPin);

  Serial.write(buttonState ? '1' : '0');

  if (Serial.available() > 0) {
    char response = Serial.read();
    if (response == '0') {
      digitalWrite(ledPin, LOW);
    }
    else if (response == '1') {
      digitalWrite(ledPin, HIGH);
    }
  }

  delay(5);
}

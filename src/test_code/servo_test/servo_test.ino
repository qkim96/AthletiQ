#include <Servo.h>

const int servoPin = A5;
Servo testServo;

void setup()
{
  testServo.attach(servoPin);
  Serial.begin(9600);
}

void loop()
{
  for (int pos = 0; pos <= 180; pos += 10) {
    testServo.write(pos);
    Serial.print("Servo position: ");
    Serial.println(pos);
    delay(500);
  }

  for (int pos = 180; pos >= 0; pos -= 10) {
    testServo.write(pos);
    Serial.print("Servo position: ");
    Serial.println(pos);
    delay(500);
  }
}

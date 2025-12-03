const int segPins[8] = {2, 3, 4, 5, 6, 7, 8, 9};  // A ~ DP
const int digitPins[5] = {A0, A1, A2, A3, A4};    // Digit 1 ~ 5

const byte alphabets[10] =
{
  B01110001,  // F
  B01010000,  // r
  B01011100,  // o
  B01010100,  // n
  B01111000,  // t

  B01101101,  // S
  B00000110,  // i
  B01011110,  // d
  B01111001,  // E
  B00000000   // blank
};

void setup()
{
  for (int i = 0; i < 8; i++) {
    pinMode(segPins[i], OUTPUT);    // initialize pins as outputs
    digitalWrite(segPins[i], LOW);  // turn off all pins
  }
}

void loop()
{
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 8; j++) {
      digitalWrite(segPins[j], bitRead(alphabets[i], j));
    }
    delay(1000);
  }
}

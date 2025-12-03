const int segPins[8] = {2, 3, 4, 5, 6, 7, 8, 9};  // A-DP pins
const int digitPins[5] = {A0, A1, A2, A3, A4};    // Digit 1-5 pins

const int freq = 100;                             // Multiplex refresh rate (Hz)
const int multiplex_delay = 1000 / (freq * 5);    // Delay for each digit's display "on" time (ms)
unsigned long currentTime = 0;                    // Current time in ms
unsigned long lastSwitchTime = 0;                 // Last time when words were switched
bool showFront = true;                            // Which word to show

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
    pinMode(segPins[i], OUTPUT);      // initialize segment pins as outputs
    digitalWrite(segPins[i], LOW);    // turn off all segment pins
  }

  for (int j = 0; j < 5; j++) {
    pinMode(digitPins[j], OUTPUT);    // initialize digit pins as outputs
    digitalWrite(digitPins[j], LOW);  // turn off all digit pins
  }
}

void loop()
{
  currentTime = millis();

  // Switch words (Front <-> Side) every 1 second
  if (currentTime - lastSwitchTime >= 1000) {
    showFront = !showFront;
    lastSwitchTime = currentTime;
  }

  // Multiplex display
  for (int d = 0; d < 5; d++) {
    // Select current alphabet to display
    byte currAlph = showFront ? alphabets[d] : alphabets[d+5];

    // Turn on the current digit
    digitalWrite(digitPins[d], HIGH);

    // Set segments to display selected alphabet
    for (int s = 0; s < 8; s++) digitalWrite(segPins[s], bitRead(currAlph, s));

    // Turn off the current digit
    delay(multiplex_delay);  // delay for the display "on" time for each digit to visibly flicker (100Hz)
    digitalWrite(digitPins[d], LOW);
  }
}

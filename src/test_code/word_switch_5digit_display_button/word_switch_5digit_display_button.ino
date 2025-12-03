// ------------------------------------------------------------------------------------------------------------
// VARIABLES
// ------------------------------------------------------------------------------------------------------------
const int segPins[8] = {2, 3, 4, 5, 6, 7, 8, 9};  // A-DP pins
const int digitPins[5] = {A0, A1, A2, A3, A4};    // Digit 1-5 pins
const int buttonPin = 13;                         // Button pin

const int freq = 100;                             // Multiplex refresh rate (Hz)
const int multiplex_delay = 1000 / (freq * 5);    // Delay for each digit's display "on" time (ms)

const unsigned int debounceDelay = 50;            // How long the input must remain stable before we "trust" it
unsigned long lastDebounceT = 0;                  // Last time the input has changed

bool prevReading = LOW;                           // Actual voltage reading (previous)
bool currReading = LOW;                           // Actual voltage reading (current)
bool confirmedBState = LOW;                       // Confirmed button state (pressed / not pressed)
bool isFront = true;                              // Boolean to switch the word to display

const byte alphabets[10] =                        // Byte data for each 7-segment alphabets (order of DP-A)
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


// ------------------------------------------------------------------------------------------------------------
// Button Debounce Function:
// returns TRUE only on a confirmed button press/release
// ------------------------------------------------------------------------------------------------------------
bool buttonPressed()
{
  currReading = digitalRead(buttonPin);

  // If voltage reading is changed (including bouncing),
  if (currReading != prevReading) {
    lastDebounceT = millis();        // reset the debounce timer.
  }

  // If voltage is stable enough (over delay threshold),
  if ((millis() - lastDebounceT) > debounceDelay) {

    if (currReading != confirmedBState) {  // check change in button state,
      confirmedBState = currReading;       // then finally update button state.

      // Detect confirmed button "press" (HIGH) / "release" (LOW)
      if (confirmedBState == HIGH) {
        prevReading = currReading;
        return true;
      }
    }
  }

  // Button press not detected
  prevReading = currReading;
  return false;
}


// ------------------------------------------------------------------------------------------------------------
// Multiplex Display Function:
// display a word (front/side)
// ------------------------------------------------------------------------------------------------------------
void displayWord(bool showFront)
{
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


// ------------------------------------------------------------------------------------------------------------
// Setup
// ------------------------------------------------------------------------------------------------------------
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

  pinMode(buttonPin, INPUT);          // initialize button pin as input
}


// ------------------------------------------------------------------------------------------------------------
// Main Loop
// ------------------------------------------------------------------------------------------------------------
void loop()
{
  if (buttonPressed()) isFront = !isFront;
  displayWord(isFront);
}

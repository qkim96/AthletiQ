#include <Servo.h>

// ------------------------------------------------------------------------------------------------------------
// VARIABLES
// ------------------------------------------------------------------------------------------------------------

// --- Global Time ---
unsigned long currentTime = 0;

// --- Pins ---
const int segPins[8] = {7, 3, A3, A1, A0, 6, A4, A2};  // A-DP pins
const int digitPins[5] = {8, 5, 4, A5, 2};             // Digit 1-5 pins
const int servoPin = 9;                                // Servo control pin
const int buttonPin = 13;                              // Button pin

// --- Button Debouncing Variables ---
const int debounceDelay = 50;                          // Time the input must remain stable to "trust" it
unsigned long lastDebounceTime = 0;                    // Last time the input has been changed

// --- Multiplex Display Variables ---
const int freq = 100;                                  // Multiplex refresh rate (Hz)
const int multiplexDelay = 1000 / (freq * 5);          // Delay for each digit's display "on" time (ms)
unsigned long lastDisplayTime = 0;                     // Last time when all the digits were displayed
int currentDigit = 0;                                  // Current digit to display (0-4)

// --- Servo Control Variables ---
Servo camServo;                                        // Servo object
unsigned int servoPos = 90;                            // Current servo position (initially 90 degrees)
const int stepDelay = 10;                              // Delay between servo steps
const int stepSize = 3;                                // Degrees to rotate per step
unsigned long lastServoStepTime = 0;                   // Last time when the servo was rotated

// --- Booleans ---
bool prevReading = LOW;                                // Actual voltage reading (previous)
bool currReading = LOW;                                // Actual voltage reading (current)
bool confirmedBState = LOW;                            // Confirmed button state (pressed / not pressed)
bool isFront = false;                                  // Boolean to switch the word to display
bool wifiConnected = false;                            // Boolean to indicate WiFi connection

// --- Display Alphabet Bytes ---
const byte alphabets[15] =                             // Byte data for each 7-segment alphabets
{                                                      // (bit order: DP-A)
  B10001110,  // F
  B10101111,  // r
  B10100011,  // o
  B10101011,  // n
  B10000111,  // t

  B10010010,  // S
  B11111001,  // i
  B10100001,  // d
  B10000110,  // E
  B11111111,  // blank

  B01111111,  // .
  B01111111,  // .
  B01111111,  // .
  B01111111,  // .
  B01111111   // .
};


// ------------------------------------------------------------------------------------------------------------
// UART RX Function:
// return the most recent byte received via UART
// ------------------------------------------------------------------------------------------------------------
char receiveData() {
  int byte_num = Serial.available();
  char rxData;

  for (int i = 0; i < byte_num; i++) {
    rxData = Serial.read();
  }

  return rxData;
}


// ------------------------------------------------------------------------------------------------------------
// Button Debounce Function:
// returns TRUE only on a confirmed button press/release
// ------------------------------------------------------------------------------------------------------------
bool buttonPressed()
{
  currReading = digitalRead(buttonPin);

  // If voltage reading is changed (including bouncing),
  if (currReading != prevReading) {
    lastDebounceTime = currentTime;        // reset the debounce timer.
  }

  // If voltage is stable enough (over delay threshold),
  if ((currentTime - lastDebounceTime) > debounceDelay) {

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
// display a word (front/side), or "....." (dots) when Raspberry Pi is not connected to WiFi
// ------------------------------------------------------------------------------------------------------------
void updateDisplay(bool showFront, bool noWifi=false)
{
  // Delay for the display "on" time for each digit to visibly flicker (100Hz)
  if (currentTime - lastDisplayTime >= multiplexDelay) {
    // Turn off all digits
    for (int d = 0; d < 5; d++) digitalWrite(digitPins[d], LOW);

    // Select a starting byte to choose a word to display
    byte currAlph;
    if (noWifi) {
      // Select a byte to display "....." (dots)
      currAlph = alphabets[currentDigit+10];
    }
    else {
      // Select an alphabet to display, depending on the mode (front/side)
      currAlph = showFront ? alphabets[currentDigit] : alphabets[currentDigit+5];
    }

    // Set segments to display the selected alphabet
    for (int s = 0; s < 8; s++) digitalWrite(segPins[s], bitRead(currAlph, s));

    // Turn on the current digit
    digitalWrite(digitPins[currentDigit], HIGH);

    // Move to next digit
    currentDigit = (currentDigit + 1) % 5;

    // Update last display time
    lastDisplayTime = currentTime;
  }
}


// ------------------------------------------------------------------------------------------------------------
// Servo Control Function:
// rotate servo to the left/right depending on the received serial data from Raspberry Pi
// ------------------------------------------------------------------------------------------------------------
void updateServo(char rotateDir)
{
  // Delay for the servo control to slow down the rotation
  if (currentTime - lastServoStepTime >= stepDelay) {

    // Rotate left
    if (rotateDir == 'l') {
      servoPos -= stepSize;
      if (servoPos < 0) servoPos = 0;
    }

    // Rotate right
    else if (rotateDir == 'r') {
      servoPos += stepSize;
      if (servoPos > 180) servoPos = 180;
    }

    // Update the servo position and the timing variable
    camServo.write(servoPos);
    lastServoStepTime = currentTime;
  }
}


// ------------------------------------------------------------------------------------------------------------


void setup()
{
  Serial.begin(230400);

  pinMode(buttonPin, INPUT_PULLUP);   // initialize button pin as input

  for (int i = 0; i < 8; i++) {
    pinMode(segPins[i], OUTPUT);      // initialize segment pins as outputs
    digitalWrite(segPins[i], LOW);    // turn off all segment pins
  }

  for (int j = 0; j < 5; j++) {
    pinMode(digitPins[j], OUTPUT);    // initialize digit pins as outputs
    digitalWrite(digitPins[j], LOW);  // turn off all digit pins
  }

  camServo.attach(servoPin);          // attach servo pin to the servo
  camServo.write(servoPos);           // set servo to initial position
}



// ------------------------------------------------------------------------------------------------------------


void loop()
{
  // Current time
  currentTime = millis();
  
  // Blocking code to wait for Raspberry Pi to be connected to WiFi
  if (!wifiConnected) {
    updateDisplay(isFront, true);

    // Perform an initial startup handshake
    if (Serial.available() > 0) {
      byte rxByte = receiveData();
      if (rxByte == 0x55) {   // wait for start byte
        delay(2000);
        Serial.write(0xAA);   // send ACK byte
        wifiConnected = true;
      }
    }
    return;
  }

  // Detect a button press
  if (buttonPressed()) {
    isFront = !isFront;
    Serial.write(isFront ? '1' : '0');
  }

  // Update the display
  updateDisplay(isFront);

  // Receive serial data via UART to control servo
  if (Serial.available() > 0) {
    char rxData = receiveData();  // Receive serial data ('l' or 'r')
    updateServo(rxData);          // Update servo position
  }
}

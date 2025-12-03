const int buttonPin = 13;
const int ledPin = 12;

const unsigned int debounceDelay = 50;
unsigned long lastDebounceT = 0;

bool prevReading = LOW;      // actual voltage reading (previous)
bool currReading = LOW;      // actual voltage reading (current)
bool confirmedBState = LOW;  // confirmed button state (pressed / not pressed)
bool ledState = LOW;         // LED state

void setup()
{
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  digitalWrite(ledPin, ledState);
}

void loop()
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
      if (confirmedBState == HIGH) {       // detect confirmed button press (HIGH) / release (LOW)
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
        Serial.println(ledState);
      }
    }
  }

  prevReading = currReading;
}

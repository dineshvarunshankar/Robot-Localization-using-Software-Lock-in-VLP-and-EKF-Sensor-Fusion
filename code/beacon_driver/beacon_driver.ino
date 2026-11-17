// Both beacons use the same modulation signal.

const int PIN_L = 9;
const int PIN_R = 10;
const unsigned long INTERVAL = 35461; // Half-period in microseconds (14.1 Hz)

unsigned long last_time = 0;
int state = LOW;

void setup() {
  pinMode(PIN_L, OUTPUT);
  pinMode(PIN_R, OUTPUT);
}

void loop() {
  unsigned long now = micros();
  
  if (now - last_time >= INTERVAL) {
    state = !state;
    digitalWrite(PIN_L, state);
    digitalWrite(PIN_R, state);
    last_time = now;
  }
}

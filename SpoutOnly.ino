// R2-D2 Brewbot — Spout Only
//
// What it does:
// - Uses an ultrasonic distance sensor (PING/HC-SR04 style) to detect a cup
// - When a cup is close enough, it opens the pour spout (servo) and activates the relay
// - Otherwise, it closes the spout and deactivates the relay
// - Stops pouring automatically after a maximum time
//
// Wiring (adjust pin numbers below if your wiring is different):
// OPTION A — 3-pin ultrasonic (shared TRIGGER/ECHO on one pin):
// - Connect the sensor's SIG to a single Arduino pin.
// - Set TRIGGER_PIN and ECHO_PIN to the SAME pin number below.
//   The code will automatically detect this and switch modes.
//
// OPTION B — 4-pin ultrasonic (separate TRIGGER and ECHO):
// - Ultrasonic TRIGGER -> Arduino pin 2 (TRIGGER_PIN)
// - Ultrasonic ECHO    -> Arduino pin 4 (ECHO_PIN)
// - Servo signal       -> Arduino pin 9 (power the servo from a stable 5V source)
// - Relay signal       -> Arduino pin 13 (module is typically active-LOW)
//
// Customize these values for your setup:
// - POUR_OPEN_ANGLE / POUR_CLOSED_ANGLE: the angles that open/close your spout
// - POUR_DISTANCE_CM: how close the cup must be to trigger pouring (in centimeters)
// - SAMPLE_COUNT / SAMPLE_DELAY_MS: smoothing for distance readings
// - MAX_POUR_TIME_MS: how long to pour (max), then stop until the cup moves away
// - ENABLE_SERIAL_DEBUG: turn serial debug on/off (baud 115200)

#include <Servo.h>

// ############ PIN CONFIGURATION ############
// if using a 3 pin ultrasonic sensor set these to same. the code will handle the rest.
const int TRIGGER_PIN = 2;
const int ECHO_PIN    = 4;

// Servo
const int SERVO_PIN   = 9;
// Relay
const int RELAY_PIN   = 13; // Many relay boards are active-LOW

// ############ END PIN CONFIGURATION ############

// ############ POUR CONTROL ############
const int POUR_OPEN_ANGLE   = 120;
const int POUR_CLOSED_ANGLE = 55;
const int POUR_DISTANCE_CM  = 14;   // Cup is considered "present" when closer than this
const unsigned long MAX_POUR_TIME_MS = 8000; // Max pour duration (8 seconds)

// ############ SAMPLING ############
const int SAMPLE_COUNT     = 3;     // Average this many readings
const int SAMPLE_DELAY_MS  = 25;    // Delay between readings for stability

// ############ END SAMPLING ############

// ############ DEBUG ############
const bool ENABLE_SERIAL_DEBUG = false;      // Set to true to enable debug prints
const unsigned long SERIAL_BAUD_RATE = 115200;

// ############ END DEBUG ############

Servo pourServo;

// Simple pour state
bool isPouring = false;
bool wasNear   = false;             // Tracks if the previous reading was "near" to detect arrival
unsigned long pourStartMs = 0;      // When the current pour started
bool ultrasonicIs3Pin = false;      // Auto-detected in setup

void setup() {
  // Detect 3-pin vs 4-pin by whether TRIGGER and ECHO are the same pin
  ultrasonicIs3Pin = (TRIGGER_PIN == ECHO_PIN);

  if (ultrasonicIs3Pin) {
    // Prepare shared pin (kept LOW) — we'll switch to INPUT only during echo read
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
  } else {
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
  }

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Off by default (active-LOW module)

  pourServo.attach(SERVO_PIN);
  pourServo.write(POUR_CLOSED_ANGLE); // Start closed

  if (ENABLE_SERIAL_DEBUG) {
    Serial.begin(SERIAL_BAUD_RATE);
    if (ultrasonicIs3Pin) {
      Serial.print("Ultrasonic mode: 3-pin on D");
      Serial.println(TRIGGER_PIN);
    } else {
      Serial.print("Ultrasonic mode: 4-pin TRIG=D");
      Serial.print(TRIGGER_PIN);
      Serial.print(" ECHO=D");
      Serial.println(ECHO_PIN);
    }
  }
}

void loop() {
  int distanceCm = readAverageDistanceCm();
  bool pourNow = shouldPour(distanceCm);

  if (pourNow) {
    pourOn();
  } else {
    pourOff();
  }

  if (ENABLE_SERIAL_DEBUG) Serial.println(distanceCm);

  delay(50);
}

// --- Helpers -----------------------------------------------------------------

// Decide whether we should be pouring on this cycle.
// Handles: arrival edge detection, max pour time, and re-arm when cup leaves.
bool shouldPour(int distanceCm) {
  bool isNear = distanceCm < POUR_DISTANCE_CM;

  if (!isPouring) {
    // Start pouring only when the cup arrives (far -> near transition)
    if (isNear && !wasNear) {
      isPouring = true;
      pourStartMs = millis();
      if (ENABLE_SERIAL_DEBUG) Serial.println("Pour start");
    }
  } else {
    // While pouring, stop if time exceeded or cup moved away
    unsigned long elapsed = millis() - pourStartMs;
    if (elapsed >= MAX_POUR_TIME_MS || !isNear) {
      isPouring = false;
      if (ENABLE_SERIAL_DEBUG) Serial.println("Pour stop");
    }
  }

  wasNear = isNear;
  return isPouring;
}

void pourOn() {
  digitalWrite(RELAY_PIN, LOW);        // Active-LOW: engage relay
  pourServo.write(POUR_OPEN_ANGLE);    // Open spout
}

void pourOff() {
  digitalWrite(RELAY_PIN, HIGH);       // Active-LOW: disengage relay
  pourServo.write(POUR_CLOSED_ANGLE);  // Close spout
}

int readAverageDistanceCm() {
  long sum = 0;
  int count = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int d = readDistanceOnceCm();
    if (d > 0) { // Ignore timeouts/noise
      sum += d;
      count++;
    }
    delay(SAMPLE_DELAY_MS);
  }

  if (count == 0) {
    return 999; // No valid reading: treat as "far away"
  }

  return (int)(sum / count);
}

int readDistanceOnceCm() {
  if (ultrasonicIs3Pin) {
    // 3-pin: TRIGGER and ECHO share one pin. We drive a trigger pulse as OUTPUT,
    // then switch to INPUT to listen for the echo pulse.
    digitalWrite(TRIGGER_PIN, LOW);
    pinMode(TRIGGER_PIN, OUTPUT);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    // Listen for echo
    pinMode(TRIGGER_PIN, INPUT);
    // Ensure pull-up is disabled
    digitalWrite(TRIGGER_PIN, LOW);

    unsigned long duration = pulseIn(TRIGGER_PIN, HIGH, 30000UL);

    // Return pin to a known idle state (LOW output) for next cycle
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);

    if (duration == 0) {
      return 0;
    }

    return (int)(duration / 29 / 2);
  } else {
    // 4-pin: separate TRIGGER and ECHO
    // Ensure a clean HIGH pulse
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    // Use a timeout (~30ms) to avoid long blocking if no echo
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
    if (duration == 0) {
      return 0; // Timeout: no object detected within range
    }

    // Convert microseconds to centimeters
    // Sound speed ~ 29 microseconds per cm (round-trip), so divide by 29, then by 2
    return (int)(duration / 29 / 2);
  }
}



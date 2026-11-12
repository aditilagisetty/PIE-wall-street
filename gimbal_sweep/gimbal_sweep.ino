/*
 * Gimbal Sweep (two gimbals, one axis each, moving together)
 *
 * Moves both gimbal servos back and forth together so you can find the level angle.
 * Control it from the Serial Monitor: 9600 baud, line ending "Newline".
 *
 * Servos: goBILDA Dual Mode servos, set to SERVO mode (not continuous).
 * Angles are in degrees from the middle. 0 = middle.
 * Every command moves BOTH servos to the same angle.
 *
 * Wiring:
 *   Servo 1 signal -> pin D9
 *   Servo 2 signal -> pin D10
 *   Power -> separate 5-7.4 V battery (not the Arduino 5V pin),
 *            big enough for both servos
 *   Connect the battery ground to Arduino GND
 *
 * Trim: if the two gimbals don't line up (the horns rarely fit at exactly
 * the same spot), add a small offset to one servo:
 *   1 3          servo 1 is always 3 degrees more
 *   2 -2.5       servo 2 is always 2.5 degrees less
 *   1 0          clear servo 1's trim
 *
 * Commands (type one, then press Enter):
 *   s            sweep back and forth until you stop it
 *   r -40 40     set the sweep range
 *   t 1          set the step size (degrees)
 *   d 150        set the pause between steps (ms)
 *   g 10         go to an angle
 *   + or -       move one step
 *   m            mark this angle (works during a sweep too)
 *   l            save this angle as "level" (prints the pulse for each servo)
 *   z            go back to level
 *   p            print settings and marks
 *
 * To stop a sweep, type anything (except m) and press Enter.
 * The servo stays where it stopped. Type z to go back to level.
 */

#include <Servo.h>

const int NUM_SERVOS = 2;
const int SERVO_PINS[NUM_SERVOS] = {9, 10};

// Pulse widths for the servo (500-2500 us covers 300 degrees)
const int   PULSE_MIN_US    = 500;
const int   PULSE_MAX_US    = 2500;
const int   PULSE_CENTER_US = 1500;
const float US_PER_DEG      = (PULSE_MAX_US - PULSE_MIN_US) / 300.0;

// Never go past this angle, so a typo can't crash the gimbal
const float SAFE_LIMIT_DEG = 90.0;

// Sweep settings (you can change these with commands)
float sweepLo   = -30.0;
float sweepHi   = 30.0;
float stepDeg   = 2.0;
unsigned long dwellMs = 150;

Servo servos[NUM_SERVOS];
float trim[NUM_SERVOS] = {0, 0};   // extra offset for each servo
float angle = 0;      // where the servos are now (before trim)
float level = 0;      // saved level angle
const int MAX_MARKS = 8;
int numMarks = 0;
float marks[MAX_MARKS];

int angleToPulse(float deg) {
  return constrain((int)(PULSE_CENTER_US + deg * US_PER_DEG + 0.5), PULSE_MIN_US, PULSE_MAX_US);
}

// The real angle sent to one servo: shared angle + that servo's trim
float servoAngle(int i) {
  return constrain(angle + trim[i], -SAFE_LIMIT_DEG, SAFE_LIMIT_DEG);
}

// Move both servos at the same moment
void setAngle(float deg) {
  angle = constrain(deg, -SAFE_LIMIT_DEG, SAFE_LIMIT_DEG);
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].writeMicroseconds(angleToPulse(servoAngle(i)));
  }
}

void printPos() {
  Serial.print(angle, 1);
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print(',');
    Serial.print(angleToPulse(servoAngle(i)));
  }
  Serial.println();
}

void mark() {
  if (numMarks < MAX_MARKS) marks[numMarks++] = angle;
  Serial.print(F("# MARK @ ")); Serial.print(angle, 1);
  Serial.print(F(" deg (")); Serial.print(angleToPulse(angle)); Serial.println(F(" us)"));
}

void printParams() {
  Serial.println(F("# ---- parameters ----"));
  Serial.print(F("# range: ")); Serial.print(sweepLo, 1); Serial.print(F(" .. ")); Serial.println(sweepHi, 1);
  Serial.print(F("# step: ")); Serial.print(stepDeg, 1); Serial.print(F(" deg  dwell: ")); Serial.print(dwellMs); Serial.println(F(" ms"));
  Serial.print(F("# trim: servo1=")); Serial.print(trim[0], 1);
  Serial.print(F("  servo2=")); Serial.println(trim[1], 1);
  Serial.print(F("# now=")); Serial.print(angle, 1);
  Serial.print(F(" level=")); Serial.print(level, 1);
  Serial.print(F(" marks:"));
  for (int k = 0; k < numMarks; k++) { Serial.print(' '); Serial.print(marks[k], 1); }
  Serial.println();
}

// Print the level pulse for each servo (trim included)
void printLevel() {
  Serial.print(F("# LEVEL = ")); Serial.print(level, 1); Serial.println(F(" deg"));
  for (int i = 0; i < NUM_SERVOS; i++) {
    float a = constrain(level + trim[i], -SAFE_LIMIT_DEG, SAFE_LIMIT_DEG);
    Serial.print(F("#   servo ")); Serial.print(i + 1);
    Serial.print(F(" (pin ")); Serial.print(SERVO_PINS[i]); Serial.print(F("): "));
    Serial.print(angleToPulse(a)); Serial.println(F(" us  <- put this in your gimbal code"));
  }
}

// During a sweep: "m" marks and keeps going, anything else stops.
bool checkInterrupt() {
  if (!Serial.available()) return true;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line == "m") { mark(); return true; }
  Serial.println(F("# sweep stopped"));
  return false;
}

// Move step by step to the target angle. Returns false if stopped.
bool sweepTo(float target) {
  float dir = (target >= angle) ? 1 : -1;
  while ((target - angle) * dir > 0.001) {
    float next = angle + dir * stepDeg;
    if ((target - next) * dir < 0) next = target;
    setAngle(next);
    printPos();
    unsigned long t0 = millis();
    while (millis() - t0 < dwellMs) {
      if (!checkInterrupt()) return false;
    }
  }
  return true;
}

// Sweep back and forth forever, until you type something (other than "m").
void runSweep() {
  Serial.println(F("# sweeping - type anything + Enter to stop"));
  Serial.println(F("angle_deg,servo1_us,servo2_us"));
  while (true) {
    if (!sweepTo(sweepLo)) return;
    if (!sweepTo(sweepHi)) return;
  }
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  char c = line.charAt(0);
  String arg = line.substring(1);
  arg.trim();

  switch (c) {
    case 's': runSweep(); break;
    case 'r': {
      int sp = arg.indexOf(' ');
      if (sp < 0) { Serial.println(F("# usage: r <lo> <hi>")); break; }
      float lo = arg.substring(0, sp).toFloat();
      float hi = arg.substring(sp + 1).toFloat();
      if (lo > hi) { float tmp = lo; lo = hi; hi = tmp; }
      sweepLo = constrain(lo, -SAFE_LIMIT_DEG, SAFE_LIMIT_DEG);
      sweepHi = constrain(hi, -SAFE_LIMIT_DEG, SAFE_LIMIT_DEG);
      printParams();
      break;
    }
    case 't': { float v = arg.toFloat(); if (v > 0) stepDeg = v; printParams(); break; }
    case 'd': { long v = arg.toInt(); if (v >= 0) dwellMs = v; printParams(); break; }
    case 'g': setAngle(arg.toFloat()); printPos(); break;
    case '+': setAngle(angle + stepDeg); printPos(); break;
    case '-': setAngle(angle - stepDeg); printPos(); break;
    case 'm': mark(); break;
    case 'l': level = angle; printLevel(); break;
    case 'z': setAngle(level); printPos(); break;
    case 'p': printParams(); break;
    case '1':
    case '2': {
      int i = c - '1';
      trim[i] = constrain(arg.toFloat(), -SAFE_LIMIT_DEG, SAFE_LIMIT_DEG);
      setAngle(angle);                 // apply the new trim right away
      Serial.print(F("# servo ")); Serial.print(i + 1);
      Serial.print(F(" trim = ")); Serial.println(trim[i], 1);
      printPos();
      break;
    }
    default: Serial.println(F("# unknown command (s r t d g + - m l z p 1 2)"));
  }
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i], PULSE_MIN_US, PULSE_MAX_US);
  }
  setAngle(0);
  delay(1000);                         // give the servos time to reach the middle

  Serial.println(F("# Gimbal parameter sweep ready. Line ending = Newline."));
  printParams();
}

void loop() {
  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));
}

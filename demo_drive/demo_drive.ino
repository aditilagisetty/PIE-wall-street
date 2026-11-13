/*
 * Demo Drive: drives the car forward, then backward, with no laptop.
 *
 * Power on the car and it runs the demo once by itself:
 *   1. 10 s countdown (LED blinks slow, then fast). Step away from the props!
 *   2. Tilt gimbals forward, spin props up, drive forward.
 *   3. Props down, tilt gimbals backward, spin props up, drive backward.
 *   4. Props off, gimbals level. Done (stays off until you power cycle).
 *
 * Change the numbers in "Demo settings" to tune it.
 *
 * Uses the same wiring and gimbal zeros as test_rig. Set the zeros with
 * test_rig + rig_control first; this sketch reads them from the Arduino.
 *
 * Wiring (same as test_rig):
 *   Gimbal 1 -> D9       Gimbal 2 -> D10
 *   ESC 1 signal -> D5   ESC 2 signal -> D6
 *   Nano power: UBEC 5 V -> Nano 5V pin, UBEC GND -> Nano GND (no USB needed)
 *   Optional STOP button: D7 to GND. Press it any time to stop the props.
 *
 * SAFETY:
 *   - There's no laptop STOP. The kill switch (and the optional button) are
 *     your only stops. Keep a hand near the kill switch.
 *   - Test it first with the props OFF the motors.
 *   - Don't plug the battery in while uploading: the countdown starts as soon
 *     as the Arduino turns on.
 */

#include <Servo.h>
#include <EEPROM.h>

// Demo settings

// What the "power" numbers below mean:
//   ESC_US  = ESC pulse width in us, like RCBenchmark (1000 = off, 2000 = full)
//   PERCENT = throttle % (1000 us = 0 %, 2000 us = 100 %)
//   AMPS    = amps per motor (estimated from the motor table below)
enum Units { ESC_US, PERCENT, AMPS };
const Units DRIVE_UNITS = ESC_US;

const float DRIVE_POWER = 1250;          // power while driving, in DRIVE_UNITS (1250 us = 25 %)
const float MAX_THROTTLE_PCT = 40.0;     // hard cap: 1400 us, ~7.9 A per motor

// Motor table: T-Motor V3115 900KV, 10.5" prop, 6S. Amps per motor at each throttle %.
// Swap in your RCBenchmark numbers to match your motors.
const int   CURVE_N = 11;
const float CURVE_THR[CURVE_N]  = {0, 10,   20,   30,   40,   50,    60,    70,    80,    90,    100};
const float CURVE_AMPS[CURVE_N] = {0, 0.24, 1.27, 3.69, 7.91, 13.69, 21.28, 30.46, 41.19, 52.47, 64.37};

const float FORWARD_TILT_DEG  = 45.0;    // gimbal angle to drive forward
const float BACKWARD_TILT_DEG = -45.0;   // gimbal angle to drive backward
                                         // (swap the signs if the car goes the wrong way)

const unsigned long COUNTDOWN_MS = 10000; // wait before anything moves

// Each step: gimbal angle, power (in DRIVE_UNITS), how long (ms). Power 0 = off.
// The rig glides smoothly to each step's values.
struct Step { float tilt; float power; unsigned long ms; };

const Step STEPS[] = {
  // tilt                power         ms
  { 0,                   0,                   1000 },   // settle
  { FORWARD_TILT_DEG,    0,                   1500 },   // aim forward
  { FORWARD_TILT_DEG,    DRIVE_POWER,  3000 },   // drive forward
  { FORWARD_TILT_DEG,    0,                   2000 },   // props down, coast
  { BACKWARD_TILT_DEG,   0,                   1500 },   // aim backward
  { BACKWARD_TILT_DEG,   DRIVE_POWER,  3000 },   // drive backward
  { BACKWARD_TILT_DEG,   0,                   2000 },   // props down, coast
  { 0,                   0,                   1000 },   // level, done
};
const int NUM_STEPS = sizeof(STEPS) / sizeof(STEPS[0]);

// Smoothing (same idea as test_rig)
const float GIMBAL_SMOOTH_S = 0.15;
const float GIMBAL_SPEED_DEG_PER_S = 120.0;
const float THROTTLE_SMOOTH_S = 0.3;
const float THROTTLE_SPEED_PCT_PER_S = 50.0;

// Hardware

const int GIMBAL_PINS[2] = {9, 10};
const int ESC_PINS[2]    = {5, 6};
const int STOP_PIN       = 7;      // optional button to GND
const int LED_PIN        = 13;     // the Nano's built-in LED

const int GIMBAL_DIR[2] = {1, -1}; // gimbal 2 is on the other side of the car
const float GIMBAL_LIMIT_DEG = 150.0;

const int   SERVO_MIN_US = 500, SERVO_MAX_US = 2500, SERVO_CENTER_US = 1500;
const float SERVO_US_PER_DEG = (SERVO_MAX_US - SERVO_MIN_US) / 300.0;
const int   ESC_MIN_US = 1000, ESC_MAX_US = 2000;

// Saved zeros from test_rig (same spot in memory)
const int  EEPROM_ADDR = 0;
const long EEPROM_MAGIC = 0x5A45524FL;
struct SavedZeros { long magic; float zero[2]; };

// ===== State =====

Servo gimbals[2];
Servo escs[2];
float gimbalZero[2] = {0, 0};
float tiltNow = 0, throttleNow = 0;
bool stopped = false;

void loadZeros() {
  SavedZeros z;
  EEPROM.get(EEPROM_ADDR, z);
  bool ok = z.magic == EEPROM_MAGIC;
  for (int i = 0; i < 2; i++) ok = ok && !isnan(z.zero[i]) && fabs(z.zero[i]) <= 150.0;
  for (int i = 0; i < 2; i++) gimbalZero[i] = ok ? z.zero[i] : 0;
}

void writeGimbals() {
  for (int i = 0; i < 2; i++) {
    float deg = gimbalZero[i] + GIMBAL_DIR[i] * tiltNow;
    int us = (int)(SERVO_CENTER_US + deg * SERVO_US_PER_DEG + 0.5);
    gimbals[i].writeMicroseconds(constrain(us, SERVO_MIN_US, SERVO_MAX_US));
  }
}

void writeEscs() {
  float pct = stopped ? 0 : constrain(throttleNow, 0, MAX_THROTTLE_PCT);
  int us = (int)(ESC_MIN_US + pct * (ESC_MAX_US - ESC_MIN_US) / 100.0 + 0.5);
  for (int i = 0; i < 2; i++) escs[i].writeMicroseconds(us);
}

float glide(float now, float target, float dtS, float smoothS, float maxSpeed) {
  float step = (target - now) * min(1.0, dtS / smoothS);
  float maxStep = maxSpeed * dtS;
  return now + constrain(step, -maxStep, maxStep);
}

// Throttle % that should draw this many amps per motor (from the table)
float throttleForAmps(float amps) {
  if (amps <= CURVE_AMPS[0]) return CURVE_THR[0];
  for (int i = 1; i < CURVE_N; i++) {
    if (amps <= CURVE_AMPS[i]) {
      float f = (amps - CURVE_AMPS[i - 1]) / (CURVE_AMPS[i] - CURVE_AMPS[i - 1]);
      return CURVE_THR[i - 1] + f * (CURVE_THR[i] - CURVE_THR[i - 1]);
    }
  }
  return CURVE_THR[CURVE_N - 1];
}

// Turn a step's power (in DRIVE_UNITS) into throttle %
float stepThrottle(float power) {
  float t;
  if (DRIVE_UNITS == AMPS)        t = throttleForAmps(power);
  else if (DRIVE_UNITS == ESC_US) t = power <= 0 ? 0 : (power - ESC_MIN_US) * 100.0 / (ESC_MAX_US - ESC_MIN_US);
  else                            t = power;
  return constrain(t, 0, MAX_THROTTLE_PCT);
}

bool stopPressed() {
  return digitalRead(STOP_PIN) == LOW;
}

// Props off for good (until power cycle)
void stopForever() {
  stopped = true;
  throttleNow = 0;
  writeEscs();
}

// Keep outputs updated and watch the stop button for this long.
// Returns false if stopped.
bool runFor(unsigned long ms, float tiltTarget, float throttleTarget) {
  unsigned long start = millis(), last = start;
  while (millis() - start < ms) {
    if (stopPressed()) { stopForever(); return false; }
    unsigned long now = millis();
    float dtS = (now - last) / 1000.0;
    last = now;
    tiltNow = glide(tiltNow, constrain(tiltTarget, -GIMBAL_LIMIT_DEG, GIMBAL_LIMIT_DEG),
                    dtS, GIMBAL_SMOOTH_S, GIMBAL_SPEED_DEG_PER_S);
    throttleNow = glide(throttleNow, constrain(throttleTarget, 0, MAX_THROTTLE_PCT),
                        dtS, THROTTLE_SMOOTH_S, THROTTLE_SPEED_PCT_PER_S);
    if (throttleTarget == 0 && throttleNow < 0.05) throttleNow = 0;
    writeGimbals();
    writeEscs();
    digitalWrite(LED_PIN, throttleNow > 0 ? HIGH : LOW);   // LED on = props spinning
    delay(5);
  }
  return true;
}

void setup() {
  Serial.begin(115200);          // optional: messages if USB is plugged in
  pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  // Props off first. ESCs also need to see "off" for a few seconds to arm.
  for (int i = 0; i < 2; i++) escs[i].attach(ESC_PINS[i], ESC_MIN_US, ESC_MAX_US);
  writeEscs();

  loadZeros();
  for (int i = 0; i < 2; i++) gimbals[i].attach(GIMBAL_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
  writeGimbals();

  Serial.println(F("# demo: countdown"));

  // Countdown: slow blink, then fast blink for the last 3 s
  unsigned long start = millis();
  while (millis() - start < COUNTDOWN_MS) {
    if (stopPressed()) { stopForever(); Serial.println(F("# demo: stopped")); return; }
    unsigned long left = COUNTDOWN_MS - (millis() - start);
    int period = left < 3000 ? 150 : 500;
    digitalWrite(LED_PIN, (millis() / period) % 2);
    delay(5);
  }

  for (int i = 0; i < NUM_STEPS; i++) {
    Serial.print(F("# demo: step ")); Serial.println(i + 1);
    Serial.print(F("#   throttle % = ")); Serial.println(stepThrottle(STEPS[i].power));
    if (!runFor(STEPS[i].ms, STEPS[i].tilt, stepThrottle(STEPS[i].power))) {
      Serial.println(F("# demo: stopped"));
      return;
    }
  }

  stopForever();
  Serial.println(F("# demo: done"));
}

void loop() {
  // Demo is over: keep props off and gimbals where they are.
  writeEscs();
  digitalWrite(LED_PIN, LOW);
}

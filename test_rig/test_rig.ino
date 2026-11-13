/*
 * Test Rig: props + gimbals
 *
 * Controls 2 props and 2 gimbals. Use the laptop app (rig_control.py),
 * or type commands in the Serial Monitor (115200 baud, "Newline").
 *
 * Motors: T-Motor V3115 900KV. Very strong! Clamp the rig down and start slow.
 *
 * Wiring:
 *   Gimbal 1 -> D9       Gimbal 2 -> D10
 *   ESC 1 signal -> D5   ESC 2 signal -> D6   (props plug into the ESCs)
 *   Connect all the grounds together.
 *   Don't connect the ESC red wires to the Arduino.
 *
 * Commands:
 *   s 10 10 20 20   gimbal 1 angle, gimbal 2 angle, prop 1 %, prop 2 %
 *   a               arm the props (throttle must be 0)
 *   x               STOP
 *   z 1 3.5         make 3.5 deg (from servo middle) gimbal 1's new zero
 *   z 2 -2          make -2 deg gimbal 2's new zero
 *   z               print both zeros
 *   Zeros are saved on the Arduino, so they stay after unplugging.
 *   All other angles are measured from these zeros.
 *
 * Gimbal 2 is mounted on the other side of the car, so it turns the opposite
 * way (GIMBAL_DIR). Same angle in = both props tilt the same way.
 *
 * Safety:
 *   - Props start off.
 *   - Props stop if no command comes for 0.5 s (app crashed, cable out).
 *   - Throttle and gimbals glide smoothly to new values. STOP is instant.
 *
 * Sends back 20 times a second:
 *   T,time,gimbal1,gimbal2,prop1 %,prop2 %,armed
 */

#include <Servo.h>
#include <EEPROM.h>

// Settings

// Zero for each gimbal, in degrees from the servo middle.
// Only used until you set one with the "z" command.
const float DEFAULT_ZERO_DEG[2] = {0.0, 0.0};

// Biggest zero allowed (degrees from the servo middle)
const float ZERO_LIMIT_DEG = 150.0;

// Turn direction: 1 = normal, -1 = mirrored (gimbal 2 is on the other side)
const int GIMBAL_DIR[2] = {1, -1};

// Max tilt from zero
const float GIMBAL_LIMIT_DEG = 150.0;   // servo can only do 300 deg total (+-150)

// Smoothing: how fast the gimbals glide to a new angle.
// SMOOTH_S = how soft the start/stop is (bigger = softer, slower)
// SPEED = top speed (degrees per second)
const float GIMBAL_SMOOTH_S = 0.15;
const float GIMBAL_SPEED_DEG_PER_S = 120.0;

// Same for throttle
const float THROTTLE_SMOOTH_S = 0.2;

// Max throttle %
const float MAX_THROTTLE_PCT = 100.0;

// Top speed for throttle changes (% per second)
const float THROTTLE_RAMP_PCT_PER_S = 100.0;

// Stop props if no command for this long
const unsigned long COMMAND_TIMEOUT_MS = 500;

// Pins

const int GIMBAL_PINS[2] = {9, 10};
const int ESC_PINS[2]    = {5, 6};

// Signal ranges

// Servo: 500-2500 us = 300 degrees
const int   SERVO_MIN_US    = 500;
const int   SERVO_MAX_US    = 2500;
const int   SERVO_CENTER_US = 1500;
const float SERVO_US_PER_DEG = (SERVO_MAX_US - SERVO_MIN_US) / 300.0;

// ESC: 1000 us = off, 2000 us = full
const int ESC_MIN_US = 1000;
const int ESC_MAX_US = 2000;

const unsigned long TELEMETRY_MS = 50;

// State

Servo gimbals[2];
Servo escs[2];

float gimbalZero[2]     = {0, 0};      // each gimbal's zero (from servo middle)
float gimbalTarget[2]   = {0, 0};      // gimbal angle asked for (from zero)
float gimbalNow[2]      = {0, 0};      // gimbal angle right now (gliding to target)
float throttleTarget[2] = {0, 0};      // throttle asked for
float throttle[2]       = {0, 0};      // throttle right now (ramping up to target)
bool armed = false;
unsigned long lastCommandMs = 0;
unsigned long lastLoopMs = 0;
unsigned long lastTelemetryMs = 0;

char lineBuf[48];
int lineLen = 0;

// zero offsets (saved in EEPROM)

const int EEPROM_ADDR = 0;
const long EEPROM_MAGIC = 0x5A45524FL;  // marks that zeros were saved

struct SavedZeros {
  long magic;
  float zero[2];
};

void loadZeros() {
  SavedZeros z;
  EEPROM.get(EEPROM_ADDR, z);
  bool ok = z.magic == EEPROM_MAGIC;
  for (int i = 0; i < 2; i++) {
    ok = ok && !isnan(z.zero[i]) && fabs(z.zero[i]) <= ZERO_LIMIT_DEG;
  }
  for (int i = 0; i < 2; i++) gimbalZero[i] = ok ? z.zero[i] : DEFAULT_ZERO_DEG[i];
}

void saveZeros() {
  SavedZeros z;
  z.magic = EEPROM_MAGIC;
  for (int i = 0; i < 2; i++) z.zero[i] = gimbalZero[i];
  EEPROM.put(EEPROM_ADDR, z);
}

// The app reads this line
void printZeros() {
  Serial.print(F("# zero: "));
  Serial.print(gimbalZero[0], 1);
  Serial.print(' ');
  Serial.println(gimbalZero[1], 1);
}

// outputs

void writeGimbals() {
  for (int i = 0; i < 2; i++) {
    float deg = gimbalZero[i] + GIMBAL_DIR[i] * gimbalNow[i];
    int us = (int)(SERVO_CENTER_US + deg * SERVO_US_PER_DEG + 0.5);
    gimbals[i].writeMicroseconds(constrain(us, SERVO_MIN_US, SERVO_MAX_US));
  }
}

void writeEscs() {
  for (int i = 0; i < 2; i++) {
    float pct = armed ? throttle[i] : 0;
    int us = (int)(ESC_MIN_US + pct * (ESC_MAX_US - ESC_MIN_US) / 100.0 + 0.5);
    escs[i].writeMicroseconds(constrain(us, ESC_MIN_US, ESC_MAX_US));
  }
}

// turn the props off
void disarm(const __FlashStringHelper *why) {
  bool wasArmed = armed;
  armed = false;
  for (int i = 0; i < 2; i++) { throttle[i] = 0; throttleTarget[i] = 0; }
  writeEscs();
  if (wasArmed) { Serial.print(F("# DISARMED: ")); Serial.println(why); }
}

// Move value a little closer to target: soft start/stop, capped speed
float glide(float now, float target, float dtS, float smoothS, float maxSpeed) {
  float step = (target - now) * min(1.0, dtS / smoothS);
  float maxStep = maxSpeed * dtS;
  return now + constrain(step, -maxStep, maxStep);
}

void rampThrottle(float dtS) {
  for (int i = 0; i < 2; i++) {
    if (!armed) { throttle[i] = 0; continue; }
    throttle[i] = glide(throttle[i], throttleTarget[i], dtS, THROTTLE_SMOOTH_S, THROTTLE_RAMP_PCT_PER_S);
    if (throttle[i] < 0.05 && throttleTarget[i] == 0) throttle[i] = 0;
  }
}

void moveGimbals(float dtS) {
  for (int i = 0; i < 2; i++) {
    gimbalNow[i] = glide(gimbalNow[i], gimbalTarget[i], dtS, GIMBAL_SMOOTH_S, GIMBAL_SPEED_DEG_PER_S);
  }
  writeGimbals();
}

// commands

void handleLine(char *line) {
  while (*line == ' ') line++;
  char c = line[0];

  if (c == 's') {
    char *p = line + 1;
    float v[4];
    for (int k = 0; k < 4; k++) {
      char *end;
      v[k] = strtod(p, &end);
      if (end == p) { Serial.println(F("# usage: s <gimbal1> <gimbal2> <throttle1> <throttle2>")); return; }
      p = end;
    }
    for (int i = 0; i < 2; i++) {
      gimbalTarget[i]   = constrain(v[i],     -GIMBAL_LIMIT_DEG, GIMBAL_LIMIT_DEG);
      throttleTarget[i] = constrain(v[i + 2], 0, MAX_THROTTLE_PCT);
    }
    lastCommandMs = millis();
  } else if (c == 'a') {
    if (throttleTarget[0] > 0 || throttleTarget[1] > 0) {
      Serial.println(F("# not armed: set throttle to 0 first"));
    } else {
      armed = true;
      lastCommandMs = millis();
      Serial.println(F("# ARMED"));
    }
  } else if (c == 'x') {
    disarm(F("stop command"));
    Serial.println(F("# stopped"));
  } else if (c == 'z') {
    char *p = line + 1;
    char *end;
    long which = strtol(p, &end, 10);
    if (end == p) { printZeros(); return; }       // just "z"
    p = end;
    float deg = strtod(p, &end);
    if (end == p || (which != 1 && which != 2)) {
      Serial.println(F("# usage: z <1 or 2> <degrees>"));
      return;
    }
    gimbalZero[which - 1] = constrain(deg, -ZERO_LIMIT_DEG, ZERO_LIMIT_DEG);
    saveZeros();
    writeGimbals();
    printZeros();
  } else if (c != '\0') {
    Serial.println(F("# unknown command (s a x z)"));
  }
}

// Read typed commands without pausing the program
void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      lineBuf[lineLen] = '\0';
      handleLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < (int)sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = ch;
    }
  }
}

// ---------- Send data back ----------

void sendTelemetry() {
  Serial.print(F("T,"));
  Serial.print(millis());              Serial.print(',');
  Serial.print(gimbalNow[0], 1);       Serial.print(',');
  Serial.print(gimbalNow[1], 1);       Serial.print(',');
  Serial.print(armed ? throttle[0] : 0, 1); Serial.print(',');
  Serial.print(armed ? throttle[1] : 0, 1); Serial.print(',');
  Serial.println(armed ? 1 : 0);
}

// ---------- Main ----------

void setup() {
  Serial.begin(115200);

  // Props off first
  for (int i = 0; i < 2; i++) escs[i].attach(ESC_PINS[i], ESC_MIN_US, ESC_MAX_US);
  writeEscs();

  loadZeros();
  for (int i = 0; i < 2; i++) gimbals[i].attach(GIMBAL_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
  writeGimbals();

  lastLoopMs = millis();
  Serial.println(F("# test rig ready (disarmed)"));
  printZeros();
}

void loop() {
  readSerial();

  unsigned long now = millis();
  float dtS = (now - lastLoopMs) / 1000.0;
  lastLoopMs = now;

  // Safety: stop if the laptop goes quiet
  if (armed && now - lastCommandMs > COMMAND_TIMEOUT_MS) {
    disarm(F("no command for 0.5 s"));
  }

  rampThrottle(dtS);
  writeEscs();
  moveGimbals(dtS);

  if (now - lastTelemetryMs >= TELEMETRY_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }
}

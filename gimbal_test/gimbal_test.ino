/*
 * Gimbal Test - 2 DOF
 *
 * Sweeps each gimbal axis -30 -> +30 deg in small steps, then both together
 *   Outer axis (frame): servo drives the frame directly
 *   Inner axis (bar):   DC motor with encoder drives the bar through the spur gears
 *
 * The encoder only counts movement, so wherever the bar is at power-up is 0 deg
 * Start it level
 *
 * Wiring (Arduino Uno + Adafruit Motor Shield v2):
 *   Outer servo       -> shield "Servo 2" header (D9)
 *   Inner DC motor    -> shield M1 terminals
 *   Encoder channel A -> D2
 *   Encoder channel B -> D3
 *   Encoder VCC / GND -> 5V / GND
 */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Pins
const int PIN_SERVO     = 9;
const int MOTOR_PORT    = 1;         // shield port M1
const int PIN_ENCODER_A = 2;
const int PIN_ENCODER_B = 3;

// Tuning
const int MIN_ANGLE = -30;           // degrees from level
const int MAX_ANGLE = 30;
const int STEP_DEG  = 2;
const unsigned long STEP_MS  = 40;   // pause between steps
const unsigned long PAUSE_MS = 1000; // pause between sweeps

const int SERVO_LEVEL = 90;          // servo angle that puts the frame level

// Encoder counts for one full turn of the bar:
// encoder pulses per motor rev x motor gearbox ratio x (bar gear teeth / motor gear teeth)
const float COUNTS_PER_REV = 12 * 75.0 * 1.0;
const float COUNTS_PER_DEG = COUNTS_PER_REV / 360.0;

// Inner motor: speed = KP x error (degrees) limited to MIN_SPEED..MAX_SPEED
const float KP = 20.0;
const int MIN_SPEED = 60;            // slowest speed that still turns the motor
const int MAX_SPEED = 200;
const float TOLERANCE_DEG = 1.0;     // close enough to stop
const unsigned long MOVE_TIMEOUT_MS = 1500;

// State
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *innerMotor = AFMS.getMotor(MOTOR_PORT);
Servo outerServo;

volatile long encoderCount = 0;

// Runs on every rising edge of channel A. Channel B tells us the direction.
void readEncoder() {
  if (digitalRead(PIN_ENCODER_B) == HIGH) encoderCount++;
  else                                    encoderCount--;
}

float innerAngle() {
  noInterrupts();
  long count = encoderCount;
  interrupts();
  return count / COUNTS_PER_DEG;
}

// Drive the inner motor until the bar reaches target (or we time out)
void moveInner(int target) {
  unsigned long start = millis();
  while (millis() - start < MOVE_TIMEOUT_MS) {
    float error = target - innerAngle();
    if (abs(error) <= TOLERANCE_DEG) break;

    innerMotor->setSpeed(constrain(KP * abs(error), MIN_SPEED, MAX_SPEED));
    innerMotor->run(error > 0 ? FORWARD : BACKWARD);
  }
  innerMotor->run(RELEASE);
}

void moveTo(int outer, int inner) {
  outerServo.write(SERVO_LEVEL + outer);
  moveInner(inner);

  Serial.print("Outer: ");         Serial.print(outer);
  Serial.print("  Inner target: "); Serial.print(inner);
  Serial.print("  Inner actual: "); Serial.println(innerAngle(), 1);
  delay(STEP_MS);
}

// Step 0 -> MAX -> MIN -> 0 on the chosen axes. The other axis stays level.
void sweep(bool outer, bool inner) {
  for (int a = 0; a <= MAX_ANGLE; a += STEP_DEG)         moveTo(outer ? a : 0, inner ? a : 0);
  for (int a = MAX_ANGLE; a >= MIN_ANGLE; a -= STEP_DEG) moveTo(outer ? a : 0, inner ? a : 0);
  for (int a = MIN_ANGLE; a <= 0; a += STEP_DEG)         moveTo(outer ? a : 0, inner ? a : 0);
  delay(PAUSE_MS);
}

// Main
void setup() {
  Serial.begin(9600);

  if (!AFMS.begin()) {               // default I2C address 0x60
    Serial.println("Motor shield not found - check that it's seated.");
    while (true) {}
  }
  innerMotor->run(RELEASE);          // shield remembers its last command across a reset

  outerServo.attach(PIN_SERVO);
  outerServo.write(SERVO_LEVEL);

  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), readEncoder, RISING);

  delay(1000);                       // give the servo time to settle
}

void loop() {
  Serial.println("Outer axis sweep");
  sweep(true, false);

  Serial.println("Inner axis sweep");
  sweep(false, true);

  Serial.println("Both axes sweep");
  sweep(true, true);
}

/*
 * =====================================================================================
 * BRAC University - CSE461: Robotics & Embedded Systems
 * Group 5: Robotic Arm Controller - Hardware-Debounced IR Fallback Firmware
 * =====================================================================================
 */

#include <Servo.h>

// ==========================================
// PIN ALLOCATIONS
// ==========================================
// Pin A5 functions as a clean Digital GPIO, completely bypassing D12/D13
const uint8_t PIN_IR_SENSOR  = A5; // IR Proximity Sensor (Active LOW)
const uint8_t SERVO_ARM_PIN  = 3;  // Arm Base Tilt Servo PWM (D3)
const uint8_t SERVO_CLAW_PIN = 11; // Claw Gripper Servo PWM (D11)
const uint8_t PIN_LED_ALERT  = 13; // Status / Target Confirmation Indicator

// ==========================================
// STRICT PHYSICAL ANGLES (0° to 180° Range)
// ==========================================
const int ARM_INIT_ANGLE   = 30;   // Upright Home Position
const int ARM_PICKUP_ANGLE = 158;  // CCW sweep (+128°) to touch floor firmly
const int ARM_DROP_ANGLE   = 0;    // CW sweep (-158°) to reach deep rear bin
const int ARM_RETURN_ANGLE = 30;   // CCW sweep (+30°) to return upright

// Claw Gripper Physical Angles
const int CLAW_OPEN_ANGLE  = 120;  // Open jaws wide for intake/release
const int CLAW_GRIP_ANGLE  = 20;   // Firm locked grasp on garbage

// Motion Pacing
const int DELAY_ARM_STEP_MS  = 24;
const int DELAY_CLAW_STEP_MS = 18;

// Multi-Frame Filter Settings: Require 3 consecutive confirmed frames
const uint8_t REQUIRED_CONFIRMATIONS = 3;
uint8_t targetConfirmCounter = 0;
bool isExecutingSequence = false;

// Sampling Interval for IR Debounce
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL_MS = 50; 

Servo armServo;
Servo clawServo;

int currentArmPos  = ARM_INIT_ANGLE;
int currentClawPos = CLAW_GRIP_ANGLE;

void attachServos();
void detachServos();
void moveArmSmooth(int target);
void moveClawSmooth(int target);
void executePickAndDrop();

void setup() {
  Serial.begin(9600);

  pinMode(PIN_IR_SENSOR, INPUT);
  pinMode(PIN_LED_ALERT, OUTPUT);
  digitalWrite(PIN_LED_ALERT, LOW);

  Serial.println(F("\n======================================================="));
  Serial.println(F(" BRACU CSE461 Group 5: Robotic Arm IR-Fallback System  "));
  Serial.println(F(" Status: Hardware-Debounced Floor Trigger Active       "));
  Serial.println(F(" Angles: Init 30° | Floor 158° | Bin Drop 0° | Home 30°"));
  Serial.println(F("======================================================="));

  // Align and detach servos to prevent boot startup shudder
  attachServos();
  armServo.write(ARM_INIT_ANGLE);
  clawServo.write(CLAW_GRIP_ANGLE);
  currentArmPos  = ARM_INIT_ANGLE;
  currentClawPos = CLAW_GRIP_ANGLE;
  delay(350);
  detachServos();

  Serial.println(F("[SYSTEM] Servos zeroed & detached."));
  Serial.println(F("[SYSTEM] Calibrating IR sensor input on Pin A5..."));
  delay(1000);

  targetConfirmCounter = 0;
  isExecutingSequence = false;
  Serial.println(F("[READY] Sensor armed. Monitoring floor at grasp distance (12-14 cm)...\n"));
}

void loop() {
  if (isExecutingSequence) {
    return;
  }

  // Sample the IR sensor at regular intervals (every 50 ms)
  if (millis() - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = millis();

    // Active LOW: Most standard IR proximity modules output LOW when reflecting
    int irRawState = digitalRead(PIN_IR_SENSOR);

    if (irRawState == LOW) {
      targetConfirmCounter++;
      digitalWrite(PIN_LED_ALERT, HIGH);

      Serial.print(F("[IR SENSOR DETECT] Target in grasp range (12-14 cm) | Confirmation Frame: "));
      Serial.print(targetConfirmCounter);
      Serial.print(F("/"));
      Serial.println(REQUIRED_CONFIRMATIONS);

      if (targetConfirmCounter >= REQUIRED_CONFIRMATIONS) {
        Serial.println(F("\n>>> [TARGET CONFIRMED] Object locked in grasp zone! <<<"));
        Serial.println(F(">>> Initiating calibrated 6-step pick & drop routine. <<<\n"));
        executePickAndDrop();
      }
    } else {
      // If beam clears before reaching threshold, reset counter
      if (targetConfirmCounter > 0) {
        Serial.println(F("[IR SENSOR IDLE] Target cleared / Glare rejected. Resetting frames."));
        targetConfirmCounter = 0;
        digitalWrite(PIN_LED_ALERT, LOW);
      }
    }
  }
}

void attachServos() {
  if (!armServo.attached()) armServo.attach(SERVO_ARM_PIN);
  if (!clawServo.attached()) clawServo.attach(SERVO_CLAW_PIN);
}

void detachServos() {
  if (armServo.attached()) armServo.detach();
  if (clawServo.attached()) clawServo.detach();
}

void moveArmSmooth(int target) {
  target = constrain(target, 0, 180);
  if (currentArmPos < target) {
    for (int p = currentArmPos; p <= target; p++) {
      armServo.write(p);
      delay(DELAY_ARM_STEP_MS);
    }
  } else {
    for (int p = currentArmPos; p >= target; p--) {
      armServo.write(p);
      delay(DELAY_ARM_STEP_MS);
    }
  }
  currentArmPos = target;
}

void moveClawSmooth(int target) {
  target = constrain(target, 15, 140);
  if (currentClawPos < target) {
    for (int p = currentClawPos; p <= target; p++) {
      clawServo.write(p);
      delay(DELAY_CLAW_STEP_MS);
    }
  } else {
    for (int p = currentClawPos; p >= target; p--) {
      clawServo.write(p);
      delay(DELAY_CLAW_STEP_MS);
    }
  }
  currentClawPos = target;
}

void executePickAndDrop() {
  // SENSOR BLINDING ACTIVATED: Locks out loop checks
  isExecutingSequence = true;
  targetConfirmCounter = 0;
  digitalWrite(PIN_LED_ALERT, HIGH);

  attachServos();

  // Step 1: Open Jaws
  Serial.println(F("[STEP 1/6] Opening Claw Jaws to 120°..."));
  moveClawSmooth(CLAW_OPEN_ANGLE);
  delay(150);

  // Step 2: Sweep CCW to Floor Pickup (30° -> 158°)
  Serial.print(F("[STEP 2/6] Lowering Arm CCW: "));
  Serial.print(currentArmPos);
  Serial.print(F("° -> "));
  Serial.print(ARM_PICKUP_ANGLE);
  Serial.println(F("° (Reaching Floor Contact)..."));
  moveArmSmooth(ARM_PICKUP_ANGLE);
  delay(300);

  // Step 3: Firmly Close Claw Grip
  Serial.println(F("[STEP 3/6] Locking Claw Grip to 20° Around Target..."));
  moveClawSmooth(CLAW_GRIP_ANGLE);
  delay(400);

  // Step 4: Swing CW to Overhead Rear Bin (158° -> 0°)
  Serial.print(F("[STEP 4/6] Lifting Overhead CW: "));
  Serial.print(currentArmPos);
  Serial.print(F("° -> "));
  Serial.print(ARM_DROP_ANGLE);
  Serial.println(F("° (Reaching Rear Bin Drop)..."));
  moveArmSmooth(ARM_DROP_ANGLE);
  delay(400);

  // Step 5: Release Claw into Bin
  Serial.println(F("[STEP 5/6] Opening Claw to 120° (Depositing Waste)..."));
  moveClawSmooth(CLAW_OPEN_ANGLE);
  delay(500);

  // Step 6: Return CCW to Upright Home (0° -> 30°)
  Serial.print(F("[STEP 6/6] Returning Arm CCW: "));
  Serial.print(currentArmPos);
  Serial.print(F("° -> "));
  Serial.print(ARM_RETURN_ANGLE);
  Serial.println(F("° (Upright Park Position)..."));
  moveArmSmooth(ARM_RETURN_ANGLE);
  
  moveClawSmooth(CLAW_GRIP_ANGLE);
  delay(200);

  detachServos();

  // Mechanical Settle & Self-Detection Quarantine Window
  Serial.println(F("[QUARANTINE] Claw parked. Blinding sensor for 800 ms to clear reflections..."));
  delay(800);

  // Force reset counters to eliminate lingering reflections
  targetConfirmCounter = 0;
  digitalWrite(PIN_LED_ALERT, LOW);
  isExecutingSequence = false;

  Serial.println(F("[SYSTEM READY] Blind window closed. Resuming floor scanning.\n"));
}
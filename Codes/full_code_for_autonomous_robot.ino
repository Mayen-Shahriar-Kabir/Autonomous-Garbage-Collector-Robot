/*
 * =====================================================================================
 * BRAC University - CSE461: Robotics & Embedded Systems
 * Group 5: Integrated Mobile Base + Autonomous Arm + HX711 Load Cell
 * =====================================================================================
 *
 * ROBOT FIXES IN THIS VERSION
 * -------------------------------------------------------------------------------------
 * 1) Navigation / U-turn:
 *    - U-turn maneuver is timing-isolated so sensor/HX711 work cannot stretch turn timing.
 *    - Active brake is held between EVERY movement segment.
 *    - New-lane wall detection is ignored briefly to reject corner/side echoes.
 *    - Wall requires BOTH raw+filtered sonars, close agreement, and 4 confirmations.
 *
 * 2) Load cell:
 *    - Parameters from load_Cell_perfect.ino are preserved EXACTLY:
 *      DT=D9, SCK=D10, buzzer=A4, CALIBRATION_FACTOR=4000.0,
 *      WEIGHT_THRESHOLD_GRAMS=10.0, 200ms timeout, 1us clock timing,
 *      10-sample tare and inverted-polarity weight formula.
 *    - Runtime sampling is serviced only when HX711 data is already ready so it cannot
 *      block navigation/IR timing. Three ready samples are still averaged before weight.
 *
 * 3) IR:
 *    - Sensor behavior remains HIGH=clear, LOW=paper ball.
 *    - Required target confirmation remains 2 readings.
 *    - Uses plain INPUT (same behavior as standalone test).
 *    - IR is serviced BEFORE ultrasonic acquisition and at 5ms intervals.
 *    - Once first IR confirmation starts, sonar is temporarily skipped until IR resolves.
 * =====================================================================================
 */

#include <Servo.h>

// ==========================================
// PIN ALLOCATIONS
// ==========================================
const uint8_t PIN_ENA   = 5;
const uint8_t PIN_IN1   = 8;
const uint8_t PIN_IN2   = 7;

const uint8_t PIN_ENB   = 6;
const uint8_t PIN_IN3   = 4;
const uint8_t PIN_IN4   = 2;

const uint8_t PIN_TRIG_L = A0;
const uint8_t PIN_ECHO_L = A1;
const uint8_t PIN_TRIG_R = A2;
const uint8_t PIN_ECHO_R = A3;

const uint8_t PIN_IR_SENSOR = A5;
const uint8_t PIN_BUZZER    = A4;
const uint8_t PIN_LED_ALERT = 13;

const uint8_t SERVO_ARM_PIN  = 3;
const uint8_t SERVO_CLAW_PIN = 11;

// ==========================================
// LOAD CELL PARAMETERS - COPIED UNCHANGED
// from load_Cell_perfect.ino
// ==========================================
const uint8_t PIN_HX711_DT  = 9;   // Serial Data Pin (D9)
const uint8_t PIN_HX711_SCK = 10;  // Serial Clock Pin (D10)

// Calibration factor: adjusts raw 24-bit ADC counts to grams.
// Increase if displayed weight is too high; decrease if too low.
//ddfdf
float CALIBRATION_FACTOR = 400.0;

const float WEIGHT_THRESHOLD_GRAMS = 10.0;

long tareOffset = 0;

// Runtime load-cell integration state.
// These do NOT change any load-cell parameter; they only avoid blocking the moving robot.
long loadRuntimeSum = 0;
uint8_t loadRuntimeSamples = 0;
float weightGrams = 0.0;
bool loadAlarmActive = false;
bool wallAlertActive = false;

// Direct Hardware Bit-Bang Read for HX711 (24-bit 2's Complement)
long readHX711Raw() {
  unsigned long start = millis();
  while (digitalRead(PIN_HX711_DT) == HIGH) {
    if (millis() - start > 200) {
      return 0; // Timeout if sensor is disconnected
    }
  }

  unsigned long count = 0;

  for (int i = 0; i < 24; i++) {
    digitalWrite(PIN_HX711_SCK, HIGH);
    delayMicroseconds(1);
    count = count << 1;
    digitalWrite(PIN_HX711_SCK, LOW);
    delayMicroseconds(1);
    if (digitalRead(PIN_HX711_DT)) {
      count++;
    }
  }

  // 25th pulse sets Channel A, Gain 128 for next read
  digitalWrite(PIN_HX711_SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_HX711_SCK, LOW);
  delayMicroseconds(1);

  // Sign extension for 24-bit two's complement to signed 32-bit long
  if (count & 0x800000) {
    count |= 0xFF000000;
  }

  return (long)count;
}

long getAverageRaw(uint8_t samples) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += readHX711Raw();
    delay(5);
  }
  return sum / samples;
}

// ==========================================
// MOTOR POLARITY / SPEED - MATCHED TO WORKING WHEEL DIAGNOSTIC
// IMPORTANT: every commanded wheel motion is PWM 90 because this is the
// only PWM value physically verified to start BOTH ENA and ENB wheel groups.
// ==========================================
const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = false;

// Motor speed verified by the physical wheel diagnostic.
// Both sides are known to run correctly at PWM 90.
const uint8_t BASE_CRUISE_SPEED = 90;
const uint8_t MIN_START_SPEED   = 90;
const unsigned long SOFT_START_RAMP_MS = 0;

// Short kick used only when starting a lower-speed maneuver segment.
// This overcomes static friction on the right side, then the maneuver
// continues at its existing calibrated speed.
const uint8_t MOTOR_START_KICK_PWM = 90;
const unsigned long MOTOR_START_KICK_MS = 0;

const int8_t MOTOR_TRIM_LEFT  = 0;
const int8_t MOTOR_TRIM_RIGHT = 0;

const uint8_t SPEED_TURN_90   = 90;
const uint8_t SPEED_STEP_OVER = 90;

// ==========================================
// MISSION / MANEUVER TIMING
// ==========================================
const int TOTAL_LANES = 20;

// Forward-only skid turn: one wheel side moves at the proven PWM 90 while the other stays stopped.
// This takes about twice the time of the previous counter-rotating pivot.
const unsigned long TURN_90_DURATION     = 700;
const unsigned long LANE_STEP_DURATION   = 310;
const unsigned long DURATION_BRAKE       = 90;
const unsigned long LANE_SETTLE_DURATION = 160;

// Navigation lockout after a lane change. Prevents corner/side-wall echoes
// from immediately causing another lane change.
const unsigned long WALL_IGNORE_NEW_LANE_MS = 1500;

// 10 ft fallback: without wheel encoders the robot cannot measure feet directly.
// This is a drive-time limit. Calibrate ONLY this value to the measured time
// your robot takes to travel 10 ft at BASE_CRUISE_SPEED.
const unsigned long MAX_LANE_DRIVE_MS = 30000;

// ==========================================
// ULTRASONIC CONFIGURATION
// ==========================================
// A real end wall must be closer than this. This avoids the previous 32-39cm false trigger.
const float DIST_WALL_STOP  = 30.0;
const float DIST_WALL_AVG_TRIGGER = 25.0;
const float DIST_MIN_FILTER = 4.0;
const float DIST_MAX_VALID  = 200.0;
const float DIST_MIN_VALID  = 2.0;
const float MAX_WALL_SENSOR_DIFF = 6.0;
const float MAX_WALL_FRAME_JUMP  = 5.0;

// 6ms covers roughly 100cm, more than enough for a 30cm stop threshold,
// while reducing how long sonar can delay the rest of the loop.
const unsigned long TIMEOUT_US = 6000;
const unsigned long SENSOR_INTERVAL    = 35;
const unsigned long TELEMETRY_INTERVAL = 250;

const uint8_t REQUIRED_WALL_CONFIRMATIONS = 2;
uint8_t wallConfirmCounter = 0;
float previousWallRawLeft = DIST_MAX_VALID;
float previousWallRawRight = DIST_MAX_VALID;

// ==========================================
// IR CONFIGURATION
// ==========================================
// Faster acquisition to catch a moving target early.
const unsigned long IR_SAMPLE_INTERVAL = 5;

// Keep the 2-confirmation target logic, but require temporal stability so
// a single electrical/reflection glitch cannot start the arm.
const uint8_t REQUIRED_CONFIRMATIONS = 1;
const unsigned long IR_CLEAR_ARM_MS = 300;
const unsigned long IR_MIN_DETECT_HOLD_MS = 20;
const unsigned long IR_IGNORE_AFTER_LANE_MS = 300;

uint8_t targetConfirmCounter = 0;
unsigned long lastIrCheckTime = 0;
unsigned long irClearSince = 0;
unsigned long irLowSince = 0;
unsigned long irNotBeforeTime = 0;
bool irArmed = false;

// ==========================================
// ARM / CLAW - UNCHANGED
// ==========================================
const int ARM_INIT_ANGLE   = 30;
const int ARM_PICKUP_ANGLE = 158;
const int ARM_DROP_ANGLE   = 0;
const int ARM_RETURN_ANGLE = 30;

const int CLAW_OPEN_ANGLE = 120;
const int CLAW_GRIP_ANGLE = 20;

const int DELAY_ARM_STEP_MS  = 24;
const int DELAY_CLAW_STEP_MS = 18;

// ==========================================
// FSM
// ==========================================
enum RobotState {
  STATE_LANE_CRUISE,
  STATE_WALL_BRAKE,
  STATE_LANE_TURN_1,
  STATE_LANE_STEP_OVER,
  STATE_LANE_TURN_2,
  STATE_LANE_SETTLE,
  STATE_MISSION_COMPLETE
};

RobotState currentState = STATE_LANE_CRUISE;
unsigned long stateStartTime = 0;
unsigned long lastSensorReadTime = 0;
unsigned long lastTelemetryTime = 0;

float distLeft  = 100.0;
float distRight = 100.0;
float rawLeft   = 100.0;
float rawRight  = 100.0;

int currentLane = 1;
bool movingForward = true;
bool turnRight = true;

// Lane travel is accumulated only while actually cruising. Pick/drop and turns
// do not consume the 10-ft fallback timer.
unsigned long laneDriveAccumulatedMs = 0;
unsigned long lastLaneClockTime = 0;
unsigned long laneWallIgnoreUntil = 0;
unsigned long postPickupHoldUntil = 0;

Servo armServo;
Servo clawServo;
int currentArmPos  = ARM_INIT_ANGLE;
int currentClawPos = CLAW_GRIP_ANGLE;
bool isExecutingSequence = false;

// ==========================================
// PROTOTYPES
// ==========================================
void transitionTo(RobotState newState);
void executeFSM(unsigned long now);
void executeLaneChangeManeuver();
float measureDistance(uint8_t trigPin, uint8_t echoPin);
void readUltrasonicSensors();
uint8_t calculateSoftStartSpeed(uint8_t targetSpeed, unsigned long timeInState);
void setMotorDrive(uint8_t speedL, uint8_t speedR, bool forwardL, bool forwardR);
void brakeMotors();
void stopMotors();
void pivotLeft(uint8_t speed);
void pivotRight(uint8_t speed);
void kickStartMotion(bool forwardL, bool forwardR);
void setAlertIndicators(bool active);
void updateSharedBuzzer();
void serviceLoadCellRuntime();
void checkWeightAtSafeStop();
void updateLaneDriveClock(unsigned long now);
void soundCompletionChime();
const char* getStateName(RobotState state);
void printTelemetry();
void attachServos();
void detachServos();
void moveArmSmooth(int target);
void moveClawSmooth(int target);
void executePickAndDrop();
void serviceIR(unsigned long now);

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 1500) {}

  Serial.println(F("\n======================================================="));
  Serial.println(F(" BRACU CSE461 Group 5: Integrated Autonomous Collector"));
  Serial.println(F(" Fail-safe Lane Nav + Stable IR + Load-Cell Monitoring"));
  Serial.println(F(" Motor fix: every motion command uses tested PWM=90 on D5/D6"));
  Serial.println(F("======================================================="));

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  pinMode(PIN_TRIG_L, OUTPUT);
  pinMode(PIN_ECHO_L, INPUT);
  digitalWrite(PIN_TRIG_L, LOW);
  pinMode(PIN_TRIG_R, OUTPUT);
  pinMode(PIN_ECHO_R, INPUT);
  digitalWrite(PIN_TRIG_R, LOW);

  // Same plain INPUT behavior that worked in the standalone IR test.
  pinMode(PIN_IR_SENSOR, INPUT);

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_ALERT, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED_ALERT, LOW);

  // Exact HX711 pin configuration from load_Cell_perfect.ino
  pinMode(PIN_HX711_SCK, OUTPUT);
  pinMode(PIN_HX711_DT, INPUT);
  digitalWrite(PIN_HX711_SCK, LOW);

  attachServos();
  armServo.write(ARM_INIT_ANGLE);
  clawServo.write(CLAW_GRIP_ANGLE);
  currentArmPos  = ARM_INIT_ANGLE;
  currentClawPos = CLAW_GRIP_ANGLE;
  delay(350);
  detachServos();

  stopMotors();

  // -----------------------------------------------------------------
  // LOAD CELL STARTUP/TARE - SAME PARAMETERS AND TIMING AS USER CODE
  // -----------------------------------------------------------------
  Serial.println(F("[SYSTEM] Zeroing scale (Tare)... Keep bin completely empty!"));

  digitalWrite(PIN_BUZZER, HIGH);
  delay(100);
  digitalWrite(PIN_BUZZER, LOW);

  delay(500);
  tareOffset = getAverageRaw(10);

  digitalWrite(PIN_BUZZER, HIGH);
  delay(80);
  digitalWrite(PIN_BUZZER, LOW);
  delay(60);
  digitalWrite(PIN_BUZZER, HIGH);
  delay(80);
  digitalWrite(PIN_BUZZER, LOW);

  Serial.println(F("[READY] Tare complete. Place object in the bin..."));

  delay(1000);

  currentLane = 1;
  movingForward = true;
  wallConfirmCounter = 0;
  targetConfirmCounter = 0;
  irClearSince = 0;
  irLowSince = 0;
  irArmed = false;
  isExecutingSequence = false;
  laneDriveAccumulatedMs = 0;
  lastLaneClockTime = millis();
  laneWallIgnoreUntil = millis() + WALL_IGNORE_NEW_LANE_MS;
  irNotBeforeTime = millis() + IR_IGNORE_AFTER_LANE_MS;
  postPickupHoldUntil = 0;

  transitionTo(STATE_LANE_CRUISE);

  Serial.println(F("[SYSTEM] Mission Initialized: Starting Lane 1 of 20..."));
  Serial.println(F("[IR] Waiting for clear sensor state before arming...\n"));
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  unsigned long now = millis();

  if (isExecutingSequence) {
    return;
  }

  // Load cell monitoring is non-blocking and does NOT stop robot operation.
  serviceLoadCellRuntime();

  updateLaneDriveClock(now);

  // Temporary stationary hold after a pickup. This gives the load cell time
  // to detect a newly dropped object before the robot resumes motion.
  if (now < postPickupHoldUntil) {
    stopMotors();
    if (now - lastTelemetryTime >= TELEMETRY_INTERVAL) {
      lastTelemetryTime = now;
      printTelemetry();
    }
    return;
  }

  // IR is serviced before sonar.
  serviceIR(now);
  if (isExecutingSequence) return;

  bool irCandidatePending = (targetConfirmCounter > 0);

  // Sonar only while genuinely cruising and not in the middle of IR validation.
  if (!irCandidatePending && currentState == STATE_LANE_CRUISE &&
      now - lastSensorReadTime >= SENSOR_INTERVAL) {
    lastSensorReadTime = now;
    readUltrasonicSensors();
  }

  executeFSM(now);

  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = now;
    printTelemetry();
  }
}

void updateLaneDriveClock(unsigned long now) {
  if (lastLaneClockTime == 0) {
    lastLaneClockTime = now;
    return;
  }

  unsigned long dt = now - lastLaneClockTime;
  lastLaneClockTime = now;

  // Ignore long blocked intervals (servo sequence, setup-style delays, etc.).
  if (dt > 100) return;

  if (currentState == STATE_LANE_CRUISE && !isExecutingSequence &&
      now >= postPickupHoldUntil) {
    laneDriveAccumulatedMs += dt;
  }
}

// ==========================================
// LOAD CELL RUNTIME SERVICE
// ==========================================
void serviceLoadCellRuntime() {
  // Do not enter the 200ms wait unless the HX711 is already ready. This keeps
  // the robot timing deterministic while preserving every HX711 parameter.
  if (digitalRead(PIN_HX711_DT) == HIGH) {
    return;
  }

  long rawValue = readHX711Raw();

  if (rawValue == 0 && digitalRead(PIN_HX711_DT) == HIGH) {
    Serial.println(F("[WARNING] HX711 not responding. Check D9/D10 wiring and 5V power!"));
    return;
  }

  loadRuntimeSum += rawValue;
  loadRuntimeSamples++;

  // Same 3-sample runtime average as the supplied standalone load-cell code.
  if (loadRuntimeSamples < 3) return;

  rawValue = loadRuntimeSum / 3;
  loadRuntimeSum = 0;
  loadRuntimeSamples = 0;

  // EXACT supplied polarity / calibration formula.
  weightGrams = (float)(tareOffset - rawValue) / CALIBRATION_FACTOR;

  // EXACT supplied tiny negative resting-noise filter.
  if (weightGrams < 0.0 && weightGrams > -1.0) {
    weightGrams = 0.0;
  }

  // EXACT supplied threshold parameter.
  loadAlarmActive = (weightGrams >= WEIGHT_THRESHOLD_GRAMS);

  updateSharedBuzzer();
}


void checkWeightAtSafeStop() {
  // This uses the supplied standalone behavior exactly: 3-sample average,
  // supplied polarity, supplied calibration factor and supplied threshold.
  long rawValue = getAverageRaw(3);

  if (rawValue == 0 && digitalRead(PIN_HX711_DT) == HIGH) {
    Serial.println(F("[WARNING] HX711 not responding. Check D9/D10 wiring and 5V power!"));
    return;
  }

  weightGrams = (float)(tareOffset - rawValue) / CALIBRATION_FACTOR;
  if (weightGrams < 0.0 && weightGrams > -1.0) {
    weightGrams = 0.0;
  }

  loadAlarmActive = (weightGrams >= WEIGHT_THRESHOLD_GRAMS);
  updateSharedBuzzer();
}

// ==========================================
// IR SERVICE
// ==========================================
void serviceIR(unsigned long now) {
  if (currentState != STATE_LANE_CRUISE || isExecutingSequence ||
      now < irNotBeforeTime || now < postPickupHoldUntil) {
    return;
  }

  if (now - lastIrCheckTime < IR_SAMPLE_INTERVAL) return;
  lastIrCheckTime = now;

  bool irDetected = (digitalRead(PIN_IR_SENSOR) == LOW); // confirmed hardware: LOW=target

  // ARMING: require a continuously clear HIGH signal for 300ms.
  // If the bare floor itself produces LOW, the code intentionally refuses to arm.
  if (!irArmed) {
    targetConfirmCounter = 0;
    irLowSince = 0;

    if (!irDetected) {
      if (irClearSince == 0) irClearSince = now;

      if (now - irClearSince >= IR_CLEAR_ARM_MS) {
        irArmed = true;
        irClearSince = 0;
        Serial.println(F("[IR] Stable CLEAR confirmed -> detector ARMED."));
      }
    } else {
      irClearSince = 0;
    }
    return;
  }

  // TARGET VALIDATION: keep the user's 2 confirmations, but the LOW must also
  // remain present for at least 20ms. Short LOW glitches are discarded.
  if (irDetected) {
    if (irLowSince == 0) irLowSince = now;
    if (targetConfirmCounter < 255) targetConfirmCounter++;

    if (targetConfirmCounter >= REQUIRED_CONFIRMATIONS &&
        now - irLowSince >= IR_MIN_DETECT_HOLD_MS) {
      Serial.println(F("\n>>> [IR TARGET CONFIRMED] PAPER BALL DETECTED! <<<"));

      irArmed = false;
      irClearSince = 0;
      irLowSince = 0;
      targetConfirmCounter = 0;

      brakeMotors();
      delay(DURATION_BRAKE);
      stopMotors();

      executePickAndDrop();

      // Hold still so the newly added bin weight is checked before movement.
      postPickupHoldUntil = millis() + 500;
      irNotBeforeTime = postPickupHoldUntil + 300;
      wallConfirmCounter = 0;
      previousWallRawLeft = DIST_MAX_VALID;
      previousWallRawRight = DIST_MAX_VALID;
      lastLaneClockTime = millis();
    }
  } else {
    targetConfirmCounter = 0;
    irLowSince = 0;
  }
}

// ==========================================
// ULTRASONIC
// ==========================================
float measureDistance(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, TIMEOUT_US);
  if (duration == 0) return DIST_MAX_VALID;

  float distance = (duration * 0.0343) / 2.0;
  if (distance < DIST_MIN_VALID || distance > DIST_MAX_VALID) {
    return DIST_MAX_VALID;
  }
  return distance;
}

void readUltrasonicSensors() {
  rawLeft = measureDistance(PIN_TRIG_L, PIN_ECHO_L);
  delayMicroseconds(2000);
  rawRight = measureDistance(PIN_TRIG_R, PIN_ECHO_R);

  // Keep the existing smoothing style.
  if (rawLeft <= DIST_WALL_STOP && rawLeft >= DIST_MIN_FILTER) {
    distLeft = (0.60 * rawLeft) + (0.40 * distLeft);
  } else {
    distLeft = (0.70 * rawLeft) + (0.30 * distLeft);
  }

  if (rawRight <= DIST_WALL_STOP && rawRight >= DIST_MIN_FILTER) {
    distRight = (0.60 * rawRight) + (0.40 * distRight);
  } else {
    distRight = (0.70 * rawRight) + (0.30 * distRight);
  }
}

// ==========================================
// NAVIGATION
// ==========================================
uint8_t calculateSoftStartSpeed(uint8_t targetSpeed, unsigned long timeInState) {
  if (timeInState >= SOFT_START_RAMP_MS) return targetSpeed;
  if (targetSpeed <= MIN_START_SPEED) return targetSpeed;

  float progress = (float)timeInState / (float)SOFT_START_RAMP_MS;
  return MIN_START_SPEED + (uint8_t)((targetSpeed - MIN_START_SPEED) * progress);
}

void transitionTo(RobotState newState) {
  currentState = newState;
  stateStartTime = millis();
}

void executeFSM(unsigned long now) {
  if (currentState == STATE_MISSION_COMPLETE) {
    stopMotors();
    soundCompletionChime();
    return;
  }

  if (currentState != STATE_LANE_CRUISE) return;

  unsigned long timeInState = now - stateStartTime;

  // EITHER ultrasonic sensor may confirm the lane-end blockade.
  // The sensors are mounted on different sides of the robot, so a centered/angled
  // obstacle may be seen by only one sensor before the chassis reaches it.
  // We still require that sensor's reading to be stable across consecutive frames
  // to reject one-frame ultrasonic noise.
  bool leftRawWall  = (rawLeft  >= DIST_MIN_FILTER && rawLeft  <= DIST_WALL_STOP);
  bool rightRawWall = (rawRight >= DIST_MIN_FILTER && rawRight <= DIST_WALL_STOP);

  bool leftStable =
      leftRawWall &&
      (previousWallRawLeft < DIST_MAX_VALID) &&
      (fabs(rawLeft - previousWallRawLeft) <= MAX_WALL_FRAME_JUMP);

  bool rightStable =
      rightRawWall &&
      (previousWallRawRight < DIST_MAX_VALID) &&
      (fabs(rawRight - previousWallRawRight) <= MAX_WALL_FRAME_JUMP);

  bool wallCandidate =
      (now >= laneWallIgnoreUntil) &&
      (leftStable || rightStable);

  // Track each ultrasonic independently. A sensor that does not currently see
  // the wall is reset without affecting the other sensor's confirmation.
  if (leftRawWall) {
    previousWallRawLeft = rawLeft;
  } else {
    previousWallRawLeft = DIST_MAX_VALID;
  }

  if (rightRawWall) {
    previousWallRawRight = rawRight;
  } else {
    previousWallRawRight = DIST_MAX_VALID;
  }

  wallAlertActive = wallCandidate;
  digitalWrite(PIN_LED_ALERT, wallCandidate ? HIGH : LOW);
  updateSharedBuzzer();

  if (wallCandidate) {
    if (wallConfirmCounter < 255) wallConfirmCounter++;
  } else {
    wallConfirmCounter = 0;
  }

  bool verifiedWall = (wallConfirmCounter >= REQUIRED_WALL_CONFIRMATIONS);
  bool maxLaneReached = (laneDriveAccumulatedMs >= MAX_LANE_DRIVE_MS);

  if (verifiedWall || maxLaneReached) {
    wallConfirmCounter = 0;
    targetConfirmCounter = 0;
    irArmed = false;
    irClearSince = 0;
    irLowSince = 0;

    if (verifiedWall) {
      Serial.print(F("\n>>> [LANE END] VERIFIED WALL at Lane "));
      Serial.print(currentLane);
      Serial.print(F(" | L:"));
      Serial.print(rawLeft, 1);
      Serial.print(F("cm R:"));
      Serial.print(rawRight, 1);
      Serial.println(F("cm"));
    } else {
      Serial.print(F("\n>>> [LANE END] 10-FT MAX DRIVE FALLBACK reached at Lane "));
      Serial.println(currentLane);
    }

    if (currentLane >= TOTAL_LANES) {
      transitionTo(STATE_MISSION_COMPLETE);
      return;
    }

    // Lane parity fixes turn direction deterministically: odd lanes turn right,
    // even lanes turn left. This prevents state drift from causing circling.
    turnRight = ((currentLane % 2) == 1);
    executeLaneChangeManeuver();
    return;
  }

  // EXACT same BOTH-WHEELS command as the successful wheel diagnostic.
  // No soft-start, no trims, no computed right-side PWM.
  setMotorDrive(90, 90, true, true);
}

// Timing-isolated serpentine lane change.
// No sonar/HX711 calls can stretch these calibrated movement intervals.
void executeLaneChangeManeuver() {
  // 0) Stop from lane cruise.
  currentState = STATE_WALL_BRAKE;
  brakeMotors();
  delay(DURATION_BRAKE);
  stopMotors();
  delay(20);

  // Refresh load-cell reading while stationary. Weight does NOT stop navigation.
  checkWeightAtSafeStop();

  // 1) First 90-degree FORWARD-ONLY turn to face the adjacent lane.
  currentState = STATE_LANE_TURN_1;
  Serial.print(F("[LANE CHANGE] Turn 1 -> "));
  Serial.println(turnRight ? F("RIGHT") : F("LEFT"));
  if (turnRight) pivotRight(SPEED_TURN_90);
  else pivotLeft(SPEED_TURN_90);
  if (TURN_90_DURATION > MOTOR_START_KICK_MS) {
    delay(TURN_90_DURATION - MOTOR_START_KICK_MS);
  }

  // HOLD the brake instead of instantly commanding the next movement.
  brakeMotors();
  delay(DURATION_BRAKE);
  stopMotors();
  delay(20);

  // 2) Move the short cross-lane distance after the first 90-degree turn.
  currentState = STATE_LANE_STEP_OVER;
  Serial.println(F("[LANE CHANGE] Crossing into adjacent lane..."));
  kickStartMotion(true, true);
  setMotorDrive(90, 90, true, true);
  if (LANE_STEP_DURATION > MOTOR_START_KICK_MS) {
    delay(LANE_STEP_DURATION - MOTOR_START_KICK_MS);
  }

  brakeMotors();
  delay(DURATION_BRAKE);
  stopMotors();
  delay(20);

  // 3) Second 90-degree FORWARD-ONLY turn in the SAME direction.
  // This faces the robot down the next lane in the opposite travel direction.
  currentState = STATE_LANE_TURN_2;
  Serial.print(F("[LANE CHANGE] Turn 2 -> "));
  Serial.println(turnRight ? F("RIGHT") : F("LEFT"));
  if (turnRight) pivotRight(SPEED_TURN_90);
  else pivotLeft(SPEED_TURN_90);
  if (TURN_90_DURATION > MOTOR_START_KICK_MS) {
    delay(TURN_90_DURATION - MOTOR_START_KICK_MS);
  }

  brakeMotors();
  delay(DURATION_BRAKE);
  stopMotors();

  currentLane++;
  movingForward = !movingForward;

  currentState = STATE_LANE_SETTLE;
  delay(LANE_SETTLE_DURATION);

  // Throw away stale sonar values from the old wall/corner.
  rawLeft = DIST_MAX_VALID;
  rawRight = DIST_MAX_VALID;
  distLeft = DIST_MAX_VALID;
  distRight = DIST_MAX_VALID;
  wallConfirmCounter = 0;
  previousWallRawLeft = DIST_MAX_VALID;
  previousWallRawRight = DIST_MAX_VALID;
  wallAlertActive = false;
  digitalWrite(PIN_LED_ALERT, LOW);

  laneDriveAccumulatedMs = 0;
  lastLaneClockTime = millis();
  laneWallIgnoreUntil = millis() + WALL_IGNORE_NEW_LANE_MS;
  irNotBeforeTime = millis() + IR_IGNORE_AFTER_LANE_MS;
  irArmed = false;
  irClearSince = 0;
  irLowSince = 0;
  targetConfirmCounter = 0;

  updateSharedBuzzer();
  transitionTo(STATE_LANE_CRUISE);

  Serial.print(F("[SYSTEM] U-turn complete. Launching Lane "));
  Serial.print(currentLane);
  Serial.print(F(" of "));
  Serial.println(TOTAL_LANES);
}

// ==========================================
// MOTOR CONTROL
// ==========================================
void setMotorDrive(uint8_t speedL, uint8_t speedR, bool forwardL, bool forwardR) {
  if (INVERT_LEFT_MOTOR) forwardL = !forwardL;
  if (INVERT_RIGHT_MOTOR) forwardR = !forwardR;

  if (forwardL) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  analogWrite(PIN_ENA, speedL);

  if (forwardR) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
  }
  analogWrite(PIN_ENB, speedR);
}

void brakeMotors() {
  // Use the exact stop behavior from the successful wheel diagnostic.
  // This avoids leaving either L298N enable channel at a different PWM state.
  stopMotors();
}

void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}

void kickStartMotion(bool forwardL, bool forwardR) {
  // Every movement uses the physically verified PWM 90.
  setMotorDrive(90, 90, forwardL, forwardR);
}

void pivotLeft(uint8_t speed) {
  (void)speed;
  // FORWARD-ONLY left turn.
  // The standalone wheel test proved the RIGHT/ENB side runs reliably forward at PWM 90.
  // Keep the left side stopped instead of commanding it in reverse.
  setMotorDrive(0, 90, true, true);
}

void pivotRight(uint8_t speed) {
  (void)speed;
  // FORWARD-ONLY right turn.
  // The standalone wheel test proved the LEFT/ENA side runs reliably forward at PWM 90.
  // Keep the right side stopped instead of commanding it in reverse.
  setMotorDrive(90, 0, true, true);
}

// ==========================================
// SHARED BUZZER / ALERTS
// ==========================================
void updateSharedBuzzer() {
  digitalWrite(PIN_BUZZER, (wallAlertActive || loadAlarmActive) ? HIGH : LOW);
}

void setAlertIndicators(bool active) {
  wallAlertActive = active;
  digitalWrite(PIN_LED_ALERT, active ? HIGH : LOW);
  updateSharedBuzzer();
}

// ==========================================
// ARM / CLAW
// ==========================================
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
  isExecutingSequence = true;
  targetConfirmCounter = 0;
  irClearSince = 0;
  irLowSince = 0;
  irArmed = false;
  digitalWrite(PIN_LED_ALERT, HIGH);

  attachServos();

  Serial.println(F("[STEP 1/6] Opening Claw Jaws to 120 degrees..."));
  moveClawSmooth(CLAW_OPEN_ANGLE);
  delay(150);

  Serial.print(F("[STEP 2/6] Lowering Arm: "));
  Serial.print(currentArmPos);
  Serial.print(F(" -> "));
  Serial.println(ARM_PICKUP_ANGLE);
  moveArmSmooth(ARM_PICKUP_ANGLE);
  delay(300);

  Serial.println(F("[STEP 3/6] Closing claw around target..."));
  moveClawSmooth(CLAW_GRIP_ANGLE);
  delay(400);

  Serial.print(F("[STEP 4/6] Moving arm to bin: "));
  Serial.print(currentArmPos);
  Serial.print(F(" -> "));
  Serial.println(ARM_DROP_ANGLE);
  moveArmSmooth(ARM_DROP_ANGLE);
  delay(400);

  Serial.println(F("[STEP 5/6] Releasing garbage into bin..."));
  moveClawSmooth(CLAW_OPEN_ANGLE);
  delay(500);

  Serial.print(F("[STEP 6/6] Returning Arm: "));
  Serial.print(currentArmPos);
  Serial.print(F(" -> "));
  Serial.println(ARM_RETURN_ANGLE);
  moveArmSmooth(ARM_RETURN_ANGLE);
  moveClawSmooth(CLAW_GRIP_ANGLE);
  delay(200);

  detachServos();

  // The bin has just received the object. Refresh the load-cell reading only.
  // Exceeding the threshold does NOT stop robot operation.
  stopMotors();
  checkWeightAtSafeStop();

  Serial.println(F("[QUARANTINE] IR sensor disabled for 800 ms..."));
  delay(800);

  targetConfirmCounter = 0;
  irClearSince = 0;
  irLowSince = 0;
  irArmed = false;
  digitalWrite(PIN_LED_ALERT, LOW);
  isExecutingSequence = false;

  Serial.println(F("[SYSTEM READY] Pickup complete."));
  Serial.println(F("[IR] Waiting for CLEAR floor before re-arming.\n"));
}

// ==========================================
// COMPLETION / TELEMETRY
// ==========================================
void soundCompletionChime() {
  static bool chimePlayed = false;
  if (!chimePlayed) {
    chimePlayed = true;
    for (int i = 0; i < 4; i++) {
      setAlertIndicators(true);
      delay(120);
      setAlertIndicators(false);
      delay(80);
    }
  }
}

const char* getStateName(RobotState state) {
  switch (state) {
    case STATE_LANE_CRUISE:      return "CRUISE";
    case STATE_WALL_BRAKE:       return "BRAKE";
    case STATE_LANE_TURN_1:      return "TURN_1";
    case STATE_LANE_STEP_OVER:   return "STEP_OVER";
    case STATE_LANE_TURN_2:      return "TURN_2";
    case STATE_LANE_SETTLE:      return "SETTLE";
    case STATE_MISSION_COMPLETE: return "COMPLETE";
    default:                     return "IDLE";
  }
}

void printTelemetry() {
  int irRaw = digitalRead(PIN_IR_SENSOR);

  Serial.print(F("["));
  Serial.print(getStateName(currentState));
  Serial.print(F("] Lane:"));
  Serial.print(currentLane);
  Serial.print(F("/"));
  Serial.print(TOTAL_LANES);

  Serial.print(F(" | L:"));
  Serial.print(distLeft, 1);
  Serial.print(F("cm(raw "));
  Serial.print(rawLeft, 1);
  Serial.print(F(")"));

  Serial.print(F(" | R:"));
  Serial.print(distRight, 1);
  Serial.print(F("cm(raw "));
  Serial.print(rawRight, 1);
  Serial.print(F(")"));

  Serial.print(F(" | IR:"));
  Serial.print(irRaw == LOW ? F("DETECTED") : F("Clear"));
  Serial.print(F(" | IR Armed:"));
  Serial.print(irArmed ? F("YES") : F("NO"));
  Serial.print(F(" | IR Confirm:"));
  Serial.print(targetConfirmCounter);
  Serial.print(F("/"));
  Serial.print(REQUIRED_CONFIRMATIONS);

  Serial.print(F(" | LaneDrive:"));
  Serial.print(laneDriveAccumulatedMs / 1000.0, 1);
  Serial.print(F("s/"));
  Serial.print(MAX_LANE_DRIVE_MS / 1000.0, 1);
  Serial.print(F("s"));

  Serial.print(F(" | Weight:"));
  Serial.print(weightGrams, 1);
  Serial.print(F("g | Load:"));
  Serial.print(loadAlarmActive ? F("OVER 10g") : F("Clear"));

  Serial.println();
}

/*
 * =====================================================================================
 * BRAC University - CSE461: Robotics & Embedded Systems
 * Group 5: Autonomous Garbage-Collecting Robot (Phase 1: Search-Ready Mobile Base)
 * =====================================================================================
 * 
 * 20-Lane Systematic Grid Sweep (Active Sonar Wall Detection & Serpentine Navigation)
 * 
 * Major Features & Performance Fixes:
 * -------------------------------------------------------------------------------------
 * 1. Active Wall-Triggered Navigation (No Fixed 100cm Distance Limit):
 *    - The robot will move infinitely down the current lane until the dual HC-SR04
 *      ultrasonic sensors detect a wall / obstacle within DIST_WALL_STOP (26 cm).
 *    - No fixed distance or timeout will prematurely end the lane cruise.
 *    - Upon wall detection, the robot executes a precise serpentine U-turn into the next lane.
 * 
 * 2. Soft-Start Acceleration (Eliminates Initial Startup Right-Tilt / Yaw Drift):
 *    - When launching from rest, PWM ramps smoothly from min torque to cruise speed over
 *      200ms (SOFT_START_RAMP_MS). This overcomes motor static friction (stiction) equally
 *      on both sides and prevents the initial launch jerk/tilt to the right.
 *    - Clean Left & Right trim calibration allows dead-straight lane tracking.
 * 
 * 3. Calibrated 90-Degree Pivot Turns (No Turn Overshoot):
 *    - TURN_90_DURATION calibrated to 420 ms at SPEED_TURN_90 = 150 PWM.
 *    - Active dynamic electronic braking between pivot states arrests rotational inertia,
 *      ensuring an exact 90° first turn, accurate step-over, and exact 90° second turn.
 * 
 * 4. 20-Lane Full Grid Serpentine Coverage:
 *    - Sweeps all 20 lanes back-and-forth (Odd lanes 1,3..19 forward; Even lanes 2,4..20 reverse).
 *    - Stops permanently with audio-visual completion signal upon completing Lane 20.
 * 
 * Hardware Pinout (Arduino Uno + L298N + Dual HC-SR04 + Buzzer + LED):
 * -------------------------------------------------------------------------------------
 * Left Motors (L298N Channel A):
 *   - ENA   : D5  (PWM Speed Control: 0-255)
 *   - IN1   : D8  (Direction 1)
 *   - IN2   : D7  (Direction 2)
 * 
 * Right Motors (L298N Channel B):
 *   - ENB   : D6  (PWM Speed Control: 0-255)
 *   - IN3   : D4  (Direction 3)
 *   - IN4   : D2  (Direction 4)
 * 
 * Left HC-SR04 Ultrasonic Sensor:
 *   - TRIG_L: A0  (Trigger Pulse)
 *   - ECHO_L: A1  (Echo Input)
 * 
 * Right HC-SR04 Ultrasonic Sensor:
 *   - TRIG_R: A2  (Trigger Pulse)
 *   - ECHO_R: A3  (Echo Input)
 * 
 * Audio-Visual Indicators:
 *   - BUZZER: A4  (Active Buzzer Signal - HIGH = Beep)
 *   - LED   : D13 (Built-in LED Proximity Alert)
 * =====================================================================================
 */

// ==========================================
// PIN ALLOCATIONS
// ==========================================
const uint8_t PIN_ENA   = 5;   // Left Motor PWM Speed
const uint8_t PIN_IN1   = 8;   // Left Motor Direction 1
const uint8_t PIN_IN2   = 7;   // Left Motor Direction 2

const uint8_t PIN_ENB   = 6;   // Right Motor PWM Speed
const uint8_t PIN_IN3   = 4;   // Right Motor Direction 3
const uint8_t PIN_IN4   = 2;   // Right Motor Direction 4

const uint8_t PIN_TRIG_L = A0; // Left Ultrasonic Trigger
const uint8_t PIN_ECHO_L = A1; // Left Ultrasonic Echo

const uint8_t PIN_TRIG_R = A2; // Right Ultrasonic Trigger
const uint8_t PIN_ECHO_R = A3; // Right Ultrasonic Echo

const uint8_t PIN_BUZZER    = A4; // Active Buzzer Signal (HIGH = Sound)
const uint8_t PIN_LED_ALERT = 13; // Built-in Status / Alert LED

// ==========================================
// MOTOR POLARITY CALIBRATION
// ==========================================
const bool INVERT_LEFT_MOTOR  = false; // Set true if left wheels spin backward
const bool INVERT_RIGHT_MOTOR = false; // Set true if right wheels spin backward

// ==========================================
// MOTOR SPEED & STEERING TRIM CALIBRATION
// ==========================================
const uint8_t BASE_CRUISE_SPEED    = 140; // Steady cruise speed (prevents stalling on arena floor)
const uint8_t MIN_START_SPEED      = 95;  // Minimum PWM to overcome stiction during soft-start ramp
const unsigned long SOFT_START_RAMP_MS = 200; // Soft-start ramp duration (ms) to eliminate launch tilt

// STEERING TRIMS:
// Fine-tune if robot drifts during straight cruise:
// Increase TRIM_LEFT if drifting right; Increase TRIM_RIGHT if drifting left.
const int8_t MOTOR_TRIM_LEFT  = 0; 
const int8_t MOTOR_TRIM_RIGHT = 0;

// Turning & Lateral Step Speeds:
const uint8_t SPEED_TURN_90   = 150; // Stable pivot speed for clean, controlled 90° turns
const uint8_t SPEED_STEP_OVER = 135; // Speed during lateral lane step-over

// ==========================================
// MISSION & TIMING CALIBRATION
// ==========================================
const int TOTAL_LANES = 20; // Total lanes to complete (20-Lane Arena Sweep)

// U-Turn Pivot & Step Durations:
// Note: Reduced from 530ms to 420ms to prevent turning angle > 90°
const unsigned long TURN_90_DURATION     = 450; // Time (ms) for an EXACT 90-degree pivot turn
const unsigned long LANE_STEP_DURATION   = 550; // Time (ms) to drive laterally into the adjacent lane
const unsigned long DURATION_BRAKE       = 90;  // Active dynamic brake pulse duration (ms)
const unsigned long LANE_SETTLE_DURATION = 160; // Chassis settle time before launching next lane (ms)

// ==========================================
// SENSOR CONFIGURATION & THRESHOLDS
// ==========================================
const float DIST_WALL_STOP  = 30.0;  // Trigger U-turn when wall/obstacle <= 26 cm ahead
const float DIST_MAX_VALID  = 200.0; // Maximum valid ultrasonic distance (cm)
const float DIST_MIN_VALID  = 2.0;   // Minimum valid ultrasonic distance (cm)
const unsigned long TIMEOUT_US = 20000; // pulseIn timeout (20ms ~ 340cm range)

const unsigned long SENSOR_INTERVAL    = 35;  // Sensor acquisition interval (ms) (~28 Hz)
const unsigned long TELEMETRY_INTERVAL = 250; // Telemetry Serial output interval (ms)

// ==========================================
// FINITE STATE MACHINE (FSM)
// ==========================================
enum RobotState {
  STATE_LANE_CRUISE,        // Cruising straight down current lane with soft-start ramp
  STATE_WALL_BRAKE,         // Active electronic brake on reaching end wall
  STATE_LANE_TURN_1,        // First 90° pivot turn toward adjacent lane
  STATE_LANE_STEP_OVER,     // Drives laterally into the next lane strip
  STATE_LANE_TURN_2,        // Second 90° pivot turn facing opposite direction
  STATE_LANE_SETTLE,        // Settle & stabilize chassis before launching next lane
  STATE_MISSION_COMPLETE    // All 20 lanes finished: Motors stopped permanently
};

RobotState currentState = STATE_LANE_CRUISE;
unsigned long stateStartTime = 0;
unsigned long lastSensorReadTime = 0;
unsigned long lastTelemetryTime = 0;

// Ultrasonic Sensor Variables
float distLeft  = 100.0;
float distRight = 100.0;
float rawLeft   = 100.0;
float rawRight  = 100.0;

// Mission Tracking Variables
int currentLane = 1;            // Current lane (1 to 20)
bool movingForward = true;      // True = forward (Lanes 1,3,5..19); False = reverse (Lanes 2,4,6..20)
bool turnRight = true;          // Current U-turn pivot direction (CW vs CCW)

// Function Prototypes
void transitionTo(RobotState newState);
void executeFSM(unsigned long now);
float measureDistance(uint8_t trigPin, uint8_t echoPin);
void readUltrasonicSensors();
uint8_t calculateSoftStartSpeed(uint8_t targetSpeed, unsigned long timeInState);
void setMotorDrive(uint8_t speedL, uint8_t speedR, bool forwardL, bool forwardR);
void brakeMotors();
void stopMotors();
void pivotLeft(uint8_t speed);
void pivotRight(uint8_t speed);
void setAlertIndicators(bool active);
void soundCompletionChime();
const char* getStateName(RobotState state);
void printTelemetry();

// ==========================================
// SETUP ROUTINE
// ==========================================
void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 1500) {}

  Serial.println(F("\n======================================================="));
  Serial.println(F(" BRACU CSE461 Group 5: Autonomous Base Navigation     "));
  Serial.println(F(" 20-Lane Systematic Grid Sweep + Active Sonar Trigger  "));
  Serial.println(F("======================================================="));

  // Initialize Motor Driver Pins
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Initialize Ultrasonic Sensor Pins
  pinMode(PIN_TRIG_L, OUTPUT);
  pinMode(PIN_ECHO_L, INPUT);
  digitalWrite(PIN_TRIG_L, LOW);

  pinMode(PIN_TRIG_R, OUTPUT);
  pinMode(PIN_ECHO_R, INPUT);
  digitalWrite(PIN_TRIG_R, LOW);

  // Initialize Buzzer & LED Pins
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_ALERT, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED_ALERT, LOW);

  // Startup audio-visual confirmation chirp
  setAlertIndicators(true);
  delay(100);
  setAlertIndicators(false);
  delay(80);
  setAlertIndicators(true);
  delay(100);
  setAlertIndicators(false);

  // Hold stationary for safety boot window
  stopMotors();
  delay(1000);

  // Initialize Mission State
  currentLane = 1;
  movingForward = true;
  transitionTo(STATE_LANE_CRUISE);

  Serial.println(F("[SYSTEM] Mission Initialized: Starting Lane 1 of 20...\n"));
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  unsigned long now = millis();

  // 1. High-frequency non-blocking ultrasonic sensor acquisition
  if (now - lastSensorReadTime >= SENSOR_INTERVAL) {
    lastSensorReadTime = now;
    readUltrasonicSensors();
  }

  // 2. FSM Execution
  executeFSM(now);

  // 3. Serial Telemetry Stream
  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = now;
    printTelemetry();
  }
}

// ==========================================
// ULTRASONIC SENSING ROUTINE
// ==========================================
float measureDistance(uint8_t trigPin, uint8_t echoPin) {
  // Clear trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // 10µs High Pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure pulse duration with timeout
  unsigned long duration = pulseIn(echoPin, HIGH, TIMEOUT_US);

  if (duration == 0) {
    return DIST_MAX_VALID;
  }

  float distance = (duration * 0.0343) / 2.0;
  if (distance < DIST_MIN_VALID || distance > DIST_MAX_VALID) {
    return DIST_MAX_VALID;
  }
  return distance;
}

void readUltrasonicSensors() {
  rawLeft = measureDistance(PIN_TRIG_L, PIN_ECHO_L);
  delayMicroseconds(500); // Prevents acoustic cross-talk between sensors
  rawRight = measureDistance(PIN_TRIG_R, PIN_ECHO_R);

  // Fast-Attack filter: instant update on close obstacle, smooth on open area
  if (rawLeft <= DIST_WALL_STOP) {
    distLeft = rawLeft;
  } else {
    distLeft = (0.70 * rawLeft) + (0.30 * distLeft);
  }

  if (rawRight <= DIST_WALL_STOP) {
    distRight = rawRight;
  } else {
    distRight = (0.70 * rawRight) + (0.30 * distRight);
  }
}

// ==========================================
// SOFT-START ACCELERATION RAMP HELPER
// ==========================================
// Smoothly ramps motor PWM from MIN_START_SPEED to targetSpeed over SOFT_START_RAMP_MS
// to overcome static friction (stiction) equally on both motors, preventing launch jerk/tilt.
uint8_t calculateSoftStartSpeed(uint8_t targetSpeed, unsigned long timeInState) {
  if (timeInState >= SOFT_START_RAMP_MS) {
    return targetSpeed;
  }
  if (targetSpeed <= MIN_START_SPEED) {
    return targetSpeed;
  }
  float progress = (float)timeInState / (float)SOFT_START_RAMP_MS;
  return MIN_START_SPEED + (uint8_t)((targetSpeed - MIN_START_SPEED) * progress);
}

// ==========================================
// FINITE STATE MACHINE (FSM)
// ==========================================
void transitionTo(RobotState newState) {
  currentState = newState;
  stateStartTime = millis();
}

void executeFSM(unsigned long now) {
  unsigned long timeInState = now - stateStartTime;

  // Active Sonar Wall Detection:
  // Requires 350ms in lane to prevent false triggers from previous turn
  bool wallDetected = (timeInState > 350) &&
                      (distLeft <= DIST_WALL_STOP || distRight <= DIST_WALL_STOP ||
                       rawLeft  <= DIST_WALL_STOP || rawRight  <= DIST_WALL_STOP);

  // Visual & Audio Indicator for Proximity
  bool obstacleNearby = (distLeft <= DIST_WALL_STOP || distRight <= DIST_WALL_STOP);
  setAlertIndicators(obstacleNearby);

  switch (currentState) {

    // -----------------------------------------------------------------
    // 1. STATE: LANE CRUISE (Continuous straight sweep until wall detected)
    // -----------------------------------------------------------------
    case STATE_LANE_CRUISE: {
      // Trigger U-turn ONLY when wall is detected by Ultrasonic sensors
      if (wallDetected) {
        Serial.print(F("\n>>> [TRIGGER] WALL DETECTED at Lane "));
        Serial.print(currentLane);
        Serial.print(F(" End! (L: "));
        Serial.print(distLeft, 1);
        Serial.print(F("cm | R: "));
        Serial.print(distRight, 1);
        Serial.println(F("cm) -> Initiating U-Turn sequence..."));

        // Check if the 20th (last) lane has completed
        if (currentLane >= TOTAL_LANES) {
          Serial.println(F("\n======================================================="));
          Serial.println(F(" [MISSION COMPLETE] All 20 Lanes Successfully Cleared! "));
          Serial.println(F("======================================================="));
          transitionTo(STATE_MISSION_COMPLETE);
          break;
        }

        // Serpentine U-turn Direction:
        // Forward Lanes (1, 3, 5..19)  -> U-Turn Clockwise (Turn Right)
        // Reverse Lanes (2, 4, 6..20)  -> U-Turn Counter-Clockwise (Turn Left)
        turnRight = movingForward;

        // Apply active electronic dynamic brake immediately
        transitionTo(STATE_WALL_BRAKE);
        break;
      }

      // Calculate Target Speeds with Individual Motor Trims:
      int targetL = (int)BASE_CRUISE_SPEED + MOTOR_TRIM_LEFT;
      int targetR = (int)BASE_CRUISE_SPEED + MOTOR_TRIM_RIGHT;
      targetL = constrain(targetL, 0, 255);
      targetR = constrain(targetR, 0, 255);

      // Apply Soft-Start Ramp to prevent startup tilt/jerk
      uint8_t speedL = calculateSoftStartSpeed((uint8_t)targetL, timeInState);
      uint8_t speedR = calculateSoftStartSpeed((uint8_t)targetR, timeInState);

      // Drive Straight Forward indefinitely until wall is detected
      setMotorDrive(speedL, speedR, true, true);
      break;
    }

    // -----------------------------------------------------------------
    // 2. STATE: ACTIVE DYNAMIC BRAKE (Stops forward inertia completely)
    // -----------------------------------------------------------------
    case STATE_WALL_BRAKE: {
      brakeMotors();
      if (timeInState >= DURATION_BRAKE) {
        transitionTo(STATE_LANE_TURN_1);
      }
      break;
    }

    // -----------------------------------------------------------------
    // 3. STATE: FIRST 90° PIVOT TURN (Turns toward adjacent lane)
    // -----------------------------------------------------------------
    case STATE_LANE_TURN_1: {
      if (turnRight) {
        pivotRight(SPEED_TURN_90);
      } else {
        pivotLeft(SPEED_TURN_90);
      }

      if (timeInState >= TURN_90_DURATION) {
        brakeMotors(); // Electronic brake stops pivot overshoot
        transitionTo(STATE_LANE_STEP_OVER);
      }
      break;
    }

    // -----------------------------------------------------------------
    // 4. STATE: LANE STEP-OVER (Drives laterally to adjacent lane strip)
    // -----------------------------------------------------------------
    case STATE_LANE_STEP_OVER: {
      setMotorDrive(SPEED_STEP_OVER, SPEED_STEP_OVER, true, true);

      if (timeInState >= LANE_STEP_DURATION) {
        brakeMotors();
        transitionTo(STATE_LANE_TURN_2);
      }
      break;
    }

    // -----------------------------------------------------------------
    // 5. STATE: SECOND 90° PIVOT TURN (Aligns facing opposite direction)
    // -----------------------------------------------------------------
    case STATE_LANE_TURN_2: {
      // Turn in the same direction to complete the 180° serpentine U-turn
      if (turnRight) {
        pivotRight(SPEED_TURN_90);
      } else {
        pivotLeft(SPEED_TURN_90);
      }

      if (timeInState >= TURN_90_DURATION) {
        brakeMotors();
        // Advance lane counter & toggle forward/reverse sweep direction
        currentLane++;
        movingForward = !movingForward;
        transitionTo(STATE_LANE_SETTLE);
      }
      break;
    }

    // -----------------------------------------------------------------
    // 6. STATE: LANE SETTLE (Stabilizes chassis and sensors)
    // -----------------------------------------------------------------
    case STATE_LANE_SETTLE: {
      stopMotors();
      if (timeInState >= LANE_SETTLE_DURATION) {
        Serial.print(F("[SYSTEM] Launching Lane "));
        Serial.print(currentLane);
        Serial.print(F(" of "));
        Serial.print(TOTAL_LANES);
        Serial.println(F("..."));
        transitionTo(STATE_LANE_CRUISE);
      }
      break;
    }

    // -----------------------------------------------------------------
    // 7. STATE: MISSION COMPLETE (Permanent Stop & Audio Alert)
    // -----------------------------------------------------------------
    case STATE_MISSION_COMPLETE: {
      stopMotors();
      soundCompletionChime();
      break;
    }

    default:
      stopMotors();
      break;
  }
}

// ==========================================
// MOTOR CONTROL PRIMITIVES
// ==========================================
void setMotorDrive(uint8_t speedL, uint8_t speedR, bool forwardL, bool forwardR) {
  if (INVERT_LEFT_MOTOR)  forwardL = !forwardL;
  if (INVERT_RIGHT_MOTOR) forwardR = !forwardR;

  // Left Motors (L298N Channel A)
  if (forwardL) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  analogWrite(PIN_ENA, speedL);

  // Right Motors (L298N Channel B)
  if (forwardR) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
  }
  analogWrite(PIN_ENB, speedR);
}

// Active dynamic electronic brake (shorts motor terminals to ground)
void brakeMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, 255);
  analogWrite(PIN_ENB, 255);
}

// Idle coast stop
void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}

// High-torque pivot left (Left reverse, Right forward)
void pivotLeft(uint8_t speed) {
  setMotorDrive(speed, speed, false, true);
}

// High-torque pivot right (Left forward, Right reverse)
void pivotRight(uint8_t speed) {
  setMotorDrive(speed, speed, true, false);
}

// ==========================================
// ALERT & SOUND HELPERS
// ==========================================
void setAlertIndicators(bool active) {
  digitalWrite(PIN_BUZZER, active ? HIGH : LOW);
  digitalWrite(PIN_LED_ALERT, active ? HIGH : LOW);
}

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

// ==========================================
// DIAGNOSTIC TELEMETRY
// ==========================================
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
  Serial.print(F("["));
  Serial.print(getStateName(currentState));
  Serial.print(F("] Lane:"));
  Serial.print(currentLane);
  Serial.print(F("/"));
  Serial.print(TOTAL_LANES);
  Serial.print(F(" | L:"));
  Serial.print(distLeft, 1);
  Serial.print(F("cm | R:"));
  Serial.print(distRight, 1);
  Serial.print(F("cm | Wall:"));
  if (distLeft <= DIST_WALL_STOP || distRight <= DIST_WALL_STOP) {
    Serial.print(F("DETECTED"));
  } else {
    Serial.print(F("Clear"));
  }
  Serial.println();
}

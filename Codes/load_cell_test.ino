/*
 * =====================================================================================
 * BRAC University - CSE461: Robotics & Embedded Systems
 * Group 5: Standalone HX711 + 1kg Load Cell Diagnostic (Inverted Polarity Fixed)
 * =====================================================================================
 * 
 * Pin Configuration:
 * -------------------------------------------------------------------------------------
 *  - HX711 DT (Data)   : Pin D9
 *  - HX711 SCK (Clock) : Pin D10
 *  - Active Buzzer     : Pin A4
 * =====================================================================================
 */

const uint8_t PIN_HX711_DT  = 9;   // Serial Data Pin (D9)
const uint8_t PIN_HX711_SCK = 10;  // Serial Clock Pin (D10)
const uint8_t PIN_BUZZER    = A4;  // Active Buzzer Signal

// Calibration factor: adjusts raw 24-bit ADC counts to grams.
// Increase if displayed weight is too high; decrease if too low.
float CALIBRATION_FACTOR = 400.0; 

const float WEIGHT_THRESHOLD_GRAMS = 10.0;

long tareOffset = 0;

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

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 1500) {}

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  pinMode(PIN_HX711_SCK, OUTPUT);
  pinMode(PIN_HX711_DT, INPUT);

  digitalWrite(PIN_HX711_SCK, LOW);

  Serial.println(F("\n======================================================="));
  Serial.println(F(" BRACU CSE461: Standalone Load Cell Diagnostic Test     "));
  Serial.println(F(" Software Polarity Inverted (+ Values on Load)          "));
  Serial.println(F(" DT: D9 | SCK: D10 | Buzzer: A4 | Threshold: > 10.0 g  "));
  Serial.println(F("======================================================="));
  Serial.println(F("[SYSTEM] Zeroing scale (Tare)... Keep bin completely empty!"));

  // Buzzer chirp on tare start
  digitalWrite(PIN_BUZZER, HIGH);
  delay(100);
  digitalWrite(PIN_BUZZER, LOW);

  delay(500);
  tareOffset = getAverageRaw(10); // Take 10-sample baseline average

  // Double chirp confirmation
  digitalWrite(PIN_BUZZER, HIGH);
  delay(80);
  digitalWrite(PIN_BUZZER, LOW);
  delay(60);
  digitalWrite(PIN_BUZZER, HIGH);
  delay(80);
  digitalWrite(PIN_BUZZER, LOW);

  Serial.println(F("[READY] Tare complete. Place object in the bin...\n"));
}

void loop() {
  long rawValue = getAverageRaw(3);
  
  if (rawValue == 0 && digitalRead(PIN_HX711_DT) == HIGH) {
    Serial.println(F("[WARNING] HX711 not responding. Check D9/D10 wiring and 5V power!"));
    delay(500);
    return;
  }

  // Polarity inverted in software: (tareOffset - rawValue) instead of (rawValue - tareOffset)
  float weightGrams = (float)(tareOffset - rawValue) / CALIBRATION_FACTOR;

  // Filter tiny zero-point resting noise
  if (weightGrams < 0.0 && weightGrams > -1.0) {
    weightGrams = 0.0;
  }

  Serial.print(F("Weight: "));
  Serial.print(weightGrams, 1);
  Serial.print(F(" g | Status: "));

  if (weightGrams >= WEIGHT_THRESHOLD_GRAMS) {
    digitalWrite(PIN_BUZZER, HIGH);
    Serial.println(F(">>> OVER 10g! BUZZER ACTIVE <<<"));
  } else {
    digitalWrite(PIN_BUZZER, LOW);
    Serial.println(F("Clear"));
  }

  delay(200);
}
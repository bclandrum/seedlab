// EENG350 Demo 1 — independent left/right wheel position + velocity control.
// Adapted from the user's original wiring, encoder setup and turn-then-drive state machine.
// This is a TEST VERSION: tune on blocks first. Wheel encoders do not measure floor slip.
#include <Arduino.h>
#include <math.h>

// ===================== CHANGE FOR EACH DEMO RUN =====================
const float DISTANCE_FT = 2.0f;
const float ANGLE_DEG = -45.0f;   // + LEFT, - RIGHT
const unsigned long START_DELAY_MS = 0;

// ===================== YOUR ROBOT'S HARDWARE =====================
const float WHEEL_RADIUS_M = 0.1397f / 2.0f; // 5.5-inch diameter
const float WHEEL_SPACING_M = 0.3175f;       // 12.5-inch wheel spacing
const float TURN_WHEELBASE_M = 0.3362f;
const byte ENABLE = 4;
const byte ENC_A[2] = {2, 3};   // LEFT, RIGHT
const byte ENC_B[2] = {5, 6};
const byte DIR[2] = {7, 8};
const byte PWM[2] = {9, 10};
const int ENCODER_SIGN[2] = {1, 1};
const bool POSITIVE_DIR_HIGH[2] = {true, true};
const float COUNTS_PER_WHEEL_REV = 1600.0f; // Verify by physically rotating one wheel 1 rev
const float RAD_PER_COUNT = 2.0f * PI / COUNTS_PER_WHEEL_REV;
const float BATTERY_V = 7.8f;
const float VOLTAGE_LIMIT = 7.0f;

// ===================== TUNE THESE ON YOUR ROBOT =====================
// Outer position P: wheel position error [rad] -> desired wheel speed [rad/s].
// Inner speed PI: wheel speed error [rad/s] -> voltage [V].
// They are NOT the same units as the gains in your previous sketch.
const float KP_POS = 1.25f;    // conservative initial experiment, not verified
const float KP_VEL_L = 0.873f;  // independent right wheel gain — tune separately
const float KP_VEL_R = 1.1f;  // independent left wheel proportional gain
const float KI_VEL_L = 0.0f;  // left wheel integral gain [V/rad]; enable after P tuning
const float KI_VEL_R = 0.0f;  // right wheel integral gain [V/rad]; enable after P tuning
const float MAX_SPEED_INTEGRAL = 6.0f; // speed-error integral limit [rad]
const float MAX_SPEED_RAD_S = 4.0f; // temporary low-speed test limit
const float TURN_SPEED_RAD_S = 2.0f;

// Allow forward wheel motion only during DRIVING. Turns can spin in both directions.
// This does not prevent coasting after output is turned off.
const float TURN_POS_TOL_RAD = 0.05f;
const float DRIVE_POS_TOL_RAD = 0.15f;
const float WHEEL_SPEED_TOL_RAD_S = 0.3f;
const unsigned long SETTLE_MS = 200;
const unsigned long RUN_TIMEOUT_MS = 58000;

// Turning-only static friction compensation
const float MIN_TURN_VOLTAGE_L = 1.2f;
const float MIN_TURN_VOLTAGE_R = 1.2f;

// ===================== STATES AND ENCODERS =====================
enum State { WAITING, TURNING, DRIVING, STOPPED };
State state = WAITING;
volatile long counts[2] = {0, 0};
long originCounts[2] = {0, 0};
long previousCounts[2] = {0, 0};
float wheelAngle[2] = {0.0f, 0.0f};
float wheelSpeed[2] = {0.0f, 0.0f};
float desiredWheelAngle[2] = {0.0f, 0.0f};
float desiredWheelSpeed[2] = {0.0f, 0.0f};
float commandedVoltage[2] = {0.0f, 0.0f};
float speedIntegral[2] = {0.0f, 0.0f}; // left/right integral of speed error
float rho = 0.0f, phi = 0.0f;
unsigned long bootMs, runStartMs, lastUpdateUs, lastPrintMs;
unsigned long settledSinceMs = 0;
bool settling = false;

void leftEncoder() {
  counts[0] += digitalRead(ENC_A[0]) == digitalRead(ENC_B[0]) ? 1 : -1;
}
void rightEncoder() {
  counts[1] += digitalRead(ENC_A[1]) == digitalRead(ENC_B[1]) ? 1 : -1;
}

void motorsOff() {
  analogWrite(PWM[0], 0);
  analogWrite(PWM[1], 0);
  digitalWrite(ENABLE, LOW);
  commandedVoltage[0] = commandedVoltage[1] = 0.0f;
  speedIntegral[0] = speedIntegral[1] = 0.0f;
}

void setVoltage(byte i, float voltage) {
  voltage = constrain(voltage, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  commandedVoltage[i] = voltage;
  bool high = (voltage >= 0.0f) ? POSITIVE_DIR_HIGH[i] : !POSITIVE_DIR_HIGH[i];
  if (digitalRead(DIR[i]) != (high ? HIGH : LOW)) {
    analogWrite(PWM[i], 0);
    digitalWrite(DIR[i], high ? HIGH : LOW);
  }
  int duty = (int)(255.0f * fabsf(voltage) / BATTERY_V);
  analogWrite(PWM[i], constrain(duty, 0, 255));
}

void updateMeasurements(float dt) {
  long current[2];
  noInterrupts();
  current[0] = counts[0];
  current[1] = counts[1];
  interrupts();
  for (byte i = 0; i < 2; i++) {
    wheelAngle[i] = ENCODER_SIGN[i] * (current[i] - originCounts[i]) * RAD_PER_COUNT;
    float rawSpeed = ENCODER_SIGN[i] * (current[i] - previousCounts[i]) * RAD_PER_COUNT / dt;
    previousCounts[i] = current[i];
    wheelSpeed[i] = 0.8f * wheelSpeed[i] + 0.2f * rawSpeed;
  }
  rho = WHEEL_RADIUS_M * (wheelAngle[0] + wheelAngle[1]) / 2.0f;
  phi = WHEEL_RADIUS_M * (wheelAngle[1] - wheelAngle[0]) / WHEEL_SPACING_M;
}

// TWO outer wheel position P loops + TWO independent inner wheel speed PI loops.
void controlWheels(float dt) {
  for (byte i = 0; i < 2; i++) {
    float positionError = desiredWheelAngle[i] - wheelAngle[i];
    float speedLimit = (state == TURNING) ? TURN_SPEED_RAD_S : MAX_SPEED_RAD_S;
    float desiredSpeed = constrain(KP_POS * positionError, -speedLimit, speedLimit);
    // While driving forward, do not request reverse wheel motion.
    if (state == DRIVING) desiredSpeed = fmaxf(0.0f, desiredSpeed);
    desiredWheelSpeed[i] = desiredSpeed;

    float speedError = desiredSpeed - wheelSpeed[i];
    float kp = (i == 0) ? KP_VEL_L : KP_VEL_R;
    float ki = (i == 0) ? KI_VEL_L : KI_VEL_R;
    float minVoltage = (state == DRIVING) ? 0.0f : -VOLTAGE_LIMIT;

    // Tentative integral, clamped to prevent unbounded accumulation.
    float nextIntegral = constrain(speedIntegral[i] + speedError * dt,
                                   -MAX_SPEED_INTEGRAL, MAX_SPEED_INTEGRAL);
    float tentativeVoltage = kp * speedError + ki * nextIntegral;

    // Conditional integration anti-windup: do not integrate when saturated
    // and the current error would push farther into that saturation.
    bool highSaturated = tentativeVoltage > VOLTAGE_LIMIT;
    bool lowSaturated = tentativeVoltage < minVoltage;
    if (ki > 0.0f && !((highSaturated && speedError > 0.0f) ||
                       (lowSaturated && speedError < 0.0f))) {
      speedIntegral[i] = nextIntegral;
    }

    float voltage = kp * speedError + ki * speedIntegral[i];

    // Help motors overcome static friction near turn target
    if (state == TURNING) {

        float positionError =
            desiredWheelAngle[i] - wheelAngle[i];

        float minTurnVoltage = (i == 0)
            ? MIN_TURN_VOLTAGE_L
            : MIN_TURN_VOLTAGE_R;

        // Only boost voltage when moving toward the target
        if (fabsf(positionError) > TURN_POS_TOL_RAD &&
            voltage * positionError > 0.0f &&
            fabsf(voltage) < minTurnVoltage) {

            voltage = (positionError > 0.0f)
                    ? minTurnVoltage
                    : -minTurnVoltage;
        }
    }

    setVoltage(i, constrain(voltage, minVoltage, VOLTAGE_LIMIT));
  }
}

bool bothWheelsSettled(unsigned long now) {
    bool closeAndSlow = true;

    float posTol = (state == TURNING)
        ? TURN_POS_TOL_RAD
        : DRIVE_POS_TOL_RAD;

    for (byte i = 0; i < 2; i++) {
        if (fabsf(desiredWheelAngle[i] - wheelAngle[i]) > posTol ||
            fabsf(wheelSpeed[i]) > WHEEL_SPEED_TOL_RAD_S) {
            closeAndSlow = false;
        }
    }

    if (!closeAndSlow) {
        settling = false;
        return false;
    }

    if (!settling) {
        settling = true;
        settledSinceMs = now;
    }

    return now - settledSinceMs >= SETTLE_MS;
}

void startDrive() {
  speedIntegral[0] = speedIntegral[1] = 0.0f;
  float forwardWheelRadians = (DISTANCE_FT * 0.96) * 0.3048f / WHEEL_RADIUS_M;
  // Start from where both wheels actually are at turn completion.
  // For a zero-angle run, this starts both targets at the same distance.
  desiredWheelAngle[0] = wheelAngle[0] + forwardWheelRadians;
  desiredWheelAngle[1] = wheelAngle[1] + forwardWheelRadians;
  settling = false;
  state = DRIVING;
  Serial.println(F("DRIVING"));
}

void startRun(unsigned long now) {
  noInterrupts();
  originCounts[0] = counts[0];
  originCounts[1] = counts[1];
  interrupts();
  previousCounts[0] = originCounts[0];
  previousCounts[1] = originCounts[1];
  for (byte i = 0; i < 2; i++) {
    wheelAngle[i] = wheelSpeed[i] = 0.0f;
  }
  rho = phi = 0.0f;
  speedIntegral[0] = speedIntegral[1] = 0.0f;
  runStartMs = now;
  settling = false;
  digitalWrite(ENABLE, HIGH);
  if (ANGLE_DEG == 0.0f) {
    startDrive();
  } else {
    float rotationRad = ANGLE_DEG * PI / 180.0f;
    float wheelTurn = rotationRad * TURN_WHEELBASE_M / (2.0f * WHEEL_RADIUS_M);
    // Positive angle = left turn; left wheel backward, right wheel forward.
    desiredWheelAngle[0] = -wheelTurn;
    desiredWheelAngle[1] = wheelTurn;
    state = TURNING;
    Serial.println(F("TURNING"));
  }
}

void setup() {
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  for (byte i = 0; i < 2; i++) {
    pinMode(DIR[i], OUTPUT);
    pinMode(PWM[i], OUTPUT);
    digitalWrite(DIR[i], LOW);
    analogWrite(PWM[i], 0);
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
  }
  motorsOff();
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), rightEncoder, CHANGE);
  Serial.begin(115200);
  bootMs = millis();
  lastUpdateUs = micros();
  lastPrintMs = bootMs;
  if (!isfinite(DISTANCE_FT) || DISTANCE_FT < 0.0f || DISTANCE_FT > 10.0f ||
      !isfinite(ANGLE_DEG) || BATTERY_V <= 0 || VOLTAGE_LIMIT > BATTERY_V ||
      KP_POS <= 0 || KP_VEL_L <= 0 || KP_VEL_R <= 0 ||
      KI_VEL_L < 0 || KI_VEL_R < 0 || MAX_SPEED_INTEGRAL <= 0 ||
      WHEEL_RADIUS_M <= 0 || WHEEL_SPACING_M <= 0) {
    state = STOPPED;
    Serial.println(F("INVALID SETTINGS; motors stay off"));
  } else {
    Serial.println(F("WAITING 5 s; test on blocks first. Send s or ! for emergency stop."));
  }
}

void loop() {
  for (byte n = 0; n < 16 && Serial.available(); n++) {
    char c = Serial.read();
    if (c == 's' || c == '!') {
      motorsOff();
      state = STOPPED;
      Serial.println(F("MANUAL STOP"));
    }
  }
  unsigned long nowUs = micros();
  unsigned long elapsedUs = nowUs - lastUpdateUs;
  if (elapsedUs < 10000UL) return; // approximately 100 Hz
  float dt = elapsedUs / 1000000.0f;
  lastUpdateUs = nowUs;
  unsigned long now = millis();
  updateMeasurements(dt);

  switch (state) {
    case WAITING:
      if (now - bootMs >= START_DELAY_MS) startRun(now);
      break;
    case TURNING:
    case DRIVING:
      if (now - runStartMs >= RUN_TIMEOUT_MS) {
        motorsOff();
        state = STOPPED;
        Serial.println(F("TIMEOUT"));
        break;
      }
      controlWheels(dt);
      if (bothWheelsSettled(now)) {
        if (state == TURNING && DISTANCE_FT > 0.0f) {
          startDrive();
        } else {
          motorsOff(); // no powered hold and no commanded reverse after drive
          state = STOPPED;
          Serial.println(F("MOTION COMPLETE; MOTORS OFF"));
        }
      }
      break;
    case STOPPED:
      break;
  }

  if (now - lastPrintMs >= 200UL) {
    lastPrintMs = now;
    Serial.print(F("state=")); Serial.print((int)state);
    Serial.print(F(" dist_ft=")); Serial.print(rho / 0.3048f, 3);
    Serial.print(F(" heading_deg=")); Serial.print(phi * 180.0f / PI, 2);
    Serial.print(F(" Lpos=")); Serial.print(wheelAngle[0], 3);
    Serial.print(F(" Rpos=")); Serial.print(wheelAngle[1], 3);
    Serial.print(F(" Lspeed=")); Serial.print(wheelSpeed[0], 3);
    Serial.print(F(" Rspeed=")); Serial.print(wheelSpeed[1], 3);
    Serial.print(F(" Ltarget=")); Serial.print(desiredWheelAngle[0], 3);
    Serial.print(F(" Rtarget=")); Serial.print(desiredWheelAngle[1], 3);
    Serial.print(F(" Lvolt=")); Serial.print(commandedVoltage[0], 2);
    Serial.print(F(" Rvolt=")); Serial.print(commandedVoltage[1], 2);
    Serial.print(F(" LI=")); Serial.print(speedIntegral[0], 3);
    Serial.print(F(" RI=")); Serial.println(speedIntegral[1], 3);
  }
}

// Demo 2: Search, move and optional turn
// Team 1

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

// ==================== I2C COMMUNICATION ====================
const byte I2C_ADDRESS = 8;
const byte VISION_PACKET_SIZE = 13; 
const byte DIR_LEFT = 0, DIR_RIGHT = 1, DIR_STOP = 2, DIR_UNKNOWN = 255;
const byte STATUS_VERSION = 3;
const unsigned long VISION_TIMEOUT_MS = 500;
const unsigned long MARKER_LOST_GRACE_MS = 1200;
const float CAMERA_FORWARD_M = 0.0f;
const float CAMERA_LEFT_M = 0.0f;
const float CAMERA_MARKER_HEIGHT_DIFFERENCE_M = 0.0f;

// ==================== RUN SETTINGS ====================
const bool COMMUNICATION_TEST_ONLY = false;
const byte TRIAL_TYPE = 2; // 1 = approach and stop; 2 = approach and turn.
const unsigned long START_DELAY_MS = 0;
const float STOP_DISTANCE_M = 1.3f * 0.3048f;
const float ARRIVAL_LIMIT_M = 1.5f * 0.3048f;
const float SEARCH_YAW_RAD_S = 0.68f; 
const float ALIGN_GAIN = 0.020f; 
const float MAX_YAW_RAD_S = 0.40f;
const float ALIGN_TOLERANCE_DEG = 4.0f;
const float APPROACH_SPEED_M_S = 0.4f; 
const float STOPPED_SPEED_RAD_S = 0.65f;
const byte CONFIRM_FRAMES = 3; 

// ==================== HARDWARE AND CALIBRATION ====================
const byte ENABLE = 4;
const byte ENC_A[2] = {2, 3};
const byte ENC_B[2] = {5, 6};
const byte DIR[2] = {7, 8};
const byte PWM[2] = {9, 10};
const int ENCODER_SIGN[2] = {1, 1};
const bool POSITIVE_DIR_HIGH[2] = {true, true};
const float COUNTS_PER_WHEEL_REV = 1600.0f;
const float RAD_PER_COUNT = 2.0f * PI / COUNTS_PER_WHEEL_REV;
const float WHEEL_RADIUS_M = 0.1397f / 2.0f;
const float TURN_WHEELBASE_M = 0.3362f; 
const float BATTERY_V = 7.8f;
const float VOLTAGE_LIMIT = 7.0f;

// ==================== WHEEL CONTROL ====================
const float KP_POS = 5.0f; 
const float KI_POS = 0.0f;  
const float KP_VEL[2] = {1.978f, 2.2722f};
const float INTEGRAL_SPEED_LIMIT = 10.0f; 
const float DRIVE_SPEED_RAD_S = 6.0f;
const float TURN_SPEED_RAD_S = 3.0f;
const float LEFT_TURN_TARGET_DEG = 95.0f; 
const float RIGHT_TURN_TARGET_DEG = 90.0f;
const float HOLD_SPEED_RAD_S = 0.7f;
const float POSITION_THRESHOLD_RAD = 0.08f; 
const byte SETTLE_COUNT = 25; 
const unsigned long CONTROL_US = 10000UL;
const unsigned long RUN_TIMEOUT_MS = 58000UL;

// ==================== STATE AND ENCODERS ====================
enum State { SEARCHING, ALIGNING, APPROACHING, FINAL_TURN, DONE };
State state = SEARCHING;
bool runStarted = false, braking = false, atBeacon = false, runAborted = false;
volatile long counts[2] = {0, 0};
long previousCounts[2] = {0, 0};
float actualPos[2] = {0, 0}, desiredPos[2] = {0, 0};
float actualSpeed[2] = {0, 0}, desiredSpeed[2] = {0, 0};
float filteredSpeed[2] = {0, 0};
float integralError[2] = {0, 0};
byte settleCounter = 0;
byte coastStopCounter = 0;
unsigned long bootMs, runStartMs, lastUpdateUs, lastPrintMs;
byte arrivalFrames = 0, arrowFrames = 0;
byte confirmedArrow = DIR_UNKNOWN, previousArrow = DIR_UNKNOWN;
bool newVisionThisTick = false;
unsigned long lastMarkerVisibleMs = 0;
float smoothedBearingDeg = 0;
bool bearingInitialized = false;

// ==================== I2C DATA AND CALLBACKS ====================
volatile byte pendingVision[VISION_PACKET_SIZE];
volatile byte pendingVisionLength = 0;
volatile unsigned long visionPacketsSeen = 0;
volatile unsigned long visionPacketsRejected = 0;
unsigned long visionPacketsInvalid = 0;
volatile bool visionPending = false;
volatile unsigned long pendingVisionMs = 0;
volatile byte statusReply[4] = {STATUS_VERSION, 0, SEARCHING, 0};
bool visionReceived = false, beaconDetected = false;
byte visionSequence = 0, arrowDirection = DIR_UNKNOWN;
float beaconDistanceM = 0, beaconAngleDeg = 0;
unsigned long lastVisionMs = 0;

#include "VisionI2C.h"

void leftEncoder() {
  counts[0] += digitalRead(ENC_A[0]) == digitalRead(ENC_B[0]) ? 1 : -1;
}
void rightEncoder() {
  counts[1] += digitalRead(ENC_A[1]) == digitalRead(ENC_B[1]) ? 1 : -1;
}
void motorsOff() {
  analogWrite(PWM[0], 0); analogWrite(PWM[1], 0);
  digitalWrite(ENABLE, LOW);
}
void setVoltage(byte i, float voltage) {
  voltage = constrain(voltage, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  bool high = voltage >= 0 ? POSITIVE_DIR_HIGH[i] : !POSITIVE_DIR_HIGH[i];
  if (digitalRead(DIR[i]) != (high ? HIGH : LOW)) {
    analogWrite(PWM[i], 0);
    digitalWrite(DIR[i], high ? HIGH : LOW);
  }
  int duty = (int)(255.0f * fabsf(voltage) / BATTERY_V);
  analogWrite(PWM[i], constrain(duty, 0, 255));
}
void updateMeasurements(float dt) {
  long current[2];
  noInterrupts(); current[0] = counts[0]; current[1] = counts[1]; interrupts();
  for (byte i = 0; i < 2; i++) {
    actualPos[i] = ENCODER_SIGN[i] * current[i] * RAD_PER_COUNT;
    actualSpeed[i] = ENCODER_SIGN[i] * (current[i] - previousCounts[i]) * RAD_PER_COUNT / dt;
    previousCounts[i] = current[i];
    // Reduce encoder quantization noise at the 10 ms update rate.
    filteredSpeed[i] += 0.25f * (actualSpeed[i] - filteredSpeed[i]);
  }
}

// Zero rotation captures a hold target; positive rotation turns left.
void commandTurn(float rotationRad) {
  float turn = rotationRad * TURN_WHEELBASE_M / (2.0f * WHEEL_RADIUS_M);
  desiredPos[0] = actualPos[0] - turn;
  desiredPos[1] = actualPos[1] + turn;
  integralError[0] = integralError[1] = 0;
  settleCounter = 0;
}
void controlWheels(float dt) {
  float speedLimit = state == FINAL_TURN ? TURN_SPEED_RAD_S : HOLD_SPEED_RAD_S;
  for (byte i = 0; i < 2; i++) {
    float error = desiredPos[i] - actualPos[i];
    // Limit integral buildup when the requested speed saturates.
    if (KI_POS > 0) {
      float limit = INTEGRAL_SPEED_LIMIT / KI_POS;
      float candidate = constrain(integralError[i] + error * dt, -limit, limit);
      float request = KP_POS * error + KI_POS * candidate;
      if (fabsf(request) <= speedLimit || request * error < 0)
        integralError[i] = candidate;
    } else integralError[i] = 0;
    // Small deadband avoids rapid forward/reverse commands near the target.
    if (fabsf(error) < POSITION_THRESHOLD_RAD &&
        fabsf(filteredSpeed[i]) < STOPPED_SPEED_RAD_S) {
      desiredSpeed[i] = 0;
      setVoltage(i, 0);
    } else {
      desiredSpeed[i] = constrain(KP_POS * error + KI_POS * integralError[i],
                                  -speedLimit, speedLimit);
      setVoltage(i, KP_VEL[i] * (desiredSpeed[i] - filteredSpeed[i]));
    }
  }
}
bool relativeMotionComplete() {
  bool close = fabsf(desiredPos[0] - actualPos[0]) < POSITION_THRESHOLD_RAD &&
               fabsf(desiredPos[1] - actualPos[1]) < POSITION_THRESHOLD_RAD &&
               wheelsStopped();
  if (!close) settleCounter = 0;
  else if (settleCounter < SETTLE_COUNT) settleCounter++;
  return settleCounter >= SETTLE_COUNT;
}
// ==================== AUTONOMOUS NAVIGATION ====================
bool wheelsStopped() {
  return fabsf(filteredSpeed[0]) < STOPPED_SPEED_RAD_S &&
         fabsf(filteredSpeed[1]) < STOPPED_SPEED_RAD_S;
}
void changeState(State next) {
  state = next;
  arrivalFrames = 0;
  braking = next == ALIGNING;
  coastStopCounter = 0;
  atBeacon = false;
  
  commandTurn(0);
  Serial.print(F("STATE=")); Serial.println((int)state);
}
void finishRun() {
  changeState(DONE);
  motorsOff();
  digitalWrite(LED_BUILTIN, HIGH);
}
void stopRun() {
  runAborted = true;
  changeState(DONE);
  motorsOff();
  digitalWrite(LED_BUILTIN, LOW);
}
void driveVelocity(float forwardM_S, float yawRad_S) {
  // Convert forward speed and yaw rate to left/right wheel speeds.
  float left = (forwardM_S - yawRad_S * TURN_WHEELBASE_M / 2) / WHEEL_RADIUS_M;
  float right = (forwardM_S + yawRad_S * TURN_WHEELBASE_M / 2) / WHEEL_RADIUS_M;
  desiredSpeed[0] = constrain(left, -DRIVE_SPEED_RAD_S, DRIVE_SPEED_RAD_S);
  desiredSpeed[1] = constrain(right, -DRIVE_SPEED_RAD_S, DRIVE_SPEED_RAD_S);
  integralError[0] = integralError[1] = 0;
  digitalWrite(ENABLE, HIGH);
  for (byte i = 0; i < 2; i++)
    setVoltage(i, KP_VEL[i] * (desiredSpeed[i] - filteredSpeed[i]));
}

bool coastAndSettle() {
  motorsOff();
  if (wheelsStopped()) {
    if (coastStopCounter < SETTLE_COUNT) coastStopCounter++;
  } else {
    coastStopCounter = 0;
  }
  return coastStopCounter >= SETTLE_COUNT;
}
void trackArrow() {
  // Require consistent arrow readings on distinct camera frames.
  if (!newVisionThisTick) return;
  if (!beaconDetected || arrowDirection > DIR_STOP) {
    previousArrow = confirmedArrow = DIR_UNKNOWN;
    arrowFrames = 0;
  } else {
    if (arrowDirection != previousArrow) {
      previousArrow = arrowDirection;
      arrowFrames = 0;
      confirmedArrow = DIR_UNKNOWN;
    }
    if (arrowFrames < CONFIRM_FRAMES) arrowFrames++;
    if (arrowFrames >= CONFIRM_FRAMES) confirmedArrow = arrowDirection;
  }
}
void updateNavigation(unsigned long now, float dt) {
  if (state == DONE) { motorsOff(); return; }
  if (!runStarted) {
    motorsOff();
    if (COMMUNICATION_TEST_ONLY || now - bootMs < START_DELAY_MS) return;
    runStartMs = now;
    runStarted = true;
  }
  if (now - runStartMs >= RUN_TIMEOUT_MS) {
    Serial.println(F("RUN TIMEOUT; reset to restart"));
    stopRun();
    return;
  }
  // The final encoder-controlled turn continues without camera visibility.
  if (state == FINAL_TURN) {
    digitalWrite(ENABLE, HIGH);
    controlWheels(dt);
    if (relativeMotionComplete()) finishRun();
    return;
  }
  const bool visible = visionFresh(now) && beaconDetected;
  if (visible) lastMarkerVisibleMs = now;
  const bool markerLostLong = lastMarkerVisibleMs == 0 ||
      now - lastMarkerVisibleMs > MARKER_LOST_GRACE_MS;

  // Filter only NEW accepted camera observations; do not repeatedly re-filter
  // one measurement on every 10 ms controller tick.
  if (visible && newVisionThisTick) {
    if (!bearingInitialized) {
      smoothedBearingDeg = beaconAngleDeg;
      bearingInitialized = true;
    } else {
      smoothedBearingDeg += 0.35f * (beaconAngleDeg - smoothedBearingDeg);
    }
  }
  trackArrow();

  if (state == SEARCHING) {
    if (!visible) {
      driveVelocity(0, SEARCH_YAW_RAD_S);
      return;
    }
    changeState(APPROACHING);
  }

  // State 1 is a recovery pause after losing the marker while driving.
  if (state == ALIGNING) {
    if (!coastAndSettle()) return;
    if (visible) changeState(APPROACHING);
    else {
      if (markerLostLong) {
        bearingInitialized = false;
        changeState(SEARCHING);
      }
      return;
    }
  }

  // Arrival is latched: a missing or off-center frame cannot restart searching.
  if (!atBeacon) {
    if (!visible) {
      motorsOff();
      changeState(ALIGNING);
      return;
    }
    if (beaconDistanceM <= STOP_DISTANCE_M) {
      atBeacon = true;
      coastStopCounter = 0;
    } else {
      float yaw = fabsf(smoothedBearingDeg) <= ALIGN_TOLERANCE_DEG ? 0 :
          constrain(ALIGN_GAIN * smoothedBearingDeg, -MAX_YAW_RAD_S, MAX_YAW_RAD_S);
      driveVelocity(APPROACH_SPEED_M_S, yaw);
      return;
    }
  }

  bool settled = coastAndSettle();
  if (!visible) {
    arrivalFrames = arrowFrames = 0;
    previousArrow = confirmedArrow = DIR_UNKNOWN;
    return;
  }
  if (!settled || beaconDistanceM > ARRIVAL_LIMIT_M) arrivalFrames = 0;
  else if (newVisionThisTick && arrivalFrames < CONFIRM_FRAMES) arrivalFrames++;
  if (arrivalFrames < CONFIRM_FRAMES) return;

  if (TRIAL_TYPE == 1 || confirmedArrow == DIR_STOP) finishRun();
  else if (confirmedArrow == DIR_LEFT || confirmedArrow == DIR_RIGHT) {
    byte turn = confirmedArrow;
    changeState(FINAL_TURN);
    float targetDeg = turn == DIR_RIGHT ? RIGHT_TURN_TARGET_DEG : -LEFT_TURN_TARGET_DEG;
    commandTurn(targetDeg * PI / 180.0f);
  }

}

void setup() {
  pinMode(ENABLE, OUTPUT); digitalWrite(ENABLE, LOW);
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, LOW);
  for (byte i = 0; i < 2; i++) {
    pinMode(DIR[i], OUTPUT); digitalWrite(DIR[i], LOW);
    pinMode(PWM[i], OUTPUT); analogWrite(PWM[i], 0);
    pinMode(ENC_A[i], INPUT_PULLUP); pinMode(ENC_B[i], INPUT_PULLUP);
  }
  motorsOff();
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), rightEncoder, CHANGE);
  Serial.begin(115200);
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveVision);
  Wire.onRequest(requestStatus);
  bootMs = millis(); lastUpdateUs = micros(); lastPrintMs = bootMs;
  if ((TRIAL_TYPE != 1 && TRIAL_TYPE != 2) || STOP_DISTANCE_M <= 0 || APPROACH_SPEED_M_S <= 0 ||
      STOP_DISTANCE_M > ARRIVAL_LIMIT_M || BATTERY_V <= 0 ||
      VOLTAGE_LIMIT <= 0 || VOLTAGE_LIMIT > BATTERY_V || KP_POS <= 0 || KI_POS < 0) {
    stopRun(); Serial.println(F("INVALID SETTINGS; motors off"));
  } else {
    Serial.println(COMMUNICATION_TEST_ONLY ? F("Communication test; motors off")
      : F("Autonomous Demo 2; search starts after configured delay"));
  }
}
void loop() {
  for (byte n = 0; n < 16 && Serial.available(); n++) {
    char c = Serial.read();
    if (c == 's' || c == '!') {
      stopRun();
      Serial.println(F("STOPPED; reset to restart"));
    }
  }
  unsigned long nowUs = micros(), elapsedUs = nowUs - lastUpdateUs;
  if (elapsedUs < CONTROL_US) return;
  lastUpdateUs = nowUs;
  unsigned long now = millis();
  updateMeasurements(elapsedUs / 1000000.0f);
  updateVision(now);
  updateNavigation(now, elapsedUs / 1000000.0f);
  publishStatus(now);
  if (now - lastPrintMs >= 200UL) {
    lastPrintMs = now;
    unsigned long rx, rejected;
    noInterrupts();
    rx = visionPacketsSeen;
    rejected = visionPacketsRejected;

    // used soley for debugging in the Serial Monitor. Not needed in the actual running 
    interrupts();
    Serial.print(F("rx=")); Serial.print(rx);
    Serial.print(F(" rejected=")); Serial.print(rejected);
    Serial.print(F(" invalid=")); Serial.print(visionPacketsInvalid);
    Serial.print(F(" seq=")); Serial.print(visionSequence);
    Serial.print(F(" state=")); Serial.print((int)state);
    Serial.print(F(" braking=")); Serial.print(braking);
    Serial.print(F(" coast=")); Serial.print(coastStopCounter);
    Serial.print(F(" speedL=")); Serial.print(filteredSpeed[0], 2);
    Serial.print(F(" speedR=")); Serial.print(filteredSpeed[1], 2);
    Serial.print(F(" fresh=")); Serial.print(visionFresh(now));
    Serial.print(F(" detected=")); Serial.print(visionFresh(now) && beaconDetected);
    Serial.print(F(" distance_m=")); Serial.print(beaconDistanceM, 3);
    Serial.print(F(" angle_deg=")); Serial.print(beaconAngleDeg, 2);
    Serial.print(F(" filtered_angle=")); Serial.print(smoothedBearingDeg, 2);
    Serial.print(F(" marker_age_ms="));
    Serial.print(lastMarkerVisibleMs ? now-lastMarkerVisibleMs : 99999UL);
    Serial.print(F(" arrow=")); Serial.print(arrowDirection);
    Serial.println();
  }
}

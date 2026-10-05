// EENG 350 - Mini Project: Vision-Based Wheel Position Control
//
// STARTUP AND COMMANDS (Serial Monitor: 115200 baud)
//   Align both zero marks while stopped, send z, then enable motor power.
//   Send p and have the Pi send a fresh quadrant. Send s to stop.
//   z defines the current positions as zero; it does not physically home wheels.
//   Targets are 0 or PI radians from this reference, not additional half-turns.

// -----------------------------------------------------------------------------
// Libraries
// -----------------------------------------------------------------------------
// Arduino I/O and timing, Wire I2C communication, and floating-point math.
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// -----------------------------------------------------------------------------
// Hardware connections - array order: left, right
// -----------------------------------------------------------------------------
#define MY_ADDR 8
const byte ENABLE = 4;
const byte ENC_A[2] = {2, 3};
const byte ENC_B[2] = {5, 6};
const byte DIR[2] = {7, 8};
const byte PWM[2] = {9, 10};

// Convert raw count directions to the chosen positive direction for each wheel.
// These signs and motor direction settings include the tested right-wheel reversal.
const int ENCODER_SIGN[2] = {-1, -1};

// Direction-pin level used for a positive voltage command (not an encoder setting).
const bool POSITIVE_DIR_HIGH[2] = {false, false};

// -----------------------------------------------------------------------------
// Encoder conversion, limits, and controller gains
// -----------------------------------------------------------------------------
// A-channel CHANGE decoding measured 1600 counts per wheel revolution.
// Angle = signed counts * 2*PI/1600; approximately 800 counts is half a turn.
const float RAD_PER_COUNT = 2.0f * PI / 1600.0f;
const float BATTERY_V = 7.8f;  // Assumed supply voltage (V).
const float VOLTAGE_LIMIT = 7.0f;  // Maximum command magnitude (V).
const float SPEED_LIMIT = 2.0f;  // Maximum desired speed (rad/s).
// Position P reacts to present error; position I accumulates error over time.
// Speed P compares the desired speed with the encoder-derived speed.
const float KP_POSITION = 10.0f;
const float KI_POSITION = 0.5f;
const float KP_SPEED = 1.5f;

// -----------------------------------------------------------------------------
// Quadrant targets - each entry is multiplied by PI
// -----------------------------------------------------------------------------
const byte GOALS[4][2] = {
  {0, 0},  // 1: top right
  {0, 1},  // 2: top left
  {1, 1},  // 3: bottom left
  {1, 0}   // 4: bottom right
};

// -----------------------------------------------------------------------------
// Shared measurements and controller state
// -----------------------------------------------------------------------------
// volatile variables can change inside interrupt callbacks.
// Atomic copies below are still needed; volatile alone does not protect a copy.

volatile long counts[2] = {0, 0};
volatile byte incomingByte = 0;
volatile bool newQuadrant = false;
long previousCounts[2] = {0, 0};
float target[2] = {0, 0};  // Desired angles (rad).
float speed[2] = {0, 0};  // Filtered speeds (rad/s).
float integral[2] = {0, 0};  // Accumulated position errors (rad*s).

// piEnabled permits new Pi targets
bool piEnabled = false;
bool controlling = false;
unsigned long lastUpdate;
unsigned long lastReport = 0;
byte activeQuadrant = 0;

// MOVING runs the controllers. HOLDING commands zero output within a deadband.
// Each wheel has its own state so one can stop while the other is still moving.
enum WheelState { MOVING, HOLDING };
WheelState state[2] = {MOVING, MOVING};
float measuredAngle[2] = {0, 0};
float commandedVoltage[2] = {0, 0};

// Separate stop/resume thresholds provide hysteresis to reduce state chatter.
// HOLDING is not exact active torque holding: correction resumes beyond 4 degrees.
const float STOP_TOLERANCE = 0.035f;  // About 2 degrees.
const float RESUME_TOLERANCE = 0.070f;  // About 4 degrees, avoids rapid switching.

// -----------------------------------------------------------------------------
// Encoder interrupt service routines
// -----------------------------------------------------------------------------
// Called on either edge of encoder A. Comparing A with B determines direction.
// The conditional expression adds +1 when equal, otherwise -1.
void leftEncoder() {
  counts[0] += digitalRead(ENC_A[0]) == digitalRead(ENC_B[0]) ? 1 : -1;
}
void rightEncoder() {
  counts[1] += digitalRead(ENC_A[1]) == digitalRead(ENC_B[1]) ? 1 : -1;
}

// -----------------------------------------------------------------------------
// I2C communication with the Raspberry Pi
// -----------------------------------------------------------------------------
// Interrupt callback: store a valid numeric byte; no printing or motor work.
void receive(int byteCount) {
  // Reject multi-byte packets; the Pi must send a raw byte, not ASCII text.
  if (byteCount != 1) {
    while (Wire.available()) Wire.read();
    return;
  }
  if (Wire.available()) {
    byte q = Wire.read();
    if (q >= 1 && q <= 4) {
      incomingByte = q;
      newQuadrant = true;
    }
  }
}
// Called when the Pi requests a byte. A reply does not mean the target was reached.
void request() {
  Wire.write((byte)incomingByte);
  // Receipt acknowledgment, not goal reached.
}

// -----------------------------------------------------------------------------
// Motor output functions
// -----------------------------------------------------------------------------
// Disable both outputs and disarm Pi control. Encoder counting continues.
// A later p command is required before another fresh Pi command can move motors.
void stopMotors() {
  analogWrite(PWM[0], 0);
  analogWrite(PWM[1], 0);
  digitalWrite(ENABLE, LOW);
  piEnabled = false;
  controlling = false;
  integral[0] = integral[1] = 0;
  commandedVoltage[0] = commandedVoltage[1] = 0;
}

void drive(byte i, float voltage) {
  // Limit the signed command
  voltage = constrain(voltage, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  commandedVoltage[i] = voltage;

  // Voltage sign selects direction; voltage magnitude sets duty cycle.
  bool high = voltage >= 0 ? POSITIVE_DIR_HIGH[i] : !POSITIVE_DIR_HIGH[i];

  // Remove PWM before changing the direction output.
  if (digitalRead(DIR[i]) != (high ? HIGH : LOW)) {
    analogWrite(PWM[i], 0);
    digitalWrite(DIR[i], high ? HIGH : LOW);
  }

  // Approximate duty = |requested voltage| / assumed supply voltage.
  // analogWrite uses 0-255
  int duty = (int)(255.0f * fabsf(voltage) / BATTERY_V);
  analogWrite(PWM[i], constrain(duty, 0, 255));
}

// -----------------------------------------------------------------------------
// Startup configuration
// -----------------------------------------------------------------------------
// Runs once after power-up/reset. Begin with driver disabled and PWM at zero.
void setup() {
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  for (byte i = 0; i < 2; ++i) {
    pinMode(DIR[i], OUTPUT);
    pinMode(PWM[i], OUTPUT);
    digitalWrite(DIR[i], LOW);
    analogWrite(PWM[i], 0);
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
  }

  // Hardware interrupts count A transitions independently of loop timing.
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), rightEncoder, CHANGE);
  Serial.begin(115200);

  // Register this Arduino as I2C address 8 and install communication callbacks.
  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);
  Wire.onRequest(request);
  lastUpdate = micros();
  Serial.println(F("Stopped. Align both 0 labels up; z=zero, p=Pi enable, s=stop."));
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void loop() {
  // Handle operator commands: stop, zero, and enable Pi control.
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == '!') {
      stopMotors();
      Serial.println(F("Stopped; Pi disabled."));

    // Zero only when stopped; reset count history to avoid a false speed jump.
    } else if (c == 'z' && !piEnabled && !controlling) {
      noInterrupts();
      counts[0] = counts[1] = 0;
      interrupts();
      for (byte i = 0; i < 2; ++i) {
        previousCounts[i] = 0;

        // Low-pass filter: 80% previous estimate + 20% new speed. Adds some lag.
    speed[i] = 0;
        integral[i] = 0;
        target[i] = 0;
      }
      lastUpdate = micros();
      Serial.println(F("Both positions zeroed."));

    // Discard pending data so enabling requires a new Pi message.
    } else if (c == 'p' && !piEnabled && !controlling) {
      noInterrupts();
      newQuadrant = false;
      interrupts();
      piEnabled = true;
      Serial.println(F("Waiting for a fresh Pi quadrant."));
    }
  }

  // Copy the latest quadrant and update absolute position targets.
  // Snapshot the command and flag together, then clear the pending flag.
  // This is a latest-command mailbox, not a queue of every received quadrant.
  byte q;
  bool fresh;
  noInterrupts();
  q = incomingByte;
  fresh = newQuadrant;
  newQuadrant = false;
  interrupts();
  if (piEnabled && fresh) {
    // Repeated packets hold the same absolute target; never reset counts.
    for (byte i = 0; i < 2; ++i) {

      // q-1 maps quadrant numbers 1-4 to table rows 0-3. Multiply 0/1 by PI.
      // Example: quadrant 2 selects {0,1}, giving left 0 rad and right PI rad.
      float nextTarget = GOALS[q - 1][i] * PI;

      // Reset integral only on activation or a changed target for this wheel.
      if (!controlling || nextTarget != target[i]) {
        integral[i] = 0;
        state[i] = MOVING;
      }
      target[i] = nextTarget;
    }
    activeQuadrant = q;
    controlling = true;
    digitalWrite(ENABLE, HIGH);
  }

  // Run feedback calculations approximately every 10 milliseconds.
  // Nonblocking timing: return until 10 ms has elapsed; interrupts still run.
  unsigned long now = micros();
  unsigned long elapsed = now - lastUpdate;
  if (elapsed < 10000UL) return;
  // Approximately 100 Hz control.
  // Convert the actual interval from microseconds to seconds for speed/integration.
  // Example: 10000 microseconds * 0.000001 = 0.010 seconds.
  float dt = elapsed * 1.0e-6f;
  lastUpdate = now;

  // Briefly pause interrupts to copy multi-byte counts consistently on the Uno.
  long current[2];
  noInterrupts();
  current[0] = counts[0];
  current[1] = counts[1];
  interrupts();

  for (byte i = 0; i < 2; ++i) {

    // Position comes from total counts; speed comes from the change in counts.
    float angle = ENCODER_SIGN[i] * current[i] * RAD_PER_COUNT;
    measuredAngle[i] = angle;
    float rawSpeed = ENCODER_SIGN[i] * (current[i] - previousCounts[i]) * RAD_PER_COUNT / dt;
    previousCounts[i] = current[i];
    speed[i] = 0.8f * speed[i] + 0.2f * rawSpeed;

    // Measurements still update when stopped, but motor control is skipped.
    if (!controlling) continue;

    // Outer position PI produces desired speed; inner speed P produces voltage.
    // Positive error requests increasing angle; negative error requests decreasing.
    // Example: target 3.142 - actual 2.500 = +0.642 rad remaining.
    float error = target[i] - angle;

    // Each wheel stops independently. If pushed away, resume correction.
    if (state[i] == MOVING && fabsf(error) <= STOP_TOLERANCE)
      state[i] = HOLDING;
    else if (state[i] == HOLDING && fabsf(error) > RESUME_TOLERANCE)
      state[i] = MOVING;

    // Clear accumulated correction and command zero PWM near the target.
    // Physical stopping is not instantaneous; leaving the resume band restarts control.
    if (state[i] == HOLDING) {
      integral[i] = 0;
      drive(i, 0);
      // continue advances to the other wheel (or ends this wheel loop).
      continue;
    }

    // Try integrating the position error over this time step (rad*s).
    // The trial value is provisional until the anti-windup check accepts it.
    float trialIntegral = integral[i] + error * dt;

    // Outer PI produces desired speed in rad/s, then limits it to +/- SPEED_LIMIT.
    float rawDesired = KP_POSITION * error + KI_POSITION * trialIntegral;
    float desired = constrain(rawDesired, -SPEED_LIMIT, SPEED_LIMIT);

    // Inner P converts speed error to a signed voltage command.
    float rawVoltage = KP_SPEED * (desired - speed[i]);

    // Reject integral growth that pushes further into speed or voltage limits.
    // Accept the new integral only if it does not push farther into saturation.
    // Opposite-signed error is allowed to unwind previously accumulated integral.
    bool pushesLimit = (rawDesired > SPEED_LIMIT && error > 0) ||
                       (rawDesired < -SPEED_LIMIT && error < 0) ||
                       (rawVoltage > VOLTAGE_LIMIT && error > 0) ||
                       (rawVoltage < -VOLTAGE_LIMIT && error < 0);
    if (!pushesLimit) integral[i] = trialIntegral;

    // Recalculate using the accepted integral, then apply direction and PWM.
    desired = constrain(KP_POSITION * error + KI_POSITION * integral[i],
                        -SPEED_LIMIT, SPEED_LIMIT);
    drive(i, KP_SPEED * (desired - speed[i]));
  }

  // Display measured feedback and requested voltage commands.
  // Report at 5 Hz so printing does not dominate the 100 Hz controller.
  // F() stores fixed message strings in flash to save Uno RAM.
  // actual is encoder-derived; Vcmd is requested voltage; OFF means disarmed.
  if (millis() - lastReport >= 200UL) {
    lastReport = millis();
    Serial.print(F("Q="));
    Serial.print(activeQuadrant); // displays quadrant as sent by the pi
    for (byte i = 0; i < 2; ++i) {
      Serial.print(i == 0 ? F(" | L target=") : F(" | R target=")); // displays target position
      Serial.print(target[i], 3); // only needed for testing. Didn't end up taking out
      Serial.print(F(" actual=")); // same with this one. Measures the encoder count. Good for debugging
      Serial.print(measuredAngle[i], 3); // measures turn angle
      Serial.print(F(" Vcmd="));
      Serial.print(commandedVoltage[i], 2); // displays the commanded voltage
      Serial.print(!controlling ? F(" OFF") :
                   state[i] == HOLDING ? F(" HOLD") : F(" MOVE"));
    }
    Serial.println();
  }

}


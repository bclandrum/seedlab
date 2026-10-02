#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

const uint8_t ENABLE_PIN = 4;
const uint8_t DIR_PIN[2] = {7, 8};
const uint8_t PWM_PIN[2] = {9, 10};
const uint8_t ENC_A[2] = {2, 3};
const uint8_t ENC_B[2] = {5, 6};

const float BATTERY_VOLTAGE = 7.8f;
const float KP[2] = {2.0f, 2.0f};

// Both edges of channel A: 32 counts/shaft turn × 50:1 gearbox.
const float COUNTS_PER_REV = 1600.0f;

const uint8_t REPORT_MOTOR = 0;
const uint32_t SAMPLE_US = 10000UL;
const float STEP_TIME = 1.0f;
const float RUN_TIME = 5.0f;

volatile long encoderCount[2] = {0, 0};
long previousCount[2] = {0, 0};

uint32_t startUs = 0;
uint32_t previousUs = 0;

float requestedSpeed = 0.0f;
bool running = false;

char inputBuffer[32];
uint8_t inputLength = 0;
bool inputOverflow = false;

void encoder1ISR() {
  encoderCount[0] += digitalRead(2) == digitalRead(5) ? 1 : -1;
}

void encoder2ISR() {
  encoderCount[1] += digitalRead(3) == digitalRead(6) ? 1 : -1;
}

uint32_t readEncoders(long count[2]) {
  noInterrupts();
  uint32_t now = micros();
  count[0] = encoderCount[0];
  count[1] = encoderCount[1];
  interrupts();
  return now;
}

void stopMotors() {
  analogWrite(PWM_PIN[0], 0);
  analogWrite(PWM_PIN[1], 0);
}

void promptSpeed() {
  Serial.println(F("Enter speed (rad/s):"));
}

void startRun(float speed) {
  stopMotors();
  requestedSpeed = speed;

  startUs = previousUs = readEncoders(previousCount);
  running = true;

  Serial.println(F("0.0000,0.000,0.0000"));
}

void readSpeedInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Enter the next speed only after the prompt returns.
    if (running || c == '\r') continue;

    if (c == '\n') {
      inputBuffer[inputLength] = '\0';

      char *end;
      float speed = strtod(inputBuffer, &end);
      bool converted = end != inputBuffer;

      while (*end == ' ' || *end == '\t') ++end;

      bool valid = !inputOverflow && converted
                   && *end == '\0' && isfinite(speed);

      inputLength = 0;
      inputOverflow = false;

      if (valid) startRun(speed);
      else promptSpeed();
    } else if (inputLength < sizeof(inputBuffer) - 1) {
      inputBuffer[inputLength++] = c;
    } else {
      inputOverflow = true;
    }
  }
}

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  for (uint8_t i = 0; i < 2; ++i) {
    pinMode(DIR_PIN[i], OUTPUT);
    pinMode(PWM_PIN[i], OUTPUT);
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    digitalWrite(DIR_PIN[i], LOW);
  }

  stopMotors();

  attachInterrupt(digitalPinToInterrupt(2), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), encoder2ISR, CHANGE);

  Serial.begin(115200);

  digitalWrite(ENABLE_PIN, HIGH);
  delay(1000);

  promptSpeed();
}

void loop() {
  readSpeedInput();
  if (!running) return;

  if ((uint32_t)(micros() - previousUs) < SAMPLE_US) return;

  long count[2];
  uint32_t now = readEncoders(count);

  float dt = (uint32_t)(now - previousUs) * 1.0e-6f;
  float time = (uint32_t)(now - startUs) * 1.0e-6f;

  if (time >= RUN_TIME) {
    stopMotors();
    running = false;
    promptSpeed();
    return;
  }

  float desiredMagnitude = fabsf(requestedSpeed);
  float directionSign = requestedSpeed < 0.0f ? -1.0f : 1.0f;
  uint8_t direction = requestedSpeed < 0.0f ? HIGH : LOW;

  float measuredSpeed[2];
  float voltage[2];

  for (uint8_t i = 0; i < 2; ++i) {
    measuredSpeed[i] =
      fabsf(6.28318530718f * (count[i] - previousCount[i])
            / (COUNTS_PER_REV * dt));

    float command = 0.0f;

    if (time >= STEP_TIME && desiredMagnitude > 0.0f) {
      // Change direction only after the initial rest interval.
      if (digitalRead(DIR_PIN[i]) != direction) {
        analogWrite(PWM_PIN[i], 0);
        digitalWrite(DIR_PIN[i], direction);
      }

      // Regulate speed magnitude independently for each motor.
      command = constrain(
        KP[i] * (desiredMagnitude - measuredSpeed[i]),
        0.0f,
        BATTERY_VOLTAGE
      );
    }

    int pwm = (int)(255.0f * command / BATTERY_VOLTAGE + 0.5f);
    analogWrite(PWM_PIN[i], pwm);

    voltage[i] = pwm == 0
                 ? 0.0f
                 : directionSign * BATTERY_VOLTAGE * pwm / 255.0f;

    previousCount[i] = count[i];
  }

  previousUs = now;

  Serial.print(time, 4);
  Serial.print(',');
  Serial.print(voltage[REPORT_MOTOR], 3);
  Serial.print(',');
  Serial.println(measuredSpeed[REPORT_MOTOR], 4);
}

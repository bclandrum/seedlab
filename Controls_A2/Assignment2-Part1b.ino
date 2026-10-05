#include <Arduino.h>

const uint8_t ENABLE_PIN = 4;
const uint8_t M1_DIR_PIN = 7;
const uint8_t M2_DIR_PIN = 8;
const uint8_t M1_PWM_PIN = 9;
const uint8_t M2_PWM_PIN = 10;

const uint8_t PWM_VALUE = 192;
const bool INVERT_MOTOR_2 = false;
const unsigned long RUN_MS = 5000;
const unsigned long STOP_MS = 1000;

void stopBoth() {
  analogWrite(M1_PWM_PIN, 0);
  analogWrite(M2_PWM_PIN, 0);
}

void runBoth(bool reverse) {
  uint8_t direction1 = reverse ? HIGH : LOW;
  uint8_t direction2 = (reverse != INVERT_MOTOR_2) ? HIGH : LOW;

  digitalWrite(M1_DIR_PIN, direction1);
  digitalWrite(M2_DIR_PIN, direction2);

  analogWrite(M1_PWM_PIN, PWM_VALUE);
  analogWrite(M2_PWM_PIN, PWM_VALUE);

  Serial.print(F("Voltage sign (A-B): M1="));
  Serial.print(direction1 == LOW ? '+' : '-');
  Serial.print(F(" M2="));
  Serial.print(direction2 == LOW ? '+' : '-');
  Serial.print(F(" | PWM="));
  Serial.print(PWM_VALUE);
  Serial.print(F(" | Duty="));
  Serial.print(100.0f * PWM_VALUE / 255.0f, 1);
  Serial.println(F("%"));

  delay(RUN_MS);

  stopBoth();
  Serial.println(F("Stopped"));
  delay(STOP_MS);
}

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);

  digitalWrite(M1_DIR_PIN, LOW);
  digitalWrite(M2_DIR_PIN, LOW);
  stopBoth();

  Serial.begin(115200);

  digitalWrite(ENABLE_PIN, HIGH);
  delay(STOP_MS);
}

void loop() {
  runBoth(false);
  runBoth(true);
}
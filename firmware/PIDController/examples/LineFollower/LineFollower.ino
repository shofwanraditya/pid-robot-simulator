/**
 * LineFollower.ino — Contoh penggunaan PIDController pada robot Line Follower
 * dengan 5 sensor IR analog dan driver motor L298N / TB6612FNG.
 *
 * Hasil tuning dapat diambil langsung dari PID Robot Simulator (klik "{ } Salin C++").
 */
#include <Arduino.h>
#include <PIDController.h>

// Pin sensor IR (kiri ke kanan)
const uint8_t IR_PINS[5] = {A0, A1, A2, A3, A4};
const float SENSOR_POS[5] = {30.0f, 15.0f, 0.0f, -15.0f, -30.0f}; // mm

// Pin motor (kiri & kanan)
const uint8_t PIN_PWM_L = 5, PIN_IN1_L = 4, PIN_IN2_L = 3;
const uint8_t PIN_PWM_R = 6, PIN_IN1_R = 7, PIN_IN2_R = 8;

const float BASE_PWM = 140.0f;
const uint32_t CONTROL_PERIOD_US = 5000; // 200 Hz (dt = 0.005 s)

PIDController pid(4.5f, 0.0f, 0.25f, -255.0f, 255.0f);
uint32_t lastTickUs = 0;
float lastSign = 1.0f;

float readLineErrorMm() {
  float sum = 0.0f, weighted = 0.0f;
  for (int i = 0; i < 5; i++) {
    // Normalisasi pembacaan ADC (0..1023 -> 0.0..1.0, asumsikan garis hitam = nilai tinggi)
    float v = analogRead(IR_PINS[i]) / 1023.0f;
    sum += v;
    weighted += v * SENSOR_POS[i];
  }
  if (sum < 0.25f) return lastSign * 45.0f; // garis hilang -> putar ke arah terakhir
  float err = weighted / sum;
  if (fabsf(err) > 1.0f) lastSign = (err > 0.0f) ? 1.0f : -1.0f;
  return err;
}

void setMotor(uint8_t pwmPin, uint8_t in1, uint8_t in2, float cmd) {
  int pwm = (int)fminf(255.0f, fabsf(cmd));
  digitalWrite(in1, cmd >= 0.0f ? HIGH : LOW);
  digitalWrite(in2, cmd >= 0.0f ? LOW : HIGH);
  analogWrite(pwmPin, pwm);
}

void setup() {
  Serial.begin(115200);
  pid.setDerivativeFilter(0.005f);
  lastTickUs = micros();
}

void loop() {
  uint32_t now = micros();
  if (now - lastTickUs >= CONTROL_PERIOD_US) {
    float dt = (now - lastTickUs) * 1e-6f;
    lastTickUs = now;

    float errorMm = readLineErrorMm();
    float u = pid.compute(0.0f, errorMm, dt);

    setMotor(PIN_PWM_L, PIN_IN1_L, PIN_IN2_L, BASE_PWM + u);
    setMotor(PIN_PWM_R, PIN_IN1_R, PIN_IN2_R, BASE_PWM - u);
  }
}

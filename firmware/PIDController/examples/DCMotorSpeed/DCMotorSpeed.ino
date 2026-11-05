/**
 * DCMotorSpeed.ino — Kontrol kecepatan motor DC dengan encoder kuadratur (ESP32 / Arduino).
 *
 * Format output Serial kompatibel dengan Arduino Serial Plotter:
 *   setpoint,rpm,pwm
 */
#include <Arduino.h>
#include <PIDController.h>

const uint8_t PIN_ENC_A = 2;
const uint8_t PIN_ENC_B = 3;
const uint8_t PIN_PWM   = 9;
const uint8_t PIN_DIR   = 8;

const float COUNTS_PER_REV = 600.0f;
const uint32_t PERIOD_US   = 5000; // 200 Hz

volatile long encoderTicks = 0;
PIDController pid(0.45f, 2.5f, 0.005f, -255.0f, 255.0f);
float setpointRpm = 1000.0f;
uint32_t lastUs = 0;

void IRAM_ATTR onEncoderA() {
  encoderTicks += digitalRead(PIN_ENC_B) ? -1 : 1;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncoderA, RISING);

  pid.setDerivativeFilter(0.01f);
  lastUs = micros();
}

void loop() {
  uint32_t now = micros();
  if (now - lastUs >= PERIOD_US) {
    float dt = (now - lastUs) * 1e-6f;
    lastUs = now;

    noInterrupts();
    long ticks = encoderTicks;
    encoderTicks = 0;
    interrupts();

    float rpm = (ticks / COUNTS_PER_REV) * (60.0f / dt);
    float pwm = pid.compute(setpointRpm, rpm, dt);

    digitalWrite(PIN_DIR, pwm >= 0.0f ? HIGH : LOW);
    analogWrite(PIN_PWM, (int)fminf(255.0f, fabsf(pwm)));

    Serial.print(setpointRpm); Serial.print(',');
    Serial.print(rpm);         Serial.print(',');
    Serial.println(pwm);
  }
}

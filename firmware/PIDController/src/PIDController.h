/**
 * PIDController.h — Discrete-time PID controller untuk mikrokontroler.
 *
 * Portable C++ (tanpa dependency Arduino) → bisa dipakai di Arduino, ESP32,
 * STM32 (HAL/Mbed), maupun di PC untuk unit test.
 *
 * Fitur:
 *   - Derivative on measurement (tanpa derivative kick)
 *   - Low-pass filter orde-1 pada suku D
 *   - Output clamping + anti-windup (conditional integration)
 *   - Bumpless gain change (suku I disimpan sebagai Ki*∫e dt)
 *
 * Implementasi ini identik secara numerik dengan simulator/js/core/pid.js,
 * sehingga gain hasil tuning di simulator bisa langsung dipakai di firmware.
 */
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
  PIDController(float kp, float ki, float kd, float outMin = -255.0f, float outMax = 255.0f);

  void setTunings(float kp, float ki, float kd);
  void setOutputLimits(float outMin, float outMax);
  /** Konstanta waktu LPF untuk suku D (detik). 0 = tanpa filter. */
  void setDerivativeFilter(float tau);
  void setAntiWindup(bool enabled);
  void setDerivativeOnMeasurement(bool enabled);

  /** Reset state internal (integral, derivatif). Panggil saat mode berganti. */
  void reset();

  /**
   * Hitung output kontrol.
   * @param setpoint    nilai target
   * @param measurement nilai sensor
   * @param dt          periode sampling dalam detik (harus > 0)
   */
  float compute(float setpoint, float measurement, float dt);

  float getKp() const { return kp_; }
  float getKi() const { return ki_; }
  float getKd() const { return kd_; }
  float getPTerm() const { return pTerm_; }
  float getITerm() const { return iTerm_; }
  float getDTerm() const { return dTerm_; }
  float getError() const { return error_; }
  float getOutput() const { return output_; }
  bool isSaturated() const { return saturated_; }

private:
  static float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  float kp_, ki_, kd_;
  float outMin_, outMax_;
  float dFilterTau_ = 0.0f;
  bool antiWindup_ = true;
  bool derivativeOnMeasurement_ = true;

  float iTerm_ = 0.0f;
  float pTerm_ = 0.0f;
  float dTerm_ = 0.0f;
  float dState_ = 0.0f;
  float error_ = 0.0f;
  float prevError_ = 0.0f;
  float prevMeasurement_ = 0.0f;
  bool hasPrev_ = false;
  float output_ = 0.0f;
  bool saturated_ = false;
};

#endif // PID_CONTROLLER_H

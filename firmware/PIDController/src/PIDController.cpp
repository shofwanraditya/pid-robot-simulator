#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float outMin, float outMax)
    : kp_(kp), ki_(ki), kd_(kd), outMin_(outMin), outMax_(outMax) {}

void PIDController::setTunings(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setOutputLimits(float outMin, float outMax) {
  if (outMin >= outMax) return;
  outMin_ = outMin;
  outMax_ = outMax;
  iTerm_ = clamp(iTerm_, outMin_, outMax_);
}

void PIDController::setDerivativeFilter(float tau) { dFilterTau_ = tau > 0.0f ? tau : 0.0f; }

void PIDController::setAntiWindup(bool enabled) { antiWindup_ = enabled; }

void PIDController::setDerivativeOnMeasurement(bool enabled) { derivativeOnMeasurement_ = enabled; }

void PIDController::reset() {
  iTerm_ = pTerm_ = dTerm_ = dState_ = 0.0f;
  error_ = prevError_ = prevMeasurement_ = 0.0f;
  hasPrev_ = false;
  output_ = 0.0f;
  saturated_ = false;
}

float PIDController::compute(float setpoint, float measurement, float dt) {
  if (!(dt > 0.0f)) return output_;
  const float error = setpoint - measurement;

  // --- P ---
  pTerm_ = kp_ * error;

  // --- D (low-pass filtered) ---
  float dRaw = 0.0f;
  if (hasPrev_) {
    dRaw = derivativeOnMeasurement_ ? -(measurement - prevMeasurement_) / dt
                                    : (error - prevError_) / dt;
  }
  const float alpha = dFilterTau_ > 0.0f ? dt / (dFilterTau_ + dt) : 1.0f;
  dState_ += alpha * (dRaw - dState_);
  dTerm_ = kd_ * dState_;

  // --- I (anti-windup: conditional integration + clamping) ---
  const float iCandidate = iTerm_ + ki_ * error * dt;
  if (antiWindup_) {
    const float unsat = pTerm_ + iCandidate + dTerm_;
    const bool pushingHigh = unsat > outMax_ && error > 0.0f;
    const bool pushingLow = unsat < outMin_ && error < 0.0f;
    if (!pushingHigh && !pushingLow) iTerm_ = iCandidate;
    iTerm_ = clamp(iTerm_, outMin_, outMax_);
  } else {
    iTerm_ = iCandidate;
  }

  const float raw = pTerm_ + iTerm_ + dTerm_;
  output_ = clamp(raw, outMin_, outMax_);
  saturated_ = (output_ != raw);

  error_ = error;
  prevError_ = error;
  prevMeasurement_ = measurement;
  hasPrev_ = true;
  return output_;
}

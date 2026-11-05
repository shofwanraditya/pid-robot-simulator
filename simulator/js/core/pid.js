/**
 * pid.js — Discrete-time PID controller.
 *
 * Fitur (identik dengan library C++ di /firmware/PIDController):
 *   - Derivative on measurement  -> tidak ada "derivative kick" saat setpoint berubah
 *   - Low-pass filter orde-1 pada suku D -> meredam noise sensor
 *   - Output clamping + integrator clamping (anti-windup)
 *   - Bumpless gain change: suku I disimpan sebagai Ki*∫e dt
 *
 *   u(k) = Kp*e(k) + Σ Ki*e(k)*dt + Kd * LPF( -(y(k) - y(k-1)) / dt )
 */
(function (root, factory) {
  const isNode = typeof module === 'object' && module.exports;
  const U = isNode ? require('./utils.js') : root.PIDSim;
  const mod = factory(U);
  if (isNode) module.exports = mod;
  else root.PIDSim = Object.assign(root.PIDSim || {}, mod);
})(typeof self !== 'undefined' ? self : this, function (U) {
  'use strict';
  const { clamp } = U;

  class PIDController {
    /**
     * @param {object} opts
     * @param {number} [opts.kp=1]  gain proporsional
     * @param {number} [opts.ki=0]  gain integral
     * @param {number} [opts.kd=0]  gain derivatif
     * @param {number} [opts.outMin=-Infinity] batas bawah output (mis. -255 PWM)
     * @param {number} [opts.outMax=Infinity]  batas atas output
     * @param {number} [opts.dFilterTau=0] konstanta waktu LPF suku D (detik), 0 = tanpa filter
     * @param {boolean} [opts.antiWindup=true]
     * @param {boolean} [opts.derivativeOnMeasurement=true]
     */
    constructor(opts = {}) {
      this.kp = opts.kp ?? 1;
      this.ki = opts.ki ?? 0;
      this.kd = opts.kd ?? 0;
      this.outMin = opts.outMin ?? -Infinity;
      this.outMax = opts.outMax ?? Infinity;
      this.dFilterTau = opts.dFilterTau ?? 0;
      this.antiWindup = opts.antiWindup ?? true;
      this.derivativeOnMeasurement = opts.derivativeOnMeasurement ?? true;
      this.reset();
    }

    setTunings(kp, ki, kd) {
      this.kp = kp;
      this.ki = ki;
      this.kd = kd;
    }

    setOutputLimits(min, max) {
      this.outMin = min;
      this.outMax = max;
      this.iTerm = clamp(this.iTerm, min, max);
    }

    reset() {
      this.iTerm = 0;
      this.pTerm = 0;
      this.dTerm = 0;
      this.dState = 0;
      this.error = 0;
      this.prevError = 0;
      this.prevMeasurement = null;
      this.output = 0;
      this.saturated = false;
    }

    /**
     * Hitung satu langkah kontrol.
     * @param {number} setpoint nilai target
     * @param {number} measurement nilai terukur dari sensor
     * @param {number} dt periode sampling (detik)
     * @returns {number} output kontrol (sudah di-clamp)
     */
    compute(setpoint, measurement, dt) {
      if (!(dt > 0)) return this.output;
      const error = setpoint - measurement;

      // --- P ---
      this.pTerm = this.kp * error;

      // --- D (dengan low-pass filter) ---
      let dRaw = 0;
      if (this.prevMeasurement !== null) {
        dRaw = this.derivativeOnMeasurement
          ? -(measurement - this.prevMeasurement) / dt
          : (error - this.prevError) / dt;
      }
      const alpha = this.dFilterTau > 0 ? dt / (this.dFilterTau + dt) : 1;
      this.dState += alpha * (dRaw - this.dState);
      this.dTerm = this.kd * this.dState;

      // --- I (dengan anti-windup) ---
      const iCandidate = this.iTerm + this.ki * error * dt;
      if (this.antiWindup) {
        const unsat = this.pTerm + iCandidate + this.dTerm;
        // Conditional integration: berhenti mengintegrasi jika output sudah
        // saturasi DAN error masih mendorong ke arah saturasi yang sama.
        const pushingHigh = unsat > this.outMax && error > 0;
        const pushingLow = unsat < this.outMin && error < 0;
        if (!pushingHigh && !pushingLow) this.iTerm = iCandidate;
        this.iTerm = clamp(this.iTerm, this.outMin, this.outMax);
      } else {
        this.iTerm = iCandidate;
      }

      const raw = this.pTerm + this.iTerm + this.dTerm;
      this.output = clamp(raw, this.outMin, this.outMax);
      this.saturated = this.output !== raw;

      this.error = error;
      this.prevError = error;
      this.prevMeasurement = measurement;
      return this.output;
    }
  }

  return { PIDController };
});

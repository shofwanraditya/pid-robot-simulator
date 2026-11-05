/**
 * runner.js — memisahkan laju fisika (kontinu, step kecil) dan laju kontroler (diskrit).
 *
 * Ini meniru kondisi mikrokontroler sungguhan: plant berjalan kontinu,
 * sedangkan loop PID hanya dieksekusi tiap Ts = 1 / controlHz detik
 * (mis. dari timer interrupt 100 Hz di ESP32).
 */
(function (root, factory) {
  const mod = factory();
  if (typeof module === 'object' && module.exports) module.exports = mod;
  else root.PIDSim = Object.assign(root.PIDSim || {}, mod);
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  class SimRunner {
    /**
     * @param {object} sim objek simulasi: { t, physicsDt, control(dt), stepPhysics(dt), telemetry }
     * @param {object} opts { controlHz, onSample(telemetry) }
     */
    constructor(sim, { controlHz = 100, onSample = null } = {}) {
      this.sim = sim;
      this.onSample = onSample;
      this.setControlRate(controlHz);
      this.reset();
    }

    setControlRate(hz) {
      this.controlHz = hz;
      this.controlDt = 1 / hz;
    }

    reset() {
      this.acc = 0;
      this.nextControl = this.sim.t;
    }

    /** Majukan simulasi sebanyak `seconds` detik waktu simulasi. */
    advance(seconds, maxSteps = 50000) {
      const dt = this.sim.physicsDt;
      this.acc += seconds;
      let steps = 0;
      while (this.acc >= dt && steps < maxSteps) {
        if (this.sim.t + 1e-9 >= this.nextControl) {
          this.sim.control(this.controlDt);
          this.nextControl += this.controlDt;
          if (this.nextControl < this.sim.t) this.nextControl = this.sim.t + this.controlDt;
          if (this.onSample) this.onSample(this.sim.telemetry);
        }
        this.sim.stepPhysics(dt);
        this.acc -= dt;
        steps++;
      }
      if (steps >= maxSteps) this.acc = 0; // cegah "spiral of death" saat tab lag
    }
  }

  return { SimRunner };
});

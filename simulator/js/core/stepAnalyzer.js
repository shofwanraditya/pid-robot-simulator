/**
 * stepAnalyzer.js — menghitung metrik respons step secara online:
 *   rise time (10%→90%), overshoot (%), settling time (band ±2%), steady-state error.
 */
(function (root, factory) {
  const mod = factory();
  if (typeof module === 'object' && module.exports) module.exports = mod;
  else root.PIDSim = Object.assign(root.PIDSim || {}, mod);
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  class StepAnalyzer {
    constructor({ band = 0.02, holdTime = 0.5, ssWindow = 0.5 } = {}) {
      this.band = band; // toleransi settling (fraksi dari besar step)
      this.holdTime = holdTime; // harus bertahan di dalam band selama ini agar dianggap settled
      this.ssWindow = ssWindow; // jendela rata-rata steady-state error
      this.active = false;
    }

    start(t0, y0, target) {
      this.t0 = t0;
      this.y0 = y0;
      this.target = target;
      this.span = target - y0;
      this.active = Math.abs(this.span) > 1e-9;
      this.t10 = null;
      this.t90 = null;
      this.peak = y0;
      this.lastOutside = t0;
      this.tLast = t0;
      this.recent = [];
    }

    add(t, y) {
      if (!this.active) return;
      const dir = Math.sign(this.span);
      const frac = (y - this.y0) / this.span;
      if (this.t10 === null && frac >= 0.1) this.t10 = t;
      if (this.t90 === null && frac >= 0.9) this.t90 = t;
      if ((y - this.peak) * dir > 0) this.peak = y;
      if (Math.abs(y - this.target) > this.band * Math.abs(this.span)) this.lastOutside = t;
      this.tLast = t;
      this.recent.push([t, y]);
      while (this.recent.length && this.recent[0][0] < t - this.ssWindow) this.recent.shift();
    }

    metrics() {
      if (!this.active) return null;
      const dir = Math.sign(this.span);
      const settled = this.tLast - this.lastOutside >= this.holdTime;
      let sse = null;
      if (this.recent.length) {
        let s = 0;
        for (const [, y] of this.recent) s += this.target - y;
        sse = s / this.recent.length;
      }
      return {
        riseTime: this.t10 !== null && this.t90 !== null ? this.t90 - this.t10 : null,
        overshoot: Math.max(0, ((this.peak - this.target) * dir) / Math.abs(this.span)) * 100,
        settlingTime: settled ? this.lastOutside - this.t0 : null,
        steadyStateError: sse,
        settled,
      };
    }
  }

  return { StepAnalyzer };
});

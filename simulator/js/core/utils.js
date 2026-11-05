/**
 * utils.js — helper matematika umum.
 * Dipakai di browser (global `PIDSim`) maupun Node.js (CommonJS) untuk unit test.
 */
(function (root, factory) {
  const mod = factory();
  if (typeof module === 'object' && module.exports) module.exports = mod;
  else root.PIDSim = Object.assign(root.PIDSim || {}, mod);
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  /** Membatasi nilai v ke rentang [lo, hi]. */
  function clamp(v, lo, hi) {
    return v < lo ? lo : v > hi ? hi : v;
  }

  /** PRNG deterministik (Mulberry32) — supaya test & demo bisa direproduksi. */
  function mulberry32(seed) {
    let a = seed >>> 0;
    return function () {
      a = (a + 0x6d2b79f5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  /** Generator noise Gaussian N(0,1) (metode polar Marsaglia). */
  function makeGaussian(rng = Math.random) {
    let spare = null;
    return function () {
      if (spare !== null) {
        const s = spare;
        spare = null;
        return s;
      }
      let u, v, s;
      do {
        u = rng() * 2 - 1;
        v = rng() * 2 - 1;
        s = u * u + v * v;
      } while (s >= 1 || s === 0);
      const m = Math.sqrt((-2 * Math.log(s)) / s);
      spare = v * m;
      return u * m;
    };
  }

  const DEG = 180 / Math.PI;

  return { clamp, mulberry32, makeGaussian, DEG };
});

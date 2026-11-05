/**
 * cartPole.js — Inverted pendulum di atas kereta (cart-pole).
 *
 * Model non-linear (pendulum batang seragam, setengah panjang l):
 *   θ̈ = [ g sinθ − cosθ · (F' + m l θ̇² sinθ)/(M+m) ] / [ l (4/3 − m cos²θ/(M+m)) ]
 *   ẍ = (F' + m l (θ̇² sinθ − θ̈ cosθ)) / (M+m),     F' = F − b ẋ
 *
 * Struktur kontrol (cascade, seperti robot self-balancing):
 *   Loop luar  : posisi kereta x  → setpoint sudut θref   (PID posisi)
 *   Loop dalam : sudut θ          → gaya F               (PID sudut, slider utama)
 */
(function (root, factory) {
  const isNode = typeof module === 'object' && module.exports;
  const D = isNode ? Object.assign({}, require('../core/utils.js'), require('../core/pid.js')) : root.PIDSim;
  const mod = factory(D);
  if (isNode) module.exports = mod;
  else root.PIDSim = Object.assign(root.PIDSim || {}, mod);
})(typeof self !== 'undefined' ? self : this, function (D) {
  'use strict';
  const { makeGaussian, PIDController, DEG } = D;

  class CartPoleSim {
    constructor(opts = {}) {
      this.gauss = makeGaussian(opts.rng || Math.random);
      this.physicsDt = 1 / 1000;
      this.params = { M: 1.0, m: 0.2, l: 0.5, g: 9.81, bCart: 0.3, fMax: 40, xLimit: 2.0 };
      this.noiseStd = opts.noiseStd ?? 0.1; // derajat (noise IMU)
      this.cascade = opts.cascade ?? true;
      this.xTarget = 0;
      this.initialAngleDeg = opts.initialAngleDeg ?? 6;
      const g = CartPoleSim.meta.defaultGains;
      this.pid = new PIDController({
        kp: g.kp, ki: g.ki, kd: g.kd,
        outMin: -this.params.fMax, outMax: this.params.fMax,
        dFilterTau: 0.004,
      });
      this.posPid = new PIDController({ kp: 3, ki: 0, kd: 4, outMin: -8, outMax: 8, dFilterTau: 0.02 });
      this.falls = 0;
      this.reset();
    }

    reset() {
      this.t = 0;
      this.falls = 0;
      this.respawn();
      this.telemetry = { t: 0, setpoint: 0, measurement: 0, output: 0, p: 0, i: 0, d: 0 };
    }

    respawn() {
      this.x = 0;
      this.xd = 0;
      this.th = this.initialAngleDeg / DEG;
      this.thd = 0;
      this.force = 0;
      this.pushForce = 0;
      this.pushTime = 0;
      this.thRef = 0;
      this.fallen = false;
      this.fallTimer = 0;
      this.balanceStart = this.t;
      this.absErrAvg = 0;
      this.pid.reset();
      this.posPid.reset();
    }

    push(force, duration = 0.1) {
      this.pushForce = force;
      this.pushTime = duration;
    }

    readSensor() {
      return this.th * DEG + this.noiseStd * this.gauss();
    }

    control(dt) {
      const thMeas = this.readSensor();
      if (this.fallen) {
        this.force = 0;
      } else {
        this.thRef = this.cascade ? this.posPid.compute(this.xTarget, this.x, dt) : 0;
        // Reverse-acting: pendulum miring ke kanan (θ>0) → kereta harus didorong ke kanan (F>0)
        this.force = -this.pid.compute(this.thRef, thMeas, dt);
      }
      this.absErrAvg += (Math.abs(this.th * DEG) - this.absErrAvg) * Math.min(1, dt / 2);
      const p = this.pid;
      this.telemetry = {
        t: this.t, setpoint: this.thRef, measurement: thMeas, output: this.force,
        p: -p.pTerm, i: -p.iTerm, d: -p.dTerm,
      };
    }

    stepPhysics(dt) {
      const { M, m, l, g, bCart, xLimit } = this.params;
      let F = this.force;
      if (this.pushTime > 0) {
        F += this.pushForce;
        this.pushTime -= dt;
      }
      const s = Math.sin(this.th), c = Math.cos(this.th);
      const total = M + m, ml = m * l;
      const temp = (F - bCart * this.xd + ml * this.thd * this.thd * s) / total;
      const thAcc = (g * s - c * temp) / (l * (4 / 3 - (m * c * c) / total));
      const xAcc = temp - (ml * thAcc * c) / total;
      this.xd += xAcc * dt;
      this.x += this.xd * dt;
      this.thd += thAcc * dt;
      this.th += this.thd * dt;
      this.t += dt;

      if (Math.abs(this.x) > xLimit) {
        this.x = Math.sign(this.x) * xLimit;
        this.xd = 0;
      }
      if (!this.fallen && Math.abs(this.th) > 1.2) {
        this.fallen = true;
        this.falls++;
      }
      if (this.fallen) {
        // Pendulum rebah ke lantai, lalu respawn otomatis
        this.th = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, this.th));
        if (Math.abs(this.th) >= Math.PI / 2 - 1e-6) this.thd = 0;
        this.fallTimer += dt;
        if (this.fallTimer > 1.5) this.respawn();
      }
    }

    getControls() {
      return [
        {
          type: 'checkbox', label: 'Cascade: kunci posisi kereta (loop luar)',
          get: () => this.cascade, set: (v) => { this.cascade = v; this.posPid.reset(); },
        },
        {
          type: 'range', label: 'Target posisi kereta', unit: 'm', min: -1.5, max: 1.5, step: 0.05,
          get: () => this.xTarget, set: (v) => (this.xTarget = v),
        },
        {
          type: 'range', label: 'Loop luar Kp (°/m)', unit: '', min: 0, max: 10, step: 0.1,
          get: () => this.posPid.kp, set: (v) => (this.posPid.kp = v),
        },
        {
          type: 'range', label: 'Loop luar Kd (°/(m/s))', unit: '', min: 0, max: 10, step: 0.1,
          get: () => this.posPid.kd, set: (v) => (this.posPid.kd = v),
        },
        {
          type: 'range', label: 'Noise IMU (σ)', unit: '°', min: 0, max: 2, step: 0.05,
          get: () => this.noiseStd, set: (v) => (this.noiseStd = v),
        },
        {
          type: 'range', label: 'Massa pendulum', unit: 'kg', min: 0.05, max: 1, step: 0.05,
          get: () => this.params.m, set: (v) => (this.params.m = v),
        },
        {
          type: 'buttons', label: 'Gangguan (dorongan 0.1 s)',
          buttons: [
            { label: '⟵ Dorong 25 N', onClick: () => this.push(-25) },
            { label: 'Dorong 25 N ⟶', onClick: () => this.push(25) },
          ],
        },
      ];
    }

    getMetrics() {
      const bal = this.fallen ? 0 : this.t - this.balanceStart;
      return [
        { label: 'Sudut θ', value: `${(this.th * DEG).toFixed(2)}°` },
        { label: 'Posisi kereta', value: `${this.x.toFixed(2)} m`, warn: Math.abs(this.x) >= this.params.xLimit - 0.01 },
        { label: 'Gaya F', value: `${this.force.toFixed(1)} N`, warn: this.pid.saturated },
        { label: 'Rata-rata |θ|', value: `${this.absErrAvg.toFixed(2)}°` },
        { label: 'Waktu seimbang', value: `${bal.toFixed(1)} s` },
        { label: 'Jatuh', value: String(this.falls), warn: this.falls > 0 },
        { label: 'Status', value: this.fallen ? 'JATUH' : 'SEIMBANG', warn: this.fallen },
      ];
    }

    draw(ctx, W, H) {
      ctx.clearRect(0, 0, W, H);
      const { xLimit, l } = this.params;
      const sc = Math.min(W / (2 * xLimit + 1.0), H / 1.9);
      const gx = W / 2, gy = H * 0.78;
      const X = (x) => gx + x * sc;

      // Rel
      ctx.fillStyle = '#1e293b';
      ctx.fillRect(X(-xLimit - 0.25), gy, (2 * xLimit + 0.5) * sc, 6);
      ctx.fillStyle = '#ef4444';
      ctx.fillRect(X(-xLimit - 0.25), gy - 30, 6, 36);
      ctx.fillRect(X(xLimit + 0.25) - 6, gy - 30, 6, 36);
      ctx.fillStyle = '#64748b';
      ctx.font = '11px ui-monospace, monospace';
      ctx.textAlign = 'center';
      for (let v = -2; v <= 2; v++) {
        ctx.fillRect(X(v) - 1, gy + 6, 2, 6);
        ctx.fillText(`${v} m`, X(v), gy + 24);
      }

      // Target posisi
      if (this.cascade) {
        ctx.fillStyle = '#f59e0b';
        ctx.beginPath();
        ctx.moveTo(X(this.xTarget), gy + 8);
        ctx.lineTo(X(this.xTarget) - 8, gy + 20);
        ctx.lineTo(X(this.xTarget) + 8, gy + 20);
        ctx.fill();
      }

      // Kereta
      const cw = 0.42 * sc, ch = 0.2 * sc;
      const cx = X(this.x), cy = gy - ch - 0.06 * sc;
      ctx.fillStyle = '#2563eb';
      ctx.beginPath();
      ctx.roundRect ? ctx.roundRect(cx - cw / 2, cy, cw, ch, 6) : ctx.rect(cx - cw / 2, cy, cw, ch);
      ctx.fill();
      ctx.fillStyle = '#0f172a';
      [-0.12, 0.12].forEach((o) => {
        ctx.beginPath();
        ctx.arc(cx + o * sc, gy - 0.035 * sc, 0.035 * sc, 0, Math.PI * 2);
        ctx.fill();
      });

      // Pendulum
      const L = 2 * l * sc;
      const px = cx, py = cy + 4;
      const tx = px + L * Math.sin(this.th), ty = py - L * Math.cos(this.th);
      ctx.strokeStyle = this.fallen ? '#ef4444' : '#e2e8f0';
      ctx.lineWidth = Math.max(4, 0.03 * sc);
      ctx.lineCap = 'round';
      ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(tx, ty); ctx.stroke();
      ctx.fillStyle = '#f59e0b';
      ctx.beginPath(); ctx.arc(px, py, 6, 0, Math.PI * 2); ctx.fill();

      // Garis referensi sudut
      ctx.setLineDash([4, 6]);
      ctx.strokeStyle = '#475569';
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px, py - L * 1.05); ctx.stroke();
      ctx.setLineDash([]);

      // Panah gaya
      const fLen = (this.force / this.params.fMax) * 0.6 * sc;
      if (Math.abs(fLen) > 2) {
        const ay = cy + ch / 2;
        const ax0 = fLen > 0 ? cx - cw / 2 - 4 - fLen : cx + cw / 2 + 4 - fLen;
        const ax1 = fLen > 0 ? cx - cw / 2 - 4 : cx + cw / 2 + 4;
        ctx.strokeStyle = this.pid.saturated ? '#ef4444' : '#a78bfa';
        ctx.fillStyle = ctx.strokeStyle;
        ctx.lineWidth = 4;
        ctx.beginPath(); ctx.moveTo(ax0, ay); ctx.lineTo(ax1, ay); ctx.stroke();
        const dir = Math.sign(fLen);
        ctx.beginPath();
        ctx.moveTo(ax1, ay);
        ctx.lineTo(ax1 - dir * 10, ay - 7);
        ctx.lineTo(ax1 - dir * 10, ay + 7);
        ctx.fill();
      }

      // HUD
      ctx.textAlign = 'left';
      ctx.fillStyle = '#e2e8f0';
      ctx.font = 'bold 15px ui-monospace, monospace';
      ctx.fillText(`θ = ${(this.th * DEG).toFixed(2)}°`, 14, 24);
      ctx.font = '13px ui-monospace, monospace';
      ctx.fillStyle = '#94a3b8';
      ctx.fillText(`θref = ${this.thRef.toFixed(2)}°   F = ${this.force.toFixed(1)} N   x = ${this.x.toFixed(2)} m`, 14, 44);
      if (this.fallen) {
        ctx.fillStyle = '#ef4444';
        ctx.font = 'bold 22px ui-sans-serif, system-ui';
        ctx.textAlign = 'center';
        ctx.fillText('JATUH! respawn...', W / 2, H * 0.3);
      }
    }
  }

  CartPoleSim.meta = {
    id: 'pendulum',
    title: 'Inverted Pendulum (Cart-Pole)',
    description:
      'Sistem tidak stabil secara alami — prinsip yang sama dengan robot self-balancing 2 roda. Loop dalam (slider utama) menjaga sudut tegak, loop luar opsional menjaga posisi kereta (cascade PID).',
    measurementLabel: 'Sudut θ (°)',
    outputLabel: 'Gaya F (N)',
    defaultGains: { kp: 2.0, ki: 0, kd: 0.12 },
    gainRanges: { kp: [0, 10, 0.01], ki: [0, 10, 0.01], kd: [0, 1, 0.005] },
    controlHz: 200,
    presets: [
      { name: '✅ Tuned PD (default)', desc: 'Seimbang stabil, kembali ke target posisi.', gains: { kp: 2.0, ki: 0, kd: 0.12 } },
      { name: 'P saja (osilasi)', desc: 'Tanpa redaman D → berosilasi lalu jatuh.', gains: { kp: 2.0, ki: 0, kd: 0 } },
      { name: 'Kp terlalu kecil', desc: 'Gaya tidak cukup melawan gravitasi → langsung jatuh.', gains: { kp: 0.15, ki: 0, kd: 0.03 } },
      { name: 'Agresif', desc: 'Gain besar → sering saturasi gaya dan sensitif terhadap noise.', gains: { kp: 7, ki: 0, kd: 0.6 } },
    ],
  };

  return { CartPoleSim };
});

/**
 * dcMotor.js — kontrol kecepatan motor DC (model elektromekanik orde-2).
 *
 *   L di/dt = V - R i - Ke ω           (sisi elektrik)
 *   J dω/dt = Kt i - b ω - τ_load      (sisi mekanik)
 *
 * Plant klasik untuk mempelajari respons step: rise time, overshoot, settling time,
 * steady-state error, dan efek saturasi tegangan (integral windup).
 */
(function (root, factory) {
  const isNode = typeof module === 'object' && module.exports;
  const D = isNode
    ? Object.assign({}, require('../core/utils.js'), require('../core/pid.js'), require('../core/stepAnalyzer.js'))
    : root.PIDSim;
  const mod = factory(D);
  if (isNode) module.exports = mod;
  else root.PIDSim = Object.assign(root.PIDSim || {}, mod);
})(typeof self !== 'undefined' ? self : this, function (D) {
  'use strict';
  const { makeGaussian, PIDController, StepAnalyzer } = D;

  const RPM_PER_RADS = 30 / Math.PI;

  class DCMotorSim {
    constructor(opts = {}) {
      this.params = Object.assign(
        { R: 1.0, L: 0.01, Kt: 0.05, Ke: 0.05, J: 0.001, b: 0.0005, vMax: 24 },
        opts.params
      );
      this.gauss = makeGaussian(opts.rng || Math.random);
      this.noiseStd = opts.noiseStd ?? 5; // RPM (noise encoder)
      this.physicsDt = 1 / 2000;
      const g = DCMotorSim.meta.defaultGains;
      this.pid = new PIDController({
        kp: g.kp, ki: g.ki, kd: g.kd,
        outMin: -this.params.vMax, outMax: this.params.vMax,
        dFilterTau: 0.01,
      });
      this.setpoint = opts.setpoint ?? 1000;
      this.analyzer = new StepAnalyzer();
      this.reset();
    }

    reset() {
      this.t = 0;
      this.omega = 0;
      this.current = 0;
      this.voltage = 0;
      this.angle = 0;
      this.load = this.load || 0;
      this.pid.reset();
      this.analyzer.start(0, 0, this.setpoint);
      this.telemetry = { t: 0, setpoint: this.setpoint, measurement: 0, output: 0, p: 0, i: 0, d: 0 };
    }

    get rpm() {
      return this.omega * RPM_PER_RADS;
    }

    setSetpoint(rpm) {
      if (rpm === this.setpoint) return;
      this.setpoint = rpm;
      this.analyzer.start(this.t, this.rpm, rpm);
    }

    readSensor() {
      return this.rpm + this.noiseStd * this.gauss();
    }

    control(dt) {
      const y = this.readSensor();
      this.voltage = this.pid.compute(this.setpoint, y, dt);
      this.analyzer.add(this.t, this.rpm);
      const p = this.pid;
      this.telemetry = {
        t: this.t, setpoint: this.setpoint, measurement: y, output: this.voltage,
        p: p.pTerm, i: p.iTerm, d: p.dTerm,
      };
    }

    stepPhysics(dt) {
      const { R, L, Kt, Ke, J, b } = this.params;
      const di = (this.voltage - R * this.current - Ke * this.omega) / L;
      this.current += di * dt;
      const tauLoad = this.load * Math.tanh(this.omega / 2); // beban gesek halus di sekitar 0
      const dw = (Kt * this.current - b * this.omega - tauLoad) / J;
      this.omega += dw * dt;
      this.angle += this.omega * dt;
      this.t += dt;
    }

    getControls() {
      return [
        {
          type: 'range', label: 'Setpoint kecepatan', unit: 'RPM', min: -3000, max: 3000, step: 50,
          get: () => this.setpoint, set: (v) => this.setSetpoint(v),
        },
        {
          type: 'buttons', label: 'Step cepat',
          buttons: [0, 500, 1000, 2000, 3000].map((v) => ({ label: `${v}`, onClick: () => this.setSetpoint(v) })),
        },
        {
          type: 'range', label: 'Torsi beban (gangguan)', unit: 'N·m', min: 0, max: 0.15, step: 0.005,
          get: () => this.load, set: (v) => (this.load = v),
        },
        {
          type: 'range', label: 'Noise encoder (σ)', unit: 'RPM', min: 0, max: 60, step: 1,
          get: () => this.noiseStd, set: (v) => (this.noiseStd = v),
        },
      ];
    }

    getMetrics() {
      const m = this.analyzer.metrics();
      const fmt = (v, d, u) => (v === null || v === undefined ? '—' : `${v.toFixed(d)} ${u}`);
      return [
        { label: 'Kecepatan', value: `${this.rpm.toFixed(0)} RPM` },
        { label: 'Tegangan', value: `${this.voltage.toFixed(2)} V`, warn: this.pid.saturated },
        { label: 'Arus', value: `${this.current.toFixed(2)} A` },
        { label: 'Rise time', value: m ? fmt(m.riseTime, 3, 's') : '—' },
        { label: 'Overshoot', value: m ? fmt(m.overshoot, 1, '%') : '—', warn: m && m.overshoot > 10 },
        { label: 'Settling (±2%)', value: m ? fmt(m.settlingTime, 3, 's') : '—' },
        { label: 'Steady-state err', value: m ? fmt(m.steadyStateError, 1, 'RPM') : '—' },
      ];
    }

    draw(ctx, W, H) {
      ctx.clearRect(0, 0, W, H);
      const cx = W * 0.3, cy = H * 0.55, R = Math.min(W * 0.22, H * 0.38);
      const maxRpm = 3000;
      const a0 = Math.PI * 0.75, a1 = Math.PI * 2.25;
      const toAng = (rpm) => a0 + ((Math.min(Math.abs(rpm), maxRpm)) / maxRpm) * (a1 - a0);

      // Gauge
      ctx.lineCap = 'round';
      ctx.lineWidth = R * 0.12;
      ctx.strokeStyle = '#1f2937';
      ctx.beginPath(); ctx.arc(cx, cy, R, a0, a1); ctx.stroke();
      ctx.strokeStyle = '#22d3ee';
      ctx.beginPath(); ctx.arc(cx, cy, R, a0, toAng(this.rpm)); ctx.stroke();

      // Tick labels
      ctx.fillStyle = '#94a3b8';
      ctx.font = `${Math.max(10, R * 0.09)}px ui-monospace, monospace`;
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      for (let v = 0; v <= maxRpm; v += 500) {
        const a = toAng(v);
        ctx.fillText(String(v), cx + Math.cos(a) * R * 0.75, cy + Math.sin(a) * R * 0.75);
      }
      // Setpoint marker
      const as = toAng(this.setpoint);
      ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 4;
      ctx.beginPath();
      ctx.moveTo(cx + Math.cos(as) * R * 0.86, cy + Math.sin(as) * R * 0.86);
      ctx.lineTo(cx + Math.cos(as) * R * 1.14, cy + Math.sin(as) * R * 1.14);
      ctx.stroke();

      // Rotor (visual diperlambat 25x agar tidak aliasing)
      const rr = R * 0.32;
      ctx.save();
      ctx.translate(cx, cy);
      ctx.rotate(this.angle / 25);
      ctx.fillStyle = '#334155';
      ctx.beginPath(); ctx.arc(0, 0, rr, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = '#e2e8f0'; ctx.lineWidth = 3;
      for (let k = 0; k < 4; k++) {
        ctx.rotate(Math.PI / 2);
        ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(rr * 0.9, 0); ctx.stroke();
      }
      ctx.restore();

      ctx.fillStyle = '#e2e8f0';
      ctx.font = `bold ${Math.max(14, R * 0.16)}px ui-monospace, monospace`;
      ctx.fillText(`${this.rpm.toFixed(0)} RPM`, cx, cy + R * 0.62);
      ctx.fillStyle = '#f59e0b';
      ctx.font = `${Math.max(11, R * 0.1)}px ui-monospace, monospace`;
      ctx.fillText(`target ${this.setpoint} RPM`, cx, cy + R * 0.82);
      ctx.fillStyle = '#64748b';
      ctx.fillText('(rotor visual diperlambat 25×)', cx, cy - R * 1.22);

      // Bar tegangan & arus
      const bars = [
        { label: 'Tegangan PWM', val: this.voltage, max: this.params.vMax, unit: 'V', color: this.pid.saturated ? '#ef4444' : '#a78bfa' },
        { label: 'Arus motor', val: this.current, max: 20, unit: 'A', color: '#34d399' },
        { label: 'Torsi beban', val: this.load, max: 0.15, unit: 'N·m', color: '#f87171' },
      ];
      const bx = W * 0.6, bw = W * 0.32, bh = 18;
      bars.forEach((b, k) => {
        const y = H * 0.22 + k * 70;
        ctx.textAlign = 'left';
        ctx.fillStyle = '#94a3b8';
        ctx.font = '13px ui-sans-serif, system-ui';
        ctx.fillText(`${b.label}: ${b.val.toFixed(b.unit === 'N·m' ? 3 : 2)} ${b.unit}`, bx, y - 12);
        ctx.fillStyle = '#1f2937';
        ctx.fillRect(bx, y, bw, bh);
        const mid = bx + bw / 2;
        const frac = Math.max(-1, Math.min(1, b.val / b.max));
        ctx.fillStyle = b.color;
        if (frac >= 0) ctx.fillRect(mid, y, (bw / 2) * frac, bh);
        else ctx.fillRect(mid + (bw / 2) * frac, y, (-bw / 2) * frac, bh);
        ctx.fillStyle = '#475569';
        ctx.fillRect(mid - 1, y - 3, 2, bh + 6);
      });
      if (this.pid.saturated) {
        ctx.fillStyle = '#ef4444';
        ctx.font = 'bold 13px ui-sans-serif, system-ui';
        ctx.fillText(`⚠ OUTPUT SATURASI (±${this.params.vMax} V)`, bx, H * 0.22 + 3 * 70 - 10);
      }
    }
  }

  DCMotorSim.meta = {
    id: 'motor',
    title: 'Motor DC — Kontrol Kecepatan',
    description:
      'Plant klasik untuk memahami respons step. Ubah setpoint dan amati rise time, overshoot, settling time, dan steady-state error. Tambahkan torsi beban untuk melihat peran suku Integral.',
    measurementLabel: 'Kecepatan (RPM)',
    outputLabel: 'Tegangan (V)',
    defaultGains: { kp: 0.03, ki: 0.1, kd: 0.0004 },
    gainRanges: { kp: [0, 0.2, 0.001], ki: [0, 2, 0.005], kd: [0, 0.005, 0.0001] },
    controlHz: 200,
    presets: [
      { name: '✅ Tuned (default)', desc: 'Respons cepat, overshoot kecil, tanpa steady-state error.', gains: { kp: 0.03, ki: 0.1, kd: 0.0004 } },
      { name: 'P saja', desc: 'Tanpa Ki → selalu ada steady-state error, apalagi saat diberi beban.', gains: { kp: 0.03, ki: 0, kd: 0 } },
      { name: 'PI agresif', desc: 'Ki terlalu besar → overshoot & osilasi.', gains: { kp: 0.01, ki: 0.6, kd: 0 } },
      { name: 'Integral windup', desc: 'Ki besar + saturasi tegangan. Matikan anti-windup untuk melihat overshoot ekstrem.', gains: { kp: 0.01, ki: 0.4, kd: 0 }, antiWindup: false },
      { name: 'Kd berlebih + noise', desc: 'Suku D memperkuat noise encoder → tegangan "kasar".', gains: { kp: 0.03, ki: 0.1, kd: 0.004 }, dFilterTau: 0 },
    ],
  };

  return { DCMotorSim };
});

/**
 * lineFollower.js — robot differential-drive dengan 5 sensor IR analog.
 *
 * Kinematika (koordinat canvas, sumbu-y ke bawah):
 *   v = (vL + vR) / 2,   ω = (vL - vR) / B
 *   ẋ = v cos θ,  ẏ = v sin θ,  θ̇ = ω
 * Motor dimodelkan sebagai sistem orde-1 (time constant τ) agar ada dinamika nyata.
 *
 * Sensor: 5 IR reflektansi di depan robot. Error = rata-rata berbobot posisi sensor
 * (persis seperti yang dilakukan di firmware line follower sungguhan).
 */
(function (root, factory) {
  const isNode = typeof module === 'object' && module.exports;
  const D = isNode ? Object.assign({}, require('../core/utils.js'), require('../core/pid.js')) : root.PIDSim;
  const mod = factory(D);
  if (isNode) module.exports = mod;
  else root.PIDSim = Object.assign(root.PIDSim || {}, mod);
})(typeof self !== 'undefined' ? self : this, function (D) {
  'use strict';
  const { clamp, makeGaussian, PIDController } = D;

  const FIELD = { w: 1600, h: 1000 }; // mm

  const TRACKS = {
    oval: {
      label: 'Oval (mudah)',
      fn: (t) => [800 + 600 * Math.cos(t), 500 + 330 * Math.sin(t)],
    },
    blob: {
      label: 'Kurva S (sedang)',
      fn: (t) => {
        const r = 1 + 0.16 * Math.sin(3 * t) + 0.07 * Math.cos(5 * t);
        return [800 + 580 * r * Math.cos(t), 500 + 360 * r * Math.sin(t)];
      },
    },
    clover: {
      label: 'Clover (tikungan tajam)',
      fn: (t) => {
        const r = 1 + 0.32 * Math.cos(4 * t);
        return [800 + 520 * r * Math.cos(t), 500 + 340 * r * Math.sin(t)];
      },
    },
  };

  function buildTrack(key, N = 2400) {
    const fn = TRACKS[key].fn;
    const pts = new Float64Array(2 * N);
    for (let i = 0; i < N; i++) {
      const [x, y] = fn((2 * Math.PI * i) / N);
      pts[2 * i] = x;
      pts[2 * i + 1] = y;
    }
    let length = 0;
    for (let i = 0; i < N; i++) {
      const j = (i + 1) % N;
      length += Math.hypot(pts[2 * j] - pts[2 * i], pts[2 * j + 1] - pts[2 * i + 1]);
    }
    return { key, N, pts, length };
  }

  /** Cari titik lintasan terdekat. Jika `hint` diberikan, hanya cari di sekitar indeks itu. */
  function nearestIndex(track, x, y, hint, win) {
    const { N, pts } = track;
    const full = hint == null || 2 * win + 1 >= N;
    const start = full ? 0 : hint - win;
    const end = full ? N - 1 : hint + win;
    let best = 0, bestD = Infinity;
    for (let k = start; k <= end; k++) {
      const i = ((k % N) + N) % N;
      const dx = pts[2 * i] - x, dy = pts[2 * i + 1] - y;
      const d = dx * dx + dy * dy;
      if (d < bestD) { bestD = d; best = i; }
    }
    return { idx: best, dist: Math.sqrt(bestD) };
  }

  // Posisi lateral sensor (mm), positif = kiri robot. Indeks 0 = paling kiri.
  const SENSOR_POS = [2, 1, 0, -1, -2];

  class LineFollowerSim {
    constructor(opts = {}) {
      this.gauss = makeGaussian(opts.rng || Math.random);
      this.physicsDt = 1 / 1000;
      this.params = {
        wheelBase: 120, // mm
        motorTau: 0.06, // s
        vMax: 900, // mm/s
        sensorSpacing: 15, // mm
        sensorLookahead: 70, // mm di depan poros roda
        lineWidth: 19, // mm (lakban 3/4")
      };
      this.baseSpeed = opts.baseSpeed ?? 350;
      this.noiseStd = opts.noiseStd ?? 0.03;
      const g = LineFollowerSim.meta.defaultGains;
      this.pid = new PIDController({
        kp: g.kp, ki: g.ki, kd: g.kd,
        outMin: -this.params.vMax, outMax: this.params.vMax,
        dFilterTau: 0.005,
      });
      this.trackKey = opts.track || 'blob';
      this.track = buildTrack(this.trackKey);
      this.reset();
    }

    setTrack(key) {
      this.trackKey = key;
      this.track = buildTrack(key);
      this.reset();
    }

    reset() {
      this.t = 0;
      this.laps = 0;
      this.lastLap = null;
      this.bestLap = null;
      this.offTracks = 0;
      this.rmsSq = 0;
      this.maxErr = 0;
      this.respawn(0);
      this.telemetry = { t: 0, setpoint: 0, measurement: 0, output: 0, p: 0, i: 0, d: 0 };
    }

    respawn(idx) {
      const { pts, N } = this.track;
      const j = (idx + 3) % N;
      this.x = pts[2 * idx];
      this.y = pts[2 * idx + 1];
      this.th = Math.atan2(pts[2 * j + 1] - this.y, pts[2 * j] - this.x);
      this.vL = this.vR = this.vLcmd = this.vRcmd = 0;
      this.robotIdx = idx;
      this.crossTrack = 0;
      this.progress = 0;
      this.lapStart = this.t;
      this.lastSign = 0;
      this.lost = false;
      this.lineError = 0;
      this.readings = [0, 0, 0, 0, 0];
      this.trail = [];
      this.lastTrail = -1;
      this.pid.reset();
    }

    /** Baca 5 sensor IR dan kembalikan error posisi garis (mm, positif = garis di kiri). */
    readSensor() {
      const { sensorSpacing: s, sensorLookahead: L, lineWidth: w } = this.params;
      const c = Math.cos(this.th), sn = Math.sin(this.th);
      const lx = sn, ly = -c; // vektor normal "kiri" (y ke bawah)
      const fx = this.x + L * c, fy = this.y + L * sn;
      const center = nearestIndex(this.track, fx, fy, this.robotIdx, 150);
      let sum = 0, wsum = 0;
      for (let k = 0; k < 5; k++) {
        const pos = SENSOR_POS[k] * s;
        const d = nearestIndex(this.track, fx + pos * lx, fy + pos * ly, center.idx, 40).dist;
        const r = clamp(Math.exp(-((d / (w * 0.75)) ** 2)) + this.noiseStd * this.gauss(), 0, 1);
        this.readings[k] = r;
        sum += r;
        wsum += r * pos;
      }
      let e;
      if (sum < 0.3) {
        // Garis hilang: asumsikan garis ada di sisi terakhir yang terlihat
        this.lost = true;
        e = (this.lastSign >= 0 ? 1 : -1) * 3 * s;
      } else {
        this.lost = false;
        e = wsum / sum;
        if (Math.abs(e) > 1) this.lastSign = Math.sign(e);
      }
      this.lineError = e;
      return e;
    }

    control(dt) {
      const e = this.readSensor();
      // Setpoint 0: garis tepat di tengah sensor array
      const u = this.pid.compute(0, e, dt);
      const vMax = this.params.vMax;
      this.vLcmd = clamp(this.baseSpeed + u, -vMax, vMax);
      this.vRcmd = clamp(this.baseSpeed - u, -vMax, vMax);

      this.rmsSq += (this.crossTrack ** 2 - this.rmsSq) * Math.min(1, dt / 3);
      if (this.t - this.lapStart > 1) this.maxErr = Math.max(this.maxErr, this.crossTrack);
      if (this.t - this.lastTrail >= 0.02) {
        this.trail.push([this.x, this.y, this.crossTrack]);
        if (this.trail.length > 700) this.trail.shift();
        this.lastTrail = this.t;
      }
      const p = this.pid;
      this.telemetry = { t: this.t, setpoint: 0, measurement: e, output: u, p: p.pTerm, i: p.iTerm, d: p.dTerm };
    }

    stepPhysics(dt) {
      const { motorTau, wheelBase } = this.params;
      const k = Math.min(1, dt / motorTau);
      this.vL += (this.vLcmd - this.vL) * k;
      this.vR += (this.vRcmd - this.vR) * k;
      const v = (this.vL + this.vR) / 2;
      const w = (this.vL - this.vR) / wheelBase;
      this.x += v * Math.cos(this.th) * dt;
      this.y += v * Math.sin(this.th) * dt;
      this.th += w * dt;
      this.t += dt;

      // Tracking progres & lap
      const N = this.track.N;
      const n = nearestIndex(this.track, this.x, this.y, this.robotIdx, 30);
      let delta = n.idx - this.robotIdx;
      if (delta > N / 2) delta -= N;
      if (delta < -N / 2) delta += N;
      this.progress += delta;
      this.robotIdx = n.idx;
      this.crossTrack = n.dist;
      if (this.progress >= N) {
        const lapTime = this.t - this.lapStart;
        this.laps++;
        this.lastLap = lapTime;
        this.bestLap = this.bestLap === null ? lapTime : Math.min(this.bestLap, lapTime);
        this.lapStart = this.t;
        this.progress -= N;
      }

      // Keluar lintasan → respawn
      const out = this.x < 0 || this.y < 0 || this.x > FIELD.w || this.y > FIELD.h;
      if (n.dist > 200 || out) {
        this.offTracks++;
        const full = nearestIndex(this.track, this.x, this.y);
        this.respawn(full.idx);
      }
    }

    getControls() {
      return [
        {
          type: 'select', label: 'Lintasan',
          options: Object.entries(TRACKS).map(([value, t]) => ({ value, label: t.label })),
          get: () => this.trackKey, set: (v) => this.setTrack(v),
        },
        {
          type: 'range', label: 'Kecepatan dasar', unit: 'mm/s', min: 100, max: 800, step: 10,
          get: () => this.baseSpeed, set: (v) => (this.baseSpeed = v),
        },
        {
          type: 'range', label: 'Noise sensor IR (σ)', unit: '', min: 0, max: 0.3, step: 0.01,
          get: () => this.noiseStd, set: (v) => (this.noiseStd = v),
        },
        {
          type: 'range', label: 'Konstanta waktu motor τ', unit: 's', min: 0.01, max: 0.2, step: 0.01,
          get: () => this.params.motorTau, set: (v) => (this.params.motorTau = v),
        },
        {
          type: 'buttons', label: 'Gangguan',
          buttons: [
            { label: '↺ Senggol kiri', onClick: () => (this.th -= 0.35) },
            { label: 'Senggol kanan ↻', onClick: () => (this.th += 0.35) },
          ],
        },
      ];
    }

    getMetrics() {
      const f = (v) => (v === null ? '—' : `${v.toFixed(2)} s`);
      return [
        { label: 'Lap selesai', value: String(this.laps) },
        { label: 'Lap terakhir', value: f(this.lastLap) },
        { label: 'Lap terbaik', value: f(this.bestLap) },
        { label: 'RMS cross-track', value: `${Math.sqrt(this.rmsSq).toFixed(1)} mm`, warn: Math.sqrt(this.rmsSq) > 15 },
        { label: 'Error maks', value: `${this.maxErr.toFixed(1)} mm` },
        { label: 'Keluar lintasan', value: String(this.offTracks), warn: this.offTracks > 0 },
        { label: 'Status sensor', value: this.lost ? 'GARIS HILANG' : 'OK', warn: this.lost },
      ];
    }

    draw(ctx, W, H) {
      ctx.clearRect(0, 0, W, H);
      const sc = Math.min(W / FIELD.w, H / FIELD.h) * 0.97;
      const ox = (W - FIELD.w * sc) / 2, oy = (H - FIELD.h * sc) / 2;
      ctx.save();
      ctx.translate(ox, oy);
      ctx.scale(sc, sc);

      // Papan
      ctx.fillStyle = '#e5e7eb';
      ctx.fillRect(0, 0, FIELD.w, FIELD.h);

      // Garis lintasan
      const { pts, N } = this.track;
      ctx.strokeStyle = '#111827';
      ctx.lineWidth = this.params.lineWidth;
      ctx.lineJoin = 'round';
      ctx.beginPath();
      ctx.moveTo(pts[0], pts[1]);
      for (let i = 1; i < N; i++) ctx.lineTo(pts[2 * i], pts[2 * i + 1]);
      ctx.closePath();
      ctx.stroke();

      // Garis start
      const j = 3;
      const tx = pts[2 * j] - pts[0], ty = pts[2 * j + 1] - pts[1];
      const tl = Math.hypot(tx, ty);
      ctx.strokeStyle = '#ef4444';
      ctx.lineWidth = 6;
      ctx.beginPath();
      ctx.moveTo(pts[0] - (ty / tl) * 40, pts[1] + (tx / tl) * 40);
      ctx.lineTo(pts[0] + (ty / tl) * 40, pts[1] - (tx / tl) * 40);
      ctx.stroke();

      // Jejak robot (warna = besar error)
      ctx.lineWidth = 4;
      for (let i = 1; i < this.trail.length; i++) {
        const [x0, y0] = this.trail[i - 1];
        const [x1, y1, e] = this.trail[i];
        const k = Math.min(1, e / 30);
        ctx.strokeStyle = `rgba(${Math.round(34 + 205 * k)}, ${Math.round(197 - 130 * k)}, 94, ${0.25 + 0.6 * (i / this.trail.length)})`;
        ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
      }

      // Robot
      ctx.save();
      ctx.translate(this.x, this.y);
      ctx.rotate(this.th);
      ctx.fillStyle = 'rgba(30, 64, 175, 0.92)';
      ctx.beginPath();
      ctx.roundRect ? ctx.roundRect(-55, -52, 135, 104, 18) : ctx.rect(-55, -52, 135, 104);
      ctx.fill();
      ctx.fillStyle = '#0f172a';
      ctx.fillRect(-25, -72, 50, 16); // roda kiri
      ctx.fillRect(-25, 56, 50, 16); // roda kanan
      const { sensorSpacing: s, sensorLookahead: L } = this.params;
      ctx.fillStyle = '#334155';
      ctx.fillRect(L - 6, -2.6 * s, 12, 5.2 * s);
      for (let k = 0; k < 5; k++) {
        const r = this.readings[k];
        ctx.fillStyle = `rgb(${Math.round(60 + 195 * r)}, ${Math.round(60 * (1 - r))}, ${Math.round(60 * (1 - r))})`;
        ctx.beginPath();
        ctx.arc(L, -SENSOR_POS[k] * s, 5.5, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.fillStyle = '#facc15'; // penanda arah depan
      ctx.beginPath(); ctx.moveTo(30, -10); ctx.lineTo(48, 0); ctx.lineTo(30, 10); ctx.fill();
      ctx.restore();
      ctx.restore();

      // HUD bar sensor
      const bx = 12, by = H - 78, bw = 150, bh = 66;
      ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
      ctx.fillRect(bx, by, bw, bh);
      ctx.fillStyle = '#94a3b8';
      ctx.font = '11px ui-monospace, monospace';
      ctx.textAlign = 'left';
      ctx.fillText(`IR array  e=${this.lineError.toFixed(1)}mm`, bx + 8, by + 14);
      for (let k = 0; k < 5; k++) {
        const h = this.readings[k] * 36;
        ctx.fillStyle = this.lost ? '#ef4444' : '#22d3ee';
        ctx.fillRect(bx + 12 + k * 27, by + 58 - h, 18, h);
      }
    }
  }

  LineFollowerSim.meta = {
    id: 'line',
    title: 'Robot Line Follower',
    description:
      'Robot differential-drive dengan 5 sensor IR analog. PID mengatur selisih kecepatan roda kiri-kanan agar garis tetap di tengah array sensor. Coba naikkan kecepatan dasar atau ganti ke lintasan Clover.',
    measurementLabel: 'Error posisi garis (mm)',
    outputLabel: 'Koreksi roda Δv (mm/s)',
    defaultGains: { kp: 9, ki: 0, kd: 0.6 },
    gainRanges: { kp: [0, 40, 0.1], ki: [0, 40, 0.1], kd: [0, 3, 0.01] },
    controlHz: 200,
    presets: [
      { name: '✅ Tuned PD (default)', desc: 'PD klasik line follower — stabil dan mulus.', gains: { kp: 9, ki: 0, kd: 0.6 } },
      { name: 'P saja (osilasi)', desc: 'Tanpa redaman D → robot zig-zag di lintasan.', gains: { kp: 30, ki: 0, kd: 0 } },
      { name: 'Kp terlalu kecil', desc: 'Respons lambat → robot keluar lintasan di tikungan.', gains: { kp: 1.5, ki: 0, kd: 0.1 } },
      { name: 'Ki berlebih', desc: 'Integral menumpuk di tikungan panjang → overshoot.', gains: { kp: 9, ki: 25, kd: 0.6 } },
      { name: 'Kd berlebih', desc: 'Robot gemetar karena noise sensor diperkuat.', gains: { kp: 9, ki: 0, kd: 2.5 } },
    ],
  };

  return { LineFollowerSim, LINE_TRACKS: TRACKS };
});

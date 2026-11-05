/**
 * chart.js — plot real-time ringan berbasis Canvas 2D (tanpa library eksternal).
 * Sumbu kiri: setpoint / measurement / suku P-I-D.  Sumbu kanan: output kontrol.
 */
(function (root) {
  'use strict';

  class LiveChart {
    constructor(canvas, { windowSec = 10 } = {}) {
      this.canvas = canvas;
      this.ctx = canvas.getContext('2d');
      this.windowSec = windowSec;
      this.series = [];
      this.data = [];
    }

    /** defs: [{ key, label, color, axis: 'left'|'right', visible, dash }] */
    setSeries(defs) {
      this.series = defs;
    }

    setLabels(left, right) {
      this.leftLabel = left;
      this.rightLabel = right;
    }

    clear() {
      this.data = [];
    }

    push(sample) {
      this.data.push(sample);
      const tMin = sample.t - this.windowSec * 3;
      if (this.data.length > 20000 || (this.data.length && this.data[0].t < tMin)) {
        let k = 0;
        while (k < this.data.length && this.data[k].t < tMin) k++;
        this.data.splice(0, Math.max(k, this.data.length - 20000));
      }
    }

    _resize() {
      const dpr = window.devicePixelRatio || 1;
      const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
      if (this.canvas.width !== Math.round(w * dpr) || this.canvas.height !== Math.round(h * dpr)) {
        this.canvas.width = Math.round(w * dpr);
        this.canvas.height = Math.round(h * dpr);
      }
      this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      return { w, h };
    }

    static _range(values) {
      let lo = Infinity, hi = -Infinity;
      for (const v of values) {
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      }
      if (!isFinite(lo)) return [-1, 1];
      if (hi - lo < 1e-9) { lo -= 1; hi += 1; }
      const pad = (hi - lo) * 0.1;
      return [lo - pad, hi + pad];
    }

    static _niceStep(range, ticks) {
      const raw = range / ticks;
      const mag = Math.pow(10, Math.floor(Math.log10(raw)));
      const n = raw / mag;
      return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * mag;
    }

    static _fmt(v) {
      const a = Math.abs(v);
      if (a === 0) return '0';
      if (a >= 1000) return v.toFixed(0);
      if (a >= 10) return v.toFixed(0);
      if (a >= 1) return v.toFixed(1);
      if (a >= 0.01) return v.toFixed(2);
      return v.toExponential(0);
    }

    draw() {
      const { w, h } = this._resize();
      const ctx = this.ctx;
      const m = { l: 56, r: 56, t: 26, b: 26 };
      const pw = w - m.l - m.r, ph = h - m.t - m.b;
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = '#0b1220';
      ctx.fillRect(m.l, m.t, pw, ph);

      const tNow = this.data.length ? this.data[this.data.length - 1].t : 0;
      const t0 = Math.max(0, tNow - this.windowSec), t1 = Math.max(this.windowSec, tNow);
      let i0 = this.data.length - 1;
      while (i0 > 0 && this.data[i0 - 1].t >= t0) i0--;
      const view = this.data.slice(Math.max(0, i0));

      const vis = this.series.filter((s) => s.visible);
      const axes = {};
      for (const ax of ['left', 'right']) {
        const vals = [];
        for (const s of vis) if (s.axis === ax) for (const d of view) vals.push(d[s.key]);
        axes[ax] = vals.length ? LiveChart._range(vals) : null;
      }

      const X = (t) => m.l + ((t - t0) / (t1 - t0)) * pw;
      const Y = (v, ax) => m.t + ph - ((v - axes[ax][0]) / (axes[ax][1] - axes[ax][0])) * ph;

      // Grid + tick
      ctx.font = '11px ui-monospace, monospace';
      ctx.lineWidth = 1;
      if (axes.left) {
        const [lo, hi] = axes.left;
        const st = LiveChart._niceStep(hi - lo, 5);
        ctx.textAlign = 'right';
        ctx.textBaseline = 'middle';
        for (let v = Math.ceil(lo / st) * st; v <= hi; v += st) {
          const y = Y(v, 'left');
          ctx.strokeStyle = Math.abs(v) < st * 1e-6 ? '#334155' : '#1e293b';
          ctx.beginPath(); ctx.moveTo(m.l, y); ctx.lineTo(m.l + pw, y); ctx.stroke();
          ctx.fillStyle = '#94a3b8';
          ctx.fillText(LiveChart._fmt(v), m.l - 6, y);
        }
      }
      if (axes.right) {
        const [lo, hi] = axes.right;
        const st = LiveChart._niceStep(hi - lo, 5);
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        ctx.fillStyle = '#c4b5fd';
        for (let v = Math.ceil(lo / st) * st; v <= hi; v += st) ctx.fillText(LiveChart._fmt(v), m.l + pw + 6, Y(v, 'right'));
      }
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      ctx.fillStyle = '#64748b';
      const tst = LiveChart._niceStep(t1 - t0, 6);
      for (let t = Math.ceil(t0 / tst) * tst; t <= t1; t += tst) {
        const x = X(t);
        ctx.strokeStyle = '#1e293b';
        ctx.beginPath(); ctx.moveTo(x, m.t); ctx.lineTo(x, m.t + ph); ctx.stroke();
        ctx.fillText(`${t.toFixed(t1 - t0 < 3 ? 1 : 0)}s`, x, m.t + ph + 6);
      }

      // Data
      ctx.save();
      ctx.beginPath();
      ctx.rect(m.l, m.t, pw, ph);
      ctx.clip();
      const step = Math.max(1, Math.floor(view.length / (pw * 1.5)));
      for (const s of vis) {
        if (!axes[s.axis] || !view.length) continue;
        ctx.strokeStyle = s.color;
        ctx.lineWidth = s.width || 1.6;
        ctx.setLineDash(s.dash || []);
        ctx.beginPath();
        for (let k = 0; k < view.length; k += step) {
          const d = view[k];
          const x = X(d.t), y = Y(d[s.key], s.axis);
          k === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
        }
        ctx.stroke();
      }
      ctx.setLineDash([]);
      ctx.restore();

      // Label sumbu + legenda
      ctx.textBaseline = 'alphabetic';
      ctx.textAlign = 'left';
      ctx.fillStyle = '#94a3b8';
      if (this.leftLabel) ctx.fillText(this.leftLabel, m.l, m.t - 9);
      ctx.textAlign = 'right';
      ctx.fillStyle = '#c4b5fd';
      if (this.rightLabel && axes.right) ctx.fillText(`${this.rightLabel} →`, m.l + pw, m.t - 9);
    }

    toCSV() {
      const keys = ['t', ...this.series.map((s) => s.key)];
      const rows = [keys.join(',')];
      for (const d of this.data) rows.push(keys.map((k) => (d[k] ?? '').toString()).join(','));
      return rows.join('\n');
    }
  }

  root.PIDSim = Object.assign(root.PIDSim || {}, { LiveChart });
})(typeof self !== 'undefined' ? self : this);

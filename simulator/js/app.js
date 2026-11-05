/**
 * app.js — menghubungkan simulasi, kontroler PID, grafik, dan UI.
 */
(function () {
  'use strict';
  const P = window.PIDSim;
  const MODES = [P.DCMotorSim, P.LineFollowerSim, P.CartPoleSim];
  const $ = (id) => document.getElementById(id);
  const el = (tag, cls, html) => {
    const e = document.createElement(tag);
    if (cls) e.className = cls;
    if (html !== undefined) e.innerHTML = html;
    return e;
  };

  const SERIES = [
    { key: 'setpoint', label: 'CH1 Ref', color: '#f5c518', axis: 'left', visible: true, dash: [5, 3] },
    { key: 'measurement', label: 'CH2 Meas', color: '#3fb950', axis: 'left', visible: true },
    { key: 'output', label: 'CH3 Out u(t)', color: '#f0883e', axis: 'right', visible: true, width: 1.3 },
    { key: 'p', label: 'P', color: '#e5534b', axis: 'right', visible: false, width: 1 },
    { key: 'i', label: 'I', color: '#58a6ff', axis: 'right', visible: false, width: 1 },
    { key: 'd', label: 'D', color: '#d2a8ff', axis: 'right', visible: false, width: 1 },
  ];

  const state = {
    Cls: null, sim: null, runner: null,
    running: true, speed: 1,
    lastFrame: null, lastSimT: 0, lastMetrics: 0,
    defaultTau: 0,
  };

  const chart = new P.LiveChart($('chartCanvas'), { windowSec: 10 });
  chart.setSeries(SERIES);
  const gainInputs = {};

  // ---------------------------------------------------------------- helpers
  function fitCanvas(canvas) {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { ctx, w, h };
  }

  function toast(msg) {
    const t = $('toast');
    t.textContent = msg;
    t.classList.add('show');
    clearTimeout(toast._h);
    toast._h = setTimeout(() => t.classList.remove('show'), 1800);
  }

  const decimals = (step) => Math.max(0, -Math.floor(Math.log10(step)));

  // ---------------------------------------------------------------- mode
  function buildTabs() {
    const nav = $('modeTabs');
    MODES.forEach((Cls, i) => {
      const b = el('button', 'tab', `${i + 1}. ${Cls.meta.title.split(' — ')[0]}`);
      b.dataset.id = Cls.meta.id;
      b.onclick = () => setMode(Cls);
      nav.appendChild(b);
    });
  }

  function setMode(Cls) {
    state.Cls = Cls;
    state.sim = new Cls();
    state.defaultTau = state.sim.pid.dFilterTau;
    state.runner = new P.SimRunner(state.sim, {
      controlHz: Cls.meta.controlHz,
      onSample: (s) => chart.push(s),
    });
    state.lastSimT = 0;
    chart.clear();
    chart.setLabels(Cls.meta.measurementLabel, Cls.meta.outputLabel);

    document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('active', t.dataset.id === Cls.meta.id));
    $('modeTitle').textContent = Cls.meta.title;
    $('modeDesc').textContent = Cls.meta.description;
    $('rateSelect').value = String(Cls.meta.controlHz);
    history.replaceState(null, '', `#${Cls.meta.id}`);

    buildGainRows();
    buildPresets();
    buildPlantControls();
    syncAdvanced();
    renderMetrics();
  }

  // ---------------------------------------------------------------- gains
  function buildGainRows() {
    const box = $('gainRows');
    box.innerHTML = '';
    const { gainRanges } = state.Cls.meta;
    for (const k of ['kp', 'ki', 'kd']) {
      const [min, max, step] = gainRanges[k];
      const row = el('div', 'gain-row');
      const head = el('div', 'gain-head');
      head.appendChild(el('span', `gain-name ${k}`, `K<sub>${k[1]}</sub>`));
      const num = el('input');
      Object.assign(num, { type: 'number', min, step, value: state.sim.pid[k] });
      head.appendChild(num);
      const range = el('input');
      Object.assign(range, { type: 'range', min, max, step, value: state.sim.pid[k] });
      row.append(head, range);
      box.appendChild(row);

      const apply = (v, src) => {
        if (!isFinite(v) || v < 0) return;
        state.sim.pid[k] = v;
        if (src !== range) range.value = String(v);
        if (src !== num) num.value = v.toFixed(decimals(step));
        $('presetSelect').value = '';
        $('presetDesc').textContent = 'Custom tuning.';
      };
      range.addEventListener('input', () => apply(parseFloat(range.value), range));
      num.addEventListener('change', () => apply(parseFloat(num.value), num));
      gainInputs[k] = { range, num, step };
    }
  }

  function refreshGainInputs() {
    for (const k of ['kp', 'ki', 'kd']) {
      const { range, num, step } = gainInputs[k];
      range.value = String(state.sim.pid[k]);
      num.value = state.sim.pid[k].toFixed(decimals(step));
    }
  }

  function buildPresets() {
    const sel = $('presetSelect');
    sel.innerHTML = '<option value="" disabled>— custom —</option>';
    state.Cls.meta.presets.forEach((p, i) => sel.appendChild(new Option(p.name, String(i))));
    sel.value = '0';
    $('presetDesc').textContent = state.Cls.meta.presets[0].desc;
  }

  function applyPreset(idx) {
    const p = state.Cls.meta.presets[idx];
    if (!p) return;
    const pid = state.sim.pid;
    pid.setTunings(p.gains.kp, p.gains.ki, p.gains.kd);
    pid.antiWindup = p.antiWindup ?? true;
    pid.dFilterTau = p.dFilterTau ?? state.defaultTau;
    $('presetDesc').textContent = p.desc;
    refreshGainInputs();
    syncAdvanced();
    resetSim();
  }

  function syncAdvanced() {
    const pid = state.sim.pid;
    $('antiWindupChk').checked = pid.antiWindup;
    $('dOnMeasChk').checked = pid.derivativeOnMeasurement;
    $('dTauInput').value = pid.dFilterTau;
  }

  // ---------------------------------------------------------------- plant controls
  function buildPlantControls() {
    const box = $('plantControls');
    box.innerHTML = '';
    for (const c of state.sim.getControls()) {
      const row = el('div', 'ctrl');
      if (c.type === 'range') {
        const head = el('div', 'ctrl-head');
        const val = el('b');
        const fmt = (v) => `${(+v).toFixed(decimals(c.step))} ${c.unit || ''}`;
        val.textContent = fmt(c.get());
        head.append(el('span', '', c.label), val);
        const r = el('input');
        Object.assign(r, { type: 'range', min: c.min, max: c.max, step: c.step, value: c.get() });
        r.addEventListener('input', () => { c.set(parseFloat(r.value)); val.textContent = fmt(r.value); });
        row.append(head, r);
      } else if (c.type === 'select') {
        const lab = el('label', '', c.label);
        const s = el('select');
        c.options.forEach((o) => s.appendChild(new Option(o.label, o.value)));
        s.value = c.get();
        s.addEventListener('change', () => c.set(s.value));
        lab.appendChild(s);
        row.appendChild(lab);
      } else if (c.type === 'checkbox') {
        const lab = el('label', 'check');
        const cb = el('input');
        cb.type = 'checkbox';
        cb.checked = c.get();
        cb.addEventListener('change', () => c.set(cb.checked));
        lab.append(cb, document.createTextNode(c.label));
        row.appendChild(lab);
      } else if (c.type === 'buttons') {
        row.appendChild(el('div', 'ctrl-head', `<span>${c.label}</span>`));
        const btns = el('div', 'btns');
        c.buttons.forEach((b) => {
          const btn = el('button', 'btn', b.label);
          // Tombol bisa mengubah nilai slider lain (mis. step cepat setpoint) → bangun ulang panel
          btn.onclick = () => { b.onClick(); buildPlantControls(); };
          btns.appendChild(btn);
        });
        row.appendChild(btns);
      }
      box.appendChild(row);
    }
  }

  // ---------------------------------------------------------------- metrics
  function renderMetrics() {
    const box = $('metrics');
    const items = state.sim.getMetrics();
    if (box.children.length !== items.length) {
      box.innerHTML = '';
      items.forEach(() => box.appendChild(el('div', 'metric', '<div class="k"></div><div class="v"></div>')));
    }
    items.forEach((m, i) => {
      const c = box.children[i];
      c.classList.toggle('warn', !!m.warn);
      c.children[0].textContent = m.label;
      c.children[1].textContent = m.value;
    });
  }

  // ---------------------------------------------------------------- legend
  function buildLegend() {
    const box = $('legend');
    SERIES.forEach((s) => {
      const chip = el('span', `chip${s.visible ? ' on' : ''}`, `<span class="dot" style="background:${s.color}"></span>${s.label}`);
      chip.onclick = () => {
        s.visible = !s.visible;
        chip.classList.toggle('on', s.visible);
      };
      box.appendChild(chip);
    });
  }

  // ---------------------------------------------------------------- actions
  function resetSim() {
    state.sim.reset();
    state.runner.reset();
    state.lastSimT = 0;
    chart.clear();
    renderMetrics();
  }

  function setRunning(r) {
    state.running = r;
    $('playBtn').textContent = r ? 'PAUSE' : 'RUN';
    const b = $('statusBadge');
    b.textContent = r ? 'RUN' : 'HOLD';
    b.classList.toggle('paused', !r);
  }

  function copyCpp() {
    const pid = state.sim.pid;
    const lim = (v) => (isFinite(v) ? v : (v > 0 ? 255 : -255));
    const code = [
      `// Hasil tuning dari PID Robot Simulator (mode: ${state.Cls.meta.title})`,
      `#include <PIDController.h>`,
      ``,
      `PIDController pid(${pid.kp}f, ${pid.ki}f, ${pid.kd}f, ${lim(pid.outMin)}f, ${lim(pid.outMax)}f);`,
      ``,
      `void setup() {`,
      `  pid.setDerivativeFilter(${pid.dFilterTau}f);`,
      `  pid.setAntiWindup(${pid.antiWindup});`,
      `  pid.setDerivativeOnMeasurement(${pid.derivativeOnMeasurement});`,
      `}`,
      `// Panggil tiap ${(1000 / state.runner.controlHz).toFixed(1)} ms (${state.runner.controlHz} Hz):`,
      `// float u = pid.compute(setpoint, measurement, ${(1 / state.runner.controlHz).toFixed(4)}f);`,
    ].join('\n');
    const done = () => toast('Kode C++ disalin ke clipboard ✓');
    if (navigator.clipboard && window.isSecureContext) navigator.clipboard.writeText(code).then(done, () => fallbackCopy(code));
    else fallbackCopy(code);
    function fallbackCopy(text) {
      const ta = el('textarea');
      ta.value = text;
      document.body.appendChild(ta);
      ta.select();
      document.execCommand('copy');
      ta.remove();
      done();
    }
  }

  function exportCSV() {
    const blob = new Blob([chart.toCSV()], { type: 'text/csv' });
    const a = el('a');
    a.href = URL.createObjectURL(blob);
    a.download = `pid_${state.Cls.meta.id}_${Date.now()}.csv`;
    a.click();
    URL.revokeObjectURL(a.href);
  }

  function bindGlobalControls() {
    $('playBtn').onclick = () => setRunning(!state.running);
    $('resetBtn').onclick = resetSim;
    $('speedSelect').onchange = (e) => (state.speed = parseFloat(e.target.value));
    $('rateSelect').onchange = (e) => state.runner.setControlRate(parseFloat(e.target.value));
    $('windowSelect').onchange = (e) => (chart.windowSec = parseFloat(e.target.value));
    $('presetSelect').onchange = (e) => applyPreset(parseInt(e.target.value, 10));
    $('antiWindupChk').onchange = (e) => (state.sim.pid.antiWindup = e.target.checked);
    $('dOnMeasChk').onchange = (e) => (state.sim.pid.derivativeOnMeasurement = e.target.checked);
    $('dTauInput').onchange = (e) => {
      const v = parseFloat(e.target.value);
      if (v >= 0) state.sim.pid.dFilterTau = v;
    };
    $('copyCppBtn').onclick = copyCpp;
    $('csvBtn').onclick = exportCSV;

    window.addEventListener('keydown', (e) => {
      if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return;
      if (e.code === 'Space') { e.preventDefault(); setRunning(!state.running); }
      else if (e.key === 'r' || e.key === 'R') resetSim();
      else if (['1', '2', '3'].includes(e.key)) setMode(MODES[parseInt(e.key, 10) - 1]);
    });
  }

  // ---------------------------------------------------------------- main loop
  function frame(ts) {
    if (state.lastFrame === null) state.lastFrame = ts;
    const dt = Math.min(0.05, (ts - state.lastFrame) / 1000);
    state.lastFrame = ts;

    if (state.running) state.runner.advance(dt * state.speed);

    // Sim di-reset dari dalam (mis. ganti lintasan) → sinkronkan runner & grafik
    if (state.sim.t < state.lastSimT - 1e-6) {
      state.runner.reset();
      chart.clear();
    }
    state.lastSimT = state.sim.t;

    const { ctx, w, h } = fitCanvas($('simCanvas'));
    state.sim.draw(ctx, w, h);
    chart.draw();

    if (ts - state.lastMetrics > 150) {
      renderMetrics();
      state.lastMetrics = ts;
    }
    requestAnimationFrame(frame);
  }

  // ---------------------------------------------------------------- init
  buildTabs();
  buildLegend();
  bindGlobalControls();
  const fromHash = MODES.find((M) => `#${M.meta.id}` === location.hash);
  setMode(fromHash || MODES[0]);
  requestAnimationFrame(frame);
})();

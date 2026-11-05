/**
 * Unit & integration test — jalankan dengan:  node tests/run-tests.js
 * Tanpa dependency eksternal (hanya modul `assert` bawaan Node.js).
 */
'use strict';
const assert = require('assert');
const path = require('path');

const js = (p) => require(path.join(__dirname, '..', 'simulator', 'js', p));
const { mulberry32 } = js('core/utils.js');
const { PIDController } = js('core/pid.js');
const { StepAnalyzer } = js('core/stepAnalyzer.js');
const { SimRunner } = js('core/runner.js');
const { DCMotorSim } = js('sims/dcMotor.js');
const { LineFollowerSim } = js('sims/lineFollower.js');
const { CartPoleSim } = js('sims/cartPole.js');

let passed = 0, failed = 0;
function test(name, fn) {
  try {
    fn();
    passed++;
    console.log(`  \u2714 ${name}`);
  } catch (e) {
    failed++;
    console.log(`  \u2718 ${name}\n      ${e.message}`);
  }
}
const close = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b} (tol ${tol})`);

// ------------------------------------------------------------------ PID core
console.log('\nPIDController');
test('P-only: output = Kp * error', () => {
  const pid = new PIDController({ kp: 2, ki: 0, kd: 0 });
  close(pid.compute(10, 4, 0.01), 12, 1e-12, 'output');
});

test('I-term mengintegrasikan error konstan', () => {
  const pid = new PIDController({ kp: 0, ki: 3, kd: 0 });
  for (let k = 0; k < 100; k++) pid.compute(1, 0, 0.01); // ∫1 dt = 1 s
  close(pid.output, 3, 1e-9, 'Ki * 1 s');
});

test('D-term: derivative on measurement (tanpa kick saat setpoint lompat)', () => {
  const pid = new PIDController({ kp: 0, ki: 0, kd: 1 });
  pid.compute(0, 0, 0.01);
  const u = pid.compute(100, 0, 0.01); // setpoint lompat, measurement diam
  close(u, 0, 1e-12, 'tidak ada derivative kick');
  const u2 = pid.compute(100, 1, 0.01); // measurement naik 1 dalam 0.01 s
  close(u2, -100, 1e-9, 'D = -Kd * dy/dt');
});

test('D-term on error menghasilkan kick (pembanding)', () => {
  const pid = new PIDController({ kp: 0, ki: 0, kd: 1, derivativeOnMeasurement: false });
  pid.compute(0, 0, 0.01);
  close(pid.compute(1, 0, 0.01), 100, 1e-9, 'kick = Kd * Δe/dt');
});

test('Output di-clamp ke batas', () => {
  const pid = new PIDController({ kp: 100, outMin: -5, outMax: 5 });
  assert.strictEqual(pid.compute(10, 0, 0.01), 5);
  assert.strictEqual(pid.compute(-10, 0, 0.01), -5);
  assert.ok(pid.saturated);
});

test('Anti-windup: integrator tidak menumpuk saat saturasi', () => {
  const withAW = new PIDController({ kp: 1, ki: 10, outMin: -1, outMax: 1, antiWindup: true });
  const noAW = new PIDController({ kp: 1, ki: 10, outMin: -1, outMax: 1, antiWindup: false });
  for (let k = 0; k < 500; k++) { withAW.compute(10, 0, 0.01); noAW.compute(10, 0, 0.01); }
  assert.ok(withAW.iTerm <= 1 + 1e-9, `iTerm AW = ${withAW.iTerm}`);
  assert.ok(noAW.iTerm > 400, `iTerm tanpa AW = ${noAW.iTerm}`);
  // Setelah error berbalik, kontroler dengan AW langsung keluar dari saturasi
  const uAW = withAW.compute(-1, 0, 0.01);
  const uNo = noAW.compute(-1, 0, 0.01);
  assert.ok(uAW < 1, 'AW cepat lepas dari saturasi');
  assert.strictEqual(uNo, 1, 'tanpa AW masih tersaturasi (windup)');
});

test('Low-pass filter D meredam noise', () => {
  const rng = mulberry32(7);
  const raw = new PIDController({ kp: 0, kd: 1, dFilterTau: 0 });
  const filt = new PIDController({ kp: 0, kd: 1, dFilterTau: 0.05 });
  let sRaw = 0, sFilt = 0;
  for (let k = 0; k < 1000; k++) {
    const y = rng() - 0.5;
    sRaw += raw.compute(0, y, 0.01) ** 2;
    sFilt += filt.compute(0, y, 0.01) ** 2;
  }
  assert.ok(sFilt < sRaw * 0.2, `energi noise filtered ${sFilt.toFixed(1)} vs raw ${sRaw.toFixed(1)}`);
});

test('dt <= 0 diabaikan dengan aman', () => {
  const pid = new PIDController({ kp: 1 });
  const u = pid.compute(1, 0, 0.01);
  assert.strictEqual(pid.compute(5, 0, 0), u);
  assert.strictEqual(pid.compute(5, 0, NaN), u);
});

// ------------------------------------------------------------------ StepAnalyzer
console.log('\nStepAnalyzer');
test('Respons orde-1 ideal: rise time ≈ 2.197τ, tanpa overshoot', () => {
  const a = new StepAnalyzer();
  const tau = 0.5;
  a.start(0, 0, 1);
  for (let t = 0; t <= 6; t += 0.001) a.add(t, 1 - Math.exp(-t / tau));
  const m = a.metrics();
  close(m.riseTime, 2.197 * tau, 0.01, 'rise time');
  close(m.overshoot, 0, 1e-9, 'overshoot');
  close(m.settlingTime, 3.912 * tau, 0.01, 'settling 2%');
});

// ------------------------------------------------------------------ Plants
const run = (sim, seconds, hz) => new SimRunner(sim, { controlHz: hz }).advance(seconds);

console.log('\nDCMotorSim');
test('Gain default: settle < 1 s, overshoot < 5%, |SSE| < 5 RPM', () => {
  const sim = new DCMotorSim({ rng: mulberry32(1) });
  run(sim, 3, 200);
  const m = sim.analyzer.metrics();
  assert.ok(m.settled && m.settlingTime < 1, `settling ${m.settlingTime}`);
  assert.ok(m.overshoot < 5, `overshoot ${m.overshoot}`);
  assert.ok(Math.abs(m.steadyStateError) < 5, `sse ${m.steadyStateError}`);
});

test('P-only memiliki steady-state error, PI tidak', () => {
  const p = new DCMotorSim({ rng: mulberry32(1) });
  p.pid.setTunings(0.03, 0, 0);
  run(p, 3, 200);
  assert.ok(p.analyzer.metrics().steadyStateError > 50);
});

test('Integral menolak gangguan torsi beban', () => {
  const sim = new DCMotorSim({ rng: mulberry32(1) });
  const r = new SimRunner(sim, { controlHz: 200 });
  r.advance(2);
  sim.load = 0.08;
  r.advance(3);
  close(sim.rpm, 1000, 10, 'rpm setelah beban');
});

test('Windup: tanpa anti-windup overshoot jauh lebih besar', () => {
  const os = (aw) => {
    const s = new DCMotorSim({ rng: mulberry32(1) });
    s.pid.setTunings(0.01, 0.4, 0);
    s.pid.antiWindup = aw;
    run(s, 3, 200);
    return s.analyzer.metrics().overshoot;
  };
  assert.ok(os(false) > os(true) + 10, `noAW ${os(false)} vs AW ${os(true)}`);
});

console.log('\nLineFollowerSim');
for (const track of ['oval', 'blob', 'clover']) {
  test(`Gain default menyelesaikan lap di lintasan "${track}" tanpa keluar jalur`, () => {
    const sim = new LineFollowerSim({ rng: mulberry32(2), track });
    run(sim, 25, 200);
    assert.ok(sim.laps >= 2, `laps ${sim.laps}`);
    assert.strictEqual(sim.offTracks, 0);
    assert.ok(sim.maxErr < 40, `max error ${sim.maxErr}`);
  });
}

test('Kp terlalu kecil → keluar lintasan di tikungan', () => {
  const sim = new LineFollowerSim({ rng: mulberry32(2), track: 'clover' });
  sim.pid.setTunings(1.5, 0, 0.1);
  run(sim, 20, 200);
  assert.ok(sim.offTracks > 0);
});

console.log('\nCartPoleSim');
test('Gain default menyeimbangkan pendulum & kembali ke target posisi', () => {
  const sim = new CartPoleSim({ rng: mulberry32(3) });
  const r = new SimRunner(sim, { controlHz: 200 });
  r.advance(10);
  close(sim.x, 0, 0.1, 'x @ target 0');
  sim.xTarget = 1;
  r.advance(10);
  close(sim.x, 1, 0.1, 'x @ target 1');
  assert.strictEqual(sim.falls, 0);
});

test('Tahan dorongan 25 N', () => {
  const sim = new CartPoleSim({ rng: mulberry32(3) });
  const r = new SimRunner(sim, { controlHz: 200 });
  r.advance(3);
  sim.push(25);
  r.advance(5);
  assert.strictEqual(sim.falls, 0);
  assert.ok(Math.abs(sim.th) < 0.02);
});

test('Tanpa suku D (P-only) → jatuh', () => {
  const sim = new CartPoleSim({ rng: mulberry32(3) });
  sim.pid.setTunings(2, 0, 0);
  run(sim, 10, 200);
  assert.ok(sim.falls > 0);
});

test('Laju kontrol terlalu rendah (20 Hz) menurunkan performa', () => {
  const avg = (hz) => {
    const s = new CartPoleSim({ rng: mulberry32(3), noiseStd: 0 });
    const r = new SimRunner(s, { controlHz: hz });
    let acc = 0;
    for (let k = 0; k < 500; k++) { r.advance(0.01); acc += Math.abs(s.th); }
    return { acc, falls: s.falls };
  };
  const fast = avg(200), slow = avg(20);
  assert.ok(slow.falls > 0 || slow.acc > fast.acc * 1.5, `20Hz ${JSON.stringify(slow)} vs 200Hz ${JSON.stringify(fast)}`);
});

console.log(`\n${passed} passed, ${failed} failed\n`);
process.exit(failed ? 1 : 0);

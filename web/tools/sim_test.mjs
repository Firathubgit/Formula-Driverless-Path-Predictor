// Headless checks of the browser kart.  Run: node web/tools/sim_test.mjs
//
//  1. geometry: the track and the poster line as ported
//  2. a quasi-steady-state lap on the poster line (the method tools/kart/kart_model.py uses)
//  3. a driven lap: a Stanley controller follows the poster line at a speed the tires allow
//  4. straight-line: acceleration, braking, steady-state cornering on a wall-free track
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Track } from '../src/track.js';
import { KartSim } from '../src/physics.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const data = JSON.parse(fs.readFileSync(path.join(here, '..', 'data', 'track.json'), 'utf8'));
const track = new Track(data);

const k = data.kart;
const G = 9.80665;
const DT = 1 / 200;
const f2 = (x) => (Number.isFinite(x) ? x.toFixed(2) : String(x));
const makeSim = () => new KartSim(k, track);
const muLat = k.grip_lateral, muLong = k.grip_longitudinal;

// ---- the poster line as a reference: stations, offset from the centreline, tangent, curvature
const poster = data.posterLine.map((p) => ({ x: p[0], y: p[1], s: 0, off: 0, tx: 1, ty: 0, kappa: 0 }));
{
  let hint = 0;
  for (const p of poster) {
    const pr = track.project(p.x, p.y, hint);
    p.s = pr.s; p.off = pr.offset; hint = pr.s;
  }
}
{
  const n = poster.length;
  for (let i = 0; i < n; i++) {
    const a = poster[(i - 1 + n) % n], b = poster[i], c = poster[(i + 1) % n];
    const dx = c.x - a.x, dy = c.y - a.y, d = Math.hypot(dx, dy) || 1;
    b.tx = dx / d; b.ty = dy / d;
    const v1x = b.x - a.x, v1y = b.y - a.y, v2x = c.x - b.x, v2y = c.y - b.y;
    const d1 = Math.hypot(v1x, v1y) || 1, d2 = Math.hypot(v2x, v2y) || 1;
    b.kappa = ((v1x / d1) * (v2y / d2) - (v1y / d1) * (v2x / d2)) / ((d1 + d2) / 2);
  }
}
const pIdx = (s) => {
  const n = poster.length;
  s = ((s % track.length) + track.length) % track.length;
  let lo = 0, hi = n - 1;
  while (lo < hi) { const m = (lo + hi + 1) >> 1; if (poster[m].s <= s) lo = m; else hi = m - 1; }
  return lo;
};
const posterAt = (s) => poster[pIdx(s)];
const posterKappa = (s) => poster[pIdx(s)].kappa;

console.log('== geometry ==');
console.log('track length       : ' + f2(track.length) + ' m, corridor ' + f2(track.width) + ' m wide');
console.log('poster line points : ' + poster.length + ', stations ' + (() => {
  let ok = true; for (let i = 1; i < poster.length; i++) if (poster[i].s < poster[i - 1].s) ok = false; return ok ? 'monotonic' : 'NOT monotonic';
})());
console.log('poster offsets     : ' + f2(Math.min(...poster.map((p) => p.off))) + ' .. ' + f2(Math.max(...poster.map((p) => p.off))) + ' m from the centreline');
const tightest = Math.min(...poster.map((p) => Math.abs(p.kappa) > 1e-6 ? 1 / Math.abs(p.kappa) : Infinity));
console.log('tightest radius    : ' + f2(tightest) + ' m (repo: 2.8 m)');
console.log('kart               : ' + f2(k.mass_kg) + ' kg, ' + f2(k.max_drive_power_w / 735.49875) + ' hp, '
  + f2(k.max_drive_speed_mps * 3.6) + ' km/h cap, grip ' + f2(muLat) + '/' + f2(muLong) + ' g');

// ---- quasi-steady-state lap
console.log('\n== quasi-steady-state lap on the poster line ==');
{
  const n = poster.length, ds = track.length / n;
  const kappa = poster.map((p) => Math.abs(p.kappa));
  const v = poster.map((p, i) => Math.min(Math.sqrt(muLat * G / Math.max(kappa[i], 1e-6)), k.max_drive_speed_mps));
  const resist = (s) => (0.5 * 1.225 * k.drag_area_m2 * s * s + k.rolling_resistance * k.mass_kg * G) / k.mass_kg;
  const circle = (speed, kk) => Math.sqrt(Math.max((muLat * G) ** 2 - (speed * speed * kk) ** 2, 0));
  for (let it = 0; it < 4; it++) {
    for (let pass = 0; pass < 2; pass++) for (let i = n - 1; i >= -n; i--) {
      const j = ((i % n) + n) % n, nx = (j + 1) % n;
      const dec = Math.min(circle(v[nx], kappa[nx]), muLong * G) + resist(v[nx]);
      v[j] = Math.min(v[j], Math.sqrt(v[nx] * v[nx] + 2 * dec * ds));
    }
    for (let pass = 0; pass < 2; pass++) for (let i = 0; i < 2 * n; i++) {
      const j = ((i % n) + n) % n, nx = (j + 1) % n;
      const fade = Math.max(0, Math.min(1, (k.max_drive_speed_mps - v[j]) / (0.05 * k.max_drive_speed_mps)));
      const drive = Math.min(2 * k.max_drive_torque_nm / k.wheel_radius_m, k.max_drive_power_w / Math.max(v[j], 0.5)) * fade / k.mass_kg;
      const acc = Math.min(circle(v[j], kappa[j]), drive) - resist(v[j]);
      v[nx] = Math.min(v[nx], Math.sqrt(Math.max(v[j] * v[j] + 2 * acc * ds, 0)));
    }
  }
  const lap = v.reduce((sum, vi, i) => sum + ds / Math.max(0.5 * (vi + v[(i + 1) % n]), 0.1), 0);
  console.log('lap time           : ' + f2(lap) + ' s   (repo: 34.5 s at the fitted grip; this tune has 40% more, so ~29 s)');
  console.log('top speed on line  : ' + f2(Math.max(...v) * 3.6) + ' km/h');
}

// ---- a driven lap with a Stanley controller
console.log('\n== a driven lap (Stanley controller on the poster line) ==');
{
  const sim = makeSim();
  const cap = 60 / 3.6;
  const kCross = 1.6;
  let t = 0, laps = 0, prevS = sim.s, lapStart = 0;
  const lapTimes = [];
  let maxOffset = 0, offSteps = 0, wallSteps = 0, maxAy = 0;
  for (let i = 0; i < 200 * 200; i++) {
    const s = sim.s, speed = sim.speed;
    const ref = posterAt(s);
    // heading error against the racing line's tangent
    let herr = Math.atan2(ref.ty, ref.tx) - sim.yaw;
    while (herr > Math.PI) herr -= 2 * Math.PI;
    while (herr < -Math.PI) herr += 2 * Math.PI;
    // cross-track error, left positive, against the racing line
    const cte = sim.offset - ref.off;
    const delta = herr + Math.atan2(kCross * cte, Math.max(speed, 1.0));
    const steer = Math.max(-1, Math.min(1, delta / sim.steerLimit(speed)));
    // speed the tires allow, scanned ahead
    let vWant = cap;
    for (let a = 0; a < 45; a++) {
      const kp = Math.abs(posterKappa(s + a));
      const vc = Math.sqrt(0.85 * muLat * G / Math.max(kp, 1e-4));
      vWant = Math.min(vWant, Math.sqrt(vc * vc + 2 * (0.7 * muLong * G) * a));
    }
    let throttle = 0, brake = 0;
    if (speed < vWant - 0.2) throttle = Math.min(1, (vWant - speed) * 0.8);
    else if (speed > vWant + 0.2) brake = Math.min(1, (speed - vWant) * 0.5);
    sim.step(DT, { steer, throttle, brake });
    t += DT;
    maxOffset = Math.max(maxOffset, Math.abs(sim.offset));
    if (Math.abs(sim.offset) > track.half) offSteps++;
    if (sim.lastCollision > 0.15) wallSteps++;
    maxAy = Math.max(maxAy, Math.abs(sim.vx * sim.r));
    if (prevS > track.length * 0.75 && sim.s < track.length * 0.25) { laps++; lapTimes.push(t - lapStart); lapStart = t; }
    prevS = sim.s;
    if (laps >= 3) break;
  }
  console.log('laps timed         : ' + (lapTimes.map((x) => f2(x) + ' s').join(', ') || 'none'));
  console.log('best lap           : ' + (lapTimes.length ? f2(Math.min(...lapTimes)) + ' s' : 'none'));
  console.log('max |offset|       : ' + f2(maxOffset) + ' m of ' + f2(track.half) + ' m; past the edge: ' + offSteps + '; wall-contact steps: ' + wallSteps);
  console.log('peak lateral       : ' + f2(maxAy / G) + ' g of ' + f2(muLat) + ' g available');
}

// ---- straight-line behaviour
console.log('\n== straight-line behaviour ==');
{
  const s2 = makeSim();
  s2.reset(0, 0);
  let t50 = null;
  for (let i = 0; i < 200 * 12; i++) {
    s2.step(DT, { steer: 0, throttle: 1, brake: 0 });
    if (t50 === null && s2.kmh >= 50) t50 = (i + 1) * DT;
    if (s2.s > 14) break;
  }
  console.log('0-50 km/h          : ' + (t50 ? f2(t50) + ' s' : 'not reached in the first 14 m (the straight is short)'));

  const s3 = makeSim();
  s3.reset(0, 14);
  let stopT = 0, stopD = 0;
  for (let i = 0; i < 200 * 20 && s3.speed > 0.2; i++) {
    const v0 = s3.speed; s3.step(DT, { steer: 0, throttle: 0, brake: 1 });
    stopT += DT; stopD += (v0 + s3.speed) / 2 * DT;
  }
  console.log('14 m/s to rest     : ' + f2(stopT) + ' s over ' + f2(stopD) + ' m (' + f2(14 / stopT / G) + ' g avg; repo: 0.6-0.7 g)');

  // steady-state cornering on a wall-free copy of the track
  const wide = JSON.parse(JSON.stringify(data));
  wide.track.width_m = 600;
  const wideTrack = new Track(wide);
  const s4 = new KartSim(k, wideTrack);
  s4.reset(0, 0);
  const vTarget = 8.0;
  let ay = 0, acc = { vx: 0, vy: 0, r: 0, n: 0 };
  for (let i = 0; i < 200 * 12; i++) {
    const v = s4.vx;
    const throttle = v < vTarget ? Math.min(1, (vTarget - v) * 0.6) : 0;
    const brake = v > vTarget + 0.2 ? Math.min(1, (v - vTarget) * 0.5) : 0;
    s4.step(DT, { steer: 0.55, throttle, brake });
    if (i > 200 * 8) { ay = Math.max(ay, Math.abs(s4.vx * s4.r)); acc.vx += s4.vx; acc.vy += s4.vy; acc.r += s4.r; acc.n++; }
  }
  const L = k.wheelbase_m;
  console.log('steady turn, 0.55 stick at ' + f2(vTarget * 3.6) + ' km/h: ' + f2(ay / G) + ' g, radius ' + f2(vTarget * vTarget / ay)
    + ' m (kinematic ' + f2(L / Math.tan(Math.abs(s4.steer))) + ' m), steer ' + f2(s4.steer) + ' rad');
}

import * as THREE from '../vendor/three.module.js';
import { Track } from './track.js';
import { KartSim } from './physics.js';
import { Input } from './input.js';
import { buildHall, buildRoad, buildBarriers, buildKart } from './scene.js';

const FIXED_DT = 1 / 200;
const MAX_STEPS = 8;
const ADVICE_TEXT = { accelerate: 'GAS', coast: 'LIFT', brake: 'BRAKE' };
const ADVICE_COLOUR = { accelerate: '#3fd23f', coast: '#e8c33a', brake: '#e0473a' };

// Quality levels. Point lights, shadows and the lamp glow are the things that cost real
// fragment work, so on a machine without GPU acceleration the page falls back to `lite`.
// A HiDPI drawing buffer multiplies everything, hence the pixel-ratio cap.
const QUALITY = {
  high: { pointLights: 6, shadows: true, glow: true, maxRatio: 1.5, antialias: true },
  medium: { pointLights: 2, shadows: false, glow: false, maxRatio: 1, antialias: false },
  lite: { pointLights: 0, shadows: false, glow: false, maxRatio: 1, antialias: false },
};
const QUALITY_ORDER = ['lite', 'medium', 'high'];
// A frame slower than this means the display is stuttering; drop a level and try again.
const SLOW_FRAME_MS = 45;

// Read the GPU before building anything, because `antialias` is fixed at context creation.
function probeGpu() {
  try {
    const c = document.createElement('canvas');
    const gl = c.getContext('webgl2') || c.getContext('webgl');
    if (!gl) return { name: 'none', software: true };
    const dbg = gl.getExtension('WEBGL_debug_renderer_info');
    const name = dbg ? gl.getParameter(dbg.UNMASKED_RENDERER_WEBGL) : String(gl.getParameter(gl.RENDERER));
    const software = /swiftshader|llvmpipe|software|basic render|microsoft basic/i.test(name);
    const lose = gl.getExtension('WEBGL_lose_context');
    if (lose) lose.loseContext();
    return { name, software };
  } catch {
    return { name: 'unknown', software: false };
  }
}

const el = (id) => document.getElementById(id);

async function main() {
  const data = await (await fetch('./data/track.json')).json();
  const track = new Track(data);

  const gpu = probeGpu();
  const params = new URLSearchParams(location.search);
  const forced = params.get('quality');
  let quality = (forced && QUALITY[forced]) ? forced : (gpu.software ? 'lite' : 'high');

  // Guide stations, found once with a running hint so nothing misprojects at the seam.
  let hint = 0;
  track.guidePoints = data.guide.map((g) => {
    const p = track.project(g.x, g.y, hint);
    hint = p.s;
    return { x: g.x, y: g.y, advice: g.advice, s: p.s };
  }).sort((a, b) => a.s - b.s);

  const sim = new KartSim(data.kart, track);
  const input = new Input();

  // ---- renderer
  const canvas = el('view');
  const renderer = new THREE.WebGLRenderer({
    canvas, antialias: QUALITY[quality].antialias, powerPreference: 'high-performance',
  });
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.15;
  renderer.outputColorSpace = THREE.SRGBColorSpace;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0d0f12);
  scene.fog = new THREE.Fog(0x0d0f12, 40, 110);

  scene.add(new THREE.HemisphereLight(0x9fb4d8, 0x1a1d22, 0.55));
  scene.add(new THREE.AmbientLight(0xffffff, 0.22));
  const key = new THREE.DirectionalLight(0xfff0dc, 1.1);
  key.position.set(30, 40, 20);
  key.shadow.mapSize.set(1024, 1024);
  const S = 46;
  Object.assign(key.shadow.camera, { left: -S, right: S, top: S, bottom: -S, near: 1, far: 160 });
  key.shadow.camera.updateProjectionMatrix();
  key.shadow.bias = -0.0006;
  scene.add(key);

  // Built once with the maximum number of lamps; quality only toggles what is visible, and
  // three.js leaves an invisible light out of the shader entirely.
  const hallGroup = buildHall(data, { pointLights: QUALITY.high.pointLights, glow: true });
  scene.add(hallGroup);
  const hallLights = [];
  let glowMesh = null;
  hallGroup.traverse((o) => {
    if (o.isPointLight) hallLights.push(o);
    if (o.name === 'lamp-glow') glowMesh = o;
  });
  scene.add(buildRoad(track));
  scene.add(buildBarriers(track));
  const kart = buildKart(data.kart);
  scene.add(kart.group);

  function applyQuality(level) {
    const q = QUALITY[level];
    renderer.setPixelRatio(Math.min(devicePixelRatio, q.maxRatio));
    renderer.shadowMap.enabled = q.shadows;
    key.castShadow = q.shadows;
    hallLights.forEach((l, i) => { l.visible = i < q.pointLights; });
    if (glowMesh) glowMesh.visible = q.glow;
    renderer.setSize(innerWidth, innerHeight, false);
    // a shadow-map or light-count change needs the affected programs rebuilt
    scene.traverse((o) => {
      const m = o.material;
      if (!m) return;
      (Array.isArray(m) ? m : [m]).forEach((x) => { x.needsUpdate = true; });
    });
  }
  applyQuality(quality);

  const cameras = {
    chase: new THREE.PerspectiveCamera(62, 1, 0.1, 400),
    cockpit: new THREE.PerspectiveCamera(72, 1, 0.05, 400),
    top: new THREE.PerspectiveCamera(55, 1, 0.1, 400),
  };
  let cameraMode = 'chase';
  const MODES = ['chase', 'cockpit', 'top'];

  // ---- HUD
  const hud = {
    speed: el('speed'), throttle: el('thr'), brake: el('brk'),
    lap: el('lap'), best: el('best'), count: el('count'),
    cue: el('cue'), cueNext: el('cueNext'), pad: el('pad'),
    cam: el('cam'), status: el('status'), quality: el('quality'),
  };
  el('start').addEventListener('click', () => el('overlay').classList.add('hidden'));
  el('overlay').addEventListener('click', () => el('overlay').classList.add('hidden'));
  if (gpu.software) {
    const note = el('gpuNote');
    note.textContent = 'Your browser is drawing without GPU acceleration (' + gpu.name
      + '), so the hall is running in lite mode. Turning on hardware acceleration in the browser'
      + ' settings will restore the full lighting.';
    note.style.display = 'block';
  }

  // ---- lap timing
  let lapStart = performance.now();
  let bestLap = null;
  let laps = 0;
  let prevS = sim.s;
  let started = false;

  function coachText() {
    const s = sim.s;
    // advice at the car's station
    let lo = 0, hi = track.guidePoints.length - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (track.guidePoints[mid].s <= s) lo = mid; else hi = mid - 1;
    }
    const advice = track.guidePoints[lo] ? track.guidePoints[lo].advice : 'coast';
    // next corner ahead
    let next = null, bestD = Infinity;
    track.corners.forEach((c, i) => {
      let d = c.from_s - s;
      if (d < -20) d += track.length;
      if (d >= -5 && d < bestD) { bestD = d; next = { c, i }; }
    });
    let nextText = '';
    if (next) {
      const side = next.c.direction > 0 ? 'left' : 'right';
      nextText = `T${next.i + 1} ${side} in ${Math.max(0, Math.round(bestD))} m`;
    }
    return { advice, nextText };
  }

  // ---- resize
  function resize() {
    const w = innerWidth, h = innerHeight;
    renderer.setPixelRatio(Math.min(devicePixelRatio, QUALITY[quality].maxRatio));
    renderer.setSize(w, h, false);
    for (const c of Object.values(cameras)) { c.aspect = w / h; c.updateProjectionMatrix(); }
  }
  addEventListener('resize', resize);
  resize();

  // ---- loop
  let last = performance.now();
  let acc = 0;
  let frameAcc = 0, frameCount = 0, avgMs = 0;

  function frame(now) {
    requestAnimationFrame(frame);
    const rawMs = now - last;
    const dt = Math.min(0.05, rawMs / 1000);
    last = now;

    // Adaptive quality: the first frames of a heavy page can take a second each, which is
    // exactly what makes a browser declare the page unresponsive. Sample real frame times
    // and step down a level when they stay slow.
    if (rawMs > 0 && rawMs < 2000) { frameAcc += rawMs; frameCount++; }
    if (frameCount >= 60) {
      avgMs = frameAcc / frameCount;
      const level = QUALITY_ORDER.indexOf(quality);
      if (avgMs > SLOW_FRAME_MS && level > 0) {
        quality = QUALITY_ORDER[level - 1];
        applyQuality(quality);
      }
      frameAcc = 0; frameCount = 0;
    }

    const cmd = input.poll();
    if (input.consumeCamera()) {
      cameraMode = MODES[(MODES.indexOf(cameraMode) + 1) % MODES.length];
    }
    if (input.consumeReset()) {
      sim.reset(0, 0);
      laps = 0; bestLap = null; lapStart = now; started = false; prevS = sim.s;
    }
    const paused = !el('overlay').classList.contains('hidden');

    if (!paused) {
      acc += dt;
      let steps = 0;
      while (acc >= FIXED_DT && steps < MAX_STEPS) { sim.step(FIXED_DT, cmd); acc -= FIXED_DT; steps++; }
      if (steps === MAX_STEPS) acc = 0;
    }

    // lap detection on the wrap past the start line
    if (!paused) {
      if (!started && sim.speed > 0.5) { started = true; lapStart = now; }
      if (started && prevS > track.length * 0.75 && sim.s < track.length * 0.25) {
        const t = (now - lapStart) / 1000;
        laps += 1;
        if (bestLap === null || t < bestLap) bestLap = t;
        lapStart = now;
      }
      prevS = sim.s;
    }

    // ---- place the kart
    kart.group.position.set(sim.x, 0, -sim.y);
    kart.group.rotation.y = sim.yaw;
    const spin = sim.wheelSpin;
    for (const [name, pivot] of Object.entries(kart.wheels)) {
      pivot.rotation.x = spin;
      if (name === 'fl' || name === 'fr') pivot.rotation.y = sim.steer;
    }
    kart.steeringWheel.rotation.z = -sim.steer * 3.2;

    // ---- cameras
    const fwd = new THREE.Vector3(Math.cos(sim.yaw), 0, -Math.sin(sim.yaw));
    const right = new THREE.Vector3(-fwd.z, 0, fwd.x);
    const pos = new THREE.Vector3(sim.x, 0, -sim.y);
    if (cameraMode === 'chase') {
      const cam = cameras.chase;
      const want = pos.clone().add(fwd.clone().multiplyScalar(-4.6)).add(new THREE.Vector3(0, 2.35, 0)).add(right.clone().multiplyScalar(sim.steer * 0.6));
      cam.position.lerp(want, 1 - Math.pow(0.0015, dt));
      const look = pos.clone().add(fwd.clone().multiplyScalar(6)).add(new THREE.Vector3(0, 0.9, 0));
      cam.lookAt(look);
      cam.fov = 62 + Math.min(12, sim.kmh * 0.18);
      cam.updateProjectionMatrix();
    } else if (cameraMode === 'cockpit') {
      const cam = cameras.cockpit;
      cam.position.copy(pos).add(fwd.clone().multiplyScalar(0.1)).add(new THREE.Vector3(0, 1.02, 0));
      const look = cam.position.clone().add(fwd.clone().multiplyScalar(8)).add(right.clone().multiplyScalar(sim.steer * 2.5)).add(new THREE.Vector3(0, -0.35, 0));
      cam.lookAt(look);
      cam.rotateZ(-sim.steer * 0.12);
      cam.fov = 72 + Math.min(14, sim.kmh * 0.2);
      cam.updateProjectionMatrix();
    } else {
      const cam = cameras.top;
      cam.position.copy(pos).add(new THREE.Vector3(0, 34, 0));
      cam.up.set(Math.sin(sim.yaw), 0, Math.cos(sim.yaw));
      cam.lookAt(pos);
      cam.fov = 55;
      cam.updateProjectionMatrix();
    }

    // key light follows the kart for crisp shadows
    key.position.set(sim.x + 30, 40, -sim.y + 20);
    key.target.position.set(sim.x, 0, -sim.y);
    key.target.updateMatrixWorld();

    renderer.render(scene, cameras[cameraMode]);

    // ---- HUD
    hud.speed.textContent = Math.round(sim.kmh);
    hud.throttle.style.width = `${Math.round(sim.throttle * 100)}%`;
    hud.brake.style.width = `${Math.round(sim.brake * 100)}%`;
    const cur = started ? (now - lapStart) / 1000 : 0;
    hud.lap.textContent = fmt(cur);
    hud.best.textContent = bestLap ? fmt(bestLap) : '--.--';
    hud.count.textContent = laps;
    const cue = coachText();
    hud.cue.textContent = ADVICE_TEXT[cue.advice] || '';
    hud.cue.style.color = ADVICE_COLOUR[cue.advice] || '#fff';
    hud.cueNext.textContent = cue.nextText;
    hud.cam.textContent = cameraMode;
    hud.quality.textContent = quality + (avgMs ? ' · ' + Math.round(1000 / avgMs) + ' fps' : '');
    hud.pad.textContent = input.connected
      ? (input.padName.length > 34 ? input.padName.slice(0, 34) + '…' : input.padName)
      : 'none — press a button';
    hud.pad.style.color = input.connected ? '#7fe08a' : '#8a9099';
  }

  function fmt(t) {
    if (!isFinite(t)) return '--.--';
    const m = Math.floor(t / 60);
    const s = t - m * 60;
    return `${m > 0 ? m + ':' : ''}${s.toFixed(2).padStart(m > 0 ? 5 : 0, '0')}`;
  }

  requestAnimationFrame(frame);

  // Exposed for automated checks (and for anyone poking at it in the console).
  window.__fd = { sim, track, cameras, get cameraMode() { return cameraMode; } };
  const st = el('status');
  st.textContent = 'Ready — ' + data.track.name + ', ' + Math.round(track.length) + ' m. Press Drive, then any button on your controller.';
  st.style.color = '#7fe08a';
}

main().catch((err) => {
  console.error(err);
  const s = document.getElementById('status');
  if (s) { s.textContent = 'Failed to start: ' + err.message; s.style.color = '#ff6b6b'; }
});

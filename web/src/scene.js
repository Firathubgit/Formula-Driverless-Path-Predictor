// The hall, the track, the coach line, the lamps and the kart — built to read like the
// desktop's drawing (decisions 0050, 0051, 0052) in the browser.
import * as THREE from '../vendor/three.module.js';

const COACH_COLOURS = {
  accelerate: new THREE.Color('#3fd23f'),
  coast: new THREE.Color('#e8c33a'),
  brake: new THREE.Color('#e0473a'),
};

// Racing mode: MeshLambertMaterial is a fraction of the per-fragment cost of the PBR
// MeshStandardMaterial, which matters when every lit pixel is paid for on the CPU. Set once
// before the scene is built.
let SIMPLE = false;
export function setSimpleMaterials(value) { SIMPLE = value; }

function surf(params, simple = SIMPLE) {
  if (!simple) return new THREE.MeshStandardMaterial(params);
  const { roughness, metalness, ...rest } = params;
  return new THREE.MeshLambertMaterial(rest);
}

// Where the driver's own eyes are and where they look, in map coordinates (x, y) with yaw
// counterclockwise from +x. Shared with the automated check so it places the camera exactly as
// the app does, and cannot drift from it.
export function cockpitCameraPose(x, y, yaw, steer, kmh) {
  const fwdX = Math.cos(yaw), fwdZ = -Math.sin(yaw);
  const rightX = -fwdZ, rightZ = fwdX;
  // Just behind the helmet's centre: far enough back that the wheel is not in the face, close
  // enough that it is still a driver's view. Looking ~6 degrees down, so the wheel sits at the
  // bottom of the frame and the track fills the rest.
  const px = x + fwdX * 0.02, pz = -y + fwdZ * 0.02, py = 1.0;
  return {
    position: [px, py, pz],
    look: [px + fwdX * 10 + rightX * steer * 1.2, py - 1.05, pz + fwdZ * 10 + rightZ * steer * 1.2],
    roll: -steer * 0.09,
    fov: 78 + Math.min(8, kmh * 0.12),
  };
}

function asphaltTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 256;
  const g = c.getContext('2d');
  g.fillStyle = '#3a3d42';
  g.fillRect(0, 0, 256, 256);
  for (let i = 0; i < 6000; i++) {
    const v = 40 + Math.random() * 40;
    g.fillStyle = `rgba(${v},${v},${v + 4},${0.25 + Math.random() * 0.4})`;
    g.fillRect(Math.random() * 256, Math.random() * 256, 1.4, 1.4);
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(6, 60);
  t.anisotropy = 8;
  return t;
}

function floorTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 512;
  const g = c.getContext('2d');
  g.fillStyle = '#22252a';
  g.fillRect(0, 0, 512, 512);
  g.strokeStyle = 'rgba(255,255,255,0.05)';
  g.lineWidth = 2;
  for (let i = 0; i <= 512; i += 64) {
    g.beginPath(); g.moveTo(i, 0); g.lineTo(i, 512); g.stroke();
    g.beginPath(); g.moveTo(0, i); g.lineTo(512, i); g.stroke();
  }
  for (let i = 0; i < 4000; i++) {
    const v = 30 + Math.random() * 24;
    g.fillStyle = `rgba(${v},${v},${v},0.35)`;
    g.fillRect(Math.random() * 512, Math.random() * 512, 2, 2);
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(12, 18);
  t.anisotropy = 8;
  return t;
}

function roofTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 256;
  const g = c.getContext('2d');
  g.fillStyle = '#5a6273';
  g.fillRect(0, 0, 256, 256);
  // long panel lines, so the ceiling reads as a built surface rather than a flat fill
  g.strokeStyle = 'rgba(18,22,28,0.6)';
  g.lineWidth = 3;
  for (let i = 0; i <= 256; i += 32) {
    g.beginPath(); g.moveTo(i, 0); g.lineTo(i, 256); g.stroke();
  }
  g.strokeStyle = 'rgba(18,22,28,0.3)';
  g.lineWidth = 2;
  for (let i = 0; i <= 256; i += 128) {
    g.beginPath(); g.moveTo(0, i); g.lineTo(256, i); g.stroke();
  }
  for (let i = 0; i < 3000; i++) {
    const v = 88 + Math.random() * 40;
    g.fillStyle = `rgba(${v},${v + 6},${v + 14},0.22)`;
    g.fillRect(Math.random() * 256, Math.random() * 256, 3, 2);
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(3, 14);
  t.anisotropy = 4;
  return t;
}

export function buildHall(data, opts = {}) {
  const pointLights = opts.pointLights !== undefined ? opts.pointLights : 8;
  const glow = opts.glow !== false;
  const minimal = opts.minimal === true;
  const hall = data.hall;
  const group = new THREE.Group();
  const w = hall.max_x - hall.min_x, d = hall.max_z - hall.min_z;
  const cx = (hall.min_x + hall.max_x) / 2, cz = (hall.min_z + hall.max_z) / 2;

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(w, d),
    surf({ map: floorTexture(), color: 0xffffff, roughness: 0.95, metalness: 0.02 }));
  floor.rotation.x = -Math.PI / 2;
  floor.position.set(cx, 0, cz);
  floor.receiveShadow = true;
  group.add(floor);

  // An indoor hall is lit from below by its own lamps, so the ceiling and upper walls would
  // otherwise render almost black (we see the unlit underside of the roof). Give them a little
  // self-illumination so the room reads as a room rather than a void.
  const wallMat = surf({ color: 0x3a414c, roughness: 0.9, metalness: 0.05, side: THREE.DoubleSide, emissive: 0x14171c });
  const mkWall = (width, px, pz, ry) => {
    const m = new THREE.Mesh(new THREE.PlaneGeometry(width, hall.eave_m), wallMat);
    m.position.set(px, hall.eave_m / 2, pz);
    m.rotation.y = ry;
    group.add(m);
  };
  mkWall(w, cx, hall.min_z, 0);
  mkWall(w, cx, hall.max_z, Math.PI);
  mkWall(d, hall.min_x, cz, Math.PI / 2);
  mkWall(d, hall.max_x, cz, -Math.PI / 2);

  // A pitched roof to the ridge. Always built: without it the ceiling is a black void with the
  // lamps floating in it, which reads as broken. Racing mode drops only the rafters.
  const half = w / 2, rise = hall.ridge_m - hall.eave_m;
  {
    const roofMat = surf({ map: roofTexture(), color: 0xffffff, roughness: 0.85, metalness: 0.15, side: THREE.DoubleSide, emissive: 0x262c36 });
    const slope = Math.hypot(half, rise);
    for (const side of [-1, 1]) {
      const panel = new THREE.Mesh(new THREE.PlaneGeometry(slope, d), roofMat);
      panel.position.set(cx + side * half / 2, hall.eave_m + rise / 2, cz);
      panel.rotation.order = 'YXZ';
      panel.rotation.y = side > 0 ? 0 : Math.PI;
      panel.rotation.x = -Math.atan2(rise, half) * (side > 0 ? 1 : -1);
      group.add(panel);
    }
    // The rafters are what give the ceiling structure; without them it is a featureless
    // surface, which reads as a void however bright it is. They must hang BELOW the roof
    // everywhere to be seen: the roof is at its lowest (eave_m) at the walls, so a beam at
    // that height or above is buried inside the roof plane.
    const rafterMat = surf({ color: 0x6d7788, roughness: 0.8, metalness: 0.3, emissive: 0x1c2026 });
    const beamY = hall.eave_m - 0.35;
    for (let z = hall.min_z + 3; z < hall.max_z; z += 6) {
      const beam = new THREE.Mesh(new THREE.BoxGeometry(w, 0.26, 0.26), rafterMat);
      beam.position.set(cx, beamY, z);
      group.add(beam);
    }
    // A ridge beam just under the ridge, the one strong line overhead.
    const ridge = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.34, d), rafterMat);
    ridge.position.set(cx, hall.ridge_m - 0.35, cz);
    group.add(ridge);
  }

  // Lamps: an emissive fixture per grid position, with a handful of real lights.
  const lampGeo = new THREE.CircleGeometry(0.34, 12);
  const lampMat = new THREE.MeshBasicMaterial({ color: 0xfff6e0 });
  const glowMat = new THREE.MeshBasicMaterial({ color: 0xdfe8ff, transparent: true, opacity: 0.28, blending: THREE.AdditiveBlending, depthWrite: false });
  const lamps = new THREE.InstancedMesh(lampGeo, lampMat, hall.lamp_columns * hall.lamp_rows);
  const glows = glow ? new THREE.InstancedMesh(lampGeo, glowMat, hall.lamp_columns * hall.lamp_rows) : null;
  const dummy = new THREE.Object3D();
  let i = 0;
  const lightPositions = [];
  for (let col = 0; col < hall.lamp_columns; col++) {
    for (let row = 0; row < hall.lamp_rows; row++) {
      const x = hall.lamp_x0 + col * hall.lamp_dx;
      const z = hall.lamp_z0 + row * hall.lamp_dz;
      dummy.position.set(x, hall.lamp_height_m, z);
      dummy.rotation.set(Math.PI / 2, 0, 0);
      dummy.scale.setScalar(1);
      dummy.updateMatrix();
      lamps.setMatrixAt(i, dummy.matrix);
      if (glows) {
        dummy.position.y = hall.lamp_height_m - 0.05;
        dummy.scale.setScalar(3.2);
        dummy.updateMatrix();
        glows.setMatrixAt(i, dummy.matrix);
      }
      if ((col + row) % 3 === 0) lightPositions.push([x, hall.lamp_height_m - 0.2, z]);
      i++;
    }
  }
  lamps.instanceMatrix.needsUpdate = true;
  group.add(lamps);
  if (glows) { glows.name = 'lamp-glow'; glows.instanceMatrix.needsUpdate = true; group.add(glows); }

  // Point lights are the most expensive thing in the fragment shader, so their number is a
  // quality setting: a few on a capable GPU, none on a machine rendering in software.
  const stride = Math.max(1, Math.ceil(lightPositions.length / Math.max(1, pointLights)));
  let added = 0;
  for (let n = 0; n < lightPositions.length && added < pointLights; n += stride) {
    const [x, y, z] = lightPositions[n];
    const pl = new THREE.PointLight(0xfff2d8, 26, 26, 2.0);
    pl.position.set(x, y, z);
    group.add(pl);
    added++;
  }
  return group;
}

// The road: a ribbon from the centreline, lifted across its width by the banked bend.
export function buildRoad(track) {
  const group = new THREE.Group();
  const n = track.n;
  const half = track.half;
  const pos = [], uv = [], idx = [];
  for (let i = 0; i < n; i++) {
    const s = track.stations[i];
    for (const o of [half, -half]) {
      const p = track.offsetPoint(s, o);
      const y = track.heightAt(s, o) + 0.01;
      pos.push(p.x, y, -p.y);
      uv.push(o > 0 ? 0 : 1, s / 4);
    }
  }
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    const a = i * 2, b = i * 2 + 1, c = j * 2, dd = j * 2 + 1;
    idx.push(a, c, b, b, c, dd);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  geo.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  geo.setIndex(idx);
  geo.computeVertexNormals();
  const road = new THREE.Mesh(geo, surf({ map: asphaltTexture(), roughness: 0.88, metalness: 0.05 }));
  road.receiveShadow = true;
  group.add(road);

  // Painted edge lines.
  for (const sign of [1, -1]) {
    const pts = [];
    for (let i = 0; i <= n; i++) {
      const s = track.stations[i % n];
      const p = track.offsetPoint(s, sign * (half - 0.16));
      pts.push(new THREE.Vector3(p.x, track.heightAt(s, sign * (half - 0.16)) + 0.03, -p.y));
    }
    const line = new THREE.Mesh(
      new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts, true), 400, 0.05, 5, true),
      surf({ color: 0xd8d8d0, roughness: 0.7 }));
    group.add(line);
  }

  // Kerbs on both edges through every corner span.
  const kerbA = surf({ color: 0xc9302c, roughness: 0.8 });
  const kerbB = surf({ color: 0xe8e6e0, roughness: 0.8 });
  for (const corner of track.corners) {
    const span = corner.to_s - corner.from_s;
    const segs = Math.max(2, Math.round(span / 2));
    for (let k = 0; k < segs; k++) {
      const s0 = corner.from_s + (k * span) / segs;
      const s1 = corner.from_s + ((k + 1) * span) / segs;
      for (const sign of [1, -1]) {
        const o0 = sign * half, o1 = sign * (half + 0.55);
        const p0 = track.offsetPoint(s0, o0), p1 = track.offsetPoint(s1, o0);
        const q0 = track.offsetPoint(s0, o1), q1 = track.offsetPoint(s1, o1);
        const y = (s, o) => track.heightAt(s, o) + 0.04;
        const g = new THREE.BufferGeometry();
        g.setAttribute('position', new THREE.Float32BufferAttribute([
          p0.x, y(s0, o0), -p0.y, p1.x, y(s1, o0), -p1.y, q1.x, y(s1, o1), -q1.y,
          p0.x, y(s0, o0), -p0.y, q1.x, y(s1, o1), -q1.y, q0.x, y(s0, o1), -q0.y,
        ], 3));
        g.computeVertexNormals();
        const mesh = new THREE.Mesh(g, (k % 2 === 0) ? kerbA : kerbB);
        group.add(mesh);
      }
    }
  }

  // The poster's coaching line, painted in its own three colours (decision 0039).
  // Stations are precomputed in main.js (guideStations), so no projection happens here.
  const guide = track.guidePoints;
  if (guide && guide.length > 1) {
    const gp = [], gc = [], gi = [];
    const w = 0.20;
    for (let i = 0; i + 1 < guide.length; i++) {
      const g0 = guide[i], g1 = guide[i + 1];
      const dx = g1.x - g0.x, dy = g1.y - g0.y, d = Math.hypot(dx, dy) || 1;
      const nx = -dy / d * w, ny = dx / d * w;
      const y0 = track.heightAt(g0.s, 0) + 0.035;
      const y1 = track.heightAt(g1.s, 0) + 0.035;
      const base = gp.length / 3;
      gp.push(g0.x + nx, y0, -(g0.y + ny), g1.x + nx, y1, -(g1.y + ny),
              g1.x - nx, y1, -(g1.y - ny), g0.x - nx, y0, -(g0.y - ny));
      const c = COACH_COLOURS[g0.advice] || COACH_COLOURS.coast;
      for (let k = 0; k < 4; k++) gc.push(c.r, c.g, c.b);
      gi.push(base, base + 1, base + 2, base, base + 2, base + 3);
    }
    const geo2 = new THREE.BufferGeometry();
    geo2.setAttribute('position', new THREE.Float32BufferAttribute(gp, 3));
    geo2.setAttribute('color', new THREE.Float32BufferAttribute(gc, 3));
    geo2.setIndex(gi);
    geo2.computeVertexNormals();
    group.add(new THREE.Mesh(geo2, surf({
      vertexColors: true, roughness: 0.6, emissive: 0x111111, side: THREE.DoubleSide })));
  }

  // The start line across the corridor.
  {
    const s = 0.0;
    const p0 = track.offsetPoint(s, half), p1 = track.offsetPoint(s, -half);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute([
      p0.x - 0.2, 0.04, -p0.y, p1.x - 0.2, 0.04, -p1.y, p1.x + 0.2, 0.04, -p1.y,
      p0.x - 0.2, 0.04, -p0.y, p1.x + 0.2, 0.04, -p1.y, p0.x + 0.2, 0.04, -p0.y,
    ], 3));
    g.computeVertexNormals();
    group.add(new THREE.Mesh(g, surf({ color: 0xf2f2f2, roughness: 0.6, side: THREE.DoubleSide })));
  }
  return group;
}

// Twin-tube barriers along both edges, with posts (decision 0052's solid boundary).
export function buildBarriers(track) {
  const group = new THREE.Group();
  const mat = surf({ color: 0xd9d9d9, roughness: 0.5, metalness: 0.35 });
  const postMat = surf({ color: 0x8a8f96, roughness: 0.6, metalness: 0.4 });
  for (const sign of [1, -1]) {
    const offset = sign * (track.half + 0.32);
    for (const tubeY of [0.34, 0.62]) {
      const pts = [];
      for (let i = 0; i <= track.n; i += 2) {
        const s = track.stations[i % track.n];
        const p = track.offsetPoint(s, offset);
        pts.push(new THREE.Vector3(p.x, track.heightAt(s, offset) + tubeY, -p.y));
      }
      const curve = new THREE.CatmullRomCurve3(pts, true);
      const tube = new THREE.Mesh(new THREE.TubeGeometry(curve, 200, 0.05, 6, true), mat);
      group.add(tube);
    }
    // posts, instanced (there are a few hundred of them)
    const count = Math.ceil(track.n / 4);
    const posts = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.035, 0.035, 0.7, 6), postMat, count);
    const dummy = new THREE.Object3D();
    let pi = 0;
    for (let i = 0; i < track.n; i += 4) {
      const s = track.stations[i];
      const p = track.offsetPoint(s, offset);
      dummy.position.set(p.x, track.heightAt(s, offset) + 0.35, -p.y);
      dummy.rotation.set(0, 0, 0);
      dummy.updateMatrix();
      posts.setMatrixAt(pi++, dummy.matrix);
    }
    posts.instanceMatrix.needsUpdate = true;
    group.add(posts);
  }
  return group;
}

// A clean, readable kart: pan, pods, seat, driver, nose, bumper and four wheels.
export function buildKart(kart) {
  const g = new THREE.Group();
  const bodyMat = surf({ color: 0xd93b2b, roughness: 0.4, metalness: 0.25 });
  const darkMat = surf({ color: 0x1c1f24, roughness: 0.7, metalness: 0.2 });
  const seatMat = surf({ color: 0x15181c, roughness: 0.9 });
  const suitMat = surf({ color: 0x2b3a67, roughness: 0.7 });
  const helmetMat = surf({ color: 0xf2f2f2, roughness: 0.25, metalness: 0.1 });

  const pan = new THREE.Mesh(new THREE.BoxGeometry(1.55, 0.09, 0.86), darkMat);
  pan.name = 'pan';
  pan.position.y = 0.1;
  g.add(pan);

  const nose = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.14, 0.7), bodyMat);
  nose.name = 'nose';
  nose.position.set(0.92, 0.13, 0);
  // tilted nose-down: seen from the driver's eyes a level top face is a flat red bar with no
  // shading at all, which reads as a placeholder rather than bodywork
  nose.rotation.z = -0.12;
  g.add(nose);
  const panelMat = surf({ color: 0xe8e2d6, roughness: 0.5 });
  const nosePanel = new THREE.Mesh(new THREE.BoxGeometry(0.30, 0.03, 0.44), panelMat);
  nosePanel.name = 'nose-panel';
  nosePanel.position.set(0.92, 0.215, 0);
  nosePanel.rotation.z = -0.12;
  g.add(nosePanel);
  const bumper = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.2, 0.9), darkMat);
  bumper.name = 'bumper';
  bumper.position.set(-0.85, 0.16, 0);
  g.add(bumper);

  for (const side of [1, -1]) {
    const pod = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.22, 0.26), bodyMat);
    pod.name = side > 0 ? 'pod-left' : 'pod-right';
    pod.position.set(0.15, 0.19, side * 0.5);
    g.add(pod);
  }
  const seat = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.5, 0.5), seatMat);
  seat.name = 'seat';
  seat.position.set(-0.08, 0.36, 0);
  g.add(seat);
  const backrest = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.5, 0.46), seatMat);
  backrest.name = 'backrest';
  backrest.position.set(-0.3, 0.55, 0);
  g.add(backrest);

  const torso = new THREE.Mesh(new THREE.CapsuleGeometry(0.2, 0.42, 6, 12), suitMat);
  torso.name = 'torso';
  torso.position.set(-0.05, 0.72, 0);
  torso.rotation.z = Math.PI / 2 - 0.25;
  g.add(torso);
  const helmet = new THREE.Mesh(new THREE.SphereGeometry(0.16, 20, 16), helmetMat);
  helmet.position.set(0.16, 0.92, 0);
  helmet.name = 'helmet';
  g.add(helmet);
  const visor = new THREE.Mesh(new THREE.SphereGeometry(0.163, 20, 16, Math.PI * 0.15, Math.PI * 0.7, Math.PI * 0.3, Math.PI * 0.3),
    surf({ color: 0x101820, roughness: 0.1, metalness: 0.6 }));
  visor.position.copy(helmet.position);
  visor.name = 'visor';
  g.add(visor);

  const wheelGeo = new THREE.CylinderGeometry(kart.wheel_radius_m, kart.wheel_radius_m, 0.13, 20);
  wheelGeo.rotateX(Math.PI / 2);
  const tyreMat = surf({ color: 0x101214, roughness: 0.95 });
  const rimMat = surf({ color: 0xcfd3d8, roughness: 0.35, metalness: 0.6 });
  const wheels = { fl: null, fr: null, rl: null, rr: null };
  const axles = [
    ['fl', kart.cg_to_front_m, kart.track_front_m / 2],
    ['fr', kart.cg_to_front_m, -kart.track_front_m / 2],
    ['rl', -kart.cg_to_rear_m, kart.track_rear_m / 2],
    ['rr', -kart.cg_to_rear_m, -kart.track_rear_m / 2],
  ];
  for (const [name, ax, az] of axles) {
    const pivot = new THREE.Group();
    pivot.position.set(ax, kart.wheel_radius_m, az);
    const tyre = new THREE.Mesh(wheelGeo, tyreMat);
    tyre.castShadow = true;
    const rim = new THREE.Mesh(new THREE.CylinderGeometry(kart.wheel_radius_m * 0.55, kart.wheel_radius_m * 0.55, 0.14, 14), rimMat);
    rim.rotation.x = Math.PI / 2;
    pivot.add(tyre, rim);
    g.add(pivot);
    wheels[name] = pivot;
  }

  // Steering wheel, seen in the cockpit view. Lighter than the seat: a near-black rim on a
  // near-black floor renders as a shapeless hole.
  const wheelMat = surf({ color: 0x4d5560, roughness: 0.55 });
  const sw = new THREE.Group();
  const ring = new THREE.Mesh(new THREE.TorusGeometry(0.14, 0.022, 8, 24), wheelMat);
  ring.name = 'steer-ring';
  sw.add(ring);
  const hub = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, 0.05), wheelMat);
  hub.name = 'steer-hub';
  sw.add(hub);
  sw.name = 'steer-wheel';
  sw.position.set(0.42, 0.72, 0);
  sw.rotation.y = Math.PI / 2;
  sw.rotation.x = -0.5;
  g.add(sw);

  g.traverse((o) => { if (o.isMesh) { o.castShadow = true; } });
  return { group: g, wheels, steeringWheel: sw };
}

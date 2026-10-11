// The kart: a single-track dynamic model on the repo's fitted rental-kart parameters,
// with the desktop's 40% tire-friction bump (decisions 0046, 0055) and a grip aid on the
// person's command (decision 0047). Units are SI; the plant is planar.

const G = 9.80665;
const RHO = 1.225;

// A tire's force as a share of its peak: rises to 1 at the peak slip, then falls to
// `slide` and holds there (the repo's sliding fractions).
function tireCurve(u, slide) {
  const a = Math.abs(u);
  let f;
  if (a <= 1) f = Math.sin(a * Math.PI / 2);
  else f = Math.max(slide, 1 - (1 - slide) * (a - 1) / 2);
  return Math.sign(u) * f;
}

export class KartSim {
  constructor(kart, track) {
    this.k = kart;
    this.track = track;
    this.reset(0, 0);
  }

  reset(station = 0, speed = 0) {
    const k = this.k, tr = this.track;
    const p = tr.pointAt(station);
    const t = tr.tangentAt(station);
    this.x = p.x; this.y = p.y;
    this.yaw = Math.atan2(t.y, t.x);
    this.vx = speed; this.vy = 0; this.r = 0;
    this.steer = 0;
    this.s = station;
    this.offset = 0;
    this.throttle = 0; this.brake = 0;
    this.slipUse = 0;
    this.offTrack = false;
    this.wheelSpin = 0;
    this.lastCollision = 0;
    this.resetCounter = 0;
  }

  // Steering limit: never more lock than the tires can actually use at this speed, capped
  // at the kart's own range. This is the grip aid on the person's command (decision 0047).
  steerLimit(speed) {
    const k = this.k;
    const ayCap = k.grip_lateral * G * 0.9;
    const byGrip = Math.atan2(ayCap * k.wheelbase_m, Math.max(speed, 1.5) ** 2);
    return Math.min(k.max_steering_rad, byGrip);
  }

  step(dt, input) {
    const k = this.k, tr = this.track;
    const m = k.mass_kg;
    const a = k.cg_to_front_m, b = k.cg_to_rear_m, L = k.wheelbase_m;
    const h = k.cg_height_m;
    const Izz = k.yaw_inertia_kgm2;

    // ---- steering: rate-limited toward the command, inside the speed-dependent limit
    let cmd = Math.max(-1, Math.min(1, input.steer));
    const limit = this.steerLimit(Math.abs(this.vx));
    const target = cmd * limit;
    const maxStep = k.max_steering_rate_radps * dt;
    const dSteer = Math.max(-maxStep, Math.min(maxStep, target - this.steer));
    this.steer += dSteer;

    // ---- longitudinal demand
    let throttle = Math.max(0, Math.min(1, input.throttle));
    let brake = Math.max(0, Math.min(1, input.brake));
    const v = Math.abs(this.vx);
    const speed = Math.max(v, 0.5);
    const torqueForce = 2 * k.max_drive_torque_nm / k.wheel_radius_m;
    const powerForce = k.max_drive_power_w / speed;
    const fade = Math.max(0, Math.min(1, (k.max_drive_speed_mps - v) / (0.05 * k.max_drive_speed_mps)));
    let drive = Math.min(torqueForce, powerForce) * fade * throttle;
    const brakeForce = 2 * k.max_brake_torque_nm / k.wheel_radius_m * brake;

    // grip aid: no more longitudinal demand than the tires hold at this speed
    const latUse = Math.min(1, (this.vy * this.r + v * v * Math.abs(this.steer) / L) / (k.grip_lateral * G + 1e-6));
    const longBudget = k.grip_longitudinal * G * Math.sqrt(Math.max(0, 1 - latUse * latUse)) * 0.95;
    drive = Math.min(drive, longBudget * m);

    // ---- axle loads with quasi-static longitudinal transfer
    let axGuess = (drive - brakeForce) / m;
    let FzF = Math.max(0, m * G * b / L - m * axGuess * h / L);
    let FzR = Math.max(0, m * G * a / L + m * axGuess * h / L);

    // ---- slip angles
    const vSafe = Math.max(v, 1.0);
    const latScale = Math.min(1, v / 2.0); // no lateral force without motion
    const af = Math.atan2(this.vy + a * this.r, vSafe) - this.steer;
    const ar = Math.atan2(this.vy - b * this.r, vSafe);

    // ---- tire forces
    const muLat = k.grip_lateral, muLong = k.grip_longitudinal;
    let FyF = -tireCurve(af / k.peak_slip_angle_rad, k.sliding_fraction_lateral) * muLat * FzF * latScale;
    let FyR = -tireCurve(ar / k.peak_slip_angle_rad, k.sliding_fraction_lateral) * muLat * FzR * latScale;

    // longitudinal per axle: drive on the rear (a kart drives its rear axle), braking biased
    let FxF = -brakeForce * k.brake_bias_front;
    let FxR = drive - brakeForce * (1 - k.brake_bias_front);

    // friction circle per axle
    const clampAxle = (fx, fy, fz) => {
      const cap = Math.hypot(muLong * fz, muLat * fz);
      const mag = Math.hypot(fx, fy);
      if (mag > cap && mag > 1e-6) { const s = cap / mag; return [fx * s, fy * s]; }
      return [fx, fy];
    };
    [FxF, FyF] = clampAxle(FxF, FyF, FzF);
    [FxR, FyR] = clampAxle(FxR, FyR, FzR);

    // ---- resistance
    const drag = 0.5 * RHO * k.drag_area_m2 * this.vx * this.vx * Math.sign(this.vx || 1);
    const roll = k.rolling_resistance * m * G * Math.sign(this.vx || (drive > 0 ? 1 : 0));

    // ---- body-frame equations
    const cosS = Math.cos(this.steer);
    const FyBody = FyF * cosS + FyR;
    const Mz = a * FyF * cosS - b * FyR;

    // reverse: a kart backs up when the brake is held at rest
    let reverseDrive = 0;
    if (v < 0.3 && brake > 0.2 && throttle < 0.05) {
      reverseDrive = -Math.min(brake, 1) * 0.5 * k.max_drive_power_w / 3000;
    }

    const ax = (FxF + FxR - drag - roll) / m + this.vy * this.r + reverseDrive;
    const ay = FyBody / m - this.vx * this.r;
    this.vx += ax * dt;
    this.vy += ay * dt;
    this.r += (Mz / Izz) * dt;

    // standstill damping so the kart settles instead of jittering
    if (v < 0.2 && throttle < 0.02 && brake < 0.02) { this.vx *= 0.90; this.vy *= 0.90; this.r *= 0.90; }
    this.vy *= 0.999;

    this.yaw += this.r * dt;
    this.x += (this.vx * Math.cos(this.yaw) - this.vy * Math.sin(this.yaw)) * dt;
    this.y += (this.vx * Math.sin(this.yaw) + this.vy * Math.cos(this.yaw)) * dt;

    this.wheelSpin += this.vx / k.wheel_radius_m * dt;
    this.slipUse = Math.max(
      Math.abs(af) / k.peak_slip_angle_rad,
      Math.abs(ar) / k.peak_slip_angle_rad);

    // ---- barriers: planar inelastic contact (decision 0052).
    // The station advances with the kart's own motion projected on the local tangent, so
    // the lateral offset is measured against the nearest station and can never snap to
    // another leg of the loop.
    const tan0 = tr.tangentAt(this.s);
    const vwx = this.vx * Math.cos(this.yaw) - this.vy * Math.sin(this.yaw);
    const vwy = this.vx * Math.sin(this.yaw) + this.vy * Math.cos(this.yaw);
    this.s = ((this.s + (vwx * tan0.x + vwy * tan0.y) * dt) % tr.length + tr.length) % tr.length;

    const p = tr.pointAt(this.s);
    const tan = tr.tangentAt(this.s);
    const rx = this.x - p.x, ry = this.y - p.y;
    const offset = rx * (-tan.y) + ry * (tan.x);
    this.offset = offset;
    const clearance = tr.half - 0.55; // the kart's half width at the bumper
    this.offTrack = Math.abs(offset) > tr.half;
    if (Math.abs(offset) > clearance) {
      const excess = Math.abs(offset) - clearance;
      const sign = Math.sign(offset);
      const nx = -tan.y * sign, ny = tan.x * sign;
      this.x -= nx * excess; this.y -= ny * excess;
      // remove the inward normal velocity, keep some along the wall (scrub)
      const vn = vwx * nx + vwy * ny;
      if (vn > 0) {
        const scrub = 0.35 * Math.min(1, Math.abs(vn) / 5);
        const wx = vwx - vn * nx, wy = vwy - vn * ny;
        const c = Math.cos(-this.yaw), s2 = Math.sin(-this.yaw);
        this.vx = (wx * c - wy * s2) * (1 - scrub);
        this.vy = (wx * s2 + wy * c) * (1 - scrub);
        this.lastCollision = Math.min(1, Math.abs(vn) / 6);
      }
    } else {
      this.lastCollision *= 0.9;
    }

    this.throttle = throttle; this.brake = brake;
  }

  get speed() { return Math.hypot(this.vx, this.vy); }
  get kmh() { return this.speed * 3.6; }
}

// Track geometry: stations, projection with a continuity window, offsets, banking.
// Map coordinates (x, y) are the repo's; the scene uses x and z = -y.

export function smoothstep(t) {
  t = Math.max(0, Math.min(1, t));
  return t * t * (3 - 2 * t);
}

export class Track {
  constructor(data) {
    this.name = data.track.name;
    this.points = data.track.points;          // [[x, y], ...] closed loop
    this.stations = data.track.stations;      // cumulative metres, closed
    this.curvature = data.track.curvature;
    this.width = data.track.width_m;
    this.half = this.width / 2;
    this.length = data.track.stations[data.track.stations.length - 1] +
      Math.hypot(this.points[0][0] - this.points[this.points.length - 1][0],
                 this.points[0][1] - this.points[this.points.length - 1][1]);
    this.n = this.points.length;
    this.banking = data.banking;
    this.corners = data.corners;
  }

  // Height of the road surface at (station s, signed offset o): left edge at the floor,
  // the banked bend lifting the right edge by the corridor width times tan(tilt).
  heightAt(s, o) {
    const b = this.banking;
    if (!b || b.tilt_rad === 0) return 0;
    const span = b.to_s_m - b.from_s_m;
    let d = s - b.from_s_m;
    if (d < 0 || d > span) return 0;
    const ramp = Math.max(1e-3, b.ramp_m);
    const share = Math.min(smoothstep(d / ramp), smoothstep((span - d) / ramp));
    const rise = this.width * Math.tan(b.tilt_rad);
    return share * ((this.half - o) / this.width) * rise;
  }

  indexFor(s) {
    s = ((s % this.length) + this.length) % this.length;
    // stations are near-uniform (1 m); search is cheap and exact enough
    let lo = 0, hi = this.n - 1;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (this.stations[mid] <= s) lo = mid; else hi = mid - 1;
    }
    return lo;
  }

  pointAt(s) {
    s = ((s % this.length) + this.length) % this.length;
    const i = this.indexFor(s);
    const j = (i + 1) % this.n;
    const s0 = this.stations[i];
    const s1 = i + 1 < this.n ? this.stations[i + 1] : this.length;
    const t = s1 > s0 ? (s - s0) / (s1 - s0) : 0;
    const a = this.points[i], b = this.points[j];
    return { x: a[0] + (b[0] - a[0]) * t, y: a[1] + (b[1] - a[1]) * t };
  }

  tangentAt(s) {
    const eps = 0.5;
    const a = this.pointAt(s - eps), b = this.pointAt(s + eps);
    const dx = b.x - a.x, dy = b.y - a.y;
    const d = Math.hypot(dx, dy) || 1;
    return { x: dx / d, y: dy / d };
  }

  // Signed offset from the centreline, left positive.
  offsetPoint(s, o) {
    const p = this.pointAt(s), t = this.tangentAt(s);
    return { x: p.x - t.y * o, y: p.y + t.x * o };
  }

  curvatureAt(s) {
    return this.curvature[this.indexFor(s)];
  }

  // Nearest station and signed offset, searched only near a hint so a kart inside a
  // hairpin cannot jump to the far side of the loop. The window widens only when the
  // kart is genuinely far from the hinted station (recovering from a spin).
  project(x, y, hintS) {
    let best = null;
    let window = 6;
    for (let attempt = 0; attempt < 5; attempt++) {
      const step = 0.5;
      const count = Math.max(4, Math.round((2 * window) / step));
      best = { s: hintS, d2: Infinity };
      for (let k = 0; k <= count; k++) {
        const s = hintS - window + k * (2 * window) / count;
        const p = this.pointAt(s);
        const d2 = (p.x - x) ** 2 + (p.y - y) ** 2;
        if (d2 < best.d2) best = { s, d2 };
      }
      if (best.d2 < 25) break; // within 5 m: the hint was sound
      window *= 2;
    }
    // refine along the tangent: project the residual onto the local frame, a bounded
    // step at a time so a kart far from its hint cannot run the station away
    let s = best.s;
    for (let it = 0; it < 4; it++) {
      const p = this.pointAt(s), t = this.tangentAt(s);
      const rx = x - p.x, ry = y - p.y;
      let step = rx * t.x + ry * t.y;
      step = Math.max(-1.5, Math.min(1.5, step));
      s += step;
      if (Math.abs(step) < 1e-3) break;
    }
    const p = this.pointAt(s), t = this.tangentAt(s);
    const rx = x - p.x, ry = y - p.y;
    // left normal is (-t.y, t.x)
    const offset = rx * (-t.y) + ry * (t.x);
    const along = rx * t.x + ry * t.y;
    return { s: ((s % this.length) + this.length) % this.length, offset, along, distance: Math.hypot(rx, ry) };
  }

  // Corner span containing a station, or null.
  cornerAt(s) {
    for (const c of this.corners) {
      if (s >= c.from_s && s <= c.to_s) return c;
    }
    return null;
  }
}

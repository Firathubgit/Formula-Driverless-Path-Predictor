// Input: the browser Gamepad API (a PS4 DualShock/DualSense reports the standard
// mapping on macOS Chrome/Edge/Firefox/Safari) with a keyboard fallback.

const DEAD_ZONE = 0.12;

function past(value, dead = DEAD_ZONE) {
  const a = Math.abs(value);
  if (a <= dead) return 0;
  return Math.sign(value) * Math.min(1, (a - dead) / (1 - dead));
}

export class Input {
  constructor() {
    this.keys = new Set();
    this.padIndex = null;
    this.padName = '';
    this.steer = 0;
    this.throttle = 0;
    this.brake = 0;
    this.menuPressed = false;
    this.cameraPressed = false;
    this.resetPressed = false;
    this._menuLatch = false;
    this._camLatch = false;
    this._resetLatch = false;

    addEventListener('keydown', (e) => {
      if (e.repeat) return;
      this.keys.add(e.code);
      if (e.code === 'Escape' || e.code === 'Enter') this._menuLatch = true;
      if (e.code === 'KeyC') this._camLatch = true;
      if (e.code === 'KeyR') this._resetLatch = true;
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'].includes(e.code)) e.preventDefault();
    });
    addEventListener('keyup', (e) => this.keys.delete(e.code));
    addEventListener('gamepadconnected', (e) => this._onConnect(e.gamepad));
    addEventListener('gamepaddisconnected', (e) => {
      if (this.padIndex === e.gamepad.index) { this.padIndex = null; this.padName = ''; }
    });
  }

  _onConnect(pad) {
    this.padIndex = pad.index;
    this.padName = pad.id;
  }

  _pads() {
    const list = navigator.getGamepads ? navigator.getGamepads() : [];
    const out = [];
    for (const p of list) if (p && p.connected) out.push(p);
    return out;
  }

  get connected() { return this._pads().length > 0; }

  poll() {
    const pads = this._pads();
    let pad = null;
    if (this.padIndex !== null) pad = pads.find((p) => p.index === this.padIndex) || null;
    if (!pad) pad = pads[0] || null;
    if (pad) { this.padIndex = pad.index; this.padName = pad.id; }

    let steer = 0, throttle = 0, brake = 0;

    if (pad) {
      const ax = pad.axes;
      const bt = pad.buttons;
      // left stick steers; a person's steering is positive to the left
      const lx = ax.length > 0 ? ax[0] : 0;
      const rx = ax.length > 2 ? ax[2] : 0;
      steer = -past(Math.abs(lx) > Math.abs(rx) ? lx : rx);
      const l2 = bt[6] ? bt[6].value : 0;
      const r2 = bt[7] ? bt[7].value : 0;
      throttle = past(r2, 0.05);
      brake = past(l2, 0.05);
      // shoulders as buttons too, for pads whose triggers are digital
      if (bt[7] && bt[7].pressed) throttle = Math.max(throttle, 1);
      if (bt[6] && bt[6].pressed) brake = Math.max(brake, 1);
      const menu = (bt[9] && bt[9].pressed) || (bt[8] && bt[8].pressed);
      if (menu && !this._menuLatch) this.menuPressed = true;
      this._menuLatch = menu;
      const cam = (bt[3] && bt[3].pressed) || (bt[1] && bt[1].pressed); // triangle / circle
      if (cam && !this._camLatch) this.cameraPressed = true;
      this._camLatch = cam;
      const reset = bt[2] && bt[2].pressed; // square
      if (reset && !this._resetLatch) this.resetPressed = true;
      this._resetLatch = reset;
    }

    // keyboard fallback / merge
    const k = this.keys;
    let kSteer = 0;
    if (k.has('ArrowLeft') || k.has('KeyA')) kSteer += 1;
    if (k.has('ArrowRight') || k.has('KeyD')) kSteer -= 1;
    if (kSteer !== 0) steer = kSteer;
    if (k.has('ArrowUp') || k.has('KeyW')) throttle = 1;
    if (k.has('ArrowDown') || k.has('KeyS') || k.has('Space')) brake = 1;

    if (this._menuLatch && k.has('Escape')) { /* keyboard menu handled by caller */ }

    this.steer = steer;
    this.throttle = throttle;
    this.brake = brake;
    return { steer, throttle, brake };
  }

  consumeMenu() { const v = this.menuPressed; this.menuPressed = false; return v; }
  consumeCamera() { const v = this.cameraPressed; this.cameraPressed = false; return v; }
  consumeReset() { const v = this.resetPressed; this.resetPressed = false; return v; }
}

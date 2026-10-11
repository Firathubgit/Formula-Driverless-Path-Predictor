# Gokartcentralen Göteborg — browser build

The desktop's gokart mode, rebuilt to run entirely in the browser: **no plugin, no download,
no server-side rendering** — three.js and the kart physics run client-side, so a Mac, a phone or
a Linux box can play it, with a controller.

Play it at
**https://firathubgit.github.io/Formula-Driverless-Path-Predictor/**

## Controls

| Action | DualShock 4 / DualSense | Keyboard |
| --- | --- | --- |
| Steer | left (or right) stick | `A` / `D`, ← / → |
| Throttle | R2 | `W`, ↑ |
| Brake / reverse | L2 | `S`, space |
| Camera (cockpit / chase / top) | △ or ○ | `C` |
| Reset to the line | □ | `R` |

The view starts **in the driver's own head** — the kart's steering wheel, nose and front tyres are
in frame, as they are from a real seat. `C` cycles to a chase camera and a top-down view.

On macOS, pair the controller over Bluetooth and then **press any button on it** — browsers only
hand over a gamepad once it has been used. Chrome or Edge give the fullest controller support;
Safari works too. The pad's name appears in the HUD once it is seen.

## What is faithful to the desktop, and what is not

**Ported from the repository's own data** (`web/tools/make_data.py` reads these, unchanged):

- the traced track, `tracks/gokartcentralen-goteborg.csv` — 400 points, 399.84 m, 6 m wide;
- the operator's poster racing line, `tracks/gokartcentralen-goteborg-poster-line.csv`, painted on
  the asphalt in its own three colours;
- the poster's coaching advice, `tracks/gokartcentralen-goteborg-guide.csv` (green gas, yellow
  lift, red brake) — the same cues the desktop's coach reads;
- the hall's dimensions and the lamp grid from `apps/desktop/kart_hall.cpp`;
- the banked ninth bend from `core/src/track_relief.cpp` (292–336 m, 8 m ramp, 6°);
- the fitted rental kart from `core/src/vehicle_profiles.cpp`
  (`gokartcentralen-realistic`): 261 kg, 14 hp, 60 km/h, 1.045 m wheelbase, 0.55 rad of lock at
  3 rad/s, and the desktop's 40 % tire-friction bump (decision 0055) — 1.89 g sideways, 1.40 g
  along.

**Reimplemented, not compiled.** The desktop is C++20 and Qt Quick 3D; a Qt Quick 3D application
cannot be served to a browser as-is, so the physics and the drawing are a fresh implementation:

- the kart is a single-track dynamic model with load transfer, slip angles, a peak-then-slide tire
  curve and a per-axle friction circle, on the ported parameters above;
- barriers are planar inelastic contacts (decision 0052) against the corridor edge;
- the grip aid limits what the stick and pedals ask of the kart (decision 0047).

It is therefore *the same track, the same kart and the same coaching line* — not the same binary.
Feel will differ from the desktop.

## Layout

```
web/
  index.html          the page
  style.css           the HUD and the start overlay
  src/track.js        stations, projection, offsets, banking
  src/physics.js      the kart
  src/scene.js        hall, road, kerbs, barriers, lamps, kart
  src/input.js        Gamepad API + keyboard
  src/main.js         loop, cameras, lap timing, coach, HUD
  data/track.json     generated from the repository's tracks and profiles
  vendor/three.module.js  three.js r160 (MIT), vendored so the page has no external dependency
  tools/make_data.py  regenerates data/track.json from the repo's data
  tools/sim_test.mjs  headless physics checks (node)
  tools/browser_check.html  end-to-end check inside a real browser
```

## Regenerating and checking

```powershell
python web/tools/make_data.py     # rebuild data/track.json from the repo's tracks
node   web/tools/sim_test.mjs     # headless physics: geometry, lap, braking, steady-state cornering
```

To try it locally, serve the folder (ES modules need a real origin):

```powershell
cd web; python -m http.server 8123
# then open http://127.0.0.1:8123/
```

`web/tools/browser_check.html` loads the data, builds the whole scene, runs 10 s of physics and
renders a frame, printing a JSON summary — open it through the same local server.

## Performance and quality

**This is a racing game, so the default is the cheapest setting, not the prettiest.** The view
starts in the driver's own eyes, and the hall is drawn with no point lights, no shadows, no glow,
cheap Lambert shading instead of PBR, and a half-resolution drawing buffer that the browser scales
up. The roof and walls stay, because without them the ceiling is a black void.

The reason is that a point light costs per-fragment work on every lit pixel. On a machine without
GPU acceleration — a browser with hardware acceleration switched off, a blocklisted driver, a VM
or a remote session — the fully lit version took ~250 ms per frame at 1080p (455 ms on a HiDPI
display), which is what makes a browser declare the page unresponsive.

| level | point lights | shadows | glow | shading | pixel ratio |
| --- | --- | --- | --- | --- | --- |
| `racing` **(default)** | 0 | no | no | Lambert | 0.5 |
| `lite` | 0 | no | no | Lambert | 1 |
| `medium` | 2 | no | no | PBR | 1 |
| `high` | 6 | yes | yes | PBR | 1.5 |

Raise it with `?quality=lite|medium|high`, e.g. `?quality=high`. The page also samples real frame
times and steps down a level when frames stay slower than 45 ms, and the HUD shows the active
level and measured fps under `QUALITY`.

Measured at 1920×1080 on **software rendering** (SwiftShader — the worst case, no GPU at all;
these numbers move a few ms between runs):

| level | ms/frame | fps |
| --- | --- | --- |
| `racing` | 14 | **74** |
| `lite` | 27 | 37 |
| `medium` | 43 | 23 |
| `high` | 198 | 5 |

The fully lit version used to be the only option and ran at 4 fps. On a working GPU, `high` runs
at well over 100 fps.

Two things are worth knowing about how the view is built, because both were bugs once:

- **The roof and rafters are always drawn.** Dropping them made the ceiling a black void with the
  lamps floating in it. A ceiling lit only from below renders almost black — we see the unlit
  underside — so the roof and walls also carry a little emissive.
- **The driver's helmet, visor and body are hidden in the cockpit view.** They sit centimetres
  from the camera and otherwise fill the bottom of the frame as unshaded blobs.

The front tyres are deliberately *not* in view: from a seated driver's eye height they are below
and ahead of you. The steering wheel, the nose and the track are.

If the HUD says `racing` and you expected more, that is the default, not a failure — pass
`?quality=high`. If it says the browser is drawing without GPU acceleration, turning hardware
acceleration back on in the browser settings is the real fix.

## Deployment

`.github/workflows/pages.yml` publishes this folder to GitHub Pages on every push to `main` that
touches `web/`.

Pages must be turned on once by hand: **Settings → Pages → Build and deployment → Source =
GitHub Actions**. The workflow cannot do it for you — creating a Pages site needs repository-admin
rights, which a workflow's `GITHUB_TOKEN` does not have. After that first setting, every push
deploys on its own.

## Licence

three.js is MIT (see `web/vendor/three.module.js`). Everything else follows the repository.

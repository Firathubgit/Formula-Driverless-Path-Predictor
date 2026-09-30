# Visual reference inspection

Reviewed 2026-09-13. This document records direct observations of the supplied Mustang clip and separates them from proposed product choices. It does not claim that Ford's implementation uses Blender, prerecorded MP4 transitions, or any particular rendering engine.

## Source and method

Source: `C:/Users/Firat/Downloads/All Drive Modes -- 2024 Ford Mustang Dark Horse.mp4`.

FFmpeg reports a **19.35-second**, **720 x 1280**, **30 fps** portrait H.264 video with stereo AAC audio. The clip is a camera recording of an instrument display, with reflections, perspective distortion and visible steering wheel; it is not an exported UI asset. Source ownership/licensing and redistribution permission have not been established, so retain it as a local reference rather than shipping it as a project asset.

Inspection used whole-clip samples at approximately 0.5-second intervals, plus frame-indexed samples every three frames (0.1 seconds) through the Normal-to-Sport and Sport-to-Track transitions. Those detailed timestamps are frame number / 30. Scratch frames and contact sheets are in `tmp/references/`. Timing estimates refer to visible video content, not measured button-to-display latency. Audio was not reviewed.

## Observed visual behavior

- From approximately 3 seconds onward, one Mustang model occupies the center of the display. The left side identifies drive mode; speed and other driving information remain peripheral. The background is dark, with a subtly lit floor or road underneath the car.
- Mode changes coordinate car color, viewpoint, environment details and warm orange streaks. The car remains spatially central. The visible model is the same Mustang; the clip does not demonstrate switching between several vehicle geometries.
- Normal presents a pale car from a front three-quarter view. Sport presents a blue car from a nearly frontal view. Track turns toward a red car's rear three-quarter view with a curb. Drag Strip presents a pale car from the rear with starting lights. Slippery presents a pale car from a rear/side angle with weather-like surface effects. Custom returns to a darker car from a front/side angle.
- During transitions the surrounding mode list scrolls and the orange streaks sweep across the vehicle. Stable driving information remains visible. The clip shows coherent transitions between poses, but cannot establish seamless looping or the interpolation curve used internally.

## Measured transition windows

| Change | Evidence from 0.1-second samples | Approximate visible duration |
| --- | --- | --- |
| Normal to Sport | No clear transition streak at 5.1 s; streaks visible at 5.2 s; Sport label by 5.5 s; blue frontal pose established by approximately 5.9 s; prominent streaks gone by 6.4 s. | About 1.2 s from first visible effect to settled appearance; boundaries uncertain by roughly one sample. |
| Sport to Track | No clear transition streak at 7.7 s; streak visible at 7.8 s; Track label by 8.1 s; car rotates through side view around 8.4-8.5 s; rear three-quarter pose by about 8.7 s; prominent streaks gone by 9.2 s. | About 1.4 s from first visible effect to settled appearance; boundaries uncertain by roughly one sample. |

The transition has a short lead-in, a more substantial pose/color change, and a fading effect tail. These observations provide a pacing reference. They do not imply exact engine-side frame times, input response time, or a requirement to copy every effect.

## Translation into project requirements

The following are proposed implementation requirements drawn from the user's brief and this inspection:

1. Give the showroom a dark, restrained environment, one central vehicle, and an immediately readable selected appearance. Keep appearance selection separate from the active physics profile.
2. Begin with two original or properly licensed car appearances. A later carousel can expand to two through five; the reference itself shows modes of one car.
3. Use approximately 1.2-1.4 seconds as an initial full showroom-transition pacing target, subject to testing in the actual interface. Make the selection respond immediately even while motion finishes, and provide a reduced-motion path.
4. For a future Blender video graph, define each stable state and transition edge explicitly. Match camera pose, field of view, exposure, background, color management, car transform and final/initial frames at each join. Each stable state needs an idle loop or a still frame. Match motion at joins too, not merely image pixels.
5. Resolve rapid repeated selection deterministically. Queue the latest requested destination or transition through a stable state; avoid depending on arbitrary MP4 seeks. Preload the next clip and provide a still fallback while it loads.
6. Keep the arbitrary-track driving view live. Its bird's-eye camera, position, speed and trajectory need to respond to the simulation; a prerecorded showroom animation cannot represent that changing state.
7. Preserve the brief's distinct driving semantics: green means accelerating, yellow means near-constant speed, red means braking, derived from planned longitudinal acceleration with an explicit threshold. Orange showroom decoration has no driving-policy meaning.

## Screenshot observations and implemented translation

The root reviewer also inspected all five supplied PNGs. Images 1-2 show the near-black driving
composition with prominent speed, the ego vehicle below center, and road geometry extending
forward. Image 3 supplies a light-theme contrast reference. Image 4 shows the LiU driverless
page with a blueprint tree and odometry plots consistent with Rerun; image 5 shows its Formula
Student homepage. Neither screenshot establishes present internal team interfaces or membership.

The first Qt milestone translates that composition to an original dark theme, a 45° downward
follow camera, a separate overview, left-side speed and right-side causal plan data. World state
uses the rear axle reference; the procedural meshes face local -z and retain the shared 2.6 m
wheelbase. The actual inspected capture is `docs/assets/braking.png`. Compact 1100×700 controls
and text were also inspected in application screenshots. Color meaning comes from the core's
planned acceleration deadband, not the reference screenshots' blue path.

No clip or Blender export has been created by this milestone. The showroom animation graph and
final-quality car assets remain later work; the current vehicle appearances are original
procedural development geometry.

## Driver's display references (2026-09-27)

The user supplied four Tesla FSD visualizations (a blue lane-change ribbon behind a car, the dark cluster with speed,
speed-limit sign and a blue lane ribbon, the in-car dark view with map, tire pressures and trip cards, and the light
theme with a ride request) and asked for the simulator to follow them. Adopted in decision 0035: the full-window scene
with a low chase camera and a road fading into the dark, speed top left with the target as a sign, the assistance named
top right, a map card, one status toast, and cards and menus instead of permanent panels; the four-wheel car's tires are
shown around a car seen from above, as the tire-pressure card shows them. Not adopted: the blue path. The ribbon keeps
the core's planned-acceleration colours, as the earlier translation above did, and only takes the references' width,
softness and fade.

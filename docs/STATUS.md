# Status

2026-09-28 recorded Formula driving audio (decision 0045), correcting the oscillator engine the user rejected.
The engine now plays Speed Dreams' Formula V10 WAV, with phase-continuous pitch, smooth loop joins and filtered
multirate playback; its recorded tire-skid sample replaces the tire oscillator. Seven virtual gears add short
ignition cuts, RPM drops, downshift blips and a limiter. Lift-off blends a quieter filtered exhaust colour.
The pinned, checksum-verified source WAVs and licenses are embedded for offline playback, with a new explicit
`scripts/bootstrap-audio.ps1` step for fresh desktop builds. ExhaustNote's MIT sample-player approach was adapted;
credits and limitations are documented. One V10 recording is used, not separately recorded RPM/load banks.
The existing lap/best bells, sound switches, five presets and vehicle/timer contracts are preserved.
Evidence: Release desktop and audio targets built. All 38 audio checks passed, including asset decoding/rejection,
alias suppression, seven gears, phase continuity, 22.05/44.1/48/96/192-kHz output, and the existing event/persistence
lifecycle. The real default device streamed 4.12 s with NoError and a 19,200-byte buffer. All three targeted CTest
suites passed in 168.97 s; the desktop had 787 checks and no QML warnings. Updated Sound panel capture inspected.
The 18 s recorded-sample demo has PCM16 peak 0.656433 and RMS 0.198548, no full-scale samples. Physical controller
driving, subjective listening and device hotplug were not exercised; no full core regression for this presentation
change. Details: `docs/evidence/recorded-driving-audio.json`.

2026-09-28 Formula-style driving audio (decision 0044), at the user's request. Driver: You now has an original
V10-inspired synthesized voice responding to actual speed and pedal load, including on the kart. Six virtual gears
add upshift cuts/RPM drops and downshift blips, with quiet road/wind and slip-based tire sound. The existing kart
practice clock triggers one lap bell or a rising new-session-best bell; start/pause and successful preset operations
have small cues. Settings → Sound offers remembered volume, master mute, engine/tire and cue switches, and a bell
preview. Sounds run on a separate desktop audio thread; vehicle physics, timing and the five handling presets are
untouched. The UI labels synthesized sound and sound-only gears. Research and design choices are in decision 0044.
Evidence: Release desktop/audio-test targets built. 28 offline audio checks passed (six gears, RPM drop, downshifts,
load/coast contrast, slip gating, single lap/best events, pause/mute/mode exclusion, finite bounded output, DC,
buffer-chunk continuity and local settings persistence). The generated 18 s WAV has peak 0.436750, RMS 0.150962,
mean -0.000120471 at test master 100%. The actual default audio device processed 4.08 s with NoError and a 19,200-byte
buffer; its preferred PCM format resolved the initial unsupported-16-bit probe. `fd_race_audio`,
`fd_assistance_presets` and `fd_desktop_controls` all passed in 182.53 s; the desktop check had 787 checks, no QML
warnings, real sound controls and unchanged vehicle pose/time/revision while operating them. Sound-panel capture
inspected. Physical Xbox driving, device hotplug and subjective listening were not exercised; full core regression
was not rerun for this presentation-only change. Details: `docs/evidence/driving-audio.json`.

2026-09-28 five local Forgiveness preset slots (decision 0043), at the user's request. Pick 1–5 and Save/Replace,
Load or Delete in Settings. Presets retain Overall/Manual mode, Overall level and all manual controls across app
restarts. Loading enables Forgiveness and applies through the existing next-tick setter without resetting the kart
or lap timer. One small versioned JSON file in AppLocalDataLocation, QSaveFile atomic writes, validated values,
explicit storage errors; no database service. The screenshot's Manual 34/14/40/100/79 setup was saved once into
the previously nonexistent personal slot 1; four slots remain empty. UI tests use an isolated temporary file.
Evidence: Release desktop and new preset-test target built. All 25 storage checks passed, covering five-slot bounds,
complete reload persistence, replacement/deletion isolation, invalid values, malformed-file preservation and storage
failure. `fd_desktop_controls` passed 773 checks in 151.82 s with no QML warnings, including actual Save/Load/Delete
buttons, slot five, complete manual restoration and unchanged pose/time until the application tick. The preset-panel
capture was inspected. No core physics changed; full regression and physical Xbox driving were not rerun.
Details: `docs/evidence/kart-presets.json`.

2026-09-28 manual Forgiveness controls (decision 0042), at the user's request. Overall retains its existing 1–100
curve; Manual adds independent 1–100 acceleration, braking, joystick steering, grip assistance and top-speed sliders.
Higher means more; top speed also shows km/h. The same vehicle seam applies the requests, with validated changes
committed together at fixed ticks and individually named revision/time events. Manual remains active at grip 1 so
its pedals and steering still work. Both modes retain their values when switched; Off restores ordinary physics.
Changes preserve the driving session and lap timer. The stored physical profile is untouched.
Evidence: Release desktop and human-driving targets built. All 13 human-driving groups passed, including actual-motion
effects for every new slider, independence, off restoration, validation, tick events and reset retention; the original
Overall measurements are unchanged. `fd_desktop_controls` passed 763 checks in 185.56 s, no QML warnings. The UI test
operated all five sliders as acceleration 100, braking 10, steering 20, grip 100 and speed 70, checked their independent
application, rejected invalid input and switched modes without losing settings. The manual-panel capture was
inspected. An initial executable-link failure cleared on retry without closing a process. Only header comments and
documentation changed after this successful build. Full regression and physical Xbox driving were not rerun.
Details: `docs/evidence/kart-manual-forgiveness.json`.

2026-09-28 personal kart lap timing (decision 0041), at the user's request. You → Gokart now has a white current-lap
clock at the top right and smaller Previous line underneath; the circuit card shows session best and lap number.
First throttle above 5% starts timing, pause freezes it, and only a full circuit through twelve ordered gates updates
Previous. Crossing time is interpolated within the fixed tick. Restart returns to the existing poster start pose and
retains completed times; leaving kart mode clears the session. A checkered ground marking identifies the start line.
The passive practice timer reads true poses, never drives, and is independent of the unchanged competition judge;
practice times are unpenalized and not persisted. Track geometry and vehicle physics are unchanged.
Evidence: Release desktop and timing targets built; `fd_competition` passed 8 groups, `fd_track_conditioning_tests`
passed 13 groups (including two continuous laps of the actual kart corridor at its centre and ±2 m), and
`fd_desktop_controls` passed 750 checks in 130.95 s with no QML warnings. Checks cover gas/idle start, full versus
partial laps, reverse/outside/out-of-order gates, previous/best history, reset, spawn/marker, clock visibility and pause.
The clock and start-settings captures were inspected. A final local-variable rename removed the start marker's
shadowing warning and the desktop rebuilt successfully. Full regression and physical Xbox driving were not rerun
for this change; desktop checks inject controller inputs. Evidence: `docs/evidence/kart-practice-laps.json`.

2026-09-28 opt-in Forgiveness for You → Gokart (decision 0040), at the user's request. Settings now offers an
independent toggle and 1–100 slider: off/1 preserves the original command and plant exactly, 50 is already strongly
assisted, and 100 gives deliberately no-slip arcade handling, stronger brakes and a 35 km/h target. Intermediate
levels smoothly reduce assistance toward the original kart. The same four-wheel integrator advances the actual pose;
no path snapping or second plant. Changes validate and commit at fixed tick boundaries with revision/time events,
without resetting the drive. Off retains the slider value, reset retains assistance, and leaving kart/handing back
removes the override. Profiles stay intact; assisted driving is labelled, ordinary tire/envelope cards explain the
override instead, and setup controls are unavailable while enabled. Human drives remain unrecordable.
Evidence: `scripts/build.ps1 -Desktop` passed all 33 suites in 551.96 s, including 12 human-driving groups and 744 UI
checks with no QML warnings. New checks measure progressively lower speed, stopping distance and brake/turn sideslip
at 1/50/70/80/90/100, exact restoration when off/at 1, left/right response, timestep convergence, validation and
tick/revision/reset/handback boundaries. From 9 m/s, stopping distance falls from 5.59351 m at 1 to 2.25 m at 100;
the tested brake/turn peak sideslip falls from 14.7909° to 0.732298° at 50 and zero at 100. The actual QML switch and
slider were operated while driving. A final desktop rebuild and UI rerun after the controller-note correction and
settled-switch capture passed 744 checks in 130.37 s, no QML warnings; its settings capture was inspected.
Details and the measured level table are in decision 0040 and
`docs/evidence/kart-forgiveness.json`. Physical Xbox driving was not exercised; the tests inject controller inputs.

2026-09-28 poster coaching for Gokart mode (decision 0039), at the user's request. A bundled trace of their poster's
recommended line is painted along the whole existing track: green gas, blue coast, red brake, with six braking zones.
The 1600-sample, 0.4 m-wide closed ribbon stays inside the conditioned corridor; the poster's line is inset by at most
0.6723 m where its variable-width asphalt differs from the installed 6 m corridor. Settings under Gokart and Layers
independently switch coaching, prediction and the white motion arrow, including while driving. Coaching defaults on;
prediction, motion arrow and lattice default off in kart mode. Ordinary-track preferences stay separate. This is fixed
poster advice, not adjusted to the driver's speed. The track, physics, controls, planner and judge are unchanged.
Evidence: the Release desktop target built without compiler warnings; `ctest --preset windows-desktop -R
'^fd_desktop_controls$' --output-on-failure` passed in 170.99 s, 728 checks, no QML warnings. The checks cover all four
coaching/prediction combinations, motion-arrow visibility at speed, switches while driving, unchanged state while
toggling paused, all 9,600 ribbon vertices inside the corridor, a closed seam and all three colours, and mode exit.
Settings, overview and driving captures were inspected; an independent `scripts/run.ps1 --kart --capture
out/tracks/kart-guide/startup.png` exited 0. Re-running the generator reproduces the bundled CSV byte-for-byte.
Details: `docs/evidence/kart-coaching.json`. The earlier 725-check run also passed; its captures revealed clipped labels
and the separate motion arrow, corrected before the final run. Core suites and physical Xbox driving were not rerun
for this display-only change; the UI checks inject controller inputs.

2026-09-27 a person drives the car with an Xbox controller (decision 0037), at the user's request: accelerator, brake
and steering, no speed cap, a controllable car as in a racing game. `Simulation::set_human_driving` hands the car over
and back at a tick boundary; each control decision then takes the person's controls, sampled every 20 ms of simulation
time, through `human_command` (core): the pedals ask for their share of grip_mu g at the tires' load (downforce included),
the plant gives what it can, and steering is the stick's share, shaped to the power 1.5, of a speed-sensitive limit (the
lock at walking pace, then the geometric turn at the grip, plus the front tires' peak slip angle). The planner still
decides, shown as a guide and not driven; MPCC is not asked; each decision says `human` drove it; the same judge judges.
`RecordingWriter` refuses a person's drive at its start and mid-run. The desktop reads the first XInput controller
(`apps/desktop/gamepad.*`, XInput is part of Windows): left stick, right trigger, left trigger; the Menu button starts
and pauses, and the accelerator at the line starts, on the driving screen only. Settings has DRIVER: Autonomous or You;
while a person drives, "You drive" at the top right and a brake, steering and accelerator bar replace the plan's sign and
the WHY card. The user's controller was seen by XInput in slot 0 on this host. Evidence: `scripts/build.ps1` 29 of 29
suites (new `fd_human_driving`, 9 groups), 421.8 s of tests; `scripts/build.ps1 -Desktop` 33 of 33 (also new
`fd_gamepad`, 9 checks), 557.1 s of tests; no compiler warning in either; the UI suite 695 checks (18 new: taking the car,
the display, refused controls, driving, braking, steering, start by the accelerator, pause by Menu, handing back), no
QML warning. Not done: driving with the physical controller was not exercised by the checks (it is off during them) nor
by hand here; the kinematic car cannot slide, so braking hard in a turn only reports that its grip demand exceeds its
validity; no keyboard driving.

2026-09-27 the showroom's cars on the track, frontend only (decision 0036). At the user's request the four Blender cars
drive in the live scene instead of the silhouettes. `tools/showroom/build_live_cars.py` exports them from the prepared
showroom cars (Blender 4.5 glTF: wheels separated by circle fits of their ground-touching tread and exported about their
own centres, materials flattened to what glTF carries, textures at most 4096/2048 px, a contact shadow, the studio
panorama) and converts every part with Qt's balsam, about 49 s for all four, into `artifacts/live-cars/`, now ignored by
Git. `LiveCars` reads its index once and leaves any appearance whose car is incomplete, malformed or outside that
directory to its procedural car; `--live-cars DIR` reads another. The car stands at the plant's pose at its real size,
its front axle at its own wheelbase rather than the plant's; the front wheels steer by the plant's angle, every wheel
turns by speed × (1 + its slip ratio) ÷ its radius while the run advances, live or replayed, at most 0.7 rad a frame, and
a locked wheel glows red. A second view beneath the scene draws the car mirrored in the road through the same camera, lit
by the mirrored lights and the studio turned over, fogged with depth and blurred; the road is seen through by 0.32 and
composites to its former colour over the bare backdrop, and unseen shoulders 4 m beyond its edges keep the reflection on
the road. The layers panel's Reflections switch hides it. `fd_desktop --appearance N` draws a car for captures. No
simulation, planning, control, recording or judging code changed; `track_geometry.cpp` gained only the shoulders.
Evidence: `scripts/build.ps1 -Desktop` 31 of 31 suites (30 before, plus `fd_live_cars`), 544 s of tests, no compiler
warning in what it rebuilt; `fd_live_cars` 21 checks; the UI suite 677 checks (654 before; the new ones check the
exported car, its wheels' places, its shadow, the reflection and its switch, and the wheels still, turning and stopping)
with no QML warning; showroom playback 11,746 checks in 148.8 s, no QML warning; in-app captures of all four cars, of a
rear lock and of the procedural fallback inspected. `Main.qml`'s line endings were restored to LF after these runs.
Not done: Qt's image-based diffuse light is a blurred specular level, so the reflected underside is lighter than a real
one; only the car is reflected; the drawn car is not scaled to the plant's wheelbase; the core-only build was not rerun,
the core being unchanged.

2026-09-27 driver's display and car selection, frontend only (decision 0035). The driving view follows the user's
Tesla FSD references: full-window scene, chase camera 18° down, road and lines faded with distance by two unshaded
shaders, the prediction ribbon a soft car-width band in its unchanged plan colours, speed with the plan's target as a
sign, a status line, the WHY THIS SPEED card, and every control in a settings drawer, telemetry column and layers panel
opened from a dock. No simulation, planning, control or recording code changed; `track_geometry.cpp` changed only how
the ribbon and kerbs are drawn. The car selection (user's requests of 2026-09-26/27): each car named by its brand logo,
local files from the user's Downloads in `artifacts/showroom/media/logos/` (not in the repository), top left and between
‹ › arrows that browse AMR23, RB19, Jesko, Urus; a transparent chamfered Select car button whose single line takes the
car's gradient and runs one lap on hover; the header and bottom texts removed. Evidence: `scripts/build.ps1 -Desktop`
30 of 30 suites in 761 s with no compiler warning; the UI suite 654 checks (625 before; new ones open every panel from
the dock and hold the new layout in the compact window) with no QML warning; actual showroom playback 11,692 checks in
142.7 s, all 12 directed switches driven by the arrows and all 16 clips decoded, no QML warning. Two earlier playback
runs stopped at a premature handoff; the user confirmed clicking the test window. A third found the new logo check
running after the handoff, when the view is correctly hidden; the check now runs while the showroom is on screen.
Not done: the README's feature screenshots still show the earlier layout (the opening capture is new); the core-only
build was not rerun, the core being unchanged.

2026-09-26 22:07 1080p showroom pack installed at the user's request. AMR23 and RB19 use
the 1080p renders below; Jesko and Urus are their 720p frames Lanczos-upscaled to 1920x1080
(not re-rendered). Packaged from `revision-1080p-linkedin/app-frames` with CRF 16/slow:
source and encoded seam checks passed; staged Qt playback passed 8,127 checks in 141.2 s.
Previous media backed up at `artifacts/showroom/previous-default-20260926-220726/`; all
installed hashes match the manifest. Playback from the installed location first ended
early without evidence (exit 7, cause not found), then passed 8,119 checks on rerun.

2026-09-26 1080p LinkedIn renders of AMR23 and RB19, at the user's request, instead of
the stopped 4K pack. `rerender_existing.py` from the approved `revision-4k/source-studio`
snapshots, 1920x1080, 128 Eevee samples, otherwise unchanged: 228 frames each, about 18 s
(AMR23) and 22 s (RB19) per frame. All 456 frames decode at 1920x1080. Encoded with H.264
CRF 16/slow, yuv420p, fast-start, and fully decoded: `amr23-1080p.mp4` and `rb19-1080p.mp4`
(enter, idle, select; 192 frames, 8.0 s each) and `amr23-to-rb19-1080p.mp4` (AMR23 enter to
exit, then RB19; 348 frames, 14.5 s), in `artifacts/showroom/revision-1080p-linkedin/linkedin/`.
Not installed in the app; the active pack is still 1280x720. The Blender traceback at each
render's exit is the user's BlenderMCP add-on failing to unregister after the last frame.

2026-09-26 ~19:01 4K render stopped at the user's request (worker 16552 and Blender
11756 terminated). 51 of 912 frames exist; nothing was packaged or installed and the
active pack is still 1280x720. `job-status.json` is stale. Rerunning
`render_4k_pack.py` resumes from frame 52.

2026-09-26 18:52 4K render resumed by Claude Code, not yet complete or installed.
Before restarting: no render process was running; all four `source-studio` snapshots
hash identical to the approved `studio/` files, so frame 12 (the 1 GB shadow-pool test,
reused by the job) matches its source; FFmpeg with libx264 resolves; and actual Qt
playback of the active 720p pack with the current desktop build passed 8,047 checks in
139.1 s (scratchpad run, not a 4K result). The interrupted logs and status files are
kept in `revision-4k/render-logs-interrupted-20260926-0233/`. Worker PID 16552 reused
frames 1-49 and rendered 50 (209.2 s) and 51 (193.3 s); both decode at 3840x2160 and
the new log has no shadow-buffer overflow. 51 of 912 frames exist. Progress and outcome
remain in `revision-4k/job-status.json` and `render-progress.json`.

2026-09-26 18:41 showroom handoff check: the 4K render is incomplete and no
Blender/FFmpeg process or original worker PID is running. AMR23 frames 1-49
fully decode at 3840x2160; the log stops during frame 50, with no recorded
worker error. The cause of interruption is unknown. `revision-4k/job-status.json`
is stale; it must not be interpreted as a live job. The active pack is still
1280x720, and all 29 movie hashes match its manifest. No new native playback
test ran, no 4K pack was installed, and the job was not restarted during the
handoff requested for Claude Code. Preserve logs and resume the existing worker.

2026-09-26. The theoretical best lap (decision 0034, TrackWayFastPlan Phase 4 and its Phase 8 row), after the user
approved both downloads. `fd_headless --write-lap-problem` writes a car's minimum-time problem; fastest-lap 0.5, run
offline beside the build, solves it (`tools/optimal_lap/`); `--check-optimal-lap` reads the result as one contract; the
desktop, started with `--profile ID --optimal-lap DIR`, draws the optimum's car translucent beside the live one with the
lap's sector deltas, each tire's dissipated energy and the setup sensitivities, only while its problem is the car and
corridor on screen. The Formula One style car's optimum on the preset is 14.143 s ("Optimal Solution Found", 500
points, tolerance 1e-8); its controller's best flying lap is 19.160 s on the full envelope and 15.115 s under MPCC.
TUM's independent formulation found an error in decision 0033's TUM car, now corrected, and showed that fastest-lap
floors peak friction at 1.0, now refused unless stated; solved at that floor TUM laps within 1.0% of fastest-lap.
Builds (source fingerprint `acf7dab6`): `scripts/build.ps1` 28 of 28 suites in 688.3 s, `scripts/build.ps1 -Desktop` 30
of 30 in 723.9 s, the UI suite 615 checks with no QML warning, no compiler warning. Of the 429 recordings in `out/runs`,
397 validate with this build; the 32 refused are two runs cut off while written, one written before schema 13's dynamic
car metadata settled, 28 layout runs from before the cone memory entered the metadata (the final `p75h-*` runs pass),
and `p8-profile-electric-race-car`, which names TUM's car with the tire decision 0034 corrected. Evidence: `docs/evidence/optimal-lap.json`.

2026-09-25. Phases 7.2 to 7.5 and §8 (decisions 0030 to 0033): simulated cone perception of the course's cones after
PacSim (schema 15); driving on the path the car believes from its own detections and readings, through a port of
FaSTTUBe's cone path planner that reproduces the published package, cross-checked by a Delaunay planner written from
FS-FEUP's description, every recording checked by a driver run again (schemas 16 and 17); every run judged after
PacSim's competition logic, and PacSim's Formula Student layouts driven from where they lie (all 13 layouts, 39 runs,
every recording validated); and physics profiles for the four-wheel car from named sources, every value's provenance
marked. The measured results are in each decision and in `docs/evidence/perception.json`, `cone-driving.json`,
`competition.json` and `vehicle-profiles.json`; the showroom and its car selection were left untouched, as the user
asked.

2026-09-26 4K showroom rerender started, not yet activated. The resumable job is
`tools/showroom/render_4k_pack.py`; current progress and outcome are in
`artifacts/showroom/revision-4k/job-status.json`, `render-progress.json` and
`render-logs/`. Approved studio snapshots preserve camera keys, materials and
lighting. Settings: 3840x2160, 24 fps, 256 Eevee samples, full-resolution ray
tracing, 1 GB shadow pool; intended encoding H.264 CRF 12/slow. The first
512 MB shadow-pool test overflowed; a fresh 1 GB test frame completed without
that warning, was visually inspected, and decoded as 3840x2160. Python tools
passed compilation. The background worker renders all four cars and requires
source/encoded seam validation and actual Qt playback before backing up and
replacing the default media/master. Full rendering, packaging and app checks
are pending; a completion entry is appended only when they succeed.

2026-09-23 Python launch convenience: root `run.py` delegates to the existing
PowerShell launcher and forwards application arguments. Showroom movies remain
confined to car selection; hiding that view unloads its video player before
simulator setup. The Python launcher ran successfully with `--capture-showroom`
and decoded the default entrance/idle clips; evidence:
`artifacts/showroom/python-launch-check.png` (exit 0).

2026-09-23. The car can carry its own instruments (decision 0029, TrackWayFastPlan Phase 7.1): pose, speed, wheel
speeds, an inertial unit and steering, each late by its own dead time and wrong by its own seeded error. Nothing reads
them yet; a run records what they measured beside what was true, and the difference is on screen.

2026-09-23 cinematic showroom videos completed and activated. Each car has two
individual entrance detail shots, its existing hero/idle composition, a parked
switch-away fade, and a reserved selection angle reached by a hard cut before
fading to the paused simulator setup. No studio racing line or car movement
remains. All 912 frames rendered at 1280x720, 24 fps, 32 Eevee samples; 16 state
clips, 12 ordered switch videos and a 32-second overview were encoded and fully
decoded (29 movies). Source hero-boundary max RGB RMSE is 0.561/255; encoded max
is 1.491/255, below the 2.0 limit. All 12 black endpoints decode to exact zero.
Encoding now uses CRF 16 and forced first/last keyframes, and decoded boundary
checks run before publishing a pack. A second packaging run reproduced the
same manifest/hashes. The Blender audit confirms stationary cars, matching hero
poses, explicit camera cuts, distinct selection views and no racing ribbons.
The showroom controller passed 146 checks. Actual Qt playback passed 7,839
checks in 140.099 seconds: all 16 clips, all 12 switches, four paused handoffs,
no QML warnings, and no simulation advance. Evidence is under
`artifacts/showroom/revision-porsche/ui-verification/`; source/encoded reports
are in that revision's `media/`. The verified pack, cars, studios and master
are now also the defaults under `artifacts/showroom/`. Prior media and master
are retained in `artifacts/showroom/previous-default-20260923-224304/`.

2026-09-23 targeted showroom follow-up: softened the horizon in all four GT
studios by broadening its height gradient and increasing the floor/backdrop
shader blend from 0.08 m to 0.65 m across the curved sweep. Urus white front
lighting is now readable: its Y signature is isolated from the housing atlas
and emitted in white, with dark housing and transparent cover retained. The
separate front optical atlas is neutralized and modestly brightened; rear
lamps are unchanged. Four final review angles per car (12/25/44/73) rendered
at 1280x720, 32 samples, and both comparison sheets and the staged Blender
master were refreshed. This follow-up changes only those two visual issues;
no replacement movies were rendered. Earlier endpoint render measurements
below belong to the preceding environment pass, not this refreshed still set.

2026-09-23 environment review, awaiting the user's acceptance of the four looks:
the GT studio now uses a continuous curved backdrop with a low horizon glow.
AMR23 has dark green fading upward to black; RB19 has extremely muted red/blue;
Jesko and Urus have neutral graphite. The foreground floor and car lighting stay
neutral. The third supplied Porsche clip was also sampled for visual comparison.
The first green pass was revised downward after inspecting its height in frame.
Ten frames per car rendered at 1280x720, 32 Eevee samples (40 final stills).
All 12 sampled black endpoints are exactly RGB zero. Idle endpoint RMSE is
0 for both F1 cars, 0.107 for Jesko and 0.172 for Urus on the 0-255 scale,
within the existing media pack's 2.0 limit; road-car images are not bit-identical.
The staged master reopens with four cycloramas/four scenes and 164 packed images.
Review: `artifacts/showroom/revision-porsche/environment-review/four-environments.jpg`;
close-up sheet: `artifacts/showroom/revision-porsche/four-car-review.jpg`.
These are preview checks, not a complete movie-pack validation. Replacement
videos remain pending the user's explicitly requested environment approval.

The latest 2026-09-23 follow-up slightly lightens the Urus cabin window tint;
the red interior is now more visible in six refreshed studio views. This was
saved into the staged four-car master and its transmitted tint verified after
reopening. The lamp geometry/material correction and silver Jesko rims with
dark textured rubber remain in that same project and refreshed review sheet.

2026-09-23 second material review: the user's latest wheel specification is
silver Jesko rims with black rubber, superseding the earlier black-rim request.
The nine rim materials now use silver alloy; tire albedo and specular response
are lower, with fine molded grain layered over the retained sidewall relief.
Urus front lamps were isolated with a material-mask render: an opaque optical
card was covering the entire headlamp. Its source opacity is now restored,
with dark backing, separate dark housing and restrained clear-cover reflections.
Six final review frames for each road car were rendered at 1280×720, 32 samples;
the editable staged master and contact sheet were refreshed. Master reopen
verified two rubber materials, nine silver-rim materials, all three front-lamp
layers assigned to geometry and 164 packed image dependencies with none missing.
Full replacement movies still await the user's requested visual confirmation.

2026-09-23 material correction after user review: Jesko's two tire materials now
use bounded dark rubber albedo, no metal/clearcoat, and retained molded texture
detail; its nine rim materials use neutral black with controlled reflections.
Urus paint now uses a metallic base with fine flake roughness beneath clearcoat.
Both revised assets passed preparation with zero missing used images and zero
materialless faces. Six new frames per road car rendered at 1280×720 and 32
Eevee samples. The staged four-car project was reassembled and reopened with
164 packed images, and the four-car review sheet was refreshed. These are
material/preview corrections; the replacement full videos remain pending the
visual review described below.

2026-09-23 GT-studio revision, awaiting the requested four-car visual
confirmation before replacement videos: both supplied Porsche CGI clips were
reviewed for close shots, cuts, reflective strips and lamp presentation, and
official manufacturer/team pages were checked for the four car identities and
visible design cues. The review and source limits are recorded in
[SHOWROOM_GT_REVIEW.md](SHOWROOM_GT_REVIEW.md). Blender 4.5.3 prepared all four
revision cars with 164 used packed images total, zero missing used images and
zero materialless faces. It rendered six review frames for each car at 960×540,
16 Eevee samples, assembled and reopened an editable four-scene revision project
with 164 packed images and zero missing image dependencies, and
produced `artifacts/showroom/revision-porsche/four-car-review.jpg`. The new
road-car glazing, carbon, lamp and graphite studio shots have been visually
reviewed. No revised full video pack or native playback is claimed yet; the
prior validated pack remains the default.

2026-09-20 showroom: the user's explicit request added the four-car Blender
selection experience ahead of the next sensor milestone for this session. The
black architectural studio, close lighting reveals, original F1 sponsor colors,
black road-car finishes and native selection flow are described in
[SHOWROOM.md](SHOWROOM.md) and decision 0028. Selecting a car reveals the existing
track setup paused; Start advances the selected simulation model.

Executed showroom evidence (local generated reports are under ignored
`artifacts/showroom/`):

- Blender 4.5.3 LTS rendered all 912 final frames at 1280×720, 24 fps and 32
  Eevee samples. Both final road render processes exited 0. All four sequences
  have 228 frames: enter 72, idle 48, exit 36 and select 72.
- Packaging exported 16 base clips, all 12 directed switch videos, four posters
  and a 32-second overview. All 29 H.264 videos decoded to the expected frame
  counts. All 12 canonical black frames are exactly RGB 0. Maximum shared hero
  pose RMSE is 0.2179/255 (limit 2.0); both F1 sets are pixel-identical at those
  endpoints. Evidence: `media/seam_validation.json` and `media/manifest.json`.
- The assembled `Black_Showroom.blend` was reopened and audited: four scenes,
  one car and camera per scene, all 164 images packed, no missing used texture
  dependencies or materialless faces, and matching evaluated animation
  endpoints. Evidence: `master-audit.json` and `cars/final-material-audit.json`.
- The desktop target rebuilt and all 146 showroom controller checks passed.
  A preliminary Qt Multimedia run used simple local H.264 fixtures and passed
  all 12 switches, all 16 clip roles, current-car titles and paused handoff with
  zero QML warnings. This fixture run is separate from final-render playback.
- Final-render playback passed in the native Qt application: 7,911 checks over
  all 12 directed car switches and all four selection handoffs, with all 16 clip
  roles decoded, zero QML warnings, and the simulation still at time zero in
  track setup. The application captured each selected car and track setup.
  Evidence: `ui-verification/verification.json` and its captured PNGs.

2026-09-15. The rolling forward prediction feeds an actual choice: the car evaluates local
lines, takes the fastest clear one, and brakes when none is clear. Recordings now keep every
one of those decisions, so a replayed run shows the choice, its rejected lines and its
prediction exactly as the car made them. Work follows [TrackWayFastPlan.md](TrackWayFastPlan.md):
Phase 0.1 (exact fast projection), Phase 0.3 (recording schema 2), Phase 1.1 (tire force
model) and Phase 1.2 (dynamic single-track plant, with the vehicle seam of Phases 0.2/0.4) are
implemented. The dynamic plant is now fast enough to plan with, recorded, and selectable in both
applications (decision 0009). Phase 1.3 is implemented too: MAP steering from the plant's own
steady-state table, recorded as schema 4 and selectable in both applications (decision 0010), and
Phase 1's tire slip and balance visuals (decision 0011). Phase 2.1 is implemented: a car with four
rotating wheels that can lock and spin (decision 0012), recorded as schema 5 and selectable in both
applications with each wheel's state shown (decision 0013). Phase 2.2 is implemented: that car's load
moves between its wheels quasi-statically with centre of gravity height and roll balance, recorded as
schema 6, with each wheel's load and friction circle shown (decision 0014). Phase 2.3 is implemented with
the viscous differential Phase 2.1 deferred: drag, downforce and a viscous coupling across each driven axle,
recorded as schema 7, with drag and downforce shown (decision 0015). Phase 2.4 is implemented: the plan's
seven setup parameters are controls in both applications, each proved on a fixed scenario, and AGENTS.md's mass
invariant now covers the dynamic plants (decision 0016). Phase 3.1 is implemented: the four-wheel car's G-G-V
performance envelope is derived from the plant by holding it at speed and sweeping steering, exported in TUM's
`ggv.csv` format with a fingerprint, and drawn in the desktop as a live G-G diagram (decision 0017). Phase 3.2 is
implemented: that car's speed can be planned from 80% of its envelope instead of the grip fractions, in both
applications, recorded as schema 8, with the controller bounded by the car's capacity (decision 0018). Phase 3.3 is
implemented: a closed centreline file, traced or drawn, is conditioned into a smooth track with continuous curvature and
left normals, refused if it crosses itself or folds, and driven and recorded by the headless runner (decision 0019).
Phase 3.4 is implemented, and with it Phase 3: a minimum-curvature racing line solved with OSQP inside the corridor and
the steering limit, driven by the unchanged controller, prediction and planner, recorded with its corridor edges as
schema 9, and offered in both applications. The desktop draws it beside the centreline, with an estimated lap for each
(decision 0020). Phase 5.1 is implemented: an offline lattice of layers, nodes and steerable edges with an offline cost
is laid along whichever line the car drives, written by the headless runner and drawn faintly ahead of the car
(decision 0021). Phase 5.2 is implemented: a lattice planner searches that lattice for the cheapest path past a stated
blockage on either side, returns along it, and otherwise brakes toward the widest gap, replacing the five offsets as the
default in both applications, recorded as schema 10 with each action's path and cost, and shown with each action's line
in its own colour and the chosen action's cost (decision 0022). Phase 5.3 is implemented, and with it Phase 5: every
action the planner compares has its own speed profile along its own path from the car's actual speed, and the car takes
the clear action of least estimated time, recorded as schema 11 and labelled with its time in the desktop
(decision 0023). Phase 6.1 is implemented: model predictive contouring control, in its own target `fd_mpcc` with OSQP,
can drive the chosen action instead of the policy for the cars with tires, its plan the ribbon and its solve time on
screen, recorded as schema 12 with who drove each decision, and each plan's error ahead measured from the recording
(decision 0024). Phase 6.2 is implemented: every plan MPCC drives is recorded with its planned commands as schema 13,
and the plant's response to those commands measures the prediction model apart from its replanning. The measurement
found the four-wheel car's pitch and drag to be what MPCC mispredicted, so its prediction model now carries them, and
that car no longer spins its front wheels pulling away (decision 0025). Phase 6.3 is implemented: MAP steers the
four-wheel car too, from a table of its own steady turns with its wheel speeds and loads solved along with them, so the
plan's baseline and MPCC drive the same car through the same scenarios. MPCC wins the clear laps and the grip change and
loses the car at the corner bollard, where it arrives faster than the planner's own yardstick allows for (decision
0026). Phase 6.4 finishes Phase 6: while such a plan drives the four-wheel car, the tire slip card draws what each
wheel has left of its own friction circle along that plan, measured by driving the plant along it (decision 0027).
Domain terms are collected in `CONTEXT.md`.

## Current behavior

Milestone 0 environment/contracts and Milestone 1 first autonomous lap are complete on the
Windows reference host. Milestone 2's explainable grip, shared color semantics, live control
and recorded changes are implemented. Milestone 3 is partly done: validated recording
ingestion and seekable playback now work in the native application. There is still no
ROS/rosbag2 adapter or public demonstration video. The local four-car Blender showroom
was added on 2026-09-20; its validation evidence is recorded with that update.

The car now generates a three-second prediction from its actual pose, speed and steering
at every simulation control tick and accepted grip change. Simulation applies the first
command; the rest of the prediction uses copied state. The live ribbon follows those
predicted positions, with an actual-state anchor, local speed plot and achieved acceleration
colors. The full circuit remains reference context on the map. Prediction validity reports
anticipated speed, grip and rear-axle track-margin violations. This rolls the existing
Pure Pursuit policy forward. No dependency was added. See decision 0003.

Since decision 0022 the lattice planner makes that decision by default. It follows the reference line while it stays
clear; when the line enters a stated blockage it searches the offline lattice for the cheapest path passing the blockage
on its left and on its right, within the corridor, clear of every blockage and within the car's lateral budget; after the
pass it returns along the lattice; when no pass is clear it brakes toward the widest gap beside the blockage. Since
decision 0023 each of those paths carries its own speed profile, from the car's actual speed within the same limits the
reference plan uses, over the path's own curvature, to a horizon common to the decision; the car drives that profile and
takes the clear action of least estimated time, keeping its previous choice while that is within 0.05 s of the fastest;
with the blockage behind it and no way home clear, it keeps the path it is on rather than turning back to the reference.
Without a blockage in the way it still follows the reference plan with one prediction. The five-offset planner described
next stays selectable for comparison.

MAP steers any car with tires from a table generated for that car (decisions 0010 and 0026); the kinematic bicycle,
whose table would be its own geometry, keeps Pure Pursuit. The four-wheel car's table takes about a third of a second to
generate and is regenerated whenever the car or the grip changes, as the dynamic car's is.

Since decision 0024 the planner's choice can be driven by MPCC instead of the policy (`--controller mpcc`, or CONTROL /
MPCC in the desktop). At every decision it optimises three seconds of the car's motion with the plant's own single-track
equations, reduced from the four-wheel car where that is the plant, with that car's pitch and air (decision 0025), to
make the most progress along the chosen action's path inside the corridor that action leaves open, with each axle's slip
angle and friction ellipse as constraints. Each plan that drives states its command at each point, and
`--prediction-error` reports how far the plant under those commands, and the car itself, departed from the plans. For
the four-wheel car the same commands drive the plant along the plan to say what each wheel is using of its friction
circle at every point of it, which the tire slip card draws while that plan drives, live or replayed. Its
plan becomes the chosen prediction and its first command drives, but only while it is solved, inside its own envelope and
the corridor, and clear of every blockage by the planners' own check; a hold for a blockage, and any plan that fails,
is driven by the policy, and the decision records who drove and why. MPCC plans its own speeds, so while it drives the
target speed is its plan's and running faster than the reference plan is not a validity violation. It refuses the
kinematic bicycle, which has no tires to predict with.

That rollout feeds a decision rather than only a picture. Under the five-offset planner, at every control decision
`choose_local_action` evaluates candidate lateral lines, rejects those that leave the model
envelope or enter a stated blocked region, and takes the clear one with the greatest
predicted advance along the reference, preferring the line nearest the reference on ties.
When nothing is clear it holds the reference line under a speed cap derived from the
remaining room, which brakes and, if necessary, stops. A blocked region is scenario ground
truth supplied by `--obstruct` or the in-app scenario buttons; nothing detects it. With no
blockage in range exactly one candidate is evaluated and ordinary driving is unchanged.
The driving view draws the stated blockage and the rejected alternatives beside the chosen
line, and names the reason. This selects among feasible lines; it is not minimum-time
optimisation, and offsets do not re-derive curvature limits. See decision 0004.

New recordings are schema 2 (decision 0007): besides reference plans, motion and
`scenario_obstructions`, they hold every decision that governed recorded motion, each
evaluated line with its reason and its predicted points. Replay shows the decision in force
at the cursor, draws its rejected lines and its prediction ribbon, and labels them as
recorded. Schema 1 recordings still load; their replay hides the ribbon and states that
neither a prediction nor a decision was recorded. Replay never generates a new forecast or
choice.

C++20 pure planner/controller functions and a separate fixed-tick simulation adapter drive
the native Qt Quick 3D scene. The `adapters/recording/` library owns schema 2 as one
contract, shared by the headless runner and the Qt application. Two original procedural
appearances share the physics profile. The project began empty; Git was initialized locally.
There is no commit, remote or publication.

## Verified evidence, the car's own instruments (2026-09-23)

Decision 0029, TrackWayFastPlan Phase 7.1. Executed on the reference host with source fingerprint
`40bab28c858e354f3b9aed81af362549d203f3facb54ca8fac24673d70039a69`.

- **Builds.** `scripts/build.ps1`: 21 of 21 CTest suites pass in 400.3 s, the new `fd_sensors` among them.
  `scripts/build.ps1 -Desktop`: 23 of 23 in 576.0 s, the QML controls suite running 539 checks in 127.9 s with no QML
  warning. Neither build reports a compiler warning.
- **The plan's step holds.** `fd_sensors` (`adapters/sensors/`, on `fd_core` alone) gives pose, speed, wheel speeds, the
  inertial unit and steering each a rate, a dead-time queue and a Gaussian error drawn from its own stream of a recorded
  seed, after PacSim's structure (read on disk, nothing copied or downloaded). The simulation observes them at every
  fixed tick; nothing reads what they measure.
- **Measured.** Over 39 997 samples the errors have the standard deviations the channels state (0.0299 m against 0.03,
  0.0199 rad against 0.02, 0.0199 m/s against 0.02). On a one-lap run of the dynamic car with `--sensors 7` the pose was
  up to 95 ms old, 72.5 ms on average, and out by up to 1.974 m; the speed by up to 0.609 m/s. Nearly all of that is
  age: a 20 Hz channel behind 50 ms is between one and two periods old whenever it is read.
- **Watching does not move the car.** The same lap with and without instruments wrote byte-identical `telemetry.csv` and
  `decisions.csv` (`out/runs/p71-measured-lap`, `out/runs/p71-true-lap`).
- **Recording schema 14.** `metadata.sensors`, `measurements.csv` and the achieved acceleration in `telemetry.csv`; the
  loader runs the instruments again from the seed and requires the same stream, refusing a measurement nudged by a
  millimetre, a changed seed, a sample from the future, a missing row and an acceleration that does not follow the
  speed.
- **On screen.** A SENSORS card beneath the tire card chooses the true state or measured, and shows the pose's age, how
  far out it is, the speed read and the yaw rate's error. Inspected capture: `docs/assets/sensors.png`, 50 ms and 0.96 m
  out at 68 km/h four seconds into a four-wheel run. The choice first sat in the left column, where it pushed the
  decision panel below the run metrics in the compact window; it moved.
- **Mutations:** five run and reverted. Four failed a group at once; the fifth, two channels sharing one stream, passed
  until a check that no two channels' first errors are alike was added.
- **Recordings:** With this build 305 of the 307 recordings in `out/runs` pass `--check-recording` (schemas 1 to 14),
  the two new ones among them. The two refused are the same as in the last three sessions: `mpcc-dynamic-laps-mpcc`, cut
  off while it was written, and the exploratory `p62x-dynamic-1789837332`, written before the dynamic car's schema 13
  metadata was settled.
- Machine-readable evidence is in `docs/evidence/sensors.json`.

## Verified evidence, each tire's margin along the horizon (2026-09-20)

Decision 0027, TrackWayFastPlan Phase 6.4, which completes Phase 6. Executed on the reference host with source
fingerprint `63e663bc93f8bc95fdbbbbbf243bf6158aede23992e725534f29c6bb3bd4291e`.

- **Builds.** `scripts/build.ps1`: 20 of 20 CTest suites pass in 483.2 s. `scripts/build.ps1 -Desktop`: 21 of 21 in
  669.6 s, the QML controls suite running 517 checks in 109.4 s with no QML warning; neither build reports a compiler
  warning. `fd_local_trajectory` has 13 groups.
- **The plan's step holds.** `plan_tire_use()` drives the plant along a plan by that plan's own commands and asks the
  vehicle seam, at each of its points, what each wheel is using of its own friction circle; the margin is one minus that
  use. The four-wheel car is the only model with four tires of its own, and the numbers are the plant's: each wheel's
  use comes from its own speed, load and slips there.
- **On screen.** Under the four wheel tiles the tire slip card gives FRICTION LEFT AHEAD, a percentage and the time it
  falls at, and plots each wheel's use across the three-second horizon against the tires' limit. It appears only while a
  plan that states its commands drives that car, live or replayed, and is hidden under the policy, whose predictions
  state no commands. Inspected capture: `docs/assets/tire-margin.png`, 7% left three seconds ahead, six seconds into an
  MPCC run.
- **Measured.** Braking, the rear tires are asked for more of their circles than the front (0.48 against 0.38), because
  the car brakes with half its torque at each axle while its load moves forward; at the cornering limit all four sit
  between 0.9 and 1.02 of their circles; and along a plan that drives and then brakes, the busier pair changes from
  front to rear.
- **Found.** The acceleration `demand()` is asked for does not enter a wheel's own use, so the command lookup this
  function first carried was inert; a mutation showed it, and it was removed.
- **Recordings:** With this build 303 of the 305 recordings in `out/runs` pass `--check-recording` (schemas 1 to 13).
  The two refused are the same as in the last two sessions: `mpcc-dynamic-laps-mpcc`, cut off while it was written, and
  the exploratory `p62x-dynamic-1789837332`, written before the dynamic car's schema 13 metadata was settled.
- Machine-readable evidence is in `docs/evidence/tire-margin.json`.

## Verified evidence, the head-to-head (2026-09-20)

Decision 0026, TrackWayFastPlan Phase 6.3. Executed on the reference host with source fingerprint
`97e22d0a1fb3c7418aba429bee0bc7eac157be2de21cd40e1280a0715917487c`.

- **Builds.** `scripts/build.ps1`: 20 of 20 CTest suites pass in 479.8 s, `fd_mpcc` taking 181.9 s and `fd_steering_law`
  46.9 s. `scripts/build.ps1 -Desktop`: 21 of 21 in 444.9 s, the QML controls suite running 488 checks in 71.6 s with no
  QML warning; neither build reports a compiler warning. `fd_steering_law` has 8 groups.
- **The plan's step holds.** The steady-turn search carries a car's four wheel speeds and its quasi-static wheel loads,
  so the four-wheel car has a MAP table: 12 rows and 390 turns, about a third of a second to generate. Driven into the
  turns it claims, the car settles within 0.5% of them inside a row. The dynamic car's tables are untouched, to the
  digit.
- **The comparison**, same car, same scenarios, one run at a time: MPCC laps in 26.97 / 25.33 s against the baseline's
  31.95 / 30.70 s (MAP with 80% of the car's envelope) and the policy's 34.05 / 32.34 s; after the grip drop
  28.59 / 27.40 s against 35.11 / 34.69 s, and only MPCC has no invalid sample there. No wheel locked in any
  clear run. The baseline asks the most of one tire, 0.978 of its friction circle against MPCC's 0.958.
  Solving took 18.5 ms at the median on the clear laps and 29.6 ms after the grip change, against the 20 ms period.
- **What it found.** At the corner bollard MPCC arrives at 19.5 m/s, where the baselines arrive at 16.5 and 15.5 m/s and
  pass without stopping. The planner, which rolls the policy forward within the reference plan's grip, finds no clear
  path and holds 15 m short; the policy cannot stop the car, which slides, locks a rear wheel from 8.3 s, leaves the
  track margin at 9.9 s, reaches 44 m off the reference and rejoins 13 s later, finishing in 47.35 s with
  3615 of 9471 samples invalid. The dynamic car survives the same scenario under MPCC.
- **Recordings:** With this build 303 of the 305 recordings in `out/runs` pass `--check-recording` (schemas 1 to 13, 48
  of them schema 13). The two refused are the same as before: `mpcc-dynamic-laps-mpcc`, cut off while it was written two
  sessions ago, and the exploratory `p62x-dynamic-1789837332`, written before the dynamic car's schema 13 metadata was
  settled.
- Machine-readable evidence is in `docs/evidence/head-to-head.json`.

## Verified evidence, measured model mismatch (2026-09-19)

Decision 0025, TrackWayFastPlan Phase 6.2. Executed on the reference host with source fingerprint
`4876ae6ce0d03902e010997689d76883ba2b264b5d6d4dd5b2ba260a4fd89a4c`.

- **Builds.** `scripts/build.ps1`: 20 of 20 CTest suites pass in 392.6 s, `fd_mpcc` taking 165.6 s.
  `scripts/build.ps1 -Desktop`: 21 of 21 in 484.6 s, the QML controls suite with 482 checks and no QML warning; neither
  build reports a compiler warning. `fd_local_trajectory` has 12 groups, `fd_vehicle_model` 20, `fd_recording_playback`
  18, `fd_mpcc` 9.
- **The plan's step holds.** Both are recorded: every plan MPCC drives keeps its planned commands (schema 13,
  `commands.csv`), and the plant's response to them, driven from the recorded state with nothing replanned, gives each
  plan's model error beside its plan error, reported every 0.25 s over the horizon by `--prediction-error`.
- **Found.** With the reduction of decision 0024, the four-wheel car's model error was 0.091 m a second ahead and 1.26 m
  three seconds ahead at the median, 4.86 m at the 95th percentile, against 0.004 and 0.07 m for the dynamic car, whose
  model is its plant. Switching the four-wheel car's effects off one at a time: drag left the plant 0.78 m behind each
  plan in three seconds; load transfer made the tail and, pulling away, unloaded the front wheels until they spun, all
  471 invalid samples; with both off its error was the dynamic car's (exploratory runs `out/runs/p62x-*`).
- **Decided.** The dynamic single-track plant gains longitudinal load transfer, drag and downforce, off by default, and
  the four-wheel car's reduction carries them. Final runs `out/runs/p62-*`, one at a time: the four-wheel car's model
  error is 0.021 m a second ahead and 0.10 m three seconds ahead at the median, 0.46 m at the 95th percentile; it laps
  under MPCC in 26.97 / 25.33 s with 0 invalid samples (27.07 / 25.35 s with 471 before) and no wheel locked. The
  dynamic car's runs are unchanged: 26.66 / 25.26 s under MPCC, 33.80 / 32.13 s under Pure Pursuit.
- **Plan error.** A second ahead the four-wheel car's fell from 0.060 to 0.026 m at the median; three seconds ahead
  it is 0.48 m, the dynamic car's 0.45 m, because there it is mostly replanning.
- **Solving.** Medians 15.5 ms for the dynamic car and 16.7 ms for the four-wheel car, whose tires are now evaluated at
  the load each axle carries; still not real time.
- **Recordings:** With this build 288 of the 290 recordings in `out/runs` pass `--check-recording` (schemas 1 to 13, 33
  of them schema 13). The two refused are `mpcc-dynamic-laps-mpcc`, cut off while it was written in the previous session
  and refused for its missing `events.csv`, and the exploratory `p62x-dynamic-1789837332`, written by this session
  before the dynamic car's schema 13 metadata was settled and refused for lacking `cg_height_m`.
- Machine-readable evidence is in `docs/evidence/model-mismatch.json`.

## Verified evidence, model predictive contouring control (2026-09-19)

Decision 0024, TrackWayFastPlan Phase 6.1 with a first measurement of 6.2 and part of 6.4. Executed on the reference
host with source fingerprint `9968a3ef3572882a58d3db22d7810016e2d98b8af170d81ef78048910fabfa12`.

- **Builds.** `scripts/build.ps1`: 20 of 20 CTest suites pass in 327.3 s, `fd_mpcc` taking 125.9 s.
  `scripts/build.ps1 -Desktop`: 21 of 21 in 418.7 s, the QML controls suite with 482 checks and no QML warning; neither
  build reports a compiler warning. New suite `fd_mpcc` has 9 groups; `fd_vehicle_model` has 19, `fd_recording_playback`
  17.
- **The plan's step holds.** `fd_mpcc`, outside `fd_core`, ports MPCC's model, contouring and lag cost, corridor,
  friction-ellipse and slip-angle constraints and its SQP loop with warm start and failure reset, with front and rear
  friction ellipses, the tire law of Phase 1 through the plant's own single-track equations, progress along the
  planner's chosen path, and OSQP as the solver.
- **Closed loop, headless**, into `out/runs/mpcc3-*`, standing lap / flying lap: the dynamic car 26.66 / 25.26 s under
  MPCC against 33.80 / 32.13 s under Pure Pursuit and 33.78 / 32.10 s under MAP; the soft-front car 26.68 / 25.29 s
  against MAP's 33.85 / 32.18 s; the four-wheel car 27.07 / 25.35 s against 31.95 / 30.70 s on 80% of its envelope. MPCC
  cuts the corners up to 3.7 m from the centreline and uses 0.95 of the dynamic car's grip, none of its samples invalid.
  With the lane blocked it passes on the planner's side and laps in 26.71 s; with the corridor blocked the policy holds
  and the car stops at 61.79 m, as under the policy alone.
- **Where it falls short.** Solving takes 17.5 to 25.8 ms at the median, 37 to 148 ms at the 95th percentile
  and up to 259 ms against a 20 ms period: not real time. The four-wheel car's 471 invalid samples are all
  tires past their peak in its first three seconds, front wheels spinning as its load moves rearward, which the reduced
  model does not predict. At the corner bollard the planner held for 18 decisions and the policy's braking there is
  flagged faster than its revised plan (84 samples); after a grip drop to 0.7 the policy took over 14 decisions
  (56 samples flagged the same way, against the policy's own 86 in the grip transient).
- **Plan error (Phase 6.2), from the recordings:** one second ahead 0.008 m at the median and 0.29 m at the 95th
  percentile for the dynamic car, whose equations MPCC predicts with, against 0.059 and 0.68 m for the four-wheel car;
  three seconds ahead 0.45 and 3.62 m against 0.57 and 3.43 m.
- **Found:** OSQP needed a proximal step penalty, warm-started multipliers and a 1e-3 tolerance to reach about 16 ms
  from 267 ms; the first guess read the policy's prediction by index and ran four times too fast; the top speed first
  followed the car (28 m/s) and then, as a hard bound, made every programme infeasible above it (139 of 141 failures),
  so it is softened; MPCC's advantage is carried by the warm start (33.7 s laps without it); and the kinematic bicycle,
  predicted as a car with tires, asked for 1.70 of its grip (`out/runs/mpcc-kinematic-laps-mpcc`), so MPCC refuses it.
- **Recordings:** schema 12 records the controller and its settings, and on every decision who drove it, MPCC's status,
  steps and restart, and why the policy drove if it did; loading checks each against the others and the plan's shape.
  With this build 255 of the 256 recordings in `out/runs` pass `--check-recording` (schemas 1 to 12, 40 of them schema
  12); the other, `mpcc-dynamic-laps-mpcc` from the first sweep, was cut off while it was written and is refused for its
  missing `events.csv`.
- **Desktop:** CONTROL offers Pursuit, MAP and MPCC for the cars with tires; under MPCC its plan is the ribbon and the
  caption gives the solver's status and this decision's solve time. Inspected capture: `docs/assets/mpcc-corner.png`.
- Machine-readable evidence is in `docs/evidence/mpcc.json`.

## Verified evidence, per-action speed profiles (2026-09-17)

Decision 0023. Executed on the reference host with source fingerprint
`030c34c3cf7248e186a6d35b369ce272d384c1c28e2db628dd1687651ea6915a`.

- **Builds.** `scripts/build.ps1`: 19 of 19 suites in 194.7 s. `scripts/build.ps1 -Desktop`: 20 of 20 in 264.7 s, with
  447 checks of actual QML controls, bindings and 1100x700 captures and no QML warnings. New suite `fd_speed_profile`
  has 10 groups and runs in 0.5 s; `fd_lattice_planning` has 16 and `fd_recording_playback` 16.
- **The plan's step holds.** Every compared path gets curvature-derived limits from its own geometry and forward and
  backward passes from the actual speed, so the planner compares time. Before the first corner the inside path costs
  2.16e6 against the outside's 1.18e6 and is still taken, because its profile drives it 0.078 s faster. On the straight
  the cheaper path is also the faster one and is taken.
- **The estimate is what the car drives.** Across five scenarios the first decision to pass estimated its horizon within
  2% of the time the car then took: 3.406 against 3.410 s, 3.405 against 3.410 s, 3.210 against 3.250 s, 3.341 against
  3.370 s, 3.170 against 3.230 s.
- **Closed loop, headless**, into `out/runs/final2-*`: a clear lap is byte-identical under both planners and to the run
  before speed profiles, and so is the full-width stop at 61.78 m. On every blocked scenario the lattice planner's peak
  grip use is the lower, and its peaks fall in corners the car would take anyway: 0.684 against 0.995 on the lane
  blockage, 0.706 against 0.953 past a bollard, 0.850 against 0.998 at the corner bollard, 0.977 against 1.000 for the
  four-wheel car on 80% of its envelope, and 0.752 against 0.946 for the soft-front dynamic car on MAP. No run has an
  invalid sample.
- **Planning time** for a blocked decision: 1.4 ms at the median for the kinematic car, 95th percentile 2.4 ms, longest
  3.3 ms, against 2.3 ms at the median for the five-offset planner; for the four-wheel car at the corner 10.0 ms, 12.4 ms
  and 21.8 ms against 16.1 ms. The control period is 20 ms. These are wall clock on this host, printed per run, not
  recorded, and not a real-time claim.
- **Found:** a path point a metre ahead of the car is not a turn the car makes, so a profile follows the path as Pure
  Pursuit drives it; near ties swapped sides every tick until a 0.05 s margin kept the previous choice; abandoning a
  pass the moment the reference line came clear once put the four-wheel car outside the corridor for 495 samples at
  0.89 rad of sideslip, so a return now continues the path the car is on; and with grip dropped to 0.45 beside the
  blockage no way home is clear at all, where the planner fell back to rejoining the reference, the sharper turn of the
  two, for 216 decisions, and now keeps the path the car is on.
- **Recordings:** schema 11 adds each option's profile point count and estimated time and `profiles.csv`; loading checks
  that a profile starts at the recorded state and speed, that one decision's profiles share a horizon, that each time is
  its profile's, and that the car took the fastest clear action or kept its previous choice within the margin.
  All 215 recordings in `out/runs`, schemas 1 to 11, load under this build.
- **Desktop:** each compared action is labelled with its estimated time 15 m along its own predicted line, framed when
  chosen, and a label that would cover an earlier one sits above it, since two paths that have barely separated anchor
  their labels in the same place. The decision panel gives the margin in words. Inspected capture:
  `docs/assets/action-times.png`, the corner bollard at 6.8 s with Right 4.28 s above Left 4.16 s.
- Machine-readable evidence is in `docs/evidence/speed-profiles.json`.

## Verified evidence, lattice planner (2026-09-17)

Decision 0022. Executed on the reference host with source fingerprint
`b0c9087f3d373e774b5630a9919e4cc53ddf7ade737fa5eefca427d0a356a742`.

- `scripts/build.ps1 -Desktop`: 19/19 suites passed, 316.24 s of tests, with 427 QML checks, up from 404, and no
  runtime warnings; the UI check took 61.7 s of its 120. `fd_recording_playback` took 49.3 s of its 60, so its limit is
  now 120 s. `scripts/build.ps1`: 18/18 suites passed, 230.16 s of tests. New suite `fd_lattice_planning` has 11
  groups and runs in 2.1 s; `fd_recording_playback` has 14.
- **The plan's tests hold.** The car passes the lane blockage at 3.00 m and returns along the lattice, within 0.267 m of
  the reference from 60 m to 200 m past it, and stops for a full-width blockage at 61.77 m at 0.02 m/s. On that pass its
  peak grip use is 0.577 against the five-offset planner's 0.780 in the test's run. A blocked decision takes 1.15 ms
  against 2.45 ms, against a 20 ms control period (measurement only).
- **Closed loop, headless**, into `out/runs/planner-*`: a clear lap is byte-identical under both planners, and so is the
  full-width stop, at 61.78 m. On the lane blockage lap the lattice car passes at 2.90-3.00 m and peaks at 0.684, in a
  corner, not the pass; the five-offset car passes at 3.07-3.08 m, is still 3.32 m off at 142 m and peaks at 0.995 at
  161 m turning back. Past a bollard the lattice car passes right at 2.40-2.52 m, peaking 0.686 in a corner, against 1.64
  m and 0.953. The four-wheel car on 80% of its envelope and the soft-front dynamic car on MAP pass and return with their
  run's peak no higher than their clear run's (0.973 against 0.977, 0.746 against 0.747); under five offsets they peak at 1.000
  and 0.946. No run has an invalid sample.
- **Found:** paths clear by their own geometry were not clear once pursued, because Pure Pursuit reaches a path's offset
  a pursuit distance late; the quintic edges and a deviation-dominated cost made late, hard turns; without keeping the
  previous path the car alternated between brake and pass and peaked at 0.986; the offline deviation cost saturated at
  1 m, so a return stayed 2.5 m off, and now saturates at 4 m (decision 0021 amended).
- **Recordings:** schema 10 records the planner, each option's action, path point count and cost terms, and `paths.csv`;
  loading checks actions against the planner, passes against their paths and blockages, brakes against holds, and every
  path against the recorded state. All 149 recordings in `out/runs`, 135 of schemas 1 to 9, load and validate.
- **Desktop:** PLANNER beside the scenario controls; the decision panel names the chosen action in its colour with its
  crossing, rejected count and cost terms; other actions' lines in their colours with a legend; replay names the recorded
  planner. The compact window's decision panel, planner row and replay legend are measured by UI checks. Inspected
  capture: `docs/assets/lattice-planner.png`.
- Machine-readable evidence is in `docs/evidence/local-planner.json`.

## Verified evidence, offline lattice (2026-09-17)

Decision 0021. Executed on the reference host with source fingerprint
`dbf72e2b12aa058ba8dbcf5ea9725a52f36e20d267d5db52d982db16d6b13a39`.

- `scripts/build.ps1 -Desktop`: 18/18 suites passed, 263.25 s of tests, with 404 QML checks, up from 398, and no runtime
  warnings; the UI check took 58.1 s of its 120. `scripts/build.ps1`: 17/17 suites passed, 203.20 s of tests. New suite
  `fd_lattice` has 10 groups and runs in 0.3 s; `fd_raceline` has 8.
- **The plan's pieces hold.** Layers every 6 m, 4 m where the line curves; nodes every 0.5 m across the corridor less the
  0.9 m half width, with the reference node at zero; edges to the next layer within 0.3 m per metre, each a quintic in
  station whose curvature matches its own samples (within 0.0015 1/m); edges beyond the steering limit removed (1690 of
  9412 on the preset), edges leaving a narrowing corridor removed, dead ends removed until none remain, and TUM's offline
  cost with ForzaETH's weights on every edge.
- **Along the racing line** of Foundry Circuit: 96 layers, 1585 of 1586 nodes usable and 7034 edges; the narrowest
  layer's usable nodes span 94% of its corridor. Along the preset: 100 layers, all 1700 nodes usable, 7722 edges. Each
  lattice takes under 0.01 s to lay (measurement only).
- **Found:** nodes given the racing line's own heading, as ForzaETH configures TUM's planner, left only 1075 of 1785
  nodes usable, because the line crosses the track at about 0.24 m per metre; node slopes that follow the corridor
  (TUM's variable heading) keep them. 3 m corner layers leave no steerable step of one node for this car, so they are 4 m.
  Nodes deep inside the preset's 18 m corners are dead ends by geometry.
- **Headless:** `--write-lattice DIR` writes `lattice.json` and four tables without a lap; six lattices are in
  `out/lattices` (preset, conditioned centreline, the racing lines of the preset, the conditioned centreline and the
  noisy trace, and a 0.3 m margin line).
- **Desktop:** the lattice is laid along the preset, the racing line or a replayed track, and the edges from the layers
  within 60 m ahead are drawn as faint lines, redrawn only as the car passes a layer. Inspected capture:
  `docs/assets/lattice.png`.
- **Recordings:** nothing recorded changed; all 135 recordings in `out/runs` still load and validate with this build.
- **Build scripts:** a blank stderr line now prints blank instead of `System.Management.Automation.RemoteException`.
- Machine-readable evidence is in `docs/evidence/lattice.json`.

## Verified evidence, racing line (2026-09-17)

Decision 0020. Executed on the reference host with source fingerprint
`f8522944ead75781f1cdf3213d4139d85304155b3fa9ec2265593a3cd5610084`.

- `scripts/build.ps1 -Desktop`: 17/17 suites passed, 272.39 s of tests, with 398 QML checks, up from 360, and no
  runtime warnings. `scripts/build.ps1`: 16/16 suites passed, 216.38 s of tests. New suite `fd_raceline` has 7 groups;
  `fd_envelope_speed_plan` has 11 and `fd_track_conditioning` 12.
- **OSQP v1.0.0** and qdldl v0.1.8 (Apache-2.0) were downloaded with the user's approval by `scripts/bootstrap-osqp.ps1`,
  checked against pinned SHA-256 hashes, and build unmodified with MSVC as a static library linked only by
  `fd_raceline`. Notices are in `THIRD_PARTY_NOTICES.md`. A core build in a fresh `build/core-no-osqp` without the
  sources passed its 15 suites in 189.17 s, skipping `fd_raceline`, and refused `--line racing` naming the script.
- **The plan's four tests hold.** On Foundry Circuit the line stays inside the corridor less the car's 0.9 m half width
  and a 0.6 m margin, to 0.02 m. Its peak curvature is 0.029 1/m against 0.060 on the centreline and a steering limit
  of 0.236, and a corridor no line can steer round is refused. Under 80% of the envelope its estimated lap is 26.20 s
  against 29.40 s on the conditioned centreline (grip fractions: 26.89 against 29.96 s). A stadium's straights stay
  straight within 0.0001 m.
- **Driven, two laps** at 80% of the envelope: 28.05 / 26.60 s on the racing line against 31.12 / 29.79 s on the
  conditioned centreline, with no invalid sample on either. The line is tracked within 0.45 m, and the tall car and the
  noisy trace's line also run clean. At grip 0.7 the line has 361 corner-entry samples past a tire's peak, where the
  centreline has none. With the grip fractions the four-wheel car spins on the line (3140 invalid samples); the kinematic
  bicycle laps it in 26.88 s flying without one.
- **Found, and adopted, amending decision 0018:** pulling away from rest on the line's curve spun the inside wheel for
  226 samples, because driving was bounded by the straight-line forward capacity. With an envelope, driving is now bounded
  by the forward limit at the lateral acceleration the reference asks; braking keeps the straight-line capacity. The spin
  is gone. Seven of the eight preset envelope runs of decision 0018 re-record byte-identical telemetry. Conditioned
  centreline laps move by at most 0.06 s. Grip fraction and kinematic runs are untouched. Decisions 0018's and 0019's
  evidence was recorded before the change.
- **A 0.3 m margin** lets the car's rear axle leave the corridor's margin in 250 samples, so the default is 0.6 m.
- **Cost:** the Foundry line converges in 6 iterations, about 1.1 s alone and up to 4.3 s beside other runs
  (measurement only).
- **Recordings:** schema 9 adds each track sample's corridor edges. Each of the 23 runs (`out/runs/raceline-*`,
  `centreline-*`, `preset-*`) validates with `--check-recording`, plans reproduced within 3.8e-10 m/s and plant motion
  within 1.0e-9 m. They were recorded with fingerprint `6bf390c3…`, before the lap estimate moved into `fd_core` and the
  desktop gained the line. The final build re-recorded `raceline-envelope` byte-identically as
  `raceline-envelope-final`.
- **All 135 recordings in `out/runs` load:** 18 schema 1, 8 schema 2, 7 schema 3, 20 schema 4, 4 schema 5, 11 schema 6,
  17 schema 7, 17 schema 8 and 33 schema 9, none rejected.
- **Desktop.** A LINE row under PLAN offers Centreline and Racing line while paused, the line solved once on a worker
  thread. It shows the plan in force's estimated lap: "racing line 26.20 s, centreline 30.29 s (29.40 s smoothed)" at 80%
  of the envelope. The road is drawn between each track's own edges, and the line not driven is drawn beside the car's
  own, on the map and in the legend. Refused while running, in replay, before the solve and with stated blockages.
  Inspected capture: `docs/assets/racing-line.png`.
- **Build scripts:** `scripts/common.ps1` no longer fails a redirected build on a native tool's stderr warning (qdldl's
  CMake deprecation notice did), and the desktop UI check's timeout is 120 s after it took 52 s of 60. The final
  desktop run after that change passed 17/17 in 278.22 s, the UI check taking 56.6 s; `fd_recording_playback` took
  42 s of its 60.
- Machine-readable evidence is in `docs/evidence/raceline.json`.

## Verified evidence, track conditioning (2026-09-16)

Decision 0019. Executed on the reference host with source fingerprint
`3522c3034519f0170c864c83c77bec2e5c843cceec94d6a99f95aec7cca02770`.

- `scripts/build.ps1 -Desktop`: 16/16 suites passed, 180.70 s of tests, with 360 QML checks and no runtime warnings. `scripts/build.ps1`: 15/15 suites passed, 146.68 s of tests. New suite `fd_track_conditioning` has
  11 groups and runs in under a second.
- **The plan's two tests hold.** Foundry Circuit traced every metre with 0.1 m of seeded noise per axis is conditioned
  back to within 0.0041 1/m of its curvature from 10 m beyond any join (the tightest corner's is 0.056), within 0.19 m of
  its centreline and 0.5 m of its length; a figure eight is refused because it crosses itself.
- **Also refused:** hairpins tighter than half the width (normals cross) and a loop whose corridor overlaps itself.
- **Continuous curvature:** the conditioned preset's curvature changes by at most 0.0025 1/m between samples against the
  preset's 0.056 steps, and its tightest apex rises 8% to 0.060 1/m.
- **Driven on Foundry Circuit, two laps** (`out/runs/cond-*` from the preset's samples, `out/runs/traced-*` from the
  noisy trace, both files in `out/tracks`): at 80% of the envelope the conditioned preset laps 31.11 / 29.80 s with no
  invalid sample and 0.35 m tracking error (preset 31.95 / 30.70 s, 0.51 m), the trace 31.25 / 29.94 s with none; the
  0.6 m high car has no invalid sample on it against 97 on the preset. The kinematic bicycle drives the trace without one.
- **Found:** with continuous curvature the grip fraction plan's separate lateral and braking caps meet, and the
  four-wheel car passes a tire's peak in 562 samples on the conditioned preset (323 on the trace) braking into corners;
  planned with the whole envelope it spins at the first corner (4747 invalid samples against 1091 on the preset).
- **Cost:** 0.007 s to condition a 500 m loop, 0.15 s for 2.5 km, 0.58 s for 5 km (quadratic checks).
- **Recordings:** each of the seven runs validates with `--check-recording` as schema 8 from this build, plans reproduced within 3.4e-10 m/s and plant motion within 1.0e-9 m.
- **All 102 recordings in `out/runs` load:** 18 schema 1, 8 schema 2, 7 schema 3, 20 schema 4, 4 schema 5, 11 schema 6, 17 schema 7 and 17 schema 8, none rejected, plant reproduction between 9.979e-11 and 1.00e-09 m.
- Machine-readable evidence is in `docs/evidence/track-conditioning.json`.

## Verified evidence, envelope speed plan (2026-09-16)

Decision 0018. Executed on the reference host with source fingerprint
`57fe8ce2c2da52e81b558221aca4a72531ee6617c4b7b207da3a9ab3c44e1a1a`.

- `scripts/build.ps1 -Desktop`: 15/15 suites passed, 180.17 s of tests, with 360 QML checks, up from 339, and no runtime warnings. `scripts/build.ps1`: 14/14 suites passed, 146.41 s of tests. New suite `fd_envelope_speed_plan` has
  10 groups; `fd_recording_playback` has 12, with an envelope plan fixture.
- **The plan's three tests hold.** Planned speed never exceeds the share of the envelope at any sample or edge, the
  seam included; a 30 m circle is planned exactly at the share of its lateral limit; lower grip still moves braking
  earlier (105.9 m at grip 0.7 against 116.6 m).
- **Planned with 80% of the envelope**, the default car laps the preset in 31.95 s from a standing start and 30.70 s
  flying, against 34.05 and 32.35 s with the grip fractions, with no invalid sample; at grip 0.7, 34.71 s flying against
  37.39 s. Braking for the first corner begins at 116.6 m against 111.2 m.
- **Setup moves braking points now**: 62% front brake bias 120.2 m, 3 m² of downforce area 117.8 m, 1200 kg 115.4 m,
  a 0.6 m centre of gravity height 113.0 m.
- **Found:** with all of the envelope the car laps in 29.03 s flying but a tire passes its peak in about 9% of samples,
  braking while turning in ahead of corners, driving out with lateral acceleration left over and running a tighter
  line than the reference mid-corner; a curvature window did not help and 80% was the largest share tried that kept
  every tire within its peak. The 0.6 m high car still passes a tire's peak in 97 of 12784 samples at 80%, turning in
  under braking. The controller now brakes up to the car's capacity (-7.80 m/s² at 10 m/s) rather than 5.39 m/s².
- **Faster derivation:** envelopes are derived in parallel, 0.27 s instead of 1.3 s, byte-identical to decision 0017's.
- **Recording schema 8** records the plan mode, envelope fingerprint and share; each of the ten runs validates with `--check-recording`, plans reproduced within 3.1e-10 to 4.2e-10 m/s and plant motion within 9.94e-10 m, and every envelope run reports that this build derives its envelope.
- **All 95 recordings in `out/runs` load:** 18 schema 1, 8 schema 2, 7 schema 3, 20 schema 4, 4 schema 5, 11 schema 6, 17 schema 7 and 10 schema 8, none rejected, plant reproduction between 9.979e-11 and 9.989e-10 m.
- **Desktop.** A PLAN row under STEERING offers Fractions and Envelope 80% for the four-wheel car while paused; WHY THIS
  SPEED names the car's envelope and states the plan's braking bound; the G-G diagram draws the plan's share inside the
  envelope. Inspected capture: `docs/assets/envelope-plan.png`.
- Machine-readable evidence is in `docs/evidence/envelope-speed-plan.json`.

## Verified evidence, performance envelope (2026-09-16)

Decision 0017. Executed on the reference host with source fingerprint
`cf58dbdcf59cf9b84ee4bedb605141346203396dd665cb802a361f049a28be65`.

- `scripts/build.ps1 -Desktop`: 14/14 suites passed, 147.89 s of tests, with 339 QML checks, up from 331, and no runtime warnings. `scripts/build.ps1`: 13/13 suites passed, 139.96 s of tests. New
  suite `fd_performance_envelope` has 12 groups and runs in about 5 s.
- **The envelope comes from the plant.** At each speed from 4 to 24 m/s the four-wheel car is held at speed while
  steering is swept until a turn cannot settle with every tire within its peak; forward and braking limits are
  probed from settled turns at 0, 25, 50, 75, 90 and 100% of that limit, left and right separately. Each car takes
  1.1 to 1.4 s. Left and right differ by at most 0.002 m/s² in lateral limit.
- **The plan's three tests hold.** Symmetric in lateral acceleration. Braking exceeds driving wherever power limits
  driving (7.87 against 7.64 m/s² at 14 m/s) and, with a brake bias matched to the braking load, everywhere; at the
  static 50:50 bias tire-limited braking only matches all-wheel-drive driving (7.72 against 7.73 m/s² at 4 m/s),
  because the unloaded rear wheels reach their peak first. Lateral capacity from 12 to 24 m/s: 9.38 to 9.45 m/s²
  without downforce, 9.66 to 10.46 with 3 m² of downforce area.
- **Found:** at 4 m/s parallel steering, not grip, limits the turn: 3.30 m/s² against 9.80 the tires could give,
  with the outer front tire's combined slip at 1.002 on full steering. Where power binds, forward capacity is 88 to
  100% of power less drag over equivalent mass (7.64 against 8.46 m/s² at 14 m/s).
- **Headless** (`out/envelope-*`, TUM `ggv.csv`, `ax_max_machines.csv` and `envelope.csv` each naming the
  fingerprint): default `033cc3dd344a93cd` lateral 9.36 m/s² at 16 m/s; 3 m² downforce 10.02; 0.6 m high 9.00;
  1200 kg 9.06 with forward 4.78 at 16 m/s against 6.89; 70% front bias braking 8.21 against 7.77 at 8 m/s; rear-only
  braking 4.2 to 4.4 m/s² across the grid; grip 0.7 lateral 6.58. The default envelope from this build is byte-identical
  to the one recorded in `out/envelope-default`.
- **MAP table fingerprints unchanged** after moving the shared model identity: for all ten recorded MAP runs, seven
  distinct tables, `--check-recording` reports that this build derives the recorded table.
- **All 85 recordings in `out/runs` load:** 18 schema 1, 8 schema 2, 7 schema 3, 20 schema 4, 4 schema 5,
  11 schema 6 and 17 schema 7, none rejected, plant reproduction between 9.979e-11 and 9.989e-10 m.
- **Desktop.** A G-G ENVELOPE card beside the TIRE SLIP card for the four-wheel car: the boundary at the current
  speed, a one-g circle and the car's own acceleration, derived on a worker thread, again after any change of car,
  setup or grip, and from the recorded car in replay. Inspected captures: `docs/assets/gg-envelope.png` (live,
  `033cc3dd`) and `docs/assets/gg-envelope-replay.png` (a rear-braked recorded car, `a85f5f62`, decelerating at
  3.59 m/s² with its rear wheels locked, inside its 4.33 m/s² braking limit because a sliding tire gives less than
  its peak).
- **Not done:** the fastest-lap cross-check (its Windows release is not here; downloading awaits approval). The
  speed plan does not use the envelope yet.
- Machine-readable evidence is in `docs/evidence/performance-envelope.json`.

## Verified evidence, honest setup controls (2026-09-16)

Decision 0016. Executed on the reference host with source fingerprint
`8e17d67b8e65efeb5f923adde63b05ba465bd322275a14811be571611a0062c9`.

- `scripts/build.ps1 -Desktop`: 13/13 suites passed, 162.53 s of tests, with 331 QML checks, up from 303, and
  no runtime warnings. `scripts/build.ps1`: 12/12 suites passed, 135.74 s of tests. New suite `fd_setup_effects` has 11 groups.
- **Every exposed vehicle parameter moves a measured outcome on the first 12 s of the preset:** mass raises
  grip use (0.805 at 600 kg, 0.868 at 1400); centre of gravity height raises the front axle's braking load
  (4087 to 4891 N) and deepens rear braking slip; brake bias at 20% locks the rear wheels (102 samples) and at
  80% moves the deepest braking slip to the front; drag area cuts distance (167.48 to 165.48 m); downforce area
  lowers grip use (0.826 to 0.788); 40 kW reaches 15 m/s at 3.29 s against 2.91; a coupling holds the driven
  wheels together (3.19 to 2.14 rad/s apart) and makes the car understeer; and the headless-only roll balance,
  aero balance and drive split each move one too.
- **The split and the schema.** Vehicle parameters already live in the vehicle model; `Config` keeps planning,
  control, timing, road grip and the shared wheelbase and steering limits, and its schema stays 1. Recordings
  need no change: setups apply between runs and their values are in metadata.
- **Laps** (`out/runs/setup-*`): default 34.045 s, identical to `aero-lap`; 600 kg 34.130 s and 1400 kg
  34.190 s; 40 kW 34.840 s; 1.5 m² of drag area 34.340 s; 4 m² of downforce area 34.025 s with grip use 0.788;
  0.6 m high 38 invalid samples; an 80 N·m·s/rad coupling 34.175 s with 0.515 m of tracking error; 20% front bias
  locked the rear wheels for 102 samples in 20 s.
- **Cost.** An 80 N·m·s/rad coupling made a prediction 6.4 to 7.7 ms and a blocked decision 48 to 50 ms, against
  2.8 to 4.2 and 17.5 to 27.6 ms open (measurement only). A secant step on the coupled wheel sweep had already cut
  the scenario's wall time from 5.3 s.
- **All 85 recordings in `out/runs` load:** 18 schema 1, 8 schema 2, 7 schema 3, 20 schema 4, 4 schema 5,
  11 schema 6 and 17 schema 7, none rejected, plant reproduction between 9.979e-11 and 9.989e-10 m.
- **Desktop.** A Car setup panel beside the TIRE SLIP card, editable while paused, each change a fresh run,
  read only in replay. Inspected captures: `docs/assets/car-setup.png`, `docs/assets/car-setup-rear-lock.png`.
- Machine-readable evidence is in `docs/evidence/setup-controls.json`.

## Verified evidence, aerodynamics and viscous coupling (2026-09-16)

Decision 0015. Executed on the reference host with source fingerprint
`dea39e16c36b432f0c94c318280d5bc14e5fc5f2eceac7423f0da840949f1495`.

- `scripts/build.ps1 -Desktop`: 12/12 suites passed, 92.41 s of tests, with 303 QML checks, up from 301, and
  no runtime warnings. `scripts/build.ps1`: 11/11 suites passed, 91.48 s of tests. New suite `fd_aerodynamics` has 7 groups;
  `fd_wheel_model` has 12.
- **Drag against its definitions.** Coasting at 29.82 m/s decelerated at 0.391182 m/s² against drag over mass
  plus wheel inertia of 0.391190. At full throttle the car approached 64.631 m/s against the 64.801 m/s
  where drag times speed equals 100 kW, and 51.271 against 51.433 with twice the drag area. Braking energy
  balances brake, slip and drag work to 0.03%.
- **Downforce.** Wheel loads carry weight plus the downforce at the car's speed on every tick. The largest
  steady turn inside the tire envelope fell from 9.24 to 8.86 m/s² between 10 and 30 m/s without downforce and
  rose from 9.40 to 10.77 m/s² with 3 m² of downforce area. Above the tire's 6000 N reference load the
  heaviest wheel's peak friction was 0.831, below the upper reference's 0.85 and above its floor.
- **Viscous coupling.** Out of a left turn on rear-wheel drive at 0.45 g, an open differential spun the inner
  rear wheel to slip ratio 2.06 and gained 2.014 m/s in 0.6 s; a 40 N·m·s/rad coupling held it at 0.33 and
  gained 2.248 m/s with less sideslip. At 0.6 g the coupling spun both rear wheels and slid to 0.572 rad
  against 0.368. In a straight line it changes nothing.
- **Laps.** Default drag costs 0.24 s (34.045 s against 33.810 s). 3 m² of downforce area kept 34.045 s,
  because the speed plan ignores downforce, and lowered the largest grip use from 0.826 to 0.794. Rear-wheel
  drive laps took 34.250 s open with 926 invalid samples and the rear wheels up to 94 rad/s apart, and
  34.215 s coupled with 771 invalid samples, 2.5 rad/s apart, but 0.107 rad of sideslip and 0.728 m of
  tracking error against 0.069 rad and 0.365 m.
- **Existing runs unchanged.** `out/runs/aero-still-air-lap` (`--drag-area 0`) is byte-identical to
  `loads-lap` apart from metadata identity and the four new keys; `aero-scenario-avoid`,
  `aero-pursuit-default` and `aero-map-soft` are byte-identical to their schema 6 recordings apart from
  metadata identity.
- **Recording schema 7.** A rear-locked fixture with load transfer, drag, downforce and a coupling
  round-trips all parameters and reproduces to 9.73e-10 m; a schema 6 copy whose loads carry downforce,
  aero keys in schema 6 metadata, a missing drag area and an edited downforce area are refused.
- **All 76 recordings in `out/runs` load:** 18 schema 1, 8 schema 2, 7 schema 3, 20 schema 4, 4 schema 5,
  11 schema 6 and 8 schema 7, none rejected, plant reproduction between 9.979e-11 and 9.989e-10 m.
- **Desktop.** DRAG / DOWNFORCE in the TIRE SLIP card, checked against the plant. Inspected capture:
  `docs/assets/four-wheel-aero.png`.
- Machine-readable evidence is in `docs/evidence/aerodynamics.json`.

## Verified evidence, quasi-static load transfer (2026-09-15)

Decision 0014. Executed on the reference host with source fingerprint
`e2714e28be22e5047688a8f80f9f5f66aaf5e106a6fe9930a8dff6d46645c138`.

- `scripts/build.ps1 -Desktop`: 11/11 suites passed, 90.63 s of tests, with 301 QML checks, up from 280,
  and no runtime warnings. `scripts/build.ps1`: 10/10 suites passed, 68.35 s of tests. New suite `fd_load_transfer` has 10 groups.
- **Against the equations and TUM's closed form.** Over 5000 random cars and forces, 3865 unlifted load
  sets satisfy weight, pitch, roll and the roll-balance closure within 1e-9 relative, and 1135 with a
  lifted wheel never go negative. TUM's form agrees within 9.1e-13 N with equal tracks and straight
  wheels; with steered front wheels it differs by up to 53.3 N and with unequal tracks by up to 95.9 N,
  never more than the terms it drops.
- **In motion.** Braking at a measured 4.699 m/s² put 506.0 N onto the front axle, `m·a·h/L` within 1%,
  and driving took 257.7 N off it. A steady 3.64 m/s² turn moved `m·a_y·h/track` outward within 0.01 N,
  and the front axle took exactly the configured share at roll balances of 0.3, 0.5 and 0.8. Near
  7 m/s² the understeer angle went from −0.00065 rad at a roll balance of 0.2 to +0.00153 rad at 0.8.
  A 0.75 m high car with all roll transfer on the front lifted its inner front wheel in a turn at 12 m/s.
- **Brake bias became delicate.** At 0.95 g the static bias of 0.5 locked the unloaded rear wheels and
  stopped in 25.59 m; the bias matched to the braking load and wheel inertia, 0.617, locked nothing and
  stopped in 21.98 m. In headless 20 s runs, 10% front bias now locks the rear wheels at 7.99 s and
  spins the car to 1.139 rad; without load transfer it never locked (decision 0013).
- **Found by testing it.** Loads first came from the centre of gravity's acceleration, and in the
  rear-locked recording fixture the low-speed blend's relaxation put up to 8.6 kN on a right-hand wheel
  of a car spinning at walking pace; loads now come from tire forces, and that run's largest front axle
  change is 840 N. Loads lagging one substep make full-power wheelspin first order in the substep,
  5.4e-3 m/s after a second at 2.5 ms against a 0.05 ms reference; against a 0.2 ms reference the other
  cases stay within 1e-3 m/s and 1.3 mm, except locked braking in a turn at 1.8e-3 m/s and 5.5 mm. A car at zero height keeps its static loads exactly.
- **Cost.** A four-wheel prediction measured 2.9 to 3.2 ms and a blocked decision 18.0 to 19.0 ms,
  within or below decision 0012's range (measurement only).
- **Existing runs unchanged.** `out/runs/loads-flat-lap` (`--cg-height 0`) is byte-identical to the
  schema 5 `wheels-lap` apart from metadata identity, the two new keys and the appended load columns.
  `loads-scenario-avoid`, `loads-pursuit-default` and `loads-map-soft` are byte-identical to their
  schema 4 recordings apart from metadata identity and eight appended wheel columns, all zero.
- **Recording schema 6.** A rear-locked run with load transfer round-trips both parameters and every
  load, reproduces to 9.77e-10 m, and read as schema 5 exposes the missing transfer at 9.0e-6 m; six
  tampered load records, including an edited mass whose weight no longer matches the loads, are refused.
- **All 68 recordings in `out/runs` load:** 18 schema 1, 8 schema 2, 7 schema 3, 20 schema 4, 4 schema 5
  and 11 schema 6, none rejected, plant reproduction between 9.979e-11 and 9.989e-10 m.
- **Desktop.** Wheel tiles show each tire's force in its friction circle, its load in kN and a load bar
  against its load at rest, checked against the plant live and against the recorded loads in replay.
  Inspected capture: `docs/assets/four-wheel-loads.png`.
- Machine-readable evidence is in `docs/evidence/load-transfer.json`.

## Verified evidence, tire slip, balance and four rotating wheels (2026-09-15)

Decisions 0011, 0012 and 0013. Executed on the reference host with source fingerprint
`4d1e4c225fcbc9f1e9bec85fbfcc7a8b54abccfbbe15a9380e6d7f20cc8c8cb4`.

- `scripts/build.ps1`: 9/9 suites passed, 77.07 s of tests. `scripts/build.ps1 -Desktop`: 10/10
  suites passed, 85.46 s of tests, with 280 QML checks, up from 248, and no runtime warnings. New
  suite `fd_wheel_model` has 11 groups; `fd_vehicle_model` has 17 and `fd_recording_playback` 11.
- **Balance (0011).** Reported slip angles equal the formula from the state within 1e-12 rad over
  5000 random states. In settled 11 m/s turns the soft-front car reads understeer at 0.005994 rad
  against 0.005980 rad of steering beyond geometry, a soft-rear car oversteer, the default car
  neutral (7.9e-6 rad). Replay derives the live slip angles and label at all 2401 samples of the
  dynamic fixture within 1e-9 rad without moving the cursor.
- **Four rotating wheels (0012), against independent oracles:** acceleration is wheel torque over
  mass plus wheel inertia within 1%; threshold braking at 0.95 g took 21.98 m from 20 to 3 m/s and
  locked braking 25.32 m; a locked car decelerated at 7.697 m/s², the tire law's force at slip ratio
  -1; 90% front bias in a braked turn locked only the front wheels and the car ran straight, 10% locked
  only the rear and it spun to 1.18 rad; rear-wheel drive spun its rear wheels to slip ratio 4.17 at
  4 m/s; with 30 kW at 25.6 m/s acceleration was 1.394 m/s² against 1.401 from power; braking energy
  balanced to 0.06%. The unchanged controller lapped it in 33.805 s.
- **Found by testing it:** splitting wheels from chassis first cost 1.9% of acceleration and 20 cm in
  a pull-away; holding each wheel's slip ratio through the chassis step fixed both. Cost began at
  10.3 ms per prediction; the 2.5 ms substep, secant wheel solves, a shared tire curve and a new
  `envelope` query brought it to 3.4 to 4.6 ms, with a blocked decision at 19 to 26 ms on a loaded
  host. A 5 ms substep would halve that but made wheelspin a hundred times less accurate.
- **Existing plants unchanged.** A kinematic scenario, a default dynamic run and a MAP run of the
  soft-front car re-recorded after the shared-code changes (`out/runs/wheels-scenario-avoid`,
  `wheels-pursuit-default`, `wheels-map-soft`) are byte-identical to their earlier recordings apart
  from the source fingerprint.
- **Recording schema 5 (0013).** A four-wheel run with all braking on the rear round-trips every
  parameter and wheel speed, records 264 samples with a rear wheel locked while moving, and reproduces
  to 9.90e-10 m; five tampered wheel records are refused by name. Headless runs
  `out/runs/wheels-lap`, `wheels-bias-front-0.9`, `-0.1` and `-0` gave a 33.805 s lap, no lock at 90%
  or 10% front bias, and a rear lock at 7.97 s followed by a spin with no front braking; all reproduce
  to below 1e-9 m. They were recorded before a one-line headless message fix changed the fingerprint.
- **All 57 recordings in `out/runs` load:** 18 schema 1, 8 schema 2, 7 schema 3, 20 schema 4 and
  4 schema 5, none rejected, plant reproduction between 9.979e-11 and 9.989e-10 m.
- **Desktop.** The tire slip card with slip bars, balance label and definition, the velocity arrow,
  the 4 wheels model with MAP refused, wheel tiles from the plant, and a replay of the rear-locking run
  showing LOCK on the rear tiles and red rear tires. A capture showed the replay scrubber covering the
  sideslip readout; REAR SIDESLIP and MAP CORRECTION moved into the card and the status line elides.
  Inspected captures: `docs/assets/map-steering.png`, `docs/assets/four-wheel-lock.png`,
  `docs/assets/dynamic-plant.png`.
- Machine-readable evidence is in `docs/evidence/four-wheel-car.json`.

## Verified evidence, MAP steering (2026-09-15)

Decision 0010. Executed on the reference host with source fingerprint
`86a4f08c3968c4ab50f4139cad339b3c4382afa90bf13ac71cb9fd8c31c8e0c3`.

- `scripts/build.ps1`: 8/8 suites passed, 67.28 s of tests. `scripts/build.ps1 -Desktop`: 9/9
  suites passed, 94.29 s of tests, with 248 QML checks, up from 220, and no runtime warnings.
  The new suite `fd_steering_law` has six groups; `fd_recording_playback` has ten.
- **The table is the plant's.** The kinematic bicycle's table equals `atan(Lκ)` within 5e-5 rad
  inside its envelope. The soft-front and soft-rear tables follow the understeer gradient from
  cornering stiffness measured on the tire law within 2%; at 6 m/s the soft-front table asks
  0.0300586 rad where the gradient predicts 0.0300634. At 22 m/s the largest steady turn is
  4.715 m/s² of 4.910 m/s² peak friction at μ 0.5 and 9.474 of 9.821 at μ 1.0. A table takes
  0.029 to 0.036 s to generate with the final generator (measurement only).
- **Found by testing it:** simulating each probe to steady state took 1.0 s per table, and once
  sped up to 0.05 s it stopped at 76% of peak friction at μ 0.5, because turns near the tire's
  peak settle too slowly. The generator now solves for the fixed point of one control period with
  Newton's method and keeps only turns whose period map is stable.
- **Closed loop, two laps** at lateral grip fraction 0.75, in the test suite and in headless runs
  with identical results: on the soft-front car Pure Pursuit reached 0.993 m of tracking error
  (RMS 0.627 m above 0.5 g) and MAP 0.436 m (RMS 0.286 m). On the default car MAP reached 0.530 m
  against Pure Pursuit's 0.501 m, with RMS equal within 1%. At the default configuration the
  soft-front car went from 0.709 m to 0.332 m. No run had an invalid sample.
- **Limits observed.** In a 12 s run with a cone cluster and a grip drop to 0.7 at 7 s
  (`out/runs/map-blockage-grip-drop` against `out/runs/pursuit-blockage-grip-drop`), MAP capped a
  swerve at 19.7 m/s to 0.062 rad where geometry asked 0.173, met the grip drop 0.19 m/s faster,
  and 579 of 2401 samples carry the braking-transient speed flag against none under Pure Pursuit;
  neither left the tire envelope. Rear-wheel drive under MAP still spun (`out/runs/map-rwd-lap`:
  1.437 rad of rear sideslip, 5538 of 8001 samples invalid).
- **Recording schema 4.** A MAP run round-trips its law, fingerprint and every geometric and MAP
  steering value; schema 3 copies load as Pure Pursuit; five tampered steering records are refused
  by name. The contract script records a MAP run through the headless binary and checks it, and
  MAP on the kinematic bicycle is refused before any output.
- **Kinematic runs unchanged.** Fresh schema 4 runs of `scenario-baseline`, `scenario-avoid`,
  `scenario-stop` and `verified-transient` kept every decision, trajectory, plan, event and track
  file byte-identical to their schema 3 runs, every schema 3 telemetry column identical on every
  row, and the summary identical. Those runs, and the default-car headless comparisons, were made
  earlier in the session, before the `--car` flag changed the fingerprint.
- **All 47 recordings in `out/runs` load:** 18 schema 1, 8 schema 2, 7 schema 3 and 14 schema 4,
  none rejected, with plant reproduction between 9.979e-11 and 9.989e-10 m where available.
- **Desktop.** The checks select Soft front and MAP, see MAP steer the understeering car 1.17°
  further than geometry in the first corner, lock the choice while running and in replay, refuse
  MAP on the kinematic bicycle with the reason, replay a recorded MAP run's correction exactly, and
  return to Pure Pursuit with the kinematic bicycle. Adding the steering row pushed the decision
  panel into the metrics at the minimum size; compact spacing was tightened and the blocked and
  avoidance captures were inspected again. Inspected captures: `docs/assets/map-steering.png`
  and a refreshed `docs/assets/dynamic-plant.png`.
- Machine-readable evidence is in `docs/evidence/map-steering.json`.

## Verified evidence, usable dynamic plant (2026-09-15)

Decision 0009. Executed on the reference host with source fingerprint
`4f462cd073f83d6c25d6b9506da56dcba0cecc2fede2c3ff47b63ab2e187c28d`.

- `scripts/build.ps1`: 7/7 suites passed in 50.70 s. `scripts/build.ps1 -Desktop`: 8/8 suites
  passed in 65.18 s, with 220 QML checks, up from 193, and no runtime warnings.
- **Prediction cost.** Scratch benchmark (Release, default dynamic car entering the first corner):
  one 5 ms tick went from 7.59 to 1.05 µs, a 3 s prediction from 5.52 to 1.35 ms, and a decision
  among six lines from 33.5 to 8.3 ms. Two changes: `TireAtLoad` validates a tire once and holds
  its constants (a test compares 2000 loads and 40,000 slips bit for bit with `tire_force`), and
  the substep grew from 1 to 2.5 ms.
- **Found by testing it:** the new convergence test failed a pull-away through the blend band by
  2.7 cm, because the low-speed blend pulled the state toward kinematic motion once per substep and
  so depended on the step. The blend now weights the state derivative with a 20 s⁻¹ relaxation.
  The default step then stays within 1 cm and 1e-3 m/s of a 0.5 ms reference in a corner entry, a
  μ 0.4 slide and the pull-away, and braking through the band changes yaw rate by at most
  5.9e-4 rad/s per tick. `fd_vehicle_model` has 14 groups and `fd_tire_model` 12.
- **Recording schema 3.** Metadata records the vehicle model with every parameter, telemetry adds
  lateral velocity, yaw rate and the grip envelope, and the summary adds maximum rear sideslip.
  `fd_recording_playback` has nine groups. A dynamic run with a non-default drive split, a grip
  change and a lane blockage round-trips every parameter, plant column and decision against the live
  simulation. Schema 2 and schema 1 copies still load. Five new tampering cases are refused by
  name: kinematic lateral velocity, kinematic yaw rate, a sample outside the envelope marked valid,
  an unknown vehicle kind and a wrong summary sideslip.
- **Plant reproduction** re-integrates every recorded step: 9.9e-10 m for the dynamic fixture and
  2.4e-5 m after editing its mass from 800 to 1100 kg. All 33 recordings in `out/runs` load: 18
  schema 1 runs report reproduction unavailable, and the 8 schema 2 and 7 schema 3 runs reproduce to
  between 1.0e-10 and 1.0e-9 m.
- **Kinematic runs unchanged.** Fresh schema 3 runs of `scenario-baseline`, `scenario-avoid`,
  `scenario-stop` and `verified-transient` kept every decision, trajectory, plan, event and track
  file byte-identical to the schema 2 runs, every schema 2 telemetry column identical on every row,
  and the summary identical apart from the new sideslip line.
- **Selection.** Headless `--plant dynamic` recorded a lap of 33.795 s (max tracking error 0.350 m,
  max rear sideslip 0.042 rad, no invalid samples). With `--drive-front-fraction 0` the car spun on
  leaving the first corner and completed no lap in 40 s (max rear sideslip 1.439 rad, 5538 of 8001
  samples invalid). `--drive-front-fraction` with the kinematic plant is refused before any output.
  The desktop checks select the dynamic plant, see sideslip in the first corner, lock the choice
  while running and in replay, replay a recorded dynamic run's sideslip exactly, and switch back.
  Inspected capture: `docs/assets/dynamic-plant.png` at 1100×700.
- Machine-readable evidence is in `docs/evidence/usable-dynamic-plant.json`.

## Verified evidence, dynamic plant and vehicle seam (2026-09-15)

Plan phases 0.2, 0.4 and 1.2, decisions 0005 and 0008. Executed on the reference host with source
fingerprint `eb315c26ee178531eb3643eb015935c98ab8cef19d29a9d26f93e7883dccdfa2`.

- `scripts/build.ps1`: 7/7 suites passed in 34.32 s. `scripts/build.ps1 -Desktop`: 8/8 suites
  passed in 73.25 s on a loaded host, with 193 QML checks and no runtime warnings.
- The new `fd_vehicle_model` suite passed 13 groups. The kinematic adapter matched
  `integrate_bicycle` and the previous demand expressions bit for bit on 20,000 random steps.
  The dynamic model followed Newton's law in a straight line (9.000 m/s after 2 s at 2 m/s² from
  5 m/s), matched the kinematic bicycle below 1 m/s within 1e-6 m, followed the linear
  understeer-gradient yaw rate within 2% for an understeering and an oversteering car, capped
  lateral acceleration at peak friction where the kinematic formula claimed more than twice as
  much, and, at μ 0.4, capped lateral acceleration and increased sideslip. It never reversed while
  braking, had no jump across the blend band, published rear-axle motion consistent with its speed,
  lateral velocity and yaw rate, and rejected invalid vehicles, states and commands.
- Closed loop on the preset with the default configuration and unchanged controller: the dynamic
  car completed the lap in 33.795 s against the kinematic 33.520 s. Its maximum tracking error was
  0.350 m against 0.267 m, maximum rear sideslip 0.042 rad and maximum axle grip use 0.75, with no
  tick past a slip peak.
- **Found by running it:** the first dynamic lap, with rear-wheel drive, spun out at 12.16 s on
  corner exit. The controller requested 5.39 m/s², more than the rear axle alone can deliver
  without load transfer (about 4.8 m/s²), so the rear lost all lateral grip. Sideslip grew from
  0.05 to 0.84 rad within a second. The default drive split is now load-proportional, and a test
  keeps the finding: -1.39 rad of rear sideslip with rear-wheel drive against -0.05 rad shared.
- **Kinematic recordings unchanged through the seam:** fresh runs into `out/runs/seam-*` of
  `scenario-baseline`, `scenario-avoid`, `scenario-stop` and `verified-transient` are byte-identical
  to their `schema2-*` counterparts in every file except metadata identity. That is 33 files,
  including all decisions and the 1.28 million predicted points of the stop run. All 26
  recordings in `out/runs` pass `--check-recording`.
- Planner cost with the kinematic plant is unchanged (0.39 ms clear, 2.05 ms blocked), and
  `fd_core_behavior` alone took 8.97 s.
- Measured cost of the dynamic plant: the closed-loop dynamic lap took 9.0 s of wall time against
  0.7 s kinematic, about 5 ms per control decision on a clear track. It must be reduced before an
  application offers it.
- `RecordingWriter` refuses a dynamic-plant simulation and names schema 3.
- Machine-readable evidence is in `docs/evidence/dynamic-plant.json`.

## Verified evidence, recorded decisions (2026-09-15)

Plan phase 0.3, decision 0007. Executed on the reference host with source fingerprint
`b208d5103158bf57d868cdcfa183d3139dbaf279b7413d31a287bcc7a8d4628c`.

- `scripts/build.ps1`: 6/6 suites passed in 30.78 s. `scripts/build.ps1 -Desktop`: 7/7 suites
  passed in 37.19 s, with 193 QML checks, up from 174, and no runtime warnings.
- `fd_recording_playback` has eight groups. Every recorded decision of a one-lap run with an
  off-grid grip change, and of a 14 s run with a lane blockage and a full-width blockage,
  equals the live simulation's decision field by field and point by point; the scenario run
  recorded 700 decisions, 167 steering around the lane blockage and 381 holding for the
  full-width one. The in-force rule is tested at time zero, at and after a control tick, and
  at and after an off-grid replan. A schema 1 copy loads with no decisions; schema 1 metadata
  beside decision files, schema 3, and a schema 2 run without `decisions.csv` are refused.
- Eight new tampering cases are each refused by name: a decision off the control schedule, a
  dropped decision, a command that disagrees with telemetry, a trajectory that does not start at
  the recorded state, one that stops short of the horizon, two selected options, an unknown
  blocking identifier and a missing option trajectory. The twelve earlier cases still pass. The
  writer refuses a scenario changed mid-recording and a skipped decision.
- The one-lap fixture recorded 1923 decisions and 290,374 predicted points; `trajectories.csv`
  is 20.2 MB and the whole lap loaded in 0.63 s. The suite took 12.5 to 15.5 s, from 2.3 s.
- `fd_recording_contract` now checks an off-grid event produces three decisions (two control
  ticks and one replan) and that `--check-recording` reports them.
- All 18 recordings made before this change, schema 1, still pass `--check-recording` and
  report `Decisions: not recorded (schema 1)`. Fresh runs into `out/runs/schema2-*` of
  `scenario-baseline`, `scenario-avoid`, `scenario-stop` and `verified-transient` kept
  `telemetry.csv`, `events.csv`, `track.csv`, `plan*.csv` and `summary.json` byte-identical to
  the originals; only metadata's schema version, run identifier and fingerprint differ.
- Cost of recording decisions, as measured: 19 MB for the clear lap (1676 decisions), 37 MB for
  the avoidance run (1662 decisions, 408 with alternatives) and 84 MB for the 30 s stop run
  (1500 decisions, 1401 holding with six lines each).
- The desktop replay of `schema2-scenario-avoid` at 3.0 s, captured as
  `docs/assets/replay-decision.png` and inspected, shows AVOIDING, "Steering left around
  cone-cluster", "4 of 5 lines rejected · chose 3.1 m left", the four rejected lines, the
  recorded prediction ribbon, and "Recorded decision, as the car made it".
- The plan's suggestion to reserve per-wheel telemetry columns was not followed: the kinematic
  plant would fill them with invented zeros. Decision 0007 explains; the dynamic plant adds
  them with schema 3.
- Machine-readable evidence is in `docs/evidence/recorded-decisions.json`.

## Verified evidence, projection and tire model (2026-09-14)

Plan phases 0.1 and 1.1. Executed on the reference host; before-and-after numbers come from
the same host in the same session.

- **Projection is exact and about seven times faster.** `project()` now skips stretches of
  track that provably cannot contain the nearest point, using a new track invariant that an
  arc is never shorter than its chord. Scratch micro-benchmark (Release, 851 samples, random
  states within 4 m of the centerline): `project` 4.27 to 0.59 µs, `compute_control` 6.73 to
  3.01 µs, one three-second prediction 1.43 to 0.48 ms, one decision with a stated blockage
  7.26 to 2.87 ms; `validate_track` 6.0 to 7.8 µs for the new check. `fd_core_behavior`
  24.09 to 12.90 s. Final `scripts/build.ps1`: 6/6 suites in 17.99 s. Final
  `scripts/build.ps1 -Desktop`: 7/7 suites in 25.84 s against 87.18 s recorded before, with
  174 QML checks and no runtime warnings. The `fd_core_behavior` timeout returns from 180 s
  to 90 s.
- **Nothing recorded changed.** The new `pruned_projection_matches_exhaustive_scan` group
  compares every returned field bit for bit with the previous exhaustive scan on the preset,
  the square fixture and four irregular tracks with a hairpin, at 72,000 random, on-sample,
  one-ulp-off, tie and extreme-magnitude positions. Fresh runs of `scenario-baseline`,
  `scenario-avoid`, `scenario-stop` and `verified-transient` with the final source, into
  `out/runs/phase0-final-*`, produced byte-identical `telemetry.csv`, `events.csv`,
  `track.csv`, `plan*.csv` and `summary.json`; only metadata's run identifier and source
  fingerprint differ. All 18 recordings in `out/runs`, the ten preserved ones and eight made
  this session, pass `--check-recording`, including re-running this build's planner against
  their plans. `scenario-stop` exits 2 as its
  original did, since it stops without completing a lap.
- **A track whose arc is shorter than its chord is now rejected** by name
  (`arc_shorter_than_chord_is_rejected`), with a tolerance that admits twelve-digit recorded
  tracks.
- **Tire force model** in `core/tire.*`, decision 0006, suite `fd_tire_model` with ten groups.
  It follows fastest-lap's load-sensitive combined-slip structure with the peak placed at
  the named slip and the curve shape set by the grip a sliding tire keeps, default 0.75.
  Measured through the interface: peaks within 1% of the named slip at four loads, a locked
  wheel keeping more than 0.75 of peak grip where fastest-lap's `Q = 1.9` shape keeps less than
  half as much, and 20,000 random combined slips inside one friction ellipse. Substituting
  fastest-lap's shape factor made five groups fail. It is not connected to the plant, so no
  claim about sliding or locking follows yet.
- **Correction to the plan's stiffness estimate.** The tire's small-slip slope is
  `mu*Fz*Q*S/kappa_peak`, 2.705 times the `mu*Fz/kappa_peak` used in TrackWayFastPlan Phase
  2.1. With that phase's numbers the wheel time constant is about 1.1 ms at 20 m/s rather than
  3 ms, and an explicit 1 ms step is unstable below about 8.8 m/s. The plan's conclusion,
  integrate wheel speed implicitly, is stronger than it states. A test pins the slope.
- Decision 0005 proposes the vehicle-model seam and keeps the rear axle as the published
  reference. It is not built: one adapter would make a hypothetical seam.
- Source fingerprint `e5dab3e98d39aa51f8c78ae43df4d76f5260c6f8f0826147bec967cdf41d3f1c`.
  Machine-readable evidence is in `docs/evidence/projection-and-tire.json`.

## Verified evidence, local action selection (2026-09-14)

Independently executed on the same reference host after the rolling-prediction work below.

- `scripts/build.ps1 -Desktop`: 6/6 CTest suites passed in 87.18 s. The QML verification
  passed 174 checks with no runtime warnings, up from 138.
- `scripts/build.ps1`: 5/5 headless suites passed. The new `fd_local_planning` suite has
  nine groups: a clear corridor reproducing plain reference following command for command,
  a blockage the prediction never reaches being ignored, a blocked line selecting the clear
  alternative, a fully blocked corridor limiting speed and stopping, invalid scenarios,
  a blockage spanning start/finish, two closed-loop driving comparisons and a cost
  measurement. All previously passing groups remain green.
- Closed loop, same configuration and start, differing only by the stated blockage. These
  are one preset, one grip setting and one blockage geometry, not a general claim:

  | Run | First lap | Max tracking error | Peak grip use | Invalid samples |
  | --- | ---: | ---: | ---: | ---: |
  | `out/runs/scenario-baseline`, clear | 33.520 s | 0.267 m | 0.686 | 0 |
  | `out/runs/scenario-avoid`, right side blocked | 33.240 s | 3.317 m | 0.995 | 0 |
  | `out/runs/scenario-stop`, full width blocked | no lap | 0.000 m | 0.550 | 0 |

- In `scenario-avoid` the car passes the blockage at a recorded +3.08 m lateral offset while
  holding 19.8 m/s, against the stated blocked interval reaching +1.0 m. The baseline drives
  the same stations at 0.000 m. The two lap times differ by 0.28 s in the avoiding run's
  favour; that is not a claim that avoiding is faster, only what these two runs measured.
- Peak grip use reaches 0.995 of the modelled circle during that lane change. The manoeuvre
  is close to the envelope, and at higher speed or lower grip the same candidate would be
  rejected and the car would brake instead.
- In `scenario-stop` the car halts at station 61.78 m at 0.0000 m/s against a blockage
  starting at 62 m. The commanded cap reaches zero at the 1.0 m clearance; the car then
  coasts the remaining tail, so the achieved clearance is 0.22 m, not the full margin.
- `--check-recording` accepts all three new runs and reports their stated blocked regions.
  Preserved earlier recordings, including `replay-demo` and `verified-high` from previous
  builds, still load and report zero blocked regions rather than failing on the new field.
- Measured planning cost on this host: about 3 ms for a clear decision and 16 ms for a
  blocked one, against a 20 ms control period. The simulation is not wall-clock bound so no
  result here depends on it, but that is not real-time headroom.
- Inspected actual Qt captures: `docs/assets/avoidance.png` and `docs/assets/blocked.png`
  at 1440×900, plus `build/desktop/ui-evidence/compact-avoidance.png` and
  `compact-blocked.png` at the 1100×700 minimum.
- Machine-readable evidence is in `docs/evidence/local-action-selection.json`.

## Verified evidence, rolling prediction (2026-09-14)

- Final `scripts/build.ps1 -Desktop`: 5/5 CTest suites passed in 52.23 s. The actual
  QML verification passed 138 checks with no runtime warnings. Captures at the 1100×700
  logical minimum size show a finite curved/braking prediction and no ribbon in replay.
- Final independent `scripts/build.ps1`: 4/4 CTest suites passed in 25.23 s. The original
  10 core behavior groups and five recording groups remain green. The eight new trajectory
  groups exercise actual-state anchoring, first-command execution, fixed-step prediction
  agreement, off-control-grid events, grip response, infeasibility, seam/horizon/acceleration
  semantics, analytic projection and invalid inputs.
- Two representative three-second forecasts at 20 m/s took 1.8313 ms total in the desktop
  test run. This is a small host measurement, not a solver deadline or real-time guarantee.
- A fresh 12 s recording `out/runs/rolling-grip-final-20260914`, with μ 1→0.45 at 6 s,
  passed the schema 1 reader. All 2401 telemetry samples (2402 lines including the header)
  are exactly identical, in order, to the matching prefix of `verified-transient`. The
  prediction and projection changes therefore preserve that baseline's actual commands,
  motion, revisions and explicit invalid transient. Peak grip use remains 1.23772533187,
  with 1020 invalid samples. Its planner reproduction error is 4.191e-10 m/s.
- Preserved `out/runs/replay-demo` also loads under the new build, reporting its source
  as a different build and matching reference speeds within 4.191e-10 m/s.
- Inspected actual Qt captures: `out/preview/rolling-prediction.png` (Formula appearance,
  8.0 simulation seconds, approximately 36 m of predicted motion), and
  `build/desktop/ui-evidence/compact-prediction-corner.png` / `compact-replay.png`.
  Generated captures and experiment data remain in ignored directories.

Current algorithm/recording source fingerprint:
`efb6745bf4d94114eae5ac1479b6c4616b1ab27a4757b9e9f32bfd8b50854ec7`.
Machine-readable evidence is in `docs/evidence/rolling-prediction.json`.

Review and execution caught two issues during this change. The initial forecast made a
desktop lap suite exceed its unchanged 60 s timeout; reusing the controller's projection
context and avoiding unnecessary distance calculations reduced the work. A stale ribbon
survived the initial replay notification; mode-aware caches now refresh before notifying
QML and geometry observers. Both affected suites passed after the fixes. No timeout limits
were increased and no behavior assertions were removed to make them pass.

The local-action work corrected three things found by running it. A candidate line was
rejected for demanding 23 m/s² of lateral grip, because aiming at an offset target one
lookahead ahead asks the car to snap sideways; offsets now aim far enough ahead for the
shift to fit the lateral budget. The approach to a blockage first planned against the full
braking budget and then overran the limit curve, because a proportional controller needs a
standing error to command deceleration; the approach now uses part of the budget and
subtracts that standing error from the commanded cap, after which the car stops at 61.78 m
before a blockage starting at 62 m. Two initial test expectations were themselves wrong
about the physics: at 32 m from a blockage the car genuinely need not brake yet, and the
code was right to say so. The `fd_core_behavior` timeout was raised from 90 s to 180 s
because the measured time is now about 70 s on this host, driven by re-predicting a
three-second horizon at every control tick; that is recorded as a cost to fix, not a
slowdown to absorb quietly.

## Previous milestone evidence (2026-09-13)

- `scripts/build.ps1 -Desktop`: successful native build and 4/4 CTest suites on the final source.
- `scripts/build.ps1`: successful independent headless-only build and 3/3 CTest suites, with no Qt.
- Fresh out-of-tree configure in `build/qtfree-check`: built and passed the same 3/3 suites.
  `dumpbin /dependents` on the resulting `fd_headless.exe` lists only MSVCP140, VCRUNTIME140,
  the UCRT `api-ms-win-crt-*` stubs and KERNEL32. No Qt DLL appears.
- Core suite: 10 behavior groups covering analytic motion, steering limits, input validation,
  periodic constraints, braking/grip, repeated laps, event continuity and deterministic scheduling.
- Recording/playback suite: 5 groups. A one-lap fixture with two live grip changes is written
  through `RecordingWriter`, reloaded, and compared sample by sample against the live
  simulation's own time, pose, speed and revision. Backward seeking is checked to restore
  revision 2 and then revision 1 with their configurations, and to keep the later change
  invisible before its recorded time. 14 tampered copies are each rejected by the specific
  rule they break, including event timing, configuration chain, sample revision, sample
  ordering, header text, summary counts, plan geometry, a missing plan revision and an
  orphan one. An edited plan speed loads but is exposed by planner reproduction.
- End-to-end recording contract: unchanged CMake checks for off-grid event time, rejected
  output reuse and per-tick event collisions still pass against the rewritten runner.
- Native UI: 78 checks through real QML handlers and bindings; no QML runtime warnings.
  Start, Pause, Reset, queued/committed grip, disabled moving appearance edits, camera switch
  and speed bindings verified as before. Added: an invalid directory is refused and the status
  names the contract; a fixture recording loads; seeking to 8 s applies the recorded change and
  seeking back to 3 s restores revision 1, its μ 1.00 readout and its higher first-corner plan
  speed; Play advances the playhead while `simulation().state().time_s` stays 0, so the plant
  is never stepped in replay; dragging the playhead seeks; Exit replay returns to the live
  simulation with its time, revision and grip untouched. Captures saved at 1100×700.
- `docs/assets/braking.png`: inspected actual Qt capture at 8.0 simulation seconds.
- `docs/assets/replay.png`: inspected actual Qt capture replaying `out/runs/replay-demo` at
  20.0 recorded seconds, showing the playhead event marker at 12 s, plan revision 2 and μ 0.55.
- `out/runs/verified-high`: μ=1.0, first lap 33.520 s, max error 0.266731 m, zero invalid samples.
- `out/runs/verified-low`: μ=0.45, first lap 48.260 s, max error 0.196933 m, zero invalid samples.
- `out/runs/grip-comparison.json`: same source/config except grip and identical track/state;
  planned braking starts 44.856861 m earlier at lower grip. First corner station 138.159132 m.
- `out/runs/verified-transient`: change μ=1→0.45 at 6.000 s, first lap 44.565 s; invalid samples
  6.005–11.100 s, peak grip utilization 1.237725. Preserved as explicit model-limit evidence.
- `out/runs/replay-demo`: two laps, 81.265 s, 16254 samples, one grip change μ=1→0.55 at
  12.000 s, laps at 40.140 s and 81.265 s, 154 invalid samples during the transient.
- `--check-recording` accepts all four preserved `out/runs` recordings, including the three
  written by the previous build, and reports planner reproduction within 4.191e-10 m/s for
  each. It rejects the older exploratory `artifacts/runs/grip-change`, whose metadata predates
  the schema and lacks `run_identifier`.

Algorithm/recording source fingerprint for the three earlier runs:
`2b3fba4eaa8ae1c071a0c12a36938c52254522949787f490f7c4e27b436a599b`. The current source is
`b698fb7f8a30805b0f3652d90d9ad46bdb967a020f7967f71dbfecaac700d10f` at that milestone; it also
covers `adapters/recording`. Byte-for-byte comparison of an 8 s run with a grip event, written
by the previous executable and the rewritten one, is identical apart from `run_identifier` and
`source_fingerprint`, so moving the writer into the shared library did not change the format.
Full local data and test screenshots live under ignored `out/` and `build/`; commands in README
reproduce them. Earlier exploratory recordings in `artifacts/runs/` are also ignored, not final evidence.

## Review fixes made

Independent review and running integration checks previously found and fixed nonfinite
controller outputs, zero-crossing stopping-distance error, stale revision files when reusing a
run directory, and the Qt plotting bottleneck caused by repeated reference-sequence reads. QML
caches plan data by revision, sharing the core color deadband instead of a separate threshold.

This session: the replay playhead crowded its own scrubber at the 1100×700 minimum size, so
the slider now has a minimum width and the revision caption hides below 520 px rather than
squeezing it. The first draft of the UI check counted the plan-revision signal emitted by
loading itself; the baseline is now taken after the load.

## Environment and limitations

Verified Windows x64, MSVC 19.44.35217, Windows SDK 10.0.26100.0, CMake 3.31.6, project-local
Qt 6.8.3 MSVC 2022 kit. Blender 4.5.3 LTS and CapCut's FFmpeg were found; no Blender render
pipeline was built. WSL/ROS were absent. Ubuntu/Windows headless CI is configured but remote
CI has not run. Linux/ROS/Gazebo and GPU performance are unverified.

The model is ideal state, known track, flat plane, no slip/aero/suspension or mass sensitivity.
Track validity is numerical and steering-based, with a rear-axle width margin; it is not swept
body collision checking, smooth-curvature geometry or Formula Student compliance. The global
periodic speed profile is reference guidance. The separate local prediction is time-based
but does not optimize lap time. Its three-second visible horizon is not a sensing range;
the known track supplies braking context beyond it.

The car's own instruments (decision 0029) read its own state late and noisy, from a recorded seed, but nothing reads what
they measure: the plant advances on its true state and every decision is made from it, so a measured run is still an
ideal-state run with its measurements recorded beside it. Their inertial unit reads the body-frame accelerations the
vehicle seam reports, with no lever arm, gravity component or road grade; no channel models dropout, bias drift, scale
error or quantisation; and nothing filters or fuses what they deliver.

The car's own instruments (decision 0029) read its own state late and noisy, from a recorded seed, but nothing reads what
they measure: the plant advances on its true state and every decision is made from it, so a measured run is still an
ideal-state run with its measurements recorded beside it. Their inertial unit reads the body-frame accelerations the
vehicle seam reports, with no lever arm, gravity component or road grade; no channel models dropout, bias drift, scale
error or quantisation; and nothing filters or fuses what they deliver.

The lattice planner compares only the cheapest path of each action, by the time its own speed profile implies over one
decision's horizon; it does not search the lattice for the fastest path, and a faster path that is no action's cheapest is
never seen. A profile's limits are the reference plan's, applied to a curvature estimated from straight segments as Pure
Pursuit drives them, so it is a model of what the car would do, checked by the prediction rather than guaranteed: an
estimate was within about 1% of the driven time in the measured scenarios, and options within 0.05 s of each other are
treated as ties, the previous choice being kept. Beyond the horizon every option is assumed to rejoin the reference plan.
Its search weights are ForzaETH's scaled values plus a turn weight and a 1 m return threshold chosen here, not tuned
across tracks or cars. It plans a pass for the nearest blockage only, checks its own clearance with straight segments at
0.5 m steps and the prediction at control points, not a swept body, and searches the centreline's lattice, since
blockages are not stated on the racing line. On the preset the lattice pass made the lane-blockage lap 0.26 s slower than
the five-offset pass, which is what those two runs measured; the five-offset car reached that lap time by staying 3.3 m
off the reference long after the blockage and using 0.995 of the modelled grip circle.

MPCC is not a real-time controller here: a decision's solving is measured on this host as a distribution whose median
sits near the 20 ms control period and whose tail runs to a few hundred milliseconds, and the simulation waits for it. Its
horizon is three seconds and its settings are fixed defaults tuned on this one preset, with a proximal step penalty that
trades convergence within one decision for convergence across decisions. It predicts the four-wheel car with that
car's longitudinal load transfer, drag and downforce but without its lateral load transfer, brake bias or wheel speeds,
which is the model error that remains; the model error is measured under the configuration each plan was made with, so
a grip change under a plan counts as plan error only. The per-wheel margin the desktop draws is measured the same way,
open loop along one plan with nothing replanned, and MPCC's own constraints stay per axle: it reports what a plan asks
of each wheel, not what the controller promised about them. It refuses the kinematic bicycle, which has no tires to predict
with; and a hold for a blockage is always the policy's. That handover is where the head-to-head found its worst result:
the planner compares actions by rolling the policy forward within the reference plan's grip fractions, while MPCC drives
to 0.95 of each axle's friction ellipse, so at the corner bollard the four-wheel car arrives at 19.5 m/s, the planner
finds no clear path and holds 15 m short, and the policy cannot stop it; the car slides, locks a rear wheel, leaves the
track and rejoins 13 s later (decision 0026). The same scenario is survived by the dynamic car, whose wheels cannot
lock, and by both baselines, which arrive slower. The planner still compares actions by rolling the policy forward, so a car MPCC has
brought to a blockage faster than the policy would can find no clear pass and hold. The desktop's WHY THIS SPEED panel
explains the reference plan, which MPCC does not drive to. The plan's Phase 6.4 friction-ellipse margin per wheel along
the horizon is not drawn yet, and its Phase 6.3 baseline, MAP with the envelope speed plan, does not exist for any one
car here: MAP has no table for the four-wheel car and the envelope plan is the four-wheel car's only, so MPCC was
compared with each car's own baselines.

Local action selection chooses among feasible lines; it does not solve for a fastest one.
Offsets do not re-derive curvature speed limits, so an alternative is never predicted to be
faster merely for being wider or tighter, and without a blockage the reference line always
wins. Blocked regions are scenario ground truth stated by the operator: there is no
detection, classification, tracking or sensor model, and none of these objects is a
pedestrian. Contact is tested at predicted control points against a sampled rectangle
inflated by a vehicle half-width, so it is neither continuous nor swept-body collision
checking and a clear result is not a safety guarantee. The stopping clearance achieved was
0.22 m against a planned 1.0 m margin, because the commanded cap reaches zero at the margin
and the car then coasts the remainder. Evaluating alternatives measured about 3 ms per
blocked decision against a 20 ms control period after the projection change, and 16 ms in the
earlier recorded measurement; both are measurements, not real-time headroom. `compute_control`
still re-validates the whole reference plan on every call, including every predicted control
point, which is now a larger share of its cost than projection.

The tire model is a steady-state force law without relaxation, camber, temperature, rolling
resistance or fitted data. The dynamic plant uses its lateral curve with a friction-ellipse
stand-in for combined slip. The plant has no load transfer, aerodynamics, drag, wheel speeds,
locking or drivetrain, and its parameters describe no real car. The four-wheel car adds wheel
speeds, brakes, a power limit, quasi-static load transfer, drag, downforce and a viscous coupling, but no
suspension, rolling resistance, wind, aerodynamic moment, engine map, ABS or limited-slip differential other
than the linear coupling; a lifted wheel is clamped rather than solved as a three-wheeled car. The grip fraction
plan ignores drag, downforce and the car; the envelope plan knows them only through the envelope, and only for this
car. MAP is not available for it, and its blocked decisions cost about the control
period, and about two and a half control periods with a stiff viscous coupling. The desktop offers seven setup
controls between runs; they move planned speeds and braking points only under the envelope plan. That envelope is the
four-wheel car's only: it takes about a quarter second per car, is resolved to its settling tolerances and bisection
depths rather than solved, and was not cross-checked against fastest-lap, whose download awaits approval. A point
inside its boundary is not a tire within its peak: a sliding tire gives less. The envelope plan uses 80% of the
envelope because Pure Pursuit turns in ahead of the centreline's curvature steps and drives a tighter line than the
reference; that share is a measured reserve on the preset (the whole envelope put a tire past its peak in about 9% of
samples), not a derived one. On a track with
continuous curvature, such as a conditioned one, the grip fraction plan asks the four-wheel car for braking and cornering
at once beyond what it has, because it caps them separately; the envelope plan does not. Track conditioning keeps one
width, has no per-point widths or inflation of narrow sections, is available in the headless runner only, and its
crossing checks are quadratic in the number of samples. The racing line is minimum curvature, not minimum time. It is
solved once before a run from the car's half width and kinematic steering limit only, with a 0.6 m margin measured for
Pure Pursuit on Foundry Circuit, and was not cross-checked against TUM's optimiser or fastest-lap. Only the envelope plan
drives the four-wheel car on it cleanly, and at grip 0.7 it still slides on corner entry. Stated blockages are positions
across the centreline, so neither application combines them with the racing line. The desktop solves the preset's line
only, and replay shows a recorded line's corridor but not the centreline it was solved from. Without OSQP's sources
`fd_raceline` is skipped and the headless runner refuses the racing line; the desktop's refusal without OSQP is compiled
but was not built or run. The offline lattice's edges are pruned by the kinematic steering limit alone, with no speed,
grip or envelope; it keeps the rear axle a half width inside the corridor rather than checking a swept body; and its cost
weights are ForzaETH's scaled values, not tuned for this car. Plant reproduction checks one step at a time and is reported, not enforced. MAP is a
steady-state correction: it ignores transients, can track slightly worse than geometry at the
limit, and leaves longitudinal allocation alone. The soft-front setup is illustrative. The recorded
table fingerprint covers the initial configuration; later grip revisions regenerated the table but
their fingerprints are not recorded.

Playback reads this project's own schema 1 to 11 directories only. It is not rosbag2, and it replays
recorded state rather than evaluating a different policy: recorded samples describe the path
the recorded controller drove, so a changed controller needs closed-loop simulation. The
desktop still records events only in memory; persistent logs come from the headless runner.
The reader loads a whole run into memory, including every recorded decision; a clear lap is
about 290,000 predicted points and a 30 s stop run 1.28 million. Only `trajectories.csv` is
parsed as a stream. Recorded decisions make run directories 19 to 84 MB in the measured runs.
Replay speed is fixed at 1.0 and there is no frame-step control. Emulation, virtualization and hardware behavior remain distinct and unimplemented.

## Next concrete milestone

Follow [TrackWayFastPlan.md](TrackWayFastPlan.md). Done: Phase 0.1 fast projection, Phase 0.3
recording schema 2, Phase 1.1 tire module, and Phase 1.2 dynamic single-track plant with the
vehicle seam of Phases 0.2/0.4 (decisions 0005 and 0008), made usable with a cheaper prediction,
recording schema 3 and plant selection in both applications (decision 0009), Phase 1.3 MAP
steering with recording schema 4 (decision 0010), Phase 1's visuals (decision 0011), Phase 2.1
four rotating wheels with recording schema 5 and wheel states in the desktop (decisions 0012 and 0013),
Phase 2.2 quasi-static load transfer with recording schema 6, wheel loads and friction circles in the
desktop (decision 0014), and Phase 2.3 aerodynamics with the viscous coupling Phase 2.1 deferred, recorded as
schema 7 (decision 0015).

Phase 2.4's setup controls are in both applications (decision 0016), which completes Phase 2 as the plan lists it.
Phase 3.1 derives the four-wheel car's G-G-V envelope from the plant, exports it in TUM's format and draws it live as
a G-G diagram (decision 0017).

Phase 3.2 plans the four-wheel car's speed from a share of that envelope, recorded as schema 8 and selectable in both
applications (decision 0018).

Phase 3.3 conditions a closed centreline into a smooth track with continuous curvature and left normals, driven and
recorded by the headless runner (decision 0019).

Phase 3.4 solves a minimum-curvature racing line with OSQP in `fd_raceline`, records it with its corridor edges as
schema 9, and offers it in both applications; the desktop draws it with the centreline and an estimated lap for each,
which completes Phase 3 (decision 0020).

Phase 5.1 lays an offline lattice of layers, nodes and steerable edges with an offline cost along whichever line the car
drives, written by the headless runner and drawn ahead of the car in the desktop (decision 0021). Phase 4 waited for the user's
approval of its downloads, given on 2026-09-25 (decision 0034).

Phase 5.2 searches that lattice online for the cheapest path per action, straight, pass left, pass right, return along
it, or brake toward the widest gap, replacing the five offsets as the default in both applications, recorded as schema 10
and shown with each action's line in its colour and the chosen action's cost (decision 0022).

Phase 5.3 gives every compared path its own speed profile and takes the clear action of least estimated time, recorded
as schema 11 and labelled in the desktop (decision 0023). That completes Phase 5, and AGENTS.md's planner invariant now
states what is optimised, over what horizon and lattice resolution.

Phase 6.1 ports MPCC into `fd_mpcc`: it drives the planner's chosen action for the cars with tires when asked, its plan
the ribbon, its status and solve time on screen, recorded as schema 12 with who drove each decision, and each plan's error
ahead measured from the recording (decision 0024).

Phase 6.2 records each plan's planned commands as schema 13, measures the model error against the plant's response to
them, and, from what that measurement found, predicts the four-wheel car with its pitch and air (decision 0025).

Phase 6.3 gives the four-wheel car a MAP steering table and compares MAP with its envelope speed plan against MPCC on
the same car and scenarios, reporting lap time, tracking error, peak per-wheel friction use, lock events and the
solve-time distribution (decision 0026).

Phase 6.4 draws the friction-ellipse margin per wheel along the horizon (decision 0027), which completes Phase 6: the
ribbon, the solver status, the solve time and now the margin are all on screen.

Phase 7.1 gives the car its own instruments, each with a rate, a dead-time queue and a seeded Gaussian error, recorded
as schema 14 and shown beside the true state; nothing reads them yet (decision 0029).

Phase 7.2 lays the course's cones along the corridor and perceives them after PacSim's model, every output a simulated
detection, recorded as schema 15 (decision 0030). Phases 7.3 and 7.4 drive on the path the car believes from those
detections and its own readings, through a port of FaSTTUBe's cone path planner checked against the published package and
cross-checked by a Delaunay planner written from FS-FEUP's description, every recording checked by a driver run again
(schema 16, decision 0031). Phase 7.5 judges every run after PacSim's competition logic and drives PacSim's Formula
Student layouts from where they lie (schema 17, decision 0032), which completes Phase 7. §8 gives the four-wheel car
physics profiles from named sources, every value's provenance marked, as physics only (decision 0033).

Phase 4, after the user approved both downloads, solves a car's minimum-time lap offline with fastest-lap 0.5, reads it
as a checked contract, cross-checks it with TUM's formulation and shows it in the desktop as a translucent car with sector
deltas, tire energy and setup sensitivities (decision 0034). The cross-check corrected decision 0033's TUM car and found
fastest-lap's friction floor. With it every phase of TrackWayFastPlan is built, and each claim in its §10 table has the
evidence it asks for.

Next, outside the plan's phases, in the order the runs point at: the planner's yardstick under MPCC, which rolls the
policy forward within the reference plan's grip while MPCC drives near each axle's limit (decision 0026); MPCC's
solve-time tail, for which a persistent OSQP workspace is the known step; an optimum for a car that drives its front
axle, which fastest-lap's `f1-3dof` cannot give (the project's roadster), for instance by TUM's formulation alone; and
offering the physics profiles in the desktop's car choice, which waits for the user to want the showroom touched.
Obstructions are stated in centreline coordinates, so on the racing line they must still be converted or restated.

ROS recording and the short driving demonstration remain planned. The user's
2026-09-14 clarification moved prediction/planning ahead of them; the explicit
2026-09-20 request added the local Blender showroom. Preserve the working
baseline and tests. The LiU contribution path remains optional and no outreach was sent.

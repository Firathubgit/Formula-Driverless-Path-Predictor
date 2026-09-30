# 0044 — Formula-style driving sound and practice-lap cues

The user rejected this oscillator engine's sound. Decision 0045 replaces the engine
and tire voices with recorded samples; the UI, audio-thread and cue lifecycle below remain.

2026-09-28. The user asks for F1-style sound on the accelerator, convincing gear
changes, a lap-time pling, and researched small racing-game audio details.

Use an original procedural classic V10-inspired voice in the desktop, under
Driver: You, including Gokart. This is a sound treatment: the RSX2 remains the
same electric kart, and its physics has no new engine or gearbox. The settings
explicitly identify synthesized sound and virtual gears. No audio dependency,
RPM, gear or cue enters fd_core, the vehicle seam, planning, recording or judging.

## Research and application

- [Wreckfest's engine-audio guide, hosted by Audiokinetic](https://www.audiokinetic.com/media/blog/LoopBasedCarEngineDesign/vehicle_audio_modding_guide_for_wreckfest-Wwise2019_2_9.pdf):
  RPM and engine load control separate on/off-load layers. Apply this principle
  with phase-continuous synthesis: brighter harmonics under throttle and a softer
  coasting voice. The guide was available through indexed excerpts; direct PDF
  retrieval returned 403. No samples or code were copied from it.
- [FMOD's vehicle-audio discussion](https://qa.fmod.com/t/random-loop-selection-question/12271):
  RPM plus load supplies variation; one pitched tone alone omits the work the
  engine is doing. Our small synthesis approach is a chosen approximation, not
  a measured recording of an F1 car.
- [Qt 6.8 QAudioSink](https://doc.qt.io/qt-6.8/qaudiosink.html): request the buffer
  size before starting, check the supported format and actual processed stream.
  Use the installed 6.8 API and an audio thread to keep feeding the device during
  expensive UI/planning work. The device can choose a different buffer size.

The additional choices are our sound design, not claims of a published optimum:
six virtual ratios scaled to the current car/assisted speed range, 15,000-RPM
upshift threshold, 75 ms load cut and RPM drop, 140 ms downshift rev blip, and
280 ms minimum shift spacing plus a separate downshift threshold. Continuous
phase and 25 ms gain smoothing avoid frame-boundary and pause/mute clicks.
Quiet speed-dependent road/wind noise and tire scrub from the plant's slip
add context. Forgiveness attenuates the ordinary tire estimate; full assistance
has no fabricated skid noise from steering. Harmonics above 43% of the sample
rate are omitted; a DC blocker and soft limiter keep the mix bounded.

A completed kart practice lap emits one short bell. Beating an existing session
best emits a three-note rising bell; the first completed lap is an ordinary bell.
The existing full-lap timer owns completion, including its ordered-gate checks.
Audio only observes its count/time; restart, re-entry and repeated display
updates cannot replay old notifications. A bell briefly lowers the engine mix.
Starting/resuming, pausing and successful preset operations get quieter short
cues. UI ticks are rate-limited. There is no continuous slider chirping.

## Integration and local preferences

`race_sound.*` is ordinary C++ presentation synthesis. `race_audio.*` wraps it in
Qt, on a dedicated output thread; immutable frame and mix copies cross queued
calls. `main.cpp` reads the existing Bridge/Simulation into audio frames. It
never writes a command, advances a second plant, or modifies the lap clock.
The sound is restricted to live human driving; pause fades out the engine,
and showroom/replay/autonomous mode do not play it. Leaving a drive clears cues.

Settings → Sound has a master switch, volume, Engine & tires, Lap & menu sounds
and a lap-chime preview. A quadratic gain curve makes volume useful at low
levels, starting at 45%. Qt INI preferences in AppLocalDataLocation/sound.ini
remember these choices, independently of the five handling presets. Verification
uses temporary preferences and disables hardware output. Missing/failed devices
show a status and leave driving available; output-device changes reopen the sink.

The device path requests 50 ms of output, generates 10 ms blocks on an 8 ms
worker timer, and retains partial writes. It tries stereo/mono 16-bit at 48/44.1
kHz then the device's preferred PCM format. The actual device here required its
preferred format. Eight-bit, 16-bit, 32-bit integer and float
conversion are supported; only the front pair is filled for surround outputs.

Evidence and limitations are in `docs/STATUS.md` and
`docs/evidence/driving-audio.json`. The WAV demonstration is generated under
`out/audio/`, never committed. Device streaming and numerical waveform checks
are evidence of playback/behavior, not a subjective listening judgment or a
claim of acoustically exact F1 reproduction.

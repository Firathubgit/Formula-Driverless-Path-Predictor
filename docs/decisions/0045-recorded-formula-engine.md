# 0045 — Replace the rejected oscillator engine with recorded racing audio

2026-09-28. The user rejected decision 0044's engine sound as unrealistic and asked
for an implementation based on open-source solutions or original forum work.

Replace its engine and tire oscillators with WAV playback. Keep the read-only audio
frame, worker thread, preferences and practice-lap cue lifecycle from 0044. No plant,
controller, handling preset, lap clock or recording contract changes.

## Sources actually used

- Speed Dreams SVN r9651, `data/cars/models/mp1-diamond-r25/f1v10.wav`: the Formula
  V10 sample shipped with its Diamond R25 car. Its adjacent readme identifies
  Xavier Bertaux, 2020, and the Free Art License. Retain the unchanged source WAV
  and license, with adaptations of the audio under Free Art License 1.3.
- Speed Dreams r9651, `data/data/sound/skid_tyres.wav`: CC0 according to the
  project's `SoundCredits.txt`, from LukaCafuka's Freesound 752837, sampled/mixed by
  Overshot. Actual tire slip still controls this layer; steering alone does not.
- [ExhaustNote](https://github.com/FASTSHIFT/ExhaustNote), MIT, commit
  `5760db9c30d59a7fdaaec7eb079df7b9b4700e3c`: read its sample player, crossfade and
  engine voice. Adapt its continuous variable-rate sample cursor/interpolation,
  adding double precision, input checks and filtered multirate sample levels.
  Preserve the MIT notice. Its engine/gearbox physics does not enter this project.

The research also inspected VDrift's car-sound configuration, engine-sim, Crankwave,
the original Granular-Synthesis-for-Engine-Audio project and licensed field
recordings on Freesound. Field recordings add audience/wind/Doppler that would
need removal. Prefer the existing game-ready Formula loop for this implementation.
No commercial game bank was extracted. Those research files stay in ignored out/.

## Playback and feel

The source engine is a 2.76-second 44.1-kHz mono recording, rather than another
generated harmonic stack. Its measured dominant firing component is approximately
580 Hz; 6960 RPM is the chosen audio calibration for five firings/revolution.
This is not measured engine RPM metadata. The whole recording retains its exhaust
texture and natural variation while pitch follows the virtual engine speed.

Prepare each loop once: remove DC, overlap the end/head over 60 ms, normalize with
peak headroom, and generate four filtered half-rate levels with a circular 63-tap
windowed-sinc filter. Variable playback blends those levels to suppress folded
high-frequency energy during pitch increases. Cursors are continuous across audio
buffers; no samples, files or filters are allocated/prepared in the output loop.

Throttle blends a quieter, low-pass coast colour into the full recorded exhaust,
sharing the same phase. **There is one engine recording, not independently recorded
on/off-load or multi-RPM banks.** A separate measured multi-RPM/load bank would be
the next fidelity improvement. This is the explicit limitation of this solution.

Seven chosen virtual ratios follow actual vehicle speed relative to the current
car/assisted speed range. They upshift at 15800 RPM only under throttle with little
braking, cut ignition volume for 55 ms with 2 ms edge smoothing, and pull revs down
with a 16 ms response during the shift. Downshifts use a lower threshold, 100 ms
throttle blip, and 220 ms minimum gear spacing to avoid hunting. An audio-only
limiter pulses at 42 Hz above 17500 RPM. None changes acceleration or speed.
The RSX2 remains an electric kart wearing this requested Formula sound treatment.

Recorded tires replace the former squeal oscillator. Quiet procedural road/wind,
short lap/best/start/pause/menu bells and cue ducking remain. Sound starts at the
same remembered master level; no user presets or sound settings are overwritten.

## Reproducibility and checks

`scripts/bootstrap-audio.ps1` downloads pinned r9651 media and credit files into
ignored `artifacts/audio/`, checking SHA-256 before replacing anything. No global
installation. Desktop CMake checks both media hashes and embeds them and the
license notices. Builds and playback need no network once bootstrapped; core
builds have no new dependency. WAV decoding validates dimensions, PCM format and
chunk boundaries before constructing a bank. Device setup reports an asset error
instead of substituting the rejected oscillator tone.

Tests exercise the embedded source assets, malformed/truncated WAV rejection,
alias suppression using a known two-tone signal, continuous looping across buffer
boundaries, seven up/down shifts, load/coast contrast, all supported device rates,
bounded PCM and the existing lap/mute/persistence lifecycle. Device streaming is
checked separately. See `docs/STATUS.md` and `docs/evidence/recorded-driving-audio.json`
for results. Numerical tests and a successful device stream do not establish a
subjective listening judgment or exact F1 acoustics.

# Black showroom

The native app opens with four locally rendered appearances: Aston Martin AMR23,
Koenigsegg Jesko Attack, Lamborghini Urus Performante, and Red Bull RB19. One car is
visible at a time. Choosing a car finishes the current motion before changing it;
the latest choice wins. Select cuts to a reserved angle of that parked car,
then fades to black and reveals the existing track setup with the simulation paused. Start begins
the simulation. Appearances do not configure manufacturer-specific physics.

Run `./scripts/run.ps1`. Use `--skip-showroom` to open the driving view directly;
replay and the existing track capture/verification modes bypass the showroom.
`--showroom-media DIR` selects a different local media package. The default is
`artifacts/showroom/media` under the repository. Missing or failed clips leave a
usable selection flow with a visible media status.

For a quality-only rerender of the approved studios, run
`python tools/showroom/render_4k_pack.py` with Python/Pillow. It snapshots the
current studio files and renders native 3840x2160 at 24 fps, 256 Eevee samples,
full-resolution ray tracing and a 1 GB shadow pool, then encodes H.264 CRF 12
with the slow preset. It preserves all authored cameras, materials and lighting.
Completed frames resume from `artifacts/showroom/revision-4k/`; the revision's
`job-status.json` and `render-logs/` report progress or failures. Source/decoded
seam validation and actual Qt playback must pass before it backs up and replaces
the default videos and Blender master. Keep the machine powered until complete.
Changing the quality settings requires a fresh `--revision` directory.

## Blender project

Open `artifacts/showroom/Black_Showroom.blend`. Its four editable scenes each
contain a single normalized car, packed PBR maps, a neutral polished floor,
a softly graded cyclorama, narrow white reflection lights, a camera,
motion keys, and a compositor black gate. The scene
selector changes cars. The `START HERE` text block documents the timeline.
Individual projects are also in `artifacts/showroom/studio/*-studio.blend`.

The four-car `gt-studio` production sources are in
`artifacts/showroom/revision-porsche/`. They retain the clip graph and native
selection contract with individual shot designs, narrow reflection cards,
corrected road-car materials and clearer lamp signatures. AMR23 has a low green
horizon, RB19 a very subtle red/blue horizon, and both road cars neutral graphite.
The material review, official-car comparisons and limits are in
[SHOWROOM_GT_REVIEW.md](SHOWROOM_GT_REVIEW.md). Render and playback evidence is
recorded in [STATUS.md](STATUS.md); the default media is updated after those checks.

The original pink project and supplied videos were read as references. The new
studio uses restrained strip lighting, two composed detail shots cut to a hero
view, and a closed, subtle idle camera/light movement. The car stays parked in
every state. The entrance cuts at frames 25 and 49; selection holds the canonical
hero through frame 162, cuts at 163 to its reserved view, then fades out. There
is no camera travel between shots and no car translation or rotation.

| Car | Entrance details | Reserved selection view |
| --- | --- | --- |
| AMR23 | Low front wing, high halo/cockpit | Low rear three-quarter |
| RB19 | Rear aero/exhaust, high diagonal nose | Low head-on symmetry |
| Jesko | Silver wheel/black tire, rear wing/lamp | Symmetrical rear silhouette |
| Urus | White Y headlight/shoulder, high rear quarter | Low opposite front quarter |

The F1 cars keep their authored green/navy liveries and sponsor colors; the road
cars use deep black lacquer, dry exposed carbon, and colored trim. Their UVs
and used source texture images remain in the prepared assets. The road-car
materials are adaptations: paint and carbon albedo are darkened, glass and lamp
lenses are separated, existing lamps emit light, and the Urus cabin upholstery
is red. Carbon retains a restrained resin reflection over a predominantly diffuse
weave, so broad panels stay dark at grazing angles. Authored normal maps remain
connected; carbon without an authored normal map gets a subtle bump derived from
its weave. Jesko source geometry with no
material assignment receives reviewed rotor or existing interior materials.
These are local derivatives, not unmodified copies of the source shaders.

Local preparation normalizes geometry to meters, nose +X, Z up, ground Z0; this
showroom origin is not the live simulation's rear-axle reference. Import audits
accompany each car. Unused source datablocks are removed before packing; used
missing images or faces without a material make preparation fail. The final
material audit records zero such faces for all four cars and packed image counts
of 5 (AMR23), 62 (Jesko), 29 (Urus), and 68 (RB19). This checks asset completeness;
it does not establish pixel-for-pixel equivalence to the reference photographs.

## Clip contract

Every scene has the same 228-frame timeline at 24 fps. All media is silent.

| Clip | Frames | Start | End |
| --- | --- | --- | --- |
| enter | 1–72 | RGB black, close detail | Hero pose |
| idle | 73–120 | Hero pose | Hero pose |
| exit | 121–156 | Hero pose | RGB black |
| select | 157–228 | Hero pose, then cut to new angle | RGB black |

Every directed switch is `exit(A) → enter(B)`: all 12 ordered pairs are listed in
the manifest and also exported as individual MP4s. The native player queues
changes through these endpoints. A fresh playback serial ignores stale decoder
events. A watchdog and decoder-error handling keep selection usable when a
file cannot play. The poster is the exact first idle frame.

There is no racing line in the Blender showroom. Once the track view is revealed,
only the existing plant advances the simulated car, and the live planner supplies
the actual prediction. Its green/yellow/red retain acceleration/hold/braking meaning
with the documented acceleration deadband.

## Reproduce and validate

`./scripts/build-showroom.ps1` imports the four user-supplied Downloads assets,
builds each studio, renders all 912 frames, assembles the editable Blender file,
and packages the media. It accepts `-Blender`, `-Python` (with Pillow), `-FFmpeg`,
`-Width` and `-Samples`; defaults are 1280×720, 32 Eevee samples and 24 fps.
No source model, video or original Blender file is modified. `-SkipPrepare`
reuses prepared cars; `-SkipRender` builds the editable projects without rendering
or packaging, and still prepares cars unless paired with `-SkipPrepare`.
Optional `-PreviewFirst` renders frame 73 before each full sequence, so a hero
preview becomes available early. It does not pause the render for approval.
`-Look gt-studio -OutputRoot artifacts/showroom/revision-porsche` stages a
separate output; `-SkipPrepare` reuses the cars prepared there. The default
look is `gt-studio`; `-Look black-vault` remains an optional legacy environment.

`tools/showroom/package_media.py --verify-only` checks all rendered inputs. Normal
packaging checks exact black frames, matching hero boundaries with an explicit
RGB RMSE tolerance, file counts and decoded MP4 frame counts. It also decodes
each state's first/last frames and checks exact black and hero RMSE <= 2/255
before publishing any assets, with `encoded-seams.json` as evidence. H.264 uses
CRF 16 and forces first/last keyframes to avoid residual pixels at the joins.
The manifest is published last. `--overview` assembles a four-car review video.
`tools/showroom/audit_animation.py`, run in the assembled Blender file, checks
stationary cars, explicit cuts, canonical poses and absence of studio racing lines.

`./scripts/run.ps1 --verify-showroom artifacts/showroom/ui-verification` exercises
the actual QML decoder through all 12 switches and each car's selection handoff.
`--capture-showroom artifacts/showroom/showroom.png` captures the selection UI.
Controller lifecycle tests belong to `fd_showroom_state`; the existing QML suite
also checks selection, paused handoff and replay isolation.
These commands describe the validation workflow; executed renders, packaging
checks and actual-media playback results are recorded separately in
[STATUS.md](STATUS.md).

Generated Blender files, extracted references, frames, videos, captures and local
audits stay under ignored `artifacts/showroom/`. Scripts and the media contract
are source-controlled. The supplied OEM meshes are local user assets; their
redistribution licenses were not supplied, so none are bundled into Git.

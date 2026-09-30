# GT studio car review

This records the visual review of the four supplied local models. The full
cinematic videos are now rendered, checked in the native player and activated;
see STATUS.md for the measured evidence. The material review sheet is the ignored local artifact
`artifacts/showroom/revision-porsche/four-car-review.jpg`. Its columns show the
rear and lamps, wheel and surface, front and lights, and selection pose.

The two user-supplied Porsche clips were used for shot language: slow close
pushes, distinct rear/wheel/front cuts, narrow light-strip reflections, and
recognizable lamp signatures. Their lighting differs (one graphite-black, one
light gray). The GT studio uses graphite-black to honor the existing showroom
direction. The native display is 16:9, while those clips are 9:16; it adapts
their visual language rather than matching their crop frame for frame.

| Car | Primary comparison | Current model/render review |
| --- | --- | --- |
| Aston Martin AMR23 (2023) | [Aston Martin launch gallery](https://www.astonmartinf1.com/en-GB/news/gallery/in-pictures-amr23-launch) | Racing green/teal, yellow trim, Aramco/Cognizant/BOSS/JCB marks, black carbon floor/wing and Pirelli yellow-lettered slicks are visible. The supplied model bakes its livery into one atlas, which is retained. |
| Oracle Red Bull RB19 (2023) | [RB19 team gallery](https://www.redbullracing.com/int-en/galleries/rb19) | Dark navy body, red/yellow bull and nose, Oracle/Bybit/Red Bull marks, aero wing and slick tire lettering are visible. The dry-studio rear rain light remains off. |
| Koenigsegg Jesko Attack | [Attack specifications](https://www.koenigsegg.com/technical-specifications-jesko-attack) and [model gallery](https://www.koenigsegg.com/model/jesko-attack) | Its large twin-profile top wing, splitter, low nose, central rear exhaust, front lamp shapes, red rear lamp, dark exposed carbon, gold source trim, and Michelin-labeled tires are visible. Koenigsegg lists forged aluminum wheels and optional Aircore carbon wheels; the supplied mesh's wheel design is preserved rather than substituting the optional wheel. |
| 2023 Lamborghini Urus Performante | [Lamborghini model history](https://www.lamborghini.com/en-en/history/urus-performante) | The carbon hood/roof/arches, side-fin spoiler, angular front lamp signatures, rear lamp strips, four exhaust tips, Pirelli-labeled tires, Lamborghini wheel badges, and red cabin are visible. This is the 2023 Performante, not the later Urus SE Performante. |

The two road-car sources had cabin glass with large white studio reflections that
made dark panels read silver. For this revision, the glazing is split from clear
lamp lenses; cabin Fresnel reflections are darkened while lamp lenses remain
clear. Black lacquer stays near-black under narrow softboxes. Carbon keeps its
source weave/normal variation and gains a controlled resin response. Existing
emissive lamp atlases were brightened, with their shapes retained. The F1 source
livery colors and sponsor textures were not recolored.

Following the user's review, Jesko's tire sidewall and tread materials were
remapped to dark rubber with a matte response and retained molded details. Its
rims now use silver alloy per the latest reference correction. Tire albedo and
specular reflection are reduced further, with fine molded grain over the
source sidewall relief. Urus paint
now has a stronger metallic substrate, subtle flake roughness and a smooth
clearcoat. Six updated frames per road car were rendered at 1280×720 with 32
Eevee samples; the earlier comparison images are retained locally under
`artifacts/showroom/revision-porsche/before-material-fix/`.

Urus headlamp review against the user's real-car front photograph found that
the source optical card covering the entire lamp had lost its transparency.
Material isolation identified that layer; restoring its source opacity exposes
the dark optics behind it. Front housing, optical card and clear cover now have
separate restrained responses. The lamp shape remains the supplied mesh; this
is a material correction, not a claim of newly modeled OEM headlamp internals.
`tools/showroom/render_lamp_review.py` renders a straight-on front and a single
lamp detail from the loaded Urus studio. It temporarily removes camera/root
animation and adds a neutral inspection fill for comparison with the daylight
photo; it does not save that light into the animation project. The two review
images are under `artifacts/showroom/revision-porsche/lamp-review/`.
The latest window adjustment slightly lightens the Urus cabin glazing's
transmitted tint while retaining its Fresnel reflection, so the red interior
is more visible. Headlamp covers remain separate from the cabin windows.

The subsequent environment review uses all three supplied Porsche references.
`horizon_studio()` in `tools/showroom/build_showroom.py` builds a continuous
360-degree cyclorama with a 4 m curved base, inside radius 10 m and outer radius
14 m. World-space height controls a low emission gradient on the backdrop,
blended into the neutral floor surface at its base. This keeps the tint in the
background through camera cuts and orbits without adding a colored car light.
AMR23 uses dark green, RB19 extremely restrained red/blue across the horizon,
and both road cars neutral graphite. The upper background fades nearly black.
The floor and existing white reflection cards retain their previous materials.
The horizon's floor-to-background shader blend was subsequently widened from
0.08 m to 0.65 m in height across the curved base, and the glow's falloff broadened
to remove the sharp dark lip. The studio dimensions and neutral floor remain.
Urus front optical emission uses the original atlas pattern converted to neutral
white with strength 3, retaining source opacity and the clear cover. The visible
Y signature is on the separate housing atlas (Image_24); its upper strip and
diagonal branch region now use a texture-driven white emission mask (strength
up to 2.5), leaving the surrounding housing dark. Simply increasing the optical
card did not expose that signature. Rear lamp emission remains unchanged.
All changes are in the staged GT studio only.

Environment sign-off image:
`artifacts/showroom/revision-porsche/environment-review/four-environments.jpg`.
Regenerate it with `tools/showroom/make_review_sheet.py --environment` and the
existing `--frames` / `--out` arguments. Ten frames per car were refreshed:
1, 12, 25, 44, 73, 120, 138, 156, 185, 228, at 1280x720 and 32 Eevee samples.
The 12 black endpoints are exactly zero. Idle 73/120 pairs have RGB RMSE
0, 0, 0.107 and 0.172 for AMR23, RB19, Jesko and Urus respectively (0-255 scale),
below the existing 2.0 media-pack limit. These selected stills do not replace
full-frame, encoded-seam and native-player verification after video rendering.
The staged master was reassembled and reopened: four scenes, one cyclorama
per scene, all 164 used file images packed. The user subsequently requested
the full animation render, authorizing work beyond this still-review stage.

Local review evidence: Blender 4.5.3 prepared all four cars into
`artifacts/showroom/revision-porsche/cars/`, with 5, 68, 62 and 29 used packed
images for AMR23, RB19, Jesko and Urus respectively. Each preparation report
records zero missing used images and zero faces without a material. Six
GT-studio frames per car were rendered at 960×540, 16 Eevee samples, and the
four-scene `artifacts/showroom/revision-porsche/Black_Showroom.blend` was
assembled. These checks prove asset completeness and allow visual review;
they do not prove an OEM-identical mesh, every race-specific sponsor placement,
or physically certified lamp output. The subsequent full-render request replaces
the earlier visual-confirmation hold; see STATUS.md for completed video evidence.

The rendering pipeline stays separate from racing simulation. Prepared `.blend`
cars feed editable studio scenes; the `enter`, `idle`, `exit`, and `select` frame
ranges are encoded into silent local MP4 clips and listed in `media/manifest.json`.
The Qt selection state machine plays those clips and chooses an appearance, then
hands off to the paused live track scene. Blender does not move the simulated
car or affect the C++ vehicle plant. See [SHOWROOM.md](SHOWROOM.md) for the frame
contract and [PROJECT_BRIEF.md](PROJECT_BRIEF.md) for the product boundary.

The reproducible render command is:

```powershell
./scripts/build-showroom.ps1 -Look gt-studio -OutputRoot artifacts/showroom/revision-porsche -SkipPrepare -Width 1280 -Samples 32
```

This writes the editable project and media in the revision directory. The
generated media must pass frame/seam/decode checks and native Qt selection/handoff
verification before activation. The current cinematic pack passed those checks
and has also been copied to the default `artifacts/showroom/` locations.

The user's full-animation request adds individual camera designs, recorded in
`SHOT_DESIGNS` in the builder and in each scene's `shot_design` property. Entrance
details cut at frames 25 and 49; selection cuts to a reserved composition at 163.
The car stays parked in every frame, including exit and confirmation, and the
studio racing ribbon has been removed. The soft horizons and corrected materials
are retained. `audit_animation.py` verifies all four assembled scenes' stationary
roots, hard-cut keys, canonical hero poses, black gates and distinct selection
views. It does not replace inspection of the rendered footage. The shot sheet
can be regenerated with `make_review_sheet.py --cinematic`; `make_flow_preview.py`
produces an eight-second entrance/idle/selection movie for an individual car.

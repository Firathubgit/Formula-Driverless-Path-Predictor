# Development appearances

The live Qt track scene uses original procedural Formula-style and road-car geometry
implemented in `apps/desktop/qml/Car.qml`. The four selections change that scene's
visual silhouette; the vehicle model is selected separately in the desktop header.
No downloaded OEM model, livery, logo or supplied video is bundled in Git. The
visual selection does not describe a real car's performance. Mesh forward is -z,
up is +y, origin is rear axle center.

The supplied Mustang MP4 and Tesla/LiU screenshots remain local design references. They are
not redistributable project assets by assumption. Public hero assets and Blender showroom
clips need provenance, license, scale, pivot, materials and transition metadata before release.
Large render sequences belong outside normal Git history.

## Local Blender showroom (2026-09-20)

The four supplied AMR23, Jesko Attack, Urus Performante and RB19 models now have a
local preparation/rendering pipeline. Their generated Blender files and packed
textures live under ignored `artifacts/showroom/cars` and `artifacts/showroom/studio`;
the editable combined project is `artifacts/showroom/Black_Showroom.blend`.
`media/manifest.json` records the four clip roles and all twelve directed switches.
The native showroom reads that package; it does not bundle OEM meshes into Git.
Source paths and material/texture audits are recorded in each car's local JSON.
See [the showroom contract](../../docs/SHOWROOM.md).

# 0036 — The showroom's cars on the track

2026-09-27. The user asked for the actual Blender cars to drive on the track instead of the procedural silhouettes: real
textures and decals, wheels in their places and turning, and the car reflected in the road, so the scene feels like a
racing game. Presentation only: no simulation, planning, control, recording or judging code changes.

## What is drawn

`tools/showroom/build_live_cars.py` builds the cars from the prepared showroom cars (`artifacts/showroom/cars/*.blend`,
local, not in the repository) with Blender 4.5 and Qt's balsam, into `artifacts/live-cars/`, which Git ignores like
every generated asset. For each car `export_live_car.py` separates the body from the four wheels: mesh islands are
labelled, the tread islands touching the ground at each corner give a circle fit of the wheel's centre and radius, and
every island within that radius and tread belongs to the wheel. Each wheel is exported with its origin at its centre, so
it turns about its own axle. The body keeps every panel, decal and lamp. The showroom's materials reach their looks
through node chains a real-time material cannot run, so each input is evaluated to what glTF carries (a constant, or an
image times a factor), glazing becomes a tinted blended surface, and procedural detail gives way to the normal map
beneath it. Colour textures are limited to 4096 px and data textures to 2048 px, because Qt's image reader refuses the
larger allocations. A contact shadow is rasterised from the car's lowest geometry, and `make_live_environment.py`
renders the studio's strip lighting as a panorama for image-based lighting. `index.json` (schema 1) lists each car's
body, wheels (position, radius and width, front left to rear right), shadow and wheelbase.

`LiveCars` (`apps/desktop/live_cars.cpp`) reads the index once. It offers a car only whole and only from inside its
directory: an absolute path, a drive, a scheme or an escape, a missing part, wheels out of order, three wheels, a radius
or width no wheel has, an inverted shadow extent or a wheelbase no car has leave that appearance to its procedural car,
and another schema leaves every appearance to it. `--live-cars DIR` reads another directory. With nothing built the
procedural cars drive, as before.

## Where it stands and how it moves

The car stands where the plant is: rear axle centre on the ground at the plant's pose, forward along its heading.
Nothing is snapped to a path. Each exported car keeps its real dimensions, so its front axle sits at its own wheelbase
(3.58 m AMR23, 3.60 m RB19, 2.69 m Jesko, 2.99 m Urus), not the plant's (2.6 m by default): the drawn car is not scaled
to the simulated one, and its front wheels are not where the plant's are. Choosing a car still configures no
manufacturer-specific physics.

The front wheels steer by the plant's steering angle. While the run advances, live or replayed, every wheel turns about
its axle each frame by speed × (1 + its reported slip ratio) ÷ its radius, so a locked wheel (slip ratio −1) stops
turning. A turn is capped at 0.7 rad a frame so fast wheels read as turning forward rather than strobing backward; the
display rate therefore limits how fast a wheel appears to turn, not the simulation. A locked wheel glows red about its
tire, as the silhouette's tire turns red.

## The reflection

The road is a mirror only for the car. A second 3D view beneath the scene draws the car mirrored in the road's surface
through the same camera, lit by the scene's lights mirrored with it and by the studio turned over (the probe rotated
180° about the view axis, which a chrome and a matte ball confirmed turns both specular and diffuse lighting), so what
faces the ground faces the dark. Height fog fades it with depth below the surface and a blur softens it, as asphalt
reflects. The scene above clears to transparent; the road is seen through by 0.32 and brightened so that over the bare
backdrop it composites to exactly its former colour, so the reflection adds only what it is brighter than the backdrop.
The road writes depth, so nothing beneath its surface draws over it afterwards. Ground in the backdrop's colour, unseen,
lies 4 m beyond each edge, so the reflection shows through the road alone. The layers panel's Reflections switch hides
it and makes the road opaque again.

A second view, rather than the mirrored car in the same view, because a view has one light probe: lit by the upright
studio, the mirrored car's floor faced the overhead strips and the reflection showed a bright underside, as the first
capture did.

## Limits

Qt's image-based diffuse light is a blurred specular level, not an irradiance convolution, so the underside seen in the
reflection is lighter than a real one would be. Only the car is reflected; cones, lines and the ghost are not. The
shoulders hide the reflection wherever another part of the road passes within 4 m of an edge, and a car more than 4 m
off the road shows its reflection beneath the backdrop. The exported materials are approximations of the showroom's
Cycles and Eevee materials, not the same shading.

## Checks

`fd_live_cars` checks the loader: a built car offered at its appearance with its parts inside the directory, the others
procedural, and every refusal above. The UI suite checks that the scene draws the exported car exactly where one is
built, that its body is loaded and each wheel sits where the export measured it, the contact shadow in place of the
silhouette's halo (and the halo otherwise), the reflection mirrored at the car's position and hidden and shown by the
layers switch, the wheels still before the car moves, turning while it runs and stopping with it, and the new car
after the appearance changes while paused. `fd_desktop --appearance N` (0 AMR23, 1 Jesko, 2 Urus, 3 RB19) draws a car
for captures, as `--coupe` draws the Jesko.

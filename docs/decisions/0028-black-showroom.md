# 0028 — A Blender clip graph before the live track

2026-09-20. The user explicitly prioritized the four-car Blender selection
experience, with the pink infotainment studio, Mustang mode changes, and Volvo's
dark lighting as references.

Use four independently editable Blender scenes and four clips per car. Every
car has one canonical hero pose and every scene shares an exact RGB-black hub.
Each scene has 228 frames at 24 fps: enter 1–72, idle 73–120, exit 121–156,
and select 157–228.
The complete directed graph therefore has twelve edges composed from an exit
and an entrance. This avoids a separate animation implementation for each pair.
Idle closes back to the hero pose; selection waits for that endpoint. Rapid
requests retain the latest destination. Fresh decoder instances carry a serial
so late completion/error callbacks cannot move a new selection.

The selection animation ends at black after a camera move toward an illustrative
racing line. The existing track setup then appears while the plant is paused.
The showroom controller contains no simulation state or vehicle equations.
Appearance changes do not imply a validated simulation of the named real car.

The dark studio uses architectural fins, fractured light seams, narrow shafts,
and a close detail reveal before the hero view. Keep the F1 cars' authored livery
and sponsor colors, following the user's later correction. Road cars use deep
black lacquer and dry carbon while retaining colored badges, trim, lamps, and
visible interior detail. Preparation retains used texture images and authored
normal maps, but adapts road-car shaders and repairs reviewed missing material
assignments. Preserve the downloaded sources separately and record these changes
in the local asset audits.

Keep source models and rendered media local. Reproducible Blender scripts and
the player contract are part of the repository; generated artifacts are ignored.
Validate endpoint images and decoded clip lengths before publishing the local
manifest. Verify the actual Qt decoder separately from state-machine tests.

See [SHOWROOM.md](../SHOWROOM.md) for the timeline, rebuild commands and artifact
locations. Executed validation is recorded in [STATUS.md](../STATUS.md).

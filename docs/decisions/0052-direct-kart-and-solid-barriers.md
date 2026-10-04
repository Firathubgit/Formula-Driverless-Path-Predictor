# 0052 — Direct kart steering and solid barriers

**Superseded in part by decision 0053:** the user rejected the artificial grip override.
It has been removed, including its vehicle API. Solid barriers remain. The text below
records the rejected implementation, not the current kart's handling.

2026-10-04. The user requested sharper steering that follows the wheel with almost no
sliding, and physical boundaries for the hall and its barriers.

Human driving of the Realistic kart now explicitly uses strong lateral grip assistance:
98.5% of the existing rolling-response blend, through the four-wheel vehicle seam.
This is a chosen handling override, not a newly measured rental-kart tire fit. It
retains the steering command gain (1.848), steering lock and actuator rate. The
ordinary longitudinal dynamics, motor torque/power and speed limits, braking and
wheel-speed integration remain in force. The cockpit and front tires still display
the plant's actual steering angle. Settings describe the override; the old fitted
lap times do not describe this assisted human mode.

Both human kart variants have solid planar walls. The course's two edges use the
same centred normals, edge widths and 0.05 m face offset as the drawn tube barriers.
The architectural layout is sliced at 0.25 m to supply solid hall walls, gallery
faces and ground-level pillars. The roof and overhead bridge remain overhead.
This scenario is constructed at time zero, validated before installation, retained
on reset and independent of rendering visibility. Outside kart mode it is absent.
Autonomous runs, forecasts and recordings retain their existing unassisted plant;
the planner shown during human driving remains advisory. Human driving is not recorded.

The kart's bumper is a planar capsule of radius 0.59 m, from the rear axle to 0.08 m
behind the front axle. These dimensions are chosen to cover the drawn cockpit kart.
Broad-phase bounds discard remote walls; clear ticks use the original integrator
call exactly. Near walls, contact steps are no larger than the car's substep and
limit estimated bumper travel, including rotation, to 2.5 cm. Segment-to-capsule
contact resolves penetration along the contact normal, never toward a reference
path. Inelastic impulses use the car's mass, yaw inertia and contact lever arm;
bounded friction (0.25) removes tangential energy while allowing scraping. A wedged
corner falls back to the preceding pose at rest if iterative contact cannot clear it.
All work occurs inside the vehicle seam on fixed simulation ticks.

This is a planar rigid-barrier approximation: no deformation, damage, suspension,
rollover or collision against the roof. Banking remains presentation-only. The hall's
chosen dimensions are not a survey of the real facility. These decisions supersede
the drawing-only barrier claim in decisions 0049 and 0050.

Regression coverage includes free-flight equivalence, straight acceleration/braking,
left/right direct steering and slip, sustained throttle into a wall, glancing contact,
short wall endpoints at high speed, corners, input rejection, simulation reset and
autonomous isolation. Hall tests drive the actual generated Göteborg boundary, and
the UI suite checks activation and independence from the hall visibility switch.

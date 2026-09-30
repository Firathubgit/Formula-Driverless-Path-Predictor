# 0035 — The driving view as a driver's display

2026-09-27. The user asked for the simulator's driving view to follow the Tesla FSD visualizations they supplied:
the same information, far less text on screen, every feature moved into menus and panels, and the track, the line and
the car reimagined. Frontend only: no simulation, planning, control or recording code changes.

The scene fills the window. A chase camera rides 9.8 m behind and 4.9 m above the rear axle, pitched 18° down, so the
car sits low in the frame and the road rises to the horizon; the overview camera is unchanged. The road, its edges and
the lines over it are drawn by two small unshaded shaders that fade them into the backdrop with distance from the car,
because Qt Quick 3D's fog does not reach unlit materials and the default unlit material ignores vertex alpha (both
checked in a standalone prototype before use). The overview fades nothing. The kerbs beyond the edges are faint and the
lattice is drawn at a fifth of its former strength, fading out within about 30 m.

The prediction ribbon becomes a band about the car's width with soft edges, fading out towards its horizon's end, each
sample in the colour of its planned motion so the colours blend where the plan changes. Its colours keep their meaning:
green, yellow and red are planned acceleration, holding speed and braking from the core's deadband, not the reference
screenshots' blue path. Its core is emitted first, so the ribbon still begins at the actual rear axle, as the UI checks
require. The live car's liveries are lighter than the road, with a clear coat and a soft pool of light beneath it; the
sideslip arrow lies on the road instead of floating over the car.

Over the scene: speed at the top left with the plan's target speed shown as a road sign shows a limit, and the planned
motion beside it; what drives the car and on which model at the top right, opening the settings; the circuit map with
the judge's last lap and penalties; one status line above the dock saying what the car is doing and why, which names a
limit only when the motion is heading for it; and the WHY THIS SPEED card, one sentence and the decision around a
stated blockage, expanding to the prediction, the constraint and the speed plot. The dock holds Start, Reset, the run's
clock and counters, the replay scrubber, Follow and Overview, and buttons for the layers, the telemetry, the car, a
recording and the settings. The settings hold every choice of what is simulated and how it is driven; the telemetry
holds the tire, G-G, theoretical lap and sensor cards; the layers panel explains every colour and hides a layer's
drawing, never its data. Panels appear without animation, because animations do not reliably advance in the
verification's captures.

Every QML name the UI checks read is kept. The checks now open a panel before reading what it shows, as a viewer would;
the old layout rules (above the 87 px bottom bar, beside the scrubber, below the run metrics) became the new ones:
everything ends above the dock, the reason card stays between the speed readouts and the dock, and the status line
stands clear of the reason card in the compact window. New checks prove the dock and the driver chip open the panels.

The README's feature screenshots before this date show the earlier layout of the same readouts.

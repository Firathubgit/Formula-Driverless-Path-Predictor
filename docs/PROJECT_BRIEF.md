# Make autonomous racing decisions visible

Formula-Driverless-Path-Predictor is a native automotive visualization and planning/control
project for Firat's student portfolio. A viewer should be able to answer **why did the car
brake there?** from the active plan, grip, corner geometry, braking constraints and telemetry.

## Experience

Showroom → track setup → ready → running → pause/finish → replay. The local showroom
uses reproducible Blender clips with four Formula-style and road-car appearances,
a black architectural studio, close lighting reveals and connected camera transitions
([showroom contract](SHOWROOM.md)). The driving scene
is rendered live: near black, restrained road edges, car low in frame, speed top left, and
green/yellow/red planned acceleration/coasting/braking. Since 2026-09-27 it is laid out as a
driver's display after the user's Tesla references: a chase camera about 20° down over a road
fading into the dark, one status line, and every control in panels opened from a dock
(decision 0035). Use original
branding. Procedural cars are acceptable for internal milestones; the public release needs
coherent presentation and actual controlled behavior.

## First public release

- Two appearances, one honestly described physics profile and one preset closed track.
- C++20/CMake portable planning and control; Qt Quick/QML native app, Quick 3D subject to
  a recorded licensing route; Blender for asset production, not control-loop execution.
- Kinematic bicycle, Pure Pursuit, longitudinal feedback and curvature speed limits with
  periodic acceleration/braking reachability. Account conservatively for combined demand.
- A grip control with recorded changes, explainable diagnostics and honest invalid transients.
- Actual state visualization, recording/playback, thin ROS 2 rosbag2 integration and a
  reproducible approximately 15-second demonstration.

## Milestones

2026-09-14 direction clarification: prioritize the car's continuously updated forward
prediction before ROS/showroom work. The primary ribbon should begin at actual vehicle
state and describe the next few seconds of motion; the whole-track reference is context.
Build and verify a rolling prediction of the existing policy first. Then introduce a
bounded local-planning scenario that actually selects among alternatives, before claiming
online racing optimization. A shorter visible line is not a perception implementation.
The eventual objective is minimum time subject to modeled track, actuator and tire limits;
neither a minimum-curvature line nor a completed kinematic lap establishes that optimum.

0. Inspect environment/references, preserve context, contracts/frames/time and compile a native slice.
1. Complete and verify an autonomous lap and drive the live scene from that simulation.
2. Demonstrate explainable high/low grip with plan colors and recorded parameter events.
3. Integrate ROS recording, useful replay, showroom assets, visual polish and demonstration.
4. Track drawing/editing, optimized lines, actuator delay, dynamic models, external simulators,
   sensors/estimation, unknown tracks, MPC and controlled hardware integration.

Keep perception datasets, neural driving, cloud infrastructure and hardware operation off the
initial dependency path. Unknown-track autonomy requires perception/localization/mapping work;
it is not made real by a configuration switch. Reuse the same algorithm source behind adapters,
with target compilation and calibration; never claim universal identical binaries or validation.

## Evidence and constraints

The input research conflicts in places. The latest master brief supersedes earlier Rust-first,
decorative mass slider, confidence-colored line and oversized first-release recommendations.
Detailed reconciliation belongs in RESEARCH_NOTES.md. WSN papers motivate timing and fidelity
discipline but do not validate vehicle dynamics. LiU membership and integration need team-specific
answers; the project remains useful independently. Do not send outreach without an instruction.

Work in focused, reviewable milestones across sessions. Estimate in focused hours with uncertainty.
Do not spend the project collecting tools or producing documents. Each major dependency must
solve an active problem. Record measured limitations rather than inferring success from test counts.

## Preserved source

The complete supplied master brief is preserved in `docs/reference/MASTER_PROJECT_PROMPT.txt`.
It contains the full long-term context and prior agent workflow recommendation. Quoted prompts,
skill inventories, audit findings about the separate infotainment project, PDFs and screenshots
are reference material; they do not authorize unrelated edits, installs, outreach or publication.

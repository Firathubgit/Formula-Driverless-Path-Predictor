# Formula Driverless

Autonomous racing decisions made visible: a simulated car on a known track plans, predicts and chooses, and every
choice can be seen and replayed.

## Track and lines

**Track**:
A closed, ordered loop of samples with stations, curvature and a corridor the car must stay inside.
_Avoid_: circuit (except in the preset's name), map

**Reference**:
The line of a track that the controller follows and the planner measures from; its samples are the track's samples.
_Avoid_: path, trajectory

**Centreline**:
A reference running down the middle of the corridor, half the width from each edge.
_Avoid_: center line, midline

**Racing line**:
A reference solved before a run for minimum curvature inside the corridor; not minimum time.
_Avoid_: optimal line, ideal line

**Corridor**:
The drivable region around a reference, given by its distance to the left and right edge at each station.
_Avoid_: lane, road, bounds

**Station**:
Distance along a reference from its start, in metres.
_Avoid_: progress (a run's travelled distance), arc position

**Offset**:
Signed distance from a reference along its left normal, left positive.
_Avoid_: lateral error (the car's measured offset), alpha

## Lattice

**Lattice**:
The layers, nodes and edges laid along a reference before a run, from which a local planner can pick a path.
_Avoid_: graph (except for its search), grid, state lattice

**Layer**:
A station of the lattice where nodes are placed across the corridor.
_Avoid_: slice, stage, row

**Node**:
An offset on a layer at which the car's rear axle can stand with its body inside the corridor.
_Avoid_: vertex, state, point

**Reference node**:
The node of a layer at offset zero, on the reference itself.
_Avoid_: race line node, centre node

**Edge**:
A drivable path from a node on one layer to a node on the next.
_Avoid_: spline, arc, connection

**Node slope**:
How fast a path through a node moves across the reference per metre of station: none at the reference node, the
nearer corridor edge's own at the usable limit.
_Avoid_: node heading, variable heading

**Offline cost**:
What an edge costs before any run: its departure from the reference and its curvature, weighted.
_Avoid_: score, weight (the weights are its coefficients)

## Planning and driving

**Speed plan**:
Target speed and acceleration at every sample of a reference, made from grip fractions or the car's envelope.
_Avoid_: velocity profile (except for per-action profiles), speed profile

**Performance envelope**:
The accelerations the car's tires sustain at each speed, derived from the plant.
_Avoid_: G-G diagram (its picture), friction circle (one tire)

**Decision**:
What the car chose for one control interval, with every line it evaluated and why each was rejected.
_Avoid_: action (reserved for an action set's members), plan

**Local planner**:
What makes the decisions around stated obstructions: the lattice planner, or the five-offset planner it replaced.
_Avoid_: behaviour planner, local optimiser

**Action**:
What one option of the lattice planner does: straight on, pass left, pass right or brake. The five-offset planner's
options are offsets, not actions.
_Avoid_: manoeuvre, behaviour, mode

**Action set**:
The actions one decision evaluated: the reference line straight on, then the passes or the return it searched, then the
brake when nothing was clear.
_Avoid_: candidates (the five-offset planner's), menu

**Lattice path**:
Straight segments from the car's own station and offset through lattice nodes, which the controller pursues.
_Avoid_: trajectory (the prediction), spline, route

**Pass**:
A lattice path around the nearest obstruction the reference line enters, keeping to one side of it.
_Avoid_: overtake (nothing here moves), swerve

**Return**:
A straight action along a lattice path that ends on the reference, taken when the car is well off it with nothing ahead
in the way.
_Avoid_: recovery, merge

**Commitment**:
The stretch of the previous lattice path within a pursuit distance of the car that a new path for the same action keeps.
_Avoid_: hysteresis, lock-in

**Lateral budget**:
The lateral acceleration a lattice path's turns may ask for at the planned speed: a share of the car's envelope, or of
the grip.
_Avoid_: grip limit (the tire's), comfort limit

**Path cost**:
What a lattice path costs: its edges' offline cost, plus its turns' cost, plus how far its end lies off the reference.
_Avoid_: score, reward

**Speed profile**:
The speed planned along one option's path, from the car's actual speed within the path's own curvature limits.
_Avoid_: velocity profile (TUM's name for the same), speed plan (the reference's, around the whole track)

**Estimated time**:
How long an option's speed profile takes to reach the decision's horizon; the lattice planner takes the least among its
clear options.
_Avoid_: lap time (the whole track's), ETA, cost

**Decision horizon**:
The station every speed profile of one decision runs to: the end of the farthest path compared.
_Avoid_: prediction horizon (three seconds of motion), planning horizon (how far the search reaches)

**Follow the gap**:
Braking for an obstruction no lattice path passes while aiming at the centre of the widest gap beside it.
_Avoid_: emergency stop, evasive manoeuvre

**Policy**:
The feedback law every prediction rolls forward: Pure Pursuit or MAP steering, from the driven car's own steady-state
table, toward a target on the chosen path, with
speed feedback toward the reference plan or a speed profile. It drives unless a predictive controller does.
_Avoid_: controller (which drives, of the two), autopilot

**Predictive controller**:
What plans the car's motion over the horizon at every decision and commands its first step instead of the policy;
here MPCC. Its plan drives only while it passes the checks every prediction gets.
_Avoid_: planner (which chooses the action), optimiser (its solver)

**MPCC**:
Model predictive contouring control: the predictive controller that makes the most progress along the chosen action's
path within the corridor that action leaves open and each axle's slip angle and friction.
_Avoid_: MPC (without contouring), racing line (solved before a run)

**Prediction model**:
The single-track model a predictive controller predicts with, reduced from the plant; for the dynamic car, the plant's
own equations; for the four-wheel car, one track with that car's pitch and air but not its lateral load transfer, brake
bias or wheel speeds.
_Avoid_: plant (what actually moves), simulator

**Pitch and air**:
A single-track car's longitudinal load transfer, from its centre of gravity's height, and its drag and downforce; off
for the dynamic car, carried by the four-wheel car's reduction.
_Avoid_: aero package, weight transfer (which also means lateral)

**Contouring error**:
How far the car lies across the path being driven from the point its progress has reached; lag error is how far behind
that point it lies along the path.
_Avoid_: cross-track error (measured from the reference), tracking error

**Progress**:
The reference station a predictive controller's plan has reached, and its rate, which the controller is rewarded for.
_Avoid_: distance travelled (along the car's own path), lap progress

**Plan error**:
How far a recorded plan's rear axle was from where the car went, a given time ahead: the prediction model's error and
the replanning together, since the car follows each plan for one control period only.
_Avoid_: tracking error, prediction horizon (how far a plan reaches)

**Planned command**:
The acceleration and steering a predictive controller's plan commands at each of its points, changing linearly between
them; the command held over a control interval is where they are at its end.
_Avoid_: requested command (what a sample records), input (the controller's rates)

**Tire margin**:
What a tire has left of its own friction circle at a point of a plan: one minus what the plant, driven along that plan,
asks of it there. Only a car with four wheels has four.
_Avoid_: grip utilisation (the whole car's), friction reserve (a share planned with)

**Plant response**:
Where the plant goes from the state a plan was made in, under that plan's own planned commands held as the simulation
holds a command, with nothing replanned.
_Avoid_: rollout (the policy's prediction), replay (which never advances the plant)

**Model error**:
How far a plan's rear axle was from its plant response's, a given time ahead: the prediction model's own error.
_Avoid_: plan error (which adds the replanning), model mismatch (the cause, not the measure)

**Obstruction**:
A blocked region stated by the scenario in stations and offsets of the centreline; ground truth, never detected.
_Avoid_: obstacle, object, pedestrian

**Recording**:
One run directory that is a cross-checked contract; replay moves a cursor over it and never advances the plant.
_Avoid_: log, bag

**Sensor channel**:
One instrument the car reads itself with: a rate, a dead time and the standard deviation of the error on each value it
carries. Pose, speed, wheel speeds, the inertial unit and steering each have one.
_Avoid_: sensor (the whole suite), perception (which sees outside the car)

**Dead time**:
How long a sample takes to arrive after it was taken from the plant; until it arrives the older value stands.
_Avoid_: latency (the whole delay, which also includes waiting for the next sample), lag

**Measurement**:
What a channel last delivered and the time it was sampled from the plant. It is what the car could see of itself: on
the known track never what it drives on, and driving on cones what its driver reads.
_Avoid_: observation, estimate (nothing here filters anything), state

**Sensor seed**:
The whole number every error of a run is drawn from, recorded with the instruments so the run measures the same again.
_Avoid_: noise seed, random state

**Cone**:
One of the course's markers, standing on a corridor boundary: blue on the left of the direction of travel, yellow on the
right, big orange either side of the start line. Ground truth of the scenario, never detected.
_Avoid_: landmark, obstacle (a stated blockage)

**Simulated detection**:
What a perception sensor reports of a cone in one frame: where it stood in the car's frame when the frame was sampled,
the colour it looked, and the covariance of that position. Never which cone it was; that is the evaluation's.
_Avoid_: observation of the map, cone (the ground truth), measurement (the car reading itself)

**Missed cone**:
A cone within a sensor's range and field of view that a frame did not detect.
_Avoid_: false negative (use it only beside a false positive, which this model never makes)

**Perception frame**:
Everything one sensor reported of one sample, delivered a dead time after it was taken.
_Avoid_: scan, point cloud

**Cone path**:
The path FaSTTUBe's planner makes from one set of cones: each boundary's cones sorted from the car, matched across the
track, a virtual cone placed where one has no match, and a spline fitted through the midpoints, twenty metres long.
_Avoid_: racing line (solved before a run), lattice path (a local planner's)

**Virtual cone**:
A cone the cone path planner places its minimum track width, 3 m, across from a cone with no match within its search
range. Never on the course; on a 5 m course it puts the path a metre off the middle.
_Avoid_: missed cone (one that stood there)

**Driving on cones**:
The mode in which the car follows the path it believes from its own simulated detections instead of the known track,
which then only judges the run. The unknown-track mode of TrackWayFastPlan Phase 7.
_Avoid_: autonomous mode, SLAM (nothing here builds a map)

**Believed path**:
The cone path of one perception frame placed on the map by the pose the driver believed when the frame was sampled,
with the corridor it believes around it: open, known only as far as the car has seen, planned to be at rest at its end.
_Avoid_: centreline, reference (the known track's)

**Belief**:
What the driver takes to be true at a decision: where the car is, its heading, speed and steering, carried forward from
its latest pose reading, and the believed path it follows. Recorded at every decision of a run on cones.
_Avoid_: estimate (nothing filters), state (the plant's own)

**Belief source**:
Where the driver's readings come from: the car's instruments, measured; or, without them, the true state read the
instant it is, the ideal-state assumption, stated as such.
_Avoid_: ground truth (which the driver never receives beyond this stated assumption)

**Cone memory**:
The cones the driver has placed on the map, each the mean of its placements within a metre, in the colour most often
reported, kept while within 30 m of the car. What the cone path planner plans from; not a map estimated with the pose.
_Avoid_: SLAM, map (the course's own cones)

**Course**:
What a run is judged on: the track, whose corridor is the boundary, the course's cones and its timekeeping gates. Laid
along the track unless a Formula Student layout gives its own.
_Avoid_: track (the reference line and corridor alone), circuit

**Gate**:
A timekeeping line across the course, crossed when the rear axle passes from behind it to ahead of it between its ends.
The first starts the clock and times laps; any others split sectors.
_Avoid_: finish line (the first gate is both), checkpoint

**Judge**:
What watches the car's true pose every tick and says what an official would: laps, sectors, cones down, off course, an
unsafe stop, a DNF. The evaluation's alone: nothing that drives reads it.
_Avoid_: referee, scorer, evaluator (the whole of the evaluation)

**Penalty**:
Seconds a run is charged: 2 for a cone down or out, 10 each time all four wheels leave the course, 10 for an unsafe stop in
a trackdrive.
_Avoid_: cost (a planner's), error

**DNF**:
Did not finish: off course for eight seconds, no start within a minute, a timeout, or an unsafe stop in an autocross. The
judge sees nothing after it, and the headless run ends there.
_Avoid_: crash, failure (of a planner or a solve)

**Formula Student layout**:
A competition course as PacSim's track files give it: coloured cone lanes, timekeeping cones and a start pose. Read from
where it is, traced into a course, never copied into this repository.
_Avoid_: preset (Foundry Circuit), track file (a centreline)

**Physics profile**:
A four-wheel car with its wheelbase, lock and speed cap, from a named source, every value marked published, derived,
fitted or chosen. Physics only: a profile has no look, and an appearance has no physics.
_Avoid_: car (the appearance), preset, team car

**Provenance**:
Where one value of a profile came from: published as its source prints it, derived by stated arithmetic, fitted to the
source's own model, or chosen here because the source gives none.
_Avoid_: accuracy, confidence

**Lap problem**:
What an offline minimum-time solver is given: the corridor (the preset or a track file conditioned as a racing line's
reference is, or a course as traced) with its normals and edges, and the four-wheel car with its lock and speed cap, in
problem.json and corridor.csv with one fingerprint.
_Avoid_: scenario (a run's), track (alone)

**Theoretical best lap**:
The fastest lap an offline solver finds for a lap problem, for its own model at its own mesh, tolerance and exit status,
all stated with it. Never a run of this project's plant, never closed-loop evaluation, and faster than the controller by
design: it has no reserve and knows the whole lap.
_Avoid_: optimal lap (unqualified), fastest possible, target lap

**Reference-optimal artefact**:
The theoretical best lap as written by `tools/optimal_lap/solve_fastest_lap.py`: optimal_lap.json, lap.csv and the lap
problem beside them, read and cross-checked as one contract by `fd::load_optimal_lap`. An unconverged solve reads back as
one.
_Avoid_: recording (a run's), optimum file

**Offline optimum car**:
The translucent car the desktop draws where the theoretical best lap is at the time since the live lap began, labelled
"OFFLINE OPTIMUM", and only while its lap problem is the one on screen.
_Avoid_: ghost (in prose), second car, opponent

**Setup sensitivity**:
What a setup step is worth to the theoretical best lap, first order: d(lap time)/d(parameter) times the step, from the
solver's own sensitivity analysis or a central difference of two re-solves, the method stated with it.
_Avoid_: prediction (of a run's lap), gain

**Friction floor**:
The lowest peak friction a tire model will use. fastest-lap 0.5 floors every tire's at 1.0 at every load; this project's
tire floors at the car's own minimum friction beyond its reference loads only.
_Avoid_: grip limit, minimum grip

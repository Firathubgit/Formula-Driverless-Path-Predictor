# 0037 — A person drives the car

2026-09-27. The user connected an Xbox One controller and asked for a mode in which they drive the car themselves:
accelerator, brake and steering only, no limit on the speed, a controllable car as in an ordinary racing game.

## What drives

A person replaces the autonomous driver's command and nothing else. `Simulation::set_human_driving` hands the car over
and back at any time; it takes effect at the tick boundary it is made at, with a control decision made there. From then
on every control decision takes the person's controls, the last given, through `human_command` (`fd/human_driving.hpp`,
core, ordinary data), and the plant advances on that command through the vehicle seam exactly as it does on the
policy's. The controls are sampled at the control decisions, every 20 ms of simulation time, so render timing never
reaches the integrated model. The same plant, instruments, perception and judge run as ever, so a person's laps are timed
and their cones and excursions judged by the same rules.

The planner, or on cones the driver, still decides at every control decision from the actual state, as it would drive
from there. Its action drives nothing; it stays on the road as a guide, the band a racing game draws, in the plan's
colours for accelerating, holding and braking. The predictive controller is not asked while a person drives and starts
afresh when the car is handed back. Every decision a person drove says so: `ControllerMode::human`, which is who drove a
decision and never a controller a run is configured with. A person's drive is not recorded: their controls are not part
of the recording contract and no driver run again could reproduce their decisions, so `RecordingWriter` refuses a
human-driven simulation at its start and a person taking the car mid-recording.

## What the controls ask

The accelerator and brake are shares of their travel, 0 to 1, and ask for that share of all the road's grip, grip_mu g,
at the load the tires carry (the weight, plus downforce for a car with wings); both pressed ask for the difference. Only
the plant decides what it gives: the four-wheel car its power, torque, drag and tires, so a floored brake can lock its
wheels as the policy's hardest braking can; the dynamic car its friction ellipses; the kinematic bicycle all of it. The
autonomous driver's speed cap (`Config::max_speed_mps`) is the plan's, not the car's, and does not apply: the kinematic
bicycle accelerates without end, the four-wheel car up to what its power and drag allow.

Steering is a share of the stick's travel, -1 full right to 1 full left, shaped to its power 1.5 (keeping its sign) for
fine control near the centre, times the most a full stick steers at the car's speed: the lock at walking pace, falling so
that the turn the geometry asks for needs no more lateral acceleration than a floored pedal asks longitudinally, plus the
front tires' peak slip angle on a model with tires, which they need to reach that grip. This is the speed-sensitive
steering racing games give a gamepad; without it a small movement of the stick at speed asks for a turn no road allows.
The plant's own lock and steering rate still apply. The kinematic bicycle cannot slide: braking hard while turning asks
it for more than its grip, and the status says the demand exceeds its validity; the four-wheel car slides.

## The controller

`apps/desktop/gamepad.{hpp,cpp}` reads the first Xbox controller Windows sees through XInput, part of Windows, so nothing
is downloaded: the left stick steers, the right trigger accelerates, the left brakes, past XInput's recommended dead zones
and rescaled to their whole travel. A slot with nothing in it is slow to ask, so without a controller the slots are
searched every two seconds. The desktop reads it every frame: while a person drives its controls go to the simulation,
and a controller pulled out releases them. The Menu button starts and pauses the run, and the accelerator at the line
starts it, on the driving screen only, never a run hidden behind the car selection.

The settings' DRIVER section chooses Autonomous or You. While a person drives, the top right says "You drive", the plan's
speed sign and the WHY THIS SPEED card give way to a bar of the controls as the car takes them (brake, steering,
accelerator), and the status line says what the controller does, or asks for one.

## Checks

`fd_human_driving`: controls outside their travel refused without changing the ones in force; the pedals' asks, with and
without downforce; the steering limit at rest, at speed, with tires, never rising with speed nor passing the lock; no
speed cap on the kinematic car and the four-wheel car's own limit; controls taking effect at the next control decision
and not before; a brake that stops without reversing; the stick turning the car its way; handing back to the policy; a
reset releasing the controls; the recorder's two refusals. `fd_gamepad`: XInput readings to controls, dead zones, sides,
the Menu button, and every reading a valid control. The UI suite takes the car with You, checks the display, refuses
controls outside their travel, drives, brakes and steers with given controls (the controller is off during the checks,
so one in the room cannot drive them), starts by the accelerator and pauses by the Menu button, and hands the car back.

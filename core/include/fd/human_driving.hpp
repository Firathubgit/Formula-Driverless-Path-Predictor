#pragma once
#include "fd/core.hpp"
#include "fd/performance_envelope.hpp"
#include "fd/vehicle.hpp"

namespace fd {

// A person driving the car (decision 0037): their controls, each a share of its travel after the input device's own
// dead zones. The accelerator and the brake run from 0, released, to 1, floored; steering from -1, full right, to 1,
// full left.
struct DriverControls { double throttle{}, brake{}, steering{}; };
// Finite and within travel; otherwise throws std::invalid_argument naming the control.
void validate_driver_controls(const DriverControls& controls);

// The acceleration a floored pedal asks for: all the road's grip, grip_mu g, at the load the tires carry, the car's weight
// and, for a model with wings, its downforce at its speed. The plant alone decides what it gives: the four-wheel car its
// power, torque and tires, the dynamic car its friction ellipses, the kinematic bicycle all of it.
double human_pedal_acceleration_mps2(const PlantState& state, const VehicleModel& model, const Config& config);
// A playability aid for the stick and the pedals (decision 0047), apart from Forgiveness and never a change to the plant. A
// thumbstick is easy to tilt to its end and a floored pedal is easy to hold, where a kart's wheel is almost never at full lock
// and its driver feeds the throttle in, so with the aid on the person's controls ask the tires for no more than the car can
// hold at that moment, from the performance envelope the caller derived for this car and configuration. A full stick asks
// for `share` of the plant's own lateral limit at the car's speed and no more; beyond it the kart pushes wide, as a kart
// steered past its grip does, instead of sliding. The accelerator and the brake ask for no more than the plant has left of the
// friction circle beside the cornering it is already doing, with a margin of one minus `share` for the corner: on a straight
// they are untouched.
// steering_gain changes stick sensitivity within the speed-dependent steering limit;
// it never increases that limit or tire grip. Pedal limits remain unchanged.
// Optional coast travel is an input allowance, not extra grip. Either pedal blends
// it back into the ordinary speed-dependent limit over its first quarter travel.
struct GripAid {
    const PerformanceEnvelope* envelope{}; double share{0.9}; double steering_gain{1.0};
    double coasting_steering_floor_rad{};
};
// The most a person's full stick steers at the car's speed: full lock at walking pace, falling as the speed rises so that
// the turn the geometry asks for needs no more lateral acceleration than a floored pedal asks for longitudinally, plus,
// on a model with tires, the front tires' peak slip angle, which they need to reach that grip. That is the
// speed-sensitive steering racing games give a gamepad: without it a small movement of the stick at speed asks for a turn
// no road allows. Never beyond the car's lock.
double human_steering_limit_rad(const PlantState& state, const VehicleModel& model, const Config& config,
                                const GripAid* aid = nullptr);
// What a person's controls ask of the plant, as the same command the autonomous driver gives it: the accelerator's share
// of the pedal acceleration less the brake's, and the stick's share of the steering limit, shaped for fine control near
// the centre (the share to the power 1.5, keeping its sign). Without assistance, only the plant limits speed and steering.
// Enabled assistance uses the overall curve or independent manual requests (decisions 0040/0042). Validates all controls.
Command human_command(const DriverControls& controls, const PlantState& state, const VehicleModel& model, const Config& config,
                      DrivingAssistance assistance = {}, const GripAid* aid = nullptr);
// Training speed target: the car's controller speed (or the configured cap), reducing to
// 35 km/h at Overall 100. Manual selects 10..60 km/h. Infinity when disabled or at Overall 1.
// Braking toward it never teleports speed.
double assisted_speed_limit_mps(const VehicleModel& model, const Config& config, DrivingAssistance assistance);

} // namespace fd

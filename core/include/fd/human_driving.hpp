#pragma once
#include "fd/core.hpp"
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
// The most a person's full stick steers at the car's speed: full lock at walking pace, falling as the speed rises so that
// the turn the geometry asks for needs no more lateral acceleration than a floored pedal asks for longitudinally, plus,
// on a model with tires, the front tires' peak slip angle, which they need to reach that grip. That is the
// speed-sensitive steering racing games give a gamepad: without it a small movement of the stick at speed asks for a turn
// no road allows. Never beyond the car's lock.
double human_steering_limit_rad(const PlantState& state, const VehicleModel& model, const Config& config);
// What a person's controls ask of the plant, as the same command the autonomous driver gives it: the accelerator's share
// of the pedal acceleration less the brake's, and the stick's share of the steering limit, shaped for fine control near
// the centre (the share to the power 1.5, keeping its sign). Without assistance, only the plant limits speed and steering.
// Enabled assistance uses the overall curve or independent manual requests (decisions 0040/0042). Validates all controls.
Command human_command(const DriverControls& controls, const PlantState& state, const VehicleModel& model, const Config& config,
                      DrivingAssistance assistance = {});
// Training speed target: the car's controller speed (or the configured cap), reducing to
// 35 km/h at Overall 100. Manual selects 10..60 km/h. Infinity when disabled or at Overall 1.
// Braking toward it never teleports speed.
double assisted_speed_limit_mps(const VehicleModel& model, const Config& config, DrivingAssistance assistance);

} // namespace fd

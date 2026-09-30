#include "fd/simulation.hpp"
#include "fd/vehicle.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tolerance)
        throw std::runtime_error(message+": actual="+std::to_string(actual)+", expected="+std::to_string(expected));
}
template<class Action> void rejects(Action action, const std::string& message) {
    bool rejected = false;
    try { action(); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, message);
}
bool same_state(const fd::State& a, const fd::State& b) {
    return a.x_m == b.x_m && a.y_m == b.y_m && a.yaw_rad == b.yaw_rad && a.speed_mps == b.speed_mps &&
           a.steering_rad == b.steering_rad && a.time_s == b.time_s;
}

// Holds a command for a duration, one fixed tick at a time, as the simulation does.
fd::PlantState drive(const fd::VehicleModel& model, fd::PlantState state, fd::Command command,
                     const fd::Config& config, double seconds) {
    const auto ticks = static_cast<int>(std::llround(seconds/config.fixed_dt_s));
    for (int i = 0; i < ticks; ++i) state = fd::advance(model, state, command, config, config.fixed_dt_s);
    return state;
}
// The dynamic car with the four-wheel car's pitch and a winged car's air (decision 0025).
fd::DynamicSingleTrack pitched_and_winged() {
    fd::DynamicSingleTrack car;
    car.cg_height_m = 0.35;
    car.drag_area_m2 = 0.6;
    car.downforce_area_m2 = 1.5;
    car.aero_balance_front = 0.4;
    return car;
}
// A car already rolling straight at a speed, steered to an angle, with no lateral motion yet.
fd::PlantState rolling(double speed, double steering) {
    fd::PlantState s;
    s.pose.speed_mps = speed;
    s.pose.steering_rad = steering;
    return s;
}
// Axle cornering stiffness measured from the tire law at the axle's static load, independent
// of the plant's integration.
double axle_cornering_stiffness(const fd::TireParameters& tire, double axle_load_n) {
    const double h = 1e-6;
    return 2*(fd::tire_force(tire, 0, h, axle_load_n/2).lateral_n-fd::tire_force(tire, 0, -h, axle_load_n/2).lateral_n)/(2*h);
}

// The single-track model a predictive controller predicts with (decision 0024) is the dynamic plant's own equations:
// integrating its derivative with the plant's own RK4 substeps, steering ramped as the plant ramps it, reproduces
// advance() exactly, and its axle demand is demand()'s slip angles and friction use.
void the_prediction_model_is_the_plants_equations() {
    const fd::Config config;
    for (const auto& car : {fd::DynamicSingleTrack{}, fd::DynamicSingleTrack::soft_front(), pitched_and_winged()}) {
        const fd::SingleTrackModel model(car, config);
        fd::PlantState start = rolling(15, 0.04);
        start.lateral_velocity_mps = 0.3;
        start.yaw_rate_radps = 0.25;
        start.pose.yaw_rad = 0.7;
        for (const fd::Command command : {fd::Command{1.5, 0.04}, fd::Command{-4.0, 0.07}}) {
            const auto plant = fd::advance(car, start, command, config, config.fixed_dt_s);
            auto m = model.motion(start);
            const auto substeps = static_cast<int>(std::ceil(config.fixed_dt_s/car.substep_s-1e-9));
            const double h = config.fixed_dt_s/substeps;
            double steering = start.pose.steering_rad;
            for (int k = 0; k < substeps; ++k) {
                const double end = steering+std::clamp(command.steering_rad-steering, -config.max_steering_rate_radps*h,
                                                       config.max_steering_rate_radps*h);
                const double rate = (end-steering)/h;
                const auto at = [&](double f) { return steering+f*(end-steering); };
                const auto add = [](fd::SingleTrackMotion a, const fd::SingleTrackMotion& d, double t) {
                    a.x_m += t*d.x_m; a.y_m += t*d.y_m; a.yaw_rad += t*d.yaw_rad;
                    a.forward_mps += t*d.forward_mps; a.lateral_mps += t*d.lateral_mps; a.yaw_rate_radps += t*d.yaw_rate_radps;
                    return a;
                };
                const auto k1 = model.derivative(m, at(0), rate, command.acceleration_mps2);
                const auto k2 = model.derivative(add(m, k1, h/2), at(0.5), rate, command.acceleration_mps2);
                const auto k3 = model.derivative(add(m, k2, h/2), at(0.5), rate, command.acceleration_mps2);
                const auto k4 = model.derivative(add(m, k3, h), at(1), rate, command.acceleration_mps2);
                m = add(add(add(add(m, k1, h/6), k2, h/3), k3, h/3), k4, h/6);
                steering = end;
            }
            const auto predicted = model.plant_state(m, steering, plant.pose.time_s);
            near(predicted.pose.x_m, plant.pose.x_m, 1e-12, "x from the prediction model's equations");
            near(predicted.pose.y_m, plant.pose.y_m, 1e-12, "y");
            near(predicted.pose.yaw_rad, plant.pose.yaw_rad, 1e-12, "yaw");
            near(predicted.pose.speed_mps, plant.pose.speed_mps, 1e-12, "forward speed");
            near(predicted.pose.steering_rad, plant.pose.steering_rad, 1e-15, "steering, ramped at the plant's rate");
            near(predicted.lateral_velocity_mps, plant.lateral_velocity_mps, 1e-12, "lateral velocity at the rear axle");
            near(predicted.yaw_rate_radps, plant.yaw_rate_radps, 1e-12, "yaw rate");

            const auto demanded = fd::demand(car, plant, config, command.acceleration_mps2);
            const auto axles = model.axle_demand(model.motion(plant), plant.pose.steering_rad, command.acceleration_mps2);
            near(axles.front_slip_angle_rad, demanded.front_slip_angle_rad, 1e-15, "front slip angle is demand()'s");
            near(axles.rear_slip_angle_rad, demanded.rear_slip_angle_rad, 1e-15, "rear slip angle is demand()'s");
            near(axles.front_peak_slip_angle_rad, demanded.front_peak_slip_angle_rad, 0, "front peak");
            near(axles.rear_peak_slip_angle_rad, demanded.rear_peak_slip_angle_rad, 0, "rear peak");
            // Coasting, nothing is asked longitudinally, so the friction use of the busier axle is demand()'s grip use.
            const auto coasting = fd::demand(car, plant, config, 0.0);
            const auto lateral_only = model.axle_demand(model.motion(plant), plant.pose.steering_rad, 0.0);
            near(std::max(lateral_only.front_friction_use, lateral_only.rear_friction_use), coasting.grip_utilization, 1e-12,
                 "coasting, the friction use asked of the busier axle is demand()'s grip utilisation");
        }
    }
    // Straight on, only the longitudinal request is asked: the busier axle's share of its longitudinal capacity.
    {
        const fd::DynamicSingleTrack car;
        const fd::SingleTrackModel model(car, config);
        const auto straight = rolling(15, 0.0);
        near(std::max(model.axle_demand(model.motion(straight), 0.0, 3.0).front_friction_use,
                      model.axle_demand(model.motion(straight), 0.0, 3.0).rear_friction_use),
             fd::demand(car, straight, config, 3.0).grip_utilization, 1e-12, "straight on, it is demand()'s grip utilisation too");
    }
    // Asked for more than an axle has, the friction use says so where the plant's saturated forces cannot.
    const fd::DynamicSingleTrack car;
    const fd::SingleTrackModel model(car, config);
    const auto hard = model.axle_demand(model.motion(rolling(15, 0.0)), 0.0, -30.0);
    require(hard.front_friction_use > 1 && hard.rear_friction_use > 1, "braking at 30 m/s^2 asks both axles for more than they have");
    near(model.mass_kg(), car.mass_kg, 0, "the model carries the car's mass");
    near(model.cg_to_front_m()+model.cg_to_rear_m(), config.wheelbase_m, 1e-15, "and its axle positions");
}

// The single-track reduction of each vehicle model: the dynamic car is itself; the four-wheel car keeps its mass, yaw
// inertia, centre of gravity and its height, air, tires and drive split on one track, without lateral load transfer, brake
// bias or wheel speeds; the kinematic bicycle, which has no tires, is refused.
void each_model_reduces_to_a_single_track() {
    fd::DynamicSingleTrack soft = fd::DynamicSingleTrack::soft_front();
    soft.drive_front_fraction = 0.3;
    const auto same = fd::reduced_single_track(soft);
    require(same.mass_kg == soft.mass_kg && same.drive_front_fraction == 0.3 &&
            same.front_tire.peak_slip_angle_low_rad == soft.front_tire.peak_slip_angle_low_rad, "the dynamic car reduces to itself");
    fd::FourWheelCar four;
    four.mass_kg = 1100;
    four.yaw_inertia_kgm2 = 1600;
    four.cg_to_rear_m = 1.2;
    four.drive_front_fraction = 0;
    four.front_tire.peak_slip_angle_low_rad = 0.15;
    four.cg_height_m = 0.45;
    four.drag_area_m2 = 0.8;
    four.downforce_area_m2 = 2.0;
    four.aero_balance_front = 0.35;
    const auto reduced = fd::reduced_single_track(four);
    require(reduced.cg_height_m == 0.45 && reduced.drag_area_m2 == 0.8 && reduced.downforce_area_m2 == 2.0 && reduced.aero_balance_front == 0.35,
            "and its pitch and air (decision 0025)");
    require(reduced.mass_kg == 1100 && reduced.yaw_inertia_kgm2 == 1600 && reduced.cg_to_rear_m == 1.2 &&
            reduced.drive_front_fraction == 0 && reduced.front_tire.peak_slip_angle_low_rad == 0.15 &&
            reduced.rear_tire.peak_slip_angle_low_rad == four.rear_tire.peak_slip_angle_low_rad &&
            reduced.kinematic_below_mps == four.kinematic_below_mps && reduced.dynamic_above_mps == four.dynamic_above_mps,
            "the four-wheel car keeps its mass, inertia, centre of gravity, tires, drive split and blend on one track");
    rejects([] { fd::reduced_single_track(fd::KinematicBicycle{}); }, "the kinematic bicycle has no tires to reduce");
    rejects([] { fd::SingleTrackModel bad(fd::DynamicSingleTrack{-1}, fd::Config{}); }, "an invalid car is refused");
}

// Each of the dynamic car's pitch and air parameters does what its comment says, as the four-wheel car's do (decision
// 0025): drag decelerates a coasting car by the four-wheel car's drag force over its mass; the centre of gravity's height
// moves the braking force times the height over the wheelbase onto the front axle, whose tires are then evaluated at that
// load, and makes a car braking into a turn turn more; downforce adds to the axle loads by the aero balance.
void pitch_and_air_act_as_on_the_four_wheel_car() {
    const fd::Config config;
    const double g = fd::gravity_mps2;
    const fd::DynamicSingleTrack flat;
    const double front_static = flat.mass_kg*g*flat.cg_to_rear_m/config.wheelbase_m;
    const double rear_static = flat.mass_kg*g-front_static;
    const auto straight = rolling(20, 0.0);

    fd::DynamicSingleTrack dragged;
    dragged.drag_area_m2 = 0.6;
    fd::FourWheelCar four;
    four.drag_area_m2 = 0.6;
    const fd::SingleTrackModel dragging(dragged, config);
    const auto coast = dragging.derivative(dragging.motion(straight), 0, 0, 0);
    near(coast.forward_mps, fd::aerodynamic_force(four, 20, 0).longitudinal_n/dragged.mass_kg, 1e-12, "drag decelerates as the four-wheel car's");
    near(fd::SingleTrackModel(flat, config).derivative(dragging.motion(straight), 0, 0, 0).forward_mps, 0, 0, "and without it nothing does");
    require(drive(dragged, straight, {0, 0}, config, 2.0).pose.speed_mps < drive(flat, straight, {0, 0}, config, 2.0).pose.speed_mps-0.1,
            "so the plant coasts slower");

    // Braking at 6 m/s^2 on the straight: braking is shared by static load; the front axle carries the transferred load.
    fd::DynamicSingleTrack tall;
    tall.cg_height_m = 0.35;
    const double transfer = tall.mass_kg*6*0.35/config.wheelbase_m;
    const auto braking = fd::SingleTrackModel(tall, config).axle_demand(fd::SingleTrackModel(tall, config).motion(straight), 0, -6);
    const auto use = [&](double request, double load) { return std::abs(request)/(config.grip_mu*fd::tire_peak(flat.front_tire, load/2).friction_longitudinal*load); };
    const double front_request = flat.mass_kg*6*front_static/(front_static+rear_static);
    near(braking.front_friction_use, use(front_request, front_static+transfer), 1e-12, "the front axle carries m a h / L more");
    near(braking.rear_friction_use, use(flat.mass_kg*6-front_request, rear_static-transfer), 1e-12, "and the rear that much less");
    near(braking.front_peak_slip_angle_rad, fd::tire_peak(flat.front_tire, (front_static+transfer)/2).slip_angle_rad, 0, "its tires at that load");
    const auto level = fd::SingleTrackModel(flat, config).axle_demand(fd::SingleTrackModel(flat, config).motion(straight), 0, -6);
    require(braking.rear_friction_use > level.rear_friction_use && braking.front_friction_use < level.front_friction_use,
            "so braking asks more of the rear than on a car without height");
    fd::PlantState turning = rolling(15, 0.05);
    turning.yaw_rate_radps = 15*std::tan(0.05)/config.wheelbase_m;
    const auto tall_turn = drive(tall, turning, {-5, 0.05}, config, 0.5);
    const auto flat_turn = drive(flat, turning, {-5, 0.05}, config, 0.5);
    require(tall_turn.yaw_rate_radps > flat_turn.yaw_rate_radps+0.005, "braking into a turn, the loaded front turns the tall car more");

    // Downforce at 20 m/s: half the air density times the area times speed squared, shared by the aero balance.
    fd::DynamicSingleTrack winged;
    winged.downforce_area_m2 = 1.5;
    winged.aero_balance_front = 0.4;
    const double down = 0.5*fd::air_density_kgpm3*1.5*20*20;
    const auto pressed = fd::SingleTrackModel(winged, config).axle_demand(fd::SingleTrackModel(winged, config).motion(straight), 0, -6);
    near(pressed.front_friction_use, use(front_request, front_static+0.4*down), 1e-12, "the front carries its share of downforce");
    near(pressed.rear_friction_use, use(flat.mass_kg*6-front_request, rear_static+0.6*down), 1e-12, "the rear the rest");
    winged.aero_balance_front = 1.0;
    const auto forward_balance = fd::SingleTrackModel(winged, config).axle_demand(fd::SingleTrackModel(winged, config).motion(straight), 0, -6);
    near(forward_balance.front_friction_use, use(front_request, front_static+down), 1e-12, "with the balance forward the front carries it all");
    near(forward_balance.rear_friction_use, level.rear_friction_use, 1e-12, "and the rear none");

    for (const auto& change : std::vector<std::function<void(fd::DynamicSingleTrack&)>>{
             [](fd::DynamicSingleTrack& c) { c.cg_height_m = 2; }, [](fd::DynamicSingleTrack& c) { c.drag_area_m2 = -0.1; },
             [](fd::DynamicSingleTrack& c) { c.downforce_area_m2 = 11; }, [](fd::DynamicSingleTrack& c) { c.aero_balance_front = 1.2; }}) {
        fd::DynamicSingleTrack bad;
        change(bad);
        rejects([&] { fd::validate_vehicle(bad, config); }, "pitch and air outside their ranges are refused");
    }
}

void kinematic_adapter_is_the_existing_bicycle() {
    const fd::Config config;
    const fd::VehicleModel model = fd::KinematicBicycle{};
    std::mt19937 random(3);
    std::uniform_real_distribution<double> speed(0, 25), steer(-0.55, 0.55), accel(-8, 6), yaw(-3, 3);
    for (int i = 0; i < 20000; ++i) {
        const fd::State state{10*yaw(random), -7*yaw(random), yaw(random), speed(random), steer(random), 0.005*i};
        const fd::Command command{accel(random), steer(random)};
        const auto plant = fd::plant_state_from(state, model, config);
        const auto next = fd::advance(model, plant, command, config, config.fixed_dt_s);
        require(same_state(next.pose, fd::integrate_bicycle(state, command, config, config.fixed_dt_s)),
                "the kinematic adapter reproduces integrate_bicycle bit for bit");
        require(next.lateral_velocity_mps == 0, "the kinematic rear axle never slides sideways");
        const auto d = fd::demand(model, next, config, command.acceleration_mps2);
        const double lateral = next.pose.speed_mps*next.pose.speed_mps*std::tan(next.pose.steering_rad)/config.wheelbase_m;
        require(d.lateral_acceleration_mps2 == lateral, "kinematic lateral demand is the previous formula exactly");
        require(d.grip_utilization == std::hypot(lateral, command.acceleration_mps2)/(config.grip_mu*fd::gravity_mps2),
                "kinematic grip use is the previous formula exactly");
        require(d.within_envelope == !(std::hypot(lateral, command.acceleration_mps2) > config.grip_mu*fd::gravity_mps2*(1+1e-6)),
                "kinematic envelope is the previous comparison exactly");
        require(!d.tire_slip_modeled, "the kinematic bicycle does not claim to model slip");
        near(next.yaw_rate_radps, next.pose.speed_mps*std::tan(next.pose.steering_rad)/config.wheelbase_m, 1e-12,
             "kinematic yaw rate is speed times curvature");
    }
}

void straight_acceleration_follows_newton() {
    const fd::Config config;
    const fd::VehicleModel model = fd::DynamicSingleTrack{};
    const auto end = drive(model, rolling(5, 0), {2, 0}, config, 2.0);
    near(end.pose.speed_mps, 9, 1e-9, "2 m/s^2 for 2 s adds 4 m/s");
    near(end.pose.x_m, 5*2+0.5*2*4, 1e-6, "distance follows constant acceleration");
    near(end.pose.y_m, 0, 1e-12, "straight driving stays on the line");
    near(end.lateral_velocity_mps, 0, 1e-12, "no lateral motion without steering");
    near(end.yaw_rate_radps, 0, 1e-12, "no yaw without steering");
    const auto d = fd::demand(model, end, config, 2);
    const fd::DynamicSingleTrack car;
    const double L = config.wheelbase_m, weight = car.mass_kg*fd::gravity_mps2;
    const double front_load = weight*car.cg_to_rear_m/L, rear_load = weight*(L-car.cg_to_rear_m)/L;
    const double front_use = car.drive_front_fraction*car.mass_kg*2/(fd::tire_peak(car.front_tire, front_load/2).friction_longitudinal*front_load);
    const double rear_use = (1-car.drive_front_fraction)*car.mass_kg*2/(fd::tire_peak(car.rear_tire, rear_load/2).friction_longitudinal*rear_load);
    near(d.grip_utilization, std::max(front_use, rear_use), 1e-9, "driving force is split by the drive layout and each axle uses its own friction");
    require(d.within_envelope && d.tire_slip_modeled, "straight acceleration is within the tires' envelope");
}

void matches_the_kinematic_bicycle_at_walking_speed() {
    const fd::Config config;
    const fd::VehicleModel dynamic = fd::DynamicSingleTrack{};
    const fd::VehicleModel kinematic = fd::KinematicBicycle{};
    auto a = fd::plant_state_from({}, dynamic, config);
    auto b = fd::plant_state_from({}, kinematic, config);
    // Pull away from rest while turning, staying below the 1 m/s lower blend speed.
    for (int i = 0; i < 180; ++i) {
        const fd::Command command{0.2, 0.4};
        a = fd::advance(dynamic, a, command, config, config.fixed_dt_s);
        b = fd::advance(kinematic, b, command, config, config.fixed_dt_s);
    }
    require(a.pose.speed_mps < 1.0 && a.pose.speed_mps > 0.1, "the car is moving slowly");
    near(a.pose.x_m, b.pose.x_m, 1e-6, "slow dynamic motion is kinematic in x");
    near(a.pose.y_m, b.pose.y_m, 1e-6, "slow dynamic motion is kinematic in y");
    near(a.pose.yaw_rad, b.pose.yaw_rad, 1e-6, "slow dynamic motion is kinematic in yaw");
    near(a.lateral_velocity_mps, 0, 1e-12, "no sideslip below the blend speed");
}

void steady_turns_follow_the_linear_understeer_gradient() {
    fd::Config config;
    for (const bool soft_front : {true, false}) {
        fd::DynamicSingleTrack car;
        // A softer tire at one end: a larger peak slip angle means less cornering stiffness.
        (soft_front ? car.front_tire : car.rear_tire).peak_slip_angle_low_rad = 0.2;
        (soft_front ? car.front_tire : car.rear_tire).peak_slip_angle_high_rad = 0.18;
        const fd::VehicleModel model = car;
        const double L = config.wheelbase_m, lr = car.cg_to_rear_m, lf = L-lr;
        const double g = fd::gravity_mps2;
        const double front_load = car.mass_kg*g*lr/L, rear_load = car.mass_kg*g*lf/L;
        const double Cf = axle_cornering_stiffness(car.front_tire, front_load);
        const double Cr = axle_cornering_stiffness(car.rear_tire, rear_load);
        const double K = car.mass_kg/L*(lr/Cf-lf/Cr);  // rad per m/s^2 of lateral acceleration
        const double steer = 0.01;
        const auto end = drive(model, rolling(12, steer), {0, steer}, config, 4.0);
        const double v = end.pose.speed_mps;
        const double predicted = v*steer/(L+K*v*v);
        const double kinematic = v*std::tan(steer)/L;
        near(end.yaw_rate_radps, predicted, 0.02*std::abs(predicted),
             std::string(soft_front ? "understeering" : "oversteering")+" yaw rate follows the linear understeer gradient");
        if (soft_front) require(end.yaw_rate_radps < 0.9*kinematic, "a soft front turns less than geometry demands");
        else require(end.yaw_rate_radps > 1.1*kinematic, "a soft rear turns more than geometry demands");
    }
}

void lateral_acceleration_saturates_at_the_friction_limit() {
    const fd::Config config;
    const fd::DynamicSingleTrack car;
    const fd::VehicleModel model = car;
    const auto end = drive(model, rolling(15, 0), {0, 0.4}, config, 1.5);
    const auto d = fd::demand(model, end, config, 0);
    const double v = end.pose.speed_mps;
    const double kinematic_claim = v*v*std::tan(end.pose.steering_rad)/config.wheelbase_m;
    const double peak = fd::tire_peak(car.front_tire, 1000).friction_lateral*fd::gravity_mps2;  // light-load friction bounds both axles
    require(std::abs(d.lateral_acceleration_mps2) <= 1.001*peak, "lateral acceleration cannot exceed the tires' peak friction");
    require(std::abs(d.lateral_acceleration_mps2) > 0.6*fd::gravity_mps2, "the tires are working hard");
    require(kinematic_claim > 2*std::abs(d.lateral_acceleration_mps2), "the kinematic formula would claim far more");
    require(!d.within_envelope, "front slip past the peak leaves the envelope");
    require(d.grip_utilization > 0.75 && d.grip_utilization <= 1+1e-9, "an axle works near the edge of its friction ellipse");
}

void lower_road_grip_makes_the_car_slide() {
    fd::Config grippy, slippery;
    slippery.grip_mu = 0.4;
    const fd::VehicleModel model = fd::DynamicSingleTrack{};
    const double steer = 0.1;  // about 0.55 g of kinematic demand at 12 m/s
    const auto a = drive(model, rolling(12, steer), {0, steer}, grippy, 3.0);
    const auto b = drive(model, rolling(12, steer), {0, steer}, slippery, 3.0);
    require(std::abs(fd::rear_sideslip_rad(b)) > 2*std::abs(fd::rear_sideslip_rad(a)), "lower grip means more sideslip");
    const double slippery_limit = slippery.grip_mu*fd::tire_peak(fd::DynamicSingleTrack::roadster_tire(), 1000).friction_lateral*fd::gravity_mps2;
    const double lateral_a = fd::demand(model, a, grippy, 0).lateral_acceleration_mps2;
    const double lateral_b = fd::demand(model, b, slippery, 0).lateral_acceleration_mps2;
    require(lateral_b <= 1.001*slippery_limit, "lower grip caps lateral acceleration at the lower friction");
    require(lateral_b < 0.8*lateral_a, "the slippery car cannot corner as hard as the grippy one");
    require(fd::demand(model, a, grippy, 0).within_envelope, "the grippy corner is within the envelope");
    require(!fd::demand(model, b, slippery, 0).within_envelope, "the slippery corner is sliding");
}

void rear_wheel_drive_spins_under_power_in_a_corner() {
    const fd::Config config;
    // Settle into a corner, then ask for the controller's full acceleration budget, as a corner exit does.
    const double full = config.longitudinal_grip_fraction*config.grip_mu*fd::gravity_mps2;
    const auto exit_corner = [&](double drive_front_fraction) {
        fd::DynamicSingleTrack car;
        car.drive_front_fraction = drive_front_fraction;
        const fd::VehicleModel model = car;
        const auto cornering = drive(model, rolling(11, 0.08), {0, 0.08}, config, 2.0);
        return std::pair{cornering, drive(model, cornering, {full, 0.08}, config, 1.0)};
    };
    const auto [rwd_corner, rwd] = exit_corner(0.0);
    const auto [awd_corner, awd] = exit_corner(fd::DynamicSingleTrack{}.drive_front_fraction);
    std::cout << "  corner exit at " << full << " m/s^2: rear sideslip rear-wheel drive " << fd::rear_sideslip_rad(rwd)
              << " rad, shared drive " << fd::rear_sideslip_rad(awd) << " rad\n";
    require(std::abs(fd::rear_sideslip_rad(rwd)) > 0.3, "rear-wheel drive at the whole-car budget loses the rear");
    require(std::abs(fd::rear_sideslip_rad(awd)) < 0.1, "load-shared drive keeps the rear");
}

// The default substep trades cost for accuracy; this pins that trade against a five-times finer
// reference, through a corner entry, a slide on low grip and a pull-away through the blend band.
void default_substep_is_converged() {
    fd::DynamicSingleTrack fine;
    fine.substep_s = 0.0005;
    const fd::VehicleModel reference = fine;
    const fd::VehicleModel coarse = fd::DynamicSingleTrack{};
    struct Case { const char* name; double grip, speed, steer, accel, seconds; };
    for (const Case c : {Case{"corner entry", 1.0, 14, 0.12, -1.5, 3.0}, Case{"low-grip slide", 0.4, 12, 0.1, 0, 3.0},
                         Case{"pull-away through the blend band", 1.0, 0.5, 0.3, 1.5, 3.0}}) {
        fd::Config config;
        config.grip_mu = c.grip;
        const auto a = drive(reference, rolling(c.speed, 0), {c.accel, c.steer}, config, c.seconds);
        const auto b = drive(coarse, rolling(c.speed, 0), {c.accel, c.steer}, config, c.seconds);
        const std::string name = c.name;
        near(b.pose.x_m, a.pose.x_m, 0.01, name+": x converged within 1 cm");
        near(b.pose.y_m, a.pose.y_m, 0.01, name+": y converged within 1 cm");
        near(b.pose.speed_mps, a.pose.speed_mps, 1e-3, name+": speed converged");
        near(b.yaw_rate_radps, a.yaw_rate_radps, 1e-3, name+": yaw rate converged");
        near(b.lateral_velocity_mps, a.lateral_velocity_mps, 1e-3, name+": lateral velocity converged");
    }
}

void braking_to_rest_never_reverses() {
    const fd::Config config;
    const fd::VehicleModel model = fd::DynamicSingleTrack{};
    auto s = rolling(10, 0.1);
    for (int i = 0; i < 800; ++i) {
        s = fd::advance(model, s, {-6, 0.1}, config, config.fixed_dt_s);
        require(s.pose.speed_mps >= 0, "braking never reverses the car");
    }
    near(s.pose.speed_mps, 0, 0, "the car comes to rest");
    near(s.lateral_velocity_mps, 0, 0, "no sideslip at rest");
    near(s.yaw_rate_radps, 0, 0, "no yaw at rest");
    const auto still = fd::advance(model, s, {-6, 0.3}, config, config.fixed_dt_s);
    near(still.pose.x_m, s.pose.x_m, 0, "braking at rest does not move the car");
}

void blend_is_continuous_in_speed() {
    const fd::Config config;
    const fd::VehicleModel model = fd::DynamicSingleTrack{};
    double previous_rate = 0, previous_lateral = 0;
    for (int i = 0; i <= 2400; ++i) {
        const double v = 0.8+i*0.001;
        auto start = fd::plant_state_from({0, 0, 0, v, 0.2, 0}, model, config);
        const auto next = fd::advance(model, start, {0, 0.2}, config, config.fixed_dt_s);
        if (i) {
            require(std::abs(next.yaw_rate_radps-previous_rate) < 2e-3, "yaw rate has no jump across the blend band at "+std::to_string(v));
            require(std::abs(next.lateral_velocity_mps-previous_lateral) < 2e-3, "lateral velocity has no jump across the blend band at "+std::to_string(v));
        }
        previous_rate = next.yaw_rate_radps;
        previous_lateral = next.lateral_velocity_mps;
    }
    // Slowing through the band while turning: the state must arrive on kinematic motion, so the
    // switch to the exact kinematic bicycle at the lower speed does not jump.
    auto s = drive(model, rolling(8, 0.15), {0, 0.15}, config, 2.0);
    double largest_rate_step = 0, largest_lateral_step = 0;
    while (s.pose.speed_mps > 0.2) {
        const auto next = fd::advance(model, s, {-2, 0.15}, config, config.fixed_dt_s);
        largest_rate_step = std::max(largest_rate_step, std::abs(next.yaw_rate_radps-s.yaw_rate_radps));
        largest_lateral_step = std::max(largest_lateral_step, std::abs(next.lateral_velocity_mps-s.lateral_velocity_mps));
        s = next;
    }
    std::cout << "  slowing through the blend band: largest tick-to-tick change in yaw rate " << largest_rate_step
              << " rad/s, in rear lateral velocity " << largest_lateral_step << " m/s\n";
    require(largest_rate_step < 5e-3 && largest_lateral_step < 5e-3, "slowing through the blend band has no jump");
}

void published_rear_axle_motion_is_consistent() {
    const fd::Config config;
    const fd::VehicleModel model = fd::DynamicSingleTrack{};
    auto s = drive(model, rolling(14, 0.08), {0.5, 0.08}, config, 2.0);
    for (int i = 0; i < 200; ++i) {
        const auto next = fd::advance(model, s, {0.5, 0.08}, config, config.fixed_dt_s);
        const double dt = config.fixed_dt_s;
        const double c = std::cos(s.pose.yaw_rad), si = std::sin(s.pose.yaw_rad);
        const double vx = (next.pose.x_m-s.pose.x_m)/dt, vy = (next.pose.y_m-s.pose.y_m)/dt;
        const double forward = (c*vx+si*vy), sideways = (-si*vx+c*vy);
        near(forward, 0.5*(s.pose.speed_mps+next.pose.speed_mps), 0.02, "rear axle moves forward at the published speed");
        near(sideways, 0.5*(s.lateral_velocity_mps+next.lateral_velocity_mps), 0.05, "rear axle moves sideways at the published lateral velocity");
        near(fd::wrap_angle(next.pose.yaw_rad-s.pose.yaw_rad)/dt, 0.5*(s.yaw_rate_radps+next.yaw_rate_radps), 1e-3, "heading turns at the published yaw rate");
        near(next.pose.time_s, s.pose.time_s+dt, 1e-12, "the clock advances by the tick");
        s = next;
    }
}

void invalid_vehicles_and_states_are_rejected() {
    const fd::Config config;
    const auto broken = [&](const std::function<void(fd::DynamicSingleTrack&)>& change, const std::string& what) {
        fd::DynamicSingleTrack car;
        change(car);
        const fd::VehicleModel model = car;
        rejects([&] { fd::validate_vehicle(model, config); }, what+" is rejected");
        rejects([&] { fd::advance(model, rolling(5, 0), {}, config, config.fixed_dt_s); }, what+" is rejected when advancing");
    };
    broken([](auto& c) { c.mass_kg = 0; }, "zero mass");
    broken([](auto& c) { c.yaw_inertia_kgm2 = -1; }, "negative yaw inertia");
    broken([](auto& c) { c.cg_to_rear_m = 2.6; }, "a centre of gravity on the front axle");
    broken([](auto& c) { c.cg_to_rear_m = 0; }, "a centre of gravity on the rear axle");
    broken([](auto& c) { c.dynamic_above_mps = c.kinematic_below_mps; }, "an empty blend band");
    broken([](auto& c) { c.slip_speed_floor_mps = 0; }, "no slip-speed floor");
    broken([](auto& c) { c.drive_front_fraction = 1.5; }, "a drive split beyond the front axle");
    broken([](auto& c) { c.substep_s = 0.02; }, "a substep longer than the plant tick");
    broken([](auto& c) { c.front_tire.peak_friction_lateral_low = -1; }, "an invalid tire");
    const fd::VehicleModel model = fd::DynamicSingleTrack{};
    auto bad = rolling(5, 0);
    bad.yaw_rate_radps = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { fd::advance(model, bad, {}, config, config.fixed_dt_s); }, "a non-finite plant state is rejected");
    rejects([&] { fd::advance(model, rolling(5, 0), {std::numeric_limits<double>::infinity(), 0}, config, config.fixed_dt_s); }, "a non-finite command is rejected");
    rejects([&] { fd::advance(model, rolling(5, 0), {}, config, 0); }, "a zero step is rejected");
}

void prediction_rolls_the_dynamic_plant_forward() {
    const auto track = fd::make_preset_track();
    const fd::Config config;
    const auto plan = fd::make_speed_plan(track, config);
    const fd::VehicleModel model = fd::DynamicSingleTrack{};
    fd::Simulation simulation(track, config, model);
    for (int i = 0; i < 1600; ++i) simulation.step();  // 8 s, approaching the first corner
    const auto& plant = simulation.plant_state();
    const auto prediction = fd::make_local_trajectory(track, plan, plant, config, model);
    require(same_state(prediction.points.front().state, plant.pose), "prediction starts at the actual plant state");
    auto copy = plant;
    for (int i = 0; i < 4; ++i) copy = fd::advance(model, copy, prediction.first_control.applied, config, config.fixed_dt_s);
    const auto& held = prediction.points[1].state;
    require(held.x_m == copy.pose.x_m && held.y_m == copy.pose.y_m && held.yaw_rad == copy.pose.yaw_rad &&
            held.speed_mps == copy.pose.speed_mps && held.steering_rad == copy.pose.steering_rad,
            "the prediction's first hold is the dynamic plant under the first command");
    near(held.time_s, copy.pose.time_s, 1e-12, "prediction time is the tick count on the fixed grid");
    const auto decision = fd::choose_local_action(track, plan, plant, config, model, {});
    require(decision.options.size() == 1 && same_state(decision.trajectory().points.back().state, prediction.points.back().state),
            "the planner predicts with the same plant");
}

// Axle slip angles from the state alone, independent of the plant's force calculation.
struct Slip { double front, rear; };
Slip slip_from_state(const fd::DynamicSingleTrack& car, const fd::Config& config, const fd::PlantState& s) {
    const double lr = car.cg_to_rear_m, lf = config.wheelbase_m-lr;
    const double forward = std::max(s.pose.speed_mps, car.slip_speed_floor_mps);
    const double vy_cg = s.lateral_velocity_mps+lr*s.yaw_rate_radps;  // rear-axle lateral velocity moved to the centre of gravity
    return {s.pose.steering_rad-std::atan2(vy_cg+lf*s.yaw_rate_radps, forward),
            -std::atan2(vy_cg-lr*s.yaw_rate_radps, forward)};
}

void demand_reports_axle_slip_angles() {
    const fd::Config config;
    const fd::DynamicSingleTrack car;
    const fd::VehicleModel model = car;
    const double L = config.wheelbase_m, weight = car.mass_kg*fd::gravity_mps2;
    const double front_peak = fd::tire_peak(car.front_tire, weight*car.cg_to_rear_m/L/2).slip_angle_rad;
    const double rear_peak = fd::tire_peak(car.rear_tire, weight*(L-car.cg_to_rear_m)/L/2).slip_angle_rad;
    std::mt19937 random(11);
    std::uniform_real_distribution<double> speed(3.5, 25), steer(-0.5, 0.5), lateral(-2, 2), yaw(-1.5, 1.5), accel(-6, 4);
    for (int i = 0; i < 5000; ++i) {
        fd::PlantState s;
        s.pose.speed_mps = speed(random);
        s.pose.steering_rad = steer(random);
        s.lateral_velocity_mps = lateral(random);
        s.yaw_rate_radps = yaw(random);
        const auto d = fd::demand(model, s, config, accel(random));
        const auto expected = slip_from_state(car, config, s);
        near(d.front_slip_angle_rad, expected.front, 1e-12, "front slip angle follows the axle's velocity and steering");
        near(d.rear_slip_angle_rad, expected.rear, 1e-12, "rear slip angle follows the axle's velocity");
        near(d.front_peak_slip_angle_rad, front_peak, 1e-15, "front peak slip is the tire's at its static load");
        near(d.rear_peak_slip_angle_rad, rear_peak, 1e-15, "rear peak slip is the tire's at its static load");
        require(d.within_envelope == (std::abs(expected.front) <= front_peak && std::abs(expected.rear) <= rear_peak),
                "the envelope is both slip angles within their peaks");
    }
    const auto kinematic = fd::demand(fd::KinematicBicycle{}, rolling(12, 0.1), config, 0);
    require(kinematic.front_slip_angle_rad == 0 && kinematic.rear_slip_angle_rad == 0 && kinematic.front_peak_slip_angle_rad == 0 &&
            kinematic.rear_peak_slip_angle_rad == 0, "the kinematic bicycle reports no slip angles");
    require(fd::handling_balance(kinematic).balance == fd::Balance::not_modeled && fd::handling_balance(kinematic).understeer_angle_rad == 0,
            "the kinematic bicycle has no handling balance");
    const auto walking = fd::demand(model, rolling(0.5, 0.3), config, 0);
    require(!walking.tire_slip_modeled && fd::handling_balance(walking).balance == fd::Balance::not_modeled,
            "below the blend speed the dynamic car is the kinematic bicycle and has no balance");
}

// Settled turns, left and right: a soft front understeers, a soft rear oversteers, the default car is
// neutral, and the understeer angle is the steering beyond geometry, from the state alone.
void balance_follows_front_against_rear_slip() {
    const fd::Config config;
    const auto soft_rear = [] {
        fd::DynamicSingleTrack car;
        car.rear_tire.peak_slip_angle_low_rad = 0.2;
        car.rear_tire.peak_slip_angle_high_rad = 0.18;
        return car;
    }();
    const std::vector<std::pair<std::string, std::pair<fd::DynamicSingleTrack, fd::Balance>>> cars{
        {"soft front", {fd::DynamicSingleTrack::soft_front(), fd::Balance::understeer}},
        {"soft rear", {soft_rear, fd::Balance::oversteer}},
        {"default", {fd::DynamicSingleTrack{}, fd::Balance::neutral}}};
    for (const auto& [name, entry] : cars) {
        const auto& [car, expected] = entry;
        const fd::VehicleModel model = car;
        for (const double side : {1.0, -1.0}) {
            const double steer = side*0.05;
            const auto s = drive(model, rolling(11, steer), {0, steer}, config, 4.0);
            const auto d = fd::demand(model, s, config, 0);
            const auto balance = fd::handling_balance(d);
            const double beyond_geometry = side*(steer-config.wheelbase_m*s.yaw_rate_radps/s.pose.speed_mps);
            std::cout << "  " << name << (side > 0 ? " left" : " right") << ": lateral acceleration " << d.lateral_acceleration_mps2
                      << " m/s^2, understeer angle " << balance.understeer_angle_rad << " rad, steering beyond geometry "
                      << beyond_geometry << " rad" << "\n";
            require(std::abs(d.lateral_acceleration_mps2) > 1.5, name+": the car is cornering");
            require(balance.balance == expected, name+" is labelled "+fd::balance_name(expected)+", got "+fd::balance_name(balance.balance));
            near(balance.understeer_angle_rad, beyond_geometry, 2e-4, name+": the understeer angle is the steering beyond geometry");
        }
    }
    const fd::VehicleModel model = fd::DynamicSingleTrack::soft_front();
    const auto straight = fd::demand(model, drive(model, rolling(15, 0), {0, 0}, config, 1.0), config, 0);
    require(fd::handling_balance(straight).balance == fd::Balance::not_cornering && fd::handling_balance(straight).understeer_angle_rad == 0,
            "driving straight is not cornering");
    require(std::string(fd::balance_name(fd::Balance::understeer)) == "understeer" && std::string(fd::balance_name(fd::Balance::oversteer)) == "oversteer" &&
            std::string(fd::balance_name(fd::Balance::neutral)) == "neutral" && std::string(fd::balance_name(fd::Balance::not_cornering)) == "not cornering" &&
            std::string(fd::balance_name(fd::Balance::not_modeled)) == "not modeled", "balance names are stable text");

    // The simulation reports the same numbers the seam gives for its own state.
    fd::Simulation simulation(fd::make_preset_track(), config, fd::DynamicSingleTrack::soft_front());
    for (int i = 0; i < 1900; ++i) simulation.step();  // 9.5 s, in the first corner
    const auto& diagnostics = simulation.diagnostics();
    const auto seam = fd::demand(simulation.vehicle_model(), simulation.plant_state(), config, diagnostics.applied.acceleration_mps2);
    require(diagnostics.front_slip_angle_rad == seam.front_slip_angle_rad && diagnostics.rear_slip_angle_rad == seam.rear_slip_angle_rad &&
            diagnostics.front_peak_slip_angle_rad == seam.front_peak_slip_angle_rad && diagnostics.rear_peak_slip_angle_rad == seam.rear_peak_slip_angle_rad,
            "diagnostics carry the seam's slip angles");
    const auto expected_balance = fd::handling_balance(seam);
    require(diagnostics.balance == expected_balance.balance && diagnostics.understeer_angle_rad == expected_balance.understeer_angle_rad,
            "diagnostics carry the seam's balance");
    require(diagnostics.balance == fd::Balance::understeer, "the soft-front car understeers in the first corner");
}

// The prediction's cheap envelope query must never disagree with the full demand.
void envelope_query_agrees_with_demand() {
    const fd::Config config;
    auto soft_rear = fd::DynamicSingleTrack{};
    soft_rear.rear_tire.peak_slip_angle_low_rad = 0.06;
    const std::vector<fd::VehicleModel> models{fd::KinematicBicycle{}, fd::DynamicSingleTrack{}, soft_rear, fd::DynamicSingleTrack::soft_front(),
                                               pitched_and_winged(), fd::FourWheelCar{}};
    std::mt19937 random(23);
    std::uniform_real_distribution<double> speed(0, 25), steer(-0.5, 0.5), lateral(-3, 3), yaw(-2, 2), accel(-12, 8), wheel(0, 100);
    std::size_t outside = 0, inside = 0;
    for (const auto& model : models)
        for (int i = 0; i < 4000; ++i) {
            fd::PlantState s;
            s.pose.speed_mps = speed(random);
            s.pose.steering_rad = steer(random);
            s.lateral_velocity_mps = std::holds_alternative<fd::KinematicBicycle>(model) ? 0 : lateral(random);
            s.yaw_rate_radps = yaw(random);
            if (std::holds_alternative<fd::FourWheelCar>(model)) {
                for (auto& w : s.wheel_speeds_radps) w = i%3 == 0 ? s.pose.speed_mps/0.3*(1+0.02*lateral(random)) : wheel(random);
                // Loads across the tire's reference range, and now and then a lifted wheel.
                for (auto& load : s.wheel_loads_n) load = i%17 == 0 ? 0.0 : 500+60*wheel(random);
            }
            const double longitudinal = accel(random);
            const auto d = fd::demand(model, s, config, longitudinal);
            const auto e = fd::envelope(model, s, config, longitudinal);
            require(e.within == d.within_envelope && e.tire_slip_modeled == d.tire_slip_modeled,
                    std::string("the envelope query agrees with demand for ")+fd::vehicle_model_name(model));
            (d.within_envelope ? inside : outside) += 1;
        }
    require(inside > 1000 && outside > 1000, "the sample covers both sides of the envelope");
}

void completes_a_lap_on_the_dynamic_plant() {
    const auto track = fd::make_preset_track();
    const fd::Config config;
    fd::Simulation kinematic(track, config);
    fd::Simulation dynamic(track, config, fd::DynamicSingleTrack{});
    require(std::holds_alternative<fd::KinematicBicycle>(kinematic.vehicle_model()), "the default plant is the kinematic bicycle");
    const auto began = std::chrono::steady_clock::now();
    const auto run = [&](fd::Simulation& simulation, double& lap_time, double& max_error, double& max_slip, double& max_use, int& sliding) {
        while (simulation.diagnostics().laps < 1 && simulation.state().time_s < 120) {
            simulation.step();
            max_error = std::max(max_error, std::abs(simulation.diagnostics().cross_track_error_m));
            max_slip = std::max(max_slip, std::abs(simulation.diagnostics().rear_sideslip_rad));
            max_use = std::max(max_use, simulation.diagnostics().combined_grip_utilization);
            if (!simulation.diagnostics().within_grip_envelope) ++sliding;
        }
        lap_time = simulation.state().time_s;
    };
    double kinematic_lap = 0, kinematic_error = 0, kinematic_slip = 0, kinematic_use = 0;
    double dynamic_lap = 0, dynamic_error = 0, dynamic_slip = 0, dynamic_use = 0;
    int kinematic_sliding = 0, dynamic_sliding = 0;
    run(kinematic, kinematic_lap, kinematic_error, kinematic_slip, kinematic_use, kinematic_sliding);
    const auto middle = std::chrono::steady_clock::now();
    run(dynamic, dynamic_lap, dynamic_error, dynamic_slip, dynamic_use, dynamic_sliding);
    const auto ended = std::chrono::steady_clock::now();
    std::cout << "  kinematic lap " << kinematic_lap << " s, max tracking error " << kinematic_error << " m ("
              << std::chrono::duration<double>(middle-began).count() << " s wall)\n"
              << "  dynamic lap " << dynamic_lap << " s, max tracking error " << dynamic_error << " m, max rear sideslip "
              << dynamic_slip << " rad, max axle grip use " << dynamic_use << ", ticks past the slip peak " << dynamic_sliding
              << " (" << std::chrono::duration<double>(ended-middle).count() << " s wall; measurement only)\n";
    require(kinematic_slip == 0, "the kinematic plant reports no sideslip");
    require(dynamic.diagnostics().laps == 1, "the dynamic car completes a lap with the unchanged controller");
    require(dynamic_error < track.width_m/2-fd::vehicle_half_width_m, "the dynamic car stays within the track margin");
    require(dynamic_slip > 0, "the dynamic car reports sideslip in corners");
}
} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"kinematic_adapter_is_the_existing_bicycle", kinematic_adapter_is_the_existing_bicycle},
        {"straight_acceleration_follows_newton", straight_acceleration_follows_newton},
        {"matches_the_kinematic_bicycle_at_walking_speed", matches_the_kinematic_bicycle_at_walking_speed},
        {"steady_turns_follow_the_linear_understeer_gradient", steady_turns_follow_the_linear_understeer_gradient},
        {"lateral_acceleration_saturates_at_the_friction_limit", lateral_acceleration_saturates_at_the_friction_limit},
        {"lower_road_grip_makes_the_car_slide", lower_road_grip_makes_the_car_slide},
        {"rear_wheel_drive_spins_under_power_in_a_corner", rear_wheel_drive_spins_under_power_in_a_corner},
        {"default_substep_is_converged", default_substep_is_converged},
        {"braking_to_rest_never_reverses", braking_to_rest_never_reverses},
        {"blend_is_continuous_in_speed", blend_is_continuous_in_speed},
        {"published_rear_axle_motion_is_consistent", published_rear_axle_motion_is_consistent},
        {"invalid_vehicles_and_states_are_rejected", invalid_vehicles_and_states_are_rejected},
        {"prediction_rolls_the_dynamic_plant_forward", prediction_rolls_the_dynamic_plant_forward},
        {"demand_reports_axle_slip_angles", demand_reports_axle_slip_angles},
        {"balance_follows_front_against_rear_slip", balance_follows_front_against_rear_slip},
        {"envelope_query_agrees_with_demand", envelope_query_agrees_with_demand},
        {"completes_a_lap_on_the_dynamic_plant", completes_a_lap_on_the_dynamic_plant},
        {"the_prediction_model_is_the_plants_equations", the_prediction_model_is_the_plants_equations},
        {"each_model_reduces_to_a_single_track", each_model_reduces_to_a_single_track},
        {"pitch_and_air_act_as_on_the_four_wheel_car", pitch_and_air_act_as_on_the_four_wheel_car}
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " vehicle groups passed\n";
    return failures ? 1 : 0;
}

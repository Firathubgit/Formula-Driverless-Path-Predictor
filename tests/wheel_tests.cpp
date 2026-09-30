#include "fd/simulation.hpp"
#include "fd/steering.hpp"
#include "fd/vehicle.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Phase 2.1: a car with four rotating wheels. Oracles are independent of the integrator: Newton's law
// with the wheels' rotational inertia, the tire law at a locked wheel, the linear understeer gradient,
// the power limit, and an energy balance computed from the tire law at each recorded state. The car has
// load transfer (Phase 2.2); where an oracle needs a wheel's load it reads the load the state carries,
// which fd_load_transfer checks against the balance equations. It also has drag (Phase 2.3), which the
// oracles add from its definition and fd_aerodynamics checks on its own.
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

const fd::Config config;

fd::PlantState rolling(const fd::FourWheelCar& car, double speed, double steering = 0) {
    return fd::plant_state_from({0, 0, 0, speed, steering, 0}, car, config);
}
fd::PlantState drive(const fd::FourWheelCar& car, fd::PlantState s, fd::Command command, double seconds,
                     const std::function<void(const fd::PlantState&)>& each = {}) {
    const auto ticks = static_cast<int>(std::llround(seconds/config.fixed_dt_s));
    for (int i = 0; i < ticks; ++i) {
        s = fd::advance(car, s, command, config, config.fixed_dt_s);
        if (each) each(s);
    }
    return s;
}
// Static load, before any transfer.
double wheel_load(const fd::FourWheelCar& car, bool front) {
    const double L = config.wheelbase_m;
    return car.mass_kg*fd::gravity_mps2*(front ? car.cg_to_rear_m : L-car.cg_to_rear_m)/L/2;
}
double equivalent_mass(const fd::FourWheelCar& car) {
    return car.mass_kg+4*car.wheel_inertia_kgm2/(car.wheel_radius_m*car.wheel_radius_m);
}
bool locked(const fd::PlantState& s, std::size_t wheel) { return s.wheel_speeds_radps[wheel] == 0; }
// Drag at a speed, from its definition with standard sea-level air.
double drag_n(const fd::FourWheelCar& car, double speed) { return 0.5*1.225*car.drag_area_m2*speed*speed; }

void free_rolling_wheels_turn_like_the_single_track_car() {
    // Without drag, so a coasting car is a free-rolling one.
    fd::FourWheelCar car;
    car.drag_area_m2 = 0;
    const auto start = rolling(car, 15);
    for (const double w : start.wheel_speeds_radps) near(w, 15/car.wheel_radius_m, 1e-12, "a rolling start spins every wheel at speed over radius");
    const auto coasting = drive(car, start, {0, 0}, 2.0);
    near(coasting.pose.speed_mps, 15, 1e-6, "with no torque and no drag the car keeps its speed");
    for (std::size_t i = 0; i < 4; ++i)
        near(coasting.wheel_speeds_radps[i], coasting.pose.speed_mps/car.wheel_radius_m, 1e-6, "free wheels roll without slip");

    // Steady gentle turn against the linear understeer gradient, from the axles' cornering stiffness at static
    // load. Moving load across an axle changes its total cornering stiffness only in second order.
    const double L = config.wheelbase_m, lr = car.cg_to_rear_m, lf = L-lr, h = 1e-6;
    const auto axle_stiffness = [&](const fd::TireParameters& tire, double load) {
        return 2*(fd::tire_force(tire, 0, h, load).lateral_n-fd::tire_force(tire, 0, -h, load).lateral_n)/(2*h);
    };
    for (const bool soft_front : {false, true}) {
        auto turning = car;
        if (soft_front) { turning.front_tire.peak_slip_angle_low_rad = 0.2; turning.front_tire.peak_slip_angle_high_rad = 0.18; }
        const double Cf = axle_stiffness(turning.front_tire, wheel_load(turning, true));
        const double Cr = axle_stiffness(turning.rear_tire, wheel_load(turning, false));
        const double K = turning.mass_kg/L*(lr/Cf-lf/Cr);
        const double steer = 0.02;
        const auto end = drive(turning, rolling(turning, 12, steer), {0, steer}, 4.0);
        const double v = end.pose.speed_mps;
        near(end.yaw_rate_radps, v*steer/(L+K*v*v), 0.03*v*steer/(L+K*v*v),
             std::string(soft_front ? "soft-front" : "default")+" car turns as the understeer gradient predicts");
        // A firmer turn than the linear check, so the car is cornering above the balance threshold.
        const auto firm = drive(turning, rolling(turning, 12, 0.05), {0, 0.05}, 4.0);
        const auto balance = fd::handling_balance(fd::demand(turning, firm, config, 0)).balance;
        require(balance == (soft_front ? fd::Balance::understeer : fd::Balance::neutral),
                std::string("the ")+(soft_front ? "soft-front" : "default")+" four-wheel car is labelled "+fd::balance_name(balance));
    }
}

void driving_torque_accelerates_the_car_and_its_wheels() {
    const fd::FourWheelCar car;
    const double command = 2;
    const auto start = rolling(car, 5);
    const auto end = drive(car, start, {command, 0}, 2.0);
    // The torque also spins up four wheels, and drag grows with the square of speed along the way.
    const double v0 = start.pose.speed_mps, v1 = end.pose.speed_mps;
    const double mean_drag = 0.5*1.225*car.drag_area_m2*(v0*v0+v0*v1+v1*v1)/3;
    const double expected = (car.mass_kg*command-mean_drag)/equivalent_mass(car);
    near((end.pose.speed_mps-5)/2, expected, 0.01*expected, "acceleration is the wheel torque over mass plus wheel inertia");
    require(car.mass_kg*command/equivalent_mass(car) < 0.96*command, "wheel inertia visibly reduces the acceleration");
    const auto d = fd::demand(car, end, config, command);
    for (std::size_t i = 0; i < 4; ++i)
        require(d.wheel_slip_ratios[i] > 0 && d.wheel_slip_ratios[i] < 0.05, "driven wheels creep a little ahead of the ground");
}

// Stopping from 20 m/s to 3 m/s, where the model is still fully dynamic.
struct Stop { double distance_m{}, seconds{}; double min_slip_ratio{}; bool any_locked{}; };
Stop stop(const fd::FourWheelCar& car, double deceleration) {
    Stop result;
    auto s = rolling(car, 20);
    while (s.pose.speed_mps > 3 && result.seconds < 20) {
        s = fd::advance(car, s, {-deceleration, 0}, config, config.fixed_dt_s);
        result.seconds += config.fixed_dt_s;
        const auto d = fd::demand(car, s, config, -deceleration);
        for (std::size_t i = 0; i < 4; ++i) {
            result.min_slip_ratio = std::min(result.min_slip_ratio, d.wheel_slip_ratios[i]);
            result.any_locked = result.any_locked || locked(s, i);
        }
    }
    result.distance_m = s.pose.x_m;
    return result;
}

void threshold_braking_stops_shorter_than_locked_braking() {
    // Braking moves load forward, so the bias that reaches threshold on both axles is the braking load's share
    // (fd_load_transfer derives it); the static share of 0.5 locks the rear wheels first.
    fd::FourWheelCar car;
    car.brake_bias_front = 0.62;
    // Wheel inertia absorbs part of the brake torque, so 0.95 g of command is still short of the tires' peak.
    const auto threshold = stop(car, 0.95*fd::gravity_mps2);
    const auto locked_stop = stop(car, 2.5*fd::gravity_mps2);
    std::cout << "  20 to 3 m/s: threshold braking " << threshold.distance_m << " m in " << threshold.seconds
              << " s, locked " << locked_stop.distance_m << " m in " << locked_stop.seconds << " s\n";
    require(!threshold.any_locked && threshold.min_slip_ratio > -0.1, "threshold braking keeps every wheel turning below its peak slip");
    require(locked_stop.any_locked, "excessive brake torque locks the wheels");
    require(threshold.distance_m < 0.95*locked_stop.distance_m, "threshold braking stops measurably shorter than locked braking");
}

void locked_wheels_slide_at_the_tires_sliding_force() {
    const fd::FourWheelCar car;
    auto s = rolling(car, 20);
    s = drive(car, s, {-2.5*fd::gravity_mps2, 0}, 0.3);
    for (std::size_t i = 0; i < 4; ++i) require(locked(s, i), "every wheel is locked");
    const auto d = fd::demand(car, s, config, -2.5*fd::gravity_mps2);
    for (std::size_t i = 0; i < 4; ++i) near(d.wheel_slip_ratios[i], -1, 1e-12, "a locked wheel sliding forward has slip ratio -1");
    const double v0 = s.pose.speed_mps;
    s = drive(car, s, {-2.5*fd::gravity_mps2, 0}, 0.5);
    const double measured = (v0-s.pose.speed_mps)/0.5;
    // Locked wheels do not turn, so the tires' sliding force and drag slow the chassis alone.
    const double mid_drag = drag_n(car, 0.5*(v0+s.pose.speed_mps))/car.mass_kg;
    double sliding = mid_drag, peak = 0;
    for (std::size_t i = 0; i < 4; ++i) {
        const auto& tire = i < 2 ? car.front_tire : car.rear_tire;
        const double load = s.wheel_loads_n[i];
        sliding -= fd::tire_force(tire, -1, 0, load).longitudinal_n/car.mass_kg;
        peak += fd::tire_peak(tire, load).friction_longitudinal*load/car.mass_kg;
    }
    std::cout << "  locked deceleration " << measured << " m/s^2, tire law at slip ratio -1 with drag " << sliding << ", tire peak " << peak << "\n";
    near(measured, sliding, 0.01*sliding, "a locked car decelerates at the tires' force at slip ratio -1 and drag");
    require(sliding < 0.9*peak, "sliding decelerates less than the tires' peak");
}

void brake_bias_decides_which_axle_locks() {
    const auto brake_in_a_turn = [](double bias) {
        fd::FourWheelCar car;
        car.brake_bias_front = bias;
        const double steer = 0.04;
        const auto turning = drive(car, rolling(car, 16, steer), {0, steer}, 2.0);
        const auto end = drive(car, turning, {-1.6*fd::gravity_mps2, steer}, 1.0);
        return std::pair{turning, end};
    };
    const auto [front_turn, front_end] = brake_in_a_turn(0.9);
    const auto [rear_turn, rear_end] = brake_in_a_turn(0.1);
    std::cout << "  front-biased lock: yaw rate " << front_turn.yaw_rate_radps << " to " << front_end.yaw_rate_radps
              << " rad/s, sideslip " << fd::rear_sideslip_rad(front_end) << " rad; rear-biased lock: yaw rate "
              << rear_turn.yaw_rate_radps << " to " << rear_end.yaw_rate_radps << " rad/s, sideslip " << fd::rear_sideslip_rad(rear_end) << " rad\n";
    require(locked(front_end, fd::front_left) && locked(front_end, fd::front_right) &&
            !locked(front_end, fd::rear_left) && !locked(front_end, fd::rear_right), "front bias locks the front wheels only");
    require(std::abs(front_end.yaw_rate_radps) < 0.5*std::abs(front_turn.yaw_rate_radps) &&
            std::abs(fd::rear_sideslip_rad(front_end)) < 0.05, "with the front locked the car stops turning and runs straight on");
    require(locked(rear_end, fd::rear_left) && locked(rear_end, fd::rear_right) &&
            !locked(rear_end, fd::front_left) && !locked(rear_end, fd::front_right), "rear bias locks the rear wheels only");
    require(std::abs(fd::rear_sideslip_rad(rear_end)) > 0.3, "with the rear locked the car's yaw diverges into a spin");
}

void wheelspin_at_low_speed_and_the_power_limit_at_high_speed() {
    fd::FourWheelCar rear_drive;
    rear_drive.drive_front_fraction = 0;
    auto s = drive(rear_drive, rolling(rear_drive, 4), {1.5*fd::gravity_mps2, 0}, 0.4);
    const double v0 = s.pose.speed_mps;
    s = drive(rear_drive, s, {1.5*fd::gravity_mps2, 0}, 0.3);
    const auto d = fd::demand(rear_drive, s, config, 1.5*fd::gravity_mps2);
    const double peak_ratio = fd::tire_peak(rear_drive.rear_tire, wheel_load(rear_drive, false)).slip_ratio;
    require(d.wheel_slip_ratios[fd::rear_left] > peak_ratio && d.wheel_slip_ratios[fd::rear_right] > peak_ratio,
            "full power at low speed spins the rear wheels past their peak slip");
    // Undriven wheels only need the small force that spins them up with the car.
    require(std::abs(d.wheel_slip_ratios[fd::front_left]) < 0.01, "the undriven front wheels roll");
    const double measured = (s.pose.speed_mps-v0)/0.3;
    double rear_peak = 0;  // at the rear wheels' loads, which acceleration has raised
    for (const std::size_t i : {fd::rear_left, fd::rear_right})
        rear_peak += fd::tire_peak(rear_drive.rear_tire, s.wheel_loads_n[i]).friction_longitudinal*s.wheel_loads_n[i]/rear_drive.mass_kg;
    std::cout << "  wheelspin: acceleration " << measured << " m/s^2 against a rear peak of " << rear_peak
              << ", rear slip ratio " << d.wheel_slip_ratios[fd::rear_left] << "\n";
    require(measured < rear_peak && measured > 0.6*rear_peak, "a spinning rear axle accelerates at its sliding force, below its peak");

    fd::FourWheelCar limited;
    limited.max_drive_power_w = 30000;
    auto fast = rolling(limited, 25);
    fast = drive(limited, fast, {4, 0}, 0.2);
    const double v1 = fast.pose.speed_mps;
    fast = drive(limited, fast, {4, 0}, 0.5);
    const double v_mid = 0.5*(v1+fast.pose.speed_mps);
    const double power_limited = (limited.max_drive_power_w/v_mid-drag_n(limited, v_mid))/equivalent_mass(limited);
    const double high = (fast.pose.speed_mps-v1)/0.5;
    std::cout << "  power limit: acceleration " << high << " m/s^2 at " << v_mid << " m/s, power over speed less drag " << power_limited
              << ", commanded 4\n";
    near(high, power_limited, 0.03*power_limited, "at speed the power limit sets the acceleration");
    const auto fd_fast = fd::demand(limited, fast, config, 4);
    for (std::size_t i = 0; i < 4; ++i) require(fd_fast.wheel_combined_slip[i] < 1, "power-limited driving does not spin a wheel");
}

// Energy: the kinetic energy of the chassis and wheels lost while braking equals the work of the brakes, the
// tires' slip work and drag, each computed from its own law at the recorded states.
void braking_energy_balances() {
    const fd::FourWheelCar car;
    const double deceleration = 0.85*fd::gravity_mps2;
    const double R = car.wheel_radius_m;
    const auto energy = [&](const fd::PlantState& s) {
        double e = 0.5*car.mass_kg*s.pose.speed_mps*s.pose.speed_mps;
        for (const double w : s.wheel_speeds_radps) e += 0.5*car.wheel_inertia_kgm2*w*w;
        return e;
    };
    const auto dissipation = [&](const fd::PlantState& s) {
        const double total_torque = car.mass_kg*deceleration*R;
        double power = 0;
        for (std::size_t i = 0; i < 4; ++i) {
            const bool front = i < 2;
            const double brake = std::min(car.max_brake_torque_nm, total_torque*(front ? car.brake_bias_front : 1-car.brake_bias_front)/2);
            const double u = s.pose.speed_mps, w = s.wheel_speeds_radps[i];
            const double kappa = (w*R-u)/std::max(u, car.slip_speed_floor_mps);
            const double fx = fd::tire_force(front ? car.front_tire : car.rear_tire, kappa, 0, s.wheel_loads_n[i]).longitudinal_n;
            power += brake*w-fx*(u-w*R);  // brake work plus the tire's slip work, both losses
        }
        return power+drag_n(car, s.pose.speed_mps)*s.pose.speed_mps;
    };
    auto s = rolling(car, 20);
    const double e0 = energy(s);
    double work = 0, previous = dissipation(s);
    for (int i = 0; i < 240; ++i) {
        s = fd::advance(car, s, {-deceleration, 0}, config, config.fixed_dt_s);
        const double now = dissipation(s);
        work += 0.5*(previous+now)*config.fixed_dt_s;
        previous = now;
    }
    const double lost = e0-energy(s);
    std::cout << "  braking energy: lost " << lost << " J, brake, slip and drag work " << work << " J\n";
    near(work, lost, 0.01*lost, "energy lost equals brake, tire slip and drag work");
}

void default_substep_is_converged() {
    // Lock onset falls on a substep boundary, so after a locked stop speed is only first order in the substep. So is
    // full-power wheelspin with load transfer, whose loads lag the acceleration by a substep (decision 0014).
    struct Case { std::string name; double speed, steering; fd::Command command; double seconds, speed_tolerance, brake_bias_front; };
    for (const auto& c : std::vector<Case>{{"pull-away from rest", 0, 0.1, {3, 0.1}, 3.0, 1e-3, 0.5},
                                           {"locked braking in a turn", 16, 0.04, {-2.5*fd::gravity_mps2, 0.04}, 1.2, 5e-3, 0.5},
                                           {"threshold braking", 20, 0, {-0.85*fd::gravity_mps2, 0}, 1.5, 1e-3, 0.62},
                                           {"wheelspin", 4, 0.05, {1.5*fd::gravity_mps2, 0.05}, 1.0, 1e-2, 0.5},
                                           {"cornering", 12, 0.06, {0.5, 0.06}, 3.0, 1e-3, 0.5}}) {
        fd::FourWheelCar coarse;
        coarse.brake_bias_front = c.brake_bias_front;
        auto fine = coarse;
        fine.substep_s = 0.0002;
        const auto a = drive(coarse, rolling(coarse, c.speed, c.steering), c.command, c.seconds);
        const auto b = drive(fine, rolling(fine, c.speed, c.steering), c.command, c.seconds);
        const double position = std::hypot(a.pose.x_m-b.pose.x_m, a.pose.y_m-b.pose.y_m);
        std::cout << "  " << c.name << ": position difference " << position << " m, speed " << std::abs(a.pose.speed_mps-b.pose.speed_mps)
                  << " m/s, yaw rate " << std::abs(a.yaw_rate_radps-b.yaw_rate_radps) << " rad/s\n";
        near(position, 0, 0.01, c.name+": position within 1 cm of the fine substep");
        near(a.pose.speed_mps, b.pose.speed_mps, c.speed_tolerance, c.name+": speed within tolerance of the fine substep");
        near(a.yaw_rate_radps, b.yaw_rate_radps, 1e-3, c.name+": yaw rate within 1e-3 rad/s");
    }
}

void standing_start_and_stop_stay_physical() {
    const fd::FourWheelCar car;
    auto s = fd::plant_state_from({}, car, config);
    double lowest_wheel = 0;
    bool finite = true;
    const auto watch = [&](const fd::PlantState& p) {
        for (const double w : p.wheel_speeds_radps) { lowest_wheel = std::min(lowest_wheel, w); finite = finite && std::isfinite(w); }
        finite = finite && std::isfinite(p.pose.speed_mps) && p.pose.speed_mps >= 0;
    };
    s = drive(car, s, {4, 0}, 2.0, watch);
    require(s.pose.speed_mps > 7, "the car pulls away from rest");
    s = drive(car, s, {-8, 0}, 3.0, watch);
    require(s.pose.speed_mps == 0, "hard braking brings the car to rest");
    s = drive(car, s, {0, 0}, 1.0, watch);
    require(s.pose.speed_mps == 0 && s.pose.x_m == drive(car, s, {0, 0}, 0.5).pose.x_m, "at rest with no command the car does not creep");
    require(finite && lowest_wheel >= 0, "speeds stay finite and wheels never turn backward");
}

void invalid_cars_and_wheel_states_are_rejected() {
    const auto broken = [&](const std::function<void(fd::FourWheelCar&)>& change, const std::string& what) {
        fd::FourWheelCar car;
        change(car);
        rejects([&] { fd::validate_vehicle(car, config); }, what+" is rejected");
        rejects([&] { fd::advance(car, fd::PlantState{}, {}, config, config.fixed_dt_s); }, what+" is rejected when advancing");
    };
    broken([](auto& c) { c.mass_kg = 0; }, "zero mass");
    broken([](auto& c) { c.yaw_inertia_kgm2 = -1; }, "negative yaw inertia");
    broken([](auto& c) { c.cg_to_rear_m = 2.6; }, "a centre of gravity on the front axle");
    broken([](auto& c) { c.track_front_m = 0; }, "a zero front track");
    broken([](auto& c) { c.track_rear_m = 5; }, "an implausible rear track");
    broken([](auto& c) { c.wheel_radius_m = 0; }, "a zero wheel radius");
    broken([](auto& c) { c.wheel_inertia_kgm2 = 0; }, "a massless wheel");
    broken([](auto& c) { c.drive_front_fraction = -0.1; }, "a negative drive share");
    broken([](auto& c) { c.brake_bias_front = 1.2; }, "a brake bias beyond the front axle");
    broken([](auto& c) { c.max_drive_power_w = 0; }, "no drive power");
    broken([](auto& c) { c.max_drive_torque_nm = -5; }, "negative drive torque");
    broken([](auto& c) { c.max_brake_torque_nm = 0; }, "no brakes");
    broken([](auto& c) { c.dynamic_above_mps = c.kinematic_below_mps; }, "an empty blend band");
    broken([](auto& c) { c.substep_s = 0.02; }, "a substep longer than the plant tick");
    broken([](auto& c) { c.rear_tire.sliding_fraction_lateral = 2; }, "an invalid tire");
    const fd::FourWheelCar car;
    auto spinning_backward = rolling(car, 5);
    spinning_backward.wheel_speeds_radps[fd::rear_left] = -1;
    rejects([&] { fd::advance(car, spinning_backward, {}, config, config.fixed_dt_s); }, "a wheel turning backward is rejected");
    auto nan = rolling(car, 5);
    nan.wheel_speeds_radps[fd::front_right] = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { fd::advance(car, nan, {}, config, config.fixed_dt_s); }, "a non-finite wheel speed is rejected");
    require(std::string(fd::vehicle_model_name(car)) == "four-wheel car with rotating wheels, load transfer and aerodynamics, rear axle reference",
            "the model names itself");
}

// Rear-wheel drive out of a left turn at 10 m/s. Load transfer unloads the inner (left) rear wheel, which spins
// up under its half of the torque with an open differential; a viscous coupling moves torque from it to the
// loaded outer wheel.
struct CornerExit { double speed_gain{}, largest_spread{}, inner_slip{}, sideslip{}; };
CornerExit corner_exit(double coupling, double command) {
    fd::FourWheelCar car;
    car.drive_front_fraction = 0;
    car.viscous_coupling_nms = coupling;
    const double steer = 0.12;
    CornerExit result;
    auto s = drive(car, rolling(car, 10, steer), {0.2, steer}, 2.0);
    const double v0 = s.pose.speed_mps;
    s = drive(car, s, {command, steer}, 0.6, [&](const fd::PlantState& p) {
        result.largest_spread = std::max(result.largest_spread, std::abs(p.wheel_speeds_radps[fd::rear_left]-p.wheel_speeds_radps[fd::rear_right]));
    });
    result.speed_gain = s.pose.speed_mps-v0;
    result.inner_slip = fd::demand(car, s, config, command).wheel_slip_ratios[fd::rear_left];
    result.sideslip = fd::rear_sideslip_rad(s);
    return result;
}
void viscous_coupling_moves_torque_to_the_slower_wheel() {
    const auto report = [](const char* what, const CornerExit& e) {
        std::cout << "  " << what << ": gains " << e.speed_gain << " m/s, rear wheels up to " << e.largest_spread << " rad/s apart, inner rear slip ratio "
                  << e.inner_slip << ", rear sideslip " << e.sideslip << " rad\n";
    };
    const auto open = corner_exit(0, 0.45*fd::gravity_mps2);
    const auto coupled = corner_exit(40, 0.45*fd::gravity_mps2);
    report("0.45 g out of the corner, open differential", open);
    report("0.45 g out of the corner, 40 N m s/rad coupling", coupled);
    const double peak = fd::tire_peak(fd::FourWheelCar{}.rear_tire, 2000).slip_ratio;
    require(open.inner_slip > 5*peak, "with an open differential the unloaded inner rear wheel spins far past its peak");
    require(coupled.inner_slip < 0.3*open.inner_slip && coupled.largest_spread < 0.1*open.largest_spread,
            "the coupling holds the inner wheel near the outer one");
    require(coupled.speed_gain > 1.05*open.speed_gain, "torque moved to the loaded wheel accelerates the car harder");
    require(std::abs(coupled.sideslip) < std::abs(open.sideslip), "and with less rear sideslip");
    // With more throttle than the outer wheel can take, the coupling spins both rear wheels: more power oversteer.
    const auto open_hard = corner_exit(0, 0.6*fd::gravity_mps2);
    const auto coupled_hard = corner_exit(40, 0.6*fd::gravity_mps2);
    report("0.6 g out of the corner, open differential", open_hard);
    report("0.6 g out of the corner, 40 N m s/rad coupling", coupled_hard);
    require(std::abs(coupled_hard.sideslip) > std::abs(open_hard.sideslip)+0.1,
            "once both rear wheels are past their peak the coupling makes the car oversteer harder");

    // In a straight line both wheels of an axle turn alike, so the coupling has nothing to transmit.
    fd::FourWheelCar free_axle, locked_axle;
    locked_axle.viscous_coupling_nms = 40;
    const auto a = drive(free_axle, rolling(free_axle, 10), {3, 0}, 1.0);
    const auto b = drive(locked_axle, rolling(locked_axle, 10), {3, 0}, 1.0);
    near(b.pose.x_m, a.pose.x_m, 1e-9, "in a straight line the coupling changes nothing");
    near(b.wheel_speeds_radps[fd::rear_left], a.wheel_speeds_radps[fd::rear_left], 1e-9, "in a straight line the wheels turn as with an open differential");
}

// The drive controller's top speed (decision 0038): with the accelerator floored the car settles just below it, where the
// drive falls away, while the same car without a limit runs on past it; braking is untouched; a top speed that is not
// positive is refused and infinity, the default, is none.
void the_drive_controllers_top_speed_holds_the_car() {
    fd::FourWheelCar free;
    fd::FourWheelCar limited = free;
    limited.max_drive_speed_mps = 15;
    auto a = rolling(limited, 10), b = rolling(free, 10);
    for (int k = 0; k < 20*200; ++k) {
        a = fd::advance(limited, a, {fd::gravity_mps2, 0}, config, config.fixed_dt_s);
        b = fd::advance(free, b, {fd::gravity_mps2, 0}, config, config.fixed_dt_s);
    }
    require(a.pose.speed_mps > 14 && a.pose.speed_mps <= 15.0+1e-9, "the limited car settles just below its top speed: "+std::to_string(a.pose.speed_mps));
    require(b.pose.speed_mps > 25, "the same car without a limit runs on past it: "+std::to_string(b.pose.speed_mps));
    auto c = rolling(limited, 20), d = rolling(free, 20);
    for (int k = 0; k < 200; ++k) {
        c = fd::advance(limited, c, {-5, 0}, config, config.fixed_dt_s);
        d = fd::advance(free, d, {-5, 0}, config, config.fixed_dt_s);
    }
    near(c.pose.speed_mps, d.pose.speed_mps, 1e-12, "braking is untouched by the drive's limit");
    for (const double bad : {0.0, -3.0, std::numeric_limits<double>::quiet_NaN()}) {
        fd::FourWheelCar wrong;
        wrong.max_drive_speed_mps = bad;
        rejects([&] { fd::validate_vehicle(wrong, config); }, "a top speed that is not positive is refused");
    }
    require(!std::isfinite(fd::FourWheelCar{}.max_drive_speed_mps), "the default is no limit");
}

void completes_a_lap_with_four_rotating_wheels() {
    const auto track = fd::make_preset_track();
    fd::Simulation simulation(track, config, fd::FourWheelCar{});
    double max_error = 0, max_slip = 0;
    int sliding = 0;
    const auto began = std::chrono::steady_clock::now();
    while (simulation.diagnostics().laps < 1 && simulation.state().time_s < 120) {
        simulation.step();
        max_error = std::max(max_error, std::abs(simulation.diagnostics().cross_track_error_m));
        max_slip = std::max(max_slip, std::abs(simulation.diagnostics().rear_sideslip_rad));
        if (!simulation.diagnostics().within_grip_envelope) ++sliding;
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
    std::cout << "  four-wheel lap " << simulation.state().time_s << " s, max tracking error " << max_error << " m, max rear sideslip "
              << max_slip << " rad, ticks past a tire peak " << sliding << " (" << wall << " s wall; measurement only)\n";
    require(simulation.diagnostics().laps == 1, "the unchanged controller laps the four-wheel car");
    require(max_error < track.width_m/2-fd::vehicle_half_width_m, "the car stays within the track margin");
    const auto& plant = simulation.plant_state();
    {
        // Cost of one prediction and of a decision among six lines, as the planner makes them.
        fd::Simulation corner(track, config, fd::FourWheelCar{});
        for (int i = 0; i < 1800; ++i) corner.step();
        const auto plan = fd::make_speed_plan(track, config);
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < 20; ++i) (void)fd::make_local_trajectory(track, plan, corner.plant_state(), config, corner.vehicle_model());
        const auto t1 = std::chrono::steady_clock::now();
        for (int i = 0; i < 5; ++i)
            (void)fd::choose_local_action(track, plan, corner.plant_state(), config, corner.vehicle_model(), {{140, 146, -5, 5, "stalled-car"}});
        const auto t2 = std::chrono::steady_clock::now();
        std::cout << "  four-wheel prediction " << std::chrono::duration<double, std::milli>(t1-t0).count()/20 << " ms, blocked decision "
              << std::chrono::duration<double, std::milli>(t2-t1).count()/5 << " ms (measurement only)" << std::endl;
    }
    require(std::all_of(plant.wheel_speeds_radps.begin(), plant.wheel_speeds_radps.end(), [](double w) { return w > 0; }),
            "the simulation carries the wheel speeds");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"free_rolling_wheels_turn_like_the_single_track_car", free_rolling_wheels_turn_like_the_single_track_car},
        {"driving_torque_accelerates_the_car_and_its_wheels", driving_torque_accelerates_the_car_and_its_wheels},
        {"threshold_braking_stops_shorter_than_locked_braking", threshold_braking_stops_shorter_than_locked_braking},
        {"locked_wheels_slide_at_the_tires_sliding_force", locked_wheels_slide_at_the_tires_sliding_force},
        {"brake_bias_decides_which_axle_locks", brake_bias_decides_which_axle_locks},
        {"wheelspin_at_low_speed_and_the_power_limit_at_high_speed", wheelspin_at_low_speed_and_the_power_limit_at_high_speed},
        {"braking_energy_balances", braking_energy_balances},
        {"default_substep_is_converged", default_substep_is_converged},
        {"standing_start_and_stop_stay_physical", standing_start_and_stop_stay_physical},
        {"invalid_cars_and_wheel_states_are_rejected", invalid_cars_and_wheel_states_are_rejected},
        {"viscous_coupling_moves_torque_to_the_slower_wheel", viscous_coupling_moves_torque_to_the_slower_wheel},
        {"the_drive_controllers_top_speed_holds_the_car", the_drive_controllers_top_speed_holds_the_car},
        {"completes_a_lap_with_four_rotating_wheels", completes_a_lap_with_four_rotating_wheels},
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " wheel groups passed\n";
    return failures ? 1 : 0;
}

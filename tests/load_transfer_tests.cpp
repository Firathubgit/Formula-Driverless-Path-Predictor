#include "fd/simulation.hpp"
#include "fd/vehicle.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Phase 2.2: quasi-static load transfer on the four-wheel car (decision 0014). Oracles are independent of
// the implementation: the four balance equations themselves, TUM's hand-written closed form as the plan
// quotes it, the textbook transfers m a h / L and m a_y h / track at accelerations measured from the
// motion, the brake bias that matches each axle's braking load, and a fine substep.
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

double weight(const fd::FourWheelCar& car) { return car.mass_kg*fd::gravity_mps2; }
double total(const std::array<double, 4>& loads) { return loads[0]+loads[1]+loads[2]+loads[3]; }
double equivalent_mass(const fd::FourWheelCar& car) {
    return car.mass_kg+4*car.wheel_inertia_kgm2/(car.wheel_radius_m*car.wheel_radius_m);
}
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
// Lateral acceleration of the centre of gravity between two consecutive ticks, in the body frame.
double lateral_acceleration(const fd::FourWheelCar& car, const fd::PlantState& before, const fd::PlantState& after) {
    const double lr = car.cg_to_rear_m;
    const double lateral_rate = ((after.lateral_velocity_mps+lr*after.yaw_rate_radps)-(before.lateral_velocity_mps+lr*before.yaw_rate_radps))/config.fixed_dt_s;
    return lateral_rate+0.5*(after.pose.speed_mps*after.yaw_rate_radps+before.pose.speed_mps*before.yaw_rate_radps);
}

void loads_balance_weight_pitch_and_roll() {
    std::mt19937 random(2022);
    std::uniform_real_distribution<double> unit(0, 1), signed_unit(-1, 1);
    int balanced = 0, lifted = 0;
    for (int i = 0; i < 5000; ++i) {
        fd::FourWheelCar car;
        car.cg_to_rear_m = 0.8+unit(random);
        car.track_front_m = 1.2+0.6*unit(random);
        car.track_rear_m = 1.2+0.6*unit(random);
        car.cg_height_m = unit(random)*0.5*std::min(car.track_front_m, car.track_rear_m);
        car.roll_balance_front = unit(random);
        const double W = weight(car), L = config.wheelbase_m, lr = car.cg_to_rear_m, lf = L-lr;
        const double h = car.cg_height_m, D = car.roll_balance_front, tf = car.track_front_m, tr = car.track_rear_m;
        const double fx = 1.5*W*signed_unit(random), fy = 1.5*W*signed_unit(random);
        const auto z = fd::quasi_static_wheel_loads(car, config, fx, fy, 0);
        for (const double load : z) require(std::isfinite(load) && load >= 0, "loads are finite and never negative");
        if (std::any_of(z.begin(), z.end(), [](double load) { return load == 0; })) {
            ++lifted;
            require(total(z) >= W*(1-1e-12), "a clamped wheel can only add to the loads");
            continue;
        }
        ++balanced;
        near(total(z), W, 1e-9*W, "loads sum to weight");
        near(lf*(z[fd::front_left]+z[fd::front_right])-lr*(z[fd::rear_left]+z[fd::rear_right]), -h*fx, 1e-9*W*L,
             "the pitch moment about the centre of gravity balances");
        near(tf/2*(z[fd::front_left]-z[fd::front_right])+tr/2*(z[fd::rear_left]-z[fd::rear_right]), -h*fy, 1e-9*W*L,
             "the roll moment about the centre of gravity balances");
        near((z[fd::front_right]-z[fd::front_left])*(1-D)+(z[fd::rear_left]-z[fd::rear_right])*D, 0, 1e-9*W,
             "the roll-balance closure holds");
    }
    std::cout << "  " << balanced << " balanced load sets, " << lifted << " with a lifted wheel\n";
    require(balanced > 1000 && lifted > 100, "the sample covers lifted wheels and balanced ones");

    const fd::FourWheelCar car;
    const auto still = fd::quasi_static_wheel_loads(car, config, 0, 0, 0);
    const double L = config.wheelbase_m;
    near(still[fd::front_left], weight(car)*car.cg_to_rear_m/L/2, 1e-9, "without force a front wheel carries its static load");
    near(still[fd::rear_right], weight(car)*(L-car.cg_to_rear_m)/L/2, 1e-9, "without force a rear wheel carries its static load");
}

// TUM's opt_mintime closed form, as TrackWayFastPlan section 4.2 quotes it, with forces in each wheel's
// own frame and this project's signs: braking loads the front, a left turn loads the right-hand wheels.
std::array<double, 4> tum_closed_form(const fd::FourWheelCar& car, const std::array<double, 4>& fx,
                                      const std::array<double, 4>& fy, double steering) {
    const double L = config.wheelbase_m, lr = car.cg_to_rear_m, lf = L-lr, h = car.cg_height_m, k = car.roll_balance_front;
    const double t_mean = 0.5*(car.track_front_m+car.track_rear_m);
    const double front_static = 0.5*car.mass_kg*fd::gravity_mps2*lr/L, rear_static = 0.5*car.mass_kg*fd::gravity_mps2*lf/L;
    const double longitudinal = 0.5*h/L*(fx[0]+fx[1]+fx[2]+fx[3]);
    const double gamma = h/t_mean*((fy[0]+fy[1])*std::cos(steering)+fy[2]+fy[3]+(fx[0]+fx[1])*std::sin(steering));
    return {front_static-longitudinal-k*gamma, front_static-longitudinal+k*gamma,
            rear_static+longitudinal-(1-k)*gamma, rear_static+longitudinal+(1-k)*gamma};
}

void agrees_with_tums_closed_form() {
    std::mt19937 random(7);
    std::uniform_real_distribution<double> unit(0, 1), signed_unit(-1, 1);
    struct Case { std::string name; bool unequal_tracks, steered; };
    for (const auto& c : std::vector<Case>{{"equal tracks, wheels straight", false, false},
                                           {"equal tracks, front wheels steered", false, true},
                                           {"unequal tracks, wheels straight", true, false}}) {
        double largest_difference = 0, largest_term = 0;
        int compared = 0;
        for (int i = 0; i < 2000; ++i) {
            fd::FourWheelCar car;
            car.cg_height_m = 0.1+0.25*unit(random);
            car.roll_balance_front = unit(random);
            car.cg_to_rear_m = 1.0+0.6*unit(random);
            if (c.unequal_tracks) { car.track_front_m = 1.2+0.6*unit(random); car.track_rear_m = 1.2+0.6*unit(random); }
            const double W = weight(car), L = config.wheelbase_m, h = car.cg_height_m;
            const double delta = c.steered ? 0.35*signed_unit(random) : 0.0;
            std::array<double, 4> fx{}, fy{};
            for (std::size_t w = 0; w < 4; ++w) { fx[w] = 0.2*W*signed_unit(random); fy[w] = 0.2*W*signed_unit(random); }
            const double cs = std::cos(delta), sn = std::sin(delta);
            const double body_x = (fx[0]+fx[1])*cs-(fy[0]+fy[1])*sn+fx[2]+fx[3];
            const double body_y = (fx[0]+fx[1])*sn+(fy[0]+fy[1])*cs+fy[2]+fy[3];
            const auto ours = fd::quasi_static_wheel_loads(car, config, body_x, body_y, 0);
            if (std::any_of(ours.begin(), ours.end(), [](double load) { return load == 0; })) continue;
            const auto theirs = tum_closed_form(car, fx, fy, delta);
            // What TUM drops: the steered front forces' longitudinal components in pitch, and the exact roll
            // moment arms, for which it uses the mean track.
            const double t_min = std::min(car.track_front_m, car.track_rear_m);
            const double term = 0.5*h/L*std::abs((fx[0]+fx[1])*(1-cs)+(fy[0]+fy[1])*sn)+
                                h*std::abs(body_y)*std::abs(car.track_front_m-car.track_rear_m)/(2*t_min*t_min);
            for (std::size_t w = 0; w < 4; ++w) {
                const double difference = std::abs(ours[w]-theirs[w]);
                require(difference <= term+1e-9*W, c.name+": our loads and TUM's differ by no more than the terms TUM drops");
                largest_difference = std::max(largest_difference, difference);
            }
            largest_term = std::max(largest_term, term);
            ++compared;
        }
        std::cout << "  " << c.name << ": " << compared << " load sets, largest difference " << largest_difference
                  << " N, largest dropped term " << largest_term << " N\n";
        require(compared > 1500, c.name+": most random load sets are compared");
        if (!c.unequal_tracks && !c.steered) require(largest_difference < 1e-6, "with equal tracks and straight wheels the two forms agree");
        else require(largest_term > 10, c.name+": the dropped terms are large enough to matter in the comparison");
    }
}

void braking_and_driving_move_load_between_the_axles() {
    fd::FourWheelCar car;
    const double L = config.wheelbase_m, W = weight(car), static_front = W*car.cg_to_rear_m/L;
    struct Case { std::string name; double speed, command; };
    for (const auto& c : std::vector<Case>{{"braking at 0.5 g", 20, -0.5*fd::gravity_mps2}, {"driving at 2.5 m/s^2", 8, 2.5}}) {
        const auto before = drive(car, rolling(car, c.speed), {c.command, 0}, 0.995);
        const auto after = fd::advance(car, before, {c.command, 0}, config, config.fixed_dt_s);
        const double a = (after.pose.speed_mps-before.pose.speed_mps)/config.fixed_dt_s;
        // The chassis accelerates under the tires' force and drag. Drag acts through the centre of gravity, so only
        // the tires' part of the force, mass times acceleration plus drag, moves load (decision 0015).
        const double v = 0.5*(after.pose.speed_mps+before.pose.speed_mps);
        const double tire_force = car.mass_kg*a+0.5*1.225*car.drag_area_m2*v*v;
        const double transfer = tire_force*car.cg_height_m/L;
        const double front = after.wheel_loads_n[fd::front_left]+after.wheel_loads_n[fd::front_right];
        std::cout << "  " << c.name << ": measured " << a << " m/s^2, front axle " << front << " N against static " << static_front
                  << " N less the tire force times h / L = " << transfer << " N\n";
        near(front, static_front-transfer, 0.01*std::abs(transfer), c.name+": the front axle gains the tire force times h / L under braking and loses it driving");
        near(total(after.wheel_loads_n), W, 1e-9*W, c.name+": loads sum to weight");
        near(after.wheel_loads_n[fd::front_left], after.wheel_loads_n[fd::front_right], 1e-9*W, c.name+": left and right carry equal loads");
    }
}

void cornering_moves_load_outward_as_the_roll_balance_splits_it() {
    for (const double balance : {0.3, 0.5, 0.8}) {
        fd::FourWheelCar car;
        car.roll_balance_front = balance;
        const double steer = 0.05;
        const auto before = drive(car, rolling(car, 14, 0), {0, steer}, 3.995);
        const auto after = fd::advance(car, before, {0, steer}, config, config.fixed_dt_s);
        const double ay = lateral_acceleration(car, before, after);
        const auto& z = after.wheel_loads_n;
        const double outward = 0.5*((z[fd::front_right]+z[fd::rear_right])-(z[fd::front_left]+z[fd::rear_left]));
        const double expected = car.mass_kg*ay*car.cg_height_m/car.track_front_m;
        const double front_share = (z[fd::front_right]-z[fd::front_left])/((z[fd::front_right]-z[fd::front_left])+(z[fd::rear_right]-z[fd::rear_left]));
        std::cout << "  roll balance " << balance << ": lateral acceleration " << ay << " m/s^2, load moved outward " << outward
                  << " N against m a_y h / track " << expected << " N, front share " << front_share << "\n";
        require(ay > 3, "the car is cornering firmly to the left");
        near(outward, expected, 0.01*expected, "cornering moves m a_y h / track outward");
        near(front_share, balance, 1e-9, "the front axle takes the configured share of lateral transfer");
        near(total(z), weight(car), 1e-9*weight(car), "loads still sum to weight");
    }
}

// Load sensitivity: a tire's friction falls as its load rises, so the axle that takes more of the lateral
// transfer loses more of its grip. Moving roll balance forward makes the car understeer more.
void roll_balance_moves_the_cars_balance() {
    const auto settle = [](double balance) {
        fd::FourWheelCar car;
        car.roll_balance_front = balance;
        const double speed = 15, steer = 0.085;
        auto s = rolling(car, speed, 0);
        for (int i = 0; i < 1000; ++i) s = fd::advance(car, s, {2*(speed-s.pose.speed_mps), steer}, config, config.fixed_dt_s);
        const auto d = fd::demand(car, s, config, 0);
        return std::pair{d.lateral_acceleration_mps2, fd::handling_balance(d).understeer_angle_rad};
    };
    const auto [rear_ay, rear_angle] = settle(0.2);
    const auto [even_ay, even_angle] = settle(0.5);
    const auto [front_ay, front_angle] = settle(0.8);
    std::cout << "  near the limit: roll balance 0.2 gives " << rear_ay << " m/s^2 at an understeer angle of " << rear_angle
              << " rad, 0.5 gives " << even_ay << " at " << even_angle << ", 0.8 gives " << front_ay << " at " << front_angle << "\n";
    require(rear_ay > 6 && front_ay > 6, "the car corners near its limit");
    require(rear_angle < even_angle && even_angle < front_angle, "understeer grows as roll balance moves forward");
    require(front_angle-rear_angle > 0.0015, "the roll balance changes the understeer angle measurably");
}

// The bias that brakes both axles to the same share of their grip follows the braking load, and wheel
// inertia: each wheel's torque also slows the wheel itself.
struct Stop { double distance_m{}; bool front_locked{}, rear_locked{}; };
Stop stop(const fd::FourWheelCar& car, double command) {
    Stop result;
    auto s = rolling(car, 20);
    double seconds = 0;
    while (s.pose.speed_mps > 3 && seconds < 20) {
        s = fd::advance(car, s, {-command, 0}, config, config.fixed_dt_s);
        seconds += config.fixed_dt_s;
        result.front_locked = result.front_locked || s.wheel_speeds_radps[fd::front_left] == 0 || s.wheel_speeds_radps[fd::front_right] == 0;
        result.rear_locked = result.rear_locked || s.wheel_speeds_radps[fd::rear_left] == 0 || s.wheel_speeds_radps[fd::rear_right] == 0;
    }
    result.distance_m = s.pose.x_m;
    return result;
}
void centre_of_gravity_height_moves_the_ideal_brake_bias() {
    const double command = 0.95*fd::gravity_mps2;
    fd::FourWheelCar flat;
    flat.cg_height_m = 0;
    const auto flat_stop = stop(flat, command);
    const fd::FourWheelCar tall;
    const auto static_bias = stop(tall, command);
    auto matched = tall;
    const double L = config.wheelbase_m, R = matched.wheel_radius_m;
    const double achieved = matched.mass_kg*command/equivalent_mass(matched);
    const double front_share = (matched.cg_to_rear_m+matched.cg_height_m*achieved/fd::gravity_mps2)/L;
    matched.brake_bias_front = (matched.mass_kg*front_share+2*matched.wheel_inertia_kgm2/(R*R))/equivalent_mass(matched);
    const auto matched_stop = stop(matched, command);
    std::cout << "  0.95 g from 20 to 3 m/s: no load transfer " << flat_stop.distance_m << " m, rear locked " << flat_stop.rear_locked
              << "; 0.35 m high at bias 0.5 " << static_bias.distance_m << " m, rear locked " << static_bias.rear_locked
              << "; at the matched bias " << matched.brake_bias_front << " " << matched_stop.distance_m << " m, locked "
              << (matched_stop.front_locked || matched_stop.rear_locked) << "\n";
    require(!flat_stop.front_locked && !flat_stop.rear_locked, "without load transfer the static bias brakes at 0.95 g without locking");
    require(static_bias.rear_locked && !static_bias.front_locked, "with load transfer the static bias locks the unloaded rear wheels");
    require(!matched_stop.front_locked && !matched_stop.rear_locked, "the bias matched to the braking loads locks no wheel");
    require(matched_stop.distance_m < static_bias.distance_m, "the matched bias stops shorter than the static one");
}

void a_high_centre_of_gravity_lifts_the_inner_front_wheel() {
    fd::FourWheelCar tall;
    tall.cg_height_m = 0.75;
    tall.roll_balance_front = 1;
    const double W = weight(tall);
    bool lifted = false, balanced_before = true, rear_even = true, finite = true;
    drive(tall, rolling(tall, 12, 0), {0, 0.1}, 3.0, [&](const fd::PlantState& s) {
        const auto& z = s.wheel_loads_n;
        for (const double load : z) finite = finite && std::isfinite(load) && load >= 0;
        rear_even = rear_even && std::abs(z[fd::rear_left]-z[fd::rear_right]) < 1e-9*W;
        if (z[fd::front_left] == 0) {
            if (!lifted) {
                const auto d = fd::demand(tall, s, config, 0);
                require(d.wheel_longitudinal_use[fd::front_left] == 0 && d.wheel_lateral_use[fd::front_left] == 0,
                        "a lifted wheel transmits no force");
                require(!d.within_envelope, "a lifted wheel puts the car outside its envelope");
                require(!fd::envelope(tall, s, config, 0).within, "the envelope query agrees about a lifted wheel");
                require(total(z) > W, "only a clamped wheel lets the loads exceed weight");
                std::cout << "  inner front wheel lifted at " << s.pose.time_s << " s, " << s.pose.speed_mps << " m/s; outer front "
                          << z[fd::front_right] << " N, rear " << z[fd::rear_left] << " N each\n";
            }
            lifted = true;
        } else if (!lifted) {
            balanced_before = balanced_before && std::abs(total(z)-W) < 1e-9*W;
        }
    });
    require(finite, "loads stay finite and never negative");
    require(lifted, "a tall car with all roll transfer on the front lifts its inner front wheel in a firm turn");
    require(balanced_before, "until then loads sum to weight");
    require(rear_even, "with no roll transfer at the rear both rear wheels carry the same load");
}

void load_transfer_is_converged_with_the_substep() {
    struct Case { std::string name; fd::FourWheelCar car; double speed, steering; fd::Command command; double seconds; };
    fd::FourWheelCar tall;
    tall.cg_height_m = 0.75;
    tall.roll_balance_front = 1;
    fd::FourWheelCar rear_drive;
    rear_drive.drive_front_fraction = 0;
    fd::FourWheelCar matched;
    matched.brake_bias_front = 0.62;
    for (const auto& c : std::vector<Case>{{"hard braking into a turn at the matched bias", matched, 16, 0.05, {-0.8*fd::gravity_mps2, 0.05}, 1.2},
                                           {"threshold braking at the matched bias", matched, 20, 0, {-0.9*fd::gravity_mps2, 0}, 1.5},
                                           {"turning in until a wheel lifts", tall, 12, 0, {0, 0.1}, 3.0},
                                           {"rear-wheel drive out of a corner", rear_drive, 8, 0.08, {3, 0.02}, 2.0}}) {
        auto fine = c.car;
        fine.substep_s = 0.0002;
        const auto a = drive(c.car, rolling(c.car, c.speed, c.steering), c.command, c.seconds);
        const auto b = drive(fine, rolling(fine, c.speed, c.steering), c.command, c.seconds);
        const double position = std::hypot(a.pose.x_m-b.pose.x_m, a.pose.y_m-b.pose.y_m);
        double load = 0;
        for (std::size_t w = 0; w < 4; ++w) load = std::max(load, std::abs(a.wheel_loads_n[w]-b.wheel_loads_n[w]));
        std::cout << "  " << c.name << ": position difference " << position << " m, speed " << std::abs(a.pose.speed_mps-b.pose.speed_mps)
                  << " m/s, yaw rate " << std::abs(a.yaw_rate_radps-b.yaw_rate_radps) << " rad/s, load " << load << " N\n";
        near(position, 0, 0.01, c.name+": position within 1 cm of the fine substep");
        near(a.pose.speed_mps, b.pose.speed_mps, 1e-3, c.name+": speed within 1e-3 m/s of the fine substep");
        near(a.yaw_rate_radps, b.yaw_rate_radps, 1e-3, c.name+": yaw rate within 1e-3 rad/s");
        near(load, 0, 0.005*weight(c.car), c.name+": every load within 0.5% of weight");
    }
}

void zero_height_keeps_every_wheel_at_its_static_load() {
    fd::FourWheelCar flat;
    flat.cg_height_m = 0;
    const double L = config.wheelbase_m, W = flat.mass_kg*fd::gravity_mps2;
    const double front = W*flat.cg_to_rear_m/L/2, rear = W*(L-flat.cg_to_rear_m)/L/2;
    bool exact = true;
    const auto check = [&](const fd::PlantState& s) {
        exact = exact && s.wheel_loads_n[fd::front_left] == front && s.wheel_loads_n[fd::front_right] == front &&
                s.wheel_loads_n[fd::rear_left] == rear && s.wheel_loads_n[fd::rear_right] == rear;
    };
    auto s = rolling(flat, 16, 0.05);
    check(s);
    s = drive(flat, s, {0, 0.05}, 1.0, check);
    s = drive(flat, s, {-2.5*fd::gravity_mps2, 0.05}, 1.0, check);
    s = drive(flat, fd::plant_state_from({}, flat, config), {4, 0.2}, 2.0, check);
    require(exact, "with the centre of gravity on the ground every wheel keeps exactly its static load, braking, cornering and at walking pace");
    auto bare = flat;
    bare.drag_area_m2 = 0;
    require(std::string(fd::vehicle_model_name(bare)) == "four-wheel car with rotating wheels, rear axle reference",
            "without load transfer or aerodynamics the car keeps the name decision 0012 gave it");
    auto no_air = fd::FourWheelCar{};
    no_air.drag_area_m2 = 0;
    require(std::string(fd::vehicle_model_name(no_air)) == "four-wheel car with rotating wheels and load transfer, rear axle reference",
            "with load transfer the car names it");
    const auto turning = rolling(fd::FourWheelCar{}, 14, 0.05);
    require(turning.wheel_loads_n[fd::front_right] > turning.wheel_loads_n[fd::front_left],
            "a car started in a left turn starts with load on its right-hand wheels");
}

void invalid_load_transfer_is_rejected() {
    const auto broken = [&](const std::function<void(fd::FourWheelCar&)>& change, const std::string& what) {
        fd::FourWheelCar car;
        change(car);
        rejects([&] { fd::validate_vehicle(car, config); }, what+" is rejected");
        rejects([&] { fd::quasi_static_wheel_loads(car, config, 0, 0, 0); }, what+" is rejected by the load query");
        rejects([&] { fd::advance(car, rolling(fd::FourWheelCar{}, 5), {}, config, config.fixed_dt_s); }, what+" is rejected when advancing");
    };
    broken([](auto& c) { c.cg_height_m = -0.01; }, "a centre of gravity below the ground");
    broken([](auto& c) { c.cg_height_m = std::numeric_limits<double>::quiet_NaN(); }, "a non-finite centre of gravity height");
    broken([](auto& c) { c.cg_height_m = 0.76; }, "a centre of gravity above half the track");
    broken([](auto& c) { c.track_rear_m = 1.2; c.cg_height_m = 0.61; }, "a centre of gravity above half the narrower track");
    broken([](auto& c) { c.roll_balance_front = -0.1; }, "a negative roll balance");
    broken([](auto& c) { c.roll_balance_front = 1.1; }, "a roll balance beyond the front axle");
    broken([](auto& c) { c.roll_balance_front = std::numeric_limits<double>::infinity(); }, "a non-finite roll balance");
    fd::FourWheelCar edge;
    edge.cg_height_m = 0.75;
    edge.roll_balance_front = 0;
    fd::validate_vehicle(edge, config);
    edge.roll_balance_front = 1;
    fd::validate_vehicle(edge, config);
    const fd::FourWheelCar car;
    rejects([&] { fd::quasi_static_wheel_loads(car, config, std::numeric_limits<double>::quiet_NaN(), 0, 0); }, "a non-finite force is rejected");
    auto negative = rolling(car, 5);
    negative.wheel_loads_n[fd::rear_right] = -1;
    rejects([&] { fd::advance(car, negative, {}, config, config.fixed_dt_s); }, "a negative wheel load is rejected");
    auto nan = rolling(car, 5);
    nan.wheel_loads_n[fd::front_left] = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { fd::advance(car, nan, {}, config, config.fixed_dt_s); }, "a non-finite wheel load is rejected");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"loads_balance_weight_pitch_and_roll", loads_balance_weight_pitch_and_roll},
        {"agrees_with_tums_closed_form", agrees_with_tums_closed_form},
        {"braking_and_driving_move_load_between_the_axles", braking_and_driving_move_load_between_the_axles},
        {"cornering_moves_load_outward_as_the_roll_balance_splits_it", cornering_moves_load_outward_as_the_roll_balance_splits_it},
        {"roll_balance_moves_the_cars_balance", roll_balance_moves_the_cars_balance},
        {"centre_of_gravity_height_moves_the_ideal_brake_bias", centre_of_gravity_height_moves_the_ideal_brake_bias},
        {"a_high_centre_of_gravity_lifts_the_inner_front_wheel", a_high_centre_of_gravity_lifts_the_inner_front_wheel},
        {"load_transfer_is_converged_with_the_substep", load_transfer_is_converged_with_the_substep},
        {"zero_height_keeps_every_wheel_at_its_static_load", zero_height_keeps_every_wheel_at_its_static_load},
        {"invalid_load_transfer_is_rejected", invalid_load_transfer_is_rejected},
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " load transfer groups passed\n";
    return failures ? 1 : 0;
}

#include "fd/simulation.hpp"
#include "fd/steering.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
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

// The kinematic bicycle's steady turn is geometry: curvature tan(steering) / wheelbase at any speed.
// A table generated from it by simulation must return that law, which is independent of the
// generator, inside the bicycle's envelope: the steering limit, and lateral acceleration within
// mu g, beyond which the kinematic model is no longer valid and the table saturates.
void kinematic_table_is_the_geometric_law() {
    const fd::Config config;
    const fd::SteeringTable table(fd::KinematicBicycle{}, config);
    const double L = config.wheelbase_m, mu_g = config.grip_mu*fd::gravity_mps2;
    const double reachable = std::tan(config.max_steering_rad)/L;
    for (const double speed : {0.0, 0.4, 1.0, 2.5, 7.3, 12.0, 19.9, 22.0, 25.0}) {
        // Neighbouring rows each saturate at their own envelope; inside the faster one's the law holds.
        const double faster_row = speed <= 1.0 ? 1.0 : std::min(2*std::ceil(speed/2), 22.0);
        const double limit = speed <= 1.0 ? reachable : std::min(reachable, mu_g/(faster_row*faster_row));
        for (int i = -40; i <= 40; ++i) {
            const double curvature = 0.995*limit*i/40.0;
            near(table.steering_for(curvature, speed), std::atan(L*curvature), 5e-5,
                 "kinematic table steering at "+std::to_string(speed)+" m/s, curvature "+std::to_string(curvature));
        }
    }
    near(table.steering_for(2*reachable, 0.5), config.max_steering_rad, 1e-12, "a sharper turn than the steering allows saturates");
    near(table.steering_for(-2*reachable, 0.5), -config.max_steering_rad, 1e-12, "and symmetrically to the right");
    const double envelope_steering = std::atan(L*mu_g/144);
    const double at_limit = table.steering_for(1.0, 12);
    require(at_limit <= envelope_steering+1e-9 && at_limit > envelope_steering-1e-4,
            "at 12 m/s a turn beyond mu g saturates at the envelope's steering, found "+std::to_string(at_limit)+
            " against "+std::to_string(envelope_steering));
    require(table.steering_for(-1.0, 12) == -at_limit, "saturation is symmetric");
}

// Axle cornering stiffness measured from the tire law at the axle's static load, independent of
// the plant and of the table generator.
double axle_cornering_stiffness(const fd::TireParameters& tire, double axle_load_n) {
    const double h = 1e-6;
    return 2*(fd::tire_force(tire, 0, h, axle_load_n/2).lateral_n-fd::tire_force(tire, 0, -h, axle_load_n/2).lateral_n)/(2*h);
}
// A car whose front (or rear) tire is softer: a larger peak slip angle means less cornering stiffness.
// The soft front is the named setup the applications offer.
fd::DynamicSingleTrack soft(bool front) {
    if (front) return fd::DynamicSingleTrack::soft_front();
    fd::DynamicSingleTrack car;
    car.rear_tire.peak_slip_angle_low_rad = 0.2;
    car.rear_tire.peak_slip_angle_high_rad = 0.18;
    return car;
}
double seconds_since(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
}

// In gentle turns a dynamic car needs steering = curvature * (wheelbase + K v^2), with the
// understeer gradient K from the axles' cornering stiffness. That is what MAP adds to geometry.
void dynamic_table_follows_the_understeer_gradient() {
    const fd::Config config;
    for (const bool soft_front : {true, false}) {
        const auto car = soft(soft_front);
        const auto start = std::chrono::steady_clock::now();
        const fd::SteeringTable table(car, config);
        std::cout << "  " << (soft_front ? "understeering" : "oversteering") << " table: " << table.rows().size()
                  << " rows generated in " << seconds_since(start) << " s (measurement only)\n";
        const double L = config.wheelbase_m, lr = car.cg_to_rear_m, lf = L-lr, g = fd::gravity_mps2;
        const double Cf = axle_cornering_stiffness(car.front_tire, car.mass_kg*g*lr/L);
        const double Cr = axle_cornering_stiffness(car.rear_tire, car.mass_kg*g*lf/L);
        const double K = car.mass_kg/L*(lr/Cf-lf/Cr);
        for (const double speed : {6.0, 10.0, 14.0}) {
            const double curvature = 0.4/(speed*speed);  // 0.4 m/s^2: well inside the linear range
            const double expected = curvature*(L+K*speed*speed);
            const double steering = table.steering_for(curvature, speed);
            const std::string car_name = soft_front ? "understeering" : "oversteering";
            near(steering, expected, 0.02*expected, car_name+" steering at "+std::to_string(speed)+" m/s");
            // The part MAP adds to geometry: K times the lateral acceleration, positive for understeer.
            const double correction = steering-std::atan(L*curvature);
            near(correction, K*0.4, 0.1*std::abs(K*0.4), car_name+" correction to geometry at "+std::to_string(speed)+" m/s");
        }
    }
}

// MAP must not ask for a turn the car cannot hold. The largest steady turn in a fast row sits just
// inside the tires' peak friction, scales with road grip, and asking for more returns its steering.
void dynamic_table_stops_at_the_grip_limit() {
    const fd::DynamicSingleTrack car;
    const double peak_mu = fd::tire_peak(car.front_tire, car.mass_kg*fd::gravity_mps2/4).friction_lateral;
    double previous_limit = 0;
    for (const double grip : {0.5, 1.0}) {
        fd::Config config;
        config.grip_mu = grip;
        const fd::SteeringTable table(car, config);
        for (const auto& row : table.rows()) {
            for (std::size_t i = 1; i < row.turns.size(); ++i)
                require(row.turns[i].curvature_per_m > row.turns[i-1].curvature_per_m &&
                        row.turns[i].steering_rad > row.turns[i-1].steering_rad, "rows increase in steering and curvature");
            near(row.turns.back().lateral_acceleration_mps2, row.turns.back().curvature_per_m*row.speed_mps*row.speed_mps,
                 1e-9*row.speed_mps*row.speed_mps, "lateral acceleration is speed squared times curvature");
        }
        const auto& fast = table.rows().back();
        const double limit = fast.turns.back().lateral_acceleration_mps2;
        const double friction = grip*peak_mu*fd::gravity_mps2;
        std::cout << "  grip " << grip << ": largest steady turn at " << fast.speed_mps << " m/s is " << limit
                  << " m/s^2 of " << friction << " m/s^2 peak friction, steering " << fast.turns.back().steering_rad << " rad\n";
        require(limit < friction, "the largest steady turn stays inside peak friction");
        require(limit > 0.85*friction, "the table reaches close to the grip limit rather than stopping early");
        if (previous_limit > 0) near(limit, 2*previous_limit, 0.1*limit, "twice the grip holds about twice the lateral acceleration");
        previous_limit = limit;
        const double largest = fast.turns.back().curvature_per_m;
        near(table.steering_for(3*largest, fast.speed_mps), fast.turns.back().steering_rad, 1e-12,
             "asking for a sharper turn returns the steering of the largest steady one");
        double last = 0;
        for (int i = 0; i <= 200; ++i) {
            const double steering = table.steering_for(1.5*largest*i/200, 0.9*fast.speed_mps);
            require(steering >= last, "steering never decreases as the demanded turn tightens");
            last = steering;
        }
    }
}

// A table describes one plant under one configuration. Handing it to a prediction for any other
// is refused, and a simulation under MAP regenerates its table when grip changes at a tick.
void a_table_for_another_plant_is_rejected() {
    const fd::Config config;
    const auto track = fd::make_preset_track();
    const auto plan = fd::make_speed_plan(track, config);
    const fd::DynamicSingleTrack car;
    const fd::SteeringTable table(car, config);
    const auto plant = fd::plant_state_from({track.points[0].x_m, track.points[0].y_m, 0, 8, 0, 0}, car, config);
    require(table.generated_for(car, config), "a table matches the plant it came from");
    require(table.fingerprint().size() == 16 && table.fingerprint() == fd::steering_table_fingerprint(car, config),
            "its fingerprint is the plant's");
    fd::make_local_trajectory(track, plan, plant, config, car, 0, {}, &table);

    auto heavier = car;
    heavier.mass_kg = 1100;
    auto slippery = config;
    slippery.grip_mu = 0.7;
    auto rear_drive = car;
    rear_drive.drive_front_fraction = 0;
    require(fd::steering_table_fingerprint(heavier, config) != table.fingerprint() &&
            fd::steering_table_fingerprint(car, slippery) != table.fingerprint() &&
            fd::steering_table_fingerprint(rear_drive, config) != table.fingerprint() &&
            fd::steering_table_fingerprint(fd::KinematicBicycle{}, config) != table.fingerprint(),
            "mass, grip, drive layout and model each change the fingerprint");
    rejects([&] { fd::make_local_trajectory(track, plan, plant, config, heavier, 0, {}, &table); },
            "a table for a lighter car is rejected");
    rejects([&] { fd::make_local_trajectory(track, fd::make_speed_plan(track, slippery), plant, slippery, car, 0, {}, &table); },
            "a table for more grip is rejected");
    rejects([&] { fd::choose_local_action(track, plan, plant, config, fd::KinematicBicycle{}, {}, 0, &table); },
            "a dynamic car's table is rejected for the kinematic bicycle");
    rejects([&] { (void)table.steering_for(std::nan(""), 5); }, "a lookup rejects a non-finite curvature");
    rejects([&] { (void)table.steering_for(0.01, -1); }, "a lookup rejects a negative speed");

    fd::Simulation simulation(track, config, car, fd::SteeringMode::model_acceleration_pursuit);
    require(simulation.steering_table() && simulation.steering_table()->fingerprint() == table.fingerprint(),
            "a MAP simulation generates the table for its plant");
    for (int i = 0; i < 400; ++i) simulation.step();
    simulation.set_grip(0.7);
    require(simulation.steering_table()->fingerprint() == table.fingerprint(), "a pending change keeps the table until it commits");
    simulation.step();
    require(simulation.config().grip_mu == 0.7 && simulation.steering_table()->generated_for(car, simulation.config()),
            "the committed change regenerated the table for the new grip");
    fd::Simulation pursuit(track, config, car);
    require(pursuit.steering_mode() == fd::SteeringMode::pure_pursuit && !pursuit.steering_table(),
            "Pure Pursuit is the default and has no table");
}

// MAP keeps Pure Pursuit's target and changes only how its turn becomes steering. For every state
// the geometric steering is identical, and MAP's steering is the table's for that arc's curvature.
void map_keeps_the_pursuit_target_and_converts_its_turn() {
    fd::Config config;
    const auto track = fd::make_preset_track();
    const auto plan = fd::make_speed_plan(track, config);
    const auto car = soft(true);
    const fd::SteeringTable table(car, config);
    const double L = config.wheelbase_m;
    int corrected = 0;
    for (std::size_t i = 0; i < track.points.size(); i += 37) {
        const auto& p = track.points[i];
        const auto& q = track.points[(i+1)%track.points.size()];
        const double heading = std::atan2(q.y_m-p.y_m, q.x_m-p.x_m);
        for (const double offset : {-1.5, 0.0, 2.0}) {
            const fd::State state{p.x_m-std::sin(heading)*offset, p.y_m+std::cos(heading)*offset, heading+0.05*offset,
                                  plan[i].speed_mps, 0, 0};
            const auto pursuit = fd::compute_control(track, plan, state, config);
            const auto map = fd::compute_control(track, plan, state, config, {}, &table);
            require(pursuit.geometric_steering_rad == pursuit.requested.steering_rad, "Pure Pursuit requests its geometric steering");
            require(map.geometric_steering_rad == pursuit.geometric_steering_rad, "MAP aims at the same target");
            require(map.requested.acceleration_mps2 == pursuit.requested.acceleration_mps2 && map.lookahead_m == pursuit.lookahead_m,
                    "MAP leaves the speed control and lookahead alone");
            const double curvature = std::tan(pursuit.geometric_steering_rad)/L;
            near(map.requested.steering_rad, table.steering_for(curvature, state.speed_mps), 1e-9,
                 "MAP steers by the table for the pursuit arc's curvature");
            if (std::abs(map.requested.steering_rad-pursuit.requested.steering_rad) > 1e-3) ++corrected;
        }
    }
    require(corrected > 10, "on an understeering car MAP corrects geometry in many of these states");
}

// Tracking over whole laps, split by how hard the car is cornering.
struct LapResult {
    int laps{};
    double lap_time_s{}, max_error_m{}, rms_error_m{}, max_error_hard_m{}, rms_error_hard_m{};
    std::size_t samples{}, hard_samples{}, invalid{};
};
LapResult drive_laps(const fd::VehicleModel& model, const fd::Config& config, fd::SteeringMode mode, int laps) {
    fd::Simulation simulation(fd::make_preset_track(), config, model, mode);
    LapResult result;
    double sum = 0, hard_sum = 0;
    const double hard = 0.5*config.grip_mu*fd::gravity_mps2;
    while (simulation.diagnostics().laps < laps && simulation.state().time_s < 60.0*laps) {
        simulation.step();
        const auto& d = simulation.diagnostics();
        const double error = std::abs(d.cross_track_error_m);
        ++result.samples;
        sum += error*error;
        result.max_error_m = std::max(result.max_error_m, error);
        if (std::abs(d.lateral_acceleration_mps2) > hard) {
            ++result.hard_samples;
            hard_sum += error*error;
            result.max_error_hard_m = std::max(result.max_error_hard_m, error);
        }
        if (!d.plan_valid) ++result.invalid;
        if (d.laps == 1 && result.laps == 0) { result.laps = 1; result.lap_time_s = simulation.state().time_s; }
    }
    result.laps = simulation.diagnostics().laps;
    result.rms_error_m = std::sqrt(sum/static_cast<double>(result.samples));
    result.rms_error_hard_m = result.hard_samples ? std::sqrt(hard_sum/static_cast<double>(result.hard_samples)) : 0;
    return result;
}
void print(const std::string& name, const LapResult& r) {
    std::cout << "  " << name << ": laps " << r.laps << ", first lap " << r.lap_time_s << " s, tracking error max "
              << r.max_error_m << " m rms " << r.rms_error_m << " m; above 0.5 g max " << r.max_error_hard_m << " m rms "
              << r.rms_error_hard_m << " m over " << r.hard_samples << " samples; invalid " << r.invalid << "/" << r.samples << "\n";
}

// The plan's acceptance for MAP: two laps on the dynamic plant with bounded tracking error, and Pure
// Pursuit measurably worse at high lateral acceleration on a car whose steering is not geometry.
// The default car has equal axle loads and tires, so it is nearly neutral and its steady steering is
// almost geometry: there MAP must change little, and the test says so rather than claiming a gain.
void map_tracks_better_than_pure_pursuit_when_the_car_understeers() {
    fd::Config config;
    config.lateral_grip_fraction = 0.75;       // the planner's most aggressive cornering
    config.longitudinal_grip_fraction = 0.45;  // keeping the required 10% reserve
    for (const auto& [name, car] : {std::pair<std::string, fd::DynamicSingleTrack>{"default car", fd::DynamicSingleTrack{}},
                                    std::pair<std::string, fd::DynamicSingleTrack>{"understeering car", soft(true)}}) {
        const auto start = std::chrono::steady_clock::now();
        const auto pursuit = drive_laps(car, config, fd::SteeringMode::pure_pursuit, 2);
        const auto map = drive_laps(car, config, fd::SteeringMode::model_acceleration_pursuit, 2);
        print(name+", Pure Pursuit", pursuit);
        print(name+", MAP", map);
        std::cout << "  (" << seconds_since(start) << " s wall for both, measurement only)\n";
        require(map.laps == 2 && pursuit.laps == 2, name+": both laws complete two laps");
        require(map.max_error_m < 0.6 && map.invalid == 0, name+": MAP keeps tracking error bounded and every sample valid");
        if (name == "default car") {
            near(map.rms_error_m, pursuit.rms_error_m, 0.1*pursuit.rms_error_m, "on a nearly neutral car MAP and Pure Pursuit track alike");
        } else {
            require(pursuit.rms_error_hard_m > 1.8*map.rms_error_hard_m, "above 0.5 g Pure Pursuit's error is measurably worse when the car understeers");
            require(pursuit.max_error_m > 1.5*map.max_error_m, "and its largest error too");
        }
    }
}

// The four-wheel car's wheel speeds and loads are states of its own, so its steady turns are solved with them
// (decision 0026). A row's turns must be turns the car actually holds: driven at that steering with a throttle loop
// holding the speed, it settles at the curvature the table claims.
void four_wheel_table_holds_the_turns_it_claims() {
    const fd::Config config;
    const fd::FourWheelCar car;
    const auto start = std::chrono::steady_clock::now();
    const fd::SteeringTable table(car, config);
    const double generated = seconds_since(start);
    require(table.rows().size() > 5, "the table has a row for each speed");
    std::size_t total_turns = 0;
    for (const auto& row : table.rows()) total_turns += row.turns.size();
    std::cout << "  four-wheel table: " << table.rows().size() << " rows, " << total_turns << " turns, " << table.fingerprint() << ", generated in "
              << generated << " s (measurement only)\n";
    for (const auto& row : table.rows()) {
        require(row.turns.front().steering_rad == 0 && row.turns.front().curvature_per_m == 0, "each row starts driving straight");
        for (std::size_t i = 1; i < row.turns.size(); ++i)
            require(row.turns[i].steering_rad > row.turns[i-1].steering_rad && row.turns[i].curvature_per_m > row.turns[i-1].curvature_per_m,
                    "steering and curvature increase together along a row");
    }
    // Driving each claimed turn: hold the speed with the same simple loop the envelope uses, and compare the curvature
    // the car settles at with the table's.
    // Eased into the turn, as a car is: the steering is taken to the angle over two seconds and then held, so the car
    // settles into the turn rather than being thrown at it. A step to a turn at the grip limit lands in another turn.
    const auto settled_curvature = [&](double speed, double steering) {
        auto s = fd::plant_state_from({0, 0, 0, speed, 0, 0}, car, config);
        double integral = 0;
        const double dt = config.fixed_dt_s;
        const int ramp = static_cast<int>(std::lround(2.0/dt));
        for (int i = 0; i < static_cast<int>(std::lround(5.0/dt)); ++i) {
            const double error = speed-s.pose.speed_mps;
            integral += error*dt;
            const double aim = steering*std::min(1.0, static_cast<double>(i)/ramp);
            s = fd::advance(car, s, {2.0*error+0.5*integral, aim}, config, dt);
        }
        return s.yaw_rate_radps/s.pose.speed_mps;
    };
    std::size_t checked = 0;
    double worst_inside = 0, worst_limit = 0;
    for (const auto& row : table.rows()) {
        if (row.speed_mps < 5 || row.turns.size() < 3) continue;
        for (const std::size_t i : {row.turns.size()/2, row.turns.size()-1}) {
            const auto& turn = row.turns[i];
            const double off = std::abs(settled_curvature(row.speed_mps, turn.steering_rad)-turn.curvature_per_m)/turn.curvature_per_m;
            (i+1 == row.turns.size() ? worst_limit : worst_inside) = std::max(i+1 == row.turns.size() ? worst_limit : worst_inside, off);
            ++checked;
        }
    }
    std::cout << "  driven against the table: " << checked << " turns, worst curvature difference " << 100*worst_inside
              << "% inside a row and " << 100*worst_limit << "% at its last turn\n";
    // Inside a row the car settles into the turn the table claims. A row's last turn is at the tires' limit, where the
    // turn is only weakly attracting: eased into it the car arrives within a few percent, and thrown at it lands elsewhere.
    require(checked >= 10 && worst_inside < 0.015 && worst_limit < 0.05, "every claimed turn is the one the car settles into");
    // Its load moves onto the outer wheels and off the front axle, so it understeers: at speed it needs more steering
    // than geometry for the same curvature.
    const double k = 0.02;
    const double geometric = std::atan(config.wheelbase_m*k);
    // Its axles carry alike tires at alike loads, so in gentle turns this car is nearly neutral and MAP barely corrects
    // geometry; near the grip limit its load moves and it understeers, and there MAP adds to the steering.
    std::cout << "  correction to geometry at curvature " << k << ": " << table.steering_for(k, 10)-geometric << " rad at 10 m/s, "
              << table.steering_for(k, 18)-geometric << " rad at 18 m/s, " << table.steering_for(k, 22)-geometric << " rad at 22 m/s\n";
    require(std::abs(table.steering_for(k, 10)-geometric) < 1e-3, "in a gentle turn the table is geometry");
    require(table.steering_for(k, 22) > geometric+0.01, "at 9.7 m/s^2 of lateral acceleration it asks for measurably more");
    require(table.steering_for(k, 22) > table.steering_for(k, 18) && table.steering_for(k, 18) > table.steering_for(k, 10),
            "and the correction grows with lateral acceleration");
    require(table.generated_for(car, config), "the table knows the car it was generated for");
    fd::FourWheelCar other = car;
    other.cg_height_m = 0.2;
    require(!table.generated_for(other, config), "and not another");
}

// Through the simulation: MAP drives the four-wheel car, with the envelope speed plan of decision 0018, which is the
// baseline the plan's Phase 6.3 compares MPCC against.
void map_drives_the_four_wheel_car() {
    fd::Config config;
    const auto lap = [&](fd::SteeringMode mode, fd::SpeedPlanMode plan) {
        fd::Simulation simulation(fd::make_preset_track(), config, fd::FourWheelCar{}, mode, plan);
        double sum = 0, max_error = 0;
        std::size_t samples = 0, invalid = 0;
        while (simulation.diagnostics().laps < 1 && simulation.state().time_s < 60) {
            simulation.step();
            const double error = std::abs(simulation.diagnostics().cross_track_error_m);
            sum += error*error;
            max_error = std::max(max_error, error);
            ++samples;
            if (!simulation.diagnostics().plan_valid) ++invalid;
        }
        require(simulation.diagnostics().laps == 1, "the lap is completed");
        std::cout << "  four-wheel " << fd::steering_mode_name(mode) << " with " << (plan == fd::SpeedPlanMode::performance_envelope ? "the envelope" : "the grip fractions")
                  << ": lap " << simulation.state().time_s << " s, tracking error max " << max_error << " m rms "
                  << std::sqrt(sum/static_cast<double>(samples)) << " m, invalid " << invalid << "/" << samples << "\n";
        return std::make_pair(max_error, std::sqrt(sum/static_cast<double>(samples)));
    };
    const auto pursuit = lap(fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions);
    const auto map = lap(fd::SteeringMode::model_acceleration_pursuit, fd::SpeedPlanMode::grip_fractions);
    const auto envelope = lap(fd::SteeringMode::model_acceleration_pursuit, fd::SpeedPlanMode::performance_envelope);
    require(map.second < pursuit.second, "MAP tracks the reference better than geometry does on this car");
    require(envelope.first < 1.0, "and keeps tracking bounded while driving the envelope's speeds");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"kinematic_table_is_the_geometric_law", kinematic_table_is_the_geometric_law},
        {"dynamic_table_follows_the_understeer_gradient", dynamic_table_follows_the_understeer_gradient},
        {"dynamic_table_stops_at_the_grip_limit", dynamic_table_stops_at_the_grip_limit},
        {"a_table_for_another_plant_is_rejected", a_table_for_another_plant_is_rejected},
        {"map_keeps_the_pursuit_target_and_converts_its_turn", map_keeps_the_pursuit_target_and_converts_its_turn},
        {"map_tracks_better_than_pure_pursuit_when_the_car_understeers", map_tracks_better_than_pure_pursuit_when_the_car_understeers},
        {"four_wheel_table_holds_the_turns_it_claims", four_wheel_table_holds_the_turns_it_claims},
        {"map_drives_the_four_wheel_car", map_drives_the_four_wheel_car},
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " steering groups passed\n";
    return failures ? 1 : 0;
}

#include "fd/cone_driving.hpp"
#include "fd/path_following.hpp"
#include "fd/simulation.hpp"

#include <algorithm>
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

template <class F> bool refused(F&& f) {
    try { f(); } catch (const std::invalid_argument&) { return true; }
    return false;
}

fd::OpenPath straight_path(double length_m, double y_m = 0) {
    fd::OpenPath path;
    for (int i = 0; i <= static_cast<int>(length_m/0.5); ++i) path.points.push_back({0.5*i, y_m, 0.5*i, 0});
    return path;
}

// The preset course at the 5 m width Formula Student tracks are laid to, which FaSTTUBe's planner is tuned for.
fd::Track narrow_preset() {
    auto track = fd::make_preset_track();
    track.width_m = 5.0;
    fd::validate_track(track);
    return track;
}

// Speeds along an open path keep to the reference plan's rules and come to rest at its end: the car can always stop
// within what it has seen.
void the_open_path_plan_stops_within_sight() {
    const fd::Config config;
    const double braking = config.grip_mu*fd::gravity_mps2*config.longitudinal_grip_fraction;
    const double lateral = config.grip_mu*fd::gravity_mps2*config.lateral_grip_fraction;
    const auto path = straight_path(30);
    const auto plan = fd::plan_open_path(path, config);
    require(plan.back().speed_mps == 0 && plan.back().reason == "end_of_believed_path", "at rest at the end of the path");
    for (std::size_t i = 0; i < plan.size(); ++i) {
        const double to_end = path.points.back().s_m-path.points[i].s_m;
        require(plan[i].speed_mps <= std::sqrt(2*braking*to_end)+1e-9, "never faster than a stop by the end allows");
        require(plan[i].speed_mps <= config.max_speed_mps, "never above the speed cap");
    }
    // Far enough from the end the cap governs, and the last samples brake within the grip budget.
    require(std::abs(plan.front().speed_mps-std::min(config.max_speed_mps, std::sqrt(2*braking*30))) < 1e-9, "the first sample brakes toward the end");
    // A bend limits the speed on it by its curvature.
    auto bend = straight_path(60);
    for (auto& p : bend.points) if (p.s_m >= 20 && p.s_m <= 30) p.curvature = 0.1;
    const auto bent = fd::plan_open_path(bend, config);
    for (std::size_t i = 0; i < bent.size(); ++i)
        if (bend.points[i].curvature > 0) require(bent[i].speed_mps <= std::sqrt(lateral/0.1)+1e-9, "within lateral grip on the bend");
    // Reachable in both directions within the longitudinal budget.
    for (std::size_t i = 0; i+1 < bent.size(); ++i) {
        const double ds = bend.points[i+1].s_m-bend.points[i].s_m;
        require(bent[i+1].speed_mps*bent[i+1].speed_mps <= bent[i].speed_mps*bent[i].speed_mps+2*braking*ds+1e-9 &&
                bent[i].speed_mps*bent[i].speed_mps <= bent[i+1].speed_mps*bent[i+1].speed_mps+2*braking*ds+1e-9,
                "each step reachable within the grip budget");
    }
    // What is not a path is refused.
    require(refused([&] { fd::plan_open_path({{{0, 0, 0, 0}}, 3.0}, config); }), "a single sample is refused");
    require(refused([&] { fd::plan_open_path({{{0, 0, 1, 0}, {1, 0, 2, 0}}, 3.0}, config); }), "a first station other than zero is refused");
    require(refused([&] { fd::plan_open_path({{{0, 0, 0, 0}, {2, 0, 1, 0}}, 3.0}, config); }), "a step shorter than its chord is refused");
    require(refused([&] { fd::plan_open_path({{{0, 0, 0, 0}, {1, 0, 1, 0}}, 0.0}, config); }), "a corridor of no width is refused");
}

// The policy along an open path pulls a car that starts off it onto it, and stops it before the path ends.
void following_a_path_brings_the_car_onto_it_and_stops_it() {
    const fd::Config config;
    const auto path = straight_path(60);
    const auto plan = fd::plan_open_path(path, config);
    fd::State state{0, 0.8, 0, 5, 0, 0};
    double worst_late = 0;
    for (int tick = 0; tick < 4000; ++tick) {
        static fd::Command held;
        if (tick%4 == 0) held = fd::follow_open_path(path, plan, state, config).applied;
        state = fd::integrate_bicycle(state, held, config, config.fixed_dt_s);
        if (tick*config.fixed_dt_s > 6) worst_late = std::max(worst_late, std::abs(state.y_m));
    }
    std::cout << "  from 0.8 m off a 60 m path at 5 m/s: within " << worst_late << " m of it after 6 s, at rest "
              << 60-state.x_m << " m before its end\n";
    require(worst_late < 0.05, "onto the path within six seconds");
    require(state.speed_mps < 1e-3 && state.x_m < 60.0, "at rest before the path ends");
    const auto near = fd::project_open(path, {61, 0.3});
    require(near.index == path.points.size()-2 && near.fraction > 1 && std::abs(near.s_m-61) < 1e-9 && std::abs(near.signed_error_m-0.3) < 1e-9,
            "beyond the end a position projects onto the path's continuation");
}

struct LapResult {
    int laps{};
    double lap_time_s{}, worst_error_m{}, outside_s{}, top_speed_mps{};
    std::size_t paths{}, decisions_without_path{};
};

LapResult drive_a_lap(bool instruments, std::uint64_t seed) {
    fd::Simulation simulation(narrow_preset(), fd::Config{});
    if (instruments) simulation.set_sensors(fd::SensorSettings{.seed = seed});
    simulation.set_perception(fd::PerceptionSettings{.seed = seed});
    simulation.set_cone_driving(fd::ConePathSettings{});
    LapResult result;
    const double dt = simulation.config().fixed_dt_s;
    for (int tick = 0; tick < static_cast<int>(200/dt) && simulation.diagnostics().laps < 1; ++tick) {
        simulation.step();
        const auto& d = simulation.diagnostics();
        result.worst_error_m = std::max(result.worst_error_m, std::abs(d.cross_track_error_m));
        if (!d.within_track) result.outside_s += dt;
        result.top_speed_mps = std::max(result.top_speed_mps, simulation.state().speed_mps);
        if (!simulation.cone_driver()->belief().path) ++result.decisions_without_path;
    }
    result.laps = simulation.diagnostics().laps;
    result.lap_time_s = simulation.state().time_s;
    result.paths = simulation.cone_driver()->paths_made();
    return result;
}

// On the preset course laid with cones, the car drives a lap on the path it believes from its own simulated detections,
// with the true state read the instant it is (the ideal-state assumption), and then with its own late and noisy
// instruments. The known track is only what the lap is judged against.
void the_car_drives_a_lap_on_cones_alone() {
    for (const bool instruments : {false, true}) {
        const auto r = drive_a_lap(instruments, 1);
        std::cout << "  " << (instruments ? "measured" : "ideal") << " state: " << r.laps << " lap in " << r.lap_time_s << " s, top speed "
                  << r.top_speed_mps << " m/s, " << r.paths << " believed paths, at most " << r.worst_error_m
                  << " m from the true centreline, " << r.outside_s << " s outside the track's margin\n";
        require(r.laps >= 1, std::string("a lap on cones with the ") + (instruments ? "measured" : "ideal") + " state");
        require(r.worst_error_m < 2.5, "never leaving the 5 m course");
    }
}

// Phase 7.4's boundary: a second driver, given only what the simulation publishes of the car's readings and perception,
// and the pose it was placed at, decides exactly as the simulation's driver did, command for command and belief for
// belief. So nothing else reached the simulation's driver: not the track, not the cones, not a frame's truth.
void the_driver_decides_from_observations_alone() {
    for (const bool instruments : {false, true}) {
        fd::Simulation simulation(narrow_preset(), fd::Config{});
        if (instruments) simulation.set_sensors(fd::SensorSettings{.seed = 7});
        simulation.set_perception(fd::PerceptionSettings{.seed = 7});
        simulation.set_cone_driving(fd::ConePathSettings{});
        fd::ConeDriver shadow;
        shadow.reset(simulation.state());
        const auto observe = [&] {
            if (const auto* m = simulation.measurements()) {
                if (m->pose.measured) shadow.read_pose({m->pose.sampled_at_s, m->x_m, m->y_m, m->yaw_rad});
                shadow.read_motion({m->speed.measured ? m->speed_mps : 0.0, m->imu.measured ? m->yaw_rate_radps : 0.0,
                                    m->steering.measured ? m->steering_rad : 0.0});
            } else {
                const auto& s = simulation.state();
                shadow.read_pose({s.time_s, s.x_m, s.y_m, s.yaw_rad});
                shadow.read_motion({s.speed_mps, simulation.plant_state().yaw_rate_radps, s.steering_rad});
            }
            for (const auto& frame : simulation.perception()->delivered()) shadow.perceive(frame);
        };
        std::size_t compared = 0;
        const auto control_ticks = static_cast<int>(std::lround(simulation.config().control_dt_s/simulation.config().fixed_dt_s));
        for (int tick = 0; tick < 6000; ++tick) {
            std::optional<fd::LocalDecision> expected;
            if (tick%control_ticks == 0)
                expected = shadow.decide(simulation.state().time_s, simulation.config(), simulation.vehicle_model(),
                                         static_cast<std::uint64_t>(control_ticks), simulation.steering_table(), simulation.performance_envelope());
            simulation.step();
            if (expected) {
                const auto& got = simulation.decision().trajectory().first_control.applied;
                const auto& want = expected->trajectory().first_control.applied;
                require(got.acceleration_mps2 == want.acceleration_mps2 && got.steering_rad == want.steering_rad,
                        "the same command at "+std::to_string(simulation.state().time_s)+" s");
                const auto& a = simulation.cone_driver()->belief();
                const auto& b = shadow.belief();
                require(a.state.x_m == b.state.x_m && a.state.y_m == b.state.y_m && a.state.yaw_rad == b.state.yaw_rad &&
                        a.state.speed_mps == b.state.speed_mps && a.path == b.path, "the same belief");
                ++compared;
            }
            observe();
        }
        require(shadow.paths_made() == simulation.cone_driver()->paths_made() && shadow.paths_made() > 100, "the same believed paths");
        std::cout << "  " << (instruments ? "measured" : "ideal") << " state: " << compared << " decisions over "
                  << simulation.state().time_s << " s, identical from the readings and frames alone\n";
    }
}

// Driving on cones needs perception, and refuses what only the known track can state.
void driving_on_cones_refuses_what_it_cannot_know() {
    fd::Simulation plain(narrow_preset(), fd::Config{});
    require(refused([&] { plain.set_cone_driving(fd::ConePathSettings{}); }), "cone driving without perception is refused");
    plain.set_obstructions({{100, 110, -1, 1, "block"}});
    plain.set_perception(fd::PerceptionSettings{});
    require(refused([&] { plain.set_cone_driving(fd::ConePathSettings{}); }), "cone driving with a blockage stated is refused");
    plain.set_obstructions({});
    plain.set_cone_driving(fd::ConePathSettings{});
    require(refused([&] { plain.set_obstructions({{100, 110, -1, 1, "block"}}); }), "a blockage while driving on cones is refused");
    require(refused([&] { plain.set_perception(std::nullopt); }), "taking perception off while driving on cones is refused");
    auto bad = fd::ConePathSettings{};
    bad.min_track_width_m = -1;
    require(refused([&] { plain.set_cone_driving(bad); }), "settings the planner refuses are refused");
    plain.set_cone_driving(std::nullopt);
    require(plain.cone_driver() == nullptr && plain.state().time_s == 0, "back on the known track, from the start");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"the_open_path_plan_stops_within_sight", the_open_path_plan_stops_within_sight},
        {"following_a_path_brings_the_car_onto_it_and_stops_it", following_a_path_brings_the_car_onto_it_and_stops_it},
        {"the_car_drives_a_lap_on_cones_alone", the_car_drives_a_lap_on_cones_alone},
        {"the_driver_decides_from_observations_alone", the_driver_decides_from_observations_alone},
        {"driving_on_cones_refuses_what_it_cannot_know", driving_on_cones_refuses_what_it_cannot_know}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " cone driving groups passed\n";
    return failures ? 1 : 0;
}

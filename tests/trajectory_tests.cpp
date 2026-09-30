#include "fd/simulation.hpp"
#include "fd/trajectory.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
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
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error(message + ": actual=" + std::to_string(actual) +
                                 ", expected=" + std::to_string(expected));
}
void same_state(const fd::State& actual, const fd::State& expected,
                double tolerance, const std::string& context) {
    near(actual.x_m, expected.x_m, tolerance, context + " x");
    near(actual.y_m, expected.y_m, tolerance, context + " y");
    near(actual.yaw_rad, expected.yaw_rad, tolerance, context + " yaw");
    near(actual.speed_mps, expected.speed_mps, tolerance, context + " speed");
    near(actual.steering_rad, expected.steering_rad, tolerance, context + " steering");
    near(actual.time_s, expected.time_s, tolerance, context + " time");
}
template<class Action> void rejects(Action action, const std::string& message) {
    bool rejected = false;
    try { action(); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, message);
}

fd::State on_track(const fd::Track& track, const fd::Config& config,
                   double station, double speed) {
    const auto here = fd::sample(track, station);
    const auto ahead = fd::sample(track, station + 0.1);
    return {here.x_m, here.y_m,
            std::atan2(ahead.y_m - here.y_m, ahead.x_m - here.x_m), speed,
            std::atan(config.wheelbase_m * here.curvature), 0};
}

void actual_state_anchor_and_recovery() {
    fd::Simulation simulation;
    const auto source = simulation.state();
    const auto plan_speed = simulation.plan().front().speed_mps;
    auto perturbed = source;
    perturbed.y_m += 1.2;
    perturbed.yaw_rad += 0.1;
    perturbed.speed_mps = 6;
    perturbed.time_s = 17;
    const auto input = perturbed;
    const auto prediction = fd::make_local_trajectory(
        simulation.track(), simulation.plan(), perturbed, simulation.config());
    require(prediction.points.size() > 3, "prediction contains a finite sequence of future states");
    same_state(prediction.points.front().state, input, 0, "actual-state anchor");
    same_state(perturbed, input, 0, "prediction does not mutate input state");
    same_state(simulation.state(), source, 0, "prediction does not advance the live plant");
    near(simulation.plan().front().speed_mps, plan_speed, 0, "prediction does not edit the reference envelope");
    const auto& next = prediction.points[1];
    require(fd::project(simulation.track(), {next.state.x_m, next.state.y_m}).distance_m > 1,
            "future path retains initial tracking deviation instead of snapping to centerline");
    require(prediction.first_control.applied.steering_rad < 0,
            "left displacement and left yaw produce corrective right steering");
    const auto aligned = fd::make_local_trajectory(
        simulation.track(), simulation.plan(), on_track(simulation.track(), simulation.config(), 0, 6),
        simulation.config());
    require(std::abs(aligned.first_control.applied.steering_rad -
                     prediction.first_control.applied.steering_rad) > 0.05,
            "actual pose changes the commanded steering");
    const auto& last = prediction.points.back();
    near(last.state.time_s - input.time_s, 3, 1e-10, "default prediction horizon is three seconds");
    require(last.distance_m > 10 && last.distance_m < 70,
            "prediction distance follows reachable motion over its finite time horizon");
}

void compare_near_future(fd::Simulation& simulation, const fd::LocalTrajectory& prediction) {
    const double limit = prediction.points.front().state.time_s + 0.12;
    std::size_t compared = 0;
    for (const auto& point : prediction.points) {
        if (point.state.time_s <= simulation.state().time_s + 1e-12) continue;
        if (point.state.time_s > limit + 1e-12) break;
        while (simulation.state().time_s < point.state.time_s - 1e-12) simulation.step();
        same_state(simulation.state(), point.state, 1e-10,
                   "forecast matches separately advanced closed-loop simulation");
        ++compared;
    }
    require(compared >= 4, "checked multiple future control intervals");
}

void command_source_and_fixed_tick_prediction() {
    fd::Simulation simulation;
    const auto prediction = simulation.trajectory();
    const auto initial_state = simulation.state();
    simulation.step();
    near(simulation.diagnostics().requested.acceleration_mps2,
         prediction.first_control.requested.acceleration_mps2, 0,
         "live plant receives the prediction's first requested acceleration");
    near(simulation.diagnostics().requested.steering_rad,
         prediction.first_control.requested.steering_rad, 0,
         "live plant receives the prediction's first requested steering");
    near(simulation.diagnostics().applied.acceleration_mps2,
         prediction.first_control.applied.acceleration_mps2, 0,
         "live plant receives the prediction's first bounded acceleration");
    require(simulation.state().time_s > initial_state.time_s, "only explicit stepping advances the plant");
    compare_near_future(simulation, prediction);
}

void off_grid_grip_change_keeps_regular_schedule() {
    fd::Simulation simulation;
    for (int i = 0; i < 3; ++i) simulation.step();
    const auto before = simulation.state();
    near(before.time_s, 0.015, 1e-12, "event is deliberately off the control grid");
    simulation.set_grip(0.45);
    simulation.step();
    const auto prediction = simulation.trajectory();
    same_state(prediction.points.front().state, before, 0,
               "event-triggered prediction is anchored at the accepted tick boundary");
    near(prediction.points[1].state.time_s, 0.02, 1e-12,
         "first event command lasts only until the existing control grid resumes");
    same_state(prediction.points[1].state, simulation.state(), 1e-10,
               "off-grid prediction uses the actual first fixed tick");
    near(simulation.events().front().time_s, before.time_s, 0, "event timing remains unchanged");
    require(simulation.diagnostics().revision == 2, "grip change keeps configuration revision semantics");
    compare_near_future(simulation, prediction);
}

void grip_changes_immediate_action() {
    const auto track = fd::make_preset_track();
    fd::Config high;
    auto low = high;
    low.grip_mu = 0.45;
    const auto state = on_track(track, high, 80, 20);
    const auto high_plan = fd::make_speed_plan(track, high);
    const auto low_plan = fd::make_speed_plan(track, low);
    const auto began = std::chrono::steady_clock::now();
    const auto high_prediction = fd::make_local_trajectory(track, high_plan, state, high);
    const auto low_prediction = fd::make_local_trajectory(track, low_plan, state, low);
    const double milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - began).count();
    std::cout << "  two three-second forecasts at 20 m/s: " << milliseconds
              << " ms total (measurement only, no real-time guarantee)\n";
    same_state(high_prediction.points.front().state, state, 0, "high-grip initial actual state");
    same_state(low_prediction.points.front().state, state, 0, "low-grip initial actual state");
    require(high_prediction.first_control.applied.acceleration_mps2 > -0.1,
            "high-grip car can hold speed at this approach station");
    require(low_prediction.first_control.applied.acceleration_mps2 < -1,
            "low-grip car brakes now for the upcoming known corner");
    require(low_prediction.points[1].state.speed_mps < high_prediction.points[1].state.speed_mps,
            "revised grip affects reachable future motion, not just path color");
    require(low_prediction.points[0].acceleration_mps2 < -fd::plan_color_deadband_mps2,
            "first low-grip segment has actual predicted braking semantics");
    require(!low_prediction.within_model_envelope,
            "sudden grip loss retains and exposes the now-infeasible actual speed");
}

void infeasible_state_is_reported_without_repair() {
    const auto track = fd::make_preset_track();
    const fd::Config config;
    const auto plan = fd::make_speed_plan(track, config);
    auto state = on_track(track, config, 80, 20);
    state.steering_rad = config.max_steering_rad;
    const auto excessive_grip = fd::make_local_trajectory(track, plan, state, config);
    same_state(excessive_grip.points.front().state, state, 0,
               "impossible lateral demand is not hidden by rewriting actual steering");
    require(!excessive_grip.within_model_envelope &&
                excessive_grip.validity_reason.find("grip") != std::string::npos,
            "prediction exposes infeasible grip demand before claiming a feasible path");
    state = on_track(track, config, 80, 2);
    state.y_m += track.width_m;
    const auto outside_track = fd::make_local_trajectory(track, plan, state, config);
    same_state(outside_track.points.front().state, state, 0,
               "out-of-track initial state is preserved");
    require(!outside_track.within_model_envelope &&
                outside_track.validity_reason.find("track") != std::string::npos,
            "prediction names its rear-axle corridor violation");
}

void seam_horizon_and_segment_contract() {
    const auto track = fd::make_preset_track();
    fd::Config config;
    const auto initial = on_track(track, config, track.length_m - 3, 8);
    const auto prediction = fd::make_local_trajectory(track, fd::make_speed_plan(track, config), initial, config);
    double distance = 0;
    for (std::size_t i = 0; i + 1 < prediction.points.size(); ++i) {
        const auto& current = prediction.points[i];
        const auto& next = prediction.points[i + 1];
        const double duration = next.state.time_s - current.state.time_s;
        require(duration > 0, "trajectory timestamps increase through periodic seam");
        distance += std::hypot(next.state.x_m - current.state.x_m,
                               next.state.y_m - current.state.y_m);
        require(next.distance_m >= current.distance_m, "trajectory traveled distance is monotonic");
        near(current.acceleration_mps2,
             (next.state.speed_mps - current.state.speed_mps) / duration, 1e-8,
             "segment color acceleration describes achieved speed change");
        require(std::abs(next.state.steering_rad - current.state.steering_rad) <=
                    config.max_steering_rate_radps * duration + 1e-10,
                "predicted motion respects steering actuator rate");
        require(std::abs(next.state.steering_rad) <= config.max_steering_rad + 1e-12,
                "predicted steering stays within the angle limit");
    }
    const auto& last = prediction.points.back();
    require(fd::project(track, {last.state.x_m, last.state.y_m}).s_m < 80,
            "finite prediction crosses start/finish instead of stopping or jumping at the seam");
    require(last.distance_m >= distance - 1e-9 && last.distance_m < distance + 0.01,
            "reported traveled distance agrees with the geometric rollout");
    near(last.acceleration_mps2, 0, 0, "terminal point has no outgoing acceleration segment");
    config.fixed_dt_s = 0.007;
    config.control_dt_s = 0.028;
    const auto nondividing = fd::make_local_trajectory(track, fd::make_speed_plan(track, config), initial, config);
    const double horizon = nondividing.points.back().state.time_s - initial.time_s;
    require(horizon >= 3 - 1e-12 && horizon < 3 + config.fixed_dt_s,
            "horizon rounds to a plant tick for supported nondividing fixed dt");
}

void invalid_prediction_inputs() {
    const auto track = fd::make_preset_track();
    const fd::Config config;
    const auto plan = fd::make_speed_plan(track, config);
    const auto state = on_track(track, config, 0, 0);
    auto invalid_state = state;
    invalid_state.yaw_rad = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { fd::make_local_trajectory(track, plan, invalid_state, config); },
            "prediction rejects nonfinite actual state");
    invalid_state = state;
    invalid_state.speed_mps = -1;
    rejects([&] { fd::make_local_trajectory(track, plan, invalid_state, config); },
            "prediction rejects reverse motion unsupported by this model");
    auto bad_plan = plan;
    bad_plan.pop_back();
    rejects([&] { fd::make_local_trajectory(track, bad_plan, state, config); },
            "prediction rejects mismatched reference envelope");
    bad_plan = plan;
    bad_plan[4].speed_mps = std::numeric_limits<double>::infinity();
    rejects([&] { fd::make_local_trajectory(track, bad_plan, state, config); },
            "prediction rejects a nonfinite reference speed");
    auto bad_config = config;
    bad_config.fixed_dt_s = 0;
    rejects([&] { fd::make_local_trajectory(track, plan, state, bad_config); },
            "prediction rejects an invalid integration configuration");
    rejects([&] { fd::make_local_trajectory(track, plan, state, config, 5); },
            "prediction rejects next-control delay beyond the configured period");
    rejects([&] { fd::make_local_trajectory(track, plan, state, config,
                                          std::numeric_limits<std::uint64_t>::max()); },
            "prediction rejects overflowing tick input before computation");
}

void analytic_track_projection() {
    // Exact square geometry supplies independent distance, station and side oracles.
    // It is a projection fixture, not a steerable racing-track proposal.
    fd::Track square;
    square.points = {{0, 0, 0, 0}, {10, 0, 10, 0},
                     {10, 10, 20, 0}, {0, 10, 30, 0}};
    square.length_m = 40;
    fd::validate_track(square);
    const auto inside = fd::project(square, {4, 2});
    require(inside.index == 0, "interior point projects onto the nearest bottom edge");
    near(inside.fraction, 0.4, 1e-15, "interior projection fraction");
    near(inside.point.x, 4, 1e-14, "interior projected x");
    near(inside.point.y, 0, 0, "interior projected y");
    near(inside.s_m, 4, 1e-14, "interior arc station");
    near(inside.distance_m, 2, 1e-14, "interior perpendicular distance");
    near(inside.signed_error_m, 2, 1e-14, "left side of CCW edge has positive error");
    const auto outside = fd::project(square, {4, -3});
    near(outside.distance_m, 3, 1e-14, "exterior perpendicular distance");
    near(outside.signed_error_m, -3, 1e-14, "right side of CCW edge has negative error");
    const auto endpoint = fd::project(square, {10, 0});
    require(endpoint.index == 0, "shared endpoint chooses the first segment deterministically");
    near(endpoint.fraction, 1, 0, "shared endpoint remains the first segment's endpoint");
    near(endpoint.s_m, 10, 0, "shared endpoint station");
    near(endpoint.distance_m, 0, 0, "shared endpoint distance");
    const auto seam = fd::project(square, {0, 0});
    require(seam.index == 0, "periodic seam tie chooses the first segment deterministically");
    near(seam.s_m, 0, 0, "periodic seam station normalizes to zero");
    const auto closing = fd::project(square, {-2, 4});
    require(closing.index == 3, "closing segment participates in nearest-point selection");
    near(closing.s_m, 36, 1e-14, "closing segment station follows its downward direction");
    near(closing.signed_error_m, -2, 1e-14, "closing segment side follows its own tangent");
    const auto far = fd::project(square, {4, -1e200});
    require(std::isfinite(far.distance_m), "finite large position retains a representable distance");
    near(far.distance_m / 1e200, 1, 1e-15, "large distance survives overflowing squared distance");
    near(far.signed_error_m / 1e200, -1, 1e-15, "large signed distance preserves its side");
    near(far.point.x, 4, 1e-14, "large-position deterministic nearest projection");
    near(far.point.y, 0, 0, "large-position projected y remains finite");
}

// The exhaustive scan that project() replaced, kept verbatim as the oracle. project() may
// skip work but must return the same bits: recordings are compared byte for byte.
fd::Projection exhaustive_projection(const fd::Track& track, fd::Vec2 position) {
    const auto segment_length = [&](std::size_t i) {
        return i+1 < track.points.size() ? track.points[i+1].s_m-track.points[i].s_m
                                         : track.length_m-track.points[i].s_m;
    };
    fd::Projection result;
    double best = std::numeric_limits<double>::infinity();
    double best_squared = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < track.points.size(); ++i) {
        const auto& a = track.points[i];
        const auto& b = track.points[(i+1)%track.points.size()];
        const double ex = b.x_m-a.x_m, ey = b.y_m-a.y_m;
        const double rx = position.x-a.x_m, ry = position.y-a.y_m;
        const double fraction = std::clamp((rx*ex+ry*ey)/(ex*ex+ey*ey), 0.0, 1.0);
        const fd::Vec2 p{a.x_m+fraction*ex, a.y_m+fraction*ey};
        const double dx = position.x-p.x, dy = position.y-p.y;
        const double squared = dx*dx+dy*dy;
        constexpr double distance_guard = 1+8*std::numeric_limits<double>::epsilon();
        if (squared <= best_squared*distance_guard || !std::isfinite(squared)) {
            const double distance = std::hypot(dx, dy);
            if (distance < best) {
                best_squared = squared;
                best = distance;
                result.index = i;
                result.fraction = fraction;
                result.point = p;
            }
        }
    }
    const auto& a = track.points[result.index];
    const auto& b = track.points[(result.index+1)%track.points.size()];
    const double ex = b.x_m-a.x_m, ey = b.y_m-a.y_m;
    result.s_m = a.s_m+result.fraction*segment_length(result.index);
    result.signed_error_m = (ex*(position.y-result.point.y)-ey*(position.x-result.point.x))/std::hypot(ex, ey);
    result.distance_m = best;
    if (result.s_m >= track.length_m) result.s_m -= track.length_m;
    return result;
}

void identical_projection(const fd::Track& track, fd::Vec2 position, const std::string& context) {
    const auto fast = fd::project(track, position);
    const auto oracle = exhaustive_projection(track, position);
    // Exact comparison on purpose: a rounding difference would change recorded telemetry.
    require(fast.index == oracle.index && fast.fraction == oracle.fraction &&
            fast.s_m == oracle.s_m && fast.signed_error_m == oracle.signed_error_m &&
            fast.distance_m == oracle.distance_m &&
            fast.point.x == oracle.point.x && fast.point.y == oracle.point.y,
            context+": projection differs from the exhaustive scan at ("+
            std::to_string(position.x)+", "+std::to_string(position.y)+")");
}

// A closed polyline with irregular spacing, sharp and gentle turns and a narrow hairpin,
// so a far part of the track passes close to the query point. Arc lengths are chords
// plus a random surplus, as sampled curves have.
fd::Track irregular_track(std::mt19937& random) {
    std::uniform_real_distribution<double> radius(30, 90), surplus(0, 0.02), jitter(-0.3, 0.3);
    fd::Track track;
    constexpr int count = 400;
    double s = 0;
    fd::Vec2 previous{};
    for (int i = 0; i < count; ++i) {
        const double angle = 2*std::numbers::pi*i/count;
        // Pinch one sector toward the centre to create a hairpin beside another sector.
        const double pinch = std::abs(std::remainder(angle-1.0, 2*std::numbers::pi)) < 0.4 ? 0.25 : 1.0;
        const double r = (i%37 == 0 ? radius(random) : 60)*pinch+jitter(random);
        const fd::Vec2 here{r*std::cos(angle), r*std::sin(angle)};
        if (i > 0) s += std::hypot(here.x-previous.x, here.y-previous.y)*(1+surplus(random));
        track.points.push_back({here.x, here.y, s, 0});
        previous = here;
    }
    const auto& first = track.points.front();
    track.length_m = s+std::hypot(first.x_m-previous.x, first.y_m-previous.y)*(1+surplus(random));
    fd::validate_track(track);
    return track;
}

void pruned_projection_matches_exhaustive_scan() {
    std::mt19937 random(20260914);
    const auto preset = fd::make_preset_track();
    fd::Track square;
    square.points = {{0, 0, 0, 0}, {10, 0, 10, 0}, {10, 10, 20, 0}, {0, 10, 30, 0}};
    square.length_m = 40;
    std::vector<std::pair<std::string, fd::Track>> tracks{{"preset", preset}, {"square", square}};
    for (int k = 0; k < 4; ++k) tracks.push_back({"irregular "+std::to_string(k), irregular_track(random)});
    std::uniform_real_distribution<double> unit(0, 1), near_offset(-8, 8), far_offset(-400, 400);
    for (const auto& [name, track] : tracks) {
        std::uniform_real_distribution<double> station(0, track.length_m);
        for (int i = 0; i < 4000; ++i) {
            const double s = station(random);
            const auto on = fd::sample(track, s);
            identical_projection(track, {on.x_m, on.y_m}, name+" on the centerline");
            identical_projection(track, {on.x_m+near_offset(random), on.y_m+near_offset(random)}, name+" near");
            identical_projection(track, {on.x_m+far_offset(random), on.y_m+far_offset(random)}, name+" far");
        }
        for (const auto& point : track.points) {
            identical_projection(track, {point.x_m, point.y_m}, name+" exactly on a sample");
            identical_projection(track, {std::nextafter(point.x_m, 1e9), point.y_m}, name+" one ulp off a sample");
        }
        for (const double magnitude : {1e6, 1e150, 1e200, 1e300}) {
            identical_projection(track, {magnitude*unit(random), -magnitude}, name+" extreme finite position");
            identical_projection(track, {-magnitude, magnitude}, name+" extreme corner position");
        }
    }
    // Equidistant from two parallel edges: the first index must still win the tie.
    identical_projection(square, {5, 5}, "square centre tie");
    identical_projection(square, {5, 5+1e-12}, "square near-centre tie");
}

void arc_shorter_than_chord_is_rejected() {
    fd::Track square;
    square.points = {{0, 0, 0, 0}, {10, 0, 10, 0}, {10, 10, 20, 0}, {0, 10, 30, 0}};
    square.length_m = 40;
    fd::validate_track(square);
    auto short_edge = square;
    short_edge.points[1].s_m = 9;  // bottom edge claims 9 m for a 10 m chord
    rejects([&] { fd::validate_track(short_edge); }, "an arc length shorter than its chord is rejected");
    auto short_closing = square;
    short_closing.length_m = 39;  // closing edge claims 9 m for a 10 m chord
    rejects([&] { fd::validate_track(short_closing); }, "a closing arc shorter than its chord is rejected");
    auto recorded = square;
    // Twelve significant digits, as recordings store tracks, stays within tolerance.
    recorded.points[1].s_m = 10-2e-9;
    fd::validate_track(recorded);
}

// A plan's commands change linearly between its points and hold beyond them (decision 0025).
void a_plan_s_commands_change_linearly_between_its_points() {
    const std::vector<double> times{1.0, 1.05, 1.1};
    const std::vector<fd::Command> commands{{0, 0}, {1, 0.1}, {1, -0.1}};
    const auto halfway = fd::planned_command(times, commands, 1.025);
    near(halfway.acceleration_mps2, 0.5, 1e-12, "halfway to the second point, acceleration");
    near(halfway.steering_rad, 0.05, 1e-12, "and steering");
    near(fd::planned_command(times, commands, 1.075).steering_rad, 0, 1e-12, "halfway to the third");
    near(fd::planned_command(times, commands, 1.05).steering_rad, 0.1, 0, "at a point, its command");
    near(fd::planned_command(times, commands, 0.5).acceleration_mps2, 0, 0, "the first holds before the plan");
    near(fd::planned_command(times, commands, 2.0).steering_rad, -0.1, 0, "the last holds after it");
    rejects([&] { fd::planned_command(times, {{0, 0}, {1, 0}}, 1.0); }, "a command at every point");
    rejects([&] { fd::planned_command({1.0, 1.0, 1.1}, commands, 1.0); }, "times that ascend");
    rejects([&] { fd::planned_command(times, {{0, 0}, {std::nan(""), 0}, {1, 0}}, 1.0); }, "finite commands");
}

// Keeps to one plan: at every decision it commands c(t), acceleration and steering growing linearly in time, at its points
// every 0.05 s, so each first command is where the first plan's commands are at the end of that control interval.
class Keeping final : public fd::PredictiveController {
public:
    static fd::Command at(double t) { return {2.0+0.5*t, 0.02+0.01*t}; }
    std::vector<double> first_times;
    std::vector<fd::Command> first_commands;
    fd::PlantState first_state;
    std::uint64_t first_ticks{};
    fd::ControllerPlan plan(const fd::ControlRequest& request) override {
        fd::ControllerPlan plan;
        plan.status = fd::ControllerStatus::solved;
        plan.iterations = 1;
        auto& trajectory = plan.trajectory;
        const double now = request.state.pose.time_s;
        for (int k = 0; k <= 60; ++k) {
            fd::TrajectorySample point;
            point.state = request.state.pose;
            point.state.time_s = now+0.05*k;
            trajectory.points.push_back(point);
            trajectory.commands.push_back(at(point.state.time_s));
        }
        const auto first = fd::planned_command(fd::point_times(trajectory), trajectory.commands,
                                               now+static_cast<double>(request.ticks_to_next_control)*request.config.fixed_dt_s);
        trajectory.first_control.requested = trajectory.first_control.applied = first;
        if (first_times.empty()) {
            first_times = fd::point_times(trajectory);
            first_commands = trajectory.commands;
            first_state = request.state;
            first_ticks = request.ticks_to_next_control;
        }
        return plan;
    }
    void reset() override { first_times.clear(); first_commands.clear(); }
    fd::ControllerSettings settings() const override { return {fd::ControllerMode::mpcc, {}}; }
    void check(const fd::VehicleModel&, const fd::Config&) const override {}
};

// The plant's response to a plan is what the simulation does when every decision keeps to that plan: the same plant,
// holding over each control interval the command where the plan is at its end, from the state the plan was made in.
void the_plant_response_is_the_simulation_keeping_to_a_plan() {
    const fd::Config config;
    const fd::VehicleModel model = fd::FourWheelCar{};
    auto keeping = std::make_shared<Keeping>();
    fd::Simulation simulation(fd::make_preset_track(), config, model, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions,
                              fd::LocalPlannerMode::lattice, keeping);
    std::vector<fd::PlantState> driven{simulation.plant_state()};
    while (simulation.state().time_s < 3.0-1e-9) {
        simulation.step();
        const double t = simulation.state().time_s;
        if (std::abs(t/0.05-std::round(t/0.05)) < 1e-9) driven.push_back(simulation.plant_state());
        require(simulation.controller_outcome().driven_by == fd::ControllerMode::mpcc, "the plan drives every decision: "+simulation.controller_outcome().note);
    }
    require(keeping->first_times.size() == 61 && keeping->first_ticks == 4, "the first plan was made at the start, a control period ahead");
    const auto response = fd::plant_response(model, keeping->first_state, config, keeping->first_times, keeping->first_commands, keeping->first_ticks);
    require(response.size() == 61 && driven.size() == 61, "a state at each of the plan's points");
    for (std::size_t k = 0; k < response.size(); ++k) {
        const std::string at = "at "+std::to_string(keeping->first_times[k])+" s";
        near(response[k].pose.time_s, driven[k].pose.time_s, 1e-9, at+", time");
        near(response[k].pose.x_m, driven[k].pose.x_m, 1e-9, at+", x");
        near(response[k].pose.y_m, driven[k].pose.y_m, 1e-9, at+", y");
        near(response[k].pose.speed_mps, driven[k].pose.speed_mps, 1e-9, at+", speed");
        near(response[k].pose.steering_rad, driven[k].pose.steering_rad, 1e-9, at+", steering");
        for (std::size_t w = 0; w < 4; ++w) near(response[k].wheel_speeds_radps[w], driven[k].wheel_speeds_radps[w], 1e-7, at+", wheel speed");
    }
    require(driven.back().pose.speed_mps > 5, "the car pulled away under the plan");
    std::cout << "  kept to for 3 s: " << driven.back().pose.speed_mps << " m/s at the end, response within 1e-9 m\n";

    // A shorter first interval, after an off-grid event, takes the plan's command at its own end, then whole periods.
    const std::vector<double> times{0, 0.05, 0.1};
    const std::vector<fd::Command> ramp{{0, 0}, {1, 0.05}, {2, 0.1}};
    const auto start = fd::plant_state_from({0, 0, 0, 8, 0, 0}, model, config);
    auto by_hand = start;
    std::vector<fd::PlantState> expected{start};
    for (int tick = 0; tick < 20; ++tick) {
        const int end = tick < 2 ? 2 : 2+4*((tick-2)/4+1);
        by_hand = fd::advance(model, by_hand, fd::planned_command(times, ramp, end*config.fixed_dt_s), config, config.fixed_dt_s);
        by_hand.pose.time_s = (tick+1)*config.fixed_dt_s;
        if ((tick+1)%10 == 0) expected.push_back(by_hand);
    }
    const auto shortened = fd::plant_response(model, start, config, times, ramp, 2);
    for (std::size_t k = 0; k < 3; ++k) near(shortened[k].pose.x_m, expected[k].pose.x_m, 0, "a shortened first interval, point "+std::to_string(k));
    rejects([&] { fd::plant_response(model, start, config, {0, 0.052, 0.1}, ramp, 0); }, "times off the tick grid");
    rejects([&] { fd::plant_response(model, start, config, {0.05, 0.1, 0.15}, ramp, 0); }, "a plan that does not start at the state");
    rejects([&] { fd::plant_response(model, start, config, times, ramp, 5); }, "a first interval beyond a control period");
    rejects([&] { fd::plant_response(model, start, config, times, {{0, 0}}, 0); }, "a command at every point");
}
// What each tire is asked of its friction circle along a plan (decision 0027): the plant driven by the plan's own
// commands, and at each of its points what the vehicle seam says each wheel uses of its own limit. Only a car with four
// wheels has four tires to report; the others have none of their own.
void each_tire_s_use_along_a_plan_is_the_plant_s_own() {
    const fd::Config config;
    const fd::VehicleModel car = fd::FourWheelCar{};
    const auto start = fd::plant_state_from({0, 0, 0, 18, 0.03, 0}, car, config);
    // A plan of one command held for three seconds, on the fixed-tick grid every 0.05 s.
    std::vector<double> times;
    std::vector<fd::Command> commands;
    const fd::Command held{-4.0, 0.03};
    for (int k = 0; k <= 60; ++k) { times.push_back(0.05*k); commands.push_back(held); }
    const auto use = fd::plan_tire_use(car, start, config, times, commands, 0);
    require(use.size() == times.size(), "one entry per point of the plan");
    for (std::size_t k = 0; k < use.size(); ++k) near(use[k].time_s, times[k], 1e-12, "at the plan's own times");
    // The first entry is what the seam says of the state the plan was made in, under the command it holds first.
    const auto first = fd::demand(car, start, config, held.acceleration_mps2);
    for (std::size_t w = 0; w < 4; ++w)
        near(use.front().use[w], std::hypot(first.wheel_longitudinal_use[w], first.wheel_lateral_use[w]), 1e-12,
             "the first entry is demand()'s own friction use");
    // Braking moves load onto the front wheels, but the car brakes with half its torque at each axle, so it is the
    // unloaded rear tires that are asked for more of their own circle.
    const auto& late = use[40];
    require(late.use[fd::rear_left] > late.use[fd::front_left]+0.05 && late.use[fd::rear_right] > late.use[fd::front_right]+0.05,
            "braking asks more of the rear tires' circles than the front's");
    // Held at the cornering limit, every tire works: the loaded outer pair carries more force on less friction, the
    // unloaded inner pair less force on more, and all four end up near their own circles together.
    std::vector<fd::Command> turning(commands.size(), fd::Command{0.0, 0.12});
    const auto cornering = fd::plan_tire_use(car, fd::plant_state_from({0, 0, 0, 16, 0.12, 0}, car, config), config, times, turning, 0);
    const auto& mid = cornering[40];
    const double lowest = *std::min_element(mid.use.begin(), mid.use.end());
    const double highest = *std::max_element(mid.use.begin(), mid.use.end());
    require(lowest > 0.9 && highest < 1.02 && highest-lowest < 0.02, "at the cornering limit all four tires are near their circles");
    std::cout << "  tire use three seconds into a braking plan: FL " << use.back().use[fd::front_left] << ", FR "
              << use.back().use[fd::front_right] << ", RL " << use.back().use[fd::rear_left] << ", RR "
              << use.back().use[fd::rear_right] << "\n";
    // What a tire is asked at a point follows the command the plant is holding there, not the plan's first: driving,
    // the load leaves the front wheels and they are asked for more of their circles; braking, the rear ones are.
    std::vector<fd::Command> ramp;
    for (int k = 0; k <= 60; ++k) ramp.push_back({k < 30 ? 3.0 : -6.0, 0.0});
    const auto changing = fd::plan_tire_use(car, fd::plant_state_from({0, 0, 0, 12, 0, 0}, car, config), config, times, ramp, 0);
    require(changing[20].use[fd::front_left] > changing[20].use[fd::rear_left]+0.02, "a second in, driving, the front tires are the busier pair");
    require(changing[55].use[fd::rear_left] > changing[55].use[fd::front_left]+0.05, "and once the plan brakes, the rear ones are");
    // A car without four wheels has no per-wheel use of its own to report.
    require(fd::plan_tire_use(fd::DynamicSingleTrack{}, fd::plant_state_from({0, 0, 0, 18, 0.03, 0}, fd::DynamicSingleTrack{}, config),
                              config, times, commands, 0).empty(), "a single-track car reports none");
    require(fd::plan_tire_use(fd::KinematicBicycle{}, fd::plant_state_from({0, 0, 0, 18, 0.03, 0}, fd::KinematicBicycle{}, config),
                              config, times, commands, 0).empty(), "and the kinematic bicycle none");
    rejects([&] { fd::plan_tire_use(car, start, config, {0, 0.052, 0.1}, {held, held, held}, 0); }, "times off the tick grid are refused");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"actual_state_anchor_and_recovery", actual_state_anchor_and_recovery},
        {"command_source_and_fixed_tick_prediction", command_source_and_fixed_tick_prediction},
        {"off_grid_grip_change_keeps_regular_schedule", off_grid_grip_change_keeps_regular_schedule},
        {"grip_changes_immediate_action", grip_changes_immediate_action},
        {"infeasible_state_is_reported_without_repair", infeasible_state_is_reported_without_repair},
        {"seam_horizon_and_segment_contract", seam_horizon_and_segment_contract},
        {"invalid_prediction_inputs", invalid_prediction_inputs},
        {"analytic_track_projection", analytic_track_projection},
        {"pruned_projection_matches_exhaustive_scan", pruned_projection_matches_exhaustive_scan},
        {"arc_shorter_than_chord_is_rejected", arc_shorter_than_chord_is_rejected},
        {"a_plan_s_commands_change_linearly_between_its_points", a_plan_s_commands_change_linearly_between_its_points},
        {"the_plant_response_is_the_simulation_keeping_to_a_plan", the_plant_response_is_the_simulation_keeping_to_a_plan},
        {"each_tire_s_use_along_a_plan_is_the_plant_s_own", each_tire_s_use_along_a_plan_is_the_plant_s_own}
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size() - static_cast<std::size_t>(failures) << '/' << tests.size()
              << " trajectory groups passed\n";
    return failures ? 1 : 0;
}

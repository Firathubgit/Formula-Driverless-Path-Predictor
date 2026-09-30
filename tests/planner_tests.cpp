#include "fd/simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
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
        throw std::runtime_error(message+": actual="+std::to_string(actual)+
                                 ", expected="+std::to_string(expected));
}
template<class Action> void rejects(Action action, const std::string& message) {
    bool rejected = false;
    try { action(); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, message);
}

fd::State on_track(const fd::Track& track, const fd::Config& config, double station, double speed) {
    const auto here = fd::sample(track, station);
    const auto ahead = fd::sample(track, station+0.1);
    return {here.x_m, here.y_m, std::atan2(ahead.y_m-here.y_m, ahead.x_m-here.x_m), speed,
            std::atan(config.wheelbase_m*here.curvature), 0};
}

// Lateral position of a predicted point relative to the reference centerline.
double offset_at(const fd::Track& track, const fd::State& state) {
    return fd::project(track, {state.x_m, state.y_m}).signed_error_m;
}
double closest_offset_within(const fd::Track& track, const fd::LocalTrajectory& trajectory,
                             double from_s, double to_s) {
    double extreme = 0;
    bool seen = false;
    for (const auto& point : trajectory.points) {
        const auto projection = fd::project(track, {point.state.x_m, point.state.y_m});
        if (projection.s_m < from_s || projection.s_m > to_s) continue;
        if (!seen || std::abs(projection.signed_error_m) > std::abs(extreme)) {
            extreme = projection.signed_error_m;
            seen = true;
        }
    }
    require(seen, "predicted path reaches the inspected station interval");
    return extreme;
}

// A blockage covering the right side and the centerline of the main straight, so the
// only clear line is to the left. Stated by the scenario, not detected by anything.
fd::Obstruction right_side_blockage() {
    return {62, 68, -4.5, 1.0, "cone-cluster"};
}
fd::Obstruction full_width_blockage() {
    return {62, 68, -4.5, 4.5, "stalled-car"};
}

// Failures here are easier to read with the rejected alternatives attached.
std::string describe(const fd::LocalDecision& decision) {
    std::string text = " [reason='"+decision.reason+"' holding="+(decision.holding ? "yes" : "no")+
                       " selected="+std::to_string(decision.selected);
    for (const auto& option : decision.options)
        text += " | offset="+std::to_string(option.lateral_offset_m)+
                " clear="+(option.clear ? "yes" : "no")+
                " gain="+std::to_string(option.station_gain_m)+
                " cap="+(std::isfinite(option.speed_limit_mps) ? std::to_string(option.speed_limit_mps) : "none")+
                " reason='"+option.reason+"'";
    return text+"]";
}

void clear_track_reproduces_reference_following() {
    const auto track = fd::make_preset_track();
    const fd::Config config;
    const auto reference = fd::make_speed_plan(track, config);
    const auto state = on_track(track, config, 30, 15);
    const auto plain = fd::make_local_trajectory(track, reference, state, config);
    const auto decision = fd::choose_local_action(track, reference, state, config, {});
    require(decision.options.size() == 1, "an unobstructed corridor evaluates exactly one option");
    require(!decision.holding, "an unobstructed corridor is not a braking decision");
    near(decision.choice().lateral_offset_m, 0, 0, "unobstructed driving stays on the reference line");
    require(decision.blocking_identifier.empty(), "nothing is named as blocking");
    near(decision.trajectory().first_control.applied.steering_rad,
         plain.first_control.applied.steering_rad, 0,
         "unobstructed steering command is unchanged by the planner");
    near(decision.trajectory().first_control.applied.acceleration_mps2,
         plain.first_control.applied.acceleration_mps2, 0,
         "unobstructed acceleration command is unchanged by the planner");
    require(decision.choice().station_gain_m > 30, "reported advance covers the horizon");
}

void blockage_outside_the_predicted_path_is_ignored() {
    const auto track = fd::make_preset_track();
    const fd::Config config;
    const auto reference = fd::make_speed_plan(track, config);
    const auto state = on_track(track, config, 30, 15);
    // Far beyond the three-second horizon.
    const auto decision = fd::choose_local_action(track, reference, state, config,
                                                  {{300, 306, -4.5, 1.0, "distant"}});
    require(decision.options.size() == 1 && decision.choice().clear,
            "a blockage the prediction never reaches leaves the reference line selected");
    require(!decision.holding, "a distant blockage does not cause braking");
    // Alongside the racing line rather than on it.
    const auto beside = fd::choose_local_action(track, reference, state, config,
                                                {{62, 68, 3.0, 4.5, "verge"}});
    require(beside.options.size() == 1 && beside.choice().clear,
            "a blockage clear of the predicted path leaves the reference line selected");
}

void blocked_line_selects_the_clear_alternative() {
    const auto track = fd::make_preset_track();
    const fd::Config config;
    const auto reference = fd::make_speed_plan(track, config);
    const auto state = on_track(track, config, 30, 15);
    const auto blockage = right_side_blockage();
    const auto decision = fd::choose_local_action(track, reference, state, config, {blockage});
    require(decision.options.size() > 1, "a blocked reference line forces alternatives to be evaluated");
    require(!decision.holding, "a clear alternative exists, so this is not a braking decision"+describe(decision));
    require(decision.blocking_identifier == "cone-cluster", "the decision names what forced the change");
    const auto& choice = decision.choice();
    require(choice.clear, "the selected option is one that was found clear"+describe(decision));
    require(choice.lateral_offset_m > 1, "the only clear side is left of the blockage"+describe(decision));
    require(decision.reason.find("left") != std::string::npos &&
            decision.reason.find("cone-cluster") != std::string::npos,
            "the decision explains the direction and the cause: "+decision.reason);
    const double passing = closest_offset_within(track, decision.trajectory(),
                                                 blockage.from_s_m, blockage.to_s_m);
    require(passing > blockage.to_offset_m,
            "the selected path passes left of the blocked lateral interval");
    require(decision.trajectory().first_control.applied.steering_rad > 0,
            "the very first command already steers left, not only the distant future");
    const auto centerline = std::find_if(decision.options.begin(), decision.options.end(),
        [](const fd::LocalOption& o) { return o.lateral_offset_m == 0; });
    require(centerline != decision.options.end() && !centerline->clear &&
            centerline->reason.find("cone-cluster") != std::string::npos,
            "the rejected reference line records why it was rejected");
}

void fully_blocked_corridor_brakes_instead_of_committing() {
    const auto track = fd::make_preset_track();
    const fd::Config config;
    const auto reference = fd::make_speed_plan(track, config);
    const auto state = on_track(track, config, 48, 15);
    const auto decision = fd::choose_local_action(track, reference, state, config, {full_width_blockage()});
    require(decision.holding, "no clear line across the corridor produces a speed-limited decision"+describe(decision));
    require(decision.blocking_identifier == "stalled-car", "the decision names the blockage");
    require(decision.reason.find("stalled-car") != std::string::npos,
            "the decision explains itself: "+decision.reason);
    require(std::none_of(decision.options.begin(), decision.options.end(),
                         [](const fd::LocalOption& o) { return o.clear; }),
            "every evaluated alternative was rejected before limiting speed"+describe(decision));
    const auto& choice = decision.choice();
    near(choice.lateral_offset_m, 0, 0, "the speed-limited option holds the reference line");
    require(choice.speed_limit_mps < state.speed_mps,
            "this little remaining room caps the target below the current speed"+describe(decision));
    require(decision.reason.find("braking") != std::string::npos,
            "a binding cap is described as braking: "+decision.reason);
    require(decision.trajectory().first_control.applied.acceleration_mps2 < -1,
            "the first command is actual braking"+describe(decision));
    // Far enough back the same blockage is a limit the car has not reached yet.
    const auto distant = fd::choose_local_action(track, reference, on_track(track, config, 20, 15),
                                                 config, {full_width_blockage()});
    require(distant.holding && distant.choice().speed_limit_mps > 15,
            "ample room leaves the cap above the current speed"+describe(distant));
    require(distant.reason.find("braking") == std::string::npos,
            "a cap that does not bind is not called braking: "+distant.reason);
    require(distant.choice().speed_limit_mps > choice.speed_limit_mps,
            "more remaining room permits more speed");
    // Closing on the blockage tightens the cap until it demands a stop.
    const auto touching = fd::choose_local_action(track, reference, on_track(track, config, 63, 5),
                                                 config, {full_width_blockage()});
    require(touching.holding, "a car already inside the blocked stations still brakes");
    near(touching.choice().speed_limit_mps, 0, 0, "no remaining room demands a full stop");
}

void invalid_scenarios_are_rejected() {
    const auto track = fd::make_preset_track();
    const fd::Config config;
    const auto reference = fd::make_speed_plan(track, config);
    const auto state = on_track(track, config, 30, 15);
    const auto reject = [&](fd::Obstruction o, const std::string& message) {
        rejects([&] { fd::choose_local_action(track, reference, state, config, {o}); }, message);
    };
    auto o = right_side_blockage();
    o.from_s_m = std::numeric_limits<double>::quiet_NaN();
    reject(o, "nonfinite station is rejected");
    o = right_side_blockage();
    o.to_s_m = track.length_m;
    reject(o, "station at the wrapped length is rejected");
    o = right_side_blockage();
    o.from_s_m = -1;
    reject(o, "negative station is rejected");
    o = right_side_blockage();
    o.from_offset_m = 2;
    o.to_offset_m = -2;
    reject(o, "inverted lateral interval is rejected");
    o = right_side_blockage();
    o.to_s_m = o.from_s_m;
    reject(o, "an empty station span is rejected");
    o = right_side_blockage();
    o.identifier = "has,separator";
    reject(o, "an identifier that would corrupt recorded text is rejected");
    o = right_side_blockage();
    o.identifier.clear();
    reject(o, "an unnamed blockage is rejected");
}

void wrapped_blockage_is_handled_at_the_seam() {
    const auto track = fd::make_preset_track();
    const fd::Config config;
    const auto reference = fd::make_speed_plan(track, config);
    // Spanning start/finish: from near the end of the lap to just after it.
    const fd::Obstruction seam{track.length_m-4, 3, -4.5, 4.5, "seam-blockage"};
    const auto approaching = on_track(track, config, track.length_m-18, 12);
    const auto decision = fd::choose_local_action(track, reference, approaching, config, {seam});
    require(decision.holding, "a blockage across the seam is seen from before the seam"+describe(decision));
    require(decision.blocking_identifier == "seam-blockage", "the seam blockage is named");
    require(decision.choice().speed_limit_mps < 12,
            "the seam blockage limits speed across start/finish"+describe(decision));
    const auto clear_side = fd::choose_local_action(track, reference, on_track(track, config, 60, 15),
                                                    config, {seam});
    require(clear_side.options.size() == 1 && clear_side.choice().clear,
            "the same seam blockage is behind a car further along the lap");
}

// Driving the plant, not predicting: does the chosen action change where the car ends up?
struct DrivenRun {
    std::vector<fd::State> states;
    double lateral_at_blockage{};   // most displaced offset while inside the blocked stations
    bool entered_blockage{};
    double furthest_station{};
    double final_speed{};
    std::string trace;              // half-second samples, so a failure shows the approach
};

DrivenRun drive(const std::vector<fd::Obstruction>& obstructions, const fd::Obstruction& region,
                double until_progress_m, double time_limit_s) {
    fd::Simulation simulation;
    simulation.set_obstructions(obstructions);
    DrivenRun run;
    while (simulation.diagnostics().progress_m < until_progress_m &&
           simulation.state().time_s < time_limit_s) {
        // Once the car has come to rest it stays there; keep the run bounded.
        if (simulation.state().time_s > 3 && simulation.state().speed_mps < 0.02) break;
        simulation.step();
        const auto& state = simulation.state();
        run.states.push_back(state);
        const auto projection = fd::project(simulation.track(), {state.x_m, state.y_m});
        run.furthest_station = std::max(run.furthest_station, projection.s_m);
        if (run.states.size()%100 == 0) {
            const auto& choice = simulation.decision().choice();
            run.trace += "\n    t="+std::to_string(state.time_s)+
                         " s="+std::to_string(projection.s_m)+
                         " v="+std::to_string(state.speed_mps)+
                         " cap="+(std::isfinite(choice.speed_limit_mps)
                                      ? std::to_string(choice.speed_limit_mps) : "none")+
                         " offset="+std::to_string(choice.lateral_offset_m)+
                         " holding="+(simulation.decision().holding ? "yes" : "no");
        }
        if (projection.s_m < region.from_s_m || projection.s_m > region.to_s_m) continue;
        if (std::abs(projection.signed_error_m) > std::abs(run.lateral_at_blockage))
            run.lateral_at_blockage = projection.signed_error_m;
        // The stated region itself, with no vehicle-margin inflation.
        if (projection.signed_error_m >= region.from_offset_m &&
            projection.signed_error_m <= region.to_offset_m)
            run.entered_blockage = true;
    }
    run.final_speed = simulation.state().speed_mps;
    return run;
}

void selection_changes_the_driven_path() {
    const auto blockage = right_side_blockage();
    const auto baseline = drive({}, blockage, 90, 40);
    const auto avoiding = drive({blockage}, blockage, 90, 40);
    require(baseline.entered_blockage,
            "the unobstructed car drives straight through that region, so the scenario is meaningful");
    require(!avoiding.entered_blockage,
            "the same car given the blockage does not drive into it");
    require(avoiding.lateral_at_blockage > blockage.to_offset_m,
            "the driven path passes left of the stated blockage: actual offset "+
            std::to_string(avoiding.lateral_at_blockage));
    require(std::abs(baseline.lateral_at_blockage) < 0.5,
            "the baseline stays on the reference line there");
    require(avoiding.furthest_station > 80 && baseline.furthest_station > 80,
            "both runs continue past the blockage rather than stalling");
}

void unavoidable_blockage_stops_the_car() {
    const auto blockage = full_width_blockage();
    const auto stopping = drive({blockage}, blockage, 200, 25);
    const std::string reached = " reached station "+std::to_string(stopping.furthest_station)+
                                " at "+std::to_string(stopping.final_speed)+" m/s"+stopping.trace;
    require(!stopping.entered_blockage, "a fully blocked corridor is never entered;"+reached);
    require(stopping.furthest_station < blockage.from_s_m,
            "the car halts before the blocked stations;"+reached);
    require(stopping.furthest_station > 50,
            "the car still drives most of the straight before it has to stop");
    near(stopping.final_speed, 0, 0.35, "the car comes to rest rather than creeping into the blockage");
}

// A reference line that is not the corridor's centre, such as a racing line, carries its distance to each edge; the
// corridor checks of the simulation and the prediction, and the planner's alternatives, use it (decision 0020). A
// centreline without edges behaves exactly as before.
void an_off_centre_reference_uses_its_own_corridor() {
    const fd::Config config;
    const auto centred = fd::make_preset_track();
    auto shifted = centred;
    shifted.left_edge_m.assign(shifted.points.size(), 1.6);
    shifted.right_edge_m.assign(shifted.points.size(), 8.4);
    fd::validate_track(shifted);
    const auto beside = [&](const fd::Track& t, double station, double offset) {
        const auto p = fd::sample(t, station);
        const auto a = fd::sample(t, station+0.1);
        const double heading = std::atan2(a.y_m-p.y_m, a.x_m-p.x_m);
        return fd::State{p.x_m-std::sin(heading)*offset, p.y_m+std::cos(heading)*offset, heading, 10, 0, 0};
    };
    const auto projected = [&](const fd::Track& t, double offset) {
        const auto s = beside(t, 60, offset);
        return fd::project(t, {s.x_m, s.y_m});
    };
    require(fd::within_corridor(centred, projected(centred, 4.0), 0.9) && !fd::within_corridor(centred, projected(centred, -4.2), 0.9),
            "a centreline's corridor is half its width either side, less the margin");
    const auto corridor = fd::corridor_at(shifted, projected(shifted, 0.3));
    near(corridor.left_m, 1.6, 1e-12, "the corridor's left edge is the stated distance");
    near(corridor.right_m, 8.4, 1e-12, "and its right edge");
    near(fd::corridor_at(centred, projected(centred, 0.3)).left_m, 5, 1e-12, "a centreline's edges are half its width");
    require(fd::within_corridor(shifted, projected(shifted, 0.6), 0.9) && !fd::within_corridor(shifted, projected(shifted, 0.8), 0.9),
            "the left edge is 1.6 m away");
    require(fd::within_corridor(shifted, projected(shifted, -7.4), 0.9) && !fd::within_corridor(shifted, projected(shifted, -7.6), 0.9),
            "the right edge is 8.4 m away");

    auto bad = shifted;
    bad.right_edge_m.pop_back();
    rejects([&] { fd::validate_track(bad); }, "edges need one distance per sample");
    bad = shifted;
    bad.right_edge_m.clear();
    rejects([&] { fd::validate_track(bad); }, "edges come in pairs");
    bad = shifted;
    bad.left_edge_m[3] = -0.1;
    rejects([&] { fd::validate_track(bad); }, "an edge cannot be behind the reference");
    bad = shifted;
    bad.left_edge_m[5] = std::nan("");
    rejects([&] { fd::validate_track(bad); }, "edges must be finite");

    // The prediction checks the same corridor: 0.8 m left is outside it on the shifted reference, not on the centreline.
    const auto state = beside(shifted, 60, 0.8);
    const auto shifted_prediction = fd::make_local_trajectory(shifted, fd::make_speed_plan(shifted, config), state, config);
    const auto centred_prediction = fd::make_local_trajectory(centred, fd::make_speed_plan(centred, config), state, config);
    require(!shifted_prediction.within_model_envelope && shifted_prediction.validity_reason == "Predicted rear axle exceeds track margin",
            "the prediction leaves the shifted corridor");
    require(centred_prediction.validity_reason != "Predicted rear axle exceeds track margin", "but not the centreline's");

    // The planner's alternatives reach as far as each edge allows, not half the width both ways.
    const auto decision = fd::choose_local_action(shifted, fd::make_speed_plan(shifted, config), beside(shifted, 40, 0), config,
                                                  {right_side_blockage()});
    require(decision.options.size() >= 5, "five alternatives were evaluated"+describe(decision));
    std::vector<double> offsets;
    for (std::size_t i = 0; i < 5; ++i) offsets.push_back(decision.options[i].lateral_offset_m);
    std::sort(offsets.begin(), offsets.end());
    near(offsets[0], -0.75*7.5, 1e-9, "the right alternatives use the right edge");
    near(offsets[1], -0.4*7.5, 1e-9, "the right alternatives use the right edge");
    near(offsets[3], 0.4*0.7, 1e-9, "the left alternatives use the left edge");
    near(offsets[4], 0.75*0.7, 1e-9, "the left alternatives use the left edge");

    // The simulation's on-track flag uses it too: on a reference 0.5 m from its left edge the car is already beyond the margin.
    auto narrow = centred;
    narrow.left_edge_m.assign(narrow.points.size(), 0.5);
    narrow.right_edge_m.assign(narrow.points.size(), 9.5);
    fd::Simulation simulation(narrow, config);
    fd::Simulation centred_simulation(centred, config);
    require(!simulation.diagnostics().within_track && centred_simulation.diagnostics().within_track,
            "the simulation judges the car against the reference's own corridor");
}

void planning_cost_is_measured() {
    const auto track = fd::make_preset_track();
    const fd::Config config;
    const auto reference = fd::make_speed_plan(track, config);
    const auto state = on_track(track, config, 30, 15);
    const auto blockage = right_side_blockage();
    const auto timed = [&](const std::vector<fd::Obstruction>& obstructions) {
        const auto began = std::chrono::steady_clock::now();
        constexpr int repeats = 20;
        for (int i = 0; i < repeats; ++i)
            fd::choose_local_action(track, reference, state, config, obstructions);
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now()-began).count()/repeats;
    };
    const double clear_ms = timed({});
    const double blocked_ms = timed({blockage});
    std::cout << "  planning cost: clear " << clear_ms << " ms, blocked " << blocked_ms
              << " ms per decision (measurement only, no real-time guarantee)\n";
    require(blocked_ms < clear_ms*12+5,
            "evaluating alternatives stays within a small multiple of one prediction");
}
} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"clear_track_reproduces_reference_following", clear_track_reproduces_reference_following},
        {"blockage_outside_the_predicted_path_is_ignored", blockage_outside_the_predicted_path_is_ignored},
        {"blocked_line_selects_the_clear_alternative", blocked_line_selects_the_clear_alternative},
        {"fully_blocked_corridor_brakes_instead_of_committing", fully_blocked_corridor_brakes_instead_of_committing},
        {"invalid_scenarios_are_rejected", invalid_scenarios_are_rejected},
        {"wrapped_blockage_is_handled_at_the_seam", wrapped_blockage_is_handled_at_the_seam},
        {"selection_changes_the_driven_path", selection_changes_the_driven_path},
        {"unavoidable_blockage_stops_the_car", unavoidable_blockage_stops_the_car},
        {"an_off_centre_reference_uses_its_own_corridor", an_off_centre_reference_uses_its_own_corridor},
        {"planning_cost_is_measured", planning_cost_is_measured}
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size()
              << " planner groups passed\n";
    return failures ? 1 : 0;
}

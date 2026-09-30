#include "fd/lattice.hpp"
#include "fd/performance_envelope.hpp"
#include "fd/local_planner.hpp"
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

// Phases 5.2 and 5.3 (decisions 0022 and 0023): the car searches the offline lattice from where it is, times each
// action's path by its own speed profile and takes the fastest clear one. The oracles are the reference's geometry, the
// stated blockages and the car's own predicted and driven motion.
namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tolerance)
        throw std::runtime_error(message+": actual="+std::to_string(actual)+", expected="+std::to_string(expected));
}
template<class Action> std::string refusal(Action action) {
    try { action(); } catch (const std::invalid_argument& error) { return error.what(); }
    return {};
}

const fd::Track preset = fd::make_preset_track();
const fd::Config config;

fd::State on_track(double station, double speed) {
    const auto here = fd::sample(preset, station);
    const auto ahead = fd::sample(preset, station+0.1);
    return {here.x_m, here.y_m, std::atan2(ahead.y_m-here.y_m, ahead.x_m-here.x_m), speed, 0, 0};
}

// A path intent aims the pursuit target at the path's offset at the target's own station: none of the path is a
// constant offset, so the car follows a line that moves across the reference. On the preset's opening straight the
// target of a car at 30 m and 10 m/s is 6.5 m ahead, where this path has reached 3 m left.
void the_pursuit_target_takes_the_paths_offset_at_its_station() {
    const auto plan = fd::make_speed_plan(preset, config);
    const auto state = on_track(30, 10);
    const std::vector<fd::StationOffset> zeros{{30, 0}, {40, 0}, {90, 0}};
    const auto plain = fd::compute_control(preset, plan, state, config);
    fd::ControlIntent along_zeros;
    along_zeros.path = zeros;
    const auto on_zeros = fd::compute_control(preset, plan, state, config, along_zeros);
    require(on_zeros.requested.steering_rad == plain.requested.steering_rad && on_zeros.applied.acceleration_mps2 == plain.applied.acceleration_mps2,
            "a path of zero offsets is plain reference following, command for command");

    const std::vector<fd::StationOffset> across{{30, 0}, {32, 0}, {36, 3}, {90, 3}};
    fd::ControlIntent intent;
    intent.path = across;
    const auto moving = fd::compute_control(preset, plan, state, config, intent);
    const double ahead = config.lookahead_base_m+config.lookahead_time_s*10;
    near(moving.geometric_steering_rad, std::atan2(2*config.wheelbase_m*3, ahead*ahead+9), 1e-9, "the target is the path's offset at its station");
    // At 6 m/s the target is 5.5 m ahead, at 35.5 m, part way up the path's ramp from 0 at 32 m to 3 at 36 m.
    const double slower = config.lookahead_base_m+config.lookahead_time_s*6;
    const double ramp = 3*(30+slower-32)/4;
    near(fd::compute_control(preset, plan, on_track(30, 6), config, intent).geometric_steering_rad,
         std::atan2(2*config.wheelbase_m*ramp, slower*slower+ramp*ramp), 1e-9, "between the path's points its offset is interpolated");

    const std::vector<fd::StationOffset> later{{40, -2}, {50, -2}};
    intent.path = later;
    near(fd::compute_control(preset, plan, state, config, intent).geometric_steering_rad, std::atan2(-2*config.wheelbase_m*2, ahead*ahead+4), 1e-9,
         "before the path begins the target takes its first offset");
    const std::vector<fd::StationOffset> shorter{{30, 0}, {33, 1.5}};
    intent.path = shorter;
    near(fd::compute_control(preset, plan, state, config, intent).geometric_steering_rad, std::atan2(2*config.wheelbase_m*1.5, ahead*ahead+2.25), 1e-9,
         "beyond its end the target takes its last offset");

    const std::vector<fd::StationOffset> backwards{{40, 0}, {35, 1}};
    intent.path = backwards;
    require(refusal([&] { fd::compute_control(preset, plan, state, config, intent); }).find("path") != std::string::npos,
            "a path whose stations do not increase is refused");
    const std::vector<fd::StationOffset> broken{{30, 0}, {40, std::numeric_limits<double>::quiet_NaN()}};
    intent.path = broken;
    require(refusal([&] { fd::compute_control(preset, plan, state, config, intent); }).find("path") != std::string::npos,
            "a path with a non-finite offset is refused");
}

// Stations may run past the track's length, so a path can cross the start/finish line: a car just before it aims at
// the offset the path has just after it.
void a_path_crosses_the_seam() {
    const auto plan = fd::make_speed_plan(preset, config);
    const double start = preset.length_m-3;
    const auto state = on_track(start, 10);
    const std::vector<fd::StationOffset> across{{start, 0}, {start+2, 0}, {start+5, 2}, {start+40, 2}};
    fd::ControlIntent intent;
    intent.path = across;
    const std::vector<fd::StationOffset> constant{{start, 2}, {start+40, 2}};
    fd::ControlIntent level;
    level.path = constant;
    near(fd::compute_control(preset, plan, state, config, intent).geometric_steering_rad,
         fd::compute_control(preset, plan, state, config, level).geometric_steering_rad, 1e-12,
         "past the seam the target takes the offset the path has there");
}

const fd::Lattice lattice = fd::make_lattice(preset, config);

fd::LocalDecision decide(double station, double speed, const std::vector<fd::Obstruction>& obstructions) {
    const auto plan = fd::make_speed_plan(preset, config);
    const fd::VehicleModel kinematic = fd::KinematicBicycle{};
    return fd::choose_lattice_action(preset, plan, fd::plant_state_from(on_track(station, speed), kinematic, config), config, kinematic,
                                     &lattice, obstructions);
}

std::string describe(const fd::LocalDecision& decision) {
    std::string text = " [reason='"+decision.reason+"' holding="+(decision.holding ? "yes" : "no")+" selected="+std::to_string(decision.selected);
    for (const auto& o : decision.options)
        text += " | "+std::string(fd::local_action_name(o.action))+" offset="+std::to_string(o.lateral_offset_m)+" clear="+(o.clear ? "yes" : "no")+
                " cost="+std::to_string(o.path_cost.total())+" reason='"+o.reason+"'";
    return text+"]";
}

// Where a path lies across the reference at the stations a blockage covers, widened by the car's half width: its
// offsets there, least and greatest.
std::pair<double, double> path_offsets_within(const fd::LocalOption& option, const fd::Obstruction& o) {
    double lowest = 1e9, highest = -1e9;
    for (const auto& p : option.path) {
        const double s = std::fmod(p.s_m, preset.length_m);
        if (s < o.from_s_m-fd::vehicle_half_width_m || s > o.to_s_m+fd::vehicle_half_width_m) continue;  // at least this widened
        lowest = std::min(lowest, p.offset_m);
        highest = std::max(highest, p.offset_m);
    }
    return {lowest, highest};
}

// While the reference line stays clear, the lattice planner follows it with one prediction, exactly as the five-offset
// planner does: a clear track, and a blockage the prediction never reaches.
void a_clear_reference_line_is_followed_with_one_prediction() {
    const auto plan = fd::make_speed_plan(preset, config);
    for (const auto& obstructions : {std::vector<fd::Obstruction>{}, std::vector<fd::Obstruction>{{300, 306, -4.5, 4.5, "far-away"}}}) {
        const auto lattice_decision = decide(30, 15, obstructions);
        const auto offsets_decision = fd::choose_local_action(preset, plan, on_track(30, 15), config, obstructions);
        require(lattice_decision.options.size() == 1 && lattice_decision.choice().action == fd::LocalAction::straight && !lattice_decision.holding,
                "one straight option"+describe(lattice_decision));
        require(lattice_decision.choice().path.empty() && lattice_decision.choice().path_cost.total() == 0, "following the reference needs no lattice path");
        const auto& a = lattice_decision.trajectory().first_control;
        const auto& b = offsets_decision.trajectory().first_control;
        require(a.applied.acceleration_mps2 == b.applied.acceleration_mps2 && a.applied.steering_rad == b.applied.steering_rad &&
                lattice_decision.trajectory().points.size() == offsets_decision.trajectory().points.size(),
                "command for command the same as the five-offset planner");
        require(lattice_decision.reason == offsets_decision.reason, "and explained the same way: "+lattice_decision.reason);
    }
}

// A blockage across the reference line is passed on its clear side along a lattice path: the path and the car's
// predicted motion keep the car's half width clear of it, and the decision names the action and the blockage.
void a_blocked_reference_is_passed_on_the_clear_side() {
    const std::vector<std::pair<fd::Obstruction, fd::LocalAction>> cases{
        {{62, 68, -4.5, 1.0, "cone-cluster"}, fd::LocalAction::pass_left},
        {{62, 68, -1.0, 4.5, "barrier"}, fd::LocalAction::pass_right},
    };
    for (const auto& [blockage, side] : cases) {
        const auto decision = decide(30, 15, {blockage});
        const auto& choice = decision.choice();
        const bool left = side == fd::LocalAction::pass_left;
        require(!decision.holding && choice.action == side && choice.clear, "the clear side is passed"+describe(decision));
        require(decision.blocking_identifier == blockage.identifier && decision.reason.find(blockage.identifier) != std::string::npos &&
                decision.reason.find(left ? "left" : "right") != std::string::npos, "the decision names the side and the blockage: "+decision.reason);
        const auto [lowest, highest] = path_offsets_within(choice, blockage);
        require(left ? lowest >= blockage.to_offset_m+fd::vehicle_half_width_m-1e-9 : highest <= blockage.from_offset_m-fd::vehicle_half_width_m+1e-9,
                "the lattice path keeps the half width clear of the blockage"+describe(decision));
        double passing = left ? 1e9 : -1e9;
        for (const auto& point : choice.trajectory.points) {
            const auto at = fd::project(preset, {point.state.x_m, point.state.y_m});
            if (at.s_m < blockage.from_s_m || at.s_m > blockage.to_s_m) continue;
            passing = left ? std::min(passing, at.signed_error_m) : std::max(passing, at.signed_error_m);
        }
        require(left ? passing > blockage.to_offset_m : passing < blockage.from_offset_m, "and so does the car's predicted motion");
        const auto straight = std::find_if(decision.options.begin(), decision.options.end(), [](const fd::LocalOption& o) { return o.action == fd::LocalAction::straight; });
        require(straight != decision.options.end() && !straight->clear && straight->reason.find(blockage.identifier) != std::string::npos,
                "the blocked reference line is kept with its reason");
        require(choice.path.front().s_m == fd::project(preset, {on_track(30, 15).x_m, on_track(30, 15).y_m}).s_m, "the path starts at the car");
        const auto& terms = choice.path_cost;
        near(terms.edges.total()+terms.turns+terms.goal, terms.total(), 1e-9, "the path's cost is its edges' terms, its turns and its goal term");
        require(terms.edges.reference_deviation > 0 && terms.edges.average_curvature > 0 && terms.turns > 0, "leaving the reference, curving and turning all cost");
    }
}

// When both sides are clear, the side whose path its own speed profile drives in less time is taken (decision 0023).
// Past a narrow blockage left of centre on the straight, passing right is both the cheaper and the faster path. Just
// before the first corner, a left bend of 18 m, a blockage from -0.5 to 1.5 m makes passing right the cheaper path, but
// passing left, on the inside, is shorter and takes less time, so the car passes left.
void the_faster_side_is_passed_when_both_are_clear() {
    const auto sides = [&](const fd::LocalDecision& decision) {
        const auto find = [&](fd::LocalAction action) {
            return std::find_if(decision.options.begin(), decision.options.end(), [&](const fd::LocalOption& o) { return o.action == action; });
        };
        const auto left = find(fd::LocalAction::pass_left), right = find(fd::LocalAction::pass_right);
        require(left != decision.options.end() && right != decision.options.end() && left->clear && right->clear, "both sides are clear"+describe(decision));
        return std::pair{*left, *right};
    };
    const fd::Obstruction narrow{62, 68, -0.5, 1.5, "bollard"};
    const auto straight = decide(30, 15, {narrow});
    const auto [bollard_left, bollard_right] = sides(straight);
    require(straight.choice().action == fd::LocalAction::pass_right && bollard_right.estimated_time_s < bollard_left.estimated_time_s &&
            bollard_right.path_cost.total() < bollard_left.path_cost.total(), "on the straight the faster path, to the right, is also the cheaper"+describe(straight));
    std::cout << "  past a bollard from -0.5 to 1.5 m: left " << bollard_left.estimated_time_s << " s costing " << bollard_left.path_cost.total()
              << ", right " << bollard_right.estimated_time_s << " s costing " << bollard_right.path_cost.total() << "\n";

    const fd::Obstruction corner{130, 136, -0.5, 1.5, "bollard"};
    const auto bend = decide(95, 20, {corner});
    const auto [inside, outside] = sides(bend);
    std::cout << "  before the first corner: inside, left, " << inside.estimated_time_s << " s costing " << inside.path_cost.total()
              << "; outside, right, " << outside.estimated_time_s << " s costing " << outside.path_cost.total() << "\n";
    require(outside.path_cost.total() < inside.path_cost.total(), "the outside path is the cheaper"+describe(bend));
    require(bend.choice().action == fd::LocalAction::pass_left && inside.estimated_time_s < outside.estimated_time_s,
            "yet the inside path is faster and taken"+describe(bend));
    require(bend.reason.find("faster than passing right") != std::string::npos, "the reason says how much faster: "+bend.reason);
    const auto lowest = [](const fd::LocalOption& option) {
        return std::min_element(option.speed_profile.begin(), option.speed_profile.end(),
                                [](const fd::ProfilePoint& a, const fd::ProfilePoint& b) { return a.speed_mps < b.speed_mps; })->speed_mps;
    };
    require(lowest(inside) < lowest(outside) && inside.speed_profile.back().distance_m < outside.speed_profile.back().distance_m,
            "the inside is slower at its slowest but shorter");
}

// Estimates are about 1% off over the planning horizon, so a decision keeps the previous choice while it is clear and
// within lattice_switch_margin_s of the fastest: past the bollard on the straight, passing right is faster by a few
// milliseconds, so a car that was passing left keeps passing left; before the corner, passing left is faster by more
// than the margin, so a car that was passing right switches.
void a_near_tie_keeps_the_previous_choice() {
    const auto plan = fd::make_speed_plan(preset, config);
    const fd::VehicleModel kinematic = fd::KinematicBicycle{};
    const auto from = [&](double station, double speed, const fd::Obstruction& o, const fd::LocalOption* previous) {
        return fd::choose_lattice_action(preset, plan, fd::plant_state_from(on_track(station, speed), kinematic, config), config, kinematic, &lattice,
                                         {o}, 0, nullptr, nullptr, previous);
    };
    const auto option = [](const fd::LocalDecision& decision, fd::LocalAction action) {
        return *std::find_if(decision.options.begin(), decision.options.end(), [&](const fd::LocalOption& o) { return o.action == action; });
    };
    const fd::Obstruction narrow{62, 68, -0.5, 1.5, "bollard"};
    const auto fresh = from(30, 15, narrow, nullptr);
    const auto left = option(fresh, fd::LocalAction::pass_left), right = option(fresh, fd::LocalAction::pass_right);
    require(fresh.choice().action == fd::LocalAction::pass_right && left.estimated_time_s-right.estimated_time_s < fd::lattice_switch_margin_s,
            "without a previous choice the faster, right, is taken by less than the margin"+describe(fresh));
    const auto kept = from(30, 15, narrow, &left);
    require(kept.choice().action == fd::LocalAction::pass_left && kept.reason.find("kept within") != std::string::npos,
            "a car that was passing left keeps passing left: "+kept.reason);

    const fd::Obstruction corner{130, 136, -0.5, 1.5, "bollard"};
    const auto bend = from(95, 20, corner, nullptr);
    const auto outside = option(bend, fd::LocalAction::pass_right);
    const auto switched = from(95, 20, corner, &outside);
    require(switched.choice().action == fd::LocalAction::pass_left, "a car that was passing right switches when left is faster by more"+describe(switched));
}

// Every path the lattice planner compares has its own speed profile from the car's station and actual speed to a horizon
// common to the decision, its estimated time is that profile's, and its prediction follows that profile. The blocked
// reference line is not compared and has none.
void each_action_is_predicted_along_its_own_speed_profile() {
    const fd::Obstruction corner{130, 136, -0.5, 1.5, "bollard"};
    const auto decision = decide(95, 20, {corner});
    const double here = fd::project(preset, {on_track(95, 20).x_m, on_track(95, 20).y_m}).s_m;
    std::size_t profiled = 0;
    double horizon = 0;
    for (const auto& option : decision.options) {
        if (option.action == fd::LocalAction::straight) {
            require(option.speed_profile.empty() && std::isinf(option.estimated_time_s) && !option.clear, "the blocked reference line has no profile");
            continue;
        }
        const auto& profile = option.speed_profile;
        require(profile.size() > 2, "each pass has a profile");
        near(profile.front().s_m, here, 1e-9, "starting at the car's station");
        near(profile.front().speed_mps, 20, 0, "from its actual speed");
        if (profiled == 0) horizon = profile.back().s_m;
        near(profile.back().s_m, horizon, 1e-9, "to the decision's common horizon");
        require(horizon-here >= fd::lattice_planning_horizon_m-1e-9, "at least the planning horizon ahead");
        near(option.estimated_time_s, fd::profile_time(profile), 0, "its time is its profile's");
        near(option.trajectory.first_control.target_speed_mps, profile.front().speed_mps, 1e-12, "its prediction targets the profile's speed");
        ++profiled;
    }
    require(profiled == 2, "both passes are profiled");
}

// The estimate is what the car drives: approaching the corner blockage from the preset's start, the first decision to
// pass it estimates the time to its horizon, and the car, re-planning every control tick, takes about that long.
void the_estimate_is_what_the_car_drives() {
    const fd::Obstruction corner{130, 136, -0.5, 1.5, "bollard"};
    fd::Simulation simulation(preset, config, fd::KinematicBicycle{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions,
                              fd::LocalPlannerMode::lattice);
    simulation.set_obstructions({corner});
    auto seen = simulation.decision_count();
    double estimate = 0, horizon = 0, decided = 0;
    std::string action;
    while (simulation.state().time_s < 30) {
        simulation.step();
        if (estimate == 0 && simulation.decision_count() != seen) {
            seen = simulation.decision_count();
            const auto& choice = simulation.decision().choice();
            if (choice.action == fd::LocalAction::pass_left || choice.action == fd::LocalAction::pass_right) {
                estimate = choice.estimated_time_s;
                horizon = choice.speed_profile.back().s_m;
                action = std::string(fd::local_action_name(choice.action));
                // The decision was made from the state before this step.
                decided = simulation.state().time_s-simulation.config().fixed_dt_s;
            }
        }
        if (estimate > 0 && simulation.diagnostics().path_s_m >= horizon) break;
    }
    require(estimate > 0, "the car decided to pass the corner blockage");
    const double driven = simulation.state().time_s-decided;
    std::cout << "  corner blockage: first decision to " << action << " estimated " << estimate << " s to " << horizon << " m; the car took "
              << driven << " s\n";
    near(driven, estimate, 0.05*estimate, "within 5% of the estimate");
}

// When no lattice path passes a blockage, the car brakes toward the widest gap across the corridor at the blockage,
// follow the gap, rather than holding the reference line: a full-width blockage has no gap, so it brakes on the
// reference; a car too close to a blockage with a gap on its left steers toward that gap's centre while braking.
void a_blocked_corridor_brakes_toward_the_widest_gap() {
    const fd::Obstruction wall{62, 68, -4.5, 4.5, "stalled-car"};
    const auto stopping = decide(48, 15, {wall});
    const auto& brake = stopping.choice();
    require(stopping.holding && brake.action == fd::LocalAction::brake, "a full-width blockage brakes"+describe(stopping));
    near(brake.lateral_offset_m, 0, 0, "with no gap it brakes along the reference");
    require(brake.speed_limit_mps < 15 && brake.trajectory.first_control.applied.acceleration_mps2 < -1, "and the first command brakes");
    require(stopping.reason.find("stalled-car") != std::string::npos && stopping.reason.find("no gap") != std::string::npos,
            "the reason says there is no gap: "+stopping.reason);

    const fd::Obstruction most{62, 68, -4.5, 2.0, "trailer"};
    const auto late = decide(58, 12, {most});
    const auto& gap = late.choice();
    require(late.holding && gap.action == fd::LocalAction::brake, "too close to steer a lattice path round, it brakes"+describe(late));
    // The corridor less the half width reaches 4.1 m left; the blockage, widened by the half width, reaches 2.9 m.
    near(gap.lateral_offset_m, (2.9+4.1)/2, 1e-9, "toward the centre of the gap left of the blockage");
    require(late.reason.find("gap") != std::string::npos && late.reason.find("left") != std::string::npos, "and says so: "+late.reason);
    // Recordings refuse separators in reasons.
    for (const auto* reason : {&stopping.reason, &late.reason, &brake.reason, &gap.reason})
        require(reason->find_first_of(",\n\r") == std::string::npos, "the reason is recordable: "+*reason);
}

// A new plan keeps the stretch of the previous plan for the same action that lies within a pursuit distance of the car,
// so it cannot keep deferring what the last plan began; without the previous plan it would start afresh.
void a_new_plan_keeps_the_committed_stretch() {
    const fd::Obstruction lane{62, 68, -4.5, 1.0, "cone-cluster"};
    const auto plan = fd::make_speed_plan(preset, config);
    const fd::VehicleModel kinematic = fd::KinematicBicycle{};
    const auto first = decide(20, 15, {lane});
    require(first.choice().action == fd::LocalAction::pass_left, "the first plan passes left"+describe(first));
    // Where the first plan's prediction puts the car a quarter second later.
    const auto& predicted = first.trajectory().points;
    const auto later = std::find_if(predicted.begin(), predicted.end(), [](const fd::TrajectorySample& p) { return p.state.time_s >= 0.25; });
    require(later != predicted.end(), "the prediction reaches a quarter second");
    const auto plant = fd::plant_state_from(later->state, kinematic, config);
    const auto second = fd::choose_lattice_action(preset, plan, plant, config, kinematic, &lattice, {lane}, 0, nullptr, nullptr, &first.choice());
    require(second.choice().action == fd::LocalAction::pass_left, "the second plan still passes left"+describe(second));
    const double here = second.choice().path.front().s_m;
    const double reach = config.lookahead_base_m+config.lookahead_time_s*later->state.speed_mps;
    const auto& old = first.choice().path;
    const double moved = std::fmod(here-old.front().s_m+preset.length_m, preset.length_m);
    std::size_t kept = 0;
    for (const auto& point : second.choice().path) {
        const double s = point.s_m-here;
        if (s <= 1e-6 || s > reach+6) continue;
        const auto match = std::find_if(old.begin(), old.end(), [&](const fd::StationOffset& o) { return std::abs(o.s_m-old.front().s_m-moved-s) < 1e-6; });
        if (s <= reach) require(match != old.end() && match->offset_m == point.offset_m, "a committed point is the previous plan's");
        if (match != old.end() && match->offset_m == point.offset_m) ++kept;
    }
    std::cout << "  a quarter second on: " << kept << " points of the previous plan kept within " << reach << " m\n";
    require(kept >= 1, "the committed stretch reaches a node of the previous plan");
}

// With the blockage behind and the reference line clear, a car well off the reference returns to it along a lattice
// path rather than turning straight back, and one near it simply follows it.
void a_car_off_the_reference_returns_along_the_lattice() {
    const fd::Obstruction lane{62, 68, -4.5, 1.0, "cone-cluster"};
    const auto plan = fd::make_speed_plan(preset, config);
    const fd::VehicleModel kinematic = fd::KinematicBicycle{};
    auto state = on_track(80, 18);
    const auto here = fd::sample(preset, 80), ahead = fd::sample(preset, 80.1);
    const double nx = -(ahead.y_m-here.y_m)/0.1, ny = (ahead.x_m-here.x_m)/0.1;
    state.x_m += 2.5*nx;
    state.y_m += 2.5*ny;
    const auto away = fd::choose_lattice_action(preset, plan, fd::plant_state_from(state, kinematic, config), config, kinematic, &lattice, {lane});
    const auto& home = away.choice();
    require(home.action == fd::LocalAction::straight && !home.path.empty() && home.clear && away.reason.find("Returning") != std::string::npos,
            "2.5 m off it returns along a lattice path"+describe(away));
    near(home.path.back().offset_m, 0, 1e-12, "which ends on the reference");
    require(away.options.size() == 2 && away.options[0].path.empty() && !away.options[0].speed_profile.empty() && std::isfinite(away.options[0].estimated_time_s),
            "rejoining the reference directly is weighed too, with its own profile"+describe(away));
    require(!away.options[0].clear || home.estimated_time_s <= away.options[0].estimated_time_s, "the return taken is the faster clear one");
    const auto near_it = decide(80, 18, {lane});
    require(near_it.choice().action == fd::LocalAction::straight && near_it.choice().path.empty() && near_it.options.size() == 1,
            "on the reference it simply follows it"+describe(near_it));
}

// A lattice must have been laid along the track it plans on.
void a_lattice_for_another_track_is_refused() {
    auto other = preset;
    for (auto& p : other.points) p.x_m += 5;
    const auto plan = fd::make_speed_plan(other, config);
    const fd::VehicleModel kinematic = fd::KinematicBicycle{};
    const auto message = refusal([&] {
        fd::choose_lattice_action(other, plan, fd::plant_state_from(on_track(30, 15), kinematic, config), config, kinematic, &lattice,
                                  {{62, 68, -4.5, 1.0, "cone-cluster"}});
    });
    require(message.find("lattice") != std::string::npos, "refused, naming the lattice: "+message);
    const auto missing = refusal([&] {
        fd::choose_lattice_action(preset, fd::make_speed_plan(preset, config), fd::plant_state_from(on_track(30, 15), kinematic, config), config,
                                  kinematic, nullptr, {{62, 68, -4.5, 1.0, "cone-cluster"}});
    });
    require(missing.find("lattice") != std::string::npos, "a stated blockage without a lattice is refused too: "+missing);
    require(fd::choose_lattice_action(preset, fd::make_speed_plan(preset, config), fd::plant_state_from(on_track(30, 15), kinematic, config), config,
                                      kinematic, nullptr, {}).choice().action == fd::LocalAction::straight,
            "without blockages no lattice is needed");
}

struct Drive {
    bool entered{};
    double passing_offset{};       // the car's offset furthest from the reference within the blockage's stations
    double furthest_station{};
    double final_speed{};
    double peak_grip{};            // largest combined grip use
    double offset_after{};         // largest |offset| from 60 m past the blockage to 200 m past it
    bool returned{};               // a decision returned to the reference along a lattice path
    std::size_t blocked_decisions{};
    double blocked_ms{};           // mean planning time of those decisions
    std::vector<double> blocked_times_ms;
    std::string actions;           // the chosen actions in order, without repeats
};

// Drives the kinematic bicycle from the preset's start with a stated blockage, under a planner, until it has come
// `until` metres or stopped, timing each decision made while the blockage blocked the reference.
Drive drive(fd::LocalPlannerMode planner, const fd::Obstruction& blockage, double until) {
    fd::Simulation simulation(preset, config, fd::KinematicBicycle{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions, planner);
    simulation.set_obstructions({blockage});
    Drive run;
    std::uint64_t decisions = simulation.decision_count();
    std::string last_action;
    while (simulation.diagnostics().progress_m < until && simulation.state().time_s < 60) {
        if (simulation.state().time_s > 3 && simulation.state().speed_mps < 0.02) break;
        const auto began = std::chrono::steady_clock::now();
        simulation.step();
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-began).count();
        if (simulation.decision_count() != decisions) {
            decisions = simulation.decision_count();
            const auto& decision = simulation.decision();
            if (decision.options.size() > 1) { ++run.blocked_decisions; run.blocked_ms += ms; run.blocked_times_ms.push_back(ms); }
            if (decision.choice().action == fd::LocalAction::straight && !decision.choice().path.empty()) run.returned = true;
            const std::string name(fd::local_action_name(decision.choice().action));
            if (name != last_action) run.actions += (run.actions.empty() ? "" : ", ")+name;
            last_action = name;
        }
        const auto& state = simulation.state();
        const auto at = fd::project(preset, {state.x_m, state.y_m});
        run.furthest_station = std::max(run.furthest_station, at.s_m);
        run.peak_grip = std::max(run.peak_grip, simulation.diagnostics().combined_grip_utilization);
        if (at.s_m > blockage.to_s_m+60 && at.s_m < blockage.to_s_m+200) run.offset_after = std::max(run.offset_after, std::abs(at.signed_error_m));
        if (at.s_m < blockage.from_s_m || at.s_m > blockage.to_s_m) continue;
        if (std::abs(at.signed_error_m) > std::abs(run.passing_offset)) run.passing_offset = at.signed_error_m;
        if (at.signed_error_m >= blockage.from_offset_m && at.signed_error_m <= blockage.to_offset_m) run.entered = true;
    }
    run.final_speed = simulation.state().speed_mps;
    if (run.blocked_decisions > 0) run.blocked_ms /= static_cast<double>(run.blocked_decisions);
    std::sort(run.blocked_times_ms.begin(), run.blocked_times_ms.end());
    return run;
}

// The plan's closed-loop tests: with the lattice planner the car still avoids a blockage on its right and passes left
// of it, then returns to the reference along the lattice, within its ordinary tracking error from 60 m past the
// blockage; it still stops before a blockage across the whole corridor.
void the_car_still_avoids_and_stops() {
    const fd::Obstruction lane{62, 68, -4.5, 1.0, "cone-cluster"};
    const auto avoiding = drive(fd::LocalPlannerMode::lattice, lane, 300);
    std::cout << "  right side blocked: passes at " << avoiding.passing_offset << " m, back within " << avoiding.offset_after
              << " m of the reference from 60 m past it; actions: " << avoiding.actions << "\n";
    require(!avoiding.entered && avoiding.passing_offset > lane.to_offset_m, "the car passes left of the blockage without entering it");
    require(avoiding.furthest_station > 250, "and drives on past it");
    require(avoiding.actions.find("pass left") != std::string::npos, "by passing left along the lattice: "+avoiding.actions);
    require(avoiding.offset_after < 0.3, "the chosen path converges back to the reference once the blockage is behind");
    require(avoiding.returned, "returning along the lattice");

    const fd::Obstruction wall{62, 68, -4.5, 4.5, "stalled-car"};
    const auto stopping = drive(fd::LocalPlannerMode::lattice, wall, 200);
    std::cout << "  full width blocked: stopped at " << stopping.furthest_station << " m at " << stopping.final_speed << " m/s; actions: "
              << stopping.actions << "\n";
    require(!stopping.entered && stopping.furthest_station < wall.from_s_m && stopping.furthest_station > 50, "the car halts before the blocked stations");
    near(stopping.final_speed, 0, 0.35, "and comes to rest");
    require(stopping.actions.find("brake") != std::string::npos, "by braking: "+stopping.actions);
}

// Losing grip on the way home leaves no clear way home at all: every prediction, along the path the car is on and along
// the reference, exceeds the envelope. The car then keeps the path it is on rather than turning back to the reference,
// which would ask for the sharper turn, and says so. Grip drops to 0.45 while the car is out beside the lane blockage.
void no_clear_way_home_keeps_the_path_the_car_is_on() {
    fd::Simulation simulation(preset, config, fd::KinematicBicycle{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions,
                              fd::LocalPlannerMode::lattice);
    simulation.set_obstructions({{62, 68, -4.5, 1.0, "cone-cluster"}});
    auto seen = simulation.decision_count();
    bool dropped = false, was_on_a_path = false;
    std::size_t nothing_clear = 0, turned_back = 0, said_so = 0;
    double furthest = 0;
    while (simulation.state().time_s < 12) {
        if (!dropped && simulation.state().time_s+1e-10 >= 6.0) { simulation.set_grip(0.45); dropped = true; }
        simulation.step();
        const auto& state = simulation.state();
        furthest = std::max(furthest, std::abs(fd::project(preset, {state.x_m, state.y_m}).signed_error_m));
        if (simulation.decision_count() == seen) continue;
        seen = simulation.decision_count();
        const auto& decision = simulation.decision();
        const bool clear = std::any_of(decision.options.begin(), decision.options.end(), [](const fd::LocalOption& o) { return o.clear; });
        if (!clear && decision.options.size() > 1 && !decision.holding) {
            ++nothing_clear;
            if (was_on_a_path && decision.choice().path.empty()) ++turned_back;
            if (decision.reason.find("no way home is clear") != std::string::npos) ++said_so;
        }
        was_on_a_path = !decision.choice().path.empty();
    }
    std::cout << "  grip lost on the way home: " << nothing_clear << " decisions with nothing clear, " << turned_back
              << " turned back to the reference; furthest " << furthest << " m off\n";
    require(nothing_clear > 20, "the dropped grip leaves the car with no clear way home");
    require(turned_back == 0, "and it keeps the path it is on");
    require(said_so == nothing_clear, "and every such decision says no way home is clear");
    require(furthest < fd::vehicle_half_width_m+3.0, "while staying within the corridor");
}

// A pass is finished rather than abandoned: once the car is far enough across, the reference line no longer touches the
// blockage, and the decision becomes a return. The return continues the path the car is on, so it does not turn back
// sharply while still beside the blockage. The four-wheel car planning from its envelope shows it: at the corner
// blockage it once left the corridor and slid when the return could not continue its pass.
void a_pass_is_finished_before_returning() {
    const fd::Obstruction corner{130, 136, -0.5, 1.5, "corner-bollard"};
    const fd::VehicleModel car = fd::FourWheelCar{};
    const fd::PerformanceEnvelope envelope(car, config);
    fd::Simulation simulation(preset, config, car, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::performance_envelope,
                              fd::LocalPlannerMode::lattice);
    simulation.set_obstructions({corner});
    double furthest = 0, worst_sideslip = 0, back_to = 1e9;
    std::size_t outside = 0;
    bool passed = false;
    while (simulation.state().time_s < 14) {
        simulation.step();
        const auto& state = simulation.state();
        const auto at = fd::project(preset, {state.x_m, state.y_m});
        if (!fd::within_corridor(preset, at, fd::vehicle_half_width_m)) ++outside;
        furthest = std::max(furthest, std::abs(at.signed_error_m));
        worst_sideslip = std::max(worst_sideslip, std::abs(simulation.diagnostics().rear_sideslip_rad));
        if (at.s_m > corner.to_s_m) passed = true;
        if (passed && at.s_m > corner.to_s_m+40) back_to = std::min(back_to, std::abs(at.signed_error_m));
    }
    std::cout << "  four-wheel car at the corner blockage: furthest " << furthest << " m off, " << outside
              << " samples outside the corridor, worst rear sideslip " << worst_sideslip << " rad\n";
    require(outside == 0, "the car stays inside the corridor");
    require(furthest < 4.0, "without swinging wide");
    require(worst_sideslip < 0.2, "and without sliding");
    require(back_to < 1.0, "and comes back to the reference once the blockage is behind");
}

// The plan's comparison: on the avoidance scenario the lattice's smoother lane change uses less of the car's grip than
// the five-offset planner's, and each blocked decision's planning time is measured against the 20 ms control period.
void the_lattice_uses_less_grip_to_pass() {
    const fd::Obstruction lane{62, 68, -4.5, 1.0, "cone-cluster"};
    const auto offsets = drive(fd::LocalPlannerMode::five_offsets, lane, 120);
    const auto lattice_run = drive(fd::LocalPlannerMode::lattice, lane, 120);
    const auto spread = [](const Drive& run) {
        const auto& times = run.blocked_times_ms;
        const auto at = [&](double share) { return times[std::min(times.size()-1, static_cast<std::size_t>(share*static_cast<double>(times.size())))]; };
        return std::to_string(run.blocked_decisions)+" blocked decisions, mean "+std::to_string(run.blocked_ms)+" ms, median "+std::to_string(at(0.5))+
               ", 95th percentile "+std::to_string(at(0.95))+", longest "+std::to_string(times.back());
    };
    std::cout << "  five offsets: passes at " << offsets.passing_offset << " m, peak grip use " << offsets.peak_grip << "; " << spread(offsets) << "\n"
              << "  lattice: passes at " << lattice_run.passing_offset << " m, peak grip use " << lattice_run.peak_grip << "; " << spread(lattice_run)
              << " (measurement only)\n";
    require(!offsets.entered && !lattice_run.entered, "both pass the blockage");
    require(lattice_run.peak_grip < offsets.peak_grip, "the lattice planner's pass uses less grip");
    require(lattice_run.blocked_decisions > 0 && lattice_run.blocked_ms < 20, "a blocked lattice decision takes less than the control period here");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"the_pursuit_target_takes_the_paths_offset_at_its_station", the_pursuit_target_takes_the_paths_offset_at_its_station},
        {"a_path_crosses_the_seam", a_path_crosses_the_seam},
        {"a_clear_reference_line_is_followed_with_one_prediction", a_clear_reference_line_is_followed_with_one_prediction},
        {"a_blocked_reference_is_passed_on_the_clear_side", a_blocked_reference_is_passed_on_the_clear_side},
        {"the_faster_side_is_passed_when_both_are_clear", the_faster_side_is_passed_when_both_are_clear},
        {"a_near_tie_keeps_the_previous_choice", a_near_tie_keeps_the_previous_choice},
        {"each_action_is_predicted_along_its_own_speed_profile", each_action_is_predicted_along_its_own_speed_profile},
        {"the_estimate_is_what_the_car_drives", the_estimate_is_what_the_car_drives},
        {"a_blocked_corridor_brakes_toward_the_widest_gap", a_blocked_corridor_brakes_toward_the_widest_gap},
        {"a_new_plan_keeps_the_committed_stretch", a_new_plan_keeps_the_committed_stretch},
        {"a_car_off_the_reference_returns_along_the_lattice", a_car_off_the_reference_returns_along_the_lattice},
        {"a_lattice_for_another_track_is_refused", a_lattice_for_another_track_is_refused},
        {"no_clear_way_home_keeps_the_path_the_car_is_on", no_clear_way_home_keeps_the_path_the_car_is_on},
        {"a_pass_is_finished_before_returning", a_pass_is_finished_before_returning},
        {"the_car_still_avoids_and_stops", the_car_still_avoids_and_stops},
        {"the_lattice_uses_less_grip_to_pass", the_lattice_uses_less_grip_to_pass},
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cout << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-failures << '/' << tests.size() << " lattice planning groups passed\n";
    return failures == 0 ? 0 : 1;
}

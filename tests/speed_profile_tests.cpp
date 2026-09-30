#include "fd/performance_envelope.hpp"
#include "fd/speed_profile.hpp"
#include "fd/vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Phase 5.3 (decision 0023): a speed profile along a path from the car. The oracles are constant-acceleration
// kinematics, the curvature of a circle beside the reference, and the reference plan, which uses the same limits.
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
const double lateral = config.lateral_grip_fraction*config.grip_mu*fd::gravity_mps2;
const double longitudinal = config.longitudinal_grip_fraction*config.grip_mu*fd::gravity_mps2;

fd::ProfileRequest along(double station, double speed, double horizon, double end_speed, std::span<const fd::StationOffset> path = {}) {
    fd::ProfileRequest request;
    request.start_s_m = station;
    request.start_speed_mps = speed;
    request.horizon_m = horizon;
    request.end_speed_mps = end_speed;
    request.path = path;
    return request;
}

// The preset's first corner: a left arc of 18 m radius after a straight, and the station of its first sample.
double first_arc_station() {
    for (const auto& p : preset.points) if (p.curvature > 0) return p.s_m;
    throw std::runtime_error("the preset has a left corner");
}

// On the opening straight at the top speed, a profile along the reference holds that speed: 60 m in 3 s, with samples
// every half metre and the last exactly at the horizon.
void on_a_straight_at_top_speed_the_profile_holds_it() {
    const auto profile = fd::make_speed_profile(preset, config, along(20, 20, 60, 20));
    require(profile.points.size() == 121 && profile.within_limits, "a sample every half metre, within limits");
    near(profile.points.front().s_m, 20, 0, "it starts at the car's station");
    near(profile.points.back().s_m, 80, 1e-12, "and ends at the horizon");
    near(profile.points.back().distance_m, 60, 1e-9, "along the reference the distance is the station difference");
    for (const auto& p : profile.points) near(p.speed_mps, 20, 1e-12, "the speed holds");
    near(profile.estimated_time_s, 3, 1e-9, "60 m at 20 m/s is 3 s");
    near(fd::profile_time(profile.points), profile.estimated_time_s, 0, "the estimate is the points' own time");
}

// From rest the profile accelerates at the longitudinal grip fraction: speed squared grows by twice that per metre, and
// covering 30 m takes the square root of 60 m over it.
void from_rest_it_accelerates_within_the_grip_fraction() {
    const auto profile = fd::make_speed_profile(preset, config, along(10, 0, 30, 20));
    for (const auto& p : profile.points) near(p.speed_mps, std::sqrt(2*longitudinal*p.distance_m), 1e-9, "speed from rest at constant acceleration");
    near(profile.estimated_time_s, std::sqrt(2*30/longitudinal), 1e-9, "the time of constant acceleration");
    const std::vector<fd::ProfilePoint> standing{{0, 0, 0, 0}, {0.5, 0.5, 0, 0}};
    require(std::isinf(fd::profile_time(standing)), "a profile that does not move never arrives");
}

// Inside the corner the speed is capped by the path's own curvature: on the reference sqrt(lateral * 18), and on a path
// held 2 m inside or outside it, the curvature of a circle of 16 or 20 m, whose length is shorter or longer by the same
// ratio.
void a_curve_caps_speed_by_its_own_curvature() {
    const double arc = first_arc_station();
    const double start = arc+5;
    const double radius = 1/preset.points[static_cast<std::size_t>(std::find_if(preset.points.begin(), preset.points.end(),
                                                                                 [](const fd::PathPoint& p) { return p.curvature > 0; })-preset.points.begin())].curvature;
    near(radius, 18, 1e-9, "the preset's first corner has an 18 m radius");
    for (const double offset : {0.0, 2.0, -2.0}) {
        const std::vector<fd::StationOffset> held{{start, offset}, {start+5, offset}};
        const double r = radius-offset;
        const double cap = std::sqrt(lateral*r);
        const auto profile = fd::make_speed_profile(preset, config, along(start, cap, 15, 20, offset == 0 ? std::span<const fd::StationOffset>{} : held));
        for (const auto& p : profile.points) {
            near(p.curvature_1pm, 1/r, 1e-9, "the path's curvature is that of its circle, offset "+std::to_string(offset));
            near(p.speed_mps, cap, 1e-9, "the speed is capped by it");
        }
        near(profile.points.back().distance_m, 15*r/radius, 1e-9, "the path's length scales with its radius");
    }
}

// Approaching the corner at the top speed, the profile brakes at the longitudinal grip fraction so that it reaches the
// corner's first sample at the corner's cap: before it, speed squared is the cap's plus twice the braking per metre left.
void braking_reaches_the_curve_on_time() {
    const double arc = first_arc_station();
    const double cap = std::sqrt(lateral*18);
    const auto profile = fd::make_speed_profile(preset, config, along(60, 20, 100, 20));
    const auto first = std::find_if(profile.points.begin(), profile.points.end(), [&](const fd::ProfilePoint& p) { return p.s_m >= arc; });
    require(first != profile.points.end(), "the profile reaches the corner");
    std::size_t braking = 0;
    for (auto p = profile.points.begin(); p <= first; ++p) {
        const double bound = std::min(20.0, std::sqrt(cap*cap+2*longitudinal*(first->distance_m-p->distance_m)));
        near(p->speed_mps, bound, 1e-9, "the speed at "+std::to_string(p->s_m)+" m");
        if (bound < 20) ++braking;
    }
    require(braking > 40, "the profile brakes over the last stretch of the straight");
}

// The profile ends no faster than asked: 10 m/s at the end of a straight, braking into it from the top speed.
void the_profile_ends_no_faster_than_asked() {
    const auto profile = fd::make_speed_profile(preset, config, along(20, 20, 60, 10));
    for (const auto& p : profile.points)
        near(p.speed_mps, std::min(20.0, std::sqrt(100+2*longitudinal*(60-p.distance_m))), 1e-9, "braking to the end speed");
}

// A car already faster than the path allows brakes at the limit until it is within, and the profile says so.
void a_start_beyond_the_paths_limit_brakes_at_the_limit() {
    const double start = first_arc_station()+2;
    const double cap = std::sqrt(lateral*18);
    const auto profile = fd::make_speed_profile(preset, config, along(start, 15, 15, 20));
    require(!profile.within_limits, "the profile says it starts beyond the path's limits");
    double previous = 15;
    for (std::size_t k = 1; k < profile.points.size(); ++k) {
        const auto& p = profile.points[k];
        const double braked = std::sqrt(std::max(0.0, previous*previous-2*longitudinal*(p.distance_m-profile.points[k-1].distance_m)));
        near(p.speed_mps, std::max(cap, braked), 1e-9, "braking at the limit until within the cap");
        previous = p.speed_mps;
    }
    near(profile.points.back().speed_mps, cap, 1e-9, "and then holding the cap");
    require(fd::make_speed_profile(preset, config, along(start, cap, 15, 20)).within_limits, "a start at the cap is within limits");
}

// A path that steps 2 m left over 10 m on the straight turns at the middles of its segments: its slope changes by 0.2
// over 10 m there, so its offset's second derivative is 0.02 per metre and its curvature that over (1 + slope^2)^1.5.
// That turn caps the speed below the top speed, and the path is longer than its stations by its slope. From 17 m/s the
// profile slows to the turn's cap in time; from the top speed 5 m before the turn it cannot, and says so.
void a_turn_between_segments_curves_as_the_search_costs_it() {
    const std::vector<fd::StationOffset> step{{20, 0}, {30, 0}, {40, 2}, {50, 2}};
    const auto profile = fd::make_speed_profile(preset, config, along(20, 17, 50, 20, step));
    require(profile.within_limits, "from 17 m/s the turn is within limits");
    require(!fd::make_speed_profile(preset, config, along(20, 20, 50, 20, step)).within_limits,
            "from 20 m/s, 5 m before a turn capped near 17.85 m/s, it is not");
    const auto at = [&](double s) {
        const auto p = std::min_element(profile.points.begin(), profile.points.end(),
                                        [&](const fd::ProfilePoint& a, const fd::ProfilePoint& b) { return std::abs(a.s_m-s) < std::abs(b.s_m-s); });
        near(p->s_m, s, 1e-9, "a sample lies at "+std::to_string(s)+" m");
        return *p;
    };
    near(at(20).curvature_1pm, 0, 1e-12, "before the first segment's middle nothing turns");
    near(at(30).curvature_1pm, 0.02/std::pow(1.01, 1.5), 1e-12, "turning left toward the ramp's slope");
    near(at(40).curvature_1pm, -0.02/std::pow(1.01, 1.5), 1e-12, "turning back right after the ramp's middle, 2 m left");
    near(at(46).curvature_1pm, 0, 1e-12, "parallel again beyond the last segment's middle");
    double expected = 50, slope_length = 0;
    const int steps = 200000;
    for (int k = 0; k < steps; ++k) {
        const double s = 25+20*(k+0.5)/steps;
        const double slope = s < 35 ? 0.02*(s-25) : 0.2-0.02*(s-35);
        slope_length += std::sqrt(1+slope*slope)*20/steps;
    }
    expected += slope_length-20;
    near(profile.points.back().distance_m, expected, 2e-4, "the path is longer than its stations by its slope");
    const double sharpest = std::max_element(profile.points.begin(), profile.points.end(), [](const fd::ProfilePoint& a, const fd::ProfilePoint& b) {
        return std::abs(a.curvature_1pm) < std::abs(b.curvature_1pm); })->curvature_1pm;
    const auto turn = std::find_if(profile.points.begin(), profile.points.end(), [](const fd::ProfilePoint& p) { return p.s_m >= 25; });
    const double slowest = std::min_element(turn, profile.points.end(), [](const fd::ProfilePoint& a, const fd::ProfilePoint& b) {
        return a.speed_mps < b.speed_mps; })->speed_mps;
    near(slowest, std::sqrt(lateral/std::abs(sharpest)), 1e-9, "the turn caps the speed");
    require(slowest < 20, "below the top speed");
}

// Along the reference itself a profile uses the reference plan's limits, so wherever the plan is limited by curvature
// inside the corner the profile's speed is the plan's, with the grip fractions and with the four-wheel car's envelope.
void on_the_reference_it_reproduces_the_plans_curvature_limits() {
    const fd::VehicleModel four_wheel = fd::FourWheelCar{};
    const fd::PerformanceEnvelope envelope(four_wheel, config);
    for (const fd::PerformanceEnvelope* with : {static_cast<const fd::PerformanceEnvelope*>(nullptr), &envelope}) {
        const auto plan = fd::make_speed_plan(preset, config, with);
        const double arc = first_arc_station();
        const auto first = static_cast<std::size_t>(std::find_if(preset.points.begin(), preset.points.end(),
                                                                 [](const fd::PathPoint& p) { return p.curvature > 0; })-preset.points.begin());
        const auto profile = fd::make_speed_profile(preset, config, along(preset.points[first-67].s_m, plan[first-67].speed_mps, 60, 20), with);
        std::size_t compared = 0;
        for (const auto& p : profile.points) {
            const auto sample = static_cast<std::size_t>(std::upper_bound(preset.points.begin(), preset.points.end(), p.s_m,
                [](double value, const fd::PathPoint& q) { return value < q.s_m; })-preset.points.begin()-1);
            // Well inside the corner, both samples around the point are limited by the same curvature.
            if (p.s_m < arc+3 || preset.points[sample].curvature <= 0 || preset.points[sample+1].curvature <= 0) continue;
            if (plan[sample].reason != "curvature_limit" || plan[sample+1].reason != "curvature_limit") continue;
            near(p.speed_mps, plan[sample].speed_mps, 1e-6, std::string(with ? "envelope" : "grip fraction")+" plan's curvature limit at "+std::to_string(p.s_m)+" m");
            ++compared;
        }
        require(compared > 20, "the profile is compared inside the corner");
        std::cout << "  " << (with ? "envelope" : "grip fractions") << ": " << compared << " points match the plan's curvature limit, "
                  << profile.estimated_time_s << " s over 60 m\n";
    }
}

// The controller follows a profile while the car is within it: the target speed is interpolated in speed squared and
// the feedforward is the span's acceleration; beyond the profile the reference plan governs again.
void the_controller_follows_a_profile() {
    const auto plan = fd::make_speed_plan(preset, config);
    const auto profile = fd::make_speed_profile(preset, config, along(60, 20, 100, 11));
    const auto here = fd::sample(preset, 120.2), ahead = fd::sample(preset, 120.3);
    const fd::State state{here.x_m, here.y_m, std::atan2(ahead.y_m-here.y_m, ahead.x_m-here.x_m), 15, 0, 0};
    fd::ControlIntent intent;
    intent.speed_profile = profile.points;
    const auto result = fd::compute_control(preset, plan, state, config, intent);
    const double s = fd::project(preset, {state.x_m, state.y_m}).s_m;
    const auto upper = std::upper_bound(profile.points.begin(), profile.points.end(), s, [](double value, const fd::ProfilePoint& p) { return value < p.s_m; });
    const auto& a = *(upper-1);
    const auto& b = *upper;
    const double f = (s-a.s_m)/(b.s_m-a.s_m);
    const double target = std::sqrt((1-f)*a.speed_mps*a.speed_mps+f*b.speed_mps*b.speed_mps);
    const double feedforward = (b.speed_mps*b.speed_mps-a.speed_mps*a.speed_mps)/(2*(b.distance_m-a.distance_m));
    near(result.target_speed_mps, target, 1e-12, "the target is the profile's speed at the car's station");
    near(result.requested.acceleration_mps2, feedforward+config.speed_gain*(target-15), 1e-12, "the feedforward is the profile's acceleration");
    require(feedforward < 0 && target < 20, "here the profile brakes for the corner");

    const auto beyond = fd::sample(preset, 300), onward = fd::sample(preset, 300.1);
    const fd::State later{beyond.x_m, beyond.y_m, std::atan2(onward.y_m-beyond.y_m, onward.x_m-beyond.x_m), 15, 0, 0};
    const auto plain = fd::compute_control(preset, plan, later, config);
    const auto past = fd::compute_control(preset, plan, later, config, intent);
    require(past.target_speed_mps == plain.target_speed_mps && past.requested.acceleration_mps2 == plain.requested.acceleration_mps2,
            "beyond the profile the reference plan governs, command for command");

    const auto before = fd::sample(preset, 40), onto = fd::sample(preset, 40.1);
    const fd::State earlier{before.x_m, before.y_m, std::atan2(onto.y_m-before.y_m, onto.x_m-before.x_m), 15, 0, 0};
    const auto early = fd::compute_control(preset, plan, earlier, config, intent);
    near(early.target_speed_mps, 20, 0, "before the profile its first speed is the target");
    near(early.requested.acceleration_mps2, config.speed_gain*(20-15), 1e-12, "without feedforward");

    std::vector<fd::ProfilePoint> broken(profile.points.begin(), profile.points.begin()+3);
    broken[2].s_m = broken[0].s_m;
    intent.speed_profile = broken;
    require(refusal([&] { fd::compute_control(preset, plan, state, config, intent); }).find("profile") != std::string::npos,
            "a profile whose stations do not increase is refused");
    broken[2] = profile.points[2];
    broken[1].speed_mps = -1;
    require(refusal([&] { fd::compute_control(preset, plan, state, config, intent); }).find("profile") != std::string::npos,
            "a negative profile speed is refused");
}

void bad_requests_are_refused() {
    const auto refused = [&](const fd::ProfileRequest& request, const std::string& what) {
        require(!refusal([&] { fd::make_speed_profile(preset, config, request); }).empty(), what+" is refused");
    };
    refused(along(20, -1, 60, 20), "a negative start speed");
    refused(along(20, std::numeric_limits<double>::quiet_NaN(), 60, 20), "a start speed that is not a number");
    refused(along(20, 10, 0, 20), "a horizon of nothing");
    refused(along(20, 10, 60, -2), "a negative end speed");
    const std::vector<fd::StationOffset> elsewhere{{25, 0}, {30, 1}};
    refused(along(20, 10, 60, 20, elsewhere), "a path that does not start at the car");
    const std::vector<fd::StationOffset> backwards{{20, 0}, {30, 1}, {25, 1}};
    refused(along(20, 10, 60, 20, backwards), "a path whose stations do not increase");
    auto slope = along(20, 10, 60, 20);
    slope.start_slope = std::numeric_limits<double>::infinity();
    refused(slope, "a start slope that is not finite");
    fd::Config other = config;
    other.grip_mu = 0.7;
    const fd::VehicleModel four_wheel = fd::FourWheelCar{};
    const fd::PerformanceEnvelope envelope(four_wheel, other);
    require(!refusal([&] { fd::make_speed_profile(preset, config, along(20, 10, 60, 20), &envelope); }).empty(),
            "an envelope derived under another configuration is refused");
}
}  // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"on_a_straight_at_top_speed_the_profile_holds_it", on_a_straight_at_top_speed_the_profile_holds_it},
        {"from_rest_it_accelerates_within_the_grip_fraction", from_rest_it_accelerates_within_the_grip_fraction},
        {"a_curve_caps_speed_by_its_own_curvature", a_curve_caps_speed_by_its_own_curvature},
        {"braking_reaches_the_curve_on_time", braking_reaches_the_curve_on_time},
        {"the_profile_ends_no_faster_than_asked", the_profile_ends_no_faster_than_asked},
        {"a_start_beyond_the_paths_limit_brakes_at_the_limit", a_start_beyond_the_paths_limit_brakes_at_the_limit},
        {"a_turn_between_segments_curves_as_the_search_costs_it", a_turn_between_segments_curves_as_the_search_costs_it},
        {"on_the_reference_it_reproduces_the_plans_curvature_limits", on_the_reference_it_reproduces_the_plans_curvature_limits},
        {"the_controller_follows_a_profile", the_controller_follows_a_profile},
        {"bad_requests_are_refused", bad_requests_are_refused},
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cout << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-failures << '/' << tests.size() << " speed profile groups passed\n";
    return failures == 0 ? 0 : 1;
}

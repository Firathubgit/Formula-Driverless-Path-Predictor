#include "fd/performance_envelope.hpp"
#include "fd/lattice.hpp"
#include "fd/raceline.hpp"
#include "fd/simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Phase 3.4 (decision 0020): the minimum-curvature racing line. The plan's tests: it stays inside the corridor; its
// curvature is within the steering limit; its estimated lap with Phase 3.2's envelope plan is shorter than the
// centreline's for the same car; a straight stays straight. A circle, whose answer is known, comes first.
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
const double pi = std::numbers::pi;
const fd::Config config;

std::vector<fd::Vec2> circle(double radius, double spacing) {
    std::vector<fd::Vec2> points;
    const auto count = static_cast<int>(std::ceil(2*pi*radius/spacing));
    for (int i = 0; i < count; ++i) {
        const double a = 2*pi*i/count;
        points.push_back({radius*std::sin(a), radius*(1-std::cos(a))});
    }
    return points;
}
fd::ConditionedTrack clean(const std::vector<fd::Vec2>& points, double width, const std::string& name) {
    fd::ConditioningOptions tight;
    tight.smoothing_rms_m = 0.002;
    return fd::condition_track(points, width, name, tight);
}

// On a circle every sample asks the same, and the least curvature is the widest circle the corridor allows.
void a_circle_moves_to_its_outer_edge() {
    const auto centreline = clean(circle(40, 1.0), 12, "circle");
    const auto began = std::chrono::steady_clock::now();
    const auto line = fd::make_minimum_curvature_line(centreline, config);
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
    const double expected = 40+6-(0.9+0.6);
    double worst_radius = 0, worst_curvature = 0;
    for (const auto& p : line.track.points) {
        worst_radius = std::max(worst_radius, std::abs(std::hypot(p.x_m, p.y_m-40)-expected));
        worst_curvature = std::max(worst_curvature, std::abs(p.curvature-1/expected));
    }
    std::cout << "  40 m circle, 12 m wide: " << line.iterations << " iterations, last shift " << line.last_shift_m << " m, radius error "
              << worst_radius << " m, curvature error " << worst_curvature << " 1/m, " << seconds << " s (measurement only)\n";
    require(line.converged, "the iteration converges");
    require(worst_radius < 0.05 && worst_curvature < 1e-4, "the line is the circle at the outer edge less the vehicle and margin");
    require(line.squared_curvature < line.centreline_squared_curvature, "with less squared curvature than the centreline");
}

// Foundry Circuit's centreline, conditioned from the preset sampled every metre.
const fd::ConditionedTrack& foundry() {
    static const fd::ConditionedTrack conditioned = fd::condition_track(fd::make_preset_track());
    return conditioned;
}
const fd::RacingLine& foundry_line() {
    static const fd::RacingLine line = [] {
        const auto began = std::chrono::steady_clock::now();
        auto result = fd::make_minimum_curvature_line(foundry(), config);
        std::cout << "  Foundry Circuit racing line: " << result.iterations << " iterations, last shift " << result.last_shift_m << " m, "
                  << std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count() << " s (measurement only)\n";
        return result;
    }();
    return line;
}

// The plan's test: the line stays inside the corridor, less the vehicle's half width and the margin, and carries its
// distance to each edge.
void stays_inside_the_corridor() {
    const auto& centreline = foundry();
    const auto& line = foundry_line();
    require(line.converged, "the iteration converges on Foundry Circuit");
    fd::validate_track(line.track);
    const double usable = centreline.track.width_m/2-(0.9+0.6);
    double widest = 0, closest_edge = 1e9;
    for (std::size_t i = 0; i < line.track.points.size(); ++i) {
        const auto& p = line.track.points[i];
        const auto projection = fd::project(centreline.track, {p.x_m, p.y_m});
        widest = std::max(widest, std::abs(projection.signed_error_m));
        require(std::abs(projection.signed_error_m) <= usable+0.02, "every sample within the usable corridor, at "+std::to_string(p.s_m)+" m");
        near(line.offset_m[i], projection.signed_error_m, 1e-9, "the recorded offset is the distance from the centreline");
        near(line.track.left_edge_m[i]+line.track.right_edge_m[i], centreline.track.width_m, 1e-9, "the two edges span the corridor");
        closest_edge = std::min({closest_edge, line.track.left_edge_m[i], line.track.right_edge_m[i]});
    }
    std::cout << "  widest offset " << widest << " m of " << usable << " usable; closest edge " << closest_edge << " m\n";
    require(widest > usable-0.05, "the line uses the corridor's width somewhere");
    require(closest_edge >= 0.9+0.6-0.02, "the vehicle and margin stay inside every edge");
}

// The plan's test: the line's curvature stays within the steering limit, and a corridor within which no line can meet
// it is refused.
void curvature_within_the_steering_limit() {
    const auto& line = foundry_line();
    const double limit = std::tan(config.max_steering_rad)/config.wheelbase_m;
    double peak = 0, centreline_peak = 0;
    for (const auto& p : line.track.points) peak = std::max(peak, std::abs(p.curvature));
    for (const auto& p : foundry().track.points) centreline_peak = std::max(centreline_peak, std::abs(p.curvature));
    std::cout << "  peak curvature " << peak << " 1/m on the line, " << centreline_peak << " on the centreline, limit " << limit << "\n";
    require(peak <= limit && peak < centreline_peak, "the line is within the steering limit and bends less than the centreline");
    auto stiff = config;
    stiff.wheelbase_m = 4;
    stiff.max_steering_rad = 0.1;  // a turning radius of 39.9 m, the least steering the configuration allows
    const auto message = refusal([&] { fd::make_minimum_curvature_line(clean(circle(20, 1.0), 12, "circle"), stiff); });
    std::cout << "  20 m circle, 12 m wide, 39.9 m turning radius: " << message << "\n";
    require(message.find("steering limit") != std::string::npos, "a corridor no line can steer round is refused");
}

double lap_estimate(const fd::Track& track, const std::vector<fd::PlanPoint>& plan) { return fd::estimated_lap_time(track, plan); }

// The plan's test: with Phase 3.2's envelope plan the same car laps the racing line faster than the centreline; so it
// does with the grip fractions.
void laps_faster_than_the_centreline() {
    const fd::PerformanceEnvelope envelope(fd::FourWheelCar{}, config);
    const auto& centreline = foundry().track;
    const auto& line = foundry_line().track;
    const double centre_envelope = lap_estimate(centreline, fd::make_speed_plan(centreline, config, &envelope));
    const double line_envelope = lap_estimate(line, fd::make_speed_plan(line, config, &envelope));
    const double centre_fractions = lap_estimate(centreline, fd::make_speed_plan(centreline, config));
    const double line_fractions = lap_estimate(line, fd::make_speed_plan(line, config));
    std::cout << "  estimated lap, envelope plan: centreline " << centre_envelope << " s, racing line " << line_envelope << " s; grip fractions: "
              << centre_fractions << " s against " << line_fractions << " s; lengths " << centreline.length_m << " and " << line.length_m << " m\n";
    require(line_envelope < centre_envelope-0.5, "the racing line laps faster under the envelope plan");
    require(line_fractions < centre_fractions-0.5, "and under the grip fractions");
    std::cout << "  squared curvature over the lap: centreline " << foundry_line().centreline_squared_curvature << " 1/m, racing line "
              << foundry_line().squared_curvature << " 1/m\n";
    require(foundry_line().squared_curvature < 0.6*foundry_line().centreline_squared_curvature, "it has at least 40% less squared curvature than the centreline");
}

// The plan's test: a straight stays straight. On a stadium the line swings wide into and out of each end, and along the
// middle of each straight it is a straight line.
void a_straight_stays_straight() {
    std::vector<fd::Vec2> points;
    const double straight = 150, radius = 25;
    const auto arc_count = static_cast<int>(std::ceil(pi*radius));
    for (int i = 0; i < 150; ++i) points.push_back({static_cast<double>(i), 0});
    for (int i = 0; i < arc_count; ++i) { const double a = pi*i/arc_count; points.push_back({straight+radius*std::sin(a), radius*(1-std::cos(a))}); }
    for (int i = 0; i < 150; ++i) points.push_back({straight-i, 2*radius});
    for (int i = 0; i < arc_count; ++i) { const double a = pi*i/arc_count; points.push_back({-radius*std::sin(a), radius*(1+std::cos(a))}); }
    const auto line = fd::make_minimum_curvature_line(clean(points, 10, "stadium"), config);
    double worst_curvature = 0, worst_straightness = 0;
    int samples = 0;
    for (const double y : {0.0, 2*radius}) {
        std::vector<fd::Vec2> middle;
        for (const auto& p : line.track.points)
            if (p.x_m > 50 && p.x_m < 100 && std::abs(p.y_m-y) < 10) { middle.push_back({p.x_m, p.y_m}); worst_curvature = std::max(worst_curvature, std::abs(p.curvature)); }
        require(middle.size() > 50, "the middle of each straight is sampled");
        const auto a = middle.front(), b = middle.back();
        for (const auto& m : middle)
            worst_straightness = std::max(worst_straightness, std::abs((b.x-a.x)*(m.y-a.y)-(b.y-a.y)*(m.x-a.x))/std::hypot(b.x-a.x, b.y-a.y));
        samples += static_cast<int>(middle.size());
    }
    std::cout << "  stadium: " << line.iterations << " iterations; middle of the straights, " << samples << " samples: curvature up to "
              << worst_curvature << " 1/m, " << worst_straightness << " m from a straight line\n";
    require(line.converged && worst_curvature < 2e-3 && worst_straightness < 0.05, "along the middle of each straight the line is straight");
}

void rejects_bad_input() {
    const auto ring = clean(circle(40, 1.0), 12, "circle");
    fd::RacingLineOptions wide;
    wide.vehicle_half_width_m = 3;
    wide.margin_m = 3.5;
    require(refusal([&] { fd::make_minimum_curvature_line(ring, config, wide); }).find("narrower") != std::string::npos,
            "a corridor narrower than the vehicle and margin is refused");
    auto no_normals = ring;
    no_normals.left_normals.clear();
    require(refusal([&] { fd::make_minimum_curvature_line(no_normals, config); }).find("left normal") != std::string::npos,
            "a centreline without normals is refused");
    fd::RacingLineOptions none;
    none.max_iterations = 0;
    require(refusal([&] { fd::make_minimum_curvature_line(ring, config, none); }).find("max_iterations") != std::string::npos,
            "options out of range are refused");
}

// Phase 5.1 (decision 0021): the lattice laid along the racing line keeps its nodes usable although the line crosses the
// corridor from edge to edge: every layer's usable nodes still reach across most of its corridor.
void a_lattice_along_the_line_keeps_its_nodes() {
    const auto lattice = fd::make_lattice(foundry_line().track, config);
    std::size_t nodes = 0, used = 0;
    double narrowest = 1;
    for (std::size_t i = 0; i < lattice.layers.size(); ++i) {
        const auto& offsets = lattice.layers[i].offsets_m;
        double lowest = 1e9, highest = -1e9;
        for (std::size_t n = 0; n < offsets.size(); ++n) {
            ++nodes;
            if (lattice.edges_from(i, n).empty()) continue;
            ++used;
            lowest = std::min(lowest, offsets[n]);
            highest = std::max(highest, offsets[n]);
        }
        narrowest = std::min(narrowest, (highest-lowest)/(offsets.back()-offsets.front()));
    }
    std::cout << "  lattice along the racing line: " << lattice.layers.size() << " layers, " << used << " of " << nodes << " nodes usable, "
              << lattice.edges.size() << " edges; the narrowest layer's usable nodes span " << narrowest << " of its corridor\n";
    require(used >= 0.95*static_cast<double>(nodes), "at least 95% of the nodes stay usable");
    require(narrowest >= 0.9, "every layer's usable nodes span at least 90% of its corridor");
}

// The racing line is driven as it is: the four-wheel car, planned from 80% of its envelope, laps it inside the corridor,
// faster than the centreline, and with every tire within its peak. Pulling away from rest where the line still curves
// out of the last corner spun the inside wheel until the controller bounded driving at the lateral acceleration the
// line asks (decision 0020); both counts are kept apart to show it.
void the_car_drives_the_racing_line() {
    struct Outcome { double second_lap{-1}; int outside{}, beyond_launch{}, beyond_after{}; double worst{}; };
    const auto drive = [](const fd::Track& track) {
        fd::Simulation simulation(track, config, fd::FourWheelCar{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::performance_envelope);
        Outcome outcome;
        double first = -1;
        while (simulation.state().time_s < 62) {
            simulation.step();
            const auto& d = simulation.diagnostics();
            if (!d.within_track) ++outcome.outside;
            if (!d.within_grip_envelope) ++(d.progress_m < 50 ? outcome.beyond_launch : outcome.beyond_after);
            outcome.worst = std::max(outcome.worst, std::abs(d.cross_track_error_m));
            if (d.laps >= 1 && first < 0) first = simulation.state().time_s;
            if (d.laps >= 2 && outcome.second_lap < 0) outcome.second_lap = simulation.state().time_s-first;
        }
        return outcome;
    };
    const auto line = drive(foundry_line().track);
    const auto centre = drive(foundry().track);
    std::cout << "  four-wheel car, 80% of its envelope, second lap: centreline " << centre.second_lap << " s, racing line " << line.second_lap
              << " s; on the line tracking error up to " << line.worst << " m, " << line.outside << " samples outside the corridor, "
              << line.beyond_launch << " past a tire's peak in the first 50 m from rest and " << line.beyond_after << " after\n";
    require(line.outside == 0, "the car stays inside the corridor on the racing line");
    require(line.beyond_launch == 0 && line.beyond_after == 0, "no tire passes its peak, pulling away or after");
    require(line.second_lap > 0 && line.second_lap < centre.second_lap-1, "the racing line laps faster than the centreline, closed loop");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"a_circle_moves_to_its_outer_edge", a_circle_moves_to_its_outer_edge},
        {"stays_inside_the_corridor", stays_inside_the_corridor},
        {"curvature_within_the_steering_limit", curvature_within_the_steering_limit},
        {"laps_faster_than_the_centreline", laps_faster_than_the_centreline},
        {"a_straight_stays_straight", a_straight_stays_straight},
        {"rejects_bad_input", rejects_bad_input},
        {"a_lattice_along_the_line_keeps_its_nodes", a_lattice_along_the_line_keeps_its_nodes},
        {"the_car_drives_the_racing_line", the_car_drives_the_racing_line},
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cout << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-failures << '/' << tests.size() << " racing line groups passed\n";
    return failures == 0 ? 0 : 1;
}

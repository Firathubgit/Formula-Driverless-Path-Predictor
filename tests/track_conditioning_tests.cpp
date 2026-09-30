#include "fd/simulation.hpp"
#include "fd/track_conditioning.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Phase 3.3 (decision 0019): a closed centreline, traced or drawn, conditioned into a track the planner and controller
// can use: a smooth closed curve fitted within a stated allowance, resampled uniformly by arc length, with continuous
// curvature and left normals, refused when it crosses itself or its normals cross.
namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tolerance)
        throw std::runtime_error(message+": actual="+std::to_string(actual)+", expected="+std::to_string(expected));
}
const double pi = std::numbers::pi;

std::vector<fd::Vec2> circle(double radius, double spacing, bool clockwise = false) {
    std::vector<fd::Vec2> points;
    const auto count = static_cast<int>(std::ceil(2*pi*radius/spacing));
    for (int i = 0; i < count; ++i) {
        const double angle = (clockwise ? -1 : 1)*2*pi*i/count;
        points.push_back({radius*std::sin(angle), radius*(1-std::cos(angle))});
    }
    return points;
}

void a_circle_stays_that_circle() {
    fd::ConditioningOptions tight;
    tight.smoothing_rms_m = 0.002;
    const auto conditioned = fd::condition_track(circle(30, 1.3), 8, "circle", tight);
    const auto& track = conditioned.track;
    fd::validate_track(track);
    near(track.length_m, 2*pi*30, 0.05, "the length is the circle's");
    double worst_radius = 0, worst_curvature = 0;
    for (const auto& p : track.points) {
        worst_radius = std::max(worst_radius, std::abs(std::hypot(p.x_m, p.y_m-30)-30));
        worst_curvature = std::max(worst_curvature, std::abs(p.curvature-1.0/30));
    }
    std::cout << "  30 m circle: length " << track.length_m << " m, " << track.points.size() << " samples, largest radius error "
              << worst_radius << " m, curvature error " << worst_curvature << " 1/m, max deviation " << conditioned.max_deviation_m << " m\n";
    require(worst_radius < 0.02, "every sample lies on the circle");
    require(worst_curvature < 1e-3, "every sample has the circle's curvature");
    require(track.width_m == 8 && track.name == "circle", "width and name are kept");
}

// The plan's test. Foundry Circuit sampled every metre with 0.1 m of seeded noise, as a trace might be, is conditioned
// with the default allowance; away from the joins where the preset's curvature steps, the conditioned curvature is the
// preset's, and everywhere the line stays close to the preset's centreline.
struct Recovery { double worst_near{}, worst_far{}, deviation{}, length{}; std::map<int, double> by_distance; };
Recovery recover_foundry(double noise, const fd::ConditioningOptions& options, double exclusion) {
    const auto preset = fd::make_preset_track();
    std::mt19937 generator(20260916);
    std::normal_distribution<double> jitter(0.0, noise);
    std::vector<fd::Vec2> traced;
    for (double s = 0; s < preset.length_m-0.5; s += 1.0) {
        const auto p = fd::sample(preset, s);
        traced.push_back({p.x_m+jitter(generator), p.y_m+jitter(generator)});
    }
    const auto conditioned = fd::condition_track(traced, preset.width_m, "traced Foundry Circuit", options);
    std::vector<double> joins;
    for (std::size_t i = 0; i < preset.points.size(); ++i)
        if (preset.points[i].curvature != preset.points[(i+preset.points.size()-1)%preset.points.size()].curvature) joins.push_back(preset.points[i].s_m);
    Recovery r;
    r.length = conditioned.track.length_m;
    for (const auto& p : conditioned.track.points) {
        const auto nearest = fd::project(preset, {p.x_m, p.y_m});
        r.deviation = std::max(r.deviation, nearest.distance_m);
        double to_join = preset.length_m;
        for (const double j : joins) {
            double d = std::abs(nearest.s_m-j);
            to_join = std::min(to_join, std::min(d, preset.length_m-d));
        }
        const double error = std::abs(p.curvature-preset.points[nearest.index].curvature);
        (to_join >= exclusion ? r.worst_far : r.worst_near) = std::max(to_join >= exclusion ? r.worst_far : r.worst_near, error);
        auto& bucket = r.by_distance[static_cast<int>(to_join/2)*2];
        bucket = std::max(bucket, error);
    }
    return r;
}

void recovers_a_noisy_foundry_circuit() {
    const auto r = recover_foundry(0.1, {}, 10);
    std::cout << "  traced Foundry Circuit, 0.1 m noise: length " << r.length << " m against " << fd::make_preset_track().length_m
              << ", largest distance from the preset's centreline " << r.deviation << " m, curvature error " << r.worst_far
              << " 1/m from 10 m beyond a join (" << r.worst_near << " within)\n   worst curvature error by distance from a join:";
    for (const auto& [distance, error] : r.by_distance) if (distance <= 30) std::cout << " " << distance << " m " << error;
    std::cout << "\n";
    // The tightest corner's curvature is 1/18 m; away from the joins the error is a tenth of it.
    require(r.worst_far < 0.1/18, "away from the joins the conditioned curvature is the preset's");
    require(r.worst_near < 0.6/18, "at a join the step becomes a ramp, not an overshoot");
    require(r.deviation < 0.3, "the conditioned line stays within 0.3 m of the preset's centreline");
    near(r.length, fd::make_preset_track().length_m, 1.0, "the length is the preset's within a metre");
}

template<class Action> std::string refusal(Action action) {
    try { action(); } catch (const std::invalid_argument& error) { return error.what(); }
    return {};
}

// The plan's test: a centreline that crosses itself is refused.
void rejects_a_self_intersecting_centreline() {
    std::vector<fd::Vec2> figure_eight;
    for (int i = 0; i < 400; ++i) {
        const double t = 2*pi*i/400;
        figure_eight.push_back({60*std::sin(t), 60*std::sin(t)*std::cos(t)});
    }
    const auto message = refusal([&] { fd::condition_track(figure_eight, 6, "figure eight"); });
    std::cout << "  figure eight: " << message << "\n";
    require(message.find("crosses itself") != std::string::npos, "a figure eight is refused because it crosses itself");
    fd::ConditioningOptions line;
    line.corridor_checks = false;
    require(refusal([&] { fd::condition_track(figure_eight, 6, "figure eight line", line); }).find("crosses itself") != std::string::npos,
            "a line must not cross itself either");
}

// A hairpin tighter than half the width would fold the corridor over itself at the inside of the bend.
std::vector<fd::Vec2> stadium(double straight, double radius, double spacing) {
    std::vector<fd::Vec2> points;
    const auto arc_count = static_cast<int>(std::ceil(pi*radius/spacing));
    const auto line_count = static_cast<int>(std::ceil(straight/spacing));
    for (int i = 0; i < line_count; ++i) points.push_back({straight*i/line_count, 0});
    for (int i = 0; i < arc_count; ++i) { const double a = pi*i/arc_count; points.push_back({straight+radius*std::sin(a), radius*(1-std::cos(a))}); }
    for (int i = 0; i < line_count; ++i) points.push_back({straight*(1-static_cast<double>(i)/line_count), 2*radius});
    for (int i = 0; i < arc_count; ++i) { const double a = pi*i/arc_count; points.push_back({-radius*std::sin(a), radius*(1+std::cos(a))}); }
    return points;
}
void rejects_normals_that_cross() {
    fd::ConditioningOptions tight;
    tight.smoothing_rms_m = 0.01;
    const auto message = refusal([&] { fd::condition_track(stadium(60, 4, 1), 10, "tight stadium", tight); });
    std::cout << "  4 m hairpins on a 10 m wide track: " << message << "\n";
    require(message.find("Normals across the track cross") != std::string::npos && message.find("tighter than half") != std::string::npos,
            "hairpins tighter than half the width are refused because their normals cross");
    const auto wide = fd::condition_track(stadium(60, 7, 1), 10, "wide stadium", tight);
    require(wide.track.points.size() > 8, "7 m hairpins on the same track are accepted");
}

// Two parts of the loop closer than the width share tarmac, which the corridor model cannot represent.
void rejects_a_corridor_that_overlaps_itself() {
    std::vector<fd::Vec2> waisted;
    for (int i = 0; i < 500; ++i) {
        const double t = 2*pi*i/500;
        waisted.push_back({80*std::cos(t), std::sin(t)*(4.5+25*std::cos(t)*std::cos(t))});
    }
    const auto message = refusal([&] { fd::condition_track(waisted, 10, "waisted loop"); });
    std::cout << "  9 m waist on a 10 m wide track: " << message << "\n";
    require(message.find("overlaps itself") != std::string::npos, "a waist narrower than the width is refused because the corridor overlaps itself");
    const auto accepted = fd::condition_track(waisted, 8, "narrower waisted loop");
    require(accepted.track.width_m == 8, "the same loop with an 8 m wide track is accepted");
    // A line inside a known corridor, such as a racing line, is not judged as a corridor itself.
    fd::ConditioningOptions line;
    line.corridor_checks = false;
    const auto as_line = fd::condition_track(waisted, 10, "waisted line", line);
    require(as_line.track.points.size() > 8, "without corridor checks the same loop is accepted as a line");
}

// Conditioning turns the preset's curvature steps into ramps: neighbouring samples differ by a small fraction of a step.
void curvature_is_continuous() {
    const auto preset = fd::make_preset_track();
    std::vector<fd::Vec2> exact;
    for (double s = 0; s < preset.length_m-0.5; s += 1.0) { const auto p = fd::sample(preset, s); exact.push_back({p.x_m, p.y_m}); }
    const auto conditioned = fd::condition_track(exact, preset.width_m, "Foundry Circuit");
    double preset_step = 0, conditioned_step = 0, largest = 0;
    for (std::size_t i = 0; i < preset.points.size(); ++i)
        preset_step = std::max(preset_step, std::abs(preset.points[(i+1)%preset.points.size()].curvature-preset.points[i].curvature));
    const auto& points = conditioned.track.points;
    for (std::size_t i = 0; i < points.size(); ++i) {
        conditioned_step = std::max(conditioned_step, std::abs(points[(i+1)%points.size()].curvature-points[i].curvature));
        largest = std::max(largest, std::abs(points[i].curvature));
    }
    std::cout << "  Foundry Circuit: largest curvature step between samples " << preset_step << " 1/m in the preset, " << conditioned_step
              << " conditioned; largest curvature " << largest << " against 1/18 = " << 1.0/18 << "; residual " << conditioned.residual_rms_m
              << " m rms, max deviation " << conditioned.max_deviation_m << " m\n";
    require(conditioned_step < 0.05*preset_step, "neighbouring conditioned samples differ by under a twentieth of the preset's step");
    require(largest < 1.1/18, "rounding the corners does not tighten them by more than a tenth");
}

// The allowance is an effective control: a larger one moves the line further from the trace and smooths it more.
void smoothing_trades_distance_for_smoothness() {
    const auto preset = fd::make_preset_track();
    std::mt19937 generator(7);
    std::normal_distribution<double> jitter(0.0, 0.1);
    std::vector<fd::Vec2> traced;
    for (double s = 0; s < preset.length_m-0.5; s += 1.0) { const auto p = fd::sample(preset, s); traced.push_back({p.x_m+jitter(generator), p.y_m+jitter(generator)}); }
    const auto roughness = [](const fd::ConditionedTrack& c) {
        double sum = 0;
        const auto& points = c.track.points;
        for (std::size_t i = 0; i < points.size(); ++i) sum += std::abs(points[(i+1)%points.size()].curvature-points[i].curvature);
        return sum;
    };
    fd::ConditioningOptions light, heavy;
    light.smoothing_rms_m = 0.1;
    heavy.smoothing_rms_m = 0.3;
    const auto a = fd::condition_track(traced, preset.width_m, "light", light);
    const auto b = fd::condition_track(traced, preset.width_m, "heavy", heavy);
    std::cout << "  allowance 0.1 m: residual " << a.residual_rms_m << " m rms, total curvature variation " << roughness(a)
              << " 1/m; 0.3 m: residual " << b.residual_rms_m << " m rms, variation " << roughness(b) << " 1/m\n";
    require(a.residual_rms_m <= 0.1+1e-9 && b.residual_rms_m <= 0.3+1e-9, "each fit stays within its allowance");
    // Below the trace's own noise the fit follows the noise into kinks, which the normals check refuses.
    fd::ConditioningOptions below;
    below.smoothing_rms_m = 0.05;
    const auto kinked = refusal([&] { fd::condition_track(traced, preset.width_m, "below the noise", below); });
    std::cout << "  allowance 0.05 m under 0.1 m noise: " << kinked << "\n";
    require(kinked.find("Normals") != std::string::npos, "an allowance below the noise follows it into kinks the normals check refuses");
    require(b.residual_rms_m > a.residual_rms_m && roughness(b) < roughness(a), "a larger allowance moves the line further and smooths it more");
}

// Direction, start, spacing and normals follow the input.
void keeps_direction_start_spacing_and_normals() {
    fd::ConditioningOptions options;
    options.smoothing_rms_m = 0.01;
    options.output_spacing_m = 1.5;
    const auto input = circle(40, 1.0, true);
    const auto conditioned = fd::condition_track(input, 6, "clockwise", options);
    const auto& points = conditioned.track.points;
    near(conditioned.track.length_m/static_cast<double>(points.size()), 1.5, 0.01, "samples are the requested spacing apart, evenly");
    for (std::size_t i = 0; i < points.size(); ++i) {
        near(points[i].s_m, conditioned.track.length_m*static_cast<double>(i)/static_cast<double>(points.size()), 1e-6, "stations are uniform");
        require(points[i].curvature < 0, "a clockwise loop turns right, negative curvature");
        const auto& n = conditioned.left_normals[i];
        near(std::hypot(n.x, n.y), 1, 1e-12, "normals are unit length");
        const fd::Vec2 to_centre{0-points[i].x_m, 40-points[i].y_m};
        require(n.x*to_centre.x+n.y*to_centre.y < 0, "on a right-hand loop the left normal points away from the centre");
    }
    require(std::hypot(points[0].x_m-input[0].x, points[0].y_m-input[0].y) < 0.05, "sample 0 is beside the first centreline point");
}

// A track is conditioned from points taken along it every input spacing, as a centreline file of the same points would
// be, keeping its name and width.
void a_track_is_conditioned_from_points_along_it() {
    const auto preset = fd::make_preset_track();
    std::vector<fd::Vec2> exact;
    for (double s = 0; s < preset.length_m-0.5; s += 1.0) { const auto p = fd::sample(preset, s); exact.push_back({p.x_m, p.y_m}); }
    const auto from_points = fd::condition_track(exact, preset.width_m, preset.name);
    const auto from_track = fd::condition_track(preset);
    require(from_track.track.name == preset.name && from_track.track.width_m == preset.width_m, "the track keeps its name and width");
    require(from_track.track.points.size() == from_points.track.points.size() && from_track.resampled_points == from_points.resampled_points,
            "as many samples as from the same points");
    for (std::size_t i = 0; i < from_track.track.points.size(); ++i) {
        const auto& a = from_track.track.points[i];
        const auto& b = from_points.track.points[i];
        require(a.x_m == b.x_m && a.y_m == b.y_m && a.curvature == b.curvature, "the same conditioned track, sample "+std::to_string(i));
    }
    fd::ConditioningOptions coarse;
    coarse.input_spacing_m = 10;
    require(refusal([&] { fd::condition_track(preset, coarse); }).find("input_spacing_m") != std::string::npos, "a spacing out of range is refused");
}

void rejects_bad_input() {
    const auto ok = circle(30, 1);
    auto nan = ok;
    nan[5].x = std::nan("");
    fd::ConditioningOptions coarse;
    coarse.input_spacing_m = 10;
    const std::vector<std::pair<std::string, std::function<void()>>> cases{
        {"at least four distinct points", [] { fd::condition_track({{0, 0}, {10, 0}, {10, 10}, {10, 10}}, 5, "three"); }},
        {"non-finite", [&] { fd::condition_track(nan, 5, "nan"); }},
        {"width", [&] { fd::condition_track(ok, 0, "no width"); }},
        {"input_spacing_m", [&] { fd::condition_track(ok, 5, "coarse", coarse); }},
        {"too short", [] { fd::condition_track(circle(2, 0.5), 1, "tiny"); }},
    };
    for (const auto& [reason, action] : cases) {
        const auto message = refusal(action);
        require(message.find(reason) != std::string::npos, "refused with its reason ("+reason+"), got: "+message);
    }
}

void loads_a_centreline_file() {
    const auto directory = std::filesystem::temp_directory_path()/("fd-centreline-"+std::to_string(std::random_device{}()));
    std::filesystem::create_directories(directory);
    const auto write = [&](const std::string& name, const std::string& text) {
        std::ofstream(directory/name) << text;
        return directory/name;
    };
    const auto good = fd::load_centreline(write("good.csv", "# traced by hand\nx_m,y_m\n0,0\n\n10.5, 0 # a comment\n10.5,7\n"));
    require(good.size() == 3 && good[1].x == 10.5 && good[2].y == 7, "a header, comments and blank lines are allowed");
    const auto bad = refusal([&] { fd::load_centreline(write("bad.csv", "x_m,y_m\n0,0\n1,two\n")); });
    require(bad.find("line 3") != std::string::npos, "a bad row is refused, naming its line: "+bad);
    const auto three = refusal([&] { fd::load_centreline(write("three.csv", "0,0,0\n")); });
    require(three.find("line 1") != std::string::npos, "a row with three values is refused: "+three);
    std::filesystem::remove_all(directory);
}

// A conditioned trace is an ordinary track: the planner plans it and the car drives it.
void the_conditioned_track_is_driven() {
    const auto preset = fd::make_preset_track();
    std::mt19937 generator(11);
    std::normal_distribution<double> jitter(0.0, 0.1);
    std::vector<fd::Vec2> traced;
    for (double s = 0; s < preset.length_m-0.5; s += 1.0) { const auto p = fd::sample(preset, s); traced.push_back({p.x_m+jitter(generator), p.y_m+jitter(generator)}); }
    auto track = fd::condition_track(traced, preset.width_m, "traced Foundry Circuit").track;
    fd::Simulation simulation(track, fd::Config{});
    double worst = 0;
    int invalid = 0;
    while (simulation.state().time_s < 20) {
        simulation.step();
        worst = std::max(worst, std::abs(simulation.diagnostics().cross_track_error_m));
        if (!simulation.diagnostics().plan_valid || !simulation.diagnostics().within_track) ++invalid;
    }
    std::cout << "  kinematic car on the traced track: " << simulation.diagnostics().progress_m << " m in 20 s, largest tracking error "
              << worst << " m, " << invalid << " invalid samples\n";
    require(simulation.diagnostics().progress_m > 250 && invalid == 0 && worst < 0.5, "the car drives the conditioned track within it");
}

} // namespace

// The Gokartcentralen Göteborg track (decision 0038), as traced from the operator's poster and verified against it: the
// published 400 m, a 6 m corridor the simulator accepts (no bend tighter than half its width, no two parts closer than its
// width), driven clockwise, and the poster's ten turns in its order and directions from the start line: T1 right, T2 a left
// kink, T3 right, T4 right, T5 left, T6 left, T7 right, T8 right, T9 left, T10 right.
void the_gokart_track_is_the_poster() {
    const auto points = fd::load_centreline(FD_KART_TRACK);
    double raw = 0;
    for (std::size_t i = 0; i < points.size(); ++i) raw += std::hypot(points[(i+1)%points.size()].x-points[i].x, points[(i+1)%points.size()].y-points[i].y);
    near(raw, 400, 1.0, "the traced lap is the published 400 m");
    fd::ConditioningOptions options;
    options.smoothing_rms_m = 0.02;
    const auto conditioned = fd::condition_track(points, 6.0, "Gokartcentralen Göteborg", options);
    const auto& track = conditioned.track;
    near(track.length_m, 400, 2.0, "conditioned, still 400 m");
    double area = 0;
    for (std::size_t i = 0; i < track.points.size(); ++i) {
        const auto& a = track.points[i];
        const auto& b = track.points[(i+1)%track.points.size()];
        area += a.x_m*b.y_m-b.x_m*a.y_m;
    }
    require(area < 0, "clockwise, a right-hand circuit");
    // A turn is at least 5 m of arc tighter than 12 m; one that briefly opens up mid-corner, within 8 m, is still one
    // turn, and a shorter ripple is none.
    struct Turn { double from, to; char side; };
    std::vector<Turn> found;
    for (const auto& p : track.points) {
        if (std::abs(p.curvature) <= 1.0/12) continue;
        const char side = p.curvature > 0 ? 'L' : 'R';
        if (!found.empty() && found.back().side == side && p.s_m-found.back().to < 8) found.back().to = p.s_m;
        else found.push_back({p.s_m, p.s_m, side});
    }
    std::erase_if(found, [](const Turn& t) { return t.to-t.from < 5; });
    std::string turns;
    for (const auto& t : found) turns += t.side;
    std::cout << "  Gokartcentralen Göteborg: " << track.length_m << " m, turns tighter than 12 m: " << turns << " at";
    for (const auto& t : found) std::cout << ' ' << std::lround(t.from) << '-' << std::lround(t.to);
    std::cout << " m" << '\n';
    require(turns == "RLRRLLRRLR", "the poster's ten turns in order: "+turns);
    // Two complete practice laps at each side of the actual kart corridor. These
    // are timing fixtures, not a second vehicle plant or a driving controller.
    for (double offset : {-2.0,0.0,2.0}) {
        fd::PracticeLaps clock(track);
        const auto pose=[&](double station) {
            const auto p=fd::sample(track,station),q=fd::sample(track,station+0.05);
            const double dx=q.x_m-p.x_m,dy=q.y_m-p.y_m,n=std::hypot(dx,dy);
            return fd::State{p.x_m-dy/n*offset,p.y_m+dx/n*offset,std::atan2(dy,dx),10,0,station/10};
        };
        for(double s=0;s<2*track.length_m+0.1;s+=0.05) clock.observe(pose(s),pose(s+0.05),1);
        require(clock.completed()==2,"the actual kart corridor times both complete laps");
        near(clock.previous_seconds(),track.length_m/10,0.01,"previous is one full kart lap, not total session time");
        near(clock.best_seconds(),track.length_m/10,0.01,"session best uses completed kart laps");
    }
}

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"a_circle_stays_that_circle", a_circle_stays_that_circle},
        {"recovers_a_noisy_foundry_circuit", recovers_a_noisy_foundry_circuit},
        {"rejects_a_self_intersecting_centreline", rejects_a_self_intersecting_centreline},
        {"rejects_normals_that_cross", rejects_normals_that_cross},
        {"rejects_a_corridor_that_overlaps_itself", rejects_a_corridor_that_overlaps_itself},
        {"curvature_is_continuous", curvature_is_continuous},
        {"smoothing_trades_distance_for_smoothness", smoothing_trades_distance_for_smoothness},
        {"keeps_direction_start_spacing_and_normals", keeps_direction_start_spacing_and_normals},
        {"a_track_is_conditioned_from_points_along_it", a_track_is_conditioned_from_points_along_it},
        {"rejects_bad_input", rejects_bad_input},
        {"loads_a_centreline_file", loads_a_centreline_file},
        {"the_conditioned_track_is_driven", the_conditioned_track_is_driven},
        {"the_gokart_track_is_the_poster", the_gokart_track_is_the_poster},
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cout << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-failures << '/' << tests.size() << " track conditioning groups passed\n";
    return failures == 0 ? 0 : 1;
}

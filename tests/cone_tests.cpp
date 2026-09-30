#include "fd/cones.hpp"

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
void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tolerance)
        throw std::runtime_error(message+": actual="+std::to_string(actual)+", expected="+std::to_string(expected));
}

// Which side of the direction of travel a point lies, and how far from the reference: positive to the left.
double signed_offset(const fd::Track& track, fd::Vec2 point) {
    const auto at = fd::project(track, point);
    const auto& p = track.points[at.index];
    const auto& q = track.points[(at.index+1)%track.points.size()];
    const double tx = q.x_m-p.x_m, ty = q.y_m-p.y_m;
    const double cross = tx*(point.y-at.point.y)-ty*(point.x-at.point.x);
    return cross >= 0 ? at.distance_m : -at.distance_m;
}

// Blue cones stand on the left boundary and yellow on the right, each half the corridor's width from the centreline, and
// the big orange pair stands on the start line, one on each boundary.
void cones_stand_on_the_corridor_s_boundaries() {
    const auto track = fd::make_preset_track();
    const auto cones = fd::make_cone_layout(track);
    std::size_t blue = 0, yellow = 0, big = 0;
    for (const auto& cone : cones) {
        const double offset = signed_offset(track, cone.position);
        if (cone.colour == fd::ConeColour::blue) { ++blue; near(offset, track.width_m/2, 0.05, "a blue cone stands on the left boundary"); }
        if (cone.colour == fd::ConeColour::yellow) { ++yellow; near(offset, -track.width_m/2, 0.05, "a yellow cone stands on the right boundary"); }
        if (cone.colour == fd::ConeColour::big_orange) {
            ++big;
            near(std::abs(offset), track.width_m/2, 0.05, "a big orange cone stands on a boundary");
            // The start line is both ends of the closed loop.
            const double s = fd::project(track, cone.position).s_m;
            near(std::min(s, track.length_m-s), 0, 0.05, "at the start line");
        }
        require(cone.colour != fd::ConeColour::orange, "the course lays no small orange cones");
    }
    std::cout << "  preset: " << blue << " blue, " << yellow << " yellow, " << big << " big orange over "
              << track.length_m << " m\n";
    require(big == 2, "one big orange cone either side of the start line");
    require(blue > track.length_m/5 && yellow > track.length_m/5, "each boundary is coned at least every five metres");
    // The left boundary comes first, in the direction of travel, its big orange cone at the start.
    require(cones.front().colour == fd::ConeColour::big_orange && cones[1].colour == fd::ConeColour::blue,
            "the left boundary is laid first, from the start line");
    const auto second_big = std::find_if(cones.begin()+1, cones.end(),
                                         [](const fd::Cone& c) { return c.colour == fd::ConeColour::big_orange; });
    require(second_big != cones.end() && std::all_of(second_big+1, cones.end(),
                                                     [](const fd::Cone& c) { return c.colour == fd::ConeColour::yellow; }),
            "then the right, after its own big orange cone");
    const double first = fd::project(track, cones[1].position).s_m;
    const double second = fd::project(track, cones[2].position).s_m;
    require(first > 0 && second > first, "each boundary is laid in the direction of travel");
}

// Consecutive cones on a boundary are never more than the spacing on straights apart, nor closer than the minimum,
// and a boundary that turns is coned more densely than one that runs straight.
void corners_are_coned_more_densely_than_straights() {
    const auto track = fd::make_preset_track();
    const fd::ConeLayoutOptions options;
    const auto cones = fd::make_cone_layout(track, options);
    // Walk each boundary's cones in order, the loop closing back onto its big orange cone.
    const auto second_big = static_cast<std::size_t>(
        std::find_if(cones.begin()+1, cones.end(), [](const fd::Cone& c) { return c.colour == fd::ConeColour::big_orange; })-cones.begin());
    double straight_cones = 0, straight_length = 0, corner_cones = 0, corner_length = 0;
    for (const auto& [from, to] : {std::pair<std::size_t, std::size_t>{0, second_big}, {second_big, cones.size()}}) {
        for (std::size_t i = from; i < to; ++i) {
            const auto& a = cones[i].position;
            const auto& b = cones[i+1 < to ? i+1 : from].position;
            const double gap = std::hypot(b.x-a.x, b.y-a.y);
            require(gap <= options.straight_spacing_m+1e-6, "no two neighbouring cones stand more than five metres apart");
            require(gap >= options.min_spacing_m*0.9, "nor closer than the minimum spacing");
            const double curvature = std::abs(fd::sample(track, fd::project(track, a).s_m).curvature);
            if (curvature > 0.03) { corner_cones += 1; corner_length += gap; }
            if (curvature < 0.005) { straight_cones += 1; straight_length += gap; }
        }
    }
    require(straight_length > 0 && corner_length > 0, "the preset has both straights and corners");
    const double straight_density = straight_cones/straight_length, corner_density = corner_cones/corner_length;
    std::cout << "  cones per metre of boundary: " << straight_density << " on straights, " << corner_density << " in corners\n";
    near(straight_density, 1/options.straight_spacing_m, 0.02, "straights carry a cone every five metres");
    require(corner_density > 1.3*straight_density, "corners carry cones more densely than straights");
    // Turning less often per cone places more cones.
    fd::ConeLayoutOptions finer = options;
    finer.turn_angle_rad = options.turn_angle_rad/2;
    require(fd::make_cone_layout(track, finer).size() > cones.size(), "a smaller turn per cone places more cones");
}

// A track that records its corridor's edges, such as a racing line, cones those edges rather than half its width.
void recorded_edges_are_coned_where_they_are() {
    auto track = fd::make_preset_track();
    track.left_edge_m.assign(track.points.size(), 3.0);
    track.right_edge_m.assign(track.points.size(), 7.0);
    for (const auto& cone : fd::make_cone_layout(track)) {
        const double offset = signed_offset(track, cone.position);
        if (cone.colour == fd::ConeColour::blue) near(offset, 3.0, 0.05, "the left boundary is where the left edge is");
        if (cone.colour == fd::ConeColour::yellow) near(offset, -7.0, 0.05, "the right boundary is where the right edge is");
    }
}

// Options outside their ranges, and a track the rest of the project would refuse, are refused.
void a_layout_that_cannot_be_laid_is_refused() {
    const auto refused = [](const std::function<void()>& lay) {
        try { lay(); } catch (const std::invalid_argument&) { return true; }
        return false;
    };
    const auto track = fd::make_preset_track();
    require(refused([&] { fd::ConeLayoutOptions o; o.straight_spacing_m = 0; fd::make_cone_layout(track, o); }), "no spacing is refused");
    require(refused([&] { fd::ConeLayoutOptions o; o.min_spacing_m = 6; fd::make_cone_layout(track, o); }),
            "a minimum above the spacing on straights is refused");
    require(refused([&] { fd::ConeLayoutOptions o; o.turn_angle_rad = 0; fd::make_cone_layout(track, o); }), "no turn is refused");
    require(refused([&] { fd::Track broken; fd::make_cone_layout(broken); }), "a track with no samples is refused");
    require(std::string(fd::cone_colour_name(fd::ConeColour::big_orange)) == "big orange", "colours have names");
}

}  // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"cones_stand_on_the_corridor_s_boundaries", cones_stand_on_the_corridor_s_boundaries},
        {"corners_are_coned_more_densely_than_straights", corners_are_coned_more_densely_than_straights},
        {"recorded_edges_are_coned_where_they_are", recorded_edges_are_coned_where_they_are},
        {"a_layout_that_cannot_be_laid_is_refused", a_layout_that_cannot_be_laid_is_refused}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " cone groups passed\n";
    return failures ? 1 : 0;
}

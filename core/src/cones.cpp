#include "fd/cones.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace fd {
namespace {

// One boundary as a closed polyline, from the first sample around to it again, with the arc length at each vertex.
struct Boundary {
    std::vector<Vec2> points;
    std::vector<double> arc;  // arc[k] is the length from points[0] to points[k]; arc.back() closes the loop
};

Boundary boundary(const Track& track, double side) {
    const std::size_t n = track.points.size();
    Boundary b;
    b.points.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const auto& before = track.points[(i+n-1)%n];
        const auto& after = track.points[(i+1)%n];
        const double dx = after.x_m-before.x_m, dy = after.y_m-before.y_m, length = std::hypot(dx, dy);
        // The left normal of the reference's direction at this sample, as the corridor is measured along it.
        const Vec2 normal{-dy/length, dx/length};
        const double offset = side > 0 ? (track.left_edge_m.empty() ? track.width_m/2 : track.left_edge_m[i])
                                       : -(track.right_edge_m.empty() ? track.width_m/2 : track.right_edge_m[i]);
        b.points.push_back({track.points[i].x_m+normal.x*offset, track.points[i].y_m+normal.y*offset});
    }
    b.arc.assign(n+1, 0.0);
    for (std::size_t k = 1; k <= n; ++k) {
        const auto& p = b.points[k-1];
        const auto& q = b.points[k%n];
        b.arc[k] = b.arc[k-1]+std::hypot(q.x-p.x, q.y-p.y);
    }
    return b;
}

Vec2 point_at(const Boundary& b, double arc) {
    const std::size_t n = b.points.size();
    std::size_t k = 0;
    while (k+1 < b.arc.size()-1 && b.arc[k+1] <= arc) ++k;
    const double span = b.arc[k+1]-b.arc[k];
    const double t = span > 0 ? (arc-b.arc[k])/span : 0.0;
    const auto& p = b.points[k];
    const auto& q = b.points[(k+1)%n];
    return {p.x+(q.x-p.x)*t, p.y+(q.y-p.y)*t};
}

double heading(const Vec2& from, const Vec2& to) { return std::atan2(to.y-from.y, to.x-from.x); }

double turn(double from, double to) {
    double d = to-from;
    while (d > std::numbers::pi) d -= 2*std::numbers::pi;
    while (d < -std::numbers::pi) d += 2*std::numbers::pi;
    return std::abs(d);
}

// Where along one boundary its cones stand, as arc lengths from the start line, the start line itself first.
std::vector<double> stations(const Boundary& b, const ConeLayoutOptions& o) {
    const std::size_t n = b.points.size();
    const double total = b.arc.back();
    std::vector<double> placed{0.0};
    double turned = 0;
    for (std::size_t k = 1; k <= n; ++k) {
        // Along the segment ending at vertex k, a cone every straight spacing since the last.
        while (placed.back()+o.straight_spacing_m < b.arc[k]) {
            placed.push_back(placed.back()+o.straight_spacing_m);
            turned = 0;
        }
        if (k == n) break;
        // At the vertex, the boundary turns; enough turning since the last cone places one here.
        turned += turn(heading(b.points[k-1], b.points[k]), heading(b.points[k], b.points[(k+1)%n]));
        if (turned >= o.turn_angle_rad && b.arc[k]-placed.back() >= o.min_spacing_m) {
            placed.push_back(b.arc[k]);
            turned = 0;
        }
    }
    // Close the loop: the last cone must not crowd the start line's, and the gap it leaves must not exceed a spacing.
    if (placed.size() > 1 && total-placed.back() < o.min_spacing_m) placed.pop_back();
    if (total-placed.back() > o.straight_spacing_m) placed.push_back(placed.back()+(total-placed.back())/2);
    return placed;
}

}  // namespace

const char* cone_colour_name(ConeColour colour) {
    switch (colour) {
    case ConeColour::blue: return "blue";
    case ConeColour::yellow: return "yellow";
    case ConeColour::orange: return "orange";
    case ConeColour::big_orange: return "big orange";
    }
    return "unknown";
}

void validate_cone_layout_options(const ConeLayoutOptions& o) {
    if (!std::isfinite(o.straight_spacing_m) || o.straight_spacing_m <= 0 || o.straight_spacing_m > 50)
        throw std::invalid_argument("Cone spacing on straights must lie within 0..50 m");
    if (!std::isfinite(o.min_spacing_m) || o.min_spacing_m <= 0 || o.min_spacing_m > o.straight_spacing_m)
        throw std::invalid_argument("The minimum cone spacing must be positive and no more than the spacing on straights");
    if (!std::isfinite(o.turn_angle_rad) || o.turn_angle_rad <= 0 || o.turn_angle_rad > std::numbers::pi)
        throw std::invalid_argument("The turn that places a cone must lie within 0..pi rad");
}

std::vector<Cone> make_cone_layout(const Track& track, const ConeLayoutOptions& options) {
    validate_track(track);
    validate_cone_layout_options(options);
    std::vector<Cone> cones;
    for (const double side : {1.0, -1.0}) {
        const auto edge = boundary(track, side);
        const auto along = stations(edge, options);
        for (std::size_t i = 0; i < along.size(); ++i)
            cones.push_back({point_at(edge, along[i]),
                             i == 0 ? ConeColour::big_orange : side > 0 ? ConeColour::blue : ConeColour::yellow});
    }
    return cones;
}

}  // namespace fd

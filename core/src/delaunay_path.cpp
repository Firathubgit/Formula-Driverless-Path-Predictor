// A Delaunay midpoint planner written from the published description of FS-FEUP's planner (its README: triangulate the
// visible cones, take midpoints of triangle edges filtered by length, search a graph of them with a recursive
// lookahead, smooth with a B-spline). No FS-FEUP code was read or copied; it is GPL-3.0. See decision 0031.
#include "fd/delaunay_path.hpp"
#include "fitpack.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <numbers>
#include <stdexcept>

namespace fd {
namespace {

double cross(Vec2 o, Vec2 a, Vec2 b) { return (a.x-o.x)*(b.y-o.y)-(a.y-o.y)*(b.x-o.x); }
double distance(Vec2 a, Vec2 b) { return std::hypot(a.x-b.x, a.y-b.y); }

double wrap(double a) {
    while (a > std::numbers::pi) a -= 2*std::numbers::pi;
    while (a < -std::numbers::pi) a += 2*std::numbers::pi;
    return a;
}

}  // namespace

// Bowyer-Watson with one ghost vertex at infinity in place of a finite super-triangle, whose far corners can fall inside
// the circumcircle of a thin triangle on the hull and so lose it. Every triangle is counterclockwise; a ghost triangle
// (a, b, ghost) lies beyond the hull edge b -> a, and its "circumcircle" is the open half-plane beyond that edge together
// with the open edge itself. Cone rows along a straight are exactly collinear, which this handles as well.
std::vector<std::array<int, 3>> delaunay_triangles(std::span<const Vec2> input) {
    const std::vector<Vec2> p(input.begin(), input.end());
    const int n = static_cast<int>(p.size());
    constexpr int ghost = -1;
    if (n < 3) return {};
    double scale = 1;
    for (const auto& q : p) scale = std::max({scale, std::abs(q.x-p[0].x), std::abs(q.y-p[0].y)});
    const auto at = [&](int i) { return p[static_cast<std::size_t>(i)]; };
    // A third point off the line of the first two starts the triangulation; without one there is no triangle.
    int third = -1;
    for (int i = 2; i < n && third < 0; ++i)
        if (std::abs(cross(at(0), at(1), at(i))) > 1e-12*scale*scale) third = i;
    if (third < 0) return {};
    std::vector<std::array<int, 3>> triangles;
    int a = 0, b = 1, c = third;
    if (cross(at(a), at(b), at(c)) < 0) std::swap(b, c);
    triangles = {{a, b, c}, {b, a, ghost}, {c, b, ghost}, {a, c, ghost}};
    const auto in_circle = [&](const std::array<int, 3>& t, Vec2 q) {
        if (t[2] == ghost) {
            const Vec2 u = at(t[0]), v = at(t[1]);
            const double side = cross(u, v, q);
            if (side > 1e-12*scale*scale) return true;
            if (side < -1e-12*scale*scale) return false;
            const double along = (q.x-u.x)*(v.x-u.x)+(q.y-u.y)*(v.y-u.y), length2 = (v.x-u.x)*(v.x-u.x)+(v.y-u.y)*(v.y-u.y);
            return along > 0 && along < length2;
        }
        const Vec2 A = at(t[0]), B = at(t[1]), C = at(t[2]);
        const double ax = A.x-q.x, ay = A.y-q.y, bx = B.x-q.x, by = B.y-q.y, cx = C.x-q.x, cy = C.y-q.y;
        const double det = (ax*ax+ay*ay)*(bx*cy-cx*by)-(bx*bx+by*by)*(ax*cy-cx*ay)+(cx*cx+cy*cy)*(ax*by-bx*ay);
        return det > 1e-12*scale*scale*scale*scale;
    };
    for (int i = 2; i < n; ++i) {
        if (i == third) continue;
        const Vec2 q = at(i);
        std::vector<std::array<int, 3>> kept;
        std::map<std::pair<int, int>, int> removed_edges;
        for (const auto& t : triangles) {
            if (in_circle(t, q))
                for (std::size_t k = 0; k < 3; ++k) ++removed_edges[{t[k], t[(k+1)%3]}];
            else
                kept.push_back(t);
        }
        // The cavity's boundary: directed edges of removed triangles whose reverse was not removed with them.
        for (const auto& [edge, count] : removed_edges) {
            if (removed_edges.count({edge.second, edge.first})) continue;
            const auto [u, v] = edge;
            if (u == ghost) kept.push_back({v, i, ghost});
            else if (v == ghost) kept.push_back({i, u, ghost});
            else kept.push_back({u, v, i});
        }
        triangles = std::move(kept);
    }
    std::vector<std::array<int, 3>> out;
    for (const auto& t : triangles)
        if (t[2] != ghost) out.push_back(t);
    return out;
}

DelaunayPath plan_delaunay_path(std::span<const TypedCone> cones, Vec2 position, Vec2 direction, const DelaunaySettings& s) {
    const double heading0 = std::atan2(direction.y, direction.x);
    DelaunayPath out;
    // Distinct cones only: a triangulation of coincident points is not defined.
    std::vector<TypedCone> distinct;
    for (const auto& c : cones) {
        bool seen = false;
        for (const auto& d : distinct) seen = seen || distance(c.position, d.position) < 1e-6;
        if (!seen) distinct.push_back(c);
    }
    std::vector<Vec2> points;
    for (const auto& c : distinct) points.push_back(c.position);
    const auto triangles = delaunay_triangles(points);
    for (const auto& t : triangles) out.triangles.push_back({points[static_cast<std::size_t>(t[0])], points[static_cast<std::size_t>(t[1])],
                                                            points[static_cast<std::size_t>(t[2])]});
    // The candidate midpoints: edges of about a track's width that do not join two cones of the same side's colour.
    std::map<std::pair<int, int>, int> midpoint_of;
    std::vector<std::vector<int>> neighbours;
    const auto same_side = [&](int a, int b) {
        const auto ta = distinct[static_cast<std::size_t>(a)].type, tb = distinct[static_cast<std::size_t>(b)].type;
        return (ta == ConeType::blue || ta == ConeType::yellow) && ta == tb;
    };
    for (const auto& t : triangles) {
        std::vector<int> here;
        for (int k = 0; k < 3; ++k) {
            int a = t[static_cast<std::size_t>(k)], b = t[static_cast<std::size_t>((k+1)%3)];
            if (a > b) std::swap(a, b);
            const double length = distance(points[static_cast<std::size_t>(a)], points[static_cast<std::size_t>(b)]);
            if (length < s.min_edge_m || length > s.max_edge_m || same_side(a, b)) continue;
            auto [it, inserted] = midpoint_of.try_emplace({a, b}, static_cast<int>(out.midpoints.size()));
            if (inserted) {
                const auto pa = points[static_cast<std::size_t>(a)], pb = points[static_cast<std::size_t>(b)];
                out.midpoints.push_back({(pa.x+pb.x)/2, (pa.y+pb.y)/2});
                neighbours.emplace_back();
            }
            here.push_back(it->second);
        }
        // Midpoints of one triangle are neighbours: the track runs from one of its crossing edges to another.
        for (const int a : here)
            for (const int b : here)
                if (a != b && std::find(neighbours[static_cast<std::size_t>(a)].begin(), neighbours[static_cast<std::size_t>(a)].end(), b) ==
                                  neighbours[static_cast<std::size_t>(a)].end())
                    neighbours[static_cast<std::size_t>(a)].push_back(b);
    }
    const auto& mids = out.midpoints;
    const auto ahead_of = [&](Vec2 from, double heading, Vec2 to) {
        return std::cos(heading)*(to.x-from.x)+std::sin(heading)*(to.y-from.y) > 0;
    };
    // The walk starts at the nearest midpoint ahead of the car.
    int current = -1;
    double nearest = s.max_first_m;
    for (std::size_t i = 0; i < mids.size(); ++i) {
        const double d = distance(mids[i], position);
        if (ahead_of(position, heading0, mids[i]) && d < nearest) { nearest = d; current = static_cast<int>(i); }
    }
    if (current < 0) return out;
    std::vector<bool> visited(mids.size(), false);
    visited[static_cast<std::size_t>(current)] = true;
    out.walk.push_back(mids[static_cast<std::size_t>(current)]);
    double heading = std::atan2(mids[static_cast<std::size_t>(current)].y-position.y, mids[static_cast<std::size_t>(current)].x-position.x);
    double walked = nearest;
    const auto valid_step = [&](int from, double h, int to) {
        const Vec2 a = mids[static_cast<std::size_t>(from)], b = mids[static_cast<std::size_t>(to)];
        const double length = distance(a, b);
        if (length < 1e-6 || length > s.max_step_m) return false;
        return std::abs(wrap(std::atan2(b.y-a.y, b.x-a.x)-h)) <= s.max_turn_rad;
    };
    // The recursive lookahead: of every sequence of up to `lookahead` steps, the deepest, then the one that turns least.
    struct Score { int depth{}; double turn{}; };
    std::function<Score(int, double, int)> look = [&](int from, double h, int depth) -> Score {
        Score best;
        if (depth == 0) return best;
        for (const int next : neighbours[static_cast<std::size_t>(from)]) {
            if (visited[static_cast<std::size_t>(next)] || !valid_step(from, h, next)) continue;
            const Vec2 a = mids[static_cast<std::size_t>(from)], b = mids[static_cast<std::size_t>(next)];
            const double nh = std::atan2(b.y-a.y, b.x-a.x);
            visited[static_cast<std::size_t>(next)] = true;
            const Score sub = look(next, nh, depth-1);
            visited[static_cast<std::size_t>(next)] = false;
            const Score here{sub.depth+1, sub.turn+std::abs(wrap(nh-h))};
            if (here.depth > best.depth || (here.depth == best.depth && here.turn < best.turn)) best = here;
        }
        return best;
    };
    while (walked < s.path_length_m+5) {
        int chosen = -1;
        Score best;
        for (const int next : neighbours[static_cast<std::size_t>(current)]) {
            if (visited[static_cast<std::size_t>(next)] || !valid_step(current, heading, next)) continue;
            const Vec2 a = mids[static_cast<std::size_t>(current)], b = mids[static_cast<std::size_t>(next)];
            const double nh = std::atan2(b.y-a.y, b.x-a.x);
            visited[static_cast<std::size_t>(next)] = true;
            const Score sub = look(next, nh, s.lookahead-1);
            visited[static_cast<std::size_t>(next)] = false;
            const Score here{sub.depth+1, sub.turn+std::abs(wrap(nh-heading))};
            if (chosen < 0 || here.depth > best.depth || (here.depth == best.depth && here.turn < best.turn)) { best = here; chosen = next; }
        }
        if (chosen < 0) break;
        const Vec2 a = mids[static_cast<std::size_t>(current)], b = mids[static_cast<std::size_t>(chosen)];
        heading = std::atan2(b.y-a.y, b.x-a.x);
        walked += distance(a, b);
        visited[static_cast<std::size_t>(chosen)] = true;
        current = chosen;
        out.walk.push_back(b);
    }
    // Fitted through the car and the walk with FaSTTUBe's smoothing, and sampled over the path's length.
    std::vector<std::array<double, 2>> fit_points{{position.x, position.y}};
    std::vector<double> u{0.0};
    for (const auto& w : out.walk) {
        u.push_back(u.back()+std::hypot(w.x-fit_points.back()[0], w.y-fit_points.back()[1]));
        fit_points.push_back({w.x, w.y});
    }
    if (fit_points.size() < 2) return out;
    const int k = std::clamp(static_cast<int>(fit_points.size())-1, 1, 3);
    const auto curve = fitpack::fit_curve(u, fit_points, k, 0.2);
    const double length = std::min(s.path_length_m, u.back());
    for (int i = 0; i < s.horizon_points; ++i) {
        const double at = length*i/(s.horizon_points-1);
        const auto p = fitpack::evaluate(curve, at);
        out.path.push_back({at, p[0], p[1], 0.0});
    }
    // Curvature from each sample and its neighbours: the turn of the chord over its length.
    for (std::size_t i = 1; i+1 < out.path.size(); ++i) {
        const auto& a = out.path[i-1];
        const auto& b = out.path[i];
        const auto& c = out.path[i+1];
        const double h1 = std::atan2(b.y_m-a.y_m, b.x_m-a.x_m), h2 = std::atan2(c.y_m-b.y_m, c.x_m-b.x_m);
        const double arc = std::hypot(c.x_m-a.x_m, c.y_m-a.y_m)/2;
        out.path[i].curvature_1pm = arc > 0 ? wrap(h2-h1)/arc : 0.0;
    }
    if (out.path.size() >= 3) {
        out.path.front().curvature_1pm = out.path[1].curvature_1pm;
        out.path.back().curvature_1pm = out.path[out.path.size()-2].curvature_1pm;
    }
    return out;
}

}  // namespace fd

#include "kart_hall.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <string>

namespace {
using V = SceneVertex;
struct P3 { double x{}, y{}, z{}; };

void tri(std::vector<V>& out, const P3& a, const P3& b, const P3& c, const QColor& ca, const QColor& cb, const QColor& cc,
         double alpha_a = 1, double alpha_b = 1, double alpha_c = 1) {
    out.push_back(scene_vertex(a.x, a.y, a.z, ca, alpha_a));
    out.push_back(scene_vertex(b.x, b.y, b.z, cb, alpha_b));
    out.push_back(scene_vertex(c.x, c.y, c.z, cc, alpha_c));
}
// A quad by its corners in order round it, each with its own colour.
void quad(std::vector<V>& out, const P3& a, const P3& b, const P3& c, const P3& d, const QColor& ca, const QColor& cb,
          const QColor& cc, const QColor& cd) {
    tri(out, a, b, c, ca, cb, cc);
    tri(out, a, c, d, ca, cc, cd);
}
void quad(std::vector<V>& out, const P3& a, const P3& b, const P3& c, const P3& d, const QColor& colour) {
    quad(out, a, b, c, d, colour, colour, colour, colour);
}
// A box standing square to the axes, each pair of faces in its own shade, as light from above leaves them.
void box(std::vector<V>& out, const P3& lo, const P3& hi, const QColor& faces_x, const QColor& faces_z, const QColor& top,
         const QColor& bottom) {
    quad(out, {lo.x, lo.y, lo.z}, {lo.x, lo.y, hi.z}, {lo.x, hi.y, hi.z}, {lo.x, hi.y, lo.z}, faces_x);
    quad(out, {hi.x, lo.y, lo.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {hi.x, hi.y, lo.z}, faces_x);
    quad(out, {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z}, faces_z);
    quad(out, {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}, faces_z);
    quad(out, {lo.x, hi.y, lo.z}, {hi.x, hi.y, lo.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}, top);
    quad(out, {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, lo.y, hi.z}, {lo.x, lo.y, hi.z}, bottom);
}

QColor scaled(const QColor& colour, double factor, double towards_white = 0) {
    const auto channel = [&](double v) { return std::clamp(v * factor + towards_white * (1 - v * factor), 0.0, 1.0); };
    return QColor::fromRgbF(float(channel(colour.redF())), float(channel(colour.greenF())), float(channel(colour.blueF())));
}

// A sample of the track with its left normal, its edges and how tightly it bends there (positive to the left).
struct Frame { fd::Vec2 p, n; double s{}, left{}, right{}, curvature{}; };

std::vector<Frame> frames_of(const fd::Track& track) {
    const auto& points = track.points;
    const std::size_t n = points.size();
    std::vector<Frame> frames(n);
    std::vector<double> raw(n);
    for (std::size_t k = 0; k < n; ++k) {
        const auto& before = points[(k + n - 1) % n];
        const auto& here = points[k];
        const auto& after = points[(k + 1) % n];
        const double dx = after.x_m - before.x_m, dy = after.y_m - before.y_m, chord = std::hypot(dx, dy);
        frames[k].p = {here.x_m, here.y_m};
        frames[k].n = {-dy / chord, dx / chord};
        frames[k].s = here.s_m;
        frames[k].left = track.left_edge_m.empty() ? track.width_m / 2 : track.left_edge_m[k];
        frames[k].right = track.right_edge_m.empty() ? track.width_m / 2 : track.right_edge_m[k];
        const double ax = here.x_m - before.x_m, ay = here.y_m - before.y_m, bx = after.x_m - here.x_m, by = after.y_m - here.y_m;
        const double product = std::hypot(ax, ay) * std::hypot(bx, by) * chord;
        raw[k] = product > 1e-12 ? 2 * (ax * by - ay * bx) / product : 0;
    }
    for (std::size_t k = 0; k < n; ++k) frames[k].curvature = (raw[(k + n - 1) % n] + raw[k] + raw[(k + 1) % n]) / 3;
    return frames;
}

// A point beside the track: `offset` to the left of the reference, `along` further down it, at a height.
P3 at(const Frame& f, double offset, double height, double along = 0) {
    const double x = f.p.x + f.n.x * offset + f.n.y * along, y = f.p.y + f.n.y * offset - f.n.x * along;
    return {x, height, -y};
}
// How far left of the reference lies a point `distance` beyond the edge on a side (+1 left, -1 right). Beyond the inside
// edge of a tight bend the normals meet at the bend's centre, so nothing is placed past it.
double beyond(const Frame& f, int side, double distance) {
    const double edge = side > 0 ? f.left : f.right;
    double offset = edge + distance;
    if (distance > 0 && f.curvature * side > 1e-6) offset = std::min(offset, std::max(edge, 1 / std::abs(f.curvature) - 0.03));
    return side * offset;
}

double wrapped(double value, double length) {
    const double v = std::fmod(value, length);
    return v < 0 ? v + length : v;
}

// The bend a station lies in: which way it turns, which of the corners it is, how far into it and how far from its end.
struct Bend { int direction{}; int index{-1}; double into_m{}, remaining_m{}; };
Bend bend_at(const std::vector<CornerSpan>& corners, double s, double length) {
    for (std::size_t k = 0; k < corners.size(); ++k) {
        const double span = wrapped(corners[k].to_s_m - corners[k].from_s_m, length);
        const double into = wrapped(s - corners[k].from_s_m, length);
        if (into <= span) return {corners[k].direction, static_cast<int>(k), into, span - into};
    }
    return {};
}

bool is_gokartcentralen(const fd::Track& track, const std::vector<CornerSpan>& corners) {
    return corners.size() == 10 && std::abs(track.length_m - 400.0) < 1.0 && track.name.find("Gokartcentralen") != std::string::npos;
}

double half_width(const fd::Track& track) {
    double half = track.width_m / 2;
    for (const double edge : track.left_edge_m) half = std::max(half, edge);
    for (const double edge : track.right_edge_m) half = std::max(half, edge);
    return half;
}

// ---- Barriers ----------------------------------------------------------------------------------------------------------

constexpr double tube_face_m = fd::KartEnvironment::barrier_face_m; // shared with solid contact
constexpr double tube_radius_m = 0.115;
constexpr double barrier_top_m = 0.46;
constexpr double barrier_back_m = tube_face_m + 2 * tube_radius_m;
// Gokartcentralen's fence stands on the island the ninth corner wraps round: slats round the corner and a plain banner along
// the stretch after it, the barrier beneath red-rimmed all the way. It begins where the bend tightens, 10 m into the bend as
// the coach marks it (a negative lead), and ends this far past it.
constexpr double fence_lead_m = -10.0, fence_tail_m = 14.0;

// One length of a barrier tube between two samples: an eight-sided tube whose faces are shaded as light from above leaves
// them, brightest on its upper track-side face.
void tube(std::vector<V>& out, const Frame& fi, const Frame& fj, int side, double centre_height, const QColor& base) {
    static const std::array<double, 7> shade{0.40, 0.70, 1.00, 1.22, 0.80, 0.50, 0.36};
    static const std::array<double, 7> gleam{0.00, 0.00, 0.04, 0.10, 0.00, 0.00, 0.00};
    const auto corner = [&](const Frame& f, int k) {
        const double angle = (-67.5 + 45.0 * k) * std::numbers::pi / 180;
        return at(f, beyond(f, side, tube_face_m + tube_radius_m - tube_radius_m * std::cos(angle)),
                  centre_height + tube_radius_m * std::sin(angle));
    };
    for (int k = 0; k < 7; ++k) {
        const QColor colour = scaled(base, shade[static_cast<std::size_t>(k)], gleam[static_cast<std::size_t>(k)]);
        quad(out, corner(fi, k), corner(fi, k + 1), corner(fj, k + 1), corner(fj, k), colour);
    }
}

void build_barriers(const fd::Track& track, const std::vector<CornerSpan>& corners, const fd::TrackRelief* relief, std::vector<V>& out) {
    const auto frames = frames_of(track);
    const std::size_t n = frames.size();
    const bool own = is_gokartcentralen(track, corners);
    const QColor black("#26282c"), red("#e0262a"), white("#e4e5e8"), blue("#2a62d8"), brace("#3ad23c"), brace_face("#63e860");
    for (std::size_t i = 0; i < n; ++i) {
        const Frame& fi = frames[i];
        const Frame& fj = frames[(i + 1) % n];
        const Bend bend = bend_at(corners, fi.s, track.length_m);
        for (const int side : {1, -1}) {
            const bool inner = bend.direction == side, outer = bend.direction == -side;
            QColor lower = black, upper = inner ? red : black;
            // The fourth corner's outside is a wall of red, white and black blocks; the last corner's outside has a blue rim.
            if (own && outer && bend.index == 3) lower = upper = std::array<QColor, 3>{red, white, black}[static_cast<std::size_t>(fi.s / 1.3) % 3];
            if (own && outer && bend.index == 9) upper = blue;
            if (own && side == corners[8].direction &&
                wrapped(fi.s - (corners[8].from_s_m - fence_lead_m), track.length_m) <
                    wrapped(corners[8].to_s_m - corners[8].from_s_m, track.length_m) + fence_lead_m + fence_tail_m)
                upper = red;
            tube(out, fi, fj, side, tube_radius_m, lower);
            tube(out, fi, fj, side, barrier_top_m - tube_radius_m, upper);
            // Green braces lean on the island's side of a red-rimmed barrier, every other metre.
            if (inner && i % 2 == 0) {
                const double foot = std::abs(beyond(fi, side, 0.62)) - (side > 0 ? fi.left : fi.right);
                if (foot > 0.5) {
                    const double top = side * ((side > 0 ? fi.left : fi.right) + barrier_back_m), base = beyond(fi, side, 0.62);
                    for (const double along : {-0.035, 0.035})
                        tri(out, at(fi, top, 0.44, along), at(fi, top, 0.03, along), at(fi, base, 0.03, along), brace, brace, brace);
                    quad(out, at(fi, top, 0.44, -0.035), at(fi, top, 0.44, 0.035), at(fi, base, 0.03, 0.035), at(fi, base, 0.03, -0.035), brace_face);
                }
            }
            // Where the bend is banked its raised edge stands on a retaining wall: the ground falls away just behind the barrier.
            if (relief && (relief->tilt_at(fi.s) != 0 || relief->tilt_at(fj.s) != 0)) {
                const QColor wall("#0b0c0e");
                quad(out, at(fi, beyond(fi, side, 0.30), 0), at(fi, beyond(fi, side, 0.345), 0), at(fj, beyond(fj, side, 0.345), 0),
                     at(fj, beyond(fj, side, 0.30), 0), wall);
            }
        }
    }
}

// ---- Floor -------------------------------------------------------------------------------------------------------------

constexpr double island_reach_m = 6.5;   // green floor reaches this far from the inside barrier of a bend
constexpr double island_fade_m = 1.2;    // and fades into the bare floor over its last stretch
constexpr double barrier_foot_m = 0.34;  // nothing of the floor is laid nearer the corridor's edge than this: the barrier stands there

fd::Vec2 map_point(const Frame& f, double offset) { return {f.p.x + f.n.x * offset, f.p.y + f.n.y * offset}; }

// How green the floor is at a point of the map, 0 to 1: the islands are the floor behind the inside barrier of each bend,
// as far as island_reach_m from it, wherever nothing of the road lies between.
struct Islands {
    const fd::Track& track;
    double half{};
    std::vector<fd::Vec2> barriers;  // the island side of every inside barrier
    double at(fd::Vec2 q) const {
        double least = island_reach_m * island_reach_m;
        const fd::Vec2* nearest = nullptr;
        for (const auto& b : barriers) {
            const double dx = b.x - q.x, dy = b.y - q.y, squared = dx * dx + dy * dy;
            if (squared < least) { least = squared; nearest = &b; }
        }
        if (!nearest) return 0;
        for (const double share : {0.25, 0.5, 0.75})
            if (fd::project(track, {q.x + (nearest->x - q.x) * share, q.y + (nearest->y - q.y) * share}).distance_m < half) return 0;
        const double fade = std::clamp((island_reach_m - std::sqrt(least)) / island_fade_m, 0.0, 1.0);
        return fade * fade * (3 - 2 * fade);
    }
};

QColor mixed(const QColor& a, const QColor& b, double share) {
    return QColor::fromRgbF(float(a.redF() + (b.redF() - a.redF()) * share), float(a.greenF() + (b.greenF() - a.greenF()) * share),
                            float(a.blueF() + (b.blueF() - a.blueF()) * share));
}

// A digit of the start grid as seven painted strokes, in a frame's own coordinates: u to the driver's right, v ahead.
void digit(std::vector<V>& out, const Frame& f, int value, double centre_u, double foot_v, const QColor& paint) {
    static const std::array<int, 10> strokes{0b0111111, 0b0000110, 0b1011011, 0b1001111, 0b1100110,
                                             0b1101101, 0b1111101, 0b0000111, 0b1111111, 0b1101111};
    constexpr double w = 0.24, h = 0.42, t = 0.09, y = 0.022;  // half the width, half the height, a stroke's width
    const auto bar = [&](double u0, double v0, double u1, double v1) {
        quad(out, at(f, -(centre_u + u0), y, foot_v + v0), at(f, -(centre_u + u1), y, foot_v + v0), at(f, -(centre_u + u1), y, foot_v + v1),
             at(f, -(centre_u + u0), y, foot_v + v1), paint);
    };
    const int lit = strokes[static_cast<std::size_t>(std::clamp(value, 0, 9))];
    if (lit & 1) bar(-w, 2 * h - t, w, 2 * h);            // top
    if (lit & 2) bar(w - t, h, w, 2 * h);                  // upper right
    if (lit & 4) bar(w - t, 0, w, h);                      // lower right
    if (lit & 8) bar(-w, 0, w, t);                         // bottom
    if (lit & 16) bar(-w, 0, -w + t, h);                   // lower left
    if (lit & 32) bar(-w, h, -w + t, 2 * h);               // upper left
    if (lit & 64) bar(-w, h - t / 2, w, h + t / 2);        // middle
}

void build_floor(const fd::Track& track, const std::vector<CornerSpan>& corners, std::vector<V>& out) {
    const auto frames = frames_of(track);
    const std::size_t n = frames.size();
    const auto layout = kart_hall_layout(track);
    const double half = half_width(track);
    const bool own = is_gokartcentralen(track, corners);

    std::vector<Bend> bends(n);
    Islands islands{track, half, {}};
    for (std::size_t i = 0; i < n; ++i) {
        bends[i] = bend_at(corners, frames[i].s, track.length_m);
        if (bends[i].direction != 0) islands.barriers.push_back(map_point(frames[i], beyond(frames[i], bends[i].direction, barrier_back_m)));
    }
    const QColor concrete("#0b0c0e"), island("#18976a");
    const auto floor_colour = [&](fd::Vec2 q) { return mixed(concrete, island, islands.at(q)); };

    // The hall's floor away from the road, in half-metre squares coloured at their corners, so an island fades into the bare
    // floor without a step. No square is laid where a barrier stands.
    constexpr double cell = 0.5;
    const int columns = static_cast<int>(std::ceil((layout.max_x - layout.min_x) / cell)), rows = static_cast<int>(std::ceil((layout.max_z - layout.min_z) / cell));
    const auto index = [&](int column, int row) { return static_cast<std::size_t>(row) * static_cast<std::size_t>(columns + 1) + static_cast<std::size_t>(column); };
    std::vector<char> blocked(static_cast<std::size_t>(columns + 1) * static_cast<std::size_t>(rows + 1));
    std::vector<QColor> colour(blocked.size());
    const auto corner_x = [&](int column) { return std::min(layout.min_x + column * cell, layout.max_x); };
    const auto corner_z = [&](int row) { return std::min(layout.min_z + row * cell, layout.max_z); };
    for (int row = 0; row <= rows; ++row)
        for (int column = 0; column <= columns; ++column) {
            const fd::Vec2 q{corner_x(column), -corner_z(row)};
            blocked[index(column, row)] = fd::project(track, q).distance_m < half + barrier_foot_m;
            if (!blocked[index(column, row)]) colour[index(column, row)] = floor_colour(q);
        }
    for (int row = 0; row < rows; ++row)
        for (int column = 0; column < columns; ++column) {
            if (blocked[index(column, row)] || blocked[index(column + 1, row)] || blocked[index(column + 1, row + 1)] || blocked[index(column, row + 1)]) continue;
            quad(out, {corner_x(column), 0.012, corner_z(row)}, {corner_x(column + 1), 0.012, corner_z(row)}, {corner_x(column + 1), 0.012, corner_z(row + 1)},
                 {corner_x(column), 0.012, corner_z(row + 1)}, colour[index(column, row)], colour[index(column + 1, row)], colour[index(column + 1, row + 1)],
                 colour[index(column, row + 1)]);
        }
    // Behind every barrier a strip of the same floor follows the edge, so the squares' stepped border is never seen; it stops
    // short of whatever other part of the track lies behind.
    std::array<std::vector<double>, 2> reach;
    for (const int side : {1, -1}) {
        auto& far = reach[side > 0 ? 0 : 1];
        far.assign(n, 1.3);
        for (std::size_t i = 0; i < n; ++i) {
            const double edge = side > 0 ? frames[i].left : frames[i].right;
            for (const double d : {0.6, 0.85, 1.1, 1.35})
                if (fd::project(track, map_point(frames[i], side * (edge + d))).distance_m < half + barrier_foot_m) { far[i] = d - 0.3; break; }
        }
    }
    for (std::size_t i = 0; i < n; ++i)
        for (const int side : {1, -1}) {
            const std::size_t j = (i + 1) % n;
            const auto& far = reach[side > 0 ? 0 : 1];
            if (far[i] <= barrier_foot_m + 0.05 || far[j] <= barrier_foot_m + 0.05) continue;
            const double near_i = beyond(frames[i], side, barrier_foot_m), near_j = beyond(frames[j], side, barrier_foot_m);
            const double far_i = beyond(frames[i], side, far[i]), far_j = beyond(frames[j], side, far[j]);
            // At the hub of a hairpin the barrier's own footing reaches the bend's centre and there is no floor to lay.
            const double edge_i = side > 0 ? frames[i].left : frames[i].right, edge_j = side > 0 ? frames[j].left : frames[j].right;
            if (std::abs(near_i) < edge_i + barrier_foot_m - 1e-9 || std::abs(near_j) < edge_j + barrier_foot_m - 1e-9 ||
                (std::abs(far_i) - std::abs(near_i) < 0.05 && std::abs(far_j) - std::abs(near_j) < 0.05)) continue;
            quad(out, at(frames[i], near_i, 0.014), at(frames[i], far_i, 0.014), at(frames[j], far_j, 0.014), at(frames[j], near_j, 0.014),
                 floor_colour(map_point(frames[i], near_i)), floor_colour(map_point(frames[i], far_i)), floor_colour(map_point(frames[j], far_j)),
                 floor_colour(map_point(frames[j], near_j)));
        }

    // Paint on the road's own edges: a red and white kerb inside each bend, a white line and a red band everywhere else.
    const QColor kerb_red("#c9282a"), kerb_white("#d3d4d8"), line_white("#c4c6ca"), line_red("#a2171b");
    const auto strip = [&](std::size_t i, int side, double from, double to, const QColor& colour) {
        const std::size_t j = (i + 1) % n;
        quad(out, at(frames[i], beyond(frames[i], side, from), 0.02), at(frames[i], beyond(frames[i], side, to), 0.02),
             at(frames[j], beyond(frames[j], side, to), 0.02), at(frames[j], beyond(frames[j], side, from), 0.02), colour);
    };
    for (std::size_t i = 0; i < n; ++i)
        for (const int side : {1, -1}) {
            if (bends[i].direction == side) {
                strip(i, side, -0.62, tube_face_m, static_cast<int>(bends[i].into_m / 1.1) % 2 == 0 ? kerb_red : kerb_white);
                strip(i, side, -0.72, -0.62, kerb_white);
            } else {
                strip(i, side, -0.20, tube_face_m, line_red);
                strip(i, side, -0.32, -0.20, line_white);
            }
        }

    // The start grid: eight boxes behind the line, staggered, each a painted bracket with its number inside.
    if (own) {
        const QColor paint("#b9bcc0");
        for (int place = 1; place <= 8; ++place) {
            const double s = wrapped(-2.4 - 3.4 * (place - 1), track.length_m);
            std::size_t nearest = 0;
            for (std::size_t k = 0; k < n; ++k)
                if (std::abs(frames[k].s - s) < std::abs(frames[nearest].s - s)) nearest = k;
            const Frame& f = frames[nearest];
            const double u = place % 2 ? -1.25 : 1.25;  // to the driver's left for odd places
            const auto bar = [&](double u0, double v0, double u1, double v1) {
                quad(out, at(f, -(u + u0), 0.022, v0), at(f, -(u + u1), 0.022, v0), at(f, -(u + u1), 0.022, v1), at(f, -(u + u0), 0.022, v1), paint);
            };
            bar(-0.8, 1.15, 0.8, 1.25);
            bar(-0.8, 0.65, -0.7, 1.15);
            bar(0.7, 0.65, 0.8, 1.15);
            digit(out, f, place, u, -0.1, paint);
        }
    }
}

// ---- Shell -------------------------------------------------------------------------------------------------------------

// Every pair of neighbouring samples whose first lies within a stretch of the lap, which may run through the start line.
template <typename Visit> void along_stretch(const std::vector<Frame>& frames, double from_s, double to_s, double length, Visit visit) {
    const double span = wrapped(to_s - from_s, length);
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const double into = wrapped(frames[i].s - from_s, length);
        if (into < span) visit(frames[i], frames[(i + 1) % frames.size()], into);
    }
}

void build_shell(const fd::Track& track, const std::vector<CornerSpan>& corners, std::vector<V>& out) {
    const auto frames = frames_of(track);
    const auto hall = kart_hall_layout(track);
    const double half = half_width(track);
    const double centre_x = (hall.min_x + hall.max_x) / 2;

    // The roof: two slopes of sheet metal meeting at a ridge along the hall, in strips, on rafters every six metres.
    const QColor sheet_a("#1c2122"), sheet_b("#191e1f"), rafter("#2c3335");
    int strip = 0;
    for (double z = hall.min_z; z < hall.max_z; z += 1.2, ++strip) {
        const double z1 = std::min(z + 1.2, hall.max_z);
        const QColor& colour = strip % 2 ? sheet_a : sheet_b;
        quad(out, {hall.min_x, hall.eave_m, z}, {centre_x, hall.ridge_m, z}, {centre_x, hall.ridge_m, z1}, {hall.min_x, hall.eave_m, z1}, colour);
        quad(out, {hall.max_x, hall.eave_m, z}, {centre_x, hall.ridge_m, z}, {centre_x, hall.ridge_m, z1}, {hall.max_x, hall.eave_m, z1}, colour);
    }
    for (double z = hall.min_z + 3.0; z < hall.max_z; z += 6.0)
        for (const double eave_x : {hall.min_x, hall.max_x})
            quad(out, {eave_x, hall.eave_m, z}, {centre_x, hall.ridge_m, z}, {centre_x, hall.ridge_m - 0.5, z}, {eave_x, hall.eave_m - 0.5, z}, rafter);

    // The walls, lit from the lamps' height down: dark along the west, the north and the south, red along the east.
    const QColor dark_low("#141518"), dark_high("#08090a"), red_low("#9a1417"), red_high("#3a0809");
    quad(out, {hall.min_x, 0, hall.min_z}, {hall.min_x, 0, hall.max_z}, {hall.min_x, hall.eave_m, hall.max_z}, {hall.min_x, hall.eave_m, hall.min_z},
         dark_low, dark_low, dark_high, dark_high);
    quad(out, {hall.max_x, 0, hall.min_z}, {hall.max_x, 0, hall.max_z}, {hall.max_x, hall.eave_m, hall.max_z}, {hall.max_x, hall.eave_m, hall.min_z},
         red_low, red_low, red_high, red_high);
    for (const double z : {hall.min_z, hall.max_z}) {
        quad(out, {hall.min_x, 0, z}, {hall.max_x, 0, z}, {hall.max_x, hall.eave_m, z}, {hall.min_x, hall.eave_m, z}, dark_low, dark_low, dark_high, dark_high);
        tri(out, {hall.min_x, hall.eave_m, z}, {hall.max_x, hall.eave_m, z}, {centre_x, hall.ridge_m, z}, dark_high, dark_high, dark_high);
    }
    // A spectators' balcony along the red wall: the edge of its deck, its posts and a yellow rail.
    {
        const double x = hall.max_x - 0.05;
        quad(out, {x, 2.7, hall.min_z}, {x, 2.7, hall.max_z}, {x, 3.1, hall.max_z}, {x, 3.1, hall.min_z}, QColor("#14090a"));
        quad(out, {x, 4.0, hall.min_z}, {x, 4.0, hall.max_z}, {x, 4.07, hall.max_z}, {x, 4.07, hall.min_z}, QColor("#d8c42c"));
        for (double z = hall.min_z + 2.0; z < hall.max_z; z += 4.0)
            quad(out, {x, 3.1, z}, {x, 3.1, z + 0.12}, {x, 4.0, z + 0.12}, {x, 4.0, z}, QColor("#1c0c0d"));
    }

    // The timing board at the far end of the west straight, with the painted wedges beside it: rows of coloured bars, no lettering.
    {
        const double z = hall.min_z + 0.06, x0 = hall.min_x + 2.6, x1 = x0 + 4.6, y0 = 2.5, y1 = 5.0;
        quad(out, {x0 - 0.12, y0 - 0.12, z}, {x1 + 0.12, y0 - 0.12, z}, {x1 + 0.12, y1 + 0.12, z}, {x0 - 0.12, y1 + 0.12, z}, QColor("#1c1e24"));
        quad(out, {x0, y0, z + 0.01}, {x1, y0, z + 0.01}, {x1, y1, z + 0.01}, {x0, y1, z + 0.01}, QColor("#05060a"));
        const std::array<QColor, 4> lit{QColor("#e9edf2"), QColor("#58e07a"), QColor("#d86be6"), QColor("#f1d04a")};
        quad(out, {x0 + 0.15, y1 - 0.3, z + 0.02}, {x1 - 0.15, y1 - 0.3, z + 0.02}, {x1 - 0.15, y1 - 0.12, z + 0.02}, {x0 + 0.15, y1 - 0.12, z + 0.02}, lit[3]);
        for (int row = 0; row < 9; ++row) {
            const double top = y1 - 0.42 - 0.23 * row, bottom = top - 0.15;
            const auto cell = [&](double from, double to, const QColor& colour) {
                quad(out, {x0 + from, bottom, z + 0.02}, {x0 + to, bottom, z + 0.02}, {x0 + to, top, z + 0.02}, {x0 + from, top, z + 0.02}, colour);
            };
            cell(0.15, 0.4, lit[0]);
            cell(0.55, 2.2 - 0.12 * ((row * 5) % 4), QColor("#aeb6c2"));
            cell(2.5, 3.3, lit[static_cast<std::size_t>(1 + (row * 3) % 2)]);
            cell(3.5, 4.4, lit[static_cast<std::size_t>((row * 7) % 3)]);
        }
        const QColor yellow("#e6d02f"), green("#35c46a"), pink_low("#6d2c78"), pink_high("#a843b8");
        quad(out, {x0 - 1.7, 2.7, z}, {x0 - 0.4, 2.7, z}, {x0 - 0.4, 4.6, z}, {x0 - 1.7, 4.6, z}, pink_low, pink_low, pink_high, pink_high);
        tri(out, {x1 + 0.5, 5.0, z}, {x1 + 0.5, 3.1, z}, {x1 + 3.4, 5.0, z}, yellow, yellow, yellow);
        tri(out, {x1 + 0.9, 2.9, z}, {x1 + 7.5, 4.6, z}, {x1 + 7.5, 5.0, z}, green, green, green);
    }
    // A long yellow wedge and two thin rays on the west wall past the first corner, and arches along the south wall.
    {
        const double x = hall.min_x + 0.06, z0 = hall.max_z - 9.0, z1 = hall.max_z - 30.0;
        const QColor yellow("#c9b526");
        tri(out, {x, 2.0, z0}, {x, 2.0, z1}, {x, 3.3, z0}, yellow, yellow, yellow);
        quad(out, {x, 4.3, z0}, {x, 4.42, z0}, {x, 5.9, z1}, {x, 5.78, z1}, yellow);
        quad(out, {x, 4.9, z0}, {x, 5.0, z0}, {x, 5.95, z1 + 6}, {x, 5.85, z1 + 6}, QColor("#35c46a"));
        const QColor recess("#1c1d21");
        for (int arch = 0; arch < 4; ++arch) {
            const double ax = hall.min_x + 5.0 + 4.2 * arch, z = hall.max_z - 0.06, w = 1.3, spring = 2.3;
            quad(out, {ax - w, 0, z}, {ax + w, 0, z}, {ax + w, spring, z}, {ax - w, spring, z}, recess);
            for (int k = 0; k < 8; ++k) {
                const double a0 = std::numbers::pi * k / 8, a1 = std::numbers::pi * (k + 1) / 8;
                tri(out, {ax, spring, z}, {ax + w * std::cos(a0), spring + w * std::sin(a0), z}, {ax + w * std::cos(a1), spring + w * std::sin(a1), z},
                    recess, recess, recess);
            }
        }
    }

    if (!is_gokartcentralen(track, corners)) return;
    const double length = track.length_m;

    // The gallery along the outside of the start straight: a red wall with a band of dark glass, then plain red with a door.
    {
        const double from_s = length - 34.0, to_s = 18.0, stand = 1.3, top = 4.6;
        const QColor red_foot("#c9181c"), red_sill("#d81e22"), red_head("#c2171b"), red_top("#6e0d10"), glass_low("#0a0d14"), glass_high("#111724");
        const Frame* first = nullptr;
        const Frame* last = nullptr;
        double first_into = 1e9, last_into = -1;
        along_stretch(frames, from_s, to_s, length, [&](const Frame& fi, const Frame& fj, double into) {
            if (into < first_into) { first_into = into; first = &fi; }
            if (into > last_into) { last_into = into; last = &fj; }
            const double oi = beyond(fi, 1, stand), oj = beyond(fj, 1, stand);
            const auto band = [&](double y0, double y1, const QColor& low, const QColor& high, double proud = 0) {
                quad(out, at(fi, oi - proud, y0), at(fj, oj - proud, y0), at(fj, oj - proud, y1), at(fi, oi - proud, y1), low, low, high, high);
            };
            band(0, 1.05, red_foot, red_sill);
            const bool glazed = into < 27.0;
            if (glazed) band(1.05, 2.75, glass_low, glass_high); else band(1.05, 2.75, red_sill, red_head);
            band(2.75, top, red_head, red_top);
            const int metre = static_cast<int>(into);
            if (glazed && metre % 3 == 0)  // a mullion
                quad(out, at(fi, oi - 0.03, 1.05), at(fi, oi - 0.03, 1.05, 0.16), at(fi, oi - 0.03, 2.75, 0.16), at(fi, oi - 0.03, 2.75), red_head);
            if (glazed && metre % 2 == 1)  // a light inside
                quad(out, at(fi, oi - 0.02, 2.5, 0.4), at(fi, oi - 0.02, 2.5, 0.56), at(fi, oi - 0.02, 2.56, 0.56), at(fi, oi - 0.02, 2.56, 0.4), QColor("#6f7fa6"));
            if (metre == 44) {  // the door, and a sign beside it
                quad(out, at(fi, oi - 0.03, 0, 0.02), at(fi, oi - 0.03, 0, 0.95), at(fi, oi - 0.03, 2.05, 0.95), at(fi, oi - 0.03, 2.05, 0.02), QColor("#7b1013"));
                quad(out, at(fi, oi - 0.03, 1.55, -0.6), at(fi, oi - 0.03, 1.55, -0.25), at(fi, oi - 0.03, 1.75, -0.25), at(fi, oi - 0.03, 1.75, -0.6), QColor("#e6e7ea"));
            }
        });
        // Its two ends turn away from the track and run to the hall's own walls.
        for (const Frame* end : {first, last})
            if (end) {
                const double o = beyond(*end, 1, stand);
                const fd::Vec2 from = map_point(*end, o);
                double run = 16.0;
                if (end->n.x > 1e-9) run = std::min(run, (hall.max_x - from.x) / end->n.x);
                if (end->n.x < -1e-9) run = std::min(run, (hall.min_x - from.x) / end->n.x);
                if (end->n.y > 1e-9) run = std::min(run, (-hall.min_z - from.y) / end->n.y);
                if (end->n.y < -1e-9) run = std::min(run, (-hall.max_z - from.y) / end->n.y);
                run = std::max(0.0, run);
                const QColor shade_low("#8f1215"), shade_high("#3d090b");
                quad(out, at(*end, o, 0), at(*end, o + run, 0), at(*end, o + run, top), at(*end, o, top), shade_low, shade_low, shade_high, shade_high);
            }
    }

    // The footbridge over the start straight, a little before the grid: a red deck with dark sides and a red rail.
    {
        const double s = length - 22.0;
        const Frame* f = &frames.front();
        for (const auto& candidate : frames)
            if (std::abs(candidate.s - s) < std::abs(f->s - s)) f = &candidate;
        const double outer = beyond(*f, 1, 1.3), inner = beyond(*f, -1, 1.9), w = 1.1, low = 2.75, deck = 3.05, rail = 4.05;
        const QColor under("#4b0b0d"), face("#c21a1e"), walk("#8a1215"), mesh("#17181b");
        quad(out, at(*f, outer, low, -w), at(*f, inner, low, -w), at(*f, inner, low, w), at(*f, outer, low, w), under);
        quad(out, at(*f, outer, deck, -w), at(*f, inner, deck, -w), at(*f, inner, deck, w), at(*f, outer, deck, w), walk);
        for (const double along : {-w, w}) {
            quad(out, at(*f, outer, low, along), at(*f, inner, low, along), at(*f, inner, deck, along), at(*f, outer, deck, along), face);
            quad(out, at(*f, outer, deck, along), at(*f, inner, deck, along), at(*f, inner, rail, along), at(*f, outer, rail, along), mesh);
            quad(out, at(*f, outer, rail, along), at(*f, inner, rail, along), at(*f, inner, rail + 0.09, along), at(*f, outer, rail + 0.09, along), face);
        }
        // The column that carries its inner end.
        for (const double along : {-0.2, 0.2})
            quad(out, at(*f, inner - 0.2, 0, along), at(*f, inner + 0.2, 0, along), at(*f, inner + 0.2, low, along), at(*f, inner - 0.2, low, along), face);
        for (const double across : {-0.2, 0.2})
            quad(out, at(*f, inner + across, 0, -0.2), at(*f, inner + across, 0, 0.2), at(*f, inner + across, low, 0.2), at(*f, inner + across, low, -0.2), walk);
    }

    // The middle of a bend, where its radii meet, and how far that is from the middle of the road; nothing if the bend is too
    // gentle to have one or the place is not clear of the road.
    struct Middle { bool found{}; double x{}, y{}, radius{}; };
    const auto middle_of = [&](int index) {
        const auto& corner = corners[static_cast<std::size_t>(index)];
        Middle middle;
        int count = 0;
        along_stretch(frames, corner.from_s_m, corner.to_s_m, length, [&](const Frame& fi, const Frame&, double) {
            if (fi.curvature * corner.direction < 1.0 / 12) return;
            const double radius = 1 / std::abs(fi.curvature);
            middle.x += fi.p.x + fi.n.x * corner.direction * radius;
            middle.y += fi.p.y + fi.n.y * corner.direction * radius;
            middle.radius += radius;
            ++count;
        });
        if (count == 0) return middle;
        middle.x /= count;
        middle.y /= count;
        middle.radius /= count;
        middle.found = fd::project(track, {middle.x, middle.y}).distance_m >= half + 0.9;
        return middle;
    };

    // A pillar in the island of the fourth corner and of the last, at the middle of the bend, padded at its foot.
    for (const int index : {3, 9}) {
        const auto middle = middle_of(index);
        if (!middle.found) continue;
        const double x = middle.x, z = -middle.y, roof = hall.roof_at(x);
        box(out, {x - 0.35, 0, z - 0.35}, {x + 0.35, roof, z + 0.35}, QColor("#1b1c1f"), QColor("#141518"), QColor("#141518"), QColor("#141518"));
        box(out, {x - 0.42, 0, z - 0.42}, {x + 0.42, 1.4, z + 0.42}, QColor("#2e3035"), QColor("#25272b"), QColor("#3a3c41"), QColor("#25272b"));
    }

    // A round red deck over the island the ninth corner wraps round, on one column, with small violet lights under its rim.
    if (const auto middle = middle_of(8); middle.found) {
        const double reach = std::min(4.6, middle.radius - half - 0.9), x = middle.x, z = -middle.y;
        if (reach > 1.5) {
            constexpr int sides = 28;
            constexpr double low = 3.1, high = 4.1;
            const QColor fascia_low("#7c1013"), fascia_high("#b5181b"), under("#1a0809"), light("#b26cf0");
            for (int k = 0; k < sides; ++k) {
                const double a0 = 2 * std::numbers::pi * k / sides, a1 = 2 * std::numbers::pi * (k + 1) / sides;
                const P3 p0{x + reach * std::cos(a0), low, z + reach * std::sin(a0)}, p1{x + reach * std::cos(a1), low, z + reach * std::sin(a1)};
                quad(out, p0, p1, {p1.x, high, p1.z}, {p0.x, high, p0.z}, fascia_low, fascia_low, fascia_high, fascia_high);
                tri(out, {x, low, z}, p0, p1, under, under, under);
                if (k % 2 == 0) {
                    const double r0 = reach - 0.35, r1 = reach - 0.15, am = (a0 + a1) / 2, w = 0.06;
                    quad(out, {x + r0 * std::cos(am - w), low - 0.02, z + r0 * std::sin(am - w)}, {x + r1 * std::cos(am - w), low - 0.02, z + r1 * std::sin(am - w)},
                         {x + r1 * std::cos(am + w), low - 0.02, z + r1 * std::sin(am + w)}, {x + r0 * std::cos(am + w), low - 0.02, z + r0 * std::sin(am + w)}, light);
                }
            }
            box(out, {x - 0.25, 0, z - 0.25}, {x + 0.25, low, z + 0.25}, QColor("#8a1215"), QColor("#6a0e11"), under, under);
        }
    }

    // The slatted green fence on the island the ninth corner wraps round, and after the corner a plain dark banner under a
    // green rail.
    {
        const auto& corner = corners[8];
        const int side = corner.direction;
        const double from_s = wrapped(corner.from_s_m - fence_lead_m, length), to_s = wrapped(corner.to_s_m + fence_tail_m, length);
        const double span = wrapped(to_s - from_s, length);
        const QColor slat_a("#7ad62a"), slat_b("#93e63c"), post("#101113"), banner("#121316");
        along_stretch(frames, from_s, to_s, length, [&](const Frame& fi, const Frame& fj, double into) {
            const double oi = beyond(fi, side, 0.36), oj = beyond(fj, side, 0.36);
            if (std::abs(oi) - (side > 0 ? fi.left : fi.right) < 0.3 || std::abs(oj) - (side > 0 ? fj.left : fj.right) < 0.3) return;
            if (into < span - fence_tail_m - 4.0) {
                for (int k = 0; k < 8; ++k) {
                    const double y = 0.58 + 0.15 * k;
                    quad(out, at(fi, oi, y), at(fj, oj, y), at(fj, oj, y + 0.06), at(fi, oi, y + 0.06), k % 2 ? slat_a : slat_b);
                }
            } else {
                quad(out, at(fi, oi, 0.5), at(fj, oj, 0.5), at(fj, oj, 1.45), at(fi, oi, 1.45), banner);
                quad(out, at(fi, oi, 1.45), at(fj, oj, 1.45), at(fj, oj, 1.53), at(fi, oi, 1.53), slat_a);
            }
            if (static_cast<int>(into) % 3 == 0) {
                const double front = oi - side * 0.03;
                quad(out, at(fi, front, barrier_top_m), at(fi, front, barrier_top_m, 0.07), at(fi, front, 1.9, 0.07), at(fi, front, 1.9), post);
            }
        });
    }
}

// ---- Wear --------------------------------------------------------------------------------------------------------------

// The banked ninth corner is scuffed pale across its middle and its outside, where the karts load their outer tires: a faint
// band laid over the road, fading in and out along the bend and across it.
void build_wear(const fd::Track& track, const std::vector<CornerSpan>& corners, std::vector<V>& out) {
    if (!is_gokartcentralen(track, corners)) return;
    const auto frames = frames_of(track);
    const auto& corner = corners[8];
    const double span = wrapped(corner.to_s_m - corner.from_s_m, track.length_m);
    const QColor dust("#cdd1d6");
    // Across the road from the inside toward the outside: nothing, full, full, nothing.
    const std::array<double, 4> across{-0.9, 0.2, 1.6, 2.5};
    const std::array<double, 4> weight{0.0, 1.0, 1.0, 0.0};
    along_stretch(frames, corner.from_s_m, corner.to_s_m, track.length_m, [&](const Frame& fi, const Frame& fj, double into) {
        const auto strength = [&](double at_m) {
            const double edge = std::clamp(std::min(at_m, span - at_m) / 6.0, 0.0, 1.0);
            return 0.2 * edge * edge * (3 - 2 * edge);
        };
        const double ai = strength(into), aj = strength(std::min(span, into + wrapped(fj.s - fi.s, track.length_m)));
        for (std::size_t k = 0; k + 1 < across.size(); ++k) {
            // Offsets are to the left; the outside of this bend is on the other side of it.
            const double o0 = -corner.direction * across[k], o1 = -corner.direction * across[k + 1];
            tri(out, at(fi, o0, 0.016), at(fi, o1, 0.016), at(fj, o1, 0.016), dust, dust, dust, ai * weight[k], ai * weight[k + 1], aj * weight[k + 1]);
            tri(out, at(fi, o0, 0.016), at(fj, o1, 0.016), at(fj, o0, 0.016), dust, dust, dust, ai * weight[k], aj * weight[k + 1], aj * weight[k]);
        }
    });
}

// ---- Lamps -------------------------------------------------------------------------------------------------------------

void build_lamps(const fd::Track& track, bool glow, std::vector<V>& out) {
    const auto hall = kart_hall_layout(track);
    constexpr int sides = 10;
    const QColor white("#ffffff"), haze("#dfe8ff"), fixture("#c8d2e2");
    for (int column = 0; column < hall.lamp_columns; ++column)
        for (int row = 0; row < hall.lamp_rows; ++row) {
            const double x = hall.lamp_x0 + column * hall.lamp_dx, z = hall.lamp_z0 + row * hall.lamp_dz;
            const auto disc = [&](double radius, double y, const QColor& colour, double centre_alpha, double rim_alpha) {
                for (int k = 0; k < sides; ++k) {
                    const double a0 = 2 * std::numbers::pi * k / sides, a1 = 2 * std::numbers::pi * (k + 1) / sides;
                    tri(out, {x, y, z}, {x + radius * std::cos(a0), y, z + radius * std::sin(a0)}, {x + radius * std::cos(a1), y, z + radius * std::sin(a1)},
                        colour, colour, colour, centre_alpha, rim_alpha, rim_alpha);
                }
            };
            if (glow) {
                disc(1.1, hall.lamp_height_m - 0.02, haze, 0.45, 0.0);
                disc(0.5, hall.lamp_height_m - 0.03, haze, 0.55, 0.0);
            } else {
                disc(0.24, hall.lamp_height_m, white, 1, 1);
                // A small strip light on the roof between the rows.
                if (column + 1 < hall.lamp_columns) {
                    const double fx = x + hall.lamp_dx / 2, fz = z + hall.lamp_dz / 2, fy = hall.roof_at(fx) - 0.2;
                    if (fz < hall.max_z - 1) quad(out, {fx - 0.08, fy, fz - 0.45}, {fx + 0.08, fy, fz - 0.45}, {fx + 0.08, fy, fz + 0.45}, {fx - 0.08, fy, fz + 0.45}, fixture);
                }
            }
        }
}
} // namespace

std::vector<CornerSpan> kart_corner_spans(const fd::Track& track, const fd::KartCoach& coach, double margin_m) {
    std::vector<CornerSpan> spans;
    for (const auto& corner : coach.corners())
        spans.push_back({wrapped(fd::project(track, coach.position_at(corner.turn_in_m)).s_m - margin_m, track.length_m),
                         wrapped(fd::project(track, coach.position_at(corner.exit_m)).s_m + margin_m, track.length_m), corner.direction});
    return spans;
}

double KartHallLayout::roof_at(double x) const {
    const double centre = (min_x + max_x) / 2, reach = (max_x - min_x) / 2;
    return eave_m + (ridge_m - eave_m) * std::clamp(1 - std::abs(x - centre) / reach, 0.0, 1.0);
}

KartHallLayout kart_hall_layout(const fd::Track& track) {
    double low_x = 1e300, high_x = -1e300, low_y = 1e300, high_y = -1e300;
    for (const auto& point : track.points) {
        low_x = std::min(low_x, point.x_m);
        high_x = std::max(high_x, point.x_m);
        low_y = std::min(low_y, point.y_m);
        high_y = std::max(high_y, point.y_m);
    }
    const double half = half_width(track);
    KartHallLayout hall;
    // The west and north walls stand close behind the barriers, as the track runs along them; the east and south stand back.
    hall.min_x = low_x - half - 1.6;
    hall.max_x = high_x + half + 7.0;
    hall.min_z = -(high_y + half + 1.6);
    hall.max_z = -(low_y - half - 5.0);
    hall.eave_m = 6.0;
    hall.ridge_m = 9.0;
    hall.lamp_height_m = 5.6;
    hall.lamp_dx = 9.0;
    hall.lamp_dz = 3.6;
    hall.lamp_columns = std::max(1, static_cast<int>((hall.max_x - hall.min_x - 5.0) / hall.lamp_dx) + 1);
    hall.lamp_rows = std::max(1, static_cast<int>((hall.max_z - hall.min_z - 4.0) / hall.lamp_dz) + 1);
    hall.lamp_x0 = (hall.min_x + hall.max_x) / 2 - (hall.lamp_columns - 1) * hall.lamp_dx / 2;
    hall.lamp_z0 = (hall.min_z + hall.max_z) / 2 - (hall.lamp_rows - 1) * hall.lamp_dz / 2;
    return hall;
}

bool kart_hall_part_follows_ground(KartHallPart part) {
    return part == KartHallPart::barriers || part == KartHallPart::floor || part == KartHallPart::wear;
}

namespace {
// What a built part depends on, folded into one number: the track, its bends and its banking.
std::uint64_t fingerprint(const fd::Track& track, const std::vector<CornerSpan>& corners, const fd::TrackRelief* relief) {
    std::uint64_t hash = 1469598103934665603ull;
    const auto fold = [&](double value) {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        hash = (hash ^ bits) * 1099511628211ull;
    };
    fold(track.length_m);
    fold(track.width_m);
    fold(static_cast<double>(track.points.size()));
    for (const char letter : track.name) fold(letter);
    for (const auto& point : track.points) { fold(point.x_m); fold(point.y_m); }
    for (const double edge : track.left_edge_m) fold(edge);
    for (const double edge : track.right_edge_m) fold(edge);
    for (const auto& corner : corners) { fold(corner.from_s_m); fold(corner.to_s_m); fold(corner.direction); }
    if (relief)
        for (const auto& banking : relief->bankings()) { fold(banking.from_s_m); fold(banking.to_s_m); fold(banking.ramp_m); fold(banking.tilt_rad); }
    return hash;
}
struct Built { bool ready{}; std::uint64_t made_for{}; std::vector<SceneVertex> vertices; };
} // namespace

fd::KartEnvironment kart_environment(const fd::Track& track,const std::vector<CornerSpan>& corners) {
    auto walls=fd::track_barriers(track);
    // Slice the shared architectural layout at bumper height. This includes the
    // hall walls, gallery and pillars, but excludes the roof/overhead footbridge.
    // No rendering state, camera, visibility or frame timing enters the plant.
    std::vector<V> shell;
    build_shell(track,corners,shell);
    constexpr double height=0.25;
    for (std::size_t i=0;i+2<shell.size();i+=3) {
        std::vector<fd::Vec2> hits;
        for (std::size_t k=0;k<3;++k) {
            const auto& a=shell[i+k]; const auto& b=shell[i+(k+1)%3];
            if ((a.y<=height && b.y>height) || (b.y<=height && a.y>height)) {
                const double t=(height-a.y)/(b.y-a.y);
                hits.push_back({std::lerp(double(a.x),double(b.x),t),-std::lerp(double(a.z),double(b.z),t)});
            }
        }
        if (hits.size()==2 && std::hypot(hits[1].x-hits[0].x,hits[1].y-hits[0].y)>1e-6)
            walls.push_back({hits[0],hits[1]});
    }
    return fd::KartEnvironment(std::move(walls));
}

void build_kart_hall(KartHallPart part, const fd::Track& track, const std::vector<CornerSpan>& corners, const fd::TrackRelief* relief,
                     std::vector<SceneVertex>& out) {
    // The hall does not change while its track does not, and the scene asks for it again at every change of mode, so each
    // part is kept as it was last built.
    static std::array<Built, 6> kept;
    Built& built = kept[static_cast<std::size_t>(static_cast<int>(part) - static_cast<int>(KartHallPart::barriers))];
    const std::uint64_t now = fingerprint(track, corners, relief);
    if (!built.ready || built.made_for != now) {
        built.vertices.clear();
        switch (part) {
        case KartHallPart::barriers: build_barriers(track, corners, relief, built.vertices); break;
        case KartHallPart::floor: build_floor(track, corners, built.vertices); break;
        case KartHallPart::shell: build_shell(track, corners, built.vertices); break;
        case KartHallPart::lamps: build_lamps(track, false, built.vertices); break;
        case KartHallPart::glow: build_lamps(track, true, built.vertices); break;
        case KartHallPart::wear: build_wear(track, corners, built.vertices); break;
        }
        // What stands on the ground rises with the banked bend (decision 0050).
        if (relief && kart_hall_part_follows_ground(part))
            for (auto& v : built.vertices) v.y += static_cast<float>(relief->ground_at({v.x, -v.z}).height_m);
        built.ready = true;
        built.made_for = now;
    }
    out.insert(out.end(), built.vertices.begin(), built.vertices.end());
}

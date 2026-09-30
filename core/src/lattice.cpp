#include "fd/lattice.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace fd {
namespace {

std::string metres(double value) {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(2);
    out << value << " m";
    return out.str();
}

void validate_options(const LatticeOptions& o) {
    const auto within = [](double value, double low, double high) { return std::isfinite(value) && value >= low && value <= high; };
    if (!within(o.lateral_spacing_m, 0.1, 5)) throw std::invalid_argument("Lattice lateral_spacing_m must lie within 0.1 and 5 m");
    if (!within(o.straight_layer_spacing_m, 1, 50)) throw std::invalid_argument("Lattice straight_layer_spacing_m must lie within 1 and 50 m");
    if (!within(o.curve_layer_spacing_m, 0.5, o.straight_layer_spacing_m))
        throw std::invalid_argument("Lattice curve_layer_spacing_m must lie within 0.5 m and the straight spacing");
    if (!within(o.curve_threshold_1pm, 0, 1)) throw std::invalid_argument("Lattice curve_threshold_1pm must lie within 0 and 1 1/m");
    if (!within(o.lateral_change_per_metre, 0.01, 2)) throw std::invalid_argument("Lattice lateral_change_per_metre must lie within 0.01 and 2");
    if (!within(o.vehicle_half_width_m, 0, 5)) throw std::invalid_argument("Lattice vehicle_half_width_m must lie within 0 and 5 m");
    if (!within(o.sample_spacing_m, 0.05, 5)) throw std::invalid_argument("Lattice sample_spacing_m must lie within 0.05 and 5 m");
    const auto& w = o.weights;
    for (const double weight : {w.reference_deviation, w.reference_deviation_limit, w.length, w.average_curvature, w.curvature_range})
        if (!within(weight, 0, 1e12)) throw std::invalid_argument("Lattice cost weights must be finite and non-negative");
}

// Stations of the layers: from the reference's start, the curve spacing wherever the reference curves more than the
// threshold within the straight spacing ahead, otherwise the straight spacing; the last gap is split in two when a
// single one would be longer than the spacing that applies.
std::vector<double> layer_stations(const Track& track, const LatticeOptions& o) {
    std::vector<double> stations{0.0};
    double s = 0;
    while (true) {
        bool curves_ahead = false;
        for (const auto& p : track.points)
            if (p.s_m > s && p.s_m <= s+o.straight_layer_spacing_m && std::abs(p.curvature) > o.curve_threshold_1pm) { curves_ahead = true; break; }
        const double step = curves_ahead ? o.curve_layer_spacing_m : o.straight_layer_spacing_m;
        const double remaining = track.length_m-s;
        if (remaining <= step) break;
        if (remaining < 1.5*step) { stations.push_back(s+remaining/2); break; }
        s += step;
        stations.push_back(s);
    }
    return stations;
}

// The reference at a station: position, curvature and its rate of change interpolated between the two samples it lies
// between, the left normal from the chord through points a quarter metre either side, and the corridor's edges.
struct ReferenceAt {
    PathPoint point;
    Vec2 left_normal;
    double curvature_rate{};
    Corridor corridor;
    double left_rate{}, right_rate{};  // how fast each corridor edge's distance changes along the reference
};

Corridor corridor_at_station(const Track& track, double s) {
    s = std::fmod(s, track.length_m);
    if (s < 0) s += track.length_m;
    const auto upper = std::upper_bound(track.points.begin(), track.points.end(), s, [](double value, const PathPoint& p) { return value < p.s_m; });
    const std::size_t i = upper == track.points.begin() ? 0 : static_cast<std::size_t>(upper-track.points.begin()-1);
    const std::size_t j = (i+1)%track.points.size();
    Projection projection;
    projection.index = i;
    projection.fraction = (s-track.points[i].s_m)/((j == 0 ? track.length_m : track.points[j].s_m)-track.points[i].s_m);
    projection.s_m = s;
    return corridor_at(track, projection);
}

ReferenceAt reference_at(const Track& track, double s) {
    s = std::fmod(s, track.length_m);
    if (s < 0) s += track.length_m;
    const auto upper = std::upper_bound(track.points.begin(), track.points.end(), s, [](double value, const PathPoint& p) { return value < p.s_m; });
    const std::size_t i = upper == track.points.begin() ? 0 : static_cast<std::size_t>(upper-track.points.begin()-1);
    const std::size_t j = (i+1)%track.points.size();
    const auto& a = track.points[i];
    const auto& b = track.points[j];
    const double span = (j == 0 ? track.length_m : b.s_m)-a.s_m;
    const double f = (s-a.s_m)/span;
    ReferenceAt at;
    at.point = {a.x_m+f*(b.x_m-a.x_m), a.y_m+f*(b.y_m-a.y_m), s, a.curvature+f*(b.curvature-a.curvature)};
    at.curvature_rate = (b.curvature-a.curvature)/span;
    const auto behind = sample(track, s-0.25);
    const auto ahead = sample(track, s+0.25);
    const double tx = ahead.x_m-behind.x_m, ty = ahead.y_m-behind.y_m, length = std::hypot(tx, ty);
    at.left_normal = {-ty/length, tx/length};
    Projection projection;
    projection.index = i;
    projection.fraction = f;
    projection.s_m = s;
    projection.point = {at.point.x_m, at.point.y_m};
    at.corridor = corridor_at(track, projection);
    if (!track.left_edge_m.empty()) {
        const auto behind_corridor = corridor_at_station(track, s-1), ahead_corridor = corridor_at_station(track, s+1);
        at.left_rate = (ahead_corridor.left_m-behind_corridor.left_m)/2;
        at.right_rate = (ahead_corridor.right_m-behind_corridor.right_m)/2;
    }
    return at;
}

// The lateral slope a node leaves and is reached with: zero at the reference, and toward each edge a growing share of
// that edge's own slope, reaching it at the usable limit, so a node keeps its place across a corridor the reference
// crosses (TUM's variable heading, between the race line's and the bounds' orientation).
double node_slope(const ReferenceAt& at, double offset, double half_width) {
    const double left = at.corridor.left_m-half_width, right = at.corridor.right_m-half_width;
    if (offset > 0 && left > 0) return offset/left*at.left_rate;
    if (offset < 0 && right > 0) return -offset/right*(-at.right_rate);
    return 0;
}

// An edge's offset over u from 0 at its start layer to 1 at its end layer: the quintic Hermite curve with the given
// slopes per metre of station and zero second derivative at both ends, so chained edges keep continuous heading and
// curvature. With zero slopes it leaves and reaches the reference parallel.
struct Offset { double value, first, second; };  // d, dd/ds, d2d/ds2

Offset offset_along(double from, double to, double u, double span, double from_slope = 0, double to_slope = 0) {
    const double change = to-from;
    const double m0 = from_slope*span, m1 = to_slope*span;
    const double u2 = u*u, u3 = u2*u;
    return {from+change*u3*(10+u*(-15+6*u))+m0*(u+u3*(-6+u*(8-3*u)))+m1*u3*(-4+u*(7-3*u)),
            (change*u2*(30+u*(-60+30*u))+m0*(1+u2*(-18+u*(32-15*u)))+m1*u2*(-12+u*(28-15*u)))/span,
            (change*u*(60+u*(-180+120*u))+m0*u*(-36+u*(96-60*u))+m1*u*(-24+u*(84-60*u)))/(span*span)};
}

// The curvature of the curve a station-dependent offset traces beside a reference of curvature k and rate k': with
// P = 1 - k d, the curve's derivative along the reference is P along its tangent and d' along its normal, and its second
// derivative is -(k' d + 2 k d') along the tangent and k P + d'' along the normal.
double offset_curvature(double k, double rate, const Offset& d) {
    const double p = 1-k*d.value;
    return (p*(k*p+d.second)+d.first*(rate*d.value+2*k*d.first))/std::pow(p*p+d.first*d.first, 1.5);
}

// The offline cost of an edge ending `deviation` off the reference, as LatticeCostWeights states it.
LatticeCostTerms offline_cost(const LatticeEdge& edge, double deviation, const LatticeCostWeights& w) {
    double sum = 0, lowest = edge.samples.front().curvature, highest = lowest;
    for (const auto& s : edge.samples) {
        sum += std::abs(s.curvature);
        lowest = std::min(lowest, s.curvature);
        highest = std::max(highest, s.curvature);
    }
    const double mean = sum/static_cast<double>(edge.samples.size());
    const double length = edge.length_m;
    return {w.average_curvature*mean*mean*length, w.curvature_range*(highest-lowest)*(highest-lowest)*length, w.length*length,
            std::min(w.reference_deviation*std::abs(deviation), w.reference_deviation_limit)*length};
}

// Removes edges into nodes nothing leaves and out of nodes nothing reaches, and then the edges those removals strand,
// until every node is either reached and left or unused.
void remove_dead_ends(Lattice& lattice) {
    std::vector<std::size_t> first_node{0};
    for (const auto& layer : lattice.layers) first_node.push_back(first_node.back()+layer.offsets_m.size());
    const auto node_of = [&](std::size_t layer, std::size_t node) { return first_node[layer]+node; };
    const std::size_t nodes = first_node.back();
    std::vector<std::size_t> in(nodes, 0), out(nodes, 0);
    std::vector<std::vector<std::size_t>> leaving(nodes), arriving(nodes);
    for (std::size_t e = 0; e < lattice.edges.size(); ++e) {
        const auto& edge = lattice.edges[e];
        const auto from = node_of(edge.from_layer, edge.from_node), to = node_of(edge.to_layer, edge.to_node);
        ++out[from];
        ++in[to];
        leaving[from].push_back(e);
        arriving[to].push_back(e);
    }
    std::vector<bool> removed(lattice.edges.size(), false);
    std::vector<std::size_t> pending;
    for (std::size_t n = 0; n < nodes; ++n)
        if ((in[n] > 0) != (out[n] > 0)) pending.push_back(n);
    const auto remove = [&](std::size_t e) {
        if (removed[e]) return;
        removed[e] = true;
        ++lattice.removed_as_dead_ends;
        const auto& edge = lattice.edges[e];
        const auto from = node_of(edge.from_layer, edge.from_node), to = node_of(edge.to_layer, edge.to_node);
        if (--out[from] == 0 && in[from] > 0) pending.push_back(from);
        if (--in[to] == 0 && out[to] > 0) pending.push_back(to);
    };
    while (!pending.empty()) {
        const auto n = pending.back();
        pending.pop_back();
        if (out[n] == 0) for (const auto e : arriving[n]) remove(e);
        if (in[n] == 0) for (const auto e : leaving[n]) remove(e);
    }
    std::size_t kept = 0;
    for (std::size_t e = 0; e < lattice.edges.size(); ++e)
        if (!removed[e]) lattice.edges[kept++] = std::move(lattice.edges[e]);
    lattice.edges.resize(kept);
}

} // namespace

std::span<const LatticeEdge> Lattice::edges_from(std::size_t layer, std::size_t node) const {
    const auto key = [](const LatticeEdge& edge) { return std::pair{edge.from_layer, edge.from_node}; };
    const auto first = std::lower_bound(edges.begin(), edges.end(), std::pair{layer, node},
                                        [&](const LatticeEdge& edge, const std::pair<std::size_t, std::size_t>& at) { return key(edge) < at; });
    auto last = first;
    while (last != edges.end() && key(*last) == std::pair{layer, node}) ++last;
    return {first, last};
}

Lattice make_lattice(const Track& reference, const Config& config, const LatticeOptions& options) {
    validate_config(config);
    validate_track(reference);
    validate_options(options);
    for (std::size_t i = 0; i < reference.points.size(); ++i) {
        const double left = reference.left_edge_m.empty() ? reference.width_m/2 : reference.left_edge_m[i];
        const double right = reference.right_edge_m.empty() ? reference.width_m/2 : reference.right_edge_m[i];
        if (left < options.vehicle_half_width_m || right < options.vehicle_half_width_m)
            throw std::invalid_argument("The reference near "+metres(reference.points[i].s_m)+" is closer to the corridor's edge than the vehicle half width, "+
                                        metres(options.vehicle_half_width_m));
    }
    Lattice lattice;
    for (const double s : layer_stations(reference, options)) {
        const auto at = reference_at(reference, s);
        const double left = std::max(0.0, at.corridor.left_m-options.vehicle_half_width_m);
        const double right = std::max(0.0, at.corridor.right_m-options.vehicle_half_width_m);
        LatticeLayer layer;
        layer.s_m = s;
        layer.reference = at.point;
        layer.left_normal = at.left_normal;
        // Offsets are whole multiples of the spacing, so the reference node is exactly zero.
        const auto lowest = static_cast<long>(std::floor(right/options.lateral_spacing_m+1e-9));
        const auto highest = static_cast<long>(std::floor(left/options.lateral_spacing_m+1e-9));
        for (long k = -lowest; k <= highest; ++k) layer.offsets_m.push_back(static_cast<double>(k)*options.lateral_spacing_m);
        layer.reference_node = static_cast<std::size_t>(lowest);
        lattice.layers.push_back(std::move(layer));
    }

    const double steering_limit = std::tan(config.max_steering_rad)/config.wheelbase_m;
    const auto& layers = lattice.layers;
    for (std::size_t i = 0; i < layers.size(); ++i) {
        const std::size_t next = (i+1)%layers.size();
        const double start = layers[i].s_m;
        const double span = (next == 0 ? reference.length_m : layers[next].s_m)-start;
        const auto intervals = static_cast<std::size_t>(std::max(1.0, std::ceil(span/options.sample_spacing_m-1e-9)));
        // The reference at each sample station is shared by every edge between these two layers.
        std::vector<ReferenceAt> along;
        for (std::size_t k = 0; k <= intervals; ++k)
            along.push_back(reference_at(reference, k == intervals ? layers[next].s_m : start+span*static_cast<double>(k)/static_cast<double>(intervals)));
        const double allowance = options.lateral_change_per_metre*span;
        for (std::size_t from = 0; from < layers[i].offsets_m.size(); ++from)
            for (std::size_t to = 0; to < layers[next].offsets_m.size(); ++to) {
                const double d0 = layers[i].offsets_m[from], d1 = layers[next].offsets_m[to];
                const double v0 = node_slope(along.front(), d0, options.vehicle_half_width_m);
                const double v1 = node_slope(along.back(), d1, options.vehicle_half_width_m);
                if (std::abs(d1-(d0+v0*span)) > allowance+1e-9) continue;
                ++lattice.generated_edges;
                LatticeEdge edge;
                edge.from_layer = i;
                edge.from_node = from;
                edge.to_layer = next;
                edge.to_node = to;
                bool steerable = true;
                for (std::size_t k = 0; k <= intervals && steerable; ++k) {
                    const auto& at = along[k];
                    const auto d = offset_along(d0, d1, static_cast<double>(k)/static_cast<double>(intervals), span, v0, v1);
                    // At or past the reference's centre of curvature the offset curve folds back on itself.
                    if (1-at.point.curvature*d.value <= 0) { steerable = false; break; }
                    LatticeSample s;
                    s.x_m = at.point.x_m+at.left_normal.x*d.value;
                    s.y_m = at.point.y_m+at.left_normal.y*d.value;
                    s.s_m = at.point.s_m;
                    s.offset_m = k == intervals ? d1 : d.value;
                    s.curvature = offset_curvature(at.point.curvature, at.curvature_rate, d);
                    if (std::abs(s.curvature) > steering_limit) { steerable = false; break; }
                    if (k > 0) edge.length_m += std::hypot(s.x_m-edge.samples.back().x_m, s.y_m-edge.samples.back().y_m);
                    edge.samples.push_back(s);
                }
                if (!steerable) { ++lattice.removed_for_steering; continue; }
                bool inside = true;
                for (std::size_t k = 0; k <= intervals && inside; ++k) {
                    const auto& corridor = along[k].corridor;
                    const double offset = edge.samples[k].offset_m;
                    inside = offset <= corridor.left_m-options.vehicle_half_width_m+1e-9 && -offset <= corridor.right_m-options.vehicle_half_width_m+1e-9;
                }
                if (!inside) { ++lattice.removed_for_corridor; continue; }
                edge.terms = offline_cost(edge, d1, options.weights);
                edge.cost = edge.terms.total();
                lattice.edges.push_back(std::move(edge));
            }
    }
    remove_dead_ends(lattice);
    return lattice;
}

} // namespace fd

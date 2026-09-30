#include "fd/lattice.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Phase 5.1 (decision 0021): an offline lattice along a reference, after Stahl et al. 2019 as TUM lays it out. The
// oracles are the reference and its corridor: where layers fall along it, where nodes sit across it, and what the
// edges between them do.
namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tolerance)
        throw std::runtime_error(message+": actual="+std::to_string(actual)+", expected="+std::to_string(expected));
}

const fd::Config config;
const fd::Track preset = fd::make_preset_track();

// Layers follow the reference at the straight spacing where it is straight and at the curve spacing through its
// corners, and close the loop without a gap longer than either.
void layers_are_denser_through_corners() {
    const fd::LatticeOptions options;
    const auto lattice = fd::make_lattice(preset, config, options);
    const auto& layers = lattice.layers;
    require(layers.size() > 10, "the preset gets layers");
    near(layers.front().s_m, 0, 1e-12, "the first layer is at the reference's start");
    std::size_t straight = 0, curved = 0;
    for (std::size_t i = 0; i < layers.size(); ++i) {
        const double from = layers[i].s_m;
        const double to = i+1 < layers.size() ? layers[i+1].s_m : preset.length_m;
        require(to > from, "stations increase around the loop");
        bool curves_ahead = false;
        for (const auto& p : preset.points)
            if (p.s_m > from && p.s_m <= from+options.straight_layer_spacing_m && std::abs(p.curvature) > options.curve_threshold_1pm) curves_ahead = true;
        const double allowed = curves_ahead ? options.curve_layer_spacing_m : options.straight_layer_spacing_m;
        require(to-from <= allowed+1e-9, "layer "+std::to_string(i)+" is followed within its spacing, "+std::to_string(to-from)+" m");
        (curves_ahead ? curved : straight) += 1;
    }
    std::cout << "  preset, " << preset.length_m << " m: " << layers.size() << " layers, " << straight << " before straights and "
              << curved << " before corners\n";
    require(straight > 0 && curved > 0, "the preset has layers before both straights and corners");
}

// A reference 2 m right of the corridor's centre: every sample 3 m from the left edge and 7 m from the right.
fd::Track off_centre() {
    auto track = preset;
    track.name = "off-centre";
    track.left_edge_m.assign(track.points.size(), 3.0);
    track.right_edge_m.assign(track.points.size(), 7.0);
    return track;
}

// Nodes lie across the corridor, the lateral spacing apart and as far toward each edge as the vehicle half width
// allows, on the layer's normal, with the reference node at offset zero.
void nodes_span_the_corridor() {
    const fd::LatticeOptions options;
    for (const auto& track : {preset, off_centre()}) {
        const auto lattice = fd::make_lattice(track, config, options);
        const double left = (track.left_edge_m.empty() ? track.width_m/2 : 3.0)-options.vehicle_half_width_m;
        const double right = (track.right_edge_m.empty() ? track.width_m/2 : 7.0)-options.vehicle_half_width_m;
        for (const auto& layer : lattice.layers) {
            const auto& offsets = layer.offsets_m;
            require(!offsets.empty() && layer.reference_node < offsets.size() && offsets[layer.reference_node] == 0, "each layer has its reference node");
            for (std::size_t k = 1; k < offsets.size(); ++k) near(offsets[k]-offsets[k-1], options.lateral_spacing_m, 1e-12, "nodes are the lateral spacing apart");
            require(offsets.front() >= -right-1e-12 && offsets.back() <= left+1e-12, "every node keeps the vehicle half width inside the corridor");
            require(offsets.front()-options.lateral_spacing_m < -right && offsets.back()+options.lateral_spacing_m > left, "and nodes reach as far as it allows");
            near(std::hypot(layer.left_normal.x, layer.left_normal.y), 1, 1e-12, "the normal is a unit vector");
            const auto behind = fd::sample(track, layer.s_m-0.2), ahead = fd::sample(track, layer.s_m+0.2);
            const double tx = ahead.x_m-behind.x_m, ty = ahead.y_m-behind.y_m;
            // The reference is a polyline: its direction turns by up to a segment's length times its curvature at a sample.
            near((layer.left_normal.x*tx+layer.left_normal.y*ty)/std::hypot(tx, ty), 0, 0.6/18, "across the reference");
            require(tx*layer.left_normal.y-ty*layer.left_normal.x > 0, "and to its left");
        }
        std::cout << "  " << track.name << ": " << lattice.layers.front().offsets_m.size() << " nodes per layer from "
                  << lattice.layers.front().offsets_m.front() << " to " << lattice.layers.front().offsets_m.back() << " m\n";
    }
}

double gap(const fd::Lattice& lattice, const fd::Track& track, std::size_t layer) {
    const auto& layers = lattice.layers;
    return layer+1 < layers.size() ? layers[layer+1].s_m-layers[layer].s_m : track.length_m-layers[layer].s_m;
}

// Every node is joined to each node of the next layer within the lateral allowance, the last layer to the first: an
// edge starts at its node and ends at the other, its offset moving steadily from one to the other, its samples no
// further apart than the sample spacing. On a centreline every node's slope is zero, so the allowance is measured from
// the start node's own offset.
void edges_join_nodes_within_the_lateral_allowance() {
    const fd::LatticeOptions options;
    const auto lattice = fd::make_lattice(preset, config, options);
    const auto& layers = lattice.layers;
    std::size_t within = 0;
    for (std::size_t i = 0; i < layers.size(); ++i) {
        const auto& next = layers[(i+1)%layers.size()];
        const double allowance = options.lateral_change_per_metre*gap(lattice, preset, i);
        for (const double from : layers[i].offsets_m)
            for (const double to : next.offsets_m)
                if (std::abs(to-from) <= allowance+1e-9) ++within;
    }
    require(lattice.generated_edges == within, "one edge is generated per pair of nodes within the allowance, "+std::to_string(lattice.generated_edges)+
                                               " against "+std::to_string(within));
    require(!lattice.edges.empty() && lattice.edges.size() <= within, "edges remain, none beyond those generated");
    std::size_t listed = 0;
    for (std::size_t i = 0; i < layers.size(); ++i)
        for (std::size_t n = 0; n < layers[i].offsets_m.size(); ++n) {
            const auto leaving = lattice.edges_from(i, n);
            listed += leaving.size();
            for (std::size_t e = 0; e < leaving.size(); ++e) {
                const auto& edge = leaving[e];
                require(edge.from_layer == i && edge.from_node == n && edge.to_layer == (i+1)%layers.size(), "edges_from lists the node's own edges to the next layer");
                require(e == 0 || leaving[e-1].to_node < edge.to_node, "ordered by end node");
                const double from = layers[i].offsets_m[n], to = layers[edge.to_layer].offsets_m[edge.to_node];
                require(std::abs(to-from) <= options.lateral_change_per_metre*gap(lattice, preset, i)+1e-9, "within the lateral allowance");
                const auto& first = edge.samples.front();
                const auto& last = edge.samples.back();
                const auto& start = layers[i];
                const auto& end = layers[edge.to_layer];
                near(first.x_m, start.reference.x_m+start.left_normal.x*from, 1e-9, "an edge starts at its node");
                near(first.y_m, start.reference.y_m+start.left_normal.y*from, 1e-9, "an edge starts at its node");
                near(last.x_m, end.reference.x_m+end.left_normal.x*to, 1e-9, "and ends at the other");
                near(last.y_m, end.reference.y_m+end.left_normal.y*to, 1e-9, "and ends at the other");
                near(first.offset_m, from, 1e-12, "starting at the start node's offset");
                near(last.offset_m, to, 1e-12, "ending at the end node's offset");
                double length = 0;
                for (std::size_t k = 1; k < edge.samples.size(); ++k) {
                    const auto& a = edge.samples[k-1];
                    const auto& b = edge.samples[k];
                    const double station = std::fmod(b.s_m-a.s_m+preset.length_m, preset.length_m);
                    require(station > 0 && station <= options.sample_spacing_m+1e-9, "samples advance at most the sample spacing");
                    require((to-from)*(b.offset_m-a.offset_m) >= -1e-12, "the offset moves steadily toward the end node");
                    length += std::hypot(b.x_m-a.x_m, b.y_m-a.y_m);
                }
                near(edge.length_m, length, 1e-9, "the length is measured along the samples");
            }
        }
    require(listed == lattice.edges.size(), "every edge is listed from its start node");
    std::cout << "  preset: " << lattice.generated_edges << " edges within the allowance of " << options.lateral_change_per_metre << " m per metre\n";
}

// A left-hand circle traced exactly every `spacing`, 12 m wide.
fd::Track circle(double radius, double spacing) {
    fd::Track track;
    track.name = "circle";
    track.width_m = 12;
    const auto count = static_cast<int>(std::ceil(2*std::numbers::pi*radius/spacing));
    for (int i = 0; i < count; ++i) {
        const double angle = 2*std::numbers::pi*i/count;
        track.points.push_back({radius*std::sin(angle), radius*(1-std::cos(angle)), radius*angle, 1/radius});
    }
    track.length_m = 2*std::numbers::pi*radius;
    return track;
}

// Each sample's curvature is the curvature of the edge's own path: beside a circle an edge parallel to it is a circle
// of radius R - d, and an edge changing offset curves as the circle through its neighbouring samples does.
void edges_curve_as_their_paths_do() {
    const double radius = 40;
    const auto track = circle(radius, 0.05);
    fd::LatticeOptions options;
    options.sample_spacing_m = 0.05;
    fd::Config loose = config;  // the tightest turning the configuration allows, so few edges are removed for curvature
    loose.max_steering_rad = 0.7;
    loose.wheelbase_m = 1.5;
    const auto lattice = fd::make_lattice(track, loose, options);
    double worst_parallel = 0, worst_change = 0, sharpest = 0;
    for (const auto& edge : lattice.edges) {
        const double from = lattice.layers[edge.from_layer].offsets_m[edge.from_node];
        const double to = lattice.layers[edge.to_layer].offsets_m[edge.to_node];
        const auto& s = edge.samples;
        if (from == to) {
            for (const auto& sample : s) worst_parallel = std::max(worst_parallel, std::abs(sample.curvature-1/(radius-from)));
            continue;
        }
        for (std::size_t k = 2; k+2 < s.size(); ++k) {
            const auto& a = s[k-2];
            const auto& b = s[k];
            const auto& c = s[k+2];
            const double cross = (b.x_m-a.x_m)*(c.y_m-a.y_m)-(b.y_m-a.y_m)*(c.x_m-a.x_m);
            const double measured = 2*cross/(std::hypot(b.x_m-a.x_m, b.y_m-a.y_m)*std::hypot(c.x_m-b.x_m, c.y_m-b.y_m)*std::hypot(c.x_m-a.x_m, c.y_m-a.y_m));
            worst_change = std::max(worst_change, std::abs(b.curvature-measured));
            sharpest = std::max(sharpest, std::abs(b.curvature));
        }
    }
    std::cout << "  40 m circle: parallel edges within " << worst_parallel << " 1/m of 1/(R - d); offset changes up to " << sharpest
              << " 1/m within " << worst_change << " 1/m of the curvature through their samples\n";
    require(worst_parallel < 1e-9, "an edge parallel to a circle has the curvature of a circle of radius R - d");
    require(sharpest > 0.3 && worst_change < 0.02, "an edge changing offset curves as its samples do");
}

// Edges the car cannot steer are removed: every edge left curves within tan(max_steering_rad)/wheelbase_m, a tighter
// limit removes more, and the reference node's edges, which follow the reference itself, are never among them.
void edges_the_car_cannot_steer_are_removed() {
    fd::Config stiff = config;
    stiff.max_steering_rad = 0.25;
    std::size_t previous_removed = 0;
    for (const auto& car : {config, stiff}) {
        const auto lattice = fd::make_lattice(preset, car, {});
        const double limit = std::tan(car.max_steering_rad)/car.wheelbase_m;
        double sharpest = 0;
        for (const auto& edge : lattice.edges)
            for (const auto& s : edge.samples) sharpest = std::max(sharpest, std::abs(s.curvature));
        std::cout << "  steering limit " << limit << " 1/m: " << lattice.removed_for_steering << " of " << lattice.generated_edges
                  << " edges removed, the sharpest left curving " << sharpest << " 1/m\n";
        require(sharpest <= limit+1e-12, "no edge left curves beyond the steering limit");
        require(lattice.removed_for_steering > previous_removed, "a tighter steering limit removes more edges");
        previous_removed = lattice.removed_for_steering;
        for (std::size_t i = 0; i < lattice.layers.size(); ++i) {
            const auto& layer = lattice.layers[i];
            const auto leaving = lattice.edges_from(i, layer.reference_node);
            const auto along = std::find_if(leaving.begin(), leaving.end(), [&](const fd::LatticeEdge& e) {
                return e.to_node == lattice.layers[e.to_layer].reference_node;
            });
            require(along != leaving.end(), "the reference node keeps its edge to the next reference node at layer "+std::to_string(i));
            for (const auto& s : along->samples) {
                const auto on = fd::sample(preset, s.s_m);
                require(s.offset_m == 0 && std::hypot(s.x_m-on.x_m, s.y_m-on.y_m) < 1e-9, "that edge lies on the reference");
            }
        }
    }
}

// The preset with its corridor's left side narrowed to 2 m for the samples strictly between two layers on the first
// straight, 1 m clear of each; the layer before the narrowing is returned too.
std::pair<fd::Track, std::size_t> narrowing() {
    const fd::LatticeOptions options;
    const auto plain = fd::make_lattice(preset, config, options);
    std::size_t layer = 0;
    while (plain.layers[layer+1].s_m-plain.layers[layer].s_m < options.straight_layer_spacing_m-1e-9) ++layer;
    const double from = plain.layers[layer].s_m+1, to = plain.layers[layer+1].s_m-1;
    auto track = preset;
    track.name = "narrowing";
    track.left_edge_m.assign(preset.points.size(), preset.width_m/2);
    track.right_edge_m.assign(preset.points.size(), preset.width_m/2);
    for (std::size_t i = 0; i < preset.points.size(); ++i)
        if (preset.points[i].s_m > from && preset.points[i].s_m < to) track.left_edge_m[i] = 2;
    return {track, layer};
}

// Edges must keep the car's body inside the corridor between layers too: where the corridor narrows between two layers
// the edges that would cross the narrow part are removed, and every edge left stays inside at every sample.
void edges_that_leave_the_corridor_are_removed() {
    const fd::LatticeOptions options;
    const auto plain = fd::make_lattice(preset, config, options);
    require(plain.removed_for_corridor == 0, "a corridor of constant width removes nothing between its layers");
    const auto [narrowing, layer] = ::narrowing();
    const double from = plain.layers[layer].s_m+1, to = plain.layers[layer+1].s_m-1;
    const auto lattice = fd::make_lattice(narrowing, config, options);
    std::cout << "  corridor narrowed to 2 m on the left between " << from << " and " << to << " m: " << lattice.removed_for_corridor
              << " edges removed\n";
    require(lattice.layers.size() == plain.layers.size() && lattice.layers[layer].offsets_m == plain.layers[layer].offsets_m,
            "the layers and their nodes do not change, the narrowing lies between them");
    require(lattice.removed_for_corridor > 0, "edges crossing the narrow part are removed");
    for (const auto& edge : lattice.edges)
        for (const auto& s : edge.samples) {
            const auto on = fd::project(narrowing, fd::Vec2{fd::sample(narrowing, s.s_m).x_m, fd::sample(narrowing, s.s_m).y_m});
            const auto corridor = fd::corridor_at(narrowing, on);
            require(s.offset_m <= corridor.left_m-options.vehicle_half_width_m+1e-9 && -s.offset_m <= corridor.right_m-options.vehicle_half_width_m+1e-9,
                    "every edge left keeps the car inside the corridor at station "+std::to_string(s.s_m));
        }
}

// Edges into a node nothing leaves, or out of a node nothing reaches, are removed until none is left, so every path
// along edges can continue around the loop; every generated edge is either kept or counted as removed for one reason.
void dead_ends_are_removed() {
    const auto [track, layer] = narrowing();
    for (const auto& reference : {preset, track}) {
        const auto lattice = fd::make_lattice(reference, config, {});
        const auto& layers = lattice.layers;
        std::vector<std::vector<int>> in(layers.size()), out(layers.size());
        for (std::size_t i = 0; i < layers.size(); ++i) {
            in[i].assign(layers[i].offsets_m.size(), 0);
            out[i].assign(layers[i].offsets_m.size(), 0);
        }
        for (const auto& edge : lattice.edges) {
            ++out[edge.from_layer][edge.from_node];
            ++in[edge.to_layer][edge.to_node];
        }
        for (std::size_t i = 0; i < layers.size(); ++i)
            for (std::size_t n = 0; n < layers[i].offsets_m.size(); ++n)
                require((in[i][n] > 0) == (out[i][n] > 0), reference.name+": node "+std::to_string(n)+" of layer "+std::to_string(i)+
                                                           " is reached and left, or neither");
        require(lattice.generated_edges == lattice.edges.size()+lattice.removed_for_steering+lattice.removed_for_corridor+lattice.removed_as_dead_ends,
                "every generated edge is kept or counted as removed");
        std::cout << "  " << reference.name << ": " << lattice.edges.size() << " edges kept of " << lattice.generated_edges << "; removed "
                  << lattice.removed_for_steering << " for steering, " << lattice.removed_for_corridor << " for the corridor, "
                  << lattice.removed_as_dead_ends << " as dead ends\n";
        if (reference.name == "narrowing") {
            require(lattice.removed_as_dead_ends > 0, "the narrowing leaves dead ends, which are removed");
            require(out[layer][layers[layer].offsets_m.size()-1] == 0, "the leftmost node before the narrowing is left unused");
        }
    }
}

// The offline cost as the interface states it, from an edge's own samples and length.
double stated_cost(const fd::Lattice& lattice, const fd::LatticeEdge& edge, const fd::LatticeCostWeights& w) {
    double sum = 0, lowest = edge.samples.front().curvature, highest = lowest;
    for (const auto& s : edge.samples) {
        sum += std::abs(s.curvature);
        lowest = std::min(lowest, s.curvature);
        highest = std::max(highest, s.curvature);
    }
    const double mean = sum/static_cast<double>(edge.samples.size());
    const double deviation = std::abs(lattice.layers[edge.to_layer].offsets_m[edge.to_node]);
    return w.average_curvature*mean*mean*edge.length_m+w.curvature_range*(highest-lowest)*(highest-lowest)*edge.length_m+
           w.length*edge.length_m+std::min(w.reference_deviation*deviation, w.reference_deviation_limit)*edge.length_m;
}

// Each edge costs what the stated formula gives for its samples. Along a straight, where nothing curves, staying on the
// reference costs nothing, running parallel to it costs the deviation weight per metre of offset per metre up to the
// limit, and ending further from the reference costs more.
void edges_carry_the_offline_cost() {
    fd::LatticeOptions options;
    options.weights = {10000, 25000, 3, 7500, 2500};
    const auto lattice = fd::make_lattice(preset, config, options);
    double worst = 0, worst_terms = 0;
    for (const auto& edge : lattice.edges) {
        const double stated = stated_cost(lattice, edge, options.weights);
        worst = std::max(worst, std::abs(edge.cost-stated)/std::max(1.0, stated));
        require(edge.cost >= 0, "costs are never negative");
        // The terms add up to the cost, each the weighted part it names.
        const auto& terms = edge.terms;
        const double deviation = std::abs(lattice.layers[edge.to_layer].offsets_m[edge.to_node]);
        worst_terms = std::max({worst_terms, std::abs(terms.average_curvature+terms.curvature_range+terms.length+terms.reference_deviation-edge.cost),
                                std::abs(terms.length-3*edge.length_m),
                                std::abs(terms.reference_deviation-std::min(10000*deviation, 25000.0)*edge.length_m)});
        require(terms.average_curvature >= 0 && terms.curvature_range >= 0, "curvature terms are never negative");
    }
    require(worst < 1e-12, "every edge's cost is the stated formula of its samples");
    require(worst_terms < 1e-6, "an edge's cost terms add up to its cost, each the part it names");
    // A gap between layers where the reference curves nowhere within a metre of either layer.
    const auto straight = [&](std::size_t i) {
        const double a = lattice.layers[i].s_m-1, b = lattice.layers[i+1].s_m+1;
        return std::all_of(preset.points.begin(), preset.points.end(), [&](const fd::PathPoint& p) { return p.s_m < a || p.s_m > b || p.curvature == 0; });
    };
    std::size_t layer = 1;
    while (!straight(layer)) ++layer;
    const auto& from = lattice.layers[layer];
    const auto& to = lattice.layers[layer+1];
    const auto edge_between = [&](double a, double b) -> const fd::LatticeEdge& {
        for (const auto& edge : lattice.edges_from(layer, static_cast<std::size_t>(std::lround(a/options.lateral_spacing_m))+from.reference_node))
            if (to.offsets_m[edge.to_node] == b) return edge;
        throw std::runtime_error("no edge from "+std::to_string(a)+" to "+std::to_string(b));
    };
    const double span = to.s_m-from.s_m;
    near(edge_between(0, 0).cost, 3*span, 1e-9, "on the reference only the length weight is paid");
    near(edge_between(2, 2).cost, (10000*2+3)*span, 1e-6, "parallel to it the deviation weight is paid per metre of offset");
    near(edge_between(-3, -3).cost, (25000+3)*span, 1e-6, "up to the limit");
    require(edge_between(0, 1).cost > edge_between(0, 0.5).cost && edge_between(0, 0.5).cost > edge_between(0, 0).cost,
            "ending further from the reference costs more");
    std::cout << "  a straight, " << span << " m between layers: reference " << edge_between(0, 0).cost << ", 0 to 0.5 m "
              << edge_between(0, 0.5).cost << ", 0 to 1 m " << edge_between(0, 1).cost << ", parallel at 2 m " << edge_between(2, 2).cost << "\n";
}

// Where the reference crosses the corridor, a node leaves and is reached with a share of the nearer edge's slope: none
// at the reference, all of it at the usable limit. A node then keeps its place in the corridor from layer to layer
// without curving, so nodes stay usable; the few that do not lie deep on the inside of a corner, where a metre of the
// reference is much less of the car's path and any step across curves beyond the steering limit. Here the left edge
// swings between 3 and 7 m from the reference four times a lap, the right edge the other way.
void nodes_keep_their_place_across_a_corridor_the_reference_crosses() {
    const double omega = 2*std::numbers::pi*4/preset.length_m;
    auto swinging = preset;
    swinging.name = "swinging corridor";
    for (const auto& p : preset.points) {
        swinging.left_edge_m.push_back(5+2*std::sin(omega*p.s_m));
        swinging.right_edge_m.push_back(5-2*std::sin(omega*p.s_m));
    }
    fd::LatticeOptions options;
    options.sample_spacing_m = 0.05;
    const auto lattice = fd::make_lattice(swinging, config, options);
    double worst = 0, steepest = 0;
    for (const auto& edge : lattice.edges) {
        const auto& s = edge.samples;
        for (const auto& [a, b] : {std::pair{s[0], s[1]}, std::pair{s[s.size()-2], s[s.size()-1]}}) {
            const double station = a.s_m;
            const double left_rate = 2*omega*std::cos(omega*station);  // d(left)/ds; the right edge's is its negative
            const double d = a.offset_m;
            const double expected = d > 0 ? d/(5+2*std::sin(omega*station)-options.vehicle_half_width_m)*left_rate
                                  : d < 0 ? -d/(5-2*std::sin(omega*station)-options.vehicle_half_width_m)*left_rate : 0.0;
            const double measured = (b.offset_m-a.offset_m)/std::fmod(b.s_m-a.s_m+preset.length_m, preset.length_m);
            worst = std::max(worst, std::abs(measured-expected));
            steepest = std::max(steepest, std::abs(expected));
        }
    }
    std::size_t nodes = 0, used = 0;
    double shallowest_inside = 0;  // of the unusable nodes, the largest 1 - curvature * offset nearby, the path per metre of reference
    for (std::size_t i = 0; i < lattice.layers.size(); ++i)
        for (std::size_t n = 0; n < lattice.layers[i].offsets_m.size(); ++n) {
            ++nodes;
            if (!lattice.edges_from(i, n).empty()) { ++used; continue; }
            const double s = lattice.layers[i].s_m, d = lattice.layers[i].offsets_m[n];
            double deepest = 1;
            for (const auto& p : preset.points)
                if (std::abs(p.s_m-s) <= options.straight_layer_spacing_m) deepest = std::min(deepest, 1-p.curvature*d);
            shallowest_inside = std::max(shallowest_inside, deepest);
            // 1 - curvature * offset is the car's path length per metre of reference: below 0.85 it is inside a corner.
            require(deepest < 0.85, "an unusable node lies inside a corner: offset "+std::to_string(d)+" m at "+std::to_string(s)+" m");
        }
    std::cout << "  swinging corridor: node slopes up to " << steepest << " within " << worst << " of the share of the edge's slope; "
              << used << " of " << nodes << " nodes usable, the rest where the path is at most " << shallowest_inside
              << " m per metre of reference\n";
    require(steepest > 0.05 && worst < 0.005, "each edge leaves and reaches its nodes with their share of the corridor edge's slope");
    require(used >= 0.99*static_cast<double>(nodes), "at least 99% of the nodes stay usable");
}

std::string refusal(const std::function<void()>& action) {
    try { action(); } catch (const std::invalid_argument& error) { return error.what(); }
    return {};
}

// Options out of range, and a reference that comes closer to a corridor edge than the vehicle half width anywhere, not
// only at a layer, are refused with their reason.
void rejects_bad_input() {
    const auto with = [](const std::function<void(fd::LatticeOptions&)>& change) {
        fd::LatticeOptions options;
        change(options);
        return options;
    };
    const std::vector<std::pair<std::string, fd::LatticeOptions>> options{
        {"lateral_spacing_m", with([](auto& o) { o.lateral_spacing_m = 0; })},
        {"straight_layer_spacing_m", with([](auto& o) { o.straight_layer_spacing_m = 60; })},
        {"curve_layer_spacing_m", with([](auto& o) { o.curve_layer_spacing_m = 7; })},
        {"curve_threshold_1pm", with([](auto& o) { o.curve_threshold_1pm = -0.1; })},
        {"lateral_change_per_metre", with([](auto& o) { o.lateral_change_per_metre = 0; })},
        {"vehicle_half_width_m", with([](auto& o) { o.vehicle_half_width_m = std::nan(""); })},
        {"sample_spacing_m", with([](auto& o) { o.sample_spacing_m = 10; })},
        {"cost weights", with([](auto& o) { o.weights.curvature_range = -1; })},
    };
    for (const auto& [reason, bad] : options) {
        const auto message = refusal([&] { fd::make_lattice(preset, config, bad); });
        require(message.find(reason) != std::string::npos, "refused with its reason ("+reason+"), got: "+message);
    }
    auto hugging = preset;
    hugging.left_edge_m.assign(preset.points.size(), 5.0);
    hugging.right_edge_m.assign(preset.points.size(), 5.0);
    hugging.left_edge_m[100] = 0.5;  // one sample, between layers
    const auto message = refusal([&] { fd::make_lattice(hugging, config, {}); });
    std::cout << "  " << message << "\n";
    require(message.find("closer to the corridor's edge than the vehicle half width") != std::string::npos, "a reference hugging an edge is refused");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"layers_are_denser_through_corners", layers_are_denser_through_corners},
        {"nodes_span_the_corridor", nodes_span_the_corridor},
        {"edges_join_nodes_within_the_lateral_allowance", edges_join_nodes_within_the_lateral_allowance},
        {"edges_curve_as_their_paths_do", edges_curve_as_their_paths_do},
        {"edges_the_car_cannot_steer_are_removed", edges_the_car_cannot_steer_are_removed},
        {"edges_that_leave_the_corridor_are_removed", edges_that_leave_the_corridor_are_removed},
        {"dead_ends_are_removed", dead_ends_are_removed},
        {"edges_carry_the_offline_cost", edges_carry_the_offline_cost},
        {"nodes_keep_their_place_across_a_corridor_the_reference_crosses", nodes_keep_their_place_across_a_corridor_the_reference_crosses},
        {"rejects_bad_input", rejects_bad_input},
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cout << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-failures << '/' << tests.size() << " lattice groups passed\n";
    return failures == 0 ? 0 : 1;
}

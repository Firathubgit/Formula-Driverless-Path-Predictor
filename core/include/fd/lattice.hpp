#pragma once
#include "fd/core.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace fd {

// Offline lattice (TrackWayFastPlan Phase 5.1, decision 0021), after Stahl et al., ITSC 2019, as TUM's graph-based
// local trajectory planner lays it out, reimplemented: layers along a reference, denser where it curves; nodes across
// the corridor on each layer; edges from each node to nodes of the next layer within a lateral allowance; edges the car
// cannot steer or that leave the corridor removed, then dead ends; and an offline cost on each edge. Nothing here
// searches the lattice or drives it.
//
// An edge is an offset from the reference that changes with station as the quintic with zero second derivative at both
// ends and, at each end, its node's slope (Werling et al. 2010's lateral quintic, in station rather than time), so
// chained edges keep continuous heading and curvature. A node's slope is none at the reference and, toward each corridor
// edge, a growing share of that edge's own slope, all of it at the usable limit (TUM's variable heading): a node keeps
// its place in a corridor the reference crosses, as a racing line does, instead of following the reference across it.

// The offline cost's coefficients, with ForzaETH's scaled values for TUM's planner as defaults except the deviation limit.
// An edge of length L whose end node lies |d| off the reference costs
//   average_curvature * (mean |curvature|)^2 * L + curvature_range * (max curvature - min curvature)^2 * L
//   + length * L + min(reference_deviation * |d|, reference_deviation_limit) * L
// with curvature measured along the edge's samples. ForzaETH's limit saturates the deviation term at 1 m, a third of its
// 1/10-scale track; here it saturates at 4 m, about the usable half width of a 10 m corridor, so a lattice path is always
// drawn back toward the reference (decision 0022).
struct LatticeCostWeights {
    double reference_deviation{10000};
    double reference_deviation_limit{40000};
    double length{0};
    double average_curvature{7500};
    double curvature_range{2500};
};

struct LatticeOptions {
    double lateral_spacing_m{0.5};           // between neighbouring nodes of a layer (0.1 to 5 m)
    double straight_layer_spacing_m{6.0};    // between layers where the reference is straight (1 to 50 m)
    // Where it curves (0.5 m to the straight spacing). At 4 m a step of one lateral spacing, 0.5 m, bends the car by
    // about 0.18 1/m, which with a racing line's 0.03 1/m stays within the default car's 0.236 1/m; at 3 m it would not.
    double curve_layer_spacing_m{4.0};
    // A layer is followed at the curve spacing when the reference curves more than this anywhere within the straight
    // spacing ahead of it (0 to 1 1/m).
    double curve_threshold_1pm{0.02};
    // An edge may end at most this far per metre of station between its layers from where its start node's slope would
    // carry it, the same offset on a centreline (0.01 to 2).
    double lateral_change_per_metre{0.3};
    // Kept between a node or edge and each corridor edge, for the car's body around its rear axle (0 to 5 m).
    double vehicle_half_width_m{0.9};
    double sample_spacing_m{0.5};            // at most this much station between an edge's samples (0.05 to 5 m)
    LatticeCostWeights weights;
};

struct LatticeLayer {
    double s_m{};                       // station on the reference
    PathPoint reference;                // the reference there, with its curvature
    Vec2 left_normal;                   // unit normal the layer's offsets are measured along
    std::vector<double> offsets_m;      // its nodes, ascending, each within the corridor less the vehicle half width
    std::size_t reference_node{};       // the node at offset zero
};

struct LatticeSample {
    double x_m{}, y_m{};
    double s_m{};                       // reference station
    double offset_m{};
    double curvature{};                 // of the edge itself, 1/m, positive to the left
};

// An offline cost's weighted terms, one per coefficient of LatticeCostWeights; they add up to the cost.
struct LatticeCostTerms {
    double average_curvature{}, curvature_range{}, length{}, reference_deviation{};
    double total() const { return average_curvature+curvature_range+length+reference_deviation; }
};

// An edge from a node of one layer to a node of the next; the last layer's edges end on layer 0.
struct LatticeEdge {
    std::size_t from_layer{}, from_node{}, to_layer{}, to_node{};
    std::vector<LatticeSample> samples; // from the start node to the end node inclusive
    double length_m{};                  // along its samples
    double cost{};                      // offline cost, terms.total()
    LatticeCostTerms terms;
};

struct Lattice {
    std::vector<LatticeLayer> layers;
    std::vector<LatticeEdge> edges;     // ordered by start layer, start node, end node
    std::size_t generated_edges{};      // within the lateral allowance, before any was removed
    std::size_t removed_for_steering{};  // curving more than the car can steer, or past the reference's centre of curvature
    std::size_t removed_for_corridor{};  // leaving the corridor less the vehicle half width between layers
    std::size_t removed_as_dead_ends{};  // into a node nothing leaves, or out of one nothing reaches

    // The edges leaving a node, ordered by end node; empty for a node whose edges were all removed.
    std::span<const LatticeEdge> edges_from(std::size_t layer, std::size_t node) const;
};

// Lays the lattice along a closed reference accepted by validate_track, for the steering limit
// tan(max_steering_rad)/wheelbase_m of a configuration accepted by validate_config. Rejects, naming the reason, options
// out of range and a reference that comes closer to a corridor edge than the vehicle half width anywhere, which could
// leave a layer without its reference node. The reference node's edges from layer to layer follow the reference itself.
// Laying Foundry Circuit's lattice takes under 0.01 s on the reference host (measurement only).
Lattice make_lattice(const Track& reference, const Config& config, const LatticeOptions& options = {});

} // namespace fd

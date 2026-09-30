#pragma once
#include "fd/core.hpp"

#include <span>
#include <vector>

namespace fd {

// Cone-to-path planning (TrackWayFastPlan Phase 7.3, decision 0031): a port of FaSTTUBe's ft-fsd-path-planning (MIT,
// Copyright (c) 2022 Panagiotis; see THIRD_PARTY_NOTICES.md), which turns cones that may be uncoloured, one-sided and
// missing into a local centreline. Its three steps are ported as published: sorting each boundary's cones into a trace
// by searching every plausible ordering from the car and scoring them; matching each cone with one across the track,
// placing a virtual cone a minimum track width away where none is; and fitting a path through the midpoints of the
// matches, joined to the car, extended when short, trimmed to a horizon and sampled with its curvature. Its paths are
// fitted with FITPACK's smoothing splines, ported too (core/src/fitpack.*). It knows nothing of any map: it sees the
// cones it is given and the pose it is given, in one frame, and nothing else.

// FaSTTUBe's own cone types, in its own numbering.
enum class ConeType : int { unknown = 0, yellow = 1, blue = 2, orange_small = 3, orange_big = 4 };

struct TypedCone {
    Vec2 position;
    ConeType type{ConeType::unknown};
};

// FaSTTUBe's defaults, from its config.py, for a trackdrive or autocross: no relocalisation, no global path.
struct ConePathSettings {
    // Sorting.
    int max_neighbours{5};
    double max_neighbour_distance_m{6.5};
    double max_distance_to_first_m{6.0};
    int max_trace_length{12};
    double directional_angle_rad{0.6981317007977318};   // 40 degrees
    double absolute_angle_rad{1.1344640137963142};      // 65 degrees
    bool use_unknown_cones{true};
    // Matching.
    double min_track_width_m{3.0};
    double max_search_range_m{5.0};
    double max_search_angle_rad{0.8726646259971648};    // 50 degrees
    // Path.
    double smoothing{0.2};
    double predict_every_m{0.1};
    int max_degree{3};
    double max_distance_for_valid_path_m{5.0};
    double path_length_m{20.0};
    int horizon_points{40};
};
void validate_cone_path_settings(const ConePathSettings& settings);

// One sample of a planned path: its spline parameter (metres along the fitted path), where it is, and its curvature,
// positive to the left.
struct ConePathPoint {
    double parameter_m{}, x_m{}, y_m{}, curvature_1pm{};
};

// Everything one call makes of its cones, each step's result kept so that it can be drawn and checked.
struct ConePath {
    std::vector<Vec2> left, right;                           // each boundary's cones, sorted from the car
    std::vector<Vec2> left_with_virtual, right_with_virtual; // the same with virtual cones where one had no match
    std::vector<int> left_to_right, right_to_left;           // each cone's match across the track, -1 for none
    std::vector<Vec2> basis;                                 // the points the path was fitted through
    std::vector<ConePathPoint> path;                         // horizon_points samples from the car
    bool from_previous{};                                    // the path was made from the previous one, not the cones
};

// One planner for a run. FaSTTUBe's planner keeps the previous path, which it falls back on when the cones make none,
// starting from a gentle curve ahead; this one does the same, so a planner lives as long as a run does.
class ConePathPlanner {
public:
    explicit ConePathPlanner(ConePathSettings settings = {});
    // The path from these cones for a car at this position facing this direction, all in one frame. Where FaSTTUBe's own
    // code divides by zero, on cones in a perfectly straight line, this treats the curvature as zero instead.
    ConePath plan(std::span<const TypedCone> cones, Vec2 position, Vec2 direction);
    const std::vector<ConePathPoint>& previous() const noexcept { return previous_; }
    const ConePathSettings& settings() const noexcept { return settings_; }

private:
    ConePathSettings settings_;
    std::vector<ConePathPoint> previous_;
};

}  // namespace fd

#pragma once
#include "fd/cone_path.hpp"

#include <span>
#include <vector>

namespace fd {

// A second, independent cone-to-path method (TrackWayFastPlan Phase 7.3, decision 0031), written from the published
// description of FS-FEUP's planner only, never from its code, which is GPL-3.0: triangulate the cones, take the
// midpoints of the triangle edges that could cross the track, and walk from the car through neighbouring midpoints,
// looking a few steps ahead at each, then fit the walk as FaSTTUBe fits its matches, with the same FITPACK spline. It is
// here to cross-check the FaSTTUBe port on the same cone sets: where the two disagree, the cones are a good test.

struct DelaunaySettings {
    double min_edge_m{2.0}, max_edge_m{7.0};   // a midpoint's edge must be about a track's width
    double max_first_m{6.0};                    // how far ahead of the car the walk may start
    double max_step_m{7.0};                     // the farthest one midpoint may lie from the next
    int lookahead{3};                           // steps looked ahead before each step taken
    double max_turn_rad{1.0};                   // the sharpest turn between steps
    double path_length_m{20.0};
    int horizon_points{40};
};

struct DelaunayPath {
    std::vector<std::array<Vec2, 3>> triangles;  // the triangulation, for drawing
    std::vector<Vec2> midpoints;                 // every candidate midpoint
    std::vector<Vec2> walk;                      // the midpoints the walk took, from the car
    std::vector<ConePathPoint> path;             // fitted and sampled as FaSTTUBe's path is; empty without a walk
};

// Bowyer-Watson triangulation of distinct points; each triangle as indices into them, counterclockwise.
std::vector<std::array<int, 3>> delaunay_triangles(std::span<const Vec2> points);

DelaunayPath plan_delaunay_path(std::span<const TypedCone> cones, Vec2 position, Vec2 direction, const DelaunaySettings& settings = {});

}  // namespace fd

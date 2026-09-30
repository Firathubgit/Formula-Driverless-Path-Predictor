#pragma once
#include "fd/track_conditioning.hpp"

#include <string>
#include <vector>

namespace fd {

// Minimum-curvature racing line (TrackWayFastPlan Phase 3.4, decision 0020), after Heilmeier et al. 2019 as TUM uses
// it, solved with OSQP. Each sample of the current line may shift along its left normal; the line's curvature at each
// sample is linearised in the shifts, the summed squared curvature is minimised subject to the corridor, less the
// vehicle's half width and a margin, and to the steering limit, and the shifted line is fitted again and linearised
// anew until no sample moves further than a set distance.
struct RacingLineOptions {
    double vehicle_half_width_m{0.9};  // the local planner's vehicle margin
    // Kept between the vehicle and each corridor edge besides the half width. Pure Pursuit tracks the line within
    // about 0.45 m; with 0.3 m the car left the corridor on Foundry Circuit, with 0.6 m it did not (decision 0020).
    double margin_m{0.6};
    int max_iterations{30};
    double converged_shift_m{0.02};    // stop once no sample moves further than this in an iteration
};

struct RacingLine {
    // The line as a track: closed, uniform in arc length, with its curvature, the corridor's width, and each sample's
    // distance to the corridor's left and right edge, so the planner, controller and prediction drive it unchanged.
    Track track;
    std::vector<Vec2> left_normals;
    std::vector<double> offset_m;  // each sample's signed distance from the centreline, left positive
    int iterations{};
    double last_shift_m{};         // the largest move of any sample in the last iteration
    bool converged{};
    double squared_curvature{}, centreline_squared_curvature{};  // integral of curvature squared over the lap, 1/m
};

// Rejects, naming the reason: an invalid configuration, options out of range, a corridor narrower than the vehicle and
// margin, and a corridor within which no line meets the steering limit tan(max_steering_rad)/wheelbase_m.
RacingLine make_minimum_curvature_line(const ConditionedTrack& centreline, const Config& config, const RacingLineOptions& options = {});

} // namespace fd

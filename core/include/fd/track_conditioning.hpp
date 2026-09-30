#pragma once
#include "fd/core.hpp"

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace fd {

// Track conditioning (TrackWayFastPlan Phase 3.3, decision 0019), after TUM's prep_track, reimplemented: a closed
// centreline, traced or drawn, is resampled linearly, fitted with a smooth closed curve, resampled uniformly by arc
// length and checked, so the planner, the controller and a later racing line get continuous curvature and normals.
struct ConditioningOptions {
    // The centreline is first resampled linearly at this spacing; the fit's knots are twice as far apart (0.2 to 5 m).
    double input_spacing_m{1.0};
    // The fitted curve is the smoothest one whose root-mean-square distance from those resampled points is at most
    // this (0 to 5 m). The allowance is used in full, so a clean shape also moves by about this much where smoothing
    // it helps; zero follows the points as closely as the knots allow. TUM's smoothing factor is the same allowance
    // as a sum of squares, the number of points times this squared.
    double smoothing_rms_m{0.1};
    // Arc length between the conditioned samples (0.2 to 5 m), rounded so the samples close the loop evenly.
    double output_spacing_m{0.6};
    // Judge the result as a corridor of the given width: normals across it must not cross and it must not overlap
    // itself. A line inside a corridor that is already known, such as a racing line, turns these off; a line that
    // crosses itself is refused either way.
    bool corridor_checks{true};
};

struct ConditionedTrack {
    // Closed, accepted by validate_track, uniform in arc length, with the fitted curve's curvature at each sample
    // (positive to the left), so curvature is continuous rather than stepping at joins.
    Track track;
    // Unit normal at each sample pointing to the left of the direction of travel, the side offsets are positive on.
    std::vector<Vec2> left_normals;
    std::size_t resampled_points{};  // centreline points after linear resampling, the ones the fit is measured against
    double residual_rms_m{};         // their root-mean-square distance from the fitted curve at their parameters
    double max_deviation_m{};        // the largest distance of one of them from the conditioned track
};

// Conditions a closed centreline, in order, the last point joining the first (a repeated closing point is ignored).
// The direction of travel and the start are kept: sample 0 is where the fitted curve begins, beside the first point.
// Rejects, naming the reason: fewer than four distinct points, non-finite coordinates, a width or options outside
// their ranges, a loop too short for the fit, a conditioned line that crosses itself, normals that cross within half
// the width (a bend tighter than half the track), and a corridor that overlaps itself (two parts of the track closer
// than its width).
ConditionedTrack condition_track(const std::vector<Vec2>& centreline, double width_m, std::string name,
                                 const ConditioningOptions& options = {});
// Conditions a closed track accepted by validate_track, such as the preset, from points taken along it every
// input_spacing_m of arc length from its start, keeping its name and width: the centreline a racing line is solved
// against when no centreline file is given.
ConditionedTrack condition_track(const Track& track, const ConditioningOptions& options = {});

// Reads a centreline file: one "x_m,y_m" pair per line, in metres; '#' starts a comment; one header line of names
// is allowed. Rejects anything else, naming the line.
std::vector<Vec2> load_centreline(const std::filesystem::path& file);

} // namespace fd

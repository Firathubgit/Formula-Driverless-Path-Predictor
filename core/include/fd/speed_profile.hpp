#pragma once
#include "fd/core.hpp"

#include <span>
#include <vector>

namespace fd {

// Samples of a speed profile lie at most this far apart along the reference.
inline constexpr double speed_profile_spacing_m = 0.5;

// What a speed profile is for: a path from the car, where it starts and how far it runs, and the speeds at its ends.
struct ProfileRequest {
    double start_s_m{};        // the car's station on the reference
    double start_slope{};      // how fast the car moves across the reference per metre of station, as it starts
    // How far along the reference the car's own slope takes to become the path's first segment's, such as the pursuit
    // distance the controller turns over; never less than that segment's length.
    double start_turn_m{};
    double start_speed_mps{};  // the car's actual speed
    double horizon_m{};        // how far along the reference the profile runs
    double end_speed_mps{};    // the most the profile may end at, such as the reference plan's speed there
    // Offsets at stations increasing from start_s_m, as a lattice path's, which may run past the track's length. Beyond
    // its last point the path keeps its last offset. Empty follows the reference itself.
    std::span<const StationOffset> path;
};

struct SpeedProfile {
    // From the start station to the horizon, at most speed_profile_spacing_m apart, the last exactly at the horizon.
    std::vector<ProfilePoint> points;
    double estimated_time_s{};  // to drive the whole profile
    // False when the start speed is beyond what the path allows there, so the profile brakes at the limit instead.
    bool within_limits{true};
};

// A speed profile along a path (TrackWayFastPlan Phase 5.3, decision 0023), after TUM's forward-backward velocity
// planner: curvature-derived speed limits from the path's own geometry, a backward pass from the end speed and a forward
// pass from the car's actual speed.
//
// Geometry. The path is straight segments between its points, as a path intent pursues them. Its offset from the
// reference is interpolated between the points; how fast that offset changes varies linearly between the middles of
// adjacent segments, from the car's own slope start_turn_m, or the first segment's length if longer, before the first
// segment's middle, to nothing half the last segment beyond the path, so each change of slope between segments is spread
// over the mean of the two segments' lengths as the lattice search costs it. The path's curvature then follows from the
// reference's curvature, interpolated between samples, its rate and the offset's derivatives, and its length from the
// offset and its slope.
//
// Limits. The same as the reference plan's (make_speed_plan): without an envelope, lateral acceleration within
// lateral_grip_fraction and longitudinal within longitudinal_grip_fraction of mu g; with the envelope the plan was made
// from, envelope_fraction of the car's own lateral, forward and braking limits; never above max_speed_mps. The profile
// ends at no more than end_speed_mps.
//
// If the start speed is beyond what the path allows, the profile brakes at the limit until it is within, and says so.
// The estimated time covers each span at constant acceleration between its end speeds; it is infinite if the profile
// stops. Rejects non-finite or negative speeds, a horizon that is not positive, a path whose first station is not the
// start or whose stations do not increase, and an envelope derived for another configuration.
SpeedProfile make_speed_profile(const Track& track, const Config& config, const ProfileRequest& request,
                                const PerformanceEnvelope* envelope = nullptr);

// The time to drive profile points: each span at constant acceleration between its end speeds, along its distance.
double profile_time(std::span<const ProfilePoint> points);

} // namespace fd

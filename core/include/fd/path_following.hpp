#pragma once
#include "fd/trajectory.hpp"

#include <span>
#include <vector>

namespace fd {

// Driving an open path the car believes rather than a closed track it knows (TrackWayFastPlan Phase 7.3, decision 0031):
// the path runs from about where the car is to as far as it has seen, and nothing is known beyond its end. The rules are
// the reference plan's and the policy's, made for a path that ends.

// A believed path: samples at increasing stations, the first at zero, each with its curvature, positive to the left;
// and the corridor it believes around them, half its width either side.
struct OpenPath {
    std::vector<PathPoint> points;
    double width_m{3.0};
};
// Rejects fewer than two samples, non-finite values, a first station that is not zero, stations that do not increase,
// a station step shorter than the chord it spans, and a width that is not finite and positive.
void validate_open_path(const OpenPath& path);

// Speeds along the path by make_speed_plan's rules, without the seam that closes a lap: each sample within its curvature
// limit and the speed cap, reached forward from the first sample's own limit and braked into backward from rest at the
// last, so that the car can always stop within what it has seen. Each point's acceleration is to the next sample; the
// last is zero. An envelope derived under another configuration is rejected.
std::vector<PlanPoint> plan_open_path(const OpenPath& path, const Config& config, const PerformanceEnvelope* envelope = nullptr);

// Where a position lies along the path: the nearest point of its segments, the first before the path and the last beyond
// it included, with its station, its signed distance (left positive) and its distance.
Projection project_open(const OpenPath& path, Vec2 position);

// The policy along the path, as compute_control is along a track with no intent: the target speed interpolated in speed
// squared between the two samples the car lies between, with the span's acceleration as feedforward; Pure Pursuit, or
// MAP with a steering table, toward the point a lookahead ahead of the car's station, which beyond the path's end
// continues along its last segment; and the same acceleration bounds.
ControlResult follow_open_path(const OpenPath& path, const std::vector<PlanPoint>& plan, const State& state, const Config& config,
                               const SteeringTable* steering = nullptr, const PerformanceEnvelope* envelope = nullptr);

// The policy rolled forward along the path from a plant state, copies only, as make_local_trajectory rolls it along a
// track: the first command held ticks_to_next_control fixed ticks (zero for a whole control period), then one per control
// period, over prediction_horizon_s. The prediction is flagged when it leaves the believed corridor less the car's half
// width, runs faster than the plan, or passes the model's envelope.
LocalTrajectory predict_open_path(const OpenPath& path, const std::vector<PlanPoint>& plan, const PlantState& state,
                                  const Config& config, const VehicleModel& model, std::uint64_t ticks_to_next_control = 0,
                                  const SteeringTable* steering = nullptr, const PerformanceEnvelope* envelope = nullptr);

} // namespace fd

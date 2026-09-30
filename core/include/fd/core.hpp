#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace fd {

class SteeringTable;        // fd/steering.hpp
class PerformanceEnvelope;  // fd/performance_envelope.hpp

struct Vec2 { double x{}, y{}; };
struct State {
    double x_m{}, y_m{}, yaw_rad{}, speed_mps{}, steering_rad{}, time_s{};
};
struct PathPoint { double x_m{}, y_m{}, s_m{}, curvature{}; };
struct PlanPoint {
    double speed_mps{}, acceleration_mps2{};
    std::size_t limiting_index{};
    std::string reason;
};
struct Track {
    std::vector<PathPoint> points;
    double width_m{10.0}, length_m{};
    std::string name{"Foundry Circuit"};
    // For a reference line that is not the corridor's centre, such as a racing line (decision 0020): each sample's
    // distance to the corridor's left and right edge. Empty for a centreline, whose edges are half the width away.
    std::vector<double> left_edge_m, right_edge_m;
};
// How far the corridor's edges lie left and right of the reference at a projection.
struct Corridor { double left_m{}, right_m{}; };
struct Config {
    double grip_mu{1.0};
    double fixed_dt_s{0.005};
    double control_dt_s{0.02};
    double wheelbase_m{2.6};
    double max_speed_mps{20.0};
    double max_steering_rad{0.55};
    double max_steering_rate_radps{0.9};
    double lateral_grip_fraction{0.65};
    double longitudinal_grip_fraction{0.55};
    double speed_gain{1.5};
    double lookahead_base_m{4.0};
    double lookahead_time_s{0.25};
    // The share of the car's performance envelope a plan made from it uses, 0.5 to 1 (decision 0018): the envelope
    // shrunk toward its origin, a reserve for tracking. Unused by the grip fraction plan.
    double envelope_fraction{0.8};
};
struct Command { double acceleration_mps2{}, steering_rad{}; };
// A point of a path given by where it lies across the reference: its offset, left positive, at a station.
struct StationOffset { double s_m{}, offset_m{}; };
// A point of a speed profile along a path (decision 0023): its station on the reference, running on as a path's does;
// how far along the path it lies from the profile's start; the path's curvature there; and the speed planned there.
struct ProfilePoint { double s_m{}, distance_m{}, curvature_1pm{}, speed_mps{}; };
// How the caller wants this control step to deviate from the reference line. A
// default-constructed intent reproduces plain reference following exactly.
struct ControlIntent {
    // Aim this far left of the reference centerline. It shifts the pursuit target,
    // so the vehicle converges toward the offset line rather than jumping to it.
    double lateral_offset_m{};
    // Extra cap on the reference target speed; infinity means the reference governs.
    // While the cap binds, the reference acceleration feedforward is dropped because
    // it belongs to the uncapped profile.
    double speed_limit_mps{std::numeric_limits<double>::infinity()};
    // A path to aim along instead of one offset, such as a lattice path (decision 0022): offsets at stations that
    // increase from the first and may run past the track's length. The pursuit target takes the path's offset at its
    // own station, interpolated, the first offset before the path and the last beyond it; the path is steerable, so the
    // aim is not pushed further ahead as it is for a constant offset. Not owned: it must outlive the call. Empty uses
    // lateral_offset_m.
    std::span<const StationOffset> path;
    // A speed profile to follow instead of the reference plan while the car is within its stations (decision 0023),
    // such as make_speed_profile's for the same path: the target speed is interpolated in speed squared between the two
    // points the car lies between, and the feedforward is that span's constant acceleration along its distance. Before
    // the profile the target is its first speed without feedforward; beyond its end the reference plan governs again.
    // The speed cap still applies on top. Not owned: it must outlive the call. Empty follows the reference plan.
    std::span<const ProfilePoint> speed_profile;
};
struct Projection {
    std::size_t index{};
    double fraction{}, s_m{}, signed_error_m{}, distance_m{};
    Vec2 point;
};
struct ControlResult {
    Command requested{}, applied{};
    double lookahead_m{};
    // Reference context at this actual state, shared with prediction diagnostics.
    double target_speed_mps{}, track_distance_m{};
    // Pure Pursuit's steering toward the same target. It is the requested steering unless a steering
    // table converted the demanded turn, in which case the difference is what the table added.
    double geometric_steering_rad{};
    // Where the state projects onto the reference, for corridor checks.
    Projection reference{};
};

inline constexpr double gravity_mps2 = 9.80665;
inline constexpr double plan_color_deadband_mps2 = 0.15;

// All coordinates use a right-handed map frame, SI, CCW yaw, rear-axle pose.
void validate_config(const Config& config);
Config load_config(const std::filesystem::path& file);
Track make_preset_track();
void validate_track(const Track& track);
// The periodic reference speed plan. Without an envelope it plans with the configured grip fractions, the baseline:
// lateral acceleration within lateral_grip_fraction and longitudinal within longitudinal_grip_fraction of mu g. With
// the car's performance envelope (decision 0018) it plans with envelope_fraction of that car's own capacity instead,
// the envelope shrunk toward its origin: each sample's speed within the share of the lateral limit at that speed, the
// first crossing from rest, so every slower speed is also within it; forward from a sample within the share of the
// forward limit at its speed and lateral acceleration; braking into a sample within the share of the braking limit at
// its speed and lateral acceleration. Holding speed within the lateral limit is always allowed, because that limit is
// a turn the plant held steady. The envelope's accelerations already include drag, power and wheel inertia, so
// nothing is subtracted again. An envelope derived under another configuration is rejected.
std::vector<PlanPoint> make_speed_plan(const Track& track, const Config& config, const PerformanceEnvelope* envelope = nullptr);
// The lap time a plan implies on its closed track: each span between samples, the one closing the loop included,
// covered at constant acceleration from one sample's planned speed to the next. An estimate of the plan, not of a
// car driving it. Rejects a plan with another number of points, and a span planned at rest at both ends.
double estimated_lap_time(const Track& track, const std::vector<PlanPoint>& plan);
// Projection/sampling require a Track already accepted by validate_track().
Projection project(const Track& track, Vec2 position);
PathPoint sample(const Track& track, double s_m);
// The corridor at a projection: half the width either side for a centreline, otherwise the track's edge distances
// interpolated between the two samples the projection lies between.
Corridor corridor_at(const Track& track, const Projection& at);
// Whether a projected point lies within the corridor less a margin on each side. For a centreline this is exactly the
// distance to the reference against half the width less the margin; with edges it is the signed offset against each.
bool within_corridor(const Track& track, const Projection& at, double margin_m);
// Stateless controller for simulation or future hardware adapters. Track/config
// must be validated and plan must correspond to that track and configuration.
// Rejects non-finite states and malformed/non-finite plan entries at this boundary.
// An intent offsets the pursuit target or aims along a path, caps the target speed, and may replace the reference
// plan's speed with a speed profile for its own path (decision 0023); without a profile, offsetting does not re-derive
// curvature speed limits for the shifted line.
// Without a steering table this is Pure Pursuit: steering atan(wheelbase * curvature) for the arc
// through the target. With one it is MAP (decision 0010): the same arc's curvature, converted by the
// table into the steering the plant needs at this speed. The caller must pass a table generated for
// the plant and configuration being controlled.
// The applied acceleration is bounded by the longitudinal grip fraction of mu g, the budget the baseline plan
// used. With the performance envelope a plan was made from (decision 0018) it is bounded instead by the car's own
// capacity at its present speed, so the car can follow that plan's braking: driving by the forward limit at the
// lateral acceleration the reference's curvature asks at that speed, so pulling out of a bend does not spin the inside
// wheel, and braking by the whole straight-line capacity, so the car can always slow down (decision 0020).
ControlResult compute_control(const Track& track, const std::vector<PlanPoint>& plan,
                              const State& state, const Config& config,
                              ControlIntent intent = {}, const SteeringTable* steering = nullptr,
                              const PerformanceEnvelope* envelope = nullptr);
// The caller owns longitudinal allocation; this plant bounds steering and rate.
// A valid Config is required. No tire saturation, drag, mass or slip is simulated.
State integrate_bicycle(const State& state, Command applied, const Config& config, double dt_s);
double wrap_angle(double angle_rad);

} // namespace fd

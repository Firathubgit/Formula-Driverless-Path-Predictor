#pragma once
#include "fd/core.hpp"
#include "fd/steering.hpp"
#include "fd/vehicle.hpp"

namespace fd {

struct TrajectorySample {
    State state;
    // Achieved average acceleration over the outgoing segment; zero at the endpoint.
    double acceleration_mps2{};
    double distance_m{};
};

struct LocalTrajectory {
    std::vector<TrajectorySample> points;
    // A predictive controller's plan also states the acceleration and steering it commands at each point, changing
    // linearly between them (decision 0025). The policy's predictions leave it empty: their commands follow from the policy.
    std::vector<Command> commands;
    ControlResult first_control;
    bool within_model_envelope{true};
    std::string validity_reason{"Prediction within checked kinematic envelope"};
};

inline constexpr double prediction_horizon_s = 3.0;

// Predict the existing feedback policy from actual state, using copies only. This
// is neither an optimized racing line nor perception: the validated known-track
// reference still supplies braking context beyond the visible three-second horizon.
// The caller applies first_control, then replans at its next control boundary.
// An off-grid parameter event can shorten the first command hold: pass the number
// of fixed ticks until the next regular control tick (1..control_dt/fixed_dt).
// Zero uses a full control period. Horizon rounds up to a whole fixed tick.
// Input failures throw invalid_argument; envelope violations return a flagged
// prediction, not a safety guarantee or an imaginary instant recovery.
// An intent rolls the same policy forward toward an offset line or along a path, under an extra
// speed cap, and following the intent's speed profile when it has one (decision 0023).
// The kinematic bicycle from a published state.
LocalTrajectory make_local_trajectory(const Track& track, const std::vector<PlanPoint>& reference,
                                      const State& state, const Config& config,
                                      std::uint64_t ticks_to_next_control = 0,
                                      ControlIntent intent = {});
// Any vehicle model from its plant state. The prediction copies the state and rolls the copy
// forward through advance(), checking the model's own envelope from demand(). With a steering
// table every predicted control uses MAP; the table must have been generated for this model and
// configuration, and one generated for any other is rejected. With the performance envelope the
// reference was planned from, every predicted control is bounded by it as compute_control describes;
// an envelope derived for any other model or configuration is rejected.
LocalTrajectory make_local_trajectory(const Track& track, const std::vector<PlanPoint>& reference,
                                      const PlantState& state, const Config& config,
                                      const VehicleModel& model,
                                      std::uint64_t ticks_to_next_control = 0,
                                      ControlIntent intent = {},
                                      const SteeringTable* steering = nullptr,
                                      const PerformanceEnvelope* envelope = nullptr);

} // namespace fd

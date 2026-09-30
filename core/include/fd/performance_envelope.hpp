#pragma once
#include "fd/vehicle.hpp"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace fd {

// The car's G-G-V performance envelope, derived from the plant (TrackWayFastPlan Phase 3.1, decision 0017): at
// each speed of a grid, the accelerations the four-wheel car sustains with every tire within its peak.
//
// Lateral limit: the plant is held at the speed by a throttle loop while steering is swept up, each turn settled
// from the last, until a turn cannot be settled with every tire within its peak; the boundary is refined by
// bisection. Lateral levels: settled turns at fixed fractions of that limit. Longitudinal limits: from each level's
// settled turn, the largest forward and braking commands whose next 0.1 s keeps every tire within its peak, found
// by bisection, reported as the acceleration the plant achieved, drag and wheel inertia included. Left and right
// turns are derived separately, so symmetry is measured rather than assumed.
//
// Only the four-wheel car has one: the kinematic bicycle's limits are its stated grip fractions, and the
// single-track car clamps longitudinal force at its capacity, so a probe would find its allocation, not its tires.
enum class EnvelopeLimit {
    tire,       // every tire within its peak up to here, and not beyond
    steering,   // the steering limit was reached with the tires still within their peak
    actuator,   // the largest command probed kept the tires within their peak: power, torque or brake capacity limits
    settling,   // a turn beyond here could not be shown steady within the settling time
};
const char* envelope_limit_name(EnvelopeLimit limit);

struct EnvelopeLevel {
    double lateral_mps2{};  // the settled turn's lateral acceleration, speed times yaw rate, toward the turn
    double forward_mps2{};  // largest longitudinal acceleration achieved from it
    double braking_mps2{};  // most negative longitudinal acceleration achieved from it
    EnvelopeLimit forward_limit{EnvelopeLimit::tire}, braking_limit{EnvelopeLimit::tire};
};
struct EnvelopeSide {
    double lateral_limit_mps2{};
    EnvelopeLimit lateral_limit{EnvelopeLimit::tire};
    std::vector<EnvelopeLevel> levels;  // one per envelope_lateral_fractions, ascending; the first is straight ahead
};
struct EnvelopeRow { double speed_mps{}; EnvelopeSide left, right; };
enum class TurnSide { left, right };

// What a speed plan is made from (decision 0018): the configured grip fractions, the baseline every model can use, or
// the four-wheel car's performance envelope.
enum class SpeedPlanMode { grip_fractions, performance_envelope };
const char* speed_plan_mode_name(SpeedPlanMode mode);

// Lateral levels, as fractions of each side's lateral limit.
inline constexpr std::array<double, 6> envelope_lateral_fractions{0, 0.25, 0.5, 0.75, 0.9, 1.0};

class PerformanceEnvelope {
public:
    // Derives the envelope on a speed grid from 4 m/s, where the plant is fully dynamic, to 4 m/s above the
    // configured top speed, every 2 m/s. Rejects any model but the four-wheel car and invalid configurations.
    PerformanceEnvelope(const VehicleModel& model, const Config& config);
    const std::vector<EnvelopeRow>& rows() const noexcept { return rows_; }
    const std::string& fingerprint() const noexcept { return fingerprint_; }
    bool generated_for(const VehicleModel& model, const Config& config) const;
    // Derived for its own car under this configuration.
    bool generated_for(const Config& config) const;
    // Linear in speed between grid rows, clamped to the grid; linear in lateral acceleration between levels.
    double lateral_limit(double speed_mps, TurnSide side) const;
    // Lateral acceleration is signed, positive for a left turn. Beyond the lateral limit there is no capacity: zero.
    double forward_limit(double speed_mps, double lateral_mps2) const;
    double braking_limit(double speed_mps, double lateral_mps2) const;
    // One side at any speed, every level interpolated level by level between grid rows and clamped to the grid:
    // the boundary to draw at that speed.
    EnvelopeSide side_at(double speed_mps, TurnSide side) const;
private:
    std::vector<EnvelopeRow> rows_;
    FourWheelCar car_;
    std::string identity_, fingerprint_;
};

// The fingerprint an envelope derived from this model and configuration would carry, without deriving it.
std::string performance_envelope_fingerprint(const VehicleModel& model, const Config& config);

// Writes three files headed by comment lines naming the fingerprint: envelope.csv with every row, side and level;
// and TUM's ggv.csv (v_mps, ax_max_mps2, ay_max_mps2) and ax_max_machines.csv (v_mps, ax_max_machines_mps2). TUM's
// velocity profile adds drag itself and takes the tires' longitudinal limit and the powertrain's separately, so
// ggv.csv's ax_max is the straight-line braking limit less drag deceleration, ay_max the smaller side's lateral limit,
// and ax_max_machines the straight-line forward limit plus drag deceleration. Rejects a model the envelope was not
// derived for.
void write_performance_envelope(const PerformanceEnvelope& envelope, const VehicleModel& model, const Config& config,
                                const std::filesystem::path& directory);

} // namespace fd

#pragma once
#include "fd/vehicle.hpp"

#include <array>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace fd {

// The sensor seam (TrackWayFastPlan Phase 7.1, decision 0029): between the plant, which knows everything, and whatever
// reads it, which should not. A channel samples at its own rate, its sample arrives a dead time later, and what arrives
// carries a Gaussian error. After PacSim (MIT), read for structure only: its sensors hold a rate, a dead-time queue and
// a normal error per channel, and sample when the time has come round rather than on every tick.
//
// Nothing here detects, classifies or tracks anything: these are the car's own instruments reading its own state.
// Cones, and a planner that sees only what is measured, are Phases 7.2 and 7.4; until then the simulation still drives
// on ground truth and these measurements are reported beside it.

// One channel's instrument: how often it samples, how long its sample takes to arrive, and the standard deviation of
// the error on each value it carries. A rate of zero switches the channel off; a noise of zero measures exactly.
struct SensorChannel {
    double rate_hz{200};
    double dead_time_s{0.005};
    double noise{};         // standard deviation, in the channel's own unit
};

// The instruments this car carries. The defaults follow PacSim's own sensors.yaml where the channels correspond: its
// steering, wheel speed and IMU sensors sample at 200 Hz after 5 ms, and its GNSS at 20 Hz after 50 ms.
struct SensorSettings {
    SensorChannel pose{20, 0.05, 0.03};            // metres on each of x and y
    double pose_yaw_noise_rad{0.02};
    SensorChannel speed{20, 0.05, 0.02};           // metres per second
    SensorChannel wheel_speeds{200, 0.005, 0.1};   // radians per second, each wheel
    SensorChannel imu{200, 0.005, 0.05};           // metres per second squared on each acceleration
    double imu_yaw_rate_noise_radps{0.001};
    SensorChannel steering{200, 0.005, 1e-5};      // radians
    // Every error of a run comes from this seed, so a recording of one reproduces it exactly.
    std::uint64_t seed{1};
};
// Rejects rates, dead times and standard deviations outside their supported ranges.
void validate_sensors(const SensorSettings& settings);

// What one channel last delivered, and when that value was sampled from the plant. A channel that has delivered
// nothing yet is not measured; its values stay zero.
struct SensorReading {
    bool measured{};
    double sampled_at_s{};
};

// The car as its own instruments last reported it: each channel's newest sample that has arrived.
struct Measurements {
    SensorReading pose;
    double x_m{}, y_m{}, yaw_rad{};
    SensorReading speed;
    double speed_mps{};
    SensorReading wheel_speeds;
    std::array<double, 4> wheel_speeds_radps{};
    SensorReading imu;
    double longitudinal_acceleration_mps2{}, lateral_acceleration_mps2{}, yaw_rate_radps{};
    SensorReading steering;
    double steering_rad{};
};

// What the plant is at one instant, as the instruments read it: its state, and the accelerations the vehicle seam says
// it is under, which an inertial unit measures and the state alone does not hold.
struct TrueState {
    PlantState plant;
    double longitudinal_acceleration_mps2{};
    double lateral_acceleration_mps2{};
};

// The instruments of one run. Observed at every fixed tick with the plant's true state, it samples each channel when
// that channel's turn has come, applies its error, and delivers the sample once its dead time has passed. Every error
// comes from the settings' seed, so two runs of one seed measure alike, and a recording can be checked against it.
class SensorSuite {
public:
    explicit SensorSuite(SensorSettings settings = {});
    // Advances the instruments to this time, which must not go backwards, with the plant's true state. The first
    // observation starts their cadence, so instruments switched on part way through a run sample from there.
    void observe(const TrueState& state, double time_s);
    // What has arrived by the time last observed.
    const Measurements& measurements() const noexcept { return delivered_; }
    const SensorSettings& settings() const noexcept { return settings_; }
    // Forgets every queued sample and starts the errors again from the seed, as at the start of a run.
    void reset();
    // Identifies the settings and seed: 16 hexadecimal digits, as the steering table and the envelope carry.
    const std::string& fingerprint() const noexcept { return fingerprint_; }

private:
    // One channel's pending samples, in the order they were taken.
    struct Pending {
        double sampled_at_s{}, delivered_at_s{};
        std::array<double, 4> values{};
    };
    struct Channel {
        SensorChannel instrument;
        double last_sample_s{};
        std::vector<Pending> queue;
        std::mt19937_64 random;
    };
    // Samples a channel whose turn has come and delivers what its dead time has released, into `into`.
    template <class Deliver>
    void run(Channel& channel, double time_s, std::size_t values, const std::array<double, 4>& truth,
             const std::array<double, 4>& noise, Deliver deliver);
    SensorSettings settings_;
    std::string fingerprint_;
    Measurements delivered_;
    Channel pose_, speed_, wheels_, imu_, steering_;
    double observed_s_{-1};
};

} // namespace fd

#pragma once
#include "fd/cones.hpp"

#include <cstdint>
#include <random>
#include <span>
#include <string>
#include <vector>

namespace fd {

// Mock cone perception (TrackWayFastPlan Phase 7.2, decision 0030): simulated detections of the course's cones, after
// PacSim's perception sensor (MIT), read for structure only. A sensor mounted on the car samples at its own rate; of the
// cones within its range and field of view, each is detected with a probability that falls with distance, reported with
// a colour that is right with a probability that also falls with distance and unknown beyond a range, and placed with
// noise in bearing, range and position whose covariance is propagated from polar to Cartesian; the frame arrives a dead
// time later. Everything it reports is a simulated detection, never a real one, and it is planar: cones stand on the
// ground plane and have no height.

// What a detection says a cone is. Unknown is a detection's colour only, never a cone's.
enum class ObservedColour { unknown, blue, yellow, orange, big_orange };
const char* observed_colour_name(ObservedColour colour);
ObservedColour observed(ConeColour colour);

// One perception sensor, after PacSim's perception.yaml for its front LiDAR: mounted on the car, from the rear axle,
// forward and left positive, facing its own yaw; ranges, probabilities and noise as that file states them.
struct PerceptionSensorSettings {
    std::string name{"front"};
    double mount_x_m{1.0}, mount_y_m{0.0}, mount_yaw_rad{0.0};
    double rate_hz{10.0};
    double dead_time_s{0.2};
    double min_range_m{1.0}, max_range_m{45.0};
    double half_field_of_view_rad{1.0472};
    // Detection probability at distance d: max(floor, at_min_range - linear (d - min range) - quadratic (d - min range)^2).
    double detection_at_min_range{0.99}, detection_linear_per_m{0.01}, detection_quadratic_per_m2{0.00012};
    double detection_floor{0.1};
    // The probability of the true colour, of the same form; beyond its range every colour is unknown. Its floor of 0.2
    // is guessing among the five observed colours.
    double colour_max_range_m{15.0};
    double colour_at_min_range{0.99}, colour_linear_per_m{0.01}, colour_quadratic_per_m2{0.000125};
    double colour_floor{0.2};
    // Standard deviations: of the bearing, of the range, of the range in proportion to it, and of each Cartesian axis.
    double bearing_noise_rad{0.001};
    double range_noise_m{0.02};
    double range_relative_noise{0.0001};
    double position_noise_m{0.01};
};

// The car's perception: its sensors and the seed every one of their draws comes from.
struct PerceptionSettings {
    std::vector<PerceptionSensorSettings> sensors{PerceptionSensorSettings{}};
    std::uint64_t seed{1};
};
void validate_perception(const PerceptionSettings& settings);

// One detection, in the car's frame at the moment the frame was sampled: rear axle origin, forward x, left y. The
// covariance is of that position, in the same frame. The colour's probability is what the sensor assigns to the colour
// it reports. Nothing here says which cone it was: that is evaluation, kept apart in PerceptionTruth.
struct ConeDetection {
    Vec2 position;
    ObservedColour colour{ObservedColour::unknown};
    double colour_probability{};
    double detection_probability{};
    double covariance_xx{}, covariance_xy{}, covariance_yy{};
};

// Everything one sensor reported of one sample, delivered a dead time after it was taken.
struct PerceptionFrame {
    std::size_t sensor{};
    double sampled_at_s{}, delivered_at_s{};
    std::vector<ConeDetection> detections;
};

// What the evaluation knows about a frame and the planner never may: the car's true pose when it was sampled, which
// cone each detection was, in the frame's order, and which cones stood within range and view but were missed.
struct PerceptionTruth {
    State pose;
    std::vector<std::size_t> source_cone;
    std::vector<std::size_t> missed;
};

// The car's perception of one course. Observed at every fixed tick with the car's true pose, each sensor samples when
// its turn has come, and its frame is delivered once its dead time has passed; every draw comes from the seed, one
// stream per sensor, so a run perceives alike every time.
class Perception {
public:
    Perception(std::vector<Cone> cones, PerceptionSettings settings);
    // Advances every sensor to this time, which must not go backwards, with the car's true rear-axle pose.
    void observe(const State& pose, double time_s);
    // The frames delivered by the last observation, oldest first, and their truth in the same order; both empty when
    // none arrived. Each is replaced by the next observation.
    const std::vector<PerceptionFrame>& delivered() const noexcept { return delivered_; }
    const std::vector<PerceptionTruth>& delivered_truth() const noexcept { return delivered_truth_; }
    // Each sensor's newest delivered frame and its truth, which stay until the next arrives; null before the first.
    const PerceptionFrame* latest(std::size_t sensor) const;
    const PerceptionTruth* latest_truth(std::size_t sensor) const;
    const std::vector<Cone>& cones() const noexcept { return cones_; }
    const PerceptionSettings& settings() const noexcept { return settings_; }
    // Forgets every queued and delivered frame and starts the draws again from the seed.
    void reset();
    // Identifies the settings, the seed and the cones: 16 hexadecimal digits.
    const std::string& fingerprint() const noexcept { return fingerprint_; }

private:
    struct Queued { PerceptionFrame frame; PerceptionTruth truth; };
    struct Sensor {
        PerceptionSensorSettings settings;
        double last_sample_s{};
        std::vector<Queued> queue;
        std::mt19937_64 random;
        bool has_latest{};
        Queued latest;
    };
    Queued sample(Sensor& sensor, std::size_t index, const State& pose, double time_s);
    std::vector<Cone> cones_;
    PerceptionSettings settings_;
    std::string fingerprint_;
    std::vector<Sensor> sensors_;
    std::vector<PerceptionFrame> delivered_;
    std::vector<PerceptionTruth> delivered_truth_;
    double observed_s_{-1};
};

// The detection probability and the probability of the true colour a sensor gives a cone at a distance, as the model
// above states them; the colour's is zero beyond its range, where every colour is unknown.
double detection_probability(const PerceptionSensorSettings& sensor, double distance_m);
double colour_probability(const PerceptionSensorSettings& sensor, double distance_m);

}  // namespace fd

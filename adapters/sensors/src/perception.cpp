#include "fd/perception.hpp"
#include "fingerprint.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <sstream>
#include <stdexcept>

namespace fd {
namespace {

// Changing how a sensor samples, detects, colours or disturbs a cone changes this, and with it every fingerprint.
constexpr const char* generator_version = "perception 1";
// The perception sensors draw from streams numbered apart from the instruments' five.
constexpr std::uint64_t first_perception_stream = 100;

constexpr std::array<ObservedColour, 5> observed_colours{ObservedColour::unknown, ObservedColour::blue, ObservedColour::yellow,
                                                         ObservedColour::orange, ObservedColour::big_orange};

void check(bool ok, const std::string& sensor, const char* what) {
    if (!ok) throw std::invalid_argument("Perception sensor "+sensor+": "+what);
}

bool probability(double p) { return std::isfinite(p) && p >= 0 && p <= 1; }
bool nonnegative(double v, double most) { return std::isfinite(v) && v >= 0 && v <= most; }

// max(floor, at_min_range - linear (d - min range) - quadratic (d - min range)^2), as PacSim's perception states it.
double falling(double at_min_range, double linear, double quadratic, double floor, double from_min_range) {
    return std::max(floor, at_min_range-linear*from_min_range-quadratic*from_min_range*from_min_range);
}

std::string identity(const std::vector<Cone>& cones, const PerceptionSettings& settings) {
    std::ostringstream out;
    out.precision(17);
    out << generator_version << ";seed:" << settings.seed << ';';
    for (const auto& s : settings.sensors)
        out << s.name << ':' << s.mount_x_m << ',' << s.mount_y_m << ',' << s.mount_yaw_rad << ',' << s.rate_hz << ','
            << s.dead_time_s << ',' << s.min_range_m << ',' << s.max_range_m << ',' << s.half_field_of_view_rad << ','
            << s.detection_at_min_range << ',' << s.detection_linear_per_m << ',' << s.detection_quadratic_per_m2 << ','
            << s.detection_floor << ',' << s.colour_max_range_m << ',' << s.colour_at_min_range << ','
            << s.colour_linear_per_m << ',' << s.colour_quadratic_per_m2 << ',' << s.colour_floor << ','
            << s.bearing_noise_rad << ',' << s.range_noise_m << ',' << s.range_relative_noise << ',' << s.position_noise_m << ';';
    out << "cones:" << cones.size() << ';';
    for (const auto& c : cones) out << c.position.x << ',' << c.position.y << ',' << static_cast<int>(c.colour) << ';';
    return out.str();
}

double gaussian(std::mt19937_64& random, double sigma) {
    if (sigma <= 0) return 0;
    std::normal_distribution<double> draw(0.0, sigma);
    return draw(random);
}

}  // namespace

const char* observed_colour_name(ObservedColour colour) {
    switch (colour) {
    case ObservedColour::unknown: return "unknown";
    case ObservedColour::blue: return "blue";
    case ObservedColour::yellow: return "yellow";
    case ObservedColour::orange: return "orange";
    case ObservedColour::big_orange: return "big orange";
    }
    return "unknown";
}

ObservedColour observed(ConeColour colour) {
    switch (colour) {
    case ConeColour::blue: return ObservedColour::blue;
    case ConeColour::yellow: return ObservedColour::yellow;
    case ConeColour::orange: return ObservedColour::orange;
    case ConeColour::big_orange: return ObservedColour::big_orange;
    }
    return ObservedColour::unknown;
}

double detection_probability(const PerceptionSensorSettings& s, double distance_m) {
    return falling(s.detection_at_min_range, s.detection_linear_per_m, s.detection_quadratic_per_m2, s.detection_floor,
                   distance_m-s.min_range_m);
}

double colour_probability(const PerceptionSensorSettings& s, double distance_m) {
    if (distance_m > s.colour_max_range_m) return 0;
    return falling(s.colour_at_min_range, s.colour_linear_per_m, s.colour_quadratic_per_m2, s.colour_floor,
                   distance_m-s.min_range_m);
}

void validate_perception(const PerceptionSettings& settings) {
    if (settings.sensors.empty() || settings.sensors.size() > 8)
        throw std::invalid_argument("Perception needs one to eight sensors");
    for (const auto& s : settings.sensors) {
        const std::string& n = s.name;
        check(!s.name.empty() && s.name.find_first_of(",\"\n\r") == std::string::npos, n, "needs a plain name");
        check(std::isfinite(s.mount_x_m) && std::isfinite(s.mount_y_m) && std::abs(s.mount_x_m) <= 10 && std::abs(s.mount_y_m) <= 10,
              n, "must be mounted within 10 m of the rear axle");
        check(std::isfinite(s.mount_yaw_rad) && std::abs(s.mount_yaw_rad) <= std::numbers::pi, n, "mount yaw must lie within -pi..pi");
        check(std::isfinite(s.rate_hz) && s.rate_hz > 0 && s.rate_hz <= 1000, n, "rate must lie within 0..1000 Hz");
        check(nonnegative(s.dead_time_s, 2), n, "dead time must lie within 0..2 s");
        check(nonnegative(s.min_range_m, 500) && std::isfinite(s.max_range_m) && s.max_range_m > s.min_range_m && s.max_range_m <= 500,
              n, "range must run from 0..500 m to a farther 500 m at most");
        check(std::isfinite(s.half_field_of_view_rad) && s.half_field_of_view_rad > 0 && s.half_field_of_view_rad <= std::numbers::pi,
              n, "half its field of view must lie within 0..pi rad");
        check(probability(s.detection_at_min_range) && probability(s.detection_floor) && nonnegative(s.detection_linear_per_m, 1) &&
                  nonnegative(s.detection_quadratic_per_m2, 1),
              n, "detection probabilities must lie within 0..1, falling by nonnegative terms");
        check(nonnegative(s.colour_max_range_m, 500) && probability(s.colour_at_min_range) && probability(s.colour_floor) &&
                  nonnegative(s.colour_linear_per_m, 1) && nonnegative(s.colour_quadratic_per_m2, 1),
              n, "colour probabilities must lie within 0..1, falling by nonnegative terms");
        check(nonnegative(s.bearing_noise_rad, 1) && nonnegative(s.range_noise_m, 10) && nonnegative(s.range_relative_noise, 1) &&
                  nonnegative(s.position_noise_m, 10),
              n, "noise must be nonnegative and within bounds");
    }
}

Perception::Perception(std::vector<Cone> cones, PerceptionSettings settings)
    : cones_(std::move(cones)), settings_(std::move(settings)) {
    validate_perception(settings_);
    for (const auto& c : cones_)
        if (!std::isfinite(c.position.x) || !std::isfinite(c.position.y))
            throw std::invalid_argument("Every cone must stand at a finite position");
    fingerprint_ = fingerprint_hex(identity(cones_, settings_));
    reset();
}

void Perception::reset() {
    sensors_.clear();
    for (std::size_t i = 0; i < settings_.sensors.size(); ++i)
        sensors_.push_back({settings_.sensors[i], 0, {}, std::mt19937_64(sensors_detail::stream_seed(settings_.seed, first_perception_stream+i)),
                            false, {}});
    delivered_.clear();
    delivered_truth_.clear();
    observed_s_ = -1;
}

const PerceptionFrame* Perception::latest(std::size_t sensor) const {
    return sensor < sensors_.size() && sensors_[sensor].has_latest ? &sensors_[sensor].latest.frame : nullptr;
}

const PerceptionTruth* Perception::latest_truth(std::size_t sensor) const {
    return sensor < sensors_.size() && sensors_[sensor].has_latest ? &sensors_[sensor].latest.truth : nullptr;
}

Perception::Queued Perception::sample(Sensor& sensor, std::size_t index, const State& pose, double time_s) {
    const auto& s = sensor.settings;
    Queued q;
    q.frame.sensor = index;
    q.frame.sampled_at_s = time_s;
    q.frame.delivered_at_s = time_s+s.dead_time_s;
    q.truth.pose = pose;
    // Where the sensor stands and faces on the map.
    const double c = std::cos(pose.yaw_rad), sn = std::sin(pose.yaw_rad);
    const double sx = pose.x_m+c*s.mount_x_m-sn*s.mount_y_m, sy = pose.y_m+sn*s.mount_x_m+c*s.mount_y_m;
    const double facing = pose.yaw_rad+s.mount_yaw_rad;
    const double fc = std::cos(facing), fs = std::sin(facing);
    const double mc = std::cos(s.mount_yaw_rad), ms = std::sin(s.mount_yaw_rad);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    for (std::size_t i = 0; i < cones_.size(); ++i) {
        const double dx = cones_[i].position.x-sx, dy = cones_[i].position.y-sy;
        const double x = fc*dx+fs*dy, y = -fs*dx+fc*dy;  // in the sensor's frame
        const double range = std::hypot(x, y), bearing = std::atan2(y, x);
        if (range < s.min_range_m || range > s.max_range_m || std::abs(bearing) > s.half_field_of_view_rad) continue;
        // Missed with the probability that falls with distance.
        const double detected = detection_probability(s, range);
        if (uniform(sensor.random) >= detected) { q.truth.missed.push_back(i); continue; }
        ConeDetection d;
        d.detection_probability = detected;
        // Coloured: beyond its range unknown for certain; within it the true colour with its probability, the other four
        // sharing the rest, one drawn from those weights. The sensor gives the colour it reports that probability.
        if (range > s.colour_max_range_m) {
            d.colour = ObservedColour::unknown;
            d.colour_probability = 1;
        } else {
            const ObservedColour truth = observed(cones_[i].colour);
            const double right = colour_probability(s, range), other = (1-right)/(observed_colours.size()-1);
            const double u = uniform(sensor.random);
            double sum = 0;
            d.colour = observed_colours.back();
            for (const auto colour : observed_colours) {
                sum += colour == truth ? right : other;
                if (u < sum) { d.colour = colour; break; }
            }
            d.colour_probability = right;
        }
        // Placed with noise in bearing, range, range in proportion and each axis; the covariance is the polar one at
        // the true position carried through the Jacobian of polar to Cartesian, plus the Cartesian terms.
        const double sigma_range_sq = s.range_noise_m*s.range_noise_m+range*range*s.range_relative_noise*s.range_relative_noise;
        const double sigma_bearing_sq = s.bearing_noise_rad*s.bearing_noise_rad;
        const double cb = std::cos(bearing), sb = std::sin(bearing);
        // Columns of the Jacobian: along the range (cb, sb), and along the bearing (-range sb, range cb).
        const double sxx = cb*cb*sigma_range_sq+range*range*sb*sb*sigma_bearing_sq+s.position_noise_m*s.position_noise_m;
        const double sxy = cb*sb*sigma_range_sq-range*range*sb*cb*sigma_bearing_sq;
        const double syy = sb*sb*sigma_range_sq+range*range*cb*cb*sigma_bearing_sq+s.position_noise_m*s.position_noise_m;
        const double noisy_bearing = bearing+gaussian(sensor.random, s.bearing_noise_rad);
        double noisy_range = range+range*gaussian(sensor.random, s.range_relative_noise);
        noisy_range += gaussian(sensor.random, s.range_noise_m);
        const double px = noisy_range*std::cos(noisy_bearing)+gaussian(sensor.random, s.position_noise_m);
        const double py = noisy_range*std::sin(noisy_bearing)+gaussian(sensor.random, s.position_noise_m);
        // Into the car's frame: rotated by the mount's yaw and moved to where it is mounted.
        d.position = {s.mount_x_m+mc*px-ms*py, s.mount_y_m+ms*px+mc*py};
        d.covariance_xx = mc*mc*sxx-2*mc*ms*sxy+ms*ms*syy;
        d.covariance_xy = mc*ms*(sxx-syy)+(mc*mc-ms*ms)*sxy;
        d.covariance_yy = ms*ms*sxx+2*mc*ms*sxy+mc*mc*syy;
        q.frame.detections.push_back(d);
        q.truth.source_cone.push_back(i);
    }
    return q;
}

void Perception::observe(const State& pose, double time_s) {
    if (!std::isfinite(time_s) || time_s < 0 || time_s < observed_s_)
        throw std::invalid_argument("Perception is observed forward in time, from zero");
    if (!std::isfinite(pose.x_m) || !std::isfinite(pose.y_m) || !std::isfinite(pose.yaw_rad))
        throw std::invalid_argument("Perception needs a finite pose");
    // Sensors switched on part way through a run start their cadence there, as the instruments do.
    if (observed_s_ < 0)
        for (auto& sensor : sensors_) sensor.last_sample_s = time_s;
    observed_s_ = time_s;
    delivered_.clear();
    delivered_truth_.clear();
    for (std::size_t i = 0; i < sensors_.size(); ++i) {
        auto& sensor = sensors_[i];
        const double period = 1.0/sensor.settings.rate_hz;
        while (time_s >= sensor.last_sample_s+period-1e-12) {
            sensor.last_sample_s += period;
            sensor.queue.push_back(sample(sensor, i, pose, sensor.last_sample_s));
        }
        std::size_t arrived = 0;
        while (arrived < sensor.queue.size() && time_s >= sensor.queue[arrived].frame.delivered_at_s-1e-12) ++arrived;
        for (std::size_t k = 0; k < arrived; ++k) {
            delivered_.push_back(sensor.queue[k].frame);
            delivered_truth_.push_back(sensor.queue[k].truth);
        }
        if (arrived > 0) {
            sensor.latest = std::move(sensor.queue[arrived-1]);
            sensor.has_latest = true;
            sensor.queue.erase(sensor.queue.begin(), sensor.queue.begin()+static_cast<std::ptrdiff_t>(arrived));
        }
    }
}

}  // namespace fd

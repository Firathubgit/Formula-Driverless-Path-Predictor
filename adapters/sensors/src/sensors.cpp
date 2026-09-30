#include "fd/sensors.hpp"
#include "fd/core.hpp"
#include "fingerprint.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace fd {
namespace {

// Changing how a channel samples, delays or disturbs its values changes this, and with it every fingerprint.
constexpr const char* generator_version = "sensor suite 1";

void validate_channel(const SensorChannel& channel, const char* name) {
    if (!std::isfinite(channel.rate_hz) || channel.rate_hz < 0 || channel.rate_hz > 10000)
        throw std::invalid_argument(std::string("Sensor ")+name+" rate must lie within 0..10000 Hz");
    if (!std::isfinite(channel.dead_time_s) || channel.dead_time_s < 0 || channel.dead_time_s > 1)
        throw std::invalid_argument(std::string("Sensor ")+name+" dead time must lie within 0..1 s");
    if (!std::isfinite(channel.noise) || channel.noise < 0 || channel.noise > 100)
        throw std::invalid_argument(std::string("Sensor ")+name+" noise must lie within 0..100 of its unit");
}

void append(std::ostringstream& out, const char* name, const SensorChannel& channel) {
    out << name << ':' << channel.rate_hz << ',' << channel.dead_time_s << ',' << channel.noise << ';';
}

std::string identity(const SensorSettings& settings) {
    std::ostringstream out;
    out.precision(17);
    out << generator_version << ';';
    append(out, "pose", settings.pose);
    out << "pose_yaw:" << settings.pose_yaw_noise_rad << ';';
    append(out, "speed", settings.speed);
    append(out, "wheel_speeds", settings.wheel_speeds);
    append(out, "imu", settings.imu);
    out << "imu_yaw_rate:" << settings.imu_yaw_rate_noise_radps << ';';
    append(out, "steering", settings.steering);
    out << "seed:" << settings.seed << ';';
    return out.str();
}

// The channels in the order their errors are drawn, so each keeps its own stream whatever the others do.
enum ChannelOrder : std::uint64_t { pose_channel, speed_channel, wheels_channel, imu_channel, steering_channel };

std::mt19937_64 stream(std::uint64_t seed, std::uint64_t channel) {
    return std::mt19937_64(sensors_detail::stream_seed(seed, channel));
}

} // namespace

void validate_sensors(const SensorSettings& settings) {
    validate_channel(settings.pose, "pose");
    validate_channel(settings.speed, "speed");
    validate_channel(settings.wheel_speeds, "wheel speed");
    validate_channel(settings.imu, "inertial unit");
    validate_channel(settings.steering, "steering");
    if (!std::isfinite(settings.pose_yaw_noise_rad) || settings.pose_yaw_noise_rad < 0 || settings.pose_yaw_noise_rad > 3.2)
        throw std::invalid_argument("Sensor pose yaw noise must lie within 0..3.2 rad");
    if (!std::isfinite(settings.imu_yaw_rate_noise_radps) || settings.imu_yaw_rate_noise_radps < 0 || settings.imu_yaw_rate_noise_radps > 100)
        throw std::invalid_argument("Sensor yaw rate noise must lie within 0..100 rad/s");
}

SensorSuite::SensorSuite(SensorSettings settings) : settings_(settings) {
    validate_sensors(settings_);
    fingerprint_ = fingerprint_hex(identity(settings_));
    reset();
}

void SensorSuite::reset() {
    pose_ = {settings_.pose, 0, {}, stream(settings_.seed, pose_channel)};
    speed_ = {settings_.speed, 0, {}, stream(settings_.seed, speed_channel)};
    wheels_ = {settings_.wheel_speeds, 0, {}, stream(settings_.seed, wheels_channel)};
    imu_ = {settings_.imu, 0, {}, stream(settings_.seed, imu_channel)};
    steering_ = {settings_.steering, 0, {}, stream(settings_.seed, steering_channel)};
    delivered_ = {};
    observed_s_ = -1;
}

template <class Deliver>
void SensorSuite::run(Channel& channel, double time_s, std::size_t values, const std::array<double, 4>& truth,
                      const std::array<double, 4>& noise, Deliver deliver) {
    if (channel.instrument.rate_hz <= 0) return;
    const double period = 1.0/channel.instrument.rate_hz;
    // A channel samples when its own turn has come round, and keeps its cadence rather than the tick's.
    while (time_s >= channel.last_sample_s+period-1e-12) {
        channel.last_sample_s += period;
        Pending sample;
        sample.sampled_at_s = channel.last_sample_s;
        sample.delivered_at_s = channel.last_sample_s+channel.instrument.dead_time_s;
        for (std::size_t i = 0; i < values; ++i) {
            // One draw per value, in a fixed order, so the stream depends on the seed alone.
            std::normal_distribution<double> error(0.0, noise[i]);
            sample.values[i] = truth[i]+(noise[i] > 0 ? error(channel.random) : 0.0);
        }
        channel.queue.push_back(sample);
    }
    // Everything whose dead time has passed has arrived; the newest of those is what is read.
    std::size_t arrived = 0;
    while (arrived < channel.queue.size() && time_s >= channel.queue[arrived].delivered_at_s-1e-12) ++arrived;
    if (arrived == 0) return;
    deliver(channel.queue[arrived-1]);
    channel.queue.erase(channel.queue.begin(), channel.queue.begin()+static_cast<std::ptrdiff_t>(arrived));
}

void SensorSuite::observe(const TrueState& state, double time_s) {
    if (!std::isfinite(time_s) || time_s < 0 || time_s < observed_s_)
        throw std::invalid_argument("Sensors are observed forward in time, from zero");
    // Instruments switched on part way through a run start their cadence where they are switched on, rather than
    // catching up on every sample they would have taken since the run began.
    if (observed_s_ < 0)
        pose_.last_sample_s = speed_.last_sample_s = wheels_.last_sample_s = imu_.last_sample_s = steering_.last_sample_s = time_s;
    observed_s_ = time_s;
    const auto& pose = state.plant.pose;
    run(pose_, time_s, 3, {pose.x_m, pose.y_m, pose.yaw_rad, 0},
        {settings_.pose.noise, settings_.pose.noise, settings_.pose_yaw_noise_rad, 0}, [&](const Pending& sample) {
            delivered_.pose = {true, sample.sampled_at_s};
            delivered_.x_m = sample.values[0];
            delivered_.y_m = sample.values[1];
            delivered_.yaw_rad = wrap_angle(sample.values[2]);
        });
    run(speed_, time_s, 1, {pose.speed_mps, 0, 0, 0}, {settings_.speed.noise, 0, 0, 0}, [&](const Pending& sample) {
        delivered_.speed = {true, sample.sampled_at_s};
        delivered_.speed_mps = sample.values[0];
    });
    run(wheels_, time_s, 4,
        {state.plant.wheel_speeds_radps[0], state.plant.wheel_speeds_radps[1], state.plant.wheel_speeds_radps[2],
         state.plant.wheel_speeds_radps[3]},
        {settings_.wheel_speeds.noise, settings_.wheel_speeds.noise, settings_.wheel_speeds.noise, settings_.wheel_speeds.noise},
        [&](const Pending& sample) {
            delivered_.wheel_speeds = {true, sample.sampled_at_s};
            for (std::size_t i = 0; i < 4; ++i) delivered_.wheel_speeds_radps[i] = sample.values[i];
        });
    run(imu_, time_s, 3, {state.longitudinal_acceleration_mps2, state.lateral_acceleration_mps2, state.plant.yaw_rate_radps, 0},
        {settings_.imu.noise, settings_.imu.noise, settings_.imu_yaw_rate_noise_radps, 0}, [&](const Pending& sample) {
            delivered_.imu = {true, sample.sampled_at_s};
            delivered_.longitudinal_acceleration_mps2 = sample.values[0];
            delivered_.lateral_acceleration_mps2 = sample.values[1];
            delivered_.yaw_rate_radps = sample.values[2];
        });
    run(steering_, time_s, 1, {pose.steering_rad, 0, 0, 0}, {settings_.steering.noise, 0, 0, 0}, [&](const Pending& sample) {
        delivered_.steering = {true, sample.sampled_at_s};
        delivered_.steering_rad = sample.values[0];
    });
}

} // namespace fd

#include "fd/sensors.hpp"
#include "fd/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tolerance)
        throw std::runtime_error(message+": actual="+std::to_string(actual)+", expected="+std::to_string(expected));
}

const double tick = 0.005;

// A car standing still at a known pose, so what a channel reports is its error alone.
fd::TrueState still(double x, double y, double yaw, double speed, double steering) {
    fd::TrueState state;
    state.plant.pose = {x, y, yaw, speed, steering, 0};
    return state;
}

// Settings with one channel's instrument and every other channel switched off.
template <class Pick>
fd::SensorSettings only(Pick pick, fd::SensorChannel instrument) {
    fd::SensorSettings settings;
    settings.pose.rate_hz = settings.speed.rate_hz = settings.wheel_speeds.rate_hz = 0;
    settings.imu.rate_hz = settings.steering.rate_hz = 0;
    pick(settings) = instrument;
    return settings;
}

// A channel samples at its own rate: its first sample is one period in, and it reports a new one each period, whatever
// the tick.
void a_channel_samples_at_its_own_rate() {
    auto settings = only([](fd::SensorSettings& s) -> fd::SensorChannel& { return s.speed; }, {20, 0.0, 0.0});
    fd::SensorSuite sensors(settings);
    std::vector<double> sampled;
    for (int i = 0; i <= 400; ++i) {
        const double time = i*tick;
        sensors.observe(still(0, 0, 0, 10+time, 0), time);
        const auto& measured = sensors.measurements();
        if (measured.speed.measured && (sampled.empty() || measured.speed.sampled_at_s != sampled.back()))
            sampled.push_back(measured.speed.sampled_at_s);
    }
    require(!sampled.empty(), "a channel that is on delivers something");
    near(sampled.front(), 0.05, 1e-12, "a 20 Hz channel takes its first sample one period in");
    require(sampled.size() == 40, "and one every period over two seconds");
    for (std::size_t i = 1; i < sampled.size(); ++i) near(sampled[i]-sampled[i-1], 0.05, 1e-12, "its samples are a period apart");
    // What it reports is the truth at the moment it sampled, not at the moment it was read.
    fd::SensorSuite exact(settings);
    exact.observe(still(0, 0, 0, 10, 0), 0.0);
    exact.observe(still(0, 0, 0, 11, 0), 0.05);
    exact.observe(still(0, 0, 0, 99, 0), 0.10-tick);
    near(exact.measurements().speed_mps, 11, 1e-12, "the value read is the value sampled");
    // A channel switched off never reports.
    fd::SensorSuite off(only([](fd::SensorSettings& s) -> fd::SensorChannel& { return s.speed; }, {0, 0, 0}));
    for (int i = 0; i <= 400; ++i) off.observe(still(0, 0, 0, 10, 0), i*tick);
    require(!off.measurements().speed.measured, "a channel with no rate measures nothing");
}

// A sample arrives its dead time after it was taken, and until then the older value stands.
void a_sample_arrives_a_dead_time_later() {
    auto settings = only([](fd::SensorSettings& s) -> fd::SensorChannel& { return s.steering; }, {100, 0.02, 0.0});
    fd::SensorSuite sensors(settings);
    // The first sample is taken at 0.01 s and arrives at 0.03 s.
    for (double time = 0; time <= 0.025+1e-12; time += tick) sensors.observe(still(0, 0, 0, 0, 0.1), time);
    require(!sensors.measurements().steering.measured, "nothing has arrived before its dead time has passed");
    sensors.observe(still(0, 0, 0, 0, 0.9), 0.030);
    require(sensors.measurements().steering.measured, "the sample arrives a dead time after it was taken");
    near(sensors.measurements().steering.sampled_at_s, 0.010, 1e-12, "and says when it was taken");
    near(sensors.measurements().steering_rad, 0.1, 1e-12, "carrying what was true then, not what is true now");
    // Reading on, the value catches up to within a dead time of the truth.
    for (double time = 0.030; time <= 0.2+1e-12; time += tick) sensors.observe(still(0, 0, 0, 0, 0.9), time);
    near(sensors.measurements().steering_rad, 0.9, 1e-12, "once the queue has run through, it reads the new truth");
    near(0.2-sensors.measurements().steering.sampled_at_s, 0.02, 0.011, "and stays about a dead time behind");
}

// The error on each value is Gaussian with the channel's own standard deviation, and it is the measurement that is
// disturbed, not the plant.
void the_error_is_the_channel_s_own() {
    fd::SensorSettings settings;
    settings.pose.rate_hz = 200;
    settings.pose.dead_time_s = 0;
    settings.pose.noise = 0.03;
    settings.pose_yaw_noise_rad = 0.02;
    settings.speed.rate_hz = 200;
    settings.speed.dead_time_s = 0;
    settings.speed.noise = 0.02;
    fd::SensorSuite sensors(settings);
    const auto truth = still(12.5, -3.25, 0.4, 17.0, 0.05);
    std::vector<double> x_errors, yaw_errors, speed_errors;
    double last = -1;
    for (int i = 1; i <= 40000; ++i) {
        const double time = i*tick;
        sensors.observe(truth, time);
        const auto& measured = sensors.measurements();
        if (!measured.pose.measured || !measured.speed.measured || measured.pose.sampled_at_s == last) continue;
        last = measured.pose.sampled_at_s;
        x_errors.push_back(measured.x_m-truth.plant.pose.x_m);
        yaw_errors.push_back(measured.yaw_rad-truth.plant.pose.yaw_rad);
        speed_errors.push_back(measured.speed_mps-truth.plant.pose.speed_mps);
    }
    const auto spread = [](const std::vector<double>& values) {
        double sum = 0, square = 0;
        for (const double v : values) { sum += v; square += v*v; }
        const double mean = sum/static_cast<double>(values.size());
        return std::make_pair(mean, std::sqrt(square/static_cast<double>(values.size())-mean*mean));
    };
    const auto [x_mean, x_sigma] = spread(x_errors);
    const auto [yaw_mean, yaw_sigma] = spread(yaw_errors);
    const auto [speed_mean, speed_sigma] = spread(speed_errors);
    std::cout << "  over " << x_errors.size() << " samples: position error mean " << x_mean << " m, sigma " << x_sigma
              << " m; yaw sigma " << yaw_sigma << " rad; speed sigma " << speed_sigma << " m/s\n";
    require(x_errors.size() > 1000, "the sample is large enough to measure a spread");
    near(x_mean, 0, 0.005, "the position error has no bias");
    near(x_sigma, 0.03, 0.002, "and the standard deviation the channel states");
    near(yaw_mean, 0, 0.004, "the yaw error has no bias");
    near(yaw_sigma, 0.02, 0.002, "and its own standard deviation");
    near(speed_sigma, 0.02, 0.002, "the speed error has the speed channel's standard deviation");
    // The plant is untouched: the suite reads a copy.
    auto state = truth;
    sensors.observe(state, 400.0);
    require(state.plant.pose.x_m == truth.plant.pose.x_m && state.plant.pose.speed_mps == truth.plant.pose.speed_mps,
            "observing does not disturb the state it reads");
}

// Each channel draws from its own stream, so switching one off, or changing what it measures, leaves the others
// measuring exactly as they did. One shared stream would couple them.
void each_channel_draws_from_its_own_stream() {
    const auto steering_of = [](fd::SensorSettings settings) {
        fd::SensorSuite sensors(settings);
        std::vector<double> values;
        for (int i = 1; i <= 400; ++i) {
            sensors.observe(still(3.0, 4.0, 0.2, 20.0, 0.05), i*tick);
            values.push_back(sensors.measurements().steering_rad);
        }
        return values;
    };
    fd::SensorSettings all;
    const auto with_everything = steering_of(all);
    auto without_pose = all;
    without_pose.pose.rate_hz = 0;
    require(steering_of(without_pose) == with_everything, "switching the pose channel off leaves the steering's errors alone");
    auto noisier_wheels = all;
    noisier_wheels.wheel_speeds.noise = 5;
    require(steering_of(noisier_wheels) == with_everything, "and so does changing what another channel's error is");
    auto slower_imu = all;
    slower_imu.imu.rate_hz = 50;
    require(steering_of(slower_imu) == with_everything, "or how often it samples");
    auto quieter_steering = all;
    quieter_steering.steering.noise = 1e-6;
    require(steering_of(quieter_steering) != with_everything, "while changing the channel's own error changes it");
    // No two channels draw the same sequence: with every channel sampling together, their first errors, each over its
    // own standard deviation, all differ. Two channels given one stream would err in step.
    fd::SensorSettings together;
    for (auto* channel : {&together.pose, &together.speed, &together.wheel_speeds, &together.imu, &together.steering}) {
        channel->rate_hz = 200;
        channel->dead_time_s = 0;
        channel->noise = 1;
    }
    together.pose_yaw_noise_rad = together.imu_yaw_rate_noise_radps = 1;
    fd::SensorSuite sensors(together);
    sensors.observe(still(0, 0, 0, 0, 0), 0);
    sensors.observe(still(0, 0, 0, 0, 0), tick);
    const auto& m = sensors.measurements();
    const std::vector<double> first{m.x_m, m.speed_mps, m.wheel_speeds_radps[0], m.longitudinal_acceleration_mps2, m.steering_rad};
    for (std::size_t i = 0; i < first.size(); ++i)
        for (std::size_t j = i+1; j < first.size(); ++j)
            require(first[i] != first[j], "each channel's first error is its own, channel "+std::to_string(i)+" against "+std::to_string(j));
}

// Instruments switched on part way through a run start their cadence there, rather than catching up on every sample
// they would have taken since the run began.
void instruments_start_when_they_are_first_read() {
    auto settings = only([](fd::SensorSettings& s) -> fd::SensorChannel& { return s.speed; }, {20, 0.0, 0.0});
    fd::SensorSuite sensors(settings);
    sensors.observe(still(0, 0, 0, 30, 0), 10.0);
    require(!sensors.measurements().speed.measured, "switched on at ten seconds, nothing has been sampled yet");
    sensors.observe(still(0, 0, 0, 30, 0), 10.05);
    require(sensors.measurements().speed.measured, "the first sample comes one period after it was switched on");
    near(sensors.measurements().speed.sampled_at_s, 10.05, 1e-12, "dated when it was taken, not backdated to zero");
    // And it keeps that cadence from there: two more seconds is forty more samples, not forty thousand.
    std::vector<double> sampled{10.05};
    for (double time = 10.05; time <= 12.05+1e-12; time += tick) {
        sensors.observe(still(0, 0, 0, 30, 0), time);
        if (sensors.measurements().speed.sampled_at_s != sampled.back()) sampled.push_back(sensors.measurements().speed.sampled_at_s);
    }
    require(sampled.size() == 41, "one sample a period, counted from when the instruments were switched on");
}

// Instruments on a running car read it without changing it: the plant drives exactly as it would without them, and
// what they deliver is an older, noisier car than the one that is actually driving.
void instruments_read_a_run_without_changing_it() {
    const fd::Config config;
    fd::Simulation bare(fd::make_preset_track(), config, fd::DynamicSingleTrack{});
    fd::Simulation watched(fd::make_preset_track(), config, fd::DynamicSingleTrack{});
    fd::SensorSettings settings;
    settings.seed = 42;
    watched.set_sensors(settings);
    require(watched.sensors() != nullptr && watched.measurements() != nullptr, "the car has instruments");
    require(bare.measurements() == nullptr, "and the car without them has none");
    require(!watched.measurements()->pose.measured, "which have delivered nothing at the start of the run");
    double worst_age = 0, worst_position_error = 0, worst_speed_error = 0, age_sum = 0;
    int ages = 0;
    for (int i = 0; i < 2000; ++i) {
        bare.step();
        watched.step();
        const auto& truth = watched.plant_state();
        const auto& copy = bare.plant_state();
        require(truth.pose.x_m == copy.pose.x_m && truth.pose.y_m == copy.pose.y_m &&
                truth.pose.speed_mps == copy.pose.speed_mps && truth.yaw_rate_radps == copy.yaw_rate_radps,
                "watching the car does not move it");
        const auto& m = *watched.measurements();
        if (!m.pose.measured) continue;
        const double age = truth.pose.time_s-m.pose.sampled_at_s;
        worst_age = std::max(worst_age, age);
        age_sum += age;
        ++ages;
        worst_position_error = std::max(worst_position_error,
                                        std::hypot(m.x_m-truth.pose.x_m, m.y_m-truth.pose.y_m));
        if (m.speed.measured) worst_speed_error = std::max(worst_speed_error, std::abs(m.speed_mps-truth.pose.speed_mps));
    }
    std::cout << "  over ten seconds at " << watched.state().speed_mps << " m/s: pose up to " << worst_age*1000
              << " ms old (" << age_sum/ages*1000 << " ms on average), out by up to " << worst_position_error
              << " m, speed by up to " << worst_speed_error << " m/s\n";
    require(ages > 100, "the pose was measured throughout");
    // A 20 Hz channel behind a 50 ms dead time is between one and two periods old when it is read.
    near(worst_age, 0.095, 0.011, "the pose read is a dead time plus up to a period old");
    require(age_sum/ages > 0.05, "and on average older than the dead time alone");
    // That age, at speed, is metres of position: the error is not the noise alone.
    require(worst_position_error > 4*settings.pose.noise,
            "the pose read is out by far more than its noise, because it is also out of date");
    require(worst_speed_error > 3*settings.speed.noise, "and the speed likewise");
    // The instruments are the car's own: replacing them restarts their cadence where the car is now.
    auto quiet = settings;
    quiet.pose.noise = 0;
    quiet.pose_yaw_noise_rad = 0;
    watched.set_sensors(quiet);
    require(!watched.measurements()->pose.measured, "new instruments have delivered nothing yet");
    for (int i = 0; i < 40; ++i) watched.step();
    const auto& m = *watched.measurements();
    require(m.pose.measured, "and deliver once their first sample has arrived");
    require(m.pose.sampled_at_s > 9.9, "sampled since they were fitted, not before");
    // With no noise left, what is read is exactly where the car was when it was sampled, an age behind.
    require(std::hypot(m.x_m-watched.state().x_m, m.y_m-watched.state().y_m) > 0.5,
            "an exact instrument still reports an out-of-date car");
    // A run resets its instruments with it.
    watched.reset();
    require(!watched.measurements()->pose.measured, "a reset run has measured nothing yet");
}

// One seed, one run: the same errors in the same order. Another seed measures differently, and the fingerprint says so.
void a_seed_makes_a_run_reproducible() {
    fd::SensorSettings settings;
    settings.seed = 7;
    const auto stream = [](fd::SensorSettings s) {
        fd::SensorSuite sensors(s);
        std::vector<double> values;
        for (int i = 1; i <= 2000; ++i) {
            const double time = i*tick;
            sensors.observe(still(1.0, 2.0, 0.1, 15.0, 0.02), time);
            const auto& m = sensors.measurements();
            values.push_back(m.x_m);
            values.push_back(m.speed_mps);
            values.push_back(m.steering_rad);
            values.push_back(m.yaw_rate_radps);
        }
        return values;
    };
    const auto first = stream(settings);
    const auto again = stream(settings);
    require(first == again, "one seed measures the same run twice, value for value");
    auto other = settings;
    other.seed = 8;
    const auto different = stream(other);
    require(different.size() == first.size() && different != first, "another seed measures differently");
    fd::SensorSuite sensors(settings);
    const std::string fingerprint = sensors.fingerprint();
    require(fingerprint.size() == 16, "the settings and seed carry a fingerprint");
    require(fd::SensorSuite(other).fingerprint() != fingerprint, "which another seed changes");
    auto quieter = settings;
    quieter.pose.noise = 0.01;
    require(fd::SensorSuite(quieter).fingerprint() != fingerprint, "and another instrument too");
    // reset() starts the errors again from the seed.
    for (int i = 1; i <= 100; ++i) sensors.observe(still(1.0, 2.0, 0.1, 15.0, 0.02), i*tick);
    const auto before_reset = sensors.measurements().x_m;
    sensors.reset();
    require(!sensors.measurements().pose.measured, "reset forgets what had arrived");
    for (int i = 1; i <= 100; ++i) sensors.observe(still(1.0, 2.0, 0.1, 15.0, 0.02), i*tick);
    near(sensors.measurements().x_m, before_reset, 0, "and measures the run again exactly");
}

// Every channel reports what it is for, and settings outside their ranges are refused.
void every_channel_reports_its_own_quantity() {
    fd::SensorSettings settings;
    settings.pose.dead_time_s = settings.speed.dead_time_s = settings.wheel_speeds.dead_time_s = 0;
    settings.imu.dead_time_s = settings.steering.dead_time_s = 0;
    settings.pose.noise = settings.speed.noise = settings.wheel_speeds.noise = settings.imu.noise = settings.steering.noise = 0;
    settings.pose_yaw_noise_rad = settings.imu_yaw_rate_noise_radps = 0;
    fd::SensorSuite sensors(settings);
    fd::TrueState state = still(4.0, 5.0, 0.3, 18.0, 0.07);
    state.plant.yaw_rate_radps = 0.25;
    state.plant.wheel_speeds_radps = {60, 61, 62, 63};
    state.longitudinal_acceleration_mps2 = -3.5;
    state.lateral_acceleration_mps2 = 7.5;
    for (double time = 0; time <= 0.2+1e-12; time += tick) sensors.observe(state, time);
    const auto& m = sensors.measurements();
    require(m.pose.measured && m.speed.measured && m.wheel_speeds.measured && m.imu.measured && m.steering.measured,
            "every channel has reported");
    near(m.x_m, 4.0, 1e-12, "the pose channel reports where the car is");
    near(m.y_m, 5.0, 1e-12, "in both axes");
    near(m.yaw_rad, 0.3, 1e-12, "and which way it faces");
    near(m.speed_mps, 18.0, 1e-12, "the speed channel reports how fast it goes");
    near(m.steering_rad, 0.07, 1e-12, "the steering channel reports the wheels' angle");
    near(m.yaw_rate_radps, 0.25, 1e-12, "the inertial unit reports the yaw rate");
    near(m.longitudinal_acceleration_mps2, -3.5, 1e-12, "and both accelerations");
    near(m.lateral_acceleration_mps2, 7.5, 1e-12, "as the seam gives them");
    for (std::size_t i = 0; i < 4; ++i)
        near(m.wheel_speeds_radps[i], 60.0+static_cast<double>(i), 1e-12, "the wheel channel reports each wheel");
    const auto refused = [](const std::function<void(fd::SensorSettings&)>& change) {
        fd::SensorSettings bad;
        change(bad);
        try { fd::SensorSuite sensors(bad); } catch (const std::invalid_argument&) { return true; }
        return false;
    };
    require(refused([](fd::SensorSettings& s) { s.pose.rate_hz = -1; }), "a negative rate is refused");
    require(refused([](fd::SensorSettings& s) { s.imu.dead_time_s = 2; }), "a dead time beyond a second is refused");
    require(refused([](fd::SensorSettings& s) { s.steering.noise = -0.1; }), "a negative error is refused");
    require(refused([](fd::SensorSettings& s) { s.pose_yaw_noise_rad = 4; }), "a yaw error beyond a turn is refused");
    fd::SensorSuite forward(settings);
    forward.observe(state, 1.0);
    bool backwards = false;
    try { forward.observe(state, 0.5); } catch (const std::invalid_argument&) { backwards = true; }
    require(backwards, "the instruments cannot be asked to read the past");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"a_channel_samples_at_its_own_rate", a_channel_samples_at_its_own_rate},
        {"a_sample_arrives_a_dead_time_later", a_sample_arrives_a_dead_time_later},
        {"the_error_is_the_channel_s_own", the_error_is_the_channel_s_own},
        {"each_channel_draws_from_its_own_stream", each_channel_draws_from_its_own_stream},
        {"instruments_start_when_they_are_first_read", instruments_start_when_they_are_first_read},
        {"a_seed_makes_a_run_reproducible", a_seed_makes_a_run_reproducible},
        {"instruments_read_a_run_without_changing_it", instruments_read_a_run_without_changing_it},
        {"every_channel_reports_its_own_quantity", every_channel_reports_its_own_quantity}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " sensor groups passed\n";
    return failures ? 1 : 0;
}

#include "fd/perception.hpp"
#include "fd/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
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

// A sensor that sees everything within its view exactly, so what a test measures is one effect at a time.
fd::PerceptionSensorSettings exact() {
    fd::PerceptionSensorSettings s;
    s.dead_time_s = 0;
    s.detection_at_min_range = 1; s.detection_linear_per_m = 0; s.detection_quadratic_per_m2 = 0; s.detection_floor = 1;
    s.colour_at_min_range = 1; s.colour_linear_per_m = 0; s.colour_quadratic_per_m2 = 0; s.colour_floor = 1;
    s.colour_max_range_m = s.max_range_m;
    s.bearing_noise_rad = s.range_noise_m = s.range_relative_noise = s.position_noise_m = 0;
    return s;
}

fd::PerceptionSettings one(const fd::PerceptionSensorSettings& sensor, std::uint64_t seed = 1) {
    fd::PerceptionSettings settings;
    settings.sensors = {sensor};
    settings.seed = seed;
    return settings;
}

// Drives a perception forward from rest at a fixed pose, collecting every frame delivered.
std::vector<std::pair<fd::PerceptionFrame, fd::PerceptionTruth>> run(fd::Perception& perception, const fd::State& pose, double seconds) {
    std::vector<std::pair<fd::PerceptionFrame, fd::PerceptionTruth>> frames;
    for (int i = 0; i*tick <= seconds+1e-9; ++i) {
        perception.observe(pose, i*tick);
        for (std::size_t k = 0; k < perception.delivered().size(); ++k)
            frames.emplace_back(perception.delivered()[k], perception.delivered_truth()[k]);
    }
    return frames;
}

// A sensor reports only the cones within its range and its field of view, and says where they stand in the car's frame.
void a_sensor_sees_only_what_is_in_range_and_view() {
    // The sensor stands a metre ahead of the rear axle, facing forward; the car sits at the origin facing +x.
    const std::vector<fd::Cone> cones{
        {{1.5, 0}, fd::ConeColour::blue},      // half a metre from the sensor: too near
        {{11, 2}, fd::ConeColour::yellow},     // in view
        {{60, 0}, fd::ConeColour::blue},       // too far
        {{1+10*std::cos(1.2), 10*std::sin(1.2)}, fd::ConeColour::blue},  // 69 degrees off: out of view
        {{-10, 0}, fd::ConeColour::orange},    // behind
        {{31, -12}, fd::ConeColour::big_orange}};  // in view
    fd::Perception perception(cones, one(exact()));
    const auto frames = run(perception, {0, 0, 0, 0, 0, 0}, 0.1);
    require(frames.size() == 1, "one frame in its first period");
    const auto& [frame, truth] = frames.front();
    require(truth.source_cone == std::vector<std::size_t>{1, 5}, "it detects exactly the cones in range and view");
    require(truth.missed.empty(), "and misses none of them when it cannot miss");
    near(frame.detections[0].position.x, 11, 1e-9, "a detection stands where its cone does, in the car's frame");
    near(frame.detections[0].position.y, 2, 1e-9, "on both axes");
    require(frame.detections[0].colour == fd::ObservedColour::yellow && frame.detections[1].colour == fd::ObservedColour::big_orange,
            "with its colour when the sensor cannot mistake it");
}

// The car's frame is the rear axle's: a detection turned and moved by the car's true pose stands on its cone, whatever
// the pose and wherever the sensor is mounted.
void detections_are_in_the_car_s_frame() {
    const std::vector<fd::Cone> cones{{{120, 60}, fd::ConeColour::blue}, {{112, 71}, fd::ConeColour::yellow}};
    auto sensor = exact();
    sensor.mount_x_m = 1.5; sensor.mount_y_m = 0.2; sensor.mount_yaw_rad = 0.3;
    fd::Perception perception(cones, one(sensor));
    const fd::State pose{100, 50, 0.8, 0, 0, 0};
    const auto frames = run(perception, pose, 0.1);
    require(frames.size() == 1 && frames[0].first.detections.size() == 2, "both cones are seen");
    for (std::size_t k = 0; k < 2; ++k) {
        const auto& p = frames[0].first.detections[k].position;
        const double x = pose.x_m+std::cos(pose.yaw_rad)*p.x-std::sin(pose.yaw_rad)*p.y;
        const double y = pose.y_m+std::sin(pose.yaw_rad)*p.x+std::cos(pose.yaw_rad)*p.y;
        const auto& cone = cones[frames[0].second.source_cone[k]].position;
        near(x, cone.x, 1e-9, "turned and moved by the pose, a detection stands on its cone");
        near(y, cone.y, 1e-9, "on both axes");
    }
}

// Each cone in view is detected with the probability PacSim's model gives its distance, and the rest are listed missed.
void detection_falls_with_distance() {
    const std::vector<fd::Cone> cones{{{6, 0}, fd::ConeColour::blue}, {{36, 0}, fd::ConeColour::blue}};
    fd::PerceptionSensorSettings sensor;
    sensor.dead_time_s = 0;
    fd::Perception perception(cones, one(sensor, 7));
    const auto frames = run(perception, {0, 0, 0, 0, 0, 0}, 400);
    std::map<std::size_t, double> seen, missed;
    for (const auto& [frame, truth] : frames) {
        require(frame.detections.size() == truth.source_cone.size(), "the truth names every detection's cone");
        require(truth.source_cone.size()+truth.missed.size() == 2, "every cone in view is either detected or missed");
        for (const auto i : truth.source_cone) seen[i] += 1;
        for (const auto i : truth.missed) missed[i] += 1;
    }
    const double near_rate = seen[0]/(seen[0]+missed[0]), far_rate = seen[1]/(seen[1]+missed[1]);
    const double near_p = fd::detection_probability(sensor, 5), far_p = fd::detection_probability(sensor, 35);
    std::cout << "  over " << frames.size() << " frames: detected " << near_rate << " at 5 m (model " << near_p << "), "
              << far_rate << " at 35 m (model " << far_p << ")\n";
    near(near_p, 0.99-0.04-0.00012*16, 1e-12, "the model is PacSim's at 5 m");
    near(far_p, 0.99-0.34-0.00012*34*34, 1e-12, "and at 35 m");
    near(near_rate, near_p, 0.02, "a near cone is detected as often as the model says");
    near(far_rate, far_p, 0.03, "and a far one");
    near(fd::detection_probability(sensor, 200), sensor.detection_floor, 1e-12, "never below the floor");
}

// A colour is right less often far away, unknown beyond its range for certain, and when it is wrong, any of the other
// four colours, unknown among them, is as likely as another.
void colour_is_right_less_often_far_away() {
    const std::vector<fd::Cone> cones{{{4, 0}, fd::ConeColour::blue}, {{15, 0}, fd::ConeColour::yellow}, {{21, 0}, fd::ConeColour::blue}};
    fd::PerceptionSensorSettings sensor = exact();
    sensor.colour_max_range_m = 15;
    sensor.colour_at_min_range = 0.99; sensor.colour_linear_per_m = 0.01; sensor.colour_quadratic_per_m2 = 0.000125;
    sensor.colour_floor = 0.2;
    fd::Perception perception(cones, one(sensor, 3));
    const auto frames = run(perception, {0, 0, 0, 0, 0, 0}, 400);
    std::map<fd::ObservedColour, double> near_colours, mid_colours;
    double count = 0;
    for (const auto& [frame, truth] : frames) {
        require(frame.detections.size() == 3, "every cone is seen");
        near_colours[frame.detections[0].colour] += 1;
        mid_colours[frame.detections[1].colour] += 1;
        require(frame.detections[2].colour == fd::ObservedColour::unknown && frame.detections[2].colour_probability == 1,
                "beyond its range a colour is unknown for certain");
        near(frame.detections[0].colour_probability, fd::colour_probability(sensor, 3), 1e-12,
             "the sensor gives the colour it reports its probability");
        count += 1;
    }
    const double near_right = near_colours[fd::ObservedColour::blue]/count, mid_right = mid_colours[fd::ObservedColour::yellow]/count;
    const double near_p = fd::colour_probability(sensor, 3), mid_p = fd::colour_probability(sensor, 14);
    std::cout << "  right colour " << near_right << " at 3 m (model " << near_p << "), " << mid_right << " at 14 m (model "
              << mid_p << ")\n";
    near(near_right, near_p, 0.02, "a near colour is right as often as the model says");
    near(mid_right, mid_p, 0.03, "and one at the edge of the colour range");
    require(near_p > mid_p, "farther colours are right less often");
    // The mistaken colours at 14 m share the rest evenly.
    const double wrong = count-mid_colours[fd::ObservedColour::yellow];
    for (const auto colour : {fd::ObservedColour::unknown, fd::ObservedColour::blue, fd::ObservedColour::orange, fd::ObservedColour::big_orange})
        near(mid_colours[colour]/wrong, 0.25, 0.08, std::string("a mistaken colour is as often ")+fd::observed_colour_name(colour));
    near(fd::colour_probability(sensor, 16), 0, 0, "beyond the colour range no colour is right");
}

// A detection's scatter is the covariance it reports: polar noise carried into the car's frame through the Jacobian,
// turned with the sensor's mount.
void noise_matches_its_propagated_covariance() {
    const std::vector<fd::Cone> cones{{{1+20*std::cos(0.5), 20*std::sin(0.5)}, fd::ConeColour::blue}};
    auto sensor = exact();
    sensor.bearing_noise_rad = 0.01; sensor.range_noise_m = 0.1; sensor.range_relative_noise = 0.002; sensor.position_noise_m = 0.02;
    fd::Perception perception(cones, one(sensor, 11));
    const auto frames = run(perception, {0, 0, 0, 0, 0, 0}, 1000);
    double mx = 0, my = 0;
    for (const auto& f : frames) { mx += f.first.detections[0].position.x; my += f.first.detections[0].position.y; }
    const double n = static_cast<double>(frames.size());
    mx /= n; my /= n;
    double sxx = 0, sxy = 0, syy = 0;
    for (const auto& f : frames) {
        const double dx = f.first.detections[0].position.x-mx, dy = f.first.detections[0].position.y-my;
        sxx += dx*dx; sxy += dx*dy; syy += dy*dy;
    }
    sxx /= n-1; sxy /= n-1; syy /= n-1;
    const auto& d = frames.front().first.detections[0];
    std::cout << "  over " << frames.size() << " frames at 20 m: scatter xx " << sxx << " xy " << sxy << " yy " << syy
              << "; reported " << d.covariance_xx << ", " << d.covariance_xy << ", " << d.covariance_yy << '\n';
    near(sxx, d.covariance_xx, 0.08*d.covariance_xx, "the scatter along x is the reported variance");
    near(syy, d.covariance_yy, 0.08*d.covariance_yy, "and along y");
    near(sxy, d.covariance_xy, 0.1*std::sqrt(d.covariance_xx*d.covariance_yy), "and their covariance");
    require(d.covariance_yy > d.covariance_xx*1.5 || d.covariance_xx > d.covariance_yy*1.5,
            "at twenty metres the bearing's share makes the ellipse long across the line of sight");
    // The same cone seen by a sensor turned on its mount reports the same ellipse on the map.
    auto turned = sensor;
    turned.mount_yaw_rad = 0.4;
    fd::Perception rotated(cones, one(turned, 11));
    const auto again = run(rotated, {0, 0, 0, 0, 0, 0}, 0.1);
    const auto& r = again.front().first.detections[0];
    near(r.covariance_xx, d.covariance_xx, 1e-12, "a sensor turned on its mount reports the same ellipse in the car's frame");
    near(r.covariance_xy, d.covariance_xy, 1e-12, "with the same tilt");
    near(r.covariance_yy, d.covariance_yy, 1e-12, "and the same width");
}

// A sensor samples at its own rate, and its frame arrives a dead time later; the newest stays until the next.
void a_frame_arrives_a_dead_time_later() {
    const std::vector<fd::Cone> cones{{{10, 0}, fd::ConeColour::blue}};
    auto sensor = exact();
    sensor.dead_time_s = 0.2;
    fd::Perception perception(cones, one(sensor));
    std::vector<double> sampled, delivered;
    for (int i = 0; i*tick <= 1.0+1e-9; ++i) {
        const double time = i*tick;
        perception.observe({0, 0, 0, 0, 0, 0}, time);
        for (const auto& frame : perception.delivered()) { sampled.push_back(frame.sampled_at_s); delivered.push_back(time); }
        if (time < 0.3-1e-9) require(perception.latest(0) == nullptr, "nothing has arrived before its dead time");
    }
    require(sampled.size() == 8, "ten frames a second, the last two still on their way at one second");
    near(sampled.front(), 0.1, 1e-9, "the first frame is sampled one period in");
    near(delivered.front(), 0.3, 1e-9, "and arrives its dead time later");
    for (std::size_t k = 1; k < sampled.size(); ++k) near(sampled[k]-sampled[k-1], 0.1, 1e-9, "a period apart");
    require(perception.latest(0) != nullptr && perception.latest_truth(0) != nullptr, "the newest frame stays until the next");
    near(perception.latest(0)->sampled_at_s, 0.8, 1e-9, "which is the one sampled a dead time and a little before");
}

// One seed perceives a run the same every time; another seed differently; the fingerprint says which.
void a_seed_makes_perception_reproducible() {
    const auto track = fd::make_preset_track();
    const auto cones = fd::make_cone_layout(track);
    const auto frames_of = [&](std::uint64_t seed) {
        fd::Perception perception(cones, one(fd::PerceptionSensorSettings{}, seed));
        std::vector<double> values;
        for (int i = 0; i < 400; ++i) {
            const auto& p = track.points[static_cast<std::size_t>(i)%track.points.size()];
            perception.observe({p.x_m, p.y_m, 0.3, 10, 0, i*tick}, i*tick);
            for (const auto& f : perception.delivered())
                for (const auto& d : f.detections) { values.push_back(d.position.x); values.push_back(static_cast<double>(d.colour)); }
        }
        return values;
    };
    const auto first = frames_of(5);
    require(!first.empty(), "the preset's cones are seen");
    require(frames_of(5) == first, "one seed perceives the same run twice, value for value");
    require(frames_of(6) != first, "another seed perceives it differently");
    const fd::Perception a(cones, one(fd::PerceptionSensorSettings{}, 5));
    require(a.fingerprint().size() == 16, "the perception carries a fingerprint");
    require(fd::Perception(cones, one(fd::PerceptionSensorSettings{}, 6)).fingerprint() != a.fingerprint(), "which the seed changes");
    auto fewer = cones;
    fewer.pop_back();
    require(fd::Perception(fewer, one(fd::PerceptionSensorSettings{}, 5)).fingerprint() != a.fingerprint(), "and the cones");
    auto wider = fd::PerceptionSensorSettings{};
    wider.half_field_of_view_rad = 1.5;
    require(fd::Perception(cones, one(wider, 5)).fingerprint() != a.fingerprint(), "and the sensor");
}

// Perception on a running car watches the course without changing the run: the car drives exactly as it would without
// it, and every frame it delivers was sampled from where the car truly was, of the cones the car had ahead of it.
void perception_watches_a_run_without_changing_it() {
    const fd::Config config;
    fd::Simulation bare(fd::make_preset_track(), config, fd::DynamicSingleTrack{});
    fd::Simulation watching(fd::make_preset_track(), config, fd::DynamicSingleTrack{});
    fd::PerceptionSettings settings;
    settings.seed = 9;
    watching.set_perception(settings);
    require(watching.perception() != nullptr && bare.perception() == nullptr, "only the car given perception has it");
    require(watching.perception()->cones().size() == fd::make_cone_layout(watching.track()).size(), "of the track's own cones");
    std::vector<fd::State> poses{watching.state()};
    std::size_t frames = 0, detections = 0, missed = 0, wrong = 0, coloured = 0;
    double farthest = 0;
    for (int i = 0; i < 2000; ++i) {
        bare.step();
        watching.step();
        const auto& truth = watching.plant_state();
        require(truth.pose.x_m == bare.plant_state().pose.x_m && truth.pose.speed_mps == bare.plant_state().pose.speed_mps,
                "watching the cones does not move the car");
        poses.push_back(watching.state());
        const auto& p = *watching.perception();
        for (std::size_t k = 0; k < p.delivered().size(); ++k) {
            const auto& frame = p.delivered()[k];
            const auto& seen = p.delivered_truth()[k];
            ++frames;
            near(frame.delivered_at_s-frame.sampled_at_s, 0.2, 1e-9, "a frame arrives its dead time after it was sampled");
            // The pose it was sampled from is the car's own at that tick.
            const auto at = std::find_if(poses.begin(), poses.end(), [&](const fd::State& s) { return std::abs(s.time_s-frame.sampled_at_s) < 1e-9; });
            require(at != poses.end() && at->x_m == seen.pose.x_m && at->y_m == seen.pose.y_m, "sampled from where the car truly was");
            detections += frame.detections.size();
            missed += seen.missed.size();
            for (std::size_t d = 0; d < frame.detections.size(); ++d) {
                const auto& det = frame.detections[d];
                require(det.position.x > 0, "every cone it reports is ahead of the car");
                farthest = std::max(farthest, std::hypot(det.position.x, det.position.y));
                if (det.colour == fd::ObservedColour::unknown) continue;
                ++coloured;
                if (det.colour != fd::observed(p.cones()[seen.source_cone[d]].colour)) ++wrong;
            }
        }
    }
    std::cout << "  over ten seconds: " << frames << " frames, " << detections << " detections (" << static_cast<double>(detections)/frames
              << " a frame, the farthest " << farthest << " m), " << missed << " missed, " << wrong << " of " << coloured
              << " coloured detections the wrong colour\n";
    require(frames == 98, "ten frames a second, the first a period and a dead time in");
    require(detections > 10*frames, "the car sees many cones in every frame");
    require(missed > 0 && wrong > 0, "some are missed and some mistaken");
    // The sensor stands a metre ahead of the rear axle and sees 45 m, give or take its noise.
    require(farthest <= 45+1+0.1, "none beyond the sensor's range");
    watching.set_perception(std::nullopt);
    require(watching.perception() == nullptr, "perception can be taken off again");
}

// Settings outside their ranges are refused, as is a perception asked about the past.
void a_perception_that_cannot_be_is_refused() {
    const std::vector<fd::Cone> cones{{{10, 0}, fd::ConeColour::blue}};
    const auto refused = [&](const std::function<void(fd::PerceptionSettings&)>& change) {
        fd::PerceptionSettings bad;
        change(bad);
        try { fd::Perception p(cones, bad); } catch (const std::invalid_argument&) { return true; }
        return false;
    };
    require(refused([](fd::PerceptionSettings& s) { s.sensors.clear(); }), "no sensor is refused");
    require(refused([](fd::PerceptionSettings& s) { s.sensors[0].rate_hz = 0; }), "no rate is refused");
    require(refused([](fd::PerceptionSettings& s) { s.sensors[0].max_range_m = 0.5; }), "a range that ends before it begins is refused");
    require(refused([](fd::PerceptionSettings& s) { s.sensors[0].detection_floor = 1.2; }), "a probability above one is refused");
    require(refused([](fd::PerceptionSettings& s) { s.sensors[0].bearing_noise_rad = -1; }), "negative noise is refused");
    require(refused([](fd::PerceptionSettings& s) { s.sensors[0].name = "a,b"; }), "a name that would break a record is refused");
    fd::Perception perception(cones, fd::PerceptionSettings{});
    perception.observe({0, 0, 0, 0, 0, 0}, 1.0);
    bool backwards = false;
    try { perception.observe({0, 0, 0, 0, 0, 0}, 0.5); } catch (const std::invalid_argument&) { backwards = true; }
    require(backwards, "perception cannot be asked about the past");
}

}  // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"a_sensor_sees_only_what_is_in_range_and_view", a_sensor_sees_only_what_is_in_range_and_view},
        {"detections_are_in_the_car_s_frame", detections_are_in_the_car_s_frame},
        {"detection_falls_with_distance", detection_falls_with_distance},
        {"colour_is_right_less_often_far_away", colour_is_right_less_often_far_away},
        {"noise_matches_its_propagated_covariance", noise_matches_its_propagated_covariance},
        {"a_frame_arrives_a_dead_time_later", a_frame_arrives_a_dead_time_later},
        {"a_seed_makes_perception_reproducible", a_seed_makes_perception_reproducible},
        {"perception_watches_a_run_without_changing_it", perception_watches_a_run_without_changing_it},
        {"a_perception_that_cannot_be_is_refused", a_perception_that_cannot_be_is_refused}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " perception groups passed\n";
    return failures ? 1 : 0;
}

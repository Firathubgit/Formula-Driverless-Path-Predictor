#include "fd/recording.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tolerance)
        throw std::runtime_error(message+": actual="+std::to_string(actual)+", expected="+std::to_string(expected));
}
// Recordings store twelve significant digits.
void same_value(double actual, double expected, const std::string& message) {
    near(actual, expected, 1e-9*std::max(1.0, std::abs(expected)), message);
}
template<class Function> std::string rejects(Function action, const std::string& message) {
    try { action(); } catch (const std::exception& error) { return error.what(); }
    throw std::runtime_error(message);
}

struct TemporaryDirectory {
    fs::path path;
    TemporaryDirectory() {
        std::random_device device;
        path = fs::temp_directory_path()/("fd-recording-tests-"+std::to_string(device()));
        fs::create_directories(path);
    }
    ~TemporaryDirectory() { std::error_code error; fs::remove_all(path, error); }
};

struct LiveSample {
    double time_s, x_m, y_m, speed_mps; std::uint64_t revision; double lateral_velocity_mps, yaw_rate_radps; bool within_grip_envelope;
    double requested_steering_rad, geometric_steering_rad;
    double front_slip_angle_rad, rear_slip_angle_rad, understeer_angle_rad;
    fd::Balance balance;
    std::array<double, 4> wheel_speeds_radps, wheel_loads_n;
};

struct Fixture {
    fs::path directory;
    std::vector<LiveSample> live;
    // Every decision the live simulation made during a recorded step, in order, and who drove it.
    std::vector<fd::LocalDecision> decisions;
    std::vector<fd::ControllerOutcome> outcomes;
    std::vector<double> event_times;
    fd::RunSummary written;
    double dt{};
    int control_ticks{};
};

// Drives a simulation through the same writer the headless runner uses and captures what
// the live simulation actually did, as the oracle for what the files must contain.
Fixture record_run(const fs::path& directory, const fd::RunRequest& request,
                   const std::vector<fd::Obstruction>& obstructions, int stop_after_laps,
                   const fd::VehicleModel& model = fd::KinematicBicycle{},
                   fd::SteeringMode steering = fd::SteeringMode::pure_pursuit,
                   fd::SpeedPlanMode speed_plan = fd::SpeedPlanMode::grip_fractions,
                   const fd::Track& track = fd::make_preset_track(),
                   fd::LocalPlannerMode planner = fd::LocalPlannerMode::lattice,
                   std::shared_ptr<fd::PredictiveController> controller = nullptr,
                   std::optional<fd::SensorSettings> sensors = std::nullopt,
                   std::optional<fd::PerceptionSettings> perception = std::nullopt, bool on_cones = false,
                   const std::function<void(fd::Simulation&)>& setup = {}) {
    Fixture fixture;
    fixture.directory = directory;
    fd::Simulation simulation(track, fd::Config{}, model, steering, speed_plan, planner, std::move(controller));
    if (setup) setup(simulation);
    simulation.set_obstructions(obstructions);
    if (sensors) simulation.set_sensors(*sensors);
    if (perception) simulation.set_perception(*perception);
    if (on_cones) simulation.set_cone_driving(fd::ConePathSettings{});
    fixture.dt = simulation.config().fixed_dt_s;
    fixture.control_ticks = static_cast<int>(std::llround(simulation.config().control_dt_s/fixture.dt));
    fd::RecordingWriter writer(fixture.directory, simulation, request, {"test-fingerprint", "test-compiler", "0"});
    const auto capture = [&] {
        fixture.live.push_back({simulation.state().time_s, simulation.state().x_m, simulation.state().y_m,
                                simulation.state().speed_mps, simulation.diagnostics().revision,
                                simulation.plant_state().lateral_velocity_mps, simulation.plant_state().yaw_rate_radps,
                                simulation.diagnostics().within_grip_envelope, simulation.diagnostics().requested.steering_rad,
                                simulation.diagnostics().geometric_steering_rad, simulation.diagnostics().front_slip_angle_rad,
                                simulation.diagnostics().rear_slip_angle_rad, simulation.diagnostics().understeer_angle_rad,
                                simulation.diagnostics().balance, simulation.plant_state().wheel_speeds_radps,
                                simulation.plant_state().wheel_loads_n});
    };
    capture();
    auto seen = simulation.decision_count();
    std::size_t next_event = 0;
    while (simulation.state().time_s+fixture.dt <= request.duration_cap_s+1e-9 &&
           (stop_after_laps == 0 || simulation.diagnostics().laps < stop_after_laps)) {
        if (next_event < request.grip_events.size() && simulation.state().time_s+1e-10 >= request.grip_events[next_event].time_s) {
            simulation.set_grip(request.grip_events[next_event].grip_mu);
            ++next_event;
        }
        simulation.step();
        writer.record(simulation);
        capture();
        if (simulation.decision_count() != seen) {
            fixture.decisions.push_back(simulation.decision());
            fixture.outcomes.push_back(simulation.controller_outcome());
            seen = simulation.decision_count();
        }
    }
    fixture.written = writer.finish(simulation);
    for (const auto& event : simulation.events()) fixture.event_times.push_back(event.time_s);
    return fixture;
}

// A few seconds of the dynamic car reading itself through its own instruments, so a recording carries what was
// measured beside what was true (decision 0029). The seed is not the default, so a recording that ignored it would
// measure differently.
Fixture record_measured(const fs::path& root) {
    fd::RunRequest request;
    request.requested_laps = 0;
    request.duration_cap_s = 8;
    fd::SensorSettings sensors;
    sensors.seed = 2026;
    auto fixture = record_run(root/"measured", request, {}, 0, fd::DynamicSingleTrack{}, fd::SteeringMode::pure_pursuit,
                              fd::SpeedPlanMode::grip_fractions, fd::make_preset_track(), fd::LocalPlannerMode::lattice,
                              nullptr, sensors);
    require(fixture.written.samples > 1000, "the measured fixture drives eight seconds");
    return fixture;
}

// Eight seconds of the dynamic car with simulated cone perception, so a recording carries the cones, every frame and
// its truth (decision 0030). The seed is not the default, so a recording that ignored it would perceive differently.
// Ten seconds on the preset laid to 5 m, driven on the cones the car perceives, from its own instruments or, with
// measured false, from its true state read the instant it is.
Fixture record_on_cones(const fs::path& root, bool measured) {
    fd::RunRequest request;
    request.requested_laps = 0;
    request.duration_cap_s = measured ? 10 : 6;
    auto track = fd::make_preset_track();
    track.width_m = 5;
    fd::PerceptionSettings perception;
    perception.seed = 41;
    std::optional<fd::SensorSettings> sensors;
    if (measured) sensors = fd::SensorSettings{.seed = 41};
    auto fixture = record_run(root/(measured ? "on-cones" : "on-cones-ideal"), request, {}, 0, fd::KinematicBicycle{},
                              fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions, track, fd::LocalPlannerMode::lattice,
                              nullptr, sensors, perception, true);
    return fixture;
}

Fixture record_perceived(const fs::path& root) {
    fd::RunRequest request;
    request.requested_laps = 0;
    request.duration_cap_s = 8;
    fd::PerceptionSettings perception;
    perception.seed = 31;
    auto fixture = record_run(root/"perceived", request, {}, 0, fd::DynamicSingleTrack{}, fd::SteeringMode::pure_pursuit,
                              fd::SpeedPlanMode::grip_fractions, fd::make_preset_track(), fd::LocalPlannerMode::lattice,
                              nullptr, std::nullopt, perception);
    require(fixture.written.samples > 1000, "the perceived fixture drives eight seconds");
    return fixture;
}

// One lap with two live grip changes, the first between control ticks.
// Twelve seconds of an autocross on the preset with three cones left across the lane 60 m on, which the car runs into
// (decision 0032).
Fixture record_knocked(const fs::path& root) {
    fd::RunRequest request;
    request.requested_laps = 0;
    request.duration_cap_s = 12;
    auto fixture = record_run(root/"knocked", request, {}, 0, fd::KinematicBicycle{}, fd::SteeringMode::pure_pursuit,
                              fd::SpeedPlanMode::grip_fractions, fd::make_preset_track(), fd::LocalPlannerMode::lattice, nullptr,
                              std::nullopt, std::nullopt, false, [&](fd::Simulation& simulation) {
        const auto& track = simulation.track();
        auto cones = fd::make_cone_layout(track);
        const auto at = fd::sample(track, 60), ahead = fd::sample(track, 60.05);
        const double tx = ahead.x_m-at.x_m, ty = ahead.y_m-at.y_m, n = std::hypot(tx, ty);
        for (const double offset : {-0.4, 0.0, 0.4}) cones.push_back({{at.x_m-ty/n*offset, at.y_m+tx/n*offset}, fd::ConeColour::orange});
        simulation.set_course(cones, fd::make_gates(track), "cones left in the lane");
        fd::CompetitionRules rules;
        rules.discipline = fd::Discipline::autocross;
        simulation.set_competition(rules);
    });
    return fixture;
}

Fixture record_fixture(const fs::path& root) {
    fd::RunRequest request;
    request.requested_laps = 1;
    request.duration_cap_s = 120;
    request.grip_events = {{4.01, 0.8}, {20.0, 0.6}};
    auto fixture = record_run(root/"run", request, {}, 1);
    require(fixture.written.completed && fixture.event_times.size() == 2, "fixture completes a lap with two accepted changes");
    return fixture;
}

// The dynamic plant with a non-default drive split, a grip change and a lane blockage, steered by
// MAP, so its parameters, plant state, steering table and decisions are all exercised.
fd::DynamicSingleTrack dynamic_car() {
    fd::DynamicSingleTrack car;
    car.drive_front_fraction = 0.4;
    car.rear_tire.peak_slip_angle_high_rad = 0.11;
    return car;
}
Fixture record_dynamic(const fs::path& root) {
    fd::RunRequest request;
    request.requested_laps = 0;
    request.duration_cap_s = 12;
    request.grip_events = {{7.0, 0.7}};
    return record_run(root/"dynamic", request, {{62, 68, -4.5, 1.0, "cone-cluster"}}, 0, dynamic_car(),
                      fd::SteeringMode::model_acceleration_pursuit);
}

// The four-wheel car with all braking on the rear axle, a non-default drive split, centre of gravity height,
// roll balance, aerodynamics and viscous coupling, braking into the first corner, so wheel speeds, wheel loads,
// a locked rear axle and every parameter are exercised.
fd::FourWheelCar wheeled_car() {
    fd::FourWheelCar car;
    car.brake_bias_front = 0;
    car.drive_front_fraction = 0.3;
    car.max_drive_power_w = 90000;
    car.cg_height_m = 0.4;
    car.roll_balance_front = 0.6;
    car.drag_area_m2 = 0.7;
    car.downforce_area_m2 = 1.5;
    car.aero_balance_front = 0.45;
    car.viscous_coupling_nms = 25;
    return car;
}
Fixture record_wheels(const fs::path& root) {
    fd::RunRequest request;
    request.requested_laps = 0;
    request.duration_cap_s = 11;
    return record_run(root/"wheels", request, {}, 0, wheeled_car());
}

// The four-wheel car planning with its performance envelope through a grip change, so the envelope is derived again
// for the second plan revision.
Fixture record_envelope(const fs::path& root) {
    fd::RunRequest request;
    request.requested_laps = 0;
    request.duration_cap_s = 9;
    request.grip_events = {{5.0, 0.8}};
    return record_run(root/"envelope", request, {}, 0, fd::FourWheelCar{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::performance_envelope);
}

// A reference line that is not the corridor's centre, as a racing line is: 3 m from the left edge and 7 m from the right.
fd::Track off_centre_track() {
    auto track = fd::make_preset_track();
    track.left_edge_m.assign(track.points.size(), 3.0);
    track.right_edge_m.assign(track.points.size(), 7.0);
    return track;
}
Fixture record_edged(const fs::path& root) {
    fd::RunRequest request;
    request.requested_laps = 0;
    request.duration_cap_s = 3;
    return record_run(root/"edged", request, {}, 0, fd::KinematicBicycle{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions,
                      off_centre_track());
}

// A bollard just before the first corner that the car can pass on either side, so decisions compare clear actions by
// their estimated times (decision 0023).
Fixture record_corner(const fs::path& root) {
    fd::RunRequest request;
    request.requested_laps = 0;
    request.duration_cap_s = 10;
    return record_run(root/"corner", request, {{130, 136, -0.5, 1.5, "bollard"}}, 0);
}

// Plays a predictive controller's part in the recording contract without an optimiser: its plan is the policy's
// prediction read every 0.05 s for 60 stages, its commands the policy's first command held throughout, and every seventh
// plan is not solved. The car moves exactly as under the policy; only who drove is at stake.
class Resampling final : public fd::PredictiveController {
public:
    fd::ControllerPlan plan(const fd::ControlRequest& request) override {
        const auto& policy = request.decision.trajectory();
        fd::ControllerPlan plan;
        ++answered_;
        plan.status = answered_%7 == 0 ? fd::ControllerStatus::not_solved : fd::ControllerStatus::solved;
        plan.iterations = plan.status == fd::ControllerStatus::solved ? 2 : 0;
        plan.restarted = answered_ == 1;
        plan.trajectory.first_control = policy.first_control;
        plan.trajectory.commands.assign(61, policy.first_control.applied);
        plan.trajectory.within_model_envelope = policy.within_model_envelope;
        plan.trajectory.validity_reason = policy.validity_reason;
        const auto& points = policy.points;
        for (int k = 0; k <= 60; ++k) {
            const double time = points.front().state.time_s+0.05*k;
            const auto later = std::find_if(points.begin(), points.end(), [&](const fd::TrajectorySample& p) { return p.state.time_s > time-1e-12; });
            fd::TrajectorySample sample = later == points.end() ? points.back() : *later;
            if (later != points.begin() && later != points.end() && later->state.time_s > time+1e-12) {
                const auto& a = *(later-1);
                const double f = (time-a.state.time_s)/(later->state.time_s-a.state.time_s);
                sample.state.x_m = a.state.x_m+f*(later->state.x_m-a.state.x_m);
                sample.state.y_m = a.state.y_m+f*(later->state.y_m-a.state.y_m);
                sample.state.speed_mps = a.state.speed_mps+f*(later->state.speed_mps-a.state.speed_mps);
            }
            sample.state.time_s = time;
            if (!plan.trajectory.points.empty())
                plan.trajectory.points.back().acceleration_mps2 = (sample.state.speed_mps-plan.trajectory.points.back().state.speed_mps)/0.05;
            sample.acceleration_mps2 = 0;
            plan.trajectory.points.push_back(sample);
        }
        return plan;
    }
    void reset() override { answered_ = 0; }
    fd::ControllerSettings settings() const override { return {fd::ControllerMode::mpcc, {{"stages", 60}, {"stage_s", 0.05}}}; }
    void check(const fd::VehicleModel&, const fd::Config&) const override {}
private:
    int answered_{};
};

// Keeps to one plan on the four-wheel car: at every decision it commands c(t), acceleration and steering growing linearly
// in time, and plans the plant's response to those commands from the actual state, so the car goes exactly where each
// plan said, except where the grip changed under a plan made before (decision 0025).
class Keeping final : public fd::PredictiveController {
public:
    fd::ControllerPlan plan(const fd::ControlRequest& request) override {
        fd::ControllerPlan plan;
        plan.status = fd::ControllerStatus::solved;
        plan.iterations = 1;
        auto& trajectory = plan.trajectory;
        std::vector<double> times;
        for (int k = 0; k <= 60; ++k) {
            times.push_back(request.state.pose.time_s+0.05*k);
            trajectory.commands.push_back({2.0+0.5*times.back(), 0.02+0.01*times.back()});
        }
        const auto response = fd::plant_response(request.model, request.state, request.config, times, trajectory.commands,
                                                 request.ticks_to_next_control);
        for (std::size_t k = 0; k < response.size(); ++k) {
            fd::TrajectorySample point;
            point.state = response[k].pose;
            point.state.time_s = times[k];
            if (k) trajectory.points.back().acceleration_mps2 = (point.state.speed_mps-trajectory.points.back().state.speed_mps)/0.05;
            trajectory.points.push_back(point);
        }
        const double end = request.state.pose.time_s+static_cast<double>(request.ticks_to_next_control)*request.config.fixed_dt_s;
        trajectory.first_control.requested = trajectory.first_control.applied = fd::planned_command(times, trajectory.commands, end);
        trajectory.first_control.geometric_steering_rad = trajectory.first_control.requested.steering_rad;  // as MPCC's: no pursuit
        return plan;
    }
    void reset() override {}
    fd::ControllerSettings settings() const override { return {fd::ControllerMode::mpcc, {{"stages", 60}, {"stage_s", 0.05}}}; }
    void check(const fd::VehicleModel&, const fd::Config&) const override {}
};

// Four seconds of the four-wheel car kept to its plans, with the grip changing between control ticks at 2.01 s.
Fixture record_kept(const fs::path& root) {
    fd::RunRequest request;
    request.requested_laps = 0;
    request.duration_cap_s = 4;
    request.grip_events = {{2.01, 0.8}};
    return record_run(root/"kept", request, {}, 0, fd::FourWheelCar{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions,
                      fd::make_preset_track(), fd::LocalPlannerMode::lattice, std::make_shared<Keeping>());
}

// The lane blockage and the full-width one of record_scenario, driven through the controller seam, so decisions are
// driven by the controller, by the policy while holding, and by the policy when a plan is not solved.
Fixture record_controlled(const fs::path& root) {
    fd::RunRequest request;
    request.requested_laps = 0;
    request.duration_cap_s = 14;
    return record_run(root/"controlled", request, {{62, 68, -4.5, 1.0, "cone-cluster"}, {140, 146, -5, 5, "stalled-car"}}, 0,
                      fd::KinematicBicycle{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions, fd::make_preset_track(),
                      fd::LocalPlannerMode::lattice, std::make_shared<Resampling>());
}

// A lane blockage the car steers around, with grip falling to 0.45 while it is out beside it, so that on the way home
// neither the path the car is on nor the reference is clear and the car drives on without holding for anything.
Fixture record_lost_grip(const fs::path& root) {
    fd::RunRequest request;
    request.requested_laps = 0;
    request.duration_cap_s = 12;
    request.grip_events = {{6.0, 0.45}};
    return record_run(root/"lost-grip", request, {{62, 68, -4.5, 1.0, "cone-cluster"}}, 0);
}

// A lane blockage the car can steer around, then a full-width one it must stop for, under either local planner.
Fixture record_scenario(const fs::path& root, fd::LocalPlannerMode planner = fd::LocalPlannerMode::lattice) {
    fd::RunRequest request;
    request.requested_laps = 0;
    request.duration_cap_s = 14;
    return record_run(root/(planner == fd::LocalPlannerMode::lattice ? "scenario" : "scenario-five-offsets"), request,
                      {{62, 68, -4.5, 1.0, "cone-cluster"}, {140, 146, -5, 5, "stalled-car"}}, 0, fd::KinematicBicycle{},
                      fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions, fd::make_preset_track(), planner);
}

std::string read_text(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}
void write_text(const fs::path& path, const std::string& text) {
    std::ofstream file(path, std::ios::binary);
    file << text;
}
// Copy the fixture and apply one edit so each rule is exercised in isolation. Unedited
// files are hard links, because decision trajectories make a copy of every file slow.
fs::path variant(const Fixture& fixture, const std::string& name, const std::string& file,
                 const std::function<void(std::string&)>& edit) {
    const fs::path copy = fixture.directory.parent_path()/("variant-"+name);
    fs::create_directories(copy);
    for (const auto& entry : fs::directory_iterator(fixture.directory)) {
        const fs::path target = copy/entry.path().filename();
        std::error_code error;
        if (entry.path().filename() != file) fs::create_hard_link(entry.path(), target, error);
        if (entry.path().filename() == file || error) fs::copy_file(entry.path(), target);
    }
    std::string text = read_text(copy/file);
    edit(text);
    fs::remove(copy/file);
    write_text(copy/file, text);
    return copy;
}
// Replaces one file of a variant in place, never through a hard link to the fixture.
void rewrite_file(const fs::path& directory, const std::string& file, const std::function<void(std::string&)>& change) {
    std::string text = read_text(directory/file);
    change(text);
    fs::remove(directory/file);
    write_text(directory/file, text);
}
// Metadata as an older schema would have written it, apart from what the test changes: the schema version, the local
// planner that schema 10 added, and the speed plan and envelope share that schema 8 added.
void as_schema(std::string& text, int schema) {
    const auto erase_line = [&](const std::string& key) {
        const auto at = text.find("  \""+key+"\": ");
        if (at != std::string::npos) text.erase(at, text.find('\n', at)-at+1);
    };
    const auto version = text.find("\"schema_version\": 18");
    if (version != std::string::npos) text.replace(version, 20, "\"schema_version\": "+std::to_string(schema));
    erase_line("competition");
    erase_line("predictive_controller");
    erase_line("local_planner");
    erase_line("speed_plan");
    erase_line("envelope_fraction");
    const auto last = text.find("\"lookahead_time_s\": ");
    const auto comma = text.find(',', last);
    if (last != std::string::npos && comma < text.find('\n', last)) text.erase(comma, 1);
}

void replace_first(std::string& text, const std::string& from, const std::string& to) {
    const auto at = text.find(from);
    require(at != std::string::npos, "fixture text contains '"+from+"'");
    text.replace(at, from.size(), to);
}
// Metadata declaring an older schema, without the competition member schema 17 added, so that whatever else the variant
// leaves in is what the loader objects to.
void as_older(std::string& text, int schema) {
    replace_first(text, "\"schema_version\": 18", "\"schema_version\": "+std::to_string(schema));
    const auto at = text.find("  \"competition\": ");
    require(at != std::string::npos, "fixture metadata has a competition line");
    text.erase(at, text.find('\n', at)-at+1);
}
std::vector<std::string> lines(const std::string& text) {
    std::vector<std::string> result;
    std::stringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();  // files are written in text mode
        result.push_back(line);
    }
    return result;
}
std::string join(const std::vector<std::string>& parts) {
    std::string out;
    for (const auto& part : parts) out += part+'\n';
    return out;
}
std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> fields;
    std::string current;
    for (const char c : line) { if (c == ',') { fields.push_back(current); current.clear(); } else current += c; }
    fields.push_back(current);
    return fields;
}
std::string join_fields(const std::vector<std::string>& fields) {
    std::string out;
    for (std::size_t i = 0; i < fields.size(); ++i) out += (i ? "," : "")+fields[i];
    return out;
}
// Edits the first data row matching a predicate, by column name.
void edit_row(std::string& text, const std::function<bool(const std::vector<std::string>&, const std::vector<std::string>&)>& matches,
              const std::string& column, const std::string& value) {
    auto rows = lines(text);
    const auto header = split(rows[0]);
    const auto at = static_cast<std::size_t>(std::find(header.begin(), header.end(), column)-header.begin());
    require(at < header.size(), "fixture header has column "+column);
    for (std::size_t i = 1; i < rows.size(); ++i) {
        auto fields = split(rows[i]);
        if (!matches(header, fields)) continue;
        fields[at] = value;
        rows[i] = join_fields(fields);
        text = join(rows);
        return;
    }
    throw std::runtime_error("fixture has no row to edit in column "+column);
}
std::function<bool(const std::vector<std::string>&, const std::vector<std::string>&)> where(
    std::vector<std::pair<std::string, std::string>> conditions) {
    return [conditions](const std::vector<std::string>& header, const std::vector<std::string>& fields) {
        for (const auto& [column, value] : conditions) {
            const auto at = static_cast<std::size_t>(std::find(header.begin(), header.end(), column)-header.begin());
            if (at >= header.size() || fields[at] != value) return false;
        }
        return true;
    };
}
// Drops every data row matching a predicate.
void drop_rows(std::string& text, const std::function<bool(const std::vector<std::string>&, const std::vector<std::string>&)>& matches) {
    auto rows = lines(text);
    const auto header = split(rows[0]);
    std::vector<std::string> kept{rows[0]};
    for (std::size_t i = 1; i < rows.size(); ++i) if (!matches(header, split(rows[i]))) kept.push_back(rows[i]);
    require(kept.size() < rows.size(), "fixture has rows to drop");
    text = join(kept);
}

void same_decision(const fd::RecordedDecision& recorded, const fd::LocalDecision& live, const std::string& context) {
    const auto& first = live.trajectory().points.front().state;
    same_value(recorded.time_s, first.time_s, context+" time");
    require(recorded.selected == live.selected && recorded.holding == live.holding, context+" selection and holding");
    require(recorded.reason == live.reason && recorded.blocking_identifier == live.blocking_identifier, context+" reasons");
    const auto& control = live.trajectory().first_control;
    same_value(recorded.requested.acceleration_mps2, control.requested.acceleration_mps2, context+" requested acceleration");
    same_value(recorded.requested.steering_rad, control.requested.steering_rad, context+" requested steering");
    same_value(recorded.applied.acceleration_mps2, control.applied.acceleration_mps2, context+" applied acceleration");
    same_value(recorded.applied.steering_rad, control.applied.steering_rad, context+" applied steering");
    require(recorded.options.size() == live.options.size(), context+" option count");
    for (std::size_t o = 0; o < live.options.size(); ++o) {
        const auto& r = recorded.options[o];
        const auto& l = live.options[o];
        const std::string where_option = context+" option "+std::to_string(o);
        same_value(r.lateral_offset_m, l.lateral_offset_m, where_option+" offset");
        require(r.speed_limit_mps.has_value() == std::isfinite(l.speed_limit_mps), where_option+" speed limit presence");
        if (r.speed_limit_mps) same_value(*r.speed_limit_mps, l.speed_limit_mps, where_option+" speed limit");
        require(r.clear == l.clear && r.within_envelope == l.trajectory.within_model_envelope, where_option+" flags");
        same_value(r.station_gain_m, l.station_gain_m, where_option+" station gain");
        require(r.reason == l.reason && r.validity_reason == l.trajectory.validity_reason, where_option+" reasons");
        require(r.action == l.action && r.path.size() == l.path.size(), where_option+" action and path length");
        for (std::size_t p = 0; p < r.path.size(); ++p) {
            same_value(r.path[p].s_m, l.path[p].s_m, where_option+" path station");
            same_value(r.path[p].offset_m, l.path[p].offset_m, where_option+" path offset");
        }
        same_value(r.path_cost.edges.average_curvature, l.path_cost.edges.average_curvature, where_option+" curvature cost");
        same_value(r.path_cost.edges.curvature_range, l.path_cost.edges.curvature_range, where_option+" curvature range cost");
        same_value(r.path_cost.edges.length, l.path_cost.edges.length, where_option+" length cost");
        same_value(r.path_cost.edges.reference_deviation, l.path_cost.edges.reference_deviation, where_option+" deviation cost");
        same_value(r.path_cost.turns, l.path_cost.turns, where_option+" turn cost");
        same_value(r.path_cost.goal, l.path_cost.goal, where_option+" goal cost");
        require(r.speed_profile.size() == l.speed_profile.size() && r.estimated_time_s.has_value() == (!l.speed_profile.empty() && std::isfinite(l.estimated_time_s)),
                where_option+" speed profile length and estimate");
        for (std::size_t p = 0; p < r.speed_profile.size(); ++p) {
            same_value(r.speed_profile[p].s_m, l.speed_profile[p].s_m, where_option+" profile station");
            same_value(r.speed_profile[p].distance_m, l.speed_profile[p].distance_m, where_option+" profile distance");
            same_value(r.speed_profile[p].curvature_1pm, l.speed_profile[p].curvature_1pm, where_option+" profile curvature");
            same_value(r.speed_profile[p].speed_mps, l.speed_profile[p].speed_mps, where_option+" profile speed");
        }
        if (r.estimated_time_s) same_value(*r.estimated_time_s, l.estimated_time_s, where_option+" estimated time");
        require(r.trajectory.size() == l.trajectory.points.size(), where_option+" trajectory length");
        for (std::size_t p = 0; p < r.trajectory.size(); ++p) {
            const auto& rp = r.trajectory[p];
            const auto& lp = l.trajectory.points[p];
            same_value(rp.time_s, lp.state.time_s, where_option+" point time");
            same_value(rp.x_m, lp.state.x_m, where_option+" point x");
            same_value(rp.y_m, lp.state.y_m, where_option+" point y");
            same_value(rp.speed_mps, lp.state.speed_mps, where_option+" point speed");
            same_value(rp.acceleration_mps2, lp.acceleration_mps2, where_option+" point acceleration");
        }
    }
}

// ---------------------------------------------------------------- tests

fs::path downgrade(const Fixture& fixture, const std::string& name, int schema, bool keep_decision_files = false);

void round_trip(const Fixture& fixture) {
    const auto recording = fd::load_recording(fixture.directory);
    require(recording.metadata.schema_version == 18 && recording.competition_recorded() && recording.metadata.competition &&
            recording.metadata.competition->course == fd::Simulation::laid_along_the_track && !recording.cones.empty() &&
            recording.cone_driving_files_recorded() && !recording.cone_driving_recorded() &&
            recording.beliefs.empty() && recording.believed_paths.empty() &&
            recording.perception_files_recorded() && recording.measurements_recorded() &&
            recording.commands_recorded() &&
            recording.controller_recorded() && recording.speed_profiles_recorded() &&
            recording.local_planner_recorded() && recording.edges_recorded() &&
            recording.speed_plan_recorded() && recording.decisions_recorded() && recording.plant_recorded() && recording.steering_recorded() &&
            recording.wheels_recorded() && recording.loads_recorded(), "new recordings are schema 18");
    require(recording.metadata.predictive_controller.mode == fd::ControllerMode::policy &&
            std::all_of(recording.decisions.begin(), recording.decisions.end(), [](const fd::RecordedDecision& d) {
                return d.driven_by == fd::ControllerMode::policy && !d.controller_status && d.controller_note.empty(); }),
            "a run without a predictive controller records every decision driven by the policy, unasked");
    require(recording.metadata.local_planner_mode == fd::LocalPlannerMode::lattice, "the default run records the lattice planner");
    require(std::all_of(recording.decisions.begin(), recording.decisions.end(), [](const fd::RecordedDecision& d) {
        return d.options.size() == 1 && d.choice().action == fd::LocalAction::straight && d.choice().path.empty() && d.choice().speed_profile.empty(); }),
            "without a blockage every decision follows the reference line with one option, no path and the plan's speed");
    require(recording.track.left_edge_m.empty() && recording.track.right_edge_m.empty(), "a centreline's recorded edges read back as a centreline");
    require(std::all_of(recording.samples.begin(), recording.samples.end(), [](const fd::RecordedSample& s) {
        return s.wheel_speeds_radps == std::array<double, 4>{} && s.wheel_loads_n == std::array<double, 4>{}; }),
            "the kinematic bicycle records no wheel speeds or loads");
    require(std::holds_alternative<fd::KinematicBicycle>(recording.metadata.vehicle_model), "the default run records the kinematic bicycle");
    require(recording.metadata.steering_mode == fd::SteeringMode::pure_pursuit && recording.metadata.steering_table_fingerprint.empty() &&
            !fd::recorded_steering_table_matches(recording), "the default run records Pure Pursuit, which has no table");
    const auto reproduced = fd::plant_reproduction_error_m(recording);
    require(reproduced && *reproduced < 1e-6, "re-integrating every recorded step reproduces the recorded motion");
    require(recording.metadata.source_fingerprint == "test-fingerprint", "metadata identity round-trips");
    require(recording.plans.size() == 3 && recording.events.size() == 2, "two accepted changes yield three plan revisions");
    require(recording.samples.size() == fixture.written.samples, "sample count matches the written summary");
    require(recording.samples.size() == fixture.live.size(), "one sample per recorded step plus the initial state");
    require(recording.summary.laps == 1 && recording.summary.lap_crossing_times_s.size() == 1, "lap crossing recorded");
    near(recording.plans[1].generated_time_s, fixture.event_times[0], 1e-9, "revision 2 generated at the first event");
    near(recording.plans[2].generated_time_s, fixture.event_times[1], 1e-9, "revision 3 generated at the second event");
    near(recording.config_for_revision(1).grip_mu, 1.0, 0, "initial configuration");
    near(recording.config_for_revision(2).grip_mu, 0.8, 1e-12, "first change applied at revision 2");
    near(recording.config_for_revision(3).grip_mu, 0.6, 1e-12, "second change applied at revision 3");
    require(recording.track.points.size() == recording.plans[0].points.size(), "plan rows equal track samples");
    require(recording.track.name == "Foundry Circuit", "track name preserved");
    for (std::size_t i = 0; i < recording.samples.size(); ++i) {
        const auto& s = recording.samples[i];
        const auto& l = fixture.live[i];
        near(s.time_s, l.time_s, 1e-9, "recorded time equals the live simulation");
        near(s.x_m, l.x_m, 1e-8, "recorded x equals the live simulation");
        near(s.y_m, l.y_m, 1e-8, "recorded y equals the live simulation");
        near(s.speed_mps, l.speed_mps, 1e-8, "recorded speed equals the live simulation");
        require(s.revision == l.revision, "recorded revision equals the live simulation");
        require(s.lateral_velocity_mps == 0 && s.within_grip_envelope == l.within_grip_envelope, "kinematic plant columns equal the live simulation");
        same_value(s.yaw_rate_radps, l.yaw_rate_radps, "recorded yaw rate equals the live simulation");
        require(s.geometric_steering_rad == s.requested_steering_rad, "under Pure Pursuit the geometric steering is the requested steering");
        same_value(s.geometric_steering_rad, l.geometric_steering_rad, "recorded geometric steering equals the live simulation");
    }
    near(fd::plan_reproduction_error_mps(recording), 0, 1e-6, "this build's planner reproduces every recorded plan");

    // One decision per control tick plus one where the off-grid grip change replanned.
    require(recording.decisions.size() == fixture.decisions.size(), "every live decision that governed motion is recorded");
    std::size_t control_ticks = 0;
    for (std::size_t i = 0; i+1 < recording.samples.size(); ++i)
        if (std::llround(recording.samples[i].time_s/fixture.dt)%fixture.control_ticks == 0) ++control_ticks;
    require(recording.decisions.size() == control_ticks+1, "control-tick decisions plus exactly one off-grid replan");
    for (std::size_t d = 0; d < recording.decisions.size(); ++d)
        same_decision(recording.decisions[d], fixture.decisions[d], "decision "+std::to_string(d));
}

void decision_in_force_follows_the_tick_rule(const Fixture& fixture) {
    const auto recording = fd::load_recording(fixture.directory);
    const double dt = fixture.dt;
    const auto* at_start = recording.decision_at(0);
    require(at_start && at_start->time_s == 0, "the first sample shows the decision made at time zero");
    // The sample at a control tick still shows the previous decision: the new one governs the motion after it.
    const auto* at_tick = recording.decision_at(4*dt);
    require(at_tick == &recording.decisions[0], "the sample at a control tick shows the decision that produced it");
    const auto* after_tick = recording.decision_at(5*dt);
    require(after_tick == &recording.decisions[1], "one tick later the new decision is in force");
    const double replan = fixture.event_times[0];
    const auto replanned = std::find_if(recording.decisions.begin(), recording.decisions.end(),
                                        [&](const fd::RecordedDecision& d) { return std::abs(d.time_s-replan) < 1e-9; });
    require(replanned != recording.decisions.end(), "the off-grid grip change produced its own decision");
    require(recording.decision_at(replan) == &*(replanned-1), "the sample at the replan still shows the earlier decision");
    require(recording.decision_at(replan+dt) == &*replanned, "the replanned decision governs the next sample");

    fd::Playback playback(fd::load_recording(fixture.directory));
    for (const double time : {0.0, 3*dt, 4*dt, 5*dt, replan, replan+dt, 30.0}) {
        playback.seek_time(time);
        const auto* expected = playback.recording().decision_at(playback.sample().time_s);
        require(playback.decision() && playback.decision() == expected, "playback exposes the decision in force at its sample");
        same_value(playback.decision()->choice().trajectory.front().time_s, playback.decision()->time_s,
                   "the chosen line starts at its decision time");
    }
    playback.seek_time(1e9);
    require(playback.decision() == &playback.recording().decisions.back(), "the final sample shows the last decision");
}

void scenario_decisions_round_trip(const Fixture& scenario) {
    const auto recording = fd::load_recording(scenario.directory);
    require(recording.metadata.scenario_obstructions.size() == 2, "the scenario is recorded");
    require(recording.decisions.size() == scenario.decisions.size(), "every scenario decision is recorded");
    require(recording.metadata.local_planner_mode == fd::LocalPlannerMode::lattice, "the scenario records the lattice planner");
    std::size_t avoiding = 0, holding = 0, returning = 0, path_points = 0, compared = 0;
    for (std::size_t d = 0; d < recording.decisions.size(); ++d) {
        const auto& decision = recording.decisions[d];
        same_decision(decision, scenario.decisions[d], "scenario decision "+std::to_string(d));
        const auto action = decision.choice().action;
        if (!decision.holding && decision.blocking_identifier == "cone-cluster" &&
            (action == fd::LocalAction::pass_left || action == fd::LocalAction::pass_right)) ++avoiding;
        if (decision.holding && action == fd::LocalAction::brake && decision.blocking_identifier == "stalled-car") ++holding;
        if (action == fd::LocalAction::straight && !decision.choice().path.empty()) ++returning;
        for (const auto& option : decision.options) {
            path_points += option.path.size();
            require(option.path.empty() || !option.speed_profile.empty(), "every lattice path is recorded with its speed profile");
        }
        const auto clear_timed = std::count_if(decision.options.begin(), decision.options.end(), [](const fd::RecordedOption& o) {
            return o.clear && o.estimated_time_s; });
        if (clear_timed >= 2) ++compared;
    }
    require(avoiding > 0, "decisions passing the lane blockage along the lattice are recorded with their alternatives");
    require(holding > 0, "decisions braking for the full-width blockage are recorded");
    require(returning > 0, "decisions returning to the reference along the lattice are recorded");
    const auto blocked = std::find_if(recording.decisions.begin(), recording.decisions.end(), [](const fd::RecordedDecision& d) {
        return std::any_of(d.options.begin(), d.options.end(), [](const fd::RecordedOption& o) { return o.reason == "Blocked by cone-cluster"; });
    });
    require(blocked != recording.decisions.end(), "a rejected line keeps the reason it was rejected");
    std::cout << "  scenario: " << recording.decisions.size() << " decisions, " << avoiding << " passing, "
              << returning << " returning, " << holding << " braking, " << path_points << " path points, " << compared
              << " decisions comparing clear actions by time\n";
}

// Where both sides of the bollard are clear, each pass is recorded with its speed profile and estimated time, and the
// car took the faster.
void corner_decisions_round_trip(const Fixture& corner) {
    const auto recording = fd::load_recording(corner.directory);
    require(recording.decisions.size() == corner.decisions.size(), "every corner decision is recorded");
    std::size_t compared = 0, left = 0, right = 0;
    for (std::size_t d = 0; d < recording.decisions.size(); ++d) {
        const auto& decision = recording.decisions[d];
        same_decision(decision, corner.decisions[d], "corner decision "+std::to_string(d));
        std::vector<const fd::RecordedOption*> timed;
        for (const auto& option : decision.options) if (option.clear && option.estimated_time_s) timed.push_back(&option);
        if (timed.size() < 2) continue;
        ++compared;
        double fastest = 1e9;
        for (const auto* option : timed) fastest = std::min(fastest, *option->estimated_time_s);
        const bool kept = d > 0 && recording.decisions[d-1].choice().action == decision.choice().action &&
                          recording.decisions[d-1].choice().path.empty() == decision.choice().path.empty();
        require(*decision.choice().estimated_time_s <= fastest+(kept ? fd::lattice_switch_margin_s : 0.0)+1e-9,
                "the fastest clear action was taken, or the previous choice kept within the margin");
        if (decision.choice().action == fd::LocalAction::pass_left) ++left;
        if (decision.choice().action == fd::LocalAction::pass_right) ++right;
    }
    require(compared > 0 && left > 0, "decisions comparing both passes by time are recorded, passing left on the inside");
    std::cout << "  corner: " << compared << " decisions compared clear actions by time; " << left << " passed left and " << right << " right\n";
}

// A decision may take a line that is not clear only when no option is clear and it names no blockage in its way: the
// car is past the blockage and grip has fallen, so there is nothing to hold for and it keeps the path it is on. Such a
// run round-trips, and the loaded decisions say the same.
void a_run_with_no_clear_way_home_round_trips(const Fixture& lost) {
    const auto recording = fd::load_recording(lost.directory);
    require(recording.decisions.size() == lost.decisions.size(), "every decision of the run is recorded");
    std::size_t nothing_clear = 0, kept_a_path = 0;
    for (std::size_t d = 0; d < recording.decisions.size(); ++d) {
        const auto& decision = recording.decisions[d];
        same_decision(decision, lost.decisions[d], "lost-grip decision "+std::to_string(d));
        if (decision.options.size() < 2 || decision.holding) continue;
        if (std::any_of(decision.options.begin(), decision.options.end(), [](const fd::RecordedOption& o) { return o.clear; })) continue;
        ++nothing_clear;
        require(decision.blocking_identifier.empty(), "a decision that drives on with nothing clear names no blockage in its way");
        if (!decision.choice().path.empty()) ++kept_a_path;
    }
    require(nothing_clear > 20 && kept_a_path == nothing_clear, "the car kept its lattice path through every such decision");
    std::cout << "  lost grip: " << nothing_clear << " recorded decisions had no clear option and kept the path the car was on\n";
}

// Schema 12: the controller and who drove each decision round-trip, and a recording that says otherwise is refused:
// a policy run claiming a controller, a controller decision without a solved plan or with a plan of another shape, a
// policy decision under a controller without its reason, and a hold the controller was asked about.
void controller_decisions_round_trip(const Fixture& controlled, const Fixture& plain) {
    const auto recording = fd::load_recording(controlled.directory);
    require(recording.metadata.predictive_controller.mode == fd::ControllerMode::mpcc &&
            recording.metadata.predictive_controller.values.size() == 2, "the run records its controller and settings");
    require(recording.decisions.size() == controlled.outcomes.size(), "every decision is recorded");
    std::size_t by_controller = 0, holding = 0, unsolved = 0;
    for (std::size_t d = 0; d < recording.decisions.size(); ++d) {
        const auto& decision = recording.decisions[d];
        const auto& live = controlled.outcomes[d];
        same_decision(decision, controlled.decisions[d], "controlled decision "+std::to_string(d));
        require(decision.driven_by == live.driven_by && decision.controller_status.has_value() == live.asked &&
                (!live.asked || *decision.controller_status == live.status) && decision.controller_iterations == live.iterations &&
                decision.controller_restarted == live.restarted && decision.controller_note == live.note,
                "decision "+std::to_string(d)+" records who drove it as the simulation did");
        if (decision.driven_by == fd::ControllerMode::mpcc) {
            ++by_controller;
            require(decision.choice().trajectory.size() == 61, "a controller decision records its plan");
        }
        if (decision.holding) ++holding;
        if (decision.controller_status == fd::ControllerStatus::not_solved) ++unsolved;
    }
    require(by_controller > 0 && holding > 0 && unsolved > 0, "the fixture has decisions driven by the controller, holds and unsolved plans");
    // Plan errors: every plan starts at the recorded state, so none now, beyond the twelve digits trajectories.csv keeps
    // against telemetry.csv's full precision; a second ahead is measured wherever the recording reaches; and a run the
    // policy drove has none to measure.
    const auto errors = fd::controller_plan_errors(recording, {0.0, 1.0, 20.0});
    require(errors.size() == 3 && errors[0].samples == by_controller && errors[0].worst_m < 1e-9, "a plan starts where the car was");
    require(errors[1].samples > 0 && errors[1].samples <= by_controller && std::isfinite(errors[1].worst_m), "and is measured a second on");
    require(errors[2].samples == 0, "but not beyond its horizon");
    require(fd::controller_plan_errors(fd::load_recording(plain.directory), {1.0}).empty(), "a policy run has no plan errors");
    // Schema 13: every plan that drove keeps its commands, and nothing else has any.
    for (std::size_t d = 0; d < recording.decisions.size(); ++d) {
        const auto& decision = recording.decisions[d];
        for (std::size_t o = 0; o < decision.options.size(); ++o) {
            const auto& recorded = decision.options[o].commands;
            const auto& live = controlled.decisions[d].options[o].trajectory.commands;
            require(recorded.size() == live.size(), "decision "+std::to_string(d)+" option "+std::to_string(o)+" keeps its commands");
            require((decision.driven_by == fd::ControllerMode::mpcc && o == decision.selected) == !recorded.empty(),
                    "only the plan that drove has commands");
            for (std::size_t k = 0; k < recorded.size(); ++k) {
                same_value(recorded[k].acceleration_mps2, live[k].acceleration_mps2, "a command's acceleration round-trips");
                same_value(recorded[k].steering_rad, live[k].steering_rad, "and its steering");
            }
        }
    }
    // The model error: every plan starts at the recorded state and reaches its horizon whether or not the recording does;
    // these plans hold the policy's first command for three seconds, so the plant departs from them.
    const auto model = fd::controller_model_errors(recording, {0.0, 1.0, 20.0});
    require(model.size() == 3 && model[0].samples == by_controller && model[0].worst_m < 1e-9, "the plant starts where the plan does");
    require(model[1].samples == by_controller && model[1].worst_m > 0 && std::isfinite(model[1].worst_m), "and is measured a second on for every plan");
    require(model[2].samples == 0, "but not beyond the horizon");
    require(fd::controller_model_errors(fd::load_recording(plain.directory), {1.0}).empty(), "a policy run has no model errors");
    std::cout << "  controlled: " << by_controller << " decisions driven by the controller, " << holding << " holding, " << unsolved
              << " with an unsolved plan\n";

    const auto column_of = [](const std::vector<std::string>& header, const std::string& name) {
        return static_cast<std::size_t>(std::find(header.begin(), header.end(), name)-header.begin());
    };
    // Rewrites one column on every row of the first decision whose first row the predicate accepts.
    const auto tamper = [&](const Fixture& source, const std::string& name, const std::function<bool(const std::vector<std::string>&, const std::vector<std::string>&)>& choose,
                            const std::string& column, const std::string& value, const std::string& fragment) {
        const fs::path dir = variant(source, name, "decisions.csv", [&](std::string& text) {
            auto rows = lines(text);
            const auto header = split(rows[0]);
            std::string chosen;
            for (std::size_t i = 1; i < rows.size() && chosen.empty(); ++i) {
                const auto fields = split(rows[i]);
                if (choose(fields, header)) chosen = fields[0];
            }
            require(!chosen.empty(), name+": the run has a decision to tamper with");
            for (std::size_t i = 1; i < rows.size(); ++i) {
                auto fields = split(rows[i]);
                if (fields[0] != chosen) continue;
                fields[column_of(header, column)] = value;
                rows[i] = join_fields(fields);
            }
            text = join(rows);
        });
        const std::string message = rejects([&] { fd::load_recording(dir); }, name+": tampered recording must be rejected");
        require(message.find(fragment) != std::string::npos, name+": failure names the violated rule, got: "+message);
        std::cout << "  " << name << " -> " << message << '\n';
    };
    const auto is = [&](const std::string& column, const std::string& value) {
        return [=](const std::vector<std::string>& f, const std::vector<std::string>& header) { return f[column_of(header, column)] == value; };
    };
    tamper(plain, "policy-run-claims-a-controller", is("option", "0"), "driven_by", "mpcc", "controller the run did not have");
    tamper(controlled, "unsolved-plan-drove", is("driven_by", "mpcc"), "controller_status", "not solved", "without a solved plan");
    tamper(controlled, "silent-policy", [&](const std::vector<std::string>& f, const std::vector<std::string>& header) {
        return f[column_of(header, "driven_by")] == "policy" && f[column_of(header, "holding")] == "0";
    }, "controller_note", "", "without saying why");
    tamper(controlled, "asked-to-hold", is("holding", "1"), "controller_status", "solved", "asked the predictive controller to hold");
    // commands.csv: one command at every point of each plan that drove, at its times, driving its first.
    const auto refuse_commands = [&](const std::string& name, const std::function<void(std::string&)>& edit, const std::string& fragment) {
        const fs::path dir = variant(controlled, name, "commands.csv", edit);
        const std::string message = rejects([&] { fd::load_recording(dir); }, name+": tampered recording must be rejected");
        require(message.find(fragment) != std::string::npos, name+": failure names the violated rule, got: "+message);
        std::cout << "  " << name << " -> " << message << '\n';
    };
    const auto first_driven = [&] {
        for (std::size_t d = 0; d < recording.decisions.size(); ++d)
            if (recording.decisions[d].driven_by == fd::ControllerMode::mpcc) return std::to_string(d);
        return std::string();
    }();
    refuse_commands("command-missing", [&](std::string& text) { drop_rows(text, where({{"decision", first_driven}, {"point", "60"}})); },
                    "one command at every point");
    refuse_commands("command-late", [&](std::string& text) { edit_row(text, where({{"decision", first_driven}, {"point", "3"}}), "time_s", "999"); },
                    "not the time of the plan's point");
    refuse_commands("command-not-driven", [&](std::string& text) { edit_row(text, where({{"decision", first_driven}, {"point", "1"}}), "acceleration_mps2", "7.5"); },
                    "not its plan's at the end of its control interval");
    refuse_commands("command-beyond-the-limit", [&](std::string& text) { edit_row(text, where({{"decision", first_driven}, {"point", "30"}}), "steering_rad", "1.5"); },
                    "exceeds the steering limit");
    refuse_commands("command-extra", [&](std::string& text) { text += "999999,0,0,0,0,0\n"; }, "beyond the plans");
    const fs::path twelve = downgrade(controlled, "controller-schema-12", 12);
    const auto older = fd::load_recording(twelve);
    require(older.metadata.schema_version == 12 && !older.commands_recorded() && fd::controller_model_errors(older, {1.0}).empty() &&
            !fd::controller_plan_errors(older, {1.0}).empty(), "a schema 12 run has plan errors but no commands to measure the model error with");
    fs::copy_file(controlled.directory/"commands.csv", twelve/"commands.csv");
    require(rejects([&] { fd::load_recording(twelve); }, "a schema 12 run beside commands.csv must be rejected").find("commands.csv") != std::string::npos,
            "the stray commands file is named");

    const fs::path reshaped = variant(controlled, "plan-of-another-shape", "metadata.json", [](std::string& text) {
        replace_first(text, "\"stages\": 60", "\"stages\": 30");
    });
    const std::string message = rejects([&] { fd::load_recording(reshaped); }, "a plan of another shape must be rejected");
    require(message.find("not the controller's 30 stages") != std::string::npos, "the plan's shape is named, got: "+message);
    std::cout << "  plan-of-another-shape -> " << message << '\n';
}

// A controller that keeps to its plans, each the plant's response to its commands: the model error is nothing, since the
// recorded plant under a plan's recorded commands goes where the plan says, off the control grid after the grip change
// too; and the plan error is nothing wherever the grip did not change under the plan, since the car kept to it.
void a_plan_kept_to_has_no_error(const Fixture& kept) {
    const auto recording = fd::load_recording(kept.directory);
    require(recording.events.size() == 1 && std::abs(recording.events[0].time_s-2.01) < 1e-9, "the grip changes between control ticks");
    const auto driven = static_cast<std::size_t>(std::count_if(recording.decisions.begin(), recording.decisions.end(), [](const fd::RecordedDecision& d) {
        return d.driven_by == fd::ControllerMode::mpcc; }));
    require(driven == recording.decisions.size(), "the controller drove every decision");
    for (const auto& e : fd::controller_model_errors(recording, {0.0, 0.5, 1.0, 2.0, 3.0})) {
        require(e.samples == driven && e.worst_m < 1e-6 && e.percentile_95_speed_mps < 1e-6,
                "the model error "+std::to_string(e.ahead_s)+" s ahead is nothing: "+std::to_string(e.worst_m)+" m");
    }
    const double dt = recording.metadata.initial_config.fixed_dt_s;
    double worst = 0, across_the_change = 0;
    std::size_t compared = 0;
    for (const auto& decision : recording.decisions) {
        for (const auto& point : decision.choice().trajectory) {
            const auto tick = static_cast<std::size_t>(std::llround(point.time_s/dt));
            if (tick >= recording.samples.size()) break;
            const double apart = std::hypot(point.x_m-recording.samples[tick].x_m, point.y_m-recording.samples[tick].y_m);
            if (decision.time_s < 2.01-1e-9 && point.time_s > 2.01) { across_the_change = std::max(across_the_change, apart); continue; }
            worst = std::max(worst, apart);
            ++compared;
        }
    }
    std::cout << "  kept to: " << driven << " plans, " << compared << " points where the car went exactly as planned (worst " << worst
              << " m); across the grip change up to " << across_the_change << " m\n";
    require(compared > 1000 && worst < 1e-6, "the car went where each plan said");
    require(across_the_change > 1e-4, "except where the grip changed under a plan, which its commands did not foresee");
}

// The five-offset planner still records its offsets, without actions beyond them or paths; a schema 9 run of it, made
// before actions were recorded, still loads with its alternatives.
void five_offset_scenario_round_trip(const Fixture& scenario) {
    const auto recording = fd::load_recording(scenario.directory);
    require(recording.metadata.local_planner_mode == fd::LocalPlannerMode::five_offsets, "the run records the five-offset planner");
    std::size_t avoiding = 0;
    for (std::size_t d = 0; d < recording.decisions.size(); ++d) {
        const auto& decision = recording.decisions[d];
        same_decision(decision, scenario.decisions[d], "five-offset decision "+std::to_string(d));
        require(std::all_of(decision.options.begin(), decision.options.end(), [](const fd::RecordedOption& o) {
            return o.action == fd::LocalAction::offset && o.path.empty() && o.speed_profile.empty(); }), "every five-offset option is an offset without a path or profile");
        if (decision.options.size() > 1 && !decision.holding) ++avoiding;
    }
    require(avoiding > 0, "decisions steering around the lane blockage are recorded with their alternatives");
    const auto nine = fd::load_recording(downgrade(scenario, "five-offsets-schema-9", 9));
    require(nine.metadata.schema_version == 9 && nine.decisions.size() == recording.decisions.size() &&
            nine.metadata.local_planner_mode == fd::LocalPlannerMode::five_offsets, "a schema 9 scenario reads as planned by five offsets");
    const fs::path relabelled = variant(scenario, "five-offsets-as-lattice", "metadata.json", [](std::string& text) {
        replace_first(text, "\"local_planner\": \"five offsets\"", "\"local_planner\": \"lattice\"");
    });
    require(rejects([&] { fd::load_recording(relabelled); }, "offsets recorded under the lattice planner must be rejected").find("planner") != std::string::npos,
            "the planner that could not have taken the action is named");
}

// Rewrites a schema 7 run as an older schema would have written it: without the four-wheel car's aerodynamics
// and viscous coupling; before schema 6 also without the wheel load columns and the four-wheel car's centre of
// gravity height and roll balance; before schema 5 also without the wheel speed
// columns; before schema 4 also without the steering law and the geometric steering column; before schema 3
// also without the plant columns, the vehicle model and the summary's sideslip; and for schema 1 without the
// decision files.
fs::path downgrade(const Fixture& fixture, const std::string& name, int schema, bool keep_decision_files) {
    // Schema 14 appended the achieved longitudinal acceleration, after the loads, wheels, steering and plant state.
    const std::size_t dropped_columns = schema >= 14 ? 0 : schema >= 6 ? 1 : schema >= 5 ? 5 : schema >= 4 ? 9
                                        : schema >= 3 ? 10 : 13;
    const fs::path old = variant(fixture, name, "telemetry.csv", [&](std::string& text) {
        auto rows = lines(text);
        for (auto& row : rows) { auto fields = split(row); fields.resize(fields.size()-dropped_columns); row = join_fields(fields); }
        text = join(rows);
    });
    const auto rewrite = [&](const std::string& file, const std::function<void(std::string&)>& change) { rewrite_file(old, file, change); };
    // Schema 9 added each track sample's distance to the corridor's edges.
    if (schema < 9) rewrite("track.csv", [](std::string& text) {
        auto rows = lines(text);
        for (auto& row : rows) { auto fields = split(row); fields.resize(fields.size()-2); row = join_fields(fields); }
        text = join(rows);
    });
    // Schema 17 added timing.csv and the course's cones whether perceived or not, schema 16 the belief files, schema 15 the
    // cones and perception files, schema 14 measurements.csv and schema 13 commands.csv.
    if (schema < 17) {
        fs::remove(old/"timing.csv");
        rewrite("cones.csv", [](std::string& text) { text = lines(text).front()+"\n"; });
    }
    if (schema < 16)
        for (const char* file : {"beliefs.csv", "believed_paths.csv"}) fs::remove(old/file);
    if (schema < 15)
        for (const char* file : {"cones.csv", "perception_frames.csv", "detections.csv", "missed.csv"}) fs::remove(old/file);
    if (schema < 14) fs::remove(old/"measurements.csv");
    if (schema < 13) fs::remove(old/"commands.csv");
    // Schema 12 added who drove each decision and how the predictive controller fared.
    if (schema < 12) rewrite("decisions.csv", [](std::string& text) {
        auto rows = lines(text);
        for (auto& row : rows) { auto fields = split(row); fields.resize(fields.size()-5); row = join_fields(fields); }
        text = join(rows);
    });
    // Schema 11 added each option's speed profile point count and estimated time, and profiles.csv.
    if (schema < 11) {
        rewrite("decisions.csv", [](std::string& text) {
            auto rows = lines(text);
            for (auto& row : rows) { auto fields = split(row); fields.resize(fields.size()-2); row = join_fields(fields); }
            text = join(rows);
        });
        fs::remove(old/"profiles.csv");
    }
    // Schema 10 added each option's action and path cost, paths.csv and the local planner.
    if (schema < 10) {
        rewrite("decisions.csv", [](std::string& text) {
            auto rows = lines(text);
            for (auto& row : rows) { auto fields = split(row); fields.resize(fields.size()-8); row = join_fields(fields); }
            text = join(rows);
        });
        fs::remove(old/"paths.csv");
    }
    rewrite("metadata.json", [&](std::string& text) {
        as_older(text, schema);
        const auto erase_key = [&](const std::string& key) {
            const auto at = text.find(", \""+key+"\": ");
            if (at != std::string::npos) text.erase(at, text.find_first_of(",}", at+2)-at);
        };
        const auto rename = [&](const std::string& from, const std::string& to) {
            const auto at = text.find(from);
            if (at != std::string::npos) text.replace(at, from.size(), to);
        };
        const auto erase_line = [&](const std::string& key) {
            const auto at = text.find("  \""+key+"\": ");
            require(at != std::string::npos, "fixture metadata has a "+key+" line");
            text.erase(at, text.find('\n', at)-at+1);
        };
        // Schema 13 added the dynamic car's pitch and air; the four-wheel car's are older.
        if (schema < 13 && text.find("\"kind\": \"dynamic_single_track\"") != std::string::npos)
            for (const char* key : {"cg_height_m", "drag_area_m2", "downforce_area_m2", "aero_balance_front"}) erase_key(key);
        if (schema < 12) erase_line("predictive_controller");
        if (schema < 10) erase_line("local_planner");
        if (schema < 8) {
            erase_line("speed_plan");
            // The last configuration line loses its comma along with the line after it.
            erase_line("envelope_fraction");
            const auto last = text.find("\"lookahead_time_s\": ");
            const auto comma = text.find(',', last);
            if (last != std::string::npos && comma < text.find('\n', last)) text.erase(comma, 1);
        }
        if (schema < 7) {
            for (const char* key : {"viscous_coupling_nms", "drag_area_m2", "downforce_area_m2", "aero_balance_front"}) erase_key(key);
            rename("rotating wheels, load transfer and aerodynamics", "rotating wheels and load transfer");
            rename("rotating wheels and aerodynamics", "rotating wheels");
        }
        if (schema < 6) {
            for (const char* key : {"cg_height_m", "roll_balance_front"}) erase_key(key);
            rename("rotating wheels and load transfer", "rotating wheels");
        }
        if (schema < 4) erase_line("steering");
        if (schema < 3) erase_line("vehicle_model");
    });
    if (schema < 3) rewrite("summary.json", [&](std::string& text) {
        const auto at = text.find("  \"max_rear_sideslip_rad\": ");
        require(at != std::string::npos, "fixture summary has a sideslip line");
        text.erase(at, text.find('\n', at)-at+1);
    });
    if (schema == 1 && !keep_decision_files) { fs::remove(old/"decisions.csv"); fs::remove(old/"trajectories.csv"); }
    return old;
}

void older_schemas_still_load(const Fixture& fixture) {
    const auto eleven = fd::load_recording(downgrade(fixture, "schema-11", 11));
    require(eleven.metadata.schema_version == 11 && !eleven.controller_recorded() && eleven.speed_profiles_recorded() &&
            eleven.metadata.predictive_controller.mode == fd::ControllerMode::policy &&
            std::all_of(eleven.decisions.begin(), eleven.decisions.end(), [](const fd::RecordedDecision& d) {
                return d.driven_by == fd::ControllerMode::policy && !d.controller_status; }), "a schema 11 run reads as driven by the policy");
    const fs::path early_controller = variant(fixture, "controller-in-schema-11", "metadata.json",
                                              [](std::string& text) { as_older(text, 11); });
    require(rejects([&] { fd::load_recording(early_controller); }, "schema 11 metadata naming a controller must be rejected").find("schema_version 12") != std::string::npos,
            "the out-of-schema controller is explained");

    const auto ten = fd::load_recording(downgrade(fixture, "schema-10", 10));
    require(ten.metadata.schema_version == 10 && ten.local_planner_recorded() && !ten.speed_profiles_recorded() &&
            std::all_of(ten.decisions.begin(), ten.decisions.end(), [](const fd::RecordedDecision& d) {
                return d.choice().speed_profile.empty() && !d.choice().estimated_time_s; }), "a schema 10 run reads without speed profiles");
    const fs::path stray_profiles = downgrade(fixture, "schema-10-with-profiles", 10);
    fs::copy_file(fixture.directory/"profiles.csv", stray_profiles/"profiles.csv");
    require(rejects([&] { fd::load_recording(stray_profiles); }, "a schema 10 run beside profiles.csv must be rejected").find("profiles.csv") != std::string::npos,
            "the stray profiles file is named");

    const auto nine = fd::load_recording(downgrade(fixture, "schema-9", 9));
    require(nine.metadata.schema_version == 9 && !nine.local_planner_recorded() && nine.metadata.local_planner_mode == fd::LocalPlannerMode::five_offsets,
            "a schema 9 run reads as planned by the five-offset planner");
    require(std::all_of(nine.decisions.begin(), nine.decisions.end(), [](const fd::RecordedDecision& d) {
        return d.choice().action == fd::LocalAction::offset && d.choice().path.empty(); }), "a schema 9 run's options are offsets without paths");
    const fs::path early_planner = variant(fixture, "local-planner-in-schema-9", "metadata.json",
                                           [](std::string& text) { as_older(text, 9); });
    require(rejects([&] { fd::load_recording(early_planner); }, "schema 9 metadata naming a local planner must be rejected").find("schema_version 10") != std::string::npos,
            "the out-of-schema local planner is explained");
    fs::path stray_paths = downgrade(fixture, "schema-9-with-paths", 9);
    fs::copy_file(fixture.directory/"paths.csv", stray_paths/"paths.csv");
    require(rejects([&] { fd::load_recording(stray_paths); }, "a schema 9 run beside paths.csv must be rejected").find("paths.csv") != std::string::npos,
            "the stray paths file is named");

    const auto eight = fd::load_recording(downgrade(fixture, "schema-8", 8));
    require(eight.metadata.schema_version == 8 && !eight.edges_recorded() && eight.track.left_edge_m.empty(),
            "a schema 8 run reads back with a centreline, as its track was");

    const auto seven = fd::load_recording(downgrade(fixture, "schema-7", 7));
    require(seven.metadata.schema_version == 7 && seven.metadata.speed_plan_mode == fd::SpeedPlanMode::grip_fractions &&
            seven.metadata.initial_config.envelope_fraction == fd::Config{}.envelope_fraction, "a schema 7 run reads as planned with the grip fractions");
    require(fd::plan_reproduction_error_mps(seven) < 1e-9, "a schema 7 run's plans are still reproduced");
    const fs::path early_plan = variant(fixture, "speed-plan-in-schema-7", "metadata.json",
                                        [](std::string& text) { as_older(text, 7); });
    require(rejects([&] { fd::load_recording(early_plan); }, "schema 7 metadata with a speed plan must be rejected").find("schema_version 8") != std::string::npos,
            "the out-of-schema speed plan is explained");

    const auto six = fd::load_recording(downgrade(fixture, "schema-6", 6));
    require(six.metadata.schema_version == 6 && six.loads_recorded(), "a schema 6 run reads as schema 6");
    const auto six_reproduced = fd::plant_reproduction_error_m(six);
    require(six_reproduced && *six_reproduced < 1e-6, "a schema 6 run can still be re-integrated");

    const auto five = fd::load_recording(downgrade(fixture, "schema-5", 5));
    require(five.metadata.schema_version == 5 && five.wheels_recorded() && !five.loads_recorded(), "a schema 5 run reads as schema 5");
    require(std::all_of(five.samples.begin(), five.samples.end(), [](const fd::RecordedSample& s) {
        return s.wheel_loads_n == std::array<double, 4>{}; }), "a kinematic schema 5 run has no wheel loads");
    const auto five_reproduced = fd::plant_reproduction_error_m(five);
    require(five_reproduced && *five_reproduced < 1e-6, "a schema 5 run can still be re-integrated");

    const auto four = fd::load_recording(downgrade(fixture, "schema-4", 4));
    require(four.metadata.schema_version == 4 && four.steering_recorded() && !four.wheels_recorded(), "a schema 4 run reads as schema 4");
    require(std::all_of(four.samples.begin(), four.samples.end(), [](const fd::RecordedSample& s) {
        return s.wheel_speeds_radps == std::array<double, 4>{}; }), "a schema 4 run has no wheel speeds");
    const auto four_reproduced = fd::plant_reproduction_error_m(four);
    require(four_reproduced && *four_reproduced < 1e-6, "a schema 4 run can still be re-integrated");

    const auto three = fd::load_recording(downgrade(fixture, "schema-3", 3));
    require(three.metadata.schema_version == 3 && three.plant_recorded(), "a schema 3 run reads as schema 3");
    require(three.metadata.steering_mode == fd::SteeringMode::pure_pursuit, "a schema 3 run was steered by Pure Pursuit");
    require(std::all_of(three.samples.begin(), three.samples.end(), [](const fd::RecordedSample& s) {
        return s.geometric_steering_rad == s.requested_steering_rad; }), "a schema 3 run's geometric steering is its requested steering");
    const auto three_reproduced = fd::plant_reproduction_error_m(three);
    require(three_reproduced && *three_reproduced < 1e-6, "a schema 3 run can still be re-integrated");
    const fs::path early_steering = variant(fixture, "steering-in-schema-3", "metadata.json", [](std::string& text) { as_schema(text, 3); });
    require(rejects([&] { fd::load_recording(early_steering); }, "schema 3 metadata with a steering law must be rejected").find("steering") != std::string::npos,
            "the out-of-schema steering law is named");

    const auto two = fd::load_recording(downgrade(fixture, "schema-2", 2));
    require(two.metadata.schema_version == 2 && two.decisions_recorded() && !two.plant_recorded(), "a schema 2 run reads as schema 2");
    require(std::holds_alternative<fd::KinematicBicycle>(two.metadata.vehicle_model), "a schema 2 run was a kinematic run");
    require(two.decisions.size() == fixture.decisions.size(), "a schema 2 run keeps its decisions");
    const auto two_reproduced = fd::plant_reproduction_error_m(two);
    require(two_reproduced && *two_reproduced < 1e-6, "a schema 2 kinematic run can still be re-integrated");

    const fs::path old = downgrade(fixture, "schema-1", 1);
    const auto recording = fd::load_recording(old);
    require(recording.metadata.schema_version == 1 && !recording.decisions_recorded(), "a schema 1 run reads as schema 1");
    fd::Playback old_playback(fd::load_recording(old));
    old_playback.seek_time(12);
    require(fd::handling_balance(old_playback.demand()).balance == fd::Balance::not_modeled,
            "an older kinematic run reports no handling balance"); 
    require(recording.decisions.empty() && recording.decision_at(10) == nullptr, "schema 1 has no recorded decision");
    require(!fd::plant_reproduction_error_m(recording), "schema 1 recorded no steering command to re-integrate with");
    fd::Playback playback(fd::load_recording(old));
    playback.seek_time(10);
    require(playback.decision() == nullptr, "schema 1 playback exposes no decision rather than inventing one");

    const fs::path stray = downgrade(fixture, "schema-1-with-decisions", 1, true);
    const std::string message = rejects([&] { fd::load_recording(stray); }, "schema 1 metadata beside decision files must be rejected");
    require(message.find("schema 1") != std::string::npos, "the stray decision files are explained, got: "+message);
    const fs::path future = variant(fixture, "schema-19", "metadata.json",
                                    [](std::string& text) { replace_first(text, "\"schema_version\": 18", "\"schema_version\": 19"); });
    require(rejects([&] { fd::load_recording(future); }, "an unknown schema must be rejected").find("schema_version") != std::string::npos,
            "the unsupported schema is named");
    const fs::path missing = variant(fixture, "missing-decisions", "trajectories.csv", [](std::string&) {});
    fs::remove(missing/"decisions.csv");
    const fs::path no_paths = variant(fixture, "missing-paths", "trajectories.csv", [](std::string&) {});
    fs::remove(no_paths/"paths.csv");
    require(rejects([&] { fd::load_recording(no_paths); }, "schema 11 without paths.csv must be rejected").find("paths.csv") != std::string::npos,
            "the missing paths file is named");
    const fs::path no_profiles = variant(fixture, "missing-profiles", "trajectories.csv", [](std::string&) {});
    fs::remove(no_profiles/"profiles.csv");
    require(rejects([&] { fd::load_recording(no_profiles); }, "schema 11 without profiles.csv must be rejected").find("profiles.csv") != std::string::npos,
            "the missing profiles file is named");
    require(rejects([&] { fd::load_recording(missing); }, "schema 11 without decisions.csv must be rejected").find("decisions.csv") != std::string::npos,
            "the missing decision file is named");
}

void dynamic_plant_round_trip(const Fixture& dynamic) {
    const auto recording = fd::load_recording(dynamic.directory);
    const auto* car = std::get_if<fd::DynamicSingleTrack>(&recording.metadata.vehicle_model);
    require(car, "the dynamic plant is recorded as the vehicle model");
    const auto expected = dynamic_car();
    require(car->mass_kg == expected.mass_kg && car->yaw_inertia_kgm2 == expected.yaw_inertia_kgm2 &&
            car->cg_to_rear_m == expected.cg_to_rear_m && car->drive_front_fraction == expected.drive_front_fraction &&
            car->kinematic_below_mps == expected.kinematic_below_mps && car->dynamic_above_mps == expected.dynamic_above_mps &&
            car->slip_speed_floor_mps == expected.slip_speed_floor_mps && car->substep_s == expected.substep_s,
            "every vehicle parameter round-trips");
    require(car->rear_tire.peak_slip_angle_high_rad == expected.rear_tire.peak_slip_angle_high_rad &&
            car->front_tire.sliding_fraction_lateral == expected.front_tire.sliding_fraction_lateral &&
            car->rear_tire.minimum_friction == expected.rear_tire.minimum_friction, "tire parameters round-trip");
    require(recording.metadata.model == fd::vehicle_model_name(recording.metadata.vehicle_model), "the model name matches the recorded model");
    require(car->cg_height_m == 0 && car->drag_area_m2 == 0 && car->downforce_area_m2 == 0 && car->aero_balance_front == 0.5,
            "the dynamic car has no pitch or air unless given them");
    // Schema 13 records the dynamic car's pitch and air (decision 0025), and the plant is reproduced with them.
    {
        fd::DynamicSingleTrack pitched = expected;
        pitched.cg_height_m = 0.35;
        pitched.drag_area_m2 = 0.6;
        pitched.downforce_area_m2 = 1.5;
        pitched.aero_balance_front = 0.4;
        fd::Simulation run(fd::make_preset_track(), fd::Config{}, pitched);
        Fixture short_run;
        short_run.directory = dynamic.directory.parent_path()/"pitched";
        fd::RecordingWriter writer(short_run.directory, run, {0, 2.0, {}}, {});
        for (int i = 0; i < 400; ++i) { run.step(); writer.record(run); }
        writer.finish(run);
        const auto read = fd::load_recording(short_run.directory);
        const auto& back = std::get<fd::DynamicSingleTrack>(read.metadata.vehicle_model);
        require(back.cg_height_m == 0.35 && back.drag_area_m2 == 0.6 && back.downforce_area_m2 == 1.5 && back.aero_balance_front == 0.4,
                "the dynamic car's pitch and air round-trip");
        const auto reproduced = fd::plant_reproduction_error_m(read);
        require(reproduced && *reproduced < 1e-6, "and its recorded motion is reproduced with them");
        const fs::path older = variant(short_run, "pitched-in-schema-12", "metadata.json",
                                       [](std::string& text) { as_older(text, 12); });
        require(rejects([&] { fd::load_recording(older); }, "a schema 12 dynamic car with pitch must be rejected").find("requires schema_version 13") != std::string::npos,
                "the out-of-schema pitch and air are explained");
    }
    require(recording.samples.size() == dynamic.live.size(), "every dynamic sample is recorded");
    double largest_slip = 0;
    for (std::size_t i = 0; i < recording.samples.size(); ++i) {
        const auto& s = recording.samples[i];
        const auto& l = dynamic.live[i];
        same_value(s.lateral_velocity_mps, l.lateral_velocity_mps, "recorded lateral velocity equals the live plant");
        same_value(s.yaw_rate_radps, l.yaw_rate_radps, "recorded yaw rate equals the live plant");
        require(s.within_grip_envelope == l.within_grip_envelope, "recorded grip envelope equals the live plant");
        largest_slip = std::max(largest_slip, std::abs(fd::rear_sideslip_rad(s.plant_state())));
    }
    require(largest_slip > 0, "the dynamic run records nonzero sideslip");
    same_value(recording.summary.max_rear_sideslip_rad, largest_slip, "the summary's sideslip is the telemetry's");
    for (std::size_t d = 0; d < recording.decisions.size(); ++d)
        same_decision(recording.decisions[d], dynamic.decisions[d], "dynamic decision "+std::to_string(d));
    // Replay derives slip angles and balance from the recorded plant state through the seam, without
    // advancing anything, and gets what the live simulation reported.
    fd::Playback playback(fd::load_recording(dynamic.directory));
    std::size_t understeering = 0, oversteering = 0;
    for (std::size_t i = 0; i < recording.samples.size(); ++i) {
        playback.seek_index(i);
        const auto derived = playback.demand();
        const auto balance = fd::handling_balance(derived);
        const auto& l = dynamic.live[i];
        near(derived.front_slip_angle_rad, l.front_slip_angle_rad, 1e-9, "replayed front slip angle equals the live one");
        near(derived.rear_slip_angle_rad, l.rear_slip_angle_rad, 1e-9, "replayed rear slip angle equals the live one");
        near(balance.understeer_angle_rad, l.understeer_angle_rad, 1e-9, "replayed understeer angle equals the live one");
        // A value within rounding of a threshold could flip; none in this fixture is that close.
        require(balance.balance == l.balance, "replayed balance equals the live one at sample "+std::to_string(i));
        if (l.balance == fd::Balance::understeer) ++understeering;
        if (l.balance == fd::Balance::oversteer) ++oversteering;
        require(playback.time_s() == recording.samples[i].time_s, "deriving demand does not move the cursor");
    }
    std::cout << "  dynamic fixture balance: " << understeering << " understeering and " << oversteering << " oversteering samples\n";
    const auto reproduced = fd::plant_reproduction_error_m(recording);
    require(reproduced && *reproduced < 1e-6, "re-integrating the recorded dynamic plant reproduces its motion");
    std::cout << "  dynamic fixture: " << recording.samples.size() << " samples, max rear sideslip " << largest_slip
              << " rad, plant reproduction " << *reproduced << " m\n";

    // A recorded parameter edited by hand still loads, but the plant no longer reproduces the motion.
    const fs::path heavier = variant(dynamic, "dynamic-mass", "metadata.json",
                                     [](std::string& text) { replace_first(text, "\"mass_kg\": 800", "\"mass_kg\": 1100"); });
    const auto edited = fd::load_recording(heavier);
    const auto edited_error = fd::plant_reproduction_error_m(edited);
    // One 5 ms step is a small lever, but three orders of magnitude above rounding is unmistakable.
    require(edited_error && *edited_error > 1e-6 && *edited_error > 1000*(*reproduced),
            "a changed mass is exposed by plant reproduction, got "+std::to_string(edited_error.value_or(-1)));
    std::cout << "  mass edited 800 to 1100 kg: plant reproduction " << *edited_error << " m\n";
}

// The dynamic fixture is steered by MAP: the law and its table's fingerprint are recorded, the
// geometric steering column keeps what Pure Pursuit would have asked, and the difference is MAP's.
void map_steering_round_trip(const Fixture& dynamic) {
    const auto recording = fd::load_recording(dynamic.directory);
    require(recording.metadata.steering_mode == fd::SteeringMode::model_acceleration_pursuit, "the MAP run records its steering law");
    require(recording.metadata.steering_table_fingerprint == fd::steering_table_fingerprint(dynamic_car(), fd::Config{}),
            "the recorded fingerprint is the table generated for the initial plant and configuration");
    const auto matches = fd::recorded_steering_table_matches(recording);
    require(matches && *matches, "this build derives the recorded fingerprint from the recorded plant");
    double largest_correction = 0;
    for (std::size_t i = 0; i < recording.samples.size(); ++i) {
        const auto& s = recording.samples[i];
        same_value(s.geometric_steering_rad, dynamic.live[i].geometric_steering_rad, "recorded geometric steering equals the live controller");
        same_value(s.requested_steering_rad, dynamic.live[i].requested_steering_rad, "recorded MAP steering equals the live controller");
        largest_correction = std::max(largest_correction, std::abs(s.requested_steering_rad-s.geometric_steering_rad));
    }
    require(largest_correction > 1e-4, "MAP's steering differs from geometry somewhere in the run");
    std::cout << "  MAP fixture: table " << recording.metadata.steering_table_fingerprint << ", largest correction to geometry "
              << largest_correction << " rad\n";
    const fs::path other = variant(dynamic, "map-other-table", "metadata.json", [&](std::string& text) {
        replace_first(text, recording.metadata.steering_table_fingerprint, "0123456789abcdef");
    });
    const auto foreign = fd::load_recording(other);
    const auto foreign_matches = fd::recorded_steering_table_matches(foreign);
    require(foreign_matches && !*foreign_matches, "a fingerprint from another table loads but is reported as not this plant's");
}

// A run planned with the performance envelope records the plan mode, the envelope's fingerprint and the share of it
// used; this build derives the recorded envelope again for every plan revision and reproduces each plan exactly.
void envelope_plan_round_trip(const Fixture& envelope) {
    const auto recording = fd::load_recording(envelope.directory);
    require(recording.metadata.speed_plan_mode == fd::SpeedPlanMode::performance_envelope, "the run records its speed plan mode");
    require(recording.metadata.envelope_fingerprint == fd::performance_envelope_fingerprint(fd::FourWheelCar{}, fd::Config{}),
            "the recorded fingerprint is the envelope derived for the initial car and configuration");
    require(recording.metadata.initial_config.envelope_fraction == fd::Config{}.envelope_fraction, "the share of the envelope is recorded");
    const auto matches = fd::recorded_performance_envelope_matches(recording);
    require(matches && *matches, "this build derives the recorded envelope from the recorded car");
    require(recording.plans.size() == 2, "the grip change made a second plan revision");
    const double reproduced = fd::plan_reproduction_error_mps(recording);
    require(reproduced < 1e-9, "both revisions' envelope plans are reproduced, got "+std::to_string(reproduced));
    const auto baseline = fd::make_speed_plan(recording.track, recording.config_for_revision(1));
    double difference = 0;
    for (std::size_t i = 0; i < baseline.size(); ++i)
        difference = std::max(difference, std::abs(baseline[i].speed_mps-recording.plans[0].points[i].speed_mps));
    require(difference > 1, "the recorded plan is not the grip fraction plan");
    std::cout << "  envelope fixture: envelope " << recording.metadata.envelope_fingerprint << ", plans reproduced within " << reproduced
              << " m/s, up to " << difference << " m/s from the grip fraction plan\n";

    const fs::path other = variant(envelope, "envelope-other-fingerprint", "metadata.json", [&](std::string& text) {
        replace_first(text, recording.metadata.envelope_fingerprint, "0123456789abcdef");
    });
    const auto foreign = fd::load_recording(other);
    const auto foreign_matches = fd::recorded_performance_envelope_matches(foreign);
    require(foreign_matches && !*foreign_matches, "a fingerprint from another envelope loads but is reported as not this car's");
    const fs::path larger = variant(envelope, "envelope-larger-share", "metadata.json", [](std::string& text) {
        replace_first(text, "\"envelope_fraction\": 0.8", "\"envelope_fraction\": 0.95");
    });
    const auto edited = fd::load_recording(larger);
    require(fd::plan_reproduction_error_mps(edited) > 0.1, "an edited share is exposed by plan reproduction");
    const fs::path kinematic = variant(envelope, "envelope-kinematic", "metadata.json", [](std::string& text) {
        const auto at = text.find("  \"vehicle_model\": ");
        text.replace(at, text.find('\n', at)-at, "  \"vehicle_model\": {\"kind\": \"kinematic_bicycle\"},");
    });
    require(rejects([&] { fd::load_recording(kinematic); }, "an envelope plan for a car without an envelope must be rejected").find("speed_plan") != std::string::npos,
            "the impossible speed plan is named");
}

// A run on a reference line that is not the corridor's centre records each sample's distance to both edges (decision
// 0020); the edges round-trip and are validated, and an older schema cannot carry them.
void corridor_edges_round_trip(const Fixture& edged) {
    const auto recording = fd::load_recording(edged.directory);
    const auto expected = off_centre_track();
    require(recording.track.left_edge_m.size() == expected.points.size() && recording.track.right_edge_m.size() == expected.points.size(),
            "every sample's edges are recorded");
    for (std::size_t i = 0; i < expected.points.size(); ++i) {
        same_value(recording.track.left_edge_m[i], 3.0, "the left edge round-trips");
        same_value(recording.track.right_edge_m[i], 7.0, "the right edge round-trips");
    }
    const fs::path behind = variant(edged, "edge-behind-reference", "track.csv", [](std::string& text) {
        edit_row(text, where({{"index", "5"}}), "left_edge_m", "-1");
    });
    require(rejects([&] { fd::load_recording(behind); }, "an edge behind the reference must be rejected").find("edges") != std::string::npos,
            "the impossible edge is explained");
    const fs::path eight = downgrade(edged, "edges-in-schema-8", 9);
    rewrite_file(eight, "metadata.json", [](std::string& text) { replace_first(text, "\"schema_version\": 9", "\"schema_version\": 8"); });
    require(rejects([&] { fd::load_recording(eight); }, "schema 8 metadata beside a track with edges must be rejected").find("track.csv") != std::string::npos,
            "the out-of-schema edges are named");
}

// The four-wheel car round-trips its parameters and wheel speeds, and re-integrating from the recorded wheel
// speeds reproduces its motion, including a locked rear axle.
void four_wheel_round_trip(const Fixture& wheels) {
    const auto recording = fd::load_recording(wheels.directory);
    const auto* car = std::get_if<fd::FourWheelCar>(&recording.metadata.vehicle_model);
    require(car, "the four-wheel car is recorded as the vehicle model");
    const auto expected = wheeled_car();
    require(car->mass_kg == expected.mass_kg && car->yaw_inertia_kgm2 == expected.yaw_inertia_kgm2 && car->cg_to_rear_m == expected.cg_to_rear_m &&
            car->track_front_m == expected.track_front_m && car->track_rear_m == expected.track_rear_m &&
            car->wheel_radius_m == expected.wheel_radius_m && car->wheel_inertia_kgm2 == expected.wheel_inertia_kgm2 &&
            car->drive_front_fraction == expected.drive_front_fraction && car->max_drive_power_w == expected.max_drive_power_w &&
            car->max_drive_torque_nm == expected.max_drive_torque_nm && car->brake_bias_front == expected.brake_bias_front &&
            car->max_brake_torque_nm == expected.max_brake_torque_nm && car->kinematic_below_mps == expected.kinematic_below_mps &&
            car->dynamic_above_mps == expected.dynamic_above_mps && car->slip_speed_floor_mps == expected.slip_speed_floor_mps &&
            car->substep_s == expected.substep_s && car->rear_tire.peak_slip_ratio_high == expected.rear_tire.peak_slip_ratio_high &&
            car->cg_height_m == expected.cg_height_m && car->roll_balance_front == expected.roll_balance_front &&
            car->drag_area_m2 == expected.drag_area_m2 && car->downforce_area_m2 == expected.downforce_area_m2 &&
            car->aero_balance_front == expected.aero_balance_front && car->viscous_coupling_nms == expected.viscous_coupling_nms,
            "every four-wheel parameter round-trips");
    require(recording.metadata.model == fd::vehicle_model_name(recording.metadata.vehicle_model), "the model name matches the recorded model");
    require(recording.samples.size() == wheels.live.size(), "every four-wheel sample is recorded");
    std::size_t rear_locked = 0;
    double largest_front_gain = 0;
    const double static_front = expected.mass_kg*fd::gravity_mps2*expected.cg_to_rear_m/recording.metadata.initial_config.wheelbase_m/2;
    for (std::size_t i = 0; i < recording.samples.size(); ++i) {
        for (std::size_t w = 0; w < 4; ++w) {
            same_value(recording.samples[i].wheel_speeds_radps[w], wheels.live[i].wheel_speeds_radps[w], "recorded wheel speed equals the live plant");
            same_value(recording.samples[i].wheel_loads_n[w], wheels.live[i].wheel_loads_n[w], "recorded wheel load equals the live plant");
        }
        if (recording.samples[i].speed_mps > 1 && recording.samples[i].wheel_speeds_radps[fd::rear_left] == 0) ++rear_locked;
        const auto& loads = recording.samples[i].wheel_loads_n;
        largest_front_gain = std::max(largest_front_gain, loads[fd::front_left]+loads[fd::front_right]-2*static_front);
    }
    for (std::size_t d = 0; d < recording.decisions.size(); ++d)
        same_decision(recording.decisions[d], wheels.decisions[d], "four-wheel decision "+std::to_string(d));
    const auto reproduced = fd::plant_reproduction_error_m(recording);
    std::cout << "  four-wheel fixture: " << recording.samples.size() << " samples, " << rear_locked
               << " with the rear left wheel locked while moving, front axle load up to " << largest_front_gain
              << " N above static, plant reproduction " << reproduced.value_or(-1) << " m\n";
    require(rear_locked > 0, "rear-biased braking locked the rear wheels in the recorded run");
    require(largest_front_gain > 300, "braking into the corner visibly loaded the front wheels");
    require(reproduced && *reproduced < 1e-6, "re-integrating from the recorded wheel speeds reproduces the motion");
    const fs::path heavier = variant(wheels, "wheels-mass", "metadata.json",
                                     [](std::string& text) { replace_first(text, "\"mass_kg\": 800", "\"mass_kg\": 1100"); });
    require(rejects([&] { fd::load_recording(heavier); }, "a changed mass must be rejected").find("weight") != std::string::npos,
            "a changed mass no longer matches the weight on the recorded wheels");
    const fs::path lazier = variant(wheels, "wheels-yaw-inertia", "metadata.json",
                                    [](std::string& text) { replace_first(text, "\"yaw_inertia_kgm2\": 1352", "\"yaw_inertia_kgm2\": 2000"); });
    const auto edited = fd::plant_reproduction_error_m(fd::load_recording(lazier));
    require(edited && *edited > 1000*(*reproduced), "a changed yaw inertia is exposed by plant reproduction");

    // A schema 6 copy cannot hold this run: its loads carry downforce, which a schema 6 car did not have.
    const fs::path six = downgrade(wheels, "wheels-schema-6", 6);
    require(rejects([&] { fd::load_recording(six); }, "a schema 6 copy of a run with downforce must be rejected").find("weight") != std::string::npos,
            "the loads that no longer match weight are named");
    // A schema 5 copy is read as schema 5 wrote the car, without load transfer or air: every wheel at its static load.
    const auto older = fd::load_recording(downgrade(wheels, "wheels-schema-5", 5));
    const auto* flat = std::get_if<fd::FourWheelCar>(&older.metadata.vehicle_model);
    require(flat && flat->cg_height_m == 0 && flat->drag_area_m2 == 0 && flat->downforce_area_m2 == 0 && flat->viscous_coupling_nms == 0 &&
            older.metadata.model == fd::vehicle_model_name(older.metadata.vehicle_model),
            "a schema 5 four-wheel car has no load transfer, aerodynamics or coupling");
    const auto still = fd::quasi_static_wheel_loads(*flat, older.metadata.initial_config, 0, 0, 0);
    require(std::all_of(older.samples.begin(), older.samples.end(), [&](const fd::RecordedSample& s) { return s.wheel_loads_n == still; }),
            "a schema 5 four-wheel run reads back with static wheel loads");
    const auto older_reproduced = fd::plant_reproduction_error_m(older);
    std::cout << "  the same run read as schema 5, without load transfer: plant reproduction " << older_reproduced.value_or(-1) << " m\n";
    require(older_reproduced && *older_reproduced > 1000*(*reproduced), "plant reproduction exposes motion that had load transfer");
}

void backward_seek_restores_revisions(const Fixture& fixture) {
    fd::Playback playback(fd::load_recording(fixture.directory));
    const double first = fixture.event_times[0], second = fixture.event_times[1];
    const auto corner = static_cast<std::size_t>(std::find_if(playback.recording().track.points.begin(), playback.recording().track.points.end(),
        [](const fd::PathPoint& p) { return p.curvature > 0.04; })-playback.recording().track.points.begin());

    playback.seek_time(playback.duration_s());
    require(playback.at_end() && playback.sample().revision == 3, "final sample carries the last revision");
    require(playback.plan().revision == 3 && playback.applied_event_count() == 2, "plan revision 3 active at the end");
    near(playback.config().grip_mu, 0.6, 1e-12, "configuration restored for revision 3");
    const double late_corner_speed = playback.plan().points[corner].speed_mps;

    playback.seek_time((first+second)/2);
    require(playback.sample().revision == 2 && playback.plan().revision == 2, "seeking back restores revision 2");
    near(playback.config().grip_mu, 0.8, 1e-12, "configuration restored for revision 2");
    require(playback.applied_event_count() == 1, "the later change is not visible before its time");
    require(playback.plan().points[corner].speed_mps > late_corner_speed, "earlier revision keeps its higher corner speed");

    playback.seek_time(first/2);
    require(playback.sample().revision == 1 && playback.plan().revision == 1, "seeking back restores the initial plan");
    near(playback.config().grip_mu, 1.0, 0, "initial configuration restored");
    require(playback.applied_event_count() == 0, "no change is in force before the first event");

    // A change accepted at tick boundary T governs motion after T; the sample at T is still the old revision.
    playback.seek_time(first);
    require(playback.sample().revision == 1, "sample at the event boundary predates the change");
    near(playback.sample().time_s, first, 1e-9, "seek snaps to the boundary sample");
    playback.seek_time(first+fixture.dt);
    require(playback.sample().revision == 2, "the following sample uses the new plan");

    playback.seek_time(-5);
    require(playback.index() == 0 && playback.time_s() == 0, "seeking before the start clamps to the first sample");
    playback.seek_time(1e9);
    require(playback.at_end() && playback.time_s() == playback.duration_s(), "seeking past the end clamps to the final sample");
    playback.seek_index(1'000'000);
    require(playback.at_end(), "index seek clamps");
    std::string message = rejects([&] { playback.seek_time(std::nan("")); }, "nonfinite seek must fail");
    require(message.find("finite") != std::string::npos, "seek failure explains itself");
}

void advance_walks_recorded_samples(const Fixture& fixture) {
    fd::Playback playback(fd::load_recording(fixture.directory));
    const double dt = fixture.dt;
    require(playback.advance(2.5*dt), "advance moves from the start");
    require(playback.index() == 2, "advance selects the last sample at or before the cursor");
    near(playback.time_s(), 2.5*dt, 1e-12, "cursor keeps fractional playback time");
    require(playback.advance(0.7*dt), "advance continues");
    require(playback.index() == 3, "fractional time accumulates across advances");
    const auto& sample = playback.sample();
    near(sample.x_m, fixture.live[3].x_m, 1e-8, "displayed pose is the recorded pose, not a re-simulation");
    near(sample.time_s, 3*dt, 1e-9, "sample time on the tick grid");
    require(playback.advance(1e9), "advance to the end reports movement");
    require(playback.at_end() && !playback.advance(dt), "advancing at the end reports no change");
    rejects([&] { playback.advance(-1); }, "negative advance must fail");
}

// A run with instruments records what they delivered, and the recording carries the means of measuring it again: the
// loader runs the same instruments from the recorded seed over the recorded states and requires the same stream.
void measured_run_round_trips(const Fixture& measured, const Fixture& plain) {
    const auto recording = fd::load_recording(measured.directory);
    require(recording.measurements_recorded() && recording.sensors_recorded(), "the run records its instruments");
    const auto& settings = *recording.metadata.sensors;
    require(settings.seed == 2026, "the seed every error came from is recorded");
    require(settings.pose.rate_hz == 20 && settings.wheel_speeds.rate_hz == 200, "with each channel's own rate");
    require(fd::SensorSuite(settings).fingerprint() == recording.metadata.sensor_fingerprint,
            "and a fingerprint of the instruments themselves");
    require(recording.measurements.size() == recording.samples.size(), "one measurement per recorded tick");
    // What was delivered lags and differs from what was true at the same tick.
    std::size_t delivered = 0;
    double worst_age = 0, worst_position = 0, worst_wheel = 0;
    for (std::size_t i = 0; i < recording.measurements.size(); ++i) {
        const auto& m = recording.measurements[i].measured;
        const auto& s = recording.samples[i];
        same_value(recording.measurements[i].time_s, s.time_s, "a measurement is dated with its tick");
        if (!m.pose.measured) continue;
        ++delivered;
        require(m.pose.sampled_at_s <= s.time_s+1e-12, "nothing is sampled from the future");
        worst_age = std::max(worst_age, s.time_s-m.pose.sampled_at_s);
        worst_position = std::max(worst_position, std::hypot(m.x_m-s.x_m, m.y_m-s.y_m));
        if (m.wheel_speeds.measured)
            for (std::size_t w = 0; w < 4; ++w)
                worst_wheel = std::max(worst_wheel, std::abs(m.wheel_speeds_radps[w]-s.wheel_speeds_radps[w]));
    }
    std::cout << "  measured run: pose delivered on " << delivered << " of " << recording.measurements.size()
              << " ticks, up to " << worst_age*1000 << " ms old and " << worst_position << " m out; wheel speeds out by up to "
              << worst_wheel << " rad/s\n";
    require(delivered > recording.measurements.size()/2, "the pose is delivered through most of the run");
    near(worst_age, 0.095, 0.011, "the pose read is a dead time plus up to a period old");
    require(worst_position > 0.5, "and is out by metres, being both late and noisy");
    require(worst_wheel > 0, "the wheel speeds carry their own error");
    // The plant is what the car drove on: the recorded truth is not the measurement.
    require(recording.samples[recording.samples.size()/2].x_m != recording.measurements[recording.samples.size()/2].measured.x_m,
            "the car drove on its true state, which the recording keeps apart from what was measured");
    // A run without instruments records none, and says so rather than inventing them.
    const auto bare = fd::load_recording(plain.directory);
    require(bare.measurements_recorded(), "a run of schema 14 or later has the file");
    require(!bare.sensors_recorded() && bare.measurements.empty(), "but no instruments and no measurements");
}

// A run with perception records the cones and every frame, and loading it perceives the run again from the seed.
void perceived_run_round_trips(const Fixture& perceived, const Fixture& plain) {
    const auto recording = fd::load_recording(perceived.directory);
    require(recording.perception_recorded(), "the run records its perception");
    require(recording.metadata.perception->seed == 31, "and the seed every draw came from");
    const auto laid = fd::make_cone_layout(fd::make_preset_track());
    require(recording.cones.size() == laid.size(), "and the course's cones");
    for (std::size_t i = 0; i < laid.size(); ++i)
        require(recording.cones[i].position.x == laid[i].position.x && recording.cones[i].position.y == laid[i].position.y &&
                recording.cones[i].colour == laid[i].colour, "each cone exactly where it was laid, at full precision");
    std::size_t detections = 0, missed = 0;
    for (const auto& f : recording.perception_frames) {
        detections += f.frame.detections.size();
        missed += f.missed.size();
        require(f.source_cone.size() == f.frame.detections.size(), "each detection names the cone it was");
        require(f.frame.delivered_at_s <= f.time_s+1e-9 && f.frame.sampled_at_s < f.frame.delivered_at_s, "delivered after it was sampled");
    }
    std::cout << "  perceived run: " << recording.cones.size() << " cones, " << recording.perception_frames.size() << " frames, "
              << detections << " detections, " << missed << " missed\n";
    require(recording.perception_frames.size() == 78, "ten frames a second from 0.3 s to eight");
    require(detections > 0 && missed > 0, "some cones detected and some missed");
    const auto bare = fd::load_recording(plain.directory);
    // From schema 17 every run records the course's cones it was judged on, perceived or not.
    require(bare.perception_files_recorded() && !bare.perception_recorded() && bare.cones.size() == laid.size() &&
            bare.perception_frames.empty(), "a run without perception has the course's cones and no frame");
}

// A run on cones records what its driver believed at every decision and every path it believed, and loads only because a
// driver run again on the recorded readings and frames believes and commands the same (decision 0031).
void cone_driven_runs_round_trip(const Fixture& on_cones, const Fixture& ideal, const Fixture& plain) {
    for (const Fixture* fixture : {&on_cones, &ideal}) {
        const auto recording = fd::load_recording(fixture->directory);
        const bool measured = fixture == &on_cones;
        require(recording.cone_driving_recorded() && recording.metadata.cone_driving->source ==
                    (measured ? fd::BeliefSource::measured : fd::BeliefSource::ideal), "the run records that it drove on cones, and from what");
        require(recording.metadata.cone_driving->settings.min_track_width_m == fd::ConePathSettings{}.min_track_width_m,
                "with the cone path settings it planned with");
        require(recording.beliefs.size() == recording.decisions.size() && recording.decisions.size() == fixture->decisions.size(),
                "one belief for every decision that governed motion");
        require(recording.believed_paths.size() > 20, "and every path believed");
        std::size_t following = 0;
        for (std::size_t d = 0; d < recording.decisions.size(); ++d) {
            const auto& decision = recording.decisions[d];
            const auto& live = fixture->decisions[d];
            require(decision.options.size() == 1 && decision.choice().action == fd::LocalAction::cone_path, "each follows the believed path");
            same_value(decision.applied.acceleration_mps2, live.trajectory().first_control.applied.acceleration_mps2, "the command recorded");
            const auto& first = decision.choice().trajectory.front();
            same_value(first.x_m, recording.beliefs[d].state.x_m, "each prediction starts where the car believed it was");
            if (recording.beliefs[d].path) ++following;
        }
        const auto& last = recording.believed_paths.back();
        require(last.pose.time_s == recording.perception_frames[last.frame].frame.sampled_at_s, "each path is placed by its frame's sampling");
        std::cout << "  " << (measured ? "measured" : "ideal") << " run on cones: " << recording.decisions.size() << " decisions, "
                  << following << " following one of " << recording.believed_paths.size() << " believed paths\n";
        require(following+80 > recording.decisions.size(), "following a believed path from the first frames on");
    }
    const auto bare = fd::load_recording(plain.directory);
    require(bare.cone_driving_files_recorded() && !bare.cone_driving_recorded() && bare.beliefs.empty() && bare.believed_paths.empty(),
            "a run on the known track has the belief files and nothing in them");
}

// A judged run records the rules, the course and every event, and loads only because judging the recorded samples again on
// the recorded course sees the same (decision 0032).
void judged_runs_round_trip(const Fixture& knocked, const Fixture& plain) {
    const auto recording = fd::load_recording(knocked.directory);
    const auto& judged = *recording.metadata.competition;
    require(judged.rules.discipline == fd::Discipline::autocross && judged.course == "cones left in the lane" && judged.gates.size() == 3,
            "the rules, the course's source and its gates are recorded");
    require(recording.cones.size() == fd::make_cone_layout(fd::make_preset_track()).size()+3, "with every cone, the three in the lane among them");
    std::size_t hits = 0;
    double penalty = 0;
    for (const auto& e : recording.timing)
        if (e.kind == fd::TimingKind::cone_hit) { ++hits; penalty += e.seconds; require(e.cone && *e.cone >= recording.cones.size()-3, "a cone in the lane"); }
    require(recording.timing.front().kind == fd::TimingKind::start && recording.timing.front().time_s == 0.005, "the clock starts as the car leaves the line");
    require(hits == 3 && penalty == 6, "the three cones are down, two seconds each");
    std::cout << "  judged run: " << recording.timing.size() << " events, " << hits << " cones down for " << penalty << " s\n";
    const auto bare = fd::load_recording(plain.directory);
    require(bare.metadata.competition->course == fd::Simulation::laid_along_the_track && bare.metadata.competition->rules.discipline == fd::Discipline::trackdrive,
            "a run given no course is judged as a trackdrive on the course laid along its track");
    std::size_t laps = 0;
    for (const auto& e : bare.timing) if (e.kind == fd::TimingKind::lap) ++laps;
    require(laps == 1 && std::none_of(bare.timing.begin(), bare.timing.end(), [](const fd::TimingEvent& e) {
        return e.kind == fd::TimingKind::cone_hit || e.kind == fd::TimingKind::off_course; }), "and its lap is clean");
}

// Every rule timing.csv and the judged course carry, each broken on its own.
void rejects_judging_violations(const Fixture& knocked, const Fixture& plain) {
    const auto expect_failure = [&](const Fixture& fixture, const std::string& name, const std::string& file,
                                    const std::function<void(std::string&)>& edit, const std::string& fragment) {
        const fs::path dir = variant(fixture, name, file, edit);
        const std::string message = rejects([&] { fd::load_recording(dir); }, name+": tampered recording must be rejected");
        require(message.find(fragment) != std::string::npos, name+": failure names the violated rule, got: "+message);
        std::cout << "  " << name << " -> " << message << '\n';
    };
    expect_failure(knocked, "timing-hit-moved", "timing.csv", [](std::string& text) {
        edit_row(text, where({{"kind", "cone hit"}}), "time_s", "5");
    }, "is not what the judge sees");
    expect_failure(knocked, "timing-hit-dropped", "timing.csv", [](std::string& text) {
        auto rows = lines(text);
        rows.pop_back();
        text = join(rows);
    }, "where the judge sees");
    expect_failure(knocked, "cone-moved-off-the-lane", "cones.csv", [](std::string& text) {
        auto rows = lines(text);
        auto fields = split(rows.back());
        fields[2] = std::to_string(std::stod(fields[2])+3);
        rows.back() = join_fields(fields);
        text = join(rows);
    }, "course_fingerprint does not identify");
    expect_failure(knocked, "cone-penalty-changed", "metadata.json", [](std::string& text) {
        replace_first(text, "\"cone_hit_s\": 2,", "\"cone_hit_s\": 3,");
    }, "is not what the judge sees");
    expect_failure(plain, "laps-required-changed", "metadata.json", [](std::string& text) {
        replace_first(text, "\"trackdrive_laps\": 10,", "\"trackdrive_laps\": 1,");
    }, "where the judge sees");
    expect_failure(plain, "gate-moved", "metadata.json", [](std::string& text) {
        const auto at = text.find("\"gates\": [[");
        require(at != std::string::npos, "fixture metadata has gates");
        text.insert(at+11, "1");
    }, "course_fingerprint does not identify");
    const fs::path missing = variant(plain, "timing-missing", "summary.json", [](std::string&) {});
    fs::remove(missing/"timing.csv");
    require(rejects([&] { fd::load_recording(missing); }, "a schema 17 run without timing.csv must be rejected").find("missing file timing.csv") != std::string::npos,
            "the missing timing file is named");
}

// Every rule the belief files carry, each broken on its own.
void rejects_cone_driving_violations(const Fixture& on_cones, const Fixture& plain) {
    const auto expect_failure = [&](const Fixture& fixture, const std::string& name, const std::string& file,
                                    const std::function<void(std::string&)>& edit, const std::string& fragment) {
        const fs::path dir = variant(fixture, name, file, edit);
        const std::string message = rejects([&] { fd::load_recording(dir); }, name+": tampered recording must be rejected");
        require(message.find(fragment) != std::string::npos, name+": failure names the violated rule, got: "+message);
        std::cout << "  " << name << " -> " << message << '\n';
    };
    const auto nudge = [](std::string& text, std::size_t row, std::size_t column, double by) {
        auto rows = lines(text);
        auto fields = split(rows[row]);
        std::ostringstream moved;
        moved << std::setprecision(17) << std::stod(fields[column])+by;
        fields[column] = moved.str();
        rows[row] = join_fields(fields);
        text = join(rows);
    };
    expect_failure(on_cones, "belief-moved", "beliefs.csv", [&](std::string& text) { nudge(text, 1, 2, 0.001); },
                   "does not start at the state the driver believed");
    expect_failure(on_cones, "belief-forgets-its-path", "beliefs.csv", [](std::string& text) {
        auto rows = lines(text);
        auto fields = split(rows.back());
        fields.back() = "none";
        rows.back() = join_fields(fields);
        text = join(rows);
    }, "forgets the believed path it followed");
    expect_failure(on_cones, "belief-dropped", "beliefs.csv", [](std::string& text) {
        auto rows = lines(text);
        rows.pop_back();
        text = join(rows);
    }, "beliefs for");
    expect_failure(on_cones, "believed-path-moved", "believed_paths.csv", [&](std::string& text) { nudge(text, 1, 3, 0.001); },
                   "is not what a driver believes from the recorded readings and frames");
    expect_failure(on_cones, "believed-path-dropped", "believed_paths.csv", [](std::string& text) {
        auto rows = lines(text);
        const auto last = split(rows.back())[0];
        while (split(rows.back())[0] == last) rows.pop_back();
        text = join(rows);
    }, "believed_paths.csv lacks");
    expect_failure(on_cones, "cone-driving-source", "metadata.json", [](std::string& text) {
        replace_first(text, "\"source\": \"measured\"", "\"source\": \"ideal\"");
    }, "metadata.cone_driving.source");
    expect_failure(on_cones, "cone-memory-changed", "metadata.json", [](std::string& text) {
        replace_first(text, "\"merge_radius_m\": 1,", "\"merge_radius_m\": 2,");
    }, "a driver");
    expect_failure(on_cones, "cone-path-settings", "metadata.json", [](std::string& text) {
        replace_first(text, "\"min_track_width_m\": 3,", "\"min_track_width_m\": 3.5,");
    }, "a driver");
    expect_failure(on_cones, "cone-driving-in-schema-15", "metadata.json", [](std::string& text) {
        as_older(text, 15);
    }, "requires schema_version 16");
    // A cone-driven run declared schema 16 keeps its cone driving but loses its judging.
    expect_failure(plain, "cone-path-on-the-known-track", "decisions.csv", [](std::string& text) {
        edit_row(text, where({{"decision", "0"}, {"option", "0"}}), "action", "cone path");
    }, "follows a believed path on a run that did not drive on cones");
}

// Every rule the perception files carry, each broken on its own.
void rejects_perception_violations(const Fixture& perceived) {
    const auto expect_failure = [&](const std::string& name, const std::string& file,
                                    const std::function<void(std::string&)>& edit, const std::string& fragment) {
        const fs::path dir = variant(perceived, name, file, edit);
        const std::string message = rejects([&] { fd::load_recording(dir); }, name+": tampered recording must be rejected");
        require(message.find(fragment) != std::string::npos, name+": failure names the violated rule, got: "+message);
        std::cout << "  " << name << " -> " << message << '\n';
    };
    const auto first_row = [](std::string& text, std::size_t column, const std::function<std::string(const std::string&)>& change) {
        auto rows = lines(text);
        auto fields = split(rows[1]);
        fields[column] = change(fields[column]);
        rows[1] = join_fields(fields);
        text = join(rows);
    };
    expect_failure("detection-moved", "detections.csv", [&](std::string& text) {
        first_row(text, 2, [](const std::string& x) { return std::to_string(std::stod(x)+0.001); });
    }, "is not what this perception detects");
    expect_failure("detection-recoloured", "detections.csv", [&](std::string& text) {
        first_row(text, 4, [](const std::string& c) { return c == "blue" ? std::string("yellow") : std::string("blue"); });
    }, "reports another colour");
    expect_failure("detection-other-cone", "detections.csv", [&](std::string& text) {
        first_row(text, 10, [](const std::string& c) { return std::to_string(std::stoul(c)+1); });
    }, "detects or misses other cones");
    expect_failure("missed-other-cone", "missed.csv", [&](std::string& text) {
        first_row(text, 1, [](const std::string& c) { return std::to_string(std::stoul(c)+1); });
    }, "detects or misses other cones");
    expect_failure("cone-moved", "cones.csv", [&](std::string& text) {
        first_row(text, 1, [](const std::string& x) { return std::to_string(std::stod(x)+0.01); });
    }, "fingerprint does not identify");
    expect_failure("perception-seed", "metadata.json", [](std::string& text) {
        replace_first(text, "\"seed\": 31", "\"seed\": 32");
    }, "fingerprint does not identify");
    expect_failure("frame-dropped", "perception_frames.csv", [](std::string& text) {
        auto rows = lines(text);
        rows.pop_back();
        text = join(rows);
    }, "holds detections no frame counts");
}

// Every rule measurements.csv carries, each broken on its own.
void rejects_measurement_violations(const Fixture& measured) {
    const auto expect_failure = [&](const std::string& name, const std::string& file,
                                    const std::function<void(std::string&)>& edit, const std::string& fragment) {
        const fs::path dir = variant(measured, name, file, edit);
        const std::string message = rejects([&] { fd::load_recording(dir); }, name+": tampered recording must be rejected");
        require(message.find(fragment) != std::string::npos, name+": failure names the violated rule, got: "+message);
        std::cout << "  " << name << " -> " << message << '\n';
    };
    // A value nudged by a millimetre is not what these instruments measure.
    expect_failure("measured-value", "measurements.csv", [](std::string& text) {
        auto rows = lines(text);
        for (auto& row : rows) {
            auto fields = split(row);
            if (fields[1] != "1") continue;
            fields[3] = std::to_string(std::stod(fields[3])+0.001);
            row = join_fields(fields);
            break;
        }
        text = join(rows);
    }, "is not what these instruments measure");
    // So is a value drawn from another seed.
    expect_failure("measured-seed", "metadata.json", [](std::string& text) {
        replace_first(text, "\"seed\": 2026", "\"seed\": 2027");
    }, "fingerprint does not identify");
    // A channel cannot deliver a sample it has not taken yet.
    expect_failure("measured-future", "measurements.csv", [](std::string& text) {
        auto rows = lines(text);
        for (auto& row : rows) {
            auto fields = split(row);
            if (fields[1] != "1") continue;
            fields[2] = std::to_string(std::stod(fields[0])+1);
            row = join_fields(fields);
            break;
        }
        text = join(rows);
    }, "later than the tick that read it");
    // There is one measurement per recorded tick.
    expect_failure("measured-count", "measurements.csv", [](std::string& text) {
        auto rows = lines(text);
        rows.pop_back();
        text = join(rows);
    }, "one measurement per recorded tick");
    // The achieved acceleration telemetry records is the speed gained over the tick before it.
    expect_failure("achieved-acceleration", "telemetry.csv", [](std::string& text) {
        auto rows = lines(text);
        auto fields = split(rows[40]);
        fields.back() = std::to_string(std::stod(fields.back())+1);
        rows[40] = join_fields(fields);
        text = join(rows);
    }, "speed this sample gained over the tick before");
}

void rejects_contract_violations(const Fixture& fixture, const Fixture& scenario, const Fixture& five_offsets, const Fixture& corner,
                                 const Fixture& dynamic, const Fixture& wheels) {
    const auto expect_failure = [&](const Fixture& source, const std::string& name, const std::string& file,
                                    const std::function<void(std::string&)>& edit, const std::string& fragment) {
        const fs::path dir = variant(source, name, file, edit);
        const std::string message = rejects([&] { fd::load_recording(dir); }, name+": tampered recording must be rejected");
        require(message.find(fragment) != std::string::npos, name+": failure names the violated rule, got: "+message);
        std::cout << "  " << name << " -> " << message << '\n';
    };
    expect_failure(fixture, "event-time", "events.csv", [](std::string& text) {
        auto rows = lines(text); auto fields = split(rows[1]); fields[0] = "4.5"; rows[1] = join_fields(fields); text = join(rows);
    }, "generated_time_s");
    expect_failure(fixture, "event-chain", "events.csv", [](std::string& text) {
        auto rows = lines(text); auto fields = split(rows[2]); fields[3] = "0.7"; rows[2] = join_fields(fields); text = join(rows);
    }, "configuration chain");
    expect_failure(fixture, "sample-revision", "telemetry.csv", [](std::string& text) {
        auto rows = lines(text);
        for (auto& row : rows) { auto fields = split(row); if (fields[16] == "2") { fields[16] = "1"; row = join_fields(fields); break; } }
        text = join(rows);
    }, "revision");
    expect_failure(fixture, "sample-order", "telemetry.csv", [](std::string& text) {
        auto rows = lines(text); std::swap(rows[5], rows[6]); text = join(rows);
    }, "increase strictly");
    expect_failure(fixture, "header", "telemetry.csv", [](std::string& text) { replace_first(text, "speed_mps", "velocity"); }, "header");
    expect_failure(fixture, "summary-count", "summary.json", [](std::string& text) { replace_first(text, "\"samples\": ", "\"samples\": 1"); }, "summary.samples");
    expect_failure(fixture, "summary-laps", "summary.json", [](std::string& text) { replace_first(text, "\"laps\": 1", "\"laps\": 2"); }, "summary.laps");
    expect_failure(fixture, "initial-grip", "metadata.json", [](std::string& text) { replace_first(text, "    \"grip_mu\": 1,", "    \"grip_mu\": 0.9,"); }, "configuration chain");
    expect_failure(fixture, "unknown-config-key", "metadata.json", [](std::string& text) { replace_first(text, "\"speed_gain\"", "\"mass_kg\"" ); }, "unknown key");
    expect_failure(fixture, "plan-geometry", "plan-rev-2.csv", [](std::string& text) {
        auto rows = lines(text); auto fields = split(rows[3]); fields[2] = "999"; rows[3] = join_fields(fields); text = join(rows);
    }, "track.csv");
    expect_failure(fixture, "plan-copy", "plan.csv", [](std::string& text) { replace_first(text, "acceleration_reachability", "acceleration_reachabilitx"); }, "identical");
    expect_failure(fixture, "nonfinite", "telemetry.csv", [](std::string& text) {
        auto rows = lines(text); auto fields = split(rows[10]); fields[4] = "nan"; rows[10] = join_fields(fields); text = join(rows);
    }, "finite");

    // Schema 3 plant columns are cross-checked against the vehicle model and the validity flags.
    expect_failure(fixture, "kinematic-sliding", "telemetry.csv", [](std::string& text) {
        edit_row(text, where({{"laps", "0"}, {"time_s", "5"}}), "lateral_velocity_mps", "0.25");
    }, "lateral_velocity_mps");
    expect_failure(fixture, "kinematic-yaw-rate", "telemetry.csv", [](std::string& text) {
        edit_row(text, where({{"laps", "0"}, {"time_s", "12"}}), "yaw_rate_radps", "2");
    }, "yaw_rate_radps");
    expect_failure(fixture, "sliding-but-valid", "telemetry.csv", [](std::string& text) {
        edit_row(text, where({{"plan_valid", "1"}, {"within_grip_envelope", "1"}}), "within_grip_envelope", "0");
    }, "within_grip_envelope");
    expect_failure(fixture, "unknown-vehicle", "metadata.json", [](std::string& text) {
        replace_first(text, "\"kinematic_bicycle\"", "\"hovercraft\"");
    }, "vehicle_model");
    expect_failure(fixture, "summary-sideslip", "summary.json", [](std::string& text) {
        replace_first(text, "\"max_rear_sideslip_rad\": 0", "\"max_rear_sideslip_rad\": 0.5");
    }, "summary.max_rear_sideslip_rad");

    // Schema 5 wheel speeds: zero for a car without rotating wheels, never negative, and the four-wheel car
    // needs schema 5 and cannot have been steered by a MAP table this build refuses to generate.
    expect_failure(fixture, "kinematic-wheel-speed", "telemetry.csv", [](std::string& text) {
        edit_row(text, where({{"laps", "0"}, {"time_s", "7"}}), "rear_left_wheel_speed_radps", "31.5");
    }, "rear_left_wheel_speed_radps");
    expect_failure(wheels, "negative-wheel-speed", "telemetry.csv", [](std::string& text) {
        edit_row(text, where({{"time_s", "5"}}), "front_right_wheel_speed_radps", "-2");
    }, "front_right_wheel_speed_radps");
    // Schema 6 wheel loads: never negative, zero for a car without rotating wheels, summing to the car's weight
    // unless a wheel has lifted, and the load transfer parameters need schema 6.
    expect_failure(fixture, "kinematic-wheel-load", "telemetry.csv", [](std::string& text) {
        edit_row(text, where({{"laps", "0"}, {"time_s", "7"}}), "front_left_wheel_load_n", "1900");
    }, "front_left_wheel_load_n");
    expect_failure(wheels, "negative-wheel-load", "telemetry.csv", [](std::string& text) {
        edit_row(text, where({{"time_s", "5"}}), "rear_right_wheel_load_n", "-10");
    }, "rear_right_wheel_load_n");
    expect_failure(wheels, "wheel-loads-not-weight", "telemetry.csv", [](std::string& text) {
        edit_row(text, where({{"time_s", "6"}}), "rear_left_wheel_load_n", "1500");
    }, "weight");
    expect_failure(wheels, "load-transfer-in-schema-5", "metadata.json", [](std::string& text) {
        as_schema(text, 5);
    }, "schema_version 6");
    // Schema 7: aerodynamics and viscous coupling need schema 7, every key is required, and loads include downforce.
    expect_failure(wheels, "aerodynamics-in-schema-6", "metadata.json", [](std::string& text) {
        as_schema(text, 6);
    }, "schema_version 7");
    expect_failure(wheels, "missing-drag-area", "metadata.json", [](std::string& text) {
        replace_first(text, ", \"drag_area_m2\": 0.7", "");
    }, "drag_area_m2");
    expect_failure(wheels, "downforce-edited", "metadata.json", [](std::string& text) {
        replace_first(text, "\"downforce_area_m2\": 1.5", "\"downforce_area_m2\": 3");
    }, "weight");
    expect_failure(wheels, "missing-roll-balance", "metadata.json", [](std::string& text) {
        replace_first(text, ", \"roll_balance_front\": 0.6", "");
    }, "roll_balance_front");
    const fs::path early_wheels = downgrade(wheels, "four-wheel-in-schema-4", 4);
    require(rejects([&] { fd::load_recording(early_wheels); }, "a four-wheel car in schema 4 metadata must be rejected").find("schema_version 5") != std::string::npos,
            "the four-wheel car's schema requirement is named");
    expect_failure(wheels, "four-wheel-unknown-key", "metadata.json", [](std::string& text) {
        replace_first(text, "\"brake_bias_front\"", "\"abs_enabled\": 1, \"brake_bias_front\"");
    }, "unknown key");

    // Schema 4 steering: the law must be known, carry a fingerprint exactly when it has a table, and
    // under Pure Pursuit the geometric steering must be the requested steering.
    expect_failure(fixture, "pursuit-geometric", "telemetry.csv", [](std::string& text) {
        edit_row(text, where({{"laps", "0"}, {"time_s", "6"}}), "geometric_steering_rad", "0.125");
    }, "geometric_steering_rad");
    expect_failure(fixture, "unknown-steering", "metadata.json", [](std::string& text) {
        replace_first(text, "\"law\": \"pure_pursuit\"", "\"law\": \"stanley\"");
    }, "steering");
    expect_failure(fixture, "pursuit-with-table", "metadata.json", [](std::string& text) {
        replace_first(text, "\"law\": \"pure_pursuit\"", "\"law\": \"pure_pursuit\", \"table_fingerprint\": \"0123456789abcdef\"");
    }, "steering");
    expect_failure(dynamic, "map-without-table", "metadata.json", [](std::string& text) {
        const auto at = text.find(", \"table_fingerprint\": ");
        require(at != std::string::npos, "the MAP fixture records a fingerprint");
        text.erase(at, text.find('}', at)-at);
    }, "table_fingerprint");
    expect_failure(dynamic, "map-malformed-table", "metadata.json", [](std::string& text) {
        const auto at = text.find("\"table_fingerprint\": \"");
        require(at != std::string::npos, "the MAP fixture records a fingerprint");
        text.replace(at+22, 16, "not-a-fingerprint");
    }, "table_fingerprint");

    // Decisions are cross-checked against the telemetry they governed and the tick schedule.
    expect_failure(fixture, "decision-off-schedule", "decisions.csv", [](std::string& text) {
        edit_row(text, where({{"decision", "1"}}), "time_s", "0.01");
    }, "control schedule");
    expect_failure(fixture, "decision-dropped", "decisions.csv", [&](std::string& text) {
        const std::string last = std::to_string(fixture.decisions.size()-1);
        drop_rows(text, where({{"decision", last}}));
    }, "control schedule");
    expect_failure(fixture, "decision-command", "decisions.csv", [](std::string& text) {
        edit_row(text, where({{"decision", "3"}}), "requested_acceleration_mps2", "1.5");
    }, "telemetry");
    expect_failure(fixture, "trajectory-start", "trajectories.csv", [](std::string& text) {
        edit_row(text, where({{"decision", "2"}, {"option", "0"}, {"point", "0"}}), "x_m", "123.25");
    }, "recorded state");
    expect_failure(fixture, "trajectory-horizon", "trajectories.csv", [](std::string& text) {
        // A three-second horizon at 20 ms control holds is 150 holds: points 0..150.
        drop_rows(text, where({{"decision", "0"}, {"option", "0"}, {"point", "150"}}));
    }, "horizon");
    expect_failure(scenario, "two-selected", "decisions.csv", [](std::string& text) {
        edit_row(text, where({{"selected", "0"}}), "selected", "1");
    }, "exactly one");
    expect_failure(scenario, "unknown-blocker", "decisions.csv", [](std::string& text) {
        // Renamed consistently on every row, so only the scenario rule can catch it. Reasons are
        // sentences, so a comma-delimited match touches the identifier column alone.
        for (auto at = text.find(",cone-cluster,"); at != std::string::npos; at = text.find(",cone-cluster,", at))
            text.replace(at, 14, ",ghost,");
    }, "not a stated scenario obstruction");
    expect_failure(five_offsets, "orphan-option", "trajectories.csv", [](std::string& text) {
        drop_rows(text, where({{"option", "5"}}));
    }, "trajectories.csv");

    // Schema 10 actions and lattice paths: the action is known and one the recorded planner takes, a pass has a path and
    // names its blockage, only the held choice brakes, and each path is counted, starts at the car and runs forward.
    expect_failure(scenario, "unknown-action", "decisions.csv", [](std::string& text) {
        edit_row(text, where({{"action", "straight"}}), "action", "reverse");
    }, "not a known action");
    expect_failure(scenario, "offset-under-lattice", "decisions.csv", [](std::string& text) {
        edit_row(text, where({{"action", "straight"}}), "action", "offset");
    }, "lattice planner");
    expect_failure(scenario, "brake-with-path", "decisions.csv", [](std::string& text) {
        edit_row(text, where({{"action", "pass left"}}), "action", "brake");
    }, "lattice path its action does not follow");
    expect_failure(scenario, "hold-without-brake", "decisions.csv", [](std::string& text) {
        auto rows = lines(text);
        const auto header = split(rows[0]);
        const auto column = [&](const std::string& name) { return static_cast<std::size_t>(std::find(header.begin(), header.end(), name)-header.begin()); };
        for (std::size_t i = 1; i < rows.size(); ++i) {
            auto fields = split(rows[i]);
            if (fields[column("action")] != "brake") continue;
            fields[column("action")] = "straight";
            rows[i] = join_fields(fields);
            text = join(rows);
            return;
        }
        throw std::runtime_error("the scenario has no braking option");
    }, "without braking");
    expect_failure(scenario, "return-names-no-blockage", "decisions.csv", [](std::string& text) {
        auto rows = lines(text);
        const auto header = split(rows[0]);
        const auto column = [&](const std::string& name) { return static_cast<std::size_t>(std::find(header.begin(), header.end(), name)-header.begin()); };
        for (std::size_t i = 1; i < rows.size(); ++i) {
            auto fields = split(rows[i]);
            if (fields[column("action")] != "straight" || fields[column("path_points")] == "0" || fields[column("selected")] != "1") continue;
            fields[column("action")] = "pass left";
            rows[i] = join_fields(fields);
            text = join(rows);
            return;
        }
        throw std::runtime_error("the scenario has no return along the lattice");
    }, "naming a blockage");
    expect_failure(scenario, "path-cost-without-path", "decisions.csv", [](std::string& text) {
        edit_row(text, where({{"action", "brake"}}), "cost_goal", "12.5");
    }, "does not have");
    expect_failure(scenario, "path-start", "paths.csv", [](std::string& text) {
        edit_row(text, where({{"point", "0"}}), "offset_m", "3.25");
    }, "recorded state");
    expect_failure(scenario, "path-point-dropped", "paths.csv", [](std::string& text) {
        drop_rows(text, where({{"decision", split(lines(text)[1])[0]}, {"option", split(lines(text)[1])[1]}, {"point", "1"}}));
    }, "paths.csv");
    expect_failure(scenario, "path-stations", "paths.csv", [](std::string& text) {
        edit_row(text, where({{"point", "2"}}), "s_m", "0");
    }, "increase");
    expect_failure(scenario, "path-extra-point", "paths.csv", [](std::string& text) {
        text += "999999,0,0,1,0\n";
    }, "does not count");

    // Schema 11 speed profiles: each starts at the recorded state and speed, the profiles of one decision share a horizon,
    // each estimated time is its profile's, every lattice path has one, and the car took the fastest clear action.
    expect_failure(scenario, "profile-start-speed", "profiles.csv", [](std::string& text) {
        edit_row(text, where({{"point", "0"}}), "speed_mps", "3.25");
    }, "recorded state");
    expect_failure(scenario, "estimated-time", "decisions.csv", [](std::string& text) {
        auto rows = lines(text);
        const auto header = split(rows[0]);
        const auto column = static_cast<std::size_t>(std::find(header.begin(), header.end(), "estimated_time_s")-header.begin());
        for (std::size_t i = 1; i < rows.size(); ++i) {
            auto fields = split(rows[i]);
            if (fields[column] == "none") continue;
            fields[column] = std::to_string(std::stod(fields[column])+0.5);
            rows[i] = join_fields(fields);
            text = join(rows);
            return;
        }
        throw std::runtime_error("the scenario has no estimated time");
    }, "estimated_time_s disagrees");
    expect_failure(scenario, "path-without-profile", "decisions.csv", [](std::string& text) {
        auto rows = lines(text);
        const auto header = split(rows[0]);
        const auto column = [&](const std::string& name) { return static_cast<std::size_t>(std::find(header.begin(), header.end(), name)-header.begin()); };
        for (std::size_t i = 1; i < rows.size(); ++i) {
            auto fields = split(rows[i]);
            if (fields[column("action")] != "pass left") continue;
            fields[column("profile_points")] = "0";
            fields[column("estimated_time_s")] = "none";
            rows[i] = join_fields(fields);
            text = join(rows);
            return;
        }
        throw std::runtime_error("the scenario has no pass");
    }, "without its speed profile");
    // Changes one decision's decisions.csv rows and its profiles.csv rows together, found by a predicate on its rows.
    const auto tamper_decision = [&](const Fixture& source, const std::string& name, const std::function<bool(const std::vector<std::vector<std::string>>&, const std::vector<std::string>&)>& choose,
                                     const std::function<void(std::string&, const std::string&)>& decisions_edit,
                                     const std::function<void(std::string&, const std::string&)>& profiles_edit, const std::string& fragment) {
        const std::string decisions_text = read_text(source.directory/"decisions.csv");
        const auto rows = lines(decisions_text);
        const auto header = split(rows[0]);
        std::string chosen;
        std::vector<std::vector<std::string>> group;
        for (std::size_t i = 1; i <= rows.size(); ++i) {
            const auto fields = i < rows.size() ? split(rows[i]) : std::vector<std::string>{};
            if (!group.empty() && (fields.empty() || fields[0] != group.front()[0])) {
                if (choose(group, header)) { chosen = group.front()[0]; break; }
                group.clear();
            }
            if (!fields.empty()) group.push_back(fields);
        }
        require(!chosen.empty(), name+": the scenario has a decision to tamper with");
        const fs::path dir = variant(source, name, "decisions.csv", [&](std::string& text) { decisions_edit(text, chosen); });
        rewrite_file(dir, "profiles.csv", [&](std::string& text) { profiles_edit(text, chosen); });
        const std::string message = rejects([&] { fd::load_recording(dir); }, name+": tampered recording must be rejected");
        require(message.find(fragment) != std::string::npos, name+": failure names the violated rule, got: "+message);
        std::cout << "  " << name << " -> " << message << '\n';
    };
    const auto column_of = [](const std::vector<std::string>& header, const std::string& name) {
        return static_cast<std::size_t>(std::find(header.begin(), header.end(), name)-header.begin());
    };
    tamper_decision(scenario, "profile-horizon", [&](const std::vector<std::vector<std::string>>& group, const std::vector<std::string>& header) {
        return std::count_if(group.begin(), group.end(), [&](const std::vector<std::string>& f) { return f[column_of(header, "profile_points")] != "0"; }) >= 2;
    }, [](std::string&, const std::string&) {}, [&](std::string& text, const std::string& decision) {
        // The last point of the decision's first profile.
        auto rows = lines(text);
        std::size_t last = 0;
        std::string option;
        for (std::size_t i = 1; i < rows.size(); ++i) {
            const auto fields = split(rows[i]);
            if (fields[0] != decision) continue;
            if (option.empty()) option = fields[1];
            if (fields[1] != option) break;
            last = i;
        }
        require(last != 0, "the decision has a profile");
        auto fields = split(rows[last]);
        fields[3] = std::to_string(std::stod(fields[3])+1);
        rows[last] = join_fields(fields);
        text = join(rows);
    }, "same horizon");
    tamper_decision(corner, "slower-choice", [&](const std::vector<std::vector<std::string>>& group, const std::vector<std::string>& header) {
        // Two clear timed actions further apart than the switching margin, so taking the slower breaks the rule either way.
        std::vector<double> times;
        for (const auto& f : group)
            if (f[column_of(header, "clear")] == "1" && f[column_of(header, "estimated_time_s")] != "none") times.push_back(std::stod(f[column_of(header, "estimated_time_s")]));
        return times.size() >= 2 && *std::max_element(times.begin(), times.end())-*std::min_element(times.begin(), times.end()) > fd::lattice_switch_margin_s+0.01;
    }, [&](std::string& text, const std::string& decision) {
        auto rows = lines(text);
        const auto header = split(rows[0]);
        std::size_t selected = 0, slowest = 0;
        double worst = -1;
        for (std::size_t i = 1; i < rows.size(); ++i) {
            const auto fields = split(rows[i]);
            if (fields[0] != decision) continue;
            if (fields[column_of(header, "selected")] == "1") selected = i;
            if (fields[column_of(header, "clear")] == "1" && fields[column_of(header, "estimated_time_s")] != "none" &&
                std::stod(fields[column_of(header, "estimated_time_s")]) > worst) { worst = std::stod(fields[column_of(header, "estimated_time_s")]); slowest = i; }
        }
        require(selected != slowest, "the fastest clear action was taken");
        for (const auto i : {selected, slowest}) {
            auto fields = split(rows[i]);
            fields[column_of(header, "selected")] = i == selected ? "0" : "1";
            rows[i] = join_fields(fields);
        }
        text = join(rows);
    }, [](std::string&, const std::string&) {}, "did not take the fastest clear action");
    tamper_decision(scenario, "blocked-choice", [&](const std::vector<std::vector<std::string>>& group, const std::vector<std::string>& header) {
        // A decision that is not holding and offered both a clear option and a blocked one.
        const auto clear_is = [&](const std::string& value) {
            return [&, value](const std::vector<std::string>& f) { return f[column_of(header, "clear")] == value; };
        };
        return group.front()[column_of(header, "holding")] == "0" &&
               std::any_of(group.begin(), group.end(), clear_is("1")) && std::any_of(group.begin(), group.end(), clear_is("0"));
    }, [&](std::string& text, const std::string& decision) {
        auto rows = lines(text);
        const auto header = split(rows[0]);
        std::size_t selected = 0, blocked = 0;
        for (std::size_t i = 1; i < rows.size(); ++i) {
            const auto fields = split(rows[i]);
            if (fields[0] != decision) continue;
            if (fields[column_of(header, "selected")] == "1") selected = i;
            if (!blocked && fields[column_of(header, "clear")] == "0") blocked = i;
        }
        require(selected && blocked && selected != blocked, "the decision took a clear line and offered a blocked one");
        for (const auto i : {selected, blocked}) {
            auto fields = split(rows[i]);
            fields[column_of(header, "selected")] = i == selected ? "0" : "1";
            rows[i] = join_fields(fields);
        }
        text = join(rows);
    }, [](std::string&, const std::string&) {}, "not clear while a clear one was offered");
    tamper_decision(scenario, "drove-on-past-the-blockage", [&](const std::vector<std::vector<std::string>>& group, const std::vector<std::string>& header) {
        // A decision naming the blockage in its way, which must hold once nothing is clear.
        return group.front()[column_of(header, "holding")] == "0" && !group.front()[column_of(header, "blocking_identifier")].empty();
    }, [&](std::string& text, const std::string& decision) {
        auto rows = lines(text);
        const auto header = split(rows[0]);
        for (std::size_t i = 1; i < rows.size(); ++i) {
            auto fields = split(rows[i]);
            if (fields[0] != decision) continue;
            fields[column_of(header, "clear")] = "0";
            rows[i] = join_fields(fields);
        }
        text = join(rows);
    }, [](std::string&, const std::string&) {}, "drove on with no clear line past");

    const fs::path missing_plan = variant(fixture, "missing-plan", "events.csv", [](std::string&) {});
    fs::remove(missing_plan/"plan-rev-2.csv");
    require(rejects([&] { fd::load_recording(missing_plan); }, "missing plan revision must be rejected").find("plan-rev-2.csv") != std::string::npos, "missing plan is named");
    const fs::path extra_plan = variant(fixture, "extra-plan", "events.csv", [](std::string&) {});
    fs::copy_file(extra_plan/"plan-rev-3.csv", extra_plan/"plan-rev-4.csv");
    require(rejects([&] { fd::load_recording(extra_plan); }, "orphan plan revision must be rejected").find("without a recorded parameter event") != std::string::npos, "orphan plan is explained");
    rejects([&] { fd::load_recording(fixture.directory/"does-not-exist"); }, "missing directory must fail");

    // Speeds alone cannot be cross-checked without the planner: loading succeeds, reproduction exposes the edit.
    const auto edit_speed = [](std::string& text) {
        auto rows = lines(text); auto fields = split(rows[20]); fields[5] = "3"; rows[20] = join_fields(fields); text = join(rows);
    };
    const fs::path edited_speed = variant(fixture, "plan-speed", "plan-rev-1.csv", edit_speed);
    std::string copy = read_text(edited_speed/"plan.csv");
    edit_speed(copy);
    fs::remove(edited_speed/"plan.csv");
    write_text(edited_speed/"plan.csv", copy);
    const auto recording = fd::load_recording(edited_speed);
    require(fd::plan_reproduction_error_mps(recording) > 1, "planner reproduction detects an edited plan speed");
}

// A run of a physics profile (decision 0033) records the profile's id, and loads only while the recorded car and
// configuration are exactly that profile's.
void profiled_run_round_trips(const fs::path& root) {
    const auto& profile = fd::vehicle_profile("racing-kart");
    fd::Simulation simulation(fd::make_preset_track(), fd::configured_for(profile, fd::Config{}), profile.car);
    fd::RunRequest request{0, 2.0, {}, profile.id};
    const fs::path directory = root/"profiled";
    fd::RecordingWriter writer(directory, simulation, request, {"test-fingerprint", "test-compiler", "0"});
    while (simulation.state().time_s+simulation.config().fixed_dt_s <= request.duration_cap_s+1e-9) {
        simulation.step();
        writer.record(simulation);
    }
    writer.finish(simulation);
    const auto recording = fd::load_recording(directory);
    require(recording.metadata.vehicle_profile == "racing-kart" && std::get<fd::FourWheelCar>(recording.metadata.vehicle_model).mass_kg == 165,
            "the run names its profile and carries its car");
    Fixture as_fixture;
    as_fixture.directory = directory;
    const auto expect_failure = [&](const std::string& name, const std::function<void(std::string&)>& edit, const std::string& fragment) {
        const fs::path dir = variant(as_fixture, name, "metadata.json", edit);
        const std::string message = rejects([&] { fd::load_recording(dir); }, name+": tampered recording must be rejected");
        require(message.find(fragment) != std::string::npos, name+": failure names the violated rule, got: "+message);
        std::cout << "  " << name << " -> " << message << '\n';
    };
    expect_failure("profile-car-changed", [](std::string& text) { replace_first(text, "\"mass_kg\": 165,", "\"mass_kg\": 170,"); },
                   "is not that profile's");
    expect_failure("profile-unknown", [](std::string& text) { replace_first(text, "\"vehicle_profile\": \"racing-kart\"", "\"vehicle_profile\": \"road-car\""); },
                   "Unknown vehicle profile");
    // Schema 18 (decision 0038): the Göteborg rental kart's drive controller top speed is recorded, read back, checked against
    // its profile, and refused in metadata that declares an older schema; a car without one records none.
    const auto& rental = fd::vehicle_profile("gokartcentralen-rsx2");
    fd::Simulation kart(fd::make_preset_track(), fd::configured_for(rental, fd::Config{}), rental.car);
    fd::RunRequest kart_request{0, 2.0, {}, rental.id};
    const fs::path kart_directory = root/"profiled-rental";
    fd::RecordingWriter kart_writer(kart_directory, kart, kart_request, {"test-fingerprint", "test-compiler", "0"});
    while (kart.state().time_s+kart.config().fixed_dt_s <= kart_request.duration_cap_s+1e-9) {
        kart.step();
        kart_writer.record(kart);
    }
    kart_writer.finish(kart);
    const auto rental_recording = fd::load_recording(kart_directory);
    require(rental_recording.metadata.schema_version == 18 &&
            std::abs(std::get<fd::FourWheelCar>(rental_recording.metadata.vehicle_model).max_drive_speed_mps-60/3.6) < 1e-9,
            "the rental kart's top speed round-trips");
    require(read_text(directory/"metadata.json").find("max_drive_speed_mps") == std::string::npos, "a car without one records none");
    Fixture rental_fixture;
    rental_fixture.directory = kart_directory;
    const auto expect_rental_failure = [&](const std::string& name, const std::function<void(std::string&)>& edit, const std::string& fragment) {
        const fs::path dir = variant(rental_fixture, name, "metadata.json", edit);
        const std::string message = rejects([&] { fd::load_recording(dir); }, name+": tampered recording must be rejected");
        require(message.find(fragment) != std::string::npos, name+": failure names the violated rule, got: "+message);
        std::cout << "  " << name << " -> " << message << '\n';
    };
    expect_rental_failure("rental-speed-changed", [](std::string& text) {
        const auto at = text.find("\"max_drive_speed_mps\": ");
        require(at != std::string::npos, "the rental kart's metadata carries its top speed");
        text.replace(at, text.find(',', at)-at, "\"max_drive_speed_mps\": 20");
    }, "is not that profile's");
    expect_rental_failure("rental-speed-in-schema-17", [](std::string& text) {
        replace_first(text, "\"schema_version\": 18", "\"schema_version\": 17");
    }, "requires schema_version 18");
}

void writer_guards(const Fixture& fixture) {
    fd::Simulation fresh;
    rejects([&] { fd::RecordingWriter writer(fixture.directory, fresh, {}, {}); }, "nonempty directory must be rejected");
    fd::Simulation stepped;
    stepped.step();
    rejects([&] { fd::RecordingWriter writer(fixture.directory.parent_path()/"stepped", stepped, {}, {}); }, "a stepped simulation must be rejected");
    require(!fs::exists(fixture.directory.parent_path()/"stepped"), "rejected writer creates nothing");
    fd::RunRequest bad;
    bad.requested_laps = 101;
    rejects([&] { fd::RecordingWriter writer(fixture.directory.parent_path()/"bad-request", fresh, bad, {}); }, "invalid run request must be rejected");
    fd::Simulation reused;
    fd::RecordingWriter writer(fixture.directory.parent_path()/"short", reused, {0, 0.02, {}}, {});
    for (int i = 0; i < 4; ++i) { reused.step(); writer.record(reused); }
    const auto summary = writer.finish(reused);
    require(summary.samples == 5 && summary.completed, "short unlimited run completes");
    rejects([&] { writer.record(reused); }, "recording after finish must fail");
    const auto loaded = fd::load_recording(fixture.directory.parent_path()/"short");
    require(loaded.samples.size() == 5 && loaded.plans.size() == 1 && loaded.events.empty(), "short run loads");
    require(loaded.decisions.size() == 1, "four steps within one control period governed by one decision");

    // The scenario is metadata: changing it mid-run would leave decisions no file explains.
    fd::Simulation changed;
    fd::RecordingWriter scenario_writer(fixture.directory.parent_path()/"changed-scenario", changed, {0, 1, {}}, {});
    changed.step();
    scenario_writer.record(changed);
    changed.set_obstructions({{100, 110, -2, 2, "late-cone"}});
    changed.step();
    const std::string message = rejects([&] { scenario_writer.record(changed); }, "a scenario changed after recording started must be rejected");
    require(message.find("scenario") != std::string::npos, "the scenario change is explained, got: "+message);
    // A skipped step would lose the decision that governed it.
    fd::Simulation skipping;
    fd::RecordingWriter skip_writer(fixture.directory.parent_path()/"skipped-step", skipping, {0, 1, {}}, {});
    for (int i = 0; i < 5; ++i) skipping.step();
    rejects([&] { skip_writer.record(skipping); }, "a skipped decision must be rejected");
}
}  // namespace

int main() {
    TemporaryDirectory temporary;
    Fixture fixture, scenario, five_offsets, corner, dynamic, wheels, envelope, edged, lost, controlled, kept, measured, perceived;
    Fixture on_cones, on_cones_ideal, knocked;
    try {
        const auto began = std::chrono::steady_clock::now();
        fixture = record_fixture(temporary.path);
        scenario = record_scenario(temporary.path);
        five_offsets = record_scenario(temporary.path, fd::LocalPlannerMode::five_offsets);
        corner = record_corner(temporary.path);
        lost = record_lost_grip(temporary.path);
        controlled = record_controlled(temporary.path);
        kept = record_kept(temporary.path);
        dynamic = record_dynamic(temporary.path);
        wheels = record_wheels(temporary.path);
        envelope = record_envelope(temporary.path);
        edged = record_edged(temporary.path);
        measured = record_measured(temporary.path);
        perceived = record_perceived(temporary.path);
        on_cones = record_on_cones(temporary.path, true);
        on_cones_ideal = record_on_cones(temporary.path, false);
        knocked = record_knocked(temporary.path);
        const auto recorded = std::chrono::steady_clock::now();
        const auto recording = fd::load_recording(fixture.directory);
        const auto loaded = std::chrono::steady_clock::now();
        std::size_t points = 0;
        for (const auto& d : recording.decisions) for (const auto& o : d.options) points += o.trajectory.size();
        std::cout << "  one-lap fixture: " << recording.samples.size() << " samples, " << recording.decisions.size()
                  << " decisions, " << points << " trajectory points, trajectories.csv "
                  << fs::file_size(fixture.directory/"trajectories.csv")/1e6 << " MB; recording both fixtures "
                  << std::chrono::duration<double>(recorded-began).count() << " s, loading the lap "
                  << std::chrono::duration<double>(loaded-recorded).count() << " s (measurement only)\n";
    } catch (const std::exception& error) { std::cerr << "FAIL fixture: " << error.what() << '\n'; return 1; }
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"round_trip", [&] { round_trip(fixture); }},
        {"decision_in_force_follows_the_tick_rule", [&] { decision_in_force_follows_the_tick_rule(fixture); }},
        {"scenario_decisions_round_trip", [&] { scenario_decisions_round_trip(scenario); }},
        {"five_offset_scenario_round_trip", [&] { five_offset_scenario_round_trip(five_offsets); }},
        {"corner_decisions_round_trip", [&] { corner_decisions_round_trip(corner); }},
        {"a_run_with_no_clear_way_home_round_trips", [&] { a_run_with_no_clear_way_home_round_trips(lost); }},
        {"controller_decisions_round_trip", [&] { controller_decisions_round_trip(controlled, scenario); }},
        {"a_plan_kept_to_has_no_error", [&] { a_plan_kept_to_has_no_error(kept); }},
        {"older_schemas_still_load", [&] { older_schemas_still_load(fixture); }},
        {"measured_run_round_trips", [&] { measured_run_round_trips(measured, fixture); }},
        {"rejects_measurement_violations", [&] { rejects_measurement_violations(measured); }},
        {"perceived_run_round_trips", [&] { perceived_run_round_trips(perceived, fixture); }},
        {"rejects_perception_violations", [&] { rejects_perception_violations(perceived); }},
        {"cone_driven_runs_round_trip", [&] { cone_driven_runs_round_trip(on_cones, on_cones_ideal, fixture); }},
        {"rejects_cone_driving_violations", [&] { rejects_cone_driving_violations(on_cones, fixture); }},
        {"judged_runs_round_trip", [&] { judged_runs_round_trip(knocked, fixture); }},
        {"rejects_judging_violations", [&] { rejects_judging_violations(knocked, fixture); }},
        {"dynamic_plant_round_trip", [&] { dynamic_plant_round_trip(dynamic); }},
        {"backward_seek_restores_revisions", [&] { backward_seek_restores_revisions(fixture); }},
        {"advance_walks_recorded_samples", [&] { advance_walks_recorded_samples(fixture); }},
        {"map_steering_round_trip", [&] { map_steering_round_trip(dynamic); }},
        {"four_wheel_round_trip", [&] { four_wheel_round_trip(wheels); }},
        {"envelope_plan_round_trip", [&] { envelope_plan_round_trip(envelope); }},
        {"corridor_edges_round_trip", [&] { corridor_edges_round_trip(edged); }},
        {"rejects_contract_violations", [&] { rejects_contract_violations(fixture, scenario, five_offsets, corner, dynamic, wheels); }},
        {"profiled_run_round_trips", [&] { profiled_run_round_trips(temporary.path); }},
        {"writer_guards", [&] { writer_guards(fixture); }}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " checks passed\n";
    return failures ? 1 : 0;
}

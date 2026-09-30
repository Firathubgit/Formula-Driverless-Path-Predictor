#include "fd/mpcc.hpp"
#include "fd/simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
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

const fd::Track preset = fd::make_preset_track();
const fd::Config config;

// A car rolling along the reference at a station and speed, steered as the reference bends there.
fd::PlantState on_track(const fd::VehicleModel& model, double station, double speed) {
    const auto a = fd::sample(preset, station), b = fd::sample(preset, station+0.5);
    fd::State pose{a.x_m, a.y_m, std::atan2(b.y_m-a.y_m, b.x_m-a.x_m), speed, 0, 0};
    return fd::plant_state_from(pose, model, config);
}

struct Loop {
    fd::VehicleModel model;
    std::vector<fd::PlanPoint> plan = fd::make_speed_plan(preset, config);
    fd::Mpcc mpcc;
};

// One decision as the simulation will make it: the planner chooses, MPCC plans the chosen action.
fd::ControllerPlan decide(fd::Mpcc& mpcc, const fd::VehicleModel& model, const std::vector<fd::PlanPoint>& plan,
                          const fd::PlantState& state, fd::Command held, const std::vector<fd::Obstruction>& obstructions = {},
                          fd::LocalDecision* decision_out = nullptr, const fd::Lattice* lattice = nullptr,
                          const fd::LocalOption* previous = nullptr) {
    const auto decision = fd::choose_lattice_action(preset, plan, state, config, model, lattice, obstructions, 0, nullptr, nullptr, previous);
    const fd::ControlRequest request{preset, plan, config, model, state, decision, obstructions, held, 0};
    auto result = mpcc.plan(request);
    if (decision_out) *decision_out = decision;
    return result;
}

// On the straight at 10 m/s the plan is solved, spans the prediction horizon on the fixed-tick grid from the actual
// state, speeds up toward the top speed without passing it, and stays inside the corridor and each tire's peak.
void on_the_straight_it_speeds_up_within_its_limits() {
    const fd::VehicleModel model = fd::DynamicSingleTrack{};
    const auto plan = fd::make_speed_plan(preset, config);
    fd::Mpcc mpcc;
    const auto state = on_track(model, 20, 10);
    const auto result = decide(mpcc, model, plan, state, {0, 0});
    std::cout << "  straight at 10 m/s: " << fd::controller_status_name(result.status) << " in " << result.iterations
              << " steps, " << result.solve_time_s*1000 << " ms; command " << result.trajectory.first_control.requested.acceleration_mps2
              << " m/s^2, " << result.trajectory.first_control.requested.steering_rad << " rad; speed at 3 s "
              << result.trajectory.points.back().state.speed_mps << " m/s\n";
    require(result.status != fd::ControllerStatus::not_solved && result.iterations == 2, "both steps are solved");
    require(result.restarted, "the first plan starts from the policy's prediction");
    const auto& points = result.trajectory.points;
    require(points.size() == 61, "sixty stages and the start");
    near(points.front().state.time_s, 0, 1e-12, "it starts now");
    near(points.back().state.time_s, fd::prediction_horizon_s, 1e-9, "and ends at the prediction horizon");
    near(points.front().state.x_m, state.pose.x_m, 1e-9, "from the actual rear axle");
    near(points.front().state.speed_mps, 10, 1e-12, "at the actual speed");
    require(result.trajectory.first_control.requested.acceleration_mps2 > 0, "it speeds up");
    require(points.back().state.speed_mps > 15 && points.back().state.speed_mps <= config.max_speed_mps+0.5,
            "toward the top speed, not past it");
    require(result.trajectory.within_model_envelope, "inside the corridor and each tire's peak: "+result.trajectory.validity_reason);
    // The plan states its command at each point: from the command held and the actual steering, its steering the plan's own;
    // and its first command is where those commands are at the end of the control interval (decision 0025).
    const auto& commands = result.trajectory.commands;
    require(commands.size() == points.size(), "a command at every point");
    near(commands.front().acceleration_mps2, 0, 1e-12, "starting from the acceleration held");
    near(commands.front().steering_rad, state.pose.steering_rad, 1e-12, "and the actual steering");
    for (std::size_t k = 0; k < points.size(); ++k) near(commands[k].steering_rad, points[k].state.steering_rad, 1e-12, "the plan steers as it commands");
    const auto first = fd::planned_command(fd::point_times(result.trajectory), commands, config.control_dt_s);
    near(result.trajectory.first_control.applied.acceleration_mps2, first.acceleration_mps2, 0, "its first command is its plan's acceleration");
    near(result.trajectory.first_control.applied.steering_rad, first.steering_rad, 0, "and steering at the end of the control interval");
    require(first.acceleration_mps2 > commands.front().acceleration_mps2, "a fifth of the way to its second point");
}

// A car already faster than the top speed, and still accelerating, cannot shed the excess within one stage: its
// acceleration changes at a bounded rate. The top speed is therefore softened like MPCC's other constraints, so the plan is
// still solved, and it brings the car back down to the top speed.
void a_car_above_the_top_speed_is_brought_back_to_it() {
    const fd::VehicleModel model = fd::DynamicSingleTrack{};
    const auto plan = fd::make_speed_plan(preset, config);
    fd::Mpcc mpcc;
    const auto state = on_track(model, 20, config.max_speed_mps+2);
    const auto result = decide(mpcc, model, plan, state, {3.0, 0});
    const auto& points = result.trajectory.points;
    std::cout << "  at " << config.max_speed_mps+2 << " m/s and accelerating: " << fd::controller_status_name(result.status) << "; command "
              << result.trajectory.first_control.requested.acceleration_mps2 << " m/s^2; speed at 1 s " << points[20].state.speed_mps
              << " m/s, at 3 s " << points.back().state.speed_mps << " m/s\n";
    require(result.status != fd::ControllerStatus::not_solved, "the plan is solved");
    const double hold = std::llround(config.control_dt_s/config.fixed_dt_s)*config.fixed_dt_s;
    const fd::MpccOptions options;
    require(result.trajectory.first_control.requested.acceleration_mps2 <= 3.0-options.step_share*options.max_jerk_mps3*hold+1e-6,
            "it takes acceleration away at the step share of the fastest rate allowed or faster");
    require(points[20].state.speed_mps <= config.max_speed_mps, "it is back at the top speed within a second");
    require(points.back().state.speed_mps <= config.max_speed_mps+0.5, "and stays near it; the next decisions refine the rest");
}

// Each decision starts from the last plan moved on by the time since; reset() forgets it, so the next starts again from
// the policy's prediction, as a run's first decision does.
void each_decision_starts_from_the_last_plan() {
    const fd::VehicleModel model = fd::DynamicSingleTrack{};
    const auto plan = fd::make_speed_plan(preset, config);
    fd::Mpcc mpcc;
    const auto first = decide(mpcc, model, plan, on_track(model, 20, 10), {0, 0});
    auto later = on_track(model, 20.2, 10.02);
    later.pose.time_s = config.control_dt_s;
    const auto second = decide(mpcc, model, plan, later, first.trajectory.first_control.applied);
    require(first.restarted && !second.restarted, "the second decision continues from the first plan");
    require(second.status != fd::ControllerStatus::not_solved, "and is solved");
    mpcc.reset();
    require(decide(mpcc, model, plan, later, first.trajectory.first_control.applied).restarted, "after reset() it starts from the policy again");
    const auto settings = mpcc.settings();
    require(settings.mode == fd::ControllerMode::mpcc && settings.values.size() > 20, "its settings are reported for the recording");
}

// Options outside their ranges are refused, among them a horizon other than the prediction horizon, which every recorded
// prediction spans.
void options_outside_their_ranges_are_refused() {
    const auto refused = [](const std::function<void(fd::MpccOptions&)>& change) {
        fd::MpccOptions options;
        change(options);
        try { fd::Mpcc mpcc(options); } catch (const std::invalid_argument&) { return true; }
        return false;
    };
    require(refused([](fd::MpccOptions& o) { o.stages = 30; }), "30 stages of 0.05 s are not the three-second horizon");
    require(!refused([](fd::MpccOptions& o) { o.stages = 30; o.stage_s = 0.1; }), "30 stages of 0.1 s are");
    require(refused([](fd::MpccOptions& o) { o.lag_weight = -1; }), "a negative weight");
    require(refused([](fd::MpccOptions& o) { o.slip_angle_share = 1.2; }), "a slip angle past the peak");
    require(refused([](fd::MpccOptions& o) { o.iterations = 0; }), "no iterations");
    require(refused([](fd::MpccOptions& o) { o.solver_tolerance = 0; }), "no tolerance");
    fd::MpccOptions odd;
    odd.stages = 48;
    odd.stage_s = 0.0625;
    fd::Mpcc off_grid(odd);
    const fd::VehicleModel model = fd::DynamicSingleTrack{};
    const auto plan = fd::make_speed_plan(preset, config);
    bool rejected = false;
    try { decide(off_grid, model, plan, on_track(model, 20, 10), {0, 0}); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "a stage that is not a whole number of plant ticks is refused when planning");
    // A car without tires is refused before a run: a model with tire slip would steer it tighter than it plans.
    bool kinematic = false;
    try {
        fd::Simulation without_tires(preset, config, fd::KinematicBicycle{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions,
                               fd::LocalPlannerMode::lattice, std::make_shared<fd::Mpcc>());
    } catch (const std::invalid_argument&) { kinematic = true; }
    require(kinematic, "a simulation of the kinematic bicycle driven by MPCC is refused");
}

// Closed loop: at every control decision the planner chooses and MPCC drives the chosen action; the plant is advanced
// with MPCC's command. Returns what the lap measured.
struct LapMeasure {
    double lap_s{-1}, worst_offset_m{}, peak_use{}, solve_median_ms{}, solve_worst_ms{}, top_speed{}, distance_m{};
    int decisions{}, unsolved{}, outside{}, beyond_peak{}, restarts{}, invalid_plans{}, policy_drove{}, in_blockage{};
    std::vector<double> offsets_beside;  // signed offset of the rear axle while beside the first blockage, first lap
    // The model error of the plans that drove (decision 0025): how far the plant, driven open loop by each plan's commands,
    // ends up from the plan a second and three seconds on; median and 95th percentile.
    double model_1s_median_m{}, model_1s_p95_m{}, model_3s_median_m{}, model_3s_p95_m{};
};
LapMeasure drive_a_lap(const fd::VehicleModel& model, double limit_s, const std::vector<fd::Obstruction>& obstructions = {}) {
    const auto plan = fd::make_speed_plan(preset, config);
    fd::Mpcc mpcc;
    const std::optional<fd::Lattice> lattice = obstructions.empty() ? std::nullopt : std::optional<fd::Lattice>(fd::make_lattice(preset, config));
    std::optional<fd::LocalOption> last_choice;
    const auto a = preset.points[0], b = preset.points[1];
    auto state = fd::plant_state_from({a.x_m, a.y_m, std::atan2(b.y_m-a.y_m, b.x_m-a.x_m), 0, 0, 0}, model, config);
    fd::Command held{};
    LapMeasure m;
    std::vector<double> solves, one_second, three_seconds;
    const auto ticks = static_cast<int>(std::llround(config.control_dt_s/config.fixed_dt_s));
    double progress = 0, previous = 0;
    for (int tick = 0; state.pose.time_s < limit_s; ++tick) {
        if (tick%ticks == 0) {
            fd::LocalDecision decision;
            const auto result = decide(mpcc, model, plan, state, held, obstructions, &decision, lattice ? &*lattice : nullptr,
                                       last_choice ? &*last_choice : nullptr);
            last_choice = decision.choice();
            ++m.decisions;
            solves.push_back(result.solve_time_s*1000);
            if (result.status == fd::ControllerStatus::not_solved) ++m.unsolved;
            if (result.restarted) ++m.restarts;
            if (!result.trajectory.within_model_envelope) ++m.invalid_plans;
            // As the simulation will: a plan that is not solved or not valid leaves the policy to drive.
            const bool usable = result.status != fd::ControllerStatus::not_solved && result.trajectory.within_model_envelope &&
                                fd::first_blockage_entered(preset, obstructions, result.trajectory) == obstructions.size();
            if (!usable) ++m.policy_drove;
            held = usable ? result.trajectory.first_control.applied : decision.trajectory().first_control.applied;
            if (usable) {
                const auto& points = result.trajectory.points;
                const auto response = fd::plant_response(model, state, config, fd::point_times(result.trajectory), result.trajectory.commands, 0);
                const auto apart = [&](std::size_t k) { return std::hypot(response[k].pose.x_m-points[k].state.x_m, response[k].pose.y_m-points[k].state.y_m); };
                one_second.push_back(apart(20));
                three_seconds.push_back(apart(60));
            }
        }
        const auto before = state.pose;
        state = fd::advance(model, state, held, config, config.fixed_dt_s);
        m.distance_m += std::hypot(state.pose.x_m-before.x_m, state.pose.y_m-before.y_m);
        m.top_speed = std::max(m.top_speed, state.pose.speed_mps);
        state.pose.time_s = (tick+1)*config.fixed_dt_s;
        const auto at = fd::project(preset, {state.pose.x_m, state.pose.y_m});
        double ds = at.s_m-previous;
        if (ds < -preset.length_m/2) ds += preset.length_m;
        if (ds > preset.length_m/2) ds -= preset.length_m;
        progress += ds;
        previous = at.s_m;
        m.worst_offset_m = std::max(m.worst_offset_m, std::abs(at.signed_error_m));
        if (!obstructions.empty()) {
            fd::LocalTrajectory here;
            here.points.push_back({state.pose, 0, 0});
            if (fd::first_blockage_entered(preset, obstructions, here) < obstructions.size()) ++m.in_blockage;
            if (progress < preset.length_m && at.s_m >= obstructions.front().from_s_m && at.s_m <= obstructions.front().to_s_m)
                m.offsets_beside.push_back(at.signed_error_m);
        }
        if (!fd::within_corridor(preset, at, fd::vehicle_half_width_m)) ++m.outside;
        const auto demanded = fd::demand(model, state, config, held.acceleration_mps2);
        m.peak_use = std::max(m.peak_use, demanded.grip_utilization);
        if (!demanded.within_envelope) ++m.beyond_peak;
        if (progress >= preset.length_m) { m.lap_s = state.pose.time_s; break; }
    }
    std::sort(solves.begin(), solves.end());
    m.solve_median_ms = solves[solves.size()/2];
    m.solve_worst_ms = solves.back();
    const auto quantile = [](std::vector<double> v, double share) {
        std::sort(v.begin(), v.end());
        return v.empty() ? 0.0 : v[std::min(v.size()-1, static_cast<std::size_t>(share*static_cast<double>(v.size())))];
    };
    m.model_1s_median_m = quantile(one_second, 0.5);
    m.model_1s_p95_m = quantile(one_second, 0.95);
    m.model_3s_median_m = quantile(three_seconds, 0.5);
    m.model_3s_p95_m = quantile(three_seconds, 0.95);
    return m;
}

std::string describe(const LapMeasure& m) {
    return "lap "+std::to_string(m.lap_s)+" s over "+std::to_string(m.distance_m)+" m, top speed "+std::to_string(m.top_speed)+
           " m/s, worst offset "+std::to_string(m.worst_offset_m)+" m, samples outside "+std::to_string(m.outside)+
           ", beyond a tire's peak "+std::to_string(m.beyond_peak)+", peak grip use "+std::to_string(m.peak_use)+"; "+
           std::to_string(m.decisions)+" decisions, "+std::to_string(m.unsolved)+" unsolved, "+std::to_string(m.restarts)+
           " restarts, "+std::to_string(m.policy_drove)+" driven by the policy; solve median "+std::to_string(m.solve_median_ms)+
           " ms, worst "+std::to_string(m.solve_worst_ms)+" ms; model error at 1 s median "+std::to_string(m.model_1s_median_m)+
           " m, 95th percentile "+std::to_string(m.model_1s_p95_m)+" m, at 3 s "+std::to_string(m.model_3s_median_m)+" m, "+
           std::to_string(m.model_3s_p95_m)+" m";
}

void the_dynamic_car_laps_the_preset() {
    const auto m = drive_a_lap(fd::DynamicSingleTrack{}, 60);
    std::cout << "  dynamic car: " << describe(m) << "; " << m.invalid_plans << " plans flagged\n";
    require(m.lap_s > 0, "the lap is completed");
    require(m.outside == 0, "without leaving the corridor");
    require(m.beyond_peak == 0, "without passing a tire's peak");
    require(m.top_speed <= config.max_speed_mps+0.05, "nor the top speed");
    // Its model is the plant, so driven open loop by a plan's commands the plant stays close to the plan: only the plant's
    // hold of each command over a control period and its shorter integration step differ (decision 0025).
    require(m.model_1s_median_m < 0.01 && m.model_3s_median_m < 0.15, "the plant follows the plan it was predicted with");
}


// The four-wheel car is predicted with its single-track reduction, which carries its pitch and air but not its lateral
// load transfer, brake bias or wheel speeds (decision 0025). Predicted without the load moving rearward, its front wheels
// spun pulling away; with it, no tire passes its peak, and the plant stays near each plan open loop.
void the_four_wheel_car_laps_the_preset() {
    const auto m = drive_a_lap(fd::FourWheelCar{}, 60);
    std::cout << "  four-wheel car: " << describe(m) << '\n';
    require(m.lap_s > 0, "the lap is completed");
    require(m.outside == 0, "without leaving the corridor");
    require(m.beyond_peak == 0, "without passing a tire's peak, pulling away or anywhere else");
    require(m.model_1s_median_m < 0.05 && m.model_3s_median_m < 0.3, "the plant follows the plan within its reduction's error");
}

// With the right side of the lane blocked the planner passes left, and MPCC keeps to the side it chose: it never
// enters the blockage, and beside it the car is left of it.
void a_blocked_lane_is_passed_on_the_side_the_planner_chose() {
    const std::vector<fd::Obstruction> lane{{62, 68, -4.5, 1.0, "cone-cluster"}};
    const auto m = drive_a_lap(fd::DynamicSingleTrack{}, 60, lane);
    double lowest = 1e9, highest = -1e9;
    for (const double d : m.offsets_beside) { lowest = std::min(lowest, d); highest = std::max(highest, d); }
    std::cout << "  lane blocked: " << describe(m) << "; beside it at " << lowest << " to " << highest << " m\n";
    require(m.lap_s > 0 && m.outside == 0, "the lap is completed inside the corridor");
    require(m.in_blockage == 0, "without entering the blockage");
    require(m.policy_drove == 0, "and MPCC drives every decision itself: none of its plans needed refusing");
    require(!m.offsets_beside.empty() && lowest > 1.0+fd::vehicle_half_width_m, "passing left of it, clear of its edge");
}

// Through the simulation's seam: MPCC's plan replaces the chosen option's prediction and its first command is held, the
// car laps with no invalid sample, and the simulation reports who drove.
void the_simulation_drives_the_chosen_action_with_mpcc() {
    fd::Simulation simulation(preset, config, fd::DynamicSingleTrack{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions,
                              fd::LocalPlannerMode::lattice, std::make_shared<fd::Mpcc>());
    require(simulation.controller_mode() == fd::ControllerMode::mpcc, "the simulation says MPCC drives");
    auto seen = simulation.decision_count();
    int by_mpcc = 0, by_policy = 0, invalid = 0;
    while (simulation.diagnostics().laps < 1 && simulation.state().time_s < 40) {
        simulation.step();
        if (!simulation.diagnostics().plan_valid) ++invalid;
        if (simulation.decision_count() == seen) continue;
        seen = simulation.decision_count();
        const auto& outcome = simulation.controller_outcome();
        const auto& trajectory = simulation.decision().trajectory();
        if (outcome.driven_by == fd::ControllerMode::mpcc) {
            ++by_mpcc;
            require(trajectory.points.size() == 61 && outcome.note.empty() && outcome.status != fd::ControllerStatus::not_solved,
                    "MPCC's plan is the decision's prediction");
        } else {
            ++by_policy;
            require(!outcome.note.empty(), "when the policy drives the outcome says why");
        }
        near(simulation.diagnostics().requested.acceleration_mps2, trajectory.first_control.requested.acceleration_mps2, 0,
             "the command held is the prediction's first");
    }
    std::cout << "  through the simulation: lap " << simulation.state().time_s << " s; " << by_mpcc << " decisions driven by MPCC, "
              << by_policy << " by the policy; " << invalid << " invalid samples\n";
    require(simulation.diagnostics().laps == 1, "the lap is completed");
    require(invalid == 0, "without an invalid sample");
    require(by_mpcc > 20*by_policy, "MPCC drives nearly every decision");
}

// A controller whose answers the test chooses, so the simulation's checks on a plan can be seen refusing each kind.
class Scripted final : public fd::PredictiveController {
public:
    std::function<fd::ControllerPlan(const fd::ControlRequest&)> answer;
    int asked{}, resets{};
    fd::ControllerPlan plan(const fd::ControlRequest& request) override { ++asked; return answer(request); }
    void reset() override { ++resets; }
    fd::ControllerSettings settings() const override { return {fd::ControllerMode::mpcc, {}}; }
    void check(const fd::VehicleModel&, const fd::Config&) const override {}
};

// The policy drives whenever the plan is not solved, leaves its envelope, enters a blockage, or the decision holds for
// one; the prediction and command are then the policy's own, exactly as without a controller.
void a_plan_the_checks_refuse_leaves_the_policy_to_drive() {
    const std::vector<fd::Obstruction> lane{{62, 68, -4.5, 1.0, "cone-cluster"}};
    const auto run = [&](std::function<fd::ControllerPlan(const fd::ControlRequest&)> answer, const std::vector<fd::Obstruction>& blocked,
                         double seconds) {
        auto scripted = std::make_shared<Scripted>();
        scripted->answer = std::move(answer);
        fd::Simulation with(preset, config, fd::DynamicSingleTrack{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions,
                            fd::LocalPlannerMode::lattice, scripted);
        fd::Simulation without(preset, config, fd::DynamicSingleTrack{});
        with.set_obstructions(blocked);
        without.set_obstructions(blocked);
        while (with.state().time_s < seconds) { with.step(); without.step(); }
        require(with.state().x_m == without.state().x_m && with.state().speed_mps == without.state().speed_mps,
                "the car moves exactly as it does without a controller");
        require(with.decision().trajectory().points.size() == without.decision().trajectory().points.size(), "the prediction is the policy's");
        return std::make_pair(with.controller_outcome(), scripted);
    };
    const auto [unsolved, first] = run([](const fd::ControlRequest& r) { fd::ControllerPlan p; p.trajectory = r.decision.trajectory(); return p; }, {}, 1.0);
    require(unsolved.driven_by == fd::ControllerMode::policy && unsolved.note.find("not solved") != std::string::npos, "an unsolved plan: "+unsolved.note);
    require(first->asked > 0 && first->resets == 1, "the controller was asked, and reset with the run");
    const auto [flagged, second] = run([](const fd::ControlRequest& r) {
        fd::ControllerPlan p;
        p.status = fd::ControllerStatus::solved;
        p.trajectory = r.decision.trajectory();
        p.trajectory.within_model_envelope = false;
        p.trajectory.validity_reason = "MPCC plan leaves the corridor";
        return p;
    }, {}, 1.0);
    require(flagged.driven_by == fd::ControllerMode::policy && flagged.note.find("leaves the corridor") != std::string::npos, "a plan outside: "+flagged.note);
    // Straight through the blocked lane: the reference line's own prediction, which the planner did not choose.
    const auto [blocked, third] = run([&](const fd::ControlRequest& r) {
        fd::ControllerPlan p;
        p.status = fd::ControllerStatus::solved;
        p.trajectory = fd::make_local_trajectory(preset, r.plan, r.state, config, r.model, r.ticks_to_next_control);
        p.trajectory.commands.assign(p.trajectory.points.size(), p.trajectory.first_control.applied);
        return p;
    }, lane, 3.0);
    require(blocked.driven_by == fd::ControllerMode::policy && blocked.note.find("enters cone-cluster") != std::string::npos, "a plan into the blockage: "+blocked.note);
    const std::vector<fd::Obstruction> wall{{62, 68, -5, 5, "stalled-car"}};
    const auto [holding, fourth] = run([](const fd::ControlRequest& r) { fd::ControllerPlan p; p.trajectory = r.decision.trajectory(); return p; }, wall, 6.0);
    require(holding.driven_by == fd::ControllerMode::policy && holding.note.find("Holding for stalled-car") != std::string::npos, "a hold: "+holding.note);
    // MPCC itself refuses to be asked to hold.
    fd::Mpcc mpcc;
    const auto plan = fd::make_speed_plan(preset, config);
    fd::LocalDecision hold;
    hold.options.resize(1);
    hold.holding = true;
    const fd::VehicleModel model = fd::DynamicSingleTrack{};
    const auto state = on_track(model, 50, 5);
    bool refused = false;
    try { mpcc.plan({preset, plan, config, model, state, hold, wall, {}, 0}); } catch (const std::invalid_argument&) { refused = true; }
    require(refused, "MPCC refuses a hold");

    // A plan that would drive must state its command at each point and drive the first of them; a controller that does not
    // is broken, and the simulation says so rather than record a plan the car did not follow (decision 0025).
    const auto broken = [&](std::function<fd::ControllerPlan(const fd::ControlRequest&)> answer) {
        auto scripted = std::make_shared<Scripted>();
        scripted->answer = std::move(answer);
        try {
            fd::Simulation simulation(preset, config, fd::DynamicSingleTrack{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::grip_fractions,
                                      fd::LocalPlannerMode::lattice, scripted);
        } catch (const std::logic_error& error) { return std::string(error.what()); }
        return std::string();
    };
    const auto without_commands = broken([](const fd::ControlRequest& r) {
        fd::ControllerPlan p;
        p.status = fd::ControllerStatus::solved;
        p.trajectory = r.decision.trajectory();
        return p;
    });
    require(without_commands.find("predictive controller's plan must state its command") != std::string::npos,
            "a plan without commands is the controller's fault, and the simulation says so: "+without_commands);
    const auto another_first = broken([](const fd::ControlRequest& r) {
        fd::ControllerPlan p;
        p.status = fd::ControllerStatus::solved;
        p.trajectory = r.decision.trajectory();
        p.trajectory.commands.assign(p.trajectory.points.size(), fd::Command{});
        p.trajectory.first_control.applied = {1.0, 0};
        return p;
    });
    require(another_first.find("first command must be its plan's") != std::string::npos, "a first command not its plan's: "+another_first);
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"on_the_straight_it_speeds_up_within_its_limits", on_the_straight_it_speeds_up_within_its_limits},
        {"each_decision_starts_from_the_last_plan", each_decision_starts_from_the_last_plan},
        {"a_car_above_the_top_speed_is_brought_back_to_it", a_car_above_the_top_speed_is_brought_back_to_it},
        {"options_outside_their_ranges_are_refused", options_outside_their_ranges_are_refused},
        {"the_dynamic_car_laps_the_preset", the_dynamic_car_laps_the_preset},
        {"the_four_wheel_car_laps_the_preset", the_four_wheel_car_laps_the_preset},
        {"a_blocked_lane_is_passed_on_the_side_the_planner_chose", a_blocked_lane_is_passed_on_the_side_the_planner_chose},
        {"the_simulation_drives_the_chosen_action_with_mpcc", the_simulation_drives_the_chosen_action_with_mpcc},
        {"a_plan_the_checks_refuse_leaves_the_policy_to_drive", a_plan_the_checks_refuse_leaves_the_policy_to_drive},
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cout << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " MPCC groups passed\n";
    return failures == 0 ? 0 : 1;
}

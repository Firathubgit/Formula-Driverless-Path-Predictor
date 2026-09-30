#pragma once
#include "fd/local_planner.hpp"

#include <array>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace fd {

// Which controller turns the action the planner chose into commands (decision 0024): the policy every prediction rolls
// forward, Pure Pursuit or MAP with speed feedback, or model predictive contouring control, which optimises the car's
// motion over a horizon instead. A person driving (decision 0037) is who drove a decision, never a controller a run is
// configured with, and is never recorded.
enum class ControllerMode { policy, mpcc, human };
// "policy", "mpcc" or "human".
std::string_view controller_mode_name(ControllerMode mode);

// What a predictive controller is asked at a control decision: to drive the action the planner chose, from the plant's
// actual state. Everything is borrowed for the call only.
struct ControlRequest {
    const Track& track;
    const std::vector<PlanPoint>& plan;       // the reference speed plan
    const Config& config;
    const VehicleModel& model;                // the plant; the controller predicts with its own reduction of it
    const PlantState& state;
    const LocalDecision& decision;            // its choice is the action to drive, its trajectory the policy's prediction of it
    const std::vector<Obstruction>& obstructions;
    Command held;                             // the command the plant has been holding
    std::uint64_t ticks_to_next_control{};    // fixed ticks this decision's command will be held
};

enum class ControllerStatus { solved, solved_inaccurate, not_solved };
// "solved", "solved inaccurate" or "not solved".
std::string_view controller_status_name(ControllerStatus status);

// A predictive controller's plan for one decision.
struct ControllerPlan {
    // The motion it plans from the actual state, on the fixed-tick grid and ending prediction_horizon_s ahead, like every
    // prediction, with the command it plans at each point; first_control holds the command for the coming control
    // interval, which is where those commands are at the interval's end (planned_command). Its envelope flags are the
    // controller's own prediction model's, and it is left inside the corridor less the vehicle half width or flagged.
    LocalTrajectory trajectory;
    ControllerStatus status{ControllerStatus::not_solved};
    int iterations{};          // optimisation steps taken for this decision
    bool restarted{};          // it started from the policy's prediction instead of its own last plan
    double solve_time_s{};     // wall clock on this host: a measurement, never recorded as a result
};

// Who drove a decision's first command and how the predictive controller fared, when there is one. Its plan drives only
// while it is solved, within its own envelope and the corridor, clear of every blockage, and not asked to hold for one;
// otherwise the policy drives and the note says why.
struct ControllerOutcome {
    ControllerMode driven_by{ControllerMode::policy};
    bool asked{};            // a predictive controller was asked for a plan; never for a hold
    ControllerStatus status{ControllerStatus::not_solved};   // meaningful only when a predictive controller was asked
    int iterations{};
    bool restarted{};
    double solve_time_s{};   // wall clock on this host, a measurement only
    std::string note;        // empty when the configured controller drove
};

// The settings a controller was run with, recorded with a run: its kind and named numbers.
struct ControllerSettings {
    ControllerMode mode{ControllerMode::policy};
    std::vector<std::pair<std::string, double>> values;
};

// A controller that plans the car's motion over a horizon at every control decision and commands its first step. It may
// keep what it planned last time to start from; reset() forgets it, as at the start of a run.
class PredictiveController {
public:
    virtual ~PredictiveController() = default;
    virtual ControllerPlan plan(const ControlRequest& request) = 0;
    virtual void reset() = 0;
    virtual ControllerSettings settings() const = 0;
    // Rejects, before a run, a car or configuration it cannot drive. The simulation asks when it is built.
    virtual void check(const VehicleModel& model, const Config& config) const = 0;
};

// The times of a trajectory's points.
std::vector<double> point_times(const LocalTrajectory& trajectory);
// Where a plan's commands are at a time (decision 0025): each changes linearly between its points' times and holds before
// the first and after the last. The times ascend and match the commands one for one.
Command planned_command(const std::vector<double>& times, const std::vector<Command>& commands, double time_s);
// The plant's response to a plan (TrackWayFastPlan Phase 6.2, decision 0025): the plant, copied from the state the plan was
// made in, driven by the plan's commands the way the simulation holds a command, where they are at the end of each control
// interval; the first interval ends ticks_to_next_control fixed ticks on, a full control period for zero. Returns the
// plant's state at each of the plan's times, which start at the state's time and lie on the fixed-tick grid from it. Under
// the configuration the plan was made with, what the plan gets wrong against it is its prediction model's alone: nothing
// is replanned, and the plant goes wherever the plan's commands take it.
std::vector<PlantState> plant_response(const VehicleModel& model, const PlantState& start, const Config& config,
                                       const std::vector<double>& times, const std::vector<Command>& commands,
                                       std::uint64_t ticks_to_next_control);

// What each tire is asked of its own friction circle at one point of a plan (TrackWayFastPlan Phase 6.4, decision
// 0027), in the plant's wheel order. The margin a tire has left there is one minus its use; above one it is sliding.
struct PlanTireUse {
    double time_s{};
    std::array<double, 4> use{};
};
// Each tire's use along a plan: the plant's response to the plan's own commands (plant_response), and at each of its
// points what demand() says each wheel uses of its own circle in the state the plant reaches there. Only a car with four
// wheels has four tires of its own to report; every other model returns nothing, its axles being what its slip card shows.
std::vector<PlanTireUse> plan_tire_use(const VehicleModel& model, const PlantState& start, const Config& config,
                                       const std::vector<double>& times, const std::vector<Command>& commands,
                                       std::uint64_t ticks_to_next_control);

} // namespace fd

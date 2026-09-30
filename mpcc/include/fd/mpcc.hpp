#pragma once
#include "fd/predictive_control.hpp"

#include <array>
#include <vector>

namespace fd {

// Settings of model predictive contouring control (TrackWayFastPlan Phase 6.1, decision 0024), after Liniger's MPCC
// (Apache-2.0), reimplemented.
struct MpccOptions {
    // The horizon: stages of stage_s each. Sixty of 0.05 s is prediction_horizon_s, so its plan is drawn and recorded
    // like every prediction.
    int stages{60};
    double stage_s{0.05};
    int substeps{2};                    // RK4 steps integrating the prediction model over one stage
    int iterations{2};                  // sequential quadratic programming steps per decision
    double step_share{0.9};             // share of each step taken, MPCC's SQP mixing
    double step_weight{1.0};            // penalty per squared unit of each state and input change in a step
    double solver_tolerance{1e-3};      // OSQP's absolute and relative tolerance; each step is only a step
    int restart_after_failures{2};      // decisions in a row without a solved step before starting again from the policy
    // Cost, per stage. Contouring is the distance across the path being driven, lag the distance along it behind the
    // progress point; progress is rewarded at progress_weight per metre per second of the reference station's rate.
    double contouring_weight{0.1}, lag_weight{100}, progress_weight{1.0};
    double terminal_contouring_multiplier{10};
    double heading_weight{0.5};         // per squared radian off the path's heading
    double sideslip_weight{5};          // per squared radian of sideslip beyond the kinematic bicycle's
    double yaw_rate_weight{0.01};
    double acceleration_weight{1e-3}, steering_weight{1e-2}, progress_rate_weight{1e-5};
    double jerk_weight{1e-3}, steering_rate_weight{1.0}, progress_acceleration_weight{1e-4};
    double slack_quadratic_weight{1000}, slack_linear_weight{100};
    // Constraints, softened by slack. Each axle's slip angle is kept within this share of the angle of its peak grip,
    // on the rising side of the tire curve where the linearisation points the right way (MPCC keeps its front tire at
    // about 70% of the way to its peak); each axle's requested forces within this share of its friction ellipse.
    double slip_angle_share{0.7};
    double friction_share{0.95};
    // Kept from the corridor's edges and from blockages beyond the vehicle half width, at the centre of gravity.
    double corridor_margin_m{0.3};
    double max_jerk_mps3{40};
    double max_progress_acceleration_mps2{40};
};

// Model predictive contouring control: at every decision it optimises the car's motion over the horizon, with the
// dynamic single-track model reduced from the plant (reduced_single_track), to make the most progress along the path of
// the action the planner chose, subject to the corridor that action leaves open and to each axle's slip angle and
// friction ellipse. It drives cars with tires; the kinematic bicycle is refused. It solves the nonlinear problem by sequential quadratic programming with OSQP, starting from its own
// last plan moved on by the time since, or, at the start and after repeated failures, from the policy's prediction of the
// chosen action. Holding for a blockage is the policy's to drive; a request that holds is refused.
class Mpcc final : public PredictiveController {
public:
    // Rejects options outside their supported ranges, and a horizon other than prediction_horizon_s.
    explicit Mpcc(MpccOptions options = {});
    ControllerPlan plan(const ControlRequest& request) override;
    void reset() override;
    ControllerSettings settings() const override;
    // Rejects the kinematic bicycle, which has no tires to predict with, and a stage that is not a whole number of plant
    // ticks.
    void check(const VehicleModel& model, const Config& config) const override;
    const MpccOptions& options() const noexcept { return options_; }

    // Prediction model state and input sizes, for the tests' finite-difference checks.
    static constexpr int state_size = 10, input_size = 3;
private:
    MpccOptions options_;
    // The last plan: stage states and inputs, and the simulation time of stage zero.
    std::vector<std::array<double, state_size>> states_;
    std::vector<std::array<double, input_size>> inputs_;
    double planned_at_s_{};
    bool have_plan_{};
    int failures_{};
    // The last solved programme's multipliers, where the next one starts.
    std::vector<double> duals_;
};

} // namespace fd

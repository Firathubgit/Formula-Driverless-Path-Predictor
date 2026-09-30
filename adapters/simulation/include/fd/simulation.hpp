#pragma once
#include "fd/competition.hpp"
#include "fd/cone_driving.hpp"
#include "fd/human_driving.hpp"
#include "fd/local_planner.hpp"
#include "fd/performance_envelope.hpp"
#include "fd/predictive_control.hpp"
#include "fd/perception.hpp"
#include "fd/sensors.hpp"

#include <memory>
#include <optional>

namespace fd {

struct Diagnostics {
    double target_speed_mps{};
    double cross_track_error_m{};
    double progress_m{};
    double path_s_m{};
    double plan_generated_time_s{};
    double lateral_acceleration_mps2{};
    // What the car actually achieved along its body forward axis over the last fixed tick, which is what an
    // accelerometer reads; the acceleration commanded is applied.acceleration_mps2.
    double longitudinal_acceleration_mps2{};
    double combined_grip_utilization{};
    // Plant quantities. The kinematic bicycle cannot slide, so its sideslip is always zero.
    double yaw_rate_radps{};
    double rear_sideslip_rad{};
    bool within_grip_envelope{true};
    // Axle slip angles, their peaks and the handling balance, from the vehicle seam (decision 0011).
    double front_slip_angle_rad{}, rear_slip_angle_rad{}, front_peak_slip_angle_rad{}, rear_peak_slip_angle_rad{};
    Balance balance{Balance::not_modeled};
    double understeer_angle_rad{};
    double lookahead_m{};
    // Pure Pursuit's steering toward the chosen target; MAP's correction is requested minus this.
    double geometric_steering_rad{};
    std::size_t nearest_index{};
    std::size_t limiting_index{};
    std::uint64_t revision{1};
    int laps{};
    bool plan_valid{true};
    bool within_track{true};
    bool control_saturated{};
    Command requested{}, applied{};
    std::string limiting_reason;
    std::string validity_reason{"Within conservative kinematic limits"};
};
struct ParameterEvent {
    double time_s{};
    std::uint64_t revision{};
    std::string parameter;
    double old_value{}, new_value{};
};

class Simulation {
public:
    explicit Simulation(Config config = {});
    Simulation(Track track, Config config);
    // Selects the vehicle model; the two-argument form uses the kinematic bicycle.
    Simulation(Track track, Config config, VehicleModel model);
    // Selects the steering law. MAP generates a steering table from the model and configuration
    // and regenerates it whenever a parameter change commits, so the table always matches.
    Simulation(Track track, Config config, VehicleModel model, SteeringMode steering);
    // Selects what the speed plan is made from. The performance envelope is derived for the model and configuration
    // (four-wheel car only, about a second), planned with, bounds every control decision, and is derived again with
    // the plan whenever a parameter change commits (decision 0018).
    Simulation(Track track, Config config, VehicleModel model, SteeringMode steering, SpeedPlanMode speed_plan);
    // Selects the local planner: the lattice planner by default (decision 0022), or the five-offset planner for
    // comparison. The lattice planner's lattice is laid along the track when blockages are first stated.
    Simulation(Track track, Config config, VehicleModel model, SteeringMode steering, SpeedPlanMode speed_plan, LocalPlannerMode planner);
    // Drives the planner's chosen action with a predictive controller instead of the policy (decision 0024); null keeps
    // the policy. At every decision its plan replaces the chosen option's prediction and its first command drives, unless
    // the plan is not solved, leaves its own envelope or the corridor, or enters a blockage, or the decision holds for a
    // blockage: then the policy drives, as without it, and controller_outcome() says why. The controller is reset with
    // the run, and a car it cannot drive is refused here.
    Simulation(Track track, Config config, VehicleModel model, SteeringMode steering, SpeedPlanMode speed_plan, LocalPlannerMode planner,
               std::shared_ptr<PredictiveController> controller);
    void step();
    void reset();
    // Validated immediately; committed at the next fixed tick, preserving vehicle state.
    void set_grip(double grip_mu);
    // Gives the car instruments to read itself with, replaces the ones it has, or with nothing takes them off again
    // (decision 0029). Scenario setup like
    // the blockages below: validated immediately, carrying no configuration revision, and starting its cadence and its
    // seeded errors at the next fixed tick. On the known track nothing in the simulation reads what they measure; the
    // plant always advances on its own state, and driving on cones the driver decides from what they measure.
    void set_sensors(std::optional<SensorSettings> settings);
    // The instruments in force, and what they last delivered; both null until sensors are set.
    const SensorSuite* sensors() const noexcept { return sensors_ ? &*sensors_ : nullptr; }
    const Measurements* measurements() const noexcept { return sensors_ ? &sensors_->measurements() : nullptr; }
    // Gives the car simulated cone perception of the course's cones (decision 0030), replaces it, or with nothing takes it
    // off. The cones are ground truth like the blockages, laid along the corridor unless a course says otherwise; the sensors observe
    // them from the car's true pose at every fixed tick, starting their cadence here. On the known track nothing in the
    // simulation reads what they detect; driving on cones the driver plans from it. Taking perception off while driving
    // on cones is refused.
    void set_perception(std::optional<PerceptionSettings> settings);
    // The perception in force, with the cones it observes; null until perception is set.
    const Perception* perception() const noexcept { return perception_ ? &*perception_ : nullptr; }
    // Drives on cones (decision 0031), or with nothing on the known track again, and starts the run over either way, so
    // that the car's map begins where it is placed. On cones a ConeDriver makes every decision from the car's readings and
    // perception frames alone: the instruments' measurements when the car has them, otherwise the true state, the
    // ideal-state assumption. It follows the path it believes by the policy's rules; the local planner, the reference
    // plan and the known track decide nothing, and remain only as what the run is evaluated against. Needs perception;
    // refuses blockages, which are stated in the known track's stations, and the predictive controller, which plans along
    // the known track.
    // The driver remembers the cones it places, as memory says (on by default).
    void set_cone_driving(std::optional<ConePathSettings> settings, ConeMemory memory = {});
    // The driver while driving on cones; null on the known track.
    const ConeDriver* cone_driver() const noexcept { return cone_driver_ ? &*cone_driver_ : nullptr; }
    // Where the driver's readings come from: the instruments when fitted, otherwise the true state.
    BeliefSource belief_source() const noexcept { return sensors_ ? BeliefSource::measured : BeliefSource::ideal; }
    // The course the run is judged on (decision 0032): the track's corridor, its cones and its timekeeping gates. By
    // default the cones are laid along the corridor and the gates across the start and at thirds of the lap; a traced
    // layout gives its own, for the track it was made into, and names where it came from. Scenario setup like the
    // blockages: validated immediately, the run started over, and the perception, if fitted, looking at these cones from
    // then on.
    void set_course(std::vector<Cone> cones, std::vector<Gate> gates, std::string source);
    // Where the course's cones and gates came from: laid_along_the_track unless a layout was given.
    static constexpr const char* laid_along_the_track = "laid along the track";
    const std::string& course_source() const noexcept { return course_source_; }
    // The rules the run is judged by; a trackdrive of ten laps unless set. Scenario setup; the run starts over.
    void set_competition(CompetitionRules rules);
    // The judge, observing the car's true pose every fixed tick. It is the evaluation's: nothing that drives reads it.
    const Judge& judge() const noexcept { return *judge_; }
    const std::vector<Cone>& cones() const noexcept { return judge_->course().cones; }
    // The plant as it truly is, with the accelerations an inertial unit would read: what the instruments observe.
    TrueState true_state() const noexcept { return {plant_, diagnostics_.longitudinal_acceleration_mps2,
                                                    diagnostics_.lateral_acceleration_mps2}; }
    // Scenario setup, not a live parameter: validated immediately and in force from the
    // next control decision. These are stated blocked regions, not detected objects, so
    // they carry no configuration revision or parameter event. Under the lattice planner the
    // first blockages lay the lattice, and a track that refuses one refuses the blockages.
    void set_obstructions(std::vector<Obstruction> obstructions);
    const State& state() const noexcept { return plant_.pose; }
    const PlantState& plant_state() const noexcept { return plant_; }
    const VehicleModel& vehicle_model() const noexcept { return model_; }
    SteeringMode steering_mode() const noexcept { return steering_mode_; }
    // The table in force under MAP; null under Pure Pursuit.
    const SteeringTable* steering_table() const noexcept { return steering_table_ ? &*steering_table_ : nullptr; }
    SpeedPlanMode speed_plan_mode() const noexcept { return speed_plan_mode_; }
    LocalPlannerMode local_planner_mode() const noexcept { return planner_mode_; }
    // The lattice the lattice planner searches; null until blockages are stated, and under the five-offset planner.
    const Lattice* lattice() const noexcept { return lattice_ ? &*lattice_ : nullptr; }
    // The envelope the plan in force was made from; null under the grip fractions.
    const PerformanceEnvelope* performance_envelope() const noexcept { return envelope_ ? &*envelope_ : nullptr; }
    const Track& track() const noexcept { return track_; }
    const std::vector<PlanPoint>& plan() const noexcept { return plan_; }
    const std::vector<Obstruction>& obstructions() const noexcept { return obstructions_; }
    const LocalDecision& decision() const noexcept { return decision_; }
    // Which controller drives: the policy, or the predictive controller given at construction.
    ControllerMode controller_mode() const noexcept { return controller_ ? controller_->settings().mode : ControllerMode::policy; }
    ControllerSettings controller_settings() const { return controller_ ? controller_->settings() : ControllerSettings{}; }
    // Who drove the decision in force, and how the predictive controller fared on it.
    const ControllerOutcome& controller_outcome() const noexcept { return outcome_; }
    // A person drives instead of the autonomous driver (decision 0037), or with false hands the car back. From the control
    // decision made here, and at every one after, the command is the person's controls' (human_command), the last given;
    // the planner, or on cones the driver, still decides from the actual state as it would drive from there, which is what
    // the view shows, but its action drives nothing and the predictive controller is not asked. Each such decision says
    // the person drove it. The plant, the instruments, the perception and the judge are as ever. Nothing records it: a
    // person's controls are not part of the recording contract.
    void set_human_driving(bool on);
    bool human_driving() const noexcept { return human_; }
    // The person's controls: validated immediately, and in force from the next control decision. Released at a reset.
    void set_driver_controls(DriverControls controls);
    const DriverControls& driver_controls() const noexcept { return controls_; }
    // Human-only, four-wheel training override. Validated now, committed before the next
    // fixed tick's motion, with a revision and event at that tick's starting time.
    void set_driving_assistance(DrivingAssistance assistance);
    DrivingAssistance driving_assistance() const noexcept { return assistance_; }
    DrivingAssistance requested_driving_assistance() const noexcept { return pending_assistance_.value_or(assistance_); }
    bool assistance_active() const { return human_ && driving_assistance_active(assistance_); }
    // Control decisions made since construction, counting those superseded without
    // governing any motion. A recorder compares it across steps to see a new decision.
    std::uint64_t decision_count() const noexcept { return decisions_; }
    const LocalTrajectory& trajectory() const noexcept { return decision_.choice().trajectory; }
    const Config& config() const noexcept { return config_; }
    const Diagnostics& diagnostics() const noexcept { return diagnostics_; }
    const std::vector<ParameterEvent>& events() const noexcept { return events_; }
private:
    void update_control();
    void drive_with_controller();
    void drive_by_hand();
    void update_diagnostics();
    void feed_driver();
    Config config_;
    Track track_;
    VehicleModel model_;
    SteeringMode steering_mode_{SteeringMode::pure_pursuit};
    std::optional<SteeringTable> steering_table_;
    SpeedPlanMode speed_plan_mode_{SpeedPlanMode::grip_fractions};
    std::optional<PerformanceEnvelope> envelope_;
    LocalPlannerMode planner_mode_{LocalPlannerMode::lattice};
    std::optional<Lattice> lattice_;
    std::vector<PlanPoint> plan_;
    std::vector<Obstruction> obstructions_;
    LocalDecision decision_;
    std::shared_ptr<PredictiveController> controller_;
    ControllerOutcome outcome_;
    PlantState plant_;
    std::optional<SensorSuite> sensors_;
    std::optional<Perception> perception_;
    std::optional<ConeDriver> cone_driver_;
    std::optional<Judge> judge_;
    std::string course_source_{laid_along_the_track};
    Diagnostics diagnostics_;
    std::vector<ParameterEvent> events_;
    Command held_command_;
    bool human_{};
    DriverControls controls_;
    DrivingAssistance assistance_;
    std::optional<DrivingAssistance> pending_assistance_;
    std::uint64_t tick_{}, control_ticks_{4}, decisions_{};
    double previous_s_{}, total_progress_{}, pending_grip_{};
    bool has_pending_grip_{};
};

} // namespace fd

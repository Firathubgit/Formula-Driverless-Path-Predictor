#include "fd/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace fd {
namespace {

// The judge's car: the simulation's wheelbase, with a Formula Student car's track and overhangs.
Footprint footprint_for(const Config& config) {
    Footprint footprint;
    footprint.wheelbase_m=config.wheelbase_m;
    footprint.body_front_m=config.wheelbase_m+0.6;
    return footprint;
}

} // namespace

Simulation::Simulation(Config config) : Simulation(make_preset_track(),config) {}

Simulation::Simulation(Track track, Config config) : Simulation(std::move(track),config,KinematicBicycle{}) {}

Simulation::Simulation(Track track, Config config, VehicleModel model)
    : Simulation(std::move(track),config,std::move(model),SteeringMode::pure_pursuit) {}

Simulation::Simulation(Track track, Config config, VehicleModel model, SteeringMode steering)
    : Simulation(std::move(track),config,std::move(model),steering,SpeedPlanMode::grip_fractions) {}

Simulation::Simulation(Track track, Config config, VehicleModel model, SteeringMode steering, SpeedPlanMode speed_plan)
    : Simulation(std::move(track),config,std::move(model),steering,speed_plan,LocalPlannerMode::lattice) {}

Simulation::Simulation(Track track, Config config, VehicleModel model, SteeringMode steering, SpeedPlanMode speed_plan,
                       LocalPlannerMode planner)
    : Simulation(std::move(track),config,std::move(model),steering,speed_plan,planner,nullptr) {}

Simulation::Simulation(Track track, Config config, VehicleModel model, SteeringMode steering, SpeedPlanMode speed_plan,
                       LocalPlannerMode planner, std::shared_ptr<PredictiveController> controller)
    : config_(config),track_(std::move(track)),model_(std::move(model)),steering_mode_(steering),speed_plan_mode_(speed_plan),
      planner_mode_(planner),controller_(std::move(controller)) {
    if (planner_mode_!=LocalPlannerMode::lattice && planner_mode_!=LocalPlannerMode::five_offsets)
        throw std::invalid_argument("Unknown local planner");
    validate_vehicle(model_,config_);
    if (steering_mode_!=SteeringMode::pure_pursuit && steering_mode_!=SteeringMode::model_acceleration_pursuit)
        throw std::invalid_argument("Unknown steering mode");
    if (speed_plan_mode_!=SpeedPlanMode::grip_fractions && speed_plan_mode_!=SpeedPlanMode::performance_envelope)
        throw std::invalid_argument("Unknown speed plan mode");
    if (steering_mode_==SteeringMode::model_acceleration_pursuit) steering_table_.emplace(model_,config_);
    validate_track(track_);
    judge_.emplace(Course{track_,make_cone_layout(track_),make_gates(track_)},CompetitionRules{},footprint_for(config_));
    if (controller_) controller_->check(model_,config_);
    if (speed_plan_mode_==SpeedPlanMode::performance_envelope) envelope_.emplace(model_,config_);
    control_ticks_=static_cast<std::uint64_t>(std::llround(config_.control_dt_s/config_.fixed_dt_s));
    plan_=make_speed_plan(track_,config_,performance_envelope());
    reset();
}

void Simulation::reset() {
    assistance_=requested_driving_assistance();
    pending_assistance_.reset();
    const auto& a=track_.points[0];
    const auto& b=track_.points[1];
    plant_=plant_state_from({a.x_m,a.y_m,std::atan2(b.y_m-a.y_m,b.x_m-a.x_m),0,0,0},model_,config_);
    diagnostics_={};
    diagnostics_.revision=1;
    tick_=0;
    previous_s_=0;
    total_progress_=0;
    has_pending_grip_=false;
    events_.clear();
    held_command_={};
    controls_={};
    if (controller_) controller_->reset();
    if (cone_driver_) cone_driver_->reset(plant_.pose);
    update_control();
    update_diagnostics();
    // The instruments start with the run: nothing has arrived at rest, and their first sample is one period in.
    if (sensors_) { sensors_->reset(); sensors_->observe(true_state(),plant_.pose.time_s); }
    if (perception_) { perception_->reset(); perception_->observe(plant_.pose,plant_.pose.time_s); }
    if (cone_driver_) feed_driver();
    judge_->reset();
    judge_->observe(plant_.pose,plant_.pose.time_s);
}

void Simulation::set_grip(double grip_mu) {
    auto candidate=config_;
    candidate.grip_mu=grip_mu;
    validate_config(candidate);
    pending_grip_=grip_mu;
    has_pending_grip_=true;
}

void Simulation::set_sensors(std::optional<SensorSettings> settings) {
    if (!settings) { sensors_.reset(); return; }
    validate_sensors(*settings);
    sensors_.emplace(*settings);
    // Their clock starts where they are switched on, as it does at a reset, so a run they are set up with
    // measures from its first tick.
    sensors_->observe(true_state(),plant_.pose.time_s);
}

void Simulation::set_course(std::vector<Cone> cones, std::vector<Gate> gates, std::string source) {
    if (source.empty()) throw std::invalid_argument("A course names where it came from");
    Judge next(Course{track_,std::move(cones),std::move(gates)},judge_->rules(),judge_->footprint());
    std::optional<Perception> seeing;
    if (perception_) seeing.emplace(next.course().cones,perception_->settings());
    judge_=std::move(next);
    course_source_=std::move(source);
    if (seeing) perception_=std::move(seeing);
    reset();
}

void Simulation::set_competition(CompetitionRules rules) {
    // Built beside the judge in force, which is replaced only once the new one is accepted.
    Judge next(judge_->course(),rules,judge_->footprint());
    judge_=std::move(next);
    reset();
}

void Simulation::set_perception(std::optional<PerceptionSettings> settings) {
    if (!settings && cone_driver_) throw std::invalid_argument("Driving on cones needs cone perception; drive on the known track first");
    if (!settings) { perception_.reset(); return; }
    perception_.emplace(judge_->course().cones,*settings);
    perception_->observe(plant_.pose,plant_.pose.time_s);
}

void Simulation::set_cone_driving(std::optional<ConePathSettings> settings, ConeMemory memory) {
    if (settings) {
        validate_cone_path_settings(*settings);
        if (!perception_) throw std::invalid_argument("Driving on cones needs cone perception; set it first");
        if (!obstructions_.empty())
            throw std::invalid_argument("Driving on cones takes no blockages: they are stated in the known track's stations, which the car does not know");
        if (controller_)
            throw std::invalid_argument("Driving on cones is by the policy: the predictive controller plans along the known track");
        cone_driver_.emplace(*settings,memory);
    } else {
        cone_driver_.reset();
    }
    reset();
}

void Simulation::feed_driver() {
    // Everything the driver is given, every fixed tick: its readings, and the perception frames delivered this tick.
    if (sensors_) {
        const auto& m=sensors_->measurements();
        if (m.pose.measured) cone_driver_->read_pose({m.pose.sampled_at_s,m.x_m,m.y_m,m.yaw_rad});
        cone_driver_->read_motion({m.speed.measured ? m.speed_mps : 0.0,m.imu.measured ? m.yaw_rate_radps : 0.0,
                                   m.steering.measured ? m.steering_rad : 0.0});
    } else {
        // The ideal-state assumption: the true state, read the instant it is.
        const auto& p=plant_.pose;
        cone_driver_->read_pose({p.time_s,p.x_m,p.y_m,p.yaw_rad});
        cone_driver_->read_motion({p.speed_mps,plant_.yaw_rate_radps,p.steering_rad});
    }
    if (perception_)
        for (const auto& frame : perception_->delivered()) cone_driver_->perceive(frame);
}

void Simulation::set_human_driving(bool on) {
    if (on==human_) return;
    if (!on) {
        auto assistance=requested_driving_assistance();
        assistance.enabled=false;
        set_driving_assistance(assistance);
    }
    human_=on;
    // Handed back, the predictive controller starts afresh: it was not asked while the person drove.
    if (!on && controller_) controller_->reset();
    update_control();
}

void Simulation::set_driver_controls(DriverControls controls) {
    validate_driver_controls(controls);
    controls_=controls;
}

void Simulation::set_driving_assistance(DrivingAssistance assistance) {
    validate_driving_assistance(assistance);
    if (assistance.enabled && (!human_ || !std::holds_alternative<FourWheelCar>(model_)))
        throw std::invalid_argument("Forgiveness is available only to a person driving a four-wheel car");
    pending_assistance_=assistance;
}

void Simulation::set_obstructions(std::vector<Obstruction> obstructions) {
    if (cone_driver_ && !obstructions.empty())
        throw std::invalid_argument("Driving on cones takes no blockages: they are stated in the known track's stations, which the car does not know");
    validate_obstructions(track_,obstructions);
    if (planner_mode_==LocalPlannerMode::lattice && !obstructions.empty() && !lattice_) lattice_=make_lattice(track_,config_);
    obstructions_=std::move(obstructions);
    update_control();
}

void Simulation::step() {
    bool replan=false;
    if (pending_assistance_) {
        const auto next=*pending_assistance_;
        if (next!=assistance_) {
            ++diagnostics_.revision;
            if (next.enabled!=assistance_.enabled)
                events_.push_back({plant_.pose.time_s,diagnostics_.revision,"forgiveness_enabled",double(assistance_.enabled),double(next.enabled)});
            if (next.level!=assistance_.level)
                events_.push_back({plant_.pose.time_s,diagnostics_.revision,"forgiveness_level",assistance_.level,next.level});
            const auto changed=[&](const char* name,double old_value,double new_value) {
                if (old_value!=new_value) events_.push_back({plant_.pose.time_s,diagnostics_.revision,name,old_value,new_value});
            };
            changed("forgiveness_manual",double(assistance_.manual),double(next.manual));
            changed("forgiveness_acceleration",assistance_.acceleration,next.acceleration);
            changed("forgiveness_braking",assistance_.braking,next.braking);
            changed("forgiveness_steering",assistance_.steering,next.steering);
            changed("forgiveness_grip",assistance_.grip,next.grip);
            changed("forgiveness_speed",assistance_.speed,next.speed);
            assistance_=next;
            replan=true;
        }
        pending_assistance_.reset();
    }
    if (has_pending_grip_) {
        if (pending_grip_!=config_.grip_mu) {
            auto next=config_;
            next.grip_mu=pending_grip_;
            // The envelope depends on grip, so an envelope plan derives it again first, before anything commits.
            std::optional<PerformanceEnvelope> next_envelope;
            if (envelope_) next_envelope.emplace(model_,next);
            auto next_plan=make_speed_plan(track_,next,next_envelope ? &*next_envelope : nullptr);
            // The table depends on grip, so MAP regenerates it with the plan, before anything commits.
            std::optional<SteeringTable> next_table;
            if (steering_table_) next_table.emplace(model_,next);
            const double prior=config_.grip_mu;
            config_=next;
            plan_=std::move(next_plan);
            if (next_table) steering_table_=std::move(next_table);
            if (next_envelope) envelope_=std::move(next_envelope);
            ++diagnostics_.revision;
            diagnostics_.plan_generated_time_s=plant_.pose.time_s;
            events_.push_back({plant_.pose.time_s,diagnostics_.revision,"grip_mu",prior,config_.grip_mu});
            replan=true;
        }
        has_pending_grip_=false;
    }
    if (tick_%control_ticks_==0 || replan) update_control();
    const double previous_steering=plant_.pose.steering_rad;
    const double previous_speed=plant_.pose.speed_mps;
    const double previous_lateral=plant_.lateral_velocity_mps;
    plant_=human_ ? advance_assisted(model_,plant_,held_command_,config_,config_.fixed_dt_s,assistance_)
                  : advance(model_,plant_,held_command_,config_,config_.fixed_dt_s);
    ++tick_;
    plant_.pose.time_s=static_cast<double>(tick_)*config_.fixed_dt_s;
    const auto& state=plant_.pose;
    diagnostics_.applied={held_command_.acceleration_mps2,state.steering_rad};
    const double steering_rate=assistance_active() ? assisted_steering_rate(config_,assistance_)
                                                   : config_.max_steering_rate_radps;
    diagnostics_.control_saturated=
        std::abs(diagnostics_.requested.acceleration_mps2-held_command_.acceleration_mps2)>1e-8 ||
        std::abs(diagnostics_.requested.steering_rad-state.steering_rad)>1e-6 ||
        std::abs(state.steering_rad-previous_steering)>steering_rate*config_.fixed_dt_s+1e-9;
    diagnostics_.longitudinal_acceleration_mps2=(state.speed_mps-previous_speed)/config_.fixed_dt_s;
    update_diagnostics();
    if (assistance_active())
        diagnostics_.lateral_acceleration_mps2=(plant_.lateral_velocity_mps-previous_lateral)/config_.fixed_dt_s+
                                              plant_.pose.speed_mps*plant_.yaw_rate_radps;
    // One reading a tick, of the state the tick ended in: a channel faster than the tick would still sample once.
    if (sensors_) sensors_->observe(true_state(),state.time_s);
    if (perception_) perception_->observe(state,state.time_s);
    if (cone_driver_) feed_driver();
    judge_->observe(state,state.time_s);
}

void Simulation::update_control() {
    // A parameter event between control ticks must preserve the regular schedule.
    // Only the chosen first command advances the live plant; every predicted state,
    // including those of the rejected alternatives, is a copy.
    if (cone_driver_) {
        // Driving on cones the driver decides alone, from what it has been given; the known track is not consulted.
        decision_=cone_driver_->decide(plant_.pose.time_s,config_,model_,control_ticks_-tick_%control_ticks_,steering_table(),
                                       performance_envelope());
        ++decisions_;
        outcome_={};
        const auto& control=decision_.trajectory().first_control;
        held_command_=control.applied;
        diagnostics_.requested=control.requested;
        diagnostics_.lookahead_m=control.lookahead_m;
        diagnostics_.geometric_steering_rad=control.geometric_steering_rad;
        if (human_) drive_by_hand();
        return;
    }
    auto next=planner_mode_==LocalPlannerMode::lattice
        ? choose_lattice_action(track_,plan_,plant_,config_,model_,lattice(),obstructions_,
                                control_ticks_-tick_%control_ticks_,steering_table(),performance_envelope(),
                                decision_.options.empty() ? nullptr : &decision_.choice())
        : choose_local_action(track_,plan_,plant_,config_,model_,obstructions_,
                              control_ticks_-tick_%control_ticks_,steering_table(),performance_envelope());
    decision_=std::move(next);
    ++decisions_;
    outcome_={};
    if (human_) { drive_by_hand(); return; }
    if (controller_) drive_with_controller();
    const auto& control=decision_.trajectory().first_control;
    held_command_=control.applied;
    diagnostics_.requested=control.requested;
    diagnostics_.lookahead_m=control.lookahead_m;
    diagnostics_.geometric_steering_rad=control.geometric_steering_rad;
}

void Simulation::drive_by_hand() {
    // The decision just made stays as the planner made it, shown and not driven; the person's controls drive.
    held_command_=human_command(controls_,plant_,model_,config_,assistance_);
    diagnostics_.requested=held_command_;
    // Nothing pursued a target: no lookahead, and no correction to geometry.
    diagnostics_.lookahead_m=0;
    diagnostics_.geometric_steering_rad=held_command_.steering_rad;
    outcome_.driven_by=ControllerMode::human;
    outcome_.note="A person drives: the planner's action is shown, not driven";
    if (assistance_active()) outcome_.note="A person drives with forgiveness: the unassisted planner is shown, not driven";
}

void Simulation::drive_with_controller() {
    if (decision_.holding) {
        outcome_.note="Holding for "+decision_.blocking_identifier+": the policy brakes";
        return;
    }
    const ControlRequest request{track_,plan_,config_,model_,plant_,decision_,obstructions_,held_command_,
                                 control_ticks_-tick_%control_ticks_};
    auto plan=controller_->plan(request);
    outcome_.asked=true;
    outcome_.status=plan.status;
    outcome_.iterations=plan.iterations;
    outcome_.restarted=plan.restarted;
    outcome_.solve_time_s=plan.solve_time_s;
    const std::string name(controller_mode_name(controller_->settings().mode));
    if (plan.status==ControllerStatus::not_solved) { outcome_.note="The "+name+" plan was not solved: the policy drives"; return; }
    if (!plan.trajectory.within_model_envelope) { outcome_.note=plan.trajectory.validity_reason+": the policy drives"; return; }
    const auto hit=first_blockage_entered(track_,obstructions_,plan.trajectory);
    if (hit<obstructions_.size()) { outcome_.note="The "+name+" plan enters "+obstructions_[hit].identifier+": the policy drives"; return; }
    // A plan that drives states its command at each point, and its first command is where those commands are at the end
    // of this control interval, as the recording and the plant's response to the plan take it to be (decision 0025).
    const auto& planned=plan.trajectory;
    if (planned.commands.size()!=planned.points.size())
        throw std::logic_error("A predictive controller's plan must state its command at each of its points");
    const auto first=planned_command(point_times(planned),planned.commands,
                                     plant_.pose.time_s+static_cast<double>(request.ticks_to_next_control)*config_.fixed_dt_s);
    if (std::abs(first.acceleration_mps2-planned.first_control.applied.acceleration_mps2)>1e-9 ||
        std::abs(first.steering_rad-planned.first_control.applied.steering_rad)>1e-9)
        throw std::logic_error("A predictive controller's first command must be its plan's command at the end of the control interval");
    // The plan passed every check a prediction gets, so it is the chosen action's prediction now.
    auto& chosen=decision_.options[decision_.selected];
    const auto start=project(track_,{plant_.pose.x_m,plant_.pose.y_m});
    const auto& end=plan.trajectory.points.back().state;
    double gain=std::fmod(project(track_,{end.x_m,end.y_m}).s_m-start.s_m,track_.length_m);
    if (gain<0) gain+=track_.length_m;
    chosen.trajectory=std::move(plan.trajectory);
    chosen.station_gain_m=gain;
    chosen.clear=true;
    chosen.reason="Clear";
    outcome_.driven_by=controller_->settings().mode;
}

void Simulation::update_diagnostics() {
    const auto& state_=plant_.pose;
    const auto nearest=project(track_,{state_.x_m,state_.y_m});
    double ds=nearest.s_m-previous_s_;
    if (ds < -track_.length_m/2) ds+=track_.length_m;
    if (ds > track_.length_m/2) ds-=track_.length_m;
    total_progress_=std::max(0.0,total_progress_+ds);
    previous_s_=nearest.s_m;
    const auto i=nearest.index,j=(i+1)%plan_.size();
    const double target_sq=(1-nearest.fraction)*plan_[i].speed_mps*plan_[i].speed_mps+
                           nearest.fraction*plan_[j].speed_mps*plan_[j].speed_mps;
    // The reference plan's speed here, which the policy drives toward; a predictive controller plans its own, so while it
    // drives the target is the speed its plan reaches at its next stage.
    // Driving on cones the target is the one the driver set along the path it believes, at its last decision. While a
    // person drives it is the one the planner's action, shown and not driven, aims for from where the car is.
    const bool planned_by_controller=outcome_.driven_by!=ControllerMode::policy;
    diagnostics_.target_speed_mps=planned_by_controller || cone_driver_ ? decision_.trajectory().first_control.target_speed_mps
                                                                        : std::sqrt(std::max(0.0,target_sq));
    diagnostics_.cross_track_error_m=nearest.signed_error_m;
    diagnostics_.progress_m=total_progress_;
    diagnostics_.path_s_m=nearest.s_m;
    diagnostics_.laps=static_cast<int>(std::floor(total_progress_/track_.length_m));
    diagnostics_.nearest_index=i;
    diagnostics_.limiting_index=plan_[i].limiting_index;
    diagnostics_.limiting_reason=plan_[i].reason;
    const auto demanded=demand(model_,plant_,config_,diagnostics_.applied.acceleration_mps2);
    diagnostics_.lateral_acceleration_mps2=demanded.lateral_acceleration_mps2;
    diagnostics_.combined_grip_utilization=demanded.grip_utilization;
    diagnostics_.yaw_rate_radps=plant_.yaw_rate_radps;
    diagnostics_.rear_sideslip_rad=rear_sideslip_rad(plant_);
    diagnostics_.front_slip_angle_rad=demanded.front_slip_angle_rad;
    diagnostics_.rear_slip_angle_rad=demanded.rear_slip_angle_rad;
    diagnostics_.front_peak_slip_angle_rad=demanded.front_peak_slip_angle_rad;
    diagnostics_.rear_peak_slip_angle_rad=demanded.rear_peak_slip_angle_rad;
    const auto balance=handling_balance(demanded);
    diagnostics_.balance=balance.balance;
    diagnostics_.understeer_angle_rad=balance.understeer_angle_rad;
    // Reserve 0.9 m for the visual car's half width; this is a point-model margin,
    // not a swept-body collision calculation.
    diagnostics_.within_track=within_corridor(track_,nearest,0.9);
    // Running faster than the revised reference plan is the policy's braking transient; a predictive controller or a
    // person is not driving to that plan, so either is judged by the corridor and the tires only.
    const bool over_speed=!planned_by_controller && state_.speed_mps>diagnostics_.target_speed_mps+0.75;
    // The kinematic comparison is unchanged, so its recordings reproduce exactly. A dynamic plant
    // can slide, so for it the question is whether a tire has passed its peak slip angle.
    const bool over_grip=demanded.tire_slip_modeled ? !demanded.within_envelope
                                                    : diagnostics_.combined_grip_utilization>1.0+1e-6;
    diagnostics_.within_grip_envelope=!over_grip;
    diagnostics_.plan_valid=diagnostics_.within_track && !over_speed && !over_grip;
    if (!diagnostics_.within_track)
        diagnostics_.validity_reason="Rear axle exceeds track margin";
    else if (over_grip)
        diagnostics_.validity_reason=demanded.tire_slip_modeled ? "Tire slip past its peak; the car is sliding"
                                                                : "Grip demand exceeds kinematic validity; tire slip is not modeled";
    else if (over_speed)
        diagnostics_.validity_reason="Current speed exceeds revised plan; braking transient";
    else
        diagnostics_.validity_reason="Within conservative kinematic limits";
    if (assistance_active()) {
        // Tire limits describe the unassisted car, not the deliberately artificial
        // forces. The true pose still decides track validity and the judge's result.
        diagnostics_.plan_valid=diagnostics_.within_track;
        diagnostics_.validity_reason=diagnostics_.within_track ? "Forgiveness active: assisted handling" : "Rear axle exceeds track margin";
    }
}

} // namespace fd

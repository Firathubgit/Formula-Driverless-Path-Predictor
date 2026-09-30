#pragma once
#include "fd/sensors.hpp"
#include "fd/simulation.hpp"
#include "fd/vehicle_profiles.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace fd {

// Recording contract, schema 17. One run directory holds metadata.json, track.csv,
// plan.csv, plan-rev-N.csv for every revision N >= 1, telemetry.csv, events.csv,
// decisions.csv, trajectories.csv, paths.csv, profiles.csv, commands.csv, measurements.csv, cones.csv,
// perception_frames.csv, detections.csv, missed.csv, beliefs.csv, believed_paths.csv, timing.csv and summary.json.
// RecordingWriter produces those files; load_recording() reads them and cross-checks them as one contract. Schema 17
// records how the run was judged (decision 0032): the rules, the car's footprint, the gates and where the course came
// from in metadata; the course's cones in cones.csv whether or not the car perceived them; and everything the judge saw in
// timing.csv, which the loader requires again by judging the recorded samples on the recorded course. The track is
// written at full precision so that judging it again decides alike. Schema 16 records driving on
// cones (decision 0031): whether the run did, where its driver's readings came from and its cone path settings in
// metadata; what the driver believed at each decision in beliefs.csv, from which state each decision's prediction
// starts; and every path it believed in believed_paths.csv. The loader runs a driver again on the recorded readings and
// frames alone and requires the same beliefs, paths and commands, so every recording of a run on cones shows that its
// decisions came from observations and nothing else. The readings it needs, telemetry.csv, measurements.csv and the
// frames' times, are written to full precision. A run on the known track writes the two headers alone. Schema 15 records simulated cone perception
// (decision 0030): its sensors and seed in metadata, the course's cones in cones.csv, every frame delivered in
// perception_frames.csv, each detection in detections.csv with the cone it was, for evaluation, and each cone in view
// but missed in missed.csv. The loader runs the perception again from the recorded seed and cones over the recorded
// poses and requires the same frames. A run without perception records none. Schema 14 records what the car's own
// instruments measured (decision 0029): the instruments and their seed in metadata, the acceleration the car achieved
// along its body forward axis in telemetry.csv, and measurements.csv, one row per recorded tick holding what each
// channel had delivered by then and when that value was sampled. The loader runs the instruments again from the
// recorded seed over the recorded states and requires the same stream, so a recorded run's measurements reproduce
// exactly. A run without instruments records none, as every older run did; the car drives on its true state either way.
// Schema 13 adds commands.csv: the acceleration
// and steering every plan a predictive controller drove commands at each of its points, the first command being where
// they are at the end of the decision's control interval, so that the plant's response to each plan can be measured
// against it; older runs recorded none. It also adds the dynamic car's centre of gravity height and aerodynamics to
// metadata; older dynamic cars had none. Schema 12 added which controller drove the chosen actions, the policy or model
// predictive contouring control with its settings, and for each decision who drove it and how the predictive controller
// fared; under MPCC the chosen option's prediction is MPCC's plan whenever it drove, and older runs were all driven by
// the policy. Schema 11 added each lattice option's speed profile
// point count and estimated time to decisions.csv and profiles.csv with every profile; older runs compared lattice
// paths by cost and followed the reference plan's speed. Schema 10 added which local planner chose the car's
// actions, each evaluated option's action, lattice path cost terms and path point count, and paths.csv with every
// lattice path; older runs were planned by the five-offset planner, whose options are all offsets. Schema 9 added each
// track sample's distance to
// the corridor's left and right edge, so a run on a racing line keeps its corridor; a centreline records half its
// width both ways and reads back as a centreline. Schema 8 added what the speed plan
// was made from, the performance envelope's fingerprint, and the share of it used (initial_config
// envelope_fraction); older runs were planned with the grip fractions. Schema 7 added the four-wheel
// car's aerodynamics and viscous coupling to metadata. Schema 6 added its centre of gravity height and
// roll balance and four wheel loads; schema 5 the four-wheel car and its wheel speeds; schema 4 the
// steering law and Pure Pursuit's geometric steering; schema 3 the vehicle model and the plant state.
// Schema 6 four-wheel cars had no aerodynamics or coupling, schema 5 ones no load transfer either, older
// runs had no rotating wheels, schema 3 runs were steered by Pure Pursuit, schema 2 runs were all
// kinematic, and schema 1 runs have no decisions.csv or trajectories.csv; all still load and report what
// they did not record. Times are simulation seconds; frames and units follow core.hpp. These files are
// inspectable evidence, not rosbag2 integration, and playing them back never advances the plant. See
// docs/decisions/0002, 0007, 0009, 0010, 0013, 0014, 0015, 0018, 0020, 0022, 0023, 0024, 0025, 0029, 0030, 0031 and 0032.

inline constexpr int recording_schema_version = 18;
inline constexpr int oldest_readable_recording_schema = 1;

struct BuildIdentity {
    std::string source_fingerprint{"unavailable"};
    std::string compiler_id{"unavailable"};
    std::string compiler_version{"unavailable"};
};

struct GripEventRequest { double time_s{}; double grip_mu{}; };

struct RunRequest {
    int requested_laps{1};
    double duration_cap_s{180};
    std::vector<GripEventRequest> grip_events;  // ascending time
    // The physics profile the car was built from (decision 0033), empty when none was named.
    std::string vehicle_profile;
};

struct RecordingMetadata {
    int schema_version{recording_schema_version};
    std::string run_identifier, source_fingerprint, compiler_id, compiler_version;
    std::string model, frame, controller, planner, track_name;
    std::vector<std::string> assumptions;
    double plan_color_deadband_mps2{}, track_length_m{}, track_width_m{}, duration_cap_s{};
    int requested_laps{};
    State initial_state;
    Command initial_requested_command, initial_applied_command;
    std::vector<GripEventRequest> requested_grip_events;
    Config initial_config;
    // Blocked regions the scenario stated for this run. Absent from recordings written
    // before scenarios existed, which read back as an empty list rather than failing.
    std::vector<Obstruction> scenario_obstructions;
    // Schema 3 records the plant and its parameters. Older recordings were made with the kinematic
    // bicycle, which is what they read back as.
    VehicleModel vehicle_model{KinematicBicycle{}};
    // Schema 4 records the steering law, and under MAP the fingerprint of the table generated for the
    // initial configuration; each parameter event regenerated the table. Older recordings were
    // steered by Pure Pursuit.
    SteeringMode steering_mode{SteeringMode::pure_pursuit};
    std::string steering_table_fingerprint;  // empty under Pure Pursuit
    // Schema 8 records what the speed plan was made from, and for the performance envelope the fingerprint of the
    // envelope derived for the initial configuration; each parameter event derived it again. Older runs were planned
    // with the grip fractions.
    SpeedPlanMode speed_plan_mode{SpeedPlanMode::grip_fractions};
    std::string envelope_fingerprint;  // empty under the grip fractions
    // Schema 10 records which local planner chose the car's actions. Older runs were planned by the five-offset planner.
    LocalPlannerMode local_planner_mode{LocalPlannerMode::five_offsets};
    // Schema 12 records which controller drove the chosen actions and its settings. Older runs were driven by the policy.
    ControllerSettings predictive_controller;
    // Schema 14 records the instruments the car carried, their seed and the fingerprint of both; absent for a run
    // without instruments and for every older run. What they measured drove nothing: the car drives on its true state.
    std::optional<SensorSettings> sensors;
    std::string sensor_fingerprint;  // empty without instruments
    // Schema 15 records the simulated cone perception, its seed, and the fingerprint of both with the cones it observed;
    // absent without perception and for every older run. What it detected drove nothing.
    std::optional<PerceptionSettings> perception;
    std::string perception_fingerprint;  // empty without perception
    // Schema 16 records driving on cones: where the driver's readings came from and the cone path settings it planned
    // with. Absent for a run on the known track and for every older run, all of which were.
    // Schema 17 also records the driver's cone memory; a schema 16 driver planned from each frame alone.
    struct ConeDriving { BeliefSource source{BeliefSource::measured}; ConePathSettings settings; ConeMemory memory{.enabled = false}; };
    std::optional<ConeDriving> cone_driving;
    // Schema 17 records how the run was judged: the rules, the car's footprint, the course's gates, where its cones and
    // gates came from, and a fingerprint of both. Absent for every older run, which was not judged.
    struct Competition {
        CompetitionRules rules;
        Footprint footprint;
        std::vector<Gate> gates;
        std::string course;
        std::string course_fingerprint;
    };
    std::optional<Competition> competition;
    // Schema 17: the physics profile the car was built from, when one was named; the loader requires the recorded car and
    // configuration to be that profile's exactly. Empty otherwise and for every older run.
    std::string vehicle_profile;
};

// Exactly the telemetry.csv columns. Quantities the simulation does not record, such
// as the controller lookahead, are deliberately absent rather than invented.
struct RecordedSample {
    double time_s{}, x_m{}, y_m{}, yaw_rad{}, speed_mps{}, steering_rad{};
    double target_speed_mps{}, cross_track_error_m{}, progress_m{};
    int laps{};
    double requested_acceleration_mps2{}, applied_acceleration_mps2{}, requested_steering_rad{};
    double lateral_acceleration_mps2{}, combined_grip_utilization{}, grip_mu{};
    std::uint64_t revision{};
    bool plan_valid{}, within_track{}, control_saturated{};
    std::size_t nearest_index{}, limiting_index{};
    std::string limiting_reason, validity_reason;
    double path_s_m{}, plan_generated_time_s{};
    // Schema 3 columns. For older kinematic recordings the reader derives them: no lateral velocity,
    // yaw rate from speed and steering, and the envelope from grip utilization.
    double lateral_velocity_mps{}, yaw_rate_radps{};
    bool within_grip_envelope{true};
    // Schema 4: Pure Pursuit's steering toward the same target. Under Pure Pursuit it equals the
    // requested steering; under MAP the difference is the table's correction. Older recordings were
    // Pure Pursuit, so the reader copies the requested steering.
    double geometric_steering_rad{};
    // Schema 5: wheel speeds, front left, front right, rear left, rear right. Zero for a car without
    // rotating wheels and for every older recording, none of which had one.
    std::array<double, 4> wheel_speeds_radps{};
    // Schema 6: wheel loads in the same order, including downforce from schema 7. Zero for a car without
    // rotating wheels; a schema 5 four-wheel car had no load transfer, so the reader gives it its static loads.
    std::array<double, 4> wheel_loads_n{};
    // Schema 14: the acceleration the car achieved along its body forward axis over the tick that ended here, which is
    // what an inertial unit reads. For older recordings the reader derives it from the speed the tick before.
    double longitudinal_acceleration_mps2{};
    State state() const { return {x_m, y_m, yaw_rad, speed_mps, steering_rad, time_s}; }
    PlantState plant_state() const { return {state(), lateral_velocity_mps, yaw_rate_radps, wheel_speeds_radps, wheel_loads_n}; }
};

// One predicted point of a recorded alternative: exactly the trajectories.csv columns.
// The rollout also carried yaw, steering and cumulative distance, which are not recorded.
struct RecordedTrajectoryPoint {
    double time_s{}, x_m{}, y_m{}, speed_mps{}, acceleration_mps2{};
};

// One line the planner evaluated, with the reason it was accepted or rejected.
struct RecordedOption {
    double lateral_offset_m{};
    std::optional<double> speed_limit_mps;  // absent when the reference envelope governed
    bool clear{}, within_envelope{};
    double station_gain_m{};
    std::string reason, validity_reason;
    std::vector<RecordedTrajectoryPoint> trajectory;  // starts at the decision's actual state
    // Schema 10: the action, and for a lattice path its points, from the car's projection with stations running on,
    // and its cost. Older runs recorded offsets without paths.
    LocalAction action{LocalAction::offset};
    std::vector<StationOffset> path;
    LatticePathCost path_cost;
    // Schema 11: the speed profile the option was predicted with and the time it estimates, absent when it followed the
    // reference plan. Older runs recorded none.
    std::vector<ProfilePoint> speed_profile;
    std::optional<double> estimated_time_s;
    // Schema 13: when this is a predictive controller's plan that drove, the acceleration and steering it commands at each
    // point of its trajectory; empty otherwise, and for every older run.
    std::vector<Command> commands;
};

// A control decision as the planner made it, at the start of the fixed step it governed.
// Every decision that governed recorded motion is recorded; one superseded before any
// motion, such as the one made on reset, is not.
struct RecordedDecision {
    double time_s{};
    std::size_t selected{};
    bool holding{};
    std::string blocking_identifier;  // empty when no blockage forced a change
    std::string reason;
    Command requested, applied;       // the first command of the chosen line
    // Schema 12: who drove the decision, and when a predictive controller was asked, how it fared and why the policy
    // drove instead if it did. Older runs, and every decision of a policy run, were driven by the policy unasked.
    ControllerMode driven_by{ControllerMode::policy};
    std::optional<ControllerStatus> controller_status;  // absent when no predictive controller was asked
    int controller_iterations{};
    bool controller_restarted{};
    std::string controller_note;
    std::vector<RecordedOption> options;
    const RecordedOption& choice() const { return options.at(selected); }
};

struct PlanRevision {
    std::uint64_t revision{};
    double generated_time_s{};
    std::vector<PlanPoint> points;  // one per track sample
};

struct RunSummary {
    bool completed{};
    int laps{};
    double elapsed_simulation_s{};
    std::size_t samples{};
    double max_cross_track_error_m{}, rms_cross_track_error_m{};
    double max_speed_mps{}, max_combined_grip_utilization{};
    double max_rear_sideslip_rad{};  // schema 3; zero for older, kinematic, recordings
    std::size_t invalid_samples{}, parameter_events{};
    std::vector<double> lap_crossing_times_s;
};

// One recorded tick of what the car's instruments had delivered by then (schema 14), beside the sample of the same
// time. Exactly the measurements.csv columns.
struct RecordedMeasurement {
    double time_s{};
    Measurements measured;
};

// One frame of simulated detections as recorded (schema 15): the tick that delivered it, the frame, and the evaluation's
// truth of it, which cone each detection was and which cones in view were missed.
struct RecordedPerceptionFrame {
    double time_s{};
    PerceptionFrame frame;
    std::vector<std::size_t> source_cone, missed;
};

// What the driver believed at one decision of a run on cones (schema 16): exactly the beliefs.csv columns.
struct RecordedBelief {
    std::size_t decision{};
    State state;                          // time, position, heading, speed and steering believed
    double pose_sampled_at_s{};           // when the pose it was carried forward from was sampled
    std::optional<std::uint64_t> path;    // the believed path followed; none while it had made none
};

// One path the driver believed (schema 16): the frame it came from, the pose that placed that frame on the map, and
// the path with the corridor it believed.
struct RecordedBelievedPath {
    std::uint64_t index{}, frame{};
    State pose;                           // position and heading only
    OpenPath path;
};

struct Recording {
    std::filesystem::path directory;
    RecordingMetadata metadata;
    Track track;
    std::vector<PlanRevision> plans;      // plans[i].revision == i+1
    std::vector<ParameterEvent> events;   // events[i].revision == i+2, strictly ascending time
    std::vector<RecordedSample> samples;  // strictly ascending time; samples[0].time_s == 0
    // Schema 2: every decision that governed recorded motion, strictly ascending time.
    // Empty for schema 1, which did not record decisions.
    std::vector<RecordedDecision> decisions;
    // Schema 14: what the instruments had delivered at each recorded tick, one row per sample and in the same order.
    // Empty for a run without instruments and for every older run.
    std::vector<RecordedMeasurement> measurements;
    // Schema 15: the course's cones, and every frame of simulated detections in the order delivered. Both empty for a
    // run without perception and for every older run.
    std::vector<Cone> cones;
    std::vector<RecordedPerceptionFrame> perception_frames;
    // Schema 16, for a run on cones: one belief per decision, in the same order, and every path believed, in the order
    // made. Both empty for a run on the known track and for every older run.
    std::vector<RecordedBelief> beliefs;
    std::vector<RecordedBelievedPath> believed_paths;
    // Schema 17: everything the judge saw, in order. Empty for every older run.
    std::vector<TimingEvent> timing;
    RunSummary summary;
    bool decisions_recorded() const noexcept { return metadata.schema_version >= 2; }
    bool plant_recorded() const noexcept { return metadata.schema_version >= 3; }
    bool steering_recorded() const noexcept { return metadata.schema_version >= 4; }
    bool wheels_recorded() const noexcept { return metadata.schema_version >= 5; }
    bool loads_recorded() const noexcept { return metadata.schema_version >= 6; }
    bool aerodynamics_recorded() const noexcept { return metadata.schema_version >= 7; }
    bool speed_plan_recorded() const noexcept { return metadata.schema_version >= 8; }
    bool edges_recorded() const noexcept { return metadata.schema_version >= 9; }
    bool local_planner_recorded() const noexcept { return metadata.schema_version >= 10; }
    bool speed_profiles_recorded() const noexcept { return metadata.schema_version >= 11; }
    bool controller_recorded() const noexcept { return metadata.schema_version >= 12; }
    bool commands_recorded() const noexcept { return metadata.schema_version >= 13; }
    bool measurements_recorded() const noexcept { return metadata.schema_version >= 14; }
    // Whether this run carried instruments at all, which only a schema 14 run could.
    bool sensors_recorded() const noexcept { return metadata.sensors.has_value(); }
    bool perception_files_recorded() const noexcept { return metadata.schema_version >= 15; }
    // Whether this run perceived the course's cones, which only a schema 15 run could.
    bool perception_recorded() const noexcept { return metadata.perception.has_value(); }
    bool cone_driving_files_recorded() const noexcept { return metadata.schema_version >= 16; }
    // Whether the run was judged, which every schema 17 run was.
    bool competition_recorded() const noexcept { return metadata.schema_version >= 17; }
    // Whether this run drove on cones, which only a schema 16 run could.
    bool cone_driving_recorded() const noexcept { return metadata.cone_driving.has_value(); }
    const PlanRevision& plan_for_revision(std::uint64_t revision) const;
    // The initial configuration with every accepted change up to and including revision.
    Config config_for_revision(std::uint64_t revision) const;
    // The decision whose outcome a sample at time_s shows, as the live application showed
    // it: the latest made strictly before time_s, because a decision made at time T governs
    // the motion after T. At time zero, the first decision. Null when none was recorded.
    const RecordedDecision* decision_at(double time_s) const;
};

// How far a predictive controller's recorded plans were from what followed them, a given time ahead, over every decision
// it drove whose plan reaches that far (TrackWayFastPlan Phase 6.2, decisions 0024 and 0025): the distance between the
// plan's rear axle and the other's, that distance's component along the plan's direction of travel, positive ahead of the
// plan, and its size across it, and the difference in speed, positive faster than the plan.
struct PredictionError {
    double ahead_s{};
    std::size_t samples{};
    double median_m{}, percentile_95_m{}, worst_m{};
    double median_along_m{}, median_across_m{};
    double median_speed_mps{}, percentile_95_speed_mps{};  // a signed median, and the 95th percentile of the difference's size
};
// One plan's deviation a time ahead: the decision it was made at, the distance, its component along the plan's direction
// of travel and its size across it, and the difference in speed, signed as PredictionError's.
struct PlanDeviation { std::size_t decision{}; double distance_m{}, along_m{}, across_m{}, speed_mps{}; };
// For each time ahead, one deviation for every decision the controller drove whose plan reaches that far, in decision order:
// from where the recorded car went, and from the plant's response to the plan, as the errors below summarise them. Empty
// where the errors are.
std::vector<std::vector<PlanDeviation>> controller_plan_deviations(const Recording& recording, const std::vector<double>& ahead_s);
std::vector<std::vector<PlanDeviation>> controller_model_deviations(const Recording& recording, const std::vector<double>& ahead_s);
// Summarises deviations measured the same time ahead, as the errors below do.
PredictionError prediction_error(double ahead_s, const std::vector<PlanDeviation>& deviations);
// The plan error: against where the recorded car went, wherever the recording reaches that far. The car follows no
// single plan, since each is replaced a control period later, so this measures the prediction model and the replanning
// together. Empty for a run no predictive controller drove.
std::vector<PredictionError> controller_plan_errors(const Recording& recording, const std::vector<double>& ahead_s);
// The model error: against the plant's response to each plan (plant_response), from the recorded state its decision was
// made in, under the configuration it was made with, with this build's plant. Nothing is replanned, so this is the
// prediction model's own error; with the plant the model was reduced from, only the plant's command hold and integration
// step remain. Computing it drives the plant over the horizon once per decision, seconds for a lap: a measurement, not a
// replay, which never advances the plant. Empty for a run no predictive controller drove and for runs before schema 13,
// which did not record the plans' commands.
std::vector<PredictionError> controller_model_errors(const Recording& recording, const std::vector<double>& ahead_s);

// Reads and cross-validates every file. Throws std::runtime_error naming the first
// violated rule; a partially readable directory is never returned.
Recording load_recording(const std::filesystem::path& directory);
// Re-runs the portable planner on the recorded track with each revision's configuration
// and returns the largest absolute speed difference in m/s. A run planned with the performance
// envelope derives the envelope again for each revision, about a quarter second each. A difference
// beyond numerical noise means the recording came from different planner source or inputs than this build.
double plan_reproduction_error_mps(const Recording& recording);
// Re-integrates every recorded step with this build's plant: from the recorded plant state, under
// the recorded acceleration and the governing decision's steering command, with that step's
// configuration. Returns the largest position difference from the next recorded sample in metres,
// or nothing for schema 1, which recorded no steering command. Like the planner reproduction, a
// value beyond rounding means different plant source or parameters, not an invalid file.
std::optional<double> plant_reproduction_error_m(const Recording& recording);
// Under MAP, whether this build derives the recorded table fingerprint from the recorded vehicle
// model and initial configuration; nothing under Pure Pursuit. Reported, not enforced: a different
// generator, or parameters that do not survive twelve significant digits, give another fingerprint
// without making the recording invalid.
std::optional<bool> recorded_steering_table_matches(const Recording& recording);
// For a run planned with the performance envelope, whether this build derives the recorded envelope fingerprint from
// the recorded car and initial configuration; nothing under the grip fractions. Reported, not enforced, like the
// steering table's.
std::optional<bool> recorded_performance_envelope_matches(const Recording& recording);

class RecordingWriter {
public:
    // Rejects a nonempty directory so revisions from different runs cannot mix, and
    // requires a fresh (unstepped, revision 1) simulation. Records the initial sample.
    // The scenario's blocked regions must already be stated; they are part of metadata.
    RecordingWriter(std::filesystem::path directory, const Simulation& simulation,
                    const RunRequest& request, const BuildIdentity& identity);
    // Append the current sample after every step; writes plan-rev-N.csv on a new revision
    // and the decision that governed the step, when that step made one. Rejects a changed
    // scenario and any skipped step.
    void record(const Simulation& simulation);
    // Writes events.csv and summary.json and returns the summary that was written.
    RunSummary finish(const Simulation& simulation);
    const std::filesystem::path& directory() const noexcept { return directory_; }
private:
    std::filesystem::path directory_;
    RunRequest request_;
    std::vector<Obstruction> scenario_;
    std::ofstream telemetry_, decisions_, trajectories_, paths_, profiles_, commands_, measurements_;
    std::ofstream perception_frames_, detections_, missed_, beliefs_, believed_paths_, timing_;
    std::size_t timing_written_{};
    std::uint64_t perception_frames_written_{}, believed_paths_written_{};
    RunSummary summary_;
    double sum_error_sq_{};
    std::uint64_t last_revision_{}, decisions_seen_{}, decisions_written_{};
    int last_laps_{};
    bool finished_{};
};

// Cursor over a loaded recording. Seeking in either direction never runs the plant:
// the active plan and configuration are exactly the revision the recorded sample used,
// so a backward seek restores earlier revisions and hides later parameter changes.
class Playback {
public:
    explicit Playback(Recording recording);
    const Recording& recording() const noexcept { return recording_; }
    std::size_t index() const noexcept { return index_; }
    double time_s() const noexcept { return cursor_time_s_; }
    double duration_s() const noexcept { return recording_.samples.back().time_s; }
    bool at_end() const noexcept { return index_+1 >= recording_.samples.size(); }
    void seek_index(std::size_t index);  // clamped to the final sample
    void seek_time(double time_s);       // the last sample at or before time_s, clamped
    // Moves the cursor forward by playback time. Returns false when the cursor already
    // sits on the final sample and therefore nothing changed.
    bool advance(double dt_s);
    const RecordedSample& sample() const noexcept { return recording_.samples[index_]; }
    const PlanRevision& plan() const;
    const Config& config() const noexcept { return config_; }
    // The recorded decision in force at the cursor sample; null for schema 1.
    const RecordedDecision* decision() const noexcept {
        return decision_index_ < recording_.decisions.size() ? &recording_.decisions[decision_index_] : nullptr;
    }
    // What the recorded plant asked of its tires at the cursor, including axle slip angles, derived
    // from the recorded plant state and applied acceleration through the vehicle seam with the
    // recorded model and the configuration in force. Nothing is advanced or recomputed by a controller.
    Demand demand() const;
    // Accepted parameter changes already in force at the cursor.
    std::size_t applied_event_count() const noexcept { return static_cast<std::size_t>(sample().revision-1); }
private:
    void sync_revision();
    Recording recording_;
    std::size_t index_{};
    double cursor_time_s_{};
    std::uint64_t config_revision_{};
    Config config_;
    // An index rather than a pointer, so a copied Playback never points into another copy.
    std::size_t decision_index_{static_cast<std::size_t>(-1)};
};

}  // namespace fd

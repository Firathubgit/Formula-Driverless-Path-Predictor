#include "fd/recording.hpp"
#include "fd/fingerprint.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace fd {
namespace {

constexpr const char* telemetry_header =
    "time_s,x_m,y_m,yaw_rad,speed_mps,steering_rad,target_speed_mps,cross_track_error_m,progress_m,laps,"
    "requested_acceleration_mps2,applied_acceleration_mps2,requested_steering_rad,lateral_acceleration_mps2,"
    "combined_grip_utilization,grip_mu,revision,plan_valid,within_track,control_saturated,nearest_index,"
    "limiting_index,limiting_reason,validity_reason,path_s_m,plan_generated_time_s";
constexpr const char* plan_header =
    "index,s_m,x_m,y_m,curvature_1pm,speed_mps,acceleration_mps2,limiting_index,reason,revision,generated_time_s";
constexpr const char* track_header = "index,x_m,y_m,s_m,curvature_1pm";
constexpr const char* track_edge_columns = ",left_edge_m,right_edge_m";
constexpr const char* events_header = "time_s,revision,parameter,old_value,new_value";
// Schema 2. One row per evaluated option of every decision; the decision-level columns repeat
// on each of its rows and must agree. speed_limit_mps is "none" when the reference governed.
constexpr const char* decisions_header =
    "decision,time_s,option,selected,lateral_offset_m,speed_limit_mps,clear,within_envelope,station_gain_m,"
    "option_reason,validity_reason,holding,blocking_identifier,decision_reason,"
    "requested_acceleration_mps2,requested_steering_rad,applied_acceleration_mps2,applied_steering_rad";
constexpr const char* trajectories_header = "decision,option,point,time_s,x_m,y_m,speed_mps,acceleration_mps2";
constexpr std::size_t trajectory_columns = 8;
// Schema 10 appends each option's action, its lattice path's point count and the path's cost terms, zero without a
// path, and adds paths.csv: one row per point of every lattice path, in decisions.csv order.
constexpr const char* decisions_lattice_columns =
    ",action,path_points,cost_average_curvature,cost_curvature_range,cost_length,cost_reference_deviation,cost_turns,cost_goal";
constexpr const char* paths_header = "decision,option,point,s_m,offset_m";
// Schema 11 appends each option's speed profile point count and estimated time, "none" without a profile, and adds
// profiles.csv: one row per point of every speed profile, in decisions.csv order.
constexpr const char* decisions_profile_columns = ",profile_points,estimated_time_s";
constexpr const char* profiles_header = "decision,option,point,s_m,distance_m,curvature_1pm,speed_mps";
// Schema 12 appends who drove each decision, the policy or a predictive controller, and when one was asked its status,
// the optimisation steps it took, whether it started again from the policy, and why the policy drove if it did. The
// status is "none" when no predictive controller was asked.
constexpr const char* decisions_controller_columns = ",driven_by,controller_status,controller_iterations,controller_restarted,controller_note";
// Schema 13 adds commands.csv: one row per point of every plan a predictive controller drove, in decisions.csv order, with
// the acceleration and steering the plan commands there.
constexpr const char* commands_header = "decision,option,point,time_s,acceleration_mps2,steering_rad";
// Schema 3 appends the plant state to every telemetry row.
constexpr const char* telemetry_plant_columns = ",lateral_velocity_mps,yaw_rate_radps,within_grip_envelope";
// Schema 4 appends Pure Pursuit's steering toward the chosen target.
constexpr const char* telemetry_steering_columns = ",geometric_steering_rad";
// Schema 5 appends the wheel speeds, in the plant's wheel order.
constexpr const char* telemetry_wheel_columns =
    ",front_left_wheel_speed_radps,front_right_wheel_speed_radps,rear_left_wheel_speed_radps,rear_right_wheel_speed_radps";
constexpr std::array<const char*, 4> wheel_speed_columns{"front_left_wheel_speed_radps", "front_right_wheel_speed_radps",
                                                        "rear_left_wheel_speed_radps", "rear_right_wheel_speed_radps"};
// Schema 6 appends the wheel loads, in the same order.
constexpr const char* telemetry_load_columns =
    ",front_left_wheel_load_n,front_right_wheel_load_n,rear_left_wheel_load_n,rear_right_wheel_load_n";
constexpr std::array<const char*, 4> wheel_load_columns{"front_left_wheel_load_n", "front_right_wheel_load_n",
                                                       "rear_left_wheel_load_n", "rear_right_wheel_load_n"};
// Schema 14 appends the acceleration the car achieved along its body forward axis over the tick that ended at the
// sample, which is what an inertial unit reads; the acceleration commanded is applied_acceleration_mps2.
constexpr const char* telemetry_acceleration_columns = ",longitudinal_acceleration_mps2";
// Schema 14 adds measurements.csv: one row per telemetry sample, holding what each of the car's channels had
// delivered by then and the time it was sampled from the plant. A channel that has delivered nothing is not measured
// and its values are zero. A run without instruments writes the header alone.
constexpr const char* measurements_header =
    "time_s,pose_measured,pose_sampled_at_s,x_m,y_m,yaw_rad,speed_measured,speed_sampled_at_s,speed_mps,"
    "wheel_speeds_measured,wheel_speeds_sampled_at_s,front_left_wheel_speed_radps,front_right_wheel_speed_radps,"
    "rear_left_wheel_speed_radps,rear_right_wheel_speed_radps,imu_measured,imu_sampled_at_s,"
    "longitudinal_acceleration_mps2,lateral_acceleration_mps2,yaw_rate_radps,steering_measured,steering_sampled_at_s,"
    "steering_rad";
// Schema 15 adds the course's cones and simulated cone perception. cones.csv holds every cone at full precision, so
// the perception can be run again from it; perception_frames.csv one row per frame delivered, with the tick that
// delivered it; detections.csv one row per detection, with the cone it was for evaluation; missed.csv one row per cone
// in view that a frame missed. A run without perception writes the headers alone.
constexpr const char* cones_header = "cone,x_m,y_m,colour";
constexpr const char* perception_frames_header = "frame,time_s,sensor,sampled_at_s,delivered_at_s,detections,missed";
constexpr const char* detections_header =
    "frame,detection,x_m,y_m,colour,colour_probability,detection_probability,covariance_xx,covariance_xy,covariance_yy,cone";
constexpr const char* missed_header = "frame,cone";
// Schema 16 adds driving on cones. beliefs.csv holds one row per decision of a run on cones: the state the driver
// believed, when the pose it carried forward was sampled, and the believed path it followed, "none" before it had one.
// believed_paths.csv holds every path it believed, one row per sample, with the frame it came from, the corridor width it
// believed and the pose that placed the frame's detections on the map. A run on the known track writes the headers alone.
constexpr const char* beliefs_header = "decision,time_s,x_m,y_m,yaw_rad,speed_mps,steering_rad,pose_sampled_at_s,path";
constexpr const char* believed_paths_header = "path,frame,point,x_m,y_m,s_m,curvature_1pm,width_m,pose_x_m,pose_y_m,pose_yaw_rad";
// Schema 17 adds timing.csv: everything the judge saw, one row per event, with the lap it fell in, a lap's or a sector's
// time or a penalty's seconds, the cone knocked or "none", where the rear axle was, and the reason for a DNF.
constexpr const char* timing_header = "event,kind,time_s,lap,seconds,cone,x_m,y_m,reason";
constexpr std::array<TimingKind, 10> all_timing_kinds{TimingKind::start, TimingKind::sector, TimingKind::lap, TimingKind::finish,
                                                     TimingKind::cone_hit, TimingKind::off_course, TimingKind::back_on_course,
                                                     TimingKind::stopped, TimingKind::unsafe_stop, TimingKind::dnf};
constexpr std::array<LocalAction, 6> all_actions{LocalAction::offset, LocalAction::straight, LocalAction::pass_left,
                                                LocalAction::pass_right, LocalAction::brake, LocalAction::cone_path};
constexpr double time_tolerance_s = 1e-9;
constexpr double value_tolerance = 1e-9;

[[noreturn]] void fail(const std::string& message) { throw std::runtime_error("Recording contract: "+message); }

bool near(double a, double b, double tolerance) { return std::isfinite(a) && std::isfinite(b) && std::abs(a-b) <= tolerance; }

// ---------------------------------------------------------------- writing

std::ofstream open_output(const std::filesystem::path& path) {
    std::ofstream file(path);
    if (!file) throw std::runtime_error("Cannot create output: "+path.string());
    file << std::setprecision(12);
    return file;
}

std::string json_string(const std::string& value) {
    std::string result{"\""};
    for (const unsigned char c : value) {
        switch (c) {
        case '"': result += "\\\""; break;
        case '\\': result += "\\\\"; break;
        case '\n': result += "\\n"; break;
        case '\r': result += "\\r"; break;
        case '\t': result += "\\t"; break;
        default:
            if (c < 0x20) {
                constexpr char digits[] = "0123456789abcdef";
                result += "\\u00"; result += digits[c>>4]; result += digits[c&0xf];
            } else result += static_cast<char>(c);
        }
    }
    return result+'"';
}

// CSV cells are written unquoted; reasons are fixed identifiers, so a separator inside one
// would silently corrupt the contract. Refuse instead.
const std::string& csv_text(const std::string& value) {
    if (value.empty() || value.find_first_of(",\n\r") != std::string::npos)
        throw std::runtime_error("Recorded text must be nonempty and free of separators: "+value);
    return value;
}

// The blocking identifier is legitimately empty when nothing forced a change.
std::string csv_optional_text(const std::string& value) {
    if (value.find_first_of(",\n\r") != std::string::npos)
        throw std::runtime_error("Recorded text must be free of separators: "+value);
    return value;
}

bool same_scenario(const std::vector<Obstruction>& a, const std::vector<Obstruction>& b) {
    return std::equal(a.begin(), a.end(), b.begin(), b.end(), [](const Obstruction& x, const Obstruction& y) {
        return x.from_s_m == y.from_s_m && x.to_s_m == y.to_s_m && x.from_offset_m == y.from_offset_m &&
               x.to_offset_m == y.to_offset_m && x.identifier == y.identifier;
    });
}

void write_plan(const Simulation& sim, const std::filesystem::path& path) {
    auto file = open_output(path);
    file << plan_header << '\n';
    for (std::size_t i = 0; i < sim.plan().size(); ++i) {
        const auto& p = sim.track().points[i];
        const auto& v = sim.plan()[i];
        file << i << ',' << p.s_m << ',' << p.x_m << ',' << p.y_m << ',' << p.curvature << ','
             << v.speed_mps << ',' << v.acceleration_mps2 << ',' << v.limiting_index << ','
             << csv_text(v.reason) << ',' << sim.diagnostics().revision << ',' << sim.diagnostics().plan_generated_time_s << '\n';
    }
}

std::string tire_json(const TireParameters& t) {
    std::ostringstream out;
    out << std::setprecision(12) << "{\"reference_load_low_n\": " << t.reference_load_low_n
        << ", \"reference_load_high_n\": " << t.reference_load_high_n
        << ", \"peak_friction_longitudinal_low\": " << t.peak_friction_longitudinal_low
        << ", \"peak_friction_longitudinal_high\": " << t.peak_friction_longitudinal_high
        << ", \"peak_friction_lateral_low\": " << t.peak_friction_lateral_low
        << ", \"peak_friction_lateral_high\": " << t.peak_friction_lateral_high
        << ", \"peak_slip_ratio_low\": " << t.peak_slip_ratio_low << ", \"peak_slip_ratio_high\": " << t.peak_slip_ratio_high
        << ", \"peak_slip_angle_low_rad\": " << t.peak_slip_angle_low_rad << ", \"peak_slip_angle_high_rad\": " << t.peak_slip_angle_high_rad
        << ", \"sliding_fraction_longitudinal\": " << t.sliding_fraction_longitudinal
        << ", \"sliding_fraction_lateral\": " << t.sliding_fraction_lateral
        << ", \"minimum_friction\": " << t.minimum_friction << '}';
    return out.str();
}

// One line, so a reader can see the whole car at a glance.
// The controller that drove the chosen actions, with its settings (schema 12).
std::string controller_json(const ControllerSettings& settings) {
    std::ostringstream out;
    out.precision(17);
    out << "{\"mode\": " << json_string(std::string(controller_mode_name(settings.mode)));
    if (settings.mode != ControllerMode::policy) {
        out << ", \"settings\": {";
        for (std::size_t i = 0; i < settings.values.size(); ++i)
            out << (i ? ", " : "") << json_string(settings.values[i].first) << ": " << settings.values[i].second;
        out << '}';
    }
    out << '}';
    return out.str();
}

// The instruments the car carried and the seed every one of their errors came from (schema 14).
std::string sensors_json(const SensorSuite& sensors) {
    const auto& s = sensors.settings();
    const auto channel = [](const char* name, const SensorChannel& c) {
        std::ostringstream out;
        out << std::setprecision(12) << json_string(name) << ": {\"rate_hz\": " << c.rate_hz
            << ", \"dead_time_s\": " << c.dead_time_s << ", \"noise\": " << c.noise << '}';
        return out.str();
    };
    std::ostringstream out;
    out << std::setprecision(12) << "{\"seed\": " << s.seed << ", " << channel("pose", s.pose)
        << ", \"pose_yaw_noise_rad\": " << s.pose_yaw_noise_rad << ", " << channel("speed", s.speed)
        << ", " << channel("wheel_speeds", s.wheel_speeds) << ", " << channel("imu", s.imu)
        << ", \"imu_yaw_rate_noise_radps\": " << s.imu_yaw_rate_noise_radps << ", " << channel("steering", s.steering)
        << ", \"fingerprint\": " << json_string(sensors.fingerprint()) << '}';
    return out.str();
}

// The simulated cone perception and the seed every one of its draws came from (schema 15), at full precision so the
// fingerprint over it reproduces.
std::string perception_json(const Perception& perception) {
    const auto& p = perception.settings();
    std::ostringstream out;
    out << std::setprecision(17) << "{\"seed\": " << p.seed << ", \"sensors\": [";
    for (std::size_t i = 0; i < p.sensors.size(); ++i) {
        const auto& s = p.sensors[i];
        out << (i ? ", " : "") << "{\"name\": " << json_string(s.name) << ", \"mount_x_m\": " << s.mount_x_m
            << ", \"mount_y_m\": " << s.mount_y_m << ", \"mount_yaw_rad\": " << s.mount_yaw_rad << ", \"rate_hz\": " << s.rate_hz
            << ", \"dead_time_s\": " << s.dead_time_s << ", \"min_range_m\": " << s.min_range_m << ", \"max_range_m\": " << s.max_range_m
            << ", \"half_field_of_view_rad\": " << s.half_field_of_view_rad
            << ", \"detection_at_min_range\": " << s.detection_at_min_range << ", \"detection_linear_per_m\": " << s.detection_linear_per_m
            << ", \"detection_quadratic_per_m2\": " << s.detection_quadratic_per_m2 << ", \"detection_floor\": " << s.detection_floor
            << ", \"colour_max_range_m\": " << s.colour_max_range_m << ", \"colour_at_min_range\": " << s.colour_at_min_range
            << ", \"colour_linear_per_m\": " << s.colour_linear_per_m << ", \"colour_quadratic_per_m2\": " << s.colour_quadratic_per_m2
            << ", \"colour_floor\": " << s.colour_floor << ", \"bearing_noise_rad\": " << s.bearing_noise_rad
            << ", \"range_noise_m\": " << s.range_noise_m << ", \"range_relative_noise\": " << s.range_relative_noise
            << ", \"position_noise_m\": " << s.position_noise_m << '}';
    }
    out << "], \"fingerprint\": " << json_string(perception.fingerprint()) << '}';
    return out.str();
}

// The course's cones and gates as text at full precision, whose fingerprint identifies them (schema 17).
std::string course_fingerprint(const std::vector<Cone>& cones, const std::vector<Gate>& gates) {
    std::ostringstream text;
    text << std::setprecision(17);
    for (const auto& c : cones) text << "cone," << c.position.x << ',' << c.position.y << ',' << cone_colour_name(c.colour) << '\n';
    for (const auto& g : gates) text << "gate," << g.left.x << ',' << g.left.y << ',' << g.right.x << ',' << g.right.y << '\n';
    return fingerprint_hex(text.str());
}

// How the run was judged (schema 17), at full precision.
std::string competition_json(const Simulation& sim) {
    const auto& judge = sim.judge();
    const auto& r = judge.rules();
    const auto& f = judge.footprint();
    std::ostringstream out;
    out << std::setprecision(17) << "{\"discipline\": " << json_string(discipline_name(r.discipline))
        << ", \"trackdrive_laps\": " << r.trackdrive_laps << ", \"cone_hit_s\": " << r.cone_hit_s
        << ", \"off_course_s\": " << r.off_course_s << ", \"unsafe_stop_s\": " << r.unsafe_stop_s
        << ", \"off_course_limit_s\": " << r.off_course_limit_s << ", \"start_timeout_s\": " << r.start_timeout_s
        << ", \"autocross_timeout_s\": " << r.autocross_timeout_s << ", \"first_lap_timeout_s\": " << r.first_lap_timeout_s
        << ", \"total_timeout_s\": " << r.total_timeout_s << ", \"stop_zone_m\": " << r.stop_zone_m
        << ", \"stop_within_s\": " << r.stop_within_s << ", \"stopped_below_mps\": " << r.stopped_below_mps
        << ", \"footprint\": {\"wheelbase_m\": " << f.wheelbase_m << ", \"half_track_m\": " << f.half_track_m
        << ", \"body_rear_m\": " << f.body_rear_m << ", \"body_front_m\": " << f.body_front_m
        << ", \"body_half_width_m\": " << f.body_half_width_m << ", \"cone_width_m\": " << f.cone_width_m << "}, \"gates\": [";
    const auto& gates = judge.course().gates;
    for (std::size_t i = 0; i < gates.size(); ++i)
        out << (i ? ", " : "") << '[' << gates[i].left.x << ", " << gates[i].left.y << ", " << gates[i].right.x << ", " << gates[i].right.y << ']';
    out << "], \"course\": " << json_string(sim.course_source()) << ", \"course_fingerprint\": "
        << json_string(course_fingerprint(judge.course().cones, gates)) << '}';
    return out.str();
}

// Where the driver's readings came from and every cone path setting it planned with (schema 16), at full precision.
std::string cone_driving_json(const Simulation& sim) {
    const auto& s = sim.cone_driver()->settings();
    std::ostringstream out;
    out << std::setprecision(17) << "{\"source\": " << json_string(belief_source_name(sim.belief_source()))
        << ", \"max_neighbours\": " << s.max_neighbours << ", \"max_neighbour_distance_m\": " << s.max_neighbour_distance_m
        << ", \"max_distance_to_first_m\": " << s.max_distance_to_first_m << ", \"max_trace_length\": " << s.max_trace_length
        << ", \"directional_angle_rad\": " << s.directional_angle_rad << ", \"absolute_angle_rad\": " << s.absolute_angle_rad
        << ", \"use_unknown_cones\": " << (s.use_unknown_cones ? "true" : "false")
        << ", \"min_track_width_m\": " << s.min_track_width_m << ", \"max_search_range_m\": " << s.max_search_range_m
        << ", \"max_search_angle_rad\": " << s.max_search_angle_rad << ", \"smoothing\": " << s.smoothing
        << ", \"predict_every_m\": " << s.predict_every_m << ", \"max_degree\": " << s.max_degree
        << ", \"max_distance_for_valid_path_m\": " << s.max_distance_for_valid_path_m << ", \"path_length_m\": " << s.path_length_m
        << ", \"horizon_points\": " << s.horizon_points << ", \"memory\": ";
    const auto& m = sim.cone_driver()->memory_settings();
    if (m.enabled) out << "{\"merge_radius_m\": " << m.merge_radius_m << ", \"keep_within_m\": " << m.keep_within_m << '}';
    else out << "null";
    out << '}';
    return out.str();
}

std::string vehicle_json(const VehicleModel& model) {
    if (const auto* four = std::get_if<FourWheelCar>(&model)) {
        std::ostringstream out;
        out << std::setprecision(12) << "{\"kind\": \"four_wheel_car\", \"mass_kg\": " << four->mass_kg
            << ", \"yaw_inertia_kgm2\": " << four->yaw_inertia_kgm2 << ", \"cg_to_rear_m\": " << four->cg_to_rear_m
            << ", \"cg_height_m\": " << four->cg_height_m << ", \"roll_balance_front\": " << four->roll_balance_front
            << ", \"track_front_m\": " << four->track_front_m << ", \"track_rear_m\": " << four->track_rear_m
            << ", \"wheel_radius_m\": " << four->wheel_radius_m << ", \"wheel_inertia_kgm2\": " << four->wheel_inertia_kgm2
            << ", \"drive_front_fraction\": " << four->drive_front_fraction << ", \"max_drive_power_w\": " << four->max_drive_power_w
            << ", \"max_drive_torque_nm\": " << four->max_drive_torque_nm << ", \"brake_bias_front\": " << four->brake_bias_front
            << ", \"max_brake_torque_nm\": " << four->max_brake_torque_nm << ", \"viscous_coupling_nms\": " << four->viscous_coupling_nms
            << ", \"drag_area_m2\": " << four->drag_area_m2 << ", \"downforce_area_m2\": " << four->downforce_area_m2
            << ", \"aero_balance_front\": " << four->aero_balance_front;
        // Schema 18: a car whose drive controller has a top speed (decision 0038) records it; one without has none to record.
        if (std::isfinite(four->max_drive_speed_mps)) out << ", \"max_drive_speed_mps\": " << four->max_drive_speed_mps;
        out << ", \"kinematic_below_mps\": " << four->kinematic_below_mps << ", \"dynamic_above_mps\": " << four->dynamic_above_mps
            << ", \"slip_speed_floor_mps\": " << four->slip_speed_floor_mps << ", \"substep_s\": " << four->substep_s
            << ", \"front_tire\": " << tire_json(four->front_tire) << ", \"rear_tire\": " << tire_json(four->rear_tire) << '}';
        return out.str();
    }
    const auto* car = std::get_if<DynamicSingleTrack>(&model);
    if (!car) return "{\"kind\": \"kinematic_bicycle\"}";
    std::ostringstream out;
    out << std::setprecision(12) << "{\"kind\": \"dynamic_single_track\", \"mass_kg\": " << car->mass_kg
        << ", \"yaw_inertia_kgm2\": " << car->yaw_inertia_kgm2 << ", \"cg_to_rear_m\": " << car->cg_to_rear_m
        << ", \"drive_front_fraction\": " << car->drive_front_fraction
        << ", \"kinematic_below_mps\": " << car->kinematic_below_mps << ", \"dynamic_above_mps\": " << car->dynamic_above_mps
        << ", \"slip_speed_floor_mps\": " << car->slip_speed_floor_mps << ", \"substep_s\": " << car->substep_s
        << ", \"cg_height_m\": " << car->cg_height_m << ", \"drag_area_m2\": " << car->drag_area_m2
        << ", \"downforce_area_m2\": " << car->downforce_area_m2 << ", \"aero_balance_front\": " << car->aero_balance_front
        << ", \"front_tire\": " << tire_json(car->front_tire) << ", \"rear_tire\": " << tire_json(car->rear_tire) << '}';
    return out.str();
}

void write_metadata(const std::filesystem::path& directory, const Simulation& sim,
                    const RunRequest& request, const BuildIdentity& identity) {
    const auto& c = sim.config();
    const auto& state = sim.state();
    const auto& diagnostics = sim.diagnostics();
    auto file = open_output(directory/"metadata.json");
    const auto* four = std::get_if<FourWheelCar>(&sim.vehicle_model());
    file << "{\n  \"schema_version\": " << recording_schema_version << ",\n"
            "  \"run_identifier\": " << json_string(std::filesystem::absolute(directory).generic_string()) << ",\n"
            "  \"source_fingerprint\": " << json_string(identity.source_fingerprint) << ",\n"
            "  \"compiler_id\": " << json_string(identity.compiler_id) << ",\n"
            "  \"compiler_version\": " << json_string(identity.compiler_version) << ",\n"
            "  \"model\": " << json_string(vehicle_model_name(sim.vehicle_model())) << ",\n"
            "  \"frame\": \"right-handed map x/y; z up; yaw CCW; SI units\",\n"
            "  \"assumptions\": ["
            << (!sim.cone_driver() ? "\"known closed centerline\", \"ideal state feedback\", "
                : sim.belief_source() == BeliefSource::ideal
                    ? "\"unknown track: driven on the path believed from simulated cone detections\", \"ideal state feedback\", "
                    : "\"unknown track: driven on the path believed from simulated cone detections\", "
                      "\"state measured by the car's own simulated instruments\", ")
            << (std::holds_alternative<KinematicBicycle>(sim.vehicle_model()) ? "\"no tire slip model\""
                : four ? (four->cg_height_m > 0 ? "\"tire forces with rotating wheels and quasi-static load transfer\""
                                                : "\"tire forces with rotating wheels, without load transfer\"")
                                                                            : "\"tire forces without load transfer or wheel dynamics\"")
            << (four && (four->drag_area_m2 > 0 || four->downforce_area_m2 > 0)
                    ? ", \"aerodynamic drag and downforce in still air\", \"no rolling resistance or actuator delay\"],\n"
                    : ", \"no drag or actuator delay\"],\n")
            <<
            "  \"controller\": " << (sim.cone_driver() ? "\"Along the believed path: " : sim.controller_mode() == ControllerMode::mpcc ? "\"MPCC driving the chosen action; the policy predicting alternatives and driving what MPCC may not: " : "\"")
            << (sim.steering_mode() == SteeringMode::pure_pursuit
                    ? "Pure Pursuit + speed feedback and profile feedforward\""
                    : "MAP: Pure Pursuit target, steady-state steering table + speed feedback and profile feedforward\"")
            << ",\n"
            "  \"planner\": " << (sim.cone_driver() ? (sim.performance_envelope()
                                     ? "\"FaSTTUBe's cone path port; share of the car's performance envelope: lateral cap with forward/backward reachability to rest at the believed path's end\""
                                     : "\"FaSTTUBe's cone path port; curvature cap with forward/backward reachability to rest at the believed path's end\"")
                               : sim.performance_envelope()
                                     ? "\"share of the car's performance envelope: lateral cap with periodic forward/backward reachability\""
                                     : "\"curvature cap with periodic forward/backward reachability\"") << ",\n"
            "  \"plan_color_deadband_mps2\": " << plan_color_deadband_mps2 << ",\n"
            "  \"track_name\": " << json_string(sim.track().name) << ",\n"
            "  \"track_length_m\": " << std::setprecision(17) << sim.track().length_m << std::setprecision(12) << ",\n"
            "  \"track_width_m\": " << sim.track().width_m << ",\n"
            "  \"requested_laps\": " << request.requested_laps << ",\n"
            "  \"duration_cap_s\": " << request.duration_cap_s << ",\n"
            "  \"initial_state\": {\"x_m\": " << state.x_m << ", \"y_m\": " << state.y_m
              << ", \"yaw_rad\": " << state.yaw_rad << ", \"speed_mps\": " << state.speed_mps
              << ", \"steering_rad\": " << state.steering_rad << ", \"time_s\": " << state.time_s << "},\n"
            "  \"initial_requested_command\": {\"acceleration_mps2\": " << diagnostics.requested.acceleration_mps2
              << ", \"steering_rad\": " << diagnostics.requested.steering_rad << "},\n"
            "  \"initial_applied_command\": {\"acceleration_mps2\": " << diagnostics.applied.acceleration_mps2
              << ", \"steering_rad\": " << diagnostics.applied.steering_rad << "},\n"
            "  \"requested_grip_events\": [";
    for (std::size_t i = 0; i < request.grip_events.size(); ++i)
        file << (i ? ", " : "") << "{\"time_s\": " << request.grip_events[i].time_s
             << ", \"grip_mu\": " << request.grip_events[i].grip_mu << '}';
    file << "],\n"
            "  \"scenario_obstructions\": [";
    for (std::size_t i = 0; i < sim.obstructions().size(); ++i) {
        const auto& o = sim.obstructions()[i];
        file << (i ? ", " : "") << "{\"identifier\": " << json_string(o.identifier)
             << ", \"from_s_m\": " << o.from_s_m << ", \"to_s_m\": " << o.to_s_m
             << ", \"from_offset_m\": " << o.from_offset_m << ", \"to_offset_m\": " << o.to_offset_m << '}';
    }
    file << "],\n"
            "  \"vehicle_model\": " << vehicle_json(sim.vehicle_model()) << ",\n"
            "  \"steering\": {\"law\": " << json_string(steering_mode_name(sim.steering_mode()));
    if (const auto* table = sim.steering_table()) file << ", \"table_fingerprint\": " << json_string(table->fingerprint());
    file << "},\n"
            "  \"speed_plan\": {\"mode\": " << json_string(speed_plan_mode_name(sim.speed_plan_mode()));
    if (const auto* envelope = sim.performance_envelope()) file << ", \"envelope_fingerprint\": " << json_string(envelope->fingerprint());
    file << "},\n"
            "  \"local_planner\": " << json_string(std::string(local_planner_mode_name(sim.local_planner_mode()))) << ",\n"
            "  \"predictive_controller\": " << controller_json(sim.controller_settings()) << ",\n";
    // Only a run with instruments records them; what they measured drove nothing.
    if (const auto* sensors = sim.sensors()) file << "  \"sensors\": " << sensors_json(*sensors) << ",\n";
    if (const auto* perception = sim.perception()) file << "  \"perception\": " << perception_json(*perception) << ",\n";
    if (sim.cone_driver()) file << "  \"cone_driving\": " << cone_driving_json(sim) << ",\n";
    file << "  \"competition\": " << competition_json(sim) << ",\n";
    if (!request.vehicle_profile.empty()) file << "  \"vehicle_profile\": " << json_string(request.vehicle_profile) << ",\n";
    file <<
            "  \"initial_config\": {\n"
            "    \"schema_version\": 1,\n"
            "    \"grip_mu\": " << c.grip_mu << ",\n"
            "    \"fixed_dt_s\": " << c.fixed_dt_s << ",\n"
            "    \"control_dt_s\": " << c.control_dt_s << ",\n"
            "    \"wheelbase_m\": " << c.wheelbase_m << ",\n"
            "    \"max_speed_mps\": " << c.max_speed_mps << ",\n"
            "    \"max_steering_rad\": " << c.max_steering_rad << ",\n"
            "    \"max_steering_rate_radps\": " << c.max_steering_rate_radps << ",\n"
            "    \"lateral_grip_fraction\": " << c.lateral_grip_fraction << ",\n"
            "    \"longitudinal_grip_fraction\": " << c.longitudinal_grip_fraction << ",\n"
            "    \"speed_gain\": " << c.speed_gain << ",\n"
            "    \"lookahead_base_m\": " << c.lookahead_base_m << ",\n"
            "    \"lookahead_time_s\": " << c.lookahead_time_s << ",\n"
            "    \"envelope_fraction\": " << c.envelope_fraction << "\n  }\n}\n";
}

#include "contract_io.inc"

// ---------------------------------------------------------------- loading

RecordingMetadata read_metadata(const std::filesystem::path& path) {
    const Json root = read_json(path);
    if (root.type != Json::Type::Object) fail("metadata.json must be an object");
    const std::string where = "metadata";
    RecordingMetadata m;
    m.schema_version = integer_member(root, "schema_version", where);
    if (m.schema_version < oldest_readable_recording_schema || m.schema_version > recording_schema_version)
        fail("unsupported metadata schema_version "+std::to_string(m.schema_version)+"; expected "+
             std::to_string(oldest_readable_recording_schema)+" to "+std::to_string(recording_schema_version));
    m.run_identifier = string_member(root, "run_identifier", where);
    m.source_fingerprint = string_member(root, "source_fingerprint", where);
    m.compiler_id = string_member(root, "compiler_id", where);
    m.compiler_version = string_member(root, "compiler_version", where);
    m.model = string_member(root, "model", where);
    m.frame = string_member(root, "frame", where);
    for (const Json& item : array_member(root, "assumptions", where).array) {
        if (item.type != Json::Type::String) fail("metadata.assumptions must contain strings");
        m.assumptions.push_back(item.string);
    }
    m.controller = string_member(root, "controller", where);
    m.planner = string_member(root, "planner", where);
    m.plan_color_deadband_mps2 = number_member(root, "plan_color_deadband_mps2", where);
    if (m.plan_color_deadband_mps2 < 0) fail("metadata.plan_color_deadband_mps2 must be nonnegative");
    m.track_name = string_member(root, "track_name", where);
    m.track_length_m = number_member(root, "track_length_m", where);
    m.track_width_m = number_member(root, "track_width_m", where);
    if (m.track_length_m <= 0 || m.track_width_m <= 0) fail("metadata track dimensions must be positive");
    m.requested_laps = integer_member(root, "requested_laps", where);
    if (m.requested_laps < 0 || m.requested_laps > 100) fail("metadata.requested_laps must be in 0..100");
    m.duration_cap_s = number_member(root, "duration_cap_s", where);
    if (m.duration_cap_s <= 0) fail("metadata.duration_cap_s must be positive");
    const Json& state = object_member(root, "initial_state", where);
    m.initial_state = {number_member(state, "x_m", "initial_state"), number_member(state, "y_m", "initial_state"),
                       number_member(state, "yaw_rad", "initial_state"), number_member(state, "speed_mps", "initial_state"),
                       number_member(state, "steering_rad", "initial_state"), number_member(state, "time_s", "initial_state")};
    if (m.initial_state.time_s != 0) fail("metadata.initial_state.time_s must be zero");
    if (m.initial_state.speed_mps < 0) fail("metadata.initial_state.speed_mps must be nonnegative");
    const Json& requested = object_member(root, "initial_requested_command", where);
    m.initial_requested_command = {number_member(requested, "acceleration_mps2", "initial_requested_command"),
                                   number_member(requested, "steering_rad", "initial_requested_command")};
    const Json& applied = object_member(root, "initial_applied_command", where);
    m.initial_applied_command = {number_member(applied, "acceleration_mps2", "initial_applied_command"),
                                 number_member(applied, "steering_rad", "initial_applied_command")};
    for (const Json& item : array_member(root, "requested_grip_events", where).array) {
        if (item.type != Json::Type::Object) fail("metadata.requested_grip_events must contain objects");
        m.requested_grip_events.push_back({number_member(item, "time_s", "requested_grip_events"),
                                           number_member(item, "grip_mu", "requested_grip_events")});
    }
    // Optional: recordings predating scenarios simply have no blocked regions.
    if (const Json* blocked = root.find("scenario_obstructions")) {
        if (blocked->type != Json::Type::Array) fail("metadata.scenario_obstructions must be an array");
        for (const Json& item : blocked->array) {
            if (item.type != Json::Type::Object) fail("metadata.scenario_obstructions must contain objects");
            m.scenario_obstructions.push_back({number_member(item, "from_s_m", "scenario_obstructions"),
                                               number_member(item, "to_s_m", "scenario_obstructions"),
                                               number_member(item, "from_offset_m", "scenario_obstructions"),
                                               number_member(item, "to_offset_m", "scenario_obstructions"),
                                               string_member(item, "identifier", "scenario_obstructions")});
        }
    }
    const Json& config = object_member(root, "initial_config", where);
    if (integer_member(config, "schema_version", "initial_config") != 1) fail("initial_config.schema_version must be 1");
    const std::map<std::string, double Config::*> fields = {
        {"grip_mu", &Config::grip_mu}, {"fixed_dt_s", &Config::fixed_dt_s},
        {"control_dt_s", &Config::control_dt_s}, {"wheelbase_m", &Config::wheelbase_m},
        {"max_speed_mps", &Config::max_speed_mps}, {"max_steering_rad", &Config::max_steering_rad},
        {"max_steering_rate_radps", &Config::max_steering_rate_radps},
        {"lateral_grip_fraction", &Config::lateral_grip_fraction},
        {"longitudinal_grip_fraction", &Config::longitudinal_grip_fraction},
        {"speed_gain", &Config::speed_gain}, {"lookahead_base_m", &Config::lookahead_base_m},
        {"lookahead_time_s", &Config::lookahead_time_s}};
    auto config_fields = fields;
    // Schema 8 added the share of the envelope a plan uses; older runs have the default, which they never used.
    if (m.schema_version >= 8) config_fields.emplace("envelope_fraction", &Config::envelope_fraction);
    else if (config.find("envelope_fraction")) fail("initial_config.envelope_fraction requires schema_version 8; the metadata may have been altered");
    for (const auto& key : config.keys)
        if (key != "schema_version" && !config_fields.contains(key)) fail("initial_config has unknown key "+key);
    for (const auto& [key, pointer] : config_fields) m.initial_config.*pointer = number_member(config, key, "initial_config");
    try { validate_config(m.initial_config); }
    catch (const std::exception& error) { fail(std::string("initial_config: ")+error.what()); }
    const auto only_keys = [&](const Json& object, const std::vector<std::string>& allowed, const std::string& where) {
        for (const auto& key : object.keys)
            if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) fail(where+" has unknown key "+key);
    };
    const Json* steering = root.find("steering");
    if (m.schema_version < 4) {
        if (steering) fail("metadata.steering requires schema_version 4; the metadata may have been altered");
    } else {
        if (!steering) fail("metadata lacks \"steering\"");
        if (steering->type != Json::Type::Object) fail("metadata.steering must be an object");
        const std::string law = string_member(*steering, "law", "metadata.steering");
        if (law == steering_mode_name(SteeringMode::pure_pursuit)) {
            only_keys(*steering, {"law"}, "metadata.steering");
            m.steering_mode = SteeringMode::pure_pursuit;
        } else if (law == steering_mode_name(SteeringMode::model_acceleration_pursuit)) {
            only_keys(*steering, {"law", "table_fingerprint"}, "metadata.steering");
            m.steering_mode = SteeringMode::model_acceleration_pursuit;
            m.steering_table_fingerprint = string_member(*steering, "table_fingerprint", "metadata.steering");
            if (m.steering_table_fingerprint.size() != 16 ||
                m.steering_table_fingerprint.find_first_not_of("0123456789abcdef") != std::string::npos)
                fail("metadata.steering.table_fingerprint must be 16 lowercase hexadecimal digits");
        } else {
            fail("metadata.steering law '"+law+"' is not a known steering law");
        }
    }
    const Json* speed_plan = root.find("speed_plan");
    if (m.schema_version < 8) {
        if (speed_plan) fail("metadata.speed_plan requires schema_version 8; the metadata may have been altered");
    } else {
        if (!speed_plan) fail("metadata lacks \"speed_plan\"");
        if (speed_plan->type != Json::Type::Object) fail("metadata.speed_plan must be an object");
        const std::string mode = string_member(*speed_plan, "mode", "metadata.speed_plan");
        if (mode == speed_plan_mode_name(SpeedPlanMode::grip_fractions)) {
            only_keys(*speed_plan, {"mode"}, "metadata.speed_plan");
            m.speed_plan_mode = SpeedPlanMode::grip_fractions;
        } else if (mode == speed_plan_mode_name(SpeedPlanMode::performance_envelope)) {
            only_keys(*speed_plan, {"mode", "envelope_fingerprint"}, "metadata.speed_plan");
            m.speed_plan_mode = SpeedPlanMode::performance_envelope;
            m.envelope_fingerprint = string_member(*speed_plan, "envelope_fingerprint", "metadata.speed_plan");
            if (m.envelope_fingerprint.size() != 16 || m.envelope_fingerprint.find_first_not_of("0123456789abcdef") != std::string::npos)
                fail("metadata.speed_plan.envelope_fingerprint must be 16 lowercase hexadecimal digits");
        } else {
            fail("metadata.speed_plan mode '"+mode+"' is not a known speed plan mode");
        }
    }
    const Json* local_planner = root.find("local_planner");
    if (m.schema_version < 10) {
        if (local_planner) fail("metadata.local_planner requires schema_version 10; the metadata may have been altered");
    } else {
        if (!local_planner) fail("metadata lacks \"local_planner\"");
        const std::string mode = string_member(root, "local_planner", where);
        if (mode == local_planner_mode_name(LocalPlannerMode::lattice)) m.local_planner_mode = LocalPlannerMode::lattice;
        else if (mode == local_planner_mode_name(LocalPlannerMode::five_offsets)) m.local_planner_mode = LocalPlannerMode::five_offsets;
        else fail("metadata.local_planner '"+mode+"' is not a known local planner");
    }
    const Json* sensors = root.find("sensors");
    if (m.schema_version < 14) {
        if (sensors) fail("metadata.sensors requires schema_version 14; the metadata may have been altered");
    } else if (sensors) {
        // A run without instruments records none at all, so the member's absence is not a missing field.
        if (sensors->type != Json::Type::Object) fail("metadata.sensors must be an object");
        SensorSettings settings;
        const double seed = number_member(*sensors, "seed", "sensors");
        if (seed < 0 || seed != std::floor(seed) || seed > 18446744073709549568.0)
            fail("metadata.sensors.seed must be a whole number that a seed can hold");
        settings.seed = static_cast<std::uint64_t>(seed);
        const auto channel = [&](const char* name) {
            const Json& c = object_member(*sensors, name, "sensors");
            only_keys(c, {"rate_hz", "dead_time_s", "noise"}, std::string("metadata.sensors.")+name);
            SensorChannel instrument;
            instrument.rate_hz = number_member(c, "rate_hz", std::string("sensors.")+name);
            instrument.dead_time_s = number_member(c, "dead_time_s", std::string("sensors.")+name);
            instrument.noise = number_member(c, "noise", std::string("sensors.")+name);
            return instrument;
        };
        settings.pose = channel("pose");
        settings.pose_yaw_noise_rad = number_member(*sensors, "pose_yaw_noise_rad", "sensors");
        settings.speed = channel("speed");
        settings.wheel_speeds = channel("wheel_speeds");
        settings.imu = channel("imu");
        settings.imu_yaw_rate_noise_radps = number_member(*sensors, "imu_yaw_rate_noise_radps", "sensors");
        settings.steering = channel("steering");
        only_keys(*sensors, {"seed", "pose", "pose_yaw_noise_rad", "speed", "wheel_speeds", "imu",
                             "imu_yaw_rate_noise_radps", "steering", "fingerprint"}, "metadata.sensors");
        try { validate_sensors(settings); }
        catch (const std::exception& error) { fail(std::string("metadata.sensors: ")+error.what()); }
        m.sensor_fingerprint = string_member(*sensors, "fingerprint", "metadata.sensors");
        if (m.sensor_fingerprint.size() != 16 || m.sensor_fingerprint.find_first_not_of("0123456789abcdef") != std::string::npos)
            fail("metadata.sensors.fingerprint must be 16 lowercase hexadecimal digits");
        // The fingerprint is over the settings themselves, so it catches an instrument altered without it.
        if (SensorSuite(settings).fingerprint() != m.sensor_fingerprint)
            fail("metadata.sensors.fingerprint does not identify the instruments recorded beside it");
        m.sensors = settings;
    }
    const Json* perception = root.find("perception");
    if (m.schema_version < 15) {
        if (perception) fail("metadata.perception requires schema_version 15; the metadata may have been altered");
    } else if (perception) {
        // A run without perception records none at all, so the member's absence is not a missing field.
        if (perception->type != Json::Type::Object) fail("metadata.perception must be an object");
        only_keys(*perception, {"seed", "sensors", "fingerprint"}, "metadata.perception");
        PerceptionSettings settings;
        const double seed = number_member(*perception, "seed", "perception");
        if (seed < 0 || seed != std::floor(seed) || seed > 18446744073709549568.0)
            fail("metadata.perception.seed must be a whole number that a seed can hold");
        settings.seed = static_cast<std::uint64_t>(seed);
        settings.sensors.clear();
        for (const Json& item : array_member(*perception, "sensors", "metadata.perception").array) {
            const std::string sensor_where = "metadata.perception.sensors";
            if (item.type != Json::Type::Object) fail(sensor_where+" must hold objects");
            only_keys(item, {"name", "mount_x_m", "mount_y_m", "mount_yaw_rad", "rate_hz", "dead_time_s", "min_range_m",
                             "max_range_m", "half_field_of_view_rad", "detection_at_min_range", "detection_linear_per_m",
                             "detection_quadratic_per_m2", "detection_floor", "colour_max_range_m", "colour_at_min_range",
                             "colour_linear_per_m", "colour_quadratic_per_m2", "colour_floor", "bearing_noise_rad",
                             "range_noise_m", "range_relative_noise", "position_noise_m"}, sensor_where);
            PerceptionSensorSettings s;
            s.name = string_member(item, "name", sensor_where);
            s.mount_x_m = number_member(item, "mount_x_m", sensor_where);
            s.mount_y_m = number_member(item, "mount_y_m", sensor_where);
            s.mount_yaw_rad = number_member(item, "mount_yaw_rad", sensor_where);
            s.rate_hz = number_member(item, "rate_hz", sensor_where);
            s.dead_time_s = number_member(item, "dead_time_s", sensor_where);
            s.min_range_m = number_member(item, "min_range_m", sensor_where);
            s.max_range_m = number_member(item, "max_range_m", sensor_where);
            s.half_field_of_view_rad = number_member(item, "half_field_of_view_rad", sensor_where);
            s.detection_at_min_range = number_member(item, "detection_at_min_range", sensor_where);
            s.detection_linear_per_m = number_member(item, "detection_linear_per_m", sensor_where);
            s.detection_quadratic_per_m2 = number_member(item, "detection_quadratic_per_m2", sensor_where);
            s.detection_floor = number_member(item, "detection_floor", sensor_where);
            s.colour_max_range_m = number_member(item, "colour_max_range_m", sensor_where);
            s.colour_at_min_range = number_member(item, "colour_at_min_range", sensor_where);
            s.colour_linear_per_m = number_member(item, "colour_linear_per_m", sensor_where);
            s.colour_quadratic_per_m2 = number_member(item, "colour_quadratic_per_m2", sensor_where);
            s.colour_floor = number_member(item, "colour_floor", sensor_where);
            s.bearing_noise_rad = number_member(item, "bearing_noise_rad", sensor_where);
            s.range_noise_m = number_member(item, "range_noise_m", sensor_where);
            s.range_relative_noise = number_member(item, "range_relative_noise", sensor_where);
            s.position_noise_m = number_member(item, "position_noise_m", sensor_where);
            settings.sensors.push_back(s);
        }
        try { validate_perception(settings); }
        catch (const std::exception& error) { fail(std::string("metadata.perception: ")+error.what()); }
        m.perception_fingerprint = string_member(*perception, "fingerprint", "metadata.perception");
        if (m.perception_fingerprint.size() != 16 || m.perception_fingerprint.find_first_not_of("0123456789abcdef") != std::string::npos)
            fail("metadata.perception.fingerprint must be 16 lowercase hexadecimal digits");
        m.perception = settings;
    }
    const Json* competition = root.find("competition");
    if (m.schema_version < 17) {
        if (competition) fail("metadata.competition requires schema_version 17; the metadata may have been altered");
    } else {
        const std::string at = "metadata.competition";
        if (!competition) fail("metadata lacks \"competition\"");
        if (competition->type != Json::Type::Object) fail(at+" must be an object");
        only_keys(*competition, {"discipline", "trackdrive_laps", "cone_hit_s", "off_course_s", "unsafe_stop_s", "off_course_limit_s",
                                 "start_timeout_s", "autocross_timeout_s", "first_lap_timeout_s", "total_timeout_s", "stop_zone_m",
                                 "stop_within_s", "stopped_below_mps", "footprint", "gates", "course", "course_fingerprint"}, at);
        RecordingMetadata::Competition judged;
        auto& r = judged.rules;
        const std::string discipline = string_member(*competition, "discipline", at);
        if (discipline == discipline_name(Discipline::autocross)) r.discipline = Discipline::autocross;
        else if (discipline == discipline_name(Discipline::trackdrive)) r.discipline = Discipline::trackdrive;
        else fail(at+".discipline must be \"autocross\" or \"trackdrive\"");
        r.trackdrive_laps = integer_member(*competition, "trackdrive_laps", at);
        r.cone_hit_s = number_member(*competition, "cone_hit_s", at);
        r.off_course_s = number_member(*competition, "off_course_s", at);
        r.unsafe_stop_s = number_member(*competition, "unsafe_stop_s", at);
        r.off_course_limit_s = number_member(*competition, "off_course_limit_s", at);
        r.start_timeout_s = number_member(*competition, "start_timeout_s", at);
        r.autocross_timeout_s = number_member(*competition, "autocross_timeout_s", at);
        r.first_lap_timeout_s = number_member(*competition, "first_lap_timeout_s", at);
        r.total_timeout_s = number_member(*competition, "total_timeout_s", at);
        r.stop_zone_m = number_member(*competition, "stop_zone_m", at);
        r.stop_within_s = number_member(*competition, "stop_within_s", at);
        r.stopped_below_mps = number_member(*competition, "stopped_below_mps", at);
        const Json* footprint = competition->find("footprint");
        if (!footprint || footprint->type != Json::Type::Object) fail(at+".footprint must be an object");
        only_keys(*footprint, {"wheelbase_m", "half_track_m", "body_rear_m", "body_front_m", "body_half_width_m", "cone_width_m"}, at+".footprint");
        auto& f = judged.footprint;
        f.wheelbase_m = number_member(*footprint, "wheelbase_m", at+".footprint");
        f.half_track_m = number_member(*footprint, "half_track_m", at+".footprint");
        f.body_rear_m = number_member(*footprint, "body_rear_m", at+".footprint");
        f.body_front_m = number_member(*footprint, "body_front_m", at+".footprint");
        f.body_half_width_m = number_member(*footprint, "body_half_width_m", at+".footprint");
        f.cone_width_m = number_member(*footprint, "cone_width_m", at+".footprint");
        try { validate_competition_rules(r); validate_footprint(f); }
        catch (const std::exception& error) { fail(at+": "+error.what()); }
        for (const Json& gate : array_member(*competition, "gates", at).array) {
            if (gate.type != Json::Type::Array || gate.array.size() != 4) fail(at+".gates must hold [left x, left y, right x, right y]");
            std::array<double, 4> v{};
            for (std::size_t k = 0; k < 4; ++k) {
                if (gate.array[k].type != Json::Type::Number) fail(at+".gates must hold numbers");
                v[k] = gate.array[k].number;
            }
            judged.gates.push_back({{v[0], v[1]}, {v[2], v[3]}});
        }
        if (judged.gates.empty()) fail(at+".gates must hold the start line");
        judged.course = string_member(*competition, "course", at);
        judged.course_fingerprint = string_member(*competition, "course_fingerprint", at);
        m.competition = judged;
    }
    const Json* cone_driving = root.find("cone_driving");
    if (m.schema_version < 16) {
        if (cone_driving) fail("metadata.cone_driving requires schema_version 16; the metadata may have been altered");
    } else if (cone_driving) {
        // A run on the known track records none at all, so the member's absence is not a missing field.
        const std::string at = "metadata.cone_driving";
        if (cone_driving->type != Json::Type::Object) fail(at+" must be an object");
        only_keys(*cone_driving, {"source", "max_neighbours", "max_neighbour_distance_m", "max_distance_to_first_m",
                                  "max_trace_length", "directional_angle_rad", "absolute_angle_rad", "use_unknown_cones",
                                  "min_track_width_m", "max_search_range_m", "max_search_angle_rad", "smoothing",
                                  "predict_every_m", "max_degree", "max_distance_for_valid_path_m", "path_length_m",
                                  "horizon_points", "memory"}, at);
        RecordingMetadata::ConeDriving driving;
        const std::string source = string_member(*cone_driving, "source", at);
        if (source == belief_source_name(BeliefSource::measured)) driving.source = BeliefSource::measured;
        else if (source == belief_source_name(BeliefSource::ideal)) driving.source = BeliefSource::ideal;
        else fail(at+".source must be \"measured\" or \"ideal\"");
        auto& s = driving.settings;
        s.max_neighbours = integer_member(*cone_driving, "max_neighbours", at);
        s.max_neighbour_distance_m = number_member(*cone_driving, "max_neighbour_distance_m", at);
        s.max_distance_to_first_m = number_member(*cone_driving, "max_distance_to_first_m", at);
        s.max_trace_length = integer_member(*cone_driving, "max_trace_length", at);
        s.directional_angle_rad = number_member(*cone_driving, "directional_angle_rad", at);
        s.absolute_angle_rad = number_member(*cone_driving, "absolute_angle_rad", at);
        s.use_unknown_cones = bool_member(*cone_driving, "use_unknown_cones", at);
        s.min_track_width_m = number_member(*cone_driving, "min_track_width_m", at);
        s.max_search_range_m = number_member(*cone_driving, "max_search_range_m", at);
        s.max_search_angle_rad = number_member(*cone_driving, "max_search_angle_rad", at);
        s.smoothing = number_member(*cone_driving, "smoothing", at);
        s.predict_every_m = number_member(*cone_driving, "predict_every_m", at);
        s.max_degree = integer_member(*cone_driving, "max_degree", at);
        s.max_distance_for_valid_path_m = number_member(*cone_driving, "max_distance_for_valid_path_m", at);
        s.path_length_m = number_member(*cone_driving, "path_length_m", at);
        s.horizon_points = integer_member(*cone_driving, "horizon_points", at);
        try { validate_cone_path_settings(s); }
        catch (const std::exception& error) { fail(at+": "+error.what()); }
        const Json* memory = cone_driving->find("memory");
        if (m.schema_version < 17) {
            if (memory) fail(at+".memory requires schema_version 17; the metadata may have been altered");
        } else {
            if (!memory) fail(at+" lacks \"memory\"");
            if (memory->type == Json::Type::Object) {
                only_keys(*memory, {"merge_radius_m", "keep_within_m"}, at+".memory");
                driving.memory = {true, number_member(*memory, "merge_radius_m", at+".memory"), number_member(*memory, "keep_within_m", at+".memory")};
                try { validate_cone_memory(driving.memory); }
                catch (const std::exception& error) { fail(at+".memory: "+error.what()); }
            } else if (memory->type != Json::Type::Null) {
                fail(at+".memory must be an object or null");
            }
        }
        if (!m.perception) fail(at+" requires perception: a car drives on the cones it perceives");
        if ((driving.source == BeliefSource::measured) != m.sensors.has_value())
            fail(at+(m.sensors ? ".source says the driver read the true state, but the run had instruments"
                               : ".source says the driver read instruments the run did not have"));
        m.cone_driving = driving;
    }
    const Json* controller = root.find("predictive_controller");
    if (m.schema_version < 12) {
        if (controller) fail("metadata.predictive_controller requires schema_version 12; the metadata may have been altered");
    } else {
        if (!controller) fail("metadata lacks \"predictive_controller\"");
        if (controller->type != Json::Type::Object) fail("metadata.predictive_controller must be an object");
        const std::string mode = string_member(*controller, "mode", "predictive_controller");
        if (mode == controller_mode_name(ControllerMode::policy)) {
            m.predictive_controller.mode = ControllerMode::policy;
            if (controller->keys.size() != 1) fail("metadata.predictive_controller records settings for the policy, which has none");
        } else if (mode == controller_mode_name(ControllerMode::mpcc)) {
            m.predictive_controller.mode = ControllerMode::mpcc;
            const Json& settings = object_member(*controller, "settings", "predictive_controller");
            for (std::size_t i = 0; i < settings.keys.size(); ++i) {
                if (settings.values[i].type != Json::Type::Number) fail("metadata.predictive_controller.settings."+settings.keys[i]+" must be a number");
                m.predictive_controller.values.emplace_back(settings.keys[i], settings.values[i].number);
            }
            for (const char* key : {"stages", "stage_s"})
                if (!settings.find(key)) fail(std::string("metadata.predictive_controller.settings lacks \"")+key+"\"");
        } else {
            fail("metadata.predictive_controller mode '"+mode+"' is not a known controller");
        }
    }
    const Json* vehicle = root.find("vehicle_model");
    if (m.schema_version < 3) {
        if (vehicle) fail("metadata.vehicle_model requires schema_version 3; the metadata may have been altered");
        return m;
    }
    if (!vehicle) fail("metadata lacks \"vehicle_model\"");
    if (vehicle->type != Json::Type::Object) fail("metadata.vehicle_model must be an object");
    const std::string kind = string_member(*vehicle, "kind", "vehicle_model");
    const std::map<std::string, double TireParameters::*> tire_fields = {
        {"reference_load_low_n", &TireParameters::reference_load_low_n}, {"reference_load_high_n", &TireParameters::reference_load_high_n},
        {"peak_friction_longitudinal_low", &TireParameters::peak_friction_longitudinal_low},
        {"peak_friction_longitudinal_high", &TireParameters::peak_friction_longitudinal_high},
        {"peak_friction_lateral_low", &TireParameters::peak_friction_lateral_low},
        {"peak_friction_lateral_high", &TireParameters::peak_friction_lateral_high},
        {"peak_slip_ratio_low", &TireParameters::peak_slip_ratio_low}, {"peak_slip_ratio_high", &TireParameters::peak_slip_ratio_high},
        {"peak_slip_angle_low_rad", &TireParameters::peak_slip_angle_low_rad}, {"peak_slip_angle_high_rad", &TireParameters::peak_slip_angle_high_rad},
        {"sliding_fraction_longitudinal", &TireParameters::sliding_fraction_longitudinal},
        {"sliding_fraction_lateral", &TireParameters::sliding_fraction_lateral}, {"minimum_friction", &TireParameters::minimum_friction}};
    const auto read_tire = [&](const std::string& key, TireParameters& tire) {
        const Json& object = object_member(*vehicle, key, "vehicle_model");
        std::vector<std::string> allowed;
        for (const auto& [name, pointer] : tire_fields) allowed.push_back(name);
        only_keys(object, allowed, "metadata.vehicle_model."+key);
        for (const auto& [name, pointer] : tire_fields) tire.*pointer = number_member(object, name, "vehicle_model."+key);
    };
    if (kind == "four_wheel_car") {
        if (m.schema_version < 5) fail("metadata.vehicle_model four_wheel_car requires schema_version 5; the metadata may have been altered");
        std::map<std::string, double FourWheelCar::*> car_fields = {
            {"mass_kg", &FourWheelCar::mass_kg}, {"yaw_inertia_kgm2", &FourWheelCar::yaw_inertia_kgm2},
            {"cg_to_rear_m", &FourWheelCar::cg_to_rear_m}, {"track_front_m", &FourWheelCar::track_front_m},
            {"track_rear_m", &FourWheelCar::track_rear_m}, {"wheel_radius_m", &FourWheelCar::wheel_radius_m},
            {"wheel_inertia_kgm2", &FourWheelCar::wheel_inertia_kgm2}, {"drive_front_fraction", &FourWheelCar::drive_front_fraction},
            {"max_drive_power_w", &FourWheelCar::max_drive_power_w}, {"max_drive_torque_nm", &FourWheelCar::max_drive_torque_nm},
            {"brake_bias_front", &FourWheelCar::brake_bias_front}, {"max_brake_torque_nm", &FourWheelCar::max_brake_torque_nm},
            {"kinematic_below_mps", &FourWheelCar::kinematic_below_mps}, {"dynamic_above_mps", &FourWheelCar::dynamic_above_mps},
            {"slip_speed_floor_mps", &FourWheelCar::slip_speed_floor_mps}, {"substep_s", &FourWheelCar::substep_s}};
        // Schema 6 added load transfer. A schema 5 car had none: its centre of gravity was, in effect, on the ground.
        const bool loads = m.schema_version >= 6;
        for (const char* key : {"cg_height_m", "roll_balance_front"})
            if (!loads && vehicle->find(key))
                fail(std::string("metadata.vehicle_model.")+key+" requires schema_version 6; the metadata may have been altered");
        if (loads) {
            car_fields.emplace("cg_height_m", &FourWheelCar::cg_height_m);
            car_fields.emplace("roll_balance_front", &FourWheelCar::roll_balance_front);
        }
        // Schema 7 added aerodynamics and the viscous coupling. An older car had neither.
        const bool air = m.schema_version >= 7;
        for (const char* key : {"viscous_coupling_nms", "drag_area_m2", "downforce_area_m2", "aero_balance_front"})
            if (!air && vehicle->find(key))
                fail(std::string("metadata.vehicle_model.")+key+" requires schema_version 7; the metadata may have been altered");
        if (air) {
            car_fields.emplace("viscous_coupling_nms", &FourWheelCar::viscous_coupling_nms);
            car_fields.emplace("drag_area_m2", &FourWheelCar::drag_area_m2);
            car_fields.emplace("downforce_area_m2", &FourWheelCar::downforce_area_m2);
            car_fields.emplace("aero_balance_front", &FourWheelCar::aero_balance_front);
        }
        // Schema 18 added the drive controller's top speed, recorded only by a car that has one (decision 0038).
        const bool controller_speed = m.schema_version >= 18;
        if (!controller_speed && vehicle->find("max_drive_speed_mps"))
            fail("metadata.vehicle_model.max_drive_speed_mps requires schema_version 18; the metadata may have been altered");
        std::vector<std::string> allowed{"kind", "front_tire", "rear_tire"};
        for (const auto& [name, pointer] : car_fields) allowed.push_back(name);
        if (controller_speed) allowed.push_back("max_drive_speed_mps");
        only_keys(*vehicle, allowed, "metadata.vehicle_model");
        FourWheelCar car;
        car.cg_height_m = 0;
        car.drag_area_m2 = 0;
        for (const auto& [name, pointer] : car_fields) car.*pointer = number_member(*vehicle, name, "vehicle_model");
        if (vehicle->find("max_drive_speed_mps")) car.max_drive_speed_mps = number_member(*vehicle, "max_drive_speed_mps", "vehicle_model");
        read_tire("front_tire", car.front_tire);
        read_tire("rear_tire", car.rear_tire);
        m.vehicle_model = car;
    } else if (kind == "kinematic_bicycle") {
        only_keys(*vehicle, {"kind"}, "metadata.vehicle_model");
        m.vehicle_model = KinematicBicycle{};
    } else if (kind == "dynamic_single_track") {
        std::vector<std::string> allowed{"kind", "mass_kg", "yaw_inertia_kgm2", "cg_to_rear_m", "drive_front_fraction", "kinematic_below_mps",
                                         "dynamic_above_mps", "slip_speed_floor_mps", "substep_s", "front_tire", "rear_tire"};
        // Schema 13 added its load transfer and aerodynamics (decision 0025). An older dynamic car had neither.
        const std::array<std::pair<const char*, double DynamicSingleTrack::*>, 4> pitch_and_air{{
            {"cg_height_m", &DynamicSingleTrack::cg_height_m}, {"drag_area_m2", &DynamicSingleTrack::drag_area_m2},
            {"downforce_area_m2", &DynamicSingleTrack::downforce_area_m2}, {"aero_balance_front", &DynamicSingleTrack::aero_balance_front}}};
        for (const auto& [key, pointer] : pitch_and_air) {
            if (m.schema_version < 13 && vehicle->find(key))
                fail(std::string("metadata.vehicle_model.")+key+" of the dynamic car requires schema_version 13; the metadata may have been altered");
            if (m.schema_version >= 13) allowed.push_back(key);
        }
        only_keys(*vehicle, allowed, "metadata.vehicle_model");
        DynamicSingleTrack car;
        const std::string w = "vehicle_model";
        car.mass_kg = number_member(*vehicle, "mass_kg", w);
        car.yaw_inertia_kgm2 = number_member(*vehicle, "yaw_inertia_kgm2", w);
        car.cg_to_rear_m = number_member(*vehicle, "cg_to_rear_m", w);
        car.drive_front_fraction = number_member(*vehicle, "drive_front_fraction", w);
        car.kinematic_below_mps = number_member(*vehicle, "kinematic_below_mps", w);
        car.dynamic_above_mps = number_member(*vehicle, "dynamic_above_mps", w);
        car.slip_speed_floor_mps = number_member(*vehicle, "slip_speed_floor_mps", w);
        car.substep_s = number_member(*vehicle, "substep_s", w);
        if (m.schema_version >= 13)
            for (const auto& [key, pointer] : pitch_and_air) car.*pointer = number_member(*vehicle, key, w);
        read_tire("front_tire", car.front_tire);
        read_tire("rear_tire", car.rear_tire);
        m.vehicle_model = car;
    } else {
        fail("metadata.vehicle_model kind '"+kind+"' is not a known vehicle model");
    }
    try { validate_vehicle(m.vehicle_model, m.initial_config); }
    catch (const std::exception& error) { fail(std::string("metadata.vehicle_model: ")+error.what()); }
    if (m.speed_plan_mode == SpeedPlanMode::performance_envelope && !std::holds_alternative<FourWheelCar>(m.vehicle_model))
        fail("metadata.speed_plan performance envelope needs the four-wheel car; no other model has an envelope");
    if (m.model != vehicle_model_name(m.vehicle_model)) fail("metadata.model does not name the recorded vehicle_model");
    if (const Json* profile = root.find("vehicle_profile")) {
        if (m.schema_version < 17) fail("metadata.vehicle_profile requires schema_version 17; the metadata may have been altered");
        if (profile->type != Json::Type::String) fail("metadata.vehicle_profile must be a string");
        m.vehicle_profile = profile->string;
        const VehicleProfile* named = nullptr;
        try { named = &vehicle_profile(m.vehicle_profile); }
        catch (const std::exception& error) { fail(std::string("metadata.vehicle_profile: ")+error.what()); }
        // The profile's car and its configuration members, exactly as this build's profile gives them.
        const auto* car = std::get_if<FourWheelCar>(&m.vehicle_model);
        const auto same = [](double a, double b) { return std::abs(a-b) <= 1e-9*std::max(1.0, std::abs(b)); };
        const auto same_tire = [&](const TireParameters& a, const TireParameters& b) {
            return same(a.reference_load_low_n, b.reference_load_low_n) && same(a.reference_load_high_n, b.reference_load_high_n) &&
                   same(a.peak_friction_longitudinal_low, b.peak_friction_longitudinal_low) &&
                   same(a.peak_friction_longitudinal_high, b.peak_friction_longitudinal_high) &&
                   same(a.peak_friction_lateral_low, b.peak_friction_lateral_low) && same(a.peak_friction_lateral_high, b.peak_friction_lateral_high) &&
                   same(a.peak_slip_ratio_low, b.peak_slip_ratio_low) && same(a.peak_slip_ratio_high, b.peak_slip_ratio_high) &&
                   same(a.peak_slip_angle_low_rad, b.peak_slip_angle_low_rad) && same(a.peak_slip_angle_high_rad, b.peak_slip_angle_high_rad) &&
                   same(a.sliding_fraction_longitudinal, b.sliding_fraction_longitudinal) &&
                   same(a.sliding_fraction_lateral, b.sliding_fraction_lateral) && same(a.minimum_friction, b.minimum_friction);
        };
        const auto& c = named->car;
        const bool matches = car && same(car->mass_kg, c.mass_kg) && same(car->yaw_inertia_kgm2, c.yaw_inertia_kgm2) &&
            same(car->cg_to_rear_m, c.cg_to_rear_m) && same(car->cg_height_m, c.cg_height_m) && same(car->roll_balance_front, c.roll_balance_front) &&
            same(car->track_front_m, c.track_front_m) && same(car->track_rear_m, c.track_rear_m) && same(car->wheel_radius_m, c.wheel_radius_m) &&
            same(car->wheel_inertia_kgm2, c.wheel_inertia_kgm2) && same(car->drive_front_fraction, c.drive_front_fraction) &&
            same(car->max_drive_power_w, c.max_drive_power_w) && same(car->max_drive_torque_nm, c.max_drive_torque_nm) &&
            same(car->brake_bias_front, c.brake_bias_front) && same(car->max_brake_torque_nm, c.max_brake_torque_nm) &&
            (car->max_drive_speed_mps == c.max_drive_speed_mps || same(car->max_drive_speed_mps, c.max_drive_speed_mps)) &&
            same(car->viscous_coupling_nms, c.viscous_coupling_nms) && same(car->drag_area_m2, c.drag_area_m2) &&
            same(car->downforce_area_m2, c.downforce_area_m2) && same(car->aero_balance_front, c.aero_balance_front) &&
            same_tire(car->front_tire, c.front_tire) && same_tire(car->rear_tire, c.rear_tire) &&
            same(m.initial_config.wheelbase_m, named->wheelbase_m) && same(m.initial_config.max_steering_rad, named->max_steering_rad) &&
            same(m.initial_config.max_speed_mps, named->max_speed_mps);
        if (!matches) fail("metadata.vehicle_profile names "+m.vehicle_profile+", but the recorded car or configuration is not that profile's");
    }
    return m;
}

Track read_track(const std::filesystem::path& path, const RecordingMetadata& metadata) {
    const bool edges = metadata.schema_version >= 9;
    const Table table = read_csv(path, std::string(track_header)+(edges ? track_edge_columns : ""));
    Track track;
    track.name = metadata.track_name;
    track.width_m = metadata.track_width_m;
    track.length_m = metadata.track_length_m;
    bool centred = true;
    for (std::size_t i = 0; i < table.rows.size(); ++i) {
        const RowReader row(table, i);
        if (row.index("index") != i) fail(row.where("index")+" must be sequential");
        track.points.push_back({row.number("x_m"), row.number("y_m"), row.number("s_m"), row.number("curvature_1pm")});
        if (!edges) continue;
        track.left_edge_m.push_back(row.number("left_edge_m"));
        track.right_edge_m.push_back(row.number("right_edge_m"));
        centred = centred && track.left_edge_m.back() == track.width_m/2 && track.right_edge_m.back() == track.width_m/2;
    }
    // A centreline records half its width both ways; it reads back as a centreline, with no edges.
    if (centred) { track.left_edge_m.clear(); track.right_edge_m.clear(); }
    try { validate_track(track); }
    catch (const std::exception& error) { fail(std::string("track.csv: ")+error.what()); }
    return track;
}

std::vector<ParameterEvent> read_events(const std::filesystem::path& path, const Config& initial) {
    const Table table = read_csv(path, events_header);
    std::vector<ParameterEvent> events;
    Config chain = initial;
    for (std::size_t i = 0; i < table.rows.size(); ++i) {
        const RowReader row(table, i);
        ParameterEvent event{row.number("time_s"), row.index("revision"), row.text("parameter"),
                             row.number("old_value"), row.number("new_value")};
        if (event.time_s < 0) fail(row.where("time_s")+" must be nonnegative");
        if (event.revision != i+2) fail(row.where("revision")+" must be contiguous from 2");
        if (i && event.time_s <= events.back().time_s+time_tolerance_s) fail(row.where("time_s")+" must increase; one change per tick");
        if (event.parameter != "grip_mu") fail(row.where("parameter")+" is not a recordable parameter: "+event.parameter);
        if (!near(event.old_value, chain.grip_mu, value_tolerance)) fail(row.where("old_value")+" does not continue the configuration chain");
        if (near(event.new_value, event.old_value, value_tolerance)) fail(row.where("new_value")+" records no change");
        chain.grip_mu = event.new_value;
        try { validate_config(chain); }
        catch (const std::exception& error) { fail(row.where("new_value")+": "+error.what()); }
        events.push_back(std::move(event));
    }
    return events;
}

std::vector<PlanRevision> read_plans(const std::filesystem::path& directory, const Track& track,
                                     const std::vector<ParameterEvent>& events) {
    std::map<std::uint64_t, std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        const std::string name = entry.path().filename().string();
        constexpr std::string_view prefix = "plan-rev-", suffix = ".csv";
        if (name.size() <= prefix.size()+suffix.size() || name.compare(0, prefix.size(), prefix) != 0 ||
            name.compare(name.size()-suffix.size(), suffix.size(), suffix) != 0) continue;
        const std::string digits = name.substr(prefix.size(), name.size()-prefix.size()-suffix.size());
        if (digits.find_first_not_of("0123456789") != std::string::npos || digits.size() > 18) fail("unexpected plan file "+name);
        files[std::stoull(digits)] = entry.path();
    }
    const std::uint64_t expected = events.size()+1;
    for (std::uint64_t revision = 1; revision <= expected; ++revision)
        if (!files.contains(revision)) fail("plan-rev-"+std::to_string(revision)+".csv is missing");
    if (files.size() != expected) fail("a plan revision exists without a recorded parameter event");
    std::vector<PlanRevision> plans;
    for (const auto& [revision, path] : files) {
        const Table table = read_csv(path, plan_header);
        if (table.rows.size() != track.points.size()) fail(table.file+" must hold one row per track sample");
        PlanRevision plan;
        plan.revision = revision;
        plan.generated_time_s = revision == 1 ? 0 : events[revision-2].time_s;
        const std::string& generated_text = table.rows.front()[table.column("generated_time_s")];
        for (std::size_t i = 0; i < table.rows.size(); ++i) {
            const RowReader row(table, i);
            if (row.index("index") != i) fail(row.where("index")+" must be sequential");
            const auto& p = track.points[i];
            if (!near(row.number("s_m"), p.s_m, value_tolerance) || !near(row.number("x_m"), p.x_m, value_tolerance) ||
                !near(row.number("y_m"), p.y_m, value_tolerance) || !near(row.number("curvature_1pm"), p.curvature, value_tolerance))
                fail(row.where("geometry")+" does not match track.csv");
            PlanPoint point{row.number("speed_mps"), row.number("acceleration_mps2"),
                            static_cast<std::size_t>(row.index("limiting_index")), row.text("reason")};
            if (point.speed_mps < 0) fail(row.where("speed_mps")+" must be nonnegative");
            if (point.limiting_index >= track.points.size()) fail(row.where("limiting_index")+" is out of range");
            if (point.reason.empty()) fail(row.where("reason")+" must not be empty");
            if (row.index("revision") != revision) fail(row.where("revision")+" does not match the file name");
            if (row.text("generated_time_s") != generated_text) fail(row.where("generated_time_s")+" must be identical for the whole revision");
            plan.points.push_back(std::move(point));
        }
        if (!near(parse_number(generated_text, table.file+" generated_time_s"), plan.generated_time_s, time_tolerance_s))
            fail(table.file+" generated_time_s does not match the parameter event that produced it");
        plans.push_back(std::move(plan));
    }
    std::ifstream first(files.at(1), std::ios::binary), copy(directory/"plan.csv", std::ios::binary);
    if (!copy) fail("missing file plan.csv");
    if (!std::equal(std::istreambuf_iterator<char>(first), std::istreambuf_iterator<char>(),
                    std::istreambuf_iterator<char>(copy), std::istreambuf_iterator<char>()))
        fail("plan.csv must be identical to plan-rev-1.csv");
    return plans;
}

std::vector<RecordedSample> read_samples(const std::filesystem::path& path, const Recording& r) {
    const bool plant = r.metadata.schema_version >= 3;
    const bool steering = r.metadata.schema_version >= 4;
    const bool wheels = r.metadata.schema_version >= 5;
    const bool loads = r.metadata.schema_version >= 6;
    const bool achieved = r.metadata.schema_version >= 14;
    const auto* four = std::get_if<FourWheelCar>(&r.metadata.vehicle_model);
    const bool rotating = four != nullptr;
    const Table table = read_csv(path, std::string(telemetry_header)+(plant ? telemetry_plant_columns : "")+
                                           (steering ? telemetry_steering_columns : "")+(wheels ? telemetry_wheel_columns : "")+
                                           (loads ? telemetry_load_columns : "")+(achieved ? telemetry_acceleration_columns : ""));
    // Without load transfer every wheel keeps its static load plus its share of downforce at the sample's speed,
    // which is also what a schema 5 four-wheel car had.
    const double weight = four ? four->mass_kg*gravity_mps2 : 0.0;
    const auto downforce_at = [&](double speed) { return four ? aerodynamic_force(*four, speed, 0).downforce_n : 0.0; };
    const auto static_loads = [&](double speed) {
        return four ? quasi_static_wheel_loads(*four, r.metadata.initial_config, 0, 0, downforce_at(speed)) : std::array<double, 4>{};
    };
    if (table.rows.empty()) fail("telemetry.csv holds no samples");
    const bool kinematic = std::holds_alternative<KinematicBicycle>(r.metadata.vehicle_model);
    std::vector<Config> configs;
    for (std::uint64_t revision = 1; revision <= r.plans.size(); ++revision) configs.push_back(r.config_for_revision(revision));
    std::vector<RecordedSample> samples;
    samples.reserve(table.rows.size());
    std::size_t next_event = 0;
    for (std::size_t i = 0; i < table.rows.size(); ++i) {
        const RowReader row(table, i);
        RecordedSample s;
        s.time_s = row.number("time_s"); s.x_m = row.number("x_m"); s.y_m = row.number("y_m");
        s.yaw_rad = row.number("yaw_rad"); s.speed_mps = row.number("speed_mps"); s.steering_rad = row.number("steering_rad");
        s.target_speed_mps = row.number("target_speed_mps"); s.cross_track_error_m = row.number("cross_track_error_m");
        s.progress_m = row.number("progress_m");
        s.laps = static_cast<int>(row.index("laps"));
        s.requested_acceleration_mps2 = row.number("requested_acceleration_mps2");
        s.applied_acceleration_mps2 = row.number("applied_acceleration_mps2");
        s.requested_steering_rad = row.number("requested_steering_rad");
        s.lateral_acceleration_mps2 = row.number("lateral_acceleration_mps2");
        s.combined_grip_utilization = row.number("combined_grip_utilization");
        s.grip_mu = row.number("grip_mu");
        s.revision = row.index("revision");
        s.plan_valid = row.flag("plan_valid"); s.within_track = row.flag("within_track"); s.control_saturated = row.flag("control_saturated");
        s.nearest_index = static_cast<std::size_t>(row.index("nearest_index"));
        s.limiting_index = static_cast<std::size_t>(row.index("limiting_index"));
        s.limiting_reason = row.text("limiting_reason"); s.validity_reason = row.text("validity_reason");
        s.path_s_m = row.number("path_s_m"); s.plan_generated_time_s = row.number("plan_generated_time_s");
        const double kinematic_yaw_rate = s.speed_mps*std::tan(s.steering_rad)/r.metadata.initial_config.wheelbase_m;
        if (plant) {
            s.lateral_velocity_mps = row.number("lateral_velocity_mps");
            s.yaw_rate_radps = row.number("yaw_rate_radps");
            s.within_grip_envelope = row.flag("within_grip_envelope");
            if (kinematic && s.lateral_velocity_mps != 0) fail(row.where("lateral_velocity_mps")+" must be zero for the kinematic bicycle");
            if (kinematic && !near(s.yaw_rate_radps, kinematic_yaw_rate, 1e-8*std::max(1.0, std::abs(kinematic_yaw_rate))))
                fail(row.where("yaw_rate_radps")+" disagrees with the kinematic bicycle's speed and steering");
            if (!s.within_grip_envelope && s.plan_valid) fail(row.where("within_grip_envelope")+" marks a sample outside the grip envelope as valid");
        } else {
            // Older recordings were kinematic runs; derive their plant state the way the plant defines it.
            s.yaw_rate_radps = kinematic_yaw_rate;
            s.within_grip_envelope = !(s.combined_grip_utilization > 1.0+1e-6);
        }
        if (steering) {
            s.geometric_steering_rad = row.number("geometric_steering_rad");
            if (r.metadata.steering_mode == SteeringMode::pure_pursuit && s.geometric_steering_rad != s.requested_steering_rad)
                fail(row.where("geometric_steering_rad")+" must equal requested_steering_rad under Pure Pursuit");
        } else {
            // Older recordings were steered by Pure Pursuit, which requests its geometric steering.
            s.geometric_steering_rad = s.requested_steering_rad;
        }
        if (wheels) {
            for (std::size_t w = 0; w < 4; ++w) {
                const double speed = row.number(wheel_speed_columns[w]);
                if (speed < 0) fail(row.where(wheel_speed_columns[w])+" must not be negative; the wheels do not turn backward");
                if (!rotating && speed != 0) fail(row.where(wheel_speed_columns[w])+" must be zero for a car without rotating wheels");
                s.wheel_speeds_radps[w] = speed;
            }
        }
        if (achieved) {
            s.longitudinal_acceleration_mps2 = row.number("longitudinal_acceleration_mps2");
        } else if (!samples.empty()) {
            // Older recordings did not record it; it is exactly the speed gained over the tick before.
            s.longitudinal_acceleration_mps2 =
                (s.speed_mps-samples.back().speed_mps)/r.config_for_revision(s.revision).fixed_dt_s;
        }
        if (loads) {
            const double pressed = weight+downforce_at(s.speed_mps);
            const bool without_transfer = four && four->cg_height_m == 0;
            const auto expected = without_transfer ? static_loads(s.speed_mps) : std::array<double, 4>{};
            for (std::size_t w = 0; w < 4; ++w) {
                const double load = row.number(wheel_load_columns[w]);
                if (load < 0) fail(row.where(wheel_load_columns[w])+" must not be negative; a lifted wheel carries zero load");
                if (!rotating && load != 0) fail(row.where(wheel_load_columns[w])+" must be zero for a car without rotating wheels");
                if (without_transfer && !near(load, expected[w], value_tolerance*pressed))
                    fail(row.where(wheel_load_columns[w])+" must be the static load and its share of downforce for a car without load transfer");
                s.wheel_loads_n[w] = load;
            }
            if (four) {
                // The quasi-static balance puts exactly the car's weight and its downforce on the wheels; only clamping a
                // lifted wheel adds to it.
                const double sum = s.wheel_loads_n[0]+s.wheel_loads_n[1]+s.wheel_loads_n[2]+s.wheel_loads_n[3];
                const bool lifted = std::any_of(s.wheel_loads_n.begin(), s.wheel_loads_n.end(), [](double load) { return load == 0; });
                if (sum < pressed*(1-value_tolerance) || (!lifted && sum > pressed*(1+value_tolerance)))
                    fail(row.where("front_left_wheel_load_n")+": wheel loads must sum to the car's weight plus its downforce unless a wheel has lifted");
            }
        } else if (four) {
            s.wheel_loads_n = static_loads(s.speed_mps);
        }

        if (i == 0) {
            if (s.time_s != 0) fail("telemetry.csv must start at simulation time zero");
            const State& initial = r.metadata.initial_state;
            if (!near(s.x_m, initial.x_m, value_tolerance) || !near(s.y_m, initial.y_m, value_tolerance) ||
                !near(s.yaw_rad, initial.yaw_rad, value_tolerance) || !near(s.speed_mps, initial.speed_mps, value_tolerance) ||
                !near(s.steering_rad, initial.steering_rad, value_tolerance))
                fail("telemetry.csv first sample does not match metadata initial_state");
            if (s.laps != 0) fail("telemetry.csv must start at lap zero");
        } else {
            if (s.time_s <= samples.back().time_s+time_tolerance_s) fail(row.where("time_s")+" must increase strictly");
            if (s.laps < samples.back().laps || s.laps > samples.back().laps+1) fail(row.where("laps")+" must count single crossings");
        }
        if (s.speed_mps < 0) fail(row.where("speed_mps")+" must be nonnegative");
        // A change accepted at tick boundary T first governs the sample recorded after that tick.
        while (next_event < r.events.size() && r.events[next_event].time_s < s.time_s-time_tolerance_s) ++next_event;
        if (s.revision != next_event+1) fail(row.where("revision")+" disagrees with events.csv timing");
        const PlanRevision& plan = r.plans[s.revision-1];
        if (!near(s.plan_generated_time_s, plan.generated_time_s, time_tolerance_s)) fail(row.where("plan_generated_time_s")+" disagrees with the plan revision");
        if (!near(s.grip_mu, configs[s.revision-1].grip_mu, value_tolerance)) fail(row.where("grip_mu")+" disagrees with the configuration chain");
        if (s.nearest_index >= r.track.points.size() || s.limiting_index >= r.track.points.size()) fail(row.where("index")+" is out of range");
        const PlanPoint& nearest = plan.points[s.nearest_index];
        if (s.limiting_index != nearest.limiting_index || s.limiting_reason != nearest.reason)
            fail(row.where("limiting")+" does not match the plan revision at the nearest sample");
        if (s.validity_reason.empty()) fail(row.where("validity_reason")+" must not be empty");
        if (s.path_s_m < 0 || s.path_s_m >= r.track.length_m+value_tolerance) fail(row.where("path_s_m")+" is outside the track");
        samples.push_back(std::move(s));
    }
    // Once the samples are known to be one tick apart and in order, each one's achieved acceleration must be the speed
    // it gained over the tick before it, which is what an inertial unit along that axis reads.
    if (achieved)
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const double dt = r.config_for_revision(samples[i].revision).fixed_dt_s;
            const double expected = i == 0 ? 0.0 : (samples[i].speed_mps-samples[i-1].speed_mps)/dt;
            if (!near(samples[i].longitudinal_acceleration_mps2, expected, 1e-6*std::max(1.0, std::abs(expected))))
                fail(RowReader(table, i).where("longitudinal_acceleration_mps2")+
                     " is not the speed this sample gained over the tick before it");
        }
    return samples;
}

RunSummary read_summary(const std::filesystem::path& path, const Recording& r) {
    const Json root = read_json(path);
    if (root.type != Json::Type::Object) fail("summary.json must be an object");
    const std::string where = "summary";
    RunSummary s;
    s.completed = bool_member(root, "completed", where);
    s.laps = integer_member(root, "laps", where);
    s.elapsed_simulation_s = number_member(root, "elapsed_simulation_s", where);
    const int samples = integer_member(root, "samples", where);
    const int invalid = integer_member(root, "invalid_samples", where);
    const int events = integer_member(root, "parameter_events", where);
    if (samples < 0 || invalid < 0 || events < 0) fail("summary counts must be nonnegative");
    s.samples = static_cast<std::size_t>(samples);
    s.invalid_samples = static_cast<std::size_t>(invalid);
    s.parameter_events = static_cast<std::size_t>(events);
    s.max_cross_track_error_m = number_member(root, "max_cross_track_error_m", where);
    s.rms_cross_track_error_m = number_member(root, "rms_cross_track_error_m", where);
    s.max_speed_mps = number_member(root, "max_speed_mps", where);
    s.max_combined_grip_utilization = number_member(root, "max_combined_grip_utilization", where);
    const Json* sideslip = root.find("max_rear_sideslip_rad");
    if (r.plant_recorded() != (sideslip != nullptr))
        fail(r.plant_recorded() ? "summary lacks \"max_rear_sideslip_rad\"" : "summary.max_rear_sideslip_rad requires schema_version 3");
    if (sideslip) s.max_rear_sideslip_rad = number_member(root, "max_rear_sideslip_rad", where);
    for (const Json& item : array_member(root, "lap_crossing_times_s", where).array) {
        if (item.type != Json::Type::Number) fail("summary.lap_crossing_times_s must contain numbers");
        s.lap_crossing_times_s.push_back(item.number);
    }
    // The summary is derived from telemetry; recompute it so the files cannot disagree.
    double max_error = 0, max_speed = 0, max_utilization = 0, sum_error_sq = 0, max_sideslip = 0;
    std::size_t invalid_count = 0;
    std::vector<double> crossings;
    for (std::size_t i = 0; i < r.samples.size(); ++i) {
        const auto& sample = r.samples[i];
        max_error = std::max(max_error, std::abs(sample.cross_track_error_m));
        max_speed = std::max(max_speed, sample.speed_mps);
        max_utilization = std::max(max_utilization, sample.combined_grip_utilization);
        max_sideslip = std::max(max_sideslip, std::abs(rear_sideslip_rad(sample.plant_state())));
        sum_error_sq += sample.cross_track_error_m*sample.cross_track_error_m;
        if (!sample.plan_valid) ++invalid_count;
        if (i && sample.laps > r.samples[i-1].laps) crossings.push_back(sample.time_s);
    }
    const auto& last = r.samples.back();
    if (s.samples != r.samples.size()) fail("summary.samples disagrees with telemetry.csv");
    if (s.laps != last.laps) fail("summary.laps disagrees with the final sample");
    if (!near(s.elapsed_simulation_s, last.time_s, time_tolerance_s)) fail("summary.elapsed_simulation_s disagrees with the final sample");
    if (s.invalid_samples != invalid_count) fail("summary.invalid_samples disagrees with telemetry.csv");
    if (s.parameter_events != r.events.size()) fail("summary.parameter_events disagrees with events.csv");
    const auto relative = [](double a, double b, double tolerance) { return std::abs(a-b) <= tolerance*std::max(1.0, std::abs(b)); };
    if (!relative(s.max_cross_track_error_m, max_error, 1e-9)) fail("summary.max_cross_track_error_m disagrees with telemetry.csv");
    if (!relative(s.max_speed_mps, max_speed, 1e-9)) fail("summary.max_speed_mps disagrees with telemetry.csv");
    if (!relative(s.max_combined_grip_utilization, max_utilization, 1e-9)) fail("summary.max_combined_grip_utilization disagrees with telemetry.csv");
    if (!relative(s.max_rear_sideslip_rad, max_sideslip, 1e-9)) fail("summary.max_rear_sideslip_rad disagrees with telemetry.csv");
    if (!relative(s.rms_cross_track_error_m, std::sqrt(sum_error_sq/static_cast<double>(r.samples.size())), 1e-6))
        fail("summary.rms_cross_track_error_m disagrees with telemetry.csv");
    if (s.lap_crossing_times_s.size() != crossings.size()) fail("summary.lap_crossing_times_s disagrees with telemetry.csv");
    for (std::size_t i = 0; i < crossings.size(); ++i)
        if (!near(s.lap_crossing_times_s[i], crossings[i], time_tolerance_s)) fail("summary.lap_crossing_times_s disagrees with telemetry.csv");
    const bool completed = r.metadata.requested_laps == 0 || s.laps >= r.metadata.requested_laps;
    if (s.completed != completed) fail("summary.completed disagrees with requested_laps");
    return s;
}

// ---------------------------------------------------------------- decisions (schema 2)

bool same_number(double a, double b) { return near(a, b, value_tolerance*std::max(1.0, std::abs(b))); }

double field_number(std::string_view text, const std::string& where) {
    double value{};
    const auto [end, error] = std::from_chars(text.data(), text.data()+text.size(), value);
    if (error != std::errc() || end != text.data()+text.size() || !std::isfinite(value))
        fail(where+" is not a finite number: '"+std::string(text)+"'");
    return value;
}
std::uint64_t field_index(std::string_view text, const std::string& where) {
    std::uint64_t value{};
    const auto [end, error] = std::from_chars(text.data(), text.data()+text.size(), value);
    if (text.empty() || error != std::errc() || end != text.data()+text.size()) fail(where+" must be a nonnegative integer");
    return value;
}

// Reads decisions.csv into decisions with their options, without trajectories, and checks what
// each decision says about itself and about the scenario.
std::vector<RecordedDecision> read_decision_rows(const std::filesystem::path& path, const Recording& r) {
    const bool actions = r.local_planner_recorded();
    const bool profiles = r.speed_profiles_recorded();
    const bool controlled = r.controller_recorded();
    const Table table = read_csv(path, std::string(decisions_header)+(actions ? decisions_lattice_columns : "")+(profiles ? decisions_profile_columns : "")+
                                           (controlled ? decisions_controller_columns : ""));
    std::vector<RecordedDecision> decisions;
    std::vector<std::string> identity;  // decision-level cells of the first row, per decision
    std::vector<std::string> decision_columns{"time_s", "holding", "blocking_identifier", "decision_reason",
        "requested_acceleration_mps2", "requested_steering_rad", "applied_acceleration_mps2", "applied_steering_rad"};
    if (controlled)
        for (const char* column : {"driven_by", "controller_status", "controller_iterations", "controller_restarted", "controller_note"})
            decision_columns.push_back(column);
    for (std::size_t i = 0; i < table.rows.size(); ++i) {
        const RowReader row(table, i);
        const std::uint64_t index = row.index("decision");
        const std::uint64_t option = row.index("option");
        if (index == decisions.size()) {
            if (option != 0) fail(row.where("option")+" must start at 0 for each decision");
            RecordedDecision decision;
            decision.time_s = row.number("time_s");
            decision.holding = row.flag("holding");
            decision.blocking_identifier = row.text("blocking_identifier");
            decision.reason = row.text("decision_reason");
            decision.requested = {row.number("requested_acceleration_mps2"), row.number("requested_steering_rad")};
            decision.applied = {row.number("applied_acceleration_mps2"), row.number("applied_steering_rad")};
            decision.selected = static_cast<std::size_t>(-1);
            if (controlled) {
                const std::string& driven = row.text("driven_by");
                if (driven == controller_mode_name(ControllerMode::policy)) decision.driven_by = ControllerMode::policy;
                else if (driven == controller_mode_name(ControllerMode::mpcc)) decision.driven_by = ControllerMode::mpcc;
                else fail(row.where("driven_by")+" '"+driven+"' is not a known controller");
                const std::string& status = row.text("controller_status");
                if (status != "none") {
                    const std::array<ControllerStatus, 3> known{ControllerStatus::solved, ControllerStatus::solved_inaccurate, ControllerStatus::not_solved};
                    const auto found = std::find_if(known.begin(), known.end(), [&](ControllerStatus s) { return controller_status_name(s) == status; });
                    if (found == known.end()) fail(row.where("controller_status")+" '"+status+"' is not a known status");
                    decision.controller_status = *found;
                }
                const std::uint64_t iterations = row.index("controller_iterations");
                if (iterations > 1000) fail(row.where("controller_iterations")+" must be at most 1000");
                decision.controller_iterations = static_cast<int>(iterations);
                decision.controller_restarted = row.flag("controller_restarted");
                decision.controller_note = row.text("controller_note");
            }
            decisions.push_back(std::move(decision));
            identity.clear();
            for (const auto& column : decision_columns) identity.push_back(row.text(column));
        } else if (decisions.empty() || index != decisions.size()-1) {
            fail(row.where("decision")+" must be contiguous from 0");
        } else {
            if (option != decisions.back().options.size()) fail(row.where("option")+" must be contiguous within its decision");
            for (std::size_t c = 0; c < decision_columns.size(); ++c)
                if (row.text(decision_columns[c]) != identity[c]) fail(row.where(decision_columns[c])+" differs between rows of one decision");
        }
        auto& decision = decisions.back();
        RecordedOption recorded;
        recorded.lateral_offset_m = row.number("lateral_offset_m");
        if (row.text("speed_limit_mps") != "none") {
            recorded.speed_limit_mps = row.number("speed_limit_mps");
            if (*recorded.speed_limit_mps < 0) fail(row.where("speed_limit_mps")+" must be nonnegative");
        }
        recorded.clear = row.flag("clear");
        recorded.within_envelope = row.flag("within_envelope");
        recorded.station_gain_m = row.number("station_gain_m");
        recorded.reason = row.text("option_reason");
        recorded.validity_reason = row.text("validity_reason");
        if (recorded.reason.empty() || recorded.validity_reason.empty()) fail(row.where("reason")+" must not be empty");
        if (recorded.clear && !recorded.within_envelope) fail(row.where("clear")+" marks a line clear that left the model envelope");
        if (actions) {
            const std::string& action = row.text("action");
            const auto known = std::find_if(all_actions.begin(), all_actions.end(), [&](LocalAction a) { return local_action_name(a) == action; });
            if (known == all_actions.end()) fail(row.where("action")+" '"+action+"' is not a known action");
            recorded.action = *known;
            const std::uint64_t points = row.index("path_points");
            if (points == 1 || points > 100000) fail(row.where("path_points")+" must be 0 or a path of at least two points");
            recorded.path.resize(static_cast<std::size_t>(points));  // paths.csv fills it
            auto& cost = recorded.path_cost;
            const std::array<std::pair<const char*, double*>, 6> terms{{
                {"cost_average_curvature", &cost.edges.average_curvature}, {"cost_curvature_range", &cost.edges.curvature_range},
                {"cost_length", &cost.edges.length}, {"cost_reference_deviation", &cost.edges.reference_deviation},
                {"cost_turns", &cost.turns}, {"cost_goal", &cost.goal}}};
            for (const auto& [column, value] : terms) {
                *value = row.number(column);
                if (*value < 0) fail(row.where(column)+" must be nonnegative");
                if (points == 0 && *value != 0) fail(row.where(column)+" costs a lattice path the option does not have");
            }
        }
        if (profiles) {
            const std::uint64_t points = row.index("profile_points");
            if (points == 1 || points > 1000000) fail(row.where("profile_points")+" must be 0 or a profile of at least two points");
            recorded.speed_profile.resize(static_cast<std::size_t>(points));  // profiles.csv fills it
            if (row.text("estimated_time_s") != "none") {
                recorded.estimated_time_s = row.number("estimated_time_s");
                if (*recorded.estimated_time_s < 0) fail(row.where("estimated_time_s")+" must be nonnegative");
            }
            if (points == 0 && recorded.estimated_time_s) fail(row.where("estimated_time_s")+" estimates a time without a speed profile");
        }
        if (row.flag("selected")) {
            if (decision.selected != static_cast<std::size_t>(-1)) fail(row.where("selected")+": exactly one option per decision is selected");
            decision.selected = decision.options.size();
        }
        decision.options.push_back(std::move(recorded));
    }

    for (std::size_t d = 0; d < decisions.size(); ++d) {
        const auto& decision = decisions[d];
        const std::string where = "decisions.csv decision "+std::to_string(d);
        if (decision.selected == static_cast<std::size_t>(-1)) fail(where+": exactly one option per decision is selected");
        if (d && decision.time_s <= decisions[d-1].time_s+time_tolerance_s) fail(where+" time must increase strictly");
        if (decision.reason.empty()) fail(where+" decision_reason must not be empty");
        const auto& chosen = decision.choice();
        if (decision.holding && !chosen.speed_limit_mps) fail(where+" holds for a blockage without a speed limit");
        if (r.cone_driving_recorded()) {
            // On cones the driver's one action is to follow the path it believes, by the policy, with nothing blocking.
            if (decision.options.size() != 1 || chosen.action != LocalAction::cone_path)
                fail(where+" on cones must follow the believed path, its one option");
            if (decision.holding || !decision.blocking_identifier.empty() || !chosen.path.empty() || !chosen.speed_profile.empty())
                fail(where+" on cones holds, names a blockage or follows a lattice path, which a car on cones cannot");
            if (decision.driven_by != ControllerMode::policy) fail(where+" on cones was driven by a predictive controller");
            continue;
        }
        if (std::any_of(decision.options.begin(), decision.options.end(), [](const RecordedOption& o) { return o.action == LocalAction::cone_path; }))
            fail(where+" follows a believed path on a run that did not drive on cones");
        // A blocked line may not be taken while a clear one was offered. When nothing is clear the car must be holding for
        // the blockage it named; a return, which names none, then keeps driving, since there is nothing to hold for.
        const bool any_clear = std::any_of(decision.options.begin(), decision.options.end(),
                                           [](const RecordedOption& o) { return o.clear; });
        if (!decision.holding && !chosen.clear && any_clear) fail(where+" chose a line that was not clear while a clear one was offered");
        if (!decision.holding && !chosen.clear && !decision.blocking_identifier.empty())
            fail(where+" drove on with no clear line past '"+decision.blocking_identifier+"'");
        // Returning to the reference along the lattice weighs the reference line against a path with nothing blocking; from
        // schema 11 either may be taken.
        const bool returning = r.speed_profiles_recorded()
            ? std::any_of(decision.options.begin(), decision.options.end(), [](const RecordedOption& o) { return o.action == LocalAction::straight && !o.path.empty(); })
            : chosen.action == LocalAction::straight && !chosen.path.empty();
        if (decision.options.size() > 1 && decision.blocking_identifier.empty() && !returning)
            fail(where+" evaluated alternatives without naming a blockage");
        const bool lattice = r.metadata.local_planner_mode == LocalPlannerMode::lattice;
        for (std::size_t o = 0; o < decision.options.size(); ++o) {
            const auto& option = decision.options[o];
            const std::string at = where+" option "+std::to_string(o)+" ("+std::string(local_action_name(option.action))+")";
            if (lattice == (option.action == LocalAction::offset))
                fail(at+" is not an action the "+std::string(local_planner_mode_name(r.metadata.local_planner_mode))+" planner takes");
            const bool passing = option.action == LocalAction::pass_left || option.action == LocalAction::pass_right;
            if (passing && option.path.empty()) fail(at+" passes without a lattice path");
            if (passing && decision.blocking_identifier.empty()) fail(at+" passes without naming the blockage");
            if (!passing && option.action != LocalAction::straight && !option.path.empty()) fail(at+" has a lattice path its action does not follow");
            if (option.action == LocalAction::brake && (o != decision.selected || !decision.holding)) fail(at+" brakes without holding for the blockage");
        }
        if (lattice && decision.holding && chosen.action != LocalAction::brake) fail(where+" holds for a blockage without braking");
        if (r.controller_recorded()) {
            const auto& controller = r.metadata.predictive_controller;
            const bool asked = decision.controller_status.has_value();
            if (controller.mode == ControllerMode::policy) {
                if (decision.driven_by != ControllerMode::policy || asked || decision.controller_iterations != 0 ||
                    decision.controller_restarted || !decision.controller_note.empty())
                    fail(where+" reports a predictive controller the run did not have");
            } else if (decision.driven_by == controller.mode) {
                if (!asked || *decision.controller_status == ControllerStatus::not_solved || decision.controller_iterations < 1)
                    fail(where+" was driven by "+std::string(controller_mode_name(controller.mode))+" without a solved plan");
                if (decision.holding) fail(where+" holds for a blockage but was not driven by the policy");
                if (!decision.controller_note.empty()) fail(where+" explains a policy decision it did not make");
                if (!chosen.clear || !chosen.within_envelope) fail(where+" was driven by a plan that is not clear");
            } else {
                if (decision.driven_by != ControllerMode::policy) fail(where+" was driven by a controller the run did not have");
                if (decision.controller_note.empty()) fail(where+" was driven by the policy without saying why");
                if (decision.holding == asked) fail(where+(decision.holding ? " asked the predictive controller to hold for a blockage"
                                                                              : " did not ask the predictive controller"));
            }
        }
        if (r.speed_profiles_recorded()) {
            // Every lattice path is compared by the time of its own speed profile, and the car takes the fastest clear one.
            std::size_t timed = 0;
            double fastest = std::numeric_limits<double>::infinity();
            for (std::size_t o = 0; o < decision.options.size(); ++o) {
                const auto& option = decision.options[o];
                const std::string at = where+" option "+std::to_string(o)+" ("+std::string(local_action_name(option.action))+")";
                if (!lattice && !option.speed_profile.empty()) fail(at+" has a speed profile the five-offset planner does not make");
                if (!option.path.empty() && option.speed_profile.empty()) fail(at+" follows a lattice path without its speed profile");
                if (option.clear && option.estimated_time_s) { ++timed; fastest = std::min(fastest, *option.estimated_time_s); }
            }
            // It may keep the previous decision's choice, the same action along a path or not, within
            // lattice_switch_margin_s of the fastest; the first recorded decision's previous choice was not recorded.
            const bool kept = d == 0 || (decisions[d-1].choice().action == chosen.action &&
                                         decisions[d-1].choice().path.empty() == chosen.path.empty());
            const double allowed = fastest+(kept ? lattice_switch_margin_s : 0.0)+1e-9*std::max(1.0, fastest);
            if (!decision.holding && timed >= 2 && (!chosen.clear || !chosen.estimated_time_s || *chosen.estimated_time_s > allowed))
                fail(where+" did not take the fastest clear action, or keep its previous choice within "+std::to_string(lattice_switch_margin_s)+" s of it");
        }
        if (!decision.blocking_identifier.empty()) {
            const auto& scenario = r.metadata.scenario_obstructions;
            if (std::none_of(scenario.begin(), scenario.end(), [&](const Obstruction& o) { return o.identifier == decision.blocking_identifier; }))
                fail(where+" names blocking identifier '"+decision.blocking_identifier+"', which is not a stated scenario obstruction");
        }
    }
    return decisions;
}

// Decisions happen at the start of a fixed step, on every control tick and whenever an accepted
// parameter change replans between ticks. Returns, per decision, the telemetry sample it started from.
std::vector<std::size_t> check_decision_schedule(const Recording& r) {
    const double dt = r.metadata.initial_config.fixed_dt_s;
    const auto control_ticks = std::llround(r.metadata.initial_config.control_dt_s/dt);
    std::vector<std::size_t> starts;
    std::size_t event = 0;
    for (std::size_t i = 0; i+1 < r.samples.size(); ++i) {
        const double t = r.samples[i].time_s;
        while (event < r.events.size() && r.events[event].time_s < t-time_tolerance_s) ++event;
        const bool replan = event < r.events.size() && near(r.events[event].time_s, t, time_tolerance_s);
        if (std::llround(t/dt)%control_ticks == 0 || replan) starts.push_back(i);
    }
    for (std::size_t d = 0; d < std::min(starts.size(), r.decisions.size()); ++d)
        if (!near(r.decisions[d].time_s, r.samples[starts[d]].time_s, time_tolerance_s))
            fail("decisions.csv decision "+std::to_string(d)+" at "+std::to_string(r.decisions[d].time_s)+
                 " s does not follow the control schedule; expected "+std::to_string(r.samples[starts[d]].time_s)+" s");
    if (starts.size() != r.decisions.size())
        fail("decisions.csv does not follow the control schedule: telemetry requires "+std::to_string(starts.size())+
             " decisions, found "+std::to_string(r.decisions.size()));
    return starts;
}

// Streams trajectories.csv into the decisions' options. It is by far the largest file, so rows are
// parsed in place rather than held as text. Every option gets exactly one trajectory, in order.
void read_trajectories(const std::filesystem::path& path, Recording& r, const std::vector<std::size_t>& starts) {
    std::ifstream file(path, std::ios::binary);
    if (!file) fail("missing file trajectories.csv");
    std::string line;
    if (!std::getline(file, line)) fail("trajectories.csv is empty");
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != trajectories_header) fail("trajectories.csv header does not match the schema 2 contract");
    const double dt = r.metadata.initial_config.fixed_dt_s;
    const auto horizon_ticks = static_cast<long long>(std::ceil(prediction_horizon_s/dt-1e-10));
    std::size_t decision = 0, option = 0, expected_point = 0, number = 1;
    std::array<std::string_view, trajectory_columns> fields;
    const auto close_option = [&](std::size_t d, std::size_t o) {
        const auto& points = r.decisions[d].options[o].trajectory;
        const std::string where = "trajectories.csv decision "+std::to_string(d)+" option "+std::to_string(o);
        if (points.size() < 2) fail(where+" needs at least two points");
        if (std::llround((points.back().time_s-r.decisions[d].time_s)/dt) != horizon_ticks)
            fail(where+" does not end at the prediction horizon");
    };
    while (std::getline(file, line)) {
        ++number;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const std::string where = "trajectories.csv line "+std::to_string(number);
        std::size_t count = 0, begin = 0;
        for (std::size_t k = 0; k <= line.size(); ++k) {
            if (k < line.size() && line[k] != ',') continue;
            if (count == trajectory_columns) fail(where+" has the wrong number of fields");
            fields[count++] = std::string_view(line).substr(begin, k-begin);
            begin = k+1;
        }
        if (count != trajectory_columns) fail(where+" has the wrong number of fields");
        const std::uint64_t d = field_index(fields[0], where+" decision");
        const std::uint64_t o = field_index(fields[1], where+" option");
        const std::uint64_t p = field_index(fields[2], where+" point");
        if (p == 0 && expected_point != 0) {
            // A new trajectory begins: the previous one must be complete.
            close_option(decision, option);
            if (option+1 < r.decisions[decision].options.size()) { ++option; }
            else { ++decision; option = 0; }
            expected_point = 0;
        }
        if (decision >= r.decisions.size() || d != decision || o != option || p != expected_point)
            fail(where+" expected decision "+std::to_string(decision)+" option "+std::to_string(option)+" point "+
                 std::to_string(expected_point)+" to follow decisions.csv, found "+std::to_string(d)+" "+
                 std::to_string(o)+" "+std::to_string(p));
        RecordedTrajectoryPoint point{field_number(fields[3], where+" time_s"), field_number(fields[4], where+" x_m"),
                                      field_number(fields[5], where+" y_m"), field_number(fields[6], where+" speed_mps"),
                                      field_number(fields[7], where+" acceleration_mps2")};
        if (point.speed_mps < 0) fail(where+" speed_mps must be nonnegative");
        auto& points = r.decisions[decision].options[option].trajectory;
        const auto& decided = r.decisions[decision];
        if (p == 0 && r.cone_driving_recorded()) {
            const State& believed = r.beliefs[decision].state;
            if (!near(point.time_s, decided.time_s, time_tolerance_s) || !same_number(point.x_m, believed.x_m) ||
                !same_number(point.y_m, believed.y_m) || !same_number(point.speed_mps, believed.speed_mps))
                fail(where+" does not start at the state the driver believed at the decision (beliefs.csv)");
        } else if (p == 0) {
            const RecordedSample& state = r.samples[starts[decision]];
            if (!near(point.time_s, decided.time_s, time_tolerance_s) || !same_number(point.x_m, state.x_m) ||
                !same_number(point.y_m, state.y_m) || !same_number(point.speed_mps, state.speed_mps))
                fail(where+" does not start at the recorded state the decision was made from");
        } else {
            const double ticks = (point.time_s-decided.time_s)/dt;
            if (point.time_s <= points.back().time_s+time_tolerance_s || std::abs(ticks-std::round(ticks)) > 1e-6)
                fail(where+" time_s must increase on the fixed-tick grid");
        }
        points.push_back(point);
        expected_point = static_cast<std::size_t>(p)+1;
    }
    if (r.decisions.empty()) {
        if (number > 1) fail("trajectories.csv holds points for decisions that decisions.csv lacks");
        return;
    }
    if (expected_point == 0) fail("trajectories.csv lacks the trajectory of decision 0 option 0");
    close_option(decision, option);
    if (decision+1 != r.decisions.size() || option+1 != r.decisions[decision].options.size())
        fail("trajectories.csv ends before the trajectory of every option in decisions.csv");
}

// Reads paths.csv into the lattice paths decisions.csv counted, in its order. Every path starts at the car's projection
// onto the recorded track at the state the decision was made from, and its stations increase, running on past the seam.
void read_paths(const std::filesystem::path& path, Recording& r, const std::vector<std::size_t>& starts) {
    const Table table = read_csv(path, paths_header);
    const double length = r.track.length_m;
    std::size_t row = 0;
    for (std::size_t d = 0; d < r.decisions.size(); ++d) {
        auto& decision = r.decisions[d];
        for (std::size_t o = 0; o < decision.options.size(); ++o) {
            auto& points = decision.options[o].path;
            const std::string expected = "decision "+std::to_string(d)+" option "+std::to_string(o);
            for (std::size_t p = 0; p < points.size(); ++p, ++row) {
                if (row >= table.rows.size()) fail("paths.csv ends before the path of "+expected+" that decisions.csv counts");
                const RowReader reader(table, row);
                if (reader.index("decision") != d || reader.index("option") != o || reader.index("point") != p)
                    fail(reader.where("decision")+" expected "+expected+" point "+std::to_string(p)+" to follow decisions.csv path_points");
                points[p] = {reader.number("s_m"), reader.number("offset_m")};
                if (p && points[p].s_m <= points[p-1].s_m) fail(reader.where("s_m")+" must increase along the path");
            }
            if (points.empty()) continue;
            const auto& state = r.samples[starts[d]];
            const auto here = project(r.track, {state.x_m, state.y_m});
            const double apart = std::abs(std::fmod(points.front().s_m, length)-here.s_m);
            if (points.front().s_m < 0 || std::min(apart, length-apart) > 1e-6 || std::abs(points.front().offset_m-here.signed_error_m) > 1e-6)
                fail("paths.csv "+expected+" does not start at the recorded state the decision was made from");
        }
    }
    if (row != table.rows.size()) fail("paths.csv holds points that decisions.csv does not count");
}

// Reads profiles.csv into the speed profiles decisions.csv counted, in its order. Every profile starts at the car's
// station and actual speed at the recorded state the decision was made from, at distance zero; its stations and distances
// increase and its speeds are nonnegative; every profile of one decision ends at the same station; and each option's
// estimated time is its profile's.
void read_profiles(const std::filesystem::path& path, Recording& r, const std::vector<std::size_t>& starts) {
    const Table table = read_csv(path, profiles_header);
    const double length = r.track.length_m;
    std::size_t row = 0;
    for (std::size_t d = 0; d < r.decisions.size(); ++d) {
        auto& decision = r.decisions[d];
        std::optional<double> horizon;
        for (std::size_t o = 0; o < decision.options.size(); ++o) {
            auto& option = decision.options[o];
            auto& points = option.speed_profile;
            const std::string expected = "decision "+std::to_string(d)+" option "+std::to_string(o);
            for (std::size_t p = 0; p < points.size(); ++p, ++row) {
                if (row >= table.rows.size()) fail("profiles.csv ends before the profile of "+expected+" that decisions.csv counts");
                const RowReader reader(table, row);
                if (reader.index("decision") != d || reader.index("option") != o || reader.index("point") != p)
                    fail(reader.where("decision")+" expected "+expected+" point "+std::to_string(p)+" to follow decisions.csv profile_points");
                points[p] = {reader.number("s_m"), reader.number("distance_m"), reader.number("curvature_1pm"), reader.number("speed_mps")};
                if (points[p].speed_mps < 0) fail(reader.where("speed_mps")+" must be nonnegative");
                if (p && (points[p].s_m <= points[p-1].s_m || points[p].distance_m <= points[p-1].distance_m))
                    fail(reader.where("s_m")+" and distance_m must increase along the profile");
            }
            if (points.empty()) continue;
            const auto& state = r.samples[starts[d]];
            const auto here = project(r.track, {state.x_m, state.y_m});
            const double apart = std::abs(std::fmod(points.front().s_m, length)-here.s_m);
            if (points.front().s_m < 0 || std::min(apart, length-apart) > 1e-6 || points.front().distance_m != 0 ||
                !same_number(points.front().speed_mps, state.speed_mps))
                fail("profiles.csv "+expected+" does not start at the recorded state the decision was made from");
            if (!horizon) horizon = points.back().s_m;
            else if (std::abs(points.back().s_m-*horizon) > 1e-6) fail("profiles.csv "+expected+" does not end at the same horizon as its decision's other profiles");
            const double time = profile_time(points);
            if (std::isfinite(time) != option.estimated_time_s.has_value() ||
                (option.estimated_time_s && !near(*option.estimated_time_s, time, 1e-9*std::max(1.0, time))))
                fail("decisions.csv "+expected+" estimated_time_s disagrees with its speed profile in profiles.csv");
        }
    }
    if (row != table.rows.size()) fail("profiles.csv holds points that decisions.csv does not count");
}

// A decision a predictive controller drove records the controller's plan as its chosen prediction: exactly its stages,
// one stage apart, from the decision's state (schema 12). A plan of any other shape was not the controller's.
void check_controller_plans(const Recording& r) {
    const auto& controller = r.metadata.predictive_controller;
    if (!r.controller_recorded() || controller.mode == ControllerMode::policy) return;
    const auto setting = [&](const char* key) {
        return std::find_if(controller.values.begin(), controller.values.end(), [&](const auto& v) { return v.first == key; })->second;
    };
    const auto stages = static_cast<std::size_t>(std::llround(setting("stages")));
    const double stage = setting("stage_s");
    for (std::size_t d = 0; d < r.decisions.size(); ++d) {
        const auto& decision = r.decisions[d];
        if (decision.driven_by != controller.mode) continue;
        const auto& plan = decision.choice().trajectory;
        bool shaped = plan.size() == stages+1;
        for (std::size_t k = 1; shaped && k < plan.size(); ++k) shaped = std::abs(plan[k].time_s-plan[k-1].time_s-stage) < 1e-6;
        if (!shaped)
            fail("decisions.csv decision "+std::to_string(d)+" was driven by a plan that is not the controller's "+std::to_string(stages)+
                 " stages of "+std::to_string(stage)+" s");
    }
}

// The fixed tick at which the control interval a decision at this time governs ends: the next regular control tick.
std::uint64_t interval_end_tick(const Recording& r, double time_s) {
    const double dt = r.metadata.initial_config.fixed_dt_s;
    const auto control_ticks = static_cast<std::uint64_t>(std::llround(r.metadata.initial_config.control_dt_s/dt));
    return (static_cast<std::uint64_t>(std::llround(time_s/dt))/control_ticks+1)*control_ticks;
}

// Reads commands.csv (schema 13) into the plans a predictive controller drove, in decisions.csv order: one command at every
// point of each such plan and none anywhere else, at the plan's own times, steering within the steering limit. The command
// that drove is where the plan's commands are at the end of the decision's control interval.
void read_commands(const std::filesystem::path& path, Recording& r) {
    const Table table = read_csv(path, commands_header);
    const double dt = r.metadata.initial_config.fixed_dt_s;
    std::size_t row = 0;
    for (std::size_t d = 0; d < r.decisions.size(); ++d) {
        auto& decision = r.decisions[d];
        if (decision.driven_by == ControllerMode::policy) continue;
        auto& option = decision.options[decision.selected];
        const std::string expected = "decision "+std::to_string(d)+" option "+std::to_string(decision.selected);
        std::vector<double> times;
        const double limit = r.config_for_revision(r.samples[static_cast<std::size_t>(std::llround(decision.time_s/dt))].revision).max_steering_rad;
        for (std::size_t p = 0; p < option.trajectory.size(); ++p, ++row) {
            if (row >= table.rows.size()) fail("commands.csv ends before the command at every point of the plan that drove "+expected);
            const RowReader reader(table, row);
            if (reader.index("decision") != d || reader.index("option") != decision.selected || reader.index("point") != p)
                fail(reader.where("decision")+" expected "+expected+" point "+std::to_string(p)+
                     ": commands.csv holds one command at every point of each plan a predictive controller drove, and no others");
            if (!near(reader.number("time_s"), option.trajectory[p].time_s, time_tolerance_s))
                fail(reader.where("time_s")+" is not the time of the plan's point in trajectories.csv");
            const Command command{reader.number("acceleration_mps2"), reader.number("steering_rad")};
            if (std::abs(command.steering_rad) > limit+value_tolerance) fail(reader.where("steering_rad")+" exceeds the steering limit");
            option.commands.push_back(command);
            times.push_back(option.trajectory[p].time_s);
        }
        const auto first = planned_command(times, option.commands, static_cast<double>(interval_end_tick(r, decision.time_s))*dt);
        if (!near(first.acceleration_mps2, decision.applied.acceleration_mps2, 1e-8*std::max(1.0, std::abs(first.acceleration_mps2))) ||
            !near(first.steering_rad, decision.applied.steering_rad, 1e-8))
            fail("decisions.csv "+expected+" applied a first command that is not its plan's at the end of its control interval in commands.csv");
    }
    if (row != table.rows.size())
        fail("commands.csv holds commands beyond the plans a predictive controller drove, starting at line "+std::to_string(row+2));
}

// Every recorded command equals the first command of the decision that governed the step.
void check_commands_against_decisions(const Recording& r) {
    std::size_t d = 0;
    for (std::size_t i = 1; i < r.samples.size(); ++i) {
        const double started = r.samples[i-1].time_s;
        while (d+1 < r.decisions.size() && r.decisions[d+1].time_s <= started+time_tolerance_s) ++d;
        const auto& decision = r.decisions[d];
        const auto& s = r.samples[i];
        if (!same_number(s.requested_acceleration_mps2, decision.requested.acceleration_mps2) ||
            !same_number(s.requested_steering_rad, decision.requested.steering_rad) ||
            !same_number(s.applied_acceleration_mps2, decision.applied.acceleration_mps2))
            fail("telemetry.csv line "+std::to_string(i+2)+" command disagrees with decisions.csv decision "+
                 std::to_string(d)+", which governed that step");
    }
}

}  // namespace

// ---------------------------------------------------------------- Recording

const PlanRevision& Recording::plan_for_revision(std::uint64_t revision) const {
    if (revision < 1 || revision > plans.size()) throw std::out_of_range("Unknown plan revision "+std::to_string(revision));
    return plans[revision-1];
}

Config Recording::config_for_revision(std::uint64_t revision) const {
    if (revision < 1 || revision > events.size()+1) throw std::out_of_range("Unknown configuration revision "+std::to_string(revision));
    Config config = metadata.initial_config;
    for (const auto& event : events) {
        if (event.revision > revision) break;
        if (event.parameter != "grip_mu") throw std::runtime_error("Unsupported recorded parameter "+event.parameter);
        config.grip_mu = event.new_value;
    }
    return config;
}

const RecordedDecision* Recording::decision_at(double time_s) const {
    if (decisions.empty() || !std::isfinite(time_s)) return nullptr;
    const auto first_not_before = std::lower_bound(decisions.begin(), decisions.end(), time_s-time_tolerance_s,
        [](const RecordedDecision& decision, double t) { return decision.time_s < t; });
    if (first_not_before == decisions.begin())
        return decisions.front().time_s <= time_s+time_tolerance_s ? &decisions.front() : nullptr;
    return &*(first_not_before-1);
}

// measurements.csv (schema 14): what the car's instruments had delivered at each recorded tick, one row per sample.
// Reading it runs the same instruments again from the recorded seed over the recorded states and requires the same
// stream, so a recording carries its measurements and the means of reproducing them.
std::vector<RecordedMeasurement> read_measurements(const std::filesystem::path& path, const Recording& r) {
    const Table table = read_csv(path, measurements_header);
    if (!r.metadata.sensors) {
        if (!table.rows.empty()) fail("measurements.csv holds rows for a run whose metadata records no instruments");
        return {};
    }
    if (table.rows.size() != r.samples.size())
        fail("measurements.csv holds "+std::to_string(table.rows.size())+" rows for "+std::to_string(r.samples.size())+
             " telemetry samples; there is one measurement per recorded tick");
    SensorSuite again(*r.metadata.sensors);
    std::vector<RecordedMeasurement> measurements;
    measurements.reserve(table.rows.size());
    const auto channel = [&](const RowReader& row, const char* measured, const char* sampled) {
        SensorReading reading;
        reading.measured = row.flag(measured);
        reading.sampled_at_s = row.number(sampled);
        return reading;
    };
    for (std::size_t i = 0; i < table.rows.size(); ++i) {
        const RowReader row(table, i);
        const auto& sample = r.samples[i];
        RecordedMeasurement recorded;
        recorded.time_s = row.number("time_s");
        if (!near(recorded.time_s, sample.time_s, time_tolerance_s))
            fail(row.where("time_s")+" does not match the telemetry sample of the same row");
        auto& m = recorded.measured;
        m.pose = channel(row, "pose_measured", "pose_sampled_at_s");
        m.x_m = row.number("x_m"); m.y_m = row.number("y_m"); m.yaw_rad = row.number("yaw_rad");
        m.speed = channel(row, "speed_measured", "speed_sampled_at_s");
        m.speed_mps = row.number("speed_mps");
        m.wheel_speeds = channel(row, "wheel_speeds_measured", "wheel_speeds_sampled_at_s");
        for (std::size_t w = 0; w < 4; ++w) m.wheel_speeds_radps[w] = row.number(wheel_speed_columns[w]);
        m.imu = channel(row, "imu_measured", "imu_sampled_at_s");
        m.longitudinal_acceleration_mps2 = row.number("longitudinal_acceleration_mps2");
        m.lateral_acceleration_mps2 = row.number("lateral_acceleration_mps2");
        m.yaw_rate_radps = row.number("yaw_rate_radps");
        m.steering = channel(row, "steering_measured", "steering_sampled_at_s");
        m.steering_rad = row.number("steering_rad");
        // Nothing can be sampled from the future, and a channel that has delivered nothing carries no values.
        for (const auto& [reading, name] : std::array<std::pair<const SensorReading*, const char*>, 5>{{
                 {&m.pose, "pose"}, {&m.speed, "speed"}, {&m.wheel_speeds, "wheel_speeds"}, {&m.imu, "imu"},
                 {&m.steering, "steering"}}}) {
            if (reading->measured && reading->sampled_at_s > recorded.time_s+time_tolerance_s)
                fail(row.where(std::string(name)+"_sampled_at_s")+" is later than the tick that read it");
            if (!reading->measured && reading->sampled_at_s != 0)
                fail(row.where(std::string(name)+"_sampled_at_s")+" records a sample time for a channel that has delivered nothing");
        }
        again.observe({sample.plant_state(), sample.longitudinal_acceleration_mps2, sample.lateral_acceleration_mps2},
                      sample.time_s);
        const auto& expected = again.measurements();
        const auto agrees = [&](const char* name, double recorded_value, double expected_value) {
            if (!near(recorded_value, expected_value, value_tolerance*std::max(1.0, std::abs(expected_value))))
                fail(row.where(name)+" is not what these instruments measure from the recorded seed and states");
        };
        const auto agrees_reading = [&](const char* name, const char* time_name, const SensorReading& was, const SensorReading& is) {
            if (was.measured != is.measured)
                fail(row.where(name)+" disagrees with what these instruments had delivered by this tick");
            if (was.measured) agrees(time_name, was.sampled_at_s, is.sampled_at_s);
        };
        agrees_reading("pose_measured", "pose_sampled_at_s", m.pose, expected.pose);
        agrees_reading("speed_measured", "speed_sampled_at_s", m.speed, expected.speed);
        agrees_reading("wheel_speeds_measured", "wheel_speeds_sampled_at_s", m.wheel_speeds, expected.wheel_speeds);
        agrees_reading("imu_measured", "imu_sampled_at_s", m.imu, expected.imu);
        agrees_reading("steering_measured", "steering_sampled_at_s", m.steering, expected.steering);
        if (m.pose.measured) {
            agrees("x_m", m.x_m, expected.x_m);
            agrees("y_m", m.y_m, expected.y_m);
            agrees("yaw_rad", m.yaw_rad, expected.yaw_rad);
        }
        if (m.speed.measured) agrees("speed_mps", m.speed_mps, expected.speed_mps);
        if (m.wheel_speeds.measured)
            for (std::size_t w = 0; w < 4; ++w) agrees(wheel_speed_columns[w], m.wheel_speeds_radps[w], expected.wheel_speeds_radps[w]);
        if (m.imu.measured) {
            agrees("longitudinal_acceleration_mps2", m.longitudinal_acceleration_mps2, expected.longitudinal_acceleration_mps2);
            agrees("lateral_acceleration_mps2", m.lateral_acceleration_mps2, expected.lateral_acceleration_mps2);
            agrees("yaw_rate_radps", m.yaw_rate_radps, expected.yaw_rate_radps);
        }
        if (m.steering.measured) agrees("steering_rad", m.steering_rad, expected.steering_rad);
        measurements.push_back(std::move(recorded));
    }
    return measurements;
}

// cones.csv and the perception files (schema 15). The cones must be the ones the recorded fingerprint names; the frames
// must be exactly what the recorded perception delivers when run again over the recorded poses, tick by tick.
std::vector<Cone> read_cones(const std::filesystem::path& path, const Recording& r) {
    const Table table = read_csv(path, cones_header);
    if (!r.metadata.perception && !r.competition_recorded()) {
        if (!table.rows.empty()) fail("cones.csv holds cones for a run whose metadata records no perception");
        return {};
    }
    std::vector<Cone> cones;
    for (std::size_t i = 0; i < table.rows.size(); ++i) {
        const RowReader row(table, i);
        if (row.index("cone") != i) fail(row.where("cone")+" must number the cones in order from zero");
        Cone cone;
        cone.position = {row.number("x_m"), row.number("y_m")};
        const std::string colour = row.text("colour");
        bool known = false;
        for (const auto c : {ConeColour::blue, ConeColour::yellow, ConeColour::orange, ConeColour::big_orange})
            if (colour == cone_colour_name(c)) { cone.colour = c; known = true; }
        if (!known) fail(row.where("colour")+" is not a cone colour");
        cones.push_back(cone);
    }
    if (cones.empty()) fail("cones.csv holds no cones for a run that perceived or was judged on them");
    if (r.metadata.perception) {
        try {
            if (Perception(cones, *r.metadata.perception).fingerprint() != r.metadata.perception_fingerprint)
                fail("metadata.perception.fingerprint does not identify the perception and cones recorded beside it");
        } catch (const std::invalid_argument& error) { fail(std::string("cones.csv: ")+error.what()); }
    }
    if (const auto& judged = r.metadata.competition) {
        if (course_fingerprint(cones, judged->gates) != judged->course_fingerprint)
            fail("metadata.competition.course_fingerprint does not identify the cones and gates recorded");
        // A course laid along the track is the one laid along the recorded track, as this build lays it.
        if (judged->course == Simulation::laid_along_the_track) {
            const auto laid = make_cone_layout(r.track);
            const auto gates = make_gates(r.track);
            bool same = laid.size() == cones.size() && gates.size() == judged->gates.size();
            for (std::size_t i = 0; same && i < laid.size(); ++i)
                same = laid[i].colour == cones[i].colour && near(laid[i].position.x, cones[i].position.x, 1e-9) &&
                       near(laid[i].position.y, cones[i].position.y, 1e-9);
            for (std::size_t i = 0; same && i < gates.size(); ++i)
                same = near(gates[i].left.x, judged->gates[i].left.x, 1e-9) && near(gates[i].left.y, judged->gates[i].left.y, 1e-9) &&
                       near(gates[i].right.x, judged->gates[i].right.x, 1e-9) && near(gates[i].right.y, judged->gates[i].right.y, 1e-9);
            if (!same) fail("cones.csv and metadata.competition.gates are not the course laid along the recorded track");
        }
    }
    return cones;
}

ObservedColour observed_colour(const RowReader& row) {
    const std::string colour = row.text("colour");
    for (const auto c : {ObservedColour::unknown, ObservedColour::blue, ObservedColour::yellow, ObservedColour::orange, ObservedColour::big_orange})
        if (colour == observed_colour_name(c)) return c;
    fail(row.where("colour")+" is not a colour a detection can report");
}

std::vector<RecordedPerceptionFrame> read_perception(const std::filesystem::path& directory, const Recording& r) {
    const Table frames = read_csv(directory/"perception_frames.csv", perception_frames_header);
    const Table detections = read_csv(directory/"detections.csv", detections_header);
    const Table missed = read_csv(directory/"missed.csv", missed_header);
    if (!r.metadata.perception) {
        if (!frames.rows.empty() || !detections.rows.empty() || !missed.rows.empty())
            fail("the perception files hold frames for a run whose metadata records no perception");
        return {};
    }
    std::vector<RecordedPerceptionFrame> recorded;
    std::size_t next_detection = 0, next_missed = 0;
    for (std::size_t i = 0; i < frames.rows.size(); ++i) {
        const RowReader row(frames, i);
        if (row.index("frame") != i) fail(row.where("frame")+" must number the frames in order from zero");
        RecordedPerceptionFrame f;
        f.time_s = row.number("time_s");
        f.frame.sensor = static_cast<std::size_t>(row.index("sensor"));
        f.frame.sampled_at_s = row.number("sampled_at_s");
        f.frame.delivered_at_s = row.number("delivered_at_s");
        const auto count = row.index("detections"), misses = row.index("missed");
        for (std::uint64_t k = 0; k < count; ++k, ++next_detection) {
            if (next_detection >= detections.rows.size()) fail(row.where("detections")+" counts detections detections.csv does not hold");
            const RowReader d(detections, next_detection);
            if (d.index("frame") != i || d.index("detection") != k) fail(d.where("frame")+" is out of order with perception_frames.csv");
            ConeDetection det;
            det.position = {d.number("x_m"), d.number("y_m")};
            det.colour = observed_colour(d);
            det.colour_probability = d.number("colour_probability");
            det.detection_probability = d.number("detection_probability");
            det.covariance_xx = d.number("covariance_xx");
            det.covariance_xy = d.number("covariance_xy");
            det.covariance_yy = d.number("covariance_yy");
            const auto cone = d.index("cone");
            if (cone >= r.cones.size()) fail(d.where("cone")+" names no recorded cone");
            f.frame.detections.push_back(det);
            f.source_cone.push_back(static_cast<std::size_t>(cone));
        }
        for (std::uint64_t k = 0; k < misses; ++k, ++next_missed) {
            if (next_missed >= missed.rows.size()) fail(row.where("missed")+" counts misses missed.csv does not hold");
            const RowReader m(missed, next_missed);
            if (m.index("frame") != i) fail(m.where("frame")+" is out of order with perception_frames.csv");
            const auto cone = m.index("cone");
            if (cone >= r.cones.size()) fail(m.where("cone")+" names no recorded cone");
            f.missed.push_back(static_cast<std::size_t>(cone));
        }
        recorded.push_back(std::move(f));
    }
    if (next_detection != detections.rows.size()) fail("detections.csv holds detections no frame counts");
    if (next_missed != missed.rows.size()) fail("missed.csv holds misses no frame counts");
    // Run the recorded perception again over the recorded poses and require the same frames, in the same order.
    Perception again(r.cones, *r.metadata.perception);
    std::size_t next = 0;
    const auto agrees = [&](const std::string& where, double was, double is) {
        if (!near(was, is, value_tolerance*std::max(1.0, std::abs(is))))
            fail("perception frame "+std::to_string(next)+" "+where+" is not what this perception detects from the recorded seed, cones and poses");
    };
    for (const auto& sample : r.samples) {
        again.observe(sample.state(), sample.time_s);
        for (std::size_t k = 0; k < again.delivered().size(); ++k, ++next) {
            if (next >= recorded.size()) fail("perception_frames.csv lacks frames this perception delivers from the recorded run");
            const auto& was = recorded[next];
            const auto& is = again.delivered()[k];
            const auto& truth = again.delivered_truth()[k];
            if (!near(was.time_s, sample.time_s, time_tolerance_s)) fail("perception frame "+std::to_string(next)+" was delivered at another tick");
            if (was.frame.sensor != is.sensor) fail("perception frame "+std::to_string(next)+" names another sensor");
            agrees("sampled_at_s", was.frame.sampled_at_s, is.sampled_at_s);
            agrees("delivered_at_s", was.frame.delivered_at_s, is.delivered_at_s);
            if (was.frame.detections.size() != is.detections.size() || was.source_cone != truth.source_cone || was.missed != truth.missed)
                fail("perception frame "+std::to_string(next)+" detects or misses other cones than this perception does from the recorded seed, cones and poses");
            for (std::size_t d = 0; d < is.detections.size(); ++d) {
                const auto& a = was.frame.detections[d];
                const auto& b = is.detections[d];
                if (a.colour != b.colour) fail("perception frame "+std::to_string(next)+" reports another colour");
                agrees("x_m", a.position.x, b.position.x);
                agrees("y_m", a.position.y, b.position.y);
                agrees("colour_probability", a.colour_probability, b.colour_probability);
                agrees("detection_probability", a.detection_probability, b.detection_probability);
                agrees("covariance_xx", a.covariance_xx, b.covariance_xx);
                agrees("covariance_xy", a.covariance_xy, b.covariance_xy);
                agrees("covariance_yy", a.covariance_yy, b.covariance_yy);
            }
        }
    }
    if (next != recorded.size()) fail("perception_frames.csv holds frames this perception does not deliver from the recorded run");
    return recorded;
}

namespace {

// timing.csv (schema 17), then the recorded samples judged again on the recorded course by the recorded rules: the same
// events, in the same order.
std::vector<TimingEvent> read_timing(const std::filesystem::path& path, const Recording& r) {
    const Table table = read_csv(path, timing_header);
    std::vector<TimingEvent> recorded;
    for (std::size_t i = 0; i < table.rows.size(); ++i) {
        const RowReader row(table, i);
        if (row.index("event") != i) fail(row.where("event")+" must number the events in order from zero");
        TimingEvent e;
        const std::string kind = row.text("kind");
        const auto known = std::find_if(all_timing_kinds.begin(), all_timing_kinds.end(), [&](TimingKind k) { return timing_kind_name(k) == kind; });
        if (known == all_timing_kinds.end()) fail(row.where("kind")+" is not something the judge sees");
        e.kind = *known;
        e.time_s = row.number("time_s");
        e.lap = static_cast<int>(row.index("lap"));
        e.seconds = row.number("seconds");
        if (row.text("cone") != "none") e.cone = row.index("cone");
        e.position = {row.number("x_m"), row.number("y_m")};
        e.reason = row.text("reason");
        recorded.push_back(e);
    }
    const auto& judged = *r.metadata.competition;
    Judge again(Course{r.track, r.cones, judged.gates}, judged.rules, judged.footprint);
    for (const auto& sample : r.samples) again.observe(sample.state(), sample.time_s);
    const auto& is = again.events();
    for (std::size_t i = 0; i < std::min(is.size(), recorded.size()); ++i) {
        const auto& a = recorded[i];
        const auto& b = is[i];
        const std::string where = "timing.csv event "+std::to_string(i)+" ("+timing_kind_name(a.kind)+")";
        if (a.kind != b.kind || a.lap != b.lap || a.cone != b.cone || a.reason != b.reason || !near(a.time_s, b.time_s, time_tolerance_s) ||
            !near(a.seconds, b.seconds, 1e-9*std::max(1.0, std::abs(b.seconds))) || !near(a.position.x, b.position.x, 1e-9*std::max(1.0, std::abs(b.position.x))) ||
            !near(a.position.y, b.position.y, 1e-9*std::max(1.0, std::abs(b.position.y))))
            fail(where+" is not what the judge sees in the recorded run");
    }
    if (is.size() != recorded.size())
        fail("timing.csv holds "+std::to_string(recorded.size())+" events where the judge sees "+std::to_string(is.size())+" in the recorded run");
    return recorded;
}

// beliefs.csv and believed_paths.csv (schema 16): one belief per decision of a run on cones, every believed path in the
// order made; nothing for a run on the known track.
void read_beliefs(const std::filesystem::path& directory, Recording& r) {
    const Table beliefs = read_csv(directory/"beliefs.csv", beliefs_header);
    const Table paths = read_csv(directory/"believed_paths.csv", believed_paths_header);
    if (!r.cone_driving_recorded()) {
        if (!beliefs.rows.empty() || !paths.rows.empty()) fail("the belief files hold rows for a run that did not drive on cones");
        return;
    }
    if (beliefs.rows.size() != r.decisions.size())
        fail("beliefs.csv holds "+std::to_string(beliefs.rows.size())+" beliefs for "+std::to_string(r.decisions.size())+
             " decisions; a run on cones believes something at every decision");
    for (std::size_t i = 0; i < paths.rows.size(); ++i) {
        const RowReader row(paths, i);
        const std::uint64_t index = row.index("path"), point = row.index("point");
        if (point == 0) {
            if (index != r.believed_paths.size()) fail(row.where("path")+" must number the believed paths in order from zero");
            RecordedBelievedPath path;
            path.index = index;
            path.frame = row.index("frame");
            if (path.frame >= r.perception_frames.size()) fail(row.where("frame")+" names a perception frame the run did not deliver");
            path.pose = {row.number("pose_x_m"), row.number("pose_y_m"), row.number("pose_yaw_rad"), 0, 0,
                         r.perception_frames[path.frame].frame.sampled_at_s};
            path.path.width_m = row.number("width_m");
            r.believed_paths.push_back(std::move(path));
        } else if (r.believed_paths.empty() || index+1 != r.believed_paths.size() || point != r.believed_paths.back().path.points.size()) {
            fail(row.where("point")+" must number each believed path's samples in order from zero");
        }
        auto& path = r.believed_paths.back();
        if (row.index("frame") != path.frame || row.number("width_m") != path.path.width_m)
            fail(row.where("frame")+" differs between the samples of one believed path");
        path.path.points.push_back({row.number("x_m"), row.number("y_m"), row.number("s_m"), row.number("curvature_1pm")});
    }
    for (const auto& path : r.believed_paths) {
        try { validate_open_path(path.path); }
        catch (const std::exception& error) { fail("believed_paths.csv path "+std::to_string(path.index)+": "+error.what()); }
    }
    for (std::size_t d = 0; d < beliefs.rows.size(); ++d) {
        const RowReader row(beliefs, d);
        RecordedBelief belief;
        belief.decision = row.index("decision");
        if (belief.decision != d) fail(row.where("decision")+" must number the decisions in order from zero");
        belief.state = {row.number("x_m"), row.number("y_m"), row.number("yaw_rad"), row.number("speed_mps"), row.number("steering_rad"),
                        row.number("time_s")};
        if (!near(belief.state.time_s, r.decisions[d].time_s, time_tolerance_s)) fail(row.where("time_s")+" is not its decision's time");
        belief.pose_sampled_at_s = row.number("pose_sampled_at_s");
        if (belief.pose_sampled_at_s > belief.state.time_s+time_tolerance_s) fail(row.where("pose_sampled_at_s")+" is after the decision");
        if (row.text("path") != "none") {
            belief.path = row.index("path");
            if (*belief.path >= r.believed_paths.size()) fail(row.where("path")+" names a believed path believed_paths.csv lacks");
            const auto& frame = r.perception_frames[r.believed_paths[*belief.path].frame];
            if (frame.time_s > belief.state.time_s+time_tolerance_s) fail(row.where("path")+" follows a path from a frame delivered after the decision");
            if (d && r.beliefs.back().path && *belief.path < *r.beliefs.back().path) fail(row.where("path")+" returns to an older believed path");
        } else if (d && r.beliefs.back().path) {
            fail(row.where("path")+" forgets the believed path it followed");
        }
        r.beliefs.push_back(belief);
    }
}

// Runs a driver again on the recorded readings and frames alone, fed tick by tick as the simulation fed its own, and
// requires every belief, believed path and first command the run recorded. The driver is given no track and no cones;
// so a recording that passes shows its decisions came from its observations and nothing else.
void check_cone_driving(const Recording& r, const std::vector<std::size_t>& starts) {
    if (!r.cone_driving_recorded()) return;
    const auto& driving = *r.metadata.cone_driving;
    ConeDriver driver(driving.settings, driving.memory);
    driver.reset(r.samples.front().state());
    const auto& model = r.metadata.vehicle_model;
    const double dt = r.metadata.initial_config.fixed_dt_s;
    const auto control_ticks = static_cast<std::uint64_t>(std::llround(r.metadata.initial_config.control_dt_s/dt));
    std::uint64_t revision = 0;
    Config config;
    std::optional<SteeringTable> table;
    std::optional<PerformanceEnvelope> envelope;
    std::size_t next_frame = 0, next_decision = 0;
    std::uint64_t paths_checked = 0;
    const auto same = [](double a, double b) { return near(a, b, 1e-9*std::max(1.0, std::abs(b))); };
    for (std::size_t i = 0; i < r.samples.size(); ++i) {
        const auto& sample = r.samples[i];
        if (driving.source == BeliefSource::measured) {
            const auto& m = r.measurements[i].measured;
            if (m.pose.measured) driver.read_pose({m.pose.sampled_at_s, m.x_m, m.y_m, m.yaw_rad});
            driver.read_motion({m.speed.measured ? m.speed_mps : 0.0, m.imu.measured ? m.yaw_rate_radps : 0.0,
                                m.steering.measured ? m.steering_rad : 0.0});
        } else {
            driver.read_pose({sample.time_s, sample.x_m, sample.y_m, sample.yaw_rad});
            driver.read_motion({sample.speed_mps, sample.yaw_rate_radps, sample.steering_rad});
        }
        while (next_frame < r.perception_frames.size() && near(r.perception_frames[next_frame].time_s, sample.time_s, time_tolerance_s))
            driver.perceive(r.perception_frames[next_frame++].frame);
        for (const auto& made : driver.recent_paths()) {
            if (made.index < paths_checked) continue;
            const std::string where = "believed path "+std::to_string(made.index);
            if (made.index >= r.believed_paths.size()) fail(where+" is believed from the recorded frames but believed_paths.csv lacks it");
            const auto& recorded = r.believed_paths[made.index];
            if (recorded.frame != made.frame || recorded.path.points.size() != made.path.points.size() || !same(recorded.path.width_m, made.path.width_m))
                fail(where+" is not what a driver believes from the recorded readings and frames");
            for (std::size_t k = 0; k < made.path.points.size(); ++k) {
                const auto& a = recorded.path.points[k];
                const auto& b = made.path.points[k];
                if (!same(a.x_m, b.x_m) || !same(a.y_m, b.y_m) || !same(a.s_m, b.s_m) || !same(a.curvature, b.curvature))
                    fail(where+" sample "+std::to_string(k)+" is not what a driver believes from the recorded readings and frames");
            }
            paths_checked = made.index+1;
        }
        if (next_decision < r.decisions.size() && starts[next_decision] == i) {
            const std::size_t d = next_decision++;
            // A decision governs the step after it, under the revision the next sample records.
            const auto governed = r.samples[std::min(i+1, r.samples.size()-1)].revision;
            if (governed != revision) {
                revision = governed;
                config = r.config_for_revision(revision);
                table.reset();
                envelope.reset();
                if (r.metadata.steering_mode == SteeringMode::model_acceleration_pursuit) table.emplace(model, config);
                if (r.metadata.speed_plan_mode == SpeedPlanMode::performance_envelope) envelope.emplace(model, config);
            }
            const auto tick = static_cast<std::uint64_t>(std::llround(sample.time_s/dt));
            const auto decision = driver.decide(sample.time_s, config, model, control_ticks-tick%control_ticks,
                                                table ? &*table : nullptr, envelope ? &*envelope : nullptr);
            const std::string where = "decision "+std::to_string(d)+" on cones";
            const auto& believed = driver.belief();
            const auto& recorded = r.beliefs[d];
            if (!same(recorded.state.x_m, believed.state.x_m) || !same(recorded.state.y_m, believed.state.y_m) ||
                !same(recorded.state.yaw_rad, believed.state.yaw_rad) || !same(recorded.state.speed_mps, believed.state.speed_mps) ||
                !same(recorded.state.steering_rad, believed.state.steering_rad) ||
                !same(recorded.pose_sampled_at_s, believed.pose_sampled_at_s) || recorded.path != believed.path)
                fail(where+" believed otherwise than a driver does from the recorded readings and frames alone");
            const auto& command = decision.trajectory().first_control;
            const auto& was = r.decisions[d];
            if (!same(was.applied.acceleration_mps2, command.applied.acceleration_mps2) || !same(was.applied.steering_rad, command.applied.steering_rad) ||
                !same(was.requested.acceleration_mps2, command.requested.acceleration_mps2) ||
                !same(was.requested.steering_rad, command.requested.steering_rad))
                fail(where+" commanded otherwise than a driver does from the recorded readings and frames alone");
        }
    }
    if (paths_checked != r.believed_paths.size()) fail("believed_paths.csv holds paths no driver believes from the recorded readings and frames");
}

}  // namespace

Recording load_recording(const std::filesystem::path& directory) {
    if (!std::filesystem::is_directory(directory)) fail("directory does not exist: "+directory.string());
    for (const char* name : {"metadata.json", "track.csv", "plan.csv", "plan-rev-1.csv", "telemetry.csv", "events.csv", "summary.json"})
        if (!std::filesystem::is_regular_file(directory/name)) fail(std::string("missing file ")+name);
    Recording r;
    r.directory = directory;
    r.metadata = read_metadata(directory/"metadata.json");
    const bool decision_files = std::filesystem::exists(directory/"decisions.csv") || std::filesystem::exists(directory/"trajectories.csv");
    if (r.metadata.schema_version == 1 && decision_files)
        fail("a schema 1 run must not contain decisions.csv or trajectories.csv; its metadata may have been altered");
    if (r.metadata.schema_version >= 2)
        for (const char* name : {"decisions.csv", "trajectories.csv"})
            if (!std::filesystem::is_regular_file(directory/name)) fail(std::string("missing file ")+name);
    if (r.speed_profiles_recorded() != std::filesystem::exists(directory/"profiles.csv"))
        fail(r.speed_profiles_recorded() ? "missing file profiles.csv"
                                         : "a schema "+std::to_string(r.metadata.schema_version)+
                                               " run must not contain profiles.csv; its metadata may have been altered");
    if (r.local_planner_recorded() != std::filesystem::exists(directory/"paths.csv"))
        fail(r.local_planner_recorded() ? "missing file paths.csv"
                                        : "a schema "+std::to_string(r.metadata.schema_version)+
                                              " run must not contain paths.csv; its metadata may have been altered");
    if (r.measurements_recorded() != std::filesystem::exists(directory/"measurements.csv"))
        fail(r.measurements_recorded() ? "missing file measurements.csv"
                                       : "a schema "+std::to_string(r.metadata.schema_version)+
                                             " run must not contain measurements.csv; its metadata may have been altered");
    for (const char* name : {"cones.csv", "perception_frames.csv", "detections.csv", "missed.csv"})
        if (r.perception_files_recorded() != std::filesystem::exists(directory/name))
            fail(r.perception_files_recorded() ? std::string("missing file ")+name
                                               : "a schema "+std::to_string(r.metadata.schema_version)+" run must not contain "+name+
                                                     "; its metadata may have been altered");
    if (r.competition_recorded() != std::filesystem::exists(directory/"timing.csv"))
        fail(r.competition_recorded() ? "missing file timing.csv"
                                      : "a schema "+std::to_string(r.metadata.schema_version)+
                                            " run must not contain timing.csv; its metadata may have been altered");
    for (const char* name : {"beliefs.csv", "believed_paths.csv"})
        if (r.cone_driving_files_recorded() != std::filesystem::exists(directory/name))
            fail(r.cone_driving_files_recorded() ? std::string("missing file ")+name
                                                 : "a schema "+std::to_string(r.metadata.schema_version)+" run must not contain "+name+
                                                       "; its metadata may have been altered");
    if (r.commands_recorded() != std::filesystem::exists(directory/"commands.csv"))
        fail(r.commands_recorded() ? "missing file commands.csv"
                                   : "a schema "+std::to_string(r.metadata.schema_version)+
                                         " run must not contain commands.csv; its metadata may have been altered");
    r.track = read_track(directory/"track.csv", r.metadata);
    try { validate_obstructions(r.track, r.metadata.scenario_obstructions); }
    catch (const std::exception& error) { fail(std::string("scenario_obstructions: ")+error.what()); }
    r.events = read_events(directory/"events.csv", r.metadata.initial_config);
    r.plans = read_plans(directory, r.track, r.events);
    r.samples = read_samples(directory/"telemetry.csv", r);
    r.summary = read_summary(directory/"summary.json", r);
    if (r.measurements_recorded()) r.measurements = read_measurements(directory/"measurements.csv", r);
    if (r.perception_files_recorded()) {
        r.cones = read_cones(directory/"cones.csv", r);
        r.perception_frames = read_perception(directory, r);
    }
    if (r.competition_recorded()) r.timing = read_timing(directory/"timing.csv", r);
    if (r.metadata.schema_version >= 2) {
        // Ordered cheapest first: the schedule is checked before the large trajectory file is parsed.
        r.decisions = read_decision_rows(directory/"decisions.csv", r);
        const auto starts = check_decision_schedule(r);
        if (r.cone_driving_files_recorded()) read_beliefs(directory, r);
        read_trajectories(directory/"trajectories.csv", r, starts);
        if (r.local_planner_recorded()) read_paths(directory/"paths.csv", r, starts);
        if (r.speed_profiles_recorded()) read_profiles(directory/"profiles.csv", r, starts);
        check_commands_against_decisions(r);
        check_controller_plans(r);
        if (r.commands_recorded()) read_commands(directory/"commands.csv", r);
        check_cone_driving(r, starts);
    }
    return r;
}

std::optional<double> plant_reproduction_error_m(const Recording& r) {
    if (!r.decisions_recorded()) return std::nullopt;
    double worst = 0;
    std::size_t d = 0;
    for (std::size_t i = 1; i < r.samples.size() && !r.decisions.empty(); ++i) {
        const auto& from = r.samples[i-1];
        const auto& to = r.samples[i];
        while (d+1 < r.decisions.size() && r.decisions[d+1].time_s <= from.time_s+time_tolerance_s) ++d;
        const Config config = r.config_for_revision(to.revision);
        const PlantState start = r.plant_recorded() ? from.plant_state() : plant_state_from(from.state(), r.metadata.vehicle_model, config);
        const auto next = advance(r.metadata.vehicle_model, start, {to.applied_acceleration_mps2, r.decisions[d].applied.steering_rad},
                                  config, config.fixed_dt_s);
        worst = std::max(worst, std::hypot(next.pose.x_m-to.x_m, next.pose.y_m-to.y_m));
    }
    return worst;
}

std::optional<bool> recorded_steering_table_matches(const Recording& r) {
    if (r.metadata.steering_mode == SteeringMode::pure_pursuit) return std::nullopt;
    return steering_table_fingerprint(r.metadata.vehicle_model, r.metadata.initial_config) == r.metadata.steering_table_fingerprint;
}

std::optional<bool> recorded_performance_envelope_matches(const Recording& r) {
    if (r.metadata.speed_plan_mode == SpeedPlanMode::grip_fractions) return std::nullopt;
    return performance_envelope_fingerprint(r.metadata.vehicle_model, r.metadata.initial_config) == r.metadata.envelope_fingerprint;
}

namespace {

// What followed a plan at one of its points: a rear axle and its speed.
struct Followed { double x_m{}, y_m{}, speed_mps{}; };

// Gathers, for each time ahead, how far what followed each plan was from it there. follow gives what followed a plan at
// each of its points, or nothing where it cannot be measured.
class DeviationGatherer {
public:
    explicit DeviationGatherer(const std::vector<double>& ahead_s) : ahead_(ahead_s), gathered_(ahead_s.size()) {
        for (const double ahead : ahead_s)
            if (!std::isfinite(ahead) || ahead < 0) throw std::invalid_argument("Prediction errors are measured a finite, nonnegative time ahead");
    }
    void add(std::size_t index, const RecordedDecision& decision, const std::vector<std::optional<Followed>>& followed) {
        const auto& plan = decision.choice().trajectory;
        for (std::size_t a = 0; a < ahead_.size(); ++a) {
            const double when = decision.time_s+ahead_[a];
            const auto point = std::find_if(plan.begin(), plan.end(), [&](const RecordedTrajectoryPoint& q) { return std::abs(q.time_s-when) < 1e-6; });
            if (point == plan.end()) continue;
            const auto k = static_cast<std::size_t>(point-plan.begin());
            if (!followed[k]) continue;
            // The plan's direction of travel there, from its neighbouring points.
            const auto& before = plan[k > 0 ? k-1 : k];
            const auto& after = plan[k+1 < plan.size() ? k+1 : k];
            const double dx = after.x_m-before.x_m, dy = after.y_m-before.y_m, run = std::hypot(dx, dy);
            const double ex = followed[k]->x_m-point->x_m, ey = followed[k]->y_m-point->y_m;
            gathered_[a].push_back({index, std::hypot(ex, ey), run > 1e-9 ? (ex*dx+ey*dy)/run : 0.0,
                                    run > 1e-9 ? std::abs(ex*dy-ey*dx)/run : std::hypot(ex, ey), followed[k]->speed_mps-point->speed_mps});
        }
    }
    std::vector<std::vector<PlanDeviation>> result() && { return std::move(gathered_); }
private:
    std::vector<double> ahead_;
    std::vector<std::vector<PlanDeviation>> gathered_;
};

bool driven_by_controller(const Recording& r) {
    return r.controller_recorded() && r.metadata.predictive_controller.mode != ControllerMode::policy;
}

double quantile(std::vector<double> values, double share) {
    std::sort(values.begin(), values.end());
    return values[std::min(values.size()-1, static_cast<std::size_t>(share*static_cast<double>(values.size())))];
}

std::vector<PredictionError> summarise(const std::vector<double>& ahead_s, const std::vector<std::vector<PlanDeviation>>& deviations) {
    std::vector<PredictionError> errors;
    for (std::size_t a = 0; a < ahead_s.size(); ++a)
        errors.push_back(prediction_error(ahead_s[a], a < deviations.size() ? deviations[a] : std::vector<PlanDeviation>{}));
    return errors;
}

} // namespace

PredictionError prediction_error(double ahead_s, const std::vector<PlanDeviation>& deviations) {
    PredictionError e;
    e.ahead_s = ahead_s;
    e.samples = deviations.size();
    if (deviations.empty()) return e;
    std::vector<double> distance, along, across, speed, speed_size;
    for (const auto& v : deviations) {
        distance.push_back(v.distance_m);
        along.push_back(v.along_m);
        across.push_back(v.across_m);
        speed.push_back(v.speed_mps);
        speed_size.push_back(std::abs(v.speed_mps));
    }
    e.median_m = quantile(distance, 0.5);
    e.percentile_95_m = quantile(distance, 0.95);
    e.worst_m = *std::max_element(distance.begin(), distance.end());
    e.median_along_m = quantile(along, 0.5);
    e.median_across_m = quantile(across, 0.5);
    e.median_speed_mps = quantile(speed, 0.5);
    e.percentile_95_speed_mps = quantile(speed_size, 0.95);
    return e;
}

std::vector<std::vector<PlanDeviation>> controller_plan_deviations(const Recording& r, const std::vector<double>& ahead_s) {
    DeviationGatherer gatherer(ahead_s);
    if (!driven_by_controller(r)) return {};
    const double dt = r.metadata.initial_config.fixed_dt_s;
    for (std::size_t d = 0; d < r.decisions.size(); ++d) {
        const auto& decision = r.decisions[d];
        if (decision.driven_by == ControllerMode::policy) continue;
        const auto& plan = decision.choice().trajectory;
        std::vector<std::optional<Followed>> followed(plan.size());
        for (std::size_t k = 0; k < plan.size(); ++k) {
            const auto tick = static_cast<std::size_t>(std::llround(plan[k].time_s/dt));
            if (tick >= r.samples.size() || std::abs(r.samples[tick].time_s-plan[k].time_s) > 1e-6) continue;
            const auto& s = r.samples[tick];
            followed[k] = Followed{s.x_m, s.y_m, s.speed_mps};
        }
        gatherer.add(d, decision, followed);
    }
    return std::move(gatherer).result();
}

std::vector<std::vector<PlanDeviation>> controller_model_deviations(const Recording& r, const std::vector<double>& ahead_s) {
    DeviationGatherer gatherer(ahead_s);
    if (!driven_by_controller(r) || !r.commands_recorded()) return {};
    const double dt = r.metadata.initial_config.fixed_dt_s;
    for (std::size_t d = 0; d < r.decisions.size(); ++d) {
        const auto& decision = r.decisions[d];
        if (decision.driven_by == ControllerMode::policy) continue;
        const auto& option = decision.choice();
        const auto tick = static_cast<std::size_t>(std::llround(decision.time_s/dt));
        // The decision governs the step from its sample, under the configuration that step was taken with.
        const Config config = r.config_for_revision(r.samples[std::min(tick+1, r.samples.size()-1)].revision);
        std::vector<double> times;
        for (const auto& point : option.trajectory) times.push_back(point.time_s);
        const auto ticks = interval_end_tick(r, decision.time_s)-static_cast<std::uint64_t>(tick);
        const auto response = plant_response(r.metadata.vehicle_model, r.samples[tick].plant_state(), config, times, option.commands, ticks);
        std::vector<std::optional<Followed>> followed;
        for (const auto& state : response) followed.push_back(Followed{state.pose.x_m, state.pose.y_m, state.pose.speed_mps});
        gatherer.add(d, decision, followed);
    }
    return std::move(gatherer).result();
}

std::vector<PredictionError> controller_plan_errors(const Recording& r, const std::vector<double>& ahead_s) {
    if (!driven_by_controller(r)) { DeviationGatherer check(ahead_s); return {}; }
    return summarise(ahead_s, controller_plan_deviations(r, ahead_s));
}

std::vector<PredictionError> controller_model_errors(const Recording& r, const std::vector<double>& ahead_s) {
    if (!driven_by_controller(r) || !r.commands_recorded()) { DeviationGatherer check(ahead_s); return {}; }
    return summarise(ahead_s, controller_model_deviations(r, ahead_s));
}

double plan_reproduction_error_mps(const Recording& recording) {
    double worst = 0;
    for (const auto& recorded : recording.plans) {
        const Config config = recording.config_for_revision(recorded.revision);
        // An envelope plan is reproduced from an envelope derived again for this revision's configuration.
        std::optional<PerformanceEnvelope> envelope;
        if (recording.metadata.speed_plan_mode == SpeedPlanMode::performance_envelope) envelope.emplace(recording.metadata.vehicle_model, config);
        const auto plan = make_speed_plan(recording.track, config, envelope ? &*envelope : nullptr);
        if (plan.size() != recorded.points.size()) throw std::runtime_error("Planner sample count differs from the recording");
        for (std::size_t i = 0; i < plan.size(); ++i)
            worst = std::max(worst, std::abs(plan[i].speed_mps-recorded.points[i].speed_mps));
    }
    return worst;
}

// ---------------------------------------------------------------- RecordingWriter

RecordingWriter::RecordingWriter(std::filesystem::path directory, const Simulation& simulation,
                                 const RunRequest& request, const BuildIdentity& identity)
    : directory_(std::move(directory)), request_(request) {
    if (std::filesystem::exists(directory_) && !std::filesystem::is_empty(directory_))
        throw std::invalid_argument("Run directory is not empty; choose a fresh --out directory to preserve recording provenance");
    if (simulation.state().time_s != 0 || simulation.diagnostics().revision != 1 || !simulation.events().empty())
        throw std::invalid_argument("Recording must start from a freshly constructed or reset simulation");
    // A person's controls are not part of the contract, and no driver run again could reproduce their decisions.
    if (simulation.human_driving())
        throw std::invalid_argument("A person's drive is not recorded (decision 0037); hand the car back to the autonomous driver first");
    if (request_.requested_laps < 0 || request_.requested_laps > 100 || !std::isfinite(request_.duration_cap_s) || request_.duration_cap_s <= 0)
        throw std::invalid_argument("Run request needs laps in 0..100 and a positive duration cap");
    std::filesystem::create_directories(directory_);
    scenario_ = simulation.obstructions();
    // The decision already made on reset or scenario setup is superseded by the first step's
    // own decision before any motion, so it is not recorded.
    decisions_seen_ = simulation.decision_count();
    write_metadata(directory_, simulation, request_, identity);
    write_plan(simulation, directory_/"plan.csv");
    write_plan(simulation, directory_/"plan-rev-1.csv");
    auto track = open_output(directory_/"track.csv");
    track << std::setprecision(17) << track_header << track_edge_columns << '\n';
    const auto& driven = simulation.track();
    for (std::size_t i = 0; i < driven.points.size(); ++i) {
        const auto& p = driven.points[i];
        const bool edges = !driven.left_edge_m.empty();
        track << i << ',' << p.x_m << ',' << p.y_m << ',' << p.s_m << ',' << p.curvature << ','
              << (edges ? driven.left_edge_m[i] : driven.width_m/2) << ',' << (edges ? driven.right_edge_m[i] : driven.width_m/2) << '\n';
    }
    telemetry_ = open_output(directory_/"telemetry.csv");
    telemetry_ << std::setprecision(17);
    telemetry_ << telemetry_header << telemetry_plant_columns << telemetry_steering_columns << telemetry_wheel_columns
               << telemetry_load_columns << telemetry_acceleration_columns << '\n';
    decisions_ = open_output(directory_/"decisions.csv");
    decisions_ << decisions_header << decisions_lattice_columns << decisions_profile_columns << decisions_controller_columns << '\n';
    trajectories_ = open_output(directory_/"trajectories.csv");
    trajectories_ << trajectories_header << '\n';
    paths_ = open_output(directory_/"paths.csv");
    paths_ << paths_header << '\n';
    profiles_ = open_output(directory_/"profiles.csv");
    profiles_ << profiles_header << '\n';
    measurements_ = open_output(directory_/"measurements.csv");
    measurements_ << std::setprecision(17) << measurements_header << '\n';
    // The course's cones at full precision, so the perception can be run again from them.
    {
        auto cones = open_output(directory_/"cones.csv");
        cones << std::setprecision(17) << cones_header << '\n';
        // The course's cones, which the judge and any perception share.
        for (std::size_t i = 0; i < simulation.cones().size(); ++i) {
            const auto& c = simulation.cones()[i];
            cones << i << ',' << c.position.x << ',' << c.position.y << ',' << csv_text(cone_colour_name(c.colour)) << '\n';
        }
        if (!cones) throw std::runtime_error("Writing cones.csv failed");
    }
    perception_frames_ = open_output(directory_/"perception_frames.csv");
    perception_frames_ << std::setprecision(17) << perception_frames_header << '\n';
    detections_ = open_output(directory_/"detections.csv");
    detections_ << std::setprecision(17) << detections_header << '\n';
    missed_ = open_output(directory_/"missed.csv");
    missed_ << missed_header << '\n';
    commands_ = open_output(directory_/"commands.csv");
    commands_ << commands_header << '\n';
    beliefs_ = open_output(directory_/"beliefs.csv");
    beliefs_ << std::setprecision(17) << beliefs_header << '\n';
    believed_paths_ = open_output(directory_/"believed_paths.csv");
    believed_paths_ << std::setprecision(17) << believed_paths_header << '\n';
    timing_ = open_output(directory_/"timing.csv");
    timing_ << std::setprecision(17) << timing_header << '\n';
    last_revision_ = 1;
    record(simulation);
}

void RecordingWriter::record(const Simulation& simulation) {
    if (finished_) throw std::logic_error("Recording is already finished");
    if (simulation.human_driving())
        throw std::logic_error("A person took the car after recording started; a person's drive is not recorded (decision 0037)");
    if (!same_scenario(simulation.obstructions(), scenario_))
        throw std::logic_error("Recording scenario changed after recording started; state blocked regions before creating the writer");
    if (simulation.decision_count() != decisions_seen_) {
        if (simulation.decision_count() != decisions_seen_+1)
            throw std::logic_error("Record after every step so every decision that governed motion is kept");
        const auto& decision = simulation.decision();
        const auto& chosen = decision.trajectory();
        const std::uint64_t index = decisions_written_++;
        const double decision_time = chosen.points.front().state.time_s;
        for (std::size_t o = 0; o < decision.options.size(); ++o) {
            const auto& option = decision.options[o];
            decisions_ << index << ',' << decision_time << ',' << o << ',' << (o == decision.selected) << ','
                       << option.lateral_offset_m << ',';
            if (std::isfinite(option.speed_limit_mps)) decisions_ << option.speed_limit_mps; else decisions_ << "none";
            decisions_ << ',' << option.clear << ',' << option.trajectory.within_model_envelope << ','
                       << option.station_gain_m << ',' << csv_text(option.reason) << ','
                       << csv_text(option.trajectory.validity_reason) << ',' << decision.holding << ','
                       << csv_optional_text(decision.blocking_identifier) << ',' << csv_text(decision.reason) << ','
                       << chosen.first_control.requested.acceleration_mps2 << ',' << chosen.first_control.requested.steering_rad << ','
                       << chosen.first_control.applied.acceleration_mps2 << ',' << chosen.first_control.applied.steering_rad << ','
                       << local_action_name(option.action) << ',' << option.path.size() << ','
                       << option.path_cost.edges.average_curvature << ',' << option.path_cost.edges.curvature_range << ','
                       << option.path_cost.edges.length << ',' << option.path_cost.edges.reference_deviation << ','
                       << option.path_cost.turns << ',' << option.path_cost.goal << ',' << option.speed_profile.size() << ',';
            if (!option.speed_profile.empty() && std::isfinite(option.estimated_time_s)) decisions_ << option.estimated_time_s; else decisions_ << "none";
            const auto& outcome = simulation.controller_outcome();
            decisions_ << ',' << controller_mode_name(outcome.driven_by) << ','
                       << (outcome.asked ? controller_status_name(outcome.status) : std::string_view("none")) << ','
                       << outcome.iterations << ',' << outcome.restarted << ',' << csv_optional_text(outcome.note) << '\n';
            for (std::size_t p = 0; p < option.speed_profile.size(); ++p) {
                const auto& point = option.speed_profile[p];
                profiles_ << index << ',' << o << ',' << p << ',' << point.s_m << ',' << point.distance_m << ',' << point.curvature_1pm << ','
                          << point.speed_mps << '\n';
            }
            for (std::size_t p = 0; p < option.path.size(); ++p)
                paths_ << index << ',' << o << ',' << p << ',' << option.path[p].s_m << ',' << option.path[p].offset_m << '\n';
            for (std::size_t p = 0; p < option.trajectory.points.size(); ++p) {
                const auto& point = option.trajectory.points[p];
                trajectories_ << index << ',' << o << ',' << p << ',' << point.state.time_s << ',' << point.state.x_m << ','
                              << point.state.y_m << ',' << point.state.speed_mps << ',' << point.acceleration_mps2 << '\n';
            }
            // Only a predictive controller's plan that drove states commands; the simulation has checked there is one per point.
            for (std::size_t p = 0; p < option.trajectory.commands.size(); ++p) {
                const auto& command = option.trajectory.commands[p];
                commands_ << index << ',' << o << ',' << p << ',' << option.trajectory.points[p].state.time_s << ','
                          << command.acceleration_mps2 << ',' << command.steering_rad << '\n';
            }
        }
        if (const auto* driver = simulation.cone_driver()) {
            const auto& belief = driver->belief();
            const auto& b = belief.state;
            beliefs_ << index << ',' << b.time_s << ',' << b.x_m << ',' << b.y_m << ',' << b.yaw_rad << ',' << b.speed_mps << ','
                     << b.steering_rad << ',' << belief.pose_sampled_at_s << ',';
            if (belief.path) beliefs_ << *belief.path; else beliefs_ << "none";
            beliefs_ << '\n';
        }
        decisions_seen_ = simulation.decision_count();
    }
    const auto& s = simulation.state();
    const auto& d = simulation.diagnostics();
    if (d.revision != last_revision_) {
        if (d.revision != last_revision_+1) throw std::logic_error("Record after every step so plan revisions stay contiguous");
        write_plan(simulation, directory_/("plan-rev-"+std::to_string(d.revision)+".csv"));
        last_revision_ = d.revision;
    }
    if (d.laps > last_laps_) { summary_.lap_crossing_times_s.push_back(s.time_s); last_laps_ = d.laps; }
    telemetry_ << s.time_s << ',' << s.x_m << ',' << s.y_m << ',' << s.yaw_rad << ','
               << s.speed_mps << ',' << s.steering_rad << ',' << d.target_speed_mps << ','
               << d.cross_track_error_m << ',' << d.progress_m << ',' << d.laps << ','
               << d.requested.acceleration_mps2 << ',' << d.applied.acceleration_mps2 << ','
               << d.requested.steering_rad << ',' << d.lateral_acceleration_mps2 << ','
               << d.combined_grip_utilization << ',' << simulation.config().grip_mu << ','
               << d.revision << ',' << d.plan_valid << ',' << d.within_track << ','
               << d.control_saturated << ',' << d.nearest_index << ',' << d.limiting_index << ','
               << csv_text(d.limiting_reason) << ',' << csv_text(d.validity_reason) << ',' << d.path_s_m << ',' << d.plan_generated_time_s << ','
               << simulation.plant_state().lateral_velocity_mps << ',' << simulation.plant_state().yaw_rate_radps << ','
               << d.within_grip_envelope << ',' << d.geometric_steering_rad;
    for (const double w : simulation.plant_state().wheel_speeds_radps) telemetry_ << ',' << w;
    for (const double load : simulation.plant_state().wheel_loads_n) telemetry_ << ',' << load;
    telemetry_ << ',' << d.longitudinal_acceleration_mps2 << '\n';
    // What the car's own instruments had delivered by this tick, beside what the car truly was.
    if (const auto* m = simulation.measurements()) {
        measurements_ << s.time_s << ',' << m->pose.measured << ',' << m->pose.sampled_at_s << ',' << m->x_m << ','
                      << m->y_m << ',' << m->yaw_rad << ',' << m->speed.measured << ',' << m->speed.sampled_at_s << ','
                      << m->speed_mps << ',' << m->wheel_speeds.measured << ',' << m->wheel_speeds.sampled_at_s;
        for (const double w : m->wheel_speeds_radps) measurements_ << ',' << w;
        measurements_ << ',' << m->imu.measured << ',' << m->imu.sampled_at_s << ','
                      << m->longitudinal_acceleration_mps2 << ',' << m->lateral_acceleration_mps2 << ','
                      << m->yaw_rate_radps << ',' << m->steering.measured << ',' << m->steering.sampled_at_s << ','
                      << m->steering_rad << '\n';
    }
    // Every frame of simulated detections this tick delivered, with the truth of it for evaluation.
    if (const auto* perception = simulation.perception())
        for (std::size_t k = 0; k < perception->delivered().size(); ++k) {
            const auto& frame = perception->delivered()[k];
            const auto& truth = perception->delivered_truth()[k];
            const std::uint64_t index = perception_frames_written_++;
            perception_frames_ << index << ',' << s.time_s << ',' << frame.sensor << ',' << frame.sampled_at_s << ','
                               << frame.delivered_at_s << ',' << frame.detections.size() << ',' << truth.missed.size() << '\n';
            for (std::size_t j = 0; j < frame.detections.size(); ++j) {
                const auto& det = frame.detections[j];
                detections_ << index << ',' << j << ',' << det.position.x << ',' << det.position.y << ','
                            << csv_text(observed_colour_name(det.colour)) << ',' << det.colour_probability << ','
                            << det.detection_probability << ',' << det.covariance_xx << ',' << det.covariance_xy << ','
                            << det.covariance_yy << ',' << truth.source_cone[j] << '\n';
            }
            for (const auto cone : truth.missed) missed_ << index << ',' << cone << '\n';
        }
    // Everything the judge saw since the last record.
    const auto& judged = simulation.judge().events();
    for (; timing_written_ < judged.size(); ++timing_written_) {
        const auto& e = judged[timing_written_];
        timing_ << timing_written_ << ',' << csv_text(timing_kind_name(e.kind)) << ',' << e.time_s << ',' << e.lap << ',' << e.seconds << ',';
        if (e.cone) timing_ << *e.cone; else timing_ << "none";
        timing_ << ',' << e.position.x << ',' << e.position.y << ',' << csv_optional_text(e.reason) << '\n';
    }
    if (const auto* driver = simulation.cone_driver()) {
        const auto& recent = driver->recent_paths();
        if (!recent.empty() && recent.front().index > believed_paths_written_)
            throw std::logic_error("Record after every step so every believed path is kept");
        for (const auto& believed : recent) {
            if (believed.index < believed_paths_written_) continue;
            for (std::size_t k = 0; k < believed.path.points.size(); ++k) {
                const auto& p = believed.path.points[k];
                believed_paths_ << believed.index << ',' << believed.frame << ',' << k << ',' << p.x_m << ',' << p.y_m << ','
                                << p.s_m << ',' << p.curvature << ',' << believed.path.width_m << ',' << believed.pose.x_m << ','
                                << believed.pose.y_m << ',' << believed.pose.yaw_rad << '\n';
            }
            believed_paths_written_ = believed.index+1;
        }
    }
    summary_.max_cross_track_error_m = std::max(summary_.max_cross_track_error_m, std::abs(d.cross_track_error_m));
    summary_.max_speed_mps = std::max(summary_.max_speed_mps, s.speed_mps);
    summary_.max_combined_grip_utilization = std::max(summary_.max_combined_grip_utilization, d.combined_grip_utilization);
    summary_.max_rear_sideslip_rad = std::max(summary_.max_rear_sideslip_rad, std::abs(d.rear_sideslip_rad));
    sum_error_sq_ += d.cross_track_error_m*d.cross_track_error_m;
    ++summary_.samples;
    if (!d.plan_valid) ++summary_.invalid_samples;
}

RunSummary RecordingWriter::finish(const Simulation& simulation) {
    if (finished_) throw std::logic_error("Recording is already finished");
    finished_ = true;
    telemetry_.close();
    if (!telemetry_) throw std::runtime_error("Writing telemetry.csv failed");
    decisions_.close();
    if (!decisions_) throw std::runtime_error("Writing decisions.csv failed");
    trajectories_.close();
    if (!trajectories_) throw std::runtime_error("Writing trajectories.csv failed");
    paths_.close();
    if (!paths_) throw std::runtime_error("Writing paths.csv failed");
    profiles_.close();
    if (!profiles_) throw std::runtime_error("Writing profiles.csv failed");
    commands_.close();
    if (!commands_) throw std::runtime_error("Writing commands.csv failed");
    measurements_.close();
    if (!measurements_) throw std::runtime_error("Writing measurements.csv failed");
    perception_frames_.close();
    if (!perception_frames_) throw std::runtime_error("Writing perception_frames.csv failed");
    detections_.close();
    if (!detections_) throw std::runtime_error("Writing detections.csv failed");
    missed_.close();
    if (!missed_) throw std::runtime_error("Writing missed.csv failed");
    beliefs_.close();
    if (!beliefs_) throw std::runtime_error("Writing beliefs.csv failed");
    believed_paths_.close();
    if (!believed_paths_) throw std::runtime_error("Writing believed_paths.csv failed");
    timing_.close();
    if (!timing_) throw std::runtime_error("Writing timing.csv failed");
    auto events = open_output(directory_/"events.csv");
    events << events_header << '\n';
    for (const auto& e : simulation.events())
        events << e.time_s << ',' << e.revision << ',' << csv_text(e.parameter) << ',' << e.old_value << ',' << e.new_value << '\n';
    summary_.completed = request_.requested_laps == 0 || simulation.diagnostics().laps >= request_.requested_laps;
    summary_.laps = simulation.diagnostics().laps;
    summary_.elapsed_simulation_s = simulation.state().time_s;
    summary_.parameter_events = simulation.events().size();
    summary_.rms_cross_track_error_m = std::sqrt(sum_error_sq_/static_cast<double>(summary_.samples));
    auto summary = open_output(directory_/"summary.json");
    summary << "{\n  \"completed\": " << (summary_.completed ? "true" : "false") << ",\n"
               "  \"laps\": " << summary_.laps << ",\n"
               "  \"elapsed_simulation_s\": " << summary_.elapsed_simulation_s << ",\n"
               "  \"samples\": " << summary_.samples << ",\n"
               "  \"max_cross_track_error_m\": " << summary_.max_cross_track_error_m << ",\n"
               "  \"rms_cross_track_error_m\": " << summary_.rms_cross_track_error_m << ",\n"
               "  \"max_speed_mps\": " << summary_.max_speed_mps << ",\n"
               "  \"max_combined_grip_utilization\": " << summary_.max_combined_grip_utilization << ",\n"
               "  \"max_rear_sideslip_rad\": " << summary_.max_rear_sideslip_rad << ",\n"
               "  \"invalid_samples\": " << summary_.invalid_samples << ",\n"
               "  \"parameter_events\": " << summary_.parameter_events << ",\n"
               "  \"lap_crossing_times_s\": [";
    for (std::size_t i = 0; i < summary_.lap_crossing_times_s.size(); ++i) summary << (i ? ", " : "") << summary_.lap_crossing_times_s[i];
    summary << "]\n}\n";
    return summary_;
}

// ---------------------------------------------------------------- Playback

Playback::Playback(Recording recording) : recording_(std::move(recording)) {
    if (recording_.samples.empty() || recording_.plans.empty()) throw std::invalid_argument("Playback requires a loaded recording with samples");
    seek_index(0);
}

void Playback::seek_index(std::size_t index) {
    index_ = std::min(index, recording_.samples.size()-1);
    cursor_time_s_ = sample().time_s;
    sync_revision();
}

void Playback::seek_time(double time_s) {
    if (!std::isfinite(time_s)) throw std::invalid_argument("Seek time must be finite");
    const auto& samples = recording_.samples;
    const auto upper = std::upper_bound(samples.begin(), samples.end(), time_s,
                                        [](double t, const RecordedSample& s) { return t < s.time_s; });
    index_ = upper == samples.begin() ? 0 : static_cast<std::size_t>(upper-samples.begin()-1);
    cursor_time_s_ = std::clamp(time_s, samples.front().time_s, samples.back().time_s);
    sync_revision();
}

bool Playback::advance(double dt_s) {
    if (!std::isfinite(dt_s) || dt_s < 0) throw std::invalid_argument("Playback advance requires a finite nonnegative duration");
    if (at_end()) { cursor_time_s_ = duration_s(); return false; }
    seek_time(cursor_time_s_+dt_s);
    return true;
}

const PlanRevision& Playback::plan() const { return recording_.plan_for_revision(sample().revision); }

Demand Playback::demand() const {
    return fd::demand(recording_.metadata.vehicle_model, sample().plant_state(), config_, sample().applied_acceleration_mps2);
}

void Playback::sync_revision() {
    const auto revision = sample().revision;
    if (revision != config_revision_) { config_ = recording_.config_for_revision(revision); config_revision_ = revision; }
    const auto* decision = recording_.decision_at(sample().time_s);
    decision_index_ = decision ? static_cast<std::size_t>(decision-recording_.decisions.data()) : static_cast<std::size_t>(-1);
}

}  // namespace fd

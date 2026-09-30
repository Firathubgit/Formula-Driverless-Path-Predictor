#include "fd/optimal_lap.hpp"
#include "fd/fingerprint.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace fd {
namespace {

[[noreturn]] void fail(const std::string& message) { throw std::runtime_error("Optimal lap: "+message); }

#include "contract_io.inc"

constexpr std::array<const char*, 4> wheel_names{"fl", "fr", "rl", "rr"};

// Every number of the problem, named as problem.json names it, read and written through one list so the two agree.
struct Field { const char* name; double* value; };
std::vector<Field> tire_fields(TireParameters& t) {
    return {{"reference_load_low_n", &t.reference_load_low_n}, {"reference_load_high_n", &t.reference_load_high_n},
            {"peak_friction_longitudinal_low", &t.peak_friction_longitudinal_low},
            {"peak_friction_longitudinal_high", &t.peak_friction_longitudinal_high},
            {"peak_friction_lateral_low", &t.peak_friction_lateral_low}, {"peak_friction_lateral_high", &t.peak_friction_lateral_high},
            {"peak_slip_ratio_low", &t.peak_slip_ratio_low}, {"peak_slip_ratio_high", &t.peak_slip_ratio_high},
            {"peak_slip_angle_low_rad", &t.peak_slip_angle_low_rad}, {"peak_slip_angle_high_rad", &t.peak_slip_angle_high_rad},
            {"sliding_fraction_longitudinal", &t.sliding_fraction_longitudinal},
            {"sliding_fraction_lateral", &t.sliding_fraction_lateral}, {"minimum_friction", &t.minimum_friction}};
}
std::vector<Field> car_fields(FourWheelCar& c) {
    return {{"mass_kg", &c.mass_kg}, {"yaw_inertia_kgm2", &c.yaw_inertia_kgm2}, {"cg_to_rear_m", &c.cg_to_rear_m},
            {"cg_height_m", &c.cg_height_m}, {"roll_balance_front", &c.roll_balance_front}, {"track_front_m", &c.track_front_m},
            {"track_rear_m", &c.track_rear_m}, {"wheel_radius_m", &c.wheel_radius_m}, {"wheel_inertia_kgm2", &c.wheel_inertia_kgm2},
            {"drive_front_fraction", &c.drive_front_fraction}, {"max_drive_power_w", &c.max_drive_power_w},
            {"max_drive_torque_nm", &c.max_drive_torque_nm}, {"brake_bias_front", &c.brake_bias_front},
            {"max_brake_torque_nm", &c.max_brake_torque_nm}, {"viscous_coupling_nms", &c.viscous_coupling_nms},
            {"drag_area_m2", &c.drag_area_m2}, {"downforce_area_m2", &c.downforce_area_m2},
            {"aero_balance_front", &c.aero_balance_front}, {"kinematic_below_mps", &c.kinematic_below_mps},
            {"dynamic_above_mps", &c.dynamic_above_mps}, {"slip_speed_floor_mps", &c.slip_speed_floor_mps}, {"substep_s", &c.substep_s}};
}
std::vector<Field> config_fields(Config& c) {
    return {{"grip_mu", &c.grip_mu}, {"fixed_dt_s", &c.fixed_dt_s}, {"control_dt_s", &c.control_dt_s}, {"wheelbase_m", &c.wheelbase_m},
            {"max_speed_mps", &c.max_speed_mps}, {"max_steering_rad", &c.max_steering_rad},
            {"max_steering_rate_radps", &c.max_steering_rate_radps}, {"lateral_grip_fraction", &c.lateral_grip_fraction},
            {"longitudinal_grip_fraction", &c.longitudinal_grip_fraction}, {"speed_gain", &c.speed_gain},
            {"lookahead_base_m", &c.lookahead_base_m}, {"lookahead_time_s", &c.lookahead_time_s},
            {"envelope_fraction", &c.envelope_fraction}};
}

std::string escaped(const std::string& text) {
    std::string out;
    for (const char c : text) {
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if (static_cast<unsigned char>(c) < 0x20) {
            std::ostringstream code;
            code << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
            out += code.str();
        } else out += c;
    }
    return out;
}

bool is_centreline(const Track& track) { return track.left_edge_m.empty(); }
double left_edge(const Track& track, std::size_t i) { return is_centreline(track) ? track.width_m/2 : track.left_edge_m[i]; }
double right_edge(const Track& track, std::size_t i) { return is_centreline(track) ? track.width_m/2 : track.right_edge_m[i]; }

std::string corridor_csv(const LapProblem& problem) {
    std::ostringstream out;
    out << std::setprecision(17);
    out << "s_m,x_m,y_m,heading_rad,curvature_1pm,left_normal_x,left_normal_y,left_m,right_m\n";
    const auto& t = problem.corridor;
    for (std::size_t i = 0; i < t.points.size(); ++i) {
        const auto& p = t.points[i];
        const auto& n = problem.left_normals[i];
        out << p.s_m << ',' << p.x_m << ',' << p.y_m << ',' << std::atan2(-n.x, n.y) << ',' << p.curvature << ',' << n.x << ',' << n.y
            << ',' << left_edge(t, i) << ',' << right_edge(t, i) << '\n';
    }
    return out.str();
}

void write_text(const std::filesystem::path& path, const std::string& text) {
    std::ofstream file(path, std::ios::binary);
    if (!file) fail("cannot write "+path.string());
    file << text;
    if (!file) fail("cannot write "+path.string());
}

std::string read_text(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) fail("missing file "+path.filename().string());
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string text = buffer.str();
    std::string out;
    out.reserve(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) if (!(text[i] == '\r' && i+1 < text.size() && text[i+1] == '\n')) out += text[i];
    return out;
}

void read_fields(const Json& object, std::vector<Field> fields, const std::string& where) {
    if (object.keys.size() != fields.size()) fail(where+" must hold exactly its "+std::to_string(fields.size())+" members");
    for (auto& f : fields) *f.value = number_member(object, f.name, where);
}

// The problem stated in problem.json and corridor.csv, rebuilt; lap_problem_json of it must give problem.json back.
LapProblem read_problem(const std::filesystem::path& directory) {
    const Json root = read_json(directory/"problem.json");
    if (root.type != Json::Type::Object) fail("problem.json must be an object");
    const std::string where = "problem";
    if (integer_member(root, "schema_version", where) != optimal_lap_schema_version) fail("problem.json has another schema_version");
    if (string_member(root, "kind", where) != "minimum-time problem") fail("problem.json is not a minimum-time problem");
    LapProblem problem;
    problem.profile = string_member(root, "profile", where);
    read_fields(object_member(root, "config", where), config_fields(problem.config), "problem.config");
    const Json& car = object_member(root, "car", where);
    auto fields = car_fields(problem.car);
    for (auto& f : fields) *f.value = number_member(car, f.name, "problem.car");
    read_fields(object_member(car, "front_tire", "problem.car"), tire_fields(problem.car.front_tire), "problem.car.front_tire");
    read_fields(object_member(car, "rear_tire", "problem.car"), tire_fields(problem.car.rear_tire), "problem.car.rear_tire");
    if (car.keys.size() != fields.size()+2) fail("problem.car must hold exactly its members");
    const Json& corridor = object_member(root, "corridor", where);
    auto& track = problem.corridor;
    track.name = string_member(corridor, "name", "problem.corridor");
    track.width_m = number_member(corridor, "width_m", "problem.corridor");
    track.length_m = number_member(corridor, "length_m", "problem.corridor");
    const bool centreline = bool_member(corridor, "centreline", "problem.corridor");
    const int samples = integer_member(corridor, "samples", "problem.corridor");
    const std::string corridor_fingerprint = string_member(corridor, "fingerprint", "problem.corridor");
    if (fingerprint_hex(read_text(directory/"corridor.csv")) != corridor_fingerprint)
        fail("corridor.csv is not the corridor problem.json states");
    const Table table = read_csv(directory/"corridor.csv", "s_m,x_m,y_m,heading_rad,curvature_1pm,left_normal_x,left_normal_y,left_m,right_m");
    if (static_cast<int>(table.rows.size()) != samples) fail("corridor.csv has another number of samples than problem.json states");
    for (std::size_t i = 0; i < table.rows.size(); ++i) {
        const RowReader row(table, i);
        track.points.push_back({row.number("x_m"), row.number("y_m"), row.number("s_m"), row.number("curvature_1pm")});
        problem.left_normals.push_back({row.number("left_normal_x"), row.number("left_normal_y")});
        if (!centreline) {
            track.left_edge_m.push_back(row.number("left_m"));
            track.right_edge_m.push_back(row.number("right_m"));
        }
    }
    try { validate_track(track); validate_vehicle(problem.car, problem.config); }
    catch (const std::exception& error) { fail(std::string("the stated problem is invalid: ")+error.what()); }
    return problem;
}

// Station interpolation on the corridor: the centre and normal between the two samples a station lies between.
struct CorridorAt { Vec2 centre, normal; double left_m{}, right_m{}; };
CorridorAt corridor_at_station(const LapProblem& problem, double s_m) {
    const auto& t = problem.corridor;
    double s = std::fmod(s_m, t.length_m);
    if (s < 0) s += t.length_m;
    const auto it = std::upper_bound(t.points.begin(), t.points.end(), s, [](double v, const PathPoint& p) { return v < p.s_m; });
    const std::size_t i = it == t.points.begin() ? 0 : static_cast<std::size_t>(it-t.points.begin())-1;
    const std::size_t j = (i+1)%t.points.size();
    const double span = (j == 0 ? t.length_m : t.points[j].s_m)-t.points[i].s_m;
    const double f = span > 0 ? (s-t.points[i].s_m)/span : 0;
    const auto mix = [f](double a, double b) { return a+(b-a)*f; };
    Vec2 n{mix(problem.left_normals[i].x, problem.left_normals[j].x), mix(problem.left_normals[i].y, problem.left_normals[j].y)};
    const double length = std::hypot(n.x, n.y);
    n = {n.x/length, n.y/length};
    return {{mix(t.points[i].x_m, t.points[j].x_m), mix(t.points[i].y_m, t.points[j].y_m)}, n,
            mix(left_edge(t, i), left_edge(t, j)), mix(right_edge(t, i), right_edge(t, j))};
}

}  // namespace

double OptimalPoint::speed_mps() const { return std::hypot(forward_mps, lateral_mps); }

LapProblem make_lap_problem(const Track& track, const FourWheelCar& car, const Config& config, std::string profile) {
    validate_track(track);
    validate_vehicle(car, config);
    // fastest-lap's car has no drive controller top speed (decision 0038), so its optimum would be another car's.
    if (std::isfinite(car.max_drive_speed_mps))
        throw std::invalid_argument("A car whose drive controller has a top speed has no lap problem: fastest-lap's model cannot limit its drive");
    LapProblem problem{track, {}, car, config, std::move(profile)};
    const auto& p = track.points;
    const std::size_t n = p.size();
    for (std::size_t i = 0; i < n; ++i) {
        const auto& a = p[(i+n-1)%n];
        const auto& b = p[(i+1)%n];
        const double dx = b.x_m-a.x_m, dy = b.y_m-a.y_m, length = std::hypot(dx, dy);
        problem.left_normals.push_back({-dy/length, dx/length});
    }
    return problem;
}

std::string lap_problem_json(const LapProblem& problem) {
    LapProblem copy = problem;
    std::ostringstream out;
    out << std::setprecision(17);
    const auto fields = [&out](const std::vector<Field>& list, const char* indent) {
        for (std::size_t i = 0; i < list.size(); ++i)
            out << indent << '"' << list[i].name << "\": " << *list[i].value << (i+1 < list.size() ? ",\n" : "\n");
    };
    out << "{\n  \"schema_version\": " << optimal_lap_schema_version << ",\n  \"kind\": \"minimum-time problem\",\n"
        << "  \"profile\": \"" << escaped(copy.profile) << "\",\n  \"corridor\": {\n"
        << "    \"name\": \"" << escaped(copy.corridor.name) << "\",\n    \"width_m\": " << copy.corridor.width_m << ",\n"
        << "    \"length_m\": " << copy.corridor.length_m << ",\n    \"centreline\": " << (is_centreline(copy.corridor) ? "true" : "false")
        << ",\n    \"samples\": " << copy.corridor.points.size() << ",\n    \"fingerprint\": \"" << fingerprint_hex(corridor_csv(copy))
        << "\"\n  },\n  \"config\": {\n";
    fields(config_fields(copy.config), "    ");
    out << "  },\n  \"car\": {\n";
    auto car = car_fields(copy.car);
    for (const auto& f : car) out << "    \"" << f.name << "\": " << *f.value << ",\n";
    out << "    \"front_tire\": {\n";
    fields(tire_fields(copy.car.front_tire), "      ");
    out << "    },\n    \"rear_tire\": {\n";
    fields(tire_fields(copy.car.rear_tire), "      ");
    out << "    }\n  }\n}\n";
    return out.str();
}

std::string lap_problem_fingerprint(const LapProblem& problem) { return fingerprint_hex(lap_problem_json(problem)); }

void write_lap_problem(const std::filesystem::path& directory, const LapProblem& problem) {
    if (std::filesystem::exists(directory) && !std::filesystem::is_empty(directory))
        fail("the problem directory is not empty; choose a fresh one");
    std::filesystem::create_directories(directory);
    write_text(directory/"corridor.csv", corridor_csv(problem));
    write_text(directory/"problem.json", lap_problem_json(problem));
}

OptimalPoint OptimalLap::at_time(double time_s) const {
    double t = std::fmod(time_s, lap_time_s);
    if (t < 0) t += lap_time_s;
    const auto it = std::upper_bound(points.begin(), points.end(), t, [](double v, const OptimalPoint& p) { return v < p.time_s; });
    const std::size_t i = it == points.begin() ? 0 : static_cast<std::size_t>(it-points.begin())-1;
    const std::size_t j = (i+1)%points.size();
    const double end = j == 0 ? lap_time_s : points[j].time_s;
    const double f = end > points[i].time_s ? (t-points[i].time_s)/(end-points[i].time_s) : 0;
    const auto mix = [f](double a, double b) { return a+(b-a)*f; };
    const auto& a = points[i];
    const auto& b = points[j];
    OptimalPoint p = a;
    p.time_s = t;
    p.s_m = j == 0 ? mix(a.s_m, problem.corridor.length_m) : mix(a.s_m, b.s_m);
    p.x_m = mix(a.x_m, b.x_m);
    p.y_m = mix(a.y_m, b.y_m);
    p.cog_x_m = mix(a.cog_x_m, b.cog_x_m);
    p.cog_y_m = mix(a.cog_y_m, b.cog_y_m);
    p.yaw_rad = a.yaw_rad+std::remainder(b.yaw_rad-a.yaw_rad, 2*std::numbers::pi)*f;
    p.offset_m = mix(a.offset_m, b.offset_m);
    p.forward_mps = mix(a.forward_mps, b.forward_mps);
    p.lateral_mps = mix(a.lateral_mps, b.lateral_mps);
    p.yaw_rate_radps = mix(a.yaw_rate_radps, b.yaw_rate_radps);
    p.steering_rad = mix(a.steering_rad, b.steering_rad);
    p.throttle = mix(a.throttle, b.throttle);
    for (std::size_t w = 0; w < 4; ++w) {
        p.slip_ratio[w] = mix(a.slip_ratio[w], b.slip_ratio[w]);
        p.slip_angle_rad[w] = mix(a.slip_angle_rad[w], b.slip_angle_rad[w]);
        p.force_x_n[w] = mix(a.force_x_n[w], b.force_x_n[w]);
        p.force_y_n[w] = mix(a.force_y_n[w], b.force_y_n[w]);
        p.load_n[w] = mix(a.load_n[w], b.load_n[w]);
        p.dissipation_w[w] = mix(a.dissipation_w[w], b.dissipation_w[w]);
        p.energy_j[w] = mix(a.energy_j[w], j == 0 ? tire_energy_j[w] : b.energy_j[w]);
    }
    return p;
}

double OptimalLap::time_at_station(double s_m) const {
    const double length = problem.corridor.length_m;
    double s = std::fmod(s_m, length);
    if (s < 0) s += length;
    const auto it = std::upper_bound(points.begin(), points.end(), s, [](double v, const OptimalPoint& p) { return v < p.s_m; });
    const std::size_t i = it == points.begin() ? 0 : static_cast<std::size_t>(it-points.begin())-1;
    const std::size_t j = (i+1)%points.size();
    const double s_end = j == 0 ? length : points[j].s_m, t_end = j == 0 ? lap_time_s : points[j].time_s;
    const double f = s_end > points[i].s_m ? (s-points[i].s_m)/(s_end-points[i].s_m) : 0;
    return points[i].time_s+(t_end-points[i].time_s)*f;
}

std::vector<double> OptimalLap::gate_times(const std::vector<Gate>& gates) const {
    std::vector<double> times(gates.size(), -1.0);
    // The rear axle along the lap, the closing span back to the first point included, crossing each gate's line from
    // behind to ahead between its ends, as the judge sees it. The solver's lap begins with the centre of gravity on its
    // start line, so the rear axle crosses the start gate a moment later.
    for (std::size_t g = 0; g < gates.size(); ++g) {
        const Vec2 across{gates[g].right.x-gates[g].left.x, gates[g].right.y-gates[g].left.y};
        const double length = std::hypot(across.x, across.y);
        const Vec2 forward{-across.y/length, across.x/length};
        const auto side = [&](double x, double y) {
            const double along = ((x-gates[g].left.x)*across.x+(y-gates[g].left.y)*across.y)/length;
            if (along < 0 || along > length) return std::numeric_limits<double>::quiet_NaN();
            return (x-gates[g].left.x)*forward.x+(y-gates[g].left.y)*forward.y;
        };
        for (std::size_t i = 0; i < points.size(); ++i) {
            const std::size_t j = (i+1)%points.size();
            const double a = side(points[i].x_m, points[i].y_m), b = side(points[j].x_m, points[j].y_m);
            if (std::isnan(a) || std::isnan(b) || !(a <= 1e-9 && b > 1e-9)) continue;
            const double end = j == 0 ? lap_time_s : points[j].time_s;
            times[g] = points[i].time_s+(end-points[i].time_s)*(a == b ? 0 : -a/(b-a));
            break;
        }
    }
    return times;
}

OptimalLap load_optimal_lap(const std::filesystem::path& directory) {
    OptimalLap lap;
    lap.directory = directory;
    const Json root = read_json(directory/"optimal_lap.json");
    if (root.type != Json::Type::Object) fail("optimal_lap.json must be an object");
    const std::string where = "optimal_lap";
    lap.schema_version = integer_member(root, "schema_version", where);
    if (lap.schema_version != optimal_lap_schema_version)
        fail("unsupported schema_version "+std::to_string(lap.schema_version)+"; expected "+std::to_string(optimal_lap_schema_version));
    if (string_member(root, "kind", where) != "reference-optimal") fail("optimal_lap.json is not a reference-optimal artefact");
    lap.label = string_member(root, "label", where);
    if (lap.label.empty()) fail("the artefact must say what it is");
    const Json& solver = object_member(root, "solver", where);
    lap.tool = string_member(solver, "tool", "solver");
    lap.version = string_member(solver, "version", "solver");
    lap.release = string_member(solver, "release", "solver");
    lap.release_sha256 = string_member(solver, "release_sha256", "solver");
    lap.library_sha256 = string_member(solver, "library_sha256", "solver");
    lap.model = string_member(solver, "model", "solver");
    lap.nlp_solver = string_member(solver, "nlp_solver", "solver");
    lap.status = string_member(solver, "status", "solver");
    lap.converged = bool_member(solver, "converged", "solver");
    lap.iterations = integer_member(solver, "iterations", "solver");
    lap.max_iterations = integer_member(solver, "max_iterations", "solver");
    lap.tolerance = number_member(solver, "tolerance", "solver");
    for (const auto* digest : {&lap.release_sha256, &lap.library_sha256})
        if (digest->size() != 64 || digest->find_first_not_of("0123456789abcdef") != std::string::npos)
            fail("solver digests must be 64 lowercase hexadecimal digits");
    if (lap.tool.empty() || lap.version.empty() || lap.model.empty() || lap.status.empty()) fail("the solver must be named with its version, model and status");
    // A solution the solver accepts is one of these two exits; anything else, including running out of iterations, is not.
    const bool accepted = lap.status == "Optimal Solution Found" || lap.status == "Solved To Acceptable Level";
    if (lap.converged != accepted) fail("solver.converged contradicts the status \""+lap.status+"\"");
    if (lap.iterations < 0 || lap.max_iterations <= 0 || lap.iterations > lap.max_iterations) fail("solver iterations are out of range");
    if (!(lap.tolerance > 0 && lap.tolerance < 1)) fail("solver.tolerance must lie in (0, 1)");
    lap.mesh_points = integer_member(root, "mesh_points", where);
    lap.lap_time_s = number_member(root, "lap_time_s", where);
    const Json& objective = member(root, "objective", where);
    if (objective.type == Json::Type::Number) lap.objective = objective.number;
    else if (objective.type != Json::Type::Null || lap.converged) fail("objective must be a number, and may be null only for a failed solve");
    if (lap.objective && *lap.objective < lap.lap_time_s-1e-6) fail("the objective cannot be less than the lap time it includes");
    lap.speed_cap_mps = number_member(root, "speed_cap_mps", where);
    lap.problem_fingerprint = string_member(root, "problem_fingerprint", where);
    for (const auto& item : array_member(root, "mapping", where).array) {
        if (item.type != Json::Type::String || item.string.empty()) fail("mapping must be statements");
        lap.mapping.push_back(item.string);
    }
    if (lap.mapping.empty()) fail("the artefact must state how the car was put into the solver's model");
    const Json& energy = array_member(root, "tire_energy_j", where);
    if (energy.array.size() != 4) fail("tire_energy_j must hold the four tires");
    for (std::size_t w = 0; w < 4; ++w) {
        if (energy.array[w].type != Json::Type::Number || !(energy.array[w].number >= 0)) fail("tire energies must be nonnegative numbers");
        lap.tire_energy_j[w] = energy.array[w].number;
    }
    for (const auto& item : array_member(root, "sensitivities", where).array) {
        if (item.type != Json::Type::Object) fail("sensitivities must be objects");
        LapSensitivity s{string_member(item, "parameter", "sensitivity"), string_member(item, "unit", "sensitivity"),
                         number_member(item, "value", "sensitivity"), number_member(item, "seconds_per_unit", "sensitivity"),
                         number_member(item, "step", "sensitivity"), string_member(item, "method", "sensitivity"),
                         number_member(item, "half_step", "sensitivity"), std::nullopt};
        const Json& check = member(item, "cross_check_seconds_per_unit", "sensitivity");
        if (check.type == Json::Type::Number) s.cross_check_seconds_per_unit = check.number;
        else if (check.type != Json::Type::Null) fail("sensitivity.cross_check_seconds_per_unit must be a number or null");
        for (const auto& other : lap.sensitivities) if (other.parameter == s.parameter) fail("sensitivity "+s.parameter+" is stated twice");
        lap.sensitivities.push_back(std::move(s));
    }

    // The problem, identical to the one it states and to the fingerprint the solution was made for.
    lap.problem = read_problem(directory);
    const std::string stated = read_text(directory/"problem.json");
    if (lap_problem_json(lap.problem) != stated) fail("problem.json does not read back as the problem it states");
    if (lap_problem_fingerprint(lap.problem) != lap.problem_fingerprint) fail("the solution was made for another problem than problem.json");
    if (!(lap.speed_cap_mps > 0) || lap.speed_cap_mps > lap.problem.config.max_speed_mps+1e-9)
        fail("the speed cap must be positive and no higher than the car's own");
    const double half_width = number_member(root, "vehicle_half_width_m", where);
    if (!(half_width >= 0)) fail("vehicle_half_width_m must be nonnegative");

    // The mesh.
    std::string header = "point,s_m,time_s,cog_x_m,cog_y_m,yaw_rad,offset_m,forward_mps,lateral_mps,yaw_rate_radps,steering_rad,throttle";
    for (const auto* w : wheel_names)
        header += std::string(",slip_ratio_")+w+",slip_angle_"+w+"_rad,force_x_"+w+"_n,force_y_"+w+"_n,load_"+w+"_n,dissipation_"+w+"_w";
    const Table table = read_csv(directory/"lap.csv", header);
    if (lap.mesh_points < 10 || static_cast<int>(table.rows.size()) != lap.mesh_points) fail("lap.csv must hold mesh_points points, at least ten");
    const double length = lap.problem.corridor.length_m;
    const double rear = lap.problem.car.cg_to_rear_m;
    for (std::size_t i = 0; i < table.rows.size(); ++i) {
        const RowReader row(table, i);
        if (row.index("point") != i) fail("lap.csv points must be numbered from zero");
        OptimalPoint p;
        p.s_m = row.number("s_m");
        p.time_s = row.number("time_s");
        p.cog_x_m = row.number("cog_x_m");
        p.cog_y_m = row.number("cog_y_m");
        p.yaw_rad = row.number("yaw_rad");
        p.offset_m = row.number("offset_m");
        p.forward_mps = row.number("forward_mps");
        p.lateral_mps = row.number("lateral_mps");
        p.yaw_rate_radps = row.number("yaw_rate_radps");
        p.steering_rad = row.number("steering_rad");
        p.throttle = row.number("throttle");
        for (std::size_t w = 0; w < 4; ++w) {
            const std::string n = wheel_names[w];
            p.slip_ratio[w] = row.number("slip_ratio_"+n);
            p.slip_angle_rad[w] = row.number("slip_angle_"+n+"_rad");
            p.force_x_n[w] = row.number("force_x_"+n+"_n");
            p.force_y_n[w] = row.number("force_y_"+n+"_n");
            p.load_n[w] = row.number("load_"+n+"_n");
            p.dissipation_w[w] = row.number("dissipation_"+n+"_w");
            if (p.load_n[w] < -1e-6) fail(row.where("load_"+n+"_n")+" is negative");
            if (p.dissipation_w[w] < -1e-6) fail(row.where("dissipation_"+n+"_w")+" is negative");
        }
        // The rear axle, the pose this project publishes, from the centre of gravity at the boundary.
        p.x_m = p.cog_x_m-rear*std::cos(p.yaw_rad);
        p.y_m = p.cog_y_m-rear*std::sin(p.yaw_rad);
        if (i == 0 ? (std::abs(p.s_m) > 1e-9 || std::abs(p.time_s) > 1e-9)
                   : !(p.s_m > lap.points.back().s_m && p.time_s > lap.points.back().time_s))
            fail("lap.csv line "+std::to_string(i+2)+": the mesh must start on the line and increase in station and time");
        if (p.s_m >= length) fail("lap.csv line "+std::to_string(i+2)+" lies beyond the corridor's length");
        if (!(p.forward_mps > 0) || p.forward_mps > lap.speed_cap_mps*(1+1e-6)+1e-6)
            fail("lap.csv line "+std::to_string(i+2)+": forward speed must be positive and within the speed cap");
        if (std::abs(p.steering_rad) > lap.problem.config.max_steering_rad+1e-6)
            fail("lap.csv line "+std::to_string(i+2)+": steering beyond the car's lock");
        // Where the solver says the car is, against where its station and offset put it on this corridor.
        const auto at = corridor_at_station(lap.problem, p.s_m);
        const double dx = at.centre.x+at.normal.x*p.offset_m-p.cog_x_m, dy = at.centre.y+at.normal.y*p.offset_m-p.cog_y_m;
        if (std::hypot(dx, dy) > 0.05) fail("lap.csv line "+std::to_string(i+2)+": the position is "+std::to_string(std::hypot(dx, dy))+
                                            " m from its station and offset on the corridor");
        if (p.offset_m > at.left_m-half_width+1e-3 || -p.offset_m > at.right_m-half_width+1e-3)
            fail("lap.csv line "+std::to_string(i+2)+": the car's half width leaves the corridor");
        lap.points.push_back(p);
    }
    // Time, distance and speed agree span by span, the closing span included, which also accounts the lap time.
    for (std::size_t i = 0; i < lap.points.size(); ++i) {
        const auto& a = lap.points[i];
        const auto& b = lap.points[(i+1)%lap.points.size()];
        const double dt = (i+1 == lap.points.size() ? lap.lap_time_s : b.time_s)-a.time_s;
        if (!(dt > 0)) fail("the lap time must exceed the last point's time");
        const double distance = std::hypot(b.cog_x_m-a.cog_x_m, b.cog_y_m-a.cog_y_m);
        const double covered = 0.5*(a.speed_mps()+b.speed_mps())*dt;
        if (std::abs(distance-covered) > 0.02*distance+0.01)
            fail("lap.csv line "+std::to_string(i+2)+": "+std::to_string(distance)+" m travelled in "+std::to_string(dt)+" s at "+
                 std::to_string(covered/dt)+" m/s");
    }
    // Each tire's energy: its dissipation integrated over the lap here, which must be the solver's own integral.
    std::array<double, 4> total{};
    for (std::size_t i = 0; i < lap.points.size(); ++i) {
        lap.points[i].energy_j = total;
        const auto& a = lap.points[i];
        const auto& b = lap.points[(i+1)%lap.points.size()];
        const double dt = (i+1 == lap.points.size() ? lap.lap_time_s : b.time_s)-a.time_s;
        for (std::size_t w = 0; w < 4; ++w) total[w] += 0.5*(a.dissipation_w[w]+b.dissipation_w[w])*dt;
    }
    for (std::size_t w = 0; w < 4; ++w)
        if (std::abs(total[w]-lap.tire_energy_j[w]) > 0.02*lap.tire_energy_j[w]+1.0)
            fail(std::string("tire ")+wheel_names[w]+" dissipated "+std::to_string(total[w])+" J over the mesh but the solver states "+
                 std::to_string(lap.tire_energy_j[w])+" J");
    for (const auto& s : lap.sensitivities) {
        if (s.parameter.empty() || s.unit.empty() || s.method.empty() || !std::isfinite(s.seconds_per_unit) || s.step == 0 ||
            !(s.half_step > 0))
            fail("sensitivity "+s.parameter+" must name its parameter, unit and method, with a finite derivative and its steps");
        // The solver's own derivative against the central difference of two re-solves, compared over the quoted step.
        if (s.cross_check_seconds_per_unit) {
            const double own = s.seconds_per_unit*s.step, difference = *s.cross_check_seconds_per_unit*s.step;
            if (std::abs(own-difference) > 0.05*std::max(std::abs(own), std::abs(difference))+1e-3)
                fail("sensitivity "+s.parameter+" is worth "+std::to_string(own)+" s over its step by the solver but "+
                     std::to_string(difference)+" s by re-solving");
        }
    }
    return lap;
}

}  // namespace fd

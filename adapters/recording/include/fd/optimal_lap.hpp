#pragma once
#include "fd/competition.hpp"
#include "fd/core.hpp"
#include "fd/vehicle.hpp"

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace fd {

// The theoretical best lap (TrackWayFastPlan Phase 4, decision 0034). An offline minimum-time solver, fastest-lap, is
// given a lap problem written here, and returns a reference-optimal artefact this reader validates as one contract,
// as a recording is. The optimum is the solver's, for its own model at its own mesh, tolerance and status, all stated
// with it; nothing here solves anything, and nothing here drives the car.

// The problem: the four-wheel car on a closed corridor, the track conditioned as a racing line's reference is
// (decision 0019), with the configuration's steering lock and speed cap.
struct LapProblem {
    Track corridor;                  // closed and validated; a centreline, or a reference with its edges
    std::vector<Vec2> left_normals;  // one per sample, pointing left of the direction of travel
    FourWheelCar car;
    Config config;
    std::string profile;             // the physics profile the car was built from, or empty
};
// The problem posed by a car on a closed track. The normals are the track's own direction at each sample, from its
// neighbours. Rejects an invalid track or car.
LapProblem make_lap_problem(const Track& track, const FourWheelCar& car, const Config& config, std::string profile);
// problem.json's text, deterministic and at full precision, and its fingerprint: two problems are the same when these are.
std::string lap_problem_json(const LapProblem& problem);
std::string lap_problem_fingerprint(const LapProblem& problem);
// Writes problem.json and corridor.csv (s_m, x_m, y_m, heading_rad, curvature_1pm, left_normal_x, left_normal_y,
// left_m, right_m) into a directory, which must be empty or absent.
void write_lap_problem(const std::filesystem::path& directory, const LapProblem& problem);

// One point of the optimal lap, on the solver's mesh, in this project's frame: map x/y with z up, yaw counterclockwise.
// The solver places the centre of gravity; the rear axle, the pose this project publishes, is converted from it here.
// Offset is the centre of gravity's across the corridor, left positive. Wheels are front left, front right, rear left,
// rear right.
struct OptimalPoint {
    double s_m{}, time_s{}, x_m{}, y_m{}, cog_x_m{}, cog_y_m{}, yaw_rad{}, offset_m{};
    double forward_mps{}, lateral_mps{}, yaw_rate_radps{}, steering_rad{}, throttle{};
    std::array<double, 4> slip_ratio{}, slip_angle_rad{}, force_x_n{}, force_y_n{}, load_n{}, dissipation_w{};
    std::array<double, 4> energy_j{};  // each tire's dissipated energy since the start line, integrated here
    double speed_mps() const;
};
// d(lap time)/d(parameter) at the solution: the solver's own sensitivity analysis where it reaches the parameter,
// otherwise the central difference of two re-solves a half step either side.
struct LapSensitivity {
    std::string parameter;           // the four-wheel car's member: mass_kg, cg_height_m, brake_bias_front, ...
    std::string unit;
    double value{};                  // the parameter's value in the solved problem
    double seconds_per_unit{};       // first order; a step's worth is this times the step
    double step{};                   // the step the desktop quotes, such as -10 kg
    std::string method;              // how seconds_per_unit was found
    double half_step{};              // of the central difference, where one was taken
    std::optional<double> cross_check_seconds_per_unit;  // the central difference beside the solver's own, if checked
};
struct OptimalLap {
    int schema_version{};
    std::string label;               // what it is: an offline optimum for this model, never a live plant
    std::string tool, version, release, release_sha256, library_sha256, model, nlp_solver;
    std::string status;              // the solver's own exit message
    bool converged{};                // the status is a solution the solver accepts as optimal
    int iterations{}, max_iterations{};
    double tolerance{};
    int mesh_points{};
    double lap_time_s{};             // the lap time the solver found
    std::optional<double> objective;  // what it minimised: the lap time and its penalties on how fast controls change
    double speed_cap_mps{};          // the bound on forward speed the solve kept, the car's own cap
    std::string problem_fingerprint;
    LapProblem problem;              // as written beside the solution
    std::vector<std::string> mapping;  // how the car was put into the solver's model, one statement each
    std::vector<OptimalPoint> points;
    std::vector<LapSensitivity> sensitivities;
    std::array<double, 4> tire_energy_j{};  // the solver's own integral over the lap
    std::filesystem::path directory;

    // Where the optimum is at a time since its start line, wrapped to the lap: interpolated between mesh points.
    OptimalPoint at_time(double time_s) const;
    // The optimum's time at a station of its corridor, wrapped to the lap.
    double time_at_station(double s_m) const;
    // When the optimum's rear axle crosses each gate in its lap, by the judge's crossing rule. The lap begins with the
    // centre of gravity on the start line, so the start gate is crossed a moment after zero; a lap timed from that
    // crossing, as the judge times one, still lasts lap_time_s. Negative for a gate it never crosses.
    std::vector<double> gate_times(const std::vector<Gate>& gates) const;
};

inline constexpr int optimal_lap_schema_version = 1;
// Reads and cross-checks a reference-optimal artefact (optimal_lap.json, lap.csv, problem.json, corridor.csv): every
// member present, the problem it states identical to the one written beside it, the mesh strictly increasing in
// station and time from the start line, every point inside the corridor and consistent with its station and offset,
// time and distance consistent with speed, each tire's energy the integral of its dissipation, the lap time the last
// point's time plus the closing span, and every checked sensitivity within 5 percent (or a millisecond over its step) of
// its central difference. A non-converged solve reads back as such. Throws naming the violation.
OptimalLap load_optimal_lap(const std::filesystem::path& directory);

}  // namespace fd

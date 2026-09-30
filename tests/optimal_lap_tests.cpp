#include "fd/optimal_lap.hpp"
#include "fd/track_conditioning.hpp"
#include "fd/vehicle_profiles.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
template<class Function> std::string rejects(Function action, const std::string& message) {
    try { action(); } catch (const std::exception& error) { return error.what(); }
    throw std::runtime_error(message);
}

struct TemporaryDirectory {
    fs::path path;
    TemporaryDirectory() {
        std::random_device device;
        path = fs::temp_directory_path()/("fd-optimal-lap-tests-"+std::to_string(device()));
        fs::create_directories(path);
    }
    ~TemporaryDirectory() { std::error_code error; fs::remove_all(path, error); }
};

std::string read(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}
void write(const fs::path& path, const std::string& text) {
    std::ofstream file(path, std::ios::binary);
    file << text;
}
void replace_once(std::string& text, const std::string& from, const std::string& to) {
    const auto at = text.find(from);
    if (at == std::string::npos) throw std::runtime_error("test fixture lacks '"+from+"'");
    text.replace(at, from.size(), to);
}

// The formula-one-style car on the preset, conditioned as the racing line's reference is: the problem the desktop and
// the headless check pose for that profile.
fd::LapProblem f1_problem() {
    const auto& profile = fd::vehicle_profile("formula-one-style");
    return fd::make_lap_problem(fd::condition_track(fd::make_preset_track()).track, profile.car,
                                fd::configured_for(profile, fd::Config{}), profile.id);
}

// A lap a solver could have returned: the centre of gravity on the centreline at every eighth corridor sample, at a
// steady speed, each tire dissipating a steady power. Hooks change the files before they are written.
struct Lap {
    double speed_mps{20}, dissipation_w{1000};
    std::function<void(std::string&)> json, csv;
    std::function<void(std::size_t, std::vector<double>&)> row;
};
void write_artefact(const fs::path& directory, const fd::LapProblem& problem, const Lap& lap = {}) {
    fd::write_lap_problem(directory, problem);
    const auto& points = problem.corridor.points;
    const double length = problem.corridor.length_m, lap_time = length/lap.speed_mps;
    std::ostringstream csv;
    csv << std::setprecision(17) << "point,s_m,time_s,cog_x_m,cog_y_m,yaw_rad,offset_m,forward_mps,lateral_mps,yaw_rate_radps,steering_rad,throttle";
    for (const char* w : {"fl", "fr", "rl", "rr"})
        csv << ",slip_ratio_" << w << ",slip_angle_" << w << "_rad,force_x_" << w << "_n,force_y_" << w << "_n,load_" << w << "_n,dissipation_" << w << "_w";
    csv << '\n';
    std::size_t count = 0;
    for (std::size_t i = 0; i < points.size(); i += 8, ++count) {
        const auto& p = points[i];
        const auto& n = problem.left_normals[i];
        std::vector<double> row{static_cast<double>(count), p.s_m, p.s_m/lap.speed_mps, p.x_m, p.y_m, std::atan2(-n.x, n.y), 0,
                                lap.speed_mps, 0, lap.speed_mps*p.curvature, 0.01, 0.1};
        for (int w = 0; w < 4; ++w) row.insert(row.end(), {0.01, 0.02, 100, 200, 1600, lap.dissipation_w});
        if (lap.row) lap.row(count, row);
        csv << count;  // the point number, an integer
        for (std::size_t k = 1; k < row.size(); ++k) csv << ',' << row[k];
        csv << '\n';
    }
    std::ostringstream json;
    const double energy = lap.dissipation_w*lap_time;
    json << std::setprecision(17) << "{\n  \"schema_version\": 1,\n  \"kind\": \"reference-optimal\",\n"
         << "  \"label\": \"Offline optimum for a test model: not a run of this project's plant\",\n"
         << "  \"solver\": {\"tool\": \"fastest-lap\", \"version\": \"0.5\", \"release\": \"release.zip\", \"release_sha256\": \""
         << std::string(64, 'a') << "\", \"library_sha256\": \"" << std::string(64, 'b') << "\", \"model\": \"f1-3dof\", "
         << "\"nlp_solver\": \"Ipopt\", \"status\": \"Optimal Solution Found\", \"converged\": true, \"iterations\": 40, "
         << "\"max_iterations\": 3000, \"tolerance\": 1e-08},\n"
         << "  \"mesh_points\": " << count << ",\n  \"lap_time_s\": " << lap_time << ",\n  \"objective\": " << lap_time+0.01 << ",\n"
         << "  \"speed_cap_mps\": " << problem.config.max_speed_mps << ",\n  \"vehicle_half_width_m\": 0.9,\n"
         << "  \"problem_fingerprint\": \"" << fd::lap_problem_fingerprint(problem) << "\",\n"
         << "  \"mapping\": [\"model: a steady test lap\"],\n"
         << "  \"tire_energy_j\": [" << energy << ", " << energy << ", " << energy << ", " << energy << "],\n"
         << "  \"sensitivities\": [\n"
         << "    {\"parameter\": \"mass_kg\", \"unit\": \"kg\", \"value\": 660, \"seconds_per_unit\": 0.003, \"step\": -10, "
         << "\"method\": \"solver\", \"half_step\": 5, \"cross_check_seconds_per_unit\": 0.0031},\n"
         << "    {\"parameter\": \"brake_bias_front\", \"unit\": \"fraction\", \"value\": 0.6, \"seconds_per_unit\": 0, \"step\": 0.02, "
         << "\"method\": \"central difference\", \"half_step\": 0.02, \"cross_check_seconds_per_unit\": null}\n  ]\n}\n";
    std::string json_text = json.str(), csv_text = csv.str();
    if (lap.json) lap.json(json_text);
    if (lap.csv) lap.csv(csv_text);
    write(directory/"optimal_lap.json", json_text);
    write(directory/"lap.csv", csv_text);
}

// The problem: the corridor's normals point left of travel and are unit; the text and fingerprint are deterministic and
// change with the car and the corridor; a problem directory is written only where nothing is.
void problems_are_stated_exactly() {
    const auto problem = f1_problem();
    const auto& points = problem.corridor.points;
    require(problem.left_normals.size() == points.size(), "one normal per sample");
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto& a = points[i];
        const auto& b = points[(i+1)%points.size()];
        const auto& n = problem.left_normals[i];
        require(std::abs(std::hypot(n.x, n.y)-1) < 1e-12, "normals are unit");
        // Left of travel: the chord ahead turned a quarter counterclockwise lies along the normal.
        require((b.x_m-a.x_m)*n.y-(b.y_m-a.y_m)*n.x > 0, "normals point left of the direction of travel");
    }
    require(fd::lap_problem_json(problem) == fd::lap_problem_json(f1_problem()), "the problem's text is deterministic");
    auto heavier = problem;
    heavier.car.mass_kg += 1;
    require(fd::lap_problem_fingerprint(heavier) != fd::lap_problem_fingerprint(problem), "the fingerprint changes with the car");
    auto narrower = problem;
    narrower.corridor.width_m = 8;
    require(fd::lap_problem_fingerprint(narrower) != fd::lap_problem_fingerprint(problem), "the fingerprint changes with the corridor");
    TemporaryDirectory temporary;
    fd::write_lap_problem(temporary.path/"problem", problem);
    require(fs::exists(temporary.path/"problem"/"problem.json") && fs::exists(temporary.path/"problem"/"corridor.csv"), "both files written");
    const auto refused = rejects([&] { fd::write_lap_problem(temporary.path/"problem", problem); }, "a used directory is refused");
    require(refused.find("not empty") != std::string::npos, "the refusal says why: "+refused);
    std::cout << "  problem: " << points.size() << " samples over " << problem.corridor.length_m << " m, fingerprint "
              << fd::lap_problem_fingerprint(problem) << "\n";
}

// A valid artefact reads back as written, the rear axle converted from the centre of gravity, energies integrated, and
// the lap's helpers interpolate and wrap; the gates are crossed when the rear axle reaches them.
void artefacts_read_back() {
    TemporaryDirectory temporary;
    const auto problem = f1_problem();
    write_artefact(temporary.path/"lap", problem);
    const auto lap = fd::load_optimal_lap(temporary.path/"lap");
    const double length = problem.corridor.length_m;
    require(lap.converged && lap.status == "Optimal Solution Found" && lap.tool == "fastest-lap", "the solver reads back");
    require(std::abs(lap.lap_time_s-length/20) < 1e-12 && lap.objective && std::abs(*lap.objective-lap.lap_time_s-0.01) < 1e-9,
            "lap time and objective read back");
    require(lap.points.size() == 106 && lap.mesh_points == 106, "every mesh point reads back");
    require(fd::lap_problem_fingerprint(lap.problem) == fd::lap_problem_fingerprint(problem), "the problem reads back as it was");
    const auto& first = lap.points.front();
    const double rear = problem.car.cg_to_rear_m;
    require(std::abs(first.x_m-(first.cog_x_m-rear*std::cos(first.yaw_rad))) < 1e-12 &&
            std::abs(first.y_m-(first.cog_y_m-rear*std::sin(first.yaw_rad))) < 1e-12, "the rear axle is the centre of gravity less cg_to_rear");
    require(std::abs(lap.points[50].energy_j[2]-1000*lap.points[50].time_s) < 1e-6, "energy is dissipation integrated from the line");
    require(lap.sensitivities.size() == 2 && lap.sensitivities[0].cross_check_seconds_per_unit &&
            !lap.sensitivities[1].cross_check_seconds_per_unit, "sensitivities read back, a missing cross-check as none");
    const auto start = lap.at_time(0), wrapped = lap.at_time(lap.lap_time_s);
    require(std::hypot(start.x_m-wrapped.x_m, start.y_m-wrapped.y_m) < 1e-9, "at_time wraps to the lap");
    const auto half = lap.at_time(lap.lap_time_s/2);
    require(std::abs(half.s_m-length/2) < 1e-6 && std::abs(half.energy_j[0]-1000*lap.lap_time_s/2) < 1e-6, "at_time interpolates");
    require(std::abs(lap.time_at_station(length/4)-length/80) < 1e-9 && std::abs(lap.time_at_station(length+1)-1.0/20) < 1e-9,
            "time_at_station interpolates and wraps");
    const auto gates = fd::make_gates(problem.corridor);
    const auto times = lap.gate_times(gates);
    require(times.size() == 3, "a time for every gate");
    for (std::size_t g = 0; g < 3; ++g) {
        const double expected = (g*length/3+rear)/20;
        require(std::abs(times[g]-expected) < 0.01, "sector gate "+std::to_string(g)+" at "+std::to_string(times[g])+" s, not "+
                std::to_string(expected)+" s when the rear axle reaches it");
    }
    std::cout << "  read back: " << lap.points.size() << " points, " << lap.lap_time_s << " s; the rear axle crosses the start line at "
              << times[0] << " s and the sector gates at " << times[1] << " and " << times[2] << " s\n";
}

// Every violation is refused naming it, and an unconverged solve reads back as one rather than being refused.
void violations_are_refused() {
    const auto problem = f1_problem();
    TemporaryDirectory temporary;
    int index = 0;
    const auto expect = [&](const std::string& name, const Lap& lap, const std::string& fragment,
                            const std::function<void(const fs::path&)>& after = {}) {
        const auto directory = temporary.path/(std::to_string(++index)+"-"+name);
        write_artefact(directory, problem, lap);
        if (after) after(directory);
        const auto message = rejects([&] { fd::load_optimal_lap(directory); }, name+" must be refused");
        require(message.find(fragment) != std::string::npos, name+": expected '"+fragment+"' in: "+message);
    };
    const auto json = [](std::string from, std::string to) { return [=](std::string& text) { replace_once(text, from, to); }; };
    Lap lap;
    lap.json = json("\"converged\": true", "\"converged\": false");
    expect("converged-contradicts-status", lap, "contradicts");
    lap.json = json("\"Optimal Solution Found\", \"converged\": true", "\"Maximum Number of Iterations Exceeded\", \"converged\": true");
    expect("status-contradicts-converged", lap, "contradicts");
    lap.json = json("\"objective\": ", "\"objective\": null, \"was\": ");
    expect("objective-missing-when-converged", lap, "objective must be a number");
    lap.json = [](std::string& text) {
        const auto at = text.find("\"objective\": ");
        text.replace(at, text.find(',', at)-at, "\"objective\": 1");
    };
    expect("objective-below-lap-time", lap, "cannot be less");
    lap.json = json("\"release_sha256\": \"a", "\"release_sha256\": \"A");
    expect("digest-malformed", lap, "64 lowercase");
    lap.json = json("\"mesh_points\": 106", "\"mesh_points\": 105");
    expect("mesh-count", lap, "mesh_points");
    lap.json = json("\"cross_check_seconds_per_unit\": 0.0031", "\"cross_check_seconds_per_unit\": 0.006");
    expect("sensitivity-disagrees", lap, "by re-solving");
    lap.json = [](std::string& text) {
        const auto at = text.find("\"tire_energy_j\": [");
        text.replace(at, text.find(',', at)-at, "\"tire_energy_j\": [1000");
    };
    expect("energy-not-the-integral", lap, "dissipated");
    lap = {};
    lap.row = [](std::size_t i, std::vector<double>& row) { if (i == 10) row[3] += 0.2; };
    expect("point-off-its-station", lap, "from its station and offset");
    lap.row = [&](std::size_t i, std::vector<double>& row) {
        if (i != 10) return;
        // Consistently 4.5 m left, which leaves less than the half width to the 5 m edge.
        const auto& n = problem.left_normals[80];
        row[3] += 4.5*n.x;
        row[4] += 4.5*n.y;
        row[6] = 4.5;
    };
    expect("car-leaves-the-corridor", lap, "leaves the corridor");
    lap.row = [](std::size_t i, std::vector<double>& row) { if (i == 10) row[7] = 30; };
    expect("speed-and-time-disagree", lap, "travelled in");
    lap.row = [](std::size_t i, std::vector<double>& row) { if (i == 10) row[7] = 45; };
    expect("speed-above-cap", lap, "speed cap");
    lap.row = [](std::size_t i, std::vector<double>& row) { if (i == 10) row[10] = 0.6; };
    expect("steering-beyond-lock", lap, "lock");
    lap.row = [](std::size_t i, std::vector<double>& row) { if (i == 0) row[1] = 0.1; };
    expect("mesh-off-the-line", lap, "start on the line");
    lap.row = [](std::size_t i, std::vector<double>& row) { if (i == 10) row[2] = 0; };
    expect("time-not-increasing", lap, "increase in station and time");
    lap = {};
    expect("problem-changed", lap, "another problem", [](const fs::path& d) {
        auto text = read(d/"problem.json");
        replace_once(text, "\"mass_kg\": 660", "\"mass_kg\": 650");
        write(d/"problem.json", text);
    });
    expect("corridor-changed", lap, "corridor.csv is not the corridor", [](const fs::path& d) {
        auto text = read(d/"corridor.csv");
        replace_once(text, "\n0,13.", "\n0,14.");
        write(d/"corridor.csv", text);
    });
    expect("problem-missing", lap, "missing file problem.json", [](const fs::path& d) { fs::remove(d/"problem.json"); });

    // An honest failure is not a violation: it reads back, marked.
    lap.json = [](std::string& text) {
        replace_once(text, "\"Optimal Solution Found\", \"converged\": true", "\"Maximum Number of Iterations Exceeded\", \"converged\": false");
        const auto at = text.find("\"objective\": ");
        text.replace(at, text.find(',', at)-at, "\"objective\": null");
    };
    write_artefact(temporary.path/"unconverged", problem, lap);
    const auto failed = fd::load_optimal_lap(temporary.path/"unconverged");
    require(!failed.converged && !failed.objective && failed.status == "Maximum Number of Iterations Exceeded",
            "an unconverged solve reads back as such");
    std::cout << "  " << index << " violations refused; an unconverged solve reads back marked\n";
}
}  // namespace

int main() {
    const std::vector<std::pair<const char*, void (*)()>> tests{
        {"problems_are_stated_exactly", problems_are_stated_exactly},
        {"artefacts_read_back", artefacts_read_back},
        {"violations_are_refused", violations_are_refused},
    };
    int failed = 0;
    for (const auto& [name, test] : tests) {
        try {
            std::cout << name << '\n';
            test();
        } catch (const std::exception& error) {
            ++failed;
            std::cout << "FAILED " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << (failed ? "optimal lap tests failed\n" : "optimal lap tests passed\n");
    return failed ? 1 : 0;
}

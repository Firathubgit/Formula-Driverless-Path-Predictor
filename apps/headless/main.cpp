#include "fd/recording.hpp"
#include "fd/optimal_lap.hpp"
#include "fd/lattice.hpp"
#include "fd/performance_envelope.hpp"
#include "fd/setup.hpp"
#include "fd/track_conditioning.hpp"
#ifdef FD_HAVE_RACELINE
#include "fd/raceline.hpp"
#endif
#ifdef FD_HAVE_MPCC
#include "fd/mpcc.hpp"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef FD_SOURCE_FINGERPRINT
#define FD_SOURCE_FINGERPRINT "unavailable"
#endif
#ifndef FD_COMPILER_ID
#define FD_COMPILER_ID "unavailable"
#endif
#ifndef FD_COMPILER_VERSION
#define FD_COMPILER_VERSION "unavailable"
#endif

namespace {
struct Options {
    fd::Config config;
    std::filesystem::path out{"out/runs/latest"};
    fd::RunRequest request;
    std::vector<fd::Obstruction> obstructions;
    std::optional<std::filesystem::path> check, prediction_error, envelope_out, lattice_out;
    std::optional<std::filesystem::path> lap_problem_out, optimal_check;  // the theoretical best lap (decision 0034)
    fd::VehicleModel model{fd::KinematicBicycle{}};
    std::optional<double> drive_front_fraction;
    bool soft_front{};
    std::optional<double> brake_bias_front, cg_height, roll_balance_front, drag_area, downforce_area, aero_balance_front, viscous_coupling;
    std::optional<double> mass, max_power_kw;
    fd::SteeringMode steering{fd::SteeringMode::pure_pursuit};
    fd::SpeedPlanMode speed_plan{fd::SpeedPlanMode::grip_fractions};
    std::optional<double> envelope_fraction;
    std::optional<fd::LocalPlannerMode> local_planner;  // the lattice planner unless stated
    std::optional<fd::ControllerMode> controller;       // the policy unless stated
    std::optional<std::uint64_t> sensor_seed;           // the car reads only its true state unless stated
    std::optional<std::uint64_t> perception_seed;       // no cone is perceived unless stated
    bool drive_cones{};                                 // on the known track unless stated
    std::optional<std::filesystem::path> course_file;   // a PacSim layout instead of the preset
    std::optional<fd::Discipline> discipline;           // a trackdrive unless stated
    std::optional<std::string> profile;                 // a physics profile instead of a plant's defaults
    bool plant_given{};
    std::optional<std::filesystem::path> track_file;
    std::optional<double> track_width, track_smoothing;
    bool racing_line{};
    std::optional<double> line_margin;
};

double number(const std::string& value) {
    std::size_t end{};
    const double result=std::stod(value,&end);
    if (end!=value.size() || !std::isfinite(result)) throw std::invalid_argument("Expected finite number: "+value);
    return result;
}

std::vector<std::string> split(const std::string& value,char separator) {
    std::vector<std::string> parts;
    std::string current;
    for (const char c:value) {
        if (c==separator) { parts.push_back(current); current.clear(); }
        else current+=c;
    }
    parts.push_back(current);
    return parts;
}

// FROM_S:TO_S:FROM_OFFSET:TO_OFFSET[:IDENTIFIER], all in metres along and across the track.
fd::Obstruction parse_obstruction(const std::string& value) {
    const auto parts=split(value,':');
    if (parts.size()<4 || parts.size()>5)
        throw std::invalid_argument("Obstruction must be FROM_S:TO_S:FROM_OFFSET:TO_OFFSET[:NAME]");
    fd::Obstruction obstruction{number(parts[0]),number(parts[1]),number(parts[2]),number(parts[3]),
                                parts.size()==5?parts[4]:"obstruction"};
    return obstruction;
}

Options parse(int argc,char** argv) {
    Options options;
    for (int i=1;i<argc;++i) {
        const std::string flag=argv[i];
        if (flag=="--help") {
            std::cout << "fd_headless [--config FILE] [--grip 0.2..1.5] [--laps N]\n"
                         "            [--duration SECONDS] [--out DIRECTORY] [--grip-event SECONDS:MU]\n"
                         "            [--obstruct FROM_S:TO_S:FROM_OFFSET:TO_OFFSET[:NAME]]\n"
                         "            [--plant kinematic|dynamic] [--drive-front-fraction 0..1]\n"
                         "            [--car default|soft-front] [--steering pure-pursuit|map]\n"
                         "            [--plant four-wheel] [--brake-bias-front 0..1] [--cg-height METRES]\n"
                         "            [--roll-balance-front 0..1] [--drag-area M2] [--downforce-area M2]\n"
                         "            [--aero-balance-front 0..1] [--viscous-coupling NMS] [--mass KG]\n"
                         "            [--max-power-kw KW] [--speed-plan grip-fractions|envelope]\n"
                         "            [--envelope-fraction 0.5..1]\n"
                         "            [--track CENTRELINE.csv] [--track-width METRES] [--track-smoothing METRES]\n"
                         "            [--line centreline|racing] [--line-margin METRES]\n"
                         "            [--local-planner lattice|five-offsets] [--controller policy|mpcc]\n"
                         "            [--sensors SEED] [--perception SEED] [--drive known|cones]\n"
                         "            [--course PACSIM_TRACK.yaml] [--discipline trackdrive|autocross]\n"
                         "            [--profile project-roadster|formula-one-style|formula-student-electric|\n"
                         "                       electric-race-car|racing-kart|gokartcentralen-rsx2]\n"
                         "fd_headless --check-recording DIRECTORY\n"
                         "fd_headless --prediction-error DIRECTORY\n"
                         "fd_headless --plant four-wheel [setup flags] [--config FILE] [--grip MU] --write-envelope DIRECTORY\n"
                         "fd_headless [--config FILE] [--track CENTRELINE.csv [--track-width M]] [--line racing] --write-lattice DIRECTORY\n"
                         "fd_headless --profile ID|--plant four-wheel [setup flags] [--track FILE|--course FILE] --write-lap-problem DIRECTORY\n"
                         "fd_headless --profile ID|--plant four-wheel [setup flags] [--track FILE|--course FILE] --check-optimal-lap DIRECTORY\n"
                         "Default: one lap, 180 s safety cap. --laps 0 runs until duration.\n"
                         "Events are applied at the first fixed tick at or after their requested time.\n"
                         "--obstruct states a blocked region of the corridor in metres along and across\n"
                         "the track. It is scenario ground truth, not a detected object. Repeatable.\n"
                         "--plant selects the vehicle model: the kinematic bicycle (default) or the\n"
                         "dynamic single-track model with tire forces. --drive-front-fraction sets the\n"
                         "dynamic model's share of driving force on the front axle (default 0.5; 0 is\n"
                         "rear-wheel drive). --car soft-front puts the dynamic car on softer front tires,\n"
                         "so it understeers. --plant four-wheel is the car with four rotating wheels, brakes\n"
                         "and a power limit; --brake-bias-front sets its share of braking torque on the front\n"
                         "axle (default 0.5), and --drive-front-fraction applies to it too. Its load moves\n"
                         "between the wheels with acceleration: --cg-height sets the centre of gravity's height\n"
                         "(default 0.35 m; 0 keeps static loads, at most half the track) and --roll-balance-front\n"
                         "the front axle's share of lateral load transfer (default 0.5). --drag-area is its drag\n"
                         "coefficient times frontal area (default 0.6 m^2), --downforce-area its lift coefficient\n"
                         "times area pressing down (default 0) and --aero-balance-front the front axle's share of\n"
                         "that downforce (default 0.5, refused without downforce). --viscous-coupling couples\n"
                         "the wheels of each driven axle (N m s/rad, default 0, an open differential).\n"
                         "--mass (600..1400 kg, default 800) keeps the mass distribution, so yaw inertia scales\n"
                         "with it, and --max-power-kw (30..120, default 100) limits the power at the wheels; both\n"
                         "are the desktop's setup controls, with the same ranges.\n"
                         "--steering selects how the pursuit target becomes steering: Pure Pursuit geometry\n"
                         "(default) or MAP, a steady-state steering table generated from the plant that drives,\n"
                         "the dynamic car's or the four-wheel car's. For the kinematic bicycle it would be\n"
                         "geometry, so MAP is refused for it.\n"
                         "--speed-plan selects what the reference speed plan is made from: the configured grip\n"
                         "fractions (default) or, for --plant four-wheel, the car's own performance envelope,\n"
                         "derived before the run and again at each grip change. --envelope-fraction sets the share\n"
                         "of the envelope planned with (default 0.8, a reserve for tracking) and needs\n"
                         "--speed-plan envelope.\n"
                         "--track conditions a closed centreline file (x_m,y_m per line, '#' comments, one header\n"
                         "line allowed) into the track the run drives instead of the preset: a smooth closed curve\n"
                         "fitted within --track-smoothing (root-mean-square distance, default 0.1 m), resampled every\n"
                         "0.6 m, refused if it crosses itself, bends tighter than half --track-width (default 10 m)\n"
                         "or overlaps itself. Without --track, --track-width narrows or widens the corridor of the\n"
                         "preset about its centreline, as a Formula Student course is 3 to 5 m wide (decision 0031).\n"
                         "--line racing drives the minimum-curvature racing line of the preset, or of --track, instead\n"
                         "of its centreline: solved with OSQP inside the corridor less the car's 0.9 m half width and\n"
                         "--line-margin (default 0.6 m), within the steering limit, and recorded with the corridor's\n"
                         "edges. It prints the estimated lap on the line and on the centreline under the chosen plan.\n"
                         "Stated blockages are across the centreline, so --obstruct is refused with it.\n"
                         "--write-envelope derives the four-wheel car's G-G-V performance envelope from the plant\n"
                         "and writes envelope.csv with TUM's ggv.csv and ax_max_machines.csv into a fresh directory,\n"
                         "without running a lap.\n"
                         "--write-lattice lays the offline lattice along the line a run would drive, the preset, --track\n"
                         "or --line racing, and writes lattice.json, layers.csv, nodes.csv, edges.csv and edge_samples.csv\n"
                         "into a fresh directory, without running a lap: layers every 6 m, 4 m where the line curves,\n"
                         "nodes every 0.5 m across the corridor less the 0.9 m half width, edges to the next layer's nodes\n"
                         "within 0.3 m of offset per metre, removed when the car cannot steer them or they leave the\n"
                         "corridor, then dead ends, each with its offline cost. Only the line and the configuration's\n"
                         "wheelbase and steering limit shape it; --obstruct is refused with it.\n"
                         "--local-planner selects what chooses the car's action around stated blockages: the lattice\n"
                         "planner (default), which follows the reference line while it stays clear, passes a blockage\n"
                         "on the cheaper clear side along a lattice path within the car's lateral grip, returns to the\n"
                         "reference the same way, and otherwise brakes toward the widest gap; or the five-offset planner\n"
                         "it replaces, kept for comparison. Without a blockage both follow the reference line alike.\n"
                         "--controller selects what drives the chosen action: the policy every prediction rolls forward\n"
                         "(default), or MPCC, model predictive contouring control, which optimises three seconds of the\n"
                         "car's motion with a single-track model at every decision to make the most progress along the\n"
                         "chosen path within the corridor it leaves open and each axle's slip angle and friction. Its plan\n"
                         "drives only while solved, within its own envelope and the corridor, and clear of every blockage;\n"
                         "otherwise, and when holding for a blockage, the policy drives. It needs OSQP's sources.\n"
                         "--sensors fits the car with its own instruments (decision 0029): pose and speed at 20 Hz behind\n"
                         "50 ms, wheel speeds, inertial unit and steering at 200 Hz behind 5 ms, each with its own Gaussian\n"
                         "error, every one of them drawn from SEED, a whole number the recording keeps so the run measures\n"
                         "the same again. On the known track what they measure drives nothing, and the run records it beside\n"
                         "what the car was; with --drive cones the driver decides from it. The plant always advances on its\n"
                         "true state.\n"
                         "--perception lays the course's cones along the corridor, blue left and yellow right, and gives the car\n"
                         "simulated cone perception of them (decision 0030): a front sensor at 10 Hz behind 200 ms seeing 1 to 45 m\n"
                         "and 60 degrees either side, missing and mis-colouring cones more often farther away, with noise whose\n"
                         "covariance it reports, every draw from SEED. The run records it; with --drive cones the car drives on it.\n"
                         "--drive cones drives on the cones the car perceives instead of the known track (decision 0031): at each\n"
                         "frame the port of FaSTTUBe's planner makes a path from the detections, placed on the map by the pose\n"
                         "the car believes, measured with --sensors and otherwise read true (the ideal-state assumption), and\n"
                         "the policy follows it, planning to be able to stop by its end. It needs --perception; blockages and\n"
                         "MPCC, which need the known track, are refused with it. The known track only judges the run.\n"
                         "FaSTTUBe's planner is tuned for Formula Student courses 3 to 5 m wide; on the 10 m preset give\n"
                         "--track-width 5.\n"
                         "Every run is judged as a Formula Student official would (decision 0032): the start line starts the\n"
                         "clock, laps and sectors are timed, a cone down or out costs 2 s, all four wheels off course 10 s\n"
                         "and eight seconds of it a DNF; after the finish the car must stop within 30 m. --discipline chooses\n"
                         "a trackdrive of ten laps (default) or an autocross of one; the run still ends at --laps.\n"
                         "--course drives a Formula Student layout in PacSim's track format instead of the preset: its cones\n"
                         "as laid, its centreline traced between them and conditioned, its corridor the cones' own, and its\n"
                         "timekeeping gates. PacSim's layouts are read from where they are and never copied here.\n"
                         "--profile builds the four-wheel car from a documented physics profile (decision 0033): Limebeer's\n"
                         "Formula One car, PacSim's and MPCC's Formula Student car, TUM's example electric race car, Lot's\n"
                         "racing kart, the Sodi RSX2 rental kart Gokartcentralen Göteborg runs (decision 0038; drive it on\n"
                         "tracks/gokartcentralen-goteborg.csv --track-width 6), or this project's roadster, each with its\n"
                         "wheelbase, steering lock and speed cap, and\n"
                         "every value marked published, derived, fitted or chosen in core/vehicle_profiles.cpp. It replaces\n"
                         "--plant and the four-wheel car's setup flags, which it refuses beside it. Appearance is not part of it.\n"
                         "--write-lap-problem writes the minimum-time problem an offline solver is given (decision 0034): the\n"
                         "corridor, the preset or --track conditioned as a racing line's reference, or --course as traced, with its\n"
                         "normals and edges in corridor.csv, and the four-wheel car of --profile or --plant four-wheel and its setup\n"
                         "flags, with its steering lock and speed cap, in problem.json, into a fresh directory, without running a lap.\n"
                         "tools/optimal_lap/solve_fastest_lap.py solves it with fastest-lap into a reference-optimal artefact.\n"
                         "--check-optimal-lap validates such an artefact as one contract and requires it to be the solution of the\n"
                         "problem the same flags pose in this build; it prints the solver, its status, mesh and tolerance, the lap\n"
                         "time, each tire's dissipated energy and the setup sensitivities. The optimum is the solver's, for its own\n"
                         "model; it is never a run of this project's plant.\n"
                         "--check-recording validates an existing run directory as one contract and\n"
                         "re-runs this build's planner against every recorded plan revision.\n"
                         "--prediction-error reports how far a predictive controller's recorded plans were from what\n"
                         "followed, every 0.25 s over the horizon: the plan error, from where the car went, and the model\n"
                         "error, from where this build's plant goes under each plan's own commands from the state it was\n"
                         "made in, without replanning. It needs a schema 13 recording driven by MPCC.\n";
            std::exit(0);
        }
        if (i+1>=argc) throw std::invalid_argument("Missing value for "+flag);
        const std::string value=argv[++i];
        if (flag=="--config") options.config=fd::load_config(value);
        else if (flag=="--grip") options.config.grip_mu=number(value);
        else if (flag=="--out") options.out=value;
        else if (flag=="--check-recording") options.check=value;
        else if (flag=="--prediction-error") options.prediction_error=value;
        else if (flag=="--write-envelope") options.envelope_out=value;
        else if (flag=="--write-lattice") options.lattice_out=value;
        else if (flag=="--write-lap-problem") options.lap_problem_out=value;
        else if (flag=="--check-optimal-lap") options.optimal_check=value;
        else if (flag=="--obstruct") options.obstructions.push_back(parse_obstruction(value));
        else if (flag=="--profile") options.profile=value;
        else if (flag=="--plant") {
            options.plant_given=true;
            if (value=="kinematic") options.model=fd::KinematicBicycle{};
            else if (value=="dynamic") options.model=fd::DynamicSingleTrack{};
            else if (value=="four-wheel") options.model=fd::FourWheelCar{};
            else throw std::invalid_argument("plant must be kinematic, dynamic or four-wheel");
        }
        else if (flag=="--perception") {
            const double seed=number(value);
            if (seed<0 || seed!=std::floor(seed) || seed>18446744073709549568.0)
                throw std::invalid_argument("--perception takes a whole seed a run can be perceived again from");
            options.perception_seed=static_cast<std::uint64_t>(seed);
        }
        else if (flag=="--course") options.course_file=value;
        else if (flag=="--discipline") {
            if (value=="trackdrive") options.discipline=fd::Discipline::trackdrive;
            else if (value=="autocross") options.discipline=fd::Discipline::autocross;
            else throw std::invalid_argument("--discipline must be trackdrive or autocross");
        }
        else if (flag=="--drive") {
            if (value=="known") options.drive_cones=false;
            else if (value=="cones") options.drive_cones=true;
            else throw std::invalid_argument("--drive must be known or cones");
        }
        else if (flag=="--sensors") {
            const double seed=number(value);
            if (seed<0 || seed!=std::floor(seed) || seed>18446744073709549568.0)
                throw std::invalid_argument("--sensors takes a whole seed a run can be measured again from");
            options.sensor_seed=static_cast<std::uint64_t>(seed);
        }
        else if (flag=="--drive-front-fraction") options.drive_front_fraction=number(value);
        else if (flag=="--brake-bias-front") options.brake_bias_front=number(value);
        else if (flag=="--cg-height") options.cg_height=number(value);
        else if (flag=="--roll-balance-front") options.roll_balance_front=number(value);
        else if (flag=="--drag-area") options.drag_area=number(value);
        else if (flag=="--downforce-area") options.downforce_area=number(value);
        else if (flag=="--aero-balance-front") options.aero_balance_front=number(value);
        else if (flag=="--viscous-coupling") options.viscous_coupling=number(value);
        else if (flag=="--mass") options.mass=number(value);
        else if (flag=="--max-power-kw") options.max_power_kw=number(value);
        else if (flag=="--car") {
            if (value=="default") options.soft_front=false;
            else if (value=="soft-front") options.soft_front=true;
            else throw std::invalid_argument("car must be default or soft-front");
        }
        else if (flag=="--steering") {
            if (value=="pure-pursuit") options.steering=fd::SteeringMode::pure_pursuit;
            else if (value=="map") options.steering=fd::SteeringMode::model_acceleration_pursuit;
            else throw std::invalid_argument("steering must be pure-pursuit or map");
        }
        else if (flag=="--speed-plan") {
            if (value=="grip-fractions") options.speed_plan=fd::SpeedPlanMode::grip_fractions;
            else if (value=="envelope") options.speed_plan=fd::SpeedPlanMode::performance_envelope;
            else throw std::invalid_argument("speed plan must be grip-fractions or envelope");
        }
        else if (flag=="--envelope-fraction") options.envelope_fraction=number(value);
        else if (flag=="--local-planner") {
            if (value=="lattice") options.local_planner=fd::LocalPlannerMode::lattice;
            else if (value=="five-offsets") options.local_planner=fd::LocalPlannerMode::five_offsets;
            else throw std::invalid_argument("local planner must be lattice or five-offsets");
        }
        else if (flag=="--controller") {
            if (value=="policy") options.controller=fd::ControllerMode::policy;
            else if (value=="mpcc") options.controller=fd::ControllerMode::mpcc;
            else throw std::invalid_argument("controller must be policy or mpcc");
        }
        else if (flag=="--track") options.track_file=value;
        else if (flag=="--track-width") options.track_width=number(value);
        else if (flag=="--track-smoothing") options.track_smoothing=number(value);
        else if (flag=="--line") {
            if (value=="centreline") options.racing_line=false;
            else if (value=="racing") options.racing_line=true;
            else throw std::invalid_argument("line must be centreline or racing");
        }
        else if (flag=="--line-margin") options.line_margin=number(value);
        else if (flag=="--duration") options.request.duration_cap_s=number(value);
        else if (flag=="--laps") {
            const double count=number(value);
            if (count<0 || count>100 || std::floor(count)!=count) throw std::invalid_argument("laps must be an integer in 0..100");
            options.request.requested_laps=static_cast<int>(count);
        } else if (flag=="--grip-event") {
            const auto colon=value.find(':');
            if (colon==std::string::npos) throw std::invalid_argument("grip event must be SECONDS:MU");
            const double time=number(value.substr(0,colon)),mu=number(value.substr(colon+1));
            auto candidate=options.config;
            candidate.grip_mu=mu;
            fd::validate_config(candidate);
            if (time<0) throw std::invalid_argument("event time must be nonnegative");
            options.request.grip_events.push_back({time,mu});
        } else throw std::invalid_argument("Unknown argument: "+flag);
    }
    if (options.check || options.prediction_error) return options;
    if (options.track_smoothing && !options.track_file)
        throw std::invalid_argument("--track-smoothing has no effect without --track");
    if (options.line_margin && !options.racing_line)
        throw std::invalid_argument("--line-margin has no effect without --line racing");
    if (options.lattice_out) {
        if (options.envelope_out) throw std::invalid_argument("--write-lattice and --write-envelope each write their own directory; choose one");
        if (!options.obstructions.empty()) throw std::invalid_argument("--obstruct has no effect on the offline lattice, which is laid before any scenario");
        if (options.local_planner) throw std::invalid_argument("--local-planner chooses actions during a run; it has no effect on the offline lattice");
        if (options.controller) throw std::invalid_argument("--controller drives a run; it has no effect on the offline lattice");
        if (std::filesystem::exists(*options.lattice_out) && !std::filesystem::is_empty(*options.lattice_out))
            throw std::invalid_argument("Lattice directory is not empty; choose a fresh --write-lattice directory");
    }
    if (options.controller==fd::ControllerMode::mpcc) {
#ifndef FD_HAVE_MPCC
        throw std::invalid_argument("This build has no MPCC: run scripts/bootstrap-osqp.ps1, then build again");
#endif
        if (options.envelope_out) throw std::invalid_argument("--controller drives a run; it has no effect on the envelope");
    }
    if (options.racing_line) {
#ifndef FD_HAVE_RACELINE
        throw std::invalid_argument("This build has no racing line: run scripts/bootstrap-osqp.ps1, then build again");
#endif
        if (!options.obstructions.empty())
            throw std::invalid_argument("--obstruct states blockages across the centreline; it cannot be combined with --line racing yet");
    }
    if (options.envelope_fraction) {
        if (options.speed_plan!=fd::SpeedPlanMode::performance_envelope)
            throw std::invalid_argument("--envelope-fraction has no effect without --speed-plan envelope");
        options.config.envelope_fraction=*options.envelope_fraction;
    }
    fd::validate_config(options.config);
    if (options.profile) {
        if (options.plant_given || options.soft_front || options.drive_front_fraction || options.brake_bias_front || options.cg_height ||
            options.roll_balance_front || options.drag_area || options.downforce_area || options.aero_balance_front ||
            options.viscous_coupling || options.mass || options.max_power_kw)
            throw std::invalid_argument("--profile gives the whole car; drop --plant, --car and the four-wheel car's setup flags beside it");
        const auto& profile=fd::vehicle_profile(*options.profile);
        options.model=profile.car;
        options.config=fd::configured_for(profile,options.config);
        options.request.vehicle_profile=profile.id;
    }
    if (options.soft_front) {
        if (!std::holds_alternative<fd::DynamicSingleTrack>(options.model))
            throw std::invalid_argument("--car soft-front applies only to --plant dynamic, the single-track car it modifies");
        options.model=fd::DynamicSingleTrack::soft_front();
    }
    if (options.drive_front_fraction) {
        if (auto* car=std::get_if<fd::DynamicSingleTrack>(&options.model)) car->drive_front_fraction=*options.drive_front_fraction;
        else if (auto* four=std::get_if<fd::FourWheelCar>(&options.model)) four->drive_front_fraction=*options.drive_front_fraction;
        else throw std::invalid_argument("--drive-front-fraction applies only to --plant dynamic or four-wheel");
    }
    if (options.brake_bias_front) {
        auto* four=std::get_if<fd::FourWheelCar>(&options.model);
        if (!four) throw std::invalid_argument("--brake-bias-front applies only to --plant four-wheel; the other plants have no brakes");
        four->brake_bias_front=*options.brake_bias_front;
    }
    if (options.cg_height || options.roll_balance_front) {
        auto* four=std::get_if<fd::FourWheelCar>(&options.model);
        if (!four) throw std::invalid_argument("--cg-height and --roll-balance-front apply only to --plant four-wheel; the other plants have no load transfer");
        if (options.cg_height) four->cg_height_m=*options.cg_height;
        if (options.roll_balance_front) four->roll_balance_front=*options.roll_balance_front;
    }
    if (options.drag_area || options.downforce_area || options.aero_balance_front || options.viscous_coupling) {
        auto* four=std::get_if<fd::FourWheelCar>(&options.model);
        if (!four) throw std::invalid_argument("--drag-area, --downforce-area, --aero-balance-front and --viscous-coupling apply only to --plant four-wheel");
        if (options.drag_area) four->drag_area_m2=*options.drag_area;
        if (options.downforce_area) four->downforce_area_m2=*options.downforce_area;
        if (options.viscous_coupling) four->viscous_coupling_nms=*options.viscous_coupling;
        if (options.aero_balance_front) {
            if (four->downforce_area_m2==0) throw std::invalid_argument("--aero-balance-front has no effect without downforce; set --downforce-area too");
            four->aero_balance_front=*options.aero_balance_front;
        }
    }
    if (options.mass || options.max_power_kw) {
        auto* four=std::get_if<fd::FourWheelCar>(&options.model);
        if (!four) throw std::invalid_argument("--mass and --max-power-kw apply only to --plant four-wheel, whose setup they change");
        // Through the setup controls, so ranges and the mass distribution match the desktop (decision 0016).
        if (options.mass) *four=fd::with_setup_value(*four,options.config,"mass_kg",*options.mass);
        if (options.max_power_kw) *four=fd::with_setup_value(*four,options.config,"max_drive_power_w",*options.max_power_kw*1000);
    }
    fd::validate_vehicle(options.model,options.config);
    if (options.lap_problem_out || options.optimal_check) {
        if (options.lap_problem_out && options.optimal_check)
            throw std::invalid_argument("--write-lap-problem and --check-optimal-lap are separate steps; choose one");
        if (!std::holds_alternative<fd::FourWheelCar>(options.model))
            throw std::invalid_argument("The minimum-time problem is the four-wheel car's; give --profile or --plant four-wheel");
        if (options.racing_line)
            throw std::invalid_argument("The optimum chooses its own line within the corridor; drop --line racing");
        if (!options.obstructions.empty() || options.drive_cones || options.sensor_seed || options.perception_seed || options.controller ||
            options.local_planner || options.envelope_out || options.lattice_out || options.discipline)
            throw std::invalid_argument("The minimum-time problem is the car and the corridor alone; drop the run's scenario, instruments, "
                                        "perception, planner, controller and judging flags");
        if (options.lap_problem_out && std::filesystem::exists(*options.lap_problem_out) && !std::filesystem::is_empty(*options.lap_problem_out))
            throw std::invalid_argument("Problem directory is not empty; choose a fresh --write-lap-problem directory");
    }
    if (options.steering==fd::SteeringMode::model_acceleration_pursuit && std::holds_alternative<fd::KinematicBicycle>(options.model))
        throw std::invalid_argument("--steering map applies only to --plant dynamic; the kinematic bicycle's steering table is its geometry");
    if (options.speed_plan==fd::SpeedPlanMode::performance_envelope && !std::holds_alternative<fd::FourWheelCar>(options.model))
        throw std::invalid_argument("--speed-plan envelope applies only to --plant four-wheel; no other plant has a derived envelope");
    if (options.request.duration_cap_s<=0 || options.request.duration_cap_s>3600) throw std::invalid_argument("duration must be in (0,3600]");
    auto& events=options.request.grip_events;
    std::stable_sort(events.begin(),events.end(),[](const auto& a,const auto& b){return a.time_s<b.time_s;});
    for (std::size_t i=1;i<events.size();++i)
        if (std::ceil(events[i-1].time_s/options.config.fixed_dt_s-1e-8)==
            std::ceil(events[i].time_s/options.config.fixed_dt_s-1e-8))
            throw std::invalid_argument("Only one grip event is allowed per simulation tick");
    return options;
}

// Each time ahead's prediction error on one line, after its heading.
void print_errors(const std::vector<fd::PredictionError>& errors) {
    for (const auto& e : errors) {
        std::cout << std::fixed << std::setprecision(3) << "; " << std::setprecision(1) << e.ahead_s << " s ahead " << std::setprecision(3);
        if (e.samples == 0) std::cout << "not reached";
        else std::cout << "median " << e.median_m << " m, 95th percentile " << e.percentile_95_m << " m, worst " << e.worst_m << " m over "
                       << e.samples << ", along " << e.median_along_m << " m, speed " << e.median_speed_mps << " m/s";
    }
    std::cout << std::defaultfloat;
}

// The prediction error of a predictive controller's recorded plans over the horizon (TrackWayFastPlan Phase 6.2, decision
// 0025): the plan error against where the car went, and the model error against the plant's response to each plan.
int report_prediction_error(const std::filesystem::path& directory) {
    const auto recording=fd::load_recording(directory);
    if (!recording.controller_recorded() || recording.metadata.predictive_controller.mode==fd::ControllerMode::policy)
        throw std::invalid_argument("The recording was driven by the policy; prediction errors are a predictive controller's");
    if (!recording.commands_recorded())
        throw std::invalid_argument("The recording is schema "+std::to_string(recording.metadata.schema_version)+
                                    "; its plans' commands, which the model error needs, were recorded from schema 13");
    std::vector<double> ahead;
    for (int k=0; k<=12; ++k) ahead.push_back(0.25*k);
    const auto plan=fd::controller_plan_errors(recording, ahead);
    const auto model=fd::controller_model_errors(recording, ahead);
    std::cout << "Recording: " << std::filesystem::absolute(directory).string() << '\n'
              << "Vehicle: " << recording.metadata.model << "; controller " << fd::controller_mode_name(recording.metadata.predictive_controller.mode)
              << "; " << model.front().samples << " plans that drove\n"
              << "Plan error: from where the car went. Model error: from where the recorded plant goes under each plan's commands,\n"
              << "from the state and with the configuration the plan was made in. Distances are between rear axles; along is\n"
              << "positive ahead of the plan, speed positive faster than it; medians, with the 95th percentile of the size.\n"
              << "ahead_s,plans_reached,plan_median_m,plan_p95_m,plan_worst_m,plan_along_m,plan_speed_mps,"
                 "model_median_m,model_p95_m,model_worst_m,model_along_m,model_across_m,model_speed_mps,model_speed_p95_mps\n"
              << std::fixed;
    for (std::size_t k=0; k<ahead.size(); ++k) {
        const auto& p=plan[k];
        const auto& m=model[k];
        std::cout << std::setprecision(2) << ahead[k] << ',' << p.samples << std::setprecision(4) << ',' << p.median_m << ',' << p.percentile_95_m
                  << ',' << p.worst_m << ',' << p.median_along_m << ',' << p.median_speed_mps << ',' << m.median_m << ',' << m.percentile_95_m
                  << ',' << m.worst_m << ',' << m.median_along_m << ',' << m.median_across_m << ',' << m.median_speed_mps << ','
                  << m.percentile_95_speed_mps << '\n';
    }
    // The standing start pulls away from rest; the flying laps, from the first lap crossing on, show the rest of the lap.
    const double dt=recording.metadata.initial_config.fixed_dt_s;
    const auto flying=[&](const std::vector<fd::PlanDeviation>& all) {
        std::vector<fd::PlanDeviation> kept;
        for (const auto& d : all)
            if (recording.samples[static_cast<std::size_t>(std::llround(recording.decisions[d.decision].time_s/dt))].laps>=1) kept.push_back(d);
        return kept;
    };
    const std::vector<double> summary_ahead{1.0, 3.0};
    const auto plan_deviations=fd::controller_plan_deviations(recording, summary_ahead);
    const auto model_deviations=fd::controller_model_deviations(recording, summary_ahead);
    std::cout << "Flying laps only:";
    for (std::size_t k=0; k<summary_ahead.size(); ++k) {
        const auto p=fd::prediction_error(summary_ahead[k], flying(plan_deviations[k]));
        const auto m=fd::prediction_error(summary_ahead[k], flying(model_deviations[k]));
        std::cout << std::setprecision(1) << (k ? ";" : "") << ' ' << summary_ahead[k] << " s ahead over " << m.samples << " plans, "
                  << std::setprecision(4) << "plan error median " << p.median_m << " m, 95th percentile " << p.percentile_95_m
                  << " m; model error median " << m.median_m << " m, 95th percentile " << m.percentile_95_m << " m, along " << m.median_along_m
                  << " m, across " << m.median_across_m << " m";
    }
    std::cout << '\n';
    // Where the prediction model was furthest off: the plans whose plant response strayed most a second on.
    auto worst=model_deviations.front();
    std::sort(worst.begin(),worst.end(),[](const fd::PlanDeviation& a, const fd::PlanDeviation& b){ return a.distance_m>b.distance_m; });
    std::cout << "Largest model errors 1 s ahead:\n"
                 "decision_time_s,station_m,speed_mps,applied_acceleration_mps2,distance_m,along_m,across_m,speed_difference_mps\n";
    for (std::size_t k=0; k<std::min<std::size_t>(8,worst.size()); ++k) {
        const auto& w=worst[k];
        const auto& decision=recording.decisions[w.decision];
        const auto& sample=recording.samples[static_cast<std::size_t>(std::llround(decision.time_s/dt))];
        std::cout << std::setprecision(2) << decision.time_s << ',' << sample.path_s_m << ',' << sample.speed_mps << ','
                  << decision.applied.acceleration_mps2 << std::setprecision(4) << ',' << w.distance_m << ',' << w.along_m << ','
                  << w.across_m << ',' << w.speed_mps << '\n';
    }
    return 0;
}

// How the judge saw a run: laps against those required, each lap's time, what the penalties cost, and the total.
void print_competition(const fd::CompetitionRules& rules, const std::vector<fd::TimingEvent>& events, std::size_t cones,
                       const std::string& course) {
    std::vector<double> laps;
    double penalty=0, off_course=0;
    int hits=0, excursions=0;
    bool started=false, finished=false, stopped=false, unsafe=false;
    std::string dnf;
    for (const auto& e : events) {
        switch (e.kind) {
        case fd::TimingKind::start: started=true; break;
        case fd::TimingKind::lap: laps.push_back(e.seconds); break;
        case fd::TimingKind::finish: finished=true; break;
        case fd::TimingKind::cone_hit: ++hits; penalty+=e.seconds; break;
        case fd::TimingKind::off_course: ++excursions; penalty+=e.seconds; break;
        case fd::TimingKind::back_on_course: off_course+=e.seconds; break;
        case fd::TimingKind::stopped: stopped=true; break;
        case fd::TimingKind::unsafe_stop: unsafe=true; penalty+=e.seconds; break;
        case fd::TimingKind::dnf: dnf=e.reason; break;
        case fd::TimingKind::sector: break;
        }
    }
    const int required=rules.discipline==fd::Discipline::autocross ? 1 : rules.trackdrive_laps;
    double sum=0, best=laps.empty() ? 0 : *std::min_element(laps.begin(),laps.end());
    for (const double l : laps) sum+=l;
    // Its own format, the caller's restored after: the report around it keeps whatever it had set.
    const auto flags=std::cout.flags();
    const auto precision=std::cout.precision();
    std::cout << std::fixed << std::setprecision(3) << "Competition: " << fd::discipline_name(rules.discipline) << " on the course "
              << course << " (" << cones << " cones); " << (started ? "" : "never started; ") << laps.size() << " of " << required
              << " laps timed";
    if (!laps.empty()) {
        std::cout << " (";
        for (std::size_t i=0;i<laps.size();++i) std::cout << (i ? ", " : "") << laps[i];
        std::cout << " s, best " << best << " s)";
    }
    std::cout << "; " << hits << " cone(s) down or out, " << excursions << " time(s) off course (" << off_course << " s)"
              << (unsafe ? ", an unsafe stop" : "") << ", " << penalty << " s of penalties";
    if (!dnf.empty()) std::cout << "; DNF: " << dnf;
    else if (finished) std::cout << "; finished in " << sum << " s, " << sum+penalty << " s with penalties"
                                 << (stopped ? ", stopped safely" : unsafe ? "" : ", its stop not judged: the run ended at the line");
    else std::cout << "; not finished";
    std::cout << '\n';
    std::cout.flags(flags);
    std::cout.precision(precision);
}

int check_recording(const std::filesystem::path& directory) {
    const auto recording=fd::load_recording(directory);
    const double reproduction=fd::plan_reproduction_error_mps(recording);
    const bool same_source=recording.metadata.source_fingerprint==FD_SOURCE_FINGERPRINT;
    std::cout << std::fixed << std::setprecision(3)
              << "Recording: " << std::filesystem::absolute(directory).string() << '\n'
              << "Contract: schema " << recording.metadata.schema_version << " valid; " << recording.samples.size()
              << " samples over " << recording.samples.back().time_s << " s; " << recording.plans.size()
              << " plan revisions; " << recording.events.size() << " parameter events; laps " << recording.summary.laps
              << "; invalid samples " << recording.summary.invalid_samples << '\n'
              << "Scenario: " << recording.metadata.scenario_obstructions.size() << " stated blocked region(s)";
    for (const auto& o : recording.metadata.scenario_obstructions)
        std::cout << "; " << o.identifier << " s " << o.from_s_m << ".." << o.to_s_m
                  << " m, offset " << o.from_offset_m << ".." << o.to_offset_m << " m";
    std::cout << '\n';
    if (recording.decisions_recorded()) {
        std::size_t alternatives=0, holding=0, points=0;
        for (const auto& d : recording.decisions) {
            if (d.options.size()>1) ++alternatives;
            if (d.holding) ++holding;
            for (const auto& o : d.options) points+=o.trajectory.size();
        }
        std::cout << "Decisions: " << recording.decisions.size() << " recorded; " << alternatives
                  << " evaluated alternatives; " << holding << " held for a blockage; " << points << " predicted points\n";
    } else {
        std::cout << "Decisions: not recorded (schema " << recording.metadata.schema_version << ")\n";
    }
    // What the car's own instruments made of the run, against what the run truly was at the same tick (decision 0029).
    if (recording.sensors_recorded()) {
        const auto& settings=*recording.metadata.sensors;
        double worst_age=0, age_sum=0, worst_position=0, worst_speed=0, worst_yaw_rate=0;
        std::size_t measured=0;
        for (std::size_t i=0;i<recording.measurements.size();++i) {
            const auto& m=recording.measurements[i].measured;
            const auto& s=recording.samples[i];
            if (!m.pose.measured) continue;
            ++measured;
            const double age=s.time_s-m.pose.sampled_at_s;
            worst_age=std::max(worst_age,age);
            age_sum+=age;
            worst_position=std::max(worst_position,std::hypot(m.x_m-s.x_m,m.y_m-s.y_m));
            if (m.speed.measured) worst_speed=std::max(worst_speed,std::abs(m.speed_mps-s.speed_mps));
            if (m.imu.measured) worst_yaw_rate=std::max(worst_yaw_rate,std::abs(m.yaw_rate_radps-s.yaw_rate_radps));
        }
        std::cout << "Sensors: seed " << settings.seed << ", fingerprint " << recording.metadata.sensor_fingerprint
                  << "; pose and speed at " << settings.pose.rate_hz << " Hz behind " << settings.pose.dead_time_s*1000
                  << " ms, wheels, inertial unit and steering at " << settings.wheel_speeds.rate_hz << " Hz behind "
                  << settings.wheel_speeds.dead_time_s*1000 << " ms; " << recording.measurements.size()
                  << " ticks measured, the pose delivered on " << measured << " of them, up to " << worst_age*1000
                  << " ms old (" << (measured ? age_sum/static_cast<double>(measured) : 0)*1000
                  << " ms on average), out by up to " << worst_position << " m, speed by up to " << worst_speed
                  << " m/s, yaw rate by up to " << worst_yaw_rate << " rad/s; "
                  << (recording.cone_driving_recorded() ? "the driver decided from these measurements, the plant advancing on its true state"
                                                        : "the car drove on its true state")
                  << '\n';
    } else {
        std::cout << "Sensors: none; the car read its own state exactly"
                  << (recording.measurements_recorded() ? "" : " (not recordable before schema 14)") << '\n';
    }
    // What the simulated cone perception made of the run, against the cones that were there (decision 0030).
    if (recording.perception_recorded()) {
        const auto& settings=*recording.metadata.perception;
        std::size_t detections=0, missed=0, coloured=0, wrong=0, unknown=0;
        double farthest=0;
        for (const auto& f : recording.perception_frames) {
            detections+=f.frame.detections.size();
            missed+=f.missed.size();
            for (std::size_t k=0;k<f.frame.detections.size();++k) {
                const auto& d=f.frame.detections[k];
                farthest=std::max(farthest,std::hypot(d.position.x,d.position.y));
                if (d.colour==fd::ObservedColour::unknown) { ++unknown; continue; }
                ++coloured;
                if (d.colour!=fd::observed(recording.cones[f.source_cone[k]].colour)) ++wrong;
            }
        }
        const auto& front=settings.sensors.front();
        std::cout << "Perception: seed " << settings.seed << ", fingerprint " << recording.metadata.perception_fingerprint << "; "
                  << recording.cones.size() << " cones; " << settings.sensors.size() << " sensor(s), the first at " << front.rate_hz
                  << " Hz behind " << front.dead_time_s*1000 << " ms seeing " << front.min_range_m << " to " << front.max_range_m
                  << " m; " << recording.perception_frames.size() << " frames, " << detections << " simulated detections ("
                  << (recording.perception_frames.empty() ? 0.0 : static_cast<double>(detections)/static_cast<double>(recording.perception_frames.size()))
                  << " a frame, the farthest " << farthest << " m), " << missed << " cones in view missed, " << wrong << " of "
                  << coloured << " coloured detections the wrong colour, " << unknown << " uncoloured; "
                  << (recording.cone_driving_recorded() ? "the driver planned from them" : "nothing drove on them") << '\n';
    } else {
        std::cout << "Perception: none"
                  << (recording.perception_files_recorded() ? "" : " (not recordable before schema 15)") << '\n';
    }
    // What the driver believed on cones, and how far that was from the course, which only this evaluation reads.
    if (recording.cone_driving_recorded()) {
        std::vector<double> apart;
        for (const auto& believed : recording.believed_paths) {
            double furthest=0;
            for (const auto& p : believed.path.points) {
                if (p.s_m>15) break;
                furthest=std::max(furthest,fd::project(recording.track,{p.x_m,p.y_m}).distance_m);
            }
            apart.push_back(furthest);
        }
        std::sort(apart.begin(),apart.end());
        std::size_t without=0;
        double first_followed=-1;
        for (const auto& b : recording.beliefs) {
            if (!b.path) ++without;
            else if (first_followed<0) first_followed=b.state.time_s;
        }
        std::cout << "Cones: driven on the believed path from the " << fd::belief_source_name(recording.metadata.cone_driving->source)
                  << " state; " << recording.believed_paths.size() << " paths believed, the first followed at " << first_followed
                  << " s, " << without << " decisions braking with none; over their first 15 m they lay a median "
                  << (apart.empty() ? 0.0 : apart[apart.size()/2]) << " m and at worst " << (apart.empty() ? 0.0 : apart.back())
                  << " m from the true centreline; a driver run again on the recorded readings and frames alone believed and "
                     "commanded the same at every decision\n";
    } else {
        std::cout << "Cones: not driven on" << (recording.cone_driving_files_recorded() ? "" : " (not recordable before schema 16)") << '\n';
    }
    if (recording.competition_recorded()) {
        print_competition(recording.metadata.competition->rules,recording.timing,recording.cones.size(),recording.metadata.competition->course);
        std::cout << "Judged again: the recorded samples on the recorded course give the same " << recording.timing.size() << " events\n";
    } else {
        std::cout << "Competition: not judged (schema " << recording.metadata.schema_version << ")\n";
    }
    if (!recording.metadata.vehicle_profile.empty()) {
        const auto& profile=fd::vehicle_profile(recording.metadata.vehicle_profile);
        std::cout << "Profile: " << profile.name << ", exactly as this build gives it; " << profile.source << '\n';
    }
    std::cout << "Vehicle: " << recording.metadata.model;
    if (const auto* car=std::get_if<fd::DynamicSingleTrack>(&recording.metadata.vehicle_model))
        std::cout << "; " << car->mass_kg << " kg, drive front fraction " << car->drive_front_fraction
                  << "; max rear sideslip " << recording.summary.max_rear_sideslip_rad << " rad";
    if (const auto* four=std::get_if<fd::FourWheelCar>(&recording.metadata.vehicle_model)) {
        std::size_t locked=0, lifted=0;
        double front_gain=0, peak_wheel_use=0;
        const double static_front=fd::quasi_static_wheel_loads(*four,recording.metadata.initial_config,0,0,0)[fd::front_left];
        for (const auto& s : recording.samples) {
            if (s.speed_mps>four->slip_speed_floor_mps && std::any_of(s.wheel_speeds_radps.begin(),s.wheel_speeds_radps.end(),[](double w){return w==0;}))
                ++locked;
            if (std::any_of(s.wheel_loads_n.begin(),s.wheel_loads_n.end(),[](double load){return load==0;})) ++lifted;
            front_gain=std::max(front_gain,s.wheel_loads_n[fd::front_left]+s.wheel_loads_n[fd::front_right]-2*static_front);
            // What each tire was asked of its own friction circle, through the seam, as the desktop shows it.
            const auto asked=fd::demand(recording.metadata.vehicle_model,s.plant_state(),
                                        recording.config_for_revision(s.revision),s.applied_acceleration_mps2);
            for (std::size_t w=0;w<4;++w)
                peak_wheel_use=std::max(peak_wheel_use,std::hypot(asked.wheel_longitudinal_use[w],asked.wheel_lateral_use[w]));
        }
        std::cout << "; " << four->mass_kg << " kg, brake bias front " << four->brake_bias_front << ", drive front fraction "
                  << four->drive_front_fraction << ", " << four->max_drive_power_w/1000 << " kW, centre of gravity "
                  << four->cg_height_m << " m high, roll balance front " << four->roll_balance_front << ", drag area " << four->drag_area_m2
                  << " m^2, downforce area " << four->downforce_area_m2 << " m^2, aero balance front " << four->aero_balance_front
                  << ", viscous coupling " << four->viscous_coupling_nms << " N m s/rad; max rear sideslip "
                  << recording.summary.max_rear_sideslip_rad << " rad; " << locked << " samples with a wheel locked while moving; "
                  << lifted << " with a wheel lifted; front axle load up to " << front_gain << " N above static; peak friction use of any wheel "
                  << peak_wheel_use;
        if (!recording.loads_recorded()) std::cout << " (no load transfer before schema 6)";
        else if (!recording.aerodynamics_recorded()) std::cout << " (no aerodynamics or coupling before schema 7)";
    }
    if (!recording.plant_recorded()) std::cout << " (plant state not recorded, schema " << recording.metadata.schema_version << ")";
    const auto plant=fd::plant_reproduction_error_m(recording);
    std::cout << '\n' << "Steering: ";
    if (recording.metadata.steering_mode==fd::SteeringMode::pure_pursuit) {
        std::cout << "Pure Pursuit";
        if (!recording.steering_recorded()) std::cout << " (not recorded before schema 4; those runs used it)";
    } else {
        double largest=0;
        for (const auto& s : recording.samples) largest=std::max(largest,std::abs(s.requested_steering_rad-s.geometric_steering_rad));
        std::cout << "MAP from steady-state steering table " << recording.metadata.steering_table_fingerprint
                  << (*fd::recorded_steering_table_matches(recording) ? " (this build derives it from the recorded plant)"
                                                                      : " (this build derives a different table from the recorded plant)")
                  << "; largest difference from geometric steering " << largest << " rad";
    }
    std::cout << '\n' << "Speed plan: ";
    if (recording.metadata.speed_plan_mode==fd::SpeedPlanMode::grip_fractions) {
        std::cout << "grip fractions";
        if (!recording.speed_plan_recorded()) std::cout << " (not recorded before schema 8; those runs used them)";
    } else {
        std::cout << std::fixed << std::setprecision(2) << recording.metadata.initial_config.envelope_fraction
                  << " of performance envelope " << recording.metadata.envelope_fingerprint
                  << (*fd::recorded_performance_envelope_matches(recording) ? " (this build derives it from the recorded car)"
                                                                            : " (this build derives a different envelope from the recorded car)")
                  << std::defaultfloat;
    }
    std::cout << '\n' << "Local planner: " << fd::local_planner_mode_name(recording.metadata.local_planner_mode);
    if (!recording.local_planner_recorded()) {
        std::cout << " (not recorded before schema 10; those runs used it)";
    } else if (recording.metadata.local_planner_mode==fd::LocalPlannerMode::lattice) {
        std::size_t passing=0, returning=0, braking=0, path_points=0;
        for (const auto& d : recording.decisions) {
            const auto action=d.choice().action;
            if (action==fd::LocalAction::pass_left || action==fd::LocalAction::pass_right) ++passing;
            if (action==fd::LocalAction::straight && !d.choice().path.empty()) ++returning;
            if (action==fd::LocalAction::brake) ++braking;
            for (const auto& o : d.options) path_points+=o.path.size();
        }
        std::cout << "; " << passing << " decisions passing along the lattice, " << returning << " returning along it, "
                  << braking << " braking; " << path_points << " lattice path points";
        if (!recording.speed_profiles_recorded()) {
            std::cout << "; paths compared by cost at the reference plan's speed (before schema 11)";
        } else {
            std::size_t compared=0;
            for (const auto& d : recording.decisions)
                if (std::count_if(d.options.begin(),d.options.end(),[](const fd::RecordedOption& o){return o.clear && o.estimated_time_s;})>=2) ++compared;
            std::cout << "; " << compared << " decisions took the fastest of several clear actions by estimated time";
        }
    }
    std::cout << '\n' << "Controller: " << fd::controller_mode_name(recording.metadata.predictive_controller.mode);
    if (!recording.controller_recorded()) {
        std::cout << " (not recorded before schema 12; those runs used it)";
    } else if (recording.metadata.predictive_controller.mode!=fd::ControllerMode::policy) {
        const auto& values=recording.metadata.predictive_controller.values;
        const auto value=[&](const char* key){ return std::find_if(values.begin(),values.end(),[&](const auto& v){return v.first==key;})->second; };
        std::size_t driven=0, held=0, refused=0;
        for (const auto& d : recording.decisions) {
            if (d.driven_by!=fd::ControllerMode::policy) ++driven;
            else if (!d.controller_status) ++held;
            else ++refused;
        }
        std::cout << " (" << std::llround(value("stages")) << " stages of " << std::setprecision(3) << value("stage_s") << std::defaultfloat
                  << " s); it drove " << driven << " decisions; the policy drove " << held << " holding for a blockage and " << refused
                  << " whose plan was not solved or not clear";
        const std::vector<double> ahead{0.5, 1.0, 2.0, 3.0};
        std::cout << "\nPlan error: its plan's rear axle from the recorded one";
        print_errors(fd::controller_plan_errors(recording, ahead));
        if (!recording.commands_recorded()) {
            std::cout << "\nModel error: not measured; its plans' commands were not recorded before schema 13";
        } else {
            std::cout << "\nModel error: the recorded plant under its plan's commands from its plan's rear axle";
            print_errors(fd::controller_model_errors(recording, ahead));
        }
    }
    std::cout << '\n'
              << "Source: " << recording.metadata.source_fingerprint << (same_source?" (this build)":" (different build)") << '\n'
              << std::scientific << "Planner reproduction: max speed difference " << reproduction << " m/s\n";
    if (plant) std::cout << "Plant reproduction: max position difference over one step " << *plant << " m\n";
    else std::cout << "Plant reproduction: not available (schema 1 recorded no steering command)\n";
    return 0;
}
}

// Derives the performance envelope for the configured car and writes it (decision 0017).
int write_envelope(const Options& options) {
    const auto& directory=*options.envelope_out;
    if (std::filesystem::exists(directory) && !std::filesystem::is_empty(directory))
        throw std::invalid_argument("Envelope directory is not empty; choose a fresh --write-envelope directory");
    fd::performance_envelope_fingerprint(options.model,options.config);  // refuses other plants before any work
    const auto began=std::chrono::steady_clock::now();
    const fd::PerformanceEnvelope envelope(options.model,options.config);
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
    fd::write_performance_envelope(envelope,options.model,options.config,directory);
    std::cout << std::fixed << std::setprecision(3) << "Envelope " << envelope.fingerprint() << ": " << fd::vehicle_model_name(options.model)
              << ", grip " << options.config.grip_mu << ", " << envelope.rows().size() << " speeds from " << envelope.rows().front().speed_mps
              << " to " << envelope.rows().back().speed_mps << " m/s, derived in " << seconds << " s\n";
    for (const auto& row : envelope.rows())
        std::cout << "  " << row.speed_mps << " m/s: lateral " << row.left.lateral_limit_mps2 << " left / " << row.right.lateral_limit_mps2
                  << " right m/s^2 (" << fd::envelope_limit_name(row.left.lateral_limit) << "), forward " << row.left.levels.front().forward_mps2
                  << " (" << fd::envelope_limit_name(row.left.levels.front().forward_limit) << "), braking " << row.left.levels.front().braking_mps2
                  << " m/s^2 (" << fd::envelope_limit_name(row.left.levels.front().braking_limit) << ")\n";
    std::cout << "Outputs: " << std::filesystem::absolute(directory).string() << '\n';
    return 0;
}

// Lays the offline lattice along the line a run would drive and writes it (decision 0021).
int write_lattice(const Options& options,const fd::Track& track) {
    const auto& directory=*options.lattice_out;
    const fd::LatticeOptions lattice_options;
    const auto began=std::chrono::steady_clock::now();
    const auto lattice=fd::make_lattice(track,options.config,lattice_options);
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
    std::filesystem::create_directories(directory);
    const auto open=[&](const char* name) {
        std::ofstream file(directory/name);
        if (!file) throw std::runtime_error(std::string("Cannot write ")+(directory/name).string());
        file << std::setprecision(12);
        return file;
    };
    std::size_t nodes=0;
    for (const auto& layer : lattice.layers) nodes+=layer.offsets_m.size();
    std::vector<std::vector<std::size_t>> in(lattice.layers.size());
    for (std::size_t i=0; i<lattice.layers.size(); ++i) in[i].assign(lattice.layers[i].offsets_m.size(),0);
    for (const auto& edge : lattice.edges) ++in[edge.to_layer][edge.to_node];
    {
        auto file=open("layers.csv");
        file << "layer,s_m,x_m,y_m,left_normal_x,left_normal_y,curvature_1pm,nodes,reference_node,lowest_offset_m,highest_offset_m\n";
        for (std::size_t i=0; i<lattice.layers.size(); ++i) {
            const auto& l=lattice.layers[i];
            file << i << ',' << l.s_m << ',' << l.reference.x_m << ',' << l.reference.y_m << ',' << l.left_normal.x << ',' << l.left_normal.y << ','
                 << l.reference.curvature << ',' << l.offsets_m.size() << ',' << l.reference_node << ',' << l.offsets_m.front() << ',' << l.offsets_m.back() << '\n';
        }
    }
    {
        auto file=open("nodes.csv");
        file << "layer,node,offset_m,x_m,y_m,edges_out,edges_in\n";
        for (std::size_t i=0; i<lattice.layers.size(); ++i) {
            const auto& l=lattice.layers[i];
            for (std::size_t n=0; n<l.offsets_m.size(); ++n)
                file << i << ',' << n << ',' << l.offsets_m[n] << ',' << l.reference.x_m+l.left_normal.x*l.offsets_m[n] << ','
                     << l.reference.y_m+l.left_normal.y*l.offsets_m[n] << ',' << lattice.edges_from(i,n).size() << ',' << in[i][n] << '\n';
        }
    }
    {
        auto edges=open("edges.csv");
        auto samples=open("edge_samples.csv");
        edges << "edge,from_layer,from_node,to_layer,to_node,from_offset_m,to_offset_m,length_m,peak_curvature_1pm,cost\n";
        samples << "edge,sample,x_m,y_m,s_m,offset_m,curvature_1pm\n";
        for (std::size_t e=0; e<lattice.edges.size(); ++e) {
            const auto& edge=lattice.edges[e];
            double peak=0;
            for (std::size_t k=0; k<edge.samples.size(); ++k) {
                const auto& s=edge.samples[k];
                peak=std::max(peak,std::abs(s.curvature));
                samples << e << ',' << k << ',' << s.x_m << ',' << s.y_m << ',' << s.s_m << ',' << s.offset_m << ',' << s.curvature << '\n';
            }
            edges << e << ',' << edge.from_layer << ',' << edge.from_node << ',' << edge.to_layer << ',' << edge.to_node << ','
                  << lattice.layers[edge.from_layer].offsets_m[edge.from_node] << ',' << lattice.layers[edge.to_layer].offsets_m[edge.to_node] << ','
                  << edge.length_m << ',' << peak << ',' << edge.cost << '\n';
        }
    }
    const double limit=std::tan(options.config.max_steering_rad)/options.config.wheelbase_m;
    {
        const auto& o=lattice_options;
        auto file=open("lattice.json");
        file << "{\n  \"source_fingerprint\": \"" << FD_SOURCE_FINGERPRINT << "\",\n  \"track_name\": \"" << track.name << "\",\n"
             << "  \"track_length_m\": " << track.length_m << ",\n  \"corridor_edges\": " << (track.left_edge_m.empty() ? "false" : "true") << ",\n"
             << "  \"steering_limit_1pm\": " << limit << ",\n"
             << "  \"options\": {\"lateral_spacing_m\": " << o.lateral_spacing_m << ", \"straight_layer_spacing_m\": " << o.straight_layer_spacing_m
             << ", \"curve_layer_spacing_m\": " << o.curve_layer_spacing_m << ", \"curve_threshold_1pm\": " << o.curve_threshold_1pm
             << ", \"lateral_change_per_metre\": " << o.lateral_change_per_metre << ", \"vehicle_half_width_m\": " << o.vehicle_half_width_m
             << ", \"sample_spacing_m\": " << o.sample_spacing_m << ",\n    \"weights\": {\"reference_deviation\": " << o.weights.reference_deviation
             << ", \"reference_deviation_limit\": " << o.weights.reference_deviation_limit << ", \"length\": " << o.weights.length
             << ", \"average_curvature\": " << o.weights.average_curvature << ", \"curvature_range\": " << o.weights.curvature_range << "}},\n"
             << "  \"layers\": " << lattice.layers.size() << ",\n  \"nodes\": " << nodes << ",\n  \"edges\": " << lattice.edges.size() << ",\n"
             << "  \"generated_edges\": " << lattice.generated_edges << ",\n  \"removed_for_steering\": " << lattice.removed_for_steering << ",\n"
             << "  \"removed_for_corridor\": " << lattice.removed_for_corridor << ",\n  \"removed_as_dead_ends\": " << lattice.removed_as_dead_ends << "\n}\n";
    }
    std::cout << std::fixed << std::setprecision(3) << "Lattice: " << track.name << ", " << track.length_m << " m; " << lattice.layers.size()
              << " layers, " << nodes << " nodes, " << lattice.edges.size() << " edges kept of " << lattice.generated_edges << " (removed "
              << lattice.removed_for_steering << " beyond the steering limit of " << limit << " 1/m, " << lattice.removed_for_corridor
              << " leaving the corridor, " << lattice.removed_as_dead_ends << " as dead ends), laid in " << seconds << " s\n"
              << "Outputs: " << std::filesystem::absolute(directory).string() << '\n';
    return 0;
}

// Writes the minimum-time problem an offline solver is given (TrackWayFastPlan Phase 4.1, decision 0034).
int write_lap_problem(const Options& options,const fd::LapProblem& problem) {
    const auto& directory=*options.lap_problem_out;
    fd::write_lap_problem(directory,problem);
    const auto& car=problem.car;
    std::cout << std::fixed << std::setprecision(3) << "Problem: " << (problem.profile.empty() ? std::string("the configured four-wheel car") : problem.profile)
              << ", " << car.mass_kg << " kg, " << car.max_drive_power_w/1000 << " kW, on " << problem.corridor.name << ", "
              << problem.corridor.length_m << " m in " << problem.corridor.points.size() << " samples; speed cap "
              << problem.config.max_speed_mps << " m/s, steering lock " << problem.config.max_steering_rad << " rad; fingerprint "
              << fd::lap_problem_fingerprint(problem) << '\n' << std::defaultfloat
              << "Outputs: " << std::filesystem::absolute(directory).string() << '\n';
    return 0;
}

// Validates a reference-optimal artefact and requires it to be the solution of this build's problem (decision 0034).
int check_optimal_lap(const std::filesystem::path& directory,const fd::LapProblem& problem) {
    const auto lap=fd::load_optimal_lap(directory);
    if (lap.problem_fingerprint!=fd::lap_problem_fingerprint(problem))
        throw std::invalid_argument("The artefact solves another problem than these flags pose in this build (its car "+
                                    (lap.problem.profile.empty() ? std::string("the configured four-wheel car") : lap.problem.profile)+
                                    " on "+lap.problem.corridor.name+"); give the flags it was written with");
    const auto flags=std::cout.flags();
    const auto precision=std::cout.precision();
    std::cout << std::fixed << std::setprecision(3) << "Optimal lap: " << std::filesystem::absolute(directory).string() << '\n'
              << "Solver: " << lap.tool << ' ' << lap.version << " (" << lap.release << ", sha256 " << lap.release_sha256.substr(0,12)
              << "...), model " << lap.model << ", " << lap.nlp_solver << ": " << lap.status << (lap.converged ? "" : " (NOT CONVERGED)")
              << " after " << lap.iterations << " of " << lap.max_iterations << " iterations, tolerance " << std::scientific
              << std::setprecision(1) << lap.tolerance << std::fixed << std::setprecision(3) << ", " << lap.mesh_points << " mesh points\n"
              << "Problem: " << (lap.problem.profile.empty() ? std::string("the configured four-wheel car") : lap.problem.profile) << " on "
              << lap.problem.corridor.name << ", " << lap.problem.corridor.length_m << " m; speed cap " << lap.speed_cap_mps
              << " m/s; fingerprint " << lap.problem_fingerprint << " matches this build\n"
              << "Lap time: " << lap.lap_time_s << " s, " << lap.label << '\n';
    double fastest=0, slowest=1e9;
    for (const auto& p : lap.points) { fastest=std::max(fastest,p.speed_mps()); slowest=std::min(slowest,p.speed_mps()); }
    std::cout << "Speed: " << slowest << " to " << fastest << " m/s\n"
              << "Tire energy: front left " << lap.tire_energy_j[0]/1000 << " kJ, front right " << lap.tire_energy_j[1]/1000
              << " kJ, rear left " << lap.tire_energy_j[2]/1000 << " kJ, rear right " << lap.tire_energy_j[3]/1000 << " kJ\n";
    for (const auto& s : lap.sensitivities) {
        std::cout << "Sensitivity: " << s.parameter << " at " << std::defaultfloat << std::setprecision(6) << s.value << ' ' << s.unit
                  << ": " << std::setprecision(4) << s.seconds_per_unit << " s per " << s.unit << " (" << s.method;
        if (s.cross_check_seconds_per_unit) std::cout << "; re-solving gives " << *s.cross_check_seconds_per_unit;
        std::cout << "); " << std::showpos << s.step << std::noshowpos << ' ' << s.unit << " would be worth " << std::fixed
                  << std::setprecision(4) << std::showpos << s.seconds_per_unit*s.step << std::noshowpos << " s\n" << std::setprecision(3);
    }
    for (const auto& m : lap.mapping) std::cout << "Mapping: " << m << '\n';
    std::cout.flags(flags);
    std::cout.precision(precision);
    return 0;
}

int main(int argc,char** argv) {
    try {
        const auto options=parse(argc,argv);
        if (options.check) return check_recording(*options.check);
        if (options.prediction_error) return report_prediction_error(*options.prediction_error);
        if (options.envelope_out) return write_envelope(options);
        fd::Track track=fd::make_preset_track();
        std::optional<fd::Course> course;
        if (options.course_file) {
            if (options.track_file || options.track_width || options.racing_line)
                throw std::invalid_argument("--course brings its own track and corridor; drop --track, --track-width and --line racing");
            const auto layout=fd::load_pacsim_course(*options.course_file);
            course=fd::course_from_layout(layout);
            double narrowest=1e9, widest=0, sharpest=0;
            for (std::size_t i=0;i<course->track.points.size();++i) {
                const double w=course->track.left_edge_m[i]+course->track.right_edge_m[i];
                narrowest=std::min(narrowest,w);
                widest=std::max(widest,w);
                sharpest=std::max(sharpest,std::abs(course->track.points[i].curvature));
            }
            std::cout << std::fixed << std::setprecision(3) << "Course: " << layout.name << " from a PacSim layout: " << layout.left.size()
                      << " blue, " << layout.right.size() << " yellow and " << layout.orange.size() << " orange cones, "
                      << layout.gates.size() << " timekeeping gate(s)" << (layout.gates.empty() ? " (start line laid at the start)" : "")
                      << "; traced into " << course->track.length_m << " m, " << narrowest << " to " << widest << " m wide, its tightest radius "
                      << 1/sharpest << " m\n" << std::defaultfloat;
            // Refused here, naming why, rather than by the speed plan's bare check.
            const double tightest=options.config.wheelbase_m/std::tan(options.config.max_steering_rad);
            if (1/sharpest<tightest)
                throw std::invalid_argument("--course "+layout.name+" bends to a radius of "+std::to_string(1/sharpest)+
                                            " m, tighter than this car can steer ("+std::to_string(tightest)+
                                            " m); configs/formula-student.cfg gives a Formula Student car's wheelbase and lock");
            track=course->track;
        }
        // The preset's centreline with another corridor about it, checked as any track is before anything is written.
        if (options.track_width && !options.track_file) {
            track.width_m=*options.track_width;
            fd::validate_track(track);
            double tightest=0;
            for (const auto& p : track.points) tightest=std::max(tightest,std::abs(p.curvature));
            // The inner boundary folds over itself where a bend is tighter than half the corridor.
            if (tightest*track.width_m/2>=1)
                throw std::invalid_argument("--track-width is too wide: the preset bends tighter than half of it");
        }
        std::optional<fd::ConditionedTrack> reference;
        if (options.track_file) {
            // Conditioned before anything is written, so a refused centreline leaves no run directory (decision 0019).
            const auto centreline=fd::load_centreline(*options.track_file);
            fd::ConditioningOptions conditioning;
            if (options.track_smoothing) conditioning.smoothing_rms_m=*options.track_smoothing;
            const auto conditioned=fd::condition_track(centreline,options.track_width.value_or(10.0),options.track_file->stem().string(),conditioning);
            double largest=0;
            for (const auto& p : conditioned.track.points) largest=std::max(largest,std::abs(p.curvature));
            std::cout << std::fixed << std::setprecision(3) << "Track: " << conditioned.track.name << " conditioned from " << centreline.size()
                      << " points: " << conditioned.track.length_m << " m, " << conditioned.track.points.size() << " samples, "
                      << conditioned.track.width_m << " m wide, residual " << conditioned.residual_rms_m << " m rms, max deviation "
                      << conditioned.max_deviation_m << " m, largest curvature " << largest << " 1/m\n" << std::defaultfloat;
            track=conditioned.track;
            reference=conditioned;
        }
#ifdef FD_HAVE_RACELINE
        if (options.racing_line) {
            // The racing line needs a conditioned centreline with normals; the preset is sampled every metre and conditioned.
            if (!reference) reference=fd::condition_track(track);
            fd::RacingLineOptions line_options;
            if (options.line_margin) line_options.margin_m=*options.line_margin;
            const auto began=std::chrono::steady_clock::now();
            const auto line=fd::make_minimum_curvature_line(*reference,options.config,line_options);
            const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
            std::optional<fd::PerformanceEnvelope> envelope;
            if (options.speed_plan==fd::SpeedPlanMode::performance_envelope) envelope.emplace(options.model,options.config);
            const auto lap=[&](const fd::Track& t) { return fd::estimated_lap_time(t,fd::make_speed_plan(t,options.config,envelope ? &*envelope : nullptr)); };
            const auto peak=[](const fd::Track& t) {
                double largest=0;
                for (const auto& p : t.points) largest=std::max(largest,std::abs(p.curvature));
                return largest;
            };
            std::cout << std::fixed << std::setprecision(3) << "Racing line: " << line.track.name << ", " << line.iterations << " iterations, "
                      << (line.converged ? "converged" : "not converged") << " (last shift " << line.last_shift_m << " m), derived in " << seconds
                      << " s; " << line.track.length_m << " m against the centreline's " << reference->track.length_m << " m; peak curvature "
                      << peak(line.track) << " against " << peak(reference->track) << " 1/m; estimated lap with the "
                      << fd::speed_plan_mode_name(options.speed_plan) << " plan " << lap(line.track) << " s against " << lap(reference->track) << " s\n"
                      << std::defaultfloat;
            track=line.track;
        }
#endif
        if (options.lattice_out) return write_lattice(options,track);
        if (options.lap_problem_out || options.optimal_check) {
            // The corridor the optimum is solved in: a course as traced, a centreline file as conditioned, and the preset
            // conditioned as a racing line's reference is (decision 0019).
            const fd::Track corridor=course ? course->track : reference ? reference->track : fd::condition_track(track).track;
            const auto problem=fd::make_lap_problem(corridor,std::get<fd::FourWheelCar>(options.model),options.config,
                                                    options.request.vehicle_profile);
            if (options.lap_problem_out) return write_lap_problem(options,problem);
            return check_optimal_lap(*options.optimal_check,problem);
        }
        std::shared_ptr<fd::PredictiveController> controller;
#ifdef FD_HAVE_MPCC
        if (options.controller==fd::ControllerMode::mpcc) controller=std::make_shared<fd::Mpcc>();
#endif
        fd::Simulation simulation(track,options.config,options.model,options.steering,options.speed_plan,
                                  options.local_planner.value_or(fd::LocalPlannerMode::lattice),controller);
        // The course and the rules it is judged by, stated before anything else since either starts the run over.
        if (course) simulation.set_course(course->cones,course->gates,"PacSim layout "+options.course_file->stem().string());
        if (options.discipline) {
            fd::CompetitionRules rules;
            rules.discipline=*options.discipline;
            simulation.set_competition(rules);
        }
        // Stated before recording starts, so the run directory carries the scenario it ran.
        simulation.set_obstructions(options.obstructions);
        // Fitted before recording starts too, so the instruments measure the run from its first tick.
        if (options.sensor_seed) {
            fd::SensorSettings sensors;
            sensors.seed=*options.sensor_seed;
            simulation.set_sensors(sensors);
        }
        if (options.perception_seed) {
            fd::PerceptionSettings perception;
            perception.seed=*options.perception_seed;
            simulation.set_perception(perception);
        }
        if (options.drive_cones) {
            if (!options.perception_seed) throw std::invalid_argument("--drive cones needs --perception SEED: the car drives on the cones it perceives");
            simulation.set_cone_driving(fd::ConePathSettings{});
        }
        fd::RecordingWriter writer(options.out,simulation,options.request,
                                   {FD_SOURCE_FINGERPRINT,FD_COMPILER_ID,FD_COMPILER_VERSION});
        std::size_t next_event=0;
        const auto& events=options.request.grip_events;
        // Wall-clock time of each step whose decision weighed alternatives: a measurement of this run on this host, not
        // part of the recording.
        std::vector<double> planning_ms, solve_ms;
        std::size_t decisions=0, by_controller=0, holding=0, refused=0;
        auto decisions_seen=simulation.decision_count();
        // A lap is done when the car has come round and, once the judge has started the clock, when the judge has timed it:
        // a course whose start line stands ahead of where the car is placed ends there. A DNF ends the run.
        const auto laps_left=[&] {
            const int wanted=options.request.requested_laps;
            return wanted==0 || simulation.diagnostics().laps<wanted || (simulation.judge().started() && simulation.judge().laps()<wanted);
        };
        while (simulation.state().time_s+options.config.fixed_dt_s <= options.request.duration_cap_s+1e-9 && laps_left() &&
               !simulation.judge().dnf()) {
            if (next_event<events.size() && simulation.state().time_s+1e-10>=events[next_event].time_s) {
                simulation.set_grip(events[next_event].grip_mu);
                ++next_event;
            }
            const auto began=std::chrono::steady_clock::now();
            simulation.step();
            const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
            if (simulation.decision_count()!=decisions_seen) {
                decisions_seen=simulation.decision_count();
                if (simulation.decision().options.size()>1) planning_ms.push_back(ms);
                const auto& outcome=simulation.controller_outcome();
                ++decisions;
                if (outcome.asked) solve_ms.push_back(outcome.solve_time_s*1000);
                if (outcome.driven_by!=fd::ControllerMode::policy) ++by_controller;
                else if (controller && !outcome.asked) ++holding;
                else if (controller) ++refused;
            }
            writer.record(simulation);
        }
        const auto summary=writer.finish(simulation);
        std::cout << std::fixed << std::setprecision(3)
                  << "Laps: " << summary.laps << "; simulated time: " << summary.elapsed_simulation_s
                  << " s; max tracking error: " << summary.max_cross_track_error_m << " m; max grip utilization: " << summary.max_combined_grip_utilization
                  << "; max rear sideslip: " << summary.max_rear_sideslip_rad << " rad; steering: "
                  << (options.steering==fd::SteeringMode::pure_pursuit ? "Pure Pursuit" : "MAP") << "; speed plan: ";
        if (const auto* envelope=simulation.performance_envelope())
            std::cout << "performance envelope " << envelope->fingerprint() << " at " << std::setprecision(2) << simulation.config().envelope_fraction
                      << std::setprecision(3);
        else std::cout << "grip fractions";
        std::cout << "; local planner: " << fd::local_planner_mode_name(simulation.local_planner_mode())
                  << "; controller: " << fd::controller_mode_name(simulation.controller_mode());
        std::cout << "; invalid samples: " << summary.invalid_samples << '/' << summary.samples << '\n';
        if (!planning_ms.empty()) {
            std::sort(planning_ms.begin(),planning_ms.end());
            const auto at=[&](double share){return planning_ms[std::min(planning_ms.size()-1,static_cast<std::size_t>(share*static_cast<double>(planning_ms.size())))];};
            std::cout << "Planning: " << planning_ms.size() << " steps weighed alternatives; median " << at(0.5) << " ms, 95th percentile "
                      << at(0.95) << " ms, longest " << planning_ms.back() << " ms against a " << options.config.control_dt_s*1000
                      << " ms control period (wall clock on this host, not recorded)\n";
        }
        if (!solve_ms.empty()) {
            std::sort(solve_ms.begin(),solve_ms.end());
            const auto at=[&](double share){return solve_ms[std::min(solve_ms.size()-1,static_cast<std::size_t>(share*static_cast<double>(solve_ms.size())))];};
            std::cout << "Controller: MPCC drove " << by_controller << " of " << decisions << " decisions; the policy drove " << holding
                      << " holding for a blockage and " << refused << " whose plan was not solved or not clear. Solving: median " << at(0.5)
                      << " ms, 95th percentile " << at(0.95) << " ms, longest " << solve_ms.back() << " ms against a "
                      << options.config.control_dt_s*1000 << " ms control period (wall clock on this host, not recorded)\n";
        }
        print_competition(simulation.judge().rules(),simulation.judge().events(),simulation.cones().size(),simulation.course_source());
        if (const auto* driver=simulation.cone_driver())
            std::cout << "Cones: drove on the path believed from its own simulated detections, from the "
                      << fd::belief_source_name(simulation.belief_source()) << " state; " << driver->paths_made()
                      << " paths believed; the known track judged the run and decided nothing\n";
        std::cout << "Outputs: " << std::filesystem::absolute(options.out).string() << '\n';
        return summary.completed?0:2;
    } catch (const std::exception& error) {
        std::cerr << "fd_headless: " << error.what() << '\n';
        return 1;
    }
}

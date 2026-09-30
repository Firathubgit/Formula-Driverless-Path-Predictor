#include "fd/steering.hpp"
#include "model_identity.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace fd {
namespace {

// Changing how tables are generated changes this, and with it every fingerprint.
constexpr const char* generator_version = "steady-state steering table 1";
// Steering grid spacing within each speed row, and speed spacing between rows.
constexpr double steering_step_rad = 0.01;
constexpr double speed_step_mps = 2.0;
// Rows continue this far above the configured top speed, for a car running over its plan.
constexpr double speed_margin_mps = 2.0;
// A steady turn is a fixed point of one control period of the plant at a held steering angle:
// lateral velocity, yaw rate and speed all return to themselves under the command that holds the
// speed. It is solved by Newton's method on that period, so it is exactly the simulated plant's turn.
// Residuals are rates of change; a turn is steady when every one is below steady_rate.
constexpr double steady_rate = 1e-9;
constexpr int newton_iterations = 30;
// A car with rotating wheels carries quasi-static wheel loads into its turn; the search is repeated with the loads the
// solved turn produces until they settle within a newton of the loads it was solved with.
constexpr int load_rounds = 8;
constexpr double load_tolerance_n = 1.0;
// The boundary of the usable steering range is refined by this many halvings of a grid step.
constexpr int boundary_halvings = 7;

// Everything a table depends on, as exact round-trip text.
std::string identity(const VehicleModel& model, const Config& c) {
    return std::string(generator_version)+";"+detail::model_identity(model, c);
}
std::string hash_hex(const std::string& text) { return fingerprint_hex(text); }

struct Settled {
    bool ok{};
    PlantState state;
    double command{};
};

// The unknowns of a steady turn: lateral velocity, yaw rate and the command that holds the speed. A car with rotating
// wheels adds its four wheel speeds, which are states of its own, so a turn is steady only when they too return to
// themselves (decision 0026). Its wheel loads are quasi-static: they are carried into the search and refreshed between
// solves, below.
constexpr int max_unknowns = 7;
int unknown_count(const VehicleModel& model) { return std::holds_alternative<FourWheelCar>(model) ? 7 : 3; }

// The state a turn's unknowns describe, at a held speed and steering, with the carried wheel loads.
PlantState turn_state(const PlantState& carried, double speed, double steering, const double* x, int n) {
    PlantState s;
    s.pose.speed_mps = speed;
    s.pose.steering_rad = steering;
    s.lateral_velocity_mps = x[0];
    s.yaw_rate_radps = x[1];
    if (n > 3) {
        for (std::size_t i = 0; i < s.wheel_speeds_radps.size(); ++i) s.wheel_speeds_radps[i] = x[3+static_cast<int>(i)];
        s.wheel_loads_n = carried.wheel_loads_n;
    }
    return s;
}

// One control period from such a turn: how fast each unknown state is still changing, the command's residual being the
// speed's.
void residual(const VehicleModel& model, const Config& config, const PlantState& carried, double speed, double steering,
              const double* x, int n, double* out) {
    const PlantState s = turn_state(carried, speed, steering, x, n);
    const double dt = config.control_dt_s;
    const auto next = advance(model, s, {x[2], steering}, config, dt);
    out[0] = (next.lateral_velocity_mps-s.lateral_velocity_mps)/dt;
    out[1] = (next.yaw_rate_radps-s.yaw_rate_radps)/dt;
    out[2] = (next.pose.speed_mps-speed)/dt;
    for (int i = 3; i < n; ++i) out[i] = (next.wheel_speeds_radps[static_cast<std::size_t>(i-3)]-x[i])/dt;
}

bool solve_linear(double m[max_unknowns][max_unknowns], double b[max_unknowns], int n) {
    for (int c = 0; c < n; ++c) {
        int pivot = c;
        for (int r = c+1; r < n; ++r) if (std::abs(m[r][c]) > std::abs(m[pivot][c])) pivot = r;
        if (!(std::abs(m[pivot][c]) > 1e-300)) return false;
        if (pivot != c) { for (int k = 0; k < n; ++k) std::swap(m[c][k], m[pivot][k]); std::swap(b[c], b[pivot]); }
        for (int r = c+1; r < n; ++r) {
            const double f = m[r][c]/m[c][c];
            for (int k = c; k < n; ++k) m[r][k] -= f*m[c][k];
            b[r] -= f*b[c];
        }
    }
    for (int c = n-1; c >= 0; --c) {
        for (int k = c+1; k < n; ++k) b[c] -= m[c][k]*b[k];
        b[c] /= m[c][c];
    }
    return true;
}

// Finds the steady turn at a speed and steering angle from a starting guess, and accepts it only if
// the simulated car would settle into it: the period map of lateral velocity and yaw rate must
// shrink every small disturbance, which for a two-by-two map means both eigenvalues inside the
// unit circle. An oversteering car past its limit has a steady turn it can never hold.
Settled settle(const VehicleModel& model, const Config& config, PlantState guess, double speed, double steering,
               double command) {
    const int n = unknown_count(model);
    double x[max_unknowns] = {guess.lateral_velocity_mps, guess.yaw_rate_radps, command};
    for (int i = 3; i < n; ++i) x[i] = guess.wheel_speeds_radps[static_cast<std::size_t>(i-3)];
    PlantState carried = guess;
    // Solve with the wheel loads carried, then take the loads the solved turn ends a period with and solve again, until
    // they stop moving: in a steady turn they are constant. A car without rotating wheels carries none and needs one round.
    for (int round = 0; round < (n > 3 ? load_rounds : 1); ++round) {
        bool solved = false;
        for (int iteration = 0; iteration < newton_iterations && !solved; ++iteration) {
            double base[max_unknowns];
            residual(model, config, carried, speed, steering, x, n, base);
            for (int i = 0; i < n; ++i) if (!std::isfinite(base[i])) return {};
            double m[max_unknowns][max_unknowns];
            for (int j = 0; j < n; ++j) {
                double probe[max_unknowns];
                for (int i = 0; i < n; ++i) probe[i] = x[i];
                const double h = 1e-7*std::max(1.0, std::abs(x[j]));
                probe[j] += h;
                double moved[max_unknowns];
                residual(model, config, carried, speed, steering, probe, n, moved);
                for (int i = 0; i < n; ++i) m[i][j] = (moved[i]-base[i])/h;
            }
            bool steady = true;
            for (int i = 0; i < n; ++i) steady = steady && std::abs(base[i]) < steady_rate;
            if (!steady) {
                double step[max_unknowns];
                for (int i = 0; i < n; ++i) step[i] = base[i];
                if (!solve_linear(m, step, n)) return {};
                for (int j = 0; j < n; ++j) x[j] -= step[j];
                continue;
            }
            // Period map of lateral velocity and yaw rate: identity plus the period times the Jacobian. A car's wheel
            // speeds are its fastest states and follow its chassis; the chassis decides whether a turn can be held.
            const double dt = config.control_dt_s;
            const double a = 1+dt*m[0][0], b = dt*m[0][1], c = dt*m[1][0], d = 1+dt*m[1][1];
            const double trace = a+d, determinant = a*d-b*c;
            if (!(std::abs(determinant) < 1 && std::abs(trace) < 1+determinant)) return {};
            solved = true;
        }
        if (!solved) return {};
        PlantState s = turn_state(carried, speed, steering, x, n);
        if (std::abs(rear_sideslip_rad(s)) > 0.5) return {};
        if (n == 3) return {true, s, x[2]};
        const auto next = advance(model, s, {x[2], steering}, config, config.control_dt_s);
        double moved = 0;
        for (std::size_t i = 0; i < s.wheel_loads_n.size(); ++i)
            moved = std::max(moved, std::abs(next.wheel_loads_n[i]-s.wheel_loads_n[i]));
        s.wheel_loads_n = next.wheel_loads_n;
        if (moved < load_tolerance_n) return {true, s, x[2]};
        carried = s;
    }
    return {};
}

std::vector<double> row_speeds(const VehicleModel& model, const Config& config) {
    double lowest = 1.0;
    if (const auto* car = std::get_if<DynamicSingleTrack>(&model)) lowest = std::max(0.5, car->kinematic_below_mps);
    if (const auto* car = std::get_if<FourWheelCar>(&model)) lowest = std::max(0.5, car->kinematic_below_mps);
    const double highest = config.max_speed_mps+speed_margin_mps;
    std::vector<double> speeds{lowest};
    for (double v = std::floor(lowest/speed_step_mps+1)*speed_step_mps; v < highest+speed_step_mps; v += speed_step_mps)
        speeds.push_back(v);
    return speeds;
}

// Sweeps steering upward at one speed. The row ends at the first angle that does not settle,
// leaves the tire envelope, or turns no tighter than the one before; the boundary between the last
// usable grid angle and that one is then found by halving.
SteeringRow sweep(const VehicleModel& model, const Config& config, double speed) {
    SteeringRow row;
    row.speed_mps = speed;
    const auto usable = [&](const Settled& settled, double previous_curvature) {
        if (!settled.ok) return false;
        const auto d = demand(model, settled.state, config, settled.command);
        return d.within_envelope && settled.state.yaw_rate_radps/speed > previous_curvature;
    };
    const auto turn = [&](const Settled& settled) {
        const double r = settled.state.yaw_rate_radps;
        return SteadyTurn{settled.state.pose.steering_rad, r/speed, r*speed};
    };
    Settled last = settle(model, config, plant_state_from({0, 0, 0, speed, 0, 0}, model, config), speed, 0, 0);
    if (!last.ok) throw std::runtime_error("Steering table: the plant does not settle driving straight");
    Settled before_last = last;
    row.turns.push_back({0, 0, 0});
    // Starts each probe from the steady turn extrapolated from the last two, so a probe only has to
    // settle the small remainder rather than the whole response to a steering step.
    const auto probe = [&](double steering) {
        Settled start = last;
        const double span = last.state.pose.steering_rad-before_last.state.pose.steering_rad;
        if (span > 0) {
            const double f = (steering-last.state.pose.steering_rad)/span;
            start.state.lateral_velocity_mps += f*(last.state.lateral_velocity_mps-before_last.state.lateral_velocity_mps);
            start.state.yaw_rate_radps += f*(last.state.yaw_rate_radps-before_last.state.yaw_rate_radps);
            start.command += f*(last.command-before_last.command);
        }
        return settle(model, config, start.state, speed, steering, start.command);
    };
    const auto accept = [&](const Settled& settled) {
        before_last = last;
        last = settled;
        row.turns.push_back(turn(settled));
    };
    const auto steps = static_cast<int>(std::ceil(config.max_steering_rad/steering_step_rad-1e-9));
    for (int j = 1; j <= steps; ++j) {
        const double steering = config.max_steering_rad*j/steps;
        const auto next = probe(steering);
        if (usable(next, row.turns.back().curvature_per_m)) { accept(next); continue; }
        double good = last.state.pose.steering_rad, bad = steering;
        for (int k = 0; k < boundary_halvings; ++k) {
            const double middle = (good+bad)/2;
            const auto inside = probe(middle);
            if (usable(inside, row.turns.back().curvature_per_m)) { accept(inside); good = middle; }
            else bad = middle;
        }
        break;
    }
    return row;
}

double row_steering(const SteeringRow& row, double curvature) {
    const auto& turns = row.turns;
    if (curvature >= turns.back().curvature_per_m) return turns.back().steering_rad;
    const auto upper = std::upper_bound(turns.begin(), turns.end(), curvature,
                                        [](double value, const SteadyTurn& t) { return value < t.curvature_per_m; });
    const auto& b = *upper;
    const auto& a = *(upper-1);
    const double f = (curvature-a.curvature_per_m)/(b.curvature_per_m-a.curvature_per_m);
    return a.steering_rad+f*(b.steering_rad-a.steering_rad);
}

} // namespace

const char* steering_mode_name(SteeringMode mode) {
    return mode == SteeringMode::pure_pursuit ? "pure_pursuit" : "map";
}

std::string steering_table_fingerprint(const VehicleModel& model, const Config& config) {
    return hash_hex(identity(model, config));
}

SteeringTable::SteeringTable(const VehicleModel& model, const Config& config)
    : wheelbase_m_(config.wheelbase_m), max_steering_rad_(config.max_steering_rad) {
    validate_vehicle(model, config);
    identity_ = identity(model, config);
    fingerprint_ = hash_hex(identity_);
    for (const double speed : row_speeds(model, config)) rows_.push_back(sweep(model, config, speed));
}

bool SteeringTable::generated_for(const VehicleModel& model, const Config& config) const {
    return identity(model, config) == identity_;
}

double SteeringTable::steering_for(double curvature, double speed) const {
    if (!std::isfinite(curvature) || !std::isfinite(speed) || speed < 0)
        throw std::invalid_argument("Steering lookup requires a finite curvature and a finite nonnegative speed");
    const double sign = curvature < 0 ? -1.0 : 1.0;
    const double k = std::abs(curvature);
    if (speed <= rows_.front().speed_mps)
        return sign*std::min(std::atan(wheelbase_m_*k), max_steering_rad_);
    if (speed >= rows_.back().speed_mps) return sign*row_steering(rows_.back(), k);
    const auto upper = std::upper_bound(rows_.begin(), rows_.end(), speed,
                                        [](double value, const SteeringRow& r) { return value < r.speed_mps; });
    const auto& b = *upper;
    const auto& a = *(upper-1);
    // Linear in speed squared: exact for a car whose steering grows with lateral acceleration.
    const double f = (speed*speed-a.speed_mps*a.speed_mps)/(b.speed_mps*b.speed_mps-a.speed_mps*a.speed_mps);
    return sign*((1-f)*row_steering(a, k)+f*row_steering(b, k));
}

} // namespace fd

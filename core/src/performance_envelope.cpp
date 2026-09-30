#include "fd/performance_envelope.hpp"
#include "model_identity.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <future>
#include <functional>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>

namespace fd {
namespace {

// Changing how envelopes are derived changes this, and with it every fingerprint.
constexpr const char* generator_version = "performance envelope 1";
constexpr double first_speed_mps = 4, speed_step_mps = 2, speed_margin_mps = 4;
// Steering is swept in this many steps up to its limit, and the boundary refined by this many halvings.
constexpr int sweep_steps = 24;
constexpr int lateral_halvings = 10;
// Longitudinal commands are bisected this many times between zero and the largest command probed.
constexpr int longitudinal_halvings = 14;
constexpr double largest_command_mps2 = 2.5*gravity_mps2;
// A turn is settled half a second at a time for up to six seconds, and judged on its last quarter second.
constexpr double settle_chunk_s = 0.5, settle_cap_s = 6.0, judged_window_s = 0.25;
// Steady: yaw rate spread within this share of the yaw rate (at least of 0.05 rad/s), speed within this share.
constexpr double steady_yaw_share = 2e-3, steady_speed_share = 0.01;
// Throttle loop holding the speed while a turn settles.
constexpr double speed_gain = 4, speed_integral_gain = 4;
// A longitudinal probe lasts this long from a settled turn, and its acceleration is measured over the second half,
// after the wheels have taken up their new slip: spinning the wheels up absorbs about a seventh of the power at first.
constexpr double probe_s = 0.1;
// A lateral level is steered onto within this share of its target by this many halvings of steering.
constexpr double level_share = 0.01;
constexpr int level_halvings = 12;

const FourWheelCar& four_wheel_car(const VehicleModel& model) {
    const auto* car = std::get_if<FourWheelCar>(&model);
    if (!car)
        throw std::invalid_argument("A performance envelope is derived for the four-wheel car only; the kinematic bicycle's limits are its stated "
                                    "grip fractions, and the single-track car clamps longitudinal force at its capacity");
    return *car;
}

std::string identity(const VehicleModel& model, const Config& config) {
    four_wheel_car(model);
    validate_vehicle(model, config);
    return std::string(generator_version)+";"+detail::model_identity(model, config);
}

enum class Verdict { steady, tire, settling };
struct Turn { Verdict verdict{Verdict::settling}; double steering{}, lateral{}; PlantState state; };

// Holds the speed with a throttle loop at a fixed steering angle, half a second at a time. After the first half
// second, a quarter second spent entirely beyond a tire's peak is a tire failure; a steady quarter second inside
// the envelope is a steady turn; running out of time is a settling failure.
Turn settle(const FourWheelCar& car, const Config& config, PlantState s, double speed, double steering) {
    const double dt = config.fixed_dt_s;
    const auto chunk = static_cast<int>(std::lround(settle_chunk_s/dt));
    const auto window = static_cast<int>(std::lround(judged_window_s/dt));
    const auto chunks = static_cast<int>(std::lround(settle_cap_s/settle_chunk_s));
    double integral = 0;
    Turn turn;
    turn.steering = steering;
    for (int c = 0; c < chunks; ++c) {
        double low = std::numeric_limits<double>::infinity(), high = -low;
        bool within = true, beyond = true;
        for (int i = 0; i < chunk; ++i) {
            const double error = speed-s.pose.speed_mps;
            integral += error*dt;
            const double command = speed_gain*error+speed_integral_gain*integral;
            s = advance(car, s, {command, steering}, config, dt);
            if (i >= chunk-window) {
                const bool inside = envelope(car, s, config, command).within;
                within = within && inside;
                beyond = beyond && !inside;
                low = std::min(low, s.yaw_rate_radps);
                high = std::max(high, s.yaw_rate_radps);
            }
        }
        turn.state = s;
        turn.lateral = std::abs(s.pose.speed_mps*s.yaw_rate_radps);
        if (c > 0 && beyond) { turn.verdict = Verdict::tire; return turn; }
        const bool steady = high-low < steady_yaw_share*std::max(0.05, std::abs(high)) &&
                            std::abs(s.pose.speed_mps-speed) < steady_speed_share*speed;
        if (steady) { turn.verdict = within ? Verdict::steady : Verdict::tire; return turn; }
    }
    return turn;
}

struct Capacity { double acceleration{}; EnvelopeLimit limit{EnvelopeLimit::tire}; };
// From a settled turn, the largest command of one sign whose next 0.1 s keeps every tire within its peak, reported
// as the acceleration the plant achieved over the second half of those 0.1 s.
Capacity longitudinal(const FourWheelCar& car, const Config& config, const Turn& turn, double sign) {
    const double dt = config.fixed_dt_s;
    const auto ticks = static_cast<int>(std::lround(probe_s/dt));
    const auto probe = [&](double command, double& achieved) {
        PlantState s = turn.state;
        bool within = true;
        double halfway = s.pose.speed_mps;
        for (int i = 0; i < ticks; ++i) {
            if (i == ticks/2) halfway = s.pose.speed_mps;
            s = advance(car, s, {command, turn.steering}, config, dt);
            within = within && envelope(car, s, config, command).within;
        }
        achieved = (s.pose.speed_mps-halfway)/((ticks-ticks/2)*dt);
        return within;
    };
    double achieved = 0;
    if (probe(sign*largest_command_mps2, achieved)) return {achieved, EnvelopeLimit::actuator};
    double lo = 0, hi = sign*largest_command_mps2, achieved_lo = 0;
    probe(0, achieved_lo);
    for (int k = 0; k < longitudinal_halvings; ++k) {
        const double mid = 0.5*(lo+hi);
        if (probe(mid, achieved)) { lo = mid; achieved_lo = achieved; }
        else hi = mid;
    }
    return {achieved_lo, EnvelopeLimit::tire};
}

EnvelopeLimit limit_of(Verdict verdict) { return verdict == Verdict::tire ? EnvelopeLimit::tire : EnvelopeLimit::settling; }

// One side of one speed. Returns false when the car cannot hold the speed straight ahead, which ends the grid.
bool derive_side(const FourWheelCar& car, const Config& config, double speed, double sign, EnvelopeSide& side) {
    const Turn straight = settle(car, config, plant_state_from({0, 0, 0, speed, 0, 0}, car, config), speed, 0);
    if (straight.verdict != Verdict::steady) return false;
    std::vector<Turn> turns{straight};
    Turn best = straight;
    bool steering_limited = true;
    double beyond = 0;
    Verdict failure = Verdict::settling;
    for (int k = 1; k <= sweep_steps; ++k) {
        const Turn t = settle(car, config, best.state, speed, sign*config.max_steering_rad*k/sweep_steps);
        if (t.verdict != Verdict::steady) { steering_limited = false; beyond = t.steering; failure = t.verdict; break; }
        turns.push_back(t);
        best = t;
    }
    if (!steering_limited) {
        double lo = best.steering, hi = beyond;
        for (int k = 0; k < lateral_halvings; ++k) {
            const double mid = 0.5*(lo+hi);
            const Turn t = settle(car, config, best.state, speed, mid);
            if (t.verdict == Verdict::steady) { lo = mid; best = t; turns.push_back(t); }
            else { hi = mid; failure = t.verdict; }
        }
    }
    side.lateral_limit_mps2 = best.lateral;
    side.lateral_limit = steering_limited ? EnvelopeLimit::steering : limit_of(failure);
    std::sort(turns.begin(), turns.end(), [](const Turn& a, const Turn& b) { return a.lateral < b.lateral; });
    side.levels.clear();
    for (const double fraction : envelope_lateral_fractions) {
        Turn level = fraction == 0 ? straight : best;
        if (fraction > 0 && fraction < 1) {
            // Bisect steering between the settled turns on either side of the target, settling each probe from the
            // turn below, until the settled lateral acceleration is within a percent of the target.
            const double target = fraction*best.lateral;
            auto above = std::find_if(turns.begin(), turns.end(), [&](const Turn& t) { return t.lateral >= target; });
            if (above == turns.end()) above = std::prev(turns.end());
            Turn below = above == turns.begin() ? *above : *std::prev(above);
            Turn upper = *above;
            level = std::abs(upper.lateral-target) < std::abs(below.lateral-target) ? upper : below;
            for (int k = 0; k < level_halvings && std::abs(level.lateral-target) > level_share*target; ++k) {
                const Turn t = settle(car, config, below.state, speed, 0.5*(below.steering+upper.steering));
                if (t.verdict != Verdict::steady) { upper = t; continue; }
                if (std::abs(t.lateral-target) < std::abs(level.lateral-target)) level = t;
                (t.lateral < target ? below : upper) = t;
            }
        }
        const auto forward = longitudinal(car, config, level, +1);
        const auto braking = longitudinal(car, config, level, -1);
        side.levels.push_back({level.lateral, forward.acceleration, braking.acceleration, forward.limit, braking.limit});
    }
    return true;
}

// Linear between grid rows, clamped to the grid.
double across_rows(const std::vector<EnvelopeRow>& rows, double speed, const std::function<double(const EnvelopeRow&)>& value) {
    if (rows.empty()) return 0;
    if (!(speed > rows.front().speed_mps)) return value(rows.front());
    if (!(speed < rows.back().speed_mps)) return value(rows.back());
    const auto upper = std::find_if(rows.begin(), rows.end(), [&](const EnvelopeRow& r) { return r.speed_mps >= speed; });
    const auto lower = std::prev(upper);
    const double w = (speed-lower->speed_mps)/(upper->speed_mps-lower->speed_mps);
    const double a = value(*lower), b = value(*upper);
    return a+w*(b-a);
}
// Linear between a side's levels; zero beyond its lateral limit.
double across_levels(const EnvelopeSide& side, double lateral, double EnvelopeLevel::* field) {
    if (side.levels.empty() || lateral > side.lateral_limit_mps2) return 0;
    if (!(lateral > side.levels.front().lateral_mps2)) return side.levels.front().*field;
    const auto upper = std::find_if(side.levels.begin(), side.levels.end(), [&](const EnvelopeLevel& l) { return l.lateral_mps2 >= lateral; });
    if (upper == side.levels.end()) return side.levels.back().*field;
    const auto lower = std::prev(upper);
    const double w = (lateral-lower->lateral_mps2)/(upper->lateral_mps2-lower->lateral_mps2);
    return (*lower).*field+w*((*upper).*field-(*lower).*field);
}

} // namespace

const char* speed_plan_mode_name(SpeedPlanMode mode) {
    switch (mode) {
    case SpeedPlanMode::grip_fractions: return "grip fractions";
    case SpeedPlanMode::performance_envelope: return "performance envelope";
    }
    return "unknown";
}

const char* envelope_limit_name(EnvelopeLimit limit) {
    switch (limit) {
    case EnvelopeLimit::tire: return "tire";
    case EnvelopeLimit::steering: return "steering";
    case EnvelopeLimit::actuator: return "actuator";
    case EnvelopeLimit::settling: return "settling";
    }
    return "unknown";
}

PerformanceEnvelope::PerformanceEnvelope(const VehicleModel& model, const Config& config) {
    identity_ = identity(model, config);
    fingerprint_ = fingerprint_hex(identity_);
    const auto& car = four_wheel_car(model);
    car_ = car;
    // Every speed and side is derived from its own copies of the plant, so they run in parallel; the grid still ends
    // at the first speed the car cannot hold straight ahead, exactly as deriving them in order would.
    struct Job { EnvelopeRow row; std::future<bool> left, right; };
    std::vector<Job> jobs;
    for (double speed = first_speed_mps; speed <= config.max_speed_mps+speed_margin_mps+1e-9; speed += speed_step_mps) {
        auto& job = jobs.emplace_back();
        job.row.speed_mps = speed;
    }
    for (auto& job : jobs) {
        job.left = std::async(std::launch::async, [&car, &config, &job] { return derive_side(car, config, job.row.speed_mps, +1, job.row.left); });
        job.right = std::async(std::launch::async, [&car, &config, &job] { return derive_side(car, config, job.row.speed_mps, -1, job.row.right); });
    }
    std::vector<char> held(jobs.size());
    for (std::size_t i = 0; i < jobs.size(); ++i) held[i] = static_cast<char>(jobs[i].left.get() && jobs[i].right.get());
    for (std::size_t i = 0; i < jobs.size() && held[i]; ++i) rows_.push_back(std::move(jobs[i].row));
    if (rows_.empty()) throw std::runtime_error("The car cannot hold the envelope's lowest speed straight ahead");
}

bool PerformanceEnvelope::generated_for(const VehicleModel& model, const Config& config) const {
    try { return identity(model, config) == identity_; }
    catch (const std::invalid_argument&) { return false; }
}
bool PerformanceEnvelope::generated_for(const Config& config) const { return generated_for(VehicleModel{car_}, config); }

double PerformanceEnvelope::lateral_limit(double speed, TurnSide side) const {
    return across_rows(rows_, speed, [&](const EnvelopeRow& r) { return (side == TurnSide::left ? r.left : r.right).lateral_limit_mps2; });
}
double PerformanceEnvelope::forward_limit(double speed, double lateral) const {
    return across_rows(rows_, speed, [&](const EnvelopeRow& r) {
        return across_levels(lateral >= 0 ? r.left : r.right, std::abs(lateral), &EnvelopeLevel::forward_mps2);
    });
}
double PerformanceEnvelope::braking_limit(double speed, double lateral) const {
    return across_rows(rows_, speed, [&](const EnvelopeRow& r) {
        return across_levels(lateral >= 0 ? r.left : r.right, std::abs(lateral), &EnvelopeLevel::braking_mps2);
    });
}

EnvelopeSide PerformanceEnvelope::side_at(double speed, TurnSide side) const {
    const auto of = [&](const EnvelopeRow& r) -> const EnvelopeSide& { return side == TurnSide::left ? r.left : r.right; };
    EnvelopeSide result;
    if (rows_.empty()) return result;
    // What limits a value is taken from the nearer row.
    const auto nearest = std::min_element(rows_.begin(), rows_.end(), [&](const EnvelopeRow& a, const EnvelopeRow& b) {
        return std::abs(a.speed_mps-speed) < std::abs(b.speed_mps-speed);
    });
    result = of(*nearest);
    result.lateral_limit_mps2 = across_rows(rows_, speed, [&](const EnvelopeRow& r) { return of(r).lateral_limit_mps2; });
    for (std::size_t k = 0; k < result.levels.size(); ++k) {
        auto& level = result.levels[k];
        level.lateral_mps2 = across_rows(rows_, speed, [&](const EnvelopeRow& r) { return of(r).levels[k].lateral_mps2; });
        level.forward_mps2 = across_rows(rows_, speed, [&](const EnvelopeRow& r) { return of(r).levels[k].forward_mps2; });
        level.braking_mps2 = across_rows(rows_, speed, [&](const EnvelopeRow& r) { return of(r).levels[k].braking_mps2; });
    }
    return result;
}

std::string performance_envelope_fingerprint(const VehicleModel& model, const Config& config) {
    return fingerprint_hex(identity(model, config));
}

void write_performance_envelope(const PerformanceEnvelope& envelope, const VehicleModel& model, const Config& config,
                                const std::filesystem::path& directory) {
    if (!envelope.generated_for(model, config))
        throw std::invalid_argument("The performance envelope was not derived for this car and configuration");
    const auto& car = four_wheel_car(model);
    std::filesystem::create_directories(directory);
    const auto open = [&](const char* name, const char* what) {
        std::ofstream out(directory/name);
        if (!out) throw std::runtime_error(std::string("Cannot create ")+(directory/name).string());
        out << std::setprecision(12);
        out << "# Formula-Driverless-Path-Predictor performance envelope " << envelope.fingerprint() << ": " << vehicle_model_name(model)
            << ", grip_mu " << config.grip_mu << "\n# " << what << "\n";
        return out;
    };
    const auto drag = [&](double speed) { return 0.5*air_density_kgpm3*car.drag_area_m2*speed*speed/car.mass_kg; };
    auto ggv = open("ggv.csv", "TUM ggv format. ax_max: straight-line braking limit less drag deceleration. ay_max: the smaller side's lateral limit.");
    ggv << "# v_mps,ax_max_mps2,ay_max_mps2\n";
    auto machines = open("ax_max_machines.csv", "TUM ax_max_machines format: straight-line forward limit plus drag deceleration.");
    machines << "# v_mps,ax_max_machines_mps2\n";
    auto table = open("envelope.csv", "Every speed, side and lateral level, drag and wheel inertia included, with what limits each value.");
    table << "# speed_mps,side,lateral_fraction,lateral_mps2,forward_mps2,braking_mps2,lateral_limit,forward_limit,braking_limit\n";
    for (const auto& row : envelope.rows()) {
        const auto& straight = row.left.levels.front();
        ggv << row.speed_mps << ',' << -straight.braking_mps2-drag(row.speed_mps) << ','
            << std::min(row.left.lateral_limit_mps2, row.right.lateral_limit_mps2) << '\n';
        machines << row.speed_mps << ',' << straight.forward_mps2+drag(row.speed_mps) << '\n';
        for (const auto& [name, side] : {std::pair<const char*, const EnvelopeSide*>{"left", &row.left}, {"right", &row.right}})
            for (std::size_t k = 0; k < side->levels.size(); ++k) {
                const auto& level = side->levels[k];
                table << row.speed_mps << ',' << name << ',' << envelope_lateral_fractions[k] << ',' << level.lateral_mps2 << ','
                      << level.forward_mps2 << ',' << level.braking_mps2 << ',' << envelope_limit_name(side->lateral_limit) << ','
                      << envelope_limit_name(level.forward_limit) << ',' << envelope_limit_name(level.braking_limit) << '\n';
            }
    }
}

} // namespace fd

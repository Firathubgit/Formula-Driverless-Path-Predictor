#include "fd/predictive_control.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fd {

std::vector<double> point_times(const LocalTrajectory& trajectory) {
    std::vector<double> times;
    times.reserve(trajectory.points.size());
    for (const auto& point : trajectory.points) times.push_back(point.state.time_s);
    return times;
}

namespace {
void check_commands(const std::vector<double>& times, const std::vector<Command>& commands) {
    if (times.empty() || times.size() != commands.size()) throw std::invalid_argument("A plan needs one command at each of its points");
    for (std::size_t k = 0; k < times.size(); ++k) {
        if (!std::isfinite(times[k]) || !std::isfinite(commands[k].acceleration_mps2) || !std::isfinite(commands[k].steering_rad))
            throw std::invalid_argument("A plan's times and commands must be finite");
        if (k > 0 && !(times[k] > times[k-1])) throw std::invalid_argument("A plan's times must ascend");
    }
}
} // namespace

Command planned_command(const std::vector<double>& times, const std::vector<Command>& commands, double time_s) {
    check_commands(times, commands);
    if (!std::isfinite(time_s)) throw std::invalid_argument("A planned command is asked for at a finite time");
    if (time_s <= times.front()) return commands.front();
    if (time_s >= times.back()) return commands.back();
    const auto later = static_cast<std::size_t>(std::upper_bound(times.begin(), times.end(), time_s)-times.begin());
    const auto& a = commands[later-1];
    const auto& b = commands[later];
    const double f = (time_s-times[later-1])/(times[later]-times[later-1]);
    return {a.acceleration_mps2+f*(b.acceleration_mps2-a.acceleration_mps2), a.steering_rad+f*(b.steering_rad-a.steering_rad)};
}

std::vector<PlantState> plant_response(const VehicleModel& model, const PlantState& start, const Config& config,
                                       const std::vector<double>& times, const std::vector<Command>& commands,
                                       std::uint64_t ticks_to_next_control) {
    validate_config(config);
    validate_vehicle(model, config);
    check_commands(times, commands);
    const double dt = config.fixed_dt_s;
    const auto control_ticks = static_cast<std::uint64_t>(std::llround(config.control_dt_s/dt));
    if (ticks_to_next_control > control_ticks) throw std::invalid_argument("The first control interval is at most a control period");
    const std::uint64_t first = ticks_to_next_control ? ticks_to_next_control : control_ticks;
    const double t0 = start.pose.time_s;
    // Each plan time as a whole number of ticks from the start.
    std::vector<std::uint64_t> at;
    at.reserve(times.size());
    for (const double t : times) {
        const double ticks = (t-t0)/dt;
        if (ticks < -1e-6 || std::abs(ticks-std::round(ticks)) > 1e-6)
            throw std::invalid_argument("A plan's times must lie on the fixed-tick grid from the state it was made in");
        at.push_back(static_cast<std::uint64_t>(std::llround(ticks)));
    }
    if (at.front() != 0) throw std::invalid_argument("A plan starts at the state it was made in");
    std::vector<PlantState> response{start};
    response.reserve(times.size());
    PlantState state = start;
    Command held;
    std::uint64_t interval_end = 0;
    for (std::uint64_t tick = 0, next = 1; next < at.size(); ++tick) {
        // A command is chosen at the start of each control interval, as where the plan's commands are at its end.
        if (tick == interval_end) {
            interval_end = tick == 0 ? first : interval_end+control_ticks;
            held = planned_command(times, commands, t0+static_cast<double>(interval_end)*dt);
        }
        state = advance(model, state, held, config, dt);
        state.pose.time_s = t0+static_cast<double>(tick+1)*dt;
        while (next < at.size() && at[next] == tick+1) { response.push_back(state); ++next; }
    }
    return response;
}

std::string_view controller_mode_name(ControllerMode mode) {
    switch (mode) {
    case ControllerMode::policy: return "policy";
    case ControllerMode::mpcc: return "mpcc";
    case ControllerMode::human: return "human";
    }
    return "unknown";
}

std::string_view controller_status_name(ControllerStatus status) {
    switch (status) {
    case ControllerStatus::solved: return "solved";
    case ControllerStatus::solved_inaccurate: return "solved inaccurate";
    case ControllerStatus::not_solved: return "not solved";
    }
    return "unknown";
}


std::vector<PlanTireUse> plan_tire_use(const VehicleModel& model, const PlantState& start, const Config& config,
                                       const std::vector<double>& times, const std::vector<Command>& commands,
                                       std::uint64_t ticks_to_next_control) {
    const auto response = plant_response(model, start, config, times, commands, ticks_to_next_control);
    std::vector<PlanTireUse> use;
    if (!std::holds_alternative<FourWheelCar>(model)) return use;
    use.reserve(response.size());
    for (std::size_t k = 0; k < response.size(); ++k) {
        // What each wheel uses comes from the plant's own state there: its wheel speeds, loads and slips under the
        // commands that drove it. The acceleration demand() is asked for names what the car achieved over the step,
        // which no wheel's own use depends on.
        const double achieved = k+1 < response.size() ? (response[k+1].pose.speed_mps-response[k].pose.speed_mps)/(times[k+1]-times[k]) : 0.0;
        const auto asked = demand(model, response[k], config, achieved);
        PlanTireUse point{times[k], {}};
        for (std::size_t w = 0; w < point.use.size(); ++w)
            point.use[w] = std::hypot(asked.wheel_longitudinal_use[w], asked.wheel_lateral_use[w]);
        use.push_back(point);
    }
    return use;
}

} // namespace fd

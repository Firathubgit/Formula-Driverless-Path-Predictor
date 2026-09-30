#include "fd/path_following.hpp"
#include "fd/performance_envelope.hpp"
#include "speed_limits.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fd {
namespace {

double step_length(const OpenPath& path, std::size_t i) { return path.points[i+1].s_m-path.points[i].s_m; }

} // namespace

void validate_open_path(const OpenPath& path) {
    const auto& p = path.points;
    if (p.size() < 2) throw std::invalid_argument("An open path needs at least two samples");
    if (!std::isfinite(path.width_m) || path.width_m <= 0) throw std::invalid_argument("An open path's width must be finite and positive");
    if (p.front().s_m != 0) throw std::invalid_argument("An open path's first station must be zero");
    for (std::size_t i = 0; i < p.size(); ++i) {
        if (!std::isfinite(p[i].x_m) || !std::isfinite(p[i].y_m) || !std::isfinite(p[i].s_m) || !std::isfinite(p[i].curvature))
            throw std::invalid_argument("An open path contains non-finite values");
        if (i == 0) continue;
        const double step = p[i].s_m-p[i-1].s_m;
        const double chord = std::hypot(p[i].x_m-p[i-1].x_m, p[i].y_m-p[i-1].y_m);
        if (!(step > 1e-8) || chord <= 1e-8) throw std::invalid_argument("An open path's stations must increase between distinct samples");
        if (chord > step*(1+1e-9)+1e-12) throw std::invalid_argument("An open path's station step is shorter than the chord it spans");
    }
}

std::vector<PlanPoint> plan_open_path(const OpenPath& path, const Config& c, const PerformanceEnvelope* envelope) {
    validate_config(c);
    validate_open_path(path);
    if (envelope && !envelope->generated_for(c))
        throw std::invalid_argument("The performance envelope was derived under another configuration; derive it for this one");
    const auto& points = path.points;
    const std::size_t n = points.size();
    const double lateral = c.grip_mu*gravity_mps2*c.lateral_grip_fraction;
    const double longitudinal = c.grip_mu*gravity_mps2*c.longitudinal_grip_fraction;
    const double share = c.envelope_fraction;
    const auto forward = [&](std::size_t i, double v) {
        return envelope ? share*std::max(0.0, envelope->forward_limit(v, v*v*points[i].curvature/share)) : longitudinal;
    };
    const auto braking = [&](std::size_t i, double v) {
        return envelope ? share*std::max(0.0, -envelope->braking_limit(v, v*v*points[i].curvature/share)) : longitudinal;
    };
    std::vector<PlanPoint> plan(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double k = std::abs(points[i].curvature);
        const double curve_limit = envelope ? detail::envelope_curvature_speed(*envelope, share, points[i].curvature, c.max_speed_mps)
                                            : k > 1e-9 ? std::sqrt(lateral/k) : c.max_speed_mps;
        plan[i] = {std::min(c.max_speed_mps, curve_limit), 0, i, curve_limit < c.max_speed_mps ? "curvature_limit" : "speed_cap"};
    }
    // Nothing is known beyond the last sample, so the car is to be at rest there.
    plan.back().speed_mps = 0;
    plan.back().reason = "end_of_believed_path";
    // One forward and one backward pass settle an open path: nothing wraps round to be relaxed again. The forward pass
    // uses the capacity at each sample's speed, which only grows along it, so it runs before braking lowers any speed.
    for (std::size_t i = 0; i+1 < n; ++i) {
        const double limit = std::sqrt(plan[i].speed_mps*plan[i].speed_mps+2*forward(i, plan[i].speed_mps)*step_length(path, i));
        if (plan[i+1].speed_mps > limit+1e-10) {
            plan[i+1].speed_mps = limit;
            plan[i+1].limiting_index = plan[i].limiting_index;
            plan[i+1].reason = "acceleration_reachability";
        }
    }
    for (std::size_t j = n-1; j > 0; --j) {
        const std::size_t i = j-1;
        const double limit = std::sqrt(plan[j].speed_mps*plan[j].speed_mps+2*braking(j, plan[j].speed_mps)*step_length(path, i));
        if (plan[i].speed_mps > limit+1e-10) {
            plan[i].speed_mps = limit;
            plan[i].limiting_index = plan[j].limiting_index;
            plan[i].reason = "braking_reachability";
        }
    }
    for (std::size_t i = 0; i+1 < n; ++i)
        plan[i].acceleration_mps2 = (plan[i+1].speed_mps*plan[i+1].speed_mps-plan[i].speed_mps*plan[i].speed_mps)/(2*step_length(path, i));
    return plan;
}

Projection project_open(const OpenPath& path, Vec2 position) {
    const auto& points = path.points;
    Projection result;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i+1 < points.size(); ++i) {
        const auto& a = points[i];
        const auto& b = points[i+1];
        const double ex = b.x_m-a.x_m, ey = b.y_m-a.y_m;
        // Before the first segment and beyond the last the path continues straight, so the fraction runs on there.
        double fraction = ((position.x-a.x_m)*ex+(position.y-a.y_m)*ey)/(ex*ex+ey*ey);
        const double lower = i == 0 ? -std::numeric_limits<double>::infinity() : 0.0;
        const double upper = i+2 == points.size() ? std::numeric_limits<double>::infinity() : 1.0;
        fraction = std::clamp(fraction, lower, upper);
        const Vec2 p{a.x_m+fraction*ex, a.y_m+fraction*ey};
        const double distance = std::hypot(position.x-p.x, position.y-p.y);
        if (distance < best) {
            best = distance;
            result.index = i;
            result.fraction = fraction;
            result.point = p;
            result.s_m = a.s_m+fraction*(b.s_m-a.s_m);
            result.signed_error_m = (ex*(position.y-p.y)-ey*(position.x-p.x))/std::hypot(ex, ey);
            result.distance_m = distance;
        }
    }
    return result;
}

namespace {

// The point at a station of the path, which before its start and beyond its end continues along its end segments.
Vec2 point_at(const OpenPath& path, double s) {
    const auto& points = path.points;
    const auto upper = std::upper_bound(points.begin(), points.end(), s, [](double value, const PathPoint& p) { return value < p.s_m; });
    std::size_t i = upper == points.begin() ? 0 : static_cast<std::size_t>(upper-points.begin()-1);
    i = std::min(i, points.size()-2);
    const auto& a = points[i];
    const auto& b = points[i+1];
    const double f = (s-a.s_m)/(b.s_m-a.s_m);
    return {a.x_m+f*(b.x_m-a.x_m), a.y_m+f*(b.y_m-a.y_m)};
}

} // namespace

ControlResult follow_open_path(const OpenPath& path, const std::vector<PlanPoint>& plan, const State& state, const Config& config,
                               const SteeringTable* steering, const PerformanceEnvelope* envelope) {
    if (!std::isfinite(state.x_m) || !std::isfinite(state.y_m) || !std::isfinite(state.yaw_rad) || !std::isfinite(state.speed_mps) ||
        state.speed_mps < 0 || !std::isfinite(state.steering_rad))
        throw std::invalid_argument("Following a path requires a finite nonnegative-speed state");
    if (plan.size() != path.points.size()) throw std::invalid_argument("Following a path requires a plan with one point per sample");
    for (const auto& point : plan)
        if (!std::isfinite(point.speed_mps) || point.speed_mps < 0 || !std::isfinite(point.acceleration_mps2))
            throw std::invalid_argument("Following a path requires finite nonnegative plan speeds and finite accelerations");
    const auto nearest = project_open(path, {state.x_m, state.y_m});
    const std::size_t i = nearest.index, j = i+1;
    // Before the path the first sample's speed governs, beyond it the last's, which is rest.
    const double f = std::clamp(nearest.fraction, 0.0, 1.0);
    const double target = std::sqrt(std::max(0.0, (1-f)*plan[i].speed_mps*plan[i].speed_mps+f*plan[j].speed_mps*plan[j].speed_mps));
    const double feedforward = nearest.fraction > 1 ? 0.0 : plan[i].acceleration_mps2;
    const double lookahead = config.lookahead_base_m+config.lookahead_time_s*state.speed_mps;
    const auto aim = point_at(path, nearest.s_m+lookahead);
    const double dx = aim.x-state.x_m, dy = aim.y-state.y_m;
    const double local_y = -std::sin(state.yaw_rad)*dx+std::cos(state.yaw_rad)*dy;
    const double geometric_steering = std::atan2(2*config.wheelbase_m*local_y, dx*dx+dy*dy);
    const double distance_sq = dx*dx+dy*dy;
    const double requested_steering = steering ? steering->steering_for(distance_sq > 0 ? 2*local_y/distance_sq : 0.0, state.speed_mps)
                                               : geometric_steering;
    const double requested_acceleration = feedforward+config.speed_gain*(target-state.speed_mps);
    const double budget = config.longitudinal_grip_fraction*config.grip_mu*gravity_mps2;
    const double max_acceleration = envelope ? std::max(0.0, envelope->forward_limit(state.speed_mps, state.speed_mps*state.speed_mps*path.points[i].curvature))
                                             : budget;
    const double max_braking = envelope ? std::max(0.0, -envelope->braking_limit(state.speed_mps, 0)) : budget;
    return {{requested_acceleration, requested_steering},
            {std::clamp(requested_acceleration, -max_braking, max_acceleration),
             std::clamp(requested_steering, -config.max_steering_rad, config.max_steering_rad)},
            lookahead, target, nearest.distance_m, geometric_steering, nearest};
}

LocalTrajectory predict_open_path(const OpenPath& path, const std::vector<PlanPoint>& plan, const PlantState& initial_plant,
                                  const Config& config, const VehicleModel& model, std::uint64_t ticks_to_next_control,
                                  const SteeringTable* steering, const PerformanceEnvelope* performance) {
    validate_vehicle(model, config);
    validate_open_path(path);
    if (steering && !steering->generated_for(model, config))
        throw std::invalid_argument("Steering table was generated for a different vehicle model or configuration; regenerate it");
    if (performance && !performance->generated_for(model, config))
        throw std::invalid_argument("Performance envelope was derived for a different vehicle model or configuration; derive it again");
    const auto control_ticks = static_cast<std::uint64_t>(std::llround(config.control_dt_s/config.fixed_dt_s));
    if (ticks_to_next_control > control_ticks) throw std::invalid_argument("Prediction first hold must end by the next control tick");
    if (ticks_to_next_control == 0) ticks_to_next_control = control_ticks;
    const auto horizon_ticks = static_cast<std::uint64_t>(std::ceil(prediction_horizon_s/config.fixed_dt_s-1e-10));
    const State& initial = initial_plant.pose;
    LocalTrajectory result;
    result.validity_reason = "Prediction within the believed corridor and the checked envelope";
    result.first_control = follow_open_path(path, plan, initial, config, steering, performance);
    PlantState plant = initial_plant;
    const State& predicted = plant.pose;
    auto control = result.first_control;
    std::uint64_t tick = 0;
    double distance = 0;
    const auto invalidate = [&](const char* reason) {
        if (result.within_model_envelope) result.validity_reason = reason;
        result.within_model_envelope = false;
    };
    const auto check_position_and_speed = [&] {
        if (control.reference.distance_m > std::max(0.0, path.width_m/2-0.9)) invalidate("Predicted rear axle leaves the believed corridor");
        if (predicted.speed_mps > control.target_speed_mps+0.75) invalidate("Predicted speed exceeds the believed path's plan");
    };
    check_position_and_speed();
    while (tick < horizon_ticks) {
        result.points.push_back({predicted, 0, distance});
        const auto hold_ticks = std::min(tick == 0 ? ticks_to_next_control : control_ticks, horizon_ticks-tick);
        const double start_speed = predicted.speed_mps;
        for (std::uint64_t k = 0; k < hold_ticks; ++k) {
            const auto before = plant;
            plant = advance(model, plant, control.applied, config, config.fixed_dt_s);
            ++tick;
            plant.pose.time_s = initial.time_s+static_cast<double>(tick)*config.fixed_dt_s;
            distance += std::hypot(predicted.x_m-before.pose.x_m, predicted.y_m-before.pose.y_m);
            const double longitudinal = (predicted.speed_mps-before.pose.speed_mps)/config.fixed_dt_s;
            for (const PlantState* at : std::array<const PlantState*, 2>{&before, &plant}) {
                const auto e = envelope(model, *at, config, longitudinal);
                if (!e.within)
                    invalidate(e.tire_slip_modeled ? "Predicted tire slip passes its peak; the car would slide"
                                                   : "Predicted grip demand exceeds model envelope; tire slip is not modeled");
            }
        }
        result.points.back().acceleration_mps2 = (predicted.speed_mps-start_speed)/(static_cast<double>(hold_ticks)*config.fixed_dt_s);
        control = follow_open_path(path, plan, predicted, config, steering, performance);
        check_position_and_speed();
    }
    result.points.push_back({predicted, 0, distance});
    return result;
}

} // namespace fd

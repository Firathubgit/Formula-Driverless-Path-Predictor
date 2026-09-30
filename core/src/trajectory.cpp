#include "fd/trajectory.hpp"
#include "fd/performance_envelope.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace fd {

LocalTrajectory make_local_trajectory(const Track& track, const std::vector<PlanPoint>& reference,
                                      const State& initial, const Config& config,
                                      std::uint64_t ticks_to_next_control, ControlIntent intent) {
    const VehicleModel kinematic = KinematicBicycle{};
    return make_local_trajectory(track, reference, plant_state_from(initial, kinematic, config), config, kinematic,
                                 ticks_to_next_control, intent);
}

LocalTrajectory make_local_trajectory(const Track& track, const std::vector<PlanPoint>& reference,
                                      const PlantState& initial_plant, const Config& config,
                                      const VehicleModel& model,
                                      std::uint64_t ticks_to_next_control, ControlIntent intent,
                                      const SteeringTable* steering, const PerformanceEnvelope* performance) {
    validate_vehicle(model, config);
    if (steering && !steering->generated_for(model, config))
        throw std::invalid_argument("Steering table was generated for a different vehicle model or configuration; regenerate it");
    if (performance && !performance->generated_for(model, config))
        throw std::invalid_argument("Performance envelope was derived for a different vehicle model or configuration; derive it again");
    const State& initial = initial_plant.pose;
    validate_track(track);
    const auto control_ticks = static_cast<std::uint64_t>(std::llround(config.control_dt_s/config.fixed_dt_s));
    if (ticks_to_next_control > control_ticks)
        throw std::invalid_argument("Prediction first hold must end by the next control tick");
    if (ticks_to_next_control == 0) ticks_to_next_control = control_ticks;
    const auto horizon_ticks = static_cast<std::uint64_t>(std::ceil(prediction_horizon_s/config.fixed_dt_s-1e-10));

    LocalTrajectory result;
    // Also validates the initial state, every reference-plan entry and the intent.
    result.first_control = compute_control(track, reference, initial, config, intent, steering, performance);
    result.points.reserve(static_cast<std::size_t>(horizon_ticks/control_ticks+3));
    PlantState plant = initial_plant;
    const State& predicted = plant.pose;
    auto control = result.first_control;
    std::uint64_t tick = 0;
    double distance = 0;
    auto invalidate = [&](const char* reason) {
        if (result.within_model_envelope) result.validity_reason = reason;
        result.within_model_envelope = false;
    };
    auto check_position_and_speed = [&] {
        // Control already projected this same predicted state onto the reference.
        if (!within_corridor(track, control.reference, 0.9))
            invalidate("Predicted rear axle exceeds track margin");
        if (predicted.speed_mps > control.target_speed_mps+0.75)
            invalidate("Predicted speed exceeds reference envelope");
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
            // Check both ends of each plant tick, including an initially infeasible
            // steering/speed state; clipping the command cannot remove its momentum.
            for (const PlantState* at : std::array<const PlantState*, 2>{&before, &plant}) {
                const auto e = envelope(model, *at, config, longitudinal);
                if (!e.within)
                    invalidate(e.tire_slip_modeled ? "Predicted tire slip passes its peak; the car would slide"
                                                   : "Predicted grip demand exceeds model envelope; tire slip is not modeled");
            }
        }
        result.points.back().acceleration_mps2 =
            (predicted.speed_mps-start_speed)/(static_cast<double>(hold_ticks)*config.fixed_dt_s);
        // Corridor and speed-envelope checks at every control point and endpoint.
        // This is a sampled rear-axle check, not continuous swept-body collision.
        control = compute_control(track, reference, predicted, config, intent, steering, performance);
        check_position_and_speed();
    }
    result.points.push_back({predicted, 0, distance});
    return result;
}

} // namespace fd

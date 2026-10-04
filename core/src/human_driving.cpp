#include "fd/human_driving.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

namespace fd {
namespace {

void within_travel(double value, double low, double high, const char* name) {
    if (!std::isfinite(value) || value < low || value > high)
        throw std::invalid_argument(std::string(name)+" must be a finite share of its travel, from "+std::to_string(low)+
                                    " to "+std::to_string(high));
}

// Downforce at the car's speed over its weight; zero for a model without wings.
double downforce_share(const PlantState& state, const VehicleModel& model) {
    if (const auto* car=std::get_if<FourWheelCar>(&model))
        return aerodynamic_force(*car,state.pose.speed_mps,state.lateral_velocity_mps).downforce_n/(car->mass_kg*gravity_mps2);
    if (const auto* car=std::get_if<DynamicSingleTrack>(&model))
        return 0.5*air_density_kgpm3*car->downforce_area_m2*state.pose.speed_mps*state.pose.speed_mps/(car->mass_kg*gravity_mps2);
    return 0;
}

// The slip angle at which the front tires reach their peak grip, at whichever load puts it latest; zero without tires.
double front_peak_slip_angle_rad(const VehicleModel& model) {
    if (const auto* car=std::get_if<FourWheelCar>(&model))
        return std::max(car->front_tire.peak_slip_angle_low_rad,car->front_tire.peak_slip_angle_high_rad);
    if (const auto* car=std::get_if<DynamicSingleTrack>(&model))
        return std::max(car->front_tire.peak_slip_angle_low_rad,car->front_tire.peak_slip_angle_high_rad);
    return 0;
}

} // namespace

void validate_driver_controls(const DriverControls& controls) {
    within_travel(controls.throttle,0,1,"throttle");
    within_travel(controls.brake,0,1,"brake");
    within_travel(controls.steering,-1,1,"steering");
}

double human_pedal_acceleration_mps2(const PlantState& state, const VehicleModel& model, const Config& config) {
    return config.grip_mu*gravity_mps2*(1+downforce_share(state,model));
}

double human_steering_limit_rad(const PlantState& state, const VehicleModel& model, const Config& config, const GripAid* aid) {
    const double speed=state.pose.speed_mps;
    double lateral=human_pedal_acceleration_mps2(state,model,config);
    double slip_share=1;
    if (aid && aid->envelope) {
        if (!(aid->share>0 && aid->share<=1)) throw std::invalid_argument("The steering aid's share must lie within (0, 1]");
        // What the plant itself holds at this speed, either way round, and the share of it a full stick may ask.
        const double held=std::min(aid->envelope->lateral_limit(speed,TurnSide::left),aid->envelope->lateral_limit(speed,TurnSide::right));
        lateral=aid->share*held;
        slip_share=aid->share;
    }
    // Standing still any turn asks for no lateral acceleration, so the lock is the limit.
    const double geometric=speed>0 ? std::atan(config.wheelbase_m*lateral/(speed*speed)) : std::numbers::pi/2;
    const double aided=std::min(config.max_steering_rad,geometric+slip_share*front_peak_slip_angle_rad(model));
    // The aid only ever takes steering away: where the car holds more than the plain limit assumes, the plain limit stands.
    if (aid && aid->envelope) return std::min(aided,std::min(config.max_steering_rad,
        (speed>0 ? std::atan(config.wheelbase_m*human_pedal_acceleration_mps2(state,model,config)/(speed*speed)) : std::numbers::pi/2)+front_peak_slip_angle_rad(model)));
    return aided;
}

double assisted_speed_limit_mps(const VehicleModel& model, const Config& config, DrivingAssistance assistance) {
    if (!driving_assistance_active(assistance)) return std::numeric_limits<double>::infinity();
    const auto* car=std::get_if<FourWheelCar>(&model);
    if (!car) throw std::invalid_argument("Forgiveness requires a four-wheel car");
    const double normal=std::isfinite(car->max_drive_speed_mps)?car->max_drive_speed_mps:config.max_speed_mps;
    if (assistance.manual) return std::min(normal,std::lerp(10.0,60.0,(assistance.speed-1)/99)/3.6);
    const double t=(assistance.level-1)/99;
    return std::lerp(normal,std::min(normal,35/3.6),t);
}

Command human_command(const DriverControls& controls, const PlantState& state, const VehicleModel& model, const Config& config,
                      DrivingAssistance assistance, const GripAid* aid) {
    validate_driver_controls(controls);
    const double help=driving_assistance_strength(assistance);
    const double shaped=std::copysign(std::pow(std::abs(controls.steering),1.5),controls.steering);
    double limit=human_steering_limit_rad(state,model,config,aid);
    if (aid && aid->envelope) {
        if (!std::isfinite(aid->coasting_steering_floor_rad) || aid->coasting_steering_floor_rad<0 ||
            aid->coasting_steering_floor_rad>config.max_steering_rad)
            throw std::invalid_argument("Coasting steering allowance must be finite and within physical lock");
        const double pedal=std::clamp(std::max(controls.throttle,controls.brake)/0.25,0.0,1.0);
        const double blend=pedal*pedal*(3-2*pedal);
        limit=std::lerp(std::max(limit,aid->coasting_steering_floor_rad),limit,blend);
    }
    Command command{(controls.throttle-controls.brake)*human_pedal_acceleration_mps2(state,model,config),
                    shaped*limit};
    if (aid && aid->envelope) {
        if (!std::isfinite(aid->steering_gain) || aid->steering_gain <= 0)
            throw std::invalid_argument("The steering gain must be finite and positive");
        // Gain changes thumbstick sensitivity, never the speed-dependent limit.
        command.steering_rad=std::clamp(command.steering_rad*aid->steering_gain,-limit,limit);
        // What is left of the friction circle beside the cornering already under way, the corner given 1 - share of its limit.
        const double speed=state.pose.speed_mps;
        const double lateral=speed*state.yaw_rate_radps/aid->share;
        if (command.acceleration_mps2>0) {
            command.acceleration_mps2=std::min(command.acceleration_mps2,aid->envelope->forward_limit(speed,lateral));
        } else if (command.acceleration_mps2<0) {
            // Braking is never taken away altogether: a driver already past the limit still brakes, at least a third as hard.
            const double most=std::max(-aid->envelope->braking_limit(speed,lateral),-aid->envelope->braking_limit(speed,0)/3);
            command.acceleration_mps2=std::max(command.acceleration_mps2,-most);
        }
    }
    if (!driving_assistance_active(assistance)) return command; // Without Forgiveness, only the aid, if given, differs.
    const double cap=assisted_speed_limit_mps(model,config,assistance);
    const double speed=state.pose.speed_mps;
    const double arcade_limit=std::min(config.max_steering_rad,std::atan(config.wheelbase_m*3*gravity_mps2/std::max(0.01,speed*speed)));
    const double arcade_stick=std::copysign(std::pow(std::abs(controls.steering),1.15),controls.steering);
    command.steering_rad=std::lerp(command.steering_rad,arcade_stick*arcade_limit,help);
    command.acceleration_mps2=std::lerp(command.acceleration_mps2,controls.throttle*6-controls.brake*18,help);
    if (assistance.manual) {
        command.steering_rad=arcade_stick*std::min(config.max_steering_rad,
            arcade_limit*std::lerp(0.15,1.5,(assistance.steering-1)/99));
        command.acceleration_mps2=controls.throttle*std::lerp(0.5,12.0,(assistance.acceleration-1)/99)
                               -controls.brake*std::lerp(1.0,22.0,(assistance.braking-1)/99);
    }
    if (command.acceleration_mps2>0) command.acceleration_mps2*=std::clamp((cap-speed)/1.0,0.0,1.0);
    if (speed>cap) command.acceleration_mps2=std::min(command.acceleration_mps2,-std::min(12.0,4*(speed-cap)));
    return command;
}

} // namespace fd

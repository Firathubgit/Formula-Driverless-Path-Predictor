#include "fd/vehicle.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace fd {
namespace {

bool finite_plant(const PlantState& s) {
    return std::isfinite(s.pose.x_m) && std::isfinite(s.pose.y_m) && std::isfinite(s.pose.yaw_rad) &&
           std::isfinite(s.pose.speed_mps) && std::isfinite(s.pose.steering_rad) && std::isfinite(s.pose.time_s) &&
           s.pose.speed_mps >= 0 && std::isfinite(s.lateral_velocity_mps) && std::isfinite(s.yaw_rate_radps) &&
           std::all_of(s.wheel_speeds_radps.begin(), s.wheel_speeds_radps.end(), [](double w) { return std::isfinite(w) && w >= 0; }) &&
           std::all_of(s.wheel_loads_n.begin(), s.wheel_loads_n.end(), [](double load) { return std::isfinite(load) && load >= 0; });
}

PlantState kinematic_plant(const State& pose, const Config& config) {
    return {pose, 0.0, pose.speed_mps*std::tan(pose.steering_rad)/config.wheelbase_m};
}

// Walking pace behaves as the kinematic bicycle, so a model's demand there is reported the same way.
Demand kinematic_demand(const PlantState& s, const Config& config, double longitudinal) {
    Demand d;
    d.longitudinal_acceleration_mps2 = longitudinal;
    d.lateral_acceleration_mps2 = s.pose.speed_mps*s.pose.speed_mps*std::tan(s.pose.steering_rad)/config.wheelbase_m;
    d.grip_utilization = std::hypot(d.lateral_acceleration_mps2, longitudinal)/(config.grip_mu*gravity_mps2);
    d.within_envelope = !(std::hypot(d.lateral_acceleration_mps2, longitudinal) > config.grip_mu*gravity_mps2*(1+1e-6));
    return d;
}

// ---------------------------------------------------------------- dynamic single-track

// Motion of the centre of gravity in its own body frame, with the map pose.
struct Body { double x{}, y{}, yaw{}, vx{}, vy{}, r{}; };

// Everything about the car that stays fixed while it is advanced: axle geometry, static axle
// loads, and each axle's tire validated and prepared at its per-tire load; and, when the car's
// loads move (decision 0025), each axle's tire prepared for whatever load it carries.
struct Geometry {
    double lf{}, lr{}, front_load_n{}, rear_load_n{};
    TireAtLoad front_tire, rear_tire;
    std::optional<PreparedTire> front_prepared, rear_prepared;
};
bool loads_move(const DynamicSingleTrack& car) { return car.cg_height_m > 0 || car.downforce_area_m2 > 0; }
Geometry geometry(const DynamicSingleTrack& car, const Config& config) {
    const double L = config.wheelbase_m;
    const double weight = car.mass_kg*gravity_mps2;
    const double front = weight*car.cg_to_rear_m/L, rear = weight*(L-car.cg_to_rear_m)/L;
    Geometry g{L-car.cg_to_rear_m, car.cg_to_rear_m, front, rear, TireAtLoad(car.front_tire, front/2), TireAtLoad(car.rear_tire, rear/2),
               std::nullopt, std::nullopt};
    if (loads_move(car)) { g.front_prepared.emplace(car.front_tire); g.rear_prepared.emplace(car.rear_tire); }
    return g;
}

// The axle loads under the requested tire force and at a forward speed (decision 0025): static, less the force times the
// centre of gravity's height over the wheelbase at the front and more at the rear, plus downforce by the aero balance, and
// never below a hundredth of static, since an axle lifted off the road is not modeled.
struct AxleLoads { double front{}, rear{}; };
AxleLoads axle_loads(const DynamicSingleTrack& car, const Config& config, const Geometry& g, double forward, double acceleration) {
    const double transfer = car.mass_kg*acceleration*car.cg_height_m/config.wheelbase_m;
    const double down = 0.5*air_density_kgpm3*car.downforce_area_m2*forward*forward;
    return {std::max(0.01*g.front_load_n, g.front_load_n-transfer+down*car.aero_balance_front),
            std::max(0.01*g.rear_load_n, g.rear_load_n+transfer+down*(1-car.aero_balance_front))};
}
// Each axle's tire at a load, per tire.
struct LoadedAxles { AxleLoads loads; TireAtLoad front, rear; };
LoadedAxles loaded_axles(const DynamicSingleTrack& car, const Config& config, const Geometry& g, double forward, double acceleration) {
    if (!g.front_prepared) return {{g.front_load_n, g.rear_load_n}, g.front_tire, g.rear_tire};
    const auto loads = axle_loads(car, config, g, forward, acceleration);
    return {loads, g.front_prepared->at(loads.front/2), g.rear_prepared->at(loads.rear/2)};
}

// lr is the distance from the rear axle forward to the centre of gravity.
Body to_body(const PlantState& s, double lr) {
    const double c = std::cos(s.pose.yaw_rad), si = std::sin(s.pose.yaw_rad);
    return {s.pose.x_m+lr*c, s.pose.y_m+lr*si, s.pose.yaw_rad, s.pose.speed_mps,
            s.lateral_velocity_mps+lr*s.yaw_rate_radps, s.yaw_rate_radps};
}
PlantState from_body(const Body& b, double steering, double time, double lr) {
    const double c = std::cos(b.yaw), si = std::sin(b.yaw);
    return {{b.x-lr*c, b.y-lr*si, wrap_angle(b.yaw), std::max(0.0, b.vx), steering, time}, b.vy-lr*b.r, b.r};
}

struct Axle { double fx{}, fy{}, fx_capacity{}, fy_capacity{}, slip_angle{}, peak_slip_angle{}; };

// One axle: lateral force from the tire law at its slip angle and static load, longitudinal force as
// requested but no more than its friction allows, and lateral force reduced so the pair stays inside
// the friction ellipse. Road grip scales friction, not the slip at which it peaks.
Axle axle(const TireAtLoad& tire, const Config& config, double load, double fx_request, double slip_angle) {
    const auto& peak = tire.peak();
    Axle a;
    a.fx_capacity = config.grip_mu*peak.friction_longitudinal*load;
    a.fy_capacity = config.grip_mu*peak.friction_lateral*load;
    a.fx = std::clamp(fx_request, -a.fx_capacity, a.fx_capacity);
    const double pure = config.grip_mu*2*tire.force(0, slip_angle).lateral_n;
    const double used = a.fx/a.fx_capacity;
    a.fy = pure*std::sqrt(std::max(0.0, 1-used*used));
    a.slip_angle = slip_angle;
    a.peak_slip_angle = peak.slip_angle_rad;
    return a;
}

struct Forces { Axle front, rear; double steering{}; };

Forces forces(const DynamicSingleTrack& car, const Config& config, const Geometry& g,
              double vx, double vy, double r, double steering, double longitudinal_acceleration) {
    const double forward = std::max(vx, car.slip_speed_floor_mps);
    const double front_slip = steering-std::atan2(vy+g.lf*r, forward);
    const double rear_slip = -std::atan2(vy-g.lr*r, forward);
    // Driving force split by the drive layout; braking shared in proportion to static load.
    const double total = car.mass_kg*longitudinal_acceleration;
    const double front_request = total < 0 ? total*g.front_load_n/(g.front_load_n+g.rear_load_n)
                                           : total*car.drive_front_fraction;
    if (!g.front_prepared)
        return {axle(g.front_tire, config, g.front_load_n, front_request, front_slip),
                axle(g.rear_tire, config, g.rear_load_n, total-front_request, rear_slip), steering};
    const auto at = loaded_axles(car, config, g, vx, longitudinal_acceleration);
    return {axle(at.front, config, at.loads.front, front_request, front_slip),
            axle(at.rear, config, at.loads.rear, total-front_request, rear_slip), steering};
}

struct BodyForce { double longitudinal{}, lateral{}, yaw_moment{}; };
BodyForce resultant(const Forces& f, const Geometry& g) {
    const double c = std::cos(f.steering), s = std::sin(f.steering);
    const double front_lateral = f.front.fx*s+f.front.fy*c;
    return {f.rear.fx+f.front.fx*c-f.front.fy*s, f.rear.fy+front_lateral, g.lf*front_lateral-g.lr*f.rear.fy};
}

// Weight of the dynamic model: zero below the lower blend speed, one above the upper.
template<class Car> double dynamic_weight(const Car& car, double speed) {
    return std::clamp((speed-car.kinematic_below_mps)/(car.dynamic_above_mps-car.kinematic_below_mps), 0.0, 1.0);
}

// Rate at which the blended model is drawn back toward kinematic motion as speed falls through the
// blend band. It makes the state arrive on the kinematic bicycle's motion by the lower blend speed,
// where the model switches to it exactly, without a jump.
constexpr double blend_relaxation_per_s = 20;

// State derivative. Through the blend band it is a weighted sum of the dynamic derivative and the
// kinematic bicycle's, which keeps the rear axle's lateral velocity at zero and the yaw rate at
// speed times curvature, so the blended model is itself an ordinary differential equation and its
// integration converges as the substep shrinks.
Body blend_toward_kinematic(Body d, double w, const Body& b, double steering, double steering_rate,
                            double longitudinal_acceleration, double L, double lr) {
    if (w < 1) {
        const double t = std::tan(steering), secant = 1/std::cos(steering);
        const double kinematic_rate = b.vx*t/L;
        const double kinematic_rate_dot = (longitudinal_acceleration*t+b.vx*steering_rate*secant*secant)/L;
        const double rate_dot = kinematic_rate_dot+blend_relaxation_per_s*(kinematic_rate-b.r);
        const double lateral_dot = lr*kinematic_rate_dot+blend_relaxation_per_s*(lr*kinematic_rate-b.vy);
        d.vx = w*d.vx+(1-w)*longitudinal_acceleration;
        d.vy = w*d.vy+(1-w)*lateral_dot;
        d.r = w*d.r+(1-w)*rate_dot;
    }
    return d;
}

Body derivative(const DynamicSingleTrack& car, const Config& config, const Geometry& g,
                const Body& b, double steering, double steering_rate, double longitudinal_acceleration) {
    const auto f = resultant(forces(car, config, g, b.vx, b.vy, b.r, steering, longitudinal_acceleration), g);
    const double c = std::cos(b.yaw), s = std::sin(b.yaw);
    // Drag acts against the centre of gravity's velocity and through it (decision 0025).
    const double drag = 0.5*air_density_kgpm3*car.drag_area_m2*std::hypot(b.vx, b.vy);
    Body d{b.vx*c-b.vy*s, b.vx*s+b.vy*c, b.r,
           (f.longitudinal-drag*b.vx)/car.mass_kg+b.vy*b.r, (f.lateral-drag*b.vy)/car.mass_kg-b.vx*b.r, f.yaw_moment/car.yaw_inertia_kgm2};
    return blend_toward_kinematic(d, dynamic_weight(car, b.vx), b, steering, steering_rate, longitudinal_acceleration,
                                  config.wheelbase_m, g.lr);
}

Body add(const Body& a, const Body& d, double h) {
    return {a.x+h*d.x, a.y+h*d.y, a.yaw+h*d.yaw, a.vx+h*d.vx, a.vy+h*d.vy, a.r+h*d.r};
}

PlantState advance_dynamic(const DynamicSingleTrack& car, const PlantState& state, Command applied,
                           const Config& config, double dt) {
    const Geometry g = geometry(car, config);
    const double target = std::clamp(applied.steering_rad, -config.max_steering_rad, config.max_steering_rad);
    const auto substeps = static_cast<int>(std::ceil(dt/car.substep_s-1e-9));
    const double h = dt/substeps;
    PlantState s = state;
    for (int k = 0; k < substeps; ++k) {
        if (dynamic_weight(car, s.pose.speed_mps) == 0) {
            // Walking pace: exactly the kinematic bicycle, which also stops within the substep.
            s = kinematic_plant(integrate_bicycle(s.pose, applied, config, h), config);
            continue;
        }
        const double steer_start = s.pose.steering_rad;
        const double steer_end = steer_start+std::clamp(target-steer_start, -config.max_steering_rate_radps*h,
                                                        config.max_steering_rate_radps*h);
        const auto steer_at = [&](double fraction) { return steer_start+fraction*(steer_end-steer_start); };
        const double steer_rate = (steer_end-steer_start)/h;
        const double a = applied.acceleration_mps2;
        const Body b = to_body(s, g.lr);
        const Body k1 = derivative(car, config, g, b, steer_at(0), steer_rate, a);
        const Body k2 = derivative(car, config, g, add(b, k1, h/2), steer_at(0.5), steer_rate, a);
        const Body k3 = derivative(car, config, g, add(b, k2, h/2), steer_at(0.5), steer_rate, a);
        const Body k4 = derivative(car, config, g, add(b, k3, h), steer_at(1), steer_rate, a);
        Body next{b.x+h/6*(k1.x+2*k2.x+2*k3.x+k4.x), b.y+h/6*(k1.y+2*k2.y+2*k3.y+k4.y),
                  b.yaw+h/6*(k1.yaw+2*k2.yaw+2*k3.yaw+k4.yaw), b.vx+h/6*(k1.vx+2*k2.vx+2*k3.vx+k4.vx),
                  b.vy+h/6*(k1.vy+2*k2.vy+2*k3.vy+k4.vy), b.r+h/6*(k1.r+2*k2.r+2*k3.r+k4.r)};
        next.vx = std::max(0.0, next.vx);
        s = from_body(next, steer_end, s.pose.time_s+h, g.lr);
    }
    s.pose.time_s = state.pose.time_s+dt;
    return s;
}

Demand demand_dynamic(const DynamicSingleTrack& car, const PlantState& s, const Config& config, double longitudinal) {
    if (dynamic_weight(car, s.pose.speed_mps) == 0) return kinematic_demand(s, config, longitudinal);
    Demand d;
    d.longitudinal_acceleration_mps2 = longitudinal;
    const Geometry g = geometry(car, config);
    const Body b = to_body(s, g.lr);
    const auto f = forces(car, config, g, b.vx, b.vy, b.r, s.pose.steering_rad, longitudinal);
    d.lateral_acceleration_mps2 = resultant(f, g).lateral/car.mass_kg;
    const auto use = [](const Axle& a) { return std::hypot(a.fx/a.fx_capacity, a.fy/a.fy_capacity); };
    d.grip_utilization = std::max(use(f.front), use(f.rear));
    d.within_envelope = std::abs(f.front.slip_angle) <= f.front.peak_slip_angle && std::abs(f.rear.slip_angle) <= f.rear.peak_slip_angle;
    d.tire_slip_modeled = true;
    d.front_slip_angle_rad = f.front.slip_angle;
    d.rear_slip_angle_rad = f.rear.slip_angle;
    d.front_peak_slip_angle_rad = f.front.peak_slip_angle;
    d.rear_peak_slip_angle_rad = f.rear.peak_slip_angle;
    return d;
}

void validate_dynamic(const DynamicSingleTrack& car, const Config& config) {
    const auto positive = [](double value, double high, const char* name) {
        if (!std::isfinite(value) || value <= 0 || value > high)
            throw std::invalid_argument(std::string("Vehicle ")+name+" outside supported range");
    };
    positive(car.mass_kg, 1e5, "mass_kg");
    positive(car.yaw_inertia_kgm2, 1e7, "yaw_inertia_kgm2");
    if (!std::isfinite(car.cg_to_rear_m) || car.cg_to_rear_m <= 0 || car.cg_to_rear_m >= config.wheelbase_m)
        throw std::invalid_argument("Vehicle cg_to_rear_m must lie strictly between the axles");
    if (!std::isfinite(car.kinematic_below_mps) || !std::isfinite(car.dynamic_above_mps) || car.kinematic_below_mps < 0 ||
        car.dynamic_above_mps <= car.kinematic_below_mps || car.dynamic_above_mps > 20)
        throw std::invalid_argument("Vehicle blend speeds must be ordered within 0..20 m/s");
    positive(car.slip_speed_floor_mps, 5, "slip_speed_floor_mps");
    if (!std::isfinite(car.drive_front_fraction) || car.drive_front_fraction < 0 || car.drive_front_fraction > 1)
        throw std::invalid_argument("Vehicle drive_front_fraction must lie within 0..1");
    positive(car.substep_s, config.fixed_dt_s, "substep_s (no longer than the plant tick)");
    const auto between = [](double value, double low, double high, const char* name) {
        if (!std::isfinite(value) || value < low || value > high)
            throw std::invalid_argument(std::string("Vehicle ")+name+" outside supported range");
    };
    // Every four-wheel car's centre of gravity lies within half its narrower track, at most 1.5 m, so its reduction is valid.
    between(car.cg_height_m, 0, 1.5, "cg_height_m (0..1.5 m)");
    between(car.drag_area_m2, 0, 10, "drag_area_m2 (0..10 m^2)");
    between(car.downforce_area_m2, 0, 10, "downforce_area_m2 (0..10 m^2, positive pressing down)");
    between(car.aero_balance_front, 0, 1, "aero_balance_front (0..1)");
    validate_tire(car.front_tire);
    validate_tire(car.rear_tire);
}

// ---------------------------------------------------------------- four-wheel car

struct WheelPlace { double x{}, y{}; bool front{}; };

// Fixed while the car is advanced: wheel positions from the centre of gravity and each axle's tire,
// validated once and evaluated at whatever load its wheels carry.
struct CarGeometry {
    double lf{}, lr{};
    std::array<WheelPlace, 4> wheels;
    PreparedTire front_tire, rear_tire;
    const PreparedTire& tire(std::size_t wheel) const { return wheel < 2 ? front_tire : rear_tire; }
};
CarGeometry car_geometry(const FourWheelCar& car, const Config& config) {
    const double L = config.wheelbase_m, lr = car.cg_to_rear_m, lf = L-lr;
    return {lf, lr,
            {WheelPlace{lf, car.track_front_m/2, true}, WheelPlace{lf, -car.track_front_m/2, true},
             WheelPlace{-lr, car.track_rear_m/2, false}, WheelPlace{-lr, -car.track_rear_m/2, false}},
            PreparedTire(car.front_tire), PreparedTire(car.rear_tire)};
}

// Quasi-static wheel loads (decision 0014). Weight, the pitch and roll moments of the horizontal force at
// the ground about the centre of gravity, and the roll-balance closure are four equations linear in the
// four loads; this is their solution. Braking force moves load onto the front axle, a leftward force moves
// it onto the right-hand wheels, and the closure gives the front axle roll_balance_front of that lateral
// difference. A wheel whose load would be negative has lifted: it is clamped at zero, as fastest-lap's
// smooth_pos does, and the loads then exceed weight by what was clamped. With the centre of gravity on
// the ground these are exactly the static loads. Downforce (decision 0015) presses on the axles as the aero
// balance splits it; drag passes through the centre of gravity, so it moves no load except through the tire
// force that overcomes it.
std::array<double, 4> wheel_loads(const FourWheelCar& car, const Config& config, double longitudinal_force, double lateral_force,
                                  double downforce) {
    const double L = config.wheelbase_m, lr = car.cg_to_rear_m, lf = L-lr, h = car.cg_height_m, D = car.roll_balance_front;
    const double weight = car.mass_kg*gravity_mps2;
    const double front = (weight*lr-h*longitudinal_force)/L/2+car.aero_balance_front*downforce/2;
    const double rear = (weight*lf+h*longitudinal_force)/L/2+(1-car.aero_balance_front)*downforce/2;
    // Right-hand minus left-hand load, summed over the two axles as their tracks weight it.
    const double difference = 2*h*lateral_force/(D*car.track_front_m+(1-D)*car.track_rear_m);
    const double front_shift = D*difference/2, rear_shift = (1-D)*difference/2;
    return {std::max(0.0, front-front_shift), std::max(0.0, front+front_shift),
            std::max(0.0, rear-rear_shift), std::max(0.0, rear+rear_shift)};
}

// Drag against the centre of gravity's velocity and downforce from its forward speed, without wind (decision 0015).
AerodynamicForce air(const FourWheelCar& car, double forward, double lateral) {
    const double q = 0.5*air_density_kgpm3;
    const double speed = std::hypot(forward, lateral);
    return {-q*car.drag_area_m2*speed*forward, -q*car.drag_area_m2*speed*lateral, q*car.downforce_area_m2*forward*forward};
}

// A wheel's tire at its present load, and the road grip it transmits. A lifted wheel transmits nothing; its
// tire is prepared at the lower reference load only for its peak slips, which the tire law holds constant
// below that load.
struct LoadedWheel { TireAtLoad tire; double grip{}; };
std::array<LoadedWheel, 4> loaded_wheels(const CarGeometry& g, const Config& config, const std::array<double, 4>& loads) {
    const auto wheel = [&](std::size_t i) {
        const auto& tire = g.tire(i);
        return loads[i] > 0 ? LoadedWheel{tire.at(loads[i]), config.grip_mu}
                            : LoadedWheel{tire.at(tire.parameters().reference_load_low_n), 0.0};
    };
    return {wheel(0), wheel(1), wheel(2), wheel(3)};
}

// A wheel's velocity along its own heading, and its slip angle, from the chassis motion at the centre of
// gravity. The slip angle has the single-track form, so a zero track reproduces its axle angles.
struct WheelMotion { double forward{}, slip_angle{}; };
WheelMotion wheel_motion(const FourWheelCar& car, const WheelPlace& w, double vx, double vy, double r, double steering) {
    const double vxw = vx-r*w.y, vyw = vy+r*w.x;
    const double d = w.front ? steering : 0.0;
    return {std::cos(d)*vxw+std::sin(d)*vyw, d-std::atan2(vyw, std::max(vxw, car.slip_speed_floor_mps))};
}
double slip_ratio(const FourWheelCar& car, double omega, double forward) {
    return (omega*car.wheel_radius_m-forward)/std::max(std::abs(forward), car.slip_speed_floor_mps);
}

// The controller's acceleration as wheel torque: an ideal pedal map. Driving torque is shared by the
// drive layout, equally across each open differential, capped per wheel and then scaled to the power
// limit at the wheels' present speed. Braking torque is shared by the brake bias and capped per wheel.
struct WheelTorques { std::array<double, 4> drive{}, brake{}; };
WheelTorques wheel_torques(const FourWheelCar& car, double acceleration, const std::array<double, 4>& omega) {
    WheelTorques t;
    const double force = car.mass_kg*acceleration, R = car.wheel_radius_m;
    if (force >= 0) {
        const double front = force*car.drive_front_fraction*R/2, rear = force*(1-car.drive_front_fraction)*R/2;
        double power = 0;
        for (std::size_t i = 0; i < 4; ++i) {
            t.drive[i] = std::min(i < 2 ? front : rear, car.max_drive_torque_nm);
            // The controller's top speed: full torque below 95% of it at this wheel's rim, none at it.
            if (std::isfinite(car.max_drive_speed_mps))
                t.drive[i] *= std::clamp((car.max_drive_speed_mps-omega[i]*R)/(0.05*car.max_drive_speed_mps), 0.0, 1.0);
            power += t.drive[i]*omega[i];
        }
        if (power > car.max_drive_power_w)
            for (auto& torque : t.drive) torque *= car.max_drive_power_w/power;
    } else {
        const double total = -force*R;
        const double front = total*car.brake_bias_front/2, rear = total*(1-car.brake_bias_front)/2;
        for (std::size_t i = 0; i < 4; ++i) t.brake[i] = std::min(i < 2 ? front : rear, car.max_brake_torque_nm);
    }
    return t;
}

// One wheel over one substep, implicitly, with the chassis held:
// I (w' - w) = h (drive - brake - Fx(w') R - c (w' - other)), where c is the viscous coupling to the other wheel on
// its axle at that wheel's speed, zero for an open differential. The brake opposes forward rotation and can hold
// a stopped wheel, so when even a stopped wheel would not be turned forward the wheel is locked at zero. Otherwise the root is bracketed and found by Newton's
// method with bisection as a guard. Explicit integration would need a substep below the wheel's time
// constant, I vx / (C_kappa R^2), a fraction of a millisecond at low speed (decision 0012).
double wheel_update(const FourWheelCar& car, const TireAtLoad& tire, double grip, double h, double omega,
                    const WheelMotion& motion, double drive, double brake, double coupling, double other) {
    const double R = car.wheel_radius_m, I = car.wheel_inertia_kgm2;
    const auto residual = [&](double w) {
        const double fx = grip*tire.force(slip_ratio(car, w, motion.forward), motion.slip_angle).longitudinal_n;
        return I*(w-omega)-h*(drive-brake-fx*R-coupling*(w-other));
    };
    // Without brake torque nothing can hold a wheel still against the road, so only a braked wheel can lock.
    if (brake > 0 && residual(0) >= 0) return 0;
    const double tolerance = 1e-12*I*std::max(1.0, omega);
    // Newton from the previous speed, with the slope refreshed by secant steps. It usually converges in two
    // or three evaluations; anything else falls back to the bracketed search below.
    {
        double w = omega, f = residual(w);
        if (std::abs(f) <= tolerance) return w;
        const double probe = 1e-7*std::max(1.0, w);
        double slope = (residual(w+probe)-f)/probe;
        for (int iteration = 0; iteration < 8 && slope > 0 && std::isfinite(slope); ++iteration) {
            double next = w-f/slope;
            if (next < 0) next = 0.5*w;
            const double value = residual(next);
            if (std::abs(value) <= tolerance) return next;
            if (!(std::abs(value) < std::abs(f)) || next == w) break;
            slope = (value-f)/(next-w);
            w = next;
            f = value;
        }
    }
    double lo = 0, hi = std::max({omega, motion.forward/R, 0.0})+1;
    double high_value = residual(hi);
    for (int k = 0; high_value <= 0 && k < 60; ++k) { lo = hi; hi *= 2; high_value = residual(hi); }
    if (!(high_value > 0)) throw std::runtime_error("Wheel speed update could not bracket its solution");
    double w = std::clamp(omega, lo, hi);
    for (int iteration = 0; iteration < 80; ++iteration) {
        const double f = residual(w);
        if (std::abs(f) <= tolerance) break;
        if (f > 0) hi = w; else lo = w;
        const double step = 1e-7*std::max(1.0, w);
        const double slope = (residual(w+step)-f)/step;
        double next = w-f/slope;
        if (!std::isfinite(next) || next <= lo || next >= hi) next = 0.5*(lo+hi);
        if (hi-lo <= 1e-14*std::max(1.0, hi)) { w = next; break; }
        w = next;
    }
    return w;
}

// A wheel's force at a given slip ratio and the slip angle of the chassis motion.
struct WheelForce { double slip_ratio{}, slip_angle{}, fx{}, fy{}, combined_slip{}; };
WheelForce wheel_force(const FourWheelCar& car, const CarGeometry& g, const LoadedWheel& wheel, std::size_t i,
                       double vx, double vy, double r, double steering, double kappa) {
    const auto motion = wheel_motion(car, g.wheels[i], vx, vy, r, steering);
    const auto force = wheel.tire.force(kappa, motion.slip_angle);
    const auto& peak = wheel.tire.peak();
    return {kappa, motion.slip_angle, wheel.grip*force.longitudinal_n, wheel.grip*force.lateral_n,
            std::hypot(kappa/peak.slip_ratio, motion.slip_angle/peak.slip_angle_rad)};
}

BodyForce car_forces(const FourWheelCar& car, const CarGeometry& g, const std::array<LoadedWheel, 4>& wheels, double vx, double vy,
                     double r, double steering, const std::array<double, 4>& kappa) {
    BodyForce total;
    const double c = std::cos(steering), s = std::sin(steering);
    for (std::size_t i = 0; i < 4; ++i) {
        const auto f = wheel_force(car, g, wheels[i], i, vx, vy, r, steering, kappa[i]);
        const bool front = g.wheels[i].front;
        const double bx = front ? f.fx*c-f.fy*s : f.fx, by = front ? f.fx*s+f.fy*c : f.fy;
        total.longitudinal += bx;
        total.lateral += by;
        total.yaw_moment += g.wheels[i].x*by-g.wheels[i].y*bx;
    }
    return total;
}

// The state derivative, and the horizontal force that moves load between the wheels (decision 0014): the
// tires' force in the dynamic range and, through the blend band, blended toward the force the kinematic
// bicycle's rolling motion needs, its longitudinal acceleration and speed times its own yaw rate. The blend's
// pull back toward kinematic motion is a numerical device, not a force, so it moves no load.
struct FourDerivative { Body d; double longitudinal_force{}, lateral_force{}; };
FourDerivative derivative_four(const FourWheelCar& car, const Config& config, const CarGeometry& g, const std::array<LoadedWheel, 4>& wheels,
                               const Body& b, double steering, double steering_rate, double longitudinal_acceleration,
                               const std::array<double, 4>& kappa, double assistance) {
    const auto f = car_forces(car, g, wheels, b.vx, b.vy, b.r, steering, kappa);
    // Drag acts through the centre of gravity: it slows the car but turns it and moves load not at all.
    const auto aero = air(car, b.vx, b.vy);
    const double c = std::cos(b.yaw), s = std::sin(b.yaw);
    Body d{b.vx*c-b.vy*s, b.vx*s+b.vy*c, b.r,
           (f.longitudinal+aero.longitudinal_n)/car.mass_kg+b.vy*b.r, (f.lateral+aero.lateral_n)/car.mass_kg-b.vx*b.r,
           f.yaw_moment/car.yaw_inertia_kgm2};
    const double w = dynamic_weight(car, b.vx)*(1-assistance);
    FourDerivative result{blend_toward_kinematic(d, w, b, steering, steering_rate, longitudinal_acceleration, config.wheelbase_m, g.lr),
                          f.longitudinal, f.lateral};
    if (w < 1) {
        const double rolling_lateral = car.mass_kg*b.vx*b.vx*std::tan(steering)/config.wheelbase_m;
        result.longitudinal_force = w*f.longitudinal+(1-w)*car.mass_kg*longitudinal_acceleration;
        result.lateral_force = w*f.lateral+(1-w)*rolling_lateral;
    }
    return result;
}

// The kinematic bicycle's state with every wheel rolling at its own forward speed, and the loads that balance
// the force that motion needs: mass times the given longitudinal acceleration, and laterally mass times speed
// times yaw rate.
PlantState rolling_four(const FourWheelCar& car, const Config& config, const CarGeometry& g, const State& pose,
                        double longitudinal_acceleration) {
    PlantState s = kinematic_plant(pose, config);
    const Body b = to_body(s, g.lr);
    for (std::size_t i = 0; i < 4; ++i)
        s.wheel_speeds_radps[i] = std::max(0.0, wheel_motion(car, g.wheels[i], b.vx, b.vy, b.r, pose.steering_rad).forward/car.wheel_radius_m);
    s.wheel_loads_n = wheel_loads(car, config, car.mass_kg*longitudinal_acceleration, car.mass_kg*b.vx*b.r, air(car, b.vx, b.vy).downforce_n);
    return s;
}

PlantState advance_four(const FourWheelCar& car, const PlantState& state, Command applied, const Config& config, double dt,
                        double assistance=0) {
    const CarGeometry g = car_geometry(car, config);
    const double target = std::clamp(applied.steering_rad, -config.max_steering_rad, config.max_steering_rad);
    const auto substeps = static_cast<int>(std::ceil(dt/car.substep_s-1e-9));
    const double h = dt/substeps;
    PlantState s = state;
    for (int k = 0; k < substeps; ++k) {
        if (dynamic_weight(car, s.pose.speed_mps)*(1-assistance) == 0) {
            const State pose = integrate_bicycle(s.pose, applied, config, h);
            s = rolling_four(car, config, g, pose, (pose.speed_mps-s.pose.speed_mps)/h);
            continue;
        }
        const double steer_start = s.pose.steering_rad;
        const double steer_end = steer_start+std::clamp(target-steer_start, -config.max_steering_rate_radps*h,
                                                        config.max_steering_rate_radps*h);
        const auto steer_at = [&](double fraction) { return steer_start+fraction*(steer_end-steer_start); };
        const double steer_rate = (steer_end-steer_start)/h;
        const double a = applied.acceleration_mps2;
        const Body b = to_body(s, g.lr);
        // Every wheel's tire at the load the state carries, held for the substep (decision 0014).
        const auto wheels = loaded_wheels(g, config, s.wheel_loads_n);
        // Wheels first, implicitly, against the chassis at the start of the substep. Each wheel's slip ratio
        // at its new speed is then held while the chassis is integrated, so the chassis receives exactly the
        // longitudinal force that turned the wheel, and wheel and chassis momentum stay consistent.
        const auto torques = wheel_torques(car, a, s.wheel_speeds_radps);
        std::array<double, 4> omega{}, kappa{};
        for (std::size_t axle = 0; axle < 2; ++axle) {
            const std::size_t left = 2*axle, right = left+1;
            const auto motion_left = wheel_motion(car, g.wheels[left], b.vx, b.vy, b.r, steer_start);
            const auto motion_right = wheel_motion(car, g.wheels[right], b.vx, b.vy, b.r, steer_start);
            const auto update = [&](std::size_t i, const WheelMotion& motion, double coupling, double other) {
                return wheel_update(car, wheels[i].tire, wheels[i].grip, h, s.wheel_speeds_radps[i], motion, torques.drive[i], torques.brake[i],
                                    coupling, other);
            };
            // A driven axle's viscous coupling ties its two implicit updates together (decision 0015). A sweep solves
            // the left wheel holding a guess of the right wheel's speed, then the right wheel holding that left
            // speed; the coupled solution is the guess a sweep returns unchanged. Plain repetition converges because
            // the coupling also damps the wheel being solved, but slowly for a stiff coupling, so a secant step on
            // the sweep's change is taken whenever it shrank, and a plain step otherwise (decision 0016).
            const bool driven = axle == 0 ? car.drive_front_fraction > 0 : car.drive_front_fraction < 1;
            const double coupling = driven ? car.viscous_coupling_nms : 0.0;
            if (coupling == 0) {
                omega[left] = update(left, motion_left, 0.0, 0.0);
                omega[right] = update(right, motion_right, 0.0, 0.0);
            } else {
                const auto sweep = [&](double right_guess, double& left_speed) {
                    left_speed = update(left, motion_left, coupling, right_guess);
                    return update(right, motion_right, coupling, left_speed);
                };
                double guess = s.wheel_speeds_radps[right], previous_guess = guess, previous_change = 0;
                bool settled = false, have_previous = false;
                for (int iteration = 0; iteration < 100 && !settled; ++iteration) {
                    double left_speed = 0;
                    const double right_speed = sweep(guess, left_speed);
                    const double change = right_speed-guess;
                    if (std::abs(change) <= 1e-10*std::max(1.0, right_speed)) {
                        omega[left] = left_speed;
                        omega[right] = right_speed;
                        settled = true;
                        break;
                    }
                    double next = right_speed;
                    if (have_previous && std::abs(change) < std::abs(previous_change) && change != previous_change) {
                        const double secant = guess-change*(guess-previous_guess)/(change-previous_change);
                        if (std::isfinite(secant) && secant >= 0) next = secant;
                    }
                    previous_guess = guess;
                    previous_change = change;
                    have_previous = true;
                    guess = next;
                }
                if (!settled) throw std::runtime_error("Viscous coupling wheel update did not converge");
            }
            kappa[left] = slip_ratio(car, omega[left], motion_left.forward);
            kappa[right] = slip_ratio(car, omega[right], motion_right.forward);
        }
        const auto s1 = derivative_four(car, config, g, wheels, b, steer_at(0), steer_rate, a, kappa, assistance);
        const Body& k1 = s1.d;
        const auto s2 = derivative_four(car, config, g, wheels, add(b, k1, h/2), steer_at(0.5), steer_rate, a, kappa, assistance);
        const Body& k2 = s2.d;
        const auto s3 = derivative_four(car, config, g, wheels, add(b, k2, h/2), steer_at(0.5), steer_rate, a, kappa, assistance);
        const Body& k3 = s3.d;
        const auto s4 = derivative_four(car, config, g, wheels, add(b, k3, h), steer_at(1), steer_rate, a, kappa, assistance);
        const Body& k4 = s4.d;
        Body next{b.x+h/6*(k1.x+2*k2.x+2*k3.x+k4.x), b.y+h/6*(k1.y+2*k2.y+2*k3.y+k4.y),
                  b.yaw+h/6*(k1.yaw+2*k2.yaw+2*k3.yaw+k4.yaw), b.vx+h/6*(k1.vx+2*k2.vx+2*k3.vx+k4.vx),
                  b.vy+h/6*(k1.vy+2*k2.vy+2*k3.vy+k4.vy), b.r+h/6*(k1.r+2*k2.r+2*k3.r+k4.r)};
        next.vx = std::max(0.0, next.vx);
        // The next substep's loads balance this one's horizontal force, in the body frame and weighted as RK4
        // applied it: the plan's loads from the previous substep's forces.
        const double longitudinal = (s1.longitudinal_force+2*s2.longitudinal_force+2*s3.longitudinal_force+s4.longitudinal_force)/6;
        const double lateral = (s1.lateral_force+2*s2.lateral_force+2*s3.lateral_force+s4.lateral_force)/6;
        s = from_body(next, steer_end, s.pose.time_s+h, g.lr);
        s.wheel_speeds_radps = omega;
        // Deliberate anti-lock/traction assistance: relax toward each wheel's rolling
        // speed with a rate in seconds, independent of tick/substep size.
        if (assistance>0) {
            const double settle=1-std::exp(-80*assistance*h);
            for (std::size_t i=0;i<4;++i) {
                const double rolling=std::max(0.0,wheel_motion(car,g.wheels[i],next.vx,next.vy,next.r,steer_end).forward/car.wheel_radius_m);
                s.wheel_speeds_radps[i]=std::lerp(omega[i],rolling,settle);
            }
        }
        s.wheel_loads_n = wheel_loads(car, config, longitudinal, lateral, air(car, next.vx, next.vy).downforce_n);
    }
    s.pose.time_s = state.pose.time_s+dt;
    return s;
}

Demand demand_four(const FourWheelCar& car, const PlantState& s, const Config& config, double longitudinal) {
    if (dynamic_weight(car, s.pose.speed_mps) == 0) return kinematic_demand(s, config, longitudinal);
    const CarGeometry g = car_geometry(car, config);
    const auto wheels = loaded_wheels(g, config, s.wheel_loads_n);
    const Body b = to_body(s, g.lr);
    Demand d;
    d.longitudinal_acceleration_mps2 = longitudinal;
    std::array<double, 4> kappa{};
    for (std::size_t i = 0; i < 4; ++i)
        kappa[i] = slip_ratio(car, s.wheel_speeds_radps[i], wheel_motion(car, g.wheels[i], b.vx, b.vy, b.r, s.pose.steering_rad).forward);
    d.within_envelope = true;
    const double c = std::cos(s.pose.steering_rad), sn = std::sin(s.pose.steering_rad);
    double lateral = 0;
    for (std::size_t i = 0; i < 4; ++i) {
        const auto f = wheel_force(car, g, wheels[i], i, b.vx, b.vy, b.r, s.pose.steering_rad, kappa[i]);
        lateral += g.wheels[i].front ? f.fx*sn+f.fy*c : f.fy;
        d.wheel_slip_ratios[i] = f.slip_ratio;
        d.wheel_combined_slip[i] = f.combined_slip;
        // A lifted wheel has no grip to use and puts the car beyond what this model represents.
        const bool lifted = !(s.wheel_loads_n[i] > 0);
        d.within_envelope = d.within_envelope && !lifted && f.combined_slip <= 1;
        if (lifted) continue;
        const auto& peak = wheels[i].tire.peak();
        const double capacity_x = wheels[i].grip*peak.friction_longitudinal*s.wheel_loads_n[i];
        const double capacity_y = wheels[i].grip*peak.friction_lateral*s.wheel_loads_n[i];
        d.wheel_longitudinal_use[i] = f.fx/capacity_x;
        d.wheel_lateral_use[i] = f.fy/capacity_y;
        d.grip_utilization = std::max(d.grip_utilization, std::hypot(f.fx/capacity_x, f.fy/capacity_y));
    }
    d.lateral_acceleration_mps2 = lateral/car.mass_kg;
    const double forward = std::max(b.vx, car.slip_speed_floor_mps);
    d.front_slip_angle_rad = s.pose.steering_rad-std::atan2(b.vy+g.lf*b.r, forward);
    d.rear_slip_angle_rad = -std::atan2(b.vy-g.lr*b.r, forward);
    // Each axle's peak slip angle is its two wheels' mean, at their present loads.
    d.front_peak_slip_angle_rad = (wheels[front_left].tire.peak().slip_angle_rad+wheels[front_right].tire.peak().slip_angle_rad)/2;
    d.rear_peak_slip_angle_rad = (wheels[rear_left].tire.peak().slip_angle_rad+wheels[rear_right].tire.peak().slip_angle_rad)/2;
    d.tire_slip_modeled = true;
    return d;
}

void validate_four_wheel(const FourWheelCar& car, const Config& config) {
    const auto within = [](double value, double low, double high, const char* name) {
        if (!std::isfinite(value) || value <= low || value > high)
            throw std::invalid_argument(std::string("Four-wheel car ")+name+" outside supported range");
    };
    within(car.mass_kg, 0, 1e5, "mass_kg");
    within(car.yaw_inertia_kgm2, 0, 1e7, "yaw_inertia_kgm2");
    if (!std::isfinite(car.cg_to_rear_m) || car.cg_to_rear_m <= 0 || car.cg_to_rear_m >= config.wheelbase_m)
        throw std::invalid_argument("Four-wheel car cg_to_rear_m must lie strictly between the axles");
    within(car.track_front_m, 0.3, 3, "track_front_m");
    within(car.track_rear_m, 0.3, 3, "track_rear_m");
    if (!std::isfinite(car.cg_height_m) || car.cg_height_m < 0 || car.cg_height_m > 0.5*std::min(car.track_front_m, car.track_rear_m))
        throw std::invalid_argument("Four-wheel car cg_height_m must lie within 0 and half the narrower track; a taller car would tip over "
                                    "before its tires slide, which is not modeled");
    if (!std::isfinite(car.roll_balance_front) || car.roll_balance_front < 0 || car.roll_balance_front > 1)
        throw std::invalid_argument("Four-wheel car roll_balance_front must lie within 0..1");
    within(car.wheel_radius_m, 0.05, 1, "wheel_radius_m");
    within(car.wheel_inertia_kgm2, 0, 50, "wheel_inertia_kgm2");
    if (!std::isfinite(car.drive_front_fraction) || car.drive_front_fraction < 0 || car.drive_front_fraction > 1)
        throw std::invalid_argument("Four-wheel car drive_front_fraction must lie within 0..1");
    if (!std::isfinite(car.brake_bias_front) || car.brake_bias_front < 0 || car.brake_bias_front > 1)
        throw std::invalid_argument("Four-wheel car brake_bias_front must lie within 0..1");
    within(car.max_drive_power_w, 0, 1e7, "max_drive_power_w");
    within(car.max_drive_torque_nm, 0, 1e5, "max_drive_torque_nm");
    within(car.max_brake_torque_nm, 0, 1e5, "max_brake_torque_nm");
    if (std::isnan(car.max_drive_speed_mps) || car.max_drive_speed_mps <= 0)
        throw std::invalid_argument("Four-wheel car max_drive_speed_mps must be positive, or infinity for no limit");
    const auto between = [](double value, double low, double high, const char* name) {
        if (!std::isfinite(value) || value < low || value > high)
            throw std::invalid_argument(std::string("Four-wheel car ")+name+" outside supported range");
    };
    // Above 200 N m s/rad the coupled wheel updates converge slowly; stiffer couplings behave as locked axles.
    between(car.viscous_coupling_nms, 0, 200, "viscous_coupling_nms (0..200 N m s/rad)");
    between(car.drag_area_m2, 0, 10, "drag_area_m2 (0..10 m^2)");
    between(car.downforce_area_m2, 0, 10, "downforce_area_m2 (0..10 m^2, positive pressing down)");
    between(car.aero_balance_front, 0, 1, "aero_balance_front (0..1)");
    if (!std::isfinite(car.kinematic_below_mps) || !std::isfinite(car.dynamic_above_mps) || car.kinematic_below_mps < 0 ||
        car.dynamic_above_mps <= car.kinematic_below_mps || car.dynamic_above_mps > 20)
        throw std::invalid_argument("Four-wheel car blend speeds must be ordered within 0..20 m/s");
    within(car.slip_speed_floor_mps, 0, 5, "slip_speed_floor_mps");
    within(car.substep_s, 0, config.fixed_dt_s, "substep_s (no longer than the plant tick)");
    validate_tire(car.front_tire);
    validate_tire(car.rear_tire);
}


} // namespace

void validate_vehicle(const VehicleModel& model, const Config& config) {
    validate_config(config);
    if (const auto* car = std::get_if<DynamicSingleTrack>(&model)) validate_dynamic(*car, config);
    if (const auto* car = std::get_if<FourWheelCar>(&model)) validate_four_wheel(*car, config);
}

PlantState plant_state_from(const State& state, const VehicleModel& model, const Config& config) {
    if (const auto* car = std::get_if<FourWheelCar>(&model)) return rolling_four(*car, config, car_geometry(*car, config), state, 0.0);
    return kinematic_plant(state, config);
}

PlantState advance(const VehicleModel& model, const PlantState& state, Command applied, const Config& config, double dt) {
    if (std::holds_alternative<KinematicBicycle>(model))
        return kinematic_plant(integrate_bicycle(state.pose, applied, config, dt), config);
    validate_vehicle(model, config);
    if (!finite_plant(state) || !std::isfinite(applied.acceleration_mps2) || !std::isfinite(applied.steering_rad) ||
        !std::isfinite(dt) || dt <= 0)
        throw std::invalid_argument("Vehicle advance requires a finite plant state with nonnegative speed, finite commands and positive dt");
    if (const auto* car = std::get_if<DynamicSingleTrack>(&model)) return advance_dynamic(*car, state, applied, config, dt);
    return advance_four(std::get<FourWheelCar>(model), state, applied, config, dt);
}

void validate_driving_assistance(DrivingAssistance assistance) {
    for (double value : {assistance.level,assistance.acceleration,assistance.braking,assistance.steering,assistance.grip,assistance.speed})
        if (!std::isfinite(value) || value<1 || value>100)
            throw std::invalid_argument("Forgiveness controls must be finite and between 1 and 100");
}
bool driving_assistance_active(DrivingAssistance assistance) {
    validate_driving_assistance(assistance);
    return assistance.enabled && (assistance.manual || assistance.level>1);
}
double driving_assistance_strength(DrivingAssistance assistance) {
    validate_driving_assistance(assistance);
    if (!assistance.enabled) return 0;
    const double t=((assistance.manual ? assistance.grip : assistance.level)-1)/99;
    return 1-(1-t)*(1-t)*(1-t); // 50 is already strongly assisted; a smooth route down to realism.
}
double assisted_steering_rate(const Config& config, DrivingAssistance assistance) {
    if (!driving_assistance_active(assistance)) return config.max_steering_rate_radps;
    return assistance.manual ? std::lerp(0.3,3.0,(assistance.steering-1)/99)
                             : std::lerp(config.max_steering_rate_radps,3.0,driving_assistance_strength(assistance));
}
PlantState advance_assisted(const VehicleModel& model, const PlantState& state, Command applied, const Config& config, double dt,
                           DrivingAssistance assistance) {
    const double help=driving_assistance_strength(assistance);
    if (!driving_assistance_active(assistance)) return advance(model,state,applied,config,dt);
    const auto* car=std::get_if<FourWheelCar>(&model);
    if (!car) throw std::invalid_argument("Forgiveness requires a four-wheel car");
    validate_vehicle(model,config);
    if (!finite_plant(state) || !std::isfinite(applied.acceleration_mps2) || !std::isfinite(applied.steering_rad) ||
        !std::isfinite(dt) || dt<=0)
        throw std::invalid_argument("Assisted advance requires finite state, commands and positive dt");
    auto assisted_config=config;
    assisted_config.max_steering_rate_radps=assisted_steering_rate(config,assistance);
    return advance_four(*car,state,applied,assisted_config,dt,help);
}

Demand demand(const VehicleModel& model, const PlantState& state, const Config& config, double longitudinal) {
    if (const auto* car = std::get_if<DynamicSingleTrack>(&model)) return demand_dynamic(*car, state, config, longitudinal);
    if (const auto* car = std::get_if<FourWheelCar>(&model)) return demand_four(*car, state, config, longitudinal);
    Demand d;
    d.longitudinal_acceleration_mps2 = longitudinal;
    d.lateral_acceleration_mps2 = state.pose.speed_mps*state.pose.speed_mps*std::tan(state.pose.steering_rad)/config.wheelbase_m;
    d.grip_utilization = std::hypot(d.lateral_acceleration_mps2, longitudinal)/(config.grip_mu*gravity_mps2);
    d.within_envelope = !(std::hypot(d.lateral_acceleration_mps2, longitudinal) > config.grip_mu*gravity_mps2*(1+1e-6));
    return d;
}

AerodynamicForce aerodynamic_force(const FourWheelCar& car, double forward, double lateral) {
    validate_four_wheel(car, Config{});
    if (!std::isfinite(forward) || !std::isfinite(lateral)) throw std::invalid_argument("Aerodynamic force requires a finite velocity");
    return air(car, forward, lateral);
}

std::array<double, 4> quasi_static_wheel_loads(const FourWheelCar& car, const Config& config, double longitudinal_force, double lateral_force,
                                               double downforce) {
    validate_vehicle(car, config);
    if (!std::isfinite(longitudinal_force) || !std::isfinite(lateral_force))
        throw std::invalid_argument("Wheel loads require finite horizontal forces");
    if (!std::isfinite(downforce) || downforce < 0) throw std::invalid_argument("Wheel loads require a finite, non-negative downforce");
    return wheel_loads(car, config, longitudinal_force, lateral_force, downforce);
}

namespace {
// The static axle loads of a validated single-track car.
double front_load(const DynamicSingleTrack& car, const Config& config) {
    return car.mass_kg*gravity_mps2*car.cg_to_rear_m/config.wheelbase_m;
}
double rear_load(const DynamicSingleTrack& car, const Config& config) {
    return car.mass_kg*gravity_mps2*(config.wheelbase_m-car.cg_to_rear_m)/config.wheelbase_m;
}
const DynamicSingleTrack& validated(const DynamicSingleTrack& car, const Config& config) {
    validate_config(config);
    validate_dynamic(car, config);
    return car;
}
} // namespace

SingleTrackModel::SingleTrackModel(const DynamicSingleTrack& car, const Config& config)
    : car_(validated(car, config)), config_(config), front_m_(config.wheelbase_m-car.cg_to_rear_m),
      front_load_n_(front_load(car, config)), rear_load_n_(rear_load(car, config)),
      front_tire_(car.front_tire, front_load_n_/2), rear_tire_(car.rear_tire, rear_load_n_/2) {
    if (loads_move(car)) { front_prepared_.emplace(car.front_tire); rear_prepared_.emplace(car.rear_tire); }
}

SingleTrackMotion SingleTrackModel::derivative(const SingleTrackMotion& m, double steering, double steering_rate,
                                               double acceleration) const {
    const Geometry g{front_m_, car_.cg_to_rear_m, front_load_n_, rear_load_n_, front_tire_, rear_tire_, front_prepared_, rear_prepared_};
    const Body d = fd::derivative(car_, config_, g, {m.x_m, m.y_m, m.yaw_rad, m.forward_mps, m.lateral_mps, m.yaw_rate_radps},
                                  steering, steering_rate, acceleration);
    return {d.x, d.y, d.yaw, d.vx, d.vy, d.r};
}

AxleDemand SingleTrackModel::axle_demand(const SingleTrackMotion& m, double steering, double acceleration) const {
    const double forward = std::max(m.forward_mps, car_.slip_speed_floor_mps);
    const double front_slip = steering-std::atan2(m.lateral_mps+front_m_*m.yaw_rate_radps, forward);
    const double rear_slip = -std::atan2(m.lateral_mps-car_.cg_to_rear_m*m.yaw_rate_radps, forward);
    // The longitudinal request is shared exactly as the plant shares it, and each axle's tires are at the load it carries.
    const double total = car_.mass_kg*acceleration;
    const double front_request = total < 0 ? total*front_load_n_/(front_load_n_+rear_load_n_) : total*car_.drive_front_fraction;
    const auto use = [&](const TireAtLoad& tire, double load, double request, double slip) {
        const auto& peak = tire.peak();
        const double longitudinal = request/(config_.grip_mu*peak.friction_longitudinal*load);
        const double lateral = config_.grip_mu*2*tire.force(0, slip).lateral_n/(config_.grip_mu*peak.friction_lateral*load);
        return std::hypot(longitudinal, lateral);
    };
    const Geometry g{front_m_, car_.cg_to_rear_m, front_load_n_, rear_load_n_, front_tire_, rear_tire_, front_prepared_, rear_prepared_};
    const auto at = loaded_axles(car_, config_, g, m.forward_mps, acceleration);
    return {front_slip, rear_slip, at.front.peak().slip_angle_rad, at.rear.peak().slip_angle_rad,
            use(at.front, at.loads.front, front_request, front_slip), use(at.rear, at.loads.rear, total-front_request, rear_slip)};
}

SingleTrackMotion SingleTrackModel::motion(const PlantState& state) const {
    const Body b = to_body(state, car_.cg_to_rear_m);
    return {b.x, b.y, b.yaw, b.vx, b.vy, b.r};
}

PlantState SingleTrackModel::plant_state(const SingleTrackMotion& m, double steering, double time) const {
    return from_body({m.x_m, m.y_m, m.yaw_rad, m.forward_mps, m.lateral_mps, m.yaw_rate_radps}, steering, time, car_.cg_to_rear_m);
}

DynamicSingleTrack reduced_single_track(const VehicleModel& model) {
    if (const auto* car = std::get_if<DynamicSingleTrack>(&model)) return *car;
    if (std::holds_alternative<KinematicBicycle>(model))
        throw std::invalid_argument("The kinematic bicycle has no tires to reduce to a single-track model; choose a car with tires");
    DynamicSingleTrack reduced;
    if (const auto* four = std::get_if<FourWheelCar>(&model)) {
        reduced.mass_kg = four->mass_kg;
        reduced.yaw_inertia_kgm2 = four->yaw_inertia_kgm2;
        reduced.cg_to_rear_m = four->cg_to_rear_m;
        reduced.drive_front_fraction = four->drive_front_fraction;
        reduced.front_tire = four->front_tire;
        reduced.rear_tire = four->rear_tire;
        reduced.kinematic_below_mps = four->kinematic_below_mps;
        reduced.dynamic_above_mps = four->dynamic_above_mps;
        reduced.slip_speed_floor_mps = four->slip_speed_floor_mps;
        // Its pitch and air (decision 0025): what the four-wheel car's model error was measured to come from.
        reduced.cg_height_m = four->cg_height_m;
        reduced.drag_area_m2 = four->drag_area_m2;
        reduced.downforce_area_m2 = four->downforce_area_m2;
        reduced.aero_balance_front = four->aero_balance_front;
    }
    return reduced;
}

double rear_sideslip_rad(const PlantState& state) {
    return state.pose.speed_mps > 0 ? std::atan2(state.lateral_velocity_mps, state.pose.speed_mps) : 0.0;
}

Envelope envelope(const VehicleModel& model, const PlantState& s, const Config& config, double longitudinal) {
    const auto kinematic = [&] {
        const double lateral = s.pose.speed_mps*s.pose.speed_mps*std::tan(s.pose.steering_rad)/config.wheelbase_m;
        return Envelope{!(std::hypot(lateral, longitudinal) > config.grip_mu*gravity_mps2*(1+1e-6)), false};
    };
    if (const auto* car = std::get_if<DynamicSingleTrack>(&model)) {
        if (dynamic_weight(*car, s.pose.speed_mps) == 0) return kinematic();
        const Geometry g = geometry(*car, config);
        const Body b = to_body(s, g.lr);
        const double forward = std::max(b.vx, car->slip_speed_floor_mps);
        const double front_slip = s.pose.steering_rad-std::atan2(b.vy+g.lf*b.r, forward);
        const double rear_slip = -std::atan2(b.vy-g.lr*b.r, forward);
        const auto at = loaded_axles(*car, config, g, b.vx, longitudinal);
        return {std::abs(front_slip) <= at.front.peak().slip_angle_rad && std::abs(rear_slip) <= at.rear.peak().slip_angle_rad, true};
    }
    if (const auto* car = std::get_if<FourWheelCar>(&model)) {
        if (dynamic_weight(*car, s.pose.speed_mps) == 0) return kinematic();
        const CarGeometry g = car_geometry(*car, config);
        const auto wheels = loaded_wheels(g, config, s.wheel_loads_n);
        const Body b = to_body(s, g.lr);
        bool within = true;
        for (std::size_t i = 0; i < 4; ++i) {
            const auto motion = wheel_motion(*car, g.wheels[i], b.vx, b.vy, b.r, s.pose.steering_rad);
            const double kappa = slip_ratio(*car, s.wheel_speeds_radps[i], motion.forward);
            const auto& peak = wheels[i].tire.peak();
            within = within && s.wheel_loads_n[i] > 0 && std::hypot(kappa/peak.slip_ratio, motion.slip_angle/peak.slip_angle_rad) <= 1;
        }
        return {within, true};
    }
    return kinematic();
}

HandlingBalance handling_balance(const Demand& d) {
    if (!d.tire_slip_modeled) return {Balance::not_modeled, 0.0};
    if (!(std::abs(d.lateral_acceleration_mps2) >= cornering_lateral_acceleration_mps2)) return {Balance::not_cornering, 0.0};
    const double toward_turn = d.lateral_acceleration_mps2 > 0 ? 1.0 : -1.0;
    const double angle = toward_turn*(d.front_slip_angle_rad-d.rear_slip_angle_rad);
    const Balance balance = angle > balance_deadband_rad ? Balance::understeer
                          : angle < -balance_deadband_rad ? Balance::oversteer : Balance::neutral;
    return {balance, angle};
}

const char* balance_name(Balance balance) {
    switch (balance) {
    case Balance::not_modeled: return "not modeled";
    case Balance::not_cornering: return "not cornering";
    case Balance::neutral: return "neutral";
    case Balance::understeer: return "understeer";
    case Balance::oversteer: return "oversteer";
    }
    return "unknown";
}

const char* vehicle_model_name(const VehicleModel& model) {
    if (std::holds_alternative<KinematicBicycle>(model)) return "kinematic bicycle, rear axle reference";
    if (std::holds_alternative<DynamicSingleTrack>(model)) return "dynamic single-track with tire forces, rear axle reference";
    const auto& four = std::get<FourWheelCar>(model);
    const bool loads = four.cg_height_m > 0, aerodynamics = four.drag_area_m2 > 0 || four.downforce_area_m2 > 0;
    if (loads && aerodynamics) return "four-wheel car with rotating wheels, load transfer and aerodynamics, rear axle reference";
    if (loads) return "four-wheel car with rotating wheels and load transfer, rear axle reference";
    if (aerodynamics) return "four-wheel car with rotating wheels and aerodynamics, rear axle reference";
    return "four-wheel car with rotating wheels, rear axle reference";
}

} // namespace fd

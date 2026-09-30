#pragma once
#include "fd/core.hpp"
#include "fd/tire.hpp"

#include <array>
#include <numbers>
#include <limits>
#include <optional>
#include <variant>

namespace fd {

// The vehicle seam (decision 0005). Every plant moves only through advance(), and diagnostics
// ask demand() instead of assuming one plant's formulas. Both models publish the rear-axle State
// that controllers, recordings and the view consume.

// The existing kinematic bicycle: no mass, no slip. Its geometry and actuator limits are the
// Config's. Advancing it is exactly integrate_bicycle.
struct KinematicBicycle {};

// A dynamic single-track (bicycle) model with tire forces, integrated at the centre of gravity.
// Lateral tire force comes from tire_force at the axle's slip angle and load; longitudinal
// force is the commanded acceleration times mass, split between the axles by drive_front_fraction
// when driving and in proportion to static load when braking, and each axle's force stays inside
// its friction ellipse. Load transfer and aerodynamics are off unless set (below), and there
// are no wheel-speed dynamics. Road grip (Config::grip_mu)
// scales tire friction. Parameters are illustrative, not a measured car. See decision 0008.
struct DynamicSingleTrack {
    double mass_kg{800};
    double yaw_inertia_kgm2{1352};    // mass times cg_to_front times cg_to_rear
    double cg_to_rear_m{1.3};         // the front distance is Config::wheelbase_m minus this
    // Share of driving force on the front axle: 0 is rear-wheel drive, 1 front-wheel drive. The default
    // matches the default car's static front load, so traction is shared the way the planner's
    // whole-car grip budget assumes. Rear-wheel drive at that budget spins the car out of corners,
    // because without load transfer the rear axle alone cannot deliver it (decision 0008).
    double drive_front_fraction{0.5};
    TireParameters front_tire{roadster_tire()}, rear_tire{roadster_tire()};
    // Below the lower speed the model is the kinematic bicycle; above the upper it is fully
    // dynamic; between, its state derivative blends linearly from the kinematic bicycle's to the
    // dynamic one. A car starting from rest has no meaningful slip angle, which divides by speed.
    double kinematic_below_mps{1.0}, dynamic_above_mps{3.0};
    // Slip angles use at least this forward speed in their denominator.
    double slip_speed_floor_mps{0.5};
    // RK4 substep. 2.5 ms keeps the fastest lateral mode well inside RK4's stability region above
    // the blend speed; a test holds it within 1 cm of a 0.5 ms reference over three seconds.
    double substep_s{0.0025};
    // Longitudinal load transfer and aerodynamics (decision 0025), all off by default, so the dynamic car is decision 0008's.
    // The requested tire force at the ground, mass times commanded acceleration, moves that force times cg_height_m over the
    // wheelbase of load onto the front axle when braking and off it when driving; downforce, half the air density times
    // downforce_area_m2 times forward speed squared, adds to the axle loads aero_balance_front of it at the front; and drag,
    // half the air density times drag_area_m2 times speed squared, acts against the centre of gravity's velocity, through
    // it, as on the four-wheel car. Each axle's tires are evaluated at the load they carry, never below a hundredth of
    // static. One track has no inner and outer wheel, so there is no lateral load transfer. They let the single-track
    // reduction of the four-wheel car (reduced_single_track) carry that car's pitch and air.
    double cg_height_m{0};
    double drag_area_m2{0};
    double downforce_area_m2{0};
    double aero_balance_front{0.5};
    // Loads near a quarter of 800 kg; peak friction about 1.0 at 2000 N falling with load, so
    // Config::grip_mu keeps its meaning of peak friction on the reference surface.
    static TireParameters roadster_tire() {
        TireParameters t;
        t.peak_friction_longitudinal_low = 0.98; t.peak_friction_longitudinal_high = 0.83;
        t.peak_friction_lateral_low = 1.00;      t.peak_friction_lateral_high = 0.85;
        t.peak_slip_ratio_low = 0.10;            t.peak_slip_ratio_high = 0.09;
        t.peak_slip_angle_low_rad = 0.122;       t.peak_slip_angle_high_rad = 0.105;
        t.minimum_friction = 0.5;
        return t;
    }
    // The same car on softer front tires: their grip peaks at 0.2 rad of slip instead of 0.122, so
    // the front axle has about 60% of the rear's cornering stiffness and the car understeers. Its
    // steady steering exceeds geometry by roughly 0.003 rad per m/s^2 of lateral acceleration, which
    // is what MAP steering corrects and Pure Pursuit does not (decision 0010).
    static DynamicSingleTrack soft_front() {
        DynamicSingleTrack car;
        car.front_tire.peak_slip_angle_low_rad = 0.2;
        car.front_tire.peak_slip_angle_high_rad = 0.18;
        return car;
    }
};

// A car with four rotating wheels (TrackWayFastPlan Phase 2.1, decision 0012). Every wheel has its own
// speed, slip ratio and slip angle and gets its force from the tire law's combined slip at its present
// load. Loads move between the wheels quasi-statically with the tire forces (Phase 2.2, decision 0014),
// and aerodynamic drag and downforce act on the car (Phase 2.3, decision 0015). The chassis is integrated
// at the centre of gravity like DynamicSingleTrack, with the same blend to the kinematic bicycle at
// walking pace. The controller's acceleration is applied as wheel torque: driving torque shared by
// drive_front_fraction and limited by max_drive_power_w and max_drive_torque_nm, braking torque shared by
// brake_bias_front and limited by max_brake_torque_nm. That is an ideal pedal map, so a wheel locks or
// spins only when its torque exceeds what its tire can transmit. Each driven axle shares its torque equally
// between its wheels, and a viscous coupling can move torque from the faster wheel to the slower. Wheel
// speeds are updated implicitly, because at low speed their response is far faster than any practical
// substep. Parameters are illustrative, not a measured car.
struct FourWheelCar {
    double mass_kg{800};
    double yaw_inertia_kgm2{1352};
    double cg_to_rear_m{1.3};            // the front distance is Config::wheelbase_m minus this
    // Height of the centre of gravity above the ground. Braking at a moves m a h / L of load onto the front
    // axle, and cornering at a_y moves m a_y h / track outward. Zero keeps every wheel at its static load,
    // which is the car decision 0012 built. At most half the narrower track: a taller car would tip over on
    // the reference surface before its tires slide, and nothing here models a rollover.
    double cg_height_m{0.35};
    // The front axle's share of lateral load transfer, the load moved from its inner to its outer wheel
    // against the rear axle's. Real cars set it with anti-roll bars. Zero puts it all on the rear.
    double roll_balance_front{0.5};
    double track_front_m{1.5}, track_rear_m{1.5};
    double wheel_radius_m{0.3};
    double wheel_inertia_kgm2{0.8};      // each wheel, about its axle
    TireParameters front_tire{DynamicSingleTrack::roadster_tire()}, rear_tire{DynamicSingleTrack::roadster_tire()};
    double drive_front_fraction{0.5};    // share of driving torque on the front axle; 0 is rear-wheel drive
    double max_drive_power_w{100000};    // total at the wheels
    double max_drive_torque_nm{1200};    // each driven wheel
    double brake_bias_front{0.5};        // share of braking torque on the front axle; default is the static load share, not the braking one
    double max_brake_torque_nm{2500};    // each wheel
    // The drive controller's top speed at the driven wheels' rims (decision 0038), as an electric kart's controller cuts its
    // motor: driving torque falls linearly from full at 95% of it to none at it, and braking is untouched. Infinity, the
    // default, is no limit, so every car before it drives exactly as it did.
    double max_drive_speed_mps{std::numeric_limits<double>::infinity()};
    // Viscous coupling across each driven axle: torque viscous_coupling_nms times the difference in wheel speed
    // moves from the faster wheel to the slower. Zero is an open differential. It acts only when the two wheels
    // turn at different speeds, which in a corner load transfer makes likely (decision 0015).
    double viscous_coupling_nms{0};
    // Aerodynamics without wind (decision 0015). Drag is half the air density times drag_area_m2 (drag
    // coefficient times frontal area) times speed squared, against the velocity of the centre of gravity and
    // through it. Downforce is half the air density times downforce_area_m2 (lift coefficient times area, positive
    // pressing down) times forward speed squared, aero_balance_front of it on the front axle. The defaults
    // are a road car's drag and no wings.
    double drag_area_m2{0.6};
    double downforce_area_m2{0};
    double aero_balance_front{0.5};
    double kinematic_below_mps{1.0}, dynamic_above_mps{3.0};
    double slip_speed_floor_mps{0.5};    // slip ratio and slip angle use at least this speed in their denominators
    // Chassis RK4 substep, with the wheels solved implicitly at each (decision 0012). A test holds it within 1 cm
    // and 1e-3 m/s of a 0.2 ms reference; lock onset falls on a substep boundary, which leaves a locked stop
    // within 5e-3 m/s. A 5 ms step would halve the cost but made wheelspin a hundred times less accurate.
    double substep_s{0.0025};
};

using VehicleModel = std::variant<KinematicBicycle, DynamicSingleTrack, FourWheelCar>;

// Opt-in, deliberately nonphysical training assistance for a person driving a four-wheel car.
// Overall level 1 and disabled are exactly the ordinary plant; overall/grip 100
// gives no-slip rolling motion. Manual settings are independent 1..100 controls.
struct DrivingAssistance {
    bool enabled{}; double level{100};
    bool manual{};
    double acceleration{50}, braking{50}, steering{50}, grip{100}, speed{50};
    bool operator==(const DrivingAssistance&) const = default;
};
void validate_driving_assistance(DrivingAssistance assistance);
double driving_assistance_strength(DrivingAssistance assistance);
bool driving_assistance_active(DrivingAssistance assistance);
double assisted_steering_rate(const Config& config, DrivingAssistance assistance);

// Wheel order everywhere: front left, front right, rear left, rear right.
inline constexpr std::size_t front_left = 0, front_right = 1, rear_left = 2, rear_right = 3;

// Copyable plant state. Predictions copy it and roll the copy forward.
struct PlantState {
    State pose;                        // rear axle; speed_mps is body forward velocity, >= 0
    double lateral_velocity_mps{};     // body lateral velocity at the rear axle, left positive
    double yaw_rate_radps{};
    // Wheel angular speeds, never negative: the plants do not reverse. Only FourWheelCar uses them; the
    // other models leave them at zero.
    std::array<double, 4> wheel_speeds_radps{};
    // FourWheelCar only: each wheel's vertical load, never negative, zero on a lifted wheel. It is the
    // quasi-static balance of the centre of gravity's acceleration over the last substep and governs the
    // next one, so it is state a copy must carry (decision 0014). The other models leave them at zero.
    std::array<double, 4> wheel_loads_n{};
};

// What the plant is asked of its tires in a state, given the longitudinal acceleration the
// caller attributes to it (commanded, or achieved over a step).
struct Demand {
    double lateral_acceleration_mps2{}, longitudinal_acceleration_mps2{};
    double grip_utilization{};     // kinematic: |a| / (mu g); dynamic: largest axle friction-ellipse use
    bool within_envelope{true};    // kinematic: |a| within mu g; dynamic: both slip angles within their peak
    bool tire_slip_modeled{};      // false for the kinematic bicycle, which cannot slide
    // Axle slip angles, positive when the tires push the car to its left, and the slip angle at which
    // each axle's tires reach peak grip at the load they carry. All zero when tire slip is not modeled.
    double front_slip_angle_rad{}, rear_slip_angle_rad{};
    double front_peak_slip_angle_rad{}, rear_peak_slip_angle_rad{};
    // FourWheelCar only: each wheel's slip ratio, -1 for a locked wheel sliding forward, and its combined
    // normalised slip, above one when that tire is past its peak.
    std::array<double, 4> wheel_slip_ratios{}, wheel_combined_slip{};
    // FourWheelCar only: each tire's force in its own frame over its peak force at its present load, road
    // grip included, longitudinally and laterally. The pair stays inside the unit circle, the tire's
    // friction circle. Zero for a lifted wheel, which carries no load and puts the car outside its envelope.
    std::array<double, 4> wheel_longitudinal_use{}, wheel_lateral_use{};
};

// Handling balance from front against rear slip (decision 0011). The understeer angle is the front
// axle's slip angle minus the rear's, measured toward the turn. In a steady turn it is how much more
// the car steers than geometry, steering minus wheelbase times yaw rate over speed, which is what MAP
// steering adds. Above the deadband the car understeers, below its negative it oversteers.
enum class Balance { not_modeled, not_cornering, neutral, understeer, oversteer };
inline constexpr double balance_deadband_rad = 0.25*std::numbers::pi/180;
// Below this lateral acceleration there is no turn to be balanced in.
inline constexpr double cornering_lateral_acceleration_mps2 = 1.0;
struct HandlingBalance {
    Balance balance{Balance::not_modeled};
    double understeer_angle_rad{};  // zero unless tire slip is modeled and the car is cornering
};
HandlingBalance handling_balance(const Demand& demand);

// Only the envelope question, answered from slip angles, slip ratios or accelerations without evaluating
// any tire force: always exactly demand()'s within_envelope and tire_slip_modeled, at a fraction of its
// cost. Predictions ask it at both ends of every plant tick.
struct Envelope { bool within{true}; bool tire_slip_modeled{}; };
Envelope envelope(const VehicleModel& model, const PlantState& state, const Config& config,
                  double longitudinal_acceleration_mps2);
const char* balance_name(Balance balance);

void validate_vehicle(const VehicleModel& model, const Config& config);
PlantState plant_state_from(const State& state, const VehicleModel& model, const Config& config);
// Advances by dt with the applied command held. Validates like integrate_bicycle.
PlantState advance(const VehicleModel& model, const PlantState& state, Command applied,
                   const Config& config, double dt_s);
// The same four-wheel integrator, progressively blended toward rolling motion. No track or
// second live plant is involved. Commands must come from human_command with the same assistance.
PlantState advance_assisted(const VehicleModel& model, const PlantState& state, Command applied,
                           const Config& config, double dt_s, DrivingAssistance assistance);
Demand demand(const VehicleModel& model, const PlantState& state, const Config& config,
              double longitudinal_acceleration_mps2);
// Air at sea level and 15 °C, the standard atmosphere.
inline constexpr double air_density_kgpm3 = 1.225;
// The four-wheel car's aerodynamic force (decision 0015) for the centre of gravity's velocity in the body frame:
// drag along each axis and total downforce. Validates the car and rejects non-finite velocities.
struct AerodynamicForce { double longitudinal_n{}, lateral_n{}, downforce_n{}; };
AerodynamicForce aerodynamic_force(const FourWheelCar& car, double forward_velocity_mps, double lateral_velocity_mps);
// Quasi-static wheel loads of the four-wheel car (decisions 0014 and 0015) under a horizontal tire force at
// the ground, in the body frame at the centre of gravity, and a downforce: the four loads that balance weight
// plus downforce, the pitch and roll moments of the tire force about the centre of gravity and the downforce
// split by aero_balance_front, closed by the roll balance, with a lifted wheel clamped at zero. Zero force
// and downforce give the static loads. Validates the car and rejects non-finite or negative downforce.
std::array<double, 4> quasi_static_wheel_loads(const FourWheelCar& car, const Config& config,
                                               double longitudinal_force_n, double lateral_force_n, double downforce_n);
// Body sideslip angle at the rear axle, positive when the rear slides left of the heading.
double rear_sideslip_rad(const PlantState& state);

// The motion a single-track model integrates: its centre of gravity in the map frame and that point's velocity in the
// body frame, forward and left, with the yaw rate.
struct SingleTrackMotion {
    double x_m{}, y_m{}, yaw_rad{}, forward_mps{}, lateral_mps{}, yaw_rate_radps{};
};
// What each axle's tires are asked in a motion, before the plant saturates them: slip angles against the angle of peak
// grip at the load the axle carries, and friction use, the requested longitudinal force and the pure lateral force at that
// slip angle, each over the axle's capacity, combined as an ellipse. Above one the plant clamps the longitudinal force
// and gives up lateral force, so a use above one is what saturated forces cannot show.
struct AxleDemand {
    double front_slip_angle_rad{}, rear_slip_angle_rad{};
    double front_peak_slip_angle_rad{}, rear_peak_slip_angle_rad{};
    double front_friction_use{}, rear_friction_use{};
};
// The dynamic single-track plant's equations (decision 0008), prepared once for a car and configuration so that a
// predictive controller can evaluate them many times per decision (decision 0024). They are the plant's own: advance()
// for a DynamicSingleTrack is these derivatives integrated with RK4 at the car's substep, steering ramped toward its
// command at the configuration's rate, and exactly the kinematic bicycle below the lower blend speed.
class SingleTrackModel {
public:
    // Validates the car against the configuration.
    SingleTrackModel(const DynamicSingleTrack& car, const Config& config);
    // The rate of change of a motion under a steering angle, the rate at which that angle is moving, and a commanded
    // longitudinal acceleration, including the blend toward the kinematic bicycle at walking pace.
    SingleTrackMotion derivative(const SingleTrackMotion& motion, double steering_rad, double steering_rate_radps,
                                 double acceleration_mps2) const;
    AxleDemand axle_demand(const SingleTrackMotion& motion, double steering_rad, double acceleration_mps2) const;
    // Between the published rear-axle plant state and the centre of gravity's motion.
    SingleTrackMotion motion(const PlantState& state) const;
    PlantState plant_state(const SingleTrackMotion& motion, double steering_rad, double time_s) const;
    double mass_kg() const noexcept { return car_.mass_kg; }
    double cg_to_front_m() const noexcept { return front_m_; }
    double cg_to_rear_m() const noexcept { return car_.cg_to_rear_m; }
    const DynamicSingleTrack& car() const noexcept { return car_; }
    const Config& config() const noexcept { return config_; }
private:
    DynamicSingleTrack car_;
    Config config_;
    double front_m_{}, front_load_n_{}, rear_load_n_{};
    TireAtLoad front_tire_, rear_tire_;
    // Present only when the car's axle loads move with acceleration or speed (decision 0025).
    std::optional<PreparedTire> front_prepared_, rear_prepared_;
};
// The single-track reduction of a vehicle model, what a predictive controller predicts it with: a dynamic car is itself;
// a four-wheel car keeps its mass, yaw inertia, centre of gravity and its height, drag, downforce and aero balance, tires,
// drive split and blend speeds on one track (decision 0025), without lateral load transfer, brake bias or wheel speeds.
// The kinematic bicycle has no tires to reduce, and is
// refused: a model with tire slip steers it further than a car without slip needs, so it would turn tighter than planned.
DynamicSingleTrack reduced_single_track(const VehicleModel& model);
const char* vehicle_model_name(const VehicleModel& model);

} // namespace fd

#include "fd/simulation.hpp"
#include "fd/vehicle.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Phase 2.3: aerodynamics on the four-wheel car (decision 0015). Oracles are independent of the integrator:
// the drag and lift definitions, Newton's law with wheel inertia for a coasting car, the speed where drag
// meets the power at the wheels, the vertical balance with downforce, and the tire law beyond its
// reference loads.
namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tolerance)
        throw std::runtime_error(message+": actual="+std::to_string(actual)+", expected="+std::to_string(expected));
}
template<class Action> void rejects(Action action, const std::string& message) {
    bool rejected = false;
    try { action(); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, message);
}

const fd::Config config;
const double dynamic_pressure_per_speed_squared = 0.5*1.225;  // standard sea-level air, written out here

double weight(const fd::FourWheelCar& car) { return car.mass_kg*fd::gravity_mps2; }
double equivalent_mass(const fd::FourWheelCar& car) {
    return car.mass_kg+4*car.wheel_inertia_kgm2/(car.wheel_radius_m*car.wheel_radius_m);
}
double total(const std::array<double, 4>& loads) { return loads[0]+loads[1]+loads[2]+loads[3]; }
fd::PlantState rolling(const fd::FourWheelCar& car, double speed, double steering = 0) {
    return fd::plant_state_from({0, 0, 0, speed, steering, 0}, car, config);
}
fd::PlantState drive(const fd::FourWheelCar& car, fd::PlantState s, fd::Command command, double seconds,
                     const std::function<void(const fd::PlantState&)>& each = {}) {
    const auto ticks = static_cast<int>(std::llround(seconds/config.fixed_dt_s));
    for (int i = 0; i < ticks; ++i) {
        s = fd::advance(car, s, command, config, config.fixed_dt_s);
        if (each) each(s);
    }
    return s;
}

void aerodynamic_force_follows_its_definition() {
    fd::FourWheelCar car;
    car.drag_area_m2 = 0.8;
    car.downforce_area_m2 = 2.5;
    car.aero_balance_front = 0.4;
    const double q = dynamic_pressure_per_speed_squared;
    for (const auto& [vx, vy] : std::vector<std::pair<double, double>>{{20, 0}, {30, 2}, {15, -3}, {0.5, 0.1}}) {
        const auto f = fd::aerodynamic_force(car, vx, vy);
        const double speed = std::hypot(vx, vy);
        near(f.longitudinal_n, -q*0.8*speed*vx, 1e-9, "drag opposes forward motion with speed times velocity");
        near(f.lateral_n, -q*0.8*speed*vy, 1e-9, "drag opposes sideways motion too");
        near(f.downforce_n, q*2.5*vx*vx, 1e-9, "downforce grows with forward speed squared");
    }
    near(fd::aerodynamic_force(car, 30, 0).downforce_n/fd::aerodynamic_force(car, 15, 0).downforce_n, 4, 1e-12,
         "doubling speed quadruples downforce");
    const auto still = fd::aerodynamic_force(car, 0, 0);
    require(still.longitudinal_n == 0 && still.lateral_n == 0 && still.downforce_n == 0, "a car at rest feels no air");
    rejects([&] { fd::aerodynamic_force(car, std::numeric_limits<double>::quiet_NaN(), 0); }, "a non-finite velocity is rejected");

    // The downforce reaches the wheels as its balance says.
    const auto without = fd::quasi_static_wheel_loads(car, config, 0, 0, 0);
    const auto with = fd::quasi_static_wheel_loads(car, config, 0, 0, 2000);
    near(total(with)-total(without), 2000, 1e-9, "downforce adds to the load on the wheels");
    near(with[fd::front_left]+with[fd::front_right]-without[fd::front_left]-without[fd::front_right], 0.4*2000, 1e-9,
         "the front axle takes the aero balance's share of downforce");
    near(with[fd::front_left], with[fd::front_right], 1e-9, "downforce loads both sides of an axle alike");
    rejects([&] { fd::quasi_static_wheel_loads(car, config, 0, 0, -1); }, "negative downforce is rejected");
}

void coasting_decelerates_at_drag_over_equivalent_mass() {
    const fd::FourWheelCar car;
    const auto start = drive(car, rolling(car, 30), {0, 0}, 0.2);
    const auto end = drive(car, start, {0, 0}, 0.5);
    const double v_mid = 0.5*(start.pose.speed_mps+end.pose.speed_mps);
    const double measured = (start.pose.speed_mps-end.pose.speed_mps)/0.5;
    // Drag slows the chassis and, through the tires, the four wheels.
    const double expected = dynamic_pressure_per_speed_squared*car.drag_area_m2*v_mid*v_mid/equivalent_mass(car);
    std::cout << "  coasting at " << v_mid << " m/s: deceleration " << measured << " m/s^2 against drag over equivalent mass "
              << expected << "\n";
    near(measured, expected, 0.01*expected, "a coasting car decelerates at drag over mass plus wheel inertia");
    fd::FourWheelCar slippery = car;
    slippery.drag_area_m2 = 0;
    near(drive(slippery, rolling(slippery, 30), {0, 0}, 0.7).pose.speed_mps, 30, 1e-6, "without drag a coasting car keeps its speed");
}

void top_speed_is_where_drag_meets_the_power_at_the_wheels() {
    for (const double area : {0.6, 1.2}) {
        fd::FourWheelCar car;
        car.drag_area_m2 = area;
        double previous = 0;
        auto s = drive(car, rolling(car, 45), {6, 0}, 89.0);
        previous = s.pose.speed_mps;
        s = drive(car, s, {6, 0}, 1.0);
        const double top = std::cbrt(car.max_drive_power_w/(dynamic_pressure_per_speed_squared*area));
        std::cout << "  drag area " << area << " m^2: speed after 90 s " << s.pose.speed_mps << " m/s, still gaining "
                  << s.pose.speed_mps-previous << " m/s per second; power equals drag times speed at " << top << " m/s\n";
        near(s.pose.speed_mps, top, 0.01*top, "full throttle settles where drag times speed equals the power limit");
        require(s.pose.speed_mps < top && s.pose.speed_mps-previous < 0.05, "the car approaches that speed from below and has nearly reached it");
    }
}

void downforce_loads_the_wheels_in_motion() {
    fd::FourWheelCar car;
    car.downforce_area_m2 = 3;
    car.aero_balance_front = 0.4;
    bool balanced = true;
    double largest = 0;
    drive(car, rolling(car, 40), {0, 0.01}, 1.0, [&](const fd::PlantState& s) {
        const double downforce = dynamic_pressure_per_speed_squared*car.downforce_area_m2*s.pose.speed_mps*s.pose.speed_mps;
        balanced = balanced && std::abs(total(s.wheel_loads_n)-weight(car)-downforce) < 1e-9*weight(car);
        largest = std::max(largest, downforce);
    });
    std::cout << "  downforce up to " << largest << " N at 40 m/s\n";
    require(balanced, "at every tick the wheels carry weight plus the downforce at the car's speed");
    require(largest > 2800, "the downforce is a third of the car's weight at 40 m/s");
}

// The largest lateral acceleration of a settled turn with every tire inside its peak, at a held speed.
double largest_steady_turn(const fd::FourWheelCar& car, double speed) {
    const double widest = std::atan(config.wheelbase_m*1.6*fd::gravity_mps2/(speed*speed));
    double largest = 0;
    for (int k = 1; k <= 40; ++k) {
        const double steer = widest*k/40;
        auto s = rolling(car, speed, 0);
        double earlier_rate = 0;
        const int ticks = 700;
        for (int i = 0; i < ticks; ++i) {
            if (i == ticks-50) earlier_rate = s.yaw_rate_radps;
            s = fd::advance(car, s, {4*(speed-s.pose.speed_mps), steer}, config, config.fixed_dt_s);
        }
        const auto d = fd::demand(car, s, config, 0);
        if (d.within_envelope && std::abs(s.yaw_rate_radps-earlier_rate) < 2e-3 && std::abs(s.pose.speed_mps-speed) < 0.05*speed)
            largest = std::max(largest, std::abs(d.lateral_acceleration_mps2));
    }
    return largest;
}
void downforce_raises_corner_speed_only_with_downforce() {
    fd::FourWheelCar plain;
    fd::FourWheelCar winged;
    winged.downforce_area_m2 = 3;
    std::array<double, 3> plain_turns{}, winged_turns{};
    const std::array<double, 3> speeds{10, 20, 30};
    for (std::size_t i = 0; i < speeds.size(); ++i) {
        plain_turns[i] = largest_steady_turn(plain, speeds[i]);
        winged_turns[i] = largest_steady_turn(winged, speeds[i]);
        std::cout << "  " << speeds[i] << " m/s: largest steady turn without downforce " << plain_turns[i] << " m/s^2, with "
                  << winged.downforce_area_m2 << " m^2 of downforce area " << winged_turns[i] << " m/s^2\n";
    }
    require(plain_turns[0] > 6 && winged_turns[0] > 6, "both cars corner firmly at 10 m/s");
    require(plain_turns[2] < 1.03*plain_turns[0], "without downforce the cornering limit does not rise with speed");
    require(winged_turns[2] > 1.10*winged_turns[0] && winged_turns[1] > winged_turns[0], "with downforce it rises with speed");
    require(winged_turns[2] > 1.10*plain_turns[2], "at 30 m/s the downforce car corners measurably harder");
}

// TrackWayFastPlan section 4.1(a): friction extrapolates beyond the upper reference load, so a high-downforce
// car must be tested above 6000 N per wheel.
void loads_beyond_the_tire_reference_range_stay_physical() {
    fd::FourWheelCar car;
    car.downforce_area_m2 = 4;
    car.drag_area_m2 = 1.0;
    auto s = rolling(car, 85, 0.0005);
    const double upper = car.front_tire.reference_load_high_n;
    require(std::all_of(s.wheel_loads_n.begin(), s.wheel_loads_n.end(), [&](double load) { return load > upper; }),
            "at 85 m/s every wheel carries more than the tire's upper reference load");
    bool physical = true;
    const double v0 = s.pose.speed_mps;
    const double heaviest = *std::max_element(s.wheel_loads_n.begin(), s.wheel_loads_n.end());
    drive(car, s, {0, 0.0005}, 1.0, [&](const fd::PlantState& p) {
        const double downforce = dynamic_pressure_per_speed_squared*car.downforce_area_m2*p.pose.speed_mps*p.pose.speed_mps;
        physical = physical && std::isfinite(p.pose.speed_mps) && std::abs(total(p.wheel_loads_n)-weight(car)-downforce) < 1e-9*(weight(car)+downforce);
        for (const double load : p.wheel_loads_n) physical = physical && std::isfinite(load) && load > 0;
        const auto d = fd::demand(car, p, config, 0);
        physical = physical && d.within_envelope;
        s = p;
    });
    const auto peak = fd::tire_peak(car.front_tire, heaviest);
    std::cout << "  at " << v0 << " m/s the heaviest wheel carries " << heaviest << " N with peak lateral friction " << peak.friction_lateral
              << "; a second of coasting later the car is at " << s.pose.speed_mps << " m/s\n";
    require(physical, "loads, speed and the tire envelope stay physical beyond the reference loads");
    require(s.pose.speed_mps < v0, "drag slows the coasting car");
    require(peak.friction_lateral < car.front_tire.peak_friction_lateral_high && peak.friction_lateral >= car.front_tire.minimum_friction,
            "friction beyond the upper reference load keeps falling and never below its floor");
}

void invalid_aerodynamics_and_coupling_are_rejected() {
    const auto broken = [&](const std::function<void(fd::FourWheelCar&)>& change, const std::string& what) {
        fd::FourWheelCar car;
        change(car);
        rejects([&] { fd::validate_vehicle(car, config); }, what+" is rejected");
        rejects([&] { fd::aerodynamic_force(car, 10, 0); }, what+" is rejected by the force query");
        rejects([&] { fd::advance(car, rolling(fd::FourWheelCar{}, 5), {}, config, config.fixed_dt_s); }, what+" is rejected when advancing");
    };
    broken([](auto& c) { c.drag_area_m2 = -0.1; }, "a negative drag area");
    broken([](auto& c) { c.drag_area_m2 = 11; }, "an implausible drag area");
    broken([](auto& c) { c.downforce_area_m2 = -0.5; }, "lift instead of downforce");
    broken([](auto& c) { c.downforce_area_m2 = std::numeric_limits<double>::infinity(); }, "a non-finite downforce area");
    broken([](auto& c) { c.aero_balance_front = 1.2; }, "an aero balance beyond the front axle");
    broken([](auto& c) { c.aero_balance_front = std::numeric_limits<double>::quiet_NaN(); }, "a non-finite aero balance");
    broken([](auto& c) { c.viscous_coupling_nms = -1; }, "a negative viscous coupling");
    broken([](auto& c) { c.viscous_coupling_nms = 250; }, "a viscous coupling beyond the supported range");
    fd::FourWheelCar edge;
    edge.drag_area_m2 = 0;
    edge.downforce_area_m2 = 0;
    edge.aero_balance_front = 1;
    edge.viscous_coupling_nms = 200;
    fd::validate_vehicle(edge, config);
    require(std::string(fd::vehicle_model_name(fd::FourWheelCar{})) ==
            "four-wheel car with rotating wheels, load transfer and aerodynamics, rear axle reference", "the default car names its aerodynamics");
    require(std::string(fd::vehicle_model_name(edge)) == "four-wheel car with rotating wheels and load transfer, rear axle reference",
            "without drag or downforce the car keeps decision 0014's name");
    edge.cg_height_m = 0;
    edge.drag_area_m2 = 0.3;
    require(std::string(fd::vehicle_model_name(edge)) == "four-wheel car with rotating wheels and aerodynamics, rear axle reference",
            "aerodynamics without load transfer is named so");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"aerodynamic_force_follows_its_definition", aerodynamic_force_follows_its_definition},
        {"coasting_decelerates_at_drag_over_equivalent_mass", coasting_decelerates_at_drag_over_equivalent_mass},
        {"top_speed_is_where_drag_meets_the_power_at_the_wheels", top_speed_is_where_drag_meets_the_power_at_the_wheels},
        {"downforce_loads_the_wheels_in_motion", downforce_loads_the_wheels_in_motion},
        {"downforce_raises_corner_speed_only_with_downforce", downforce_raises_corner_speed_only_with_downforce},
        {"loads_beyond_the_tire_reference_range_stay_physical", loads_beyond_the_tire_reference_range_stay_physical},
        {"invalid_aerodynamics_and_coupling_are_rejected", invalid_aerodynamics_and_coupling_are_rejected},
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " aerodynamics groups passed\n";
    return failures ? 1 : 0;
}

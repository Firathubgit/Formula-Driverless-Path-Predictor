#include "fd/setup.hpp"
#include "fd/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Phase 2.4 (decision 0016): every vehicle parameter an application exposes changes a measured outcome in a
// documented direction on one fixed scenario, or it is not exposed. The scenario is the first 12 s of the preset
// circuit from rest under the unchanged planner and Pure Pursuit: it accelerates onto the first straight, brakes
// into the first corner and turns through it. Each group drives it with the parameter at two values.
namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
template<class Action> void rejects(Action action, const std::string& message) {
    bool rejected = false;
    try { action(); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, message);
}

const fd::Config config;
constexpr double scenario_seconds = 12.0;

struct Outcome {
    double distance_m{}, time_to_15_mps_s{-1}, largest_grip_use{}, largest_front_axle_load_n{};
    double deepest_front_braking_slip{}, deepest_rear_braking_slip{};
    int rear_locked_samples{};
    double largest_driven_wheel_spread_radps{}, largest_understeer_angle_rad{};
};
Outcome drive_scenario(const fd::FourWheelCar& car) {
    fd::Simulation simulation(fd::make_preset_track(), config, car);
    Outcome o;
    while (simulation.state().time_s < scenario_seconds-1e-9) {
        simulation.step();
        const auto& d = simulation.diagnostics();
        const auto& s = simulation.plant_state();
        const auto demand = fd::demand(car, s, config, d.applied.acceleration_mps2);
        o.largest_grip_use = std::max(o.largest_grip_use, d.combined_grip_utilization);
        o.largest_front_axle_load_n = std::max(o.largest_front_axle_load_n, s.wheel_loads_n[fd::front_left]+s.wheel_loads_n[fd::front_right]);
        o.deepest_front_braking_slip = std::min({o.deepest_front_braking_slip, demand.wheel_slip_ratios[fd::front_left], demand.wheel_slip_ratios[fd::front_right]});
        o.deepest_rear_braking_slip = std::min({o.deepest_rear_braking_slip, demand.wheel_slip_ratios[fd::rear_left], demand.wheel_slip_ratios[fd::rear_right]});
        if (s.pose.speed_mps > car.slip_speed_floor_mps && (s.wheel_speeds_radps[fd::rear_left] == 0 || s.wheel_speeds_radps[fd::rear_right] == 0))
            ++o.rear_locked_samples;
        if (car.drive_front_fraction > 0)
            o.largest_driven_wheel_spread_radps = std::max(o.largest_driven_wheel_spread_radps,
                                                           std::abs(s.wheel_speeds_radps[fd::front_left]-s.wheel_speeds_radps[fd::front_right]));
        if (car.drive_front_fraction < 1)
            o.largest_driven_wheel_spread_radps = std::max(o.largest_driven_wheel_spread_radps,
                                                           std::abs(s.wheel_speeds_radps[fd::rear_left]-s.wheel_speeds_radps[fd::rear_right]));
        o.largest_understeer_angle_rad = std::max(o.largest_understeer_angle_rad, d.understeer_angle_rad);
        if (o.time_to_15_mps_s < 0 && s.pose.speed_mps >= 15) o.time_to_15_mps_s = s.pose.time_s;
        o.distance_m = d.progress_m;
    }
    return o;
}
fd::FourWheelCar car_with(const std::function<void(fd::FourWheelCar&)>& change) {
    fd::FourWheelCar car;
    change(car);
    fd::validate_vehicle(car, config);
    return car;
}
void report(const std::string& what, const Outcome& o) {
    std::cout << "  " << what << ": " << o.distance_m << " m in 12 s, 15 m/s at " << o.time_to_15_mps_s << " s, grip use " << o.largest_grip_use
              << ", front axle up to " << o.largest_front_axle_load_n << " N, braking slip front " << o.deepest_front_braking_slip << " rear "
              << o.deepest_rear_braking_slip << ", " << o.rear_locked_samples << " samples rear locked, driven wheels up to "
              << o.largest_driven_wheel_spread_radps << " rad/s apart, understeer angle up to " << o.largest_understeer_angle_rad << " rad\n";
}

// The controls the applications offer, through the registry, at two of their offered values.
fd::FourWheelCar offered(std::string_view key, double value) { return fd::with_setup_value(fd::FourWheelCar{}, config, key, value); }

void mass_raises_grip_use() {
    const auto light = drive_scenario(offered("mass_kg", 600));
    const auto heavy = drive_scenario(offered("mass_kg", 1400));
    report("600 kg", light);
    report("1400 kg", heavy);
    require(heavy.largest_grip_use > light.largest_grip_use+0.03,
            "a heavier car uses more of its grip for the same plan, because tire friction falls as load rises");
}

void centre_of_gravity_height_moves_load_forward_and_deepens_rear_braking_slip() {
    const auto low = drive_scenario(offered("cg_height_m", 0.1));
    const auto tall = drive_scenario(offered("cg_height_m", 0.6));
    report("0.1 m high", low);
    report("0.6 m high", tall);
    require(tall.largest_front_axle_load_n > low.largest_front_axle_load_n+500, "a taller car puts more load on its front axle under braking");
    require(tall.deepest_rear_braking_slip < 2*low.deepest_rear_braking_slip, "and its unloaded rear wheels slip deeper at the same brake bias");
}

void brake_bias_decides_which_axle_slips_and_locks() {
    const auto rearward = drive_scenario(offered("brake_bias_front", 0.2));
    const auto forward = drive_scenario(offered("brake_bias_front", 0.8));
    report("brake bias front 0.2", rearward);
    report("brake bias front 0.8", forward);
    require(rearward.rear_locked_samples > 0 && forward.rear_locked_samples == 0, "a rearward bias locks the rear wheels braking into the corner");
    require(rearward.deepest_rear_braking_slip < rearward.deepest_front_braking_slip &&
            forward.deepest_front_braking_slip < forward.deepest_rear_braking_slip,
            "the deepest braking slip is on the axle the bias favours");
}

void drag_area_costs_distance() {
    const auto slippery = drive_scenario(offered("drag_area_m2", 0));
    const auto draggy = drive_scenario(offered("drag_area_m2", 1.5));
    report("no drag", slippery);
    report("1.5 m^2 of drag area", draggy);
    require(draggy.distance_m < slippery.distance_m-1, "more drag covers less distance in the same time");
}

void downforce_area_lowers_grip_use() {
    const auto plain = drive_scenario(offered("downforce_area_m2", 0));
    const auto winged = drive_scenario(offered("downforce_area_m2", 4));
    report("no downforce", plain);
    report("4 m^2 of downforce area", winged);
    require(winged.largest_grip_use < plain.largest_grip_use-0.02, "downforce leaves more grip unused for the same plan");
}

void power_limit_slows_the_pull_away() {
    const auto weak = drive_scenario(offered("max_drive_power_w", 40000));
    const auto strong = drive_scenario(offered("max_drive_power_w", 100000));
    report("40 kW", weak);
    report("100 kW", strong);
    require(weak.time_to_15_mps_s > strong.time_to_15_mps_s+0.2, "less power reaches 15 m/s later");
}

void viscous_coupling_holds_driven_wheels_together_and_resists_turning() {
    const auto open = drive_scenario(offered("viscous_coupling_nms", 0));
    const auto coupled = drive_scenario(offered("viscous_coupling_nms", 80));
    report("open differentials", open);
    report("80 N m s/rad coupling", coupled);
    require(coupled.largest_driven_wheel_spread_radps < 0.8*open.largest_driven_wheel_spread_radps,
            "a coupling keeps the driven wheels of each axle closer in speed");
    require(coupled.largest_understeer_angle_rad > 2*open.largest_understeer_angle_rad, "and by resisting their difference it makes the car understeer");
}

// Parameters only the headless runner offers are held to the same rule.
void roll_balance_forward_lowers_grip_use() {
    const auto rear = drive_scenario(car_with([](auto& c) { c.roll_balance_front = 0.2; }));
    const auto front = drive_scenario(car_with([](auto& c) { c.roll_balance_front = 0.8; }));
    report("roll balance front 0.2", rear);
    report("roll balance front 0.8", front);
    require(front.largest_grip_use < rear.largest_grip_use-0.05, "moving roll balance forward lowers the largest grip use on this scenario");
}

void aero_balance_forward_loads_the_front_axle() {
    const auto rear = drive_scenario(car_with([](auto& c) { c.downforce_area_m2 = 4; c.aero_balance_front = 0.2; }));
    const auto front = drive_scenario(car_with([](auto& c) { c.downforce_area_m2 = 4; c.aero_balance_front = 0.8; }));
    report("aero balance front 0.2", rear);
    report("aero balance front 0.8", front);
    require(front.largest_front_axle_load_n > rear.largest_front_axle_load_n+300, "moving aero balance forward puts more downforce on the front axle");
}

void drive_split_forward_slows_the_pull_away() {
    const auto rear = drive_scenario(car_with([](auto& c) { c.drive_front_fraction = 0; }));
    const auto front = drive_scenario(car_with([](auto& c) { c.drive_front_fraction = 1; }));
    report("rear-wheel drive", rear);
    report("front-wheel drive", front);
    require(front.time_to_15_mps_s > rear.time_to_15_mps_s+0.5, "accelerating moves load off the front wheels, so front-wheel drive pulls away more slowly");
}

const std::set<std::string> tested_offered_controls{"mass_kg", "cg_height_m", "brake_bias_front", "drag_area_m2", "downforce_area_m2",
                                                   "max_drive_power_w", "viscous_coupling_nms"};

void the_registry_offers_only_tested_controls_within_their_valid_ranges() {
    const auto controls = fd::four_wheel_setup_controls();
    std::set<std::string> offered_keys;
    const fd::FourWheelCar standard;
    for (const auto& c : controls) {
        const std::string key(c.key);
        offered_keys.insert(key);
        require(tested_offered_controls.contains(key), "every offered control has an effect test: "+key);
        require(!c.label.empty() && !c.unit.empty() && !c.effect.empty() && c.display_scale > 0, key+" is described for people");
        require(c.minimum < c.maximum && c.step > 0 && c.step <= c.maximum-c.minimum, key+" has an ordered range and a step inside it");
        const double value = fd::setup_value(standard, key);
        require(value >= c.minimum && value <= c.maximum, key+" offers the default car's value");
        require(fd::setup_value(fd::with_setup_value(standard, config, key, c.minimum), key) == c.minimum &&
                fd::setup_value(fd::with_setup_value(standard, config, key, c.maximum), key) == c.maximum, key+" reads back both ends of its range");
        rejects([&] { fd::with_setup_value(standard, config, key, c.minimum-c.step); }, key+" below its offered range is refused");
        rejects([&] { fd::with_setup_value(standard, config, key, c.maximum+c.step); }, key+" above its offered range is refused");
        rejects([&] { fd::with_setup_value(standard, config, key, std::numeric_limits<double>::quiet_NaN()); }, key+" refuses a non-finite value");
    }
    require(offered_keys == tested_offered_controls, "the offered controls are exactly the plan's list: mass, centre of gravity height, brake bias, "
                                                     "drag and downforce areas, power and the differential");
    rejects([&] { fd::setup_control("roll_balance_front"); }, "a parameter that is not offered is not a setup control");
    rejects([&] { fd::with_setup_value(standard, config, "wheel_radius_m", 0.3); }, "an unknown key is refused");
    const auto heavy = fd::with_setup_value(standard, config, "mass_kg", 1200);
    require(heavy.mass_kg == 1200 && std::abs(heavy.yaw_inertia_kgm2/heavy.mass_kg-standard.yaw_inertia_kgm2/standard.mass_kg) < 1e-12,
            "changing mass keeps the mass distribution: yaw inertia scales with it");
    // Offered values can still make an invalid car: the centre of gravity height is limited by the narrower track too.
    auto narrow = standard;
    narrow.track_rear_m = 1.0;
    rejects([&] { fd::with_setup_value(narrow, config, "cg_height_m", 0.6); }, "a setup value the car cannot take is refused by its validation");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"the_registry_offers_only_tested_controls_within_their_valid_ranges", the_registry_offers_only_tested_controls_within_their_valid_ranges},
        {"mass_raises_grip_use", mass_raises_grip_use},
        {"centre_of_gravity_height_moves_load_forward_and_deepens_rear_braking_slip", centre_of_gravity_height_moves_load_forward_and_deepens_rear_braking_slip},
        {"brake_bias_decides_which_axle_slips_and_locks", brake_bias_decides_which_axle_slips_and_locks},
        {"drag_area_costs_distance", drag_area_costs_distance},
        {"downforce_area_lowers_grip_use", downforce_area_lowers_grip_use},
        {"power_limit_slows_the_pull_away", power_limit_slows_the_pull_away},
        {"viscous_coupling_holds_driven_wheels_together_and_resists_turning", viscous_coupling_holds_driven_wheels_together_and_resists_turning},
        {"roll_balance_forward_lowers_grip_use", roll_balance_forward_lowers_grip_use},
        {"aero_balance_forward_loads_the_front_axle", aero_balance_forward_loads_the_front_axle},
        {"drive_split_forward_slows_the_pull_away", drive_split_forward_slows_the_pull_away},
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " setup effect groups passed\n";
    return failures ? 1 : 0;
}

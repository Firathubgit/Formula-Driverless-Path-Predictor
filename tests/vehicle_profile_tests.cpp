#include "fd/performance_envelope.hpp"
#include "fd/simulation.hpp"
#include "fd/vehicle_profiles.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

// Every profile is a car the model accepts, with a source and the provenance of its values; ids are unique and an
// unknown one is refused naming the others.
void every_profile_is_a_sourced_car() {
    std::set<std::string> ids;
    for (const auto& p : fd::vehicle_profiles()) {
        require(ids.insert(p.id).second, p.id+": ids are unique");
        require(!p.name.empty() && !p.source.empty() && !p.parameters.empty(), p.id+": named, sourced and its values accounted for");
        const auto config = fd::configured_for(p, fd::Config{});
        require(config.wheelbase_m == p.wheelbase_m && config.max_steering_rad == p.max_steering_rad && config.max_speed_mps == p.max_speed_mps,
                p.id+": its configuration members are set");
        std::map<fd::Provenance, int> counts;
        for (const auto& s : p.parameters) { ++counts[s.provenance]; require(!s.note.empty() && !s.parameter.empty(), p.id+": each value says why"); }
        std::cout << "  " << p.id << ": " << p.car.mass_kg << " kg, " << p.car.max_drive_power_w/1000 << " kW, wheelbase " << p.wheelbase_m
                  << " m; " << counts[fd::Provenance::published] << " published, " << counts[fd::Provenance::derived] << " derived, "
                  << counts[fd::Provenance::fitted] << " fitted, " << counts[fd::Provenance::chosen] << " chosen\n";
    }
    require(ids.size() == 6, "six profiles: the project's roadster, four from the audited sources and the Göteborg rental kart");
    require(ids.count("road-car") == 0, "no road car, which no audited source describes");
    bool named = false;
    try { fd::vehicle_profile("road-car"); } catch (const std::invalid_argument& e) { named = std::string(e.what()).find("formula-one-style") != std::string::npos; }
    require(named, "an unknown profile is refused, naming the ones there are");
    require(fd::vehicle_profile("project-roadster").car.mass_kg == fd::FourWheelCar{}.mass_kg, "the roadster is the project's own car");
}

// What each profile's source says, as the profile carries it: spot values from each file, and the arithmetic of the
// derived ones.
void the_profiles_carry_their_sources_values() {
    const auto& f1 = fd::vehicle_profile("formula-one-style");
    require(f1.car.mass_kg == 660 && f1.car.yaw_inertia_kgm2 == 450 && std::abs(f1.wheelbase_m-3.4) < 1e-12 && f1.car.cg_to_rear_m == 1.6 &&
            f1.car.max_drive_power_w == 735499 && f1.car.brake_bias_front == 0.6 && f1.car.viscous_coupling_nms == 10.47,
            "Limebeer's car: 660 kg, Izz 450, 1.8 + 1.6 m, 735.5 kW, bias 0.6, differential 10.47");
    require(std::abs(f1.car.drag_area_m2-1.35) < 1e-12 && std::abs(f1.car.downforce_area_m2-4.5) < 1e-12 &&
            std::abs(f1.car.aero_balance_front-1.5/3.4) < 1e-12, "cd 0.9 and cl 3.0 on 1.5 m^2, the pressure centre 0.1 m behind");
    const auto& fs = fd::vehicle_profile("formula-student-electric");
    require(fs.car.mass_kg == 178 && std::abs(fs.wheelbase_m-1.44) < 1e-12 && fs.car.track_front_m == 1.15 && fs.car.wheel_radius_m == 0.206 &&
            std::abs(fs.car.downforce_area_m2-4.07) < 1e-12 && fs.car.front_tire.peak_friction_lateral_low == 1.6,
            "PacSim's car: 178 kg, 0.72 + 0.72 m, 1.15 m track, cla 3.7 on 1.1 m^2, D 1.6");
    const auto& tum = fd::vehicle_profile("electric-race-car");
    require(tum.car.mass_kg == 1200 && std::abs(tum.wheelbase_m-3.0) < 1e-12 && tum.car.cg_height_m == 0.38 && tum.max_steering_rad == 0.35 &&
            tum.car.max_drive_power_w == 230000 && std::abs(tum.car.max_drive_torque_nm-1050) < 1e-9 &&
            std::abs(tum.car.front_tire.peak_friction_lateral_low-(1-0.1*2000.0/3000.0)) < 1e-12 &&
            std::abs(tum.car.front_tire.peak_friction_lateral_high-0.8) < 1e-12 && tum.car.front_tire.peak_friction_longitudinal_high == 1.0,
            "TUM's car: 1200 kg, 1.6 + 1.4 m, 0.38 m, 0.35 rad, 230 kW, 7 kN; lateral friction 1 + eps Fz / f_z0 as its solver "
            "computes it, longitudinal its friction circle's");
    const auto& kart = fd::vehicle_profile("racing-kart");
    require(kart.car.mass_kg == 165 && std::abs(kart.wheelbase_m-1.045) < 1e-12 && kart.car.brake_bias_front == 0 &&
            kart.car.downforce_area_m2 == 0 && kart.car.max_drive_power_w == 20100, "Lot's kart: 165 kg, 0.645 + 0.4 m, rear brakes, no wings, 20.1 kW");
    // The Göteborg rental kart (decision 0038): Sodikart's 186 kg with a 75 kg driver, the operator's 14 hp and 60 km/h, on Lot's
    // geometry with rental tires, and no other car of the project limited by its drive controller.
    const auto& rental = fd::vehicle_profile("gokartcentralen-rsx2");
    require(rental.car.mass_kg == 261 && std::abs(rental.car.max_drive_power_w-14*735.49875) < 1e-9 &&
            std::abs(rental.car.max_drive_speed_mps-60/3.6) < 1e-12 && std::abs(rental.max_speed_mps-60/3.6) < 1e-12 &&
            std::abs(rental.wheelbase_m-kart.wheelbase_m) < 1e-12 && rental.car.drive_front_fraction == 0 &&
            rental.car.front_tire.peak_friction_lateral_low == 1.0 && rental.car.rear_tire.peak_friction_longitudinal_high == 1.0 &&
            std::abs(rental.car.yaw_inertia_kgm2-25*261.0/165) < 1e-9,
            "the rental kart: 186 + 75 kg, 14 hp, 60 km/h, the racing kart's geometry, friction 1.0");
    for (const auto& p : fd::vehicle_profiles())
        if (p.id != "gokartcentralen-rsx2") require(!std::isfinite(p.car.max_drive_speed_mps), p.id+": no drive controller top speed");
}

// The profiles behave as their physics must, at the same speeds for every car: a car with wings gains lateral grip from
// 15 to 30 m/s, above the speeds where the steering lock rather than the tires bounds a turn, and one without does not
// (the heavy TUM car only 6%, its tires losing friction as its load grows); the 20 kW kart drives weakest at 25 m/s; and
// on the preset the Formula One car's envelope plan laps faster than the heavier or less powerful cars'. The light Formula Student car, whose downforce is
// larger for its mass than the Formula One car's, grips hardest and laps quickest of all on this tight track. Every
// profile drives ten seconds of the preset from rest on its own envelope plan, as the judge sees it, on the course.
void the_profiles_differ_as_their_physics_must() {
    std::map<std::string, double> gain, forward_25, lap;
    const auto track = fd::make_preset_track();
    for (const auto& p : fd::vehicle_profiles()) {
        const auto config = fd::configured_for(p, fd::Config{});
        const fd::PerformanceEnvelope envelope(p.car, config);
        // A car whose drive controller stops it short of 30 m/s, the rental kart, is compared at half and nine tenths of that.
        const double top = p.car.max_drive_speed_mps;
        const double low = std::isfinite(top) ? 0.5*top : 15.0, high = std::isfinite(top) ? 0.9*top : 30.0;
        const double slow = envelope.lateral_limit(low, fd::TurnSide::left), fast = envelope.lateral_limit(high, fd::TurnSide::left);
        gain[p.id] = fast/slow;
        forward_25[p.id] = envelope.forward_limit(25.0, 0.0);
        lap[p.id] = fd::estimated_lap_time(track, fd::make_speed_plan(track, config, &envelope));
        fd::Simulation simulation(track, config, p.car, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::performance_envelope);
        for (int k = 0; k < 2000; ++k) simulation.step();
        const auto& judge = simulation.judge();
        std::cout << "  " << std::setw(26) << std::left << p.id << std::right << std::fixed << std::setprecision(2) << " lateral "
                  << slow << " m/s^2 at " << low << " m/s and " << fast << " at " << high << ", forward at 25 m/s " << forward_25[p.id] << " m/s^2, estimated lap "
                  << lap[p.id] << " s; ten seconds from rest: " << simulation.state().speed_mps << " m/s, " << judge.cones_hit()
                  << " cones, " << judge.off_course_count() << " off course\n" << std::defaultfloat;
        require(simulation.state().speed_mps > 5 && judge.off_course_count() == 0 && !judge.dnf(), p.id+": drives off from rest and stays on course");
        if (p.car.downforce_area_m2 > 0) require(gain[p.id] > 1.02, p.id+": its wings add lateral grip with speed");
        else require(gain[p.id] < 1.02, p.id+": without wings speed adds no lateral grip");
    }
    for (const auto& [id, value] : forward_25)
        if (id != "racing-kart" && id != "gokartcentralen-rsx2")
            require(forward_25["racing-kart"] < value, "the 20 kW kart drives weakest at 25 m/s, under "+id);
    // The rental kart floored from rest reaches the operator's 60 km/h in about the time its power allows and holds just
    // under it (decision 0038); in its hairpin speeds it corners at its tires' grip, not scrubbed by a locked axle.
    {
        const auto& rental = fd::vehicle_profile("gokartcentralen-rsx2");
        const auto config = fd::configured_for(rental, fd::Config{});
        auto s = fd::plant_state_from({0, 0, 0, 0, 0, 0}, rental.car, config);
        double to_50 = -1;
        for (int k = 0; k < 15*200; ++k) {
            s = fd::advance(rental.car, s, {fd::gravity_mps2, 0}, config, config.fixed_dt_s);
            if (to_50 < 0 && s.pose.speed_mps >= 50/3.6) to_50 = (k+1)*config.fixed_dt_s;
        }
        std::cout << "  rental kart floored: 50 km/h after " << to_50 << " s, " << s.pose.speed_mps*3.6 << " km/h after 15 s\n";
        require(s.pose.speed_mps > 58/3.6 && s.pose.speed_mps <= 60/3.6+1e-9, "the rental kart tops out just under 60 km/h");
        require(to_50 > 2 && to_50 < 5, "and reaches 50 km/h in two to five seconds, as 10 kW moves 261 kg");
        const fd::PerformanceEnvelope envelope(rental.car, config);
        require(envelope.lateral_limit(8.0, fd::TurnSide::left) > 0.85*fd::gravity_mps2, "at 8 m/s it corners at its tires' grip");
    }
    for (const char* id : {"project-roadster", "electric-race-car", "racing-kart"})
        require(lap["formula-one-style"] < lap[id], std::string("the Formula One car's plan laps faster than the ")+id+"'s");
    for (const auto& [id, value] : lap)
        if (id != "formula-student-electric") require(lap["formula-student-electric"] < value, "the Formula Student car laps the tight preset quickest, before "+id);
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"every_profile_is_a_sourced_car", every_profile_is_a_sourced_car},
        {"the_profiles_carry_their_sources_values", the_profiles_carry_their_sources_values},
        {"the_profiles_differ_as_their_physics_must", the_profiles_differ_as_their_physics_must}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " vehicle profile groups passed\n";
    return failures ? 1 : 0;
}

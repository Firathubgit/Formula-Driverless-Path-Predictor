#include "fd/setup.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace fd {
namespace {

// Offered ranges sit inside validate_vehicle's and cover what the plan and controller drive sensibly on the
// preset. Effects are the directions tests/setup_tests.cpp measures on its fixed scenario.
constexpr std::array<SetupControl, 7> controls{{
    {"mass_kg", "Mass", "kg", 1, 600, 1400, 10,
     "Heavier uses more of the tires' grip for the same plan, because friction falls as load rises"},
    {"cg_height_m", "Centre of gravity height", "m", 1, 0, 0.6, 0.01,
     "Taller moves more load onto the front axle under braking and lets the rear wheels slip deeper"},
    {"brake_bias_front", "Brake bias front", "%", 100, 0.2, 0.8, 0.01,
     "Rearward locks the rear wheels braking into a corner; forward moves braking slip to the front"},
    {"drag_area_m2", "Drag area", "m²", 1, 0, 1.5, 0.05,
     "More drag covers less distance in the same time"},
    {"downforce_area_m2", "Downforce area", "m²", 1, 0, 4, 0.1,
     "More downforce leaves more grip unused for the same plan"},
    {"max_drive_power_w", "Power", "kW", 0.001, 30000, 120000, 5000,
     "Less power pulls away more slowly; above what the plan asks, more changes nothing"},
    {"viscous_coupling_nms", "Viscous coupling", "N·m·s/rad", 1, 0, 100, 5,
     "Stiffer holds the driven wheels together and makes the car understeer"},
}};

double FourWheelCar::* field(std::string_view key) {
    if (key == "mass_kg") return &FourWheelCar::mass_kg;
    if (key == "cg_height_m") return &FourWheelCar::cg_height_m;
    if (key == "brake_bias_front") return &FourWheelCar::brake_bias_front;
    if (key == "drag_area_m2") return &FourWheelCar::drag_area_m2;
    if (key == "downforce_area_m2") return &FourWheelCar::downforce_area_m2;
    if (key == "max_drive_power_w") return &FourWheelCar::max_drive_power_w;
    if (key == "viscous_coupling_nms") return &FourWheelCar::viscous_coupling_nms;
    throw std::invalid_argument("Unknown setup control: "+std::string(key));
}

} // namespace

std::span<const SetupControl> four_wheel_setup_controls() { return controls; }

const SetupControl& setup_control(std::string_view key) {
    for (const auto& control : controls)
        if (control.key == key) return control;
    throw std::invalid_argument("Unknown setup control: "+std::string(key));
}

double setup_value(const FourWheelCar& car, std::string_view key) {
    setup_control(key);
    return car.*field(key);
}

FourWheelCar with_setup_value(const FourWheelCar& car, const Config& config, std::string_view key, double value) {
    const auto& control = setup_control(key);
    if (!std::isfinite(value) || value < control.minimum || value > control.maximum)
        throw std::invalid_argument("Setup "+std::string(control.label)+" must lie within its offered range");
    FourWheelCar next = car;
    // A car made heavier or lighter keeps its mass distribution, so its yaw inertia scales with its mass.
    if (key == "mass_kg") next.yaw_inertia_kgm2 = car.yaw_inertia_kgm2*value/car.mass_kg;
    next.*field(key) = value;
    validate_vehicle(next, config);
    return next;
}

} // namespace fd

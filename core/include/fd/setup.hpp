#pragma once
#include "fd/vehicle.hpp"

#include <span>
#include <string_view>

namespace fd {

// Setup controls for the four-wheel car (TrackWayFastPlan Phase 2.4, decision 0016): the vehicle parameters an
// application offers as controls, the range it offers, and the effect a test proves for each on one fixed
// scenario (tests/setup_tests.cpp). The car's other parameters stay fixed in the applications. Planning and
// control settings stay in Config; these belong to the car.
struct SetupControl {
    std::string_view key;       // the FourWheelCar field, as recordings name it
    std::string_view label;     // for people
    std::string_view unit;      // of the displayed value
    double display_scale{1};    // displayed value = SI value times this
    double minimum{}, maximum{}, step{};  // the offered range and step, SI
    std::string_view effect;    // the measured direction, in words
};

std::span<const SetupControl> four_wheel_setup_controls();
// Rejects an unknown key.
const SetupControl& setup_control(std::string_view key);
double setup_value(const FourWheelCar& car, std::string_view key);
// The car with one setup value changed. Changing mass keeps the car's mass distribution, so yaw inertia scales
// with it. Rejects an unknown key, a non-finite value or one outside the control's offered range, and any car
// validate_vehicle rejects.
FourWheelCar with_setup_value(const FourWheelCar& car, const Config& config, std::string_view key, double value);

} // namespace fd

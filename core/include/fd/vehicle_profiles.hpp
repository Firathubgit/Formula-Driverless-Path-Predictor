#pragma once
#include "fd/core.hpp"
#include "fd/vehicle.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace fd {

// Selectable vehicles with honest physics (TrackWayFastPlan §8, decision 0033): each profile is the four-wheel car with a
// documented set of parameters, and says where every one came from. Appearance stays apart: a profile is physics only,
// and nothing here draws or names a car's look. A team-named file is never presented as team data.

// Where one value came from: printed as the source gives it, derived from the source by stated arithmetic, fitted to the
// source's own model (tools/vehicle_profiles/fit_tires.py), or chosen here because the source gives none.
enum class Provenance { published, derived, fitted, chosen };
// "published", "derived", "fitted" or "chosen".
const char* provenance_name(Provenance provenance);

struct ParameterSource {
    std::string parameter;  // the FourWheelCar or Config member it sets
    Provenance provenance{Provenance::chosen};
    std::string note;       // the source's own value or the arithmetic, in a few words
};

struct VehicleProfile {
    std::string id;         // for --profile
    std::string name;
    std::string source;     // the file or files, and how the source describes its own values
    FourWheelCar car;
    // The configuration members the profile sets on top of whatever else is configured: the wheelbase and steering
    // limit that belong to the car, and the speed cap its plan uses.
    double wheelbase_m{}, max_steering_rad{}, max_speed_mps{};
    // How fast the front wheels can be turned, where the car's own differs from the configuration's default (a kart's
    // quick hands, decision 0046); zero leaves the configuration's rate alone.
    double max_steering_rate_radps{};
    std::vector<ParameterSource> parameters;
};

// Every profile offered, in a fixed order. A road car is not among them: none of the audited sources describes one.
const std::vector<VehicleProfile>& vehicle_profiles();
// The profile of this id; rejects an unknown one, naming the ids there are.
const VehicleProfile& vehicle_profile(std::string_view id);
// The configuration with the profile's own members set.
Config configured_for(const VehicleProfile& profile, Config config);

// Tuning one named value of a four-wheel car or of the configuration that belongs to it (decision 0046), for calibrating a
// profile against measurements without rebuilding: any FourWheelCar number by its member name, a tire's number as
// front_tire.NAME, rear_tire.NAME or tires.NAME (both), and the configuration's wheelbase_m, max_steering_rad and
// max_speed_mps. Rejects an unknown name, listing the valid ones. The result is not validated: the caller does that with
// validate_vehicle, as every car is.
void set_car_value(FourWheelCar& car, Config& config, std::string_view name, double value);
// Every name set_car_value accepts, in a fixed order.
std::vector<std::string> car_value_names();

} // namespace fd

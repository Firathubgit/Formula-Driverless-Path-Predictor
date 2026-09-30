#pragma once
#include "fd/vehicle.hpp"

#include <string>
#include <vector>

namespace fd {

// How the controller turns its pursuit target into a steering command (decision 0010).
// Pure Pursuit steers by geometry, which assumes the tires do not slip. Model- and
// Acceleration-based Pursuit (MAP) keeps the same target but asks the plant's steady-state
// response which steering holds the demanded turn at the current speed.
enum class SteeringMode { pure_pursuit, model_acceleration_pursuit };
const char* steering_mode_name(SteeringMode mode);

// One steady turn the plant settled into: a held steering angle at a held speed, and the path
// curvature and lateral acceleration it produced.
struct SteadyTurn {
    double steering_rad{}, curvature_per_m{}, lateral_acceleration_mps2{};
};
struct SteeringRow {
    double speed_mps{};
    // From zero steering upward, strictly increasing in curvature, ending at the steering that
    // gives the largest steady turn inside the tires' envelope or at the steering limit.
    std::vector<SteadyTurn> turns;
};

// The steady-state steering response of one plant under one configuration. Generating it drives
// copies of the plant through advance() at held speeds and steering angles until they settle, so
// it is the plant's own behaviour, including understeer and the grip limit. Only stable steady
// turns inside the tire envelope are kept: MAP cannot ask for a turn the car cannot hold.
class SteeringTable {
public:
    // Validates the model and configuration, then generates. Costs a fraction of a second.
    SteeringTable(const VehicleModel& model, const Config& config);

    // Steering that holds a turn of this signed curvature at this speed. Beyond the largest
    // steady turn of a speed it returns that turn's steering. Below the lowest row the geometric
    // law atan(wheelbase * curvature) applies, which is exact where the plant is the kinematic
    // bicycle. Rows are interpolated linearly in speed; above the highest row it is used.
    double steering_for(double curvature_per_m, double speed_mps) const;

    // True when this table was generated from exactly this model and configuration.
    bool generated_for(const VehicleModel& model, const Config& config) const;
    // Identifies the generator, model and configuration: 16 hexadecimal digits.
    const std::string& fingerprint() const noexcept { return fingerprint_; }
    const std::vector<SteeringRow>& rows() const noexcept { return rows_; }

private:
    std::string identity_, fingerprint_;
    double wheelbase_m_{}, max_steering_rad_{};
    std::vector<SteeringRow> rows_;
};

// The fingerprint a table generated from this model and configuration carries.
std::string steering_table_fingerprint(const VehicleModel& model, const Config& config);

} // namespace fd

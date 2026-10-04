#pragma once
#include "fd/core.hpp"

#include <vector>

namespace fd {

// What the poster's coloured line asks of the pedals at a point (decision 0039): green, blue, red.
enum class Advice { Accelerate, Coast, Brake };

struct CoachSample { Vec2 position; Advice advice{Advice::Accelerate}; };

// One bend of the line, found from its curvature alone (decision 0049): where it passes the turn-in curvature, where it
// is tightest, where it falls below it again.
struct CoachCorner {
    int number{};            // 1-based, in lap order from the line
    int direction{};         // +1 to the left, -1 to the right
    double turn_in_m{}, apex_m{}, exit_m{};  // stations along the line
    double radius_m{};       // of the line at its tightest
};

// What a driver is told to do next. One at a time, in this order of urgency.
enum class CoachCall { Gas, Lift, Brake, TurnIn, Apex };

struct CoachCues {
    bool located{};          // the car is on or beside the line; otherwise nothing below is meaningful
    double along_m{};        // the car's station along the line, from the start line
    double offset_m{};       // how far left of the line the car is (negative: right)
    Advice advice{};         // what the line asks at the car's station
    CoachCall call{CoachCall::Gas};
    int corner{};            // the corner the car is in or approaching, 1-based, 0 when the line has none
    int direction{};         // its direction, +1 left, -1 right
    bool in_corner{};
    double to_turn_in_m{};   // 0 inside a corner
    double to_apex_m{};      // ahead positive; negative once past it
    double to_brake_m{};     // to the start of the next red, 0 while braking is advised; negative when the lap has none
    double advised_steering_rad{};  // the front wheels' angle that follows the line's bend at the car's speed, left positive
};

// Coaching read from the poster's line and the car's own pose and speed, never from the plant or the driver. It is advice:
// where to brake, turn in, be at the apex and get back on the gas, by the line's own geometry. It decides nothing and is
// never part of a recording or the judge's reading.
class KartCoach {
public:
    // A closed loop of at least three distinct points about 0.1 to 0.5 m apart, the first at the start line, in driving
    // order; a wheelbase for the kinematic steering angle. Throws std::invalid_argument naming the fault.
    KartCoach(const std::vector<CoachSample>& line, double wheelbase_m);

    double length_m() const { return length_; }
    const std::vector<CoachCorner>& corners() const { return corners_; }
    // The line's point at a station along it, wrapped round the lap; where the apex or the turn-in is on the ground.
    Vec2 position_at(double station_m) const;
    // The cues for a car at `position` going `speed_mps`. `hint_m`, the station from the last call, keeps the search to
    // where the car was, so a hairpin's other leg is never taken for the line; negative searches the whole lap.
    CoachCues cues(Vec2 position, double speed_mps, double hint_m = -1) const;

    static constexpr double max_offset_m = 8.0;      // further from the line than this, the car is off it
    static constexpr double corner_radius_m = 20.0;  // a bend tighter than this is a corner
    static constexpr double smoothing_m = 2.0;       // the line's curvature is read after this much Gaussian smoothing

private:
    std::vector<Vec2> points_;
    std::vector<Advice> advice_;
    std::vector<double> curvature_;
    std::vector<CoachCorner> corners_;
    double length_{}, step_{}, wheelbase_{};
};

} // namespace fd

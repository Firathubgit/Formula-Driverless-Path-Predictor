#pragma once
#include "fd/core.hpp"

#include <vector>

namespace fd {

// A banked stretch of a track (decision 0050): the road tilts across its width, rising to `tilt_rad` and falling away
// smoothly over `ramp_m` at each end. Positive lifts the right edge above the left, as the road banks into a left-hand
// bend; negative the left.
struct Banking { double from_s_m{}, to_s_m{}, ramp_m{}, tilt_rad{}; };

struct Ground {
    double height_m{};  // above the flat floor
    double roll_rad{};  // how the surface tilts across the way the car faces: positive, right side higher
};

// What the ground looks like, for display (decision 0050): the height of the surface and its tilt where a car stands. It
// is drawing only. The plant is planar and never reads it, so a banked bend is seen and not driven.
class TrackRelief {
public:
    // Every banking within the lap and ending after it begins, with a ramp that fits and a tilt under 45 degrees; the
    // bankings may not overlap. Throws std::invalid_argument naming the fault.
    TrackRelief(const Track& track, std::vector<Banking> bankings);

    // The surface at a map point: the lower edge is at the floor's height and the higher edge the corridor's width times the
    // tangent of the tilt above it, linear between. Beyond an edge the height holds for the barrier's footing, `footing_m`, and
    // past that the floor is flat again: a banked bend stands on a retaining wall, it does not lift the hall behind it. Flat away
    // from every banking.
    Ground ground_at(Vec2 position) const;
    static constexpr double footing_m = 0.32;
    // The tilt of the road at a station along it, positive right side higher; 0 away from every banking.
    double tilt_at(double s_m) const;
    bool flat() const { return bankings_.empty(); }
    const std::vector<Banking>& bankings() const { return bankings_; }

private:
    Track track_;
    std::vector<Banking> bankings_;
    double min_x_{}, max_x_{}, min_y_{}, max_y_{};  // where any banking can be, so that far ground costs nothing
};

// Gokartcentralen Göteborg's one raised bend: the velodrome banking at T9 (decision 0038's poster), the ninth bend of the
// 400 m lap, from 299 to 329 m, with its tilt chosen at 6 degrees, which no source states. Displays a track other than
// that lap's flat.
TrackRelief gokartcentralen_relief(const Track& track);

} // namespace fd

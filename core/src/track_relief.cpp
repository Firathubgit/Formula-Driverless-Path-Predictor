#include "fd/track_relief.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace fd {
namespace {
double smoothstep(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3 - 2 * x);
}
// The share of a banking's tilt at a station: rising over its ramp, level through its middle, falling over its ramp.
double share(const Banking& b, double s) {
    return smoothstep((s - b.from_s_m) / b.ramp_m) * smoothstep((b.to_s_m - s) / b.ramp_m);
}
} // namespace

TrackRelief::TrackRelief(const Track& track, std::vector<Banking> bankings) : track_(track), bankings_(std::move(bankings)) {
    validate_track(track);
    std::sort(bankings_.begin(), bankings_.end(), [](const Banking& a, const Banking& b) { return a.from_s_m < b.from_s_m; });
    double last_end = -1e300;
    for (const auto& b : bankings_) {
        if (!std::isfinite(b.from_s_m) || !std::isfinite(b.to_s_m) || !std::isfinite(b.ramp_m) || !std::isfinite(b.tilt_rad))
            throw std::invalid_argument("banking has a value that is not finite");
        if (!(b.ramp_m > 0)) throw std::invalid_argument("banking's ramp must be positive");
        if (b.from_s_m < 0 || b.to_s_m > track.length_m || !(b.to_s_m - b.from_s_m > 2 * b.ramp_m))
            throw std::invalid_argument("banking must lie within the lap and be longer than its two ramps");
        if (std::abs(b.tilt_rad) >= std::numbers::pi / 4) throw std::invalid_argument("banking's tilt must be under 45 degrees");
        if (b.from_s_m < last_end) throw std::invalid_argument("bankings may not overlap");
        last_end = b.to_s_m;
    }
    min_x_ = min_y_ = 1e300;
    max_x_ = max_y_ = -1e300;
    for (const auto& p : track.points) {
        for (const auto& b : bankings_) {
            if (p.s_m < b.from_s_m - 1 || p.s_m > b.to_s_m + 1) continue;
            min_x_ = std::min(min_x_, p.x_m); max_x_ = std::max(max_x_, p.x_m);
            min_y_ = std::min(min_y_, p.y_m); max_y_ = std::max(max_y_, p.y_m);
        }
    }
    const double reach = track.width_m / 2 + 4.0;
    min_x_ -= reach; max_x_ += reach; min_y_ -= reach; max_y_ += reach;
}

double TrackRelief::tilt_at(double s_m) const {
    double tilt = 0;
    for (const auto& b : bankings_) if (s_m > b.from_s_m && s_m < b.to_s_m) tilt += b.tilt_rad * share(b, s_m);
    return tilt;
}

Ground TrackRelief::ground_at(Vec2 position) const {
    if (bankings_.empty() || position.x < min_x_ || position.x > max_x_ || position.y < min_y_ || position.y > max_y_) return {};
    const auto at = project(track_, position);
    const double tilt = tilt_at(at.s_m);
    if (tilt == 0) return {};
    const double half = track_.width_m / 2;
    if (std::abs(at.signed_error_m) > half + footing_m) return {};
    // The lower edge is the floor's own height, so the surface never dips below it.
    const double across = std::clamp(tilt > 0 ? half - at.signed_error_m : half + at.signed_error_m, 0.0, track_.width_m);
    return {across * std::tan(std::abs(tilt)), tilt};
}

TrackRelief gokartcentralen_relief(const Track& track) {
    // The poster's lap is 400 m; any other track's ground is flat.
    if (std::abs(track.length_m - 400.0) > 1.0 || track.name.find("Gokartcentralen") == std::string::npos) return TrackRelief(track, {});
    return TrackRelief(track, {{292.0, 336.0, 8.0, 6.0 * std::numbers::pi / 180}});
}

} // namespace fd

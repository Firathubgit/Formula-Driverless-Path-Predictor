#include "fd/kart_coach.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace fd {
namespace {
constexpr double resample_m = 0.25;
constexpr double merge_gap_m = 3.0;     // two bends one way this close are one corner
constexpr double shortest_corner_m = 3.0;
constexpr double search_m = 25.0;       // how far either side of the last station a hinted search looks
constexpr double apex_share = 0.85;     // the apex is the middle of where the bend is within this of its tightest

double wrap(double value, double length) {
    const double v = std::fmod(value, length);
    return v < 0 ? v + length : v;
}
// Signed distance from a to b along a loop of `length`, in [-length/2, length/2).
double signed_gap(double from, double to, double length) { return wrap(to - from + length / 2, length) - length / 2; }
} // namespace

KartCoach::KartCoach(const std::vector<CoachSample>& line, double wheelbase_m) : wheelbase_(wheelbase_m) {
    if (line.size() < 3) throw std::invalid_argument("coaching line needs at least three points");
    if (!(wheelbase_m > 0) || !std::isfinite(wheelbase_m)) throw std::invalid_argument("coaching wheelbase must be positive");
    const std::size_t n = line.size();
    std::vector<double> station(n + 1, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const auto& a = line[i].position;
        const auto& b = line[(i + 1) % n].position;
        if (!std::isfinite(a.x) || !std::isfinite(a.y)) throw std::invalid_argument("coaching line has a point that is not finite");
        const double chord = std::hypot(b.x - a.x, b.y - a.y);
        if (!(chord > 1e-6)) throw std::invalid_argument("coaching line repeats a point");
        if (chord > 2.0) throw std::invalid_argument("coaching line has points more than 2 m apart");
        station[i + 1] = station[i] + chord;
    }
    length_ = station[n];
    const std::size_t m = std::max<std::size_t>(32, static_cast<std::size_t>(std::llround(length_ / resample_m)));
    step_ = length_ / static_cast<double>(m);
    points_.resize(m);
    advice_.resize(m);
    std::size_t seg = 0;
    for (std::size_t i = 0; i < m; ++i) {
        const double u = step_ * static_cast<double>(i);
        while (seg + 1 < n && station[seg + 1] <= u) ++seg;
        const auto& a = line[seg].position;
        const auto& b = line[(seg + 1) % n].position;
        const double t = (u - station[seg]) / (station[seg + 1] - station[seg]);
        points_[i] = {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
        advice_[i] = line[seg].advice;
    }

    // The curvature of the line after smoothing away the hand-drawn stroke's wobble: a Gaussian along the loop.
    const long half = std::min<long>(static_cast<long>(std::ceil(3 * smoothing_m / step_)), static_cast<long>(m / 2) - 1);
    std::vector<double> kernel(static_cast<std::size_t>(2 * half + 1));
    double total = 0;
    for (long j = -half; j <= half; ++j) {
        const double d = static_cast<double>(j) * step_ / smoothing_m;
        total += kernel[static_cast<std::size_t>(j + half)] = std::exp(-0.5 * d * d);
    }
    for (auto& k : kernel) k /= total;
    std::vector<Vec2> smooth(m);
    const long lm = static_cast<long>(m);
    for (long i = 0; i < lm; ++i) {
        Vec2 sum{};
        for (long j = -half; j <= half; ++j) {
            const auto& p = points_[static_cast<std::size_t>(((i + j) % lm + lm) % lm)];
            const double w = kernel[static_cast<std::size_t>(j + half)];
            sum.x += p.x * w;
            sum.y += p.y * w;
        }
        smooth[static_cast<std::size_t>(i)] = sum;
    }
    curvature_.resize(m);
    for (std::size_t i = 0; i < m; ++i) {
        const auto& a = smooth[(i + m - 1) % m];
        const auto& b = smooth[i];
        const auto& c = smooth[(i + 1) % m];
        const double dx = (c.x - a.x) / (2 * step_), dy = (c.y - a.y) / (2 * step_);
        const double ddx = (c.x - 2 * b.x + a.x) / (step_ * step_), ddy = (c.y - 2 * b.y + a.y) / (step_ * step_);
        const double speed2 = dx * dx + dy * dy;
        curvature_[i] = (dx * ddy - dy * ddx) / std::pow(std::max(speed2, 1e-12), 1.5);
    }

    // Corners: runs of one sign tighter than the corner radius, starting from a place the line is straighter so that
    // none wraps the start of the array.
    const double tight = 1.0 / corner_radius_m;
    const auto is_on = [&](std::size_t i) { return std::abs(curvature_[i]) > tight; };
    std::size_t first_off = m;
    for (std::size_t i = 0; i < m && first_off == m; ++i) if (!is_on(i)) first_off = i;
    if (first_off == m) return;  // a line that is one long bend has no corners to speak of
    struct Run { std::size_t begin, end; int sign; };  // indices counted from first_off, end inclusive
    std::vector<Run> runs;
    for (std::size_t j = 0; j < m; ++j) {
        const std::size_t i = (first_off + j) % m;
        if (!is_on(i)) continue;
        const int sign = curvature_[i] > 0 ? 1 : -1;
        if (!runs.empty() && runs.back().end + 1 == j && runs.back().sign == sign) runs.back().end = j;
        else runs.push_back({j, j, sign});
    }
    std::vector<Run> merged;
    for (const auto& run : runs) {
        if (!merged.empty() && merged.back().sign == run.sign &&
            static_cast<double>(run.begin - merged.back().end - 1) * step_ < merge_gap_m) merged.back().end = run.end;
        else merged.push_back(run);
    }
    for (const auto& run : merged) {
        if (static_cast<double>(run.end - run.begin + 1) * step_ < shortest_corner_m) continue;
        double peak = 0;
        for (std::size_t j = run.begin; j <= run.end; ++j) peak = std::max(peak, std::abs(curvature_[(first_off + j) % m]));
        std::size_t low = run.end, high = run.begin;
        for (std::size_t j = run.begin; j <= run.end; ++j) {
            if (std::abs(curvature_[(first_off + j) % m]) >= apex_share * peak) { low = std::min(low, j); high = std::max(high, j); }
        }
        CoachCorner corner;
        corner.direction = run.sign;
        corner.radius_m = 1.0 / peak;
        corner.turn_in_m = wrap(static_cast<double>(first_off + run.begin) * step_, length_);
        corner.apex_m = wrap(static_cast<double>(first_off + (low + high) / 2) * step_, length_);
        corner.exit_m = wrap(static_cast<double>(first_off + run.end + 1) * step_, length_);
        corners_.push_back(corner);
    }
    std::sort(corners_.begin(), corners_.end(), [](const CoachCorner& a, const CoachCorner& b) { return a.apex_m < b.apex_m; });
    for (std::size_t i = 0; i < corners_.size(); ++i) corners_[i].number = static_cast<int>(i) + 1;
}

Vec2 KartCoach::position_at(double station_m) const {
    const double u = wrap(station_m, length_) / step_;
    const std::size_t a = static_cast<std::size_t>(u) % points_.size(), b = (a + 1) % points_.size();
    const double t = u - std::floor(u);
    return {points_[a].x + (points_[b].x - points_[a].x) * t, points_[a].y + (points_[b].y - points_[a].y) * t};
}

CoachCues KartCoach::cues(Vec2 position, double speed_mps, double hint_m) const {
    const std::size_t m = points_.size();
    const long lm = static_cast<long>(m);
    const double speed = std::isfinite(speed_mps) ? std::max(0.0, speed_mps) : 0.0;
    // The nearest point of the line, within reach of where the car last was when that is known.
    std::size_t best = 0;
    double best_d2 = 1e300;
    const auto consider = [&](std::size_t i) {
        const double dx = position.x - points_[i].x, dy = position.y - points_[i].y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) { best_d2 = d2; best = i; }
    };
    if (hint_m >= 0 && std::isfinite(hint_m)) {
        const long centre = static_cast<long>(std::llround(wrap(hint_m, length_) / step_));
        const long reach = static_cast<long>(search_m / step_);
        for (long j = -reach; j <= reach; ++j) consider(static_cast<std::size_t>(((centre + j) % lm + lm) % lm));
    } else {
        for (std::size_t i = 0; i < m; ++i) consider(i);
    }
    // Projected onto the nearer of the two segments at that point, for a smooth station and a signed offset.
    double along = static_cast<double>(best) * step_, offset = 0;
    double best_line = 1e300;
    for (const std::size_t a : {(best + m - 1) % m, best}) {
        const std::size_t b = (a + 1) % m;
        const double sx = points_[b].x - points_[a].x, sy = points_[b].y - points_[a].y;
        const double len2 = sx * sx + sy * sy;
        const double t = std::clamp(((position.x - points_[a].x) * sx + (position.y - points_[a].y) * sy) / len2, 0.0, 1.0);
        const double qx = points_[a].x + sx * t, qy = points_[a].y + sy * t;
        const double d = std::hypot(position.x - qx, position.y - qy);
        if (d < best_line) {
            best_line = d;
            along = wrap((static_cast<double>(a) + t) * step_, length_);
            offset = (sx * (position.y - qy) - sy * (position.x - qx)) / std::sqrt(len2);
        }
    }
    CoachCues out;
    out.along_m = along;
    out.offset_m = offset;
    out.located = best_line <= max_offset_m;
    const std::size_t at = static_cast<std::size_t>(std::llround(along / step_)) % m;
    out.advice = advice_[at];
    if (!out.located) return out;

    // The wheel angle that follows the bend a tenth of a second ahead, as a steering wheel is turned before the bend.
    const std::size_t lead_at = (at + static_cast<std::size_t>(std::llround(speed * 0.1 / step_))) % m;
    out.advised_steering_rad = std::atan(wheelbase_ * curvature_[lead_at]);

    if (out.advice == Advice::Brake) out.to_brake_m = 0;
    else {
        out.to_brake_m = -1;
        for (std::size_t k = 1; k < m; ++k) if (advice_[(at + k) % m] == Advice::Brake) { out.to_brake_m = static_cast<double>(k) * step_; break; }
    }

    const CoachCorner* chosen = nullptr;
    double soonest = 1e300;
    for (const auto& corner : corners_) {
        const double span = wrap(corner.exit_m - corner.turn_in_m, length_);
        if (wrap(along - corner.turn_in_m, length_) <= span) { chosen = &corner; out.in_corner = true; break; }
        const double ahead = wrap(corner.turn_in_m - along, length_);
        if (ahead < soonest) { soonest = ahead; chosen = &corner; }
    }
    if (chosen) {
        out.corner = chosen->number;
        out.direction = chosen->direction;
        out.to_turn_in_m = out.in_corner ? 0 : soonest;
        out.to_apex_m = signed_gap(along, chosen->apex_m, length_);
    }

    const double lead = std::max(1.0, 0.15 * speed);
    const bool apex = out.in_corner && std::abs(out.to_apex_m) <= lead;
    const bool brake = out.advice == Advice::Brake || (out.to_brake_m >= 0 && out.to_brake_m <= 0.3 * speed + 2.0);
    const bool turn_in = !out.in_corner && chosen && out.to_turn_in_m <= std::max(2.0, 0.3 * speed);
    if (apex) out.call = CoachCall::Apex;
    else if (brake) out.call = CoachCall::Brake;
    else if (turn_in) out.call = CoachCall::TurnIn;
    else if (out.advice == Advice::Coast) out.call = CoachCall::Lift;
    else out.call = CoachCall::Gas;
    return out;
}

} // namespace fd

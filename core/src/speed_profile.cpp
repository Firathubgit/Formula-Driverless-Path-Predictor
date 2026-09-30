#include "fd/speed_profile.hpp"
#include "fd/performance_envelope.hpp"
#include "speed_limits.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fd {
namespace {

constexpr double unlimited = std::numeric_limits<double>::infinity();

// The reference's curvature at a station, interpolated between the samples around it, and its rate there.
struct ReferenceCurvature { double curvature{}, rate{}; };
ReferenceCurvature reference_curvature(const Track& track, double s) {
    s = std::fmod(s, track.length_m);
    if (s < 0) s += track.length_m;
    const auto upper = std::upper_bound(track.points.begin(), track.points.end(), s, [](double value, const PathPoint& p) { return value < p.s_m; });
    const std::size_t i = upper == track.points.begin() ? 0 : static_cast<std::size_t>(upper-track.points.begin()-1);
    const std::size_t j = (i+1)%track.points.size();
    const auto& a = track.points[i];
    const auto& b = track.points[j];
    const double span = (j == 0 ? track.length_m : b.s_m)-a.s_m;
    const double f = (s-a.s_m)/span;
    return {a.curvature+f*(b.curvature-a.curvature), (b.curvature-a.curvature)/span};
}

// Where a path lies across the reference at a station, how fast that changes per metre, and how fast that changes.
struct Offset { double value{}, first{}, second{}; };

// The path's geometry as the profile sees it: offsets interpolated between the path's points, and slopes that vary
// linearly between knots at the middles of its segments, from the car's own slope before the first to none after the
// last.
class PathGeometry {
public:
    PathGeometry(std::span<const StationOffset> path, double start_slope, double start_turn) : path_(path) {
        if (path.size() < 2) return;
        const double first = path[1].s_m-path[0].s_m;
        knots_.push_back({path[0].s_m+first/2-std::max(first, start_turn), start_slope});
        for (std::size_t k = 0; k+1 < path.size(); ++k)
            knots_.push_back({(path[k].s_m+path[k+1].s_m)/2, (path[k+1].offset_m-path[k].offset_m)/(path[k+1].s_m-path[k].s_m)});
        knots_.push_back({path.back().s_m+(path[path.size()-1].s_m-path[path.size()-2].s_m)/2, 0.0});
    }

    Offset at(double s) const {
        Offset d;
        if (path_.empty()) return d;
        if (s <= path_.front().s_m) d.value = path_.front().offset_m;
        else if (s >= path_.back().s_m) d.value = path_.back().offset_m;
        else {
            const auto upper = std::upper_bound(path_.begin(), path_.end(), s, [](double value, const StationOffset& p) { return value < p.s_m; });
            const auto& b = *upper;
            const auto& a = *(upper-1);
            d.value = a.offset_m+(s-a.s_m)/(b.s_m-a.s_m)*(b.offset_m-a.offset_m);
        }
        if (knots_.empty() || s >= knots_.back().s) return d;
        if (s < knots_.front().s) { d.first = knots_.front().slope; return d; }
        const auto upper = std::upper_bound(knots_.begin(), knots_.end(), s, [](double value, const Knot& k) { return value < k.s; });
        const auto& b = *upper;
        const auto& a = *(upper-1);
        d.second = (b.slope-a.slope)/(b.s-a.s);
        d.first = a.slope+d.second*(s-a.s);
        return d;
    }

private:
    struct Knot { double s{}, slope{}; };
    std::span<const StationOffset> path_;
    std::vector<Knot> knots_;
};

} // namespace

double profile_time(std::span<const ProfilePoint> points) {
    double time = 0;
    for (std::size_t k = 0; k+1 < points.size(); ++k) {
        const double distance = points[k+1].distance_m-points[k].distance_m;
        const double speeds = points[k].speed_mps+points[k+1].speed_mps;
        if (distance <= 0) continue;
        if (!(speeds > 0)) return unlimited;
        time += 2*distance/speeds;
    }
    return time;
}

SpeedProfile make_speed_profile(const Track& track, const Config& c, const ProfileRequest& request, const PerformanceEnvelope* envelope) {
    validate_config(c);
    if (!std::isfinite(request.start_s_m) || !std::isfinite(request.start_slope) || !std::isfinite(request.start_turn_m) || request.start_turn_m < 0)
        throw std::invalid_argument("A speed profile needs a finite start station and slope and a nonnegative start turn");
    if (!std::isfinite(request.start_speed_mps) || request.start_speed_mps < 0 || std::isnan(request.end_speed_mps) || request.end_speed_mps < 0)
        throw std::invalid_argument("A speed profile needs a finite nonnegative start speed and a nonnegative end speed");
    if (!std::isfinite(request.horizon_m) || request.horizon_m <= 0)
        throw std::invalid_argument("A speed profile needs a positive finite horizon");
    const auto& path = request.path;
    for (std::size_t k = 0; k < path.size(); ++k)
        if (!std::isfinite(path[k].s_m) || !std::isfinite(path[k].offset_m) || (k > 0 && path[k].s_m <= path[k-1].s_m))
            throw std::invalid_argument("A speed profile's path needs finite offsets at increasing stations");
    if (!path.empty() && std::abs(path.front().s_m-request.start_s_m) > 1e-9*std::max(1.0, std::abs(request.start_s_m)))
        throw std::invalid_argument("A speed profile's path must start at the profile's start station");
    if (envelope && !envelope->generated_for(c))
        throw std::invalid_argument("The performance envelope was derived under another configuration; derive it for this one");

    const PathGeometry geometry(path, request.start_slope, request.start_turn_m);
    const auto curvature_at = [&](double s) {
        const auto reference = reference_curvature(track, s);
        const auto d = geometry.at(s);
        const double p = 1-reference.curvature*d.value;
        if (p <= 1e-9) throw std::invalid_argument("A speed profile's path reaches the reference's centre of curvature");
        const double k = reference.curvature;
        return (p*(k*p+d.second)+d.first*(reference.rate*d.value+2*k*d.first))/std::pow(p*p+d.first*d.first, 1.5);
    };
    const auto stretch_at = [&](double s) {
        const auto d = geometry.at(s);
        const double p = 1-reference_curvature(track, s).curvature*d.value;
        return std::sqrt(p*p+d.first*d.first);
    };

    const auto samples = static_cast<std::size_t>(std::max(1.0, std::ceil(request.horizon_m/speed_profile_spacing_m-1e-9)));
    const double step = request.horizon_m/static_cast<double>(samples);
    SpeedProfile profile;
    auto& points = profile.points;
    points.resize(samples+1);
    for (std::size_t k = 0; k <= samples; ++k) {
        auto& point = points[k];
        point.s_m = k == samples ? request.start_s_m+request.horizon_m : request.start_s_m+step*static_cast<double>(k);
        point.curvature_1pm = curvature_at(point.s_m);
        point.distance_m = k == 0 ? 0.0 : points[k-1].distance_m+stretch_at(points[k-1].s_m+step/2)*step;
    }

    const double lateral = c.grip_mu*gravity_mps2*c.lateral_grip_fraction;
    const double longitudinal = c.grip_mu*gravity_mps2*c.longitudinal_grip_fraction;
    const double share = c.envelope_fraction;
    const auto cap = [&](double k) {
        const double limit = envelope ? detail::envelope_curvature_speed(*envelope, share, k, c.max_speed_mps)
                                      : std::abs(k) > 1e-9 ? std::sqrt(lateral/std::abs(k)) : unlimited;
        return std::min(c.max_speed_mps, limit);
    };
    const auto forward = [&](double v, double k) {
        return envelope ? share*std::max(0.0, envelope->forward_limit(v, v*v*k/share)) : longitudinal;
    };
    const auto braking = [&](double v, double k) {
        return envelope ? share*std::max(0.0, -envelope->braking_limit(v, v*v*k/share)) : longitudinal;
    };

    // Backward from the end speed: the most each sample may carry and still slow for everything after it.
    std::vector<double> most(samples+1);
    most[samples] = std::min(cap(points[samples].curvature_1pm), request.end_speed_mps);
    for (std::size_t k = samples; k > 0; --k) {
        const double distance = points[k].distance_m-points[k-1].distance_m;
        most[k-1] = std::min(cap(points[k-1].curvature_1pm),
                             std::sqrt(most[k]*most[k]+2*braking(most[k], points[k].curvature_1pm)*distance));
    }
    // Forward from the car's actual speed, braking at the limit while it starts beyond what the path allows.
    points[0].speed_mps = request.start_speed_mps;
    for (std::size_t k = 0; k < samples; ++k) {
        const double v = points[k].speed_mps;
        const double distance = points[k+1].distance_m-points[k].distance_m;
        if (v <= most[k]+1e-9*std::max(1.0, most[k])) {
            points[k+1].speed_mps = std::min(most[k+1], std::sqrt(v*v+2*forward(v, points[k].curvature_1pm)*distance));
        } else {
            profile.within_limits = false;
            const double braked = std::sqrt(std::max(0.0, v*v-2*braking(v, points[k].curvature_1pm)*distance));
            points[k+1].speed_mps = std::max(std::min(most[k+1], v), braked);
        }
    }
    profile.estimated_time_s = profile_time(points);
    return profile;
}

} // namespace fd

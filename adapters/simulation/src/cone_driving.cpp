#include "fd/cone_driving.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fd {
namespace {

// A pose carried forward over dt at a constant speed and yaw rate: an arc, or a straight line without turning.
PoseReading carried(const PoseReading& from, const MotionReading& motion, double dt) {
    PoseReading out = from;
    out.sampled_at_s = from.sampled_at_s+dt;
    const double v = std::max(0.0, motion.speed_mps), r = motion.yaw_rate_radps, turn = r*dt;
    if (std::abs(turn) < 1e-9) {
        out.x_m += v*dt*std::cos(from.yaw_rad);
        out.y_m += v*dt*std::sin(from.yaw_rad);
    } else {
        out.x_m += v/r*(std::sin(from.yaw_rad+turn)-std::sin(from.yaw_rad));
        out.y_m -= v/r*(std::cos(from.yaw_rad+turn)-std::cos(from.yaw_rad));
    }
    out.yaw_rad = wrap_angle(from.yaw_rad+turn);
    return out;
}

ConeType cone_type(ObservedColour colour) {
    switch (colour) {
    case ObservedColour::blue: return ConeType::blue;
    case ObservedColour::yellow: return ConeType::yellow;
    case ObservedColour::orange: return ConeType::orange_small;
    case ObservedColour::big_orange: return ConeType::orange_big;
    case ObservedColour::unknown: return ConeType::unknown;
    }
    return ConeType::unknown;
}

// How wide the matched cones say the track is: the median distance between each cone and its match across, or the
// planner's minimum track width if none were matched.
double believed_width(const ConePath& cones, double minimum) {
    std::vector<double> widths;
    for (std::size_t i = 0; i < cones.left_with_virtual.size() && i < cones.left_to_right.size(); ++i) {
        const int j = cones.left_to_right[i];
        if (j < 0 || static_cast<std::size_t>(j) >= cones.right_with_virtual.size()) continue;
        const auto a = cones.left_with_virtual[i], b = cones.right_with_virtual[static_cast<std::size_t>(j)];
        widths.push_back(std::hypot(a.x-b.x, a.y-b.y));
    }
    if (widths.empty()) return minimum;
    std::nth_element(widths.begin(), widths.begin()+static_cast<std::ptrdiff_t>(widths.size()/2), widths.end());
    return std::max(minimum, widths[widths.size()/2]);
}

} // namespace

const char* belief_source_name(BeliefSource source) { return source == BeliefSource::ideal ? "ideal" : "measured"; }

void validate_cone_memory(const ConeMemory& m) {
    if (!std::isfinite(m.merge_radius_m) || m.merge_radius_m <= 0 || !std::isfinite(m.keep_within_m) || m.keep_within_m <= m.merge_radius_m)
        throw std::invalid_argument("A cone memory needs a positive merge radius and a larger distance to keep cones within");
}

ConeDriver::ConeDriver(ConePathSettings settings, ConeMemory memory) : planner_(settings), memory_settings_(memory) {
    validate_cone_memory(memory_settings_);
    reset({});
}

void ConeDriver::reset(const State& start) {
    planner_ = ConePathPlanner(planner_.settings());
    poses_ = {{start.time_s, start.x_m, start.y_m, start.yaw_rad}};
    motion_ = {};
    paths_.clear();
    memory_.clear();
    belief_ = {};
    belief_.state = {start.x_m, start.y_m, start.yaw_rad, 0, 0, start.time_s};
    belief_.pose_sampled_at_s = start.time_s;
    frames_ = 0;
    made_ = 0;
}

void ConeDriver::read_pose(const PoseReading& reading) {
    if (!std::isfinite(reading.sampled_at_s) || !std::isfinite(reading.x_m) || !std::isfinite(reading.y_m) || !std::isfinite(reading.yaw_rad))
        throw std::invalid_argument("A pose reading must be finite");
    if (reading.sampled_at_s <= poses_.back().sampled_at_s) return;  // already read
    poses_.push_back(reading);
    // Perception frames are sampled at most their dead time before they arrive; two seconds of poses covers any of them.
    const double keep_from = reading.sampled_at_s-2.0;
    const auto first = std::find_if(poses_.begin(), poses_.end(), [&](const PoseReading& p) { return p.sampled_at_s >= keep_from; });
    if (first != poses_.begin() && first != poses_.end()) poses_.erase(poses_.begin(), first-1);
}

void ConeDriver::read_motion(const MotionReading& reading) {
    if (!std::isfinite(reading.speed_mps) || !std::isfinite(reading.yaw_rate_radps) || !std::isfinite(reading.steering_rad))
        throw std::invalid_argument("A motion reading must be finite");
    motion_ = reading;
}

State ConeDriver::believed_at(double time_s) const {
    const auto upper = std::upper_bound(poses_.begin(), poses_.end(), time_s,
                                        [](double t, const PoseReading& p) { return t < p.sampled_at_s; });
    PoseReading pose;
    if (upper == poses_.end()) {
        pose = carried(poses_.back(), motion_, time_s-poses_.back().sampled_at_s);
    } else if (upper == poses_.begin()) {
        pose = poses_.front();
    } else {
        // Between two readings: position linearly, heading along the shorter way round.
        const auto& a = *(upper-1);
        const auto& b = *upper;
        const double f = (time_s-a.sampled_at_s)/(b.sampled_at_s-a.sampled_at_s);
        pose = {time_s, a.x_m+f*(b.x_m-a.x_m), a.y_m+f*(b.y_m-a.y_m), wrap_angle(a.yaw_rad+f*wrap_angle(b.yaw_rad-a.yaw_rad))};
    }
    return {pose.x_m, pose.y_m, pose.yaw_rad, std::max(0.0, motion_.speed_mps), motion_.steering_rad, time_s};
}

bool ConeDriver::perceive(const PerceptionFrame& frame) {
    const std::uint64_t index = frames_++;
    if (frame.sampled_at_s < poses_.front().sampled_at_s) return false;
    const State pose = believed_at(frame.sampled_at_s);
    const double c = std::cos(pose.yaw_rad), s = std::sin(pose.yaw_rad);
    std::vector<TypedCone> cones;
    cones.reserve(frame.detections.size());
    for (const auto& d : frame.detections)
        cones.push_back({{pose.x_m+c*d.position.x-s*d.position.y, pose.y_m+s*d.position.x+c*d.position.y}, cone_type(d.colour)});
    if (memory_settings_.enabled) {
        // Each placement refines the nearest remembered cone within the merge radius, or is remembered as a new one.
        for (const auto& placed : cones) {
            RememberedCone* nearest = nullptr;
            double best = memory_settings_.merge_radius_m;
            for (auto& r : memory_) {
                const double d = std::hypot(r.position.x-placed.position.x, r.position.y-placed.position.y);
                if (d <= best) { best = d; nearest = &r; }
            }
            if (!nearest) { memory_.push_back({placed.position, {}, 0}); nearest = &memory_.back(); }
            const double n = nearest->placements;
            nearest->position = {(nearest->position.x*n+placed.position.x)/(n+1), (nearest->position.y*n+placed.position.y)/(n+1)};
            ++nearest->colour_votes[static_cast<std::size_t>(placed.type)];
            ++nearest->placements;
        }
        std::erase_if(memory_, [&](const RememberedCone& r) {
            return std::hypot(r.position.x-pose.x_m, r.position.y-pose.y_m) > memory_settings_.keep_within_m;
        });
        // The planner sees every cone remembered, each in the colour most often reported, unknown if none was.
        cones.clear();
        for (const auto& r : memory_) {
            std::size_t colour = 0;
            for (std::size_t k = 1; k < r.colour_votes.size(); ++k)
                if (r.colour_votes[k] > 0 && r.colour_votes[k] > (colour ? r.colour_votes[colour] : 0)) colour = k;
            cones.push_back({r.position, static_cast<ConeType>(colour)});
        }
    }
    auto planned = planner_.plan(cones, {pose.x_m, pose.y_m}, {c, s});
    if (planned.from_previous) return false;
    OpenPath path;
    path.width_m = believed_width(planned, planner_.settings().min_track_width_m);
    for (const auto& p : planned.path) {
        if (!path.points.empty()) {
            const auto& last = path.points.back();
            const double chord = std::hypot(p.x_m-last.x_m, p.y_m-last.y_m);
            if (chord <= 1e-6) continue;
            path.points.push_back({p.x_m, p.y_m, last.s_m+chord, p.curvature_1pm});
        } else {
            path.points.push_back({p.x_m, p.y_m, 0.0, p.curvature_1pm});
        }
    }
    if (path.points.size() < 2) return false;
    paths_.push_back({made_++, index, pose, std::move(planned), std::move(path)});
    if (paths_.size() > kept_paths) paths_.erase(paths_.begin());
    return true;
}

LocalDecision ConeDriver::decide(double time_s, const Config& config, const VehicleModel& model, std::uint64_t ticks_to_next_control,
                                 const SteeringTable* steering, const PerformanceEnvelope* envelope) {
    belief_.state = believed_at(time_s);
    belief_.state.steering_rad = std::clamp(belief_.state.steering_rad, -config.max_steering_rad, config.max_steering_rad);
    belief_.pose_sampled_at_s = std::min(time_s, poses_.back().sampled_at_s);
    const auto plant = plant_state_from(belief_.state, model, config);
    LocalOption option;
    option.action = LocalAction::cone_path;
    option.speed_limit_mps = std::numeric_limits<double>::infinity();
    LocalDecision decision;
    OpenPath followed;
    if (const auto* believed = path()) {
        followed = believed->path;
        belief_.path = believed->index;
        belief_.plan = plan_open_path(followed, config, envelope);
        decision.reason = "Following the path believed from perception frame "+std::to_string(believed->frame);
    } else {
        // Nothing seen to follow: hold the heading and brake to rest.
        const double c = std::cos(belief_.state.yaw_rad), s = std::sin(belief_.state.yaw_rad);
        followed.points = {{belief_.state.x_m, belief_.state.y_m, 0, 0}, {belief_.state.x_m+c, belief_.state.y_m+s, 1, 0}};
        followed.width_m = planner_.settings().min_track_width_m;
        belief_.path.reset();
        belief_.plan = {{0, 0, 0, "no_believed_path"}, {0, 0, 1, "no_believed_path"}};
        decision.reason = "No believed path yet: braking to rest";
    }
    option.trajectory = predict_open_path(followed, belief_.plan, plant, config, model, ticks_to_next_control, steering, envelope);
    const auto& end = option.trajectory.points.back().state;
    option.station_gain_m = project_open(followed, {end.x_m, end.y_m}).s_m-project_open(followed, {belief_.state.x_m, belief_.state.y_m}).s_m;
    option.clear = option.trajectory.within_model_envelope;
    option.reason = option.clear ? "Clear" : option.trajectory.validity_reason;
    if (!belief_.path) belief_.plan.clear();
    decision.options = {std::move(option)};
    decision.selected = 0;
    return decision;
}

} // namespace fd

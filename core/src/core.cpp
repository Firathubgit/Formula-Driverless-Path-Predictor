#include "fd/core.hpp"
#include "fd/performance_envelope.hpp"
#include "fd/steering.hpp"
#include "speed_limits.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <numbers>
#include <set>
#include <stdexcept>

namespace fd {
namespace {
constexpr double pi = std::numbers::pi;
double norm(Vec2 p) { return std::hypot(p.x, p.y); }
Vec2 add(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
Vec2 sub(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
Vec2 mul(Vec2 a, double b) { return {a.x * b, a.y * b}; }
double dot(Vec2 a, Vec2 b) { return a.x*b.x + a.y*b.y; }
bool valid_state(const State& state) {
    return std::isfinite(state.x_m) && std::isfinite(state.y_m) && std::isfinite(state.yaw_rad) &&
           std::isfinite(state.speed_mps) && std::isfinite(state.steering_rad) &&
           std::isfinite(state.time_s) && state.speed_mps>=0;
}
double segment_length(const Track& track, std::size_t i) {
    return i+1 < track.points.size() ? track.points[i+1].s_m-track.points[i].s_m
                                   : track.length_m-track.points[i].s_m;
}
// Straight-line distance between consecutive samples may exceed their arc length by at most
// this, which absorbs coordinates stored to twelve significant digits in recordings.
double chord_tolerance(const PathPoint& a, const PathPoint& b) {
    return 1e-9*(1+std::max(std::max(std::abs(a.x_m), std::abs(a.y_m)), std::max(std::abs(b.x_m), std::abs(b.y_m))));
}
std::string trim(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last-first+1);
}
}

double wrap_angle(double angle) { return std::remainder(angle, 2*pi); }

void validate_config(const Config& c) {
    const auto range = [](double v, double lo, double hi, const char* name) {
        if (!std::isfinite(v) || v < lo || v > hi)
            throw std::invalid_argument(std::string(name)+" outside supported range");
    };
    range(c.grip_mu, 0.2, 1.5, "grip_mu");
    range(c.fixed_dt_s, 0.001, 0.01, "fixed_dt_s");
    range(c.control_dt_s, c.fixed_dt_s, 0.1, "control_dt_s");
    const double ratio = c.control_dt_s/c.fixed_dt_s;
    if (std::abs(ratio-std::round(ratio)) > 1e-9)
        throw std::invalid_argument("control_dt_s must be an integer multiple of fixed_dt_s");
    // Down to a racing kart's 1.045 m and a Formula Student car's 1.44 m (decision 0033).
    range(c.wheelbase_m, 1.0, 4.0, "wheelbase_m");
    range(c.max_speed_mps, 2.0, 40.0, "max_speed_mps");
    range(c.max_steering_rad, 0.1, 0.7, "max_steering_rad");
    range(c.max_steering_rate_radps, 0.1, 3.0, "max_steering_rate_radps");
    range(c.lateral_grip_fraction, 0.2, 0.75, "lateral_grip_fraction");
    range(c.longitudinal_grip_fraction, 0.2, 0.65, "longitudinal_grip_fraction");
    if (std::hypot(c.lateral_grip_fraction, c.longitudinal_grip_fraction) > 0.9)
        throw std::invalid_argument("combined planned grip fractions must leave at least 10% reserve");
    range(c.speed_gain, 0.1, 5.0, "speed_gain");
    range(c.lookahead_base_m, 2.0, 10.0, "lookahead_base_m");
    range(c.lookahead_time_s, 0.1, 0.6, "lookahead_time_s");
    range(c.envelope_fraction, 0.5, 1.0, "envelope_fraction");
}

Config load_config(const std::filesystem::path& file) {
    std::ifstream input(file);
    if (!input) throw std::runtime_error("Cannot open config: "+file.string());
    Config result;
    const std::map<std::string, double Config::*> fields = {
        {"grip_mu", &Config::grip_mu}, {"fixed_dt_s", &Config::fixed_dt_s},
        {"control_dt_s", &Config::control_dt_s}, {"wheelbase_m", &Config::wheelbase_m},
        {"max_speed_mps", &Config::max_speed_mps}, {"max_steering_rad", &Config::max_steering_rad},
        {"max_steering_rate_radps", &Config::max_steering_rate_radps},
        {"lateral_grip_fraction", &Config::lateral_grip_fraction},
        {"longitudinal_grip_fraction", &Config::longitudinal_grip_fraction},
        {"speed_gain", &Config::speed_gain}, {"lookahead_base_m", &Config::lookahead_base_m},
        {"lookahead_time_s", &Config::lookahead_time_s}, {"envelope_fraction", &Config::envelope_fraction}
    };
    std::set<std::string> seen;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        if (const auto hash = line.find('#'); hash != std::string::npos) line.resize(hash);
        line = trim(line);
        if (line.empty()) continue;
        const auto equal = line.find('=');
        if (equal == std::string::npos || line.find('=', equal+1) != std::string::npos)
            throw std::invalid_argument("Config line "+std::to_string(line_number)+": expected key=value");
        const auto key = trim(line.substr(0, equal));
        const auto value = trim(line.substr(equal+1));
        if (!seen.insert(key).second) throw std::invalid_argument("Duplicate config key: "+key);
        if (key=="schema_version") {
            if (value!="1") throw std::invalid_argument("Unsupported config schema_version; expected 1");
            continue;
        }
        if (!fields.contains(key)) throw std::invalid_argument("Unknown config key: "+key);
        try {
            std::size_t end{};
            const double number = std::stod(value, &end);
            if (end != value.size()) throw std::invalid_argument("trailing characters");
            result.*fields.at(key) = number;
        } catch (const std::exception&) { throw std::invalid_argument("Invalid numeric value for "+key); }
    }
    if (!seen.contains("schema_version")) throw std::invalid_argument("Config requires schema_version=1");
    validate_config(result);
    return result;
}

Track make_preset_track() {
    // A closed CCW, tangent-continuous rounded polygon: a long main straight,
    // one 18 m corner and progressively wider 24/32/38/26 m bends.
    const std::vector<Vec2> vertices{{0,0},{170,0},{170,75},{35,108},{-35,52}};
    const std::vector<double> radii{26,18,32,38,24};
    struct Corner { Vec2 entry, exit, center; double start_angle, turn, radius; };
    std::vector<Corner> corners;
    for (std::size_t i=0; i<vertices.size(); ++i) {
        const Vec2 incoming = sub(vertices[i], vertices[(i+vertices.size()-1)%vertices.size()]);
        const Vec2 outgoing = sub(vertices[(i+1)%vertices.size()], vertices[i]);
        const Vec2 u = mul(incoming, 1/norm(incoming)), v = mul(outgoing, 1/norm(outgoing));
        const double turn = std::acos(std::clamp(dot(u,v), -1.0, 1.0));
        const double distance = radii[i]*std::tan(turn/2);
        const Vec2 entry = sub(vertices[i], mul(u,distance));
        const Vec2 end = add(vertices[i], mul(v,distance));
        const Vec2 center = add(entry, mul(Vec2{-u.y,u.x},radii[i]));
        corners.push_back({entry,end,center,std::atan2(entry.y-center.y,entry.x-center.x),turn,radii[i]});
    }
    Track track;
    double s=0;
    constexpr double spacing = 0.6;
    for (std::size_t i=0; i<corners.size(); ++i) {
        const auto& prior = corners[i];
        const auto& next = corners[(i+1)%corners.size()];
        const Vec2 delta = sub(next.entry,prior.exit);
        const double line_length = norm(delta);
        const auto line_count = static_cast<int>(std::ceil(line_length/spacing));
        for (int j=0; j<line_count; ++j) {
            const double f = static_cast<double>(j)/line_count;
            const Vec2 p = add(prior.exit,mul(delta,f));
            track.points.push_back({p.x,p.y,s+f*line_length,0});
        }
        s += line_length;
        const double arc_length = next.turn*next.radius;
        const auto arc_count = static_cast<int>(std::ceil(arc_length/spacing));
        for (int j=0; j<arc_count; ++j) {
            const double f = static_cast<double>(j)/arc_count;
            const double a = next.start_angle+f*next.turn;
            track.points.push_back({next.center.x+next.radius*std::cos(a),
                                    next.center.y+next.radius*std::sin(a),s+f*arc_length,1/next.radius});
        }
        s += arc_length;
    }
    track.length_m = s;
    validate_track(track);
    return track;
}

void validate_track(const Track& track) {
    if (track.points.size()<3 || !std::isfinite(track.width_m) || track.width_m<=0 ||
        !std::isfinite(track.length_m) || track.length_m<=0)
        throw std::invalid_argument("Track must be a finite positive-width closed path with at least three samples");
    if (std::abs(track.points.front().s_m)>1e-9) throw std::invalid_argument("Track s must start at zero");
    for (std::size_t i=0; i<track.points.size(); ++i) {
        const auto& p = track.points[i];
        const auto& q = track.points[(i+1)%track.points.size()];
        const double chord = std::hypot(q.x_m-p.x_m,q.y_m-p.y_m);
        const double arc = segment_length(track,i);
        if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m) || !std::isfinite(p.s_m) ||
            !std::isfinite(p.curvature) || arc<=1e-8 || chord<=1e-8)
            throw std::invalid_argument("Track contains non-finite values, repeated points or non-increasing arc length");
        // An arc is never shorter than its chord. project() relies on this to skip
        // stretches of track that cannot contain the nearest point.
        if (chord>arc && chord-arc>chord_tolerance(p,q))
            throw std::invalid_argument("Track arc length between samples is shorter than their straight-line distance");
    }
    if (track.left_edge_m.empty() != track.right_edge_m.empty())
        throw std::invalid_argument("Track corridor edges come in pairs: both left and right, or neither");
    if (!track.left_edge_m.empty()) {
        if (track.left_edge_m.size()!=track.points.size() || track.right_edge_m.size()!=track.points.size())
            throw std::invalid_argument("Track corridor edges need one distance per sample");
        for (std::size_t i=0; i<track.points.size(); ++i)
            if (!std::isfinite(track.left_edge_m[i]) || !std::isfinite(track.right_edge_m[i]) ||
                track.left_edge_m[i]<0 || track.right_edge_m[i]<0)
                throw std::invalid_argument("Track corridor edges must be finite and not behind the reference");
    }
}

namespace detail {
// The fastest speed from rest up to the cap at which this curvature stays within the share of the envelope's lateral
// limit, or infinity if the cap is. The limit is linear in speed between the envelope's grid speeds and constant beyond
// them, so on each piece speed squared times curvature less the limit is convex: where it is within at both ends of a
// piece it is within throughout, and the first crossing is found by bisection inside the first piece that ends beyond it.
double envelope_curvature_speed(const PerformanceEnvelope& envelope, double share, double curvature, double cap) {
    const double k = std::abs(curvature);
    if (k <= 1e-9) return std::numeric_limits<double>::infinity();
    const auto side = curvature > 0 ? TurnSide::left : TurnSide::right;
    const auto beyond = [&](double v) { return v*v*k > share*envelope.lateral_limit(v, side); };
    std::vector<double> pieces{0.0};
    for (const auto& row : envelope.rows())
        if (row.speed_mps > 0 && row.speed_mps < cap) pieces.push_back(row.speed_mps);
    pieces.push_back(cap);
    for (std::size_t p = 1; p < pieces.size(); ++p) {
        if (!beyond(pieces[p])) continue;
        double lo = pieces[p-1], hi = pieces[p];
        for (int halving = 0; halving < 200 && hi-lo > 1e-12*std::max(1.0, hi); ++halving) {
            const double mid = 0.5*(lo+hi);
            (beyond(mid) ? hi : lo) = mid;
        }
        return lo;
    }
    return std::numeric_limits<double>::infinity();
}
} // namespace detail

double estimated_lap_time(const Track& track, const std::vector<PlanPoint>& plan) {
    validate_track(track);
    if (plan.size() != track.points.size()) throw std::invalid_argument("Estimated lap time needs one plan point per track sample");
    double time = 0;
    for (std::size_t i = 0; i < plan.size(); ++i) {
        const std::size_t next = (i+1) % plan.size();
        const double span = (next == 0 ? track.length_m : track.points[next].s_m)-track.points[i].s_m;
        const double speeds = plan[i].speed_mps+plan[next].speed_mps;
        if (!(std::isfinite(speeds) && speeds > 0)) throw std::invalid_argument("Estimated lap time needs a plan that moves along every span");
        time += 2*span/speeds;
    }
    return time;
}

std::vector<PlanPoint> make_speed_plan(const Track& track, const Config& c, const PerformanceEnvelope* envelope) {
    validate_config(c);
    validate_track(track);
    if (envelope && !envelope->generated_for(c))
        throw std::invalid_argument("The performance envelope was derived under another configuration; derive it for this one");
    const std::size_t n = track.points.size();
    const double lateral = c.grip_mu*gravity_mps2*c.lateral_grip_fraction;
    const double longitudinal = c.grip_mu*gravity_mps2*c.longitudinal_grip_fraction;
    // Longitudinal capacity from a sample at a speed: the grip fraction, or the share of the envelope at that speed and
    // the lateral acceleration the sample's curvature asks for there, scaled up by the share as the shrunk envelope is.
    const double share = c.envelope_fraction;
    const auto forward = [&](std::size_t i, double v) {
        return envelope ? share*std::max(0.0, envelope->forward_limit(v, v*v*track.points[i].curvature/share)) : longitudinal;
    };
    const auto braking = [&](std::size_t i, double v) {
        return envelope ? share*std::max(0.0, -envelope->braking_limit(v, v*v*track.points[i].curvature/share)) : longitudinal;
    };
    std::vector<PlanPoint> plan(n);
    for (std::size_t i=0; i<n; ++i) {
        const double k = std::abs(track.points[i].curvature);
        if (k>std::tan(c.max_steering_rad)/c.wheelbase_m+1e-10)
            throw std::invalid_argument("Track curvature exceeds configured steering capability");
        const double curve_limit = envelope ? detail::envelope_curvature_speed(*envelope, share, track.points[i].curvature, c.max_speed_mps)
                                            : k>1e-9 ? std::sqrt(lateral/k) : c.max_speed_mps;
        plan[i] = {std::min(c.max_speed_mps,curve_limit),0,i,
                   curve_limit<c.max_speed_mps ? "curvature_limit" : "speed_cap"};
    }
    // Every edge is checked including the seam; monotone relaxation converges
    // the periodic acceleration/braking reachability constraints together.
    bool converged = false;
    for (std::size_t pass=0; pass<=n; ++pass) {
        double max_change=0;
        for (std::size_t i=0; i<n; ++i) {
            const std::size_t j = (i+1)%n;
            const double limit = std::sqrt(plan[i].speed_mps*plan[i].speed_mps+2*forward(i,plan[i].speed_mps)*segment_length(track,i));
            if (plan[j].speed_mps>limit+1e-10) {
                max_change = std::max(max_change,plan[j].speed_mps-limit);
                plan[j].speed_mps=limit;
                plan[j].limiting_index=plan[i].limiting_index;
                plan[j].reason="acceleration_reachability";
            }
        }
        for (std::size_t k=n; k>0; --k) {
            const std::size_t i=k-1, j=(i+1)%n;
            const double limit=std::sqrt(plan[j].speed_mps*plan[j].speed_mps+2*braking(j,plan[j].speed_mps)*segment_length(track,i));
            if (plan[i].speed_mps>limit+1e-10) {
                max_change=std::max(max_change,plan[i].speed_mps-limit);
                plan[i].speed_mps=limit;
                plan[i].limiting_index=plan[j].limiting_index;
                plan[i].reason="braking_reachability";
            }
        }
        if (max_change<1e-9) { converged=true; break; }
    }
    if (!converged) throw std::runtime_error("Periodic speed profile did not converge");
    for (std::size_t i=0; i<n; ++i) {
        const double v=plan[i].speed_mps, next=plan[(i+1)%n].speed_mps;
        plan[i].acceleration_mps2=(next*next-v*v)/(2*segment_length(track,i));
    }
    return plan;
}

Projection project(const Track& track, Vec2 position) {
    const auto& points=track.points;
    const std::size_t n=points.size();
    // Pruning. Walking the track, the distance to sample i bounds every later point q:
    // |position-q| >= |position-a_i| - arc(a_i,q), because an arc is never shorter than
    // its chord (validate_track). Segments whose whole extent provably lies farther than
    // some already-known distance are skipped. The scan still visits the rest in index
    // order with the exact per-segment arithmetic below, so the chosen segment, including
    // ties to the lowest index, and every returned bit match a full scan.
    double seed_squared=std::numeric_limits<double>::infinity();
    constexpr std::size_t seed_stride=16;
    for (std::size_t i=0; i<n; i+=seed_stride) {
        const double dx=position.x-points[i].x_m, dy=position.y-points[i].y_m;
        seed_squared=std::min(seed_squared,dx*dx+dy*dy);
    }
    // A sample lies on its own segment, so its distance bounds the nearest distance.
    const double seed=std::sqrt(seed_squared);
    const double position_scale=std::max(std::abs(position.x),std::abs(position.y));
    const auto next_station=[&](std::size_t j){ return j<n ? points[j].s_m : track.length_m; };

    Projection result;
    double best = std::numeric_limits<double>::infinity();
    double best_squared = std::numeric_limits<double>::infinity();
    for (std::size_t i=0; i<n;) {
        const auto& a=track.points[i];
        const auto& b=track.points[(i+1)%track.points.size()];
        const Vec2 edge{b.x_m-a.x_m,b.y_m-a.y_m};
        const Vec2 relative{position.x-a.x_m,position.y-a.y_m};
        const double fraction=std::clamp(dot(relative,edge)/dot(edge,edge),0.0,1.0);
        const Vec2 p{a.x_m+fraction*edge.x,a.y_m+fraction*edge.y};
        const double dx=position.x-p.x, dy=position.y-p.y;
        const double squared=dx*dx+dy*dy;
        // Rollouts project many copied states. Compare squared distance first, then
        // calculate distance/normal only for the selected segment. Retain hypot's
        // overflow handling for extreme finite coordinates accepted by this interface.
        // Keep near ties for the original strict hypot comparison: rounded squared
        // distance and hypot can disagree in their last few bits.
        constexpr double distance_guard=1+8*std::numeric_limits<double>::epsilon();
        if (squared<=best_squared*distance_guard || !std::isfinite(squared)) {
            const double distance=std::hypot(dx,dy);
            if (distance<best) {
                best_squared=squared;
                best=distance;
                result.index=i;
                result.fraction=fraction;
                result.point=p;
            }
        }
        std::size_t next=i+1;
        const double to_sample=std::sqrt(relative.x*relative.x+relative.y*relative.y);
        const double known=std::min(seed,best);
        const double sample_scale=std::max(std::abs(a.x_m),std::abs(a.y_m));
        // Covers the chord tolerance summed over every skippable segment, whose coordinates
        // are within one lap of this sample, plus rounding in the quantities compared.
        const double margin=static_cast<double>(n)*1e-9*(1+sample_scale+2*track.length_m)+
                            64*std::numeric_limits<double>::epsilon()*
                            (position_scale+sample_scale+to_sample+known+track.length_m);
        const double reach=a.s_m+(to_sample-known-margin);
        if (std::isfinite(reach) && reach>next_station(i+1)) {
            // Skip every segment k whose far end satisfies station(k+1) < reach.
            const auto first_not_skippable=std::lower_bound(points.begin()+static_cast<std::ptrdiff_t>(i+1),points.end(),reach,
                [](const PathPoint& point,double value){ return point.s_m<value; });
            const auto j=static_cast<std::size_t>(first_not_skippable-points.begin());
            next=j<n ? j-1 : (track.length_m<reach ? n : n-1);
            next=std::max(next,i+1);
        }
        i=next;
    }
    const auto& a=track.points[result.index];
    const auto& b=track.points[(result.index+1)%track.points.size()];
    const Vec2 edge{b.x_m-a.x_m,b.y_m-a.y_m};
    result.s_m=a.s_m+result.fraction*segment_length(track,result.index);
    result.signed_error_m=(edge.x*(position.y-result.point.y)-edge.y*(position.x-result.point.x))/norm(edge);
    result.distance_m=best;
    if (result.s_m>=track.length_m) result.s_m-=track.length_m;
    return result;
}

Corridor corridor_at(const Track& track, const Projection& at) {
    if (track.left_edge_m.empty()) return {track.width_m/2, track.width_m/2};
    const std::size_t i=at.index, j=(at.index+1)%track.points.size();
    const double f=at.fraction;
    return {track.left_edge_m[i]+f*(track.left_edge_m[j]-track.left_edge_m[i]),
            track.right_edge_m[i]+f*(track.right_edge_m[j]-track.right_edge_m[i])};
}

bool within_corridor(const Track& track, const Projection& at, double margin) {
    if (track.left_edge_m.empty()) return at.distance_m <= std::max(0.0, track.width_m/2-margin);
    const auto corridor=corridor_at(track,at);
    return at.signed_error_m <= corridor.left_m-margin && -at.signed_error_m <= corridor.right_m-margin;
}

PathPoint sample(const Track& track, double s) {
    s=std::fmod(s,track.length_m);
    if (s<0) s+=track.length_m;
    const auto upper=std::upper_bound(track.points.begin(),track.points.end(),s,
                                    [](double value,const PathPoint& p){return value<p.s_m;});
    const std::size_t i=upper==track.points.begin()?0:static_cast<std::size_t>(upper-track.points.begin()-1);
    const auto& a=track.points[i];
    const auto& b=track.points[(i+1)%track.points.size()];
    const double f=(s-a.s_m)/segment_length(track,i);
    return {a.x_m+f*(b.x_m-a.x_m),a.y_m+f*(b.y_m-a.y_m),s,a.curvature};
}

ControlResult compute_control(const Track& track, const std::vector<PlanPoint>& plan,
                              const State& state, const Config& config, ControlIntent intent,
                              const SteeringTable* steering, const PerformanceEnvelope* envelope) {
    if (!valid_state(state))
        throw std::invalid_argument("Controller requires a finite nonnegative-speed state");
    if (plan.size()!=track.points.size() || plan.empty())
        throw std::invalid_argument("Controller requires a plan matching the validated track");
    for (const auto& point:plan)
        if (!std::isfinite(point.speed_mps) || point.speed_mps<0 ||
            !std::isfinite(point.acceleration_mps2) || point.limiting_index>=plan.size())
            throw std::invalid_argument("Controller requires finite nonnegative plan speeds, finite accelerations and valid limiting indices");
    if (!std::isfinite(intent.lateral_offset_m) || std::isnan(intent.speed_limit_mps) || intent.speed_limit_mps<0)
        throw std::invalid_argument("Control intent requires a finite offset and a nonnegative speed limit");
    for (std::size_t k=0; k<intent.path.size(); ++k)
        if (!std::isfinite(intent.path[k].s_m) || !std::isfinite(intent.path[k].offset_m) || (k>0 && intent.path[k].s_m<=intent.path[k-1].s_m))
            throw std::invalid_argument("Control intent path requires finite offsets at increasing stations");
    const auto& profile=intent.speed_profile;
    for (std::size_t k=0; k<profile.size(); ++k)
        if (!std::isfinite(profile[k].s_m) || !std::isfinite(profile[k].distance_m) || !std::isfinite(profile[k].speed_mps) ||
            profile[k].speed_mps<0 || (k>0 && (profile[k].s_m<=profile[k-1].s_m || profile[k].distance_m<=profile[k-1].distance_m)))
            throw std::invalid_argument("Control intent speed profile requires finite nonnegative speeds at increasing stations and distances");
    const auto nearest=project(track,{state.x_m,state.y_m});
    const auto i=nearest.index, j=(i+1)%plan.size();
    const double reference_sq=(1-nearest.fraction)*plan[i].speed_mps*plan[i].speed_mps+
                              nearest.fraction*plan[j].speed_mps*plan[j].speed_mps;
    double reference_target=std::sqrt(std::max(0.0,reference_sq));
    double reference_feedforward=plan[i].acceleration_mps2;
    if (profile.size()>=2) {
        // Where the car lies along the profile, whose stations run on from its first, as a path's do.
        const double span=profile.back().s_m-profile.front().s_m;
        const double along=std::fmod(std::fmod(nearest.s_m-profile.front().s_m,track.length_m)+track.length_m,track.length_m);
        if (along<=span) {
            const double at=profile.front().s_m+along;
            const auto upper=std::upper_bound(profile.begin(),profile.end(),at,[](double value,const ProfilePoint& p){return value<p.s_m;});
            const auto& b=upper==profile.end() ? profile.back() : *upper;
            const auto& a=upper==profile.end() ? *(profile.end()-2) : *(upper-1);
            const double f=std::clamp((at-a.s_m)/(b.s_m-a.s_m),0.0,1.0);
            reference_target=std::sqrt(std::max(0.0,(1-f)*a.speed_mps*a.speed_mps+f*b.speed_mps*b.speed_mps));
            reference_feedforward=(b.speed_mps*b.speed_mps-a.speed_mps*a.speed_mps)/(2*(b.distance_m-a.distance_m));
        } else if (along>span+(track.length_m-span)/2) {
            reference_target=profile.front().speed_mps;
            reference_feedforward=0;
        }
    }
    const bool capped=intent.speed_limit_mps<reference_target;
    const double target=capped?intent.speed_limit_mps:reference_target;
    const double lookahead=config.lookahead_base_m+config.lookahead_time_s*state.speed_mps;
    double aim_station=nearest.s_m+lookahead;
    double aim_offset=intent.lateral_offset_m;
    if (!intent.path.empty()) {
        // The path's offset at the target's station: stations run on from the path's first, so the target's distance
        // along the lap from there is either within the path, beyond its end, or, nearer the rest of the lap, before it.
        const auto& path=intent.path;
        const double span=path.back().s_m-path.front().s_m;
        const double along=std::fmod(std::fmod(aim_station-path.front().s_m,track.length_m)+track.length_m,track.length_m);
        if (along<=span) {
            const double at=path.front().s_m+along;
            const auto upper=std::upper_bound(path.begin(),path.end(),at,[](double value,const StationOffset& p){return value<p.s_m;});
            if (upper==path.end()) aim_offset=path.back().offset_m;
            else {
                const auto& b=*upper;
                const auto& a=*(upper-1);
                aim_offset=a.offset_m+(at-a.s_m)/(b.s_m-a.s_m)*(b.offset_m-a.offset_m);
            }
        } else {
            aim_offset=along<=span+(track.length_m-span)/2 ? path.back().offset_m : path.front().offset_m;
        }
    } else if (intent.lateral_offset_m!=0) {
        // Moving sideways by y within a pursuit distance L costs about 2*v^2*y/L^2 of
        // lateral acceleration. Aim far enough ahead that the shift fits the lateral grip
        // budget instead of demanding an impossible snap onto the offset line.
        const double shift=std::abs(intent.lateral_offset_m-nearest.signed_error_m);
        const double budget=config.lateral_grip_fraction*config.grip_mu*gravity_mps2;
        aim_station=nearest.s_m+std::max(lookahead,state.speed_mps*std::sqrt(2*shift/budget));
    }
    const auto aim=sample(track,aim_station);
    double aim_x=aim.x_m, aim_y=aim.y_m;
    if (aim_offset!=0) {
        // Left normal of the reference tangent at the pursuit station.
        const auto ahead=sample(track,aim_station+0.05);
        const double tx=ahead.x_m-aim.x_m, ty=ahead.y_m-aim.y_m, length=std::hypot(tx,ty);
        if (length>1e-12) {
            aim_x+=-ty/length*aim_offset;
            aim_y+= tx/length*aim_offset;
        }
    }
    const double dx=aim_x-state.x_m,dy=aim_y-state.y_m;
    const double local_y=-std::sin(state.yaw_rad)*dx+std::cos(state.yaw_rad)*dy;
    const double geometric_steering=std::atan2(2*config.wheelbase_m*local_y,dx*dx+dy*dy);
    // MAP converts the curvature of the same arc through the target with the plant's steering table.
    const double distance_sq=dx*dx+dy*dy;
    const double requested_steering=steering ?
        steering->steering_for(distance_sq>0 ? 2*local_y/distance_sq : 0.0,state.speed_mps) : geometric_steering;
    const double feedforward=capped?0.0:reference_feedforward;
    const double requested_acceleration=feedforward+config.speed_gain*(target-state.speed_mps);
    const double budget=config.longitudinal_grip_fraction*config.grip_mu*gravity_mps2;
    const double max_acceleration=envelope ? std::max(0.0,envelope->forward_limit(state.speed_mps,state.speed_mps*state.speed_mps*track.points[i].curvature)) : budget;
    const double max_braking=envelope ? std::max(0.0,-envelope->braking_limit(state.speed_mps,0)) : budget;
    return {{requested_acceleration,requested_steering},
            {std::clamp(requested_acceleration,-max_braking,max_acceleration),
             std::clamp(requested_steering,-config.max_steering_rad,config.max_steering_rad)},
             lookahead,target,nearest.distance_m,geometric_steering,nearest};
}

State integrate_bicycle(const State& state, Command applied, const Config& c, double dt) {
    if (!std::isfinite(dt) || dt<=0 || !std::isfinite(applied.acceleration_mps2) || !std::isfinite(applied.steering_rad) ||
        !valid_state(state))
        throw std::invalid_argument("Bicycle integration requires a finite nonnegative-speed state, finite commands and positive dt");
    State out=state;
    const double steer_target=std::clamp(applied.steering_rad,-c.max_steering_rad,c.max_steering_rad);
    out.steering_rad=state.steering_rad+std::clamp(steer_target-state.steering_rad,
                              -c.max_steering_rate_radps*dt,c.max_steering_rate_radps*dt);
    out.speed_mps=std::max(0.0,state.speed_mps+applied.acceleration_mps2*dt);
    // A braking tick may stop before its boundary. Integrate motion only until
    // that instant; the steering actuator and simulation clock still run all dt.
    const double moving_dt=applied.acceleration_mps2<0 ?
        std::min(dt,state.speed_mps/-applied.acceleration_mps2) : dt;
    const double moving_steering=state.steering_rad+std::clamp(steer_target-state.steering_rad,
                              -c.max_steering_rate_radps*moving_dt,c.max_steering_rate_radps*moving_dt);
    const double speed_mid=(state.speed_mps+out.speed_mps)/2;
    const double steer_mid=(state.steering_rad+moving_steering)/2;
    const double curvature=std::tan(steer_mid)/c.wheelbase_m;
    const double yaw_delta=speed_mid*curvature*moving_dt;
    // Exact constant-curvature arc for midpoint speed/steering avoids Euler drift.
    const double distance=speed_mid*moving_dt;
    const double half=yaw_delta/2;
    const double sinc=std::abs(half)>1e-10 ? std::sin(half)/half : 1.0;
    out.x_m+=distance*sinc*std::cos(state.yaw_rad+half);
    out.y_m+=distance*sinc*std::sin(state.yaw_rad+half);
    out.yaw_rad=wrap_angle(state.yaw_rad+yaw_delta);
    out.time_s=state.time_s+dt;
    return out;
}

} // namespace fd

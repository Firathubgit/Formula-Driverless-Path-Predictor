#include "fd/local_planner.hpp"
#include "fd/performance_envelope.hpp"
#include "fd/speed_profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace fd {
namespace {

constexpr double footprint_sample_m = 1.0;
// Fractions of the usable half-width offered as alternatives to the reference line.
// The outer pair stops short of the corridor edge because pursuit overshoot while
// converging onto an offset line would otherwise leave the margin and be rejected.
constexpr double candidate_fractions[] = {-0.75, -0.4, 0.0, 0.4, 0.75};
// A lattice search starts from the first layer at least this far ahead of the car.
constexpr double lattice_start_distance_m = 1.0;
constexpr double unlimited = std::numeric_limits<double>::infinity();

double forward_distance(double from_s, double to_s, double length) {
    const double delta = std::fmod(to_s-from_s, length);
    return delta < 0 ? delta+length : delta;
}

bool station_within(double s, const Obstruction& o, double length) {
    const double span = forward_distance(o.from_s_m, o.to_s_m, length);
    return forward_distance(o.from_s_m, s, length) <= span;
}

std::string metres(double value) {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(1);
    out << value << " m";
    return out.str();
}

std::string seconds(double value) {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(3);
    out << value << " s";
    return out.str();
}

// One rectangle of a blocked region, in world coordinates, inflated by the vehicle margin.
struct FootprintCell {
    Vec2 center, tangent, normal;
    double half_length{}, from_offset{}, to_offset{};
    bool contains(double x, double y) const {
        const double dx = x-center.x, dy = y-center.y;
        const double longitudinal = dx*tangent.x+dy*tangent.y;
        const double lateral = dx*normal.x+dy*normal.y;
        return std::abs(longitudinal) <= half_length &&
               lateral >= from_offset && lateral <= to_offset;
    }
};

std::vector<FootprintCell> footprint(const Track& track, const Obstruction& o) {
    const double span = forward_distance(o.from_s_m, o.to_s_m, track.length_m);
    const auto cells = static_cast<std::size_t>(std::max(1.0, std::ceil(span/footprint_sample_m)));
    const double step = span/static_cast<double>(cells);
    std::vector<FootprintCell> result;
    result.reserve(cells);
    for (std::size_t k = 0; k < cells; ++k) {
        const double middle = o.from_s_m+(static_cast<double>(k)+0.5)*step;
        const auto here = sample(track, middle);
        const auto ahead = sample(track, middle+0.05);
        const double tx = ahead.x_m-here.x_m, ty = ahead.y_m-here.y_m;
        const double norm = std::hypot(tx, ty);
        if (norm <= 1e-12) continue;
        result.push_back({{here.x_m, here.y_m}, {tx/norm, ty/norm}, {-ty/norm, tx/norm},
                          step/2+vehicle_half_width_m,
                          o.from_offset_m-vehicle_half_width_m, o.to_offset_m+vehicle_half_width_m});
    }
    return result;
}

// What both planners share for one decision: the scene, the prediction of any intent and its checks, the nearest
// blockage, and the speed cap that brakes for it.
class Candidates {
public:
    Candidates(const Track& track, const std::vector<PlanPoint>& reference, const PlantState& plant, const Config& config,
               const VehicleModel& model, const std::vector<Obstruction>& obstructions, std::uint64_t ticks,
               const SteeringTable* steering, const PerformanceEnvelope* envelope)
        : track_(track), reference_(reference), plant_(plant), config_(config), model_(model), obstructions_(obstructions),
          ticks_(ticks), steering_(steering), envelope_(envelope) {
        validate_obstructions(track, obstructions);
        footprints_.reserve(obstructions.size());
        for (const auto& o : obstructions) footprints_.push_back(footprint(track, o));
        const auto here = project(track, {plant.pose.x_m, plant.pose.y_m});
        start_s_ = here.s_m;
        start_offset_ = here.signed_error_m;
    }

    double start_station() const { return start_s_; }
    double start_offset() const { return start_offset_; }

    // Predicts the intent, which for a path must refer to the option's own path, and records whether it is clear.
    // Returns the index of the obstruction it enters, or the number of obstructions if it enters none.
    std::size_t evaluate(LocalOption& option, ControlIntent intent) const {
        option.speed_limit_mps = intent.speed_limit_mps;
        option.trajectory = make_local_trajectory(track_, reference_, plant_, config_, model_, ticks_, intent, steering_, envelope_);
        const auto& end = option.trajectory.points.back().state;
        option.station_gain_m = forward_distance(start_s_, project(track_, {end.x_m, end.y_m}).s_m, track_.length_m);
        const auto hit = first_contact(option.trajectory);
        if (hit < obstructions_.size()) {
            option.clear = false;
            option.reason = "Blocked by "+obstructions_[hit].identifier;
        } else if (!option.trajectory.within_model_envelope) {
            option.clear = false;
            option.reason = option.trajectory.validity_reason;
        } else {
            option.clear = true;
            option.reason = "Clear";
        }
        return hit;
    }

    LocalOption offset_option(double offset, double speed_limit) const {
        LocalOption option;
        option.lateral_offset_m = offset;
        evaluate(option, {offset, speed_limit});
        return option;
    }

    // The obstruction nearest ahead of the car, or the one it is already within, and how far away it begins.
    std::size_t nearest(double& distance) const {
        std::size_t blocking = obstructions_.size();
        distance = unlimited;
        for (std::size_t o = 0; o < obstructions_.size(); ++o) {
            const double d = station_within(start_s_, obstructions_[o], track_.length_m)
                                 ? 0.0
                                 : forward_distance(start_s_, obstructions_[o].from_s_m, track_.length_m);
            if (d < distance) { distance = d; blocking = o; }
        }
        return blocking;
    }

    // Holding a proportional controller on a braking curve needs a standing speed error,
    // because that error is what commands the deceleration. The car therefore runs this
    // much faster than whatever cap it is given, all the way down. Subtract it from the
    // commanded cap so the speed the car actually holds is the curve that reaches zero
    // at the clearance, instead of arriving there still moving.
    double stopping_cap(double distance) const {
        const double capacity = envelope_ ? std::max(0.0, -envelope_->braking_limit(0, 0))
                                          : config_.longitudinal_grip_fraction*config_.grip_mu*gravity_mps2;
        const double planned = obstruction_braking_fraction*capacity;
        const double lag_speed = planned/config_.speed_gain;
        const double room = std::max(0.0, distance-obstruction_stop_margin_m);
        return std::max(0.0, std::sqrt(2*planned*room)-lag_speed);
    }

    const Track& track() const { return track_; }
    const std::vector<Obstruction>& obstructions() const { return obstructions_; }
    double speed() const { return plant_.pose.speed_mps; }
    // The pursuit distance at the car's present speed.
    double lookahead() const { return config_.lookahead_base_m+config_.lookahead_time_s*plant_.pose.speed_mps; }
    // The slowest speed the car can brake to within a distance ahead, braking straight at the plan's longitudinal budget:
    // the longitudinal grip fraction, or the envelope plan's share of the car's braking limit at its present speed.
    double slowest_reachable(double distance) const {
        const double v = plant_.pose.speed_mps;
        const double braking = envelope_ ? config_.envelope_fraction*std::max(0.0, -envelope_->braking_limit(v, 0))
                                         : config_.longitudinal_grip_fraction*config_.grip_mu*gravity_mps2;
        return std::sqrt(std::max(0.0, v*v-2*braking*std::max(0.0, distance)));
    }
    // The reference plan's speed at a station, interpolated.
    double planned_speed(double s) const {
        s = std::fmod(std::fmod(s, track_.length_m)+track_.length_m, track_.length_m);
        const auto& points = track_.points;
        const auto upper = std::upper_bound(points.begin(), points.end(), s, [](double value, const PathPoint& p) { return value < p.s_m; });
        const std::size_t i = upper == points.begin() ? 0 : static_cast<std::size_t>(upper-points.begin()-1);
        const std::size_t j = (i+1)%points.size();
        const double f = (s-points[i].s_m)/((j == 0 ? track_.length_m : points[j].s_m)-points[i].s_m);
        return reference_[i].speed_mps+f*(reference_[j].speed_mps-reference_[i].speed_mps);
    }
    // The lateral acceleration a path may ask for at a speed: the plan's share of the car's envelope when it has one,
    // otherwise the lateral grip fraction of mu g the grip fraction plan uses.
    double lateral_budget(double speed) const {
        if (envelope_)
            return config_.envelope_fraction*std::min(envelope_->lateral_limit(speed, TurnSide::left), envelope_->lateral_limit(speed, TurnSide::right));
        return config_.lateral_grip_fraction*config_.grip_mu*gravity_mps2;
    }
    // How fast the car is moving across the reference per metre along it: the tangent of its heading against the
    // reference's direction where it is.
    double start_slope() const {
        const auto behind = sample(track_, start_s_-0.25), ahead = sample(track_, start_s_+0.25);
        const double relative = wrap_angle(plant_.pose.yaw_rad-std::atan2(ahead.y_m-behind.y_m, ahead.x_m-behind.x_m));
        return std::tan(std::clamp(relative, -1.0, 1.0));
    }

private:
    // Index of the first obstruction a predicted path enters, or none.
    std::size_t first_contact(const LocalTrajectory& trajectory) const {
        for (const auto& point : trajectory.points)
            for (std::size_t o = 0; o < footprints_.size(); ++o)
                for (const auto& cell : footprints_[o])
                    if (cell.contains(point.state.x_m, point.state.y_m)) return o;
        return obstructions_.size();
    }

    const Track& track_;
    const std::vector<PlanPoint>& reference_;
    const PlantState& plant_;
    const Config& config_;
    const VehicleModel& model_;
    const std::vector<Obstruction>& obstructions_;
    std::uint64_t ticks_;
    const SteeringTable* steering_;
    const PerformanceEnvelope* envelope_;
    std::vector<std::vector<FootprintCell>> footprints_;
    double start_s_{}, start_offset_{};
};

// Whether a station lies within an obstruction's stations widened by `before` ahead of it and the vehicle half width
// beyond it.
bool within_widened(double s, const Obstruction& o, double length, double before = vehicle_half_width_m) {
    const double span = forward_distance(o.from_s_m, o.to_s_m, length)+before+vehicle_half_width_m;
    return forward_distance(o.from_s_m-before, s, length) <= span;
}

enum class Side { left, right };

// The corridor at a station of the track.
Corridor corridor_at_station(const Track& track, double s) {
    s = std::fmod(std::fmod(s, track.length_m)+track.length_m, track.length_m);
    const auto upper = std::upper_bound(track.points.begin(), track.points.end(), s, [](double value, const PathPoint& p) { return value < p.s_m; });
    const std::size_t i = upper == track.points.begin() ? 0 : static_cast<std::size_t>(upper-track.points.begin()-1);
    const std::size_t j = (i+1)%track.points.size();
    Projection at;
    at.index = i;
    at.fraction = (s-track.points[i].s_m)/((j == 0 ? track.length_m : track.points[j].s_m)-track.points[i].s_m);
    at.s_m = s;
    return corridor_at(track, at);
}

// What a lattice path is for: passing an obstruction on one side, or, without one, returning to the reference.
struct Target {
    const Obstruction* passed{};
    Side side{Side::left};
    LocalAction action{LocalAction::straight};
};

// The cheapest lattice path from the car for a target, over the layers from its anchor to the last within the horizon.
// The car aims along straight segments between the path's points, as a path intent interpolates them, and it is that
// geometry that must keep the vehicle half width clear of every obstruction, from a pursuit distance before each, stay
// inside the corridor, pass the target's obstruction on its side, and turn at each node within the lateral budget at
// the speed there: from the car's own heading or the committed segment into the first edge, between edges, and back to
// the reference's direction after the last. Each turn also costs lattice_turn_weight per metre, times the square of the
// share of the budget it uses. The search runs over edges, since a turn depends on the edge before.
//
// The anchor is where the previous decision's path for the same action, if there is one, reaches the first layer a
// pursuit distance ahead: that stretch is kept, as TUM keeps its planned path's first segment, so a new plan cannot keep
// deferring what the last one began. Otherwise it is the node of the first layer at least a metre ahead nearest the car.
std::optional<LocalOption> cheapest_path(const Lattice& lattice, const Candidates& scene, const Target& target, const LocalOption* previous) {
    const auto& track = scene.track();
    const auto& layers = lattice.layers;
    const double length = track.length_m;
    const double here = scene.start_station();
    const auto ahead = [&](double s) { return forward_distance(here, s, length); };
    // Pure Pursuit reaches a path's offset about a pursuit distance after the path does, so a path must be clear of a
    // blockage that far before it, not only a half width.
    const double approach = vehicle_half_width_m+scene.lookahead();

    // Whether the straight segment between two points, stations running on from the car's, keeps clear and inside. Within
    // a pursuit distance of the car the path is where the car already steers, so there a blockage is widened only by the
    // half width.
    const auto clear_segment = [&](double s0, double d0, double s1, double d1) {
        const auto steps = static_cast<int>(std::max(1.0, std::ceil((s1-s0)/0.5)));
        for (int k = 0; k <= steps; ++k) {
            const double at = s0+(s1-s0)*k/steps;
            const double offset = d0+(d1-d0)*k/steps;
            const double s = std::fmod(at, length);
            const double before = at-here < scene.lookahead() ? vehicle_half_width_m : approach;
            for (const auto& o : scene.obstructions())
                if (within_widened(s, o, length, before) && offset > o.from_offset_m-vehicle_half_width_m-1e-9 &&
                    offset < o.to_offset_m+vehicle_half_width_m+1e-9)
                    return false;
            if (target.passed && within_widened(s, *target.passed, length, before) &&
                (target.side == Side::left ? offset < target.passed->to_offset_m : offset > target.passed->from_offset_m))
                return false;
            const auto corridor = corridor_at_station(track, s);
            if (offset > corridor.left_m-vehicle_half_width_m+1e-9 || -offset > corridor.right_m-vehicle_half_width_m+1e-9) return false;
        }
        return true;
    };
    // A turn from one slope to another over the mean of the two segment lengths: none if it asks for more lateral
    // acceleration than the budget even at the slowest speed the car can brake to by that station, since the path's own
    // speed profile will slow for it (decision 0023); otherwise its cost at the car's or the planned speed there,
    // whichever is higher, so the search still prefers paths the car can take without slowing.
    const auto turn = [&](double from_slope, double from_length, double to_slope, double to_length, double at) -> std::optional<double> {
        const double span = (from_length+to_length)/2;
        const double bend = std::abs(to_slope-from_slope)/span;
        const double slowest = scene.slowest_reachable(at-here);
        if (bend*slowest*slowest > scene.lateral_budget(slowest)) return std::nullopt;
        const double speed = std::max(scene.speed(), scene.planned_speed(at));
        const double share = bend*speed*speed/scene.lateral_budget(speed);
        return lattice_turn_weight*share*share*span;
    };
    const auto layer_at = [&](double s) -> std::size_t {
        const double on = std::fmod(std::fmod(s, length)+length, length);
        for (std::size_t i = 0; i < layers.size(); ++i)
            if (std::abs(layers[i].s_m-on) < 1e-6 || std::abs(layers[i].s_m-on) > length-1e-6) return i;
        return layers.size();
    };
    const auto node_at = [&](std::size_t layer, double offset) -> std::size_t {
        const auto& offsets = layers[layer].offsets_m;
        for (std::size_t n = 0; n < offsets.size(); ++n)
            if (std::abs(offsets[n]-offset) < 1e-9 && !lattice.edges_from(layer, n).empty()) return n;
        return offsets.size();
    };

    // The anchor: layer, node, the station there running on from the car's, the slope and length of the segment into it,
    // and the committed points between the car and it.
    std::size_t anchor_layer = layers.size(), anchor_node = 0;
    double anchor_station = 0, incoming_slope = scene.start_slope(), incoming_length = lattice_start_distance_m;
    std::vector<StationOffset> prefix;
    // A pass keeps only its own previous path; a return continues whatever path the car was on, so finishing a pass is
    // not abandoned for a sharp turn back the moment the reference line is clear again.
    if (previous && (previous->action == target.action || !target.passed) && previous->path.size() >= 2) {
        const auto& old = previous->path;
        const double moved = forward_distance(std::fmod(old.front().s_m, length), here, length);
        if (moved < length/2) {
            double last_s = here, last_d = scene.start_offset();
            bool clear = true;
            for (std::size_t k = 1; k < old.size() && clear; ++k) {
                const double s = here+(old[k].s_m-old.front().s_m-moved);
                if (s <= here+1e-6) continue;
                clear = clear_segment(last_s, last_d, s, old[k].offset_m);
                if (!clear) break;
                if (s-here >= scene.lookahead()) {
                    const auto layer = layer_at(old[k].s_m);
                    if (layer == layers.size()) { clear = false; break; }
                    const auto node = node_at(layer, old[k].offset_m);
                    if (node == layers[layer].offsets_m.size()) { clear = false; break; }
                    anchor_layer = layer;
                    anchor_node = node;
                    anchor_station = s;
                    incoming_length = s-last_s;
                    incoming_slope = (old[k].offset_m-last_d)/incoming_length;
                    break;
                }
                prefix.push_back({s, old[k].offset_m});
                last_s = s;
                last_d = old[k].offset_m;
            }
            if (!clear || anchor_layer == layers.size()) {
                anchor_layer = layers.size();
                prefix.clear();
                incoming_slope = scene.start_slope();
            }
        }
    }
    if (anchor_layer == layers.size()) {
        std::size_t start = 0;
        for (std::size_t i = 1; i < layers.size(); ++i) {
            const double d = ahead(layers[i].s_m), best = ahead(layers[start].s_m);
            const bool usable = d >= lattice_start_distance_m, best_usable = best >= lattice_start_distance_m;
            if ((usable && !best_usable) || (usable == best_usable && d < best)) start = i;
        }
        std::size_t node = layers[start].offsets_m.size();
        for (std::size_t n = 0; n < layers[start].offsets_m.size(); ++n) {
            if (lattice.edges_from(start, n).empty()) continue;
            if (node == layers[start].offsets_m.size() ||
                std::abs(layers[start].offsets_m[n]-scene.start_offset()) < std::abs(layers[start].offsets_m[node]-scene.start_offset()))
                node = n;
        }
        if (node == layers[start].offsets_m.size()) return std::nullopt;
        anchor_layer = start;
        anchor_node = node;
        anchor_station = here+ahead(layers[start].s_m);
        incoming_length = std::max(anchor_station-here, lattice_start_distance_m);
        if (!clear_segment(here, scene.start_offset(), anchor_station, layers[start].offsets_m[node])) return std::nullopt;
    }

    const double reach = target.passed ? std::max(lattice_planning_horizon_m,
                                                  ahead(target.passed->to_s_m)+vehicle_half_width_m+lattice_start_distance_m)
                                       : lattice_planning_horizon_m;
    std::vector<std::size_t> sequence{anchor_layer};
    std::vector<double> station{anchor_station};  // each layer's station, running on from the car's
    while (sequence.size() < layers.size()) {
        const std::size_t next = (sequence.back()+1)%layers.size();
        const double s = station.back()+forward_distance(layers[sequence.back()].s_m, layers[next].s_m, length);
        if (s-here > reach) break;
        sequence.push_back(next);
        station.push_back(s);
    }
    if (sequence.size() < 2) return std::nullopt;

    const auto offset_of = [&](const LatticeEdge& edge, bool end) {
        return layers[end ? edge.to_layer : edge.from_layer].offsets_m[end ? edge.to_node : edge.from_node];
    };
    const auto slope_of = [&](const LatticeEdge& edge, std::size_t k) {
        return (offset_of(edge, true)-offset_of(edge, false))/(station[k+1]-station[k]);
    };
    std::vector<signed char> admitted(lattice.edges.size(), -1);
    const auto index_of = [&](const LatticeEdge& edge) { return static_cast<std::size_t>(&edge-lattice.edges.data()); };
    const auto admissible = [&](const LatticeEdge& edge, std::size_t k) {
        auto& known = admitted[index_of(edge)];
        if (known < 0) known = clear_segment(station[k], offset_of(edge, false), station[k+1], offset_of(edge, true)) ? 1 : 0;
        return known == 1;
    };

    // Least cost of a path ending with each edge, its turns included, and the edge before it.
    constexpr std::size_t none = std::numeric_limits<std::size_t>::max();
    std::vector<double> cost(lattice.edges.size(), unlimited);
    std::vector<std::size_t> before(lattice.edges.size(), none);
    std::vector<std::size_t> frontier;
    for (const auto& edge : lattice.edges_from(anchor_layer, anchor_node)) {
        if (edge.to_layer != sequence[1] || !admissible(edge, 0)) continue;
        const auto turned = turn(incoming_slope, incoming_length, slope_of(edge, 0), station[1]-station[0], station[0]);
        if (!turned) continue;
        cost[index_of(edge)] = edge.cost+*turned;
        frontier.push_back(index_of(edge));
    }
    for (std::size_t k = 1; k+1 < sequence.size() && !frontier.empty(); ++k) {
        std::vector<std::size_t> next;
        for (const auto e : frontier) {
            const auto& previous_edge = lattice.edges[e];
            for (const auto& edge : lattice.edges_from(sequence[k], previous_edge.to_node)) {
                if (edge.to_layer != sequence[k+1] || !admissible(edge, k)) continue;
                const auto turned = turn(slope_of(previous_edge, k-1), station[k]-station[k-1], slope_of(edge, k), station[k+1]-station[k], station[k]);
                if (!turned) continue;
                const auto i = index_of(edge);
                const double reached = cost[e]+edge.cost+*turned;
                if (reached < cost[i]) {
                    if (!std::isfinite(cost[i])) next.push_back(i);
                    cost[i] = reached;
                    before[i] = e;
                }
            }
        }
        frontier = std::move(next);
    }
    // The path must end able to turn back parallel to the reference.
    const std::size_t last = sequence.size()-2;
    const double last_span = station[last+1]-station[last];
    std::size_t goal = none;
    double best = unlimited;
    for (const auto e : frontier) {
        const auto& edge = lattice.edges[e];
        if (edge.from_layer != sequence[last]) continue;
        const auto turned = turn(slope_of(edge, last), last_span, 0.0, last_span, station[last+1]);
        if (!turned) continue;
        const double total = cost[e]+*turned+lattice_goal_weight*std::abs(offset_of(edge, true));
        if (total < best) { best = total; goal = e; }
    }
    if (goal == none) return std::nullopt;

    std::vector<const LatticeEdge*> edges;
    for (auto e = goal; e != none; e = before[e]) edges.push_back(&lattice.edges[e]);
    std::reverse(edges.begin(), edges.end());

    LocalOption option;
    option.action = target.action;
    option.path.push_back({here, scene.start_offset()});
    for (const auto& p : prefix) option.path.push_back(p);
    option.path.push_back({station[0], offset_of(*edges.front(), false)});
    double slope = incoming_slope, span = incoming_length;
    for (std::size_t e = 0; e < edges.size(); ++e) {
        option.path_cost.edges.average_curvature += edges[e]->terms.average_curvature;
        option.path_cost.edges.curvature_range += edges[e]->terms.curvature_range;
        option.path_cost.edges.length += edges[e]->terms.length;
        option.path_cost.edges.reference_deviation += edges[e]->terms.reference_deviation;
        const double next_slope = slope_of(*edges[e], e), next_span = station[e+1]-station[e];
        option.path_cost.turns += turn(slope, span, next_slope, next_span, station[e]).value_or(0);
        slope = next_slope;
        span = next_span;
        option.path.push_back({station[e+1], offset_of(*edges[e], true)});
    }
    option.path_cost.turns += turn(slope, span, 0.0, span, station.back()).value_or(0);
    option.path_cost.goal = lattice_goal_weight*std::abs(offset_of(*edges.back(), true));
    // Where it passes its obstruction: its offset at the obstruction's first station, if the path reaches it.
    option.lateral_offset_m = option.path.back().offset_m;
    if (target.passed) {
        const double at = here+ahead(target.passed->from_s_m);
        for (std::size_t k = 1; k < option.path.size(); ++k)
            if (option.path[k].s_m >= at) {
                const auto& a = option.path[k-1];
                const auto& b = option.path[k];
                option.lateral_offset_m = a.offset_m+(at-a.s_m)/(b.s_m-a.s_m)*(b.offset_m-a.offset_m);
                break;
            }
    }
    return option;
}

// The widest gap across the corridor beside an obstruction, less the vehicle half width from each edge and from every
// obstruction over the same stations; empty when there is none.
std::optional<std::pair<double, double>> widest_gap(const Candidates& scene, const Obstruction& passed) {
    const auto& track = scene.track();
    const auto corridor = corridor_at(track, project(track, {sample(track, passed.from_s_m).x_m, sample(track, passed.from_s_m).y_m}));
    std::vector<std::pair<double, double>> blocked;
    for (const auto& o : scene.obstructions()) {
        const bool overlaps = within_widened(o.from_s_m, passed, track.length_m) || within_widened(passed.from_s_m, o, track.length_m);
        if (overlaps) blocked.push_back({o.from_offset_m-vehicle_half_width_m, o.to_offset_m+vehicle_half_width_m});
    }
    std::sort(blocked.begin(), blocked.end());
    std::optional<std::pair<double, double>> widest;
    double from = -(corridor.right_m-vehicle_half_width_m);
    const double limit = corridor.left_m-vehicle_half_width_m;
    const auto consider = [&](double a, double b) {
        if (b > a && (!widest || b-a > widest->second-widest->first)) widest = std::pair{a, b};
    };
    for (const auto& [a, b] : blocked) {
        consider(from, std::min(a, limit));
        from = std::max(from, b);
    }
    consider(from, limit);
    return widest;
}

} // namespace

std::string_view local_planner_mode_name(LocalPlannerMode mode) {
    return mode == LocalPlannerMode::five_offsets ? "five offsets" : "lattice";
}

std::string_view local_action_name(LocalAction action) {
    switch (action) {
    case LocalAction::offset: return "offset";
    case LocalAction::straight: return "straight";
    case LocalAction::pass_left: return "pass left";
    case LocalAction::pass_right: return "pass right";
    case LocalAction::brake: return "brake";
    case LocalAction::cone_path: return "cone path";
    }
    return "offset";
}

void validate_obstructions(const Track& track, const std::vector<Obstruction>& obstructions) {
    validate_track(track);
    for (const auto& o : obstructions) {
        if (!std::isfinite(o.from_s_m) || !std::isfinite(o.to_s_m) ||
            !std::isfinite(o.from_offset_m) || !std::isfinite(o.to_offset_m))
            throw std::invalid_argument("Obstruction bounds must be finite");
        if (o.from_s_m < 0 || o.from_s_m >= track.length_m || o.to_s_m < 0 || o.to_s_m >= track.length_m)
            throw std::invalid_argument("Obstruction stations must lie within [0, track length)");
        if (o.from_offset_m > o.to_offset_m)
            throw std::invalid_argument("Obstruction lateral interval must be ordered");
        const double span = forward_distance(o.from_s_m, o.to_s_m, track.length_m);
        if (span <= 0 || span >= track.length_m-1e-9)
            throw std::invalid_argument("Obstruction must cover a positive part of the circuit, not all of it");
        if (o.identifier.empty() || o.identifier.find_first_of(",\n\r") != std::string::npos)
            throw std::invalid_argument("Obstruction identifier must be nonempty and free of separators");
    }
}

LocalDecision choose_local_action(const Track& track, const std::vector<PlanPoint>& reference,
                                  const State& state, const Config& config,
                                  const std::vector<Obstruction>& obstructions,
                                  std::uint64_t ticks_to_next_control) {
    const VehicleModel kinematic = KinematicBicycle{};
    return choose_local_action(track, reference, plant_state_from(state, kinematic, config), config, kinematic,
                               obstructions, ticks_to_next_control);
}

std::size_t first_blockage_entered(const Track& track, const std::vector<Obstruction>& obstructions,
                                   const LocalTrajectory& trajectory) {
    validate_obstructions(track, obstructions);
    std::vector<std::vector<FootprintCell>> footprints;
    footprints.reserve(obstructions.size());
    for (const auto& o : obstructions) footprints.push_back(footprint(track, o));
    for (const auto& point : trajectory.points)
        for (std::size_t o = 0; o < footprints.size(); ++o)
            for (const auto& cell : footprints[o])
                if (cell.contains(point.state.x_m, point.state.y_m)) return o;
    return obstructions.size();
}

LocalDecision choose_local_action(const Track& track, const std::vector<PlanPoint>& reference,
                                  const PlantState& plant, const Config& config, const VehicleModel& model,
                                  const std::vector<Obstruction>& obstructions,
                                  std::uint64_t ticks_to_next_control, const SteeringTable* steering,
                                  const PerformanceEnvelope* envelope) {
    const Candidates scene(track, reference, plant, config, model, obstructions, ticks_to_next_control, steering, envelope);

    LocalDecision decision;
    auto reference_line = scene.offset_option(0, unlimited);
    // Keeping the reference line costs exactly one prediction, as before this planner.
    if (reference_line.clear || obstructions.empty()) {
        decision.reason = obstructions.empty() ? "Following the reference line"
                                               : "Reference line stays clear of the blockage";
        decision.options.push_back(std::move(reference_line));
        return decision;
    }

    // Alternatives reach toward each edge as far as the corridor allows where the car is.
    const auto corridor = corridor_at(track, project(track, {plant.pose.x_m, plant.pose.y_m}));
    const double usable_left = std::max(0.0, corridor.left_m-vehicle_half_width_m);
    const double usable_right = std::max(0.0, corridor.right_m-vehicle_half_width_m);
    for (const double fraction : candidate_fractions) {
        if (fraction == 0.0) { decision.options.push_back(std::move(reference_line)); continue; }
        decision.options.push_back(scene.offset_option(fraction*(fraction > 0 ? usable_left : usable_right), unlimited));
    }
    // Fastest clear alternative; ties prefer the line closest to the reference.
    std::size_t best = decision.options.size();
    for (std::size_t i = 0; i < decision.options.size(); ++i) {
        const auto& option = decision.options[i];
        if (!option.clear) continue;
        if (best == decision.options.size() ||
            option.station_gain_m > decision.options[best].station_gain_m+1e-9 ||
            (option.station_gain_m > decision.options[best].station_gain_m-1e-9 &&
             std::abs(option.lateral_offset_m) < std::abs(decision.options[best].lateral_offset_m)))
            best = i;
    }

    double nearest = unlimited;
    const std::size_t blocking = scene.nearest(nearest);
    if (blocking < obstructions.size()) decision.blocking_identifier = obstructions[blocking].identifier;

    if (best < decision.options.size()) {
        decision.selected = best;
        const auto& choice = decision.options[best];
        decision.reason = "Steering "+std::string(choice.lateral_offset_m > 0 ? "left" : "right")+
                          " around "+decision.blocking_identifier;
        return decision;
    }

    // Nothing is clear. Hold the reference line under the speed this much remaining room
    // allows, rather than committing to a blocked line. The cap only demands braking once
    // the car is close enough that it would otherwise arrive too fast; further back it is
    // a limit the car has not reached yet, and saying otherwise would overstate the action.
    auto hold = scene.offset_option(0, scene.stopping_cap(nearest));
    const bool slowing = hold.speed_limit_mps < plant.pose.speed_mps;
    const std::string action = slowing ? "braking for " : "limiting speed for ";
    hold.reason = (slowing ? "Braking for " : "Limiting speed for ")+decision.blocking_identifier;
    decision.options.push_back(std::move(hold));
    decision.selected = decision.options.size()-1;
    decision.holding = true;
    decision.reason = "No clear line within the corridor; "+action+decision.blocking_identifier;
    return decision;
}

LocalDecision choose_lattice_action(const Track& track, const std::vector<PlanPoint>& reference, const PlantState& plant,
                                    const Config& config, const VehicleModel& model, const Lattice* lattice,
                                    const std::vector<Obstruction>& obstructions, std::uint64_t ticks_to_next_control,
                                    const SteeringTable* steering, const PerformanceEnvelope* envelope, const LocalOption* previous) {
    const Candidates scene(track, reference, plant, config, model, obstructions, ticks_to_next_control, steering, envelope);
    if (!lattice && !obstructions.empty()) throw std::invalid_argument("Planning around stated blockages needs a lattice laid along the track");
    const auto laid_along = [&](const LatticeLayer& layer) {
        const auto on = sample(track, layer.s_m);
        return layer.s_m >= 0 && layer.s_m < track.length_m && std::hypot(on.x_m-layer.reference.x_m, on.y_m-layer.reference.y_m) < 1e-6;
    };
    if (lattice && (lattice->layers.empty() || !laid_along(lattice->layers.front()) || !laid_along(lattice->layers.back())))
        throw std::invalid_argument("The lattice was not laid along this track");

    LocalDecision decision;
    LocalOption reference_line;
    reference_line.action = LocalAction::straight;
    const bool touches = scene.evaluate(reference_line, {}) < obstructions.size();
    // Keeping the reference line costs exactly one prediction, as the five-offset planner's does.
    if (obstructions.empty() || (reference_line.clear && std::abs(scene.start_offset()) <= lattice_return_offset_m)) {
        decision.reason = obstructions.empty() ? "Following the reference line"
                                               : "Reference line stays clear of the blockage";
        decision.options.push_back(std::move(reference_line));
        return decision;
    }
    // Each option's speed profile to a horizon common to the decision (decision 0023), and its prediction along both. The
    // profile follows the path as Pure Pursuit drives it, cutting across what lies within a pursuit distance: from the car
    // straight to the path's first point at least a pursuit distance ahead, then along its points; for the reference line,
    // to the reference a pursuit distance ahead.
    const auto profile = [&](LocalOption& option, double horizon) {
        const double here = scene.start_station();
        const double aim = here+scene.lookahead();
        std::vector<StationOffset> pursued{{here, scene.start_offset()}};
        const auto& path = option.path;
        const auto beyond = std::find_if(path.begin(), path.end(), [&](const StationOffset& p) { return p.s_m >= aim; });
        if (beyond == path.end()) pursued.push_back({aim, path.empty() ? 0.0 : path.back().offset_m});
        else pursued.insert(pursued.end(), beyond, path.end());
        ProfileRequest request;
        request.start_s_m = here;
        request.start_slope = scene.start_slope();
        request.start_turn_m = scene.lookahead();
        request.start_speed_mps = scene.speed();
        request.horizon_m = horizon;
        request.end_speed_mps = scene.planned_speed(here+horizon);
        request.path = pursued;
        auto made = make_speed_profile(track, config, request, envelope);
        option.speed_profile = std::move(made.points);
        option.estimated_time_s = made.estimated_time_s;
        ControlIntent intent;
        intent.path = option.path;
        intent.speed_profile = option.speed_profile;
        scene.evaluate(option, intent);
    };
    const auto reach = [&](const LocalOption& option) { return option.path.back().s_m-scene.start_station(); };
    // The clear option of least estimated time, the cheaper path on a tie; but the previous decision's choice, the same
    // action along a path or not, while it is clear and no more than lattice_switch_margin_s slower. None if nothing is
    // clear.
    constexpr std::size_t none = std::numeric_limits<std::size_t>::max();
    const auto fastest = [&](std::size_t from) {
        std::size_t best = none, kept = none;
        for (std::size_t i = from; i < decision.options.size(); ++i) {
            const auto& option = decision.options[i];
            if (!option.clear) continue;
            if (previous && option.action == previous->action && option.path.empty() == previous->path.empty()) kept = i;
            if (best == none) { best = i; continue; }
            const auto& chosen = decision.options[best];
            const double tie = 1e-9*std::max(1.0, chosen.estimated_time_s);
            if (option.estimated_time_s < chosen.estimated_time_s-tie ||
                (option.estimated_time_s <= chosen.estimated_time_s+tie && option.path_cost.total() < chosen.path_cost.total()))
                best = i;
        }
        if (kept != none && decision.options[kept].estimated_time_s <= decision.options[best].estimated_time_s+lattice_switch_margin_s) return kept;
        return best;
    };
    // How the chosen option's estimated time compares with another clear option's, for the decision's reason.
    const auto against = [&](const LocalOption& chosen, const LocalOption& other, const std::string& name) {
        const double difference = other.estimated_time_s-chosen.estimated_time_s;
        return difference >= 0 ? "; "+seconds(difference)+" faster than "+name : "; kept within "+seconds(-difference)+" of "+name;
    };

    if (!touches) {
        // Clear of every blockage but well off the reference, or turning back to it too sharply, as after passing one:
        // return along the lattice or rejoin the reference directly, whichever is faster. Rejoining is always weighed,
        // and always along its own speed profile, so that a car that must turn back sharply slows for that turn even when
        // the lattice offers no way home.
        auto home = cheapest_path(*lattice, scene, {nullptr, Side::left, LocalAction::straight}, previous);
        const double horizon = std::max(lattice_planning_horizon_m, home ? reach(*home) : 0.0);
        LocalOption direct = reference_line;
        profile(direct, horizon);
        decision.options.push_back(std::move(direct));
        if (home) {
            profile(*home, horizon);
            decision.options.push_back(std::move(*home));
        }
        const auto best = fastest(0);
        // With neither way home clear, as when grip falls while the car is still out on a path, turning back to the
        // reference is the one thing not to do: it asks for a sharper turn than the path the car is already on, which
        // nothing has said is worse. Keep that path, and say that neither is clear.
        const std::size_t committed = decision.options.size() > 1 && previous && !previous->path.empty() ? 1 : 0;
        decision.selected = best == none ? committed : best;
        decision.reason = decision.selected == 1 ? "Returning to the reference along the lattice" : "Rejoining the reference line directly";
        if (decision.options.size() > 1) {
            const auto& other = decision.options[1-decision.selected];
            if (other.clear && decision.options[decision.selected].clear)
                decision.reason += against(decision.options[decision.selected], other,
                                           decision.selected == 1 ? "rejoining it directly" : "returning along the lattice");
        }
        if (best == none) decision.reason += "; no way home is clear";
        return decision;
    }
    decision.options.push_back(std::move(reference_line));

    double distance = unlimited;
    const std::size_t blocking = scene.nearest(distance);
    const auto& passed = obstructions[blocking];
    decision.blocking_identifier = passed.identifier;

    std::vector<LocalOption> passes;
    for (const Side side : {Side::left, Side::right}) {
        auto found = cheapest_path(*lattice, scene, {&passed, side, side == Side::left ? LocalAction::pass_left : LocalAction::pass_right}, previous);
        if (found) passes.push_back(std::move(*found));
    }
    double horizon = lattice_planning_horizon_m;
    for (const auto& pass : passes) horizon = std::max(horizon, reach(pass));
    for (auto& pass : passes) {
        profile(pass, horizon);
        decision.options.push_back(std::move(pass));
    }
    const auto best = fastest(1);
    if (best != none) {
        decision.selected = best;
        const auto side = [&](const LocalOption& option) { return std::string(option.action == LocalAction::pass_left ? "left" : "right"); };
        const auto& chosen = decision.options[best];
        decision.reason = "Passing "+side(chosen)+" of "+passed.identifier+" along the lattice";
        for (std::size_t i = 1; i < decision.options.size(); ++i)
            if (i != best && decision.options[i].clear) decision.reason += against(chosen, decision.options[i], "passing "+side(decision.options[i]));
        return decision;
    }

    // No lattice path is clear: follow the gap. Brake for the blockage while aiming at the middle of the widest gap
    // beside it, or along the reference when there is none.
    const auto gap = widest_gap(scene, passed);
    const double aim = gap ? (gap->first+gap->second)/2 : 0.0;
    auto brake = scene.offset_option(aim, scene.stopping_cap(distance));
    brake.action = LocalAction::brake;
    const bool slowing = brake.speed_limit_mps < scene.speed();
    brake.reason = (slowing ? "Braking for " : "Limiting speed for ")+passed.identifier;
    const std::string doing = slowing ? "braking" : "limiting speed";
    decision.reason = gap ? "No lattice path past "+passed.identifier+"; "+doing+" toward the widest gap at "+metres(std::abs(aim))+
                                (aim >= 0 ? " left" : " right")
                          : "No lattice path past "+passed.identifier+" and no gap across the corridor; "+doing+" for "+passed.identifier;
    decision.options.push_back(std::move(brake));
    decision.selected = decision.options.size()-1;
    decision.holding = true;
    return decision;
}

} // namespace fd

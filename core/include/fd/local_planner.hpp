#pragma once
#include "fd/lattice.hpp"
#include "fd/speed_profile.hpp"
#include "fd/trajectory.hpp"

#include <string_view>

namespace fd {

// A blocked region of the corridor supplied by the scenario. This is stated ground
// truth, not perception output: nothing here detects, classifies or tracks anything,
// and calling one of these a pedestrian would be a claim the code does not support.
struct Obstruction {
    // Station interval along the reference centerline. from_s_m > to_s_m wraps the seam.
    double from_s_m{}, to_s_m{};
    // Signed lateral interval, metres left of the centerline; from <= to.
    double from_offset_m{}, to_offset_m{};
    std::string identifier{"obstruction"};
};

// What an option does. The five-offset planner's options are offsets from the reference (decision 0004); the lattice
// planner's are actions (decision 0022): follow the reference straight on, pass a blockage on its left or right along a
// lattice path, or brake toward the widest gap. Driving on cones (decision 0031) the one action is to follow the path the
// car believes from what it perceived.
enum class LocalAction { offset, straight, pass_left, pass_right, brake, cone_path };
// "offset", "straight", "pass left", "pass right", "brake" or "cone path".
std::string_view local_action_name(LocalAction action);

// Which planner chooses the car's action: the lattice planner (decision 0022), or the five-offset planner it replaces,
// kept for comparison (decision 0004).
enum class LocalPlannerMode { lattice, five_offsets };
// "lattice" or "five offsets".
std::string_view local_planner_mode_name(LocalPlannerMode mode);

// A lattice path's cost: its edges' offline cost terms, and the goal term for how far its last node lies off the
// reference.
struct LatticePathCost {
    LatticeCostTerms edges;
    double turns{};  // what the path's turns cost, lattice_turn_weight per metre times their squared share of the budget
    double goal{};
    double total() const { return edges.total()+turns+goal; }
};

// One alternative the planner evaluated, kept so the interface can explain itself.
struct LocalOption {
    double lateral_offset_m{};       // for a lattice path, where it passes the blockage; for a brake, the gap aimed at
    double speed_limit_mps{};        // infinity when the reference envelope governs
    LocalTrajectory trajectory;
    bool clear{};                    // inside the model envelope and free of obstructions
    double station_gain_m{};         // reference distance advanced over the horizon
    std::string reason;
    LocalAction action{LocalAction::offset};
    std::vector<StationOffset> path; // the lattice path aimed along, from the car's own station and offset; else empty
    LatticePathCost path_cost;       // zero without a path
    // The speed profile the option was predicted with (decision 0023), from the car to the decision's common horizon,
    // and the time it estimates; empty and infinite when the option follows the reference plan's speed.
    std::vector<ProfilePoint> speed_profile;
    double estimated_time_s{std::numeric_limits<double>::infinity()};
};

// What the car decided to do for the next control interval, and what it rejected.
struct LocalDecision {
    std::vector<LocalOption> options;  // never empty: the five-offset planner's offsets ascending, then any hold; the
                                       // lattice planner's reference line, then the paths and the brake it tried
    std::size_t selected{};
    bool holding{};                    // nothing was clear: braking for the blockage
    std::string blocking_identifier;   // the obstruction that forced a change, if any
    std::string reason;
    const LocalOption& choice() const { return options.at(selected); }
    const LocalTrajectory& trajectory() const { return choice().trajectory; }
};

// Lateral allowance for the vehicle body around its rear-axle reference point. A
// margin, not a swept-body collision model.
inline constexpr double vehicle_half_width_m = 0.9;
// Clearance kept in front of a blockage when no alternative line is clear.
inline constexpr double obstruction_stop_margin_m = 1.0;
// Share of the longitudinal grip budget the approach plans to use. Planning against the
// full budget would put the target exactly on the maximum-braking curve, leaving the
// controller no authority to correct its own tracking lag, so it would arrive late and
// overrun. The remainder is that authority, not a claim about tire behaviour.
inline constexpr double obstruction_braking_fraction = 0.7;

// Choose this control interval's action: keep the reference line when it stays clear,
// otherwise take the fastest lateral alternative that is clear, otherwise brake for
// the blockage. "Fastest" means the greatest predicted advance along the reference
// within the fixed horizon; offsets do not re-derive curvature limits, so this selects
// among feasible lines rather than solving for a minimum-time racing line.
//
// Obstruction contact is tested at the predicted control points against a sampled
// rectangular footprint inflated by the vehicle margin. It is neither continuous nor
// swept-body collision detection, and a clear result is not a safety guarantee.
//
// With no relevant obstruction this evaluates exactly one option and reproduces plain
// reference following, so the ordinary driving cost is unchanged.
LocalDecision choose_local_action(const Track& track, const std::vector<PlanPoint>& reference,
                                  const State& state, const Config& config,
                                  const std::vector<Obstruction>& obstructions,
                                  std::uint64_t ticks_to_next_control = 0);
// The same choice for any vehicle model, predicting every candidate with that model and, when a
// steering table is given, steering every candidate with MAP. With the performance envelope the
// reference was planned from, every candidate is controlled within it, and a hold brakes toward the
// clearance at a fraction of the car's straight-line braking capacity at the grid's lowest speed, the
// least it has on the way down, rather than of the longitudinal grip fraction.
LocalDecision choose_local_action(const Track& track, const std::vector<PlanPoint>& reference,
                                  const PlantState& state, const Config& config, const VehicleModel& model,
                                  const std::vector<Obstruction>& obstructions,
                                  std::uint64_t ticks_to_next_control = 0,
                                  const SteeringTable* steering = nullptr,
                                  const PerformanceEnvelope* envelope = nullptr);

// How far ahead the lattice planner searches from the car's station, at least; it reaches past the blockage it passes.
inline constexpr double lattice_planning_horizon_m = 60.0;
// Cost per metre that a lattice path's last node lies off the reference, TUM's virtual goal weight (ForzaETH's value).
inline constexpr double lattice_goal_weight = 10000.0;
// Cost per metre of a turn that uses the whole lateral budget, scaled by the square of the share it uses.
inline constexpr double lattice_turn_weight = 50000.0;
// A car further than this off the reference, with the reference clear, returns to it along a lattice path.
inline constexpr double lattice_return_offset_m = 1.0;
// A decision keeps the previous decision's choice while it stays clear, unless another clear option is estimated faster
// by more than this (decision 0023): about the error of an estimate over the planning horizon measured on the preset,
// so options within that error of each other do not swap from one control tick to the next.
inline constexpr double lattice_switch_margin_s = 0.05;

// The lattice planner (TrackWayFastPlan Phases 5.2 and 5.3, decisions 0022 and 0023), after Stahl et al. 2019's online
// graph search and TUM's forward-backward velocity planner, for any vehicle model as the five-offset planner is. Each
// call predicts the reference line first.
//
// - Without stated blockages, or with the reference line clear and the car within lattice_return_offset_m of it, it
//   follows the reference straight on with that one prediction at the reference plan's speed, exactly as
//   choose_local_action does.
// - With the reference line clear of every blockage but the car further off it, or the line failing only the model
//   envelope, it weighs returning along the cheapest lattice path that ends on the reference against rejoining the
//   reference directly, and takes the clear one of least estimated time; if neither is clear it follows the reference.
// - When the reference line enters a blockage it searches for the cheapest path passing the nearest one on its left and
//   the cheapest passing it on its right, and takes the clear one of least estimated time, the cheaper on a tie; but it
//   keeps the previous decision's choice while that is clear and no more than lattice_switch_margin_s slower.
// - When neither is clear it follows the gap: it brakes for the blockage under the five-offset planner's speed cap while
//   aiming at the centre of the widest gap across the corridor beside it, less the half width, or along the reference
//   when there is none.
//
// The search starts from the node, nearest the car's offset, of the first layer at least a metre ahead, and runs over
// the layers within lattice_planning_horizon_m or just past the blockage passed. A path is straight segments between
// nodes, which Pure Pursuit follows as a path intent. A segment is admissible when it stays inside the corridor less the
// half width and clear of every blockage widened by the half width, and, beyond a pursuit distance from the car, by a
// pursuit distance before it, since the car reaches a path's offset about a pursuit distance late; a pass also keeps to
// its side of the blockage it passes. A path costs its edges' offline cost, plus lattice_turn_weight per metre times the
// squared share of the lateral budget each change of slope asks for at the car's or the planned speed there, whichever
// is higher, plus lattice_goal_weight per metre its last node lies off the reference. A turn is not allowed if it asks
// for more than the budget even at the slowest speed the car can brake to by then, including the one back parallel to the
// reference at the end. The budget is the envelope fraction of the performance envelope's lateral limit when the plan
// uses one, otherwise the lateral grip fraction of the grip. Given `previous`, the choice the last decision made, a new
// path for the same action keeps that path's points within a pursuit distance of the car when they are still clear, and
// continues from the node it had reached there, so it cannot keep deferring a turn the last plan began.
//
// Every option compared gets a speed profile (make_speed_profile) along the path as Pure Pursuit drives it, cutting
// across what lies nearer than a pursuit distance: from the car straight to the path's first point at least a pursuit
// distance ahead, then along its points; for the reference line, to the reference a pursuit distance ahead. Each runs from the car's actual speed to a horizon common to the decision, the
// farthest path's end, and ends no faster than the reference plan there. Its time is the option's estimated time, and the
// option is predicted following that profile, so it is checked at the speed it would be driven. This compares estimated
// times over one decision's horizon among each action's cheapest path; it does not search the lattice for the fastest
// path. Like the five-offset planner it checks contact at predicted control points, not a swept body. `lattice` must
// have been laid along `track`; one laid along another is refused. Without stated blockages no lattice is needed and
// `lattice` may be null; with them a null lattice is refused.
LocalDecision choose_lattice_action(const Track& track, const std::vector<PlanPoint>& reference, const PlantState& state,
                                    const Config& config, const VehicleModel& model, const Lattice* lattice,
                                    const std::vector<Obstruction>& obstructions, std::uint64_t ticks_to_next_control = 0,
                                    const SteeringTable* steering = nullptr, const PerformanceEnvelope* envelope = nullptr,
                                    const LocalOption* previous = nullptr);

// Rejects non-finite bounds, stations outside [0, length), inverted lateral intervals
// and a region that spans the whole circuit.
void validate_obstructions(const Track& track, const std::vector<Obstruction>& obstructions);

// The index of the first stated blockage a prediction enters, checked at its points against each blocked region inflated
// by the vehicle half width, exactly as both planners check every option; obstructions.size() when it enters none. For
// a prediction the planners did not make themselves, such as a predictive controller's horizon (decision 0024).
std::size_t first_blockage_entered(const Track& track, const std::vector<Obstruction>& obstructions,
                                   const LocalTrajectory& trajectory);

} // namespace fd

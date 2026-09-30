#include "fd/performance_envelope.hpp"
#include "fd/simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Phase 3.2 (decision 0018): a periodic speed plan from the car's own G-G-V envelope. The oracles are the envelope's
// public queries: every sample within the configured share of the lateral limit at its speed, and every edge, the
// start/finish seam included, within that share of the forward or braking capacity at the speed and lateral use it
// starts from. The share is the envelope shrunk toward its origin: a point is within it when the point divided by the
// share is within the envelope.
namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tolerance)
        throw std::runtime_error(message+": actual="+std::to_string(actual)+", expected="+std::to_string(expected));
}

const fd::Config config;
const fd::Track track = fd::make_preset_track();

// Derived once per car and configuration, because each derivation runs the plant for minutes of simulated time.
std::map<std::string, fd::PerformanceEnvelope> derived;
const fd::PerformanceEnvelope& envelope_for(const std::string& name, const fd::FourWheelCar& car, const fd::Config& with = config) {
    if (!derived.contains(name)) {
        const auto began = std::chrono::steady_clock::now();
        fd::PerformanceEnvelope envelope(car, with);
        std::cout << "  derived the " << name << " envelope in "
                  << std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count() << " s (measurement only)\n";
        derived.emplace(name, std::move(envelope));
    }
    return derived.at(name);
}

double segment(const fd::Track& t, std::size_t i) {
    return i+1 < t.points.size() ? t.points[i+1].s_m-t.points[i].s_m : t.length_m-t.points[i].s_m;
}

// Every sample within the share of its lateral limit, and every edge within the share of the capacity at the end it is
// planned from: forward from the sample it leaves, braking into the sample it reaches. Holding speed within the lateral
// limit is always feasible, because the limit is the lateral acceleration of a turn the plant held steady.
void require_within_envelope(const fd::Track& t, const std::vector<fd::PlanPoint>& plan, const fd::PerformanceEnvelope& envelope,
                             const fd::Config& with) {
    const double f = with.envelope_fraction;
    require(plan.size() == t.points.size(), "one plan point per track sample");
    for (std::size_t i = 0; i < plan.size(); ++i) {
        const std::size_t j = (i+1)%plan.size();
        const double vi = plan[i].speed_mps, vj = plan[j].speed_mps;
        const double ayi = vi*vi*t.points[i].curvature, ayj = vj*vj*t.points[j].curvature;
        const auto side = ayi >= 0 ? fd::TurnSide::left : fd::TurnSide::right;
        require(vi >= 0 && vi <= with.max_speed_mps+1e-12, "planned speed within the speed cap at "+std::to_string(i));
        require(std::abs(ayi) <= f*envelope.lateral_limit(vi, side)+1e-9, "lateral acceleration within the envelope at sample "+std::to_string(i));
        const double a = (vj*vj-vi*vi)/(2*segment(t, i));
        near(plan[i].acceleration_mps2, a, 1e-9, "planned acceleration is the edge's");
        if (a > 0)
            require(a <= f*std::max(0.0, envelope.forward_limit(vi, ayi/f))+1e-7, "acceleration within the forward limit leaving sample "+std::to_string(i));
        if (a < 0)
            require(-a <= f*std::max(0.0, -envelope.braking_limit(vj, ayj/f))+1e-7, "braking within the braking limit into sample "+std::to_string(j));
        require(plan[i].limiting_index < plan.size(), "limiting index is valid");
    }
}

// Where braking for the preset's first corner begins: the first sample from the start line planned to brake.
std::size_t first_corner() {
    const auto corner = std::find_if(track.points.begin(), track.points.end(), [](const fd::PathPoint& p) { return p.curvature > 0; });
    return static_cast<std::size_t>(corner-track.points.begin());
}
double braking_start(const std::vector<fd::PlanPoint>& plan) {
    for (std::size_t i = 0; i < first_corner(); ++i)
        if (plan[i].acceleration_mps2 < -fd::plan_color_deadband_mps2) return track.points[i].s_m;
    throw std::runtime_error("no braking before the first corner");
}
// Time to drive the plan once, each edge at the mean of its two speeds.
double lap_estimate(const fd::Track& t, const std::vector<fd::PlanPoint>& plan) { return fd::estimated_lap_time(t, plan); }
fd::Track circle(double radius) {
    fd::Track t;
    t.name = "circle";
    const auto count = static_cast<int>(std::ceil(2*3.141592653589793*radius/0.6));
    for (int i = 0; i < count; ++i) {
        const double angle = 2*3.141592653589793*i/count;
        t.points.push_back({radius*std::sin(angle), radius*(1-std::cos(angle)), radius*angle, 1/radius});
    }
    t.length_m = 2*3.141592653589793*radius;
    return t;
}

void planned_speed_never_exceeds_the_envelope() {
    const auto& envelope = envelope_for("default", fd::FourWheelCar{});
    const auto plan = fd::make_speed_plan(track, config, &envelope);
    require_within_envelope(track, plan, envelope, config);
    const auto slowest = std::min_element(plan.begin(), plan.end(), [](const auto& a, const auto& b) { return a.speed_mps < b.speed_mps; });
    std::cout << "  default car: slowest planned speed " << slowest->speed_mps << " m/s at " << track.points[static_cast<std::size_t>(slowest-plan.begin())].s_m << " m\n";
}

// The car's own capacity is well beyond the baseline's reserve: corners are taken faster, braking begins later and
// the plan drives the preset in less time.
void uses_more_of_the_car_than_the_grip_fractions() {
    const auto& envelope = envelope_for("default", fd::FourWheelCar{});
    const auto baseline = fd::make_speed_plan(track, config);
    const auto planned = fd::make_speed_plan(track, config, &envelope);
    const auto corner = first_corner();
    std::cout << "  first corner at " << track.points[corner].s_m << " m: grip fractions " << baseline[corner].speed_mps << " m/s braking from "
              << braking_start(baseline) << " m, envelope " << planned[corner].speed_mps << " m/s braking from " << braking_start(planned)
              << " m; plan lap " << lap_estimate(track, baseline) << " s against " << lap_estimate(track, planned) << " s\n";
    require(planned[corner].speed_mps > baseline[corner].speed_mps+0.5, "the envelope takes the first corner faster");
    require(braking_start(planned) > braking_start(baseline)+3, "the envelope brakes later for it");
    require(lap_estimate(track, planned) < lap_estimate(track, baseline)-1, "the envelope plan drives the preset in less time");
}

// The estimate covers each span at constant acceleration between its planned speeds, the closing span included: a circle
// planned at one speed takes its length over that speed; speeds rising along a span take its length over their mean.
void the_estimated_lap_follows_the_plan() {
    const auto round = circle(30);
    std::vector<fd::PlanPoint> steady(round.points.size());
    for (auto& point : steady) point.speed_mps = 12;
    near(fd::estimated_lap_time(round, steady), round.length_m/12, 1e-9, "a steady plan takes the length over its speed");
    auto rising = steady;
    rising.front().speed_mps = 14;
    const double span = round.points[1].s_m;
    const double closing = round.length_m-round.points.back().s_m;
    near(fd::estimated_lap_time(round, rising), (round.length_m-span-closing)/12+2*span/26+2*closing/26, 1e-9,
         "a faster sample shortens both spans it ends, the closing one included");
    const auto refused = [](const std::function<void()>& action) {
        try { action(); } catch (const std::invalid_argument& error) { return std::string(error.what()); }
        return std::string();
    };
    auto short_plan = steady;
    short_plan.pop_back();
    require(refused([&] { fd::estimated_lap_time(round, short_plan); }).find("one plan point per track sample") != std::string::npos,
            "a plan for another track is refused");
    auto standing = steady;
    standing[3].speed_mps = 0;
    standing[4].speed_mps = 0;
    require(refused([&] { fd::estimated_lap_time(round, standing); }).find("moves along every span") != std::string::npos,
            "a span planned at rest at both ends is refused");
}

// The plan's test: lower grip still moves braking earlier on the same scenario, now through the car's envelope.
void lower_grip_moves_braking_earlier() {
    auto slippery = config;
    slippery.grip_mu = 0.7;
    const auto& grippy = envelope_for("default", fd::FourWheelCar{});
    const auto& low = envelope_for("default at grip 0.7", fd::FourWheelCar{}, slippery);
    const auto high_plan = fd::make_speed_plan(track, config, &grippy);
    const auto low_plan = fd::make_speed_plan(track, slippery, &low);
    require_within_envelope(track, low_plan, low, slippery);
    const auto corner = first_corner();
    std::cout << "  grip 1.0 brakes from " << braking_start(high_plan) << " m for " << high_plan[corner].speed_mps << " m/s, grip 0.7 from "
              << braking_start(low_plan) << " m for " << low_plan[corner].speed_mps << " m/s\n";
    require(low_plan[corner].speed_mps < high_plan[corner].speed_mps-1, "lower grip lowers the corner speed");
    require(braking_start(low_plan) < braking_start(high_plan)-2, "lower grip moves braking earlier");
}

// What Phase 2.4's setup controls could not do: brake bias, height, mass and downforce now move braking points.
void the_cars_setup_moves_braking_points() {
    fd::FourWheelCar matched, tall, heavy, winged;
    matched.brake_bias_front = 0.62;
    tall.cg_height_m = 0.6;
    heavy.mass_kg = 1200;
    heavy.yaw_inertia_kgm2 *= 1200/800.0;
    winged.downforce_area_m2 = 3;
    const auto start = [&](const std::string& name, const fd::FourWheelCar& car) {
        const auto& envelope = envelope_for(name, car);
        const auto plan = fd::make_speed_plan(track, config, &envelope);
        require_within_envelope(track, plan, envelope, config);
        std::cout << "  " << name << ": braking from " << braking_start(plan) << " m, first corner " << plan[first_corner()].speed_mps
                  << " m/s, plan lap " << lap_estimate(track, plan) << " s\n";
        return std::pair{braking_start(plan), plan[first_corner()].speed_mps};
    };
    const auto standard = start("default", fd::FourWheelCar{});
    require(start("matched brake bias", matched).first > standard.first+0.5, "a brake bias matched to the braking load brakes later");
    require(start("tall", tall).first < standard.first-0.5, "a taller car brakes earlier");
    require(start("heavy", heavy).first < standard.first-0.5, "a heavier car brakes earlier");
    const auto wings = start("winged", winged);
    require(wings.second > standard.second+0.1 && wings.first > standard.first+0.5, "downforce takes the corner faster and brakes later");
}

void refuses_an_envelope_for_another_configuration() {
    const auto& envelope = envelope_for("default", fd::FourWheelCar{});
    auto slippery = config;
    slippery.grip_mu = 0.7;
    bool rejected = false;
    try { fd::make_speed_plan(track, slippery, &envelope); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "an envelope derived at another grip is rejected rather than planned with");
}

// On a circle every sample asks the same lateral acceleration, so the periodic plan holds one speed, the one at which
// the circle's lateral acceleration meets the lateral limit.
void a_circle_is_planned_at_its_lateral_limit() {
    const auto& envelope = envelope_for("default", fd::FourWheelCar{});
    const auto ring = circle(30);
    auto whole = config;
    whole.envelope_fraction = 1;
    for (const auto& with : {config, whole}) {
        const auto plan = fd::make_speed_plan(ring, with, &envelope);
        require_within_envelope(ring, plan, envelope, with);
        const double v = plan.front().speed_mps;
        for (const auto& point : plan) near(point.speed_mps, v, 1e-9, "the circle is planned at one speed");
        near(v*v/30, with.envelope_fraction*envelope.lateral_limit(v, fd::TurnSide::left), 1e-6, "that speed meets the share of the lateral limit");
        require(plan.front().reason == "curvature_limit", "the lateral limit is named as the constraint");
        std::cout << "  30 m circle at " << with.envelope_fraction << " of the envelope: " << v << " m/s, " << v*v/30 << " m/s^2\n";
    }
}

// A plan the controller cannot follow is no plan. Under the grip fractions the command is bounded by the longitudinal
// fraction; under the envelope it is bounded by the car's straight-line forward and braking capacity at its speed.
void the_controller_commands_up_to_the_cars_capacity() {
    const auto& envelope = envelope_for("default", fd::FourWheelCar{});
    auto standing = fd::make_speed_plan(track, config, &envelope);
    for (auto& point : standing) { point.speed_mps = 0; point.acceleration_mps2 = 0; }
    auto flat_out = standing;
    for (auto& point : flat_out) point.speed_mps = config.max_speed_mps;
    const auto& a = track.points[0];
    const auto& b = track.points[1];
    const auto on_the_line = [&](double speed) { return fd::State{a.x_m, a.y_m, std::atan2(b.y_m-a.y_m, b.x_m-a.x_m), speed, 0, 0}; };
    const double budget = config.longitudinal_grip_fraction*config.grip_mu*fd::gravity_mps2;

    const auto baseline = fd::compute_control(track, standing, on_the_line(10), config);
    near(baseline.applied.acceleration_mps2, -budget, 1e-12, "the grip fractions bound braking by the longitudinal fraction");
    const auto braking = fd::compute_control(track, standing, on_the_line(10), config, {}, nullptr, &envelope);
    near(braking.applied.acceleration_mps2, envelope.braking_limit(10, 0), 1e-12, "the envelope bounds braking by the car's capacity at its speed");
    const auto driving = fd::compute_control(track, flat_out, on_the_line(5), config, {}, nullptr, &envelope);
    near(driving.applied.acceleration_mps2, envelope.forward_limit(5, 0), 1e-12, "and driving likewise");
    require(braking.applied.acceleration_mps2 < -budget-1 && driving.applied.acceleration_mps2 > budget+1, "both beyond the baseline's budget");
    // In a corner, driving is bounded by the forward limit at the lateral acceleration the reference asks there, so pulling
    // out of a bend flat out does not spin the unloaded inside wheel; braking keeps the whole straight-line capacity, so
    // the car can always slow down (decision 0020).
    const auto corner = std::next(std::find_if(track.points.begin(), track.points.end(), [](const fd::PathPoint& p) { return p.curvature > 0.05; }), 5);
    const auto after = std::next(corner);
    const fd::State in_corner{corner->x_m, corner->y_m, std::atan2(after->y_m-corner->y_m, after->x_m-corner->x_m), 12, 0, 0};
    const auto cornering = fd::compute_control(track, flat_out, in_corner, config, {}, nullptr, &envelope);
    const double lateral = 144*track.points[cornering.reference.index].curvature;
    require(lateral > 144*0.05, "the reference is inside the corner");
    std::cout << "  at 12 m/s in the 18 m corner, " << lateral << " m/s^2 lateral: driving bounded at " << cornering.applied.acceleration_mps2
              << " m/s^2 against " << envelope.forward_limit(12, 0) << " straight ahead\n";
    near(cornering.applied.acceleration_mps2, std::max(0.0, envelope.forward_limit(12, lateral)), 1e-12, "driving in a corner is bounded at its lateral acceleration");
    require(cornering.applied.acceleration_mps2 < envelope.forward_limit(12, 0)-1, "which leaves less than straight ahead");
    const auto braking_in_corner = fd::compute_control(track, standing, in_corner, config, {}, nullptr, &envelope);
    near(braking_in_corner.applied.acceleration_mps2, envelope.braking_limit(12, 0), 1e-12, "braking in a corner keeps the straight-line capacity");
    std::cout << "  command bounds at 10 m/s braking " << braking.applied.acceleration_mps2 << ", at 5 m/s driving " << driving.applied.acceleration_mps2
              << " m/s^2, against the grip fraction's " << budget << "\n";
}

// The simulation plans with the envelope it derived for its own car and configuration, controls every decision within
// it, and derives both again when a grip change commits, as one new revision.
void the_simulation_plans_with_the_envelope_it_derives() {
    fd::Simulation simulation(track, config, fd::FourWheelCar{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::performance_envelope);
    require(simulation.speed_plan_mode() == fd::SpeedPlanMode::performance_envelope, "the simulation reports its speed plan mode");
    const auto* envelope = simulation.performance_envelope();
    require(envelope && envelope->generated_for(simulation.vehicle_model(), simulation.config()), "it derived the envelope for its car");
    require(envelope->fingerprint() == envelope_for("default", fd::FourWheelCar{}).fingerprint(), "the same envelope a direct derivation gives");
    const auto expected = fd::make_speed_plan(track, config, envelope);
    require(simulation.plan().size() == expected.size(), "the plan covers the track");
    for (std::size_t i = 0; i < expected.size(); ++i)
        require(simulation.plan()[i].speed_mps == expected[i].speed_mps && simulation.plan()[i].reason == expected[i].reason, "the plan is the envelope plan");
    simulation.step();
    near(simulation.diagnostics().applied.acceleration_mps2, envelope->forward_limit(0, 0), 1e-12, "pulling away is bounded by the envelope, not the grip fraction");

    simulation.set_grip(0.8);
    simulation.step();
    const auto* regenerated = simulation.performance_envelope();
    require(simulation.config().grip_mu == 0.8 && simulation.diagnostics().revision == 2, "the grip change commits as revision 2");
    require(regenerated && regenerated->generated_for(simulation.vehicle_model(), simulation.config()) && regenerated->fingerprint() != envelope_for("default", fd::FourWheelCar{}).fingerprint(),
            "with an envelope derived for the new grip");
    const auto replanned = fd::make_speed_plan(track, simulation.config(), regenerated);
    for (std::size_t i = 0; i < replanned.size(); ++i)
        require(simulation.plan()[i].speed_mps == replanned[i].speed_mps, "and the plan made from it");

    fd::Simulation baseline(track, config, fd::FourWheelCar{});
    require(baseline.speed_plan_mode() == fd::SpeedPlanMode::grip_fractions && !baseline.performance_envelope(), "the grip fractions remain the default");
    bool refused = false;
    try { fd::Simulation kinematic(track, config, fd::KinematicBicycle{}, fd::SteeringMode::pure_pursuit, fd::SpeedPlanMode::performance_envelope); }
    catch (const std::invalid_argument&) { refused = true; }
    require(refused, "a model without an envelope cannot plan with one");
    require(std::string(fd::speed_plan_mode_name(fd::SpeedPlanMode::performance_envelope)) == "performance envelope" &&
            std::string(fd::speed_plan_mode_name(fd::SpeedPlanMode::grip_fractions)) == "grip fractions", "modes have names");
}

// The share is an effective control: planning with all of the envelope is faster than with the default share, which
// is faster than the grip fractions.
void the_share_of_the_envelope_moves_the_plan() {
    const auto& envelope = envelope_for("default", fd::FourWheelCar{});
    auto whole = config;
    whole.envelope_fraction = 1;
    const auto all = fd::make_speed_plan(track, whole, &envelope);
    const auto share = fd::make_speed_plan(track, config, &envelope);
    const auto baseline = fd::make_speed_plan(track, config);
    require_within_envelope(track, all, envelope, whole);
    std::cout << "  plan lap: whole envelope " << lap_estimate(track, all) << " s, default share " << config.envelope_fraction << " "
              << lap_estimate(track, share) << " s, grip fractions " << lap_estimate(track, baseline) << " s\n";
    require(lap_estimate(track, all) < lap_estimate(track, share)-0.5 && lap_estimate(track, share) < lap_estimate(track, baseline)-0.5,
            "more of the envelope plans a faster lap");
    auto bad = config;
    for (const double fraction : {0.4, 1.01}) {
        bad.envelope_fraction = fraction;
        bool rejected = false;
        try { fd::validate_config(bad); } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "an envelope share outside 0.5 to 1 is rejected");
    }
}

// Why the default share is not all of it. Driven closed loop on the preset for two laps, the default share keeps every
// tire within its peak and still laps faster than the grip fractions; planning with the whole envelope does not keep
// the tires within their peak, because Pure Pursuit turns in before a corner and drives a tighter line than the
// reference, which a plan at the limit leaves no room for.
void the_default_share_keeps_the_tires_within_their_peak() {
    struct Outcome { double second_lap{-1}; int beyond{}; double worst_error{}; };
    const auto drive = [&](fd::SpeedPlanMode mode, const fd::Config& with) {
        fd::Simulation simulation(track, with, fd::FourWheelCar{}, fd::SteeringMode::pure_pursuit, mode);
        Outcome outcome;
        double first = -1;
        while (simulation.state().time_s < 70) {
            simulation.step();
            const auto& d = simulation.diagnostics();
            if (!d.within_grip_envelope) ++outcome.beyond;
            outcome.worst_error = std::max(outcome.worst_error, std::abs(d.cross_track_error_m));
            if (d.laps >= 1 && first < 0) first = simulation.state().time_s;
            if (d.laps >= 2 && outcome.second_lap < 0) outcome.second_lap = simulation.state().time_s-first;
        }
        return outcome;
    };
    auto whole = config;
    whole.envelope_fraction = 1;
    const auto share = drive(fd::SpeedPlanMode::performance_envelope, config);
    const auto all = drive(fd::SpeedPlanMode::performance_envelope, whole);
    const auto baseline = drive(fd::SpeedPlanMode::grip_fractions, config);
    std::cout << "  second lap: grip fractions " << baseline.second_lap << " s (" << baseline.beyond << " samples beyond a tire's peak, tracking error "
              << baseline.worst_error << " m), default share " << share.second_lap << " s (" << share.beyond << ", " << share.worst_error
              << " m), whole envelope " << all.second_lap << " s (" << all.beyond << ", " << all.worst_error << " m)\n";
    require(share.second_lap > 0 && share.second_lap < baseline.second_lap-1, "the default share laps faster than the grip fractions");
    require(share.beyond == 0, "and keeps every tire within its peak");
    require(all.beyond > 0 && all.second_lap < share.second_lap, "the whole envelope is faster still but passes the tires' peak");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"planned_speed_never_exceeds_the_envelope", planned_speed_never_exceeds_the_envelope},
        {"uses_more_of_the_car_than_the_grip_fractions", uses_more_of_the_car_than_the_grip_fractions},
        {"the_estimated_lap_follows_the_plan", the_estimated_lap_follows_the_plan},
        {"lower_grip_moves_braking_earlier", lower_grip_moves_braking_earlier},
        {"the_cars_setup_moves_braking_points", the_cars_setup_moves_braking_points},
        {"refuses_an_envelope_for_another_configuration", refuses_an_envelope_for_another_configuration},
        {"a_circle_is_planned_at_its_lateral_limit", a_circle_is_planned_at_its_lateral_limit},
        {"the_controller_commands_up_to_the_cars_capacity", the_controller_commands_up_to_the_cars_capacity},
        {"the_simulation_plans_with_the_envelope_it_derives", the_simulation_plans_with_the_envelope_it_derives},
        {"the_share_of_the_envelope_moves_the_plan", the_share_of_the_envelope_moves_the_plan},
        {"the_default_share_keeps_the_tires_within_their_peak", the_default_share_keeps_the_tires_within_their_peak},
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cout << "FAIL " << name << ": " << error.what() << '\n'; }
    }
    std::cout << tests.size()-failures << '/' << tests.size() << " envelope speed plan groups passed\n";
    return failures == 0 ? 0 : 1;
}

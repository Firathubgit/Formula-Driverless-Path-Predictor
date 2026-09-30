#include "fd/performance_envelope.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Phase 3.1 (decision 0017): the four-wheel car's G-G-V envelope derived from the plant. Oracles are independent
// of the search: the tires' own lateral capacity at the loads a turn would put on them, the power limit less drag,
// the plant's own tire slip in a tight turn at low speed, and the plan's three properties: symmetry, braking beyond
// drive, and lateral capacity that grows with speed only with downforce.
namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tolerance)
        throw std::runtime_error(message+": actual="+std::to_string(actual)+", expected="+std::to_string(expected));
}
template<class Action> void rejects(Action action, const std::string& message) {
    bool rejected = false;
    try { action(); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, message);
}

const fd::Config config;
const double q = 0.5*1.225;  // standard sea-level air, written out here

double drag_deceleration(const fd::FourWheelCar& car, double speed) { return q*car.drag_area_m2*speed*speed/car.mass_kg; }

// Derived once per car and shared by the groups, because each derivation runs the plant for minutes of simulated time.
std::map<std::string, std::pair<fd::FourWheelCar, fd::PerformanceEnvelope>> derived;
const fd::PerformanceEnvelope& envelope_for(const std::string& name, const fd::FourWheelCar& car) {
    if (!derived.contains(name)) {
        const auto began = std::chrono::steady_clock::now();
        fd::PerformanceEnvelope envelope(car, config);
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
        std::cout << "  derived the " << name << " envelope over " << envelope.rows().size() << " speeds in " << wall << " s (measurement only)\n";
        derived.emplace(name, std::pair{car, std::move(envelope)});
    }
    return derived.at(name).second;
}
const fd::EnvelopeRow& row_at(const fd::PerformanceEnvelope& envelope, double speed) {
    for (const auto& row : envelope.rows())
        if (std::abs(row.speed_mps-speed) < 1e-9) return row;
    throw std::runtime_error("no envelope row at "+std::to_string(speed)+" m/s");
}
fd::FourWheelCar winged() { fd::FourWheelCar car; car.downforce_area_m2 = 3; return car; }
fd::FourWheelCar matched_bias() { fd::FourWheelCar car; car.brake_bias_front = 0.62; return car; }

void covers_the_speed_grid_with_ordered_levels() {
    const auto& envelope = envelope_for("default", fd::FourWheelCar{});
    const auto& rows = envelope.rows();
    require(!rows.empty() && rows.front().speed_mps == 4 && rows.back().speed_mps == config.max_speed_mps+4, "the grid runs from 4 m/s to 4 m/s above the top speed");
    for (std::size_t i = 1; i < rows.size(); ++i) near(rows[i].speed_mps-rows[i-1].speed_mps, 2, 1e-12, "rows are 2 m/s apart");
    for (const auto& row : rows)
        for (const auto* side : {&row.left, &row.right}) {
            require(side->levels.size() == fd::envelope_lateral_fractions.size(), "every side has a level per lateral fraction");
            require(side->lateral_limit_mps2 > 1, "every speed has a lateral limit");
            near(side->levels.front().lateral_mps2, 0, 0.02, "the first level is straight ahead");
            near(side->levels.back().lateral_mps2, side->lateral_limit_mps2, 1e-9, "the last level is the lateral limit");
            for (std::size_t k = 0; k < side->levels.size(); ++k) {
                const auto& level = side->levels[k];
                require(level.forward_mps2 > level.braking_mps2 && level.braking_mps2 < 0, "forward capacity exceeds braking at every level");
                if (k > 0) require(level.lateral_mps2 > side->levels[k-1].lateral_mps2, "levels ascend in lateral acceleration");
                near(level.lateral_mps2, fd::envelope_lateral_fractions[k]*side->lateral_limit_mps2, 0.03*side->lateral_limit_mps2+0.05,
                     "each level sits at its fraction of the lateral limit");
            }
            require(side->levels[4].forward_mps2 < side->levels[0].forward_mps2 && side->levels[4].braking_mps2 > side->levels[0].braking_mps2,
                    "longitudinal capacity falls as lateral use rises");
        }
    std::cout << "  default car:";
    for (const auto& row : rows)
        std::cout << " " << row.speed_mps << " m/s " << row.left.lateral_limit_mps2 << " (" << fd::envelope_limit_name(row.left.lateral_limit) << ")";
    std::cout << "\n";
}

void is_symmetric_in_lateral_acceleration() {
    const auto& envelope = envelope_for("default", fd::FourWheelCar{});
    double largest_lateral = 0, largest_longitudinal = 0;
    for (const auto& row : envelope.rows()) {
        largest_lateral = std::max(largest_lateral, std::abs(row.left.lateral_limit_mps2-row.right.lateral_limit_mps2));
        near(row.left.lateral_limit_mps2, row.right.lateral_limit_mps2, 0.01*row.left.lateral_limit_mps2, "left and right lateral limits agree");
        // At the lateral limit itself longitudinal capacity falls steeply with lateral use, so a hundredth of a percent
        // of difference in the limit shows there; below it the two sides agree closely.
        for (std::size_t k = 0; k < fd::envelope_lateral_fractions.size(); ++k) {
            const auto& l = row.left.levels[k];
            const auto& r = row.right.levels[k];
            const bool at_limit = k+1 == fd::envelope_lateral_fractions.size();
            if (!at_limit) largest_longitudinal = std::max({largest_longitudinal, std::abs(l.forward_mps2-r.forward_mps2), std::abs(l.braking_mps2-r.braking_mps2)});
            near(l.forward_mps2, r.forward_mps2, at_limit ? 0.3 : 0.03*std::abs(l.forward_mps2)+0.05, "left and right forward limits agree");
            near(l.braking_mps2, r.braking_mps2, at_limit ? 0.3 : 0.03*std::abs(l.braking_mps2)+0.05, "left and right braking limits agree");
        }
        near(envelope.forward_limit(row.speed_mps, 3), envelope.forward_limit(row.speed_mps, -3), 0.03*envelope.forward_limit(row.speed_mps, 3)+0.05,
             "queries mirror left and right");
    }
    std::cout << "  largest left-right difference: lateral limit " << largest_lateral << " m/s^2, longitudinal below the limit " << largest_longitudinal << " m/s^2\n";
}

// The lateral limit cannot exceed what the four tires offer at the loads such a turn puts on them, and once the car
// is past tight low-speed turns a tire-limited turn uses most of it.
void lateral_limit_is_bounded_by_the_tires_capacity() {
    for (const auto& [name, car] : std::vector<std::pair<std::string, fd::FourWheelCar>>{{"default", fd::FourWheelCar{}}, {"winged", winged()}}) {
        const auto& envelope = envelope_for(name, car);
        for (const auto& row : envelope.rows()) {
            if (row.left.lateral_limit != fd::EnvelopeLimit::tire) continue;
            const double ay = row.left.lateral_limit_mps2;
            const double downforce = q*car.downforce_area_m2*row.speed_mps*row.speed_mps;
            const auto loads = fd::quasi_static_wheel_loads(car, config, 0, car.mass_kg*ay, downforce);
            double capacity = 0;
            for (std::size_t i = 0; i < 4; ++i)
                capacity += config.grip_mu*fd::tire_peak(i < 2 ? car.front_tire : car.rear_tire, loads[i]).friction_lateral*loads[i]/car.mass_kg;
            require(ay <= capacity*1.001, name+" lateral limit within the tires' capacity at "+std::to_string(row.speed_mps)+" m/s");
            if (row.speed_mps >= 8)
                require(ay >= 0.85*capacity, name+" a tire-limited turn uses most of the tires' capacity at "+std::to_string(row.speed_mps)+" m/s");
            if (row.speed_mps == 16) std::cout << "  " << name << " at 16 m/s: lateral limit " << ay << " m/s^2 of the tires' " << capacity << "\n";
        }
    }
}

// The front wheels steer in parallel, without Ackermann geometry, so in a tight turn the outer front tire meets the
// ground at a larger slip angle than the axle's and reaches its peak first, long before the car runs out of grip.
void low_speed_turns_are_limited_by_parallel_steering() {
    const fd::FourWheelCar car;
    const auto& row = row_at(envelope_for("default", car), 4);
    const double geometric = 16*std::tan(config.max_steering_rad)/config.wheelbase_m;
    const auto loads = fd::quasi_static_wheel_loads(car, config, 0, car.mass_kg*row.left.lateral_limit_mps2, 0);
    double capacity = 0;
    for (std::size_t i = 0; i < 4; ++i)
        capacity += config.grip_mu*fd::tire_peak(i < 2 ? car.front_tire : car.rear_tire, loads[i]).friction_lateral*loads[i]/car.mass_kg;
    // The plant itself, held at 4 m/s on full steering.
    auto s = fd::plant_state_from({0, 0, 0, 4, 0, 0}, car, config);
    double integral = 0, command = 0;
    for (int i = 0; i < 600; ++i) {
        const double error = 4-s.pose.speed_mps;
        integral += error*config.fixed_dt_s;
        command = 4*error+4*integral;
        s = fd::advance(car, s, {command, config.max_steering_rad}, config, config.fixed_dt_s);
    }
    const auto d = fd::demand(car, s, config, command);
    std::cout << "  4 m/s: lateral limit " << row.left.lateral_limit_mps2 << " (" << fd::envelope_limit_name(row.left.lateral_limit) << ") of the tires' "
              << capacity << " and full steering's geometric " << geometric << " m/s^2; on full steering the outer front tire's combined slip is "
              << d.wheel_combined_slip[fd::front_right] << " with the front axle at " << d.front_slip_angle_rad << " of " << d.front_peak_slip_angle_rad << " rad\n";
    require(row.left.lateral_limit == fd::EnvelopeLimit::tire && row.left.lateral_limit_mps2 < 0.5*capacity && row.left.lateral_limit_mps2 <= geometric,
            "at 4 m/s the lateral limit is a tire limit far below the tires' grip and within full steering's geometry");
    require(d.wheel_combined_slip[fd::front_right] > 0.95 && d.front_slip_angle_rad < 0.6*d.front_peak_slip_angle_rad,
            "on full steering the outer front tire reaches its peak while the front axle's slip angle is well below it");
}

void forward_limit_at_speed_is_the_power_limit() {
    const fd::FourWheelCar car;
    const auto& envelope = envelope_for("default", car);
    const double equivalent_mass = car.mass_kg+4*car.wheel_inertia_kgm2/(car.wheel_radius_m*car.wheel_radius_m);
    bool compared = false;
    for (const auto& row : envelope.rows()) {
        const auto& straight = row.left.levels.front();
        if (straight.forward_limit != fd::EnvelopeLimit::actuator) continue;
        const double v = row.speed_mps;
        // Power at the wheels also goes into tire slip, so the chassis gets a little less than power over speed.
        const double expected = (car.max_drive_power_w/v-q*car.drag_area_m2*v*v)/equivalent_mass;
        require(straight.forward_mps2 <= expected*1.001 && straight.forward_mps2 >= 0.88*expected,
                "the straight-line forward limit at "+std::to_string(v)+" m/s is just under power less drag over equivalent mass");
        compared = true;
        std::cout << "  " << v << " m/s: forward limit " << straight.forward_mps2 << " against power less drag " << expected << " m/s^2\n";
    }
    require(compared, "at the top of the grid the forward limit is the power limit");
}

void braking_capacity_exceeds_drive_capacity() {
    const auto& standard = envelope_for("default", fd::FourWheelCar{});
    for (const auto& row : standard.rows()) {
        const auto& straight = row.left.levels.front();
        if (straight.forward_limit == fd::EnvelopeLimit::actuator)
            require(-straight.braking_mps2 > straight.forward_mps2, "where power limits driving, braking capacity exceeds it at "+std::to_string(row.speed_mps)+" m/s");
    }
    // At the static bias the unloaded rear wheels lock first; a bias matched to the braking load brakes harder than
    // the all-wheel drive pulls at every speed.
    const auto& matched = envelope_for("matched brake bias", matched_bias());
    for (const auto& row : matched.rows()) {
        const auto& straight = row.left.levels.front();
        require(-straight.braking_mps2 > straight.forward_mps2+0.5, "with the matched bias braking capacity exceeds drive capacity at "+std::to_string(row.speed_mps)+" m/s");
    }
    std::cout << "  8 m/s straight ahead: default forward " << row_at(standard, 8).left.levels[0].forward_mps2 << " braking " << row_at(standard, 8).left.levels[0].braking_mps2
              << "; matched bias braking " << row_at(matched, 8).left.levels[0].braking_mps2 << " m/s^2\n";
}

void lateral_capacity_grows_with_speed_only_with_downforce() {
    const auto& plain = envelope_for("default", fd::FourWheelCar{});
    const auto& wings = envelope_for("winged", winged());
    const double plain_low = row_at(plain, 12).left.lateral_limit_mps2, plain_high = row_at(plain, 24).left.lateral_limit_mps2;
    const double wings_low = row_at(wings, 12).left.lateral_limit_mps2, wings_high = row_at(wings, 24).left.lateral_limit_mps2;
    std::cout << "  lateral limit 12 to 24 m/s: without downforce " << plain_low << " to " << plain_high << ", with 3 m^2 " << wings_low << " to " << wings_high << "\n";
    require(plain_high <= 1.01*plain_low, "without downforce lateral capacity does not grow with speed");
    require(wings_high >= 1.05*wings_low, "with downforce it grows with speed");
}

void setup_moves_the_envelope() {
    const auto& standard = envelope_for("default", fd::FourWheelCar{});
    const auto& matched = envelope_for("matched brake bias", matched_bias());
    fd::FourWheelCar tall;
    tall.cg_height_m = 0.6;
    const auto& taller = envelope_for("tall", tall);
    require(row_at(matched, 12).left.levels[0].braking_mps2 < row_at(standard, 12).left.levels[0].braking_mps2-0.5,
            "a brake bias matched to the braking load raises straight-line braking capacity");
    require(row_at(taller, 16).left.lateral_limit_mps2 < row_at(standard, 16).left.lateral_limit_mps2,
            "a taller car has less lateral capacity, because load moved across an axle costs grip");
    std::cout << "  16 m/s lateral limit: 0.35 m high " << row_at(standard, 16).left.lateral_limit_mps2 << ", 0.6 m " << row_at(taller, 16).left.lateral_limit_mps2 << "\n";
}

void queries_interpolate_the_rows() {
    const auto& envelope = envelope_for("default", fd::FourWheelCar{});
    const auto& a = row_at(envelope, 12);
    const auto& b = row_at(envelope, 14);
    near(envelope.lateral_limit(12, fd::TurnSide::left), a.left.lateral_limit_mps2, 1e-12, "a grid speed returns its row");
    near(envelope.lateral_limit(13, fd::TurnSide::right), 0.5*(a.right.lateral_limit_mps2+b.right.lateral_limit_mps2), 1e-9, "between rows the limit is linear in speed");
    near(envelope.lateral_limit(1, fd::TurnSide::left), envelope.rows().front().left.lateral_limit_mps2, 1e-12, "below the grid the first row holds");
    near(envelope.forward_limit(12, 0), a.left.levels[0].forward_mps2, 0.03, "straight ahead the forward limit is the first level's");
    const auto& l = a.left.levels;
    near(envelope.braking_limit(12, 0.5*(l[1].lateral_mps2+l[2].lateral_mps2)), 0.5*(l[1].braking_mps2+l[2].braking_mps2), 1e-9,
         "between levels the braking limit is linear in lateral acceleration");
    near(envelope.forward_limit(12, a.left.lateral_limit_mps2+1), 0, 0, "beyond the lateral limit there is no forward capacity");
    near(envelope.braking_limit(12, -(a.right.lateral_limit_mps2+1)), 0, 0, "and no braking capacity");
    // A side at any speed, level by level, for drawing the boundary between grid speeds.
    const auto between = envelope.side_at(13, fd::TurnSide::left);
    require(between.levels.size() == a.left.levels.size(), "an interpolated side keeps every level");
    near(between.lateral_limit_mps2, 0.5*(a.left.lateral_limit_mps2+b.left.lateral_limit_mps2), 1e-9, "its lateral limit is interpolated");
    for (std::size_t k = 0; k < between.levels.size(); ++k) {
        near(between.levels[k].lateral_mps2, 0.5*(a.left.levels[k].lateral_mps2+b.left.levels[k].lateral_mps2), 1e-9, "each level's lateral acceleration is interpolated");
        near(between.levels[k].braking_mps2, 0.5*(a.left.levels[k].braking_mps2+b.left.levels[k].braking_mps2), 1e-9, "each level's braking limit is interpolated");
    }
    near(envelope.side_at(12, fd::TurnSide::right).levels[3].forward_mps2, a.right.levels[3].forward_mps2, 1e-12, "a grid speed returns its row's side");
}

void fingerprint_identifies_the_car_and_configuration() {
    const fd::FourWheelCar car;
    const auto& envelope = envelope_for("default", car);
    require(envelope.fingerprint().size() == 16 && envelope.fingerprint() == fd::performance_envelope_fingerprint(car, config),
            "the fingerprint is the one computed without deriving");
    require(envelope.generated_for(car, config), "the envelope knows its car");
    auto heavier = car;
    heavier.mass_kg = 900;
    auto slippery = config;
    slippery.grip_mu = 0.8;
    require(fd::performance_envelope_fingerprint(heavier, config) != envelope.fingerprint() && !envelope.generated_for(heavier, config),
            "a different car has a different fingerprint");
    require(fd::performance_envelope_fingerprint(car, slippery) != envelope.fingerprint() && !envelope.generated_for(car, slippery),
            "a different road grip has a different fingerprint");
}

std::vector<std::vector<double>> numbers(const std::filesystem::path& file, std::vector<std::string>& comments) {
    std::ifstream in(file);
    require(static_cast<bool>(in), "exported file exists: "+file.string());
    std::vector<std::vector<double>> rows;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (line[0] == '#') { comments.push_back(line); continue; }
        std::vector<double> values;
        std::stringstream fields(line);
        std::string field;
        while (std::getline(fields, field, ',')) {
            try { values.push_back(std::stod(field)); } catch (const std::exception&) { values.push_back(std::nan("")); }
        }
        rows.push_back(values);
    }
    return rows;
}
void exports_tums_format_with_the_fingerprint() {
    const fd::FourWheelCar car;
    const auto& envelope = envelope_for("default", car);
    std::random_device device;
    const auto directory = std::filesystem::temp_directory_path()/("fd-envelope-tests-"+std::to_string(device()));
    fd::write_performance_envelope(envelope, car, config, directory);
    std::vector<std::string> ggv_comments, machine_comments, table_comments;
    const auto ggv = numbers(directory/"ggv.csv", ggv_comments);
    const auto machines = numbers(directory/"ax_max_machines.csv", machine_comments);
    const auto table = numbers(directory/"envelope.csv", table_comments);
    const auto names = [&](const std::vector<std::string>& comments) {
        return std::any_of(comments.begin(), comments.end(), [&](const std::string& c) { return c.find(envelope.fingerprint()) != std::string::npos; });
    };
    require(names(ggv_comments) && names(machine_comments) && names(table_comments), "every file names the fingerprint");
    require(ggv_comments.back() == "# v_mps,ax_max_mps2,ay_max_mps2" && machine_comments.back() == "# v_mps,ax_max_machines_mps2",
            "TUM's files end their comments with TUM's column line");
    require(ggv.size() == envelope.rows().size() && machines.size() == envelope.rows().size(), "one TUM row per speed");
    for (std::size_t i = 0; i < ggv.size(); ++i) {
        const auto& row = envelope.rows()[i];
        const double drag = drag_deceleration(car, row.speed_mps);
        near(ggv[i][0], row.speed_mps, 1e-9, "ggv speed");
        near(ggv[i][1], -row.left.levels[0].braking_mps2-drag, 1e-6, "ggv ax_max is the straight-line braking limit less drag");
        near(ggv[i][2], std::min(row.left.lateral_limit_mps2, row.right.lateral_limit_mps2), 1e-6, "ggv ay_max is the smaller side's lateral limit");
        near(machines[i][1], row.left.levels[0].forward_mps2+drag, 1e-6, "ax_max_machines is the straight-line forward limit plus drag");
    }
    require(table.size() == envelope.rows().size()*2*fd::envelope_lateral_fractions.size(), "envelope.csv has a line per speed, side and level");
    std::filesystem::remove_all(directory);
}

void refuses_other_models() {
    rejects([] { fd::PerformanceEnvelope(fd::KinematicBicycle{}, config); }, "the kinematic bicycle's limits are its stated grip fractions");
    rejects([] { fd::PerformanceEnvelope(fd::DynamicSingleTrack{}, config); }, "the single-track car's allocation clamps longitudinal force");
    rejects([] { fd::performance_envelope_fingerprint(fd::DynamicSingleTrack{}, config); }, "no fingerprint for a model without an envelope");
    auto broken = config;
    broken.grip_mu = 5;
    rejects([&] { fd::PerformanceEnvelope(fd::FourWheelCar{}, broken); }, "an invalid configuration is refused");
    const auto& envelope = envelope_for("default", fd::FourWheelCar{});
    fd::FourWheelCar other;
    other.mass_kg = 1000;
    rejects([&] { fd::write_performance_envelope(envelope, other, config, std::filesystem::temp_directory_path()/"fd-envelope-refused"); },
            "an envelope is not written for a car it was not derived for");
    require(std::string(fd::envelope_limit_name(fd::EnvelopeLimit::settling)) == "settling", "limits have names");
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"covers_the_speed_grid_with_ordered_levels", covers_the_speed_grid_with_ordered_levels},
        {"is_symmetric_in_lateral_acceleration", is_symmetric_in_lateral_acceleration},
        {"lateral_limit_is_bounded_by_the_tires_capacity", lateral_limit_is_bounded_by_the_tires_capacity},
        {"low_speed_turns_are_limited_by_parallel_steering", low_speed_turns_are_limited_by_parallel_steering},
        {"forward_limit_at_speed_is_the_power_limit", forward_limit_at_speed_is_the_power_limit},
        {"braking_capacity_exceeds_drive_capacity", braking_capacity_exceeds_drive_capacity},
        {"lateral_capacity_grows_with_speed_only_with_downforce", lateral_capacity_grows_with_speed_only_with_downforce},
        {"setup_moves_the_envelope", setup_moves_the_envelope},
        {"queries_interpolate_the_rows", queries_interpolate_the_rows},
        {"fingerprint_identifies_the_car_and_configuration", fingerprint_identifies_the_car_and_configuration},
        {"exports_tums_format_with_the_fingerprint", exports_tums_format_with_the_fingerprint},
        {"refuses_other_models", refuses_other_models},
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " performance envelope groups passed\n";
    return failures ? 1 : 0;
}

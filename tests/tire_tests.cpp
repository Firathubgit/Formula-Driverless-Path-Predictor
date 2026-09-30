#include "fd/tire.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tolerance)
        throw std::runtime_error(message+": actual="+std::to_string(actual)+
                                 ", expected="+std::to_string(expected));
}
template<class Action> void rejects(Action action, const std::string& message) {
    bool rejected = false;
    try { action(); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, message);
}

constexpr double degree = std::numbers::pi/180;

// Slip ratio in [0, limit] at which pure longitudinal force peaks, by dense scan.
double longitudinal_peak_slip(const fd::TireParameters& tire, double load, double limit) {
    double best_slip = 0, best_force = 0;
    for (int i = 1; i <= 200000; ++i) {
        const double slip = limit*i/200000;
        const double force = fd::tire_force(tire, slip, 0, load).longitudinal_n;
        if (force > best_force) { best_force = force; best_slip = slip; }
    }
    return best_slip;
}
double lateral_peak_angle(const fd::TireParameters& tire, double load, double limit) {
    double best_angle = 0, best_force = 0;
    for (int i = 1; i <= 200000; ++i) {
        const double angle = limit*i/200000;
        const double force = fd::tire_force(tire, 0, angle, load).lateral_n;
        if (force > best_force) { best_force = force; best_angle = angle; }
    }
    return best_angle;
}

void zero_slip_gives_zero_force() {
    const fd::TireParameters tire;
    fd::validate_tire(tire);
    for (const double load : {500.0, 2000.0, 4000.0, 9000.0}) {
        const auto force = fd::tire_force(tire, 0, 0, load);
        require(force.longitudinal_n == 0 && force.lateral_n == 0, "free rolling without slip gives no force");
    }
}

void forces_are_odd_and_signed_by_slip() {
    const fd::TireParameters tire;
    std::mt19937 random(11);
    std::uniform_real_distribution<double> slip(-0.4, 0.4), angle(-0.3, 0.3), load(300, 9000);
    for (int i = 0; i < 5000; ++i) {
        const double k = slip(random), a = angle(random), z = load(random);
        const auto forward = fd::tire_force(tire, k, a, z);
        const auto mirrored = fd::tire_force(tire, -k, -a, z);
        near(mirrored.longitudinal_n, -forward.longitudinal_n, 1e-9*z, "longitudinal force is odd in slip");
        near(mirrored.lateral_n, -forward.lateral_n, 1e-9*z, "lateral force is odd in slip angle");
        require(forward.longitudinal_n*k >= 0 && forward.lateral_n*a >= 0,
                "driving slip pushes forward and positive slip angle pushes left");
    }
}

void peak_lies_at_the_named_slip() {
    const fd::TireParameters tire;
    for (const double load : {2000.0, 3000.0, 4000.0, 6000.0}) {
        const double t = (load-2000)/4000;
        const double slip = std::lerp(tire.peak_slip_ratio_low, tire.peak_slip_ratio_high, t);
        const double angle = std::lerp(tire.peak_slip_angle_low_rad, tire.peak_slip_angle_high_rad, t);
        const double mu_x = std::lerp(tire.peak_friction_longitudinal_low, tire.peak_friction_longitudinal_high, t);
        const double mu_y = std::lerp(tire.peak_friction_lateral_low, tire.peak_friction_lateral_high, t);
        near(longitudinal_peak_slip(tire, load, 0.5), slip, 0.01*slip,
             "longitudinal peak within 1% of the named slip ratio at "+std::to_string(load)+" N");
        near(lateral_peak_angle(tire, load, 0.5), angle, 0.01*angle,
             "lateral peak within 1% of the named slip angle at "+std::to_string(load)+" N");
        near(fd::tire_force(tire, slip, 0, load).longitudinal_n, mu_x*load, 1e-9*load,
             "force at the named slip ratio equals peak friction times load");
        near(fd::tire_force(tire, 0, angle, load).lateral_n, mu_y*load, 1e-9*load,
             "force at the named slip angle equals peak friction times load");
    }
}

void force_falls_past_the_peak_toward_the_sliding_fraction() {
    const fd::TireParameters tire;
    const double load = 4000;  // halfway between the references: peak slip ratio 0.105
    const double mu_x = fd::tire_force(tire, 0.105, 0, load).longitudinal_n/load;
    double previous = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= 1000; ++i) {
        const double slip = 0.105+(1-0.105)*i/1000;
        const double force = fd::tire_force(tire, slip, 0, load).longitudinal_n;
        require(force <= previous*(1+1e-12), "force never rises again once past the peak");
        previous = force;
    }
    // A locked wheel (slip ratio -1) keeps between the asymptotic sliding fraction and peak.
    const double locked = -fd::tire_force(tire, -1, 0, load).longitudinal_n/(mu_x*load);
    require(locked > tire.sliding_fraction_longitudinal && locked < 0.9,
            "a locked wheel keeps a realistic share of peak grip, not a collapse: "+std::to_string(locked));
    const double far_slip = -fd::tire_force(tire, -1e7, 0, load).longitudinal_n/(mu_x*load);
    near(far_slip, tire.sliding_fraction_longitudinal, 1e-3, "grip at very large slip tends to the sliding fraction");
    const double far_angle = fd::tire_force(tire, 0, 1e7, load).lateral_n/
                             fd::tire_force(tire, 0, 8.5*degree, load).lateral_n;
    near(far_angle, tire.sliding_fraction_lateral, 1e-3, "lateral grip at very large slip angle tends to its sliding fraction");
}

void fastest_lap_shape_is_representable_and_harsher() {
    // fastest-lap's F1 file uses shape Q = 1.9, whose large-slip limit sin(Q*pi/2) is 0.156.
    fd::TireParameters harsh;
    harsh.sliding_fraction_longitudinal = std::sin(1.9*std::numbers::pi/2);
    fd::validate_tire(harsh);
    near(fd::tire_shape_for_sliding_fraction(harsh.sliding_fraction_longitudinal), 1.9, 1e-12,
         "sliding fraction and shape factor are one-to-one");
    const fd::TireParameters realistic;
    const double load = 4000;
    const double harsh_locked = -fd::tire_force(harsh, -1, 0, load).longitudinal_n;
    const double realistic_locked = -fd::tire_force(realistic, -1, 0, load).longitudinal_n;
    require(harsh_locked < 0.5*realistic_locked,
            "a locked wheel keeps less than half the grip with the fastest-lap shape");
    near(fd::tire_force(harsh, 0.105, 0, load).longitudinal_n, fd::tire_force(realistic, 0.105, 0, load).longitudinal_n,
         1e-9*load, "the shape changes sliding, not peak grip at the named slip");
}

// Slip stiffness at free rolling sets how stiff wheel-speed dynamics are: a wheel relaxes
// with time constant I*v/(C*R^2). For this curve C = mu*Fz*Q*S/kappa_peak, which for the
// default sliding fraction is 2.705 times the mu*Fz/kappa_peak a straight-line estimate gives.
void small_slip_stiffness_is_steeper_than_the_secant() {
    const fd::TireParameters tire;
    const double load = 4000, mu = 1.575, peak = 0.105;
    const double h = 1e-7;
    const double slope = (fd::tire_force(tire, h, 0, load).longitudinal_n-
                          fd::tire_force(tire, -h, 0, load).longitudinal_n)/(2*h);
    const double q = fd::tire_shape_for_sliding_fraction(tire.sliding_fraction_longitudinal);
    const double s = std::tan(std::numbers::pi/(2*q));
    near(slope, mu*load*q*s/peak, 1e-6*slope, "small-slip stiffness equals mu*Fz*Q*S/kappa_peak");
    near(slope/(mu*load/peak), 2.705, 0.001, "the initial slope is about 2.7 times the peak secant");
}

void heavier_load_gives_more_force_but_less_friction() {
    const fd::TireParameters tire;
    double previous_force = 0, previous_mu = std::numeric_limits<double>::infinity();
    for (double load = 1000; load <= 6000; load += 250) {
        double force = 0;
        for (int i = 1; i <= 400; ++i)
            force = std::max(force, fd::tire_force(tire, 0, 0.3*i/400, load).lateral_n);
        const double mu = force/load;
        require(force > previous_force, "peak lateral force grows with load");
        require(mu < previous_mu, "peak lateral friction falls with load");
        previous_force = force;
        previous_mu = mu;
    }
}

void extrapolation_beyond_the_reference_loads_is_bounded() {
    const fd::TireParameters tire;
    // Friction continues its reference slope above the high load, down to the floor.
    const double at_10k = fd::tire_force(tire, tire.peak_slip_ratio_high, 0, 10000).longitudinal_n/10000;
    near(at_10k, std::max(tire.minimum_friction, 1.40-(10000-6000)*0.35/4000), 1e-9, "friction extrapolates linearly above the high reference");
    const double at_100k = fd::tire_force(tire, tire.peak_slip_ratio_high, 0, 1e5).longitudinal_n/1e5;
    near(at_100k, tire.minimum_friction, 1e-9, "friction never falls below the floor");
    // Peak slip is held at its reference value outside the reference loads.
    near(longitudinal_peak_slip(tire, 20000, 0.5), tire.peak_slip_ratio_high, 0.01*tire.peak_slip_ratio_high,
         "peak slip is held above the high reference load");
    near(longitudinal_peak_slip(tire, 500, 0.5), tire.peak_slip_ratio_low, 0.01*tire.peak_slip_ratio_low,
         "peak slip is held below the low reference load");
    const auto lifted = fd::tire_force(tire, 0.1, 0.1, 0);
    require(lifted.longitudinal_n == 0 && lifted.lateral_n == 0, "a lifted wheel transmits no force");
    const auto negative = fd::tire_force(tire, 0.1, 0.1, -50);
    require(negative.longitudinal_n == 0 && negative.lateral_n == 0, "negative load is treated as lifted");
}

void combined_slip_shares_one_friction_ellipse() {
    const fd::TireParameters tire;
    const double load = 4000;
    const double angle = 8.5*degree;
    const double pure_lateral = fd::tire_force(tire, 0, angle, load).lateral_n;
    const double braking_lateral = fd::tire_force(tire, -0.105, angle, load).lateral_n;
    require(braking_lateral < 0.8*pure_lateral, "braking at the peak slip takes away lateral grip");
    std::mt19937 random(5);
    std::uniform_real_distribution<double> slip(-1, 1), slip_angle(-0.6, 0.6), vertical(100, 12000);
    for (int i = 0; i < 20000; ++i) {
        const double z = vertical(random);
        const auto f = fd::tire_force(tire, slip(random), slip_angle(random), z);
        // Recover this load's peak frictions from pure slip at the named slip values.
        const double t = std::clamp((z-2000)/4000, 0.0, 1.0);
        const double k = std::lerp(tire.peak_slip_ratio_low, tire.peak_slip_ratio_high, t);
        const double a = std::lerp(tire.peak_slip_angle_low_rad, tire.peak_slip_angle_high_rad, t);
        const double fx_max = fd::tire_force(tire, k, 0, z).longitudinal_n;
        const double fy_max = fd::tire_force(tire, 0, a, z).lateral_n;
        const double used = std::pow(f.longitudinal_n/fx_max, 2)+std::pow(f.lateral_n/fy_max, 2);
        require(used <= 1+1e-9, "combined force stays inside the friction ellipse");
    }
}

void peak_query_agrees_with_the_force_law() {
    const fd::TireParameters tire;
    for (const double load : {500.0, 2000.0, 3500.0, 6000.0, 20000.0}) {
        const auto peak = fd::tire_peak(tire, load);
        near(fd::tire_force(tire, peak.slip_ratio, 0, load).longitudinal_n, peak.friction_longitudinal*load, 1e-9*load,
             "force at the reported peak slip ratio is the reported peak friction times load");
        near(fd::tire_force(tire, 0, peak.slip_angle_rad, load).lateral_n, peak.friction_lateral*load, 1e-9*load,
             "force at the reported peak slip angle is the reported peak friction times load");
        near(longitudinal_peak_slip(tire, load, 0.5), peak.slip_ratio, 0.01*peak.slip_ratio, "reported peak slip ratio is the numerical peak");
    }
    rejects([&] { fd::tire_peak(tire, 0); }, "peak at zero load is rejected");
    rejects([&] { fd::tire_peak(tire, std::numeric_limits<double>::infinity()); }, "peak at infinite load is rejected");
}

void prepared_tire_equals_the_force_law() {
    fd::TireParameters tire;
    tire.sliding_fraction_lateral = 0.6;  // distinct shapes, so a swapped curve would show
    std::mt19937 random(17);
    std::uniform_real_distribution<double> slip(-1.2, 1.2), angle(-0.8, 0.8), load(50, 15000);
    for (int i = 0; i < 2000; ++i) {
        const double z = load(random);
        const fd::TireAtLoad prepared(tire, z);
        const auto peak = fd::tire_peak(tire, z);
        require(prepared.peak().friction_longitudinal == peak.friction_longitudinal &&
                prepared.peak().friction_lateral == peak.friction_lateral &&
                prepared.peak().slip_ratio == peak.slip_ratio && prepared.peak().slip_angle_rad == peak.slip_angle_rad,
                "a prepared tire reports exactly the peak of the force law");
        require(prepared.load_n() == z, "a prepared tire keeps its load");
        for (int k = 0; k < 20; ++k) {
            const double s = slip(random), a = angle(random);
            const auto expected = fd::tire_force(tire, s, a, z);
            const auto actual = prepared.force(s, a);
            require(actual.longitudinal_n == expected.longitudinal_n && actual.lateral_n == expected.lateral_n,
                    "a prepared tire gives exactly the force law's forces");
        }
    }
    rejects([&] { fd::TireAtLoad prepared(tire, 0); }, "a prepared tire needs a positive load");
    rejects([&] { fd::TireAtLoad prepared(tire, std::numeric_limits<double>::quiet_NaN()); }, "a prepared tire needs a finite load");
    auto broken = tire;
    broken.peak_slip_ratio_low = -1;
    rejects([&] { fd::TireAtLoad prepared(broken, 2000); }, "a prepared tire validates its parameters once");
    const fd::TireAtLoad prepared(tire, 2000);
    rejects([&] { prepared.force(std::numeric_limits<double>::infinity(), 0); }, "a prepared tire rejects non-finite slip");
}

void invalid_inputs_are_rejected() {
    const fd::TireParameters tire;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    rejects([&] { fd::tire_force(tire, nan, 0, 1000); }, "non-finite slip ratio is rejected");
    rejects([&] { fd::tire_force(tire, 0, inf, 1000); }, "non-finite slip angle is rejected");
    rejects([&] { fd::tire_force(tire, 0, 0, nan); }, "non-finite load is rejected");
    const auto broken = [&](const std::function<void(fd::TireParameters&)>& change, const std::string& what) {
        auto copy = tire;
        change(copy);
        rejects([&] { fd::validate_tire(copy); }, what+" is rejected");
        rejects([&] { fd::tire_force(copy, 0.05, 0.05, 1000); }, what+" is rejected at force evaluation");
    };
    broken([](auto& t) { t.reference_load_low_n = 0; }, "non-positive low reference load");
    broken([](auto& t) { t.reference_load_high_n = t.reference_load_low_n; }, "unordered reference loads");
    broken([&](auto& t) { t.peak_friction_lateral_high = nan; }, "non-finite peak friction");
    broken([](auto& t) { t.peak_friction_longitudinal_low = 0; }, "non-positive peak friction");
    broken([](auto& t) { t.peak_slip_ratio_high = 0; }, "non-positive peak slip ratio");
    broken([](auto& t) { t.peak_slip_angle_low_rad = 1.0; }, "peak slip angle beyond 0.5 rad");
    broken([](auto& t) { t.sliding_fraction_lateral = 1.0; }, "sliding fraction of one, which has no peak");
    broken([](auto& t) { t.sliding_fraction_longitudinal = 0.0; }, "sliding fraction of zero");
    broken([](auto& t) { t.minimum_friction = 1.5; }, "friction floor above a reference peak friction");
    rejects([] { fd::tire_shape_for_sliding_fraction(1.2); }, "shape for an impossible sliding fraction is rejected");
}
} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"zero_slip_gives_zero_force", zero_slip_gives_zero_force},
        {"forces_are_odd_and_signed_by_slip", forces_are_odd_and_signed_by_slip},
        {"peak_lies_at_the_named_slip", peak_lies_at_the_named_slip},
        {"force_falls_past_the_peak_toward_the_sliding_fraction", force_falls_past_the_peak_toward_the_sliding_fraction},
        {"fastest_lap_shape_is_representable_and_harsher", fastest_lap_shape_is_representable_and_harsher},
        {"small_slip_stiffness_is_steeper_than_the_secant", small_slip_stiffness_is_steeper_than_the_secant},
        {"heavier_load_gives_more_force_but_less_friction", heavier_load_gives_more_force_but_less_friction},
        {"extrapolation_beyond_the_reference_loads_is_bounded", extrapolation_beyond_the_reference_loads_is_bounded},
        {"combined_slip_shares_one_friction_ellipse", combined_slip_shares_one_friction_ellipse},
        {"peak_query_agrees_with_the_force_law", peak_query_agrees_with_the_force_law},
        {"prepared_tire_equals_the_force_law", prepared_tire_equals_the_force_law},
        {"invalid_inputs_are_rejected", invalid_inputs_are_rejected}
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size()
              << " tire groups passed\n";
    return failures ? 1 : 0;
}

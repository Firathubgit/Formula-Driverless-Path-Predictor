#include "fd/tire.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace fd {
namespace {

constexpr double pi = std::numbers::pi;

// Normalised force curve sin(Q*atan(S*rho)) with S = tan(pi/(2Q)), so it reaches exactly one
// at rho = 1 and tends to sin(Q*pi/2) as rho grows. fastest-lap uses S = pi/(2*atan(Q)),
// which moves its peak to rho = 0.751 for Q = 1.9; this curve keeps the peak where named.
double stiffness_for_shape(double shape) { return std::tan(pi/(2*shape)); }
// Curve value divided by rho. The ratio tends to Q*S at zero slip, which keeps the shared
// normalisation finite and exact for pure slip in one direction.
double curve_over_rho(double shape, double stiffness, double rho) {
    if (rho < 1e-8) return shape*stiffness;
    return std::sin(shape*std::atan(stiffness*rho))/rho;
}

// Linear in load between the references, then floored.
double friction_at(double low, double high, const TireParameters& tire, double load) {
    const double slope = (high-low)/(tire.reference_load_high_n-tire.reference_load_low_n);
    return std::max(tire.minimum_friction, low+(load-tire.reference_load_low_n)*slope);
}

// Linear in load between the references and held at the reference values outside them, so
// it can never reach zero or change sign however large the load.
double peak_slip_at(double low, double high, const TireParameters& tire, double load) {
    const double t = std::clamp((load-tire.reference_load_low_n)/(tire.reference_load_high_n-tire.reference_load_low_n), 0.0, 1.0);
    return low+t*(high-low);
}

} // namespace

double tire_shape_for_sliding_fraction(double sliding_fraction) {
    if (!std::isfinite(sliding_fraction) || sliding_fraction <= 0 || sliding_fraction >= 1)
        throw std::invalid_argument("Tire sliding fraction must lie strictly between zero and one");
    // sin(Q*pi/2) = fraction with Q in (1, 2), past the peak of the sine.
    return 2-2*std::asin(sliding_fraction)/pi;
}

void validate_tire(const TireParameters& t) {
    const auto range = [](double value, double low, double high, const char* name) {
        if (!std::isfinite(value) || value < low || value > high)
            throw std::invalid_argument(std::string("Tire ")+name+" outside supported range");
    };
    const auto positive = [](double value, double high, const char* name) {
        if (!std::isfinite(value) || value <= 0 || value > high)
            throw std::invalid_argument(std::string("Tire ")+name+" must be positive and at most "+std::to_string(high));
    };
    positive(t.reference_load_low_n, 1e6, "reference_load_low_n");
    positive(t.reference_load_high_n, 1e6, "reference_load_high_n");
    if (t.reference_load_high_n <= t.reference_load_low_n)
        throw std::invalid_argument("Tire reference loads must be ordered low below high");
    positive(t.peak_friction_longitudinal_low, 3, "peak_friction_longitudinal_low");
    positive(t.peak_friction_longitudinal_high, 3, "peak_friction_longitudinal_high");
    positive(t.peak_friction_lateral_low, 3, "peak_friction_lateral_low");
    positive(t.peak_friction_lateral_high, 3, "peak_friction_lateral_high");
    positive(t.peak_slip_ratio_low, 1, "peak_slip_ratio_low");
    positive(t.peak_slip_ratio_high, 1, "peak_slip_ratio_high");
    positive(t.peak_slip_angle_low_rad, 0.5, "peak_slip_angle_low_rad");
    positive(t.peak_slip_angle_high_rad, 0.5, "peak_slip_angle_high_rad");
    range(t.sliding_fraction_longitudinal, 0.05, 0.95, "sliding_fraction_longitudinal");
    range(t.sliding_fraction_lateral, 0.05, 0.95, "sliding_fraction_lateral");
    const double lowest_peak = std::min({t.peak_friction_longitudinal_low, t.peak_friction_longitudinal_high,
                                         t.peak_friction_lateral_low, t.peak_friction_lateral_high});
    positive(t.minimum_friction, lowest_peak, "minimum_friction (floor above a reference peak friction)");
}

TireAtLoad::TireAtLoad(const TireParameters& tire, double load) : TireAtLoad(PreparedTire(tire).at(load)) {}

PreparedTire::PreparedTire(const TireParameters& tire) : tire_(tire) {
    validate_tire(tire);
    shape_longitudinal_ = tire_shape_for_sliding_fraction(tire.sliding_fraction_longitudinal);
    stiffness_longitudinal_ = stiffness_for_shape(shape_longitudinal_);
    shape_lateral_ = tire_shape_for_sliding_fraction(tire.sliding_fraction_lateral);
    stiffness_lateral_ = stiffness_for_shape(shape_lateral_);
}

TireAtLoad PreparedTire::at(double load) const {
    if (!std::isfinite(load) || load <= 0) throw std::invalid_argument("Tire evaluation requires a finite positive load");
    const TireParameters& tire = tire_;
    TireAtLoad t;
    t.load_n_ = load;
    t.peak_ = {friction_at(tire.peak_friction_longitudinal_low, tire.peak_friction_longitudinal_high, tire, load),
               friction_at(tire.peak_friction_lateral_low, tire.peak_friction_lateral_high, tire, load),
               peak_slip_at(tire.peak_slip_ratio_low, tire.peak_slip_ratio_high, tire, load),
               peak_slip_at(tire.peak_slip_angle_low_rad, tire.peak_slip_angle_high_rad, tire, load)};
    t.shape_longitudinal_ = shape_longitudinal_;
    t.stiffness_longitudinal_ = stiffness_longitudinal_;
    t.shape_lateral_ = shape_lateral_;
    t.stiffness_lateral_ = stiffness_lateral_;
    return t;
}

TireForce TireAtLoad::force(double slip_ratio, double slip_angle_rad) const {
    if (!std::isfinite(slip_ratio) || !std::isfinite(slip_angle_rad))
        throw std::invalid_argument("Tire force requires finite slip ratio and slip angle");
    const double kappa_n = slip_ratio/peak_.slip_ratio;
    const double alpha_n = slip_angle_rad/peak_.slip_angle_rad;
    const double rho = std::hypot(kappa_n, alpha_n);
    const double longitudinal = curve_over_rho(shape_longitudinal_, stiffness_longitudinal_, rho);
    // The two curves are often the same shape; the value is then identical, so evaluate it once.
    const double lateral = shape_lateral_ == shape_longitudinal_ && stiffness_lateral_ == stiffness_longitudinal_
                               ? longitudinal : curve_over_rho(shape_lateral_, stiffness_lateral_, rho);
    return {peak_.friction_longitudinal*load_n_*longitudinal*kappa_n, peak_.friction_lateral*load_n_*lateral*alpha_n};
}

TirePeak tire_peak(const TireParameters& tire, double load) {
    return TireAtLoad(tire, load).peak();
}

TireForce tire_force(const TireParameters& tire, double slip_ratio, double slip_angle_rad, double load) {
    validate_tire(tire);
    if (!std::isfinite(slip_ratio) || !std::isfinite(slip_angle_rad) || !std::isfinite(load))
        throw std::invalid_argument("Tire force requires finite slip ratio, slip angle and load");
    if (load <= 0) return {};
    return TireAtLoad(tire, load).force(slip_ratio, slip_angle_rad);
}

} // namespace fd

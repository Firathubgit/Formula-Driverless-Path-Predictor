#pragma once

#include <numbers>

namespace fd {

// Steady-state tire force from slip, with load sensitivity and combined slip. The structure
// follows fastest-lap's Pacejka_simple_model (MIT); the peak is placed at the named slip by
// construction and the curve shape is set by how much grip a sliding tire keeps. See
// docs/decisions/0006-tire-force-model.md. Not yet connected to the simulated plant.
//
// Wheel frame: x forward, y left. Slip ratio kappa = (omega*R - v_x)/v_x, so driving slip is
// positive and a locked wheel rolling forward has kappa = -1. Slip angle
// alpha = -atan(v_y/v_x) at the contact patch, so a positive angle produces a leftward force.
struct TireParameters {
    // Peak friction and peak slip vary linearly with vertical load between these two loads.
    double reference_load_low_n{2000}, reference_load_high_n{6000};
    double peak_friction_longitudinal_low{1.75}, peak_friction_longitudinal_high{1.40};
    double peak_friction_lateral_low{1.80}, peak_friction_lateral_high{1.45};
    double peak_slip_ratio_low{0.11}, peak_slip_ratio_high{0.10};
    double peak_slip_angle_low_rad{9*std::numbers::pi/180}, peak_slip_angle_high_rad{8*std::numbers::pi/180};
    // Share of peak grip kept at very large slip, in [0.05, 0.95]. Dry asphalt keeps
    // roughly 0.7 to 0.85; fastest-lap's F1 file implies 0.156.
    double sliding_fraction_longitudinal{0.75}, sliding_fraction_lateral{0.75};
    // Friction extrapolated beyond the reference loads never falls below this.
    double minimum_friction{1.0};
};
// Default values: loads, frictions and peak slips from fastest-lap's
// database/vehicles/f1/limebeer-2014-f1.xml, after Limebeer et al. 2014; sliding fractions
// chosen here. Illustrative F1-scale values, not a measured tire.

struct TireForce { double longitudinal_n{}, lateral_n{}; };
// Peak friction coefficients and the slip values where they occur, at a vertical load, exactly
// as tire_force uses them: friction extrapolated to its floor, peak slip held outside the
// reference loads. Rejects invalid parameters and non-finite or non-positive loads.
struct TirePeak { double friction_longitudinal{}, friction_lateral{}, slip_ratio{}, slip_angle_rad{}; };
TirePeak tire_peak(const TireParameters& tire, double vertical_load_n);

void validate_tire(const TireParameters& tire);
// Magic Formula shape factor Q in (1, 2) whose curve tends to this share of peak grip.
double tire_shape_for_sliding_fraction(double sliding_fraction);
// Rejects invalid parameters and non-finite inputs. A load at or below zero is a lifted
// wheel and transmits no force. Longitudinal and lateral forces share one normalised slip,
// the similarity approximation of combined slip, so they stay inside one friction ellipse.
TireForce tire_force(const TireParameters& tire, double slip_ratio, double slip_angle_rad, double vertical_load_n);

// A tire validated once and evaluated at one positive vertical load, for repeated force queries at
// that load, as a plant makes on every integration substep. tire_force is implemented with it, so
// the forces and the peak are exactly the force law's.
class TireAtLoad {
public:
    TireAtLoad(const TireParameters& tire, double vertical_load_n);
    // Rejects non-finite slip.
    TireForce force(double slip_ratio, double slip_angle_rad) const;
    const TirePeak& peak() const noexcept { return peak_; }
    double load_n() const noexcept { return load_n_; }
private:
    friend class PreparedTire;
    TireAtLoad() = default;
    double load_n_{};
    TirePeak peak_;
    double shape_longitudinal_{}, stiffness_longitudinal_{}, shape_lateral_{}, stiffness_lateral_{};
};

// A tire validated once and evaluated at whatever load it carries, as a plant with load transfer asks on
// every substep. at() gives exactly what TireAtLoad(tire, load) gives, without validating the tire again.
class PreparedTire {
public:
    explicit PreparedTire(const TireParameters& tire);
    // Rejects a non-finite or non-positive load.
    TireAtLoad at(double vertical_load_n) const;
    const TireParameters& parameters() const noexcept { return tire_; }
private:
    TireParameters tire_;
    double shape_longitudinal_{}, stiffness_longitudinal_{}, shape_lateral_{}, stiffness_lateral_{};
};

} // namespace fd

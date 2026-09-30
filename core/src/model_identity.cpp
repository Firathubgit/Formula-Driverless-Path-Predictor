#include "model_identity.hpp"

#include <charconv>
#include <cmath>
#include <cstdint>

namespace fd::detail {

static void append(std::string& text, const char* name, double value) {
    char buffer[32];
    const auto end = std::to_chars(buffer, buffer+sizeof buffer, value).ptr;
    text += name;
    text += '=';
    text.append(buffer, end);
    text += ';';
}

static void append_tire(std::string& text, const char* prefix, const TireParameters& t) {
    const std::string p(prefix);
    append(text, (p+"peak_friction_longitudinal_low").c_str(), t.peak_friction_longitudinal_low);
    append(text, (p+"peak_friction_longitudinal_high").c_str(), t.peak_friction_longitudinal_high);
    append(text, (p+"peak_friction_lateral_low").c_str(), t.peak_friction_lateral_low);
    append(text, (p+"peak_friction_lateral_high").c_str(), t.peak_friction_lateral_high);
    append(text, (p+"peak_slip_ratio_low").c_str(), t.peak_slip_ratio_low);
    append(text, (p+"peak_slip_ratio_high").c_str(), t.peak_slip_ratio_high);
    append(text, (p+"peak_slip_angle_low_rad").c_str(), t.peak_slip_angle_low_rad);
    append(text, (p+"peak_slip_angle_high_rad").c_str(), t.peak_slip_angle_high_rad);
    append(text, (p+"sliding_fraction_longitudinal").c_str(), t.sliding_fraction_longitudinal);
    append(text, (p+"sliding_fraction_lateral").c_str(), t.sliding_fraction_lateral);
    append(text, (p+"minimum_friction").c_str(), t.minimum_friction);
    append(text, (p+"reference_load_low_n").c_str(), t.reference_load_low_n);
    append(text, (p+"reference_load_high_n").c_str(), t.reference_load_high_n);
}

std::string model_identity(const VehicleModel& model, const Config& c) {
    std::string text;
    append(text, "grip_mu", c.grip_mu);
    append(text, "fixed_dt_s", c.fixed_dt_s);
    append(text, "control_dt_s", c.control_dt_s);
    append(text, "wheelbase_m", c.wheelbase_m);
    append(text, "max_speed_mps", c.max_speed_mps);
    append(text, "max_steering_rad", c.max_steering_rad);
    append(text, "max_steering_rate_radps", c.max_steering_rate_radps);
    append(text, "lateral_grip_fraction", c.lateral_grip_fraction);
    append(text, "longitudinal_grip_fraction", c.longitudinal_grip_fraction);
    append(text, "speed_gain", c.speed_gain);
    append(text, "lookahead_base_m", c.lookahead_base_m);
    append(text, "lookahead_time_s", c.lookahead_time_s);
    // envelope_fraction is left out: it chooses how much of an envelope a plan uses, not what the plant does, so
    // tables and envelopes derived before it existed keep their fingerprints.
    if (const auto* car = std::get_if<DynamicSingleTrack>(&model)) {
        text += "dynamic_single_track;";
        append(text, "mass_kg", car->mass_kg);
        append(text, "yaw_inertia_kgm2", car->yaw_inertia_kgm2);
        append(text, "cg_to_rear_m", car->cg_to_rear_m);
        append(text, "drive_front_fraction", car->drive_front_fraction);
        append_tire(text, "front_", car->front_tire);
        append_tire(text, "rear_", car->rear_tire);
        append(text, "kinematic_below_mps", car->kinematic_below_mps);
        append(text, "dynamic_above_mps", car->dynamic_above_mps);
        append(text, "slip_speed_floor_mps", car->slip_speed_floor_mps);
        append(text, "substep_s", car->substep_s);
        // Load transfer and aerodynamics (decision 0025) enter only when set, so the tables of a car without them keep the
        // fingerprints they had before those existed.
        if (car->cg_height_m != 0 || car->drag_area_m2 != 0 || car->downforce_area_m2 != 0 || car->aero_balance_front != 0.5) {
            append(text, "cg_height_m", car->cg_height_m);
            append(text, "drag_area_m2", car->drag_area_m2);
            append(text, "downforce_area_m2", car->downforce_area_m2);
            append(text, "aero_balance_front", car->aero_balance_front);
        }
    } else if (const auto* four = std::get_if<FourWheelCar>(&model)) {
        text += "four_wheel_car;";
        append(text, "mass_kg", four->mass_kg);
        append(text, "yaw_inertia_kgm2", four->yaw_inertia_kgm2);
        append(text, "cg_to_rear_m", four->cg_to_rear_m);
        append(text, "cg_height_m", four->cg_height_m);
        append(text, "roll_balance_front", four->roll_balance_front);
        append(text, "track_front_m", four->track_front_m);
        append(text, "track_rear_m", four->track_rear_m);
        append(text, "wheel_radius_m", four->wheel_radius_m);
        append(text, "wheel_inertia_kgm2", four->wheel_inertia_kgm2);
        append_tire(text, "front_", four->front_tire);
        append_tire(text, "rear_", four->rear_tire);
        append(text, "drive_front_fraction", four->drive_front_fraction);
        append(text, "max_drive_power_w", four->max_drive_power_w);
        append(text, "max_drive_torque_nm", four->max_drive_torque_nm);
        append(text, "brake_bias_front", four->brake_bias_front);
        append(text, "max_brake_torque_nm", four->max_brake_torque_nm);
        // Only a car with a drive controller top speed says so, so every fingerprint made before it is unchanged.
        if (std::isfinite(four->max_drive_speed_mps)) append(text, "max_drive_speed_mps", four->max_drive_speed_mps);
        append(text, "viscous_coupling_nms", four->viscous_coupling_nms);
        append(text, "drag_area_m2", four->drag_area_m2);
        append(text, "downforce_area_m2", four->downforce_area_m2);
        append(text, "aero_balance_front", four->aero_balance_front);
        append(text, "kinematic_below_mps", four->kinematic_below_mps);
        append(text, "dynamic_above_mps", four->dynamic_above_mps);
        append(text, "slip_speed_floor_mps", four->slip_speed_floor_mps);
        append(text, "substep_s", four->substep_s);
    } else {
        text += "kinematic_bicycle;";
    }
    return text;
}

} // namespace fd::detail

namespace fd {

std::string fingerprint_hex(const std::string& text) {
    std::uint64_t h = 14695981039346656037ull;
    for (const unsigned char c : text) { h ^= c; h *= 1099511628211ull; }
    constexpr const char* digits = "0123456789abcdef";
    std::string hex(16, '0');
    for (int i = 15; i >= 0; --i) { hex[static_cast<std::size_t>(i)] = digits[h & 15]; h >>= 4; }
    return hex;
}

} // namespace fd

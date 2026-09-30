// The physics profiles of TrackWayFastPlan §8 (decision 0033). Sources, by the plan's aliases: <FL> fastest-lap (MIT),
// <PAC> PacSim (MIT), <MPCC> MPCC (Apache-2.0), <TUM> TUM's global_racetrajectory_optimization (LGPL-3.0; parameter
// values read, no code taken). Tire fits: tools/vehicle_profiles/fit_tires.py.
#include "fd/vehicle_profiles.hpp"

#include <stdexcept>

namespace fd {
namespace {

using P = Provenance;

VehicleProfile project_roadster() {
    VehicleProfile p;
    p.id = "project-roadster";
    p.name = "Project roadster";
    p.source = "This project's own four-wheel car (decisions 0012 to 0016): chosen, illustrative values; its tires the F1 "
               "file's frictions and slips with sliding shares chosen here";
    p.car = FourWheelCar{};
    const Config defaults;
    p.wheelbase_m = defaults.wheelbase_m;
    p.max_steering_rad = defaults.max_steering_rad;
    p.max_speed_mps = defaults.max_speed_mps;
    p.parameters = {{"every parameter", P::chosen, "the defaults of core/vehicle.hpp and core/core.hpp"}};
    return p;
}

VehicleProfile formula_one_style() {
    VehicleProfile p;
    p.id = "formula-one-style";
    p.name = "Formula One style (Limebeer 2014)";
    p.source = "<FL>/database/vehicles/f1/limebeer-2014-f1.xml, after Limebeer et al. 2014 (Vehicle System Dynamics, "
               "doi 10.1080/00423114.2014.889315): published academic parameters of a Formula One car, not any team's data";
    auto& c = p.car;
    c.mass_kg = 660;
    c.yaw_inertia_kgm2 = 450;
    c.cg_to_rear_m = 1.6;
    c.cg_height_m = 0.3;
    c.roll_balance_front = 0.5;
    c.track_front_m = c.track_rear_m = 1.46;
    c.wheel_radius_m = 0.33;
    c.wheel_inertia_kgm2 = 1.0;
    c.front_tire = c.rear_tire = TireParameters{};  // the file's own frictions, slips and loads
    c.drive_front_fraction = 0;
    c.max_drive_power_w = 735499;
    c.max_drive_torque_nm = 3000;
    c.brake_bias_front = 0.6;
    c.max_brake_torque_nm = 3000;
    c.viscous_coupling_nms = 10.47;
    c.drag_area_m2 = 0.9*1.5;
    c.downforce_area_m2 = 3.0*1.5;
    c.aero_balance_front = (1.6-0.1)/3.4;
    p.wheelbase_m = 1.8+1.6;
    p.max_steering_rad = Config{}.max_steering_rad;
    p.max_speed_mps = 40;
    p.parameters = {
        {"mass_kg", P::published, "660 kg"},
        {"yaw_inertia_kgm2", P::published, "Izz 450 kg m^2"},
        {"wheelbase_m, cg_to_rear_m", P::published, "front axle 1.8 m ahead of the centre of mass, rear 1.6 m behind"},
        {"cg_height_m", P::published, "centre of mass 0.3 m up"},
        {"roll_balance_front", P::published, "roll balance coefficient 0.5"},
        {"track_front_m, track_rear_m", P::published, "1.46 m"},
        {"wheel_radius_m", P::published, "0.33 m"},
        {"wheel_inertia_kgm2", P::chosen, "the file's axles have none, marked as not part of its model"},
        {"front_tire, rear_tire", P::published, "loads 2000 and 6000 N, mu x 1.75/1.40, mu y 1.80/1.45, kappa 0.11/0.10, 9/8 deg"},
        {"sliding fractions", P::chosen, "0.75; the file's Q 1.9 implies 0.156, too little for asphalt (plan section 4.1(a))"},
        {"drive_front_fraction", P::published, "engine on the rear axle"},
        {"max_drive_power_w", P::published, "735.499 kW; the 120 kW boost is not modelled"},
        {"max_drive_torque_nm", P::chosen, "the file limits power only"},
        {"brake_bias_front", P::published, "0.6"},
        {"max_brake_torque_nm", P::derived, "5000 N m times the front bias 0.6, each front wheel's most"},
        {"viscous_coupling_nms", P::published, "rear differential stiffness 10.47 N m s/rad"},
        {"drag_area_m2", P::derived, "cd 0.9 times 1.5 m^2 (the file's air at 1.2 kg/m^3, here 1.225)"},
        {"downforce_area_m2", P::derived, "cl 3.0 times 1.5 m^2"},
        {"aero_balance_front", P::derived, "pressure centre 0.1 m behind the centre of mass: (1.6 - 0.1)/3.4"},
        {"max_steering_rad", P::chosen, "the project's 0.55 rad; the file gives no lock"},
        {"max_speed_mps", P::chosen, "40 m/s, the highest cap this project supports; a Formula One car reaches about 90"}};
    return p;
}

VehicleProfile formula_student_electric() {
    VehicleProfile p;
    p.id = "formula-student-electric";
    p.name = "Formula Student electric";
    p.source = "<PAC>/config/vehicleModel.yaml, which its authors call made up, and <MPCC>/C++/Params/model.json: "
               "representative of a Formula Student car, not measured";
    auto& c = p.car;
    c.mass_kg = 178;
    c.yaw_inertia_kgm2 = 111;
    c.cg_to_rear_m = 0.72;
    c.cg_height_m = 0.3;
    c.roll_balance_front = 0.5;
    c.track_front_m = c.track_rear_m = 1.15;
    c.wheel_radius_m = 0.206;
    c.wheel_inertia_kgm2 = 0.3;
    TireParameters tire;
    tire.reference_load_low_n = 300;
    tire.reference_load_high_n = 700;
    tire.peak_friction_longitudinal_low = tire.peak_friction_longitudinal_high = 1.6;
    tire.peak_friction_lateral_low = tire.peak_friction_lateral_high = 1.6;
    tire.peak_slip_ratio_low = tire.peak_slip_ratio_high = 0.12;
    tire.peak_slip_angle_low_rad = tire.peak_slip_angle_high_rad = 0.2166;
    tire.sliding_fraction_lateral = 0.948;
    tire.sliding_fraction_longitudinal = 0.75;
    tire.minimum_friction = 1.0;
    c.front_tire = c.rear_tire = tire;
    c.drive_front_fraction = 0;
    c.max_drive_power_w = 80000;
    c.max_drive_torque_nm = 400;
    c.brake_bias_front = 0.6;
    c.max_brake_torque_nm = 400;
    c.viscous_coupling_nms = 0;
    c.drag_area_m2 = 1.1*1.1;
    c.downforce_area_m2 = 3.7*1.1;
    c.aero_balance_front = 0.5;
    p.wheelbase_m = 0.72+0.72;
    p.max_steering_rad = Config{}.max_steering_rad;
    p.max_speed_mps = 30;
    p.parameters = {
        {"mass_kg", P::published, "PacSim m 178 kg (MPCC 190 kg)"},
        {"yaw_inertia_kgm2", P::published, "PacSim Izz 111 kg m^2 (MPCC 110)"},
        {"wheelbase_m, cg_to_rear_m", P::published, "PacSim lf and lr 0.72 m"},
        {"cg_height_m", P::chosen, "0.3 m; neither file gives one"},
        {"roll_balance_front", P::chosen, "0.5"},
        {"track_front_m, track_rear_m", P::published, "PacSim sf and sr 1.15 m"},
        {"wheel_radius_m", P::published, "PacSim 0.206 m"},
        {"wheel_inertia_kgm2", P::chosen, "0.3 kg m^2"},
        {"peak frictions", P::published, "PacSim Dlat 1.6; MPCC's D 1500 N over a 190 kg car's axle load gives 1.61"},
        {"peak_slip_angle", P::fitted, "12.4 deg, MPCC's B 10, C 1.38 (PacSim's C -1.39 and E 1 never peak)"},
        {"sliding_fraction_lateral", P::fitted, "0.948 of peak kept at 0.5 rad on MPCC's curve"},
        {"peak_slip_ratio, sliding_fraction_longitudinal", P::chosen, "0.12 and 0.75; neither file models slip ratio"},
        {"tire reference loads", P::chosen, "300 and 700 N about the static 436 N; no load sensitivity in either file"},
        {"drive_front_fraction", P::chosen, "rear-wheel drive through PacSim's single gear ratio"},
        {"max_drive_power_w", P::chosen, "80 kW, the Formula Student rules' limit; PacSim gives none"},
        {"max_drive_torque_nm, max_brake_torque_nm", P::chosen, "400 N m each wheel"},
        {"brake_bias_front", P::chosen, "0.6"},
        {"drag_area_m2", P::derived, "PacSim cda 1.1 times aeroArea 1.1 (its air at 1.29 kg/m^3, here 1.225)"},
        {"downforce_area_m2", P::derived, "PacSim cla 3.7 times aeroArea 1.1"},
        {"aero_balance_front", P::chosen, "0.5"},
        {"max_steering_rad", P::chosen, "the project's 0.55 rad"},
        {"max_speed_mps", P::chosen, "30 m/s"}};
    return p;
}

VehicleProfile electric_race_car() {
    VehicleProfile p;
    p.id = "electric-race-car";
    p.name = "Electric race car (TUM example)";
    p.source = "<TUM>/params/racecar.ini, veh_params, vehicle_params_mintime and tire_params_mintime: the example "
               "parameters of TUM's minimum-time optimisation, not a measured car";
    auto& c = p.car;
    c.mass_kg = 1200;
    c.yaw_inertia_kgm2 = 1200;
    c.cg_to_rear_m = 1.4;
    c.cg_height_m = 0.38;
    c.roll_balance_front = 0.5;
    c.track_front_m = c.track_rear_m = 1.6;
    c.wheel_radius_m = 0.3;
    c.wheel_inertia_kgm2 = 1.2;
    TireParameters tire;
    tire.reference_load_low_n = 2000;
    tire.reference_load_high_n = 6000;
    // TUM's solver scales each tire's lateral peak by 1 + eps Fz / f_z0 (opt_mintime.py), not by the load's increment
    // over f_z0, and bounds its longitudinal force by the friction circle of mue alone (decision 0034 found the first).
    tire.peak_friction_longitudinal_low = tire.peak_friction_longitudinal_high = 1.0;
    tire.peak_friction_lateral_low = 1-0.1*2000.0/3000.0;
    tire.peak_friction_lateral_high = 1-0.1*6000.0/3000.0;
    tire.peak_slip_ratio_low = tire.peak_slip_ratio_high = 0.0889;
    tire.peak_slip_angle_low_rad = tire.peak_slip_angle_high_rad = 0.0889;
    tire.sliding_fraction_longitudinal = tire.sliding_fraction_lateral = 0.709;
    tire.minimum_friction = 0.6;
    c.front_tire = c.rear_tire = tire;
    c.drive_front_fraction = 0;
    c.max_drive_power_w = 230000;
    c.max_drive_torque_nm = 7000*0.3/2;
    c.brake_bias_front = 0.6;
    c.max_brake_torque_nm = 20000*0.6*0.3/2;
    c.viscous_coupling_nms = 0;
    c.drag_area_m2 = 2*0.75/1.225;
    c.downforce_area_m2 = 2*(0.45+0.75)/1.225;
    c.aero_balance_front = 0.45/(0.45+0.75);
    p.wheelbase_m = 1.6+1.4;
    p.max_steering_rad = 0.35;
    p.max_speed_mps = 40;
    p.parameters = {
        {"mass_kg", P::published, "1200 kg"},
        {"yaw_inertia_kgm2", P::published, "I_z 1200 kg m^2"},
        {"wheelbase_m, cg_to_rear_m", P::published, "wheelbase_front 1.6 m, wheelbase_rear 1.4 m"},
        {"cg_height_m", P::published, "cog_z 0.38 m"},
        {"roll_balance_front", P::published, "k_roll 0.5"},
        {"track_front_m, track_rear_m", P::published, "1.6 m"},
        {"wheel_radius_m", P::published, "r_wheel 0.3 m"},
        {"wheel_inertia_kgm2", P::chosen, "1.2 kg m^2; the file models none"},
        {"peak frictions", P::derived, "lateral: mue 1 times 1 + eps Fz / f_z0 with eps -0.1 and f_z0 3000 N, as TUM's solver "
                                       "computes it: 0.933 at 2000 N, 0.800 at 6000 N; longitudinal: the friction circle of mue 1"},
        {"minimum_friction", P::chosen, "0.6, below TUM's lateral friction at any load this car reaches"},
        {"peak slips, sliding fractions", P::fitted, "B 10, C 2.5, E 1: peak at 5.09 deg, 0.709 kept at 0.5 rad; the same for slip ratio"},
        {"drive_front_fraction", P::published, "k_drive_front 0"},
        {"max_drive_power_w", P::published, "power_max 230 kW"},
        {"max_drive_torque_nm", P::derived, "f_drive_max 7000 N at the 0.3 m wheel, over two driven wheels"},
        {"brake_bias_front", P::published, "k_brake_front 0.6"},
        {"max_brake_torque_nm", P::derived, "f_brake_max 20000 N times the front bias at the 0.3 m wheel, over two"},
        {"drag_area_m2", P::derived, "dragcoeff 0.75 kg/m is half the air density times cd A, at 1.225 kg/m^3"},
        {"downforce_area_m2, aero_balance_front", P::derived, "liftcoeff front 0.45 and rear 0.75 kg/m the same way"},
        {"max_steering_rad", P::published, "delta_max 0.35 rad"},
        {"max_speed_mps", P::chosen, "40 m/s, the highest cap this project supports; the file's v_max is 70 m/s"}};
    return p;
}

VehicleProfile racing_kart() {
    VehicleProfile p;
    p.id = "racing-kart";
    p.name = "Racing kart (Lot 2016)";
    p.source = "<FL>/database/vehicles/kart/roberto-lot-kart-2016.xml, after Lot 2016 (eprints.soton.ac.uk/385883): published "
               "academic parameters of a racing kart; its six-degree chassis reduced to this car's quasi-static load transfer";
    auto& c = p.car;
    c.mass_kg = 165;
    c.yaw_inertia_kgm2 = 25;
    c.cg_to_rear_m = 0.4;
    c.cg_height_m = 0.25;
    c.roll_balance_front = 17.7/(17.7+60.0);
    c.track_front_m = 1.055;
    c.track_rear_m = 1.2;
    c.wheel_radius_m = 0.139;
    c.wheel_inertia_kgm2 = 0.1;
    TireParameters tire;
    tire.reference_load_low_n = 300;
    tire.reference_load_high_n = 700;
    tire.peak_friction_longitudinal_low = tire.peak_friction_longitudinal_high = 0.9;
    tire.peak_friction_lateral_low = tire.peak_friction_lateral_high = 1.5;
    tire.peak_slip_ratio_low = 0.0794;
    tire.peak_slip_ratio_high = 0.0970;
    tire.peak_slip_angle_low_rad = 0.0776;
    tire.peak_slip_angle_high_rad = 0.0920;
    tire.sliding_fraction_longitudinal = 0.71;
    tire.sliding_fraction_lateral = 0.61;
    tire.minimum_friction = 0.8;
    c.front_tire = c.rear_tire = tire;
    c.drive_front_fraction = 0;
    c.max_drive_power_w = 20100;
    c.max_drive_torque_nm = 150;
    c.brake_bias_front = 0;
    c.max_brake_torque_nm = 200;
    c.viscous_coupling_nms = 200;
    c.drag_area_m2 = 0.7;
    c.downforce_area_m2 = 0;
    c.aero_balance_front = 0.5;
    p.wheelbase_m = 0.645+0.4;
    p.max_steering_rad = Config{}.max_steering_rad;
    p.max_speed_mps = 30;
    p.parameters = {
        {"mass_kg", P::published, "165 kg"},
        {"yaw_inertia_kgm2", P::published, "Izz 25 kg m^2"},
        {"wheelbase_m, cg_to_rear_m", P::published, "front axle 0.645 m ahead of the centre of mass, rear 0.4 m behind"},
        {"cg_height_m", P::published, "centre of mass 0.25 m, as the file's z gives it"},
        {"roll_balance_front", P::derived, "chassis stiffness front 17.7 and rear 60 kN/m: 17.7/77.7"},
        {"track_front_m, track_rear_m", P::published, "1.055 and 1.2 m"},
        {"wheel_radius_m", P::published, "0.139 m"},
        {"wheel_inertia_kgm2", P::derived, "the rear axle's 0.2 kg m^2 shared by its two wheels"},
        {"peak frictions", P::published, "pDy1 1.5, pDx1 0.9, no load sensitivity"},
        {"peak slips, sliding fractions", P::fitted, "MF 5.2 pure slip at 300 and 700 N: 4.44/5.27 deg, 0.079/0.097; 0.61 and 0.71 kept at 0.5"},
        {"drive_front_fraction", P::published, "engine on the rear axle"},
        {"max_drive_power_w", P::published, "20.1 kW"},
        {"max_drive_torque_nm", P::chosen, "150 N m each rear wheel; the file limits power only"},
        {"brake_bias_front", P::published, "brakes on the rear axle alone"},
        {"max_brake_torque_nm", P::published, "200 N m"},
        {"viscous_coupling_nms", P::derived, "a kart's rear axle is solid: 200 N m s/rad, the model's stiffest, behaves as locked"},
        {"drag_area_m2", P::derived, "cd 0.7 times 1 m^2; cl 0"},
        {"max_steering_rad", P::chosen, "the project's 0.55 rad; the file gives only toe"},
        {"max_speed_mps", P::chosen, "30 m/s"}};
    return p;
}

// A rental kart as Gokartcentralen Göteborg runs it (decision 0038): a Sodi RSX2 with a driver, on hard rental tires on a
// smooth indoor floor. The operator publishes its power, top speed and track; Sodikart its mass and size; neither the
// geometry nor a tire model, which come from the racing kart above, its tires' grip lowered to a rental kart's.
VehicleProfile gokartcentralen_rsx2() {
    VehicleProfile p;
    p.id = "gokartcentralen-rsx2";
    p.name = "Rental kart (Sodi RSX2, Gokartcentralen Göteborg)";
    p.source = "gokartcentralen.se: electric karts of about 14 hp and about 60 km/h on the straight, the Göteborg track 400 m "
               "long and 6 m wide; sodikart.com RSX2: 186 kg with its lithium batteries, 1865 x 1350 x 630 mm. Geometry and the "
               "tire's shape from the racing kart (Lot 2016), which neither source publishes";
    const auto lot = racing_kart();
    auto& c = p.car;
    c = lot.car;
    c.mass_kg = 186+75;
    c.yaw_inertia_kgm2 = lot.car.yaw_inertia_kgm2*c.mass_kg/lot.car.mass_kg;
    for (auto* tire : {&c.front_tire, &c.rear_tire}) {
        tire->peak_friction_longitudinal_low = tire->peak_friction_longitudinal_high = 1.0;
        tire->peak_friction_lateral_low = tire->peak_friction_lateral_high = 1.0;
    }
    c.max_drive_power_w = 14*735.49875;
    c.max_drive_torque_nm = 115;
    c.max_drive_speed_mps = 60/3.6;
    c.brake_bias_front = 0.4;
    c.max_brake_torque_nm = 150;
    c.drag_area_m2 = 0.78;
    c.viscous_coupling_nms = 1;
    p.wheelbase_m = lot.wheelbase_m;
    p.max_steering_rad = lot.max_steering_rad;
    p.max_speed_mps = 60/3.6;
    p.parameters = {
        {"mass_kg", P::published, "RSX2 186 kg with batteries (Sodikart), plus a 75 kg driver, chosen"},
        {"yaw_inertia_kgm2", P::derived, "the racing kart's 25 kg m^2 scaled by mass, 261/165"},
        {"wheelbase_m, cg_to_rear_m, cg_height_m, roll_balance_front", P::chosen, "the racing kart's (Lot 2016); unpublished for the RSX2"},
        {"track_front_m, track_rear_m, wheel_radius_m, wheel_inertia_kgm2", P::chosen, "the racing kart's; the RSX2 is 1350 mm wide overall"},
        {"peak frictions", P::chosen, "1.0 both ways: hard rental tires on a smooth indoor floor, 0.95 to 1.05"},
        {"peak slips, sliding fractions", P::fitted, "the racing kart's fit (Lot 2016)"},
        {"drive_front_fraction", P::derived, "a kart drives its rear axle"},
        {"viscous_coupling_nms", P::chosen, "1 N m s/rad: a kart's solid axle turns by lifting its inner rear wheel, which "
         "this model does not; locked it scrubs to 0.4 g at 8 m/s, open the inner rear wheel locks under braking and spins "
         "the kart; 1 corners at 0.89 g and brakes straight"},
        {"max_drive_power_w", P::derived, "about 14 hp (Gokartcentralen) at 735.5 W per metric horsepower"},
        {"max_drive_torque_nm", P::chosen, "about 1.65 kN at the rear axle at 0.139 m, over two wheels"},
        {"max_drive_speed_mps", P::published, "about 60 km/h on the straight (Gokartcentralen)"},
        {"brake_bias_front", P::chosen, "0.4: the kart brakes at its tires' 0.75 g without locking an axle first; a rear axle "
         "alone locks near 0.45 g, below the 0.60 to 0.68 g rental karts brake at, and the RSX2's brake layout is unpublished"},
        {"max_brake_torque_nm", P::chosen, "150 N m each wheel, enough to lock the rear"},
        {"drag_area_m2", P::chosen, "0.78 m^2, a seated driver and an exposed kart, no downforce"},
        {"max_steering_rad", P::chosen, "the project's 0.55 rad"},
        {"max_speed_mps", P::published, "the plan's cap at the kart's own top speed, 60 km/h"}};
    return p;
}

} // namespace

const char* provenance_name(Provenance provenance) {
    switch (provenance) {
    case Provenance::published: return "published";
    case Provenance::derived: return "derived";
    case Provenance::fitted: return "fitted";
    case Provenance::chosen: return "chosen";
    }
    return "chosen";
}

const std::vector<VehicleProfile>& vehicle_profiles() {
    static const std::vector<VehicleProfile> profiles{project_roadster(), formula_one_style(), formula_student_electric(),
                                                      electric_race_car(), racing_kart(), gokartcentralen_rsx2()};
    return profiles;
}

const VehicleProfile& vehicle_profile(std::string_view id) {
    std::string ids;
    for (const auto& p : vehicle_profiles()) {
        if (p.id == id) return p;
        ids += (ids.empty() ? "" : ", ")+p.id;
    }
    throw std::invalid_argument("Unknown vehicle profile '"+std::string(id)+"'; the profiles are "+ids);
}

Config configured_for(const VehicleProfile& profile, Config config) {
    config.wheelbase_m = profile.wheelbase_m;
    config.max_steering_rad = profile.max_steering_rad;
    config.max_speed_mps = profile.max_speed_mps;
    validate_config(config);
    validate_vehicle(profile.car, config);
    return config;
}

} // namespace fd

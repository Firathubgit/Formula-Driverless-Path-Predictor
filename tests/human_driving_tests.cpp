#include "fd/human_driving.hpp"
#include "fd/recording.hpp"
#include "fd/simulation.hpp"
#include "fd/vehicle_profiles.hpp"

#include <cmath>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// A person driving the car (decision 0037): what their controls ask of the plant, and the simulation driven by them.
namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tolerance)
        throw std::runtime_error(message+": actual="+std::to_string(actual)+", expected="+std::to_string(expected));
}
template<class Error, class Action> void rejects(Action action, const std::string& message) {
    bool rejected = false;
    try { action(); } catch (const Error&) { rejected = true; }
    require(rejected, message);
}

fd::PlantState moving(const fd::VehicleModel& model, const fd::Config& config, double speed) {
    return fd::plant_state_from({0, 0, 0, speed, 0, 0}, model, config);
}
void steps(fd::Simulation& simulation, double seconds) {
    const auto count = static_cast<int>(std::lround(seconds/simulation.config().fixed_dt_s));
    for (int i = 0; i < count; ++i) simulation.step();
}

void controls_are_validated() {
    const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
    for (const fd::DriverControls bad : {fd::DriverControls{nan, 0, 0}, fd::DriverControls{0, inf, 0}, fd::DriverControls{0, 0, nan},
                                         fd::DriverControls{-0.01, 0, 0}, fd::DriverControls{1.01, 0, 0}, fd::DriverControls{0, 2, 0},
                                         fd::DriverControls{0, -0.5, 0}, fd::DriverControls{0, 0, 1.01}, fd::DriverControls{0, 0, -1.01}})
        rejects<std::invalid_argument>([&] { fd::validate_driver_controls(bad); }, "controls outside their travel are refused");
    fd::validate_driver_controls({0, 0, 0});
    fd::validate_driver_controls({1, 1, 1});
    fd::validate_driver_controls({1, 1, -1});
    // Refused controls leave the ones in force alone.
    fd::Simulation simulation;
    simulation.set_driver_controls({0.4, 0, -0.2});
    rejects<std::invalid_argument>([&] { simulation.set_driver_controls({nan, 0, 0}); }, "the simulation refuses invalid controls");
    require(simulation.driver_controls().throttle == 0.4 && simulation.driver_controls().steering == -0.2,
            "refused controls change nothing");
}

void pedals_ask_for_the_road_grip() {
    fd::Config config;
    const fd::VehicleModel kinematic = fd::KinematicBicycle{};
    const auto state = moving(kinematic, config, 10);
    const double grip = config.grip_mu*fd::gravity_mps2;
    near(fd::human_command({1, 0, 0}, state, kinematic, config).acceleration_mps2, grip, 1e-12, "a floored accelerator asks for the grip");
    near(fd::human_command({0, 1, 0}, state, kinematic, config).acceleration_mps2, -grip, 1e-12, "a floored brake asks for it in reverse");
    near(fd::human_command({0.5, 0.25, 0}, state, kinematic, config).acceleration_mps2, 0.25*grip, 1e-12, "both pedals ask for the difference");
    near(fd::human_command({0, 0, 0}, state, kinematic, config).acceleration_mps2, 0, 0, "released pedals ask for nothing");
    config.grip_mu = 0.6;
    near(fd::human_pedal_acceleration_mps2(state, kinematic, config), 0.6*fd::gravity_mps2, 1e-12, "less grip, less asked");
    // A car with wings asks its downforce's share more, at its speed.
    fd::FourWheelCar winged;
    winged.downforce_area_m2 = 3;
    const fd::VehicleModel model = winged;
    const auto fast = moving(model, config, 50);
    const double downforce = fd::aerodynamic_force(winged, 50, 0).downforce_n;
    require(downforce > 1000, "the winged car makes downforce at 50 m/s");
    near(fd::human_pedal_acceleration_mps2(fast, model, config), 0.6*fd::gravity_mps2*(1+downforce/(winged.mass_kg*fd::gravity_mps2)), 1e-9,
         "downforce adds its share of the weight");
}

void steering_narrows_with_speed() {
    const fd::Config config;
    const fd::VehicleModel kinematic = fd::KinematicBicycle{};
    near(fd::human_command({0, 0, 1}, moving(kinematic, config, 0), kinematic, config).steering_rad, config.max_steering_rad, 1e-12,
         "standing still the full stick is the lock");
    near(fd::human_command({0, 0, -1}, moving(kinematic, config, 0), kinematic, config).steering_rad, -config.max_steering_rad, 1e-12,
         "the stick steers right as far as left");
    const auto fast = moving(kinematic, config, 30);
    const double limit = std::atan(config.wheelbase_m*config.grip_mu*fd::gravity_mps2/900);
    near(fd::human_steering_limit_rad(fast, kinematic, config), limit, 1e-12, "at speed the geometric turn asks no more than the grip");
    near(fd::human_command({0, 0, 1}, fast, kinematic, config).steering_rad, limit, 1e-12, "the full stick reaches the limit");
    near(fd::human_command({0, 0, 0.5}, fast, kinematic, config).steering_rad, std::pow(0.5, 1.5)*limit, 1e-12,
         "half the stick steers less than half, for fine control");
    near(fd::human_command({0, 0, -0.5}, fast, kinematic, config).steering_rad, -std::pow(0.5, 1.5)*limit, 1e-12, "and keeps its side");
    // The turn the kinematic bicycle then makes asks exactly the grip laterally.
    near(30*30*std::tan(limit)/config.wheelbase_m, config.grip_mu*fd::gravity_mps2, 1e-9, "the kinematic car's turn at the limit");
    // A car with tires is given its front tires' peak slip angle besides, to reach its grip.
    const fd::FourWheelCar car;
    const fd::VehicleModel wheels = car;
    const double peak = std::max(car.front_tire.peak_slip_angle_low_rad, car.front_tire.peak_slip_angle_high_rad);
    near(fd::human_steering_limit_rad(moving(wheels, config, 30), wheels, config), limit+peak, 1e-12, "tires get their peak slip angle");
    double previous = config.max_steering_rad;
    for (double speed = 1; speed <= 90; speed += 1) {
        const double now = fd::human_steering_limit_rad(moving(wheels, config, speed), wheels, config);
        require(now <= previous+1e-15 && now <= config.max_steering_rad && now > 0, "the limit never rises with speed nor passes the lock");
        previous = now;
    }
}

void a_person_drives_without_a_speed_cap() {
    fd::Simulation simulation;
    simulation.set_human_driving(true);
    require(simulation.human_driving() && simulation.controller_outcome().driven_by == fd::ControllerMode::human,
            "the decision made on taking the car says a person drives it");
    simulation.set_driver_controls({1, 0, 0});
    const auto decisions = simulation.decision_count();
    steps(simulation, 3);
    require(simulation.decision_count() >= decisions+149, "the planner still decides at every control tick");
    require(simulation.controller_outcome().driven_by == fd::ControllerMode::human, "every decision says the person drove it");
    near(simulation.diagnostics().applied.acceleration_mps2, fd::gravity_mps2, 1e-9, "the floored pedal's command drives");
    require(simulation.state().speed_mps > simulation.config().max_speed_mps+5,
            "nothing caps the speed: the plan's cap is the autonomous driver's, not the car's");
    near(simulation.state().speed_mps, 3*fd::gravity_mps2, 1e-6, "the kinematic car gives all it is asked");
    // The four-wheel car gives what its power, drag and tires allow, and still passes the plan's cap.
    fd::Simulation four(fd::make_preset_track(), fd::Config{}, fd::FourWheelCar{});
    four.set_human_driving(true);
    four.set_driver_controls({1, 0, 0});
    steps(four, 4);
    require(four.state().speed_mps > four.config().max_speed_mps && four.state().speed_mps < 4*fd::gravity_mps2-5,
            "the four-wheel car accelerates past the plan's cap as its own physics allow, "+std::to_string(four.state().speed_mps)+" m/s");
}

void controls_take_effect_at_the_next_control_decision() {
    fd::Simulation simulation;
    simulation.set_human_driving(true);
    simulation.step();   // tick 0 decided with released pedals
    simulation.set_driver_controls({1, 0, 0});
    for (int tick = 1; tick < 4; ++tick) {
        simulation.step();
        near(simulation.diagnostics().applied.acceleration_mps2, 0, 0, "the controls wait for the next control decision");
    }
    simulation.step();   // tick 4 is a control decision
    near(simulation.diagnostics().applied.acceleration_mps2, fd::gravity_mps2, 1e-9, "and drive from it");
}

void brake_stops_without_reversing() {
    fd::Simulation simulation;
    simulation.set_human_driving(true);
    simulation.set_driver_controls({1, 0, 0});
    steps(simulation, 1);
    require(simulation.state().speed_mps > 9, "the car is moving");
    simulation.set_driver_controls({0, 1, 0});
    steps(simulation, 2);
    const auto stopped = simulation.state();
    near(stopped.speed_mps, 0, 0, "the brake stops the car");
    steps(simulation, 1);
    near(simulation.state().speed_mps, 0, 0, "and holds it without reversing");
    near(simulation.state().x_m, stopped.x_m, 1e-12, "a stopped car stays where it stopped");
}

void the_stick_turns_the_car_its_way() {
    for (const double side : {1.0, -1.0}) {
        fd::Simulation simulation;
        simulation.set_human_driving(true);
        simulation.set_driver_controls({1, 0, 0});
        steps(simulation, 0.8);
        const double yaw = simulation.state().yaw_rad;
        simulation.set_driver_controls({0, 0, side});
        steps(simulation, 0.5);
        require((simulation.state().yaw_rad-yaw)*side > 0.1, side > 0 ? "a left stick turns left" : "a right stick turns right");
    }
}

void handing_back_restores_the_autonomous_driver() {
    fd::Simulation simulation;
    simulation.set_human_driving(true);
    simulation.set_driver_controls({0.3, 0, 0.2});
    steps(simulation, 1);
    simulation.set_human_driving(false);
    require(!simulation.human_driving() && simulation.controller_outcome().driven_by == fd::ControllerMode::policy,
            "handed back, the next decision is the policy's");
    const auto& first = simulation.decision().trajectory().first_control;
    near(simulation.diagnostics().requested.acceleration_mps2, first.requested.acceleration_mps2, 0, "the planner's action drives again");
    near(simulation.diagnostics().requested.steering_rad, first.requested.steering_rad, 0, "its steering too");
    // A reset releases the controls, as the run starts over.
    simulation.set_human_driving(true);
    simulation.set_driver_controls({1, 0, 1});
    simulation.reset();
    require(simulation.human_driving() && simulation.driver_controls().throttle == 0 && simulation.driver_controls().steering == 0,
            "a reset keeps who drives and releases the controls");
}

void a_persons_drive_is_not_recorded() {
    const auto root = std::filesystem::temp_directory_path()/"fd-human-driving-tests";
    std::filesystem::remove_all(root);
    fd::RunRequest request;
    request.requested_laps = 0;
    request.duration_cap_s = 1;
    fd::Simulation driven;
    driven.set_human_driving(true);
    rejects<std::invalid_argument>([&] { fd::RecordingWriter(root/"driven", driven, request, {}); },
                                   "a recording of a person's drive is refused at its start");
    require(!std::filesystem::exists(root/"driven"), "and nothing is written");
    {
        fd::Simulation taken;
        fd::RecordingWriter writer(root/"taken", taken, request, {});
        taken.step();
        writer.record(taken);
        taken.set_human_driving(true);
        taken.step();
        rejects<std::logic_error>([&] { writer.record(taken); }, "a person taking the car mid-recording is refused");
    }
    std::filesystem::remove_all(root);
}

void forgiveness_is_opt_in_and_validated() {
    const auto& profile=fd::vehicle_profile("gokartcentralen-rsx2");
    const auto config=fd::configured_for(profile,fd::Config{});
    const fd::VehicleModel model=profile.car;
    const auto state=moving(model,config,9);
    const fd::DriverControls controls{0.2,0.6,0.7};
    const auto command=fd::human_command(controls,state,model,config);
    const auto plain=fd::advance(model,state,command,config,0.005);
    for (const auto assistance : {fd::DrivingAssistance{false,100},fd::DrivingAssistance{true,1},
                                 fd::DrivingAssistance{false,100,true,100,1,100,1,100}}) {
        const auto same=fd::human_command(controls,state,model,config,assistance);
        near(same.acceleration_mps2,command.acceleration_mps2,0,"off and 1 keep the pedals exact");
        near(same.steering_rad,command.steering_rad,0,"off and 1 keep steering exact");
        const auto actual=fd::advance_assisted(model,state,same,config,0.005,assistance);
        near(actual.pose.x_m,plain.pose.x_m,0,"off and 1 keep the plant exact");
        near(actual.pose.y_m,plain.pose.y_m,0,"off and 1 keep lateral integration exact");
        near(actual.pose.speed_mps,plain.pose.speed_mps,0,"off and 1 keep speed exact");
        near(actual.yaw_rate_radps,plain.yaw_rate_radps,0,"off and 1 keep yaw exact");
        require(actual.wheel_speeds_radps==plain.wheel_speeds_radps,"off and 1 keep wheels exact");
    }
    for (double level : {0.0,101.0,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()})
        rejects<std::invalid_argument>([&]{fd::human_command(controls,state,model,config,{true,level});},"invalid forgiveness refused");
    rejects<std::invalid_argument>([&]{fd::advance_assisted(fd::KinematicBicycle{},state,command,config,0.005,{true,100});},
                                  "assistance does not alter the exact kinematic adapter");
    rejects<std::invalid_argument>([&]{fd::advance_assisted(model,state,command,config,-1,{true,100});},"invalid assisted dt refused");
}

struct AssistedRun { fd::PlantState state; double peak_slip{},peak_speed{},distance{}; };
AssistedRun assisted_drive_with(fd::DrivingAssistance assistance,double speed,double seconds,fd::DriverControls controls,double dt=0.005) {
    const auto& profile=fd::vehicle_profile("gokartcentralen-rsx2");
    const auto config=fd::configured_for(profile,fd::Config{});
    const fd::VehicleModel model=profile.car;
    AssistedRun result{moving(model,config,speed)};
    fd::Command command;
    const int cadence=static_cast<int>(std::lround(config.control_dt_s/dt));
    for(int i=0;i<static_cast<int>(std::lround(seconds/dt));++i) {
        if(i%cadence==0) command=fd::human_command(controls,result.state,model,config,assistance);
        const auto before=result.state;
        result.state=fd::advance_assisted(model,before,command,config,dt,assistance);
        result.peak_slip=std::max(result.peak_slip,std::abs(fd::rear_sideslip_rad(result.state)));
        result.peak_speed=std::max(result.peak_speed,result.state.pose.speed_mps);
        result.distance+=std::hypot(result.state.pose.x_m-before.pose.x_m,result.state.pose.y_m-before.pose.y_m);
        require(std::isfinite(result.state.pose.x_m) && std::isfinite(result.state.pose.yaw_rad),"assisted motion stays finite");
    }
    return result;
}
AssistedRun assisted_drive(double level,double speed,double seconds,fd::DriverControls controls,double dt=0.005) {
    return assisted_drive_with({true,level},speed,seconds,controls,dt);
}

void manual_forgiveness_controls_are_independent_and_effective() {
    const auto& profile=fd::vehicle_profile("gokartcentralen-rsx2");
    const auto config=fd::configured_for(profile,fd::Config{});
    const fd::VehicleModel model=profile.car;
    fd::DrivingAssistance manual{true,100,true,50,50,50,100,100};
    auto low=manual,high=manual;
    low.acceleration=1; high.acceleration=100;
    require(assisted_drive_with(high,0,1,{1,0,0}).state.pose.speed_mps>
            assisted_drive_with(low,0,1,{1,0,0}).state.pose.speed_mps*5,"acceleration independently changes actual speed gain");
    const auto state=moving(model,config,8);
    const auto weak=fd::human_command({1,0.5,0.5},state,model,config,low);
    const auto strong=fd::human_command({1,0.5,0.5},state,model,config,high);
    near(weak.steering_rad,strong.steering_rad,0,"acceleration does not change steering");
    near(fd::human_command({0,1,0},state,model,config,low).acceleration_mps2,
         fd::human_command({0,1,0},state,model,config,high).acceleration_mps2,0,"acceleration does not change braking");
    low=high=manual; low.braking=1; high.braking=100;
    require(assisted_drive_with(high,9,1,{0,1,0}).state.pose.speed_mps<0.01 &&
            assisted_drive_with(low,9,1,{0,1,0}).state.pose.speed_mps>7,"braking changes actual stopping response");
    low=high=manual; low.steering=1; high.steering=100;
    require(std::abs(assisted_drive_with(high,8,0.5,{0,0,0.6}).state.pose.yaw_rad)>
            std::abs(assisted_drive_with(low,8,0.5,{0,0,0.6}).state.pose.yaw_rad)*2,"steering changes actual turning response");
    low=high=manual; low.grip=1; high.grip=100;
    require(fd::driving_assistance_active(low) && fd::driving_assistance_strength(low)==0,"manual pedals and steering still work at normal grip");
    require(assisted_drive_with(low,9,1.5,{0,0.25,0.8}).peak_slip>0.001 &&
            assisted_drive_with(high,9,1.5,{0,0.25,0.8}).peak_slip==0,"grip independently removes sliding at 100");
    low=high=manual; low.speed=1; high.speed=100;
    near(assisted_drive_with(low,0,10,{1,0,0}).state.pose.speed_mps*3.6,10,0.02,"lowest manual speed reaches 10 km/h");
    near(assisted_drive_with(high,0,10,{1,0,0}).state.pose.speed_mps*3.6,60,0.02,"highest manual speed reaches 60 km/h");
    for(auto field : {&fd::DrivingAssistance::acceleration,&fd::DrivingAssistance::braking,&fd::DrivingAssistance::steering,
                     &fd::DrivingAssistance::grip,&fd::DrivingAssistance::speed}) {
        auto invalid=manual; invalid.*field=std::numeric_limits<double>::quiet_NaN();
        rejects<std::invalid_argument>([&]{fd::validate_driving_assistance(invalid);},"every manual control is validated");
    }
    fd::Simulation simulation(fd::make_preset_track(),config,profile.car);
    simulation.set_human_driving(true);
    simulation.set_driving_assistance(manual);
    const auto revision=simulation.diagnostics().revision;
    require(!simulation.driving_assistance().manual,"manual edits wait for a tick");
    simulation.step();
    require(simulation.driving_assistance()==manual && simulation.diagnostics().revision==revision+1,"all manual fields commit together");
    for(const auto& event:simulation.events()) require(event.time_s==0 && event.revision==revision+1,"manual events share a tick and revision");
    simulation.reset();
    require(simulation.driving_assistance()==manual,"restart retains manual tuning");
}

void forgiveness_measurably_changes_driving() {
    double previous_cap=100,previous_stop=100,previous_slip=100;
    for (double level : {1.0,50.0,70.0,80.0,90.0,100.0}) {
        const auto throttle=assisted_drive(level,0,8,{1,0,0});
        const auto braking=assisted_drive(level,9,3,{0,1,0});
        const auto turning=assisted_drive(level,9,1.5,{0,0.25,0.8});
        std::cout << "FORGIVENESS " << level << " speed_kmh=" << throttle.state.pose.speed_mps*3.6
                  << " stop_m=" << braking.distance << " slip_deg=" << turning.peak_slip*180/std::numbers::pi << '\n';
        require(throttle.peak_speed<previous_cap,"higher forgiveness lowers the achieved top speed");
        require(braking.distance<previous_stop,"higher forgiveness shortens the stopping distance");
        require(turning.peak_slip<previous_slip,"higher forgiveness reduces brake-and-steer sideslip");
        near(braking.state.pose.speed_mps,0,1e-6,"each level comes to rest without reversing");
        previous_cap=throttle.peak_speed;previous_stop=braking.distance;previous_slip=turning.peak_slip;
        if(level>=50) require(turning.peak_slip<2*std::numbers::pi/180,"50 and above is already easy and planted");
        if(level==100) {
            near(throttle.state.pose.speed_mps*3.6,35,0.01,"100 reaches its gentle 35 km/h cap");
            near(turning.peak_slip,0,0,"100 has no rear-axle sideslip");
        }
    }
    const auto left=assisted_drive(100,9,0.5,{0,0,1}),right=assisted_drive(100,9,0.5,{0,0,-1});
    require(left.state.pose.yaw_rad>0.5 && right.state.pose.yaw_rad< -0.5,"full assistance listens promptly in either direction");
    const auto normal=assisted_drive(1,9,0.15,{0,0,1}),easy=assisted_drive(100,9,0.15,{0,0,1});
    require(easy.state.pose.steering_rad>normal.state.pose.steering_rad*1.5,"assistance improves steering actuator response");
    const auto fine=assisted_drive(50,9,1.5,{0,0.25,0.8},0.0025);
    const auto coarse=assisted_drive(50,9,1.5,{0,0.25,0.8});
    near(coarse.state.pose.x_m,fine.state.pose.x_m,0.03,"assistance converges across tick sizes");
    near(coarse.state.pose.y_m,fine.state.pose.y_m,0.03,"assisted lateral motion converges across tick sizes");
}

void forgiveness_lifecycle_is_transactional() {
    const auto& profile=fd::vehicle_profile("gokartcentralen-rsx2");
    const auto config=fd::configured_for(profile,fd::Config{});
    fd::Simulation simulation(fd::make_preset_track(),config,profile.car);
    rejects<std::invalid_argument>([&]{simulation.set_driving_assistance({true,100});},"autonomous assistance refused");
    simulation.set_human_driving(true);
    simulation.set_driver_controls({1,0,0});
    simulation.step(); // Between ordinary control decisions.
    const auto before=simulation.state();
    const auto revision=simulation.diagnostics().revision;
    simulation.set_driving_assistance({true,80});
    rejects<std::invalid_argument>([&]{simulation.set_driving_assistance({true,101});},"invalid pending value refused");
    require(simulation.requested_driving_assistance().level==80 && !simulation.driving_assistance().enabled,
            "refusal leaves the valid queued change intact");
    near(simulation.state().time_s,before.time_s,0,"queuing assistance does not advance time");
    near(simulation.state().x_m,before.x_m,0,"queuing assistance never snaps position");
    require(simulation.diagnostics().revision==revision,"the sample at T keeps its old revision");
    simulation.step();
    require(simulation.assistance_active() && simulation.diagnostics().revision==revision+1,"the next tick commits the override");
    require(simulation.events().size()==2 && simulation.events().front().time_s==before.time_s &&
            simulation.events().back().revision==revision+1,"both changes have the tick's time and revision");
    simulation.reset();
    require(simulation.assistance_active() && simulation.driver_controls().throttle==0,"reset retains assistance and releases pedals");
    simulation.set_human_driving(false);
    require(!simulation.assistance_active() && !simulation.requested_driving_assistance().enabled,"handing back disables assistance");
    simulation.step();
    require(!simulation.driving_assistance().enabled,"autonomy never receives assisted physics");
    simulation.set_human_driving(true);
    simulation.set_driving_assistance({true,100});
    simulation.set_driver_controls({0.7,0,0.4});
    steps(simulation,0.5);
    simulation.set_driving_assistance({false,100});
    const auto state=simulation.plant_state();
    require(state.pose.speed_mps>1,"switch-off is exercised on a moving, turning kart");
    const auto expected=fd::advance(profile.car,state,fd::human_command(simulation.driver_controls(),state,profile.car,config),config,config.fixed_dt_s);
    simulation.step();
    near(simulation.state().x_m,expected.pose.x_m,0,"switching off restores the ordinary integrator from the actual state");
    near(simulation.state().speed_mps,expected.pose.speed_mps,0,"switching off restores ordinary speed response");
}
} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"controls_are_validated", controls_are_validated},
        {"pedals_ask_for_the_road_grip", pedals_ask_for_the_road_grip},
        {"steering_narrows_with_speed", steering_narrows_with_speed},
        {"a_person_drives_without_a_speed_cap", a_person_drives_without_a_speed_cap},
        {"controls_take_effect_at_the_next_control_decision", controls_take_effect_at_the_next_control_decision},
        {"brake_stops_without_reversing", brake_stops_without_reversing},
        {"the_stick_turns_the_car_its_way", the_stick_turns_the_car_its_way},
        {"handing_back_restores_the_autonomous_driver", handing_back_restores_the_autonomous_driver},
        {"a_persons_drive_is_not_recorded", a_persons_drive_is_not_recorded},
        {"forgiveness_is_opt_in_and_validated", forgiveness_is_opt_in_and_validated},
        {"forgiveness_measurably_changes_driving", forgiveness_measurably_changes_driving},
        {"manual_forgiveness_controls_are_independent_and_effective", manual_forgiveness_controls_are_independent_and_effective},
        {"forgiveness_lifecycle_is_transactional", forgiveness_lifecycle_is_transactional},
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " human driving groups passed\n";
    return failures ? 1 : 0;
}

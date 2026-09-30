#include "fd/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void require(bool condition,const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual,double expected,double tolerance,const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected)>tolerance)
        throw std::runtime_error(message+": actual="+std::to_string(actual)+", expected="+std::to_string(expected));
}
template<class Function> void rejects(Function action,const std::string& message) {
    bool threw=false;
    try { action(); } catch (const std::exception&) { threw=true; }
    require(threw,message);
}

void straight_line() {
    fd::Config config;
    fd::State state;
    state.speed_mps=2;
    for (int i=0;i<400;++i) state=fd::integrate_bicycle(state,{3,0},config,0.005);
    near(state.x_m,10,1e-10,"constant-acceleration displacement");
    near(state.y_m,0,1e-12,"straight y");
    near(state.speed_mps,8,1e-10,"straight speed");
    near(state.yaw_rad,0,1e-12,"straight yaw");
}

void constant_radius() {
    fd::Config config;
    const double radius=20,speed=6,steer=std::atan(config.wheelbase_m/radius);
    fd::State state{0,0,0,speed,steer,0};
    for (int i=0;i<4000;++i) state=fd::integrate_bicycle(state,{0,steer},config,0.005);
    const double angle=speed*20/radius;
    near(state.x_m,radius*std::sin(angle),2e-9,"rear axle circle x");
    near(state.y_m,radius*(1-std::cos(angle)),2e-9,"rear axle circle y");
    near(state.yaw_rad,fd::wrap_angle(angle),2e-10,"CCW circle yaw");
    near(std::hypot(state.x_m,state.y_m-radius),radius,2e-9,"circle radius");
}

void actuator_bounds() {
    fd::Config c;
    fd::State state;
    for (int i=0;i<1000;++i) {
        const double prior=state.steering_rad;
        state=fd::integrate_bicycle(state,{-2,i<500?9.0:-9.0},c,c.fixed_dt_s);
        require(std::abs(state.steering_rad)<=c.max_steering_rad+1e-12,"steering angle bound");
        require(std::abs(state.steering_rad-prior)<=c.max_steering_rate_radps*c.fixed_dt_s+1e-12,"steering rate bound");
        near(state.speed_mps,0,0,"braking may not reverse vehicle");
    }
    const auto stopped=fd::integrate_bicycle({0,0,0,0.01,0,0},{-4,0},c,0.005);
    near(stopped.x_m,0.0000125,1e-15,"braking through zero integrates only to stop time");
    near(stopped.speed_mps,0,0,"within-tick stop ends at zero speed");
    near(stopped.time_s,0.005,0,"within-tick stop advances the full clock tick");
    const auto steering_while_stopped=fd::integrate_bicycle({0,0,0,0,0,0},{-4,0.3},c,0.005);
    near(steering_while_stopped.x_m,0,0,"stationary braking produces no displacement");
    near(steering_while_stopped.steering_rad,c.max_steering_rate_radps*0.005,1e-15,"steering actuator advances while stopped");
}

void invalid_inputs() {
    fd::Config c;
    c.grip_mu=std::numeric_limits<double>::quiet_NaN();
    rejects([&]{fd::Simulation simulation(c);},"NaN grip must fail");
    c={};c.control_dt_s=0.017;
    rejects([&]{fd::validate_config(c);},"noninteger control schedule must fail");
    c={};c.wheelbase_m=0;
    rejects([&]{fd::validate_config(c);},"zero wheelbase must fail");
    c={};c.lateral_grip_fraction=0.75;c.longitudinal_grip_fraction=0.65;
    rejects([&]{fd::validate_config(c);},"allocation without combined reserve must fail");
    auto track=fd::make_preset_track();
    track.points[5].s_m=track.points[4].s_m;
    rejects([&]{fd::validate_track(track);},"repeated arc length must fail");
    rejects([&]{fd::integrate_bicycle({}, {}, {},0);},"zero integration dt must fail");
    fd::State nonfinite;
    nonfinite.yaw_rad=std::numeric_limits<double>::quiet_NaN();
    rejects([&]{fd::integrate_bicycle(nonfinite, {}, {},0.005);},"nonfinite state must fail");
    const auto valid_track=fd::make_preset_track();
    auto valid_plan=fd::make_speed_plan(valid_track,{});
    nonfinite={};nonfinite.x_m=std::numeric_limits<double>::quiet_NaN();
    rejects([&]{fd::compute_control(valid_track,valid_plan,nonfinite,{});},"controller rejects nonfinite state");
    valid_plan.front().speed_mps=std::numeric_limits<double>::quiet_NaN();
    rejects([&]{fd::compute_control(valid_track,valid_plan,{},{});},"controller rejects nonfinite speed plan");
    valid_plan.front().speed_mps=1;
    valid_plan.front().acceleration_mps2=std::numeric_limits<double>::infinity();
    rejects([&]{fd::compute_control(valid_track,valid_plan,{},{});},"controller rejects nonfinite acceleration plan");
    valid_plan.pop_back();
    rejects([&]{fd::compute_control(valid_track,valid_plan,{},{});},"controller rejects mismatched plan size");
    c={};c.max_steering_rad=0.1;
    rejects([&]{fd::Simulation impossible_turn(c);},"track beyond steering capability must fail");
    fd::Simulation sim;
    const auto before=sim.state();
    rejects([&]{sim.set_grip(-1);},"invalid queued change must fail");
    near(sim.config().grip_mu,1,0,"failed change preserves config");
    near(sim.state().time_s,before.time_s,0,"failed change preserves simulation time");
    require(sim.events().empty(),"failed change must not create event");
}

void strict_config() {
    const auto file=std::filesystem::temp_directory_path()/"fd-core-strict-config-test.cfg";
    struct Cleanup { std::filesystem::path path; ~Cleanup(){std::error_code error;std::filesystem::remove(path,error);} } cleanup{file};
    const auto write=[&](const std::string& text){std::ofstream stream(file);stream<<text;};
    write("# comment\nschema_version=1\ngrip_mu = 0.45\nmax_speed_mps=20\n");
    near(fd::load_config(file).grip_mu,0.45,0,"strict config valid value");
    write("schema_version=1\ngrip_mu=1\ngrip_mu=0.5\n");
    rejects([&]{fd::load_config(file);},"duplicate keys must fail");
    write("schema_version=1\ngrip_muu=0.5\n");
    rejects([&]{fd::load_config(file);},"unknown keys must fail");
    write("schema_version=1\ngrip_mu=0.5 metres\n");
    rejects([&]{fd::load_config(file);},"trailing text must fail");
    write("schema_version=1\ngrip_mu=inf\n");
    rejects([&]{fd::load_config(file);},"nonfinite config must fail");
    write("grip_mu=0.5\n");
    rejects([&]{fd::load_config(file);},"missing schema version must fail");
    write("schema_version=2\ngrip_mu=0.5\n");
    rejects([&]{fd::load_config(file);},"unsupported schema version must fail");
}

void periodic_profile() {
    const auto track=fd::make_preset_track();
    for (const double grip:{0.2,0.45,1.0,1.5}) {
        fd::Config c;c.grip_mu=grip;
        const auto plan=fd::make_speed_plan(track,c);
        const double a=c.longitudinal_grip_fraction*grip*fd::gravity_mps2;
        for (std::size_t i=0;i<plan.size();++i) {
            const std::size_t j=(i+1)%plan.size();
            const double ds=j?track.points[j].s_m-track.points[i].s_m:track.length_m-track.points[i].s_m;
            const double energy=plan[j].speed_mps*plan[j].speed_mps-plan[i].speed_mps*plan[i].speed_mps;
            require(std::abs(energy)<=2*a*ds+1e-7,"periodic acceleration/braking edge constraint including seam");
            const double lateral=plan[i].speed_mps*plan[i].speed_mps*std::abs(track.points[i].curvature);
            require(lateral<=c.lateral_grip_fraction*grip*fd::gravity_mps2+1e-8,"lateral curvature cap");
            require(std::hypot(lateral,plan[i].acceleration_mps2)/(grip*fd::gravity_mps2)<=0.9+1e-8,"conservative combined planned demand");
            require(plan[i].limiting_index<plan.size(),"limiting point index is valid");
        }
    }
}

void braking_and_grip() {
    const auto track=fd::make_preset_track();
    fd::Config high;high.max_speed_mps=20;
    auto low=high;low.grip_mu=0.45;
    const auto a=fd::make_speed_plan(track,high),b=fd::make_speed_plan(track,low);
    const auto first_corner=static_cast<std::size_t>(std::find_if(track.points.begin(),track.points.end(),
                          [](const fd::PathPoint& p){return p.curvature>0.04;})-track.points.begin());
    require(first_corner<track.points.size(),"preset has a tight corner");
    require(b[first_corner].speed_mps<a[first_corner].speed_mps-2,"low grip reduces corner speed");
    near(b[first_corner].speed_mps/a[first_corner].speed_mps,std::sqrt(low.grip_mu/high.grip_mu),1e-9,"corner cap follows sqrt(mu)");
    const auto braking_start=[&](const auto& plan){
        for(std::size_t i=0;i<first_corner;++i)
            if(plan[i].acceleration_mps2 < -fd::plan_color_deadband_mps2) return track.points[i].s_m;
        throw std::runtime_error("no braking before tight corner");
    };
    const double high_start=braking_start(a),low_start=braking_start(b);
    require(high_start<track.points[first_corner].s_m-5,"braking begins ahead of corner");
    require(low_start<high_start-2,"lower grip shifts braking earlier in this speed-cap-binding scenario");
    require(std::any_of(a.begin(),a.end(),[](const auto& p){return p.speed_mps>=19.999;}),"high-grip speed cap binds");
    std::cout << "  braking stations high=" << high_start << " m low=" << low_start << " m; corner=" << track.points[first_corner].s_m << " m\n";
}

void complete_laps() {
    for (const double grip:{1.0,0.45,0.2}) {
        fd::Config c;c.grip_mu=grip;
        fd::Simulation simulation(c);
        double max_error=0,max_utilization=0;
        int invalid=0;
        while (simulation.diagnostics().laps<2 && simulation.state().time_s<180) {
            simulation.step();
            const auto& d=simulation.diagnostics();
            max_error=std::max(max_error,std::abs(d.cross_track_error_m));
            max_utilization=std::max(max_utilization,d.combined_grip_utilization);
            if (!d.plan_valid) ++invalid;
            require(d.within_track,"autonomous lap must stay within track margin");
        }
        require(simulation.diagnostics().laps==2,"complete two autonomous laps");
        require(max_error<1.5,"tracking error below 1.5 m");
        require(max_utilization<=1,"actual combined demand below full grip budget");
        require(invalid==0,"nominal lap remains inside stated validity checks");
        std::cout << "  grip=" << grip << " two laps=" << simulation.state().time_s
                  << " s error=" << max_error << " m utilization=" << max_utilization << '\n';
    }
}

void parameter_event_continuity() {
    fd::Simulation simulation;
    while(simulation.state().time_s<6) simulation.step();
    const auto before=simulation.state();
    const auto config=simulation.config();
    const auto expected=fd::integrate_bicycle(before,simulation.diagnostics().applied,config,config.fixed_dt_s);
    simulation.set_grip(0.45);
    near(simulation.state().x_m,before.x_m,0,"enqueue preserves pose");
    near(simulation.state().time_s,before.time_s,0,"enqueue preserves time");
    near(simulation.config().grip_mu,1,0,"enqueue defers mutation");
    simulation.step();
    const auto after=simulation.state();
    require(simulation.events().size()==1 && simulation.diagnostics().revision==2,"one committed revision/event");
    near(simulation.events()[0].time_s,before.time_s,1e-12,"event time is tick boundary");
    near(simulation.diagnostics().plan_generated_time_s,before.time_s,1e-12,"plan generation shares event time");
    near(after.time_s,before.time_s+config.fixed_dt_s,1e-12,"tick advances normally");
    require(std::hypot(after.x_m-before.x_m,after.y_m-before.y_m)<before.speed_mps*config.fixed_dt_s+0.001,"no teleport on replan");
    require(std::hypot(after.x_m-expected.x_m,after.y_m-expected.y_m)<0.001,"replan retains integrated pose continuity");
    require(!simulation.diagnostics().plan_valid,"sudden grip loss exposes invalid transient");
    bool observed_over_grip=false;
    while(simulation.state().time_s<60) {
        simulation.step();
        if(simulation.diagnostics().combined_grip_utilization>1.0+1e-6) {
            observed_over_grip=true;
            require(!simulation.diagnostics().plan_valid,"infeasible combined demand must be visibly invalid");
        }
        require(simulation.diagnostics().within_track,"grip-change scenario retains track margin");
    }
    require(observed_over_grip,"event scenario exercises an explicit kinematic validity violation");
    require(simulation.diagnostics().plan_valid,"grip-change transient recovers");
    require(simulation.diagnostics().laps>=1,"grip-change run still completes a lap");
    simulation.reset();
    near(simulation.state().time_s,0,0,"reset clears time");
    require(simulation.events().empty() && simulation.diagnostics().revision==1,"reset clears run history");
    near(simulation.config().grip_mu,0.45,0,"reset preserves selected config");
}

void deterministic_schedule() {
    fd::Simulation a,b;
    for(int i=0;i<8000;++i) {
        if(i==733) {a.set_grip(0.6);b.set_grip(0.6);}
        a.step();b.step();
        near(a.state().x_m,b.state().x_m,0,"deterministic x");
        near(a.state().y_m,b.state().y_m,0,"deterministic y");
        near(a.state().speed_mps,b.state().speed_mps,0,"deterministic speed");
        require(a.diagnostics().revision==b.diagnostics().revision,"deterministic revisions");
    }
    near(a.state().time_s,40,0,"integer tick clock");
}
}

int main() {
    const std::vector<std::pair<std::string,std::function<void()>>> tests{
        {"straight_line",straight_line},{"constant_radius",constant_radius},{"actuator_bounds",actuator_bounds},
        {"invalid_inputs",invalid_inputs},{"strict_config",strict_config},{"periodic_profile",periodic_profile},
        {"braking_and_grip",braking_and_grip},{"complete_laps",complete_laps},
        {"parameter_event_continuity",parameter_event_continuity},{"deterministic_schedule",deterministic_schedule}
    };
    int failures=0;
    for(const auto& [name,test]:tests) {
        try {test();std::cout << "PASS " << name << '\n';}
        catch(const std::exception& error) {++failures;std::cerr << "FAIL " << name << ": " << error.what() << '\n';}
    }
    std::cout << tests.size()-static_cast<std::size_t>(failures) << '/' << tests.size() << " checks passed\n";
    return failures?1:0;
}

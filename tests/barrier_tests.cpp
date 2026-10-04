#include "fd/barriers.hpp"
#include "fd/simulation.hpp"
#include "fd/vehicle_profiles.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
int checks=0;
void check(bool ok,const char* why) { ++checks; if (!ok) throw std::runtime_error(why); }
template<class F> void rejects(F f) { bool rejected=false; try { f(); } catch (const std::invalid_argument&) { rejected=true; } check(rejected,"bad input accepted"); }
double energy(const fd::PlantState& s,const fd::FourWheelCar& car) {
    const double vy=s.lateral_velocity_mps+car.cg_to_rear_m*s.yaw_rate_radps;
    return 0.5*car.mass_kg*(s.pose.speed_mps*s.pose.speed_mps+vy*vy)+0.5*car.yaw_inertia_kgm2*s.yaw_rate_radps*s.yaw_rate_radps;
}
}
int main() {
    try {
        const auto profile=fd::vehicle_profile("gokartcentralen-realistic");
        const auto car=profile.car; const fd::VehicleModel model=car; const auto config=fd::configured_for(profile,{});
        const auto make=[&](double speed,double yaw=0.0) { return fd::plant_state_from({0,0,yaw,speed,0,0},model,config); };
        const fd::KartEnvironment far({{{100,-20},{100,20}}});
        const auto a=fd::advance(model,make(8),{0,0.1},config,0.005);
        const auto b=fd::advance_with_barriers(model,make(8),{0,0.1},config,0.005,{},far);
        check(a.pose.x_m==b.pose.x_m && a.pose.y_m==b.pose.y_m && a.pose.yaw_rad==b.pose.yaw_rad &&
              a.wheel_speeds_radps==b.wheel_speeds_radps,"free motion changed");
        // Walls must not sneak in a grip/trajectory override. Away from contact,
        // every state field is the ordinary four-wheel plant, even at 80 km/h.
        for (double speed:{6.0,16.0,80/3.6}) for (double sign:{-1.0,1.0}) {
            auto ordinary=make(speed),bounded=ordinary;
            for (int i=0;i<200;++i) {
                const fd::Command command{0,sign*config.max_steering_rad};
                ordinary=fd::advance(model,ordinary,command,config,0.005);
                bounded=fd::advance_with_barriers(model,bounded,command,config,0.005,{},far);
                check(ordinary.pose.x_m==bounded.pose.x_m && ordinary.pose.y_m==bounded.pose.y_m &&
                      ordinary.pose.yaw_rad==bounded.pose.yaw_rad && ordinary.pose.speed_mps==bounded.pose.speed_mps &&
                      ordinary.lateral_velocity_mps==bounded.lateral_velocity_mps && ordinary.yaw_rate_radps==bounded.yaw_rate_radps &&
                      ordinary.wheel_speeds_radps==bounded.wheel_speeds_radps && ordinary.wheel_loads_n==bounded.wheel_loads_n,
                      "solid walls changed free-driving physics");
            }
        }
        // Sustained full throttle cannot push the front bumper through a thin wall.
        for (double speed:{8.0,16.0}) {
            const fd::KartEnvironment wall({{{4,-20},{4,20}}});
            auto s=make(speed);
            const double before=energy(s,car);
            for (int i=0;i<200;++i) {
                s=fd::advance_with_barriers(model,s,{i<45?0.0:8.0,0},config,0.005,{},wall);
                check(fd::barrier_clearance(wall,s.pose,config)>=-1e-6,"front bumper penetrated wall");
                check(s.pose.x_m<4-config.wheelbase_m,"kart crossed wall");
            }
            check(s.pose.speed_mps<0.2,"kart keeps driving into wall");
            check(energy(s,car)<before*0.1,"impact did not dissipate energy");
            check(std::abs(s.pose.time_s-1)<1e-10,"contact changed simulation time");
        }
        // Glancing contact keeps progress along the wall, on both sides.
        for (double sign:{-1.0,1.0}) {
            const fd::KartEnvironment wall({{{4,-20},{4,20}}});
            auto s=make(12,sign*0.55);
            for (int i=0;i<150;++i) {
                s=fd::advance_with_barriers(model,s,{0,0},config,0.005,{},wall);
                check(fd::barrier_clearance(wall,s.pose,config)>=-1e-6,"glancing collision penetrates");
            }
            check(sign*s.pose.y_m>1.2,"glancing contact stops tangential motion");
        }
        // Thin, short segment/endpoints and a coarser outer tick cannot tunnel.
        const fd::KartEnvironment tip({{{4,0},{4,0.1}}});
        auto fast=make(35);
        for (int i=0;i<15;++i) {
            fast=fd::advance_with_barriers(model,fast,{0,0},config,0.02,{},tip);
            check(fast.pose.x_m<4,"high-speed endpoint tunnelling");
            check(fd::barrier_clearance(tip,fast.pose,config)>=-1e-6,"endpoint overlap");
        }
        // A corner does not launch the kart or let it escape either wall.
        const fd::KartEnvironment corner({{{4,-10},{4,4}},{{-10,4},{4,4}}});
        auto wedged=make(15,0.7853981633974483);
        for (int i=0;i<200;++i) {
            wedged=fd::advance_with_barriers(model,wedged,{8,0.1},config,0.005,{},corner);
            check(fd::barrier_clearance(corner,wedged.pose,config)>=-1e-6,"corner overlap");
            check(wedged.pose.x_m<4 && wedged.pose.y_m<4,"corner escape");
        }
        rejects([] { fd::KartEnvironment w({{{0,0},{0,0}}}); });
        rejects([] { fd::KartEnvironment w({{{0,0},{std::numeric_limits<double>::infinity(),0}}}); });
        rejects([&] { fd::advance_with_barriers(model,make(1),{},config,0.2,{},far); });
        // Scenario lifecycle and autonomous isolation.
        const auto track=fd::make_preset_track();
        fd::Simulation sim(track,config,model),plain(track,config,model);
        sim.set_kart_environment(fd::KartEnvironment(fd::track_barriers(track)));
        sim.step(); plain.step();
        check(sim.state().x_m==plain.state().x_m && sim.state().y_m==plain.state().y_m,"walls changed autonomous run");
        rejects([&] { sim.set_kart_environment(far); });
        sim.reset(); check(sim.kart_environment() && !sim.kart_environment()->walls().empty(),"reset lost world");
        sim.set_human_driving(true); sim.set_driver_controls({0.5,0,0.2}); sim.step();
        check(!sim.assistance_active(),"human kart unexpectedly enables assistance");
    } catch (const std::exception& e) { std::cerr<<"barriers: "<<e.what()<<" after "<<checks<<" checks\n"; return 1; }
    std::cout<<"barriers: "<<checks<<" checks passed\n";
}

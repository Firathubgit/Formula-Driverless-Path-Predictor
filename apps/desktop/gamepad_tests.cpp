#include "gamepad.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

// XInput's readings as a person's controls (decision 0037), checked without a controller.
namespace {
int checks = 0;
void check(bool value, const std::string& message) { ++checks; if (!value) throw std::runtime_error(message); }
bool near(double actual, double expected) { return std::abs(actual-expected) < 1e-12; }
}

int main() {
    try {
        const auto rest = pad_from_xinput(0, 0, 0, 0);
        check(rest.connected && rest.controls.throttle == 0 && rest.controls.brake == 0 && rest.controls.steering == 0 && !rest.menu,
              "a controller at rest asks for nothing");
        check(pad_from_xinput(7849, 30, 30, 0).controls.steering == 0 && pad_from_xinput(-7849, 0, 0, 0).controls.steering == 0 &&
              pad_from_xinput(0, 30, 30, 0).controls.throttle == 0 && pad_from_xinput(0, 30, 30, 0).controls.brake == 0,
              "nothing within XInput's dead zones reaches the car");
        check(near(pad_from_xinput(-32768, 0, 0, 0).controls.steering, 1) && near(pad_from_xinput(32767, 0, 0, 0).controls.steering, -1),
              "the stick pushed fully left steers fully left, and right, right");
        check(near(pad_from_xinput(7849+(32767-7849)/2, 0, 0, 0).controls.steering, -static_cast<double>((32767-7849)/2)/(32767-7849)),
              "halfway past the dead zone is about half the stick");
        check(near(pad_from_xinput(0, 0, 255, 0).controls.throttle, 1) && near(pad_from_xinput(0, 255, 0, 0).controls.brake, 1),
              "the right trigger floors the accelerator and the left the brake");
        check(near(pad_from_xinput(0, 0, 30+225/3, 0).controls.throttle, (225/3)/225.0), "a trigger rises from its threshold");
        check(pad_from_xinput(0, 0, 0, 0x0010).menu && !pad_from_xinput(0, 0, 0, 0x0020).menu, "the Menu button is read, and only it");
        // Everything a controller can report is a control the car accepts.
        for (const int x : {-32768, -20000, -7850, 0, 7850, 20000, 32767})
            for (const int trigger : {0, 31, 128, 255})
                fd::validate_driver_controls(pad_from_xinput(x, trigger, 255-trigger, 0).controls);
        ++checks;
        check(rumble_for(10,1,false).heavy==0 && rumble_for(10,1,false).light==0,"pause silences both motors");
        check(rumble_for(0,1,true).heavy==0 && rumble_for(0,1,true).light==0,"stationary kart is silent even with stale slip");
        const auto cruising=rumble_for(16.7,0,true);
        check(near(cruising.heavy,0.063) && near(cruising.light,0.0105),"coasting has light road hum without tire slip");
        check(rumble_for(8,0,true).heavy<cruising.heavy,"road hum grows with speed");
        const auto scrubbing=rumble_for(12,1,true);
        check(scrubbing.heavy>cruising.heavy && scrubbing.light>0.1925,"overloaded tires add the softened warning vibration");
        fd::Demand tire;
        tire.tire_slip_modeled=true;
        tire.wheel_combined_slip={0.4,0.5,0.4,0.5};
        tire.lateral_acceleration_mps2=15;
        check(tire_scrub_for(tire)==0,"lateral acceleration alone does not invent tire slip");
        tire.wheel_combined_slip[0]=1;
        const double at_peak=tire_scrub_for(tire);
        check(at_peak>0 && at_peak<0.3,"warning starts gently near the grip peak");
        tire.wheel_combined_slip[0]=1.5;
        check(tire_scrub_for(tire)==1,"combined braking or cornering slip reaches the warning bound");
        tire.wheel_combined_slip={}; tire.front_peak_slip_angle_rad=0.1; tire.front_slip_angle_rad=0.15;
        const double left=tire_scrub_for(tire); tire.front_slip_angle_rad=-0.15;
        check(near(left,tire_scrub_for(tire)) && left>0.99,"lateral tire slip warns in either steering direction");
        tire.tire_slip_modeled=false;
        check(tire_scrub_for(tire)==0,"unmodeled slip is not fabricated");
        check(rumble_for(std::nan(""),1,true).heavy==0 && rumble_for(5,std::nan(""),true).light==0,"invalid readings are silent");
        for (double speed:{0.0,5.0,16.7,40.0}) for (double scrub:{-1.0,0.0,0.5,1.0,2.0}) {
            const auto motors=rumble_for(speed,scrub,true);
            check(motors.heavy>=0 && motors.heavy<=0.238+1e-12 && motors.light>=0 && motors.light<=0.203+1e-12,"feedback remains bounded and soft");
        }
        // No controller in the slots is no controller, and asks for nothing.
        Gamepad gamepad;
        const auto read = gamepad.poll();
        check(read.connected || (read.controls.throttle == 0 && read.controls.brake == 0 && read.controls.steering == 0),
              "a slot with nothing in it asks for nothing");
    } catch (const std::exception& error) {
        std::cerr << "gamepad: " << error.what() << " after " << checks << " checks\n";
        return 1;
    }
    std::cout << "gamepad: " << checks << " checks passed\n";
    return 0;
}

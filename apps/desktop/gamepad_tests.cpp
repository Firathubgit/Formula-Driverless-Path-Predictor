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

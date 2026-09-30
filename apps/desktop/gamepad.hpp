#pragma once
#include "fd/human_driving.hpp"

#include <chrono>

// The first Xbox controller Windows sees, read through XInput (decision 0037), and what it asks of a person's car: the
// left stick steers, the right trigger is the accelerator and the left the brake.
struct PadState {
    bool connected{};
    fd::DriverControls controls;
    bool menu{};   // the Menu button, held
};

// XInput's raw readings as a person's controls: past XInput's recommended dead zones (7849 of the stick's 32767, 30 of a
// trigger's 255), each rescaled to reach its whole travel at the end of its own. The stick pushed left steers left. Pure,
// so it is checked without a controller.
PadState pad_from_xinput(int thumb_left_x, int left_trigger, int right_trigger, unsigned buttons);

class Gamepad {
public:
    // Reads the controller now. Asking a slot with nothing in it is slow, so without a controller the four slots are
    // searched at most every two seconds. Off Windows nothing is ever connected.
    PadState poll();
private:
    int slot_{-1};
    std::chrono::steady_clock::time_point next_search_{};
};

#include "gamepad.hpp"

#include <algorithm>
#include <cmath>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <Xinput.h>
#endif

namespace {
constexpr int stick_dead_zone = 7849;      // XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE
constexpr int stick_full = 32767;
constexpr int trigger_threshold = 30;      // XINPUT_GAMEPAD_TRIGGER_THRESHOLD
constexpr int trigger_full = 255;
constexpr unsigned menu_button = 0x0010;   // XINPUT_GAMEPAD_START, the Menu button on an Xbox One controller

// The share of travel past a dead zone, keeping the side it is on.
double past(int value, int dead, int full) {
    const int magnitude = std::min(std::abs(value), full);
    if (magnitude <= dead) return 0;
    return std::copysign(static_cast<double>(magnitude-dead)/(full-dead), static_cast<double>(value));
}
} // namespace

PadState pad_from_xinput(int thumb_left_x, int left_trigger, int right_trigger, unsigned buttons) {
    PadState pad;
    pad.connected = true;
    // XInput's stick runs from -32768, left, to 32767, right; a person's steering is positive to the left.
    pad.controls.steering = -past(thumb_left_x, stick_dead_zone, stick_full);
    pad.controls.throttle = past(std::clamp(right_trigger, 0, trigger_full), trigger_threshold, trigger_full);
    pad.controls.brake = past(std::clamp(left_trigger, 0, trigger_full), trigger_threshold, trigger_full);
    pad.menu = (buttons & menu_button) != 0;
    return pad;
}

double tire_scrub_for(const fd::Demand& demand) {
    if (!demand.tire_slip_modeled) return 0;
    double use=0;
    for (double value:demand.wheel_combined_slip) {
        if (!std::isfinite(value)) return 0;
        use=std::max(use,value);
    }
    if (demand.front_peak_slip_angle_rad>0)
        use=std::max(use,std::abs(demand.front_slip_angle_rad)/demand.front_peak_slip_angle_rad);
    if (demand.rear_peak_slip_angle_rad>0)
        use=std::max(use,std::abs(demand.rear_slip_angle_rad)/demand.rear_peak_slip_angle_rad);
    if (!std::isfinite(use)) return 0;
    const double t=std::clamp((use-0.85)/0.65,0.0,1.0);
    return t*t*(3-2*t);
}

Rumble rumble_for(double speed_mps, double tire_scrub, bool running) {
    if (!running || !std::isfinite(speed_mps) || !std::isfinite(tire_scrub) || speed_mps<=0.05) return {};
    // Road hum remains when both pedals are released; overload feedback follows
    // actual combined tire slip, including braking/acceleration, not lateral g alone.
    const double pace = std::clamp(speed_mps/16.7, 0.0, 1.0);
    const double scrub = std::clamp(tire_scrub, 0.0, 1.0);
    // Keep the softened overload effect; give ordinary coasting a light road hum.
    constexpr double strength = 0.35;
    return {strength*(0.18*pace+0.5*scrub), strength*(0.03*pace+0.55*scrub)};
}

void Gamepad::rumble(const Rumble& motors) {
#ifdef _WIN32
    if (slot_ < 0) return;
    XINPUT_VIBRATION vibration{};
    vibration.wLeftMotorSpeed = static_cast<WORD>(std::clamp(motors.heavy, 0.0, 1.0)*65535);
    vibration.wRightMotorSpeed = static_cast<WORD>(std::clamp(motors.light, 0.0, 1.0)*65535);
    XInputSetState(static_cast<DWORD>(slot_), &vibration);
#else
    (void)motors;
#endif
}

Gamepad::~Gamepad() { rumble({}); }

PadState Gamepad::poll() {
#ifdef _WIN32
    const auto now = std::chrono::steady_clock::now();
    if (slot_ < 0 && now < next_search_) return {};
    const int first = slot_ >= 0 ? slot_ : 0, last = slot_ >= 0 ? slot_ : XUSER_MAX_COUNT-1;
    for (int slot = first; slot <= last; ++slot) {
        XINPUT_STATE state{};
        if (XInputGetState(static_cast<DWORD>(slot), &state) != ERROR_SUCCESS) continue;
        slot_ = slot;
        const auto& pad = state.Gamepad;
        return pad_from_xinput(pad.sThumbLX, pad.bLeftTrigger, pad.bRightTrigger, pad.wButtons);
    }
    slot_ = -1;
    next_search_ = now+std::chrono::seconds(2);
#endif
    return {};
}

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

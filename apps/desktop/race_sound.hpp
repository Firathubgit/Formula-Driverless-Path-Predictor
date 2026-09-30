#pragma once
#include <array>
#include <cstdint>
#include <span>
#include <memory>
#include "sample_loop.hpp"

// Presentation only: this synthesizer reads the drive and never supplies a vehicle command.
// Recorded Formula V10 voice; RPM and seven gears are virtual, including on the electric kart.
namespace race_sound {
struct Frame {
    bool eligible{}, running{};
    double time{}, speed{}, throttle{}, brake{}, slip{}, top_speed{20};
    int laps{};
    double previous_lap{}, best_lap{};
};
struct Mix { bool enabled{true}, engine{true}, cues{true}; double volume{0.45}; };
enum class Cue { tap, start, pause, lap, best };
class Synth {
public:
    explicit Synth(int sample_rate = 48000, std::shared_ptr<const Bank> bank = {});
    void mix(Mix value);
    void follow(Frame value);
    void cue(Cue value);
    void render(std::span<float> mono);
    int gear() const { return gear_; }
    double rpm() const { return rpm_; }
    double target_rpm() const { return target_rpm_; }
    int upshifts() const { return upshifts_; }
    int downshifts() const { return downshifts_; }
    int lap_chimes() const { return lap_chimes_; }
    int best_chimes() const { return best_chimes_; }
    double scrub() const { return target_scrub_; }
private:
    struct Voice { double age{10}, frequency{}, duration{}, delay{}, gain{}; };
    void note(double hz, double duration, double delay, double gain);
    double noise();
    int sample_rate_, gear_{1}, upshifts_{}, downshifts_{}, lap_chimes_{}, best_chimes_{};
    bool seen_{};
    Frame frame_;
    Mix mix_;
    std::array<Voice,8> voices_{};
    double cooldown_{}, target_rpm_{4200}, rpm_{4200}, load_{}, gain_{}, engine_gain_{}, cue_gain_{};
    double clock_{}, last_tap_{-1}, shift_cut_{}, blip_{};
    double target_scrub_{}, scrub_{}, air_{}, rumble_{}, dc_in_{}, dc_out_{}, duck_{1};
    double exhaust_low_{}, exhaust_low2_{}, limiter_{1}, shift_gain_{1};
    std::shared_ptr<const Bank> bank_;
    SamplePlayer engine_player_, tire_player_;
    std::uint32_t random_{0x91ab27cd};
};
}

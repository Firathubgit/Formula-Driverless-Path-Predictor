#pragma once
#include <span>
#include <vector>

namespace race_sound {
// Immutable, prepared before streaming. Filtered levels prevent high playback rates
// from folding the recording's upper frequencies back into the audible band.
class SampleLoop {
public:
    SampleLoop(std::vector<float> pcm, int sample_rate, double rms = 0.28);
    float read(double phase, double rate) const;
    int sample_rate() const { return sample_rate_; }
    std::size_t size() const { return levels_.front().size(); }
private:
    std::vector<std::vector<float>> levels_;
    int sample_rate_;
};

// Variable-rate interpolation/continuous loop cursor adapted from ExhaustNote's
// SamplePlayer, MIT (c) 2026 VIFEX. Full notice: assets/audio-licenses/ExhaustNote-MIT.txt.
// Changes: owning immutable float bank, double cursor, anti-alias levels and safe wrapping.
class SamplePlayer {
public:
    float next(const SampleLoop& loop, double rate);
private:
    double phase_{};
};
struct Bank { SampleLoop engine, tires; };
}

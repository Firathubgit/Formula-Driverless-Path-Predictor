#pragma once
#include "sample_loop.hpp"
#include <memory>
#include <QByteArray>

namespace race_sound {
SampleLoop decode_wave(const QByteArray& bytes, double rms=0.28);
// The pinned source WAVs are embedded at build time; there is no run-time download.
std::shared_ptr<const Bank> recorded_bank();
}

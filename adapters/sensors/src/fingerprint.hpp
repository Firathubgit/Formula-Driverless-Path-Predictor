#pragma once

#include "fd/fingerprint.hpp"

#include <cstdint>

namespace fd::sensors_detail {

// A stream of draws of its own for each channel or sensor of a seed: two odd multipliers keep neighbouring seeds and
// channels far apart in the sequence.
inline std::uint64_t stream_seed(std::uint64_t seed, std::uint64_t channel) {
    return seed*6364136223846793005ULL+channel*1442695040888963407ULL+1;
}

}  // namespace fd::sensors_detail

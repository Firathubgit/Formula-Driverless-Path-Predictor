#pragma once
#include "fd/kart_coach.hpp"

#include <vector>

// The poster's coloured line, bundled with the desktop (decision 0039): display data and coaching, never passed to a driver,
// the plant or the judge. Aborts if the bundled file is missing or malformed.
const std::vector<fd::CoachSample>& kart_guide_line();

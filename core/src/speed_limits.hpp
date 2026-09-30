#pragma once
#include "fd/performance_envelope.hpp"

// Shared by the speed plans made from the performance envelope: the reference plan (decision 0018) and the speed
// profiles along a local planner's paths (decision 0023).
namespace fd::detail {

// The fastest speed from rest up to the cap at which this curvature stays within the share of the envelope's lateral
// limit, or infinity if the cap is.
double envelope_curvature_speed(const PerformanceEnvelope& envelope, double share, double curvature, double cap);

} // namespace fd::detail

#pragma once
#include "fd/core.hpp"

namespace fd::detail {
// Internal rollout path: compute_control has already validated the immutable track plan and intent.
// The changing vehicle state is still validated on every call. No caller-owned data is cached.
ControlResult compute_control_validated(const Track&, const std::vector<PlanPoint>&, const State&, const Config&,
                                        ControlIntent, const SteeringTable*, const PerformanceEnvelope*);
}

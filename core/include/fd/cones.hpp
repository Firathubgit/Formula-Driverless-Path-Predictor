#pragma once
#include "fd/core.hpp"

#include <vector>

namespace fd {

// The course's cones (TrackWayFastPlan Phase 7.2, decision 0030): the corridor's boundaries as a Formula Student course
// lays them out, blue on the left of the direction of travel, yellow on the right, and a big orange cone either side of
// the start line. They are ground truth of the scenario, like a stated blocked region: nothing here detects anything.
// A perception sensor observes them (adapters/sensors); keeping them from the planner is Phase 7.4's boundary.
enum class ConeColour { blue, yellow, orange, big_orange };
const char* cone_colour_name(ConeColour colour);

struct Cone {
    Vec2 position;
    ConeColour colour{ConeColour::blue};
};

// How densely each boundary is coned. A cone is placed at the start line, then again whenever the boundary has run
// the straight spacing since the last one, or turned through the turn angle, whichever comes first, but never closer
// than the minimum spacing; so straights carry a cone every five metres, as the Formula Student handbook's maximum,
// and corners more.
struct ConeLayoutOptions {
    double straight_spacing_m{5.0};
    double min_spacing_m{1.5};
    double turn_angle_rad{0.1};
};
void validate_cone_layout_options(const ConeLayoutOptions& options);

// The cones along a validated track's corridor: each boundary is the reference offset along its normal by the
// corridor's edge distance (half the width for a centreline), and the big orange pair stands on the boundaries at the
// first sample, where laps are counted. Cones come left boundary first, in the direction of travel, then right.
std::vector<Cone> make_cone_layout(const Track& track, const ConeLayoutOptions& options = {});

}  // namespace fd

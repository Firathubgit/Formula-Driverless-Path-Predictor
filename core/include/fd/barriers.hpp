#pragma once
#include "fd/vehicle.hpp"
#include <vector>

namespace fd {
struct BarrierSegment { Vec2 a, b; };
// Planar, immovable walls. The kart bumper is a capsule about its rear-to-front
// centreline, not a point. Dimensions and rubber contact friction are chosen.
class KartEnvironment {
public:
    explicit KartEnvironment(std::vector<BarrierSegment> walls);
    const std::vector<BarrierSegment>& walls() const { return walls_; }
    static constexpr double bumper_radius_m = 0.59;
    static constexpr double barrier_face_m = 0.05;
private:
    std::vector<BarrierSegment> walls_;
};
// Same centred normals and edge distances as the hall's rendered barrier faces.
std::vector<BarrierSegment> track_barriers(const Track& track);
// Signed clearance of the full bumper, negative on penetration.
double barrier_clearance(const KartEnvironment&, const State&, const Config&);
} // namespace fd

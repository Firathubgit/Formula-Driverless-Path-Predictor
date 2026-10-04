#pragma once
#include "fd/core.hpp"
#include "fd/barriers.hpp"
#include "fd/kart_coach.hpp"
#include "fd/track_relief.hpp"
#include "scene_vertex.hpp"

#include <vector>

// The kart hall as it is drawn (decision 0051): Gokartcentralen Göteborg's room round the traced track, after the user's own
// onboard video of a lap there. Dimensions are chosen, not surveyed. The same layout
// supplies planar walls to the human kart's vehicle seam (decision 0052).

// A bend of the kart track along the lap: the stations it spans on the track the kart drives and which way it turns, +1 left,
// -1 right. From the poster line's own corners, found by the coach (decision 0049).
struct CornerSpan { double from_s_m{}, to_s_m{}; int direction{}; };

// Each of the coach's corners on the track the kart drives, with `margin_m` either side for its kerb.
std::vector<CornerSpan> kart_corner_spans(const fd::Track& track, const fd::KartCoach& coach, double margin_m = 2.0);

// Where the hall stands and where its lamps hang, in the scene's coordinates: x is the map's x, z the map's -y, y up. The
// floor's shader reflects the same lamps the lamp geometry draws, so both read this.
struct KartHallLayout {
    double min_x{}, max_x{}, min_z{}, max_z{};   // the four walls
    double eave_m{}, ridge_m{};                  // the roof at the side walls and at its ridge, which runs along z
    double lamp_height_m{};
    double lamp_x0{}, lamp_z0{};                 // the first lamp
    double lamp_dx{}, lamp_dz{};                 // between rows across the hall, and between lamps along a row
    int lamp_columns{}, lamp_rows{};             // how many across x, how many along z
    double roof_at(double x) const;              // the roof's height above a point
};
KartHallLayout kart_hall_layout(const fd::Track& track);
fd::KartEnvironment kart_environment(const fd::Track& track, const std::vector<CornerSpan>& corners);

enum class KartHallPart {
    barriers = 14,  // the twin-tube barriers along both edges, their rims and braces, and the bank's retaining wall
    floor = 15,     // the hall's floor outside the road, the islands, the kerbs, the painted lines and the start grid
    shell = 16,     // walls, roof and rafters, the gallery, the footbridge, the pillars, the fence, the timing board
    lamps = 17,     // the lamps themselves
    glow = 18,      // the haze round each lamp, blended
    wear = 19       // the pale scuffing on the banked bend's surface, blended
};

// Whether a part lies or stands on the ground and so rises with the banked bend (decision 0050); the shell and the lamps do not.
bool kart_hall_part_follows_ground(KartHallPart part);

// The triangles of one part, appended to `out`. `corners` are the track's bends (kart_corner_spans); `relief` is the banked
// bend, or null where the ground is flat, and what stands on the ground is returned already raised by it. Gokartcentralen's
// own furniture (gallery, footbridge, fence, pillars, the hairpin's coloured blocks) is drawn only on its 400 m track with its
// ten corners; any other track gets the plain room. A part is built once for a track and kept.
void build_kart_hall(KartHallPart part, const fd::Track& track, const std::vector<CornerSpan>& corners,
                     const fd::TrackRelief* relief, std::vector<SceneVertex>& out);

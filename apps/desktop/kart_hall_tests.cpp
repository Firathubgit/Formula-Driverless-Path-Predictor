#include "kart_hall.hpp"

#include "fd/track_conditioning.hpp"
#include "fd/vehicle_profiles.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

// The kart hall as it is drawn (decision 0051): what stands where, checked against the track it stands round. The hall is
// drawing only, so what is checked is that the drawing never lies about the road: nothing of it stands in the corridor, the
// barriers stand on the corridor's edges, the lamps the floor mirrors are the lamps that are drawn, and the ground rises
// with the banked bend and nowhere else.
namespace {
int checks = 0;
void check(bool value, const std::string& message) {
    ++checks;
    if (!value) throw std::runtime_error(message);
}

const std::filesystem::path tracks_directory{FD_KART_DIR};

fd::Track kart_track() {
    fd::ConditioningOptions options;
    options.smoothing_rms_m = 0.02;  // as the desktop conditions it
    return fd::condition_track(fd::load_centreline(tracks_directory / "gokartcentralen-goteborg.csv"), 6.0, "Gokartcentralen Göteborg", options).track;
}

std::vector<fd::CoachSample> poster_line() {
    std::ifstream file(tracks_directory / "gokartcentralen-goteborg-guide.csv");
    if (!file.good()) throw std::runtime_error("the poster's coloured line is missing");
    std::vector<fd::CoachSample> line;
    for (std::string text; std::getline(file, text);) {
        if (text.empty() || text[0] == '#' || text.rfind("x_m,", 0) == 0) continue;
        const auto first = text.find(','), second = text.find(',', first + 1);
        const auto word = text.substr(second + 1);
        line.push_back({{std::stod(text.substr(0, first)), std::stod(text.substr(first + 1, second - first - 1))},
                        word.rfind("brake", 0) == 0 ? fd::Advice::Brake : word.rfind("coast", 0) == 0 ? fd::Advice::Coast : fd::Advice::Accelerate});
    }
    return line;
}

std::vector<SceneVertex> part(KartHallPart which, const fd::Track& track, const std::vector<CornerSpan>& corners, const fd::TrackRelief* relief) {
    std::vector<SceneVertex> vertices;
    build_kart_hall(which, track, corners, relief, vertices);
    return vertices;
}

// A vertex's place on the map and how far it lies from the middle of the road.
fd::Projection on_track(const fd::Track& track, const SceneVertex& v) { return fd::project(track, {v.x, -v.z}); }

// Stored colours are linear.
bool red(const SceneVertex& v) { return v.r > 0.25f && v.g < 0.08f && v.b < 0.08f; }
bool green(const SceneVertex& v) { return v.g > 0.12f && v.g > 2.2f * v.r && v.g > 1.3f * v.b; }
bool blue(const SceneVertex& v) { return v.b > 0.3f && v.b > 2 * v.r && v.b > 1.5f * v.g; }
bool pale(const SceneVertex& v) { return v.r > 0.3f && std::abs(v.r - v.g) < 0.08f && std::abs(v.g - v.b) < 0.08f; }
}

int main() {
    try {
        const auto track = kart_track();
        const fd::KartCoach coach(poster_line(), 1.045);
        const auto corners = kart_corner_spans(track, coach);
        const auto relief = fd::gokartcentralen_relief(track);
        const double half = track.width_m / 2, length = track.length_m;

        // The bends the hall is dressed by: the poster line's ten, in order, on the track the kart drives.
        check(corners.size() == 10, "ten bends");
        std::string turns;
        for (std::size_t k = 0; k < corners.size(); ++k) {
            turns += corners[k].direction > 0 ? 'L' : 'R';
            const double span = std::fmod(corners[k].to_s_m - corners[k].from_s_m + length, length);
            check(span > 6 && span < 60, "a bend spans a stretch of the lap, margins included");
            if (k > 0) check(corners[k].from_s_m > corners[k - 1].from_s_m, "bends in lap order");
        }
        check(turns == "RLRRLLRRLR", "the bends turn as the track's do: " + turns);

        // The room stands round the whole corridor, its lamps inside it and under its roof.
        const auto hall = kart_hall_layout(track);
        for (const auto& point : track.points)
            check(point.x_m - half > hall.min_x + 1 && point.x_m + half < hall.max_x - 1 && -point.y_m - half > hall.min_z + 1 && -point.y_m + half < hall.max_z - 1,
                  "the walls stand outside the corridor");
        check(hall.roof_at(hall.min_x) == hall.eave_m && hall.roof_at(hall.max_x) == hall.eave_m &&
              std::abs(hall.roof_at((hall.min_x + hall.max_x) / 2) - hall.ridge_m) < 1e-12 && hall.ridge_m > hall.eave_m,
              "the roof rises from its eaves to its ridge");
        check(hall.lamp_columns >= 4 && hall.lamp_rows >= 10 && hall.lamp_dx > 0 && hall.lamp_dz > 0, "rows of lamps");
        for (int column = 0; column < hall.lamp_columns; ++column)
            for (int row = 0; row < hall.lamp_rows; ++row) {
                const double x = hall.lamp_x0 + column * hall.lamp_dx, z = hall.lamp_z0 + row * hall.lamp_dz;
                check(x > hall.min_x + 1.5 && x < hall.max_x - 1.5 && z > hall.min_z + 1.5 && z < hall.max_z - 1.5 && hall.roof_at(x) > hall.lamp_height_m,
                      "every lamp hangs inside the room and under its roof");
            }
        // The floor's shader mirrors a lamp at every node of the grid inside the walls, so there must be none it would invent.
        check(hall.lamp_x0 - hall.lamp_dx < hall.min_x && hall.lamp_x0 + hall.lamp_columns * hall.lamp_dx > hall.max_x &&
              hall.lamp_z0 - hall.lamp_dz < hall.min_z && hall.lamp_z0 + hall.lamp_rows * hall.lamp_dz > hall.max_z,
              "the grid has no node inside the walls without a lamp");

        const auto started = std::chrono::steady_clock::now();
        const auto barriers = part(KartHallPart::barriers, track, corners, &relief);
        const auto floor = part(KartHallPart::floor, track, corners, &relief);
        const auto shell = part(KartHallPart::shell, track, corners, &relief);
        const auto lamps = part(KartHallPart::lamps, track, corners, &relief);
        const auto glow = part(KartHallPart::glow, track, corners, &relief);
        const double build_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
        std::cout << "  the hall: " << barriers.size() / 3 << " triangles of barrier, " << floor.size() / 3 << " of floor, " << shell.size() / 3 << " of shell, "
                  << lamps.size() / 3 << " of lamps, " << glow.size() / 3 << " of glow, built in " << build_ms << " ms\n";
        for (const auto* vertices : {&barriers, &floor, &shell, &lamps, &glow}) {
            check(!vertices->empty() && vertices->size() % 3 == 0, "each part is whole triangles");
            for (const auto& v : *vertices)
                check(std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) && v.r >= 0 && v.g >= 0 && v.b >= 0 && v.a >= 0 && v.a <= 1, "every vertex is a place and a colour");
        }

        // Lamps: the drawn lamps are the grid's, all of them, at the height the floor mirrors them at.
        {
            std::vector<int> drawn(static_cast<std::size_t>(hall.lamp_columns * hall.lamp_rows), 0);
            for (const auto& v : lamps) {
                if (std::abs(v.y - hall.lamp_height_m) > 1e-4) {
                    check(v.y > hall.lamp_height_m && v.y < hall.ridge_m, "what is not a lamp is a strip light on the roof above them");
                    continue;
                }
                const int column = static_cast<int>(std::lround((v.x - hall.lamp_x0) / hall.lamp_dx)), row = static_cast<int>(std::lround((v.z - hall.lamp_z0) / hall.lamp_dz));
                check(column >= 0 && column < hall.lamp_columns && row >= 0 && row < hall.lamp_rows, "a lamp at a node of the grid");
                check(std::hypot(v.x - (hall.lamp_x0 + column * hall.lamp_dx), v.z - (hall.lamp_z0 + row * hall.lamp_dz)) < 0.25, "and no wider than a lamp");
                ++drawn[static_cast<std::size_t>(column * hall.lamp_rows + row)];
            }
            check(std::all_of(drawn.begin(), drawn.end(), [](int count) { return count >= 30; }), "every node of the grid has its lamp");
            for (std::size_t k = 0; k < glow.size(); k += 3) {
                check(glow[k].a > 0.3f && glow[k + 1].a == 0 && glow[k + 2].a == 0, "a lamp's haze is brightest at the lamp and gone at its rim");
                check(glow[k].y < hall.lamp_height_m && glow[k].y > hall.lamp_height_m - 0.1, "and hangs just beneath it");
            }
        }

        // Barriers: on the corridor's edges, never in the road; red-rimmed inside the bends, with their own colours where the
        // hall has them; and raised with the banked bend.
        {
            bool rims = false, braces = false, blue_rim = false, white_blocks = false;
            float highest = 0;
            for (const auto& v : barriers) {
                const auto at = on_track(track, v);
                check(at.distance_m > half - 0.02, "no part of a barrier stands in the road");
                check(at.distance_m < half + 0.7, "and none stands away from its edge");
                rims = rims || red(v);
                braces = braces || green(v);
                blue_rim = blue_rim || blue(v);
                white_blocks = white_blocks || pale(v);
                highest = std::max(highest, v.y);
                check(v.y > -1e-4, "nothing below the floor");
            }
            check(rims && braces && blue_rim && white_blocks, "red rims, green braces, the last corner's blue rim and the fourth's white blocks");
            // The upper tube's flat top is 0.45 m up: an eight-sided tube of 0.115 m radius centred 0.345 m up.
            const double top = 0.345 + 0.115 * std::sin(67.5 * std::numbers::pi / 180), bank = track.width_m * std::tan(6 * std::numbers::pi / 180);
            check(std::abs(highest - (top + bank)) < 0.02, "the barrier on the banked bend's outside stands on the raised edge: " + std::to_string(highest));
            float flat_highest = 0;
            for (const auto& v : part(KartHallPart::barriers, track, corners, nullptr)) flat_highest = std::max(flat_highest, v.y);
            check(std::abs(flat_highest - top) < 1e-3, "and on flat ground every barrier is 0.45 m high: " + std::to_string(flat_highest));
        }

        // Floor: islands and bare floor behind the barriers, paint on the road's edges, the start grid on the straight.
        {
            bool islands = false, kerb_red = false, grid = false;
            for (const auto& v : floor) {
                const auto at = on_track(track, v);
                if (at.distance_m >= half + 0.2) {
                    // Behind the barriers: the hall's floor, bare or island.
                    check(v.y > 0.011f && v.y < 0.015f, "the hall's floor is flat, the banked bend's surroundings included");
                    islands = islands || green(v);
                    continue;
                }
                // On the road and under the barriers' faces there is only paint.
                check(!green(v), "an island never reaches the road");
                check(at.distance_m > half - 0.75 || (pale(v) && (at.s_m > length - 31 || at.s_m < 1)),
                      "inside the corridor there is only paint: lines and kerbs along its edges, the start grid on the straight");
                kerb_red = kerb_red || red(v);
                grid = grid || (at.distance_m < half - 0.75);
            }
            check(islands && kerb_red && grid, "islands, kerbs and the start grid are all there");
        }

        // Shell: inside its own walls, and nothing of it in the road below the footbridge's height.
        {
            bool gallery = false, fence = false, bridge = false;
            for (std::size_t k = 0; k < shell.size(); k += 3) {
                for (std::size_t m = 0; m < 3; ++m) {
                    const auto& v = shell[k + m];
                    check(v.x > hall.min_x - 0.02 && v.x < hall.max_x + 0.02 && v.z > hall.min_z - 0.02 && v.z < hall.max_z + 0.02 && v.y > -1e-4 && v.y < hall.ridge_m + 1e-4,
                          "the shell stays within its own walls and under its ridge");
                    // The gallery's wall ends 4.6 m up in a dark red.
                    gallery = gallery || (v.r > 0.1f && v.g < 0.3f * v.r && v.b < 0.3f * v.r && v.y > 4.5f && v.y < 4.7f);
                    fence = fence || (green(v) && v.y > 1.0f && v.y < 2.0f);
                }
                // The middle of each triangle and of each of its sides.
                const auto& a = shell[k];
                const auto& b = shell[k + 1];
                const auto& c = shell[k + 2];
                const SceneVertex probes[4]{{(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3, 0, 0, 0, 1}, {(a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2, 0, 0, 0, 1},
                                            {(b.x + c.x) / 2, (b.y + c.y) / 2, (b.z + c.z) / 2, 0, 0, 0, 1}, {(c.x + a.x) / 2, (c.y + a.y) / 2, (c.z + a.z) / 2, 0, 0, 0, 1}};
                for (const auto& probe : probes) {
                    const bool over_road = on_track(track, probe).distance_m < half;
                    check(!over_road || probe.y > 2.7f, "nothing of the shell stands in the road; what crosses it crosses overhead");
                    bridge = bridge || (over_road && probe.y < 4.2f);
                }
            }
            check(gallery && fence && bridge, "the gallery, the fence and the footbridge are there");
        }

        // The layout's physical boundary: exact track-edge faces plus architecture
        // sliced at bumper height, independent of whether the hall is drawn.
        {
            const auto world=kart_environment(track,corners);
            check(world.walls().size()>2*track.points.size(),"architecture joins both solid track edges");
            const auto edges=fd::track_barriers(track);
            for (std::size_t i=0;i<edges.size();++i)
                check(world.walls()[i].a.x==edges[i].a.x && world.walls()[i].a.y==edges[i].a.y,
                      "collision faces share the track layout");
            const auto& p=fd::vehicle_profile("gokartcentralen-realistic");
            const auto config=fd::configured_for(p,{});
            const auto& start=track.points[0]; const auto& next=track.points[1];
            auto state=fd::plant_state_from({start.x_m,start.y_m,std::atan2(next.y_m-start.y_m,next.x_m-start.x_m),0,0,0},p.car,config);
            check(fd::barrier_clearance(world,state.pose,config)>0,"start grid clears bumper");
            double least=100;
            for (int i=0;i<1000;++i) {
                state=fd::advance_with_barriers(p.car,state,{7,0},config,0.005,{},world);
                const double gap=fd::barrier_clearance(world,state.pose,config);
                least=std::min(least,gap);
                check(gap>=-1e-6,"full throttle straight ahead cannot pass through real track barriers");
            }
            check(least<0.001,"straight driving reaches a physical barrier");
        }

        // Wear: a faint pale band on the banked ninth corner's road and nowhere else, lying on its tilted surface.
        {
            const auto wear = part(KartHallPart::wear, track, corners, &relief);
            check(!wear.empty() && wear.size() % 3 == 0, "the banked bend is scuffed");
            const double from = corners[8].from_s_m, span = std::fmod(corners[8].to_s_m - from + length, length);
            float strongest = 0;
            for (const auto& v : wear) {
                const auto at = on_track(track, v);
                check(at.distance_m < half - 0.3, "the scuffing is on the road, clear of its painted edges");
                const double into = std::fmod(at.s_m - from + length + 3.0, length);
                check(into <= span + 6.0, "and on the ninth corner alone");
                check(v.a >= 0 && v.a <= 0.2001f, "it is faint");
                check(std::abs(v.y - (0.016 + relief.ground_at({v.x, -v.z}).height_m)) < 1e-4, "and lies on the banked surface, just above the road");
                strongest = std::max(strongest, v.a);
            }
            check(strongest > 0.15f, "strongest through the middle of the bend");
            check(part(KartHallPart::wear, fd::make_preset_track(), {}, nullptr).empty(), "no other track is scuffed");
        }

        // Kept: asked again, a part is the same part.
        {
            const auto again = part(KartHallPart::barriers, track, corners, &relief);
            check(again.size() == barriers.size() && std::equal(again.begin(), again.end(), barriers.begin(), [](const SceneVertex& a, const SceneVertex& b) {
                      return a.x == b.x && a.y == b.y && a.z == b.z && a.r == b.r && a.g == b.g && a.b == b.b && a.a == b.a;
                  }), "a part asked for again is the same part");
        }

        // Any other track gets the plain room: barriers and a floor round it, none of Gokartcentralen's own furniture.
        {
            const auto other = fd::make_preset_track();
            const auto plain = part(KartHallPart::shell, other, {}, nullptr);
            check(!plain.empty(), "a room round any track");
            for (const auto& v : plain) check(!(green(v) && v.y > 1.0f && v.y < 2.0f), "no fence on a track that is not the kart's");
            const auto plain_barriers = part(KartHallPart::barriers, other, {}, nullptr);
            for (const auto& v : plain_barriers) check(!red(v) && !blue(v), "and without bends named, plain black barriers");
        }
        check(kart_hall_part_follows_ground(KartHallPart::barriers) && kart_hall_part_follows_ground(KartHallPart::floor) &&
              kart_hall_part_follows_ground(KartHallPart::wear) &&
              !kart_hall_part_follows_ground(KartHallPart::shell) && !kart_hall_part_follows_ground(KartHallPart::lamps) &&
              !kart_hall_part_follows_ground(KartHallPart::glow), "the barriers, the floor and its wear are on the ground; the room and its lamps are not");
    } catch (const std::exception& error) {
        std::cerr << "kart hall: " << error.what() << " after " << checks << " checks\n";
        return 1;
    }
    std::cout << "kart hall: " << checks << " checks passed\n";
    return 0;
}
